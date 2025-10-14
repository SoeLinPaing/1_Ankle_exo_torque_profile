// ankle_exo_torque_profile - RT motor loop (500 Hz) + ADS1115 continuous read in background
//
// Build (adjust include/library paths):
//   g++ -O2 -pthread main.cpp -o ankle_exo_torque_profile \
//       -I./serialPort -I./unitreeMotor -I./ads1115 \
//       -L. -lUnitreeMotorSDK_Arm64 -lserialPort -lads1115
//
// Run (SCHED_FIFO needs sudo or CAP_SYS_NICE):
//   sudo ./ankle_exo_torque_profile /dev/ttyUSB0 [log.csv]
//
// Key idea:
//   * RT loop (SCHED_FIFO) DOES NOT TOUCH I2C.
//   * ADS1115 is put in CONTINUOUS mode and polled in a normal-priority thread.
//   * Latest voltage is shared via std::atomic<float> to the RT loop.
//   * Logger thread drains SPSC and writes CSV.
//
// CSV columns:
//   seq,time_sched_sec,time_actual_sec,jitter_ns,
//   q_des_deg,dq_des_deg_s,tau_des_Nm,
//   q_act_deg,dq_act_deg_s,temp_C,merror,ads_voltage

#include <atomic>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <pthread.h>
#include <sched.h>
#include <thread>
#include <unistd.h>
#include <sys/mman.h>
#include <time.h>

extern "C" {
#include "driver_ads1115.h"
#include "driver_ads1115_interface.h"
}
#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"

// ---------- timing utils ----------
static inline int64_t nsec_diff(const timespec& a, const timespec& b){
    return (int64_t(a.tv_sec)-int64_t(b.tv_sec))*1000000000LL +
           (int64_t(a.tv_nsec)-int64_t(b.tv_nsec));
}
static inline timespec nsec_add(const timespec& t, int64_t ns){
    timespec o = t; o.tv_nsec += ns;
    while (o.tv_nsec >= 1000000000L) { o.tv_nsec -= 1000000000L; o.tv_sec++; }
    return o;
}
static inline double sec_from(const timespec& t0, const timespec& t){
    return double(t.tv_sec - t0.tv_sec) + double(t.tv_nsec - t0.tv_nsec)*1e-9;
}

// ---------- SPSC ring ----------
template <size_t N>
struct SPSC {
    static_assert((N&(N-1))==0, "N must be power of two");
    struct Telemetry {
        uint64_t seq;
        double   t_sched_sec, t_actual_sec;
        int64_t  jitter_ns;
        double   q_des_deg, dq_des_deg_s, tau_des_Nm;
        double   q_act_deg, dq_act_deg_s, temp_C;
        int      merror;
        double   ads_voltage;
    };
    Telemetry buf[N];
    std::atomic<size_t> head{0}, tail{0};
    bool push(const Telemetry& s){
        size_t h = head.load(std::memory_order_relaxed);
        size_t t = tail.load(std::memory_order_acquire);
        if (((h+1)&(N-1)) == (t&(N-1))) return false;
        buf[h&(N-1)] = s;
        head.store(h+1, std::memory_order_release);
        return true;
    }
    bool pop(Telemetry& out){
        size_t t = tail.load(std::memory_order_relaxed);
        size_t h = head.load(std::memory_order_acquire);
        if (t == h) return false;
        out = buf[t&(N-1)];
        tail.store(t+1, std::memory_order_release);
        return true;
    }
    size_t size() const {
        size_t h = head.load(std::memory_order_acquire);
        size_t t = tail.load(std::memory_order_acquire);
        return h - t;
    }
};

constexpr size_t RING_SIZE = 1u<<15;
SPSC<RING_SIZE> q;
std::atomic<uint64_t> dropped{0};

// global run flag
static volatile sig_atomic_t running = 1;
static void on_sigint(int){ running = 0; }

// ---------- global shared ADS sample ----------
std::atomic<float> g_latest_voltage{NAN};

// ---------- logger thread ----------
void logger_thread_func(const char* path){
    // keep logger simple, non-RT
    std::ofstream log(path);
    log.setf(std::ios::fixed);
    log << std::setprecision(9);
    log << "seq,time_sched_sec,time_actual_sec,jitter_ns,"
           "q_des_deg,dq_des_deg_s,tau_des_Nm,"
           "q_act_deg,dq_act_deg_s,temp_C,merror,ads_voltage\n";

    SPSC<RING_SIZE>::Telemetry s;
    while (running || q.size()>0){
        size_t n = 0;
        while (q.pop(s)){
            log << s.seq << ","
                << s.t_sched_sec << ","
                << s.t_actual_sec << ","
                << s.jitter_ns << ","
                << s.q_des_deg << ","
                << s.dq_des_deg_s << ","
                << s.tau_des_Nm << ","
                << s.q_act_deg << ","
                << s.dq_act_deg_s << ","
                << s.temp_C << ","
                << s.merror << ","
                << s.ads_voltage << "\n";
            ++n;
        }
        if (n==0) std::this_thread::sleep_for(std::chrono::milliseconds(5));
        else       log.flush();
    }
    log << "# dropped=" << dropped.load() << "\n";
    log.close();
}

// ---------- ADS reader thread (continuous mode, non-RT) ----------
struct AdsCtx {
    ads1115_handle_t* ads;
    int sps;                // target read pacing in Hz (<= device SPS)
};

void ads_thread_func(AdsCtx ctx){
    // normal priority is fine; no RT needed
    // device is already configured for continuous mode by main()
    int16_t raw;
    float   v = NAN;

    // pace near the device SPS; 860 SPS ≈ 1.16 ms periods.
    // reading faster than SPS will just repeat last sample.
    const int period_us = (ctx.sps > 0) ? int(1e6/ctx.sps) : 2000;

    while (running){
        // One lightweight read of conversion register
        if (ads1115_continuous_read(ctx.ads, &raw, &v) == 0){
            g_latest_voltage.store(v, std::memory_order_relaxed);
        }
        // Small sleep to avoid hammering the I2C bus
        std::this_thread::sleep_for(std::chrono::microseconds(period_us));
    }
}

// ---------- RT producer: 500 Hz motor loop (NO I2C here) ----------
struct RtArgs{
    SerialPort*        serial;
    int                motor_id;
    MotorType          mtype;
};

void producer_rt(const RtArgs& A){
    // Pin & promote to RT
    cpu_set_t set; CPU_ZERO(&set); CPU_SET(0, &set);
    pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
    sched_param sp{.sched_priority = 90};
    if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) perror("sched_setscheduler");
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) perror("mlockall");

    const int loop_hz = 500;
    const int64_t period_ns = int64_t(1e9 / loop_hz);
    const double period_s = 1.0 / loop_hz;
    const double rad2deg = 180.0/M_PI;
    const double GR      = queryGearRatio(A.mtype);

    MotorCmd  cmd{}; 
    MotorData data{};
    cmd.motorType  = A.mtype; 
    data.motorType = A.mtype; 
    cmd.id         = A.motor_id;
    const uint8_t mode_foc = queryMotorMode(A.mtype, MotorMode::FOC);

    timespec t0;   clock_gettime(CLOCK_MONOTONIC, &t0);
    timespec next; next = nsec_add(t0, period_ns);
    uint64_t seq = 0;

    while (running){
        // deterministic tick
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);
        timespec now; clock_gettime(CLOCK_MONOTONIC, &now);

        double  t_sched  = (seq + 1) * period_s;
        double  t_actual = sec_from(t0, now);
        int64_t jitter   = nsec_diff(now, nsec_add(t0, int64_t((seq + 1) * period_ns)));

        // ---- read latest ADS sample (atomic, no blocking) ----
        float ads_v = g_latest_voltage.load(std::memory_order_relaxed);

        // ---- motor command (example: constant torque) ----
        cmd.mode = mode_foc;
        cmd.kp = 0.0; cmd.kd = 0.0; cmd.q = 0.0; cmd.dq = 0.0;

        double torque_amp = 5.0 / queryGearRatio(MotorType::GO_M8010_6);
        cmd.tau = -torque_amp;

        // Serial exchange (this can have latency, but much smaller jitter now)
        A.serial->sendRecv(&cmd, &data);

        // ---- telemetry sample ----
        SPSC<RING_SIZE>::Telemetry s{
            /*seq*/         seq + 1,
            /*t_sched*/     t_sched,
            /*t_actual*/    t_actual,
            /*jitter_ns*/   jitter,
            /*q_des_deg*/   0.0,
            /*dq_des_deg_s*/0.0,
            /*tau_des_Nm*/  cmd.tau,
            /*q_act_deg*/   (data.q / GR) * rad2deg,
            /*dq_act_deg_s*/(data.dq / GR) * rad2deg,
            /*temp_C*/      double(data.temp),
            /*merror*/      int(data.merror),
            /*ads_voltage*/ double(ads_v)
        };
        if (!q.push(s)) dropped.fetch_add(1, std::memory_order_relaxed);

        next = nsec_add(next, period_ns);
        ++seq;
    }

    // Stop motor safely once
    cmd.mode = queryMotorMode(A.mtype, MotorMode::BRAKE);
    cmd.kp = cmd.kd = cmd.q = cmd.dq = cmd.tau = 0.0;
    A.serial->sendRecv(&cmd, &data);
}

// ---------- main ----------
int main(int argc, char** argv){
    const char* dev = (argc >= 2) ? argv[1] : "/dev/ttyUSB0";
    const char* csv = (argc >= 3) ? argv[2] : "motor_ads_log.csv";
    std::signal(SIGINT, on_sigint);

    // --- ADS1115 init (one-time) ---
    ads1115_handle_t ads;
    ads.debug_print = ads1115_interface_debug_print;
    ads.delay_ms    = ads1115_interface_delay_ms;
    ads.iic_init    = ads1115_interface_iic_init;
    ads.iic_deinit  = ads1115_interface_iic_deinit;
    ads.iic_read    = ads1115_interface_iic_read;
    ads.iic_write   = ads1115_interface_iic_write;

    if (ads.iic_init() != 0){ std::cerr << "I2C init failed\n"; return -1; }
    if (ads1115_init(&ads) != 0){ std::cerr << "ADS1115 init failed\n"; return -1; }

    ads1115_set_addr_pin(&ads, ADS1115_ADDR_GND);
    ads1115_set_channel(&ads, ADS1115_CHANNEL_AIN0_GND);
    ads1115_set_range(&ads, ADS1115_RANGE_4P096V);
    ads1115_set_rate(&ads, ADS1115_RATE_860SPS);
    ads1115_set_compare(&ads, ADS1115_BOOL_FALSE);
    //ads1115_set_mode(&ads, ADS1115_MODE_CONTINUOUS);               // continuous mode ON
    if (ads1115_start_continuous_read(&ads) != 0){
        std::cerr << "Failed to start ADS1115 continuous read\n";
        return -1;
    }
    std::cout << "ADS1115 started in continuous mode." << std::endl;

    // --- Motor serial init ---
    SerialPort serial(dev);

    // --- Launch logger (non-RT) ---
    std::thread logger(logger_thread_func, csv);

    // --- Launch ADS reader thread (non-RT) ---
    AdsCtx actx{ .ads = &ads, .sps = 800 };  // pace reads ~800 Hz (<= 860 SPS)
    std::thread ads_thread(ads_thread_func, actx);

    // --- Run RT producer in main thread for highest priority ---
    RtArgs args{ &serial, /*motor_id*/ 0, MotorType::GO_M8010_6 };
    producer_rt(args);

    // --- Shutdown ---
    ads_thread.join();
    logger.join();

    ads1115_stop_continuous_read(&ads);
    ads1115_deinit(&ads);
    ads.iic_deinit();

    std::cerr << "Exit complete. dropped=" << (unsigned long long)dropped.load() << "\n";
    return 0;
}