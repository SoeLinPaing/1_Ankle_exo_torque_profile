// rt_motor_ads1115_log.cpp
//
// Build:
//   g++ -O2 -pthread rt_motor_ads1115_log.cpp -o rt_motor_ads1115_log \
//       -I./serialPort -I./unitreeMotor -I./ads1115 \
//       -L. -lunitreeMotor -lserialPort -lads1115
//
// Run (requires sudo for SCHED_FIFO):
//   sudo ./rt_motor_ads1115_log /dev/ttyUSB0
//
// Functionality:
//   - Real-time loop @1 kHz controlling Unitree GO_M8010_6 motor (FOC mode)
//   - Reads ADS1115 analog voltage each iteration
//   - Logs everything (motor + voltage) to CSV via lock-free SPSC buffer
//   - Safe Ctrl + C handling: stops motor and de-inits ADS1115

#include <atomic>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
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

// ------------------------------------------------------------
// small time helpers
static inline int64_t nsec_diff(const timespec& a, const timespec& b){
    return (int64_t(a.tv_sec)-int64_t(b.tv_sec))*1000000000LL +
           (int64_t(a.tv_nsec)-int64_t(b.tv_nsec));
}
static inline timespec nsec_add(const timespec& t,int64_t ns){
    timespec out=t; out.tv_nsec+=ns;
    while(out.tv_nsec>=1000000000L){out.tv_nsec-=1000000000L; out.tv_sec++;}
    return out;
}
static inline double sec_from(const timespec& t0,const timespec& t){
    return double(t.tv_sec-t0.tv_sec)+double(t.tv_nsec-t0.tv_nsec)*1e-9;
}

// ------------------------------------------------------------
// lock-free SPSC ring buffer
template <size_t N>
struct SPSC {
    static_assert((N&(N-1))==0,"N must be power of two");
    struct Telemetry {
        uint64_t seq;
        double   t_sched_sec;
        double   t_actual_sec;
        int64_t  jitter_ns;
        double q_des_deg, dq_des_deg_s, tau_des_Nm;
        double q_act_deg, dq_act_deg_s, temp_C;
        int    merror;
        double ads_voltage;   // <<<< added
    };
    Telemetry buf[N];
    std::atomic<size_t> head{0}, tail{0};
    bool push(const Telemetry& s){
        size_t h=head.load(std::memory_order_relaxed);
        size_t t=tail.load(std::memory_order_acquire);
        if(((h+1)&(N-1))==(t&(N-1)))return false;
        buf[h&(N-1)]=s;
        head.store(h+1,std::memory_order_release);
        return true;
    }
    bool pop(Telemetry& out){
        size_t t=tail.load(std::memory_order_relaxed);
        size_t h=head.load(std::memory_order_acquire);
        if(t==h)return false;
        out=buf[t&(N-1)];
        tail.store(t+1,std::memory_order_release);
        return true;
    }
    size_t size()const{
        size_t h=head.load(std::memory_order_acquire);
        size_t t=tail.load(std::memory_order_acquire);
        return h-t;
    }
};

// ------------------------------------------------------------
static void pin_thread_to_cpu(int cpu){
    cpu_set_t set; CPU_ZERO(&set); CPU_SET(cpu,&set);
    pthread_setaffinity_np(pthread_self(),sizeof(set),&set);
}
static void set_rt_fifo(int prio){
    sched_param sp{.sched_priority=prio};
    if(sched_setscheduler(0,SCHED_FIFO,&sp)!=0)
        perror("sched_setscheduler");
}

// ------------------------------------------------------------
constexpr size_t RING_SIZE=1u<<15;
SPSC<RING_SIZE> q;
std::atomic<uint64_t> dropped{0};
volatile sig_atomic_t running=1;
static void on_sigint(int){running=0;}

// ------------------------------------------------------------
// Logger thread
void logger_thread_func(const char* path){
    pin_thread_to_cpu(1);
    nice(5);
    std::ofstream log(path);
    log.setf(std::ios::fixed);
    log<<std::setprecision(9);
    log<<"seq,time_sched_sec,time_actual_sec,jitter_ns,"
          "q_des_deg,dq_des_deg_s,tau_des_Nm,"
          "q_act_deg,dq_act_deg_s,temp_C,merror,ads_voltage\n";
    SPSC<RING_SIZE>::Telemetry s;
    while(running||q.size()>0){
        size_t n=0;
        while(q.pop(s)){
            log<<s.seq<<","<<s.t_sched_sec<<","<<s.t_actual_sec<<","<<s.jitter_ns<<","
               <<s.q_des_deg<<","<<s.dq_des_deg_s<<","<<s.tau_des_Nm<<","
               <<s.q_act_deg<<","<<s.dq_act_deg_s<<","<<s.temp_C<<","<<s.merror<<","
               <<s.ads_voltage<<"\n";
            ++n;
        }
        if(n==0) std::this_thread::sleep_for(std::chrono::milliseconds(5));
        else log.flush();
    }
    log<<"# dropped="<<dropped.load()<<"\n";
    log.close();
    std::cerr<<"Logger done.\n";
}

// ------------------------------------------------------------
// RT producer: 1 kHz motor loop + ADS1115 read
struct RtArgs{
    SerialPort* serial;
    int motor_id;
    MotorType mtype;
    ads1115_handle_t* ads;
};

void producer_rt(const RtArgs& A){
    pin_thread_to_cpu(0);
    set_rt_fifo(90);
    if(mlockall(MCL_CURRENT|MCL_FUTURE)!=0) perror("mlockall");

    const int loop_hz=1000;
    const int64_t period_ns=int64_t(1e9/loop_hz);
    const double period_s=1.0/loop_hz;
    const double rad2deg=180.0/M_PI, GR=queryGearRatio(A.mtype);

    MotorCmd cmd{}; MotorData data{};
    cmd.motorType=A.mtype; data.motorType=A.mtype; cmd.id=A.motor_id;
    const uint8_t mode_foc=queryMotorMode(A.mtype,MotorMode::FOC);

    timespec t0; clock_gettime(CLOCK_MONOTONIC,&t0);
    timespec next=nsec_add(t0,period_ns);
    uint64_t seq=0;

    int16_t raw; float voltage=0.0f;

    while(running){
        clock_nanosleep(CLOCK_MONOTONIC,TIMER_ABSTIME,&next,nullptr);
        timespec now; clock_gettime(CLOCK_MONOTONIC,&now);

        double t_sched=(seq+1)*period_s;
        double t_actual=sec_from(t0,now);
        int64_t jitter=nsec_diff(now,nsec_add(t0,int64_t((seq+1)*period_ns)));

        // ----- ADS1115 read -----
        if(ads1115_single_read(A.ads,&raw,&voltage)!=0) voltage=NAN;

        // ----- Motor command -----
        cmd.mode=mode_foc;
        cmd.kp=0.0; cmd.kd=0.0; cmd.q=0.0; cmd.dq=0.0;

        double torque_amp=24.0/queryGearRatio(MotorType::GO_M8010_6);
        double t_cycle=fmod(t_sched,12.0);
        if(t_cycle<5.0) cmd.tau=0;
        else if(t_cycle<5.5) cmd.tau= torque_amp;
        else if(t_cycle<11.5) cmd.tau=0;
        else cmd.tau=-torque_amp;

        A.serial->sendRecv(&cmd,&data);

        // ----- Telemetry -----
        SPSC<RING_SIZE>::Telemetry s{
            seq+1, t_sched, t_actual, jitter,
            0.0, 0.0, cmd.tau,
            (data.q/GR)*rad2deg,
            (data.dq/GR)*rad2deg,
            double(data.temp),
            int(data.merror),
            double(voltage)
        };
        if(!q.push(s)) dropped.fetch_add(1,std::memory_order_relaxed);

        next=nsec_add(next,period_ns);
        ++seq;
    }

    // stop motor safely
    cmd.mode=queryMotorMode(A.mtype,MotorMode::BRAKE);
    cmd.kp=cmd.kd=cmd.q=cmd.dq=cmd.tau=0.0;
    A.serial->sendRecv(&cmd,&data);
}

// ------------------------------------------------------------
int main(int argc,char**argv){
    const char* dev=(argc>=2)?argv[1]:"/dev/ttyUSB0";
    const char* csv=(argc>=3)?argv[2]:"motor_ads_log.csv";
    std::signal(SIGINT,on_sigint);

    // ---- Init ADS1115 ----
    ads1115_handle_t ads;
    ads.debug_print=ads1115_interface_debug_print;
    ads.delay_ms=ads1115_interface_delay_ms;
    ads.iic_init=ads1115_interface_iic_init;
    ads.iic_deinit=ads1115_interface_iic_deinit;
    ads.iic_read=ads1115_interface_iic_read;
    ads.iic_write=ads1115_interface_iic_write;

    if(ads.iic_init()!=0){std::cerr<<"I2C init failed\n";return -1;}
    if(ads1115_init(&ads)!=0){std::cerr<<"ADS1115 init failed\n";return -1;}
    ads1115_set_addr_pin(&ads,ADS1115_ADDR_GND);
    ads1115_set_channel(&ads,ADS1115_CHANNEL_AIN0_GND);
    ads1115_set_range(&ads,ADS1115_RANGE_4P096V);
    ads1115_set_rate(&ads,ADS1115_RATE_860SPS);
    std::cout<<"ADS1115 initialized.\n";

    // ---- Init motor serial ----
    SerialPort serial(dev);

    // ---- Logger thread ----
    std::thread logger(logger_thread_func,csv);

    RtArgs args{&serial,0,MotorType::GO_M8010_6,&ads};
    producer_rt(args);

    logger.join();

    ads1115_deinit(&ads);
    ads.iic_deinit();
    std::cout<<"ADS1115 deinit done.\n";

    std::cerr<<"Exit complete. dropped="<<(unsigned long long)dropped.load()<<"\n";
    return 0;
}

//https://github.com/SoeLinPaing/1_Ankle_exo_torque_profile