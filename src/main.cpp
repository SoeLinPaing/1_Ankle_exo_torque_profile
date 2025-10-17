// ============================================================================
// ankle_exo_torque_profile - RT 500 Hz motor loop + ADS1115 continuous read +
// UDP telemetry + Gait Phase Detection + Half-Cosine Torque Profile
// + Stride Time Transmission (last 5 strides) + UDP Motor Enable/Disable
//
// Build:
//   g++ -O2 -pthread main.cpp -o ankle_exo_torque_profile \
//       -I./serialPort -I./unitreeMotor -I./ads1115 \
//       -L. -lUnitreeMotorSDK_Arm64 -lserialPort -lads1115
//
// Run (sudo for RT priority):
//   sudo ./ankle_exo_torque_profile /dev/ttyUSB0
//
// UDP Output (→ Mac 10.152.17.231:5005):
//   {t_sched, t_actual, jitter, tau_des, q, dq, adsV, gait_phase,
//    rise, peak, fall, stride1..stride5}
//
// UDP Input (port 5006, from Mac):
//   "ENABLE" or "DISABLE"
// ============================================================================

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
#include <arpa/inet.h>
#include <sys/socket.h>
#include <vector>
#include <fcntl.h>

extern "C" {
#include "driver_ads1115.h"
#include "driver_ads1115_interface.h"
}
#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"

// ------------------------------------------------------------
// Time helpers
// ------------------------------------------------------------
static inline int64_t nsec_diff(const timespec& a,const timespec& b){
    return (int64_t(a.tv_sec)-int64_t(b.tv_sec))*1000000000LL +
           (int64_t(a.tv_nsec)-int64_t(b.tv_nsec));
}
static inline timespec nsec_add(const timespec& t,int64_t ns){
    timespec o=t; o.tv_nsec+=ns;
    while(o.tv_nsec>=1000000000L){o.tv_nsec-=1000000000L;o.tv_sec++;}
    return o;
}
static inline double sec_from(const timespec& t0,const timespec& t){
    return double(t.tv_sec-t0.tv_sec)+double(t.tv_nsec-t0.tv_nsec)*1e-9;
}

// ------------------------------------------------------------
// SPSC ring buffer
// ------------------------------------------------------------
template <size_t N>
struct SPSC {
    static_assert((N&(N-1))==0,"N must be power of two");
    struct Telemetry {
        uint64_t seq;
        double t_sched_sec,t_actual_sec;
        int64_t jitter_ns;
        double q_des_deg,dq_des_deg_s,tau_des_Nm;
        double q_act_deg,dq_act_deg_s,temp_C;
        int merror;
        double ads_voltage;
        double gait_phase;
    };
    Telemetry buf[N];
    std::atomic<size_t> head{0},tail{0};
    bool push(const Telemetry&s){
        size_t h=head.load(std::memory_order_relaxed);
        size_t t=tail.load(std::memory_order_acquire);
        if(((h+1)&(N-1))==(t&(N-1)))return false;
        buf[h&(N-1)]=s;
        head.store(h+1,std::memory_order_release);
        return true;
    }
    bool pop(Telemetry&out){
        size_t t=tail.load(std::memory_order_relaxed);
        size_t h=head.load(std::memory_order_acquire);
        if(t==h)return false;
        out=buf[t&(N-1)];
        tail.store(t+1,std::memory_order_release);
        return true;
    }
};

constexpr size_t RING_SIZE=1u<<15;
SPSC<RING_SIZE> q;
std::atomic<uint64_t> dropped{0};
volatile sig_atomic_t running=1;
static void on_sigint(int){running=0;}

std::atomic<float> g_latest_voltage{NAN};
std::atomic<bool> g_enable_motor{false};

// ------------------------------------------------------------
// UDP Command Listener (no auto-disable)
// ------------------------------------------------------------
void udp_command_listener(){
    int sock=socket(AF_INET,SOCK_DGRAM,0);
    sockaddr_in addr{};
    addr.sin_family=AF_INET;
    addr.sin_port=htons(5006);
    addr.sin_addr.s_addr=INADDR_ANY;
    bind(sock,(sockaddr*)&addr,sizeof(addr));
    fcntl(sock,F_SETFL,O_NONBLOCK);

    std::cout<<"Listening for motor commands on port 5006...\n";

    char buf[64];
    sockaddr_in src{}; socklen_t slen=sizeof(src);
    g_enable_motor.store(false); // start disabled

    while(running){
        ssize_t n=recvfrom(sock,buf,sizeof(buf)-1,0,(sockaddr*)&src,&slen);
        if(n>0){
            buf[n]='\0';
            std::string msg(buf);
            if(msg.find("ENABLE")!=std::string::npos){
                g_enable_motor.store(true);
                std::cout<<"[UDP CMD] ENABLE received\n";
            } else if(msg.find("DISABLE")!=std::string::npos){
                g_enable_motor.store(false);
                std::cout<<"[UDP CMD] DISABLE received\n";
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    close(sock);
}

// ------------------------------------------------------------
// ADS Thread
// ------------------------------------------------------------
struct AdsCtx{ads1115_handle_t* ads;int sps;};
void ads_thread_func(AdsCtx ctx){
    int16_t raw; float v=NAN;
    const int period_us=(ctx.sps>0)?int(1e6/ctx.sps):2000;
    while(running){
        if(ads1115_continuous_read(ctx.ads,&raw,&v)==0)
            g_latest_voltage.store(v,std::memory_order_relaxed);
        std::this_thread::sleep_for(std::chrono::microseconds(period_us));
    }
}

// ------------------------------------------------------------
// RT Control Loop
// ------------------------------------------------------------
struct RtArgs{
    SerialPort* serial;
    int motor_id;
    MotorType mtype;
    int udp_sock;
    sockaddr_in udp_addr;
};

void producer_rt(const RtArgs& A){
    cpu_set_t set; CPU_ZERO(&set); CPU_SET(0,&set);
    pthread_setaffinity_np(pthread_self(),sizeof(set),&set);
    sched_param sp{.sched_priority=90};
    sched_setscheduler(0,SCHED_FIFO,&sp);
    mlockall(MCL_CURRENT|MCL_FUTURE);

    const int loop_hz=500;
    const int64_t period_ns=int64_t(1e9/loop_hz);
    const double period_s=1.0/loop_hz;
    const double rad2deg=180.0/M_PI, GR=queryGearRatio(A.mtype);

    MotorCmd cmd{}; MotorData data{};
    cmd.motorType=A.mtype; data.motorType=A.mtype; cmd.id=A.motor_id;
    const uint8_t mode_foc=queryMotorMode(A.mtype,MotorMode::FOC);
    const uint8_t mode_brake=queryMotorMode(A.mtype,MotorMode::BRAKE);

    timespec t0; clock_gettime(CLOCK_MONOTONIC,&t0);
    timespec next=nsec_add(t0,period_ns);
    uint64_t seq=0;

    bool zero_initialized=false;
    double q0=0.0,dq0=0.0;
    double fsr_thresh=2.0;
    bool was_in_contact=false;
    timespec last_hs_time=t0;
    double default_stride=1.0;
    std::vector<double> prev_strides;
    double gait_phase=0.0;
    bool first_strike_detected=false;

    const double peak_torque=5.0;
    const double peak_time=0.8;
    const double rise_time=0.7;
    const double fall_time=0.10;
    const double phi_start=rise_time;
    const double phi_end=peak_time+fall_time;

    while(running){
        clock_nanosleep(CLOCK_MONOTONIC,TIMER_ABSTIME,&next,nullptr);
        timespec now; clock_gettime(CLOCK_MONOTONIC,&now);

        double t_sched=(seq+1)*period_s;
        double t_actual=sec_from(t0,now);
        int64_t jitter=nsec_diff(now,nsec_add(t0,int64_t((seq+1)*period_ns)));
        float ads_v=g_latest_voltage.load(std::memory_order_relaxed);

        bool in_contact=(ads_v>fsr_thresh);
        if(in_contact && !was_in_contact){
            double t_now=sec_from(t0,now);
            if(!first_strike_detected){
                first_strike_detected=true;
                last_hs_time=now;
                std::cout<<"First heel strike detected.\n";
            } else {
                double stride_time=t_now-sec_from(t0,last_hs_time);
                last_hs_time=now;
                if(stride_time>0.3 && stride_time<2.0){
                    prev_strides.push_back(stride_time);
                    if(prev_strides.size()>5) prev_strides.erase(prev_strides.begin());
                    double avg=0.0; for(double s:prev_strides) avg+=s; avg/=prev_strides.size();
                    default_stride=avg;
                    std::cout<<"Stride detected: "<<stride_time<<" s (avg="<<avg<<")\n";
                }
            }
        }
        was_in_contact=in_contact;

        if(first_strike_detected){
            double t_since_hs=sec_from(t0,now)-sec_from(t0,last_hs_time);
            gait_phase=t_since_hs/default_stride;
            if(gait_phase<0.0) gait_phase=0.0;
            if(gait_phase>1.0) gait_phase=1.0;
        } else gait_phase=0.0;

        double tau_des=0.0;
        if(g_enable_motor.load()){
            double phi=gait_phase;
            if(phi>=phi_start && phi<peak_time){
                double ph=(phi-phi_start)/(peak_time-rise_time);
                tau_des=peak_torque*0.5*(1.0-std::cos(M_PI*ph));
            } else if(phi>=peak_time && phi<=phi_end){
                double ph=(phi-peak_time)/fall_time;
                tau_des=peak_torque*0.5*(1.0+std::cos(M_PI*ph));
            }
        } else tau_des=0.0;

        cmd.mode=g_enable_motor.load()?mode_foc:mode_brake;
        cmd.kp=cmd.kd=0.0; cmd.q=cmd.dq=0.0;
        cmd.tau=-tau_des/GR;
        A.serial->sendRecv(&cmd,&data);

        if(!zero_initialized && std::abs(data.q)>1e-6){
            q0=data.q; dq0=data.dq; zero_initialized=true;
            std::cout<<"Motor zero offsets set.\n";
        }

        double q_act_deg=((data.q-q0)/GR)*rad2deg;
        double dq_act_deg_s=((data.dq-dq0)/GR)*rad2deg;

        float pkt[16]={0};
        pkt[0]=float(t_sched); pkt[1]=float(t_actual); pkt[2]=float(jitter);
        pkt[3]=float(tau_des); pkt[4]=float(q_act_deg); pkt[5]=float(dq_act_deg_s);
        pkt[6]=ads_v; pkt[7]=float(gait_phase);
        pkt[8]=float(rise_time); pkt[9]=float(peak_time); pkt[10]=float(fall_time);
        for(size_t i=0;i<5;++i) pkt[11+i]=(i<prev_strides.size()?float(prev_strides[i]):0.0f);

        sendto(A.udp_sock,pkt,sizeof(pkt),0,(sockaddr*)&A.udp_addr,sizeof(A.udp_addr));

        next=nsec_add(next,period_ns);
        ++seq;
    }
}

// ------------------------------------------------------------
int main(int argc,char**argv){
    const char* dev=(argc>=2)?argv[1]:"/dev/ttyUSB0";
    std::signal(SIGINT,on_sigint);

    // ADS1115 init
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
    ads1115_set_compare(&ads,ADS1115_BOOL_FALSE);
    ads1115_start_continuous_read(&ads);
    std::cout<<"ADS1115 continuous mode started.\n";

    // UDP setup
    int sock=socket(AF_INET,SOCK_DGRAM,0);
    sockaddr_in addr{};
    addr.sin_family=AF_INET;
    addr.sin_port=htons(5005);
    inet_pton(AF_INET,"10.152.17.231",&addr.sin_addr);
    std::cout<<"UDP target: 10.152.17.231:5005\n";

    SerialPort serial(dev);

    AdsCtx actx{&ads,800};
    std::thread ads_thread(ads_thread_func,actx);
    std::thread cmd_thread(udp_command_listener);

    RtArgs args{&serial,0,MotorType::GO_M8010_6,sock,addr};
    producer_rt(args);

    cmd_thread.join();
    ads_thread.join();

    ads1115_stop_continuous_read(&ads);
    ads1115_deinit(&ads);
    ads.iic_deinit();
    close(sock);
    return 0;
}