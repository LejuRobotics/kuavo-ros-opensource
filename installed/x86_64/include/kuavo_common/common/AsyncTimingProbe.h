#pragma once
// Observation only. One serialized producer; ROS allocation/publication stays
// on a SCHED_OTHER worker. Queue saturation drops diagnostics, never control.
#include <array>
#include <atomic>
#include <algorithm>
#include <chrono>
#include <thread>
#include <cstdint>
#include <time.h>
#include <pthread.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

namespace kuavo_timing {
struct Clocks { int64_t wall, cpu; };
inline int64_t nanoseconds(clockid_t clock) noexcept {
  timespec t{}; clock_gettime(clock, &t);
  return int64_t(t.tv_sec)*1000000000LL+t.tv_nsec;
}
inline Clocks sample() noexcept {
  return {nanoseconds(CLOCK_MONOTONIC), nanoseconds(CLOCK_THREAD_CPUTIME_ID)};
}
inline std::array<double,3> elapsed(const Clocks& a, const Clocks& b) noexcept {
  const double wall=(b.wall-a.wall)*1e-6, cpu=(b.cpu-a.cpu)*1e-6;
  return {{wall,cpu,std::max(0.0,wall-cpu)}};
}
template<size_t N, size_t Capacity=256> class AsyncPublisher {
 public:
  using Row=std::array<double,N>;
  ~AsyncPublisher(){stop();}
  void start(ros::NodeHandle& nh, const char* topic) {
    if(worker_.joinable()) return;
    publisher_=nh.advertise<std_msgs::Float64MultiArray>(topic,32);
    stopping_.store(false);
    worker_=std::thread([this] {
      sched_param p{};
      if(pthread_setschedparam(pthread_self(),SCHED_OTHER,&p)!=0) {
        ++errors_; return;  // never run a diagnostic writer as an RT thread
      }
      std_msgs::Float64MultiArray msg; msg.data.resize(N);
      while(!stopping_.load() || read_.load()!=write_.load()) {
        const size_t r=read_.load(std::memory_order_relaxed);
        if(r==write_.load(std::memory_order_acquire)) {
          std::this_thread::sleep_for(std::chrono::milliseconds(2)); continue;
        }
        std::copy(rows_[r%Capacity].begin(),rows_[r%Capacity].end(),msg.data.begin());
        read_.store(r+1,std::memory_order_release);
        try { publisher_.publish(msg); } catch(...) { ++errors_; }
      }
    });
  }
  void push(const Row& row) noexcept {
    const size_t w=write_.load(std::memory_order_relaxed);
    if(w-read_.load(std::memory_order_acquire)>=Capacity){++dropped_;return;}
    rows_[w%Capacity]=row; write_.store(w+1,std::memory_order_release);
  }
  uint64_t dropped() const noexcept { return dropped_.load(std::memory_order_relaxed); }
  uint64_t errors() const noexcept { return errors_.load(std::memory_order_relaxed); }
  void stop() noexcept { stopping_.store(true); if(worker_.joinable())worker_.join(); }
 private:
  std::array<Row,Capacity> rows_{};
  std::atomic<size_t> read_{0},write_{0};
  std::atomic<uint64_t> dropped_{0},errors_{0};
  std::atomic<bool> stopping_{false};
  ros::Publisher publisher_; std::thread worker_;
};
// v1: version,pid,tid,serial,ROS time,start MONOTONIC ns,
// total wall/cpu/off_cpu ms, completed stage count,drops,writer errors,
// seven wall/cpu/off_cpu triples: sensor_obs,FK,MPC/MRT,cmd_prepare,
// dynamics (RNEA or WBC backend),publish,tail. Early exits retain stage count.
class WbcCycle {
 public:
  using Publisher=AsyncPublisher<33>;
  WbcCycle(Publisher& publisher,uint64_t serial,double stamp) noexcept
      : publisher_(publisher),start_(sample()),last_(start_) {
    row_[0]=1;row_[1]=getpid();row_[2]=syscall(SYS_gettid);
    row_[3]=serial;row_[4]=stamp;row_[5]=double(start_.wall);
  }
  void mark() noexcept {
    if(stage_>=6)return;
    const auto now=sample(); const auto d=elapsed(last_,now);
    std::copy(d.begin(),d.end(),row_.begin()+12+3*stage_++);last_=now;
  }
  ~WbcCycle() noexcept {
    const auto end=sample(); const auto total=elapsed(start_,end);
    if(total[0]<=2.0)return;
    const auto tail=elapsed(last_,end);
    std::copy(tail.begin(),tail.end(),row_.begin()+12+3*stage_);
    std::copy(total.begin(),total.end(),row_.begin()+6);
    row_[9]=stage_;row_[10]=publisher_.dropped();row_[11]=publisher_.errors();
    publisher_.push(row_);
  }
 private:
  Publisher& publisher_; Clocks start_,last_; Publisher::Row row_{};size_t stage_=0;
};
}  // namespace kuavo_timing
