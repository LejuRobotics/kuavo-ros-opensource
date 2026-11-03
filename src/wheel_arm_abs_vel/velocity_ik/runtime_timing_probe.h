#pragma once
// Observation only: fixed-size single-producer queue; no control decisions.
#include <array>
#include <algorithm>
#include <cstdint>
#include <atomic>
#include <chrono>
#include <functional>
#include <thread>
#include <time.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>
namespace velocity_ik {
namespace runtime_probe {
constexpr size_t kPoints=64;
struct Point { double id, wall, cpu, voluntary, involuntary, minor, major; };
inline Point sample(int id) noexcept {
  timespec wall{},cpu{}; rusage usage{};
  clock_gettime(CLOCK_MONOTONIC,&wall);
  clock_gettime(CLOCK_THREAD_CPUTIME_ID,&cpu);
  getrusage(RUSAGE_THREAD,&usage);
  return {double(id),double(wall.tv_sec)*1e9+wall.tv_nsec,
          double(cpu.tv_sec)*1e9+cpu.tv_nsec,double(usage.ru_nvcsw),
          double(usage.ru_nivcsw),double(usage.ru_minflt),double(usage.ru_majflt)};
}
// Updated before queue admission, so dropped detailed records cannot hide peaks.
struct WindowStats {
  uint64_t cycles=0, overflow=0;
  double wall_max=0., cpu_max=0., voluntary=0., involuntary=0., minor=0., major=0.;
  void merge(const WindowStats& x) {
    cycles+=x.cycles; overflow+=x.overflow;
    wall_max=std::max(wall_max,x.wall_max); cpu_max=std::max(cpu_max,x.cpu_max);
    voluntary+=x.voluntary; involuntary+=x.involuntary; minor+=x.minor; major+=x.major;
  }
};
struct Record {
  double pid=0,tid=0,serial=0,session=0,generation=0,reference=0,dropped=0;
  size_t size=0; bool overflow=false; std::array<Point,kPoints> points{};
  WindowStats window;
};
class Collector {
 public:
  using Sink=std::function<void(const Record&)>;
  ~Collector(){stop();}
  void start(Sink sink) {
    sink_=std::move(sink); enabled_=true;
    worker_=std::thread([this]{
      while(!stopping_.load() || read_.load()!=write_.load()) {
        const size_t r=read_.load(std::memory_order_relaxed);
        if(r==write_.load(std::memory_order_acquire)) {
          std::this_thread::sleep_for(std::chrono::milliseconds(2));continue;
        }
        // Copy before freeing the slot. Sink/serialization run only here.
        const Record record=queue_[r%capacity_];
        read_.store(r+1,std::memory_order_release);
        try{sink_(record);}catch(...){++sink_errors_;}
      }
    });
  }
  bool enabled()const noexcept{return enabled_;}
  void submit(Record& record) noexcept {
    ++pending_.cycles; pending_.overflow+=record.overflow;
    if(record.size>=2) {
      const auto& a=record.points[0]; const auto& b=record.points[record.size-1];
      pending_.wall_max=std::max(pending_.wall_max,b.wall-a.wall);
      pending_.cpu_max=std::max(pending_.cpu_max,b.cpu-a.cpu);
      pending_.voluntary+=b.voluntary-a.voluntary; pending_.involuntary+=b.involuntary-a.involuntary;
      pending_.minor+=b.minor-a.minor; pending_.major+=b.major-a.major;
    }
    const size_t w=write_.load(std::memory_order_relaxed);
    if(w-read_.load(std::memory_order_acquire)>=capacity_){++dropped_;return;}
    record.dropped=double(dropped_.load()+sink_errors_.load());
    record.window=pending_;
    queue_[w%capacity_]=record;
    pending_={};
    write_.store(w+1,std::memory_order_release);
  }
  void stop() noexcept {
    stopping_.store(true); if(worker_.joinable())worker_.join();
    // Called after the producer loop stops. Flush a tail that never entered the queue.
    if(sink_ && enabled_) {
      Record tail; tail.window=pending_; tail.dropped=double(dropped_.load()+sink_errors_.load());
      try { sink_(tail); } catch(...) { ++sink_errors_; }
      pending_={}; enabled_=false;
    }
  }
 private:
  static constexpr size_t capacity_=128;
  std::array<Record,capacity_> queue_{};
  std::atomic<size_t> read_{0},write_{0};
  std::atomic<uint64_t> dropped_{0},sink_errors_{0};
  std::atomic<bool> stopping_{false}; bool enabled_=false;
  WindowStats pending_;  // sole producer, read by stop only after producer exit
  Sink sink_;std::thread worker_;
};
class Scope;
inline Scope*& current(){static thread_local Scope* scope=nullptr;return scope;}
class Scope {
 public:
  Scope(Collector& collector,uint64_t serial):collector_(collector) {
    if(!collector.enabled())return;
    active_=true;previous_=current();current()=this;
    record_.pid=getpid();record_.tid=syscall(SYS_gettid);record_.serial=double(serial);mark(0);
  }
  ~Scope() noexcept {
    if(!active_)return;
    mark(99);current()=previous_;collector_.submit(record_);
  }
  void mark(int id) noexcept {
    if(!active_)return;
    if(record_.size<kPoints)record_.points[record_.size++]=sample(id);
    else record_.overflow=true;
  }
  void reference(uint64_t sequence) noexcept {record_.reference=double(sequence);}
  void identity(uint64_t session,uint64_t generation,uint64_t reference) noexcept {
    record_.session=double(session);record_.generation=double(generation);record_.reference=double(reference);
  }
 private:
  Collector& collector_;Record record_{};Scope* previous_=nullptr;bool active_=false;
};
inline void point(int id) noexcept {if(current())current()->mark(id);}
} // namespace runtime_probe
} // namespace velocity_ik
