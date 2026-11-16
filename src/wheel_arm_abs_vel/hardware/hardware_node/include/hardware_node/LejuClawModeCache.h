#pragma once
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>
#include <utility>
#include <pthread.h>

namespace HighlyDynamic {
// Only the refresh worker performs ROS parameter RPC. Coherent configuration
// snapshots are lock-free; joystick age is still evaluated on EVERY hot read.
class LejuClawModeCache {
 public:
  static constexpr int kDefaultTimeoutMs=1000;
  using Reader=std::function<std::pair<bool,int>()>;
  ~LejuClawModeCache(){stop();}
  void store(bool manual,int timeout_ms) noexcept {
    if(timeout_ms<=0)timeout_ms=kDefaultTimeoutMs;
    config_.store((uint64_t(uint32_t(timeout_ms))<<1)|uint64_t(manual),std::memory_order_relaxed);
  }
  bool highFrequency(int64_t last_us,int64_t now_us) const noexcept {
    const uint64_t config=config_.load(std::memory_order_relaxed);
    return (config&1) || (last_us!=0 && now_us-last_us<=int64_t(config>>1)*1000);
  }
  void start(Reader reader) {
    if(worker_.joinable())return;
    const auto initial=reader();store(initial.first,initial.second);
    worker_=std::thread([this,reader] {
      sched_param p{};
      if(pthread_setschedparam(pthread_self(),SCHED_OTHER,&p)!=0)return;
      std::unique_lock<std::mutex> lock(mutex_);
      while(!wake_.wait_for(lock,std::chrono::milliseconds(500),[this]{return stopping_;})) {
        lock.unlock();
        const auto value=reader();store(value.first,value.second);
        lock.lock();
      }
    });
  }
  void stop() noexcept {
    {std::lock_guard<std::mutex> lock(mutex_);stopping_=true;}
    wake_.notify_all();if(worker_.joinable())worker_.join();
  }
 private:
  static_assert(ATOMIC_LLONG_LOCK_FREE==2,"LejuClaw cache requires lock-free 64-bit atomics");
  std::atomic<uint64_t> config_{uint64_t(kDefaultTimeoutMs)<<1};
  std::mutex mutex_;std::condition_variable wake_;bool stopping_=false;std::thread worker_;
};
}  // namespace HighlyDynamic
