#pragma once

#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>
#include <utility>

namespace sg100 {

// HardwareNode owns this worker. Attempts never run on a ROS/control callback
// thread, never overlap, and stop() joins any in-flight attempt before cleanup.
class InitializationRetry {
 public:
  ~InitializationRetry() { stop(); }

  void start(std::chrono::steady_clock::duration interval,
             std::function<bool()> attempt, std::function<bool()> cancelled) {
    stop();
    stopping_ = false;
    worker_ = std::thread([this, interval, attempt = std::move(attempt),
                           cancelled = std::move(cancelled)] {
      std::unique_lock<std::mutex> lock(mutex_);
      while (!stopping_ && !cancelled()) {
        // The caller already made the first attempt. Match beta's five-second
        // respawn delay, including when initialization itself takes some time.
        if (wake_.wait_for(lock, interval, [this] { return stopping_; })) break;
        if (cancelled()) break;
        lock.unlock();
        const bool ready = attempt();  // Owner handles errors and partial cleanup.
        lock.lock();
        if (ready) break;
      }
    });
  }

  // Called only by the lifecycle owner, never from the attempt callback.
  void stop() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
    }
    wake_.notify_all();
    if (worker_.joinable()) worker_.join();
  }

 private:
  std::mutex mutex_;
  std::condition_variable wake_;
  bool stopping_{true};
  std::thread worker_;
};

}  // namespace sg100
