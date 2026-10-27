#pragma once
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>
#include <type_traits>
#include <stdexcept>
namespace velocity_ik {
// Tick identity belongs to the supervisor, never a reference/session counter.
inline bool diagnosticPhaseDue(uint64_t tick, int stride, unsigned phase) {
  return stride > 0 && tick % static_cast<unsigned>(stride) == phase % static_cast<unsigned>(stride);
}

// Single RT producer, single background consumer. No std::function creation,
// allocation, producer mutex, or CAS retry in submit(). Full queue drops mirrors
// only; the authoritative SHM commit has already completed.
template<class T, size_t Capacity = 16> class AsyncPodDiagnostics {
  static_assert(std::is_trivially_copyable<T>::value, "POD snapshot required");
 public:
  ~AsyncPodDiagnostics() { stop(); }
  void start(std::function<void(const T&)> handler) {
    if (worker_.joinable()) throw std::logic_error("POD worker already started");
    handler_ = std::move(handler);
    stopping_.store(false, std::memory_order_release);
    worker_ = std::thread([this] {
      for (;;) {
        const size_t read = read_.load(std::memory_order_relaxed);
        if (read != write_.load(std::memory_order_acquire)) {
          // Release the slot before invoking potentially slow ROS publication.
          const T snapshot = slots_[read % Capacity];
          read_.store(read + 1, std::memory_order_release);
          handler_(snapshot);
        } else {
          if (stopping_.load(std::memory_order_acquire)) return;
          std::unique_lock<std::mutex> lock(wait_mutex_);
          wake_.wait_for(lock, std::chrono::milliseconds(2));
        }
      }
    });
  }
  bool submit(const T& snapshot) {
    if (stopping_.load(std::memory_order_acquire)) { ++dropped_; return false; }
    const size_t write = write_.load(std::memory_order_relaxed);
    if (write - read_.load(std::memory_order_acquire) >= Capacity) { ++dropped_; return false; }
    slots_[write % Capacity] = snapshot;
    write_.store(write + 1, std::memory_order_release);
    wake_.notify_one();
    return true;
  }
  uint64_t dropped() const { return dropped_.load(std::memory_order_relaxed); }
  void stop() {
    stopping_.store(true, std::memory_order_release); wake_.notify_one();
    if (worker_.joinable()) worker_.join();
  }
 private:
  std::array<T, Capacity> slots_{};
  alignas(64) std::atomic<size_t> write_{0};
  alignas(64) std::atomic<size_t> read_{0};
  std::atomic<uint64_t> dropped_{0};
  std::atomic<bool> stopping_{true};
  std::function<void(const T&)> handler_;
  std::thread worker_;
  std::condition_variable wake_; std::mutex wait_mutex_;
};

// Bounded multi-producer, single-consumer ring. Producers never wait for the
// logger or its wakeup mutex. Per-slot sequence numbers publish immutable jobs.
class AsyncDiagnostics {
 public:
  AsyncDiagnostics() {
    for (size_t i=0;i<capacity_;++i) slots_[i].sequence.store(i);
    worker_=std::thread([this]{run();});
  }
  ~AsyncDiagnostics(){stop();}
  bool submit(std::function<void()> job) const {
    if(stopping_.load(std::memory_order_acquire)){++dropped_;return false;}
    size_t pos=enqueue_.load(std::memory_order_relaxed);
    Cell* cell;
    for(;;){
      cell=&slots_[pos%capacity_];
      const auto difference=static_cast<intptr_t>(cell->sequence.load(std::memory_order_acquire))-static_cast<intptr_t>(pos);
      if(difference==0){if(enqueue_.compare_exchange_weak(pos,pos+1,std::memory_order_relaxed))break;}
      else if(difference<0){++dropped_;return false;}
      else pos=enqueue_.load(std::memory_order_relaxed);
    }
    cell->job=std::move(job);
    cell->sequence.store(pos+1,std::memory_order_release);
    wake_.notify_one();return true;
  }
  uint64_t dropped()const{return dropped_.load();}
  void stop(){stopping_.store(true,std::memory_order_release);wake_.notify_one();if(worker_.joinable())worker_.join();}
 private:
  static constexpr size_t capacity_=1024;
  struct Cell { std::atomic<size_t> sequence{0};std::function<void()> job; };
  bool ready()const{return slots_[dequeue_%capacity_].sequence.load(std::memory_order_acquire)==dequeue_+1;}
  void run(){
    for(;;){
      if(ready()){
        auto& cell=slots_[dequeue_%capacity_];auto job=std::move(cell.job);
        cell.sequence.store(dequeue_+capacity_,std::memory_order_release);++dequeue_;job();
      }else{
        if(stopping_.load(std::memory_order_acquire)&&dequeue_==enqueue_.load(std::memory_order_acquire))return;
        // Timeout also covers a notification arriving before wait starts.
        std::unique_lock<std::mutex> lock(wake_mutex_);
        wake_.wait_for(lock,std::chrono::milliseconds(2),[this]{return ready()||stopping_.load();});
      }
    }
  }
  mutable std::array<Cell,capacity_> slots_;
  mutable std::atomic<size_t> enqueue_{0};size_t dequeue_=0;
  mutable std::atomic<uint64_t> dropped_{0};std::atomic<bool> stopping_{false};
  mutable std::condition_variable wake_;std::mutex wake_mutex_;std::thread worker_;
};
}
