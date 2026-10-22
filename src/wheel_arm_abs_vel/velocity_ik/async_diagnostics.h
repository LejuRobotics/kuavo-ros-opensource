#pragma once
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>
namespace velocity_ik {
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
