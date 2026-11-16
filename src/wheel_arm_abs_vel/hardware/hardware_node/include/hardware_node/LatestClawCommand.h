#pragma once
#include <mutex>
#include <chrono>
// A single immutable message reference; no vector/string copies or I/O under
// the lock. Pointer destruction happens after unlock, including overwrite.
template<class Ptr> class LatestClawCommand {
 public:
  struct Item {Ptr msg;std::chrono::steady_clock::time_point received{};bool high_freq=false;};
  bool put(Ptr msg,std::chrono::steady_clock::time_point received,bool high_freq) {
    Item old;
    {std::lock_guard<std::mutex> lock(mutex_);if(!accepting_)return false;
     old=std::move(latest_);latest_={std::move(msg),received,high_freq};}
    return true;
  }
  Item take(){Item out;{std::lock_guard<std::mutex> lock(mutex_);out=std::move(latest_);latest_=Item{};}return out;}
  void accepting(bool value){Item old;{std::lock_guard<std::mutex> lock(mutex_);accepting_=value;old=std::move(latest_);latest_=Item{};}}
 private:
  std::mutex mutex_;Item latest_;bool accepting_=true;
};
