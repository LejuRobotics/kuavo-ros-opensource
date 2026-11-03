#pragma once
// Optional bounded SPSC observer. No allocation, file IO, locks, or waits in push.
// One instance per producer; the writer explicitly leaves inherited RT policy.
#include <array>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <string>
#include <thread>
#include <pthread.h>
namespace kuavo_e2e {
inline double ns(){return std::chrono::duration<double,std::nano>(std::chrono::steady_clock::now().time_since_epoch()).count();}
template<size_t N,size_t Capacity=1024> class Trace {
 public:
  using Row=std::array<double,N>;
  explicit Trace(const char* name):name_(name){const char* p=std::getenv("KUAVO_E2E_TRACE_DIR");if(p&&*p){path_=std::string(p)+"/"+name;enabled_=true;worker_=std::thread([this]{run();});}}
  ~Trace(){stop_.store(true);if(worker_.joinable())worker_.join();}
  bool enabled()const{return enabled_;}
  uint64_t dropped()const{return dropped_.load(std::memory_order_relaxed);}
  void push(const Row& row){if(!enabled_)return;auto w=write_.load(std::memory_order_relaxed);if(w-read_.load(std::memory_order_acquire)>=Capacity){++dropped_;return;}rows_[w%Capacity]=row;write_.store(w+1,std::memory_order_release);}
 private:
  void run(){sched_param p{};pthread_setschedparam(pthread_self(),SCHED_OTHER,&p);std::ofstream f(path_+".bin",std::ios::binary);if(!f){enabled_=false;return;}
    while(!stop_.load()||read_.load()!=write_.load()){auto r=read_.load(std::memory_order_relaxed),w=write_.load(std::memory_order_acquire);if(r==w){f.flush();std::this_thread::sleep_for(std::chrono::milliseconds(4));continue;}f.write(reinterpret_cast<const char*>(rows_[r%Capacity].data()),sizeof(Row));read_.store(r+1,std::memory_order_release);}
    f.close();std::ofstream s(path_+".meta");s<<"columns "<<N<<"\nrows "<<read_.load()<<"\ndropped "<<dropped()<<"\n";
  }
  std::string name_,path_;std::atomic<bool> enabled_{false},stop_{false};std::atomic<uint64_t> write_{0},read_{0},dropped_{0};std::array<Row,Capacity> rows_{};std::thread worker_;
};
}
