#pragma once
#include <time.h>
#include <cstdlib>
#include <cstdint>
#include <cerrno>
namespace kuavo_e2e {
// One optional monotonic 2 ms grid. A late job skips expired releases; it never
// publishes catch-up copies or changes integration/physics dt to conceal loss.
class Deadline {
 public:
  explicit Deadline(int64_t phase):phase_(phase),enabled_(std::getenv("KUAVO_E2E_PHASED_2MS")!=nullptr){}
  bool enabled()const{return enabled_;}
  void wait(){
    timespec now{};clock_gettime(CLOCK_MONOTONIC,&now);
    const int64_t n=int64_t(now.tv_sec)*1000000000LL+now.tv_nsec;
    if(next_==0)next_=((n-phase_)/period_+1)*period_+phase_;
    if(next_<n){missed_+=(n-next_)/period_+1;next_=((n-phase_)/period_+1)*period_+phase_;}
    timespec due{time_t(next_/1000000000LL),long(next_%1000000000LL)};
    while(clock_nanosleep(CLOCK_MONOTONIC,TIMER_ABSTIME,&due,nullptr)==EINTR){}
    next_+=period_;
  }
  uint64_t missed()const{return missed_;}
 private:
  static constexpr int64_t period_=2000000;int64_t phase_,next_{0};bool enabled_;uint64_t missed_{0};
};
}
