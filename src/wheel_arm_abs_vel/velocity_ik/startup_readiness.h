#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <iterator>
#include <stdexcept>
#include <vector>

namespace velocity_ik {
// First acquisition only. Never filters the commanded motion or relaxes
// PREPARE completion, stop-first recovery, freshness, or execution ownership.
class StartupReadiness {
 public:
  static constexpr int N=21;
  using Values=std::array<double,N>;
  struct Settings {
    double window=.080, median_window=.010;
    double arm_position=.002, lower_position=.001;
    double arm_speed=.050, lower_speed=.030;
    double arm_raw_speed=.150, lower_raw_speed=.100;
    double raw_dwell=.010, extreme_speed=.300;
    void validate() const {
      for(double x:{window,median_window,arm_position,lower_position,arm_speed,
                    lower_speed,arm_raw_speed,lower_raw_speed,raw_dwell,extreme_speed})
        if(!std::isfinite(x)||x<=0)throw std::invalid_argument("invalid startup readiness setting");
      if(window<median_window || window<raw_dwell || arm_speed>arm_raw_speed ||
         lower_speed>lower_raw_speed || extreme_speed<std::max(arm_raw_speed,lower_raw_speed))
        throw std::invalid_argument("inconsistent startup readiness setting");
    }
  } settings;
  // 0 ready, 1 filling window, 2 position motion, 3 robust velocity,
  // 4 raw sustained/extreme motion, 5 stale/invalid/time discontinuity.
  bool ready=false;
  int reason=1, joint=-1;
  double span=0, position_range=0, speed_rms=0, raw_max=0;
  void reset() { raw_.clear();window_.clear();last_=0;hard_since_.fill(0);ready=false;last_ready_=false;reason=1;span=0; }
  bool update(double stamp,const Values& q,const Values& v,bool fresh) {
    ready=false;raw_max=0;
    if(!fresh || !std::isfinite(stamp) || stamp<=0) {reset();reason=5;return false;}
    for(int j=0;j<N;++j)if(!std::isfinite(q[j])||!std::isfinite(v[j])) {reset();reason=5;return false;}
    if(last_ && stamp==last_) {ready=last_ready_;return ready;}
    if(last_ && (stamp<last_ || stamp-last_>.020))reset();
    last_=stamp;
    Sample s{stamp,q,v,false};raw_.push_back(s);
    while(raw_.size()>1 && raw_[1].t<=stamp-settings.median_window)raw_.pop_front();
    for(int j=0;j<N;++j) {
      raw_max=std::max(raw_max,std::abs(v[j]));
      const double cap=j>=7?settings.arm_raw_speed:settings.lower_raw_speed;
      if(std::abs(v[j])>cap) {if(!hard_since_[j])hard_since_[j]=stamp;}
      else hard_since_[j]=0;
      if(std::abs(v[j])>settings.extreme_speed ||
         (hard_since_[j] && stamp-hard_since_[j]>=settings.raw_dwell-1e-9))s.hard=true;
      std::vector<double> x; x.reserve(raw_.size());
      std::transform(raw_.begin(),raw_.end(),std::back_inserter(x),
                     [j](const Sample& sample){return sample.v[j];});
      auto m=x.begin()+x.size()/2;std::nth_element(x.begin(),m,x.end());s.v[j]=*m;
    }
    window_.push_back(s);
    while(window_.size()>1 && window_[1].t<=stamp-settings.window)window_.pop_front();
    span=stamp-window_.front().t;joint=-1;position_range=0;speed_rms=0;
    reason=span+1e-9<settings.window?1:0;
    if(std::any_of(window_.begin(),window_.end(),
                   [](const Sample& sample){return sample.hard;}))reason=4;
    double worst=0;
    for(int j=0;j<N;++j) {
      double lo=q[j],hi=q[j],sum=0;
      for(const auto& a:window_){lo=std::min(lo,a.q[j]);hi=std::max(hi,a.q[j]);sum+=a.v[j]*a.v[j];}
      const double rms=std::sqrt(sum/window_.size());
      const double pc=j>=7?settings.arm_position:settings.lower_position;
      const double vc=j>=7?settings.arm_speed:settings.lower_speed;
      const double risk=std::max((hi-lo)/pc,rms/vc);
      if(risk>worst){worst=risk;joint=j;position_range=hi-lo;speed_rms=rms;}
      if(reason!=4 && (hi-lo)>pc)reason=2;
      else if(reason==0 && rms>vc)reason=3;
    }
    ready=reason==0;last_ready_=ready;return ready;
  }
 private:
  struct Sample {double t;Values q,v;bool hard;};
  std::deque<Sample> raw_,window_;
  Values hard_since_{};
  double last_=0;
  bool last_ready_=false;
};
}
