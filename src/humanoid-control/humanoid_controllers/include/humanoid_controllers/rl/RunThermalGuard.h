#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace humanoid_controller {
// Caller serializes access and supplies a monotonic clock. No control or ROS IO.
class RunThermalGuard {
 public:
  struct Config {
    bool enabled=false;
    double entry=80, exit=95, recovery_hold=3, sample_timeout=1;
    double max_sample_drop=2;
  };
  struct Status {
    bool valid=false, can_enter=false, force_exit=false, cooling=true;
    double maximum=0, sample_age=0;
    std::vector<double> measured;
    const char* reason="waiting_temperature";
  };
  RunThermalGuard() : RunThermalGuard(Config{}) {}
  explicit RunThermalGuard(Config c) : cfg_(c) {
    for(double x:{c.entry,c.exit,c.recovery_hold,c.sample_timeout,c.max_sample_drop})
      if(!std::isfinite(x)||x<=0) throw std::invalid_argument("thermal values must be finite and positive");
    if(c.entry>=c.exit || c.exit>150) throw std::invalid_argument("thermal requires entry < exit <= 150C");
  }
  void sample(const std::vector<double>& values,double now) {
    bool ok=!values.empty() && std::isfinite(now) && (!seen_ || now>=received_);
    for(size_t i=0;i<values.size();++i) {
      ok=ok && std::isfinite(values[i]) && values[i]>0 && values[i]<=150;
      if(s_.measured.size()==values.size() && std::isfinite(values[i]) &&
         s_.measured[i]-values[i]>cfg_.max_sample_drop) drop_fault_=true;
    }
    if(ok && !drop_fault_) {
      const double maximum=*std::max_element(values.begin(),values.end());
      if(maximum>=cfg_.exit) trip_=true;
    }
    // A sudden drop is latched for this process, never accepted as a cold reset.
    seen_=true; received_=now; sample_ok_=ok && !drop_fault_;
    s_.measured=values;
  }
  void update(double now,bool running) {
    const bool backwards=last_>=0 && now<last_;
    last_=now;
    s_.sample_age=seen_ ? now-received_ : std::numeric_limits<double>::infinity();
    s_.valid=seen_ && sample_ok_ && !backwards && s_.sample_age>=0 && s_.sample_age<=cfg_.sample_timeout;
    // The hottest current NTC is the sole temperature used by protection.
    s_.maximum=s_.valid ? *std::max_element(s_.measured.begin(),s_.measured.end()) : 0;
    if(!s_.valid || s_.maximum>=cfg_.exit) {
      if(running) { trip_=true; }
      cold_since_=-1;
    }
    // Entry and recovery share one inclusive NTC threshold, independent of commands.
    const bool cool_enough=s_.maximum<=cfg_.entry;
    if(!running && s_.valid && cool_enough) {
      if(cold_since_<0) cold_since_=now;
      if(now-cold_since_>=cfg_.recovery_hold) trip_=false;
    } else cold_since_=-1;
    s_.force_exit=cfg_.enabled && running && (trip_ || !s_.valid || s_.maximum>=cfg_.exit);
    s_.can_enter=!cfg_.enabled || (!running && s_.valid && !trip_ &&
      s_.maximum<=cfg_.entry && cold_since_>=0 && now-cold_since_>=cfg_.recovery_hold);
    s_.cooling=cfg_.enabled && (running ? s_.force_exit : !s_.can_enter);
    const char* invalid_reason=drop_fault_ ? "temperature_drop_fault" :
      !seen_ ? "waiting_temperature" :
      !sample_ok_ ? "temperature_invalid" :
      (backwards || !std::isfinite(s_.sample_age) || s_.sample_age<0) ? "temperature_clock_error" :
      "temperature_stale";
    s_.reason=!cfg_.enabled ? "disabled" : !s_.valid ? invalid_reason :
      s_.force_exit ? "thermal_exit" : s_.cooling ? "cooling" : "ready";
  }
  static const char* statusLabel(const Status& status, bool exit_timeout) {
    if(!status.valid || exit_timeout) return "fault";
    return status.cooling ? "cooling" : "ready";
  }
  static std::string statusMessage(const Status& status, bool exit_timeout) {
    // A stalled exit takes precedence; all temperature faults share one label.
    if(exit_timeout) return "fault: exit_timeout";
    if(!status.valid) return "fault: temperature_invalid";
    return statusLabel(status,false);
  }

  // Once physical guards have admitted RUN->WALK, completing its blend is
  // progress, not a stalled exit. Do not inject a timeout stop into that blend.
  static bool exitTimedOut(bool blending_out_of_run, double elapsed, double timeout) {
    return !blending_out_of_run && elapsed>=timeout;
  }
  // running includes both sides of a blend involving RUN. Cooling/re-entry
  // qualification must never impose a speed cap on ordinary WALK.
  static double exitVelocityLimit(bool enabled, bool running, bool force_exit,
                                  bool timed_out, double limit) {
    return enabled && running && force_exit ? (timed_out ? 0.0 : limit)
                                           : std::numeric_limits<double>::infinity();
  }
  void latchExit() { trip_=true; }
  const Status& status() const { return s_; }
 private:
  Config cfg_; Status s_;
  bool seen_=false,sample_ok_=false,drop_fault_=false,trip_=false;
  double received_=0,last_=-1,cold_since_=-1;
};
}
