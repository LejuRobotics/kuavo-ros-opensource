#pragma once
#include <algorithm>
#include <cmath>

namespace velocity_ik {
struct InputContinuityConfig {
  // grace marks fresh observations; brake bounds excess packet delay at
  // admission. Neither expires an already admitted target during follow.
  double grace=.060, brake=.150;
  bool valid() const {
    return std::isfinite(grace) && std::isfinite(brake) && grace>0 && brake>grace;
  }
};
enum class InputPhase { Fresh=0, Degraded=1, Suspended=2 };

// Called only by the reference owner's steady-time tick. Keep the last
// admitted target; explicit operation/validity changes suspend tracking.
class InputContinuity {
 public:
  explicit InputContinuity(const InputContinuityConfig& c={}) : config(c) {}
  InputContinuityConfig config;
  InputPhase phase=InputPhase::Suspended;
  double budget=0.;
  bool admitsAge(double age) const {
    return std::isfinite(age) && age>=0.;
  }
  void suspend() {
    budget=0.; phase=InputPhase::Suspended;
  }
  void update(double age, bool admitted) {
    if (!admitted || !admitsAge(age)) { suspend(); return; }
    phase=age<=config.grace ? InputPhase::Fresh : InputPhase::Degraded;
    budget=1.;
  }
  bool canTrack() const { return phase==InputPhase::Fresh || phase==InputPhase::Degraded; }
};

struct Acquisition {
  double settle=.100, release=1., progress=0.;
  bool catchup=true;
  void reset() { progress=0.; settled_=0.; catchup=true; }
  void update(double dt, bool aligned, bool fresh) {
    // Release the JOINT/FF startup budget independently of target distance.
    // A moving operator must not keep both arms indefinitely in slow mode.
    if (fresh) progress=std::min(1.,progress+dt/release);
    settled_=aligned && fresh ? settled_+dt : 0.;
    if (progress>=1. && settled_>=settle) catchup=false;
  }
  double blend() const { return progress*progress*progress*(10.+progress*(-15.+6.*progress)); }
 private:
  double settled_=0.;
};
} // namespace velocity_ik
