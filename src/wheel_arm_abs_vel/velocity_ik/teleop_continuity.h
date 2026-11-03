#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace velocity_ik {
struct InputContinuityConfig {
  double grace=.060, brake=.150, lost=.500, recovery=.100;
  int recovery_frames=3;
  bool hold_on_timeout=true;
  bool valid() const {
    return std::isfinite(grace) && std::isfinite(brake) && std::isfinite(lost) &&
        std::isfinite(recovery) && grace>0 && brake>grace && lost>brake &&
        recovery>0 && recovery_frames>=3;
  }
};
enum class InputPhase { Fresh=0, Degraded=1, Braking=2, Recovering=3, Confirm=4 };

// Called only by the reference owner's steady-time tick. Sequence is the
// accepted SOURCE observation count, not the 500 Hz loop or transport count.
class InputContinuity {
 public:
  explicit InputContinuity(const InputContinuityConfig& c={}) : config(c) {}
  InputContinuityConfig config;
  InputPhase phase=InputPhase::Braking;
  bool confirmation_required=false, interrupted=false, stable=false;
  double budget=0.;
  bool admitsAge(double age, bool automatic_hold_enabled=true) const {
    return std::isfinite(age) && age>=0. &&
        (!automatic_hold_enabled || !config.hold_on_timeout || age<=config.brake);
  }
  void suspend() {
    // Explicit PREPARE/ARMED/HOLD is not an ACTIVE VR outage. Preserve an
    // already latched loss, but do not turn a deliberate long pause into one.
    seen_=false; interrupted=true; stable=false; good_since_=-1.; frames_=0;
    budget=0.; phase=confirmation_required ? InputPhase::Confirm : InputPhase::Braking;
  }
  void update(double now, double age, uint64_t observation, bool admitted,
              bool confirm, bool stopped, bool automatic_hold_enabled=true) {
    const bool fresh=admitted && std::isfinite(age) && age>=0 && age<=config.grace;
    if (!std::isfinite(age) || age>=config.brake || !admitted) interrupted=true;
    // No observation at startup is not itself a previously active lost link.
    if (automatic_hold_enabled && config.hold_on_timeout && seen_ && (!std::isfinite(age) || age>=config.lost)) confirmation_required=true;
    if (admitted && std::isfinite(age)) seen_=true;
    if (!fresh) { good_since_=-1.; frames_=0; stable=false; }
    else if (observation!=last_observation_) {
      if (good_since_<0) good_since_=now;
      ++frames_;
      stable=frames_>=config.recovery_frames && now-good_since_>=config.recovery;
    }
    last_observation_=observation;
    // After startup the operator can opt out of automatic pause/re-confirmation.
    // The callback still admits only finite poses from the configured source and
    // never borrows a cache across a transaction/coordinate identity change.
    // Explicit stop is handled by suspend()/admitted=false, not this policy.
    if (!automatic_hold_enabled) confirmation_required=false;
    if (confirmation_required && confirm && stable && stopped)
      confirmation_required=false;
    if (confirmation_required) { phase=InputPhase::Confirm; budget=0.; return; }
    // Opt-out applies only to elapsed input age. Keep the last admitted target
    // and normal IK tracking; explicit stop/invalid input/coordinate confirmation
    // still have their existing meanings. No synthetic observations are created.
    if ((!automatic_hold_enabled || !config.hold_on_timeout) &&
        admitted && admitsAge(age, automatic_hold_enabled)) {
      interrupted=false;
      phase=fresh ? InputPhase::Fresh : InputPhase::Degraded;
      budget=1.;
      return;
    }
    if (!admitted || !std::isfinite(age) || age<0. || age>=config.brake) {
      phase=InputPhase::Braking; budget=0.; return;
    }
    if (interrupted) {
      if (!stable || !stopped) { phase=InputPhase::Recovering; budget=0.; return; }
      interrupted=false;
    }
    phase=fresh ? InputPhase::Fresh : InputPhase::Degraded;
    budget=std::clamp((config.brake-age)/(config.brake-config.grace),0.,1.);
  }
  bool canTrack() const { return phase==InputPhase::Fresh || phase==InputPhase::Degraded; }
 private:
  bool seen_=false;
  double good_since_=-1.;
  int frames_=0;
  uint64_t last_observation_=0;
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
