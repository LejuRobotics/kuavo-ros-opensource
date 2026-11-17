#pragma once

#include <cmath>

namespace velocity_ik {
inline double fixedNaturalHz(double cutoff) {
  return cutoff / std::sqrt(std::sqrt(2.) - 1.);
}

// Exact critically damped step for a target moving at constant velocity.
// Coefficients depend only on the tick and bandwidth and can serve every joint.
class CriticalDampingStep {
 public:
  CriticalDampingStep(double dt, double natural_hz)
      : dt_(dt), omega_(2. * 3.14159265358979323846 * natural_hz),
        decay_(std::exp(-omega_ * dt)) {}

  template <class Value>
  void advance(const Value& target, const Value& target_velocity,
               Value& position, Value& velocity) const {
    const Value error = position - (target - target_velocity * dt_);
    const Value relative_velocity = velocity - target_velocity;
    const Value c = relative_velocity + omega_ * error;
    position = target + (error + c * dt_) * decay_;
    velocity = target_velocity + (relative_velocity - omega_ * c * dt_) * decay_;
  }

 private:
  double dt_, omega_, decay_;
};
}  // namespace velocity_ik
