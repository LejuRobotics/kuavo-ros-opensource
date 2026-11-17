#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace humanoid_controller {
// Forward command envelope in raw command units, before model scaling.
// RUN owns independent rise/fall constants. WALK inherits only the remaining
// deceleration tail, never the RUN acceleration setting.
class ForwardVelocitySmoother {
 public:
  void configure(double decel_tau, double accel_tau = 0) {
    if (!std::isfinite(decel_tau) || decel_tau < 0)
      throw std::invalid_argument("runDecelTau must be finite and nonnegative");
    if (!std::isfinite(accel_tau) || accel_tau < 0)
      throw std::invalid_argument("runAccelTau must be finite and nonnegative");
    decel_tau_ = decel_tau;
    accel_tau_ = accel_tau;
    reset();
  }
  void reset() { valid_ = false; value_ = inherited_decel_tau_ = time_ = 0; }
  void inherit(const ForwardVelocitySmoother& source) {
    value_ = source.value_; time_ = source.time_; valid_ = source.valid_;
    inherited_decel_tau_ = source.decel_tau_ > 0 ? source.decel_tau_ : source.inherited_decel_tau_;
  }
  double value() const { return value_; }
  bool active() const {
    return valid_ && (decel_tau_ > 0 || accel_tau_ > 0 || inherited_decel_tau_ > 0);
  }
  double update(double target, double now) {
    if (!std::isfinite(target) || !std::isfinite(now)) { reset(); return 0; }
    if (valid_ && now < time_) reset();
    if (!valid_) {
      // Start a cold activation at rest; a warm handoff uses inherit() instead.
      value_ = target > 0 && accel_tau_ > 0 ? 0 : target;
      valid_ = true; time_ = now;
      return value_;
    }
    if (target < 0) { // no additional lag on a reverse command
      value_ = target;
      inherited_decel_tau_ = 0;
    } else {
      value_ = std::max(0.0, value_);
      const bool rising = target > value_;
      const double tau = rising ? accel_tau_ :
          (decel_tau_ > 0 ? decel_tau_ : inherited_decel_tau_);
      if (target >= value_) inherited_decel_tau_ = 0;
      if (tau <= 0) value_ = target;
      else value_ = target + (value_ - target) * std::exp(-(now - time_) / tau);
      if (std::abs(value_ - target) < 1e-4) {
        value_ = target;
        inherited_decel_tau_ = 0;
      }
    }
    time_ = now;
    return value_;
  }
 private:
  double decel_tau_ = 0, accel_tau_ = 0, inherited_decel_tau_ = 0, value_ = 0, time_ = 0;
  bool valid_ = false;
};
} // namespace humanoid_controller
