#pragma once
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace velocity_ik {
// Dev's quintic re-grip handoff, applied to x/height/pitch BEFORE the velocity
// QP. The certified joint reference remains the sole execution trajectory;
// yaw, arm clutches and all joint velocity/acceleration bounds are untouched.
class LowerBodyGripHandoff {
 public:
  static double checkedDuration(double value) {
    return std::isfinite(value) && value >= .2 && value <= 5. ? value : 1.;
  }
  void reset() { initialized_ = active_ = held_ = false; }
  Eigen::Vector3d step(uint64_t session, uint64_t generation, bool held,
      const Eigen::Vector3d& published, const Eigen::Vector3d& target,
      double now, double duration) {
    if (!initialized_ || session != session_ || generation != generation_) {
      initialized_ = true;
      session_ = session; generation_ = generation;
      active_ = false; held_ = held;
    }
    if (held) {
      anchor_ = published;
      active_ = false;
      held_ = true;
      return published;
    }
    if (held_) {
      anchor_ = published;
      start_ = now;
      active_ = true;
    }
    held_ = false;
    if (!active_) return target;
    const double u = std::clamp((now - start_) / checkedDuration(duration), 0., 1.);
    const double weight = u*u*u*(10. + u*(-15. + 6.*u));
    if (u >= 1.) active_ = false;
    return anchor_ + weight*(target-anchor_);
  }
 private:
  bool initialized_{false}, held_{false}, active_{false};
  uint64_t session_{0}, generation_{0};
  double start_{0.};
  Eigen::Vector3d anchor_{Eigen::Vector3d::Zero()};
};
}  // namespace velocity_ik
