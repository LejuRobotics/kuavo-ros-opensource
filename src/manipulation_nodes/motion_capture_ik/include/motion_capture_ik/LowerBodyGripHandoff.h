#pragma once

#include <Eigen/Core>
#include <algorithm>

namespace HighlyDynamic {

// Bridge only knee/leg/waist_pitch. Yaw remains on its existing follow path.
struct LowerBodyGripHandoff {
  Eigen::Vector3d anchor = Eigen::Vector3d::Zero();
  bool active = false;
  double startTime = 0.0;

  void freeze(const Eigen::Vector3d& published) {
    anchor = published;
    active = false;
  }

  void resume(const Eigen::Vector3d& published, double now) {
    anchor = published;
    startTime = now;
    active = true;
  }

  Eigen::Vector3d apply(const Eigen::Vector3d& target, double now, double duration) {
    if (!active) return target;
    const double u = std::clamp((now - startTime) / duration, 0.0, 1.0);
    const double weight = u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
    if (u >= 1.0) active = false;
    return anchor + weight * (target - anchor);
  }
};

}  // namespace HighlyDynamic
