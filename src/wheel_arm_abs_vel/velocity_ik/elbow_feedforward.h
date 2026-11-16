#pragma once
#include <Eigen/Core>
#include <cmath>
namespace velocity_ik {
// The caller supplies the existing source/frame/session/time continuity gate.
// Invalid samples break only this elbow's history; no hand/body state is reset.
inline Eigen::Vector3d continuousElbowFeedforward(
    bool previous_valid, bool current_valid, bool continuous,
    const Eigen::Vector3d& previous, const Eigen::Vector3d& current,
    const Eigen::Vector3d& previous_ff, double dt, double alpha) {
  if (!previous_valid || !current_valid || !continuous ||
      !previous.allFinite() || !current.allFinite() || !previous_ff.allFinite() ||
      !std::isfinite(dt) || dt < 0.003 || dt > 0.100 || !std::isfinite(alpha))
    return Eigen::Vector3d::Zero();
  const Eigen::Vector3d result=(1.0-alpha)*previous_ff+alpha*(current-previous)/dt;
  return result.allFinite()?result:Eigen::Vector3d::Zero();
}
}
