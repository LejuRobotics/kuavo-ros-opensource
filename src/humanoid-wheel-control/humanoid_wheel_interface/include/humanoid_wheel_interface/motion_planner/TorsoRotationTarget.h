#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>

namespace ocs2 {

// 旋转组只改变腰部 q2/q3。q0/q1 是进入该组时的快照。
// pitch/yaw 范围沿用手柄躯干工作空间，且与模型关节限位取交集。
inline Eigen::Vector4d integrateTorsoRotationTarget(
    const Eigen::Vector4d& target, double pitchVelocity, double yawVelocity,
    double dt, const Eigen::Vector4d& lower, const Eigen::Vector4d& upper) {
  Eigen::Vector4d result = target;
  if (!std::isfinite(dt) || dt <= 0.0 || !std::isfinite(pitchVelocity) ||
      !std::isfinite(yawVelocity)) return result;
  const double pitchOffset = target[0] + target[1];
  const double pitchLower = std::max(lower[2], -pitchOffset);
  const double pitchUpper = std::min(upper[2], 0.5235 - pitchOffset);
  const double yawLower = std::max(lower[3], -0.5235);
  const double yawUpper = std::min(upper[3], 0.5235);
  // 已在工作空间外的姿态只允许向内移动，避免首次进入时位置阶跃。
  if (pitchLower <= pitchUpper) {
    const double lo = std::min(target[2], pitchLower);
    const double hi = std::max(target[2], pitchUpper);
    result[2] = std::max(lo, std::min(hi, target[2] + pitchVelocity * dt));
  }
  if (yawLower <= yawUpper) {
    const double lo = std::min(target[3], yawLower);
    const double hi = std::max(target[3], yawUpper);
    result[3] = std::max(lo, std::min(hi, target[3] + yawVelocity * dt));
  }
  return result;
}

}  // namespace ocs2
