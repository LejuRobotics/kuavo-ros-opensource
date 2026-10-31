#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>

namespace ocs2 {

// 旋转组复位保留 knee/leg 快照，仅恢复初始化时的腰部 pitch/yaw。
inline Eigen::Vector4d makeTorsoRotationResetTarget(
    const Eigen::Vector4d& target, const Eigen::Vector4d& initial,
    const Eigen::Vector4d& lower, const Eigen::Vector4d& upper) {
  Eigen::Vector4d result = target;
  for (int joint = 2; joint < 4; ++joint) {
    result[joint] = std::max(lower[joint], std::min(upper[joint], initial[joint]));
  }
  return result;
}

// 旋转组只改变腰部 q2/q3。q0/q1 是进入该组时的快照。
// pitch 保留躯干工作空间限制；yaw 只遵守模型关节限位，手柄操作范围由 Joy 层限制。
inline Eigen::Vector4d integrateTorsoRotationTarget(
    const Eigen::Vector4d& target, double pitchVelocity, double yawVelocity,
    double dt, const Eigen::Vector4d& lower, const Eigen::Vector4d& upper) {
  Eigen::Vector4d result = target;
  if (!std::isfinite(dt) || dt <= 0.0 || !std::isfinite(pitchVelocity) ||
      !std::isfinite(yawVelocity)) return result;
  const double pitchOffset = target[0] + target[1];
  const double pitchLower = std::max(lower[2], -pitchOffset);
  const double pitchUpper = std::min(upper[2], 0.5235 - pitchOffset);
  const double yawLower = lower[3];
  const double yawUpper = upper[3];
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
