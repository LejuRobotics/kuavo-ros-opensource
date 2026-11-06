#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>

#include "humanoid_wheel_interface/motion_planner/TorsoYawTarget.h"

namespace ocs2 {

// 从旋转 FK 接续平移：包络外的目标不跳变，也不能继续向外累计。
inline double clampTorsoTranslationFromRotation(double requested, double previous, double lower, double upper) {
  return std::max(std::min(lower, previous), std::min(std::max(upper, previous), requested));
}

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
// pitch 保留躯干工作空间限制；连续速度积分的 yaw 遵守模型关节限位。
// VR A/B oneshot 走 applyTorsoRotationJointDelta，额外保留 kTorsoYawTargetLimit 裕量。
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


// 旋转组内 oneshot 相对增量（来自 /cmd_torso_delta 的 yaw/pitch）。
// 不退出旋转组；yaw 先套用 kTorsoYawTargetLimit，再套模型关节限位。
inline Eigen::Vector4d applyTorsoRotationJointDelta(
    const Eigen::Vector4d& target, double pitchDelta, double yawDelta,
    const Eigen::Vector4d& lower, const Eigen::Vector4d& upper) {
  Eigen::Vector4d result = target;
  if (!std::isfinite(pitchDelta) || !std::isfinite(yawDelta)) return result;
  const double pitchOffset = target[0] + target[1];
  const double pitchLower = std::max(lower[2], -pitchOffset);
  const double pitchUpper = std::min(upper[2], 0.5235 - pitchOffset);
  if (pitchLower <= pitchUpper) {
    const double lo = std::min(target[2], pitchLower);
    const double hi = std::max(target[2], pitchUpper);
    result[2] = std::max(lo, std::min(hi, target[2] + pitchDelta));
  }
  const double yawLower = lower[3];
  const double yawUpper = upper[3];
  if (yawLower <= yawUpper) {
    const double requested = clampTorsoYawTarget(target[3] + yawDelta);
    const double lo = std::min(target[3], yawLower);
    const double hi = std::max(target[3], yawUpper);
    result[3] = std::max(lo, std::min(hi, requested));
  }
  return result;
}

// /cmd_torso_delta 相对旋转组状态的处置：已在旋转则累计关节；否则 yaw/pitch-only 先暂存，
// 避免“delta 回调先于 rotation_vel、中间跑一次更新”时被笛卡尔路径消费后切组丢目标。
enum class TorsoDeltaDisposition {
  kAsRotationJoint = 0,
  kDeferPending = 1,
  kAsCartesian = 2,
};

inline TorsoDeltaDisposition classifyTorsoOneshotDelta(bool rotationRequested,
                                                       bool torsoRotationActive,
                                                       bool deltaHasCartesian) {
  if (deltaHasCartesian) {
    return TorsoDeltaDisposition::kAsCartesian;
  }
  if (rotationRequested || torsoRotationActive) {
    return TorsoDeltaDisposition::kAsRotationJoint;
  }
  return TorsoDeltaDisposition::kDeferPending;
}

// 进入旋转组时，把暂存的 yaw/pitch oneshot 折入从 initState 重建的关节目标。
inline Eigen::Vector4d handoffPendingTorsoRotationDelta(
    const Eigen::Vector4d& jointTarget, const Eigen::Vector4d& pendingDelta4,
    const Eigen::Vector4d& lower, const Eigen::Vector4d& upper) {
  return applyTorsoRotationJointDelta(jointTarget, pendingDelta4[3], pendingDelta4[2], lower,
                                      upper);
}

// 仅刷新“上一帧已暂存、本帧仍未进入旋转”的 pending 到笛卡尔；同帧刚 stash 的不冲刷。
// 空闲早退与笛卡尔速度路径共用此谓词，避免 sticky 升降跳过空闲分支后仍同帧冲刷新暂存。
inline bool shouldFlushDeferredTorsoRotationDelta(bool hasPending, bool stashedThisFrame,
                                                  bool rotationRequested) {
  return hasPending && !stashedThisFrame && !rotationRequested;
}

// kDeferPending：把 yaw/pitch-only oneshot 所有权转入 pending，并清零 delta4。
// 防止后续笛卡尔路径把 pending 加回未清零的 delta4，导致同帧累计两次。
// DeltaVec 可为 vector_t，PendingVec 可为 Eigen::Vector4d（生产路径即此组合）。
template <typename DeltaVec, typename PendingVec>
inline void transferOneshotToPendingTorsoRotationDelta(DeltaVec& delta4, bool& hasDelta,
                                                       PendingVec& pending, bool& hasPending,
                                                       bool& stashedThisFrame) {
  if (!hasPending) {
    pending.setZero();
  }
  pending[2] += delta4[2];
  pending[3] += delta4[3];
  hasPending = true;
  stashedThisFrame = true;
  hasDelta = false;
  delta4.setZero();
}

}  // namespace ocs2
