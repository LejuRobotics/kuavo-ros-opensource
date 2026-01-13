#include "motion_capture_ik/IncrementalControlModule.h"
#include <ros/ros.h>
#include <cmath>

namespace HighlyDynamic {

namespace {
inline double clampToUnit(double x) {
  if (x < 0.0) return 0.0;
  if (x > 1.0) return 1.0;
  return x;
}

inline double quaternionAngleRad(const Eigen::Quaterniond& qIn) {
  const Eigen::Quaterniond q = qIn.normalized();
  const double wAbs = std::abs(q.w());
  return 2.0 * std::acos(clampToUnit(wAbs));
}
}  // namespace

// ==================== IncrementalControlModule Implementation ====================

IncrementalControlModule::IncrementalControlModule(const IncrementalControlConfig& config)
    : result_(),
      config_(config),
      initialized_(true),
      leftHandStatus_(),
      rightHandStatus_(),
      defaultLeftHandPos_(0.05, 0.32, -0.05),
      defaultRightHandPos_(0.05, -0.32, -0.05),
      posAnchorZeroThreshold_(1e-3),
      slerpQuatFactorThreshold_(1e-6),
      zyxLimitsEE_(config.zyxLimitsEE),
      zyxLimitsLink4_(config.zyxLimitsLink4) {
  // Propagate python-compatible orientation config into result_ (stored inside IncrementalPoseResult)
  result_.usePythonIncrementalOrientation_ = config_.usePythonIncrementalOrientation;
  result_.pythonOrientationThresholdRad_ = config_.pythonOrientationThresholdRad;
  result_.zyxLimitsFinal_ = config_.zyxLimitsFinal;
  ROS_INFO("[IncrementalControlModule] Module initialized successfully");
}

// ==================== 状态机相关方法（原 IncrementalModeStateMachine） ====================

bool IncrementalControlModule::shouldEnterIncrementalModeLeftArm(bool isLeftGrip) const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_) return false;
  if (leftHandStatus_.activated) return false;  // 已经在增量模式，不需要重复进入
  return isLeftGrip;
}

bool IncrementalControlModule::shouldEnterIncrementalModeRightArm(bool isRightGrip) const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_) return false;
  if (rightHandStatus_.activated) return false;  // 已经在增量模式，不需要重复进入
  return isRightGrip;
}

bool IncrementalControlModule::shouldExitIncrementalMode(bool isLeftGrip, bool isRightGrip) const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_) return true;
  if (!leftHandStatus_.activated && !rightHandStatus_.activated) return false;  // 不是增量模式，无需重复执行退出
  return !isLeftGrip && !isRightGrip;
}

bool IncrementalControlModule::isIncrementalMode() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return initialized_ && (leftHandStatus_.activated || rightHandStatus_.activated);
}

void IncrementalControlModule::updateLeftArmPoseAnchor(const ArmPose& vrLeftPose,
                                                       const std::vector<PoseData>& latestPoseConstraintList,
                                                       const Eigen::Vector3d& pEndEffector,
                                                       const Eigen::Quaterniond& qEndEffector,
                                                       const Eigen::Quaterniond& qLink4) {
  (void)pEndEffector;
  // humanAnchor
  result_.humanLeftHandPosAnchor_ = vrLeftPose.position;
  result_.humanLeftHandQuatAnchor_ = vrLeftPose.quaternion.normalized();
  result_.humanLeftHandQuatMeas_ = vrLeftPose.quaternion.normalized();
  // Python compatible: initialize human anchor used for per-step delta computation
  result_.humanLeftHandQuatAnchorPython_ = result_.humanLeftHandQuatAnchor_;

  // robotAnchor
  if (latestPoseConstraintList.size() > POSE_DATA_LIST_INDEX_LEFT_HAND) {
    // result_.robotLeftHandQuatAnchor = qEndEffector.normalized();
    // result_.robotLeftHandQuatAnchor = vrLeftPose.quaternion.normalized();

    Eigen::Quaterniond qTargetQuatAnchor =
        Eigen::Quaterniond(latestPoseConstraintList[POSE_DATA_LIST_INDEX_LEFT_HAND].rotation_matrix).normalized();
    // 1) 相对 end-effector 的限幅（保持现有逻辑）
    Eigen::Quaterniond qTargetQuatAnchorRelEE = qEndEffector.conjugate() * qTargetQuatAnchor;
    qTargetQuatAnchorRelEE = limitQuaternionAngleEulerZYX(qTargetQuatAnchorRelEE, zyxLimitsEE_);
    Eigen::Quaterniond qTargetQuatAnchorLimited = qEndEffector * qTargetQuatAnchorRelEE;

    // 2) 额外增加：相对 link4 的限幅（zyx = [pi/2, 0.6, 0.6]）
    Eigen::Quaterniond qTargetQuatAnchorRelLink4 = qLink4.conjugate() * qTargetQuatAnchorLimited;
    qTargetQuatAnchorRelLink4 = limitQuaternionAngleEulerZYX(qTargetQuatAnchorRelLink4, zyxLimitsLink4_);
    result_.robotLeftHandQuatAnchor_ = (qLink4 * qTargetQuatAnchorRelLink4).normalized();
    // Python compatible: incremental target seed should be the current target (pose constraint), not qEndEffector.
    // This matches quest3_node_incremental.py where the target quaternion is updated incrementally each frame.
    result_.robotLeftHandQuatTarget_ = qTargetQuatAnchor;

    result_.robotLeftHandPosAnchor_ = latestPoseConstraintList[POSE_DATA_LIST_INDEX_LEFT_HAND].position;
    if (result_.robotLeftHandPosAnchor_.norm() < posAnchorZeroThreshold_) {
      result_.robotLeftHandPosAnchor_ = defaultLeftHandPos_;
    }
  }
  result_.robotLeftHandQuatMeasEE_ = qEndEffector.normalized();
  result_.robotLeftHandQuatMeasEERealTime_ = qEndEffector.normalized();  // 初始化实时更新的ee fk值
  result_.robotLeftHandQuatMeasLink4_ = qLink4.normalized();
  result_.resetLeftHandDelta();
}

void IncrementalControlModule::updateRightArmPoseAnchor(const ArmPose& vrRightPose,
                                                        const std::vector<PoseData>& latestPoseConstraintList,
                                                        const Eigen::Vector3d& pEndEffector,
                                                        const Eigen::Quaterniond& qEndEffector,
                                                        const Eigen::Quaterniond& qLink4) {
  (void)pEndEffector;
  // humanAnchor
  result_.humanRightHandPosAnchor_ = vrRightPose.position;
  result_.humanRightHandQuatAnchor_ = vrRightPose.quaternion.normalized();
  result_.humanRightHandQuatMeas_ = vrRightPose.quaternion.normalized();
  // Python compatible: initialize human anchor used for per-step delta computation
  result_.humanRightHandQuatAnchorPython_ = result_.humanRightHandQuatAnchor_;

  // robotAnchor
  if (latestPoseConstraintList.size() > POSE_DATA_LIST_INDEX_RIGHT_HAND) {
    // result_.robotRightHandQuatAnchor = qEndEffector.normalized();
    // result_.robotRightHandQuatAnchor = vrRightPose.quaternion.normalized();

    Eigen::Quaterniond qTargetQuatAnchor =
        Eigen::Quaterniond(latestPoseConstraintList[POSE_DATA_LIST_INDEX_RIGHT_HAND].rotation_matrix).normalized();
    // 1) 相对 end-effector 的限幅（与左手对称一致）
    Eigen::Quaterniond qTargetQuatAnchorRelEE = qEndEffector.conjugate() * qTargetQuatAnchor;
    qTargetQuatAnchorRelEE = limitQuaternionAngleEulerZYX(qTargetQuatAnchorRelEE, zyxLimitsEE_);
    Eigen::Quaterniond qTargetQuatAnchorLimited = qEndEffector * qTargetQuatAnchorRelEE;

    // 2) 额外增加：相对 link4 的限幅（zyx = [pi/2, 0.6, 0.6]），与左手对称一致
    Eigen::Quaterniond qTargetQuatAnchorRelLink4 = qLink4.conjugate() * qTargetQuatAnchorLimited;
    qTargetQuatAnchorRelLink4 = limitQuaternionAngleEulerZYX(qTargetQuatAnchorRelLink4, zyxLimitsLink4_);
    result_.robotRightHandQuatAnchor_ = (qLink4 * qTargetQuatAnchorRelLink4).normalized();
    // Python compatible: incremental target seed should be the current target (pose constraint), not qEndEffector.
    result_.robotRightHandQuatTarget_ = qTargetQuatAnchor;

    result_.robotRightHandPosAnchor_ = latestPoseConstraintList[POSE_DATA_LIST_INDEX_RIGHT_HAND].position;
    if (result_.robotRightHandPosAnchor_.norm() < posAnchorZeroThreshold_) {
      result_.robotRightHandPosAnchor_ = defaultRightHandPos_;
    }
  }

  result_.robotRightHandQuatMeasEE_ = qEndEffector.normalized();
  result_.robotRightHandQuatMeasEERealTime_ = qEndEffector.normalized();  // 初始化实时更新的ee fk值
  result_.robotRightHandQuatMeasLink4_ = qLink4.normalized();
  result_.resetRightHandDelta();
}

void IncrementalControlModule::updateLastOnExit(const std::vector<PoseData>& latestPoseConstraintList) {
  result_.saveLastOnExit(latestPoseConstraintList);
}

void IncrementalControlModule::resetDelta() { result_.resetDelta(); }

void IncrementalControlModule::resetSlerpFactor() { result_.resetSlerpFactor(); }

void IncrementalControlModule::computeFhanFiltering(const ArmPose& vrLeftPose,
                                                    const ArmPose& vrRightPose,
                                                    bool isLeftActive,
                                                    bool isRightActive,
                                                    const double slerpQuatFactor) {
  if (slerpQuatFactor - 1.0 > slerpQuatFactorThreshold_) {
    ROS_WARN("[IncrementalControlModule] slerpQuatFactor is not 1.0, it is %f", slerpQuatFactor);
    return;
  }
  // 平滑插值
  double fhanH = 1.0 / config_.publishRate;
  double fhanH0 = fhanH * config_.fhanKh0;

  double slerptR = config_.fhanR / 10;
  if (isLeftActive) {
    leju_utils::fhanStepForward(result_.leftSlerpQuatT_,
                                result_.leftSlerpQuatDt_,
                                slerpQuatFactor,
                                slerptR,  //加速度约束
                                fhanH,    // dt
                                fhanH0);  //平滑系数，通常为dt的2~10倍
  }
  if (isRightActive) {
    leju_utils::fhanStepForward(result_.rightSlerpQuatT_,
                                result_.rightSlerpQuatDt_,
                                slerpQuatFactor,
                                slerptR,  //加速度约束
                                fhanH,    // dt
                                fhanH0);  //平滑系数，通常为dt的2~10倍
  }

  for (int i = 0; i < 3; i++) {
    // ==================== 左臂手部位置增量滤波 ====================
    // 关键修复：只有当左臂激活时才更新左臂的 fhan 滤波状态
    // 避免未激活时滤波状态向 0 收敛导致的状态污染
    if (isLeftActive) {
      double rawLeftPosDelta = (vrLeftPose.position[i] - result_.humanLeftHandPosAnchor_[i]) * config_.deltaScale[i];
      // print deltascale[i]
      // std::cout << "[IncrementalControlModule] deltaScale[" << i << "] = " << config_.deltaScale[i] << std::endl;
      leju_utils::fhanStepForward(result_.robotLeftHandDeltaPos_[i],  //约束处理后的位置增量
                                  result_.dotLeftHandDeltaPos_[i],    //约束处理后的速度
                                  rawLeftPosDelta,                    //原始位置增量
                                  config_.fhanR,                      //加速度约束
                                  fhanH,                              // dt
                                  fhanH0);                            //平滑系数，通常为dt的2~10倍
    }
    // 未激活时保持滤波状态不变，不向 0 收敛

    // ==================== 右臂手部位置增量滤波 ====================
    // 关键修复：只有当右臂激活时才更新右臂的 fhan 滤波状态
    // 避免未激活时滤波状态向 0 收敛导致的状态污染
    if (isRightActive) {
      double rawRightPosDelta = (vrRightPose.position[i] - result_.humanRightHandPosAnchor_[i]) * config_.deltaScale[i];
      // print deltascale[i]
      // std::cout << "[IncrementalControlModule] deltaScale[" << i << "] = " << config_.deltaScale[i] << std::endl;
      leju_utils::fhanStepForward(result_.robotRightHandDeltaPos_[i],  //约束处理后的位置增量
                                  result_.dotRightHandDeltaPos_[i],    //约束处理后的速度
                                  rawRightPosDelta,                    //原始位置增量
                                  config_.fhanR,                       //加速度约束
                                  fhanH,                               // dt
                                  fhanH0);                             //平滑系数，通常为dt的2~10倍
    }
    // 未激活时保持滤波状态不变，不向 0 收敛
  }
}

void IncrementalControlModule::setHandQuatSeeds(const Eigen::Quaterniond& leftHandQuatSeed,
                                                const Eigen::Quaterniond& rightHandQuatSeed,
                                                bool isIncremetalOrientation) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!isIncremetalOrientation) {
    result_.robotLeftHandQuatAnchor_ = leftHandQuatSeed.normalized();
    result_.robotRightHandQuatAnchor_ = rightHandQuatSeed.normalized();
  }

  result_.robotLeftHandQuatSlerpDes_ = result_.robotLeftHandQuatAnchor_;
  result_.robotRightHandQuatSlerpDes_ = result_.robotRightHandQuatAnchor_;
  result_.robotLeftHandQuatTarget_ = result_.robotLeftHandQuatAnchor_;
  result_.robotRightHandQuatTarget_ = result_.robotRightHandQuatAnchor_;
}

void IncrementalControlModule::enterIncrementalModeLeftArm(const ArmPose& vrLeftPose,
                                                           const std::vector<PoseData>& latestPoseConstraintList,
                                                           const Eigen::Vector3d& pEndEffector,
                                                           const Eigen::Quaterniond& qEndEffector,
                                                           const Eigen::Quaterniond& qLink4) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_) {
    ROS_WARN("[IncrementalControlModule] Module not initialized");
    return;
  }

  ROS_INFO("[IncrementalControlModule] Entering incremental mode via left arm");

  leftHandStatus_.ready(vrLeftPose.position, vrLeftPose.quaternion);

  // 只更新左臂的锚点
  updateLeftArmPoseAnchor(vrLeftPose, latestPoseConstraintList, pEndEffector, qEndEffector, qLink4);
}

void IncrementalControlModule::enterIncrementalModeRightArm(const ArmPose& vrRightPose,
                                                            const std::vector<PoseData>& latestPoseConstraintList,
                                                            const Eigen::Vector3d& pEndEffector,
                                                            const Eigen::Quaterniond& qEndEffector,
                                                            const Eigen::Quaterniond& qLink4) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_) {
    ROS_WARN("[IncrementalControlModule] Module not initialized");
    return;
  }

  ROS_INFO("[IncrementalControlModule] Entering incremental mode via right arm");

  rightHandStatus_.ready(vrRightPose.position, vrRightPose.quaternion);

  // 只更新右臂的锚点
  updateRightArmPoseAnchor(vrRightPose, latestPoseConstraintList, pEndEffector, qEndEffector, qLink4);
}

void IncrementalControlModule::exitIncrementalModeLeftArm(const ArmPose& vrLeftPose,
                                                          const std::vector<PoseData>& latestPoseConstraintList,
                                                          const Eigen::Vector3d& pEndEffector,
                                                          const Eigen::Quaterniond& qEndEffector,
                                                          const Eigen::Quaterniond& qLink4) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_) {
    ROS_WARN("[IncrementalControlModule] Cannot exit incremental mode: module not initialized");
    return;
  }

  // 更新左臂锚点
  updateLeftArmPoseAnchor(vrLeftPose, latestPoseConstraintList, pEndEffector, qEndEffector, qLink4);

  // 标记左臂退出增量模式
  leftHandStatus_.unready(vrLeftPose.position, vrLeftPose.quaternion);

  if (!leftHandStatus_.activated && !rightHandStatus_.activated) {
    updateLastOnExit(latestPoseConstraintList);
    resetDelta();
    resetSlerpFactor();
  }
}

void IncrementalControlModule::exitIncrementalModeRightArm(const ArmPose& vrRightPose,
                                                           const std::vector<PoseData>& latestPoseConstraintList,
                                                           const Eigen::Vector3d& pEndEffector,
                                                           const Eigen::Quaterniond& qEndEffector,
                                                           const Eigen::Quaterniond& qLink4) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_) {
    ROS_WARN("[IncrementalControlModule] Cannot exit incremental mode: module not initialized");
    return;
  }

  // 更新右臂锚点
  updateRightArmPoseAnchor(vrRightPose, latestPoseConstraintList, pEndEffector, qEndEffector, qLink4);

  // 标记右臂退出增量模式
  rightHandStatus_.unready(vrRightPose.position, vrRightPose.quaternion);

  if (!leftHandStatus_.activated && !rightHandStatus_.activated) {
    updateLastOnExit(latestPoseConstraintList);
    resetDelta();
    resetSlerpFactor();
  }
}

IncrementalPoseResult IncrementalControlModule::computeIncrementalPose(const ArmPose& vrLeftPose,
                                                                       const ArmPose& vrRightPose,
                                                                       bool isLeftActive,
                                                                       bool isRightActive,
                                                                       const Eigen::Quaterniond& qLeftEndEffector,
                                                                       const Eigen::Quaterniond& qRightEndEffector) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  IncrementalPoseResult latestIncrementalResult;
  if (!initialized_ || (!leftHandStatus_.activated && !rightHandStatus_.activated)) {
    // print in red
    std::cout << "\033[91m[IncrementalControlModule] Not in incremental mode\033[0m" << std::endl;
    latestIncrementalResult.isValid_ = false;
    return latestIncrementalResult;
  }
  // print once in green color
  ROS_INFO_ONCE("[IncrementalControlModule] Computing incremental pose successfully");

  // 实时更新ee的fk值
  if (isLeftActive && qLeftEndEffector.norm() > 1e-6) {
    result_.robotLeftHandQuatMeasEERealTime_ = qLeftEndEffector.normalized();
  }
  if (isRightActive && qRightEndEffector.norm() > 1e-6) {
    result_.robotRightHandQuatMeasEERealTime_ = qRightEndEffector.normalized();
  }

  // 更新当前姿态
  if (isLeftActive) {
    result_.humanLeftHandQuatMeas_ = vrLeftPose.quaternion.normalized();
  }
  if (isRightActive) {
    result_.humanRightHandQuatMeas_ = vrRightPose.quaternion.normalized();
  }

  computeFhanFiltering(vrLeftPose, vrRightPose, isLeftActive, isRightActive);
  result_.slerpQuat(vrLeftPose.quaternion, vrRightPose.quaternion, isLeftActive, isRightActive);

  // ==================== Python compatible incremental orientation ====================
  // quest3_node_incremental.py:
  //   q_delta = q_current * q_anchor^{-1}
  //   if angle(q_delta) < threshold => identity
  //   if significant => anchor = current
  //   q_target = q_delta * q_target
  if (result_.usePythonIncrementalOrientation_) {
    if (isLeftActive) {
      const Eigen::Quaterniond qCur = result_.humanLeftHandQuatMeas_.normalized();
      const Eigen::Quaterniond qAnchor = result_.humanLeftHandQuatAnchorPython_.normalized();
      Eigen::Quaterniond qDelta = (qCur * qAnchor.conjugate()).normalized();
      const double angle = quaternionAngleRad(qDelta);
      if (angle < result_.pythonOrientationThresholdRad_) {
        qDelta.setIdentity();
      } else {
        result_.humanLeftHandQuatAnchorPython_ = qCur;
        // qDelta = Eigen::Quaterniond::Identity().slerp(1.35, qDelta).normalized();
        result_.robotLeftHandQuatTarget_ = (qDelta * result_.robotLeftHandQuatTarget_).normalized();
      }
      result_.leftHandDeltaQuatLast_ = qDelta;
    }

    if (isRightActive) {
      const Eigen::Quaterniond qCur = result_.humanRightHandQuatMeas_.normalized();
      const Eigen::Quaterniond qAnchor = result_.humanRightHandQuatAnchorPython_.normalized();
      Eigen::Quaterniond qDelta = (qCur * qAnchor.conjugate()).normalized();
      const double angle = quaternionAngleRad(qDelta);
      if (angle < result_.pythonOrientationThresholdRad_) {
        qDelta.setIdentity();
      } else {
        result_.humanRightHandQuatAnchorPython_ = qCur;
        // qDelta = Eigen::Quaterniond::Identity().slerp(1.35, qDelta).normalized();
        result_.robotRightHandQuatTarget_ = (qDelta * result_.robotRightHandQuatTarget_).normalized();
      }
      result_.rightHandDeltaQuatLast_ = qDelta;
    }
  }

  result_.isValid_ = true;

  return result_;
}

IncrementalPoseResult IncrementalControlModule::computeIncrementalPoseLeftArm(
    const ArmPose& vrLeftPose,
    bool isLeftActive,
    const Eigen::Quaterniond& qLeftEndEffector) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  IncrementalPoseResult latestIncrementalResult;
  if (!initialized_ || !leftHandStatus_.activated) {
    std::cout << "\033[91m[IncrementalControlModule] Left arm not in incremental mode\033[0m" << std::endl;
    latestIncrementalResult.isValid_ = false;
    return latestIncrementalResult;
  }

  ROS_INFO_ONCE("[IncrementalControlModule] Computing left arm incremental pose successfully");

  // 实时更新ee的fk值
  if (isLeftActive && qLeftEndEffector.norm() > 1e-6) {
    result_.robotLeftHandQuatMeasEERealTime_ = qLeftEndEffector.normalized();
  }

  // 更新左手当前姿态
  if (isLeftActive) {
    result_.humanLeftHandQuatMeas_ = vrLeftPose.quaternion.normalized();
  }

  // 创建虚拟右臂姿态（使用当前result_中的右臂数据）
  ArmPose vrRightPose;
  vrRightPose.position = result_.humanRightHandPosAnchor_;
  vrRightPose.quaternion = result_.humanRightHandQuatMeas_;

  // 计算时只激活左臂
  computeFhanFiltering(vrLeftPose, vrRightPose, isLeftActive, false);
  result_.slerpQuat(vrLeftPose.quaternion, vrRightPose.quaternion, isLeftActive, false);

  if (result_.usePythonIncrementalOrientation_ && isLeftActive) {
    const Eigen::Quaterniond qCur = result_.humanLeftHandQuatMeas_.normalized();
    const Eigen::Quaterniond qAnchor = result_.humanLeftHandQuatAnchorPython_.normalized();
    Eigen::Quaterniond qDelta = (qCur * qAnchor.conjugate()).normalized();
    const double angle = quaternionAngleRad(qDelta);
    if (angle < result_.pythonOrientationThresholdRad_) {
      qDelta.setIdentity();
    } else {
      result_.humanLeftHandQuatAnchorPython_ = qCur;
      // qDelta = Eigen::Quaterniond::Identity().slerp(1.35, qDelta).normalized();
      result_.robotLeftHandQuatTarget_ = (qDelta * result_.robotLeftHandQuatTarget_).normalized();
    }
    result_.leftHandDeltaQuatLast_ = qDelta;
  }

  result_.isValid_ = true;

  return result_;
}

IncrementalPoseResult IncrementalControlModule::computeIncrementalPoseRightArm(
    const ArmPose& vrRightPose,
    bool isRightActive,
    const Eigen::Quaterniond& qRightEndEffector) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  IncrementalPoseResult latestIncrementalResult;
  if (!initialized_ || !rightHandStatus_.activated) {
    std::cout << "\033[91m[IncrementalControlModule] Right arm not in incremental mode\033[0m" << std::endl;
    latestIncrementalResult.isValid_ = false;
    return latestIncrementalResult;
  }

  ROS_INFO_ONCE("[IncrementalControlModule] Computing right arm incremental pose successfully");

  // 实时更新ee的fk值
  if (isRightActive && qRightEndEffector.norm() > 1e-6) {
    result_.robotRightHandQuatMeasEERealTime_ = qRightEndEffector.normalized();
  }

  // 更新右手当前姿态
  if (isRightActive) {
    result_.humanRightHandQuatMeas_ = vrRightPose.quaternion.normalized();
  }

  // 创建虚拟左臂姿态（使用当前result_中的左臂数据）
  ArmPose vrLeftPose;
  vrLeftPose.position = result_.humanLeftHandPosAnchor_;
  vrLeftPose.quaternion = result_.humanLeftHandQuatMeas_;

  // 计算时只激活右臂
  computeFhanFiltering(vrLeftPose, vrRightPose, false, isRightActive);
  result_.slerpQuat(vrLeftPose.quaternion, vrRightPose.quaternion, false, isRightActive);

  if (result_.usePythonIncrementalOrientation_ && isRightActive) {
    const Eigen::Quaterniond qCur = result_.humanRightHandQuatMeas_.normalized();
    const Eigen::Quaterniond qAnchor = result_.humanRightHandQuatAnchorPython_.normalized();
    Eigen::Quaterniond qDelta = (qCur * qAnchor.conjugate()).normalized();
    const double angle = quaternionAngleRad(qDelta);
    if (angle < result_.pythonOrientationThresholdRad_) {
      qDelta.setIdentity();
    } else {
      result_.humanRightHandQuatAnchorPython_ = qCur;
      // qDelta = Eigen::Quaterniond::Identity().slerp(1.35, qDelta).normalized();
      result_.robotRightHandQuatTarget_ = (qDelta * result_.robotRightHandQuatTarget_).normalized();
    }
    result_.rightHandDeltaQuatLast_ = qDelta;
  }

  result_.isValid_ = true;

  return result_;
}

bool IncrementalControlModule::detectLeftArmMove(const Eigen::Vector3d& currentLeftHandPos) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_) return false;
  return leftHandStatus_.detectMovement(currentLeftHandPos, config_.armMoveThreshold);
}

bool IncrementalControlModule::detectRightArmMove(const Eigen::Vector3d& currentRightHandPos) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_) return false;
  return rightHandStatus_.detectMovement(currentRightHandPos, config_.armMoveThreshold);
}

bool IncrementalControlModule::hasLeftArmMoved() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return initialized_ && leftHandStatus_.moving;
}

bool IncrementalControlModule::hasRightArmMoved() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return initialized_ && rightHandStatus_.moving;
}

bool IncrementalControlModule::shouldExitIncrementalModeLeftArm(bool isLeftArmCtrlModeActive) const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_) return true;
  if (!leftHandStatus_.activated) return false;  // 不在增量模式，无需重复执行退出
  // 检查左手是否应该退出
  return !isLeftArmCtrlModeActive;
}

bool IncrementalControlModule::shouldExitIncrementalModeRightArm(bool isRightArmCtrlModeActive) const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_) return true;
  if (!rightHandStatus_.activated) return false;  // 不在增量模式，无需重复执行退出
  // 检查右手是否应该退出
  return !isRightArmCtrlModeActive;
}

bool IncrementalControlModule::isIncrementalModeLeftArm() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return initialized_ && leftHandStatus_.activated;
}

bool IncrementalControlModule::isIncrementalModeRightArm() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return initialized_ && rightHandStatus_.activated;
}

void IncrementalControlModule::updateConfig(const IncrementalControlConfig& config) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  config_ = config;
  result_.usePythonIncrementalOrientation_ = config_.usePythonIncrementalOrientation;
  result_.pythonOrientationThresholdRad_ = config_.pythonOrientationThresholdRad;
}

IncrementalPoseResult IncrementalControlModule::getLatestIncrementalResult() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_) {
    IncrementalPoseResult emptyResult;
    emptyResult.isValid_ = false;
    return emptyResult;
  }
  return result_;
}

Eigen::Vector3d IncrementalControlModule::getRobotLeftHandAnchorPos() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return result_.getRobotLeftHandAnchorPos();
}

Eigen::Vector3d IncrementalControlModule::getRobotRightHandAnchorPos() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return result_.getRobotRightHandAnchorPos();
}

Eigen::Quaterniond IncrementalControlModule::getRobotLeftHandAnchorQuat() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return result_.getRobotLeftHandAnchorQuat();
}

Eigen::Quaterniond IncrementalControlModule::getRobotRightHandAnchorQuat() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return result_.getRobotRightHandAnchorQuat();
}

const IncrementalControlConfig& IncrementalControlModule::getConfig() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return config_;
}

void IncrementalControlModule::reset() {
  std::lock_guard<std::mutex> lock(stateMutex_);

  // 重置增量计算结果
  result_ = IncrementalPoseResult();

  leftHandStatus_.reset();
  rightHandStatus_.reset();
}

}  // namespace HighlyDynamic
