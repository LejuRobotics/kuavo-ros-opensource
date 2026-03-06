#include "motion_capture_ik/WheelIncrementalControlModule.h"
#include <ros/ros.h>
#include <cmath>
#include <algorithm>

#include "humanoid_wheel_interface/filters/KinemicLimitFilter.h"

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

// ==================== WheelIncrementalControlModule Implementation ====================

WheelIncrementalControlModule::WheelIncrementalControlModule(const IncrementalControlConfig& config)
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
  initializeRuckigFiltersLocked();
  ROS_INFO("[WheelIncrementalControlModule] Module initialized successfully");
}

WheelIncrementalControlModule::~WheelIncrementalControlModule() = default;

// ==================== 状态机相关方法（原 IncrementalModeStateMachine） ====================

bool WheelIncrementalControlModule::shouldEnterIncrementalModeLeftArm(bool isLeftGrip) const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_) return false;
  if (leftHandStatus_.activated) return false;  // 已经在增量模式，不需要重复进入
  return isLeftGrip;
}

bool WheelIncrementalControlModule::shouldEnterIncrementalModeRightArm(bool isRightGrip) const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_) return false;
  if (rightHandStatus_.activated) return false;  // 已经在增量模式，不需要重复进入
  return isRightGrip;
}

bool WheelIncrementalControlModule::shouldExitIncrementalMode(bool isLeftGrip, bool isRightGrip) const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_) return true;
  if (!leftHandStatus_.activated && !rightHandStatus_.activated) return false;  // 不是增量模式，无需重复执行退出
  return !isLeftGrip && !isRightGrip;
}

bool WheelIncrementalControlModule::isIncrementalMode() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return initialized_ && (leftHandStatus_.activated || rightHandStatus_.activated);
}

bool WheelIncrementalControlModule::isIncrementalModeChest() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return initialized_ && chestActivated_;
}

void WheelIncrementalControlModule::enterIncrementalModeChest(const Eigen::Vector3d& humanChestPos,
                                                         const std::vector<PoseData>& latestPoseConstraintList) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_) {
    ROS_WARN("[WheelIncrementalControlModule] Module not initialized");
    return;
  }
  if (chestActivated_) {
    // chest 已处于增量激活状态（通常由于另一只手已进入增量模式），不重复激活、不重置 anchor
    return;
  }

  // ROS_INFO("[WheelIncrementalControlModule] Entering incremental mode via chest");
  chestActivated_ = true;

  // human anchor
  result_.humanChestPosAnchor_ = humanChestPos;

  // robot anchor (waist_yaw_link target in PoseConstraintList)
  if (latestPoseConstraintList.size() > POSE_DATA_LIST_INDEX_CHEST) {
    result_.robotChestPosAnchor_ = latestPoseConstraintList[POSE_DATA_LIST_INDEX_CHEST].position;
  } else {
    result_.robotChestPosAnchor_.setZero();
  }

  // reset chest delta
  result_.robotChestDeltaPos_.setZero();
  result_.dotChestDeltaPos_.setZero();
  resetChestFilterLocked();
}

void WheelIncrementalControlModule::updateLeftArmPoseAnchor(const ArmPose& vrLeftPose,
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
  resetLeftHandFilterLocked();
  resetLeftSlerpFilterLocked();
}

void WheelIncrementalControlModule::updateRightArmPoseAnchor(const ArmPose& vrRightPose,
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
  resetRightHandFilterLocked();
  resetRightSlerpFilterLocked();
}

void WheelIncrementalControlModule::updateLastOnExit(const std::vector<PoseData>& latestPoseConstraintList) {
  result_.saveLastOnExit(latestPoseConstraintList);
}

void WheelIncrementalControlModule::updateChestAnchorOnExit(const Eigen::Vector3d& humanChestPos,
                                                       const std::vector<PoseData>& latestPoseConstraintList) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_) {
    ROS_WARN("[WheelIncrementalControlModule] Cannot update chest anchor: module not initialized");
    return;
  }
  result_.humanChestPosAnchor_ = humanChestPos;
  if (latestPoseConstraintList.size() > POSE_DATA_LIST_INDEX_CHEST) {
    result_.robotChestPosAnchor_ = latestPoseConstraintList[POSE_DATA_LIST_INDEX_CHEST].position;
  } else {
    result_.robotChestPosAnchor_.setZero();
  }
  result_.robotChestDeltaPos_.setZero();
  result_.dotChestDeltaPos_.setZero();
  resetChestFilterLocked();
}

void WheelIncrementalControlModule::resetDelta() {
  result_.resetDelta();
  resetChestFilterLocked();
  resetLeftHandFilterLocked();
  resetRightHandFilterLocked();
}

void WheelIncrementalControlModule::resetSlerpFactor() {
  result_.resetSlerpFactor();
  resetLeftSlerpFilterLocked();
  resetRightSlerpFilterLocked();
}

void WheelIncrementalControlModule::initializeFilter(
    std::unique_ptr<ocs2::mobile_manipulator::KinemicLimitFilter>& filterPtr,
    int dimension,
    double dt,
    double velLimit,
    double accLimit,
    double jerkLimit,
    const std::string& filterName) {
  filterPtr = std::make_unique<ocs2::mobile_manipulator::KinemicLimitFilter>(dimension, dt);
  const Eigen::VectorXd velLimitVec = Eigen::VectorXd::Constant(dimension, velLimit);
  const Eigen::VectorXd accLimitVec = Eigen::VectorXd::Constant(dimension, accLimit);
  const Eigen::VectorXd jerkLimitVec = Eigen::VectorXd::Constant(dimension, jerkLimit);
  filterPtr->setFirstOrderDerivativeLimit(velLimitVec);
  filterPtr->setSecondOrderDerivativeLimit(accLimitVec);
  filterPtr->setThirdOrderDerivativeLimit(jerkLimitVec);

  // 表格化打印过滤器初始化信息
  // static bool isFirstCall = true;
  // if (isFirstCall) {
  //   ROS_INFO("[WheelIncrementalControlModule] Filter Initialization Summary:");
  //   ROS_INFO("+-------------------------+----------+--------------+--------------+--------------+--------------+");
  //   ROS_INFO("| Filter Name             | Dimension| dt (s)       | Vel Limit    | Acc Limit    | Jerk Limit   |");
  //   ROS_INFO("+-------------------------+----------+--------------+--------------+--------------+--------------+");
  //   isFirstCall = false;
  // }
  // ROS_INFO("| %-23s | %8d | %12.6f | %12.6f | %12.6f | %12.6f |",
  //          filterName.empty() ? "Unknown" : filterName.c_str(),
  //          dimension, dt, velLimit, accLimit, jerkLimit);
  
  // 检查是否是最后一个过滤器（通过检查是否所有过滤器都已初始化）
  // static int filterCount = 0;
  // filterCount++;
  // if (filterCount >= 5) {  // 总共有5个过滤器
  //   ROS_INFO("+-------------------------+----------+--------------+--------------+--------------+--------------+");
  //   filterCount = 0;  // 重置计数器，以便下次初始化时重新打印表头
  //   isFirstCall = true;
  // }
}

void WheelIncrementalControlModule::initializeRuckigFiltersLocked() {
  const double dt = 1.0 / std::max(config_.publishRate, 1.0);
  
  const double positionMaxVelocity = std::max(config_.posVelLimit, 1e-3);
  const double positionMaxAcceleration = std::max(config_.taskSpaceAccLimit, 1e-3);
  const double positionMaxJerk = std::max(config_.taskSpaceJerkLimit, 1e-3);

  const double slerpMaxVelocity = positionMaxVelocity;
  const double slerpMaxAcceleration = std::max(config_.taskSpaceAccLimit / 10.0, 1e-3);
  const double slerpMaxJerk = std::max(config_.taskSpaceJerkLimit, 1e-3);

  initializeFilter(leftHandDeltaFilterPtr_, 3, dt, positionMaxVelocity, positionMaxAcceleration, positionMaxJerk, "LeftHandDelta");
  initializeFilter(rightHandDeltaFilterPtr_, 3, dt, positionMaxVelocity, positionMaxAcceleration, positionMaxJerk, "RightHandDelta");
  initializeFilter(chestDeltaFilterPtr_, 3, dt, positionMaxVelocity, positionMaxAcceleration, positionMaxJerk, "ChestDelta");

  initializeFilter(leftSlerpFilterPtr_, 1, dt, slerpMaxVelocity, slerpMaxAcceleration, slerpMaxJerk, "LeftSlerp");
  initializeFilter(rightSlerpFilterPtr_, 1, dt, slerpMaxVelocity, slerpMaxAcceleration, slerpMaxJerk, "RightSlerp");

  resetChestFilterLocked();
  resetLeftHandFilterLocked();
  resetRightHandFilterLocked();
  resetLeftSlerpFilterLocked();
  resetRightSlerpFilterLocked();
}

void WheelIncrementalControlModule::resetChestFilterLocked() {
  if (!chestDeltaFilterPtr_) return;
  chestDeltaFilterPtr_->reset(result_.robotChestDeltaPos_);
}

void WheelIncrementalControlModule::resetLeftHandFilterLocked() {
  if (!leftHandDeltaFilterPtr_) return;
  leftHandDeltaFilterPtr_->reset(result_.robotLeftHandDeltaPos_);
}

void WheelIncrementalControlModule::resetRightHandFilterLocked() {
  if (!rightHandDeltaFilterPtr_) return;
  rightHandDeltaFilterPtr_->reset(result_.robotRightHandDeltaPos_);
}

void WheelIncrementalControlModule::resetLeftSlerpFilterLocked() {
  if (!leftSlerpFilterPtr_) return;
  leftSlerpFilterPtr_->reset(Eigen::VectorXd::Constant(1, result_.leftSlerpQuatT_));
}

void WheelIncrementalControlModule::resetRightSlerpFilterLocked() {
  if (!rightSlerpFilterPtr_) return;
  rightSlerpFilterPtr_->reset(Eigen::VectorXd::Constant(1, result_.rightSlerpQuatT_));
}

void WheelIncrementalControlModule::computeRuckigFiltering(const ArmPose& vrLeftPose,
                                                      const ArmPose& vrRightPose,
                                                      bool isLeftActive,
                                                      bool isRightActive,
                                                      const double slerpQuatFactor) {
  if (slerpQuatFactor - 1.0 > slerpQuatFactorThreshold_) {
    ROS_WARN("[WheelIncrementalControlModule] slerpQuatFactor is not 1.0, it is %f", slerpQuatFactor);
    return;
  }
  if (isLeftActive && leftSlerpFilterPtr_) {
    Eigen::VectorXd target(1);
    target(0) = slerpQuatFactor;
    const Eigen::VectorXd filtered = leftSlerpFilterPtr_->update(target);
    result_.leftSlerpQuatT_ = clampToUnit(filtered(0));
    result_.leftSlerpQuatDt_ = leftSlerpFilterPtr_->getFirstOrderDerivative()(0);
  } else if (isLeftActive) {
    result_.leftSlerpQuatT_ = clampToUnit(slerpQuatFactor);
    result_.leftSlerpQuatDt_ = 0.0;
  }
  if (isRightActive && rightSlerpFilterPtr_) {
    Eigen::VectorXd target(1);
    target(0) = slerpQuatFactor;
    const Eigen::VectorXd filtered = rightSlerpFilterPtr_->update(target);
    result_.rightSlerpQuatT_ = clampToUnit(filtered(0));
    result_.rightSlerpQuatDt_ = rightSlerpFilterPtr_->getFirstOrderDerivative()(0);
  } else if (isRightActive) {
    result_.rightSlerpQuatT_ = clampToUnit(slerpQuatFactor);
    result_.rightSlerpQuatDt_ = 0.0;
  }

  // ==================== 左臂手部位置增量滤波 ====================
  // 关键修复：只有当左臂激活时才更新左臂的 ruckig 滤波状态
  // 避免未激活时滤波状态向 0 收敛导致的状态污染
  if (isLeftActive && leftHandDeltaFilterPtr_) {
    Eigen::VectorXd rawLeftPosDelta(3);
    rawLeftPosDelta(0) = (vrLeftPose.position[0] - result_.humanLeftHandPosAnchor_[0]) * config_.deltaScale[0];
    rawLeftPosDelta(1) = (vrLeftPose.position[1] - result_.humanLeftHandPosAnchor_[1]) * config_.deltaScale[1];
    rawLeftPosDelta(2) = (vrLeftPose.position[2] - result_.humanLeftHandPosAnchor_[2]) * config_.deltaScale[2];
    const Eigen::VectorXd filtered = leftHandDeltaFilterPtr_->update(rawLeftPosDelta);
    result_.robotLeftHandDeltaPos_ = filtered;
    result_.dotLeftHandDeltaPos_ = leftHandDeltaFilterPtr_->getFirstOrderDerivative();
  } else if (isLeftActive) {
    result_.robotLeftHandDeltaPos_(0) =
        (vrLeftPose.position[0] - result_.humanLeftHandPosAnchor_[0]) * config_.deltaScale[0];
    result_.robotLeftHandDeltaPos_(1) =
        (vrLeftPose.position[1] - result_.humanLeftHandPosAnchor_[1]) * config_.deltaScale[1];
    result_.robotLeftHandDeltaPos_(2) =
        (vrLeftPose.position[2] - result_.humanLeftHandPosAnchor_[2]) * config_.deltaScale[2];
    result_.dotLeftHandDeltaPos_.setZero();
  }
  // 未激活时保持滤波状态不变，不向 0 收敛

  // ==================== 右臂手部位置增量滤波 ====================
  // 关键修复：只有当右臂激活时才更新右臂的 ruckig 滤波状态
  // 避免未激活时滤波状态向 0 收敛导致的状态污染
  if (isRightActive && rightHandDeltaFilterPtr_) {
    Eigen::VectorXd rawRightPosDelta(3);
    rawRightPosDelta(0) = (vrRightPose.position[0] - result_.humanRightHandPosAnchor_[0]) * config_.deltaScale[0];
    rawRightPosDelta(1) = (vrRightPose.position[1] - result_.humanRightHandPosAnchor_[1]) * config_.deltaScale[1];
    rawRightPosDelta(2) = (vrRightPose.position[2] - result_.humanRightHandPosAnchor_[2]) * config_.deltaScale[2];
    const Eigen::VectorXd filtered = rightHandDeltaFilterPtr_->update(rawRightPosDelta);
    result_.robotRightHandDeltaPos_ = filtered;
    result_.dotRightHandDeltaPos_ = rightHandDeltaFilterPtr_->getFirstOrderDerivative();
  } else if (isRightActive) {
    result_.robotRightHandDeltaPos_(0) =
        (vrRightPose.position[0] - result_.humanRightHandPosAnchor_[0]) * config_.deltaScale[0];
    result_.robotRightHandDeltaPos_(1) =
        (vrRightPose.position[1] - result_.humanRightHandPosAnchor_[1]) * config_.deltaScale[1];
    result_.robotRightHandDeltaPos_(2) =
        (vrRightPose.position[2] - result_.humanRightHandPosAnchor_[2]) * config_.deltaScale[2];
    result_.dotRightHandDeltaPos_.setZero();
  }
  // 未激活时保持滤波状态不变，不向 0 收敛
}

void WheelIncrementalControlModule::setHandQuatSeeds(const Eigen::Quaterniond& leftHandQuatSeed,
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

void WheelIncrementalControlModule::enterIncrementalModeLeftArm(const ArmPose& vrLeftPose,
                                                           const std::vector<PoseData>& latestPoseConstraintList,
                                                           const Eigen::Vector3d& pEndEffector,
                                                           const Eigen::Quaterniond& qEndEffector,
                                                           const Eigen::Quaterniond& qLink4) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_) {
    ROS_WARN("[WheelIncrementalControlModule] Module not initialized");
    return;
  }

  // ROS_INFO("[WheelIncrementalControlModule] Entering incremental mode via left arm");

  leftHandStatus_.ready(vrLeftPose.position, vrLeftPose.quaternion);

  // 只更新左臂的锚点
  updateLeftArmPoseAnchor(vrLeftPose, latestPoseConstraintList, pEndEffector, qEndEffector, qLink4);
}

void WheelIncrementalControlModule::enterIncrementalModeRightArm(const ArmPose& vrRightPose,
                                                            const std::vector<PoseData>& latestPoseConstraintList,
                                                            const Eigen::Vector3d& pEndEffector,
                                                            const Eigen::Quaterniond& qEndEffector,
                                                            const Eigen::Quaterniond& qLink4) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_) {
    ROS_WARN("[WheelIncrementalControlModule] Module not initialized");
    return;
  }

  // ROS_INFO("[WheelIncrementalControlModule] Entering incremental mode via right arm");

  rightHandStatus_.ready(vrRightPose.position, vrRightPose.quaternion);

  // 只更新右臂的锚点
  updateRightArmPoseAnchor(vrRightPose, latestPoseConstraintList, pEndEffector, qEndEffector, qLink4);
}

void WheelIncrementalControlModule::exitIncrementalModeLeftArm(const ArmPose& vrLeftPose,
                                                          const std::vector<PoseData>& latestPoseConstraintList,
                                                          const Eigen::Vector3d& pEndEffector,
                                                          const Eigen::Quaterniond& qEndEffector,
                                                          const Eigen::Quaterniond& qLink4) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_) {
    ROS_WARN("[WheelIncrementalControlModule] Cannot exit incremental mode: module not initialized");
    return;
  }

  // 更新左臂锚点
  updateLeftArmPoseAnchor(vrLeftPose, latestPoseConstraintList, pEndEffector, qEndEffector, qLink4);

  // 标记左臂退出增量模式
  leftHandStatus_.unready(vrLeftPose.position, vrLeftPose.quaternion);

  if (!leftHandStatus_.activated && !rightHandStatus_.activated) {
    chestActivated_ = false;
    updateLastOnExit(latestPoseConstraintList);
    resetDelta();
    resetSlerpFactor();
  }
}

void WheelIncrementalControlModule::exitIncrementalModeRightArm(const ArmPose& vrRightPose,
                                                           const std::vector<PoseData>& latestPoseConstraintList,
                                                           const Eigen::Vector3d& pEndEffector,
                                                           const Eigen::Quaterniond& qEndEffector,
                                                           const Eigen::Quaterniond& qLink4) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_) {
    ROS_WARN("[WheelIncrementalControlModule] Cannot exit incremental mode: module not initialized");
    return;
  }

  // 更新右臂锚点
  updateRightArmPoseAnchor(vrRightPose, latestPoseConstraintList, pEndEffector, qEndEffector, qLink4);

  // 标记右臂退出增量模式
  rightHandStatus_.unready(vrRightPose.position, vrRightPose.quaternion);

  if (!leftHandStatus_.activated && !rightHandStatus_.activated) {
    chestActivated_ = false;
    updateLastOnExit(latestPoseConstraintList);
    resetDelta();
    resetSlerpFactor();
  }
}

IncrementalPoseResult WheelIncrementalControlModule::computeIncrementalChestPos(const Eigen::Vector3d& humanChestPos,
                                                                           bool isChestActive) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  IncrementalPoseResult latestIncrementalResult;

  if (!initialized_ || !chestActivated_) {
    std::cout << "\033[91m[WheelIncrementalControlModule] Chest not in incremental mode\033[0m" << std::endl;
    latestIncrementalResult.isValid_ = false;
    return latestIncrementalResult;
  }

  if (!isChestActive) {
    // 不更新滤波状态，保持连续性
    result_.isValid_ = true;
    return result_;
  }

  // 平滑插值（与手部位置增量滤波保持一致）
  Eigen::VectorXd rawChestPosDelta(3);
  rawChestPosDelta(0) = (humanChestPos[0] - result_.humanChestPosAnchor_[0]) * config_.deltaScale[0];
  rawChestPosDelta(1) = (humanChestPos[1] - result_.humanChestPosAnchor_[1]) * config_.deltaScale[1];
  rawChestPosDelta(2) = (humanChestPos[2] - result_.humanChestPosAnchor_[2]) * config_.deltaScale[2];
  if (chestDeltaFilterPtr_) {
    const Eigen::VectorXd filtered = chestDeltaFilterPtr_->update(rawChestPosDelta);
    result_.robotChestDeltaPos_ = filtered;
    result_.dotChestDeltaPos_ = chestDeltaFilterPtr_->getFirstOrderDerivative();
  } else {
    result_.robotChestDeltaPos_ = rawChestPosDelta;
    result_.dotChestDeltaPos_.setZero();
  }

  result_.isValid_ = true;
  return result_;
}

IncrementalPoseResult WheelIncrementalControlModule::computeIncrementalPose(const ArmPose& vrLeftPose,
                                                                       const ArmPose& vrRightPose,
                                                                       bool isLeftActive,
                                                                       bool isRightActive,
                                                                       const Eigen::Quaterniond& qLeftEndEffector,
                                                                       const Eigen::Quaterniond& qRightEndEffector) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  IncrementalPoseResult latestIncrementalResult;
  if (!initialized_ || (!leftHandStatus_.activated && !rightHandStatus_.activated)) {
    // print in red
    // std::cout << "\033[91m[WheelIncrementalControlModule] Not in incremental mode\033[0m" << std::endl;
    latestIncrementalResult.isValid_ = false;
    return latestIncrementalResult;
  }
  // print once in green color
  ROS_INFO_ONCE("[WheelIncrementalControlModule] Computing incremental pose successfully");

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

  computeRuckigFiltering(vrLeftPose, vrRightPose, isLeftActive, isRightActive);
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

IncrementalPoseResult WheelIncrementalControlModule::computeIncrementalPoseLeftArm(
    const ArmPose& vrLeftPose,
    bool isLeftActive,
    const Eigen::Quaterniond& qLeftEndEffector) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  IncrementalPoseResult latestIncrementalResult;
  if (!initialized_ || !leftHandStatus_.activated) {
    std::cout << "\033[91m[WheelIncrementalControlModule] Left arm not in incremental mode\033[0m" << std::endl;
    latestIncrementalResult.isValid_ = false;
    return latestIncrementalResult;
  }

  ROS_INFO_ONCE("[WheelIncrementalControlModule] Computing left arm incremental pose successfully");

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
  computeRuckigFiltering(vrLeftPose, vrRightPose, isLeftActive, false);
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

IncrementalPoseResult WheelIncrementalControlModule::computeIncrementalPoseRightArm(
    const ArmPose& vrRightPose,
    bool isRightActive,
    const Eigen::Quaterniond& qRightEndEffector) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  IncrementalPoseResult latestIncrementalResult;
  if (!initialized_ || !rightHandStatus_.activated) {
    std::cout << "\033[91m[WheelIncrementalControlModule] Right arm not in incremental mode\033[0m" << std::endl;
    latestIncrementalResult.isValid_ = false;
    return latestIncrementalResult;
  }

  ROS_INFO_ONCE("[WheelIncrementalControlModule] Computing right arm incremental pose successfully");

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
  computeRuckigFiltering(vrLeftPose, vrRightPose, false, isRightActive);
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

bool WheelIncrementalControlModule::detectLeftArmMove(const Eigen::Vector3d& currentLeftHandPos) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_) return false;
  return leftHandStatus_.detectMovement(currentLeftHandPos, config_.armMoveThreshold);
}

bool WheelIncrementalControlModule::detectRightArmMove(const Eigen::Vector3d& currentRightHandPos) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_) return false;
  return rightHandStatus_.detectMovement(currentRightHandPos, config_.armMoveThreshold);
}

bool WheelIncrementalControlModule::hasLeftArmMoved() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return initialized_ && leftHandStatus_.moving;
}

bool WheelIncrementalControlModule::hasRightArmMoved() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return initialized_ && rightHandStatus_.moving;
}

bool WheelIncrementalControlModule::shouldExitIncrementalModeLeftArm(bool isLeftArmCtrlModeActive) const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_) return true;
  if (!leftHandStatus_.activated) return false;  // 不在增量模式，无需重复执行退出
  // 检查左手是否应该退出
  return !isLeftArmCtrlModeActive;
}

bool WheelIncrementalControlModule::shouldExitIncrementalModeRightArm(bool isRightArmCtrlModeActive) const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_) return true;
  if (!rightHandStatus_.activated) return false;  // 不在增量模式，无需重复执行退出
  // 检查右手是否应该退出
  return !isRightArmCtrlModeActive;
}

bool WheelIncrementalControlModule::isIncrementalModeLeftArm() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return initialized_ && leftHandStatus_.activated;
}

bool WheelIncrementalControlModule::isIncrementalModeRightArm() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return initialized_ && rightHandStatus_.activated;
}

void WheelIncrementalControlModule::updateConfig(const IncrementalControlConfig& config) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  config_ = config;
  result_.usePythonIncrementalOrientation_ = config_.usePythonIncrementalOrientation;
  result_.pythonOrientationThresholdRad_ = config_.pythonOrientationThresholdRad;
  initializeRuckigFiltersLocked();
}

IncrementalPoseResult WheelIncrementalControlModule::getLatestIncrementalResult() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_) {
    IncrementalPoseResult emptyResult;
    emptyResult.isValid_ = false;
    return emptyResult;
  }
  return result_;
}

Eigen::Vector3d WheelIncrementalControlModule::getRobotLeftHandAnchorPos() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return result_.getRobotLeftHandAnchorPos();
}

Eigen::Vector3d WheelIncrementalControlModule::getRobotChestAnchorPos() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return result_.getRobotChestAnchorPos();
}

Eigen::Vector3d WheelIncrementalControlModule::getLatestRobotChestPos() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return result_.getLatestRobotChestPos();
}

Eigen::Vector3d WheelIncrementalControlModule::getRobotRightHandAnchorPos() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return result_.getRobotRightHandAnchorPos();
}

Eigen::Quaterniond WheelIncrementalControlModule::getRobotLeftHandAnchorQuat() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return result_.getRobotLeftHandAnchorQuat();
}

Eigen::Quaterniond WheelIncrementalControlModule::getRobotRightHandAnchorQuat() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return result_.getRobotRightHandAnchorQuat();
}

const IncrementalControlConfig& WheelIncrementalControlModule::getConfig() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return config_;
}

void WheelIncrementalControlModule::reset() {
  std::lock_guard<std::mutex> lock(stateMutex_);

  // 重置增量计算结果
  result_ = IncrementalPoseResult();

  leftHandStatus_.reset();
  rightHandStatus_.reset();
  chestActivated_ = false;
  initializeRuckigFiltersLocked();
}

}  // namespace HighlyDynamic
