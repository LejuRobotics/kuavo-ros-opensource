#include "motion_capture_ik/IncrementalControlModule.h"
#include "motion_capture_ik/JoyStickHandler.h"
#include <ros/ros.h>

namespace HighlyDynamic {

// ==================== IncrementalModeStateMachine Implementation ====================

IncrementalModeStateMachine::IncrementalModeStateMachine(std::shared_ptr<JoyStickHandler> joyStickHandler)
    : joyStickHandler_(joyStickHandler) {}

bool IncrementalModeStateMachine::shouldEnterIncrementalMode() const {
  if (!joyStickHandler_) return false;
  if (isIncrementalMode()) return false;
  return joyStickHandler_->isLeftRightFirstButtonTouched() && !joyStickHandler_->isLeftRightFirstButtonPressed();
}

bool IncrementalModeStateMachine::shouldExitIncrementalMode() const {
  if (!joyStickHandler_) return true;      // 手柄未初始化，则应该退出，避免意外失控
  if (!isIncrementalMode()) return false;  // 不是增量模式，无需重复执行退出
  return !joyStickHandler_->isLeftRightFirstButtonTouched();  // 只要松开触控，就退出
}

bool IncrementalModeStateMachine::isIncrementalMode() const { return controlMode_ == ControlMode::INCREMENTAL; }

void IncrementalModeStateMachine::enterIncrementalMode() {
  controlMode_ = ControlMode::INCREMENTAL;
  std::cout << "\033[92m[IncrementalModeStateMachine] Entered incremental mode\033[0m" << std::endl;
}

void IncrementalModeStateMachine::exitIncrementalMode() {
  controlMode_ = ControlMode::NONE;
  std::cout << "\033[93m[IncrementalModeStateMachine] Exited incremental mode\033[0m" << std::endl;
}

// ==================== IncrementalPoseCalculator Implementation ====================

IncrementalPoseCalculator::IncrementalPoseCalculator(const IncrementalControlConfig& config) : config_(config) {}

void IncrementalPoseCalculator::updateHumanPoseAnchor(const ArmPose& vrLeftPose,
                                                      const ArmPose& vrRightPose,
                                                      const ArmPose& vrLeftElbowPose,
                                                      const ArmPose& vrRightElbowPose,
                                                      const std::vector<PoseData>& latestPoseConstraintList) {
  // 设置锚点位置
  result_.leftAnchorPos = vrLeftPose.position;
  result_.rightAnchorPos = vrRightPose.position;
  result_.leftElbowAnchorPos = vrLeftElbowPose.position;
  result_.rightElbowAnchorPos = vrRightElbowPose.position;

  // 初始化手肘滤波状态：从退出时的目标位置开始，平滑过渡到当前测量位置
  // 这样可以避免重新进入增量模式时的跳变
  if (latestPoseConstraintList.size() > POSE_DATA_LIST_INDEX_LEFT_ELBOW) {
    result_.leftElbowPosFiltered = latestPoseConstraintList[POSE_DATA_LIST_INDEX_LEFT_ELBOW].position;
  } else {
    result_.leftElbowPosFiltered = vrLeftElbowPose.position;
  }
  if (latestPoseConstraintList.size() > POSE_DATA_LIST_INDEX_RIGHT_ELBOW) {
    result_.rightElbowPosFiltered = latestPoseConstraintList[POSE_DATA_LIST_INDEX_RIGHT_ELBOW].position;
  } else {
    result_.rightElbowPosFiltered = vrRightElbowPose.position;
  }
  result_.dotLeftElbowPosFiltered.setZero();
  result_.dotRightElbowPosFiltered.setZero();

  // 左手默认值：[1, 1, 1]，右手默认值：[1, -1, 1]，避免初始位置过低触发 clip 下界
  result_.lastTargetLeftHandPosOnExit = latestPoseConstraintList[POSE_DATA_LIST_INDEX_LEFT_HAND].position;
  if (result_.lastTargetLeftHandPosOnExit.norm() < 1e-3) {
    result_.lastTargetLeftHandPosOnExit = Eigen::Vector3d(0.05, 0.32, -0.05);
    ROS_INFO("[IncrementalPoseCalculator] Using hardcoded default for left hand initial target: [1.0, 1.0, 1.0]");
  }

  result_.lastTargetRightHandPosOnExit = latestPoseConstraintList[POSE_DATA_LIST_INDEX_RIGHT_HAND].position;
  if (result_.lastTargetRightHandPosOnExit.norm() < 1e-3) {
    result_.lastTargetRightHandPosOnExit = Eigen::Vector3d(0.05, -0.32, -0.05);
    ROS_INFO("[IncrementalPoseCalculator] Using hardcoded default for right hand initial target: [1.0, -1.0, 1.0]");
  }

  ROS_INFO("[IncrementalPoseCalculator] Anchor positions set successfully");
}

void IncrementalPoseCalculator::updateLastTargetOnExit(const std::vector<PoseData>& latestPoseConstraintList) {
  result_.saveLastTargetOnExit(latestPoseConstraintList);
}

void IncrementalPoseCalculator::resetDelta() { result_.resetDelta(); }

void IncrementalPoseCalculator::resetSlerpFactor() { result_.resetSlerpFactor(); }

void IncrementalPoseCalculator::updateHandFkPositions(const Eigen::Vector3d& leftHandFkPos,
                                                      const Eigen::Vector3d& rightHandFkPos) {
  result_.leftHandFkPosition = leftHandFkPos;
  result_.rightHandFkPosition = rightHandFkPos;
}

IncrementalPoseResult IncrementalPoseCalculator::computeIncrementalPose(const ArmPose& vrLeftPose,
                                                                        const ArmPose& vrRightPose,
                                                                        const ArmPose& vrLeftElbowPose,
                                                                        const ArmPose& vrRightElbowPose) {
  computeFhanFiltering(vrLeftPose, vrRightPose, vrLeftElbowPose, vrRightElbowPose);
  result_.slerpQuat(vrLeftPose.quaternion, vrRightPose.quaternion);
  result_.isValid = true;
  return result_;
}

void IncrementalPoseCalculator::computeFhanFiltering(const ArmPose& vrLeftPose,
                                                     const ArmPose& vrRightPose,
                                                     const ArmPose& vrLeftElbowPose,
                                                     const ArmPose& vrRightElbowPose,
                                                     const double slerpQuatFactor) {
  if (slerpQuatFactor - 1.0 > 1e-6) {
    ROS_WARN("[IncrementalPoseCalculator] slerpQuatFactor is not 1.0, it is %f", slerpQuatFactor);
    return;
  }
  // 平滑插值
  double fhan_h = 1.0 / config_.publishRate;
  double fhan_h0 = fhan_h * config_.fhan_kh0;

  double slerpt_r = config_.fhan_r / 10;
  leju_utils::fhanStepForward(result_.slerpQuat_t_,
                              result_.slerpQuat_dt_,
                              slerpQuatFactor,
                              slerpt_r,  //加速度约束
                              fhan_h,    // dt
                              fhan_h0);  //平滑系数，通常为dt的2~10倍

  for (int i = 0; i < 3; i++) {
    // 使用deltaScale按轴缩放VR手部位置增量，用于调整操控敏感度
    const double rawLeftPosDelta = (vrLeftPose.position[i] - result_.leftAnchorPos[i]) * config_.deltaScale[i];
    const double rawRightPosDelta = (vrRightPose.position[i] - result_.rightAnchorPos[i]) * config_.deltaScale[i];

    leju_utils::fhanStepForward(result_.leftHandPosDelta[i],     //约束处理后的位置增量
                                result_.dotLeftHandPosDelta[i],  //约束处理后的速度
                                rawLeftPosDelta,                 //原始位置增量
                                config_.fhan_r,                  //加速度约束
                                fhan_h,                          // dt
                                fhan_h0);                        //平滑系数，通常为dt的2~10倍

    leju_utils::fhanStepForward(result_.rightHandPosDelta[i],  //右手
                                result_.dotRightHandPosDelta[i],
                                rawRightPosDelta,
                                config_.fhan_r,
                                fhan_h,
                                fhan_h0);

    // ==================== 肘部绝对位置滤波 ====================
    // 使用fhan算法对肘部绝对位置进行平滑滤波，避免重新进入增量模式时的跳变
    // 参考手部绝对姿态的更新逻辑，但使用绝对位置而非增量
    const double rawLeftElbowPos = vrLeftElbowPose.position[i];    // 原始左肘绝对位置
    const double rawRightElbowPos = vrRightElbowPose.position[i];  // 原始右肘绝对位置

    leju_utils::fhanStepForward(result_.leftElbowPosFiltered[i],     // 滤波后的左肘绝对位置
                                result_.dotLeftElbowPosFiltered[i],  // 滤波后的左肘速度
                                rawLeftElbowPos,                     // 原始左肘绝对位置
                                config_.fhan_r / 10,                 // CZJTEST加速度约束
                                fhan_h,                              //
                                fhan_h0);                            // 平滑系数，通常为dt的2~10倍

    leju_utils::fhanStepForward(result_.rightElbowPosFiltered[i],  // 滤波后的右肘绝对位置
                                result_.dotRightElbowPosFiltered[i],
                                rawRightElbowPos,     // 原始右肘绝对位置
                                config_.fhan_r / 10,  // CZJTEST:
                                fhan_h,
                                fhan_h0);
  }
}

void IncrementalPoseCalculator::updateConfig(const IncrementalControlConfig& config) { config_ = config; }

// ==================== IncrementalControlModule Implementation ====================

IncrementalControlModule::IncrementalControlModule(std::shared_ptr<JoyStickHandler> joyStickHandler,
                                                   const IncrementalControlConfig& config)
    : config_(config), armMoveDetector_(false) {
  stateMachine_ = std::make_unique<IncrementalModeStateMachine>(joyStickHandler);
  poseCalculator_ = std::make_unique<IncrementalPoseCalculator>(config);
  initialized_ = true;

  ROS_INFO("[IncrementalControlModule] Module initialized successfully");
}

void IncrementalControlModule::enterIncrementalMode(const ArmPose& vrLeftPose,
                                                    const ArmPose& vrRightPose,
                                                    const ArmPose& vrLeftElbowPose,
                                                    const ArmPose& vrRightElbowPose,
                                                    const std::vector<PoseData>& latestPoseConstraintList) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_) {
    ROS_WARN("[IncrementalControlModule] Module not initialized");
    return;
  }

  stateMachine_->enterIncrementalMode();
  poseCalculator_->updateHumanPoseAnchor(
      vrLeftPose, vrRightPose, vrLeftElbowPose, vrRightElbowPose, latestPoseConstraintList);
  std::cout << "\033[92m[IncrementalControlModule] Entered incremental mode successfully\033[0m" << std::endl;
}

void IncrementalControlModule::exitIncrementalMode(const ArmPose& vrLeftPose,
                                                   const ArmPose& vrRightPose,
                                                   const ArmPose& vrLeftElbowPose,
                                                   const ArmPose& vrRightElbowPose,
                                                   const std::vector<PoseData>& latestPoseConstraintList) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_) {
    ROS_WARN("[IncrementalControlModule] Cannot exit incremental mode: module not initialized");
    return;
  }
  poseCalculator_->updateHumanPoseAnchor(
      vrLeftPose, vrRightPose, vrLeftElbowPose, vrRightElbowPose, latestPoseConstraintList);
  stateMachine_->exitIncrementalMode();

  armMoveDetector_.reset();
  poseCalculator_->updateLastTargetOnExit(latestPoseConstraintList);
  poseCalculator_->resetDelta();
  poseCalculator_->resetSlerpFactor();

  ROS_INFO("[IncrementalControlModule] Exited incremental mode successfully - all cleanup steps completed");
}

IncrementalPoseResult IncrementalControlModule::computeIncrementalPose(const ArmPose& vrLeftPose,
                                                                       const ArmPose& vrRightPose,
                                                                       const ArmPose& vrLeftElbowPose,
                                                                       const ArmPose& vrRightElbowPose) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  IncrementalPoseResult latestIncrementalResult;
  if (!initialized_ || !stateMachine_->isIncrementalMode()) {
    // print in red
    std::cout << "\033[91m[IncrementalControlModule] Not in incremental mode\033[0m" << std::endl;
    latestIncrementalResult.isValid = false;
    return latestIncrementalResult;
  }
  // print once in green color
  ROS_INFO_ONCE("[IncrementalControlModule] Computing incremental pose successfully");
  latestIncrementalResult =
      poseCalculator_->computeIncrementalPose(vrLeftPose, vrRightPose, vrLeftElbowPose, vrRightElbowPose);
  return latestIncrementalResult;
}

bool IncrementalControlModule::detectHumanArmMove(const Eigen::Vector3d& currentLeftHandPos,
                                                  const Eigen::Vector3d& currentRightHandPos) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_) return false;

  if (!armMoveDetector_.hasHumanArmMoved()) {
    armMoveDetector_.detectMovement(currentLeftHandPos, currentRightHandPos, config_.armMoveThreshold);
  }
  return armMoveDetector_.hasHumanArmMoved();
}

bool IncrementalControlModule::shouldEnterIncrementalMode() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return initialized_ && stateMachine_->shouldEnterIncrementalMode();
}

bool IncrementalControlModule::shouldExitIncrementalMode() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return !initialized_ || stateMachine_->shouldExitIncrementalMode();
}

bool IncrementalControlModule::isIncrementalMode() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return initialized_ && stateMachine_->isIncrementalMode();
}

bool IncrementalControlModule::hasHumanArmMoved() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return initialized_ && armMoveDetector_.hasHumanArmMoved();
}

void IncrementalControlModule::updateConfig(const IncrementalControlConfig& config) {
  config_ = config;
  if (poseCalculator_) {
    poseCalculator_->updateConfig(config);
  }
}

IncrementalPoseResult IncrementalControlModule::getLatestIncrementalResult() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!poseCalculator_) {
    IncrementalPoseResult emptyResult;
    emptyResult.isValid = false;
    return emptyResult;
  }
  return poseCalculator_->getLatestIncrementalResult();
}

void IncrementalControlModule::updateHandFkPositions(const Eigen::Vector3d& leftHandFkPos,
                                                     const Eigen::Vector3d& rightHandFkPos) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!poseCalculator_) {
    ROS_WARN("[IncrementalControlModule] Cannot update hand FK positions: poseCalculator_ is nullptr");
    return;
  }
  poseCalculator_->updateHandFkPositions(leftHandFkPos, rightHandFkPos);
}

const IncrementalControlConfig& IncrementalControlModule::getConfig() const { return config_; }

// ==================== IncrementalControlFactory Implementation ====================

std::unique_ptr<IncrementalControlModule> IncrementalControlFactory::createStandardModule(
    std::shared_ptr<JoyStickHandler> joyStickHandler) {
  return std::make_unique<IncrementalControlModule>(joyStickHandler);
}

std::unique_ptr<IncrementalControlModule> IncrementalControlFactory::createCustomModule(
    std::shared_ptr<JoyStickHandler> joyStickHandler,
    const IncrementalControlConfig& config) {
  return std::make_unique<IncrementalControlModule>(joyStickHandler, config);
}

}  // namespace HighlyDynamic
