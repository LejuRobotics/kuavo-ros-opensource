#include "motion_capture_ik/IncrementalControlModule.h"
#include "motion_capture_ik/JoyStickHandler.h"
#include <ros/ros.h>

namespace HighlyDynamic {

// ==================== IncrementalControlModule Implementation ====================

IncrementalControlModule::IncrementalControlModule(std::shared_ptr<JoyStickHandler> joyStickHandler,
                                                   const IncrementalControlConfig& config)
    : controlMode_(ControlMode::NONE),
      joyStickHandler_(joyStickHandler),
      result_(),
      armMoveDetector_(false),
      config_(config),
      initialized_(true),
      leftArmIncrementalMode_(false),
      rightArmIncrementalMode_(false),
      leftArmMoved_(false),
      rightArmMoved_(false),
      prevLeftHandPosition_(Eigen::Vector3d::Zero()),
      prevRightHandPosition_(Eigen::Vector3d::Zero()) {
  ROS_INFO("[IncrementalControlModule] Module initialized successfully");
}

// ==================== 状态机相关方法（原 IncrementalModeStateMachine） ====================

bool IncrementalControlModule::shouldEnterIncrementalMode() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_) return false;
  if (!joyStickHandler_) return false;
  if (controlMode_ == ControlMode::INCREMENTAL) return false;
  return joyStickHandler_->isLeftRightGrip();
}

bool IncrementalControlModule::shouldEnterIncrementalModeLeftArm() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_) return false;
  if (!joyStickHandler_) return false;
  if (leftArmIncrementalMode_) return false;  // 已经在增量模式，不需要重复进入
  // 检查左手是否激活（假设JoyStickHandler有相应方法）
  return joyStickHandler_->isLeftArmCtrlModeActive() || joyStickHandler_->isLeftGrip();
}

bool IncrementalControlModule::shouldEnterIncrementalModeRightArm() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_) return false;
  if (!joyStickHandler_) return false;
  if (rightArmIncrementalMode_) return false;  // 已经在增量模式，不需要重复进入
  // 检查右手是否激活（假设JoyStickHandler有相应方法）
  return joyStickHandler_->isRightArmCtrlModeActive() || joyStickHandler_->isRightGrip();
}

bool IncrementalControlModule::shouldExitIncrementalMode() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_) return true;
  if (!joyStickHandler_) return true;  // 手柄未初始化，则应该退出，避免意外失控
  if (controlMode_ != ControlMode::INCREMENTAL) return false;  // 不是增量模式，无需重复执行退出
  // 修改：只有当双手 grip 都松开时才退出增量模式（支持独立控制）
  // 原代码：return !joyStickHandler_->isLeftRightGrip(); // 只要有一手松开就退出，不支持独立控制
  return !joyStickHandler_->isLeftGrip() && !joyStickHandler_->isRightGrip();
}

bool IncrementalControlModule::isIncrementalMode() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return initialized_ && (controlMode_ == ControlMode::INCREMENTAL);
}

// ==================== 锚点和姿态相关方法（原 IncrementalPoseCalculator） ====================

void IncrementalControlModule::updateHumanPoseAnchor(const ArmPose& vrLeftPose,
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
    ROS_INFO("[IncrementalControlModule] Using hardcoded default for left hand initial target: [1.0, 1.0, 1.0]");
  }

  result_.lastTargetRightHandPosOnExit = latestPoseConstraintList[POSE_DATA_LIST_INDEX_RIGHT_HAND].position;
  if (result_.lastTargetRightHandPosOnExit.norm() < 1e-3) {
    result_.lastTargetRightHandPosOnExit = Eigen::Vector3d(0.05, -0.32, -0.05);
    ROS_INFO("[IncrementalControlModule] Using hardcoded default for right hand initial target: [1.0, -1.0, 1.0]");
  }
}

void IncrementalControlModule::updateLeftArmPoseAnchor(const ArmPose& vrLeftPose,
                                                       const ArmPose& vrLeftElbowPose,
                                                       const std::vector<PoseData>& latestPoseConstraintList) {
  // 只设置左臂的锚点位置
  result_.leftAnchorPos = vrLeftPose.position;
  result_.leftElbowAnchorPos = vrLeftElbowPose.position;

  // 初始化左手肘滤波状态
  if (latestPoseConstraintList.size() > POSE_DATA_LIST_INDEX_LEFT_ELBOW) {
    result_.leftElbowPosFiltered = latestPoseConstraintList[POSE_DATA_LIST_INDEX_LEFT_ELBOW].position;
  } else {
    result_.leftElbowPosFiltered = vrLeftElbowPose.position;
  }
  result_.dotLeftElbowPosFiltered.setZero();

  // 设置左手初始目标位置
  if (latestPoseConstraintList.size() > POSE_DATA_LIST_INDEX_LEFT_HAND) {
    result_.lastTargetLeftHandPosOnExit = latestPoseConstraintList[POSE_DATA_LIST_INDEX_LEFT_HAND].position;
    if (result_.lastTargetLeftHandPosOnExit.norm() < 1e-3) {
      result_.lastTargetLeftHandPosOnExit = Eigen::Vector3d(0.05, 0.32, -0.05);
      ROS_INFO("[IncrementalControlModule] Using hardcoded default for left hand initial target");
    }
  }

  // 重置左手增量和速度
  result_.leftHandPosDelta.setZero();
  result_.dotLeftHandPosDelta.setZero();
  result_.dotLastTargetLeftHandPosOnExit.setZero();

  // ROS_INFO("[IncrementalControlModule] Left arm anchor updated independently");
}

void IncrementalControlModule::updateRightArmPoseAnchor(const ArmPose& vrRightPose,
                                                        const ArmPose& vrRightElbowPose,
                                                        const std::vector<PoseData>& latestPoseConstraintList) {
  // 只设置右臂的锚点位置
  result_.rightAnchorPos = vrRightPose.position;
  result_.rightElbowAnchorPos = vrRightElbowPose.position;

  // 初始化右手肘滤波状态
  if (latestPoseConstraintList.size() > POSE_DATA_LIST_INDEX_RIGHT_ELBOW) {
    result_.rightElbowPosFiltered = latestPoseConstraintList[POSE_DATA_LIST_INDEX_RIGHT_ELBOW].position;
  } else {
    result_.rightElbowPosFiltered = vrRightElbowPose.position;
  }
  result_.dotRightElbowPosFiltered.setZero();

  // 设置右手初始目标位置
  if (latestPoseConstraintList.size() > POSE_DATA_LIST_INDEX_RIGHT_HAND) {
    result_.lastTargetRightHandPosOnExit = latestPoseConstraintList[POSE_DATA_LIST_INDEX_RIGHT_HAND].position;
    if (result_.lastTargetRightHandPosOnExit.norm() < 1e-3) {
      result_.lastTargetRightHandPosOnExit = Eigen::Vector3d(0.05, -0.32, -0.05);
      ROS_INFO("[IncrementalControlModule] Using hardcoded default for right hand initial target");
    }
  }

  // 重置右手增量和速度
  result_.rightHandPosDelta.setZero();
  result_.dotRightHandPosDelta.setZero();
  result_.dotLastTargetRightHandPosOnExit.setZero();

  // ROS_INFO("[IncrementalControlModule] Right arm anchor updated independently");
}

void IncrementalControlModule::updateLastTargetOnExit(const std::vector<PoseData>& latestPoseConstraintList) {
  result_.saveLastTargetOnExit(latestPoseConstraintList);
}

void IncrementalControlModule::resetDelta() { result_.resetDelta(); }

void IncrementalControlModule::resetSlerpFactor() { result_.resetSlerpFactor(); }

void IncrementalControlModule::computeFhanFiltering(const ArmPose& vrLeftPose,
                                                    const ArmPose& vrRightPose,
                                                    const ArmPose& vrLeftElbowPose,
                                                    const ArmPose& vrRightElbowPose,
                                                    bool isLeftActive,
                                                    bool isRightActive,
                                                    const double slerpQuatFactor) {
  if (slerpQuatFactor - 1.0 > 1e-6) {
    ROS_WARN("[IncrementalControlModule] slerpQuatFactor is not 1.0, it is %f", slerpQuatFactor);
    return;
  }
  // 平滑插值
  double fhan_h = 1.0 / config_.publishRate;
  double fhan_h0 = fhan_h * config_.fhan_kh0;

  double slerpt_r = config_.fhan_r / 10;
  if (isLeftActive) {
    leju_utils::fhanStepForward(result_.leftSlerpQuat_t_,
                                result_.leftSlerpQuat_dt_,
                                slerpQuatFactor,
                                slerpt_r,  //加速度约束
                                fhan_h,    // dt
                                fhan_h0);  //平滑系数，通常为dt的2~10倍
  }
  if (isRightActive) {
    leju_utils::fhanStepForward(result_.rightSlerpQuat_t_,
                                result_.rightSlerpQuat_dt_,
                                slerpQuatFactor,
                                slerpt_r,  //加速度约束
                                fhan_h,    // dt
                                fhan_h0);  //平滑系数，通常为dt的2~10倍
  }

  for (int i = 0; i < 3; i++) {
    // ==================== 左臂手部位置增量滤波 ====================
    // 关键修复：只有当左臂激活时才更新左臂的 fhan 滤波状态
    // 避免未激活时滤波状态向 0 收敛导致的状态污染
    if (isLeftActive) {
      double rawLeftPosDelta = (vrLeftPose.position[i] - result_.leftAnchorPos[i]) * config_.deltaScale[i];
      leju_utils::fhanStepForward(result_.leftHandPosDelta[i],     //约束处理后的位置增量
                                  result_.dotLeftHandPosDelta[i],  //约束处理后的速度
                                  rawLeftPosDelta,                 //原始位置增量
                                  config_.fhan_r,                  //加速度约束
                                  fhan_h,                          // dt
                                  fhan_h0);                        //平滑系数，通常为dt的2~10倍
    }
    // 未激活时保持滤波状态不变，不向 0 收敛

    // ==================== 右臂手部位置增量滤波 ====================
    // 关键修复：只有当右臂激活时才更新右臂的 fhan 滤波状态
    if (isRightActive) {
      double rawRightPosDelta = (vrRightPose.position[i] - result_.rightAnchorPos[i]) * config_.deltaScale[i];
      leju_utils::fhanStepForward(result_.rightHandPosDelta[i],  //右手
                                  result_.dotRightHandPosDelta[i],
                                  rawRightPosDelta,
                                  config_.fhan_r,
                                  fhan_h,
                                  fhan_h0);
    }
    // 未激活时保持滤波状态不变，不向 0 收敛

    // ==================== 肘部绝对位置滤波 ====================
    // 使用fhan算法对肘部绝对位置进行平滑滤波，避免重新进入增量模式时的跳变
    // 参考手部绝对姿态的更新逻辑，但使用绝对位置而非增量
    // 同样只在激活时更新滤波状态
    if (isLeftActive) {
      double rawLeftElbowPos = vrLeftElbowPose.position[i];
      leju_utils::fhanStepForward(result_.leftElbowPosFiltered[i],     // 滤波后的左肘绝对位置
                                  result_.dotLeftElbowPosFiltered[i],  // 滤波后的左肘速度
                                  rawLeftElbowPos,                     // 原始左肘绝对位置
                                  config_.fhan_r / 10,                 // 加速度约束
                                  fhan_h,
                                  fhan_h0);
    }

    if (isRightActive) {
      double rawRightElbowPos = vrRightElbowPose.position[i];
      leju_utils::fhanStepForward(result_.rightElbowPosFiltered[i],  // 滤波后的右肘绝对位置
                                  result_.dotRightElbowPosFiltered[i],
                                  rawRightElbowPos,  // 原始右肘绝对位置
                                  config_.fhan_r / 10,
                                  fhan_h,
                                  fhan_h0);
    }
  }
}

void IncrementalControlModule::updateLeftArmAnchor(const ArmPose& vrLeftPose,
                                                   const Eigen::Vector3d& currentRobotLeftHandPos) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  result_.leftAnchorPos = vrLeftPose.position;
  result_.leftHandPosDelta.setZero();
  result_.dotLeftHandPosDelta.setZero();

  result_.lastTargetLeftHandPosOnExit = currentRobotLeftHandPos;  // latestPoseConstraintList_ 中的位置值作为起点
  result_.dotLastTargetLeftHandPosOnExit.setZero();
}

void IncrementalControlModule::updateRightArmAnchor(const ArmPose& vrRightPose,
                                                    const Eigen::Vector3d& currentRobotRightHandPos) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  result_.rightAnchorPos = vrRightPose.position;
  result_.rightHandPosDelta.setZero();
  result_.dotRightHandPosDelta.setZero();

  result_.lastTargetRightHandPosOnExit = currentRobotRightHandPos;  // latestPoseConstraintList_ 中的位置值作为起点
  result_.dotLastTargetRightHandPosOnExit.setZero();
}

void IncrementalControlModule::setHandQuatSeeds(const Eigen::Quaterniond& leftHandQuatSeed,
                                                const Eigen::Quaterniond& rightHandQuatSeed) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  result_.lastTargetLeftHandQuatOnExit = leftHandQuatSeed.normalized();
  result_.lastTargetRightHandQuatOnExit = rightHandQuatSeed.normalized();
  // 同步当前插值目标，避免首次 slerp 从单位四元数开始
  result_.latestTargetLeftHandQuatSlerp = result_.lastTargetLeftHandQuatOnExit;
  result_.latestTargetRightHandQuatSlerp = result_.lastTargetRightHandQuatOnExit;
}

// ==================== 主要功能方法 ====================

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

  controlMode_ = ControlMode::INCREMENTAL;

  leftArmIncrementalMode_ = true;
  rightArmIncrementalMode_ = true;
  leftArmMoved_ = false;
  rightArmMoved_ = false;
  prevLeftHandPosition_ = vrLeftPose.position;
  prevRightHandPosition_ = vrRightPose.position;

  updateHumanPoseAnchor(vrLeftPose, vrRightPose, vrLeftElbowPose, vrRightElbowPose, latestPoseConstraintList);
}

void IncrementalControlModule::enterIncrementalModeLeftArm(const ArmPose& vrLeftPose,
                                                           const ArmPose& vrLeftElbowPose,
                                                           const std::vector<PoseData>& latestPoseConstraintList) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_) {
    ROS_WARN("[IncrementalControlModule] Module not initialized");
    return;
  }

  // 如果不在增量模式，则进入增量模式
  if (controlMode_ != ControlMode::INCREMENTAL) {
    controlMode_ = ControlMode::INCREMENTAL;
    // ROS_INFO("[IncrementalControlModule] Entering incremental mode via left arm");
  }

  // 标记左臂进入增量模式
  leftArmIncrementalMode_ = true;
  leftArmMoved_ = false;
  prevLeftHandPosition_ = vrLeftPose.position;

  // 只更新左臂的锚点
  updateLeftArmPoseAnchor(vrLeftPose, vrLeftElbowPose, latestPoseConstraintList);
}

void IncrementalControlModule::enterIncrementalModeRightArm(const ArmPose& vrRightPose,
                                                            const ArmPose& vrRightElbowPose,
                                                            const std::vector<PoseData>& latestPoseConstraintList) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_) {
    ROS_WARN("[IncrementalControlModule] Module not initialized");
    return;
  }

  // 如果不在增量模式，则进入增量模式
  if (controlMode_ != ControlMode::INCREMENTAL) {
    controlMode_ = ControlMode::INCREMENTAL;
    ROS_INFO("[IncrementalControlModule] Entering incremental mode via right arm");
  }

  // 标记右臂进入增量模式
  rightArmIncrementalMode_ = true;
  rightArmMoved_ = false;
  prevRightHandPosition_ = vrRightPose.position;

  // 只更新右臂的锚点
  updateRightArmPoseAnchor(vrRightPose, vrRightElbowPose, latestPoseConstraintList);
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
  updateHumanPoseAnchor(vrLeftPose, vrRightPose, vrLeftElbowPose, vrRightElbowPose, latestPoseConstraintList);
  controlMode_ = ControlMode::NONE;

  // 重置独立控制状态
  leftArmIncrementalMode_ = false;
  rightArmIncrementalMode_ = false;
  leftArmMoved_ = false;
  rightArmMoved_ = false;

  armMoveDetector_.reset();
  updateLastTargetOnExit(latestPoseConstraintList);
  resetDelta();
  resetSlerpFactor();
}

void IncrementalControlModule::exitIncrementalModeLeftArm(const ArmPose& vrLeftPose,
                                                          const ArmPose& vrLeftElbowPose,
                                                          const std::vector<PoseData>& latestPoseConstraintList) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_) {
    ROS_WARN("[IncrementalControlModule] Cannot exit incremental mode: module not initialized");
    return;
  }

  // 更新左臂锚点
  updateLeftArmPoseAnchor(vrLeftPose, vrLeftElbowPose, latestPoseConstraintList);

  // 标记左臂退出增量模式
  leftArmIncrementalMode_ = false;
  leftArmMoved_ = false;

  // 如果双臂都退出，则整体退出增量模式
  if (!leftArmIncrementalMode_ && !rightArmIncrementalMode_) {
    controlMode_ = ControlMode::NONE;
    armMoveDetector_.reset();
    ROS_INFO("[IncrementalControlModule] Both arms exited, exiting incremental mode completely");
  }

  // 保存左手退出时的目标位置
  if (latestPoseConstraintList.size() > POSE_DATA_LIST_INDEX_LEFT_HAND) {
    result_.lastTargetLeftHandPosOnExit = latestPoseConstraintList[POSE_DATA_LIST_INDEX_LEFT_HAND].position;
    result_.lastTargetLeftHandQuatOnExit =
        Eigen::Quaterniond(latestPoseConstraintList[POSE_DATA_LIST_INDEX_LEFT_HAND].rotation_matrix).normalized();
  }
  if (latestPoseConstraintList.size() > POSE_DATA_LIST_INDEX_LEFT_ELBOW) {
    result_.lastTargetLeftElbowPosOnExit = latestPoseConstraintList[POSE_DATA_LIST_INDEX_LEFT_ELBOW].position;
  }

  // 重置左手增量
  result_.leftHandPosDelta.setZero();
  result_.dotLeftHandPosDelta.setZero();
  result_.dotLastTargetLeftHandPosOnExit.setZero();

  // 重置左手Slerp因子
  result_.leftSlerpQuat_t_ = 0.0;
  result_.leftSlerpQuat_dt_ = 0.0;

  ROS_INFO("[IncrementalControlModule] Left arm exited incremental mode independently");
}

void IncrementalControlModule::exitIncrementalModeRightArm(const ArmPose& vrRightPose,
                                                           const ArmPose& vrRightElbowPose,
                                                           const std::vector<PoseData>& latestPoseConstraintList) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_) {
    ROS_WARN("[IncrementalControlModule] Cannot exit incremental mode: module not initialized");
    return;
  }

  // 更新右臂锚点
  updateRightArmPoseAnchor(vrRightPose, vrRightElbowPose, latestPoseConstraintList);

  // 标记右臂退出增量模式
  rightArmIncrementalMode_ = false;
  rightArmMoved_ = false;

  // 如果双臂都退出，则整体退出增量模式
  if (!leftArmIncrementalMode_ && !rightArmIncrementalMode_) {
    controlMode_ = ControlMode::NONE;
    armMoveDetector_.reset();
    ROS_INFO("[IncrementalControlModule] Both arms exited, exiting incremental mode completely");
  }

  // 保存右手退出时的目标位置
  if (latestPoseConstraintList.size() > POSE_DATA_LIST_INDEX_RIGHT_HAND) {
    result_.lastTargetRightHandPosOnExit = latestPoseConstraintList[POSE_DATA_LIST_INDEX_RIGHT_HAND].position;
    result_.lastTargetRightHandQuatOnExit =
        Eigen::Quaterniond(latestPoseConstraintList[POSE_DATA_LIST_INDEX_RIGHT_HAND].rotation_matrix).normalized();
  }
  if (latestPoseConstraintList.size() > POSE_DATA_LIST_INDEX_RIGHT_ELBOW) {
    result_.lastTargetRightElbowPosOnExit = latestPoseConstraintList[POSE_DATA_LIST_INDEX_RIGHT_ELBOW].position;
  }

  // 重置右手增量
  result_.rightHandPosDelta.setZero();
  result_.dotRightHandPosDelta.setZero();
  result_.dotLastTargetRightHandPosOnExit.setZero();

  // 重置右手Slerp因子
  result_.rightSlerpQuat_t_ = 0.0;
  result_.rightSlerpQuat_dt_ = 0.0;

  ROS_INFO("[IncrementalControlModule] Right arm exited incremental mode independently");
}

IncrementalPoseResult IncrementalControlModule::computeIncrementalPose(const ArmPose& vrLeftPose,
                                                                       const ArmPose& vrRightPose,
                                                                       const ArmPose& vrLeftElbowPose,
                                                                       const ArmPose& vrRightElbowPose,
                                                                       bool isLeftActive,
                                                                       bool isRightActive) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  IncrementalPoseResult latestIncrementalResult;
  if (!initialized_ || controlMode_ != ControlMode::INCREMENTAL) {
    // print in red
    std::cout << "\033[91m[IncrementalControlModule] Not in incremental mode\033[0m" << std::endl;
    latestIncrementalResult.isValid = false;
    return latestIncrementalResult;
  }
  // print once in green color
  ROS_INFO_ONCE("[IncrementalControlModule] Computing incremental pose successfully");

  computeFhanFiltering(vrLeftPose, vrRightPose, vrLeftElbowPose, vrRightElbowPose, isLeftActive, isRightActive);
  result_.slerpQuat(vrLeftPose.quaternion, vrRightPose.quaternion, isLeftActive, isRightActive);
  result_.isValid = true;

  return result_;
}

IncrementalPoseResult IncrementalControlModule::computeIncrementalPoseLeftArm(const ArmPose& vrLeftPose,
                                                                              const ArmPose& vrLeftElbowPose,
                                                                              bool isLeftActive) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  IncrementalPoseResult latestIncrementalResult;
  if (!initialized_ || (!leftArmIncrementalMode_ && controlMode_ != ControlMode::INCREMENTAL)) {
    std::cout << "\033[91m[IncrementalControlModule] Left arm not in incremental mode\033[0m" << std::endl;
    latestIncrementalResult.isValid = false;
    return latestIncrementalResult;
  }

  ROS_INFO_ONCE("[IncrementalControlModule] Computing left arm incremental pose successfully");

  // 创建虚拟右臂姿态（使用当前result_中的右臂数据）
  ArmPose vrRightPose;
  vrRightPose.position = result_.rightAnchorPos;
  vrRightPose.quaternion = result_.lastTargetRightHandQuatOnExit;

  ArmPose vrRightElbowPose;
  vrRightElbowPose.position = result_.rightElbowAnchorPos;

  // 计算时只激活左臂
  computeFhanFiltering(vrLeftPose, vrRightPose, vrLeftElbowPose, vrRightElbowPose, isLeftActive, false);
  result_.slerpQuat(vrLeftPose.quaternion, vrRightPose.quaternion, isLeftActive, false);
  result_.isValid = true;

  return result_;
}

IncrementalPoseResult IncrementalControlModule::computeIncrementalPoseRightArm(const ArmPose& vrRightPose,
                                                                               const ArmPose& vrRightElbowPose,
                                                                               bool isRightActive) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  IncrementalPoseResult latestIncrementalResult;
  if (!initialized_ || (!rightArmIncrementalMode_ && controlMode_ != ControlMode::INCREMENTAL)) {
    std::cout << "\033[91m[IncrementalControlModule] Right arm not in incremental mode\033[0m" << std::endl;
    latestIncrementalResult.isValid = false;
    return latestIncrementalResult;
  }

  ROS_INFO_ONCE("[IncrementalControlModule] Computing right arm incremental pose successfully");

  // 创建虚拟左臂姿态（使用当前result_中的左臂数据）
  ArmPose vrLeftPose;
  vrLeftPose.position = result_.leftAnchorPos;
  vrLeftPose.quaternion = result_.lastTargetLeftHandQuatOnExit;

  ArmPose vrLeftElbowPose;
  vrLeftElbowPose.position = result_.leftElbowAnchorPos;

  // 计算时只激活右臂
  computeFhanFiltering(vrLeftPose, vrRightPose, vrLeftElbowPose, vrRightElbowPose, false, isRightActive);
  result_.slerpQuat(vrLeftPose.quaternion, vrRightPose.quaternion, false, isRightActive);
  result_.isValid = true;

  return result_;
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

bool IncrementalControlModule::detectLeftArmMove(const Eigen::Vector3d& currentLeftHandPos) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_) return false;

  // 如果左臂已经移动，直接返回true
  if (leftArmMoved_) {
    return true;
  }

  // 如果是第一次检测，记录初始位置
  if (prevLeftHandPosition_.isZero()) {
    prevLeftHandPosition_ = currentLeftHandPos;
    return false;
  }

  // 计算左手位置的norm误差
  double leftHandNormError = (currentLeftHandPos - prevLeftHandPosition_).norm();

  // 更新上一帧位置
  prevLeftHandPosition_ = currentLeftHandPos;

  // 检查是否超过阈值
  if (leftHandNormError > config_.armMoveThreshold) {
    leftArmMoved_ = true;
    ROS_INFO("[IncrementalControlModule] Left arm movement detected: %.4f", leftHandNormError);
    return true;
  }

  return false;
}

bool IncrementalControlModule::detectRightArmMove(const Eigen::Vector3d& currentRightHandPos) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_) return false;

  // 如果右臂已经移动，直接返回true
  if (rightArmMoved_) {
    return true;
  }

  // 如果是第一次检测，记录初始位置
  if (prevRightHandPosition_.isZero()) {
    prevRightHandPosition_ = currentRightHandPos;
    return false;
  }

  // 计算右手位置的norm误差
  double rightHandNormError = (currentRightHandPos - prevRightHandPosition_).norm();

  // 更新上一帧位置
  prevRightHandPosition_ = currentRightHandPos;

  // 检查是否超过阈值
  if (rightHandNormError > config_.armMoveThreshold) {
    rightArmMoved_ = true;
    ROS_INFO("[IncrementalControlModule] Right arm movement detected: %.4f", rightHandNormError);
    return true;
  }

  return false;
}

bool IncrementalControlModule::hasHumanArmMoved() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return initialized_ && armMoveDetector_.hasHumanArmMoved();
}

bool IncrementalControlModule::hasLeftArmMoved() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return initialized_ && leftArmMoved_;
}

bool IncrementalControlModule::hasRightArmMoved() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return initialized_ && rightArmMoved_;
}

bool IncrementalControlModule::shouldExitIncrementalModeLeftArm() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_) return true;
  if (!joyStickHandler_) return true;          // 手柄未初始化，则应该退出
  if (!leftArmIncrementalMode_) return false;  // 不在增量模式，无需重复执行退出
  // 检查左手是否应该退出（假设JoyStickHandler有相应方法）
  return !joyStickHandler_->isLeftArmCtrlModeActive();
}

bool IncrementalControlModule::shouldExitIncrementalModeRightArm() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_) return true;
  if (!joyStickHandler_) return true;           // 手柄未初始化，则应该退出
  if (!rightArmIncrementalMode_) return false;  // 不在增量模式，无需重复执行退出
  // 检查右手是否应该退出（假设JoyStickHandler有相应方法）
  return !joyStickHandler_->isRightArmCtrlModeActive();
}

bool IncrementalControlModule::isIncrementalModeLeftArm() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return initialized_ && leftArmIncrementalMode_;
}

bool IncrementalControlModule::isIncrementalModeRightArm() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return initialized_ && rightArmIncrementalMode_;
}

void IncrementalControlModule::updateConfig(const IncrementalControlConfig& config) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  config_ = config;
}

IncrementalPoseResult IncrementalControlModule::getLatestIncrementalResult() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_) {
    IncrementalPoseResult emptyResult;
    emptyResult.isValid = false;
    return emptyResult;
  }
  return result_;
}

const IncrementalControlConfig& IncrementalControlModule::getConfig() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return config_;
}

void IncrementalControlModule::reset() {
  std::lock_guard<std::mutex> lock(stateMutex_);

  // 重置控制模式
  controlMode_ = ControlMode::NONE;

  // 重置增量计算结果
  result_ = IncrementalPoseResult();

  // 重置手臂移动检测器
  armMoveDetector_.reset();

  // 重置独立控制状态
  leftArmIncrementalMode_ = false;
  rightArmIncrementalMode_ = false;
  leftArmMoved_ = false;
  rightArmMoved_ = false;

  // 重置上一帧手部位置
  prevLeftHandPosition_.setZero();
  prevRightHandPosition_.setZero();

  // ROS_INFO("[IncrementalControlModule] Module reset to initial state");
}

}  // namespace HighlyDynamic
