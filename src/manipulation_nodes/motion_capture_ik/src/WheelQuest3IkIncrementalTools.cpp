#include "motion_capture_ik/WheelQuest3IkIncrementalROS.h"

#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <drake/geometry/scene_graph.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/framework/diagram.h>
#include <drake/systems/framework/diagram_builder.h>
#include <kuavo_msgs/changeArmCtrlMode.h>
#include <kuavo_msgs/changeLbQuickModeSrv.h>
#include <kuavo_msgs/changeTorsoCtrlMode.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32MultiArray.h>
#include <iomanip>
#include <cmath>
#include <algorithm>

#include <leju_utils/define.hpp>
#include <leju_utils/math.hpp>
#include <leju_utils/RosMsgConvertor.hpp>

#include "humanoid_wheel_interface/filters/KinemicLimitFilter.h"
#include "motion_capture_ik/WheelArmControlBaseROS.h"
#include "motion_capture_ik/Quest3ArmInfoTransformer.h"
#include "motion_capture_ik/json.hpp"
#include "motion_capture_ik/WheelIncrementalControlModule.h"
#include "motion_capture_ik/WheelJoyStickHandler.h"

namespace HighlyDynamic {
using namespace leju_utils::ros_msg_convertor;

void WheelQuest3IkIncrementalROS::activateController() {
  if (controllerActivated_.load()) return;

  if (!changeMobileCtrlModeClient_.exists()) return;
  if (!humanoidArmCtrlModeClient_.exists()) return;
  if (!changeArmCtrlModeClient_.exists()) return;

  ROS_INFO("[WheelQuest3IkIncrementalROS] Activating controller");
  kuavo_msgs::changeTorsoCtrlMode srv1;
  kuavo_msgs::changeArmCtrlMode srv2;

  srv1.request.control_mode = static_cast<int>(MpcRefUpdateMode::ENABLED_ARM);
  srv2.request.control_mode = static_cast<int>(KuavoArmCtrlMode::EXTERNAL_CONTROL);

  controllerActivated_.store(changeMobileCtrlModeClient_.call(srv1) && srv1.response.result &&  //
                             humanoidArmCtrlModeClient_.call(srv2) && srv2.response.result &&   //
                             changeArmCtrlModeClient_.call(srv2) && srv2.response.result &&     //
                             true);
}

void WheelQuest3IkIncrementalROS::deactivateController() {
  if (!controllerActivated_.load()) return;
  if (!changeMobileCtrlModeClient_.exists()) return;
  if (!humanoidArmCtrlModeClient_.exists()) return;
  if (!changeArmCtrlModeClient_.exists()) return;

  kuavo_msgs::changeTorsoCtrlMode srv1;
  kuavo_msgs::changeArmCtrlMode srv2;

  srv1.request.control_mode = static_cast<int>(MpcRefUpdateMode::DISABLED_ARM);
  srv2.request.control_mode = static_cast<int>(KuavoArmCtrlMode::ARM_FIXED);

  controllerActivated_.store(!(changeMobileCtrlModeClient_.call(srv1) && srv1.response.result &&  //
                               humanoidArmCtrlModeClient_.call(srv2) && srv2.response.result &&   //
                               changeArmCtrlModeClient_.call(srv2) && srv2.response.result &&     //
                               true));
}

void WheelQuest3IkIncrementalROS::armModeCallback(const std_msgs::Int32::ConstPtr& msg) {
  // print arm mode callback
  ROS_INFO("[WheelQuest3IkIncrementalROS] Arm mode callback: %d", msg->data);
  int newMode = msg->data;
  int oldMode = armControlMode_.load();

  if (oldMode != newMode) {
    lastArmControlMode_.store(oldMode);
    armControlMode_.store(newMode);
    ROS_INFO("[WheelQuest3IkIncrementalROS] Arm control mode changed from %d to %d", oldMode, newMode);

    // 记录进入 mode 2 的时间戳（0→2 和 1→2 都需要）
    if ((oldMode == 0 || oldMode == 1) && newMode == 2) {
      std::lock_guard<std::mutex> lock(mode2EnterTimeMutex_);
      mode2EnterTime_ = ros::Time::now();
      ROS_INFO("[WheelQuest3IkIncrementalROS] Mode 2 entered at time: %.3f, timeout duration: %.1f seconds",
               mode2EnterTime_.toSec(),
               MODE_2_TIMEOUT_DURATION);
    }
  } else {
    armControlMode_.store(newMode);
  }
}

void WheelQuest3IkIncrementalROS::updateLeftConstraintList(const Eigen::Vector3d& leftHandPos,
                                                      const Eigen::Quaterniond& leftHandQuat,
                                                      const Eigen::Vector3d& leftElbowPos) {
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].position = leftHandPos;

  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].rotation_matrix = leftHandQuat.toRotationMatrix();

  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_ELBOW].position = leftElbowPos;

  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_END_EFFECTOR].position = leftEndEffectorPosition_;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_LINK6].position = leftLink6Position_;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_VIRTUAL_THUMB].position = leftVirtualThumbPosition_;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_CHEST].rotation_matrix = chestRotationQuaternion_.toRotationMatrix();
}

void WheelQuest3IkIncrementalROS::updateRightConstraintList(const Eigen::Vector3d& rightHandPos,
                                                       const Eigen::Quaterniond& rightHandQuat,
                                                       const Eigen::Vector3d& rightElbowPos) {
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].position = rightHandPos;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].rotation_matrix = rightHandQuat.toRotationMatrix();

  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_ELBOW].position = rightElbowPos;

  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_END_EFFECTOR].position = rightEndEffectorPosition_;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_LINK6].position = rightLink6Position_;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_VIRTUAL_THUMB].position = rightVirtualThumbPosition_;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_CHEST].rotation_matrix =
      chestRotationQuaternion_.toRotationMatrix();  // 确保总是更新chest参考
}

Eigen::Quaterniond WheelQuest3IkIncrementalROS::computeYawPitchOnlyQuatFromRotationMatrix(
    const Eigen::Matrix3d& rotationMatrix) {
  // CZJTODO: 待验证优化
  // Model: R = Rz(yaw) * Ry(pitch)
  // R = [[cosy*cosp, -siny, cosy*sinp],
  //      [siny*cosp,  cosy, siny*sinp],
  //      [-sinp,      0,    cosp]]
  const double yaw = std::atan2(rotationMatrix(1, 0), rotationMatrix(0, 0));
  const double pitch = std::atan2(-rotationMatrix(2, 0), rotationMatrix(2, 2));
  Eigen::Quaterniond q =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());
  return q.normalized();
}

bool WheelQuest3IkIncrementalROS::updateWholeBodyConstraintList(const WholeBodyRefInput& input) {
  if (!chestElbowHandPointOptSolverPtr_) {
    ROS_ERROR("[WheelQuest3IkIncrementalROS] chestElbowHandPointOptSolverPtr_ is not initialized");
    return false;
  }
  bool chestIncrementalUpdateEnabled =
      chestIncrementalUpdateEnabled_ && (lastLeftGripPressed_ || lastRightGripPressed_);
  const bool bothGripsReleased = !lastLeftGripPressed_ && !lastRightGripPressed_;
  ros::Time mode2EnterTime;
  {
    std::lock_guard<std::mutex> lock(mode2EnterTimeMutex_);
    mode2EnterTime = mode2EnterTime_;
  }
  const bool isInMode2Warmup =
      (armControlMode_ == 2) && !mode2EnterTime.isZero() && ((ros::Time::now() - mode2EnterTime).toSec() <= 2.0);
  const bool freezeHeightEnabled = !isInMode2Warmup && bothGripsReleased && !chestIncrementalUpdateEnabled;

  Eigen::Vector3d chestPosRef = input.chestPosRef;
  Eigen::Quaterniond chestQuatRef = input.chestQuatRef;
  if (isInMode2Warmup) {
    chestPosRef = latestWaistYawFkPos_;
    chestIncrementalUpdateEnabled = false;
  }

  Eigen::Vector3d chestPos = chestPosRef;
  if (!chestIncrementalUpdateEnabled && !isInMode2Warmup) {
    chestPos = frozenRobotChestPos_;
  }

  Eigen::Vector3d leftElbowRef = input.leftElbowRef;
  Eigen::Vector3d rightElbowRef = input.rightElbowRef;
  Eigen::Vector3d leftHandRef = input.leftHandRef;
  Eigen::Vector3d rightHandRef = input.rightHandRef;

  if (isInMode2Warmup) {
    leftElbowRef = leftLink4Position_;
    rightElbowRef = rightLink4Position_;
    leftHandRef = leftLink6Position_;
    rightHandRef = rightLink6Position_;
  } else if (freezeHeightEnabled) {
    const double frozenChestZ = frozenRobotChestPos_.z();
    leftElbowRef.z() = frozenChestZ + frozenLeftElbowHeightOffset_;
    rightElbowRef.z() = frozenChestZ + frozenRightElbowHeightOffset_;
    leftHandRef.z() = frozenChestZ + frozenLeftHandHeightOffset_;
    rightHandRef.z() = frozenChestZ + frozenRightHandHeightOffset_;
  }
  // if (freezeHeightEnabled) {
  //   const Eigen::Matrix3d chestR = chestQuatRef.normalized().toRotationMatrix();
  //   const double yaw = std::atan2(chestR(1, 0), chestR(0, 0));
  //   chestQuatRef = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
  // }

  auto offset = chestPosRef - chestPos;
  recordTimestamp("chestElbowHandPointOptSolverPtr_->solveStart", loopSyncCount_);

  Eigen::Vector3d leftHandRefRel = leftHandRef - offset;
  Eigen::Vector3d rightHandRefRel = rightHandRef - offset;
  Eigen::Vector3d leftElbowRefRel = leftElbowRef - offset;
  Eigen::Vector3d rightElbowRefRel = rightElbowRef - offset;

  // 获取最新的shoulder FK位置并转换为相对坐标
  Eigen::Vector3d leftShoulderPosRel =
      hasLatestLeftShoulderFk_ ? (latestLeftShoulderFkPos_ - offset) : (robotLeftFixedShoulderPos_ - offset);
  Eigen::Vector3d rightShoulderPosRel =
      hasLatestRightShoulderFk_ ? (latestRightShoulderFkPos_ - offset) : (robotRightFixedShoulderPos_ - offset);

  if (!chestIncrementalUpdateEnabled_) {
    remapUpperBodyRefPoints(chestPos,
                            chestQuatRef,
                            leftShoulderPosRel,
                            rightShoulderPosRel,
                            leftHandRefRel,
                            rightHandRefRel,
                            leftElbowRefRel,
                            rightElbowRefRel);
  }
  const DrakeChestElbowHandSolution sol =
      chestElbowHandPointOptSolverPtr_->solve(chestPos,
                                              chestQuatRef,
                                              leftElbowRefRel,   // leftElbowRef - offset,
                                              leftHandRefRel,    // leftHandRef - offset,
                                              rightElbowRefRel,  // rightElbowRef - offset,
                                              rightHandRefRel);  // rightHandRef - offset;
  recordTimestamp("chestElbowHandPointOptSolverPtr_->solveFinish", loopSyncCount_);
  const bool freezeFeedAfterOpt =
      !isInMode2Warmup && bothGripsReleased && !chestIncrementalUpdateEnabled && sol.success;
  {
    if (chestIncrementalUpdateEnabled) {
      const Eigen::Matrix3d chestRRef = input.chestQuatRef.normalized().toRotationMatrix();
      const Eigen::Vector3d vLeftShoulderInChest = robotLeftFixedShoulderPos_ - robotFixedWaistYawPos_;
      const Eigen::Vector3d vRightShoulderInChest = robotRightFixedShoulderPos_ - robotFixedWaistYawPos_;
      const Eigen::Vector3d leftShoulderRef = input.chestPosRef + chestRRef * vLeftShoulderInChest;
      const Eigen::Vector3d rightShoulderRef = input.chestPosRef + chestRRef * vRightShoulderInChest;

      latestChestPosBeforeOpt_ = input.chestPosRef;
      latestChestPosAfterOpt_ = sol.pChest;

      latestLeftShoulderPosBeforeOpt_ = leftShoulderRef;
      latestLeftShoulderPosAfterOpt_ = sol.pLeftShoulder;
      latestRightShoulderPosBeforeOpt_ = rightShoulderRef;
      latestRightShoulderPosAfterOpt_ = sol.pRightShoulder;
    }

    // Elbow updates (always updated)
    latestLeftElbowPosBeforeOpt_ = input.leftElbowRef;
    latestLeftElbowPosAfterOpt_ = sol.pLeftElbow;
    latestRightElbowPosBeforeOpt_ = input.rightElbowRef;
    latestRightElbowPosAfterOpt_ = sol.pRightElbow;
  }
  if (input.leftRefActive || freezeFeedAfterOpt) {
    latestLeftHandPosBeforeOpt_ = leftHandRef;
    latestLeftHandPosAfterOpt_ = sol.pLeftHand;
  }
  if (input.rightRefActive || freezeFeedAfterOpt) {
    latestRightHandPosBeforeOpt_ = rightHandRef;
    latestRightHandPosAfterOpt_ = sol.pRightHand;
  }

  // Derived link6 / ee / virtual thumb points from optimized hand positions and the desired hand quaternions.
  // For inactive hands, keep the frozen references to avoid drifting with chest motion.
  Eigen::Vector3d leftHandOut = sol.pLeftHand;
  Eigen::Vector3d rightHandOut = sol.pRightHand;
  Eigen::Vector3d leftElbowOut = sol.pLeftElbow;
  Eigen::Vector3d rightElbowOut = sol.pRightElbow;
  if (freezeHeightEnabled) {
    const double frozenChestZ = frozenRobotChestPos_.z();
    leftHandOut.z() = frozenChestZ + frozenLeftHandHeightOffset_;
    rightHandOut.z() = frozenChestZ + frozenRightHandHeightOffset_;
    leftElbowOut.z() = frozenChestZ + frozenLeftElbowHeightOffset_;
    rightElbowOut.z() = frozenChestZ + frozenRightElbowHeightOffset_;
  }
  if (!input.leftRefActive) {
    leftHandOut = leftHandRef;
    leftElbowOut = leftElbowRef;
  }
  if (!input.rightRefActive) {
    rightHandOut = rightHandRef;
    rightElbowOut = rightElbowRef;
  }

  const Eigen::Quaterniond leftHandQuat = input.leftHandQuat.normalized();
  const Eigen::Quaterniond rightHandQuat = input.rightHandQuat.normalized();

  // Commit a consistent snapshot into constraint list.
  if (latestPoseConstraintList_.size() < POSE_DATA_LIST_SIZE_PLUS) {
    latestPoseConstraintList_.resize(POSE_DATA_LIST_SIZE_PLUS, PoseData());
  }

  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_CHEST].position = sol.pChest;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_CHEST].rotation_matrix = chestQuatRef.toRotationMatrix();
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_SHOULDER].position = sol.pLeftShoulder;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_SHOULDER].position = sol.pRightShoulder;

  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_ELBOW].position = leftElbowOut;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_ELBOW].position = rightElbowOut;

  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].position = leftHandOut;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].rotation_matrix = leftHandQuat.toRotationMatrix();
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].position = rightHandOut;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].rotation_matrix = rightHandQuat.toRotationMatrix();

  leftLink6Position_ = leftHandOut;
  rightLink6Position_ = rightHandOut;
  leftEndEffectorPosition_ = leftLink6Position_ + (leftHandQuat * leftEE2Link6Offset_);
  rightEndEffectorPosition_ = rightLink6Position_ + (rightHandQuat * rightEE2Link6Offset_);
  leftVirtualThumbPosition_ = leftLink6Position_ + (leftHandQuat * leftThumb2Link6Offset_);
  rightVirtualThumbPosition_ = rightLink6Position_ + (rightHandQuat * rightThumb2Link6Offset_);

  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_LINK6].position = leftLink6Position_;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_LINK6].position = rightLink6Position_;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_END_EFFECTOR].position = leftEndEffectorPosition_;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_END_EFFECTOR].position = rightEndEffectorPosition_;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_VIRTUAL_THUMB].position = leftVirtualThumbPosition_;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_VIRTUAL_THUMB].position = rightVirtualThumbPosition_;

  return true;
}

bool WheelQuest3IkIncrementalROS::detectLeftArmMove() {
  // 检查左臂是否已经移动
  if (incrementalController_->hasLeftArmMoved()) {
    return true;
  }

  // 检测左臂移动
  incrementalController_->detectLeftArmMove(latestLeftHandPose_vr_.position);

  return incrementalController_->hasLeftArmMoved();
}

bool WheelQuest3IkIncrementalROS::detectRightArmMove() {
  // 检查右臂是否已经移动
  if (incrementalController_->hasRightArmMoved()) {
    return true;
  }

  // 检测右臂移动
  incrementalController_->detectRightArmMove(latestRightHandPose_vr_.position);

  return incrementalController_->hasRightArmMoved();
}

bool WheelQuest3IkIncrementalROS::updateLatestIncrementalResult() {
  bool isLeftActive = joyStickHandlerPtr_->isLeftArmCtrlModeActive();
  bool isRightActive = joyStickHandlerPtr_->isRightArmCtrlModeActive();

  // 计算ee的fk值用于实时更新
  Eigen::Vector3d pLeftEndEffector, pRightEndEffector;
  Eigen::Quaterniond qLeftEndEffector, qRightEndEffector;
  if (isLeftActive) {
    computeLeftEndEffectorFK(pLeftEndEffector, qLeftEndEffector);
  }
  if (isRightActive) {
    computeRightEndEffectorFK(pRightEndEffector, qRightEndEffector);
  }

  latestIncrementalResult_ = incrementalController_->computeIncrementalPose(latestLeftHandPose_vr_,
                                                                            latestRightHandPose_vr_,
                                                                            isLeftActive,
                                                                            isRightActive,
                                                                            qLeftEndEffector,
                                                                            qRightEndEffector);
  return latestIncrementalResult_.isValid();
}

bool WheelQuest3IkIncrementalROS::updateLeftHandChangingMode(const Eigen::Vector3d& leftTargetPos) {
  // 使用均值滤波后的关节数据
  if (filterJointDataForDrakeFK_.size() != drakeJointStateSize_) {
    ROS_WARN("[WheelQuest3IkIncrementalROS] Filtered joint data not available for FK calculation");
    return false;
  }

  Eigen::VectorXd armJoints = filterJointDataForDrakeFK_;
  return leftHandSmoother_->updateChangingMode(leftTargetPos,
                                               static_cast<BaseIKSolver*>(oneStageIkEndEffectorPtr_.get()),
                                               armJoints,
                                               drakeJointStateSize_,
                                               handChangingModeThreshold_);
}

bool WheelQuest3IkIncrementalROS::updateRightHandChangingMode(const Eigen::Vector3d& rightTargetPos) {
  // 使用均值滤波后的关节数据
  if (filterJointDataForDrakeFK_.size() != drakeJointStateSize_) {
    ROS_WARN("[WheelQuest3IkIncrementalROS] Filtered joint data not available for FK calculation");
    return false;
  }

  Eigen::VectorXd armJoints = filterJointDataForDrakeFK_;
  return rightHandSmoother_->updateChangingMode(rightTargetPos,
                                                static_cast<BaseIKSolver*>(oneStageIkEndEffectorPtr_.get()),
                                                armJoints,
                                                drakeJointStateSize_,
                                                handChangingModeThreshold_);
}

void WheelQuest3IkIncrementalROS::forceDeactivateAllArmCtrlMode() {  // 强制停用所有手臂控制模式，确保可以进入 fsmChange 流程
  if (joyStickHandlerPtr_) {
    if (joyStickHandlerPtr_->isLeftArmCtrlModeActive()) {
      joyStickHandlerPtr_->forceSetLeftArmCtrlMode(false);
    }
    if (joyStickHandlerPtr_->isRightArmCtrlModeActive()) {
      joyStickHandlerPtr_->forceSetRightArmCtrlMode(false);
    }
  }
}

void WheelQuest3IkIncrementalROS::forceActivateAllArmCtrlMode() {
  if (joyStickHandlerPtr_) {
    if (!joyStickHandlerPtr_->isLeftArmCtrlModeActive()) {
      joyStickHandlerPtr_->forceSetLeftArmCtrlMode(true);
    }
    if (!joyStickHandlerPtr_->isRightArmCtrlModeActive()) {
      joyStickHandlerPtr_->forceSetRightArmCtrlMode(true);
    }
  }
}

bool WheelQuest3IkIncrementalROS::setLbArmQuickMode(int quickMode) {
  // 设置轮臂快速模式服务调用函数
  // 参数：quickMode - 快速模式类型：0-关闭, 1-下肢快, 2-上肢快, 3-上下肢快
  // 返回：成功返回true，失败返回false
  {
    std::lock_guard<std::mutex> lock(lbQuickModeRequestMutex_);
    if (lastLbQuickModeRequested_ == quickMode && lastLbQuickModeRequestSuccess_) {
      // 已成功设置为该模式：不重复发送服务请求
      return true;
    }
    if (lastLbQuickModeRequested_ == quickMode && !lastLbQuickModeRequestTime_.isZero()) {
      const double elapsed = (ros::Time::now() - lastLbQuickModeRequestTime_).toSec();
      if (elapsed < LB_QUICK_MODE_RETRY_INTERVAL_SEC) {
        // 上一次失败/未确认成功且间隔过短：不刷服务
        return false;
      }
    }
    lastLbQuickModeRequested_ = quickMode;
    lastLbQuickModeRequestTime_ = ros::Time::now();
    lastLbQuickModeRequestSuccess_ = false;
  }

  if (!enableLbArmQuickModeClient_.exists()) {
    ROS_WARN("[WheelQuest3IkIncrementalROS] Service /enable_lb_arm_quick_mode does not exist");
    return false;
  }

  kuavo_msgs::changeLbQuickModeSrv srv;
  srv.request.quickMode = quickMode;

  if (enableLbArmQuickModeClient_.call(srv)) {
    if (srv.response.success) {
      // ROS_INFO("[WheelQuest3IkIncrementalROS] Successfully set lb arm quick mode to %d", quickMode);
      {
        std::lock_guard<std::mutex> lock(lbQuickModeRequestMutex_);
        lastLbQuickModeRequestSuccess_ = true;
      }
      return true;
    } else {
      ROS_WARN("[WheelQuest3IkIncrementalROS] Failed to set lb arm quick mode to %d", quickMode);
      return false;
    }
  } else {
    ROS_ERROR("[WheelQuest3IkIncrementalROS] Service call to /enable_lb_arm_quick_mode failed");
    return false;
  }
}

bool WheelQuest3IkIncrementalROS::setControlMode(int targetMode) {
  // 设置移动机械臂控制模式服务调用函数（对应lb_ctrl_api.set_control_mode）
  // 参数：targetMode - 控制模式：0-NoControl, 1-ArmOnly, 2-BaseOnly, 3-BaseArm, 4-ArmEeOnly
  // 返回：成功返回true，失败返回false
  static const std::map<int, std::string> MODES = {
      {0, "NoControl"}, {1, "ArmOnly"}, {2, "BaseOnly"}, {3, "BaseArm"}, {4, "ArmEeOnly"}};

  if (MODES.find(targetMode) == MODES.end()) {
    ROS_ERROR("[WheelQuest3IkIncrementalROS] Invalid control mode %d, allowed values: 0-4", targetMode);
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(controlModeRequestMutex_);
    if (lastControlModeRequested_ == targetMode && lastControlModeRequestSuccess_) {
      // 已成功设置为该模式：不重复发送服务请求
      return true;
    }
    if (lastControlModeRequested_ == targetMode && !lastControlModeRequestTime_.isZero()) {
      const double elapsed = (ros::Time::now() - lastControlModeRequestTime_).toSec();
      if (elapsed < CONTROL_MODE_RETRY_INTERVAL_SEC) {
        // 上一次失败/未确认成功且间隔过短：不刷服务
        return false;
      }
    }
    lastControlModeRequested_ = targetMode;
    lastControlModeRequestTime_ = ros::Time::now();
    lastControlModeRequestSuccess_ = false;
  }

  if (!changeTorsoCtrlModeClient_.exists()) {
    ROS_WARN("[WheelQuest3IkIncrementalROS] Service /mobile_manipulator_mpc_control does not exist");
    return false;
  }

  kuavo_msgs::changeTorsoCtrlMode srv;
  srv.request.control_mode = targetMode;

  if (changeTorsoCtrlModeClient_.call(srv)) {
    if (srv.response.result) {
      // ROS_INFO(
      //     "[WheelQuest3IkIncrementalROS] Successfully set control mode to %d: %s", targetMode,
      //     MODES.at(targetMode).c_str());
      {
        std::lock_guard<std::mutex> lock(controlModeRequestMutex_);
        lastControlModeRequestSuccess_ = true;
      }
      return true;
    } else {
      ROS_ERROR(
          "[WheelQuest3IkIncrementalROS] Failed to set control mode to %d: %s", targetMode, srv.response.message.c_str());
      return false;
    }
  } else {
    ROS_ERROR("[WheelQuest3IkIncrementalROS] Service call to /mobile_manipulator_mpc_control failed");
    return false;
  }
}

void WheelQuest3IkIncrementalROS::reset() {
  // 持续重置各类状态，确保进入系统时正常
  // 重置左右手状态管理：maintain为false，instant为false
  if (leftHandSmoother_) {
    leftHandSmoother_->reset();
  }
  if (rightHandSmoother_) {
    rightHandSmoother_->reset();
  }
  // 重置joyStickHandlerPtr_的各类状态，重置为与构造时完全一致的状态
  if (joyStickHandlerPtr_) {
    joyStickHandlerPtr_->reset();
  }
  // 重置增量控制模块的内部状态（包括控制模式、ruckig滤波状态、手臂移动检测状态等）
  if (incrementalController_) {
    incrementalController_->reset();
  }
  // 重置 grip 状态跟踪变量，避免系统重置后出现错误的上升沿检测
  lastLeftGripPressed_ = false;
  lastRightGripPressed_ = false;
  hasLeftHandPoseInChest_ = false;
  hasRightHandPoseInChest_ = false;
  hasLeftElbowPosInChest_ = false;
  hasRightElbowPosInChest_ = false;
  leftHandPosInChest_.setZero();
  rightHandPosInChest_.setZero();
  leftElbowPosInChest_.setZero();
  rightElbowPosInChest_.setZero();
  leftHandQuatInChest_.setIdentity();
  rightHandQuatInChest_.setIdentity();
  // 重置 grip 超时机制状态
  {
    std::lock_guard<std::mutex> lock(leftGripTimeMutex_);
    leftGripStartTime_ = ros::Time(0);
    leftGripTimeoutReached_.store(false);
  }
  {
    std::lock_guard<std::mutex> lock(rightGripTimeMutex_);
    rightGripStartTime_ = ros::Time(0);
    rightGripTimeoutReached_.store(false);
  }
  // 重置激活所有手臂控制模式的计数器，确保下次进入增量模式时可以重新执行激活逻辑
  activateAllArmCtrlModeCounter_ = 0;
  // 重置退出 mode 2 的计数器，确保下次退出时可以重新执行过渡逻辑
  exitMode2Counter_ = 0;
  // 重置进入 mode 2 时的位置重置计数器，确保下次进入时可以重新执行位置重置逻辑
  enterMode2ResetCounter_ = 0;
  // 重置IK求解结果
  {
    std::lock_guard<std::mutex> lock(ikResultMutex_);
    if (latestIkSolution_.size() != drakeJointStateSize_) {
      latestIkSolution_.resize(drakeJointStateSize_);
    }
    latestIkSolution_.setZero();
    hasValidIkSolution_ = false;
    if (ikLowerBodyJointCommand_.size() != 4) {
      ikLowerBodyJointCommand_.resize(4);
    }
    ikLowerBodyJointCommand_.setZero();
    if (ikUpperBodyJointCommand_.size() != 14) {
      ikUpperBodyJointCommand_.resize(14);
    }
    ikUpperBodyJointCommand_.setZero();
  }
  // 重置pose约束列表，使用默认手部位置初始化，确保进入增量模式时能正确初始化到默认位置
  latestPoseConstraintList_.resize(POSE_DATA_LIST_SIZE_PLUS, PoseData());
  Eigen::Quaterniond defaultHandQuat = Eigen::Quaterniond::Identity();
  // 使用默认手部位置初始化，与 WheelIncrementalControlModule 中的硬编码默认值保持一致
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].position = defaultLeftHandPosOnExit_;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].rotation_matrix = defaultHandQuat.toRotationMatrix();
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].position = defaultRightHandPosOnExit_;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].rotation_matrix = defaultHandQuat.toRotationMatrix();
  // 初始化 chest / shoulders（避免 IK shoulder/chest cost 拉到零）
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_CHEST].position = robotFixedWaistYawPos_ + chestDefaultOffset_;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_CHEST].rotation_matrix = Eigen::Matrix3d::Identity();
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_SHOULDER].position = robotLeftFixedShoulderPos_;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_SHOULDER].position = robotRightFixedShoulderPos_;
  latestChestPosBeforeOpt_ = robotFixedWaistYawPos_ + chestDefaultOffset_;
  latestChestPosAfterOpt_ = robotFixedWaistYawPos_ + chestDefaultOffset_;
  latestLeftShoulderPosBeforeOpt_ = robotLeftFixedShoulderPos_;
  latestLeftShoulderPosAfterOpt_ = robotLeftFixedShoulderPos_;
  latestRightShoulderPosBeforeOpt_ = robotRightFixedShoulderPos_;
  latestRightShoulderPosAfterOpt_ = robotRightFixedShoulderPos_;
  latestLeftElbowPosBeforeOpt_ = latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_ELBOW].position;
  latestLeftElbowPosAfterOpt_ = latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_ELBOW].position;
  latestRightElbowPosBeforeOpt_ = latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_ELBOW].position;
  latestRightElbowPosAfterOpt_ = latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_ELBOW].position;
  latestIncrementalResult_ = WheelIncrementalPoseResult();
  {
    std::lock_guard<std::mutex> lock(mode2EnterTimeMutex_);
    mode2EnterTime_ = ros::Time(0);
  }
  // 重置下肢平滑状态（与手臂同款机制保持一致，避免模式切换/复位时跳变）
  {
    std::lock_guard<std::mutex> jointLock(jointStateMutex_);
    if (armJointRuckigFilterPtr_ && q_.size() == 14) {
      armJointRuckigFilterPtr_->reset(q_);
    }
    if (lbJointRuckigFilterPtr_ && lb_q_.size() == 4) {
      lbJointRuckigFilterPtr_->reset(lb_q_);
    }
  }
  {
    std::lock_guard<std::mutex> lock(lbLegMoveTimeMutex_);
    lbLegMoveStartTime_ = ros::Time(0);
  }
  if (incrementalController_) {
    Eigen::Quaterniond defaultHandQuat = Eigen::Quaterniond::Identity();
    incrementalController_->setHandQuatSeeds(defaultHandQuat, defaultHandQuat, useIncrementalHandOrientation_);
  }

  // 重置服务调用去重状态：确保 reset 后允许重新下发 set mode 请求
  {
    std::lock_guard<std::mutex> lock(controlModeRequestMutex_);
    lastControlModeRequested_ = -1;
    lastControlModeRequestTime_ = ros::Time(0);
    lastControlModeRequestSuccess_ = false;
  }
  {
    std::lock_guard<std::mutex> lock(lbQuickModeRequestMutex_);
    lastLbQuickModeRequested_ = -1;
    lastLbQuickModeRequestTime_ = ros::Time(0);
    lastLbQuickModeRequestSuccess_ = false;
  }
}

void WheelQuest3IkIncrementalROS::initializeFilter(std::unique_ptr<ocs2::mobile_manipulator::KinemicLimitFilter>& filterPtr,
                                              int dimension,
                                              double dt,
                                              double velLimit,
                                              double accLimit,
                                              double jerkLimit,
                                              const std::string& filterName,
                                              const Eigen::VectorXd* initialState) {
  filterPtr = std::make_unique<ocs2::mobile_manipulator::KinemicLimitFilter>(dimension, dt);
  const Eigen::VectorXd velLimitVec = Eigen::VectorXd::Constant(dimension, velLimit);
  const Eigen::VectorXd accLimitVec = Eigen::VectorXd::Constant(dimension, accLimit);
  const Eigen::VectorXd jerkLimitVec = Eigen::VectorXd::Constant(dimension, jerkLimit);
  filterPtr->setFirstOrderDerivativeLimit(velLimitVec);
  filterPtr->setSecondOrderDerivativeLimit(accLimitVec);
  filterPtr->setThirdOrderDerivativeLimit(jerkLimitVec);

  // 如果提供了初始状态，则执行 reset
  if (initialState != nullptr && initialState->size() == dimension) {
    filterPtr->reset(*initialState);
  } else if (initialState == nullptr) {
    // 如果没有提供初始状态，使用零向量
    filterPtr->reset(Eigen::VectorXd::Zero(dimension));
  }

  // 表格化打印过滤器初始化信息
  static bool isFirstCall = true;
  if (isFirstCall) {
    ROS_INFO("[WheelQuest3IkIncrementalROS] Joint Filter Initialization Summary:");
    ROS_INFO("+-------------------------+----------+--------------+--------------+--------------+--------------+");
    ROS_INFO("| Filter Name             | Dimension| dt (s)       | Vel Limit    | Acc Limit    | Jerk Limit   |");
    ROS_INFO("+-------------------------+----------+--------------+--------------+--------------+--------------+");
    isFirstCall = false;
  }
  ROS_INFO("| %-23s | %8d | %12.6f | %12.6f | %12.6f | %12.6f |",
           filterName.empty() ? "Unknown" : filterName.c_str(),
           dimension,
           dt,
           velLimit,
           accLimit,
           jerkLimit);

  // 检查是否是最后一个过滤器（通过检查是否所有过滤器都已初始化）
  static int filterCount = 0;
  filterCount++;
  if (filterCount >= 2) {  // 总共有2个过滤器
    ROS_INFO("+-------------------------+----------+--------------+--------------+--------------+--------------+");
    filterCount = 0;  // 重置计数器，以便下次初始化时重新打印表头
    isFirstCall = true;
  }
}

void WheelQuest3IkIncrementalROS::chestPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  // 保存最新 chest position（用于胸部位置增量输入）
  {
    std::lock_guard<std::mutex> lock(chestPoseMutex_);
    // NOTES: 轮臂chest的y轴不可能移动，因此直接剔除掉输入的y，以免干扰
    latestChestPositionInRobot_ = Eigen::Vector3d(msg->pose.position.x, 0.0, msg->pose.position.z);
    chestRotationQuaternion_ = transformQuestPoseTORobotPose(msg->pose.orientation);
    hasChestPose_ = true;
  }
}

// ========================================================================================
// [AUTO-MOVED][stage 2] moved from WheelQuest3IkIncrementalROS.cpp
// Functions: updateSensorArmJointMeanFromSensorData, updateSensorArmJointFromSensorData, computeLeftEndEffectorFK,
// computeRightEndEffectorFK, computeLeftLink4FK, computeRightLink4FK, computeLeftLink6FK, computeRightLink6FK,
// computeLeftShoulderFK, computeRightShoulderFK
// ========================================================================================

void WheelQuest3IkIncrementalROS::updateSensorArmJointMeanFromSensorData() {
  const auto currentSensorData = getSensorData();
  if (!currentSensorData) {
    ROS_ERROR("[WheelQuest3IkIncrementalROS] currentSensorData is not valid");
    return;
  }

  if (currentSensorData->joint_data.joint_q.size() < numTotalJoints_) {
    ROS_ERROR("[WheelQuest3IkIncrementalROS] currentSensorData->joint_data.joint_q size is not valid");
    return;
  }

  Eigen::VectorXd qNew = Eigen::VectorXd::Zero(drakeJointStateSize_);
  for (int i = 0; i < drakeJointStateSize_; ++i) {
    qNew(i) = currentSensorData->joint_data.joint_q[sensorDataArmOffset_ + i];
  }

  if (drakeJointStateSize_ > 14) {
    for (int i = 0; i < drakeJointStateSize_; ++i) {
      qNew(i) = currentSensorData->joint_data.joint_q[i];
    }
  }

  static constexpr double kKeep = 0.92;
  static constexpr double kNew = 0.08;
  {
    std::lock_guard<std::mutex> jointLock(jointStateMutex_);
    filterJointDataForDrakeFK_ = kKeep * filterJointDataForDrakeFK_ + kNew * qNew;
  }
}

void WheelQuest3IkIncrementalROS::updateSensorArmJointFromSensorData() {
  const auto currentSensorData = getSensorData();
  if (!currentSensorData) {
    return;
  }

  if (currentSensorData->joint_data.joint_q.size() < numTotalJoints_) {
    ROS_ERROR("[WheelQuest3IkIncrementalROS] currentSensorData->joint_data.joint_q size is not valid");
    return;
  }

  {
    std::lock_guard<std::mutex> jointLock(jointStateMutex_);
    if (jointDataForDrakeFK_.size() != drakeJointStateSize_) {
      jointDataForDrakeFK_ = Eigen::VectorXd::Zero(drakeJointStateSize_);
    }

    for (int i = 0; i < drakeJointStateSize_; ++i) {
      jointDataForDrakeFK_(i) = currentSensorData->joint_data.joint_q[sensorDataArmOffset_ + i];
    }
    if (drakeJointStateSize_ > 14) {
      for (int i = 0; i < drakeJointStateSize_; ++i) {
        jointDataForDrakeFK_(i) = currentSensorData->joint_data.joint_q[i];
      }
    }
  }
}

void WheelQuest3IkIncrementalROS::updateFkCacheFromSensorData() {
  computeLeftEndEffectorFK(leftEndEffectorPosition_, leftEndEffectorQuat_);
  computeRightEndEffectorFK(rightEndEffectorPosition_, rightEndEffectorQuat_);
  computeLeftLink4FK(leftLink4Position_, leftLink4Quat_);
  computeRightLink4FK(rightLink4Position_, rightLink4Quat_);
  computeLeftLink6FK(leftLink6Position_, leftLink6Quat_);
  computeRightLink6FK(rightLink6Position_, rightLink6Quat_);
  computeWaistYawFK(latestWaistYawFkPos_);
  Eigen::Quaterniond qLeftShoulder, qRightShoulder;
  computeLeftShoulderFK(latestLeftShoulderFkPos_, qLeftShoulder);
  computeRightShoulderFK(latestRightShoulderFkPos_, qRightShoulder);
}

void WheelQuest3IkIncrementalROS::computeLeftEndEffectorFK(Eigen::Vector3d& pOut, Eigen::Quaterniond& qOut) {
  // 默认值
  pOut = Eigen::Vector3d::Zero();
  qOut = Eigen::Quaterniond::Identity();

  if (filterJointDataForDrakeFK_.size() != drakeJointStateSize_) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(oneStageIkMutex_);
    auto [l7Position, l7Quaternion] = oneStageIkEndEffectorPtr_->FK(filterJointDataForDrakeFK_, "zarm_l7_end_effector");
    pOut = l7Position;
    qOut = l7Quaternion;
  }
}

void WheelQuest3IkIncrementalROS::computeRightEndEffectorFK(Eigen::Vector3d& pOut, Eigen::Quaterniond& qOut) {
  // 默认值
  pOut = Eigen::Vector3d::Zero();
  qOut = Eigen::Quaterniond::Identity();

  // 使用均值滤波后的关节数据
  if (filterJointDataForDrakeFK_.size() != drakeJointStateSize_) {
    return;
  }

  Eigen::VectorXd armJoints = filterJointDataForDrakeFK_;
  {
    std::lock_guard<std::mutex> lock(oneStageIkMutex_);
    auto [r7Position, r7Quaternion] = oneStageIkEndEffectorPtr_->FK(armJoints, "zarm_r7_end_effector");
    pOut = r7Position;
    qOut = r7Quaternion;
  }
}

void WheelQuest3IkIncrementalROS::computeLeftLink4FK(Eigen::Vector3d& pOut, Eigen::Quaterniond& qOut) {
  // 默认值
  pOut = Eigen::Vector3d::Zero();
  qOut = Eigen::Quaterniond::Identity();

  // 使用均值滤波后的关节数据
  if (filterJointDataForDrakeFK_.size() != drakeJointStateSize_) {
    return;
  }

  Eigen::VectorXd armJoints = filterJointDataForDrakeFK_;
  {
    std::lock_guard<std::mutex> lock(oneStageIkMutex_);
    auto [l4Position, l4Quaternion] = oneStageIkEndEffectorPtr_->FK(armJoints, "zarm_l4_link");
    pOut = l4Position;
    qOut = l4Quaternion;
  }
}

void WheelQuest3IkIncrementalROS::computeRightLink4FK(Eigen::Vector3d& pOut, Eigen::Quaterniond& qOut) {
  // 默认值
  pOut = Eigen::Vector3d::Zero();
  qOut = Eigen::Quaterniond::Identity();

  // 使用均值滤波后的关节数据
  if (filterJointDataForDrakeFK_.size() != drakeJointStateSize_) {
    return;
  }

  Eigen::VectorXd armJoints = filterJointDataForDrakeFK_;
  {
    std::lock_guard<std::mutex> lock(oneStageIkMutex_);
    auto [r4Position, r4Quaternion] = oneStageIkEndEffectorPtr_->FK(armJoints, "zarm_r4_link");
    pOut = r4Position;
    qOut = r4Quaternion;
  }
}

void WheelQuest3IkIncrementalROS::computeLeftLink6FK(Eigen::Vector3d& pOut, Eigen::Quaterniond& qOut) {
  // 默认值
  pOut = Eigen::Vector3d::Zero();
  qOut = Eigen::Quaterniond::Identity();

  if (filterJointDataForDrakeFK_.size() != drakeJointStateSize_) {
    return;
  }

  Eigen::VectorXd armJoints = filterJointDataForDrakeFK_;
  {
    std::lock_guard<std::mutex> lock(oneStageIkMutex_);
    auto [l6Position, l6Quaternion] = oneStageIkEndEffectorPtr_->FK(armJoints, "zarm_l6_link");
    pOut = l6Position;
    qOut = l6Quaternion;
  }
}

void WheelQuest3IkIncrementalROS::computeRightLink6FK(Eigen::Vector3d& pOut, Eigen::Quaterniond& qOut) {
  // 默认值
  pOut = Eigen::Vector3d::Zero();
  qOut = Eigen::Quaterniond::Identity();

  // 使用均值滤波后的关节数据
  if (filterJointDataForDrakeFK_.size() != drakeJointStateSize_) {
    return;
  }

  Eigen::VectorXd armJoints = filterJointDataForDrakeFK_;
  {
    std::lock_guard<std::mutex> lock(oneStageIkMutex_);
    auto [r6Position, r6Quaternion] = oneStageIkEndEffectorPtr_->FK(armJoints, "zarm_r6_link");
    pOut = r6Position;
    qOut = r6Quaternion;
  }
}

void WheelQuest3IkIncrementalROS::computeWaistYawFK(Eigen::Vector3d& pOut) {
  // 默认值（FK 计算失败时的 fallback）
  pOut = robotFixedWaistYawPos_;

  if (filterJointDataForDrakeFK_.size() != drakeJointStateSize_) {
    hasLatestWaistYawFk_ = false;
    return;
  }

  {
    std::lock_guard<std::mutex> lock(oneStageIkMutex_);
    auto [waistYawPosition, waistYawQuaternion] =
        oneStageIkEndEffectorPtr_->FK(filterJointDataForDrakeFK_, "waist_yaw_joint_parent");
    (void)waistYawQuaternion;
    pOut = waistYawPosition;
    hasLatestWaistYawFk_ = true;
  }
}

void WheelQuest3IkIncrementalROS::computeLeftShoulderFK(Eigen::Vector3d& pOut, Eigen::Quaterniond& qOut) {
  // 默认值
  pOut = robotLeftFixedShoulderPos_;
  qOut = Eigen::Quaterniond::Identity();

  if (filterJointDataForDrakeFK_.size() != drakeJointStateSize_) {
    hasLatestLeftShoulderFk_ = false;
    return;
  }

  {
    std::lock_guard<std::mutex> lock(oneStageIkMutex_);
    auto [pos, quat] = oneStageIkEndEffectorPtr_->FK(filterJointDataForDrakeFK_, "zarm_l2_joint_parent");
    pOut = pos;
    qOut = quat;
    hasLatestLeftShoulderFk_ = true;
  }
}

void WheelQuest3IkIncrementalROS::computeRightShoulderFK(Eigen::Vector3d& pOut, Eigen::Quaterniond& qOut) {
  // 默认值
  pOut = robotRightFixedShoulderPos_;
  qOut = Eigen::Quaterniond::Identity();

  if (filterJointDataForDrakeFK_.size() != drakeJointStateSize_) {
    hasLatestRightShoulderFk_ = false;
    return;
  }

  {
    std::lock_guard<std::mutex> lock(oneStageIkMutex_);
    auto [pos, quat] = oneStageIkEndEffectorPtr_->FK(filterJointDataForDrakeFK_, "zarm_r2_joint_parent");
    pOut = pos;
    qOut = quat;
    hasLatestRightShoulderFk_ = true;
  }
}

// ========================================================================================
// [AUTO-MOVED][stage 3] moved from WheelQuest3IkIncrementalROS.cpp
// Functions: loadPointTrackIKSolverConfigFromJson, loadDrakeChestElbowHandWeightsFromJson,
// loadDrakeChestElbowHandBoundsFromJson, loadDrakeChestElbowHandPrevFilterConfigFromJson
// ========================================================================================

WheelPointTrackIKSolverConfig WheelQuest3IkIncrementalROS::loadPointTrackIKSolverConfigFromJson(
    const nlohmann::json& configJson) {
  ROS_INFO("==================================================================================");
  ROS_INFO("🔧 [WheelQuest3IkIncrementalROS] Loading WheelPointTrackIKSolverConfig from JSON configuration");
  ROS_INFO("==================================================================================");

  WheelPointTrackIKSolverConfig config;

  try {
    if (configJson.contains("point_track_ik_solver_config") && configJson["point_track_ik_solver_config"].is_object()) {
      const auto& ikConfig = configJson["point_track_ik_solver_config"];

      // Load WheelPointTrackIKSolverConfig specific fields
      if (ikConfig.contains("historyBufferSize")) {
        config.historyBufferSize = ikConfig["historyBufferSize"].get<int>();
        ROS_INFO("  ✅ historyBufferSize: %d", config.historyBufferSize);
      }
      if (ikConfig.contains("dynamicsDt")) {
        config.dynamicsDt = ikConfig["dynamicsDt"].get<double>();
        ROS_INFO("  ✅ dynamicsDt: %.6f", config.dynamicsDt);
      }


      if (ikConfig.contains("eeTrackingWeight")) {
        config.eeTrackingWeight = ikConfig["eeTrackingWeight"].get<double>();
        ROS_INFO("  ✅ eeTrackingWeight: %.2e", config.eeTrackingWeight);
      }

      if (ikConfig.contains("elbowTrackingWeight")) {
        config.elbowTrackingWeight = ikConfig["elbowTrackingWeight"].get<double>();
        ROS_INFO("  ✅ elbowTrackingWeight: %.2e", config.elbowTrackingWeight);
      }

      if (ikConfig.contains("link6TrackingWeight")) {
        config.link6TrackingWeight = ikConfig["link6TrackingWeight"].get<double>();
        ROS_INFO("  ✅ link6TrackingWeight: %.2e", config.link6TrackingWeight);
      }

      if (ikConfig.contains("virtualThumbTrackingWeight")) {
        config.virtualThumbTrackingWeight = ikConfig["virtualThumbTrackingWeight"].get<double>();
        ROS_INFO("  ✅ virtualThumbTrackingWeight: %.2e", config.virtualThumbTrackingWeight);
      }

      if (ikConfig.contains("shoulderTrackingWeight")) {
        config.shoulderTrackingWeight = ikConfig["shoulderTrackingWeight"].get<double>();
        ROS_INFO("  ✅ shoulderTrackingWeight: %.2e", config.shoulderTrackingWeight);
      }

      if (ikConfig.contains("chestTrackingWeight")) {
        config.chestTrackingWeight = ikConfig["chestTrackingWeight"].get<double>();
        ROS_INFO("  ✅ chestTrackingWeight: %.2e", config.chestTrackingWeight);
      }

      // Load IKSolverConfig base class fields
      if (ikConfig.contains("constraintTolerance")) {
        config.constraintTolerance = ikConfig["constraintTolerance"].get<double>();
        ROS_INFO("  ✅ constraintTolerance: %.2e", config.constraintTolerance);
      }

      if (ikConfig.contains("solverTolerance")) {
        config.solverTolerance = ikConfig["solverTolerance"].get<double>();
        ROS_INFO("  ✅ solverTolerance: %.2e", config.solverTolerance);
      }

      if (ikConfig.contains("maxIterations")) {
        config.maxIterations = ikConfig["maxIterations"].get<int>();
        ROS_INFO("  ✅ maxIterations: %d", config.maxIterations);
      }

      if (ikConfig.contains("controlArmIndex")) {
        int armIdxInt = ikConfig["controlArmIndex"].get<int>();
        if (armIdxInt == 0) {
          config.controlArmIndex = ArmIdx::LEFT;
        } else if (armIdxInt == 1) {
          config.controlArmIndex = ArmIdx::RIGHT;
        } else if (armIdxInt == 2) {
          config.controlArmIndex = ArmIdx::BOTH;
        } else {
          ROS_WARN("  ⚠️ Invalid controlArmIndex: %d, using default BOTH", armIdxInt);
          config.controlArmIndex = ArmIdx::BOTH;
        }
        ROS_INFO("  ✅ controlArmIndex: %d", armIdxInt);
      }

      if (ikConfig.contains("isWeldBaseLink")) {
        config.isWeldBaseLink = ikConfig["isWeldBaseLink"].get<bool>();
        ROS_INFO("  ✅ isWeldBaseLink: %s", config.isWeldBaseLink ? "true" : "false");
      }

      if (ikConfig.contains("useJointLimits")) {
        config.useJointLimits = ikConfig["useJointLimits"].get<bool>();
        ROS_INFO("  ✅ useJointLimits: %s", config.useJointLimits ? "true" : "false");
      }

      // Load joint smoothness weights (7 joints per arm, symmetric)
      if (ikConfig.contains("jointSmoothWeightDefault")) {
        config.jointSmoothWeightDefault = ikConfig["jointSmoothWeightDefault"].get<double>();
        ROS_INFO("  ✅ jointSmoothWeightDefault: %.2e", config.jointSmoothWeightDefault);
      }

      if (ikConfig.contains("jointSmoothWeight0")) {
        config.jointSmoothWeight0 = ikConfig["jointSmoothWeight0"].get<double>();
        ROS_INFO("  ✅ jointSmoothWeight0: %.2e", config.jointSmoothWeight0);
      }

      if (ikConfig.contains("jointSmoothWeight1")) {
        config.jointSmoothWeight1 = ikConfig["jointSmoothWeight1"].get<double>();
        ROS_INFO("  ✅ jointSmoothWeight1: %.2e", config.jointSmoothWeight1);
      }

      if (ikConfig.contains("jointSmoothWeight2")) {
        config.jointSmoothWeight2 = ikConfig["jointSmoothWeight2"].get<double>();
        ROS_INFO("  ✅ jointSmoothWeight2: %.2e", config.jointSmoothWeight2);
      }

      if (ikConfig.contains("jointSmoothWeight3")) {
        config.jointSmoothWeight3 = ikConfig["jointSmoothWeight3"].get<double>();
        ROS_INFO("  ✅ jointSmoothWeight3: %.2e", config.jointSmoothWeight3);
      }

      if (ikConfig.contains("jointSmoothWeight4")) {
        config.jointSmoothWeight4 = ikConfig["jointSmoothWeight4"].get<double>();
        ROS_INFO("  ✅ jointSmoothWeight4: %.2e", config.jointSmoothWeight4);
      }

      if (ikConfig.contains("jointSmoothWeight5")) {
        config.jointSmoothWeight5 = ikConfig["jointSmoothWeight5"].get<double>();
        ROS_INFO("  ✅ jointSmoothWeight5: %.2e", config.jointSmoothWeight5);
      }

      if (ikConfig.contains("jointSmoothWeight6")) {
        config.jointSmoothWeight6 = ikConfig["jointSmoothWeight6"].get<double>();
        ROS_INFO("  ✅ jointSmoothWeight6: %.2e", config.jointSmoothWeight6);
      }

      // Load waist joint smoothness weights
      if (ikConfig.contains("waistSmoothWeight0")) {
        config.waistSmoothWeight0 = ikConfig["waistSmoothWeight0"].get<double>();
        ROS_INFO("  ✅ waistSmoothWeight0: %.2e", config.waistSmoothWeight0);
      }

      if (ikConfig.contains("waistSmoothWeight1")) {
        config.waistSmoothWeight1 = ikConfig["waistSmoothWeight1"].get<double>();
        ROS_INFO("  ✅ waistSmoothWeight1: %.2e", config.waistSmoothWeight1);
      }

      if (ikConfig.contains("waistSmoothWeight2")) {
        config.waistSmoothWeight2 = ikConfig["waistSmoothWeight2"].get<double>();
        ROS_INFO("  ✅ waistSmoothWeight2: %.2e", config.waistSmoothWeight2);
      }

      if (ikConfig.contains("waistSmoothWeight3")) {
        config.waistSmoothWeight3 = ikConfig["waistSmoothWeight3"].get<double>();
        ROS_INFO("  ✅ waistSmoothWeight3: %.2e", config.waistSmoothWeight3);
      }

      ROS_INFO("✅ [WheelQuest3IkIncrementalROS] Successfully loaded WheelPointTrackIKSolverConfig from JSON");
    } else {
      ROS_WARN("❌ [WheelQuest3IkIncrementalROS] 'point_track_ik_solver_config' not found in JSON config, using defaults");
    }

  } catch (const std::exception& e) {
    ROS_ERROR("❌ [WheelQuest3IkIncrementalROS] Exception while loading WheelPointTrackIKSolverConfig: %s", e.what());
    ROS_WARN("🔄 [WheelQuest3IkIncrementalROS] Falling back to default configuration");
  }

  return config;
}

DrakeChestElbowHandWeightConfig WheelQuest3IkIncrementalROS::loadDrakeChestElbowHandWeightsFromJson(
    const nlohmann::json& configJson) {
  DrakeChestElbowHandWeightConfig config;
  config.name = "JSON_Loaded";

  // Default (more conservative for chest) to avoid drifting under single-hand motion.
  config.q0 = Eigen::Vector3d::Ones() * 20.0;
  config.qv0 = Eigen::Vector3d::Ones() * 2.0;
  config.wq0 = Eigen::Vector3d::Ones() * 0.2;
  config.wqv0 = Eigen::Vector3d::Ones() * 0.1;
  config.q1 = 0.8;
  config.q2 = 1.0;
  config.qv1 = 0.1;
  config.qv2 = 0.1;
  config.qa0 = 5.0e-3;
  config.qa1 = 5.0e-3;
  config.qa2 = 5.0e-3;
  config.accUseFixedDt = false;
  config.accFixedDtSec = 0.01;

  try {
    if (configJson.contains("drake_chest_elbow_hand_weights") &&
        configJson["drake_chest_elbow_hand_weights"].is_object()) {
      const auto& w = configJson["drake_chest_elbow_hand_weights"];
      const auto loadVector3d = [](const nlohmann::json& value, const Eigen::Vector3d& fallback) -> Eigen::Vector3d {
        if (value.is_array() && value.size() == 3) {
          return Eigen::Vector3d(value[0].get<double>(), value[1].get<double>(), value[2].get<double>());
        }
        if (value.is_number()) {
          return Eigen::Vector3d::Ones() * value.get<double>();
        }
        return fallback;
      };
      if (w.contains("q0_xyz")) config.q0 = loadVector3d(w["q0_xyz"], config.q0);
      if (w.contains("qv0_xyz")) config.qv0 = loadVector3d(w["qv0_xyz"], config.qv0);
      if (w.contains("q0")) config.q0 = loadVector3d(w["q0"], config.q0);
      if (w.contains("qv0")) config.qv0 = loadVector3d(w["qv0"], config.qv0);
      if (w.contains("wq0_xyz")) config.wq0 = loadVector3d(w["wq0_xyz"], config.wq0);
      if (w.contains("wqv0_xyz")) config.wqv0 = loadVector3d(w["wqv0_xyz"], config.wqv0);
      if (w.contains("wq0")) config.wq0 = loadVector3d(w["wq0"], config.wq0);
      if (w.contains("wqv0")) config.wqv0 = loadVector3d(w["wqv0"], config.wqv0);
      if (w.contains("q1")) config.q1 = w["q1"].get<double>();
      if (w.contains("q2")) config.q2 = w["q2"].get<double>();
      if (w.contains("qv1")) config.qv1 = w["qv1"].get<double>();
      if (w.contains("qv2")) config.qv2 = w["qv2"].get<double>();
      if (w.contains("qa0")) config.qa0 = w["qa0"].get<double>();
      if (w.contains("qa1")) config.qa1 = w["qa1"].get<double>();
      if (w.contains("qa2")) config.qa2 = w["qa2"].get<double>();
      if (w.contains("acc_use_fixed_dt")) config.accUseFixedDt = w["acc_use_fixed_dt"].get<bool>();
      if (w.contains("acc_fixed_dt_sec")) config.accFixedDtSec = w["acc_fixed_dt_sec"].get<double>();
    }
  } catch (const std::exception& e) {
    ROS_ERROR("[WheelQuest3IkIncrementalROS] loadDrakeChestElbowHandWeightsFromJson exception: %s", e.what());
  }
  // print details
  ROS_INFO(
      "[WheelQuest3IkIncrementalROS] DrakeChestElbowHandWeightConfig loaded: q0=[%.3f, %.3f, %.3f], "
      "qv0=[%.3f, %.3f, %.3f], wq0=[%.3f, %.3f, %.3f], wqv0=[%.3f, %.3f, %.3f], "
      "q1=%.3f, q2=%.3f, qv1=%.3f, qv2=%.3f, qa0=%.6f, qa1=%.6f, qa2=%.6f, acc_use_fixed_dt=%s, acc_fixed_dt_sec=%.6f",
      config.q0.x(),
      config.q0.y(),
      config.q0.z(),
      config.qv0.x(),
      config.qv0.y(),
      config.qv0.z(),
      config.wq0.x(),
      config.wq0.y(),
      config.wq0.z(),
      config.wqv0.x(),
      config.wqv0.y(),
      config.wqv0.z(),
      config.q1,
      config.q2,
      config.qv1,
      config.qv2,
      config.qa0,
      config.qa1,
      config.qa2,
      config.accUseFixedDt ? "true" : "false",
      config.accFixedDtSec);
  return config;
}

DrakeChestElbowHandBoundsConfig WheelQuest3IkIncrementalROS::loadDrakeChestElbowHandBoundsFromJson(
    const nlohmann::json& configJson) {
  DrakeChestElbowHandBoundsConfig config;

  // Default bounds: keep hands within the existing box bounds; chest unconstrained (large bounds).
  config.leftHandLb = boxMinBound_;
  config.leftHandUb = boxMaxBound_;
  config.rightHandLb = boxMinBound_;
  config.rightHandUb = boxMaxBound_;

  try {
    if (configJson.contains("drake_chest_elbow_hand_bounds") &&
        configJson["drake_chest_elbow_hand_bounds"].is_object()) {
      const auto& b = configJson["drake_chest_elbow_hand_bounds"];
      auto readVec3 = [&](const char* key, Eigen::Vector3d& out) {
        if (!b.contains(key)) return;
        const auto& arr = b[key];
        if (arr.is_array() && arr.size() == 3) {
          out = Eigen::Vector3d(arr[0].get<double>(), arr[1].get<double>(), arr[2].get<double>());
        }
      };
      readVec3("chest_lower_bound", config.chestLb);
      readVec3("chest_upper_bound", config.chestUb);
      readVec3("left_hand_lower_bound", config.leftHandLb);
      readVec3("left_hand_upper_bound", config.leftHandUb);
      readVec3("right_hand_lower_bound", config.rightHandLb);
      readVec3("right_hand_upper_bound", config.rightHandUb);

      if (b.contains("yaw_lower")) config.yawLb = b["yaw_lower"].get<double>();
      if (b.contains("yaw_upper")) config.yawUb = b["yaw_upper"].get<double>();
      if (b.contains("pitch_lower")) config.pitchLb = b["pitch_lower"].get<double>();
      if (b.contains("pitch_upper")) config.pitchUb = b["pitch_upper"].get<double>();

      // Configurable constraint parameters (previously hard-coded)
      if (b.contains("min_p2_xy_norm")) config.minP2XyNorm = b["min_p2_xy_norm"].get<double>();
      if (b.contains("p2_xy_norm_y_weight")) config.p2XyNormYWeight = b["p2_xy_norm_y_weight"].get<double>();
      if (b.contains("min_p1_xy_norm")) config.minP1XyNorm = b["min_p1_xy_norm"].get<double>();
      if (b.contains("p1_xy_norm_y_weight")) config.p1XyNormYWeight = b["p1_xy_norm_y_weight"].get<double>();
    }
  } catch (const std::exception& e) {
    ROS_ERROR("[WheelQuest3IkIncrementalROS] loadDrakeChestElbowHandBoundsFromJson exception: %s", e.what());
  }

  return config;
}

DrakeChestElbowHandPrevFilterConfig WheelQuest3IkIncrementalROS::loadDrakeChestElbowHandPrevFilterConfigFromJson(
    const nlohmann::json& configJson) {
  DrakeChestElbowHandPrevFilterConfig config;

  try {
    if (configJson.contains("drake_chest_elbow_hand_prev_filter") &&
        configJson["drake_chest_elbow_hand_prev_filter"].is_object()) {
      const auto& f = configJson["drake_chest_elbow_hand_prev_filter"];
      if (f.contains("p_chest")) config.pChest = f["p_chest"].get<double>();
      if (f.contains("u1_left")) config.u1Left = f["u1_left"].get<double>();
      if (f.contains("u2_left")) config.u2Left = f["u2_left"].get<double>();
      if (f.contains("u1_right")) config.u1Right = f["u1_right"].get<double>();
      if (f.contains("u2_right")) config.u2Right = f["u2_right"].get<double>();
      if (f.contains("p_left_elbow")) config.pLeftElbow = f["p_left_elbow"].get<double>();
      if (f.contains("p_left_hand")) config.pLeftHand = f["p_left_hand"].get<double>();
      if (f.contains("p_right_elbow")) config.pRightElbow = f["p_right_elbow"].get<double>();
      if (f.contains("p_right_hand")) config.pRightHand = f["p_right_hand"].get<double>();
    }
  } catch (const std::exception& e) {
    ROS_ERROR("[WheelQuest3IkIncrementalROS] loadDrakeChestElbowHandPrevFilterConfigFromJson exception: %s", e.what());
  }

  ROS_INFO(
      "[WheelQuest3IkIncrementalROS] DrakeChestElbowHandPrevFilterConfig loaded: p_chest=%.3f, u1_left=%.3f, "
      "u2_left=%.3f, u1_right=%.3f, u2_right=%.3f, p_left_elbow=%.3f, p_left_hand=%.3f, p_right_elbow=%.3f, "
      "p_right_hand=%.3f",
      config.pChest,
      config.u1Left,
      config.u2Left,
      config.u1Right,
      config.u2Right,
      config.pLeftElbow,
      config.pLeftHand,
      config.pRightElbow,
      config.pRightHand);
  return config;
}

// ========================================================================================
// [AUTO-MOVED][stage 4] moved from WheelQuest3IkIncrementalROS.cpp
// Functions: publishJointStates
// ========================================================================================

void WheelQuest3IkIncrementalROS::publishAuxiliaryStates() {
  // 发布底盘速度控制命令
  if (joyStickHandlerPtr_ != nullptr && armControlMode_ == 2) {
    geometry_msgs::Twist cmdVelMsg;
    double leftX = joyStickHandlerPtr_->getLeftJoyStickY();
    double leftY = -joyStickHandlerPtr_->getLeftJoyStickX();
    double rightX = -joyStickHandlerPtr_->getRightJoyStickX();

    // 设置线速度：x方向向前，y方向向左
    cmdVelMsg.linear.x = leftX;
    cmdVelMsg.linear.y = leftY;
    cmdVelMsg.linear.z = 0.0;

    // 设置角速度：z轴为yaw（逆时针为正）
    cmdVelMsg.angular.x = 0.0;
    cmdVelMsg.angular.y = 0.0;
    cmdVelMsg.angular.z = rightX;

    if (abs(cmdVelMsg.linear.x) > 1e-2 || abs(cmdVelMsg.linear.y) > 1e-2 || abs(cmdVelMsg.angular.z) > 1e-2) {
      cmdVelPublisher_.publish(cmdVelMsg);
    }
  }

  Eigen::VectorXd armAngleLimited;
  {
    std::lock_guard<std::mutex> lock(ikResultMutex_);
    if (!hasValidIkSolution_ || (latestIkSolution_.size() != drakeJointStateSize_)) {
      latestIkSolution_ = Eigen::VectorXd::Zero(drakeJointStateSize_);
      ROS_WARN("Joint positions size (%zu) does not match expected size (%d)",
               latestIkSolution_.size(),
               drakeJointStateSize_);
      return;
    }
    armAngleLimited = latestIkSolution_;
  }

  // 使用armAngleLimited进行FK,获得左右手的ee_pose
  Eigen::Vector3d leftEePosition;
  Eigen::Quaterniond leftEeQuaternion;
  Eigen::Vector3d rightEePosition;
  Eigen::Quaterniond rightEeQuaternion;
  Eigen::Vector3d leftLink6Position;
  Eigen::Quaterniond leftLink6Quaternion;
  Eigen::Vector3d rightLink6Position;
  Eigen::Quaterniond rightLink6Quaternion;
  Eigen::Vector3d leftEePositionMeasured;
  Eigen::Quaterniond leftEeQuaternionMeasured;
  Eigen::Vector3d rightEePositionMeasured;
  Eigen::Quaterniond rightEeQuaternionMeasured;
  {
    std::lock_guard<std::mutex> lock(oneStageIkMutex_);
    // 计算左右手末端执行器FK
    const auto leftEeResult = oneStageIkEndEffectorPtr_->FK(armAngleLimited, "zarm_l7_end_effector");
    leftEePosition = leftEeResult.first;
    leftEeQuaternion = leftEeResult.second;
    const auto rightEeResult = oneStageIkEndEffectorPtr_->FK(armAngleLimited, "zarm_r7_end_effector");
    rightEePosition = rightEeResult.first;
    rightEeQuaternion = rightEeResult.second;
    // 计算左右手link6 FK
    const auto leftLink6Result = oneStageIkEndEffectorPtr_->FK(armAngleLimited, "zarm_l6_link");
    leftLink6Position = leftLink6Result.first;
    leftLink6Quaternion = leftLink6Result.second;
    const auto rightLink6Result = oneStageIkEndEffectorPtr_->FK(armAngleLimited, "zarm_r6_link");
    rightLink6Position = rightLink6Result.first;
    rightLink6Quaternion = rightLink6Result.second;
  }

  // 发布左手末端执行器pose
  geometry_msgs::PoseStamped leftEePoseMsg;
  leftEePoseMsg.header.stamp = ros::Time::now();
  leftEePoseMsg.header.frame_id = "base_link";  // 根据实际坐标系设置
  leftEePoseMsg.pose.position.x = leftEePosition.x();
  leftEePoseMsg.pose.position.y = leftEePosition.y();
  leftEePoseMsg.pose.position.z = leftEePosition.z();
  leftEePoseMsg.pose.orientation.w = leftEeQuaternion.w();
  leftEePoseMsg.pose.orientation.x = leftEeQuaternion.x();
  leftEePoseMsg.pose.orientation.y = leftEeQuaternion.y();
  leftEePoseMsg.pose.orientation.z = leftEeQuaternion.z();
  leftEePosePublisher_.publish(leftEePoseMsg);

  // 发布右手末端执行器pose
  geometry_msgs::PoseStamped rightEePoseMsg;
  rightEePoseMsg.header.stamp = ros::Time::now();
  rightEePoseMsg.header.frame_id = "base_link";  // 根据实际坐标系设置
  rightEePoseMsg.pose.position.x = rightEePosition.x();
  rightEePoseMsg.pose.position.y = rightEePosition.y();
  rightEePoseMsg.pose.position.z = rightEePosition.z();
  rightEePoseMsg.pose.orientation.w = rightEeQuaternion.w();
  rightEePoseMsg.pose.orientation.x = rightEeQuaternion.x();
  rightEePoseMsg.pose.orientation.y = rightEeQuaternion.y();
  rightEePoseMsg.pose.orientation.z = rightEeQuaternion.z();
  rightEePosePublisher_.publish(rightEePoseMsg);

  // 发布左手link6 pose
  geometry_msgs::PoseStamped leftLink6PoseMsg;
  leftLink6PoseMsg.header.stamp = ros::Time::now();
  leftLink6PoseMsg.header.frame_id = "base_link";  // 根据实际坐标系设置
  leftLink6PoseMsg.pose.position.x = leftLink6Position.x();
  leftLink6PoseMsg.pose.position.y = leftLink6Position.y();
  leftLink6PoseMsg.pose.position.z = leftLink6Position.z();
  leftLink6PoseMsg.pose.orientation.w = leftLink6Quaternion.w();
  leftLink6PoseMsg.pose.orientation.x = leftLink6Quaternion.x();
  leftLink6PoseMsg.pose.orientation.y = leftLink6Quaternion.y();
  leftLink6PoseMsg.pose.orientation.z = leftLink6Quaternion.z();
  leftLink6PosePublisher_.publish(leftLink6PoseMsg);

  // 发布右手link6 pose
  geometry_msgs::PoseStamped rightLink6PoseMsg;
  rightLink6PoseMsg.header.stamp = ros::Time::now();
  rightLink6PoseMsg.header.frame_id = "base_link";  // 根据实际坐标系设置
  rightLink6PoseMsg.pose.position.x = rightLink6Position.x();
  rightLink6PoseMsg.pose.position.y = rightLink6Position.y();
  rightLink6PoseMsg.pose.position.z = rightLink6Position.z();
  rightLink6PoseMsg.pose.orientation.w = rightLink6Quaternion.w();
  rightLink6PoseMsg.pose.orientation.x = rightLink6Quaternion.x();
  rightLink6PoseMsg.pose.orientation.y = rightLink6Quaternion.y();
  rightLink6PoseMsg.pose.orientation.z = rightLink6Quaternion.z();
  rightLink6PosePublisher_.publish(rightLink6PoseMsg);

  Eigen::VectorXd armJointsMeasured;
  {
    std::lock_guard<std::mutex> jointLock(jointStateMutex_);
    if (filterJointDataForDrakeFK_.size() == drakeJointStateSize_) {
      armJointsMeasured = filterJointDataForDrakeFK_;
    }
  }

  if (armJointsMeasured.size() != drakeJointStateSize_) {
    ROS_ERROR("[WheelQuest3IkIncrementalROS] filterJointDataForDrakeFK_ size is not valid");
  } else {
    {
      std::lock_guard<std::mutex> lock(oneStageIkMutex_);
      // 计算左手末端执行器FK（基于滤波后的关节数据）
      const auto leftEeMeasuredResult = oneStageIkEndEffectorPtr_->FK(armJointsMeasured, "zarm_l7_end_effector");
      leftEePositionMeasured = leftEeMeasuredResult.first;
      leftEeQuaternionMeasured = leftEeMeasuredResult.second;
      // 计算右手末端执行器FK（基于滤波后的关节数据）
      const auto rightEeMeasuredResult = oneStageIkEndEffectorPtr_->FK(armJointsMeasured, "zarm_r7_end_effector");
      rightEePositionMeasured = rightEeMeasuredResult.first;
      rightEeQuaternionMeasured = rightEeMeasuredResult.second;
    }

    // 发布左手末端执行器measured pose
    geometry_msgs::PoseStamped leftEePoseMeasuredMsg;
    leftEePoseMeasuredMsg.header.stamp = ros::Time::now();
    leftEePoseMeasuredMsg.header.frame_id = "base_link";  // 根据实际坐标系设置
    leftEePoseMeasuredMsg.pose.position.x = leftEePositionMeasured.x();
    leftEePoseMeasuredMsg.pose.position.y = leftEePositionMeasured.y();
    leftEePoseMeasuredMsg.pose.position.z = leftEePositionMeasured.z();
    leftEePoseMeasuredMsg.pose.orientation.w = leftEeQuaternionMeasured.w();
    leftEePoseMeasuredMsg.pose.orientation.x = leftEeQuaternionMeasured.x();
    leftEePoseMeasuredMsg.pose.orientation.y = leftEeQuaternionMeasured.y();
    leftEePoseMeasuredMsg.pose.orientation.z = leftEeQuaternionMeasured.z();
    leftEePoseMeasuredPublisher_.publish(leftEePoseMeasuredMsg);

    // 发布右手末端执行器measured pose
    geometry_msgs::PoseStamped rightEePoseMeasuredMsg;
    rightEePoseMeasuredMsg.header.stamp = ros::Time::now();
    rightEePoseMeasuredMsg.header.frame_id = "base_link";  // 根据实际坐标系设置
    rightEePoseMeasuredMsg.pose.position.x = rightEePositionMeasured.x();
    rightEePoseMeasuredMsg.pose.position.y = rightEePositionMeasured.y();
    rightEePoseMeasuredMsg.pose.position.z = rightEePositionMeasured.z();
    rightEePoseMeasuredMsg.pose.orientation.w = rightEeQuaternionMeasured.w();
    rightEePoseMeasuredMsg.pose.orientation.x = rightEeQuaternionMeasured.x();
    rightEePoseMeasuredMsg.pose.orientation.y = rightEeQuaternionMeasured.y();
    rightEePoseMeasuredMsg.pose.orientation.z = rightEeQuaternionMeasured.z();
    rightEePoseMeasuredPublisher_.publish(rightEePoseMeasuredMsg);
  }
}

void WheelQuest3IkIncrementalROS::publishJointStates() {
  Eigen::VectorXd finalArmAngles;
  Eigen::VectorXd finalLbAngles;
  {
    std::lock_guard<std::mutex> lock(ikResultMutex_);
    if (!hasValidIkSolution_ || (latestIkSolution_.size() != drakeJointStateSize_)) {
      latestIkSolution_ = Eigen::VectorXd::Zero(drakeJointStateSize_);
      ROS_WARN_THROTTLE(2.0,
                        "[WheelQuest3IkIncrementalROS::publishJointStates] Cannot publish: hasValidIkSolution_=%s, "
                        "latestIkSolution_.size()=%zu, drakeJointStateSize_=%d",
                        hasValidIkSolution_ ? "true" : "false",
                        latestIkSolution_.size(),
                        drakeJointStateSize_);
      return;
    }
    finalArmAngles = ikUpperBodyJointCommand_;
    finalLbAngles = ikLowerBodyJointCommand_;
  }

  sensor_msgs::JointState armJintStateMsg;
  {
    std::lock_guard<std::mutex> jointLock(jointStateMutex_);
    if (armJointRuckigFilterPtr_) {
      const Eigen::VectorXd filteredArmQ = armJointRuckigFilterPtr_->update(finalArmAngles);
      if (filteredArmQ.size() == q_.size()) {
        q_ = filteredArmQ;
        dq_ = armJointRuckigFilterPtr_->getFirstOrderDerivative();
      } else {
        ROS_WARN_THROTTLE(1.0, "[WheelQuest3IkIncrementalROS] Arm ruckig output size mismatch, fallback to target");
        q_ = finalArmAngles;
        dq_.setZero();
      }
    } else {
      q_ = finalArmAngles;
      dq_.setZero();
    }
    for (int i = 0; i < 14; ++i) {
      dq_(i) = dq_(i) > 18.0 ? 18.0 : dq_(i);
      dq_(i) = dq_(i) < -18.0 ? -18.0 : dq_(i);
    }

    const bool leftGripPressed = joyStickHandlerPtr_ ? joyStickHandlerPtr_->isLeftGrip() : false;
    const bool rightGripPressed = joyStickHandlerPtr_ ? joyStickHandlerPtr_->isRightGrip() : false;
    if (leftGripPressed) {
      ros::Time currentTime = ros::Time::now();
      ros::Time startTime;
      bool leftArmMoved = incrementalController_->hasLeftArmMoved();

      {
        std::lock_guard<std::mutex> lock(leftGripTimeMutex_);
        startTime = leftGripStartTime_;
      }

      double alpha = 0.01;  // 默认 alpha 值（未超时时的平滑系数）
      // 只有在检测到移动且已开始计时时才计算 alpha
      if (leftArmMoved && !startTime.isZero()) {
        double elapsedTime = (currentTime - startTime).toSec();
        if (elapsedTime > 0.0) {
          // 根据超时进度计算 alpha：从 0.01 逐步增大到 1.0
          double progress = std::min(elapsedTime / GRIP_TIMEOUT_DURATION, 1.0);
          alpha = 0.01 + (1.0 - 0.01) * std::sin(progress * M_PI / 2.0);  // sin插值：0.01 -> 1.0
        }
      }

      latest_q_.head(7) = (1.0 - alpha) * latest_q_.head(7) + alpha * q_.head(7);
      latest_dq_.head(7) = (1.0 - alpha) * latest_dq_.head(7) + alpha * dq_.head(7);
      lowpass_dq_.head(7) = (1.0 - lowpassDqAlpha_) * lowpass_dq_.head(7) + lowpassDqAlpha_ * latest_dq_.head(7);
    }

    if (rightGripPressed) {
      ros::Time currentTime = ros::Time::now();
      ros::Time startTime;
      bool rightArmMoved = incrementalController_->hasRightArmMoved();

      {
        std::lock_guard<std::mutex> lock(rightGripTimeMutex_);
        startTime = rightGripStartTime_;
      }

      double alpha = 0.01;  // 默认 alpha 值（未超时时的平滑系数）
      // 只有在检测到移动且已开始计时时才计算 alpha
      if (rightArmMoved && !startTime.isZero()) {
        double elapsedTime = (currentTime - startTime).toSec();
        if (elapsedTime > 0.0) {
          // 根据超时进度计算 alpha：从 0.01 逐步增大到 1.0
          double progress = std::min(elapsedTime / GRIP_TIMEOUT_DURATION, 1.0);
          alpha = 0.01 + (1.0 - 0.01) * std::sin(progress * M_PI / 2.0);  // sin插值：0.01 -> 1.0
        }
      }
      // tail 无需特殊处理，一直对应着右手的7个关节
      latest_q_.tail(7) = (1.0 - alpha) * latest_q_.tail(7) + alpha * q_.tail(7);
      latest_dq_.tail(7) = (1.0 - alpha) * latest_dq_.tail(7) + alpha * dq_.tail(7);
      lowpass_dq_.tail(7) = (1.0 - lowpassDqAlpha_) * lowpass_dq_.tail(7) + lowpassDqAlpha_ * latest_dq_.tail(7);
    }

    // arm are fixed at 14 joints
    armJintStateMsg.header.stamp = ros::Time::now();
    armJintStateMsg.position.resize(14);
    armJintStateMsg.velocity.resize(14);
    armJintStateMsg.effort.resize(14);
    armJintStateMsg.name.resize(14);

    // 使用局部变量保存本帧要发送的关节位置/速度，避免填充消息时读到被其他线程改写的 latest_q_/lowpass_dq_
    Eigen::VectorXd armPositionForPublish = latest_q_;
    Eigen::VectorXd armVelocityForPublish = lowpass_dq_;

    // 根据 mode2EnterTime_ 严格按时间区间分阶段处理，避免切入 mode2 初期关节指令突变：
    // 区间 1: [0, 0.3s)          — 传感器同步，速度清零
    // 区间 2: [0.3s, 5.0s)      — 从 q_init_cmd_ 线性平滑到 q_（若 q_init_cmd_ 有效），速度清零
    // 区间 3: [5.0s, +infty)    — 不在此处改写
    ros::Time mode2EnterTime;
    {
      std::lock_guard<std::mutex> lock(mode2EnterTimeMutex_);
      mode2EnterTime = mode2EnterTime_;
    }
    const bool inMode2 = (armControlMode_.load() == 2) && !mode2EnterTime.isZero();
    if (inMode2) {
      constexpr double kMode2SensorSyncDurationSec = 0.3;
      constexpr double kMode2SmoothDurationSec = 2.0;
      const double elapsed = (ros::Time::now() - mode2EnterTime).toSec();

      if (elapsed < kMode2SensorSyncDurationSec) {
        // 区间 1: [0, 0.3s)
        Eigen::VectorXd sensorArmQ = q_;
        if (jointDataForDrakeFK_.size() >= sensorDataArmOffset_ + 14) {
          sensorArmQ = jointDataForDrakeFK_.segment(sensorDataArmOffset_, 14);
        } else if (jointDataForDrakeFK_.size() >= 14) {
          sensorArmQ = jointDataForDrakeFK_.head(14);
        }
        q_init_cmd_ = sensorArmQ;
        armPositionForPublish = sensorArmQ;
        armVelocityForPublish.setZero();
        latest_q_ = sensorArmQ;
        latest_dq_.setZero();
        lowpass_dq_.setZero();
      } else if (elapsed < kMode2SmoothDurationSec) {
        // 区间 2: [0.3s, 5.0s)，严格按时间进入，平滑仅在 q_init_cmd_ 有效时生效
        if (q_init_cmd_.size() == 14) {
          const double alpha = std::min(
              std::max((elapsed - kMode2SensorSyncDurationSec) /
                           (kMode2SmoothDurationSec - kMode2SensorSyncDurationSec),
                       0.0),
              1.0);
          armPositionForPublish = (1.0 - alpha) * q_init_cmd_ + alpha * Eigen::VectorXd::Zero(14);
          latest_q_ = armPositionForPublish;
        }
        armVelocityForPublish.setZero();
        latest_dq_.setZero();
        lowpass_dq_.setZero();
      }
      // 区间 3: elapsed >= 5.0s 时不做处理，armPositionForPublish/armVelocityForPublish 保持本帧初的拷贝
    }

    for (int i = 0; i < 14; ++i) {
      armJintStateMsg.name[i] = "arm_joint_" + std::to_string(i + 1);
    }

    for (int i = 0; i < 14; ++i) {
      armJintStateMsg.position[i] = armPositionForPublish(i) * 180.0 / M_PI;
      armJintStateMsg.velocity[i] = armVelocityForPublish(i) * 180.0 / M_PI;
      armJintStateMsg.effort[i] = 0.0;
    }
  }

  kuavoArmTrajCppPublisher_.publish(armJintStateMsg);
  {
    std::lock_guard<std::mutex> lock(lbLegTargetMutex_);
    hasLatestLbTargetAngles_ = (finalLbAngles.size() == 4);
    if (hasLatestLbTargetAngles_) {
      latestLbTargetAngles_ = finalLbAngles;
    }
    lbLegTrajPublishEnabled_ = hasLatestLbTargetAngles_ && chestIncrementalUpdateEnabled_;
  }
}

void WheelQuest3IkIncrementalROS::publishDefaultJointStates() {
  // 定义默认目标角度（度转弧度）
  // 左臂和右臂的默认角度：[45, 0, 0, -90, 0, 0, 0] 度
  Eigen::VectorXd defaultArmAngles = Eigen::VectorXd::Zero(14);
  defaultArmAngles.head(7) << 45.0 * M_PI / 180.0, 0.0, 0.0, -90.0 * M_PI / 180.0, 0.0, 0.0, 0.0;
  defaultArmAngles.tail(7) << 45.0 * M_PI / 180.0, 0.0, 0.0, -90.0 * M_PI / 180.0, 0.0, 0.0, 0.0;

  // 发布手臂关节状态
  sensor_msgs::JointState armJintStateMsg;
  {
    std::lock_guard<std::mutex> jointLock(jointStateMutex_);

    // 直接设置 q_ 为默认角度
    q_ = defaultArmAngles;

    // dq_ 设置为零
    dq_.setZero();

    // 手臂 alpha 平滑：将 q_ 平滑到 latest_q_
    const double alpha = 0.01;
    latest_q_ = (1.0 - alpha) * latest_q_ + alpha * q_;

    // latest_dq_ 设置为零
    latest_dq_.setZero();

    lowpass_dq_ = lowpassDqAlpha_ * lowpass_dq_ + (1.0 - lowpassDqAlpha_) * latest_dq_;

    // 构建手臂消息
    armJintStateMsg.header.stamp = ros::Time::now();
    armJintStateMsg.position.resize(14);
    armJintStateMsg.velocity.resize(14);
    armJintStateMsg.effort.resize(14);
    armJintStateMsg.name.resize(14);

    for (int i = 0; i < 14; ++i) {
      armJintStateMsg.name[i] = "arm_joint_" + std::to_string(i + 1);
    }

    for (int i = 0; i < 14; ++i) {
      armJintStateMsg.position[i] = latest_q_(i) * 180.0 / M_PI;
      armJintStateMsg.velocity[i] = lowpass_dq_(i) * 180.0 / M_PI;
      armJintStateMsg.effort[i] = 0.0;
    }
  }

  // 发布手臂消息
  kuavoArmTrajCppPublisher_.publish(armJintStateMsg);

  // 注意：lb_leg 由 Timer 回调独立发布，这里不处理
}

void WheelQuest3IkIncrementalROS::publishLegJointStates() {
  // 从 latestLbTargetAngles_ 读取 IK 解的下肢目标角度
  Eigen::VectorXd targetLbAngles;
  bool shouldPublish = false;
  {
    std::lock_guard<std::mutex> lock(lbLegTargetMutex_);
    shouldPublish = lbLegTrajPublishEnabled_ && hasLatestLbTargetAngles_;
    if (shouldPublish) {
      targetLbAngles = latestLbTargetAngles_;
    }
  }

  if (!shouldPublish) {
    return;
  }

  sensor_msgs::JointState lbLegTrajMsg;
  {
    std::lock_guard<std::mutex> jointLock(jointStateMutex_);
    // 1) ruckig 限速度/加速度平滑
    if (lbJointRuckigFilterPtr_) {
      const Eigen::VectorXd filteredLbQ = lbJointRuckigFilterPtr_->update(targetLbAngles);
      if (filteredLbQ.size() == lb_q_.size()) {
        lb_q_ = filteredLbQ;
        lb_dq_ = lbJointRuckigFilterPtr_->getFirstOrderDerivative();
      } else {
        ROS_WARN_THROTTLE(1.0, "[publishLegJointStates] LB ruckig output size mismatch, fallback to target");
        lb_q_ = targetLbAngles;
        lb_dq_.setZero();
      }
    } else {
      lb_q_ = targetLbAngles;
      lb_dq_.setZero();
    }

    // 与手臂保持一致的防护（极端情况下限幅，单位：rad/s）
    for (int i = 0; i < 4; ++i) {
      lb_dq_(i) = std::clamp(lb_dq_(i), -18.0, 18.0);
    }

    // 2) 超时驱动 alpha 指数平滑：对齐 kuavo_arm_traj 的"从非常平滑 -> 快速收敛"行为
    const ros::Time currentTime = ros::Time::now();
    bool lbLegMoved = false;
    if (targetLbAngles.size() == lb_q_.size()) {
      lbLegMoved = (targetLbAngles - lb_q_).cwiseAbs().maxCoeff() > lbLegMoveThresholdRad_;
    }

    ros::Time startTime;
    {
      std::lock_guard<std::mutex> lock(lbLegMoveTimeMutex_);
      // 只有"发生明显变化"时才启动计时；变化结束则清空计时，回到非常平滑的 alpha=0.01
      if (lbLegMoved) {
        if (lbLegMoveStartTime_.isZero()) {
          lbLegMoveStartTime_ = currentTime;
        }
      } else {
        lbLegMoveStartTime_ = ros::Time(0);
      }
      startTime = lbLegMoveStartTime_;
    }

    double alpha = 0.01;
    if (lbLegMoved && !startTime.isZero()) {
      const double elapsedTime = (currentTime - startTime).toSec();
      if (elapsedTime > 0.0) {
        const double progress = std::min(elapsedTime / GRIP_TIMEOUT_DURATION, 1.0);
        alpha = 0.01 + (1.0 - 0.01) * std::sin(progress * M_PI / 2.0);
      }
    }

    latest_lb_q_ = (1.0 - alpha) * latest_lb_q_ + alpha * lb_q_;
    latest_lb_dq_ = (1.0 - alpha) * latest_lb_dq_ + alpha * lb_dq_;
    lowpass_lb_dq_ = lowpassDqAlpha_ * lowpass_lb_dq_ + (1.0 - lowpassDqAlpha_) * latest_lb_dq_;

    lbLegTrajMsg.header.stamp = ros::Time::now();
    lbLegTrajMsg.position.resize(4);
    lbLegTrajMsg.velocity.resize(4);
    lbLegTrajMsg.effort.resize(4);
    lbLegTrajMsg.name.resize(4);

    // 根据文档，关节名称设置为 ['joint1', 'joint2', 'joint3', 'joint4']
    // 对应 [knee_joint, leg_joint, waist_pitch_joint, waist_yaw_joint]
    lbLegTrajMsg.name[0] = "joint1";
    lbLegTrajMsg.name[1] = "joint2";
    lbLegTrajMsg.name[2] = "joint3";
    lbLegTrajMsg.name[3] = "joint4";

    latest_lb_q_(3) = std::clamp(latest_lb_q_(3), -M_PI / 4, M_PI / 4);
    for (int i = 0; i < 4; ++i) {
      lbLegTrajMsg.position[i] = latest_lb_q_(i) * 180.0 / M_PI;
      lbLegTrajMsg.velocity[i] = lowpass_lb_dq_(i) * 180.0 / M_PI;
      lbLegTrajMsg.effort[i] = 0.0;
    }
  }

  lbLegTrajPublisher_.publish(lbLegTrajMsg);
}

void WheelQuest3IkIncrementalROS::publishDefaultLegJointStates() {
  // 下肢默认角度：[0, 0, 0, 0] 度
  Eigen::VectorXd defaultLbAngles = Eigen::VectorXd::Zero(4);

  sensor_msgs::JointState lbLegTrajMsg;
  {
    std::lock_guard<std::mutex> jointLock(jointStateMutex_);

    // 直接设置 lb_q_ 为默认角度
    lb_q_ = defaultLbAngles;

    // lb_dq_ 设置为零
    lb_dq_.setZero();

    // alpha 平滑：将 lb_q_ 平滑到 latest_lb_q_
    const double alpha = 0.001;
    latest_lb_q_ = (1.0 - alpha) * latest_lb_q_ + alpha * lb_q_;

    // latest_lb_dq_ 设置为零
    latest_lb_dq_.setZero();

    lowpass_lb_dq_ = lowpassDqAlpha_ * lowpass_lb_dq_ + (1.0 - lowpassDqAlpha_) * latest_lb_dq_;

    lbLegTrajMsg.header.stamp = ros::Time::now();
    lbLegTrajMsg.position.resize(4);
    lbLegTrajMsg.velocity.resize(4);
    lbLegTrajMsg.effort.resize(4);
    lbLegTrajMsg.name.resize(4);

    lbLegTrajMsg.name[0] = "joint1";
    lbLegTrajMsg.name[1] = "joint2";
    lbLegTrajMsg.name[2] = "joint3";
    lbLegTrajMsg.name[3] = "joint4";

    latest_lb_q_(3) = std::clamp(latest_lb_q_(3), -M_PI / 4, M_PI / 4);
    for (int i = 0; i < 4; ++i) {
      lbLegTrajMsg.position[i] = latest_lb_q_(i) * 180.0 / M_PI;
      lbLegTrajMsg.velocity[i] = lowpass_lb_dq_(i) * 180.0 / M_PI;
      lbLegTrajMsg.effort[i] = 0.0;
    }
  }

  lbLegTrajPublisher_.publish(lbLegTrajMsg);
}
// ========================================================================================
// [AUTO-MOVED][stage 5] moved from WheelQuest3IkIncrementalROS.cpp
// Functions: initialize
// ========================================================================================

void WheelQuest3IkIncrementalROS::initialize(const nlohmann::json& configJson) {
  initializeBase(configJson);

  // 初始化胸部pose订阅器
  chestPoseSubscriber_ = nodeHandle_.subscribe(
      "/robot_chest_pose", 10, &WheelQuest3IkIncrementalROS::chestPoseCallback, this, ros::TransportHints().tcpNoDelay());
  ROS_INFO("[WheelQuest3IkIncrementalROS] Subscribed to /robot_chest_pose");

  latestPoseConstraintList_.resize(POSE_DATA_LIST_SIZE_PLUS, PoseData());

  // 从JSON配置读取lowpass_dq_滤波因子
  if (configJson.contains("lowpass_dq_filter")) {
    const auto& filterConfig = configJson["lowpass_dq_filter"];
    if (filterConfig.is_object()) {
      if (filterConfig.contains("alpha")) {
        lowpassDqAlpha_ = filterConfig["alpha"].get<double>();
        ROS_INFO("✅ [WheelQuest3IkIncrementalROS] Set lowpass_dq_ alpha from JSON: %.3f (beta=%.3f)",
                 lowpassDqAlpha_,
                 1.0 - lowpassDqAlpha_);
      }
    } else {
      ROS_WARN("❌ [WheelQuest3IkIncrementalROS] 'lowpass_dq_filter' is not an object, using default value (alpha=%.3f)",
               lowpassDqAlpha_);
    }
  } else {
    ROS_INFO(
        "ℹ️  [WheelQuest3IkIncrementalROS] 'lowpass_dq_filter' not found in JSON config, using default value "
        "(alpha=%.3f)",
        lowpassDqAlpha_);
  }

  if (configJson.contains("joint_state_publish_rate_hz")) {
    jointStatePublishRateHz_ = configJson["joint_state_publish_rate_hz"].get<double>();
    if (jointStatePublishRateHz_ <= 0.0) {
      ROS_WARN("❌ [WheelQuest3IkIncrementalROS] 'joint_state_publish_rate_hz' is invalid (%.3f), using default %.1f",
               jointStatePublishRateHz_,
               DEFAULT_JOINT_STATE_PUBLISH_RATE_HZ);
      jointStatePublishRateHz_ = DEFAULT_JOINT_STATE_PUBLISH_RATE_HZ;
    } else {
      ROS_INFO("✅ [WheelQuest3IkIncrementalROS] joint_state_publish_rate_hz: %.1f", jointStatePublishRateHz_);
    }
  } else {
    ROS_INFO("ℹ️  [WheelQuest3IkIncrementalROS] 'joint_state_publish_rate_hz' not found, using default %.1f",
             jointStatePublishRateHz_);
  }

  if (configJson.contains("lb_leg_publish_rate_multiplier")) {
    lbLegPublishRateMultiplier_ = configJson["lb_leg_publish_rate_multiplier"].get<double>();
    if (lbLegPublishRateMultiplier_ <= 0.0) {
      ROS_WARN("❌ [WheelQuest3IkIncrementalROS] 'lb_leg_publish_rate_multiplier' is invalid (%.3f), using default %.2f",
               lbLegPublishRateMultiplier_,
               DEFAULT_LB_LEG_PUBLISH_RATE_MULTIPLIER);
      lbLegPublishRateMultiplier_ = DEFAULT_LB_LEG_PUBLISH_RATE_MULTIPLIER;
    } else {
      ROS_INFO("✅ [WheelQuest3IkIncrementalROS] lb_leg_publish_rate_multiplier: %.3f", lbLegPublishRateMultiplier_);
    }
  } else {
    ROS_INFO("ℹ️  [WheelQuest3IkIncrementalROS] 'lb_leg_publish_rate_multiplier' not found, using default %.2f",
             lbLegPublishRateMultiplier_);
  }

  // TEST: 初始化关节限制中间值（硬编码，从URDF中提取）
  // 关节顺序：左臂7个(zarm_l1~l7) + 右臂7个(zarm_r1~r7) = 14个

  // 从JSON配置构建URDF路径
  std::string urdfFilePath;
  if (configJson.contains("arm_urdf")) {
    std::string kuavo_assets_path = ros::package::getPath("kuavo_assets");
    std::string arm_urdf_relative = configJson["arm_urdf"].get<std::string>();
    urdfFilePath = kuavo_assets_path + "/models/" + arm_urdf_relative;
    ROS_INFO("✅ [WheelQuest3IkIncrementalROS] Constructed URDF path from JSON: %s", urdfFilePath.c_str());
  } else {
    ROS_ERROR("❌ [WheelQuest3IkIncrementalROS] 'arm_urdf' field not found in JSON configuration");
    throw std::runtime_error("Missing 'arm_urdf' field in JSON configuration");
  }

  // drake initialization
  auto diagramBuilder = std::make_unique<drake::systems::DiagramBuilder<double>>();
  auto [plant, sceneGraph] = drake::multibody::AddMultibodyPlantSceneGraph(diagramBuilder.get(), 0.0);

  drake::multibody::Parser parser(&plant);
  parser.package_map().Add("kuavo_assets", ros::package::getPath("kuavo_assets"));
  auto modelInstance = parser.AddModelFromFile(urdfFilePath);

  const auto& baseFrame = plant.GetFrameByName("base_link");
  plant.WeldFrames(plant.world_frame(), baseFrame);  // Weld base_link to world frame

  mec_limit_lower_ = Eigen::VectorXd::Zero(plant.num_joints());
  mec_limit_upper_ = Eigen::VectorXd::Zero(plant.num_joints());

  // Table header
  std::cout << std::left << std::setw(10) << "Index" << std::setw(30) << "Joint Name" << std::setw(20) << "Lower Limit"
            << std::setw(20) << "Upper Limit" << std::endl;
  std::cout << std::string(80, '-') << std::endl;

  for (drake::multibody::JointIndex i(0); i < plant.num_joints(); ++i) {
    const auto& joint = plant.get_joint(i);
    if (joint.num_positions() > 0) {
      mec_limit_lower_(i) = joint.position_lower_limits()(0);
      mec_limit_upper_(i) = joint.position_upper_limits()(0);

      std::cout << std::left << std::setw(10) << i << std::setw(30) << joint.name() << std::fixed
                << std::setprecision(4) << std::setw(20) << mec_limit_lower_(i) << std::setw(20) << mec_limit_upper_(i)
                << std::endl;
    }
  }

  // Print updated table header
  std::cout << "\nUpdated Joint Limits:" << std::endl;
  std::cout << std::left << std::setw(10) << "Index" << std::setw(30) << "Joint Name" << std::setw(20) << "Lower Limit"
            << std::setw(20) << "Upper Limit" << std::endl;
  std::cout << std::string(80, '-') << std::endl;

  for (drake::multibody::JointIndex i(0); i < plant.num_joints(); ++i) {
    const auto& joint = plant.get_joint(i);
    if (joint.num_positions() > 0) {
      std::cout << std::left << std::setw(10) << i << std::setw(30) << joint.name() << std::fixed
                << std::setprecision(4) << std::setw(20) << joint.position_lower_limits()(0) << std::setw(20)
                << joint.position_upper_limits()(0) << std::endl;
    }
  }

  plant.Finalize();

  diagram_ = diagramBuilder->Build();
  diagramContext_ = diagram_->CreateDefaultContext();

  ROS_INFO("[WheelQuest3IkIncrementalROS] Drake model DOF: positions=%d, velocities=%d, states=%d",
           plant.num_positions(),
           plant.num_velocities(),
           plant.num_multibody_states());
  drakeJointStateSize_ = plant.num_positions();
  jointMidValues_.resize(drakeJointStateSize_);
  // 初始化 sensorData 双臂关节角（维度从 JSON 加载，rad）指数均值滤波状态
  filterJointDataForDrakeFK_ = Eigen::VectorXd::Zero(drakeJointStateSize_);
  jointDataForDrakeFK_ = Eigen::VectorXd::Zero(drakeJointStateSize_);
  // 初始化关节角度滤波状态
  q_ = Eigen::VectorXd::Zero(14);
  dq_ = Eigen::VectorXd::Zero(14);
  latest_q_ = Eigen::VectorXd::Zero(14);
  latest_dq_ = Eigen::VectorXd::Zero(14);
  lowpass_dq_ = Eigen::VectorXd::Zero(14);

  ikLowerBodyJointCommand_ = Eigen::VectorXd::Zero(4);
  ikUpperBodyJointCommand_ = Eigen::VectorXd::Zero(14);

  // 初始化下肢关节平滑状态（与手臂一致）
  lb_q_ = Eigen::VectorXd::Zero(4);
  lb_dq_ = Eigen::VectorXd::Zero(4);
  latest_lb_q_ = Eigen::VectorXd::Zero(4);
  latest_lb_dq_ = Eigen::VectorXd::Zero(4);
  lowpass_lb_dq_ = Eigen::VectorXd::Zero(4);
  {
    std::lock_guard<std::mutex> lock(lbLegMoveTimeMutex_);
    lbLegMoveStartTime_ = ros::Time(0);
  }

  std::vector<std::string> frameNames = loadFrameNamesFromConfig(configJson);
  auto pointTrackConfig = loadPointTrackIKSolverConfigFromJson(configJson);
  oneStageIkEndEffectorPtr_ =
      std::make_unique<HighlyDynamic::WheelOneStageIKEndEffector>(&plant, frameNames, pointTrackConfig);

  // 计算并保存 ArmJoint 为全零时的双手位姿，避免运行时频繁调用 FK
  Eigen::VectorXd armJoints = Eigen::VectorXd::Zero(drakeJointStateSize_);

  auto [joint2Position, joint2Quaternion] = oneStageIkEndEffectorPtr_->FK(armJoints, "zarm_l2_joint_parent");
  auto [rightJoint2Position, rightJoint2Quaternion] = oneStageIkEndEffectorPtr_->FK(armJoints, "zarm_r2_joint_parent");
  (void)rightJoint2Quaternion;
  auto [joint4Position, joint4Quaternion] = oneStageIkEndEffectorPtr_->FK(armJoints, "zarm_l4_joint_parent");
  auto [joint6Position, joint6Quaternion] = oneStageIkEndEffectorPtr_->FK(armJoints, "zarm_l6_joint_parent");
  auto [waistYawPosition, waistYawQuaternion] = oneStageIkEndEffectorPtr_->FK(armJoints, "waist_yaw_joint_parent");
  auto [waistPitchPosition, waistPitchQuaternion] =
      oneStageIkEndEffectorPtr_->FK(armJoints, "waist_pitch_joint_parent");
  robotFixedWaistYawPos_ = waistYawPosition;

  auto [baseLinkPosition, baseLinkQuaternion] = oneStageIkEndEffectorPtr_->FK(armJoints, "base_link");

  // print baselink position and quaternion
  ROS_INFO("✅ [WheelQuest3IkIncrementalROS] Base link position: [%.6f, %.6f, %.6f], quaternion: [%.6f, %.6f, %.6f, %.6f]",
           baseLinkPosition.x(),
           baseLinkPosition.y(),
           baseLinkPosition.z(),
           baseLinkQuaternion.w(),
           baseLinkQuaternion.x(),
           baseLinkQuaternion.y(),
           baseLinkQuaternion.z());

  //  print waistyaw position and quaternion
  ROS_INFO("✅ [WheelQuest3IkIncrementalROS] Waist yaw position: [%.6f, %.6f, %.6f], quaternion: [%.6f, %.6f, %.6f, %.6f]",
           waistYawPosition.x(),
           waistYawPosition.y(),
           waistYawPosition.z(),
           waistYawQuaternion.w(),
           waistYawQuaternion.x(),
           waistYawQuaternion.y(),
           waistYawQuaternion.z());
  // DEBUG print baseLinkPosition and baseLinkQuaternion

  // 计算link长度：l1 = ||joint2 - joint4||, l2 = ||joint4 - joint6||
  l1_ = (joint2Position - joint4Position).norm();
  l2_ = (joint4Position - joint6Position).norm();

  robotLeftFixedShoulderPos_ = joint2Position;
  robotRightFixedShoulderPos_ = rightJoint2Position;
  robotRightFixedShoulderPos_.y() = -robotLeftFixedShoulderPos_.y();

  ROS_INFO("✅ [WheelQuest3IkIncrementalROS] Geometry parameters calculated by FK (zero joint angles):");
  ROS_INFO("   l1 (||joint2-joint4||): %.6f", l1_);
  ROS_INFO("   l2 (||joint4-joint6||): %.6f", l2_);
  ROS_INFO("   Left shoulder (joint2): [%.6f, %.6f, %.6f]",
           robotLeftFixedShoulderPos_.x(),
           robotLeftFixedShoulderPos_.y(),
           robotLeftFixedShoulderPos_.z());
  ROS_INFO("   Right shoulder (joint2, mirrored): [%.6f, %.6f, %.6f]",
           robotRightFixedShoulderPos_.x(),
           robotRightFixedShoulderPos_.y(),
           robotRightFixedShoulderPos_.z());
  ROS_INFO("   Waist yaw joint parent: [%.6f, %.6f, %.6f], quaternion: [%.6f, %.6f, %.6f, %.6f]",
           waistYawPosition.x(),
           waistYawPosition.y(),
           waistYawPosition.z(),
           waistYawQuaternion.w(),
           waistYawQuaternion.x(),
           waistYawQuaternion.y(),
           waistYawQuaternion.z());
  ROS_INFO("   Waist pitch joint parent: [%.6f, %.6f, %.6f], quaternion: [%.6f, %.6f, %.6f, %.6f]",
           waistPitchPosition.x(),
           waistPitchPosition.y(),
           waistPitchPosition.z(),
           waistPitchQuaternion.w(),
           waistPitchQuaternion.x(),
           waistPitchQuaternion.y(),
           waistPitchQuaternion.z());

  // 从JSON配置读取胸部默认偏移量
  if (configJson.contains("chest_default_offset")) {
    const auto& offsetConfig = configJson["chest_default_offset"];
    if (offsetConfig.is_array() && offsetConfig.size() == 3) {
      chestDefaultOffset_ =
          Eigen::Vector3d(offsetConfig[0].get<double>(), offsetConfig[1].get<double>(), offsetConfig[2].get<double>());
      ROS_INFO("✅ [WheelQuest3IkIncrementalROS] Loaded chest_default_offset from JSON: [%.6f, %.6f, %.6f]",
               chestDefaultOffset_.x(),
               chestDefaultOffset_.y(),
               chestDefaultOffset_.z());
    } else {
      ROS_WARN(
          "❌ [WheelQuest3IkIncrementalROS] 'chest_default_offset' is not a valid 3-element array, using default [0.0, "
          "0.0, "
          "0.3]");
      chestDefaultOffset_ = Eigen::Vector3d(0.0, 0.0, 0.3);
    }
  } else {
    ROS_INFO(
        "ℹ️  [WheelQuest3IkIncrementalROS] 'chest_default_offset' not found in JSON, using default [0.0, 0.0, 0.3]");
    chestDefaultOffset_ = Eigen::Vector3d(0.0, 0.0, 0.3);
  }

  // 计算 Link6 位姿（用于 IK 约束）
  auto [leftLink6Position, leftLink6Quaternion] = oneStageIkEndEffectorPtr_->FK(armJoints, "zarm_l6_link");
  auto [rightLink6Position, rightLink6Quaternion] = oneStageIkEndEffectorPtr_->FK(armJoints, "zarm_r6_link");
  initZeroLeftLink6Position_ = leftLink6Position;
  initZeroRightLink6Position_ = rightLink6Position;

  // 计算 End Effector 位姿（用于可视化等）
  auto [leftEndEffectorPosition, leftEndEffectorQuaternion] =
      oneStageIkEndEffectorPtr_->FK(armJoints, "zarm_l7_end_effector");
  auto [rightEndEffectorPosition, rightEndEffectorQuaternion] =
      oneStageIkEndEffectorPtr_->FK(armJoints, "zarm_r7_end_effector");
  initZeroLeftEndEffectorPosition_ = leftEndEffectorPosition;
  initZeroRightEndEffectorPosition_ = rightEndEffectorPosition;

  // 使用FK计算的结果初始化四个点的位置（全零关节角度）
  leftLink6Position_ = leftLink6Position;
  rightLink6Position_ = rightLink6Position;
  leftEndEffectorPosition_ = leftEndEffectorPosition;
  rightEndEffectorPosition_ = rightEndEffectorPosition;

  // 初始化人肘参考（沿着ee->手腕方向，长度为l2_）
  {
    const Eigen::Vector3d leftEeToWrist = leftLink6Position_ - leftEndEffectorPosition_;
    const double leftNorm = leftEeToWrist.norm();
    if (leftNorm > 1e-6) {
      latestHumanLeftElbowPos_ = leftLink6Position_ + leftEeToWrist * (l2_ / leftNorm);
    } else {
      latestHumanLeftElbowPos_ = leftLink6Position_;
    }
    const Eigen::Vector3d rightEeToWrist = rightLink6Position_ - rightEndEffectorPosition_;
    const double rightNorm = rightEeToWrist.norm();
    if (rightNorm > 1e-6) {
      latestHumanRightElbowPos_ = rightLink6Position_ + rightEeToWrist * (l2_ / rightNorm);
    } else {
      latestHumanRightElbowPos_ = rightLink6Position_;
    }
  }

  ROS_INFO("✅ [WheelQuest3IkIncrementalROS] Initial positions calculated by FK (zero joint angles):");
  ROS_INFO("   Left Link6: [%.6f, %.6f, %.6f]", leftLink6Position_.x(), leftLink6Position_.y(), leftLink6Position_.z());
  ROS_INFO(
      "   Right Link6: [%.6f, %.6f, %.6f]", rightLink6Position_.x(), rightLink6Position_.y(), rightLink6Position_.z());
  ROS_INFO("   Left End Effector: [%.6f, %.6f, %.6f]",
           leftEndEffectorPosition_.x(),
           leftEndEffectorPosition_.y(),
           leftEndEffectorPosition_.z());
  ROS_INFO("   Right End Effector: [%.6f, %.6f, %.6f]",
           rightEndEffectorPosition_.x(),
           rightEndEffectorPosition_.y(),
           rightEndEffectorPosition_.z());

  leftVirtualThumbPosition_ << leftEndEffectorPosition_.x() + 0.15, leftEndEffectorPosition_.y(),
      leftEndEffectorPosition_.z();
  rightVirtualThumbPosition_ << rightEndEffectorPosition_.x() + 0.15, rightEndEffectorPosition_.y(),
      rightEndEffectorPosition_.z();

  leftEE2Link6Offset_ = leftEndEffectorPosition_ - leftLink6Position_;
  rightEE2Link6Offset_ = rightEndEffectorPosition_ - rightLink6Position_;
  leftThumb2Link6Offset_ = leftVirtualThumbPosition_ - leftLink6Position_;
  rightThumb2Link6Offset_ = rightVirtualThumbPosition_ - rightLink6Position_;
  ROS_INFO(
      "[WheelQuest3IkIncrementalROS] Initialized zero joint pose - Link6: left=[%.4f, %.4f, %.4f], right=[%.4f, %.4f, "
      "%.4f]",
      initZeroLeftLink6Position_.x(),
      initZeroLeftLink6Position_.y(),
      initZeroLeftLink6Position_.z(),
      initZeroRightLink6Position_.x(),
      initZeroRightLink6Position_.y(),
      initZeroRightLink6Position_.z());
  ROS_INFO(
      "[WheelQuest3IkIncrementalROS] Initialized zero joint pose - EndEffector: left=[%.4f, %.4f, %.4f], right=[%.4f, "
      "%.4f, "
      "%.4f]",
      initZeroLeftEndEffectorPosition_.x(),
      initZeroLeftEndEffectorPosition_.y(),
      initZeroLeftEndEffectorPosition_.z(),
      initZeroRightEndEffectorPosition_.x(),
      initZeroRightEndEffectorPosition_.y(),
      initZeroRightEndEffectorPosition_.z());

  kuavoArmTrajCppPublisher_ = nodeHandle_.advertise<sensor_msgs::JointState>("/kuavo_arm_traj_cpp", 2);
  sensorDataArmJointsPublisher_ = nodeHandle_.advertise<sensor_msgs::JointState>("/kuavo_arm_traj_sensor_data", 2);
  leftHandPosePublisher_ = nodeHandle_.advertise<geometry_msgs::Pose>("/left_hand_pose", 2);
  rightHandPosePublisher_ = nodeHandle_.advertise<geometry_msgs::Pose>("/right_hand_pose", 2);
  leftHandRotationMatrixPublisher_ =
      nodeHandle_.advertise<std_msgs::Float32MultiArray>("/left_hand_rotation_matrix", 2);
  rightHandRotationMatrixPublisher_ =
      nodeHandle_.advertise<std_msgs::Float32MultiArray>("/right_hand_rotation_matrix", 2);
  leftAnchorQuatPublisher_ = nodeHandle_.advertise<geometry_msgs::Quaternion>("/ik_debug/left_anchor_quat", 2);
  rightAnchorQuatPublisher_ = nodeHandle_.advertise<geometry_msgs::Quaternion>("/ik_debug/right_anchor_quat", 2);
  leftHandDeltaQuatPublisher_ = nodeHandle_.advertise<geometry_msgs::Quaternion>("/ik_debug/left_hand_delta_quat", 2);
  rightHandDeltaQuatPublisher_ = nodeHandle_.advertise<geometry_msgs::Quaternion>("/ik_debug/right_hand_delta_quat", 2);
  leftEePosePublisher_ = nodeHandle_.advertise<geometry_msgs::PoseStamped>("/ik_fk_result/left_ee_pose", 2);
  rightEePosePublisher_ = nodeHandle_.advertise<geometry_msgs::PoseStamped>("/ik_fk_result/right_ee_pose", 2);
  leftEePoseFilterPublisher_ =
      nodeHandle_.advertise<geometry_msgs::PoseStamped>("/ik_fk_result/left_ee_pose_filter", 2);
  rightEePoseFilterPublisher_ =
      nodeHandle_.advertise<geometry_msgs::PoseStamped>("/ik_fk_result/right_ee_pose_filter", 2);
  leftLink6PosePublisher_ = nodeHandle_.advertise<geometry_msgs::PoseStamped>("/ik_fk_result/left_link6_pose", 2);
  rightLink6PosePublisher_ = nodeHandle_.advertise<geometry_msgs::PoseStamped>("/ik_fk_result/right_link6_pose", 2);
  leftLink6PoseFilterPublisher_ =
      nodeHandle_.advertise<geometry_msgs::PoseStamped>("/ik_fk_result/left_link6_pose_filter", 2);
  rightLink6PoseFilterPublisher_ =
      nodeHandle_.advertise<geometry_msgs::PoseStamped>("/ik_fk_result/right_link6_pose_filter", 2);
  leftLink6PoseMeasuredPublisher_ =
      nodeHandle_.advertise<geometry_msgs::PoseStamped>("/ik_fk_result/left_link6_pose_measured", 2);
  rightLink6PoseMeasuredPublisher_ =
      nodeHandle_.advertise<geometry_msgs::PoseStamped>("/ik_fk_result/right_link6_pose_measured", 2);
  leftEePoseMeasuredPublisher_ =
      nodeHandle_.advertise<geometry_msgs::PoseStamped>("/ik_fk_result/left_ee_pose_measured", 2);
  rightEePoseMeasuredPublisher_ =
      nodeHandle_.advertise<geometry_msgs::PoseStamped>("/ik_fk_result/right_ee_pose_measured", 2);
  leftHandPoseFromTransformerPublisher_ =
      nodeHandle_.advertise<geometry_msgs::PoseStamped>("/ik_debug/left_hand_pose_from_transformer", 2);
  rightHandPoseFromTransformerPublisher_ =
      nodeHandle_.advertise<geometry_msgs::PoseStamped>("/ik_debug/right_hand_pose_from_transformer", 2);

  wholeBodyRefMarkerArrayPublisher_ =
      nodeHandle_.advertise<visualization_msgs::MarkerArray>("/ik_debug/whole_body_ref_markers", 2);

  lbLegTrajPublisher_ = nodeHandle_.advertise<sensor_msgs::JointState>("/lb_leg_traj", 10);
  lbLegTrajPublishTimer_ =
      nodeHandle_.createTimer(ros::Duration(lbLegPublishRateMultiplier_ / std::max(jointStatePublishRateHz_, 1.0)),
                              [this](const ros::TimerEvent&) {
                                // 根据 armControlMode_ 决定调用哪个发布函数
                                if (armControlMode_ == 2) {
                                  publishLegJointStates();  // mode 2: 使用 IK 解
                                } else {
                                  publishDefaultLegJointStates();  // 非 mode 2: 使用默认值
                                }
                              });

  cmdVelPublisher_ = nodeHandle_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  // 初始化轮臂快速模式服务客户端：调用 /enable_lb_arm_quick_mode 服务设置轮臂快速模式
  // 用法：通过 enableLbArmQuickModeClient_.call(srv) 调用服务
  // 服务名称：/enable_lb_arm_quick_mode
  // 服务类型：kuavo_msgs::changeLbQuickModeSrv
  enableLbArmQuickModeClient_ =
      nodeHandle_.serviceClient<kuavo_msgs::changeLbQuickModeSrv>("/enable_lb_arm_quick_mode");

  // 初始化移动机械臂控制模式服务客户端：调用 /mobile_manipulator_mpc_control
  // 服务设置控制模式（对应lb_ctrl_api.set_control_mode） 用法：通过 changeTorsoCtrlModeClient_.call(srv) 调用服务
  // 服务名称：/mobile_manipulator_mpc_control
  // 服务类型：kuavo_msgs::changeTorsoCtrlMode
  // 控制模式：0-NoControl, 1-ArmOnly, 2-BaseOnly, 3-BaseArm, 4-ArmEeOnly
  changeTorsoCtrlModeClient_ =
      nodeHandle_.serviceClient<kuavo_msgs::changeTorsoCtrlMode>("/mobile_manipulator_mpc_control");

  // 初始化增量控制模块
  IncrementalControlConfig incrementalConfig;
  // 位置/姿态增量滤波截止频率统一从 JSON 读取（Hz）
  if (configJson.contains("pos_cutoff_hz")) {
    incrementalConfig.posCutoffHz = configJson["pos_cutoff_hz"].get<double>();
  }
  if (!std::isfinite(incrementalConfig.posCutoffHz)) {
    ROS_WARN("[WheelQuest3IkIncrementalROS] Invalid pos_cutoff_hz from JSON, fallback to 10.0 Hz");
    incrementalConfig.posCutoffHz = 10.0;
  }
  ROS_INFO("[WheelQuest3IkIncrementalROS] Incremental LPF pos_cutoff_hz: %.3f Hz", incrementalConfig.posCutoffHz);

  if (configJson.contains("orientation_cutoff_hz")) {
    incrementalConfig.orientationCutoffHz = configJson["orientation_cutoff_hz"].get<double>();
  }
  if (!std::isfinite(incrementalConfig.orientationCutoffHz)) {
    ROS_WARN("[WheelQuest3IkIncrementalROS] Invalid orientation_cutoff_hz from JSON, fallback to 10.0 Hz");
    incrementalConfig.orientationCutoffHz = 10.0;
  }
  ROS_INFO("[WheelQuest3IkIncrementalROS] Incremental LPF orientation_cutoff_hz: %.3f Hz",
           incrementalConfig.orientationCutoffHz);

  while (!nodeHandle_.hasParam("/ik_ros_uni_cpp_node/quest3/delta_scale_x")) {
    ROS_WARN("[WheelQuest3IkIncrementalROS] Waiting for /quest3/delta_scale_x parameter");
    ros::Duration(0.1).sleep();
  }
  while (!nodeHandle_.hasParam("/ik_ros_uni_cpp_node/quest3/delta_scale_y")) {
    ROS_WARN("[WheelQuest3IkIncrementalROS] Waiting for /quest3/delta_scale_y parameter");
    ros::Duration(0.1).sleep();
  }
  while (!nodeHandle_.hasParam("/ik_ros_uni_cpp_node/quest3/delta_scale_z")) {
    ROS_WARN("[WheelQuest3IkIncrementalROS] Waiting for /quest3/delta_scale_z parameter");
    ros::Duration(0.1).sleep();
  }
  double delta_scale_x, delta_scale_y, delta_scale_z;
  PARAM_AND_PRINT_FLOAT(nodeHandle_, "/ik_ros_uni_cpp_node/quest3/delta_scale_x", delta_scale_x, 1.0, 3);
  PARAM_AND_PRINT_FLOAT(nodeHandle_, "/ik_ros_uni_cpp_node/quest3/delta_scale_y", delta_scale_y, 1.0, 3);
  PARAM_AND_PRINT_FLOAT(nodeHandle_, "/ik_ros_uni_cpp_node/quest3/delta_scale_z", delta_scale_z, 1.0, 3);
  deltaScale_ = Eigen::Vector3d(delta_scale_x, delta_scale_y, delta_scale_z);
  incrementalConfig.deltaScale = deltaScale_;

  double delta_scale_roll, delta_scale_pitch, delta_scale_yaw;
  PARAM_AND_PRINT_FLOAT(nodeHandle_, "/ik_ros_uni_cpp_node/quest3/delta_scale_roll", delta_scale_roll, 1.0, 3);
  PARAM_AND_PRINT_FLOAT(nodeHandle_, "/ik_ros_uni_cpp_node/quest3/delta_scale_pitch", delta_scale_pitch, 1.0, 3);
  PARAM_AND_PRINT_FLOAT(nodeHandle_, "/ik_ros_uni_cpp_node/quest3/delta_scale_yaw", delta_scale_yaw, 1.0, 3);
  deltaScaleRPY_ = Eigen::Vector3d(delta_scale_roll, delta_scale_pitch, delta_scale_yaw);

  PARAM_AND_PRINT_FLOAT(
      nodeHandle_, "/ik_ros_uni_cpp_node/quest3/pos_vel_limit", incrementalConfig.posVelLimit, 10.0, 2);

  // 读取 arm_move_threshold 参数
  while (!nodeHandle_.hasParam("/ik_ros_uni_cpp_node/quest3/arm_move_threshold")) {
    ROS_WARN("[WheelQuest3IkIncrementalROS] Waiting for /quest3/arm_move_threshold parameter");
    ros::Duration(0.1).sleep();
  }
  PARAM_AND_PRINT_FLOAT(
      nodeHandle_, "/ik_ros_uni_cpp_node/quest3/arm_move_threshold", incrementalConfig.armMoveThreshold, 0.01, 3);
  incrementalConfig.publishRate = publishRate_;

  // 读取关节角度平滑参数（用于 ruckig 约束）
  PARAM_AND_PRINT_FLOAT(
      nodeHandle_, "/ik_ros_uni_cpp_node/quest3/joint_space_acc_limit", jointSpaceAccLimit_, 100.0, 1);
  PARAM_AND_PRINT_FLOAT(
      nodeHandle_, "/ik_ros_uni_cpp_node/quest3/joint_space_jerk_limit", jointSpaceJerkLimit_, 600.0, 1);
  PARAM_AND_PRINT_FLOAT(nodeHandle_, "/ik_ros_uni_cpp_node/quest3/max_joint_velocity", maxJointVelocity_, 10.0, 3);
  {
    const double armRuckigDt = 1.0 / std::max(jointStatePublishRateHz_, 1.0);
    const double lbRuckigDt = lbLegPublishRateMultiplier_ / std::max(jointStatePublishRateHz_, 1.0);

    Eigen::VectorXd armInitialState = (q_.size() == 14) ? q_ : Eigen::VectorXd::Zero(14);
    Eigen::VectorXd lbInitialState = (lb_q_.size() == 4) ? lb_q_ : Eigen::VectorXd::Zero(4);

    initializeFilter(armJointRuckigFilterPtr_,
                     14,
                     armRuckigDt,
                     maxJointVelocity_,
                     jointSpaceAccLimit_,
                     jointSpaceJerkLimit_,
                     "ArmJoint",
                     &armInitialState);
    initializeFilter(lbJointRuckigFilterPtr_,
                     4,
                     lbRuckigDt,
                     maxJointVelocity_,
                     jointSpaceAccLimit_,
                     jointSpaceJerkLimit_,
                     "LbJoint",
                     &lbInitialState);
  }

  // 读取手部位置约束参数
  PARAM_AND_PRINT_FLOAT(nodeHandle_, "/ik_ros_uni_cpp_node/quest3/sphere_radius_limit", sphereRadiusLimit_, 0.5, 2);
  PARAM_AND_PRINT_FLOAT(
      nodeHandle_, "/ik_ros_uni_cpp_node/quest3/min_reachable_distance", minReachableDistance_, 0.08, 3);
  PARAM_AND_PRINT_FLOAT(
      nodeHandle_, "/ik_ros_uni_cpp_node/quest3/hand_changing_mode_threshold", handChangingModeThreshold_, 0.055, 3);
  // 读取是否使用增量式手部姿态参数
  nodeHandle_.param(
      "/ik_ros_uni_cpp_node/quest3/use_incremental_hand_orientation", useIncrementalHandOrientation_, true);
  std::cout << "/ik_ros_uni_cpp_node/quest3/use_incremental_hand_orientation="
            << (useIncrementalHandOrientation_ ? "true" : "false") << std::endl;

  // 读取 box 边界参数（向量形式）
  PARAM_AND_PRINT_VECTOR3D(nodeHandle_,
                           "/quest3/box_min_bound",
                           boxMinBound_,
                           Eigen::Vector3d(0.25, -0.5, 0.1),
                           2,
                           "[WheelQuest3IkIncrementalROS]");
  PARAM_AND_PRINT_VECTOR3D(nodeHandle_,
                           "/quest3/box_max_bound",
                           boxMaxBound_,
                           Eigen::Vector3d(1.0, 0.5, 1.0),
                           2,
                           "[WheelQuest3IkIncrementalROS]");

  // 读取胸部中线偏移量参数
  PARAM_AND_PRINT_FLOAT(nodeHandle_, "/ik_ros_uni_cpp_node/quest3/chest_offset_y_ax", chestOffsetY_, 0.0, 3);

  // 读取圆柱体约束中心参数
  PARAM_AND_PRINT_VECTOR3D(nodeHandle_,
                           "/quest3/left_center",  //
                           leftCenter_,
                           Eigen::Vector3d(0, 0.06, 0),
                           2,
                           "[WheelQuest3IkIncrementalROS]");
  PARAM_AND_PRINT_VECTOR3D(nodeHandle_,
                           "/quest3/right_center",  //
                           rightCenter_,
                           Eigen::Vector3d(0, -0.06, 0),
                           2,
                           "[WheelQuest3IkIncrementalROS]");

  // 读取手到肘距离约束参数
  PARAM_AND_PRINT_FLOAT(nodeHandle_, "/ik_ros_uni_cpp_node/quest3/elbow_min_distance", elbowMinDistance_, 0.18, 3);
  PARAM_AND_PRINT_FLOAT(nodeHandle_, "/ik_ros_uni_cpp_node/quest3/elbow_max_distance", elbowMaxDistance_, 0.65, 3);

  // 读取退出时默认手部位置参数
  PARAM_AND_PRINT_VECTOR3D(nodeHandle_,
                           "/quest3/default_left_hand_pos_on_exit",
                           defaultLeftHandPosOnExit_,
                           Eigen::Vector3d(1.0, 1.0, 1.0),
                           2,
                           "[WheelQuest3IkIncrementalROS]");
  PARAM_AND_PRINT_VECTOR3D(nodeHandle_,
                           "/quest3/default_right_hand_pos_on_exit",
                           defaultRightHandPosOnExit_,
                           Eigen::Vector3d(1.0, -1.0, 1.0),
                           2,
                           "[WheelQuest3IkIncrementalROS]");

  // 读取zyx限制参数
  PARAM_AND_PRINT_VECTOR3D(nodeHandle_,
                           "/quest3/zyx_limits_final",
                           incrementalConfig.zyxLimitsFinal,
                           Eigen::Vector3d(0.95 * M_PI / 2.0, M_PI / 2.0, M_PI / 2.0),
                           2,
                           "[WheelQuest3IkIncrementalROS]");
  PARAM_AND_PRINT_VECTOR3D(nodeHandle_,
                           "/quest3/zyx_limits_ee",
                           incrementalConfig.zyxLimitsEE,
                           Eigen::Vector3d(M_PI / 2.0, M_PI / 2.0, M_PI / 2.0),
                           2,
                           "[WheelQuest3IkIncrementalROS]");
  PARAM_AND_PRINT_VECTOR3D(nodeHandle_,
                           "/quest3/zyx_limits_link4",
                           incrementalConfig.zyxLimitsLink4,
                           Eigen::Vector3d(M_PI / 2.0, 0.6, 0.6),
                           2,
                           "[WheelQuest3IkIncrementalROS]");

  incrementalController_ = std::make_unique<WheelIncrementalControlModule>(incrementalConfig);

  quest3ArmInfoTransformerPtr_->setDeltaScale(deltaScale_);

  // print clip result
  {
    std::ostringstream oss_left, oss_right;
    oss_left << defaultLeftHandPosOnExit_.transpose().format(
        Eigen::IOFormat(Eigen::FullPrecision, 0, ", ", ", ", "", "", "", ""));
    oss_right << defaultRightHandPosOnExit_.transpose().format(
        Eigen::IOFormat(Eigen::FullPrecision, 0, ", ", ", ", "", "", "", ""));
    ROS_INFO("[WheelQuest3IkIncrementalROS] Left hand clip result: %s", oss_left.str().c_str());
    ROS_INFO("[WheelQuest3IkIncrementalROS] Right hand clip result: %s", oss_right.str().c_str());
  }

  // 使用默认手部位置初始化 latestPoseConstraintList_，确保进入增量模式时能正确初始化
  Eigen::Quaterniond defaultHandQuat = Eigen::Quaterniond::Identity();
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].position = defaultLeftHandPosOnExit_;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].rotation_matrix = defaultHandQuat.toRotationMatrix();
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].position = defaultRightHandPosOnExit_;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].rotation_matrix = defaultHandQuat.toRotationMatrix();
  // 初始化 chest / shoulders（避免未收到 chestPoseCallback 时，IK shoulder cost 拉到零）
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_CHEST].position = robotFixedWaistYawPos_ + chestDefaultOffset_;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_CHEST].rotation_matrix = Eigen::Matrix3d::Identity();
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_SHOULDER].position = robotLeftFixedShoulderPos_;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_SHOULDER].position = robotRightFixedShoulderPos_;

  // 初始化手部平滑插值器
  leftHandSmoother_ = std::make_unique<WheelHandSmoother>("左臂", "zarm_l6_link", defaultLeftHandPosOnExit_);
  rightHandSmoother_ = std::make_unique<WheelHandSmoother>("右臂", "zarm_r6_link", defaultRightHandPosOnExit_);

  // 更新默认位置（在clip之后）
  leftHandSmoother_->setDefaultPosOnExit(defaultLeftHandPosOnExit_);
  rightHandSmoother_->setDefaultPosOnExit(defaultRightHandPosOnExit_);

  // 初始化 Drake 全身（chest + 双臂肘/手）参考点优化求解器
  {
    chestElbowHandWeightConfig_ = loadDrakeChestElbowHandWeightsFromJson(configJson);
    chestElbowHandBoundsConfig_ = loadDrakeChestElbowHandBoundsFromJson(configJson);
    chestElbowHandPrevFilterConfig_ = loadDrakeChestElbowHandPrevFilterConfigFromJson(configJson);

    const Eigen::Vector3d vClsInChest = robotLeftFixedShoulderPos_ - robotFixedWaistYawPos_;
    const Eigen::Vector3d vCrsInChest = robotRightFixedShoulderPos_ - robotFixedWaistYawPos_;

    // 使用当前 FK 结果构造初始化数据
    updateFkCacheFromSensorData();
    UpperBodyPoseList initFkResult;
    initFkResult[BODY][WAIST].p = latestWaistYawFkPos_;
    initFkResult[BODY][WAIST].q = chestRotationQuaternion_;

    initFkResult[LEFT][SHOULDER].p = latestLeftShoulderFkPos_;
    initFkResult[LEFT][SHOULDER].q = Eigen::Quaterniond::Identity();
    initFkResult[LEFT][ELBOW].p = leftLink4Position_;
    initFkResult[LEFT][ELBOW].q = leftLink4Quat_;
    initFkResult[LEFT][HAND].p = leftLink6Position_;
    initFkResult[LEFT][HAND].q = leftLink6Quat_;

    initFkResult[RIGHT][SHOULDER].p = latestRightShoulderFkPos_;
    initFkResult[RIGHT][SHOULDER].q = Eigen::Quaterniond::Identity();
    initFkResult[RIGHT][ELBOW].p = rightLink4Position_;
    initFkResult[RIGHT][ELBOW].q = rightLink4Quat_;
    initFkResult[RIGHT][HAND].p = rightLink6Position_;
    initFkResult[RIGHT][HAND].q = rightLink6Quat_;

    chestElbowHandPointOptSolverPtr_ =
        std::make_unique<DrakeChestElbowHandPointOptSolver>(vClsInChest, vCrsInChest, l1_, l2_, &initFkResult);
    chestElbowHandPointOptSolverPtr_->setWeights(chestElbowHandWeightConfig_);
    chestElbowHandPointOptSolverPtr_->setBounds(chestElbowHandBoundsConfig_);
    chestElbowHandPointOptSolverPtr_->setPrevFilterConfig(chestElbowHandPrevFilterConfig_);

    ROS_INFO("[WheelQuest3IkIncrementalROS] DrakeChestElbowHandPointOptSolver initialized with FK result (l1=%.4f, l2=%.4f)",
             l1_,
             l2_);
  }

  // 初始化增量模块的手部姿态种子，避免首次进入增量模式时从单位四元数开始插值
  if (incrementalController_) {
    incrementalController_->setHandQuatSeeds(defaultHandQuat, defaultHandQuat, useIncrementalHandOrientation_);
  }
  if (!incrementalController_) {
    // 抛出异常，终止程序
    throw std::runtime_error(
        "[WheelQuest3IkIncrementalROS] incrementalController_ is not initialized. Please ensure it is properly created "
        "before calling run().");
  }
  if (!oneStageIkEndEffectorPtr_) {
    // 抛出异常，终止程序
    throw std::runtime_error(
        "[WheelQuest3IkIncrementalROS] oneStageIkEndEffectorPtr_ is not initialized. Please ensure it is properly created "
        "before calling run().");
  }

  ROS_INFO("[WheelQuest3IkIncrementalROS] Interpolation system initialized successfully");
}

void WheelQuest3IkIncrementalROS::publishWholeBodyRefMarkers() {
  if (!wholeBodyRefMarkerArrayPublisher_) return;

  Eigen::Vector3d chestPos = Eigen::Vector3d::Zero();
  Eigen::Vector3d leftShoulderPos = Eigen::Vector3d::Zero();
  Eigen::Vector3d rightShoulderPos = Eigen::Vector3d::Zero();
  Eigen::Vector3d leftElbowPos = Eigen::Vector3d::Zero();
  Eigen::Vector3d rightElbowPos = Eigen::Vector3d::Zero();
  Eigen::Vector3d leftHandPos = Eigen::Vector3d::Zero();
  Eigen::Vector3d rightHandPos = Eigen::Vector3d::Zero();
  Eigen::Vector3d chestBeforeOpt = latestChestPosBeforeOpt_;
  Eigen::Vector3d chestAfterOpt = latestChestPosAfterOpt_;
  Eigen::Vector3d leftShoulderBeforeOpt = latestLeftShoulderPosBeforeOpt_;
  Eigen::Vector3d leftShoulderAfterOpt = latestLeftShoulderPosAfterOpt_;
  Eigen::Vector3d rightShoulderBeforeOpt = latestRightShoulderPosBeforeOpt_;
  Eigen::Vector3d rightShoulderAfterOpt = latestRightShoulderPosAfterOpt_;
  Eigen::Vector3d leftElbowBeforeOpt = latestLeftElbowPosBeforeOpt_;
  Eigen::Vector3d leftElbowAfterOpt = latestLeftElbowPosAfterOpt_;
  Eigen::Vector3d rightElbowBeforeOpt = latestRightElbowPosBeforeOpt_;
  Eigen::Vector3d rightElbowAfterOpt = latestRightElbowPosAfterOpt_;
  Eigen::Vector3d leftHandBeforeOpt = latestLeftHandPosBeforeOpt_;
  Eigen::Vector3d leftHandAfterOpt = latestLeftHandPosAfterOpt_;
  Eigen::Vector3d rightHandBeforeOpt = latestRightHandPosBeforeOpt_;
  Eigen::Vector3d rightHandAfterOpt = latestRightHandPosAfterOpt_;

  if (latestPoseConstraintList_.size() > POSE_DATA_LIST_INDEX_CHEST) {
    chestPos = latestPoseConstraintList_[POSE_DATA_LIST_INDEX_CHEST].position;
  }
  if (latestPoseConstraintList_.size() > POSE_DATA_LIST_INDEX_LEFT_SHOULDER) {
    leftShoulderPos = latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_SHOULDER].position;
  }
  if (latestPoseConstraintList_.size() > POSE_DATA_LIST_INDEX_RIGHT_SHOULDER) {
    rightShoulderPos = latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_SHOULDER].position;
  }
  if (latestPoseConstraintList_.size() > POSE_DATA_LIST_INDEX_LEFT_ELBOW) {
    leftElbowPos = latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_ELBOW].position;
  }
  if (latestPoseConstraintList_.size() > POSE_DATA_LIST_INDEX_RIGHT_ELBOW) {
    rightElbowPos = latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_ELBOW].position;
  }
  if (latestPoseConstraintList_.size() > POSE_DATA_LIST_INDEX_LEFT_HAND) {
    leftHandPos = latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].position;
  }
  if (latestPoseConstraintList_.size() > POSE_DATA_LIST_INDEX_RIGHT_HAND) {
    rightHandPos = latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].position;
  }

  const ros::Time stamp = ros::Time::now();

  auto makeSphere =
      [&](int id, const std::string& ns, const Eigen::Vector3d& p, double scale, double r, double g, double b, double a)
      -> visualization_msgs::Marker {
    visualization_msgs::Marker m;
    m.header.frame_id = "base_link";
    m.header.stamp = stamp;
    m.ns = ns;
    m.id = id;
    m.type = visualization_msgs::Marker::SPHERE;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.pose.position.x = p.x();
    m.pose.position.y = p.y();
    m.pose.position.z = p.z();
    m.scale.x = scale;
    m.scale.y = scale;
    m.scale.z = scale;
    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = a;
    return m;
  };

  auto makeLineList = [&](int id,
                          const std::string& ns,
                          const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& segments,
                          double lineWidth,
                          double r,
                          double g,
                          double b,
                          double a) -> visualization_msgs::Marker {
    visualization_msgs::Marker m;
    m.header.frame_id = "base_link";
    m.header.stamp = stamp;
    m.ns = ns;
    m.id = id;
    m.type = visualization_msgs::Marker::LINE_LIST;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = lineWidth;
    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = a;
    m.points.reserve(2 * segments.size());
    for (const auto& seg : segments) {
      geometry_msgs::Point p0;
      p0.x = seg.first.x();
      p0.y = seg.first.y();
      p0.z = seg.first.z();
      geometry_msgs::Point p1;
      p1.x = seg.second.x();
      p1.y = seg.second.y();
      p1.z = seg.second.z();
      m.points.push_back(p0);
      m.points.push_back(p1);
    }
    return m;
  };

  visualization_msgs::MarkerArray arr;
  // Chest / shoulders / elbows / hands (optimized)
  arr.markers.push_back(makeSphere(0, "wb_ref/chest", chestPos, 0.10, 0.0, 0.6, 1.0, 0.9));
  arr.markers.push_back(makeSphere(1, "wb_ref/shoulder", leftShoulderPos, 0.08, 0.0, 0.0, 1.0, 0.9));
  arr.markers.push_back(makeSphere(2, "wb_ref/shoulder", rightShoulderPos, 0.08, 0.0, 0.0, 1.0, 0.9));
  arr.markers.push_back(makeSphere(3, "wb_ref/elbow", leftElbowPos, 0.08, 0.0, 1.0, 0.0, 0.9));
  arr.markers.push_back(makeSphere(4, "wb_ref/elbow", rightElbowPos, 0.08, 0.0, 1.0, 0.0, 0.9));
  arr.markers.push_back(makeSphere(5, "wb_ref/hand", leftHandPos, 0.09, 1.0, 0.0, 0.0, 0.9));
  arr.markers.push_back(makeSphere(6, "wb_ref/hand", rightHandPos, 0.09, 1.0, 0.0, 0.0, 0.9));

  // Hand ref before/after optimization (yellow/blue)
  arr.markers.push_back(makeSphere(7, "wb_ref/hand_before", leftHandBeforeOpt, 0.07, 1.0, 1.0, 0.0, 0.6));
  arr.markers.push_back(makeSphere(8, "wb_ref/hand_after", leftHandAfterOpt, 0.07, 0.0, 0.0, 1.0, 1.0));
  arr.markers.push_back(makeSphere(9, "wb_ref/hand_before", rightHandBeforeOpt, 0.07, 1.0, 1.0, 0.0, 0.6));
  arr.markers.push_back(makeSphere(10, "wb_ref/hand_after", rightHandAfterOpt, 0.07, 0.0, 0.0, 1.0, 1.0));

  // Chest/shoulder/elbow ref before/after optimization (yellow/blue)
  arr.markers.push_back(makeSphere(13, "wb_ref/chest_before", chestBeforeOpt, 0.08, 1.0, 1.0, 0.0, 0.6));
  arr.markers.push_back(makeSphere(14, "wb_ref/chest_after", chestAfterOpt, 0.08, 0.0, 0.0, 1.0, 1.0));
  arr.markers.push_back(makeSphere(15, "wb_ref/shoulder_before", leftShoulderBeforeOpt, 0.07, 1.0, 1.0, 0.0, 0.6));
  arr.markers.push_back(makeSphere(16, "wb_ref/shoulder_after", leftShoulderAfterOpt, 0.07, 0.0, 0.0, 1.0, 1.0));
  arr.markers.push_back(makeSphere(17, "wb_ref/shoulder_before", rightShoulderBeforeOpt, 0.07, 1.0, 1.0, 0.0, 0.6));
  arr.markers.push_back(makeSphere(18, "wb_ref/shoulder_after", rightShoulderAfterOpt, 0.07, 0.0, 0.0, 1.0, 1.0));
  arr.markers.push_back(makeSphere(19, "wb_ref/elbow_before", leftElbowBeforeOpt, 0.07, 1.0, 1.0, 0.0, 0.6));
  arr.markers.push_back(makeSphere(20, "wb_ref/elbow_after", leftElbowAfterOpt, 0.07, 0.0, 0.0, 1.0, 1.0));
  arr.markers.push_back(makeSphere(21, "wb_ref/elbow_before", rightElbowBeforeOpt, 0.07, 1.0, 1.0, 0.0, 0.6));
  arr.markers.push_back(makeSphere(22, "wb_ref/elbow_after", rightElbowAfterOpt, 0.07, 0.0, 0.0, 1.0, 1.0));

  // Chains (LINE_LIST): shoulder->elbow, elbow->hand
  const double lineWidth = 0.02;
  const double alpha = 0.9;
  const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> leftSegments = {
      {leftShoulderPos, leftElbowPos},
      {leftElbowPos, leftHandPos},
  };
  const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> rightSegments = {
      {rightShoulderPos, rightElbowPos},
      {rightElbowPos, rightHandPos},
  };
  // Left chain: cyan
  arr.markers.push_back(makeLineList(11, "wb_ref/chain_left", leftSegments, lineWidth, 0.0, 1.0, 1.0, alpha));
  // Right chain: magenta
  arr.markers.push_back(makeLineList(12, "wb_ref/chain_right", rightSegments, lineWidth, 1.0, 0.0, 1.0, alpha));

  wholeBodyRefMarkerArrayPublisher_.publish(arr);
}

Eigen::Vector3d WheelQuest3IkIncrementalROS::computeElbow(const Eigen::Vector3d& link6Pos,
                                                     const Eigen::Vector3d& endEffectorPos,
                                                     const double link2Length) {
  const Eigen::Vector3d eeToWristVec = link6Pos - endEffectorPos;
  const double n = eeToWristVec.norm();
  if (n > 1e-6) {
    return link6Pos + eeToWristVec * (link2Length / n);
  }
  return link6Pos - Eigen::Vector3d::UnitZ() * link2Length;
}

void WheelQuest3IkIncrementalROS::remapUpperBodyRefPoints(const Eigen::Vector3d& chestPos,
                                                     const Eigen::Quaterniond& chestQuat,
                                                     const Eigen::Vector3d& leftShoulderPos,
                                                     const Eigen::Vector3d& rightShoulderPos,
                                                     Eigen::Vector3d& leftHandRef,
                                                     Eigen::Vector3d& rightHandRef,
                                                     Eigen::Vector3d& leftElbowRef,
                                                     Eigen::Vector3d& rightElbowRef) {
  double L = l1_ + l2_;  // total

  // 保存hand elbow 向量（在归一化之前）
  const Eigen::Vector3d leftHandElbowVec = leftHandRef - leftElbowRef;
  const Eigen::Vector3d rightHandElbowVec = rightHandRef - rightElbowRef;

  // leftHandRef - leftShoulderPos 的模长应该 < L， 否则进行归一化
  const Eigen::Vector3d leftHandToShoulder = leftHandRef - leftShoulderPos;
  const double leftHandToShoulderNorm = leftHandToShoulder.norm();
  if (leftHandToShoulderNorm > L && leftHandToShoulderNorm > 1e-6) {
    leftHandRef = leftShoulderPos + leftHandToShoulder.normalized() * L;
  }

  // rightHandRef - rightShoulderPos 的模长应该 < L， 否则进行归一化
  const Eigen::Vector3d rightHandToShoulder = rightHandRef - rightShoulderPos;
  const double rightHandToShoulderNorm = rightHandToShoulder.norm();
  if (rightHandToShoulderNorm > L && rightHandToShoulderNorm > 1e-6) {
    rightHandRef = rightShoulderPos + rightHandToShoulder.normalized() * L;
  }
  // 根据hand elbow向量，重新计算elbow
  leftElbowRef = leftHandRef - leftHandElbowVec;
  rightElbowRef = rightHandRef - rightHandElbowVec;
}

}  // namespace HighlyDynamic
