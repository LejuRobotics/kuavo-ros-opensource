#include "motion_capture_ik/WheelQuest3IkIncrementalROS.h"

#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <drake/geometry/scene_graph.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/framework/diagram.h>
#include <drake/systems/framework/diagram_builder.h>
#include "humanoid_wheel_interface/filters/KinemicLimitFilter.h"
#include <kuavo_msgs/changeArmCtrlMode.h>
#include <kuavo_msgs/changeLbQuickModeSrv.h>
#include <kuavo_msgs/changeTorsoCtrlMode.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Int32.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Float32MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <chrono>
#include <cmath>

#include <leju_utils/define.hpp>
#include <leju_utils/math.hpp>
#include <leju_utils/RosMsgConvertor.hpp>

#include "motion_capture_ik/WheelArmControlBaseROS.h"
#include "motion_capture_ik/Quest3ArmInfoTransformer.h"
#include "motion_capture_ik/WheelIncrementalControlModule.h"
#include "motion_capture_ik/WheelJoyStickHandler.h"

namespace HighlyDynamic {
using namespace leju_utils::ros_msg_convertor;

namespace {
void updateHandConstraintUnlocked(std::vector<PoseData>& poseList,
                                  int handIndex,
                                  const Eigen::Vector3d& handPos,
                                  const Eigen::Quaterniond& handQuat) {
  if (poseList.size() <= static_cast<size_t>(handIndex)) {
    return;
  }
  poseList[handIndex].position = handPos;
  poseList[handIndex].rotation_matrix = handQuat.toRotationMatrix();
}

void updateElbowConstraintUnlocked(std::vector<PoseData>& poseList, int elbowIndex, const Eigen::Vector3d& elbowPos) {
  if (poseList.size() <= static_cast<size_t>(elbowIndex)) {
    return;
  }
  poseList[elbowIndex].position = elbowPos;
}
}  // namespace

WheelQuest3IkIncrementalROS::WheelQuest3IkIncrementalROS(ros::NodeHandle& nodeHandle,
                                               double publishRate,
                                               bool debugPrint,
                                               ArmIdx ctrlArmIdx)
    : WheelArmControlBaseROS(nodeHandle, publishRate, debugPrint), ctrlArmIdx_(ctrlArmIdx) {}

WheelQuest3IkIncrementalROS::~WheelQuest3IkIncrementalROS() {
  shouldStop_ = true;

  if (ikSolveThread_.joinable()) {
    ikSolveThread_.join();
  }
  if (jointStatePublishThread_.joinable()) {
    jointStatePublishThread_.join();
  }
}

void WheelQuest3IkIncrementalROS::run() {
  ikSolveThread_ = std::thread(&WheelQuest3IkIncrementalROS::solveIkHandElbowThreadFunction, this);
  jointStatePublishThread_ = std::thread(&WheelQuest3IkIncrementalROS::publishJointStatesThreadFunction, this);

  // 标定后调 FSM 初始化手臂 mode 1（等价于 X+A 到跟随态的主路径）
  std::thread bootstrapArmModeThread = std::thread([this]() {
    while (ros::ok() && !quest3ArmInfoTransformerPtr_->isArmLengthMeasurementComplete()) {
      ros::Duration(0.05).sleep();
    }
    ros::Duration(0.1).sleep();

    ros::ServiceClient client = nodeHandle_.serviceClient<std_srvs::Trigger>("/quest3/bootstrap_wheel_arm_mode");
    std_srvs::Trigger srv;
    while (ros::ok()) {
      if (client.call(srv) && srv.response.success) {
        ROS_INFO("[WheelQuest3IkIncrementalROS] bootstrap_wheel_arm_mode ok");
        break;
      }
      if (srv.response.message.find("Not legacy wheel VR") != std::string::npos) {
        ROS_WARN("[WheelQuest3IkIncrementalROS] bootstrap_wheel_arm_mode skipped: %s",
                 srv.response.message.c_str());
        break;
      }
      ROS_WARN_THROTTLE(2.0, "[WheelQuest3IkIncrementalROS] bootstrap_wheel_arm_mode failed: %s",
                        srv.response.message.c_str());
      ros::Duration(0.5).sleep();
    }
  });

  std::cout << "\033[32m[WheelQuest3IkIncrementalROS] spinning start\033[0m" << std::endl;
  ros::spin();
}

void WheelQuest3IkIncrementalROS::solveIkHandElbowThreadFunction() {
  ros::Rate rate(publishRate_);
  // 用于统计时间差的静态变量
  static int loopCount = 0;
  static double totalTimeDiff = 0.0;

  while (!shouldStop() && ros::ok()) {
    loopSyncCount_++;
    recordTimestamp("SolveLoopStart", loopSyncCount_);
    updateSensorArmJointMeanFromSensorData();
    updateSensorArmJointFromSensorData();
    updateFkCacheFromSensorData();

    // 实物：硬件未就绪则不进入后续（放在未激活判断与 fsm 之前，少做无效状态维护）
    {
      bool is_real_robot = false;
      if (ros::param::getCached("/is_real", is_real_robot) && is_real_robot) {
        int hardware_is_ready = 0;
        if (!ros::param::getCached("/hardware/is_ready", hardware_is_ready) || hardware_is_ready == 0) {
          ROS_INFO_THROTTLE(3.0,
                            "[WheelQuest3IkIncrementalROS] Waiting /hardware/is_ready != 0 before incremental FSM (real robot)");
          reset();
          rate.sleep();
          continue;
        }
      }
    }

    const bool chestPoseUpdateEnabled = joyStickHandlerPtr_->getRightJoyStickYHoldWithX();
    bool chestPositionUpdateEnable = false;
    if (chestPoseUpdateEnabled) {
      chestPositionUpdateEnable = joyStickHandlerPtr_->getRightJoyStickYHold();
    }
    if (chestPoseUpdateEnabled != chestIncrementalUpdateEnabled_) {
      if (!chestPoseUpdateEnabled) {
        std::lock_guard<std::mutex> lock(chestPoseMutex_);
        frozenChestQuat_ =
            computeYawPitchOnlyQuatFromRotationMatrix(chestRotationQuaternion_.toRotationMatrix());
        if (latestPoseConstraintList_.size() > POSE_DATA_LIST_INDEX_CHEST) {
          frozenRobotChestPos_ = latestPoseConstraintList_[POSE_DATA_LIST_INDEX_CHEST].position;
        } else {
          frozenRobotChestPos_ = hasLatestWaistYawFk_ ? latestWaistYawFkPos_ : robotFixedWaistYawPos_;
        }
      }
      chestIncrementalUpdateEnabled_ = chestPoseUpdateEnabled;
    }
    chestPositionUpdateEnable_ = chestPoseUpdateEnabled && chestPositionUpdateEnable;
    drakeSolveUpdateChestPosition_ =
        drakeSolveUpdateChestPositionConfig_ && chestIncrementalUpdateEnabled_ && chestPositionUpdateEnable_;

    {
      std::lock_guard<std::mutex> lock(chestPoseMutex_);
      latestLeftHandPose_vr_ = quest3ArmInfoTransformerPtr_->getLeftHandPose();
      latestRightHandPose_vr_ = quest3ArmInfoTransformerPtr_->getRightHandPose();
    }

    // 【三点跳变检测】验证并过滤 VR 数据中的异常跳变
    bool currentLeftGripPressed = joyStickHandlerPtr_ ? joyStickHandlerPtr_->isLeftGrip() : false;
    bool currentRightGripPressed = joyStickHandlerPtr_ ? joyStickHandlerPtr_->isRightGrip() : false;
    validateVrPose(latestLeftHandPose_vr_, latestLeftHandPose_vr_, "Left", currentLeftGripPressed);
    validateVrPose(latestRightHandPose_vr_, latestRightHandPose_vr_, "Right", currentRightGripPressed);

    if (armControlMode_ == 0 || armControlMode_ == 1) {
      if (lastArmControlMode_ == 2) {
        fsmExit();
      }
      reset();  // 机器人未激活（含 2→0/1），持续重置各类状态，确保进入系统时正常
      rate.sleep();
      continue;  // 机器人未激活，不进行后续流程
    }
    fsmEnter();
    fsmChange();
    fsmProcess();
    fsmExit();

    publishEndEffectorControlData();
    publishAuxiliaryStates();
    publishWholeBodyRefMarkers();

    rate.sleep();
  }
}

void WheelQuest3IkIncrementalROS::publishJointStatesThreadFunction() {
  ros::Rate rate(jointStatePublishRateHz_);
  while (!shouldStop() && ros::ok()) {
    if (armControlMode_ == 2) {
      publishJointStates();
    } else {
      publishDefaultJointStates();
    }
    rate.sleep();
  }
}

void WheelQuest3IkIncrementalROS::fsmEnter() {
  auto updateChestConstraintFromFk = [&]() {
    // 如果 FK 可用，直接使用胸部IK目标frame的 FK 结果；否则使用零位胸部目标frame位置
    Eigen::Vector3d chestPos = hasLatestWaistYawFk_ ? latestWaistYawFkPos_ : robotFixedWaistYawPos_;
    Eigen::Matrix3d chestR = Eigen::Matrix3d::Identity();
    chestR = chestRotationQuaternion_.toRotationMatrix();
    if (latestPoseConstraintList_.size() > POSE_DATA_LIST_INDEX_CHEST) {
      latestPoseConstraintList_[POSE_DATA_LIST_INDEX_CHEST].position = chestPos;
      latestPoseConstraintList_[POSE_DATA_LIST_INDEX_CHEST].rotation_matrix = chestR;
    }
  };
  // 正常工作模式 Case 2: (0→2 或 1→2)
  if ((armControlMode_ == 2 && lastArmControlMode_ == 1) || (armControlMode_ == 2 && lastArmControlMode_ == 0)) {
    // S^0 → S^3 顶层状态切换
    exitMode2Counter_ = 0;
    auto resetMode2State = [&](bool resetIkSolution) {
      {
        std::lock_guard<std::mutex> jointLock(jointStateMutex_);
        q_ = Eigen::VectorXd::Zero(14);
        dq_ = Eigen::VectorXd::Zero(14);
        latest_q_ = Eigen::VectorXd::Zero(14);
        latest_dq_ = Eigen::VectorXd::Zero(14);
        lowpass_dq_ = Eigen::VectorXd::Zero(14);
        lb_q_ = Eigen::VectorXd::Zero(4);
        lb_dq_ = Eigen::VectorXd::Zero(4);
        latest_lb_q_ = Eigen::VectorXd::Zero(4);
        latest_lb_dq_ = Eigen::VectorXd::Zero(4);
        lowpass_lb_dq_ = Eigen::VectorXd::Zero(4);
      }
      {
        std::lock_guard<std::mutex> lock(lbLegMoveTimeMutex_);
        lbLegMoveStartTime_ = ros::Time(0);
      }

      {
        std::lock_guard<std::mutex> lock(ikResultMutex_);
        if (resetIkSolution) {
          if (latestIkSolution_.size() == drakeJointStateSize_) {
            latestIkSolution_.setZero();
          } else {
            // roserror latestIkSolution_ size is not equal to
            ROS_ERROR("[WheelQuest3IkIncrementalROS] latestIkSolution_ size is not equal to drakeJointStateSize");
          }
          hasValidIkSolution_ = false;
        }
        // 同步置零 ikLowerBodyJointCommand_
        ikLowerBodyJointCommand_.setZero();
      }
    };

    if (enterMode2ResetCounter_ < ENTER_MODE_2_RESET_COUNT) {
      // NOTES：确保进入mode 2时，左右手位置和姿态都为初始值
      // NOTES: 状态描述：默认机器人在5sec内已经回归到零位，因此进入mode 2时，左右手位置和姿态都为初始值
      Eigen::Vector3d currentLeftHandPos = initZeroLeftLink6Position_;
      Eigen::Vector3d currentRightHandPos = initZeroRightLink6Position_;
      Eigen::Quaterniond currentLeftHandQuat = Eigen::Quaterniond::Identity();
      Eigen::Quaterniond currentRightHandQuat = Eigen::Quaterniond::Identity();

      updateHandConstraintUnlocked(
          latestPoseConstraintList_, POSE_DATA_LIST_INDEX_LEFT_HAND, currentLeftHandPos, currentLeftHandQuat);
      updateHandConstraintUnlocked(
          latestPoseConstraintList_, POSE_DATA_LIST_INDEX_RIGHT_HAND, currentRightHandPos, currentRightHandQuat);

      if (incrementalController_) {
        incrementalController_->reset();
        incrementalController_->setHandQuatSeeds(
            currentLeftHandQuat, currentRightHandQuat, useIncrementalHandOrientation_);
      }

      resetMode2State(true);

      latestIncrementalResult_ = WheelIncrementalPoseResult();

      enterMode2ResetCounter_++;
    }

    const bool shouldEnterLeft =
        incrementalController_->shouldEnterIncrementalModeLeftArm(joyStickHandlerPtr_->isLeftGrip());
    const bool shouldEnterRight =
        incrementalController_->shouldEnterIncrementalModeRightArm(joyStickHandlerPtr_->isRightGrip());

    if (shouldEnterLeft || shouldEnterRight) {
      // 任意手进入增量模式时，同步激活 chest 增量（第二只手进入时不重复激活）
      Eigen::Vector3d humanChestPos = Eigen::Vector3d::Zero();
      {
        std::lock_guard<std::mutex> lock(chestPoseMutex_);
        if (hasChestPose_) {
          humanChestPos = latestChestPositionInRobot_;
        }
      }
      updateChestConstraintFromFk();
      incrementalController_->enterIncrementalModeChest(humanChestPos, latestPoseConstraintList_);

      // 处理左臂：计算FK -> 更新约束列表 -> 进入增量模式
      if (shouldEnterLeft) {
        // FK 已在主循环更新到缓存

        // 【核心修复】在进入增量模式前，先更新 latestPoseConstraintList_ 为当前 FK 计算的 Link6 位置
        // 避免使用上次退出时保存的旧位置，导致跳变
        updateHandConstraintUnlocked(
            latestPoseConstraintList_, POSE_DATA_LIST_INDEX_LEFT_HAND, leftLink6Position_, leftLink6Quat_);

        incrementalController_->enterIncrementalModeLeftArm(latestLeftHandPose_vr_,
                                                            latestPoseConstraintList_,
                                                            leftEndEffectorPosition_,
                                                            leftEndEffectorQuat_,
                                                            leftLink4Quat_);
      }

      // 处理右臂：计算FK -> 更新约束列表 -> 进入增量模式
      if (shouldEnterRight) {
        // FK 已在主循环更新到缓存

        // 【核心修复】在进入增量模式前，先更新 latestPoseConstraintList_ 为当前 FK 计算的 Link6 位置
        // 避免使用上次退出时保存的旧位置，导致跳变
        updateHandConstraintUnlocked(
            latestPoseConstraintList_, POSE_DATA_LIST_INDEX_RIGHT_HAND, rightLink6Position_, rightLink6Quat_);

        incrementalController_->enterIncrementalModeRightArm(latestRightHandPose_vr_,
                                                             latestPoseConstraintList_,
                                                             rightEndEffectorPosition_,
                                                             rightEndEffectorQuat_,
                                                             rightLink4Quat_);
      }
    }

    // 超时机制：0→2 和 1→2 都需要超时保护
    ros::Time currentTime = ros::Time::now();
    ros::Time enterTime;
    {
      std::lock_guard<std::mutex> lock(mode2EnterTimeMutex_);
      enterTime = mode2EnterTime_;
    }

    if (enterTime.isZero()) {  // 如果时间戳未设置，可能是回调函数还未执行，先记录当前时间作为容错机制，避免出现异常
      std::lock_guard<std::mutex> lock(mode2EnterTimeMutex_);
      if (mode2EnterTime_.isZero()) {
        mode2EnterTime_ = currentTime;
        enterTime = currentTime;
      } else {
        enterTime = mode2EnterTime_;
      }
    }

    double elapsedTime = (currentTime - enterTime).toSec();

    if (elapsedTime <= MODE_2_TIMEOUT_DURATION) {
      // print mode2 timeout duration
      // std::cout << "[WheelQuest3IkIncrementalROS] Mode 2 timeout duration: " << elapsedTime << "s" << std::endl;
      forceDeactivateAllArmCtrlMode();

      updateHandConstraintUnlocked(latestPoseConstraintList_,
                                   POSE_DATA_LIST_INDEX_LEFT_HAND,
                                   initZeroLeftLink6Position_,
                                   Eigen::Quaterniond::Identity());
      updateHandConstraintUnlocked(latestPoseConstraintList_,
                                   POSE_DATA_LIST_INDEX_RIGHT_HAND,
                                   initZeroRightLink6Position_,
                                   Eigen::Quaterniond::Identity());

      // 重置增量控制模块，清除可能被 fsmChange/fsmProcess 更新的 ruckig 滤波状态
      if (incrementalController_) {
        incrementalController_->reset();
        incrementalController_->setHandQuatSeeds(
            Eigen::Quaterniond::Identity(), Eigen::Quaterniond::Identity(), useIncrementalHandOrientation_);
      }

      resetMode2State(false);

      // 在超时时间内，执行进入增量模式（0→2 和 1→2 都需要）
      updateHandConstraintUnlocked(
          latestPoseConstraintList_, POSE_DATA_LIST_INDEX_LEFT_HAND, leftLink6Position_, leftLink6Quat_);
      updateHandConstraintUnlocked(
          latestPoseConstraintList_, POSE_DATA_LIST_INDEX_RIGHT_HAND, rightLink6Position_, rightLink6Quat_);
      // 进入增量模式前，同步激活 chest 增量（避免腰部目标跳变）
      Eigen::Vector3d humanChestPos = Eigen::Vector3d::Zero();
      {
        std::lock_guard<std::mutex> lock(chestPoseMutex_);
        if (hasChestPose_) {
          humanChestPos = latestChestPositionInRobot_;
        }
      }
      updateChestConstraintFromFk();
      incrementalController_->enterIncrementalModeChest(humanChestPos, latestPoseConstraintList_);

      incrementalController_->enterIncrementalModeLeftArm(latestLeftHandPose_vr_,
                                                          latestPoseConstraintList_,
                                                          leftEndEffectorPosition_,
                                                          leftEndEffectorQuat_,
                                                          leftLink4Quat_);

      incrementalController_->enterIncrementalModeRightArm(latestRightHandPose_vr_,
                                                           latestPoseConstraintList_,
                                                           rightEndEffectorPosition_,
                                                           rightEndEffectorQuat_,
                                                           rightLink4Quat_);
    }
  }
}

void WheelQuest3IkIncrementalROS::fsmChange() {
  modeChangeCycle_.resetAll();
  if (armControlMode_ != 2) return;
  modeChangeCycle_.leftHandCtrlModeChanged = joyStickHandlerPtr_->hasLeftArmCtrlModeChanged();
  modeChangeCycle_.rightHandCtrlModeChanged = joyStickHandlerPtr_->hasRightArmCtrlModeChanged();
}

void WheelQuest3IkIncrementalROS::fsmProcess() {
  if (armControlMode_ != 2) return;
  activateController();
  // mode2 下确保 chest 增量模式已激活，避免胸部更新刷屏警告
  if (!incrementalController_->isIncrementalModeChest()) {
    Eigen::Vector3d humanChestPos = Eigen::Vector3d::Zero();
    {
      std::lock_guard<std::mutex> lock(chestPoseMutex_);
      if (hasChestPose_) {
        humanChestPos = latestChestPositionInRobot_;
      }
    }
    // 使用当前 FK 更新胸部约束，避免胸部增量初始跳变
    Eigen::Vector3d chestPos = hasLatestWaistYawFk_ ? latestWaistYawFkPos_ : robotFixedWaistYawPos_;
    Eigen::Matrix3d chestR = chestRotationQuaternion_.toRotationMatrix();
    if (latestPoseConstraintList_.size() > POSE_DATA_LIST_INDEX_CHEST) {
      latestPoseConstraintList_[POSE_DATA_LIST_INDEX_CHEST].position = chestPos;
      latestPoseConstraintList_[POSE_DATA_LIST_INDEX_CHEST].rotation_matrix = chestR;
    }
    incrementalController_->enterIncrementalModeChest(humanChestPos, latestPoseConstraintList_);
  }
  // 0) 在 fsmProcess 统一更新 smoother 状态，确保后续 getModeChangingState() 是最新的
  {
    auto [leftChangingMaintainUpdated, leftChangingInstantUpdated] =
        leftHandSmoother_->updateModeChangingStateIfNeeded(modeChangeCycle_.leftHandCtrlModeChanged);
    auto [rightChangingMaintainUpdated, rightChangingInstantUpdated] =
        rightHandSmoother_->updateModeChangingStateIfNeeded(modeChangeCycle_.rightHandCtrlModeChanged);

    modeChangeCycle_.leftChangingMaintainUpdated = leftChangingMaintainUpdated;
    modeChangeCycle_.rightChangingMaintainUpdated = rightChangingMaintainUpdated;
    modeChangeCycle_.leftChangingInstantUpdated = leftChangingInstantUpdated;
    modeChangeCycle_.rightChangingInstantUpdated = rightChangingInstantUpdated;
  }

  auto [leftMaintainProcess, leftInstantProcess] = leftHandSmoother_->getModeChangingState();
  auto [rightMaintainProcess, rightInstantProcess] = rightHandSmoother_->getModeChangingState();

  struct FrozenRefs {
    Eigen::Vector3d leftHandPos = Eigen::Vector3d::Zero();
    Eigen::Vector3d rightHandPos = Eigen::Vector3d::Zero();
    Eigen::Vector3d leftElbowPos = Eigen::Vector3d::Zero();
    Eigen::Vector3d rightElbowPos = Eigen::Vector3d::Zero();
    Eigen::Quaterniond leftHandQuat = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond rightHandQuat = Eigen::Quaterniond::Identity();
  };

  auto computeElbowRef = [&](bool active,
                             const Eigen::Vector3d& link6Pos,
                             const Eigen::Vector3d& endEffectorPos,
                             Eigen::Vector3d& cachedElbow,
                             const Eigen::Vector3d& frozenElbow) -> Eigen::Vector3d {
    if (!active) return frozenElbow;
    const Eigen::Vector3d eeToWristVec = link6Pos - endEffectorPos;
    const double n = eeToWristVec.norm();
    if (n > 1e-6) {
      cachedElbow = link6Pos + eeToWristVec * (l2_ / n);
    }
    return cachedElbow;
  };

  auto buildWholeBodyInput = [&](bool leftActive, bool rightActive, FrozenRefs& frozen) -> WholeBodyRefInput {
    WholeBodyRefInput input;
    input.leftRefActive = leftActive;
    input.rightRefActive = rightActive;
    if (latestPoseConstraintList_.size() > POSE_DATA_LIST_INDEX_CHEST) {
      input.chestPosRef = latestPoseConstraintList_[POSE_DATA_LIST_INDEX_CHEST].position;
    } else {
      input.chestPosRef = robotFixedWaistYawPos_;
    }
    if (!chestIncrementalUpdateEnabled_) {
      input.chestPosRef = frozenRobotChestPos_;
    }
    if (latestPoseConstraintList_.size() > POSE_DATA_LIST_INDEX_LEFT_HAND) {
      frozen.leftHandPos = latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].position;
      frozen.leftHandQuat =
          Eigen::Quaterniond(latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].rotation_matrix).normalized();
    }
    if (latestPoseConstraintList_.size() > POSE_DATA_LIST_INDEX_RIGHT_HAND) {
      frozen.rightHandPos = latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].position;
      frozen.rightHandQuat =
          Eigen::Quaterniond(latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].rotation_matrix).normalized();
    }
    if (latestPoseConstraintList_.size() > POSE_DATA_LIST_INDEX_LEFT_ELBOW) {
      frozen.leftElbowPos = latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_ELBOW].position;
    }
    if (latestPoseConstraintList_.size() > POSE_DATA_LIST_INDEX_RIGHT_ELBOW) {
      frozen.rightElbowPos = latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_ELBOW].position;
    }
    {
      Eigen::Matrix3d chestR = chestRotationQuaternion_.toRotationMatrix();
      input.chestQuatRef = computeYawPitchOnlyQuatFromRotationMatrix(chestR);
      if (!chestIncrementalUpdateEnabled_) {
        input.chestQuatRef = frozenChestQuat_;
      }
    }
    return input;
  };

  auto applyWholeBodyAndSolve = [&](WholeBodyRefInput& input,
                                    Eigen::Vector3d leftHandPos,
                                    Eigen::Quaterniond leftHandQuat,
                                    Eigen::Vector3d rightHandPos,
                                    Eigen::Quaterniond rightHandQuat,
                                    const FrozenRefs& frozen) -> bool {
    const bool leftGripPressed = joyStickHandlerPtr_ ? joyStickHandlerPtr_->isLeftGrip() : false;
    const bool rightGripPressed = joyStickHandlerPtr_ ? joyStickHandlerPtr_->isRightGrip() : false;
    const Eigen::Vector3d chestPosForFk = hasLatestWaistYawFk_ ? latestWaistYawFkPos_ : input.chestPosRef;
    auto updateHandPoseInChest = [&](bool isActive,
                                     const Eigen::Vector3d& handFkPos,
                                     const Eigen::Quaterniond& handFkQuat,
                                     const Eigen::Vector3d& fallbackWorldPos,
                                     const Eigen::Quaterniond& fallbackWorldQuat,
                                     Eigen::Vector3d& handPosInChest,
                                     Eigen::Quaterniond& handQuatInChest,
                                     bool& hasPoseInChest) {
      if (isActive) {
        auto [handQuatChest, handPosChest] = transformPose(input.chestQuatRef, chestPosForFk, handFkQuat, handFkPos);
        handPosInChest = handPosChest;
        handQuatInChest = handQuatChest.normalized();
        hasPoseInChest = true;
        return;
      }
      if (!hasPoseInChest) {
        auto [handQuatChest, handPosChest] =
            transformPose(input.chestQuatRef, input.chestPosRef, fallbackWorldQuat, fallbackWorldPos);
        handPosInChest = handPosChest;
        handQuatInChest = handQuatChest.normalized();
        hasPoseInChest = true;
      }
    };
    auto updateElbowPosInChest = [&](bool isActive,
                                     const Eigen::Vector3d& link6Pos,
                                     const Eigen::Vector3d& endEffectorPos,
                                     const Eigen::Vector3d& fallbackWorldPos,
                                     Eigen::Vector3d& elbowPosInChest,
                                     bool& hasElbowPosInChest) {
      if (isActive) {
        Eigen::Vector3d elbowWorld = fallbackWorldPos;
        const Eigen::Vector3d eeToWristVec = link6Pos - endEffectorPos;
        const double n = eeToWristVec.norm();
        if (n > 1e-6) {
          elbowWorld = link6Pos + eeToWristVec * (l2_ / n);
        }
        auto [elbowQuatChest, elbowPosChest] =
            transformPose(input.chestQuatRef, chestPosForFk, Eigen::Quaterniond::Identity(), elbowWorld);
        (void)elbowQuatChest;
        elbowPosInChest = elbowPosChest;
        hasElbowPosInChest = true;
        return;
      }
      if (!hasElbowPosInChest) {
        auto [elbowQuatChest, elbowPosChest] =
            transformPose(input.chestQuatRef, input.chestPosRef, Eigen::Quaterniond::Identity(), fallbackWorldPos);
        (void)elbowQuatChest;
        elbowPosInChest = elbowPosChest;
        hasElbowPosInChest = true;
      }
    };

    updateHandPoseInChest(leftGripPressed,
                          leftLink6Position_,
                          leftLink6Quat_,
                          frozen.leftHandPos,
                          frozen.leftHandQuat,
                          leftHandPosInChest_,
                          leftHandQuatInChest_,
                          hasLeftHandPoseInChest_);
    updateHandPoseInChest(rightGripPressed,
                          rightLink6Position_,
                          rightLink6Quat_,
                          frozen.rightHandPos,
                          frozen.rightHandQuat,
                          rightHandPosInChest_,
                          rightHandQuatInChest_,
                          hasRightHandPoseInChest_);
    updateElbowPosInChest(leftGripPressed,
                          leftLink6Position_,
                          leftEndEffectorPosition_,
                          frozen.leftElbowPos,
                          leftElbowPosInChest_,
                          hasLeftElbowPosInChest_);
    updateElbowPosInChest(rightGripPressed,
                          rightLink6Position_,
                          rightEndEffectorPosition_,
                          frozen.rightElbowPos,
                          rightElbowPosInChest_,
                          hasRightElbowPosInChest_);

    if (!input.leftRefActive) {
      if (hasLeftHandPoseInChest_) {
        leftHandPos = input.chestPosRef + input.chestQuatRef * leftHandPosInChest_;
        leftHandQuat = (input.chestQuatRef * leftHandQuatInChest_).normalized();
      } else {
        leftHandPos = frozen.leftHandPos;
        leftHandQuat = frozen.leftHandQuat;
      }
    }
    if (!input.rightRefActive) {
      if (hasRightHandPoseInChest_) {
        rightHandPos = input.chestPosRef + input.chestQuatRef * rightHandPosInChest_;
        rightHandQuat = (input.chestQuatRef * rightHandQuatInChest_).normalized();
      } else {
        rightHandPos = frozen.rightHandPos;
        rightHandQuat = frozen.rightHandQuat;
      }
    }

    const Eigen::Matrix3d chestRRef = input.chestQuatRef.normalized().toRotationMatrix();
    const Eigen::Vector3d vLeftShoulderInChest = robotLeftFixedShoulderPos_ - robotFixedWaistYawPos_;
    const Eigen::Vector3d vRightShoulderInChest = robotRightFixedShoulderPos_ - robotFixedWaistYawPos_;
    const Eigen::Vector3d leftShoulderRef = input.chestPosRef + chestRRef * vLeftShoulderInChest;
    const Eigen::Vector3d rightShoulderRef = input.chestPosRef + chestRRef * vRightShoulderInChest;

    input.leftHandRef = leftHandPos;
    input.rightHandRef = rightHandPos;
    input.leftHandQuat = leftHandQuat.normalized();
    input.rightHandQuat = rightHandQuat.normalized();
    if (input.leftRefActive) {
      input.leftElbowRef = computeElbowRef(input.leftRefActive,
                                           leftLink6Position_,
                                           leftEndEffectorPosition_,
                                           latestHumanLeftElbowPos_,
                                           frozen.leftElbowPos);
    } else if (hasLeftElbowPosInChest_) {
      input.leftElbowRef = input.chestPosRef + input.chestQuatRef * leftElbowPosInChest_;
    } else {
      input.leftElbowRef = frozen.leftElbowPos;
    }
    if (input.rightRefActive) {
      input.rightElbowRef = computeElbowRef(input.rightRefActive,
                                            rightLink6Position_,
                                            rightEndEffectorPosition_,
                                            latestHumanRightElbowPos_,
                                            frozen.rightElbowPos);
    } else if (hasRightElbowPosInChest_) {
      input.rightElbowRef = input.chestPosRef + input.chestQuatRef * rightElbowPosInChest_;
    } else {
      input.rightElbowRef = frozen.rightElbowPos;
    }

    if (!updateWholeBodyConstraintList(input)) return false;
    recordTimestamp("solveIkStart", loopSyncCount_);
    solveIk();
    // 记录时间戳
    recordTimestamp("solveIkFinish", loopSyncCount_);
    return true;
  };

  // 1) 优先处理“模式切换过渡期”：由 fsmProcess 统一更新约束并求解 IK（避免与正常 grip 跟随混杂）
  if (modeChangeCycle_.leftChangingMaintainUpdated || modeChangeCycle_.rightChangingMaintainUpdated) {
    // print leftHandCtrlModeChanged and rightHandCtrlModeChanged
    // std::cout << "[fsmProcess][modeChange]:"
    //           << "leftChangingMaintainUpdated: " << modeChangeCycle_.leftChangingMaintainUpdated << ", "
    //           << "rightChangingMaintainUpdated: " << modeChangeCycle_.rightChangingMaintainUpdated << std::endl;

    if (!updateLatestIncrementalResult()) return;  // check update success

    // 任意手处于增量更新期时，同步更新 chest 位置增量
    if (chestIncrementalUpdateEnabled_) {
      Eigen::Vector3d humanChestPos = Eigen::Vector3d::Zero();
      {
        std::lock_guard<std::mutex> lock(chestPoseMutex_);
        if (hasChestPose_) {
          humanChestPos = latestChestPositionInRobot_;
        }
      }
      incrementalController_->computeIncrementalChestPos(humanChestPos, true);
      if (latestPoseConstraintList_.size() > POSE_DATA_LIST_INDEX_CHEST) {
        latestPoseConstraintList_[POSE_DATA_LIST_INDEX_CHEST].position =
            incrementalController_->getLatestRobotChestPos();
      }
    }

    // -----------------------------
    // Whole-body reference update (chest + L/R elbow/hand) with yaw/pitch-only chest orientation.
    // - For hands not in this mode-changing cycle, freeze elbow_ref/hand_ref using the latest constraints.
    // -----------------------------
    FrozenRefs frozen;
    WholeBodyRefInput input = buildWholeBodyInput(
        modeChangeCycle_.leftChangingMaintainUpdated, modeChangeCycle_.rightChangingMaintainUpdated, frozen);

    auto [incrementalLeftQuat, incrementalRightQuat, scaledLeftHandPos, scaledRightHandPos] =
        latestIncrementalResult_.getLatestIncrementalHandPose(true, useIncrementalHandOrientation_, true);

    // Apply hand smoother in mode-changing cycle (it updates the position by reference).
    if (input.leftRefActive && modeChangeCycle_.leftHandCtrlModeChanged) {
      const bool isLeftArmCtrlModeActive = joyStickHandlerPtr_->isLeftArmCtrlModeActive();
      auto [leftMaintain, leftInstant] = leftHandSmoother_->getModeChangingState();
      bool leftInstantCopy = leftInstant;
      if (isLeftArmCtrlModeActive) {
        leftHandSmoother_->processActiveModeInterpolation(
            scaledLeftHandPos, leftInstantCopy, leftHandSmoother_->getDefaultPosOnExit(), "左臂");
      } else {
        leftHandSmoother_->processInactiveModeInterpolation(
            scaledLeftHandPos, leftInstantCopy, leftHandSmoother_->getDefaultPosOnExit(), "左臂");
      }
      leftHandSmoother_->setModeChangingState(leftMaintain, leftInstantCopy);
    }
    if (input.rightRefActive && modeChangeCycle_.rightHandCtrlModeChanged) {
      const bool isRightArmCtrlModeActive = joyStickHandlerPtr_->isRightArmCtrlModeActive();
      auto [rightMaintain, rightInstant] = rightHandSmoother_->getModeChangingState();
      bool rightInstantCopy = rightInstant;
      if (isRightArmCtrlModeActive) {
        rightHandSmoother_->processActiveModeInterpolation(
            scaledRightHandPos, rightInstantCopy, rightHandSmoother_->getDefaultPosOnExit(), "右臂");
      } else {
        rightHandSmoother_->processInactiveModeInterpolation(
            scaledRightHandPos, rightInstantCopy, rightHandSmoother_->getDefaultPosOnExit(), "右臂");
      }
      rightHandSmoother_->setModeChangingState(rightMaintain, rightInstantCopy);
    }
    if (!applyWholeBodyAndSolve(
            input, scaledLeftHandPos, incrementalLeftQuat, scaledRightHandPos, incrementalRightQuat, frozen)) {
      return;
    }

    updateLeftHandChangingMode(leftHandSmoother_->getDefaultPosOnExit());
    updateRightHandChangingMode(rightHandSmoother_->getDefaultPosOnExit());

    // 重置激活计数器，为下次 mode changing 结束后重新激活做准备
    activateAllArmCtrlModeCounter_ = 0;
    return;
  }

  // 2) 非模式切换：在 fsmProcess 中执行一次“激活全部 arm ctrl mode”的收尾动作（增强鲁棒性）
  if (activateAllArmCtrlModeCounter_ < ACTIVATE_ALL_ARM_CTRL_MODE_COUNT) {
    forceActivateAllArmCtrlMode();
    kuavo_msgs::changeArmCtrlMode srv3;
    srv3.request.control_mode = static_cast<int>(kuavo_msgs::changeArmCtrlMode::Request::ik_ultra_fast_mode);
    enableWbcArmTrajectoryControlClient_.call(srv3);
    activateAllArmCtrlModeCounter_++;

    setControlMode(3);
  }

  bool currentLeftGripPressed = joyStickHandlerPtr_->isLeftGrip();
  bool currentRightGripPressed = joyStickHandlerPtr_->isRightGrip();

  auto updateGripTimeout = [&](bool currentGripPressed,
                               bool lastGripPressed,
                               bool armMoved,
                               std::mutex& gripTimeMutex,
                               ros::Time& gripStartTime,
                               std::atomic<bool>& gripTimeoutReached,
                               const char* logLabel) {
    ros::Time currentTime = ros::Time::now();
    if (currentGripPressed) {
      // 只有在检测到移动时才开始计时
      if (armMoved) {
        ros::Time startTime;
        {
          std::lock_guard<std::mutex> lock(gripTimeMutex);
          // 如果还没有开始计时，则开始计时
          if (gripStartTime.isZero()) {
            gripStartTime = currentTime;
            gripTimeoutReached.store(false);
          }
          startTime = gripStartTime;
        }

        // 检查是否达到超时
        if (!startTime.isZero()) {
          double elapsedTime = (currentTime - startTime).toSec();
          if (elapsedTime >= GRIP_TIMEOUT_DURATION && !gripTimeoutReached.load()) {
            gripTimeoutReached.store(true);
            ROS_INFO("[WheelQuest3IkIncrementalROS] %s grip timeout reached (%.3f seconds)", logLabel, elapsedTime);
            setLbArmQuickMode(2);
          }
        }
      } else {
        // 未检测到移动，重置时间戳（不开始计时）
        std::lock_guard<std::mutex> lock(gripTimeMutex);
        if (!gripStartTime.isZero()) {
          gripStartTime = ros::Time(0);
          gripTimeoutReached.store(false);
        }
      }
    } else {
      // grip 释放，重置时间戳和布尔值
      if (lastGripPressed) {
        std::lock_guard<std::mutex> lock(gripTimeMutex);
        gripStartTime = ros::Time(0);
        gripTimeoutReached.store(false);
      }
    }
  };

  updateGripTimeout(currentLeftGripPressed,
                    lastLeftGripPressed_,
                    incrementalController_->hasLeftArmMoved(),
                    leftGripTimeMutex_,
                    leftGripStartTime_,
                    leftGripTimeoutReached_,
                    "Left");

  updateGripTimeout(currentRightGripPressed,
                    lastRightGripPressed_,
                    incrementalController_->hasRightArmMoved(),
                    rightGripTimeMutex_,
                    rightGripStartTime_,
                    rightGripTimeoutReached_,
                    "Right");

  bool leftGripRisingEdge = currentLeftGripPressed && !lastLeftGripPressed_;
  bool rightGripRisingEdge = currentRightGripPressed && !lastRightGripPressed_;

  // 双扳机同时松开的下降沿：捕获当前 lb 关节命令快照
  // 目的：胸部增量模式激活时，松开扳机后仅允许 waist_yaw（关节4）跟随 VR 旋转，
  //       冻结 knee/leg/waist_pitch（关节1~3），避免 IK 优化目标切换引起手臂漂移上升
  const bool bothGripsJustReleased = (lastLeftGripPressed_ || lastRightGripPressed_) &&
                                     !currentLeftGripPressed && !currentRightGripPressed;
  if (bothGripsJustReleased && chestIncrementalUpdateEnabled_) {
    std::lock_guard<std::mutex> lock(ikResultMutex_);
    if (ikLowerBodyJointCommand_.size() == 4) {
      frozenLbJointCommand_ = ikLowerBodyJointCommand_;
      hasLbJointCommandFrozen_ = true;
    }
  }

  // 更新上一帧的 grip 状态（必须在使用完之后更新）
  lastLeftGripPressed_ = currentLeftGripPressed;
  lastRightGripPressed_ = currentRightGripPressed;

  handleGripRisingEdge(leftGripRisingEdge, rightGripRisingEdge, leftMaintainProcess, rightMaintainProcess);

  // 由胸部更新开关控制增量更新（与 grip 解耦）
  if (chestIncrementalUpdateEnabled_ && (lastLeftGripPressed_ || lastRightGripPressed_)) {
    Eigen::Vector3d humanChestPos = Eigen::Vector3d::Zero();
    {
      std::lock_guard<std::mutex> lock(transformerDataMutex_);
      if (hasChestPose_) {
        humanChestPos = latestChestPositionInRobot_;
      }
    }
    incrementalController_->computeIncrementalChestPos(humanChestPos, true);
    if (latestPoseConstraintList_.size() > POSE_DATA_LIST_INDEX_CHEST) {
      latestPoseConstraintList_[POSE_DATA_LIST_INDEX_CHEST].position = incrementalController_->getLatestRobotChestPos();
    }
  }

  bool leftCanProcess = !leftMaintainProcess && currentLeftGripPressed;

  if (leftCanProcess) {
    leftCanProcess = detectLeftArmMove() && currentLeftGripPressed;
  }

  bool rightCanProcess = !rightMaintainProcess && currentRightGripPressed;

  if (rightCanProcess) {
    rightCanProcess = detectRightArmMove() && currentRightGripPressed;
  }

  bool isLeftActive = joyStickHandlerPtr_->isLeftArmCtrlModeActive();
  bool isRightActive = joyStickHandlerPtr_->isRightArmCtrlModeActive();

  if (leftCanProcess && isLeftActive) {
    latestIncrementalResult_ = incrementalController_->computeIncrementalPoseLeftArm(
        latestLeftHandPose_vr_, leftCanProcess && isLeftActive, leftEndEffectorQuat_);
  }
  if (rightCanProcess && isRightActive) {
    latestIncrementalResult_ = incrementalController_->computeIncrementalPoseRightArm(
        latestRightHandPose_vr_, rightCanProcess && isRightActive, rightEndEffectorQuat_);
  }

  // 任意手更新增量时，同步更新 chest 位置增量，并写入约束列表
  if ((leftCanProcess && isLeftActive) || (rightCanProcess && isRightActive)) {
    if (chestIncrementalUpdateEnabled_) {
      Eigen::Vector3d humanChestPos = Eigen::Vector3d::Zero();
      {
        std::lock_guard<std::mutex> lock(chestPoseMutex_);
        if (hasChestPose_) {
          humanChestPos = latestChestPositionInRobot_;
        }
      }
      incrementalController_->computeIncrementalChestPos(humanChestPos, true);
      if (latestPoseConstraintList_.size() > POSE_DATA_LIST_INDEX_CHEST) {
        latestPoseConstraintList_[POSE_DATA_LIST_INDEX_CHEST].position = incrementalController_->getLatestRobotChestPos();
      }
    }
  }

  latestIncrementalResult_ = incrementalController_->getLatestIncrementalResult();

  // Whole-body reference update (chest + L/R elbow/hand) and then solve IK once.
  FrozenRefs frozen;
  WholeBodyRefInput input =
      buildWholeBodyInput(leftCanProcess && isLeftActive, rightCanProcess && isRightActive, frozen);

  auto [incrementalLeftQuat, incrementalRightQuat, scaledLeftHandPos, scaledRightHandPos] =
      latestIncrementalResult_.getLatestIncrementalHandPose(true, useIncrementalHandOrientation_, true);

  recordTimestamp("applyWholeBodyAndSolveStart", loopSyncCount_);
  applyWholeBodyAndSolve(
      input, scaledLeftHandPos, incrementalLeftQuat, scaledRightHandPos, incrementalRightQuat, frozen);
  recordTimestamp("applyWholeBodyAndSolveFinish", loopSyncCount_);
}

void WheelQuest3IkIncrementalROS::handleGripRisingEdge(bool leftGripRisingEdge,
                                                  bool rightGripRisingEdge,
                                                  bool leftMaintainProcess,
                                                  bool rightMaintainProcess) {
  // 处理左臂 grip 上升沿：更新锚点，使增量归零
  if (leftGripRisingEdge && !leftMaintainProcess) {
    // On grip rising edge, defensively sync constraint list to current FK to avoid jumps.
    updateHandConstraintUnlocked(
        latestPoseConstraintList_, POSE_DATA_LIST_INDEX_LEFT_HAND, leftLink6Position_, leftLink6Quat_);
    updateElbowConstraintUnlocked(latestPoseConstraintList_, POSE_DATA_LIST_INDEX_LEFT_ELBOW, leftLink4Position_);

    incrementalController_->updateLeftArmPoseAnchor(latestLeftHandPose_vr_,
                                                    latestPoseConstraintList_,
                                                    leftEndEffectorPosition_,
                                                    leftEndEffectorQuat_,
                                                    leftLink4Quat_);
  }

  if (rightGripRisingEdge && !rightMaintainProcess) {
    // On grip rising edge, defensively sync constraint list to current FK to avoid jumps.
    updateHandConstraintUnlocked(
        latestPoseConstraintList_, POSE_DATA_LIST_INDEX_RIGHT_HAND, rightLink6Position_, rightLink6Quat_);
    updateElbowConstraintUnlocked(latestPoseConstraintList_, POSE_DATA_LIST_INDEX_RIGHT_ELBOW, rightLink4Position_);

    incrementalController_->updateRightArmPoseAnchor(latestRightHandPose_vr_,
                                                     latestPoseConstraintList_,
                                                     rightEndEffectorPosition_,
                                                     rightEndEffectorQuat_,
                                                     rightLink4Quat_);
  }
}

void WheelQuest3IkIncrementalROS::fsmExit() {
  if ((armControlMode_ == 1 && lastArmControlMode_ == 2) || (armControlMode_ == 0 && lastArmControlMode_ == 2)) {
    enterMode2ResetCounter_ = 0;

    if (exitMode2Counter_ < EXIT_MODE_2_EXECUTION_COUNT) {
      forceDeactivateAllArmCtrlMode();
      setLbArmQuickMode(0);
      // setControlMode(3);
      // sleep(2);
      // setControlMode(2);
      {
        std::lock_guard<std::mutex> jointLock(jointStateMutex_);
        lb_dq_.setZero();
        latest_lb_dq_.setZero();
        lowpass_lb_dq_.setZero();
      }
      {
        std::lock_guard<std::mutex> lock(lbLegMoveTimeMutex_);
        lbLegMoveStartTime_ = ros::Time(0);
      }

      updateHandConstraintUnlocked(
          latestPoseConstraintList_, POSE_DATA_LIST_INDEX_LEFT_HAND, leftLink6Position_, leftLink6Quat_);
      updateHandConstraintUnlocked(
          latestPoseConstraintList_, POSE_DATA_LIST_INDEX_RIGHT_HAND, rightLink6Position_, rightLink6Quat_);

      incrementalController_->enterIncrementalModeLeftArm(latestLeftHandPose_vr_,
                                                          latestPoseConstraintList_,
                                                          leftEndEffectorPosition_,
                                                          leftEndEffectorQuat_,
                                                          leftLink4Quat_);

      incrementalController_->enterIncrementalModeRightArm(latestRightHandPose_vr_,
                                                           latestPoseConstraintList_,
                                                           rightEndEffectorPosition_,
                                                           rightEndEffectorQuat_,
                                                           rightLink4Quat_);
      exitMode2Counter_++;
    }
  }

  const bool leftGripPressed = joyStickHandlerPtr_->isLeftGrip();
  const bool rightGripPressed = joyStickHandlerPtr_->isRightGrip();
  const bool bothGripReleased = !leftGripPressed && !rightGripPressed;

  if (bothGripReleased && incrementalController_ && incrementalController_->isIncrementalModeChest()) {
    Eigen::Vector3d humanChestPos = Eigen::Vector3d::Zero();
    {
      std::lock_guard<std::mutex> lock(chestPoseMutex_);
      if (hasChestPose_) {
        humanChestPos = latestChestPositionInRobot_;
      }
    }
      Eigen::Vector3d chestPos = hasLatestWaistYawFk_ ? latestWaistYawFkPos_ : robotFixedWaistYawPos_;
    Eigen::Matrix3d chestR = chestRotationQuaternion_.toRotationMatrix();
    if (latestPoseConstraintList_.size() > POSE_DATA_LIST_INDEX_CHEST) {
      latestPoseConstraintList_[POSE_DATA_LIST_INDEX_CHEST].position = chestPos;
      latestPoseConstraintList_[POSE_DATA_LIST_INDEX_CHEST].rotation_matrix = chestR;
      frozenRobotChestPos_ = latestPoseConstraintList_[POSE_DATA_LIST_INDEX_CHEST].position;
      if (latestPoseConstraintList_.size() > POSE_DATA_LIST_INDEX_LEFT_HAND) {
        frozenLeftHandHeightOffset_ =
            latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].position.z() - frozenRobotChestPos_.z();
      }
      if (latestPoseConstraintList_.size() > POSE_DATA_LIST_INDEX_RIGHT_HAND) {
        frozenRightHandHeightOffset_ =
            latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].position.z() - frozenRobotChestPos_.z();
      }
      if (latestPoseConstraintList_.size() > POSE_DATA_LIST_INDEX_LEFT_ELBOW) {
        frozenLeftElbowHeightOffset_ =
            latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_ELBOW].position.z() - frozenRobotChestPos_.z();
      }
      if (latestPoseConstraintList_.size() > POSE_DATA_LIST_INDEX_RIGHT_ELBOW) {
        frozenRightElbowHeightOffset_ =
            latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_ELBOW].position.z() - frozenRobotChestPos_.z();
      }
    }
    incrementalController_->updateChestAnchorOnExit(humanChestPos, latestPoseConstraintList_);
  }

  bool shouldExitIncrementalLeftArm = incrementalController_->shouldExitIncrementalModeLeftArm(leftGripPressed);
  bool shouldExitIncrementalRightArm = incrementalController_->shouldExitIncrementalModeRightArm(rightGripPressed);

  if (shouldExitIncrementalLeftArm) {
    incrementalController_->exitIncrementalModeLeftArm(latestLeftHandPose_vr_,
                                                       latestPoseConstraintList_,
                                                       leftEndEffectorPosition_,
                                                       leftEndEffectorQuat_,
                                                       leftLink4Quat_);
    {
      std::lock_guard<std::mutex> jointLock(jointStateMutex_);
      dq_.head(7).setZero();
      latest_dq_.head(7).setZero();
      lowpass_dq_.head(7).setZero();
    }
  }

  if (shouldExitIncrementalRightArm) {
    incrementalController_->exitIncrementalModeRightArm(latestRightHandPose_vr_,
                                                        latestPoseConstraintList_,
                                                        rightEndEffectorPosition_,
                                                        rightEndEffectorQuat_,
                                                        rightLink4Quat_);
    {
      std::lock_guard<std::mutex> jointLock(jointStateMutex_);
      dq_.tail(7).setZero();
      latest_dq_.tail(7).setZero();
      lowpass_dq_.tail(7).setZero();
    }
  }
  if (!shouldExitIncrementalLeftArm && !shouldExitIncrementalRightArm) return;
  // 松 grip 仅退出增量手臂；mode 1/2 下保持 WBC/MPC 外部控制，由 deactivateController 内 guard 双重保护
  if (armControlMode_.load() != 0) return;
  deactivateController();
}

void WheelQuest3IkIncrementalROS::solveIk() {
  std::vector<PoseData> poseConstraintListCopy;
  poseConstraintListCopy = latestPoseConstraintList_;

  auto startTime = std::chrono::high_resolution_clock::now();
  auto ikResult = oneStageIkEndEffectorPtr_->solveIK(poseConstraintListCopy, ctrlArmIdx_, jointMidValues_);
  auto endTime = std::chrono::high_resolution_clock::now();
  const auto durationUs = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count();
  const double durationMs = static_cast<double>(durationUs) / 1000.0;
  ROS_INFO_THROTTLE(1.0, "[WheelQuest3IkIncrementalROS] one-stage IK latest duration: %.3f ms (%ld us)", durationMs, static_cast<long>(durationUs));

  if (ikResult.isSuccess) {
    {
      std::lock_guard<std::mutex> lock(ikResultMutex_);
      latestIkSolution_ = ikResult.solution;
      hasValidIkSolution_ = true;

      // 当IK成功且size == 18时，保存前4个关节角度到ikLowerBodyJointCommand_
      if (latestIkSolution_.size() == 18) {
        // print extract ik command into lower body and upper body
        // ROS_INFO("[WheelQuest3IkIncrementalROS] extract ik command into lower body and upper body");
        ikLowerBodyJointCommand_ = latestIkSolution_.head(4);
        ikUpperBodyJointCommand_ = latestIkSolution_.tail(14);
      }
    }
  } else {
    ROS_ERROR("[WheelQuest3IkIncrementalROS] solveIk failed: %s", ikResult.solverLog.c_str());
  }
}

bool WheelQuest3IkIncrementalROS::validateVrPose(const ::ArmPose& currentPose, ::ArmPose& validatedPose, const std::string& side, bool isArmActive) {
  Eigen::Vector3d currentPos = currentPose.position;
  
  // 【关键修改】如果手臂未激活，直接通过，不进行跳变检测
  if (!isArmActive) {
    validatedPose = currentPose;
    return true;
  }
  
  // 选择对应的缓冲区和计数器
  Eigen::Vector3d* prev1 = nullptr;
  Eigen::Vector3d* prev2 = nullptr;
  int* count = nullptr;
  int* spikeCount = nullptr;
  ros::Time* spikeStartTime = nullptr;
  
  if (side == "Left") {
    prev1 = &leftHandPrev1_;
    prev2 = &leftHandPrev2_;
    count = &leftHandCount_;
    spikeCount = &leftHandSpikeCount_;
    spikeStartTime = &leftHandSpikeStartTime_;
  } else if (side == "Right") {
    prev1 = &rightHandPrev1_;
    prev2 = &rightHandPrev2_;
    count = &rightHandCount_;
    spikeCount = &rightHandSpikeCount_;
    spikeStartTime = &rightHandSpikeStartTime_;
  } else {
    ROS_ERROR("[WheelQuest3IkIncrementalROS] Invalid side parameter: %s", side.c_str());
    validatedPose = currentPose;
    return false;
  }
  
  (*count)++;
  
  // 初始化阶段：前3个点直接通过
  if (*count < 3) {
    if (*count == 1) {
      *prev1 = currentPos;
    } else if (*count == 2) {
      *prev2 = *prev1;
      *prev1 = currentPos;
    }
    validatedPose = currentPose;
    *spikeCount = 0;  // 重置跳变计数
    return true;
  }
  
  // 核心检测逻辑：检查当前点是否异常跳变
  // 规则：如果当前点同时偏离前两点（使用欧几里得距离），且前两点相近，则认为是异常跳变
  Eigen::Vector3d diff_prev1_vec = currentPos - *prev1;
  Eigen::Vector3d diff_prev2_vec = currentPos - *prev2;
  Eigen::Vector3d diff_prev_prev_vec = *prev1 - *prev2;
  
  // 使用欧几里得距离（3D空间距离）来判断跳变
  double dist_prev1 = diff_prev1_vec.norm();
  double dist_prev2 = diff_prev2_vec.norm();
  double dist_prev_prev = diff_prev_prev_vec.norm();
  
  // 如果当前点同时偏离前两点，且前两点相近，则认为是跳变
  bool isSpike = (dist_prev1 > SPIKE_THRESHOLD && 
                  dist_prev2 > SPIKE_THRESHOLD &&
                  dist_prev_prev < SPIKE_THRESHOLD * 0.2);
  
  ros::Time currentTime = ros::Time::now();
  
  // 【超时检测】如果跳变持续超过阈值时间，强制恢复
  bool forceRecover = false;
  if (isSpike) {
    if (spikeStartTime->isZero()) {
      // 第一次检测到跳变，记录开始时间
      *spikeStartTime = currentTime;
    } else {
      // 检查是否超时
      double elapsedTime = (currentTime - *spikeStartTime).toSec();
      if (elapsedTime > SPIKE_TIMEOUT_DURATION) {
        forceRecover = true;
        ROS_WARN_THROTTLE(1.0, "[WheelQuest3IkIncrementalROS] %s hand VR pose: timeout recovery triggered (%.3f seconds)",
                          side.c_str(), elapsedTime);
      }
    }
  } else {
    // 数据正常，重置跳变开始时间
    *spikeStartTime = ros::Time(0);
  }
  
  // 恢复机制：如果连续N帧都被判定为跳变，可能是快速正常运动，应该恢复
  // 或者超时恢复机制触发
  if (isSpike && !forceRecover) {
    (*spikeCount)++;
    if (*spikeCount >= SPIKE_RECOVERY_COUNT) {
      // 连续跳变次数达到阈值，认为是快速正常运动，恢复使用当前数据
      isSpike = false;
      *spikeCount = 0;  // 重置计数
      *spikeStartTime = ros::Time(0);  // 重置时间戳
      ROS_INFO_THROTTLE(1.0, "[WheelQuest3IkIncrementalROS] %s hand VR pose: continuous spikes detected, recovering (likely fast normal motion)",
                        side.c_str());
    } else {
      // 检测到跳变，使用前一个点替代
      validatedPose.position = *prev1;
      validatedPose.quaternion = currentPose.quaternion;  // 保持当前姿态
      ROS_WARN_THROTTLE(1.0, "[WheelQuest3IkIncrementalROS] %s hand VR pose spike detected (%d/%d), using previous position",
                        side.c_str(), *spikeCount, SPIKE_RECOVERY_COUNT);
    }
  } else {
    // 数据正常，或者超时恢复触发，使用当前数据
    if (forceRecover) {
      isSpike = false;
      *spikeCount = 0;  // 重置计数
      *spikeStartTime = ros::Time(0);  // 重置时间戳
    } else {
      // 数据正常，重置跳变计数
      *spikeCount = 0;
    }
    validatedPose = currentPose;
  }
  
  // 更新缓冲区
  *prev2 = *prev1;
  *prev1 = validatedPose.position;
  
  return !isSpike;
}

}  // namespace HighlyDynamic
