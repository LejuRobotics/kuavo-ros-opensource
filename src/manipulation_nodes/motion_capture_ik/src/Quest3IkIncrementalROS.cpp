#include "motion_capture_ik/Quest3IkIncrementalROS.h"

#include <drake/geometry/scene_graph.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/framework/diagram.h>
#include <drake/systems/framework/diagram_builder.h>
#include <kuavo_msgs/changeArmCtrlMode.h>  // 新增：手臂控制模式切换服务
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Int32.h>
#include <iomanip>  // 用于格式化输出（std::setprecision, std::fixed）

#include <leju_utils/define.hpp>
#include <leju_utils/math.hpp>
#include <leju_utils/RosMsgConvertor.hpp>

#include "motion_capture_ik/ArmControlBaseROS.h"
#include "motion_capture_ik/Quest3ArmInfoTransformer.h"
#include "motion_capture_ik/json.hpp"
#include "motion_capture_ik/IncrementalControlModule.h"
#include "motion_capture_ik/KeyFramesVisualizer.h"
#include "motion_capture_ik/JoyStickHandler.h"
namespace HighlyDynamic {
using namespace leju_utils::ros_msg_convertor;

Quest3IkIncrementalROS::Quest3IkIncrementalROS(ros::NodeHandle& nodeHandle,
                                               double publishRate,
                                               bool debugPrint,
                                               ArmIdx ctrlArmIdx)
    : ArmControlBaseROS(nodeHandle, publishRate, debugPrint), ctrlArmIdx_(ctrlArmIdx) {}

Quest3IkIncrementalROS::~Quest3IkIncrementalROS() {
  shouldStop_ = true;

  if (ikSolveThread_.joinable()) {
    ikSolveThread_.join();
  }
}

void Quest3IkIncrementalROS::run() {
  if (!incrementalController_) {
    ROS_ERROR(
        "[Quest3IkIncrementalROS] incrementalController_ is not initialized. Please ensure it is properly created "
        "before calling run().");
    return;
  }
  if (!twoStageTorsoIkPtr_) {
    ROS_ERROR(
        "[Quest3IkIncrementalROS] twoStageTorsoIkPtr_ is not initialized. Please ensure it is properly created "
        "before calling run().");
    return;
  }

  ikSolveThread_ = std::thread(&Quest3IkIncrementalROS::solveIkHandElbowThreadFuntion, this);
  ros::spin();
}

void Quest3IkIncrementalROS::solveIkHandElbowThreadFuntion() {
  ros::Rate rate(publishRate_);
  while (!shouldStop() && ros::ok()) {
    if ((armControlMode_ == 0 && lastArmControlMode_ == 0) || (armControlMode_ == 1 && lastArmControlMode_ == 0)) {
      reset();  // 机器人未激活 (0→0 或 0→1)，持续重置各类状态，确保进入系统时正常
      rate.sleep();
      continue;  // 机器人未激活，不进行后续流程
    }

    fsmEnter();
    fsmChange();
    fsmProcess();
    fsmExit();
    publishSensorDataArmJoints();
    publishEndEffectorControlData();

    rate.sleep();
  }
}

void Quest3IkIncrementalROS::fsmEnter() {
  if ((armControlMode_ == 1 && lastArmControlMode_ == 2) || (armControlMode_ == 0 && lastArmControlMode_ == 2)) {
    enterMode2ResetCounter_ = 0;

    if (exitMode2Counter_ < EXIT_MODE_2_EXECUTION_COUNT) {
      forceDeactivateAllArmCtrlMode();

      std::lock_guard<std::mutex> lock(poseConstraintListMutex_);
      incrementalController_->enterIncrementalMode(
          quest3ArmInfoTransformerPtr_->getLeftHandPose(),
          quest3ArmInfoTransformerPtr_->getRightHandPose(),
          ArmPose(Eigen::Vector3d(-0.3, 0.5, 0.16), Eigen::Quaterniond::Identity()),
          ArmPose(Eigen::Vector3d(-0.3, -0.5, 0.16), Eigen::Quaterniond::Identity()),
          latestPoseConstraintList_);
      exitMode2Counter_++;
    }
    return;
  }

  // 正常工作模式 Case 2: (0→2 或 1→2)
  if ((armControlMode_ == 2 && lastArmControlMode_ == 1) || (armControlMode_ == 2 && lastArmControlMode_ == 0)) {
    exitMode2Counter_ = 0;

    if (enterMode2ResetCounter_ < ENTER_MODE_2_RESET_COUNT) {
      // print entermode count
      std::cout << "[Quest3IkIncrementalROS] Enter mode 2 reset all states (including incrementalController): "
                << enterMode2ResetCounter_ << "/" << ENTER_MODE_2_RESET_COUNT << std::endl;
      // 使用初始化时保存的全零关节角度位姿（Link6），避免运行时频繁调用 FK
      Eigen::Vector3d currentLeftHandPos = initZeroLeftLink6Position_;
      Eigen::Vector3d currentRightHandPos = initZeroRightLink6Position_;
      Eigen::Quaterniond currentLeftHandQuat = Eigen::Quaterniond::Identity();
      Eigen::Quaterniond currentRightHandQuat = Eigen::Quaterniond::Identity();

      {
        std::lock_guard<std::mutex> lock(poseConstraintListMutex_);
        latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].position = currentLeftHandPos;
        latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].rotation_matrix =
            currentLeftHandQuat.toRotationMatrix();
        latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].position = currentRightHandPos;
        latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].rotation_matrix =
            currentRightHandQuat.toRotationMatrix();
      }

      if (incrementalController_) {
        incrementalController_->reset();
        incrementalController_->setHandQuatSeeds(currentLeftHandQuat, currentRightHandQuat);
      }

      // 【核心修复】重置关节角度 fhan 滤波状态，避免从 t1 时刻的旧关节角度开始过渡
      if (q_.size() == jointStateSize_ && dq_.size() == jointStateSize_) {
        q_.setZero();
        dq_.setZero();
      }

      {
        std::lock_guard<std::mutex> lock(ikResultMutex_);
        if (latestIkSolution_.size() == jointStateSize_) {
          latestIkSolution_.setZero();
        }
        hasValidIkSolution_ = false;
      }

      // 重置增量控制结果（Quest3IkIncrementalROS 类的成员变量）
      latestIncrementalResult_ = IncrementalPoseResult();

      enterMode2ResetCounter_++;
      ROS_INFO_THROTTLE(
          1.0,
          "[Quest3IkIncrementalROS] Enter mode 2 reset all states (including incrementalController): %d/%d",
          enterMode2ResetCounter_,
          ENTER_MODE_2_RESET_COUNT);
    }

    // 使用局部作用域加锁，保护 latestPoseConstraintList_ 的访问
    {
      std::lock_guard<std::mutex> lock(poseConstraintListMutex_);
      if (incrementalController_->shouldEnterIncrementalModeLeftArm() && detectLeftGripPressed()) {
        incrementalController_->enterIncrementalModeLeftArm(
            quest3ArmInfoTransformerPtr_->getLeftHandPose(),
            ArmPose(Eigen::Vector3d(-0.3, 0.5, 0.16), Eigen::Quaterniond::Identity()),
            latestPoseConstraintList_);
      }
      if (incrementalController_->shouldEnterIncrementalModeRightArm() && detectRightGripPressed()) {
        incrementalController_->enterIncrementalModeRightArm(
            quest3ArmInfoTransformerPtr_->getRightHandPose(),
            ArmPose(Eigen::Vector3d(-0.3, -0.5, 0.16), Eigen::Quaterniond::Identity()),
            latestPoseConstraintList_);
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
      std::cout << "[Quest3IkIncrementalROS] Mode 2 timeout duration: " << elapsedTime << "s" << std::endl;
      // 在超时时间内，强制停用所有手臂控制模式，确保可以进入 fsmChange 流程
      forceDeactivateAllArmCtrlMode();

      // 使用初始化时保存的全零关节角度位姿（Link6），避免运行时频繁调用 FK
      Eigen::Vector3d currentLeftHandPos = initZeroLeftLink6Position_;
      Eigen::Vector3d currentRightHandPos = initZeroRightLink6Position_;
      Eigen::Quaterniond currentLeftHandQuat = Eigen::Quaterniond::Identity();
      Eigen::Quaterniond currentRightHandQuat = Eigen::Quaterniond::Identity();

      {
        std::lock_guard<std::mutex> lock(poseConstraintListMutex_);
        latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].position = currentLeftHandPos;
        latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].rotation_matrix =
            currentLeftHandQuat.toRotationMatrix();
        latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].position = currentRightHandPos;
        latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].rotation_matrix =
            currentRightHandQuat.toRotationMatrix();
      }

      // 重置增量控制模块，清除可能被 fsmChange/fsmProcess 更新的 fhan 滤波状态
      if (incrementalController_) {
        incrementalController_->reset();
        incrementalController_->setHandQuatSeeds(currentLeftHandQuat, currentRightHandQuat);
      }

      // 重置关节角度 fhan 滤波状态
      if (q_.size() == jointStateSize_ && dq_.size() == jointStateSize_) {
        q_.setZero();
        dq_.setZero();
      }

      // 在超时时间内，执行进入增量模式（0→2 和 1→2 都需要）
      {
        std::lock_guard<std::mutex> lock(poseConstraintListMutex_);
        incrementalController_->enterIncrementalMode(
            quest3ArmInfoTransformerPtr_->getLeftHandPose(),
            quest3ArmInfoTransformerPtr_->getRightHandPose(),
            ArmPose(Eigen::Vector3d(-0.3, 0.5, 0.16), Eigen::Quaterniond::Identity()),
            ArmPose(Eigen::Vector3d(-0.3, -0.5, 0.16), Eigen::Quaterniond::Identity()),
            latestPoseConstraintList_);
      }
    }
  }
}

void Quest3IkIncrementalROS::fsmChange() {
  if (!incrementalController_->isIncrementalMode()) return;
  bool leftHandCtrlModeChanged = joyStickHandlerPtr_->hasLeftArmCtrlModeChanged();
  bool rightHandCtrlModeChanged = joyStickHandlerPtr_->hasRightArmCtrlModeChanged();

  auto [leftChangingMaintainUpdated, leftChangingInstantUpdated] =
      leftHandSmoother_->updateModeChangingStateIfNeeded(leftHandCtrlModeChanged);
  auto [rightChangingMaintainUpdated, rightChangingInstantUpdated] =
      rightHandSmoother_->updateModeChangingStateIfNeeded(rightHandCtrlModeChanged);

  if (!leftChangingMaintainUpdated && !rightChangingMaintainUpdated) {
    // fsmChange 结束后，调用 forceActivateAllArmCtrlMode（执行指定次数，增强鲁棒性）
    if (activateAllArmCtrlModeCounter_ < ACTIVATE_ALL_ARM_CTRL_MODE_COUNT) {
      forceActivateAllArmCtrlMode();
      kuavo_msgs::changeArmCtrlMode srv3;
      srv3.request.control_mode = static_cast<int>(1);
      enableWbcArmTrajectoryControlClient_.call(srv3);
      activateAllArmCtrlModeCounter_++;
    }
    return;  // 没有模式切换，直接返回
  }
  // print leftHandCtrlModeChanged and rightHandCtrlModeChanged
  std::cout << "[fsmChange]:"
            << "leftChangingMaintainUpdated: " << leftChangingMaintainUpdated << ", "
            << ", rightChangingMaintainUpdated: " << rightChangingMaintainUpdated << std::endl;

  if (!updateLatestIncrementalResult()) return;  // check update success
  if (!processChangingData(leftChangingMaintainUpdated, rightChangingMaintainUpdated)) return;
  solveIk();
  processVisual();
  activateController();
  publishJointStates();

  updateLeftHandChangingMode(leftHandSmoother_->getDefaultPosOnExit());
  updateRightHandChangingMode(rightHandSmoother_->getDefaultPosOnExit());
  // 重置激活计数器，为下次 fsmChange 结束后重新激活做准备
  activateAllArmCtrlModeCounter_ = 0;
}

void Quest3IkIncrementalROS::fsmProcess() {
  if (!incrementalController_->isIncrementalMode()) return;  // check enter success

  auto [leftMaintainProcess, leftInstantProcess] = leftHandSmoother_->getModeChangingState();
  auto [rightMaintainProcess, rightInstantProcess] = rightHandSmoother_->getModeChangingState();

  // 获取当前 grip 状态
  bool currentLeftGripPressed = detectLeftGripPressed();
  bool currentRightGripPressed = detectRightGripPressed();

  bool leftGripRisingEdge = currentLeftGripPressed && !lastLeftGripPressed_;
  bool rightGripRisingEdge = currentRightGripPressed && !lastRightGripPressed_;

  // 更新上一帧的 grip 状态（必须在使用完之后更新）
  lastLeftGripPressed_ = currentLeftGripPressed;
  lastRightGripPressed_ = currentRightGripPressed;

  // 处理左臂 grip 上升沿：更新锚点，使增量归零
  if (leftGripRisingEdge && !leftMaintainProcess) {
    Eigen::Vector3d currentRobotLeftHandPos;
    {
      std::lock_guard<std::mutex> lock(poseConstraintListMutex_);
      currentRobotLeftHandPos = latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].position;
    }
    incrementalController_->updateLeftArmAnchor(quest3ArmInfoTransformerPtr_->getLeftHandPose(),
                                                currentRobotLeftHandPos);
    // ROS_INFO("[Quest3IkIncrementalROS] Left grip rising edge detected, anchor updated");
  }

  // 处理右臂 grip 上升沿：更新锚点，使增量归零
  if (rightGripRisingEdge && !rightMaintainProcess) {
    Eigen::Vector3d currentRobotRightHandPos;
    {
      std::lock_guard<std::mutex> lock(poseConstraintListMutex_);
      currentRobotRightHandPos = latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].position;
    }
    incrementalController_->updateRightArmAnchor(quest3ArmInfoTransformerPtr_->getRightHandPose(),
                                                 currentRobotRightHandPos);
    // ROS_INFO("[Quest3IkIncrementalROS] Right grip rising edge detected, anchor updated");
  }

  // 预先判断左右臂是否可以处理（不在模式切换中 && 检测到移动 && grip按下）
  bool leftCanProcess = !leftMaintainProcess && currentLeftGripPressed;

  if (leftCanProcess) {
    leftCanProcess = detectLeftArmMove() && currentLeftGripPressed;
  }

  bool rightCanProcess = !rightMaintainProcess && currentRightGripPressed;

  if (rightCanProcess) {
    rightCanProcess = detectRightArmMove() && currentRightGripPressed;
  }

  // 如果两臂都不能处理，直接返回，避免不必要的计算
  if (!leftCanProcess && !rightCanProcess) {
    return;
  }

  // 获取双臂的激活状态（双击激活）
  bool isLeftActive = joyStickHandlerPtr_->isLeftArmCtrlModeActive();
  bool isRightActive = joyStickHandlerPtr_->isRightArmCtrlModeActive();

  // 统一使用 computeIncrementalPose()，根据 leftCanProcess/rightCanProcess 决定激活哪只手臂
  // IncrementalControlModule 的 fhan 滤波已修复：未激活的手臂不会更新滤波状态
  if (leftCanProcess && isLeftActive) {
    latestIncrementalResult_ = incrementalController_->computeIncrementalPoseLeftArm(
        quest3ArmInfoTransformerPtr_->getLeftHandPose(),
        ArmPose(Eigen::Vector3d(-0.3, 0.5, 0.16), Eigen::Quaterniond::Identity()),
        leftCanProcess && isLeftActive);
  }
  if (rightCanProcess && isRightActive) {
    latestIncrementalResult_ = incrementalController_->computeIncrementalPoseRightArm(
        quest3ArmInfoTransformerPtr_->getRightHandPose(),
        ArmPose(Eigen::Vector3d(-0.3, -0.5, 0.16), Eigen::Quaterniond::Identity()),
        rightCanProcess && isRightActive);
  }

  latestIncrementalResult_ = incrementalController_->getLatestIncrementalResult();

  // 根据控制模式选择合适的数据处理函数
  if (leftCanProcess && rightCanProcess) {
    processData();  // 双臂同时控制
  } else if (leftCanProcess) {
    // print left only in green
    // std::cout << "\033[32m[Quest3IkIncrementalROS] Left arm only\033[0m" << std::endl;
    processDataLeftArm();  // 只有左臂控制
  } else if (rightCanProcess) {
    // print right only in green
    // std::cout << "\033[32m[Quest3IkIncrementalROS] Right arm only\033[0m" << std::endl;
    processDataRightArm();  // 只有右臂控制
  }

  solveIk();
  processVisual();
  activateController();
  publishJointStates();
}

void Quest3IkIncrementalROS::fsmExit() {
  if (!incrementalController_->shouldExitIncrementalMode()) return;
  deactivateController();
  std::lock_guard<std::mutex> lock(poseConstraintListMutex_);
  incrementalController_->exitIncrementalMode(
      quest3ArmInfoTransformerPtr_->getLeftHandPose(),
      quest3ArmInfoTransformerPtr_->getRightHandPose(),
      ArmPose(Eigen::Vector3d(-0.3, 0.5, 0.16), Eigen::Quaterniond::Identity()),
      ArmPose(Eigen::Vector3d(-0.3, -0.5, 0.16), Eigen::Quaterniond::Identity()),
      latestPoseConstraintList_);
}

bool Quest3IkIncrementalROS::processChangingData(bool leftHandCtrlModeChanged, bool rightHandCtrlModeChanged) {
  std::shared_ptr<noitom_hi5_hand_udp_python::PoseInfoList> bonePoseHandElbowPtr;
  {
    std::lock_guard<std::mutex> lock(bonePoseHandElbowMutex_);
    bonePoseHandElbowPtr = HandPoseAndElbowPositonListPtr_;
  }

  if (bonePoseHandElbowPtr == nullptr) return false;
  if (bonePoseHandElbowPtr->poses.size() < 4) return false;

  auto [incrementalLeftQuat, incrementalRightQuat, scaledLeftHandPos, scaledRightHandPos] =
      latestIncrementalResult_.getLatestIncrementalHandPose();

  // 修复：当只有一只手切换状态时，保持另一只手位置不变，避免状态同步不一致导致的异常移动
  if (!leftHandCtrlModeChanged && rightHandCtrlModeChanged) {
    std::lock_guard<std::mutex> lock(poseConstraintListMutex_);
    scaledLeftHandPos = latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].position;
  }

  if (leftHandCtrlModeChanged && !rightHandCtrlModeChanged) {
    std::lock_guard<std::mutex> lock(poseConstraintListMutex_);
    scaledRightHandPos = latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].position;
  }

  bool isLeftArmCtrlModeActive = joyStickHandlerPtr_->isLeftArmCtrlModeActive();
  bool isRightArmCtrlModeActive = joyStickHandlerPtr_->isRightArmCtrlModeActive();

  //#########################################################################################
  // process left hand changing mode
  //#########################################################################################
  if (leftHandCtrlModeChanged && isLeftArmCtrlModeActive) {
    auto [leftMaintain, leftInstant] = leftHandSmoother_->getModeChangingState();
    if (leftInstant) {
      Eigen::Vector3d leftHandPosOnExit;
      {
        std::lock_guard<std::mutex> lock(poseConstraintListMutex_);
        leftHandPosOnExit = latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].position;
      }
      incrementalController_->updateLeftArmAnchor(quest3ArmInfoTransformerPtr_->getLeftHandPose(), leftHandPosOnExit);
    }

    bool leftInstantCopy = leftInstant;
    leftHandSmoother_->processActiveModeInterpolation(
        scaledLeftHandPos, leftInstantCopy, leftHandSmoother_->getDefaultPosOnExit(), "左臂");
    leftHandSmoother_->setModeChangingState(leftMaintain, leftInstantCopy);
  }

  if (leftHandCtrlModeChanged && !isLeftArmCtrlModeActive) {
    auto [leftMaintain, leftInstant] = leftHandSmoother_->getModeChangingState();
    bool leftInstantCopy = leftInstant;
    leftHandSmoother_->processInactiveModeInterpolation(
        scaledLeftHandPos, leftInstantCopy, leftHandSmoother_->getDefaultPosOnExit(), "左臂");
    leftHandSmoother_->setModeChangingState(leftMaintain, leftInstantCopy);
  }

  //#########################################################################################
  // process right hand changing mode
  //#########################################################################################
  if (rightHandCtrlModeChanged && isRightArmCtrlModeActive) {
    auto [rightMaintain, rightInstant] = rightHandSmoother_->getModeChangingState();
    if (rightInstant) {
      Eigen::Vector3d rightHandPosOnExit;
      {
        std::lock_guard<std::mutex> lock(poseConstraintListMutex_);
        rightHandPosOnExit = latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].position;
      }
      incrementalController_->updateRightArmAnchor(quest3ArmInfoTransformerPtr_->getRightHandPose(),
                                                   rightHandPosOnExit);
    }

    bool rightInstantCopy = rightInstant;
    rightHandSmoother_->processActiveModeInterpolation(
        scaledRightHandPos, rightInstantCopy, rightHandSmoother_->getDefaultPosOnExit(), "右臂");
    rightHandSmoother_->setModeChangingState(rightMaintain, rightInstantCopy);
  }

  if (rightHandCtrlModeChanged && !isRightArmCtrlModeActive) {
    auto [rightMaintain, rightInstant] = rightHandSmoother_->getModeChangingState();
    bool rightInstantCopy = rightInstant;
    rightHandSmoother_->processInactiveModeInterpolation(
        scaledRightHandPos, rightInstantCopy, rightHandSmoother_->getDefaultPosOnExit(), "右臂");
    rightHandSmoother_->setModeChangingState(rightMaintain, rightInstantCopy);
  }

  clipHandPositionsByAllConstraints(scaledLeftHandPos,
                                    scaledRightHandPos,
                                    robotLeftFixedShoulderPos_,
                                    robotRightFixedShoulderPos_,
                                    sphereRadiusLimit_,
                                    minReachableDistance_,
                                    leftCenter_,
                                    rightCenter_,
                                    0.2,
                                    boxMinBound_,
                                    boxMaxBound_,
                                    chestOffsetY_);

  updateConstraintList(scaledLeftHandPos,
                       incrementalLeftQuat,
                       //  Eigen::Quaterniond(0.975, 0, -0.225, 0).normalized(),
                       scaledRightHandPos,
                       incrementalRightQuat,
                       //  Eigen::Quaterniond(0.975, 0, -0.225, 0).normalized(),
                       Eigen::Vector3d(-0.3, 0.5, 0.16),
                       Eigen::Vector3d(-0.3, -0.5, 0.16));

  return true;
}

bool Quest3IkIncrementalROS::processChangingDataLeftArm(bool leftHandCtrlModeChanged) {
  std::shared_ptr<noitom_hi5_hand_udp_python::PoseInfoList> bonePoseHandElbowPtr;
  {
    std::lock_guard<std::mutex> lock(bonePoseHandElbowMutex_);
    bonePoseHandElbowPtr = HandPoseAndElbowPositonListPtr_;
  }

  if (bonePoseHandElbowPtr == nullptr) return false;
  if (bonePoseHandElbowPtr->poses.size() < 4) return false;

  auto [incrementalLeftQuat, incrementalRightQuat, scaledLeftHandPos, scaledRightHandPos] =
      latestIncrementalResult_.getLatestIncrementalHandPose();

  // 只处理左手，右手位置保持不变
  {
    std::lock_guard<std::mutex> lock(poseConstraintListMutex_);
    scaledRightHandPos = latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].position;
  }

  bool isLeftArmCtrlModeActive = joyStickHandlerPtr_->isLeftArmCtrlModeActive();

  //#########################################################################################
  // process left hand changing mode only
  //#########################################################################################
  if (leftHandCtrlModeChanged && isLeftArmCtrlModeActive) {
    auto [leftMaintain, leftInstant] = leftHandSmoother_->getModeChangingState();
    if (leftInstant) {
      Eigen::Vector3d leftHandPosOnExit;
      {
        std::lock_guard<std::mutex> lock(poseConstraintListMutex_);
        leftHandPosOnExit = latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].position;
      }
      incrementalController_->updateLeftArmAnchor(quest3ArmInfoTransformerPtr_->getLeftHandPose(), leftHandPosOnExit);
    }

    bool leftInstantCopy = leftInstant;
    leftHandSmoother_->processActiveModeInterpolation(
        scaledLeftHandPos, leftInstantCopy, leftHandSmoother_->getDefaultPosOnExit(), "左臂");
    leftHandSmoother_->setModeChangingState(leftMaintain, leftInstantCopy);
  }

  if (leftHandCtrlModeChanged && !isLeftArmCtrlModeActive) {
    auto [leftMaintain, leftInstant] = leftHandSmoother_->getModeChangingState();
    bool leftInstantCopy = leftInstant;
    leftHandSmoother_->processInactiveModeInterpolation(
        scaledLeftHandPos, leftInstantCopy, leftHandSmoother_->getDefaultPosOnExit(), "左臂");
    leftHandSmoother_->setModeChangingState(leftMaintain, leftInstantCopy);
  }

  // 应用位置约束
  clipHandPositionsByAllConstraints(scaledLeftHandPos,
                                    scaledRightHandPos,
                                    robotLeftFixedShoulderPos_,
                                    robotRightFixedShoulderPos_,
                                    sphereRadiusLimit_,
                                    minReachableDistance_,
                                    leftCenter_,
                                    rightCenter_,
                                    0.2,
                                    boxMinBound_,
                                    boxMaxBound_,
                                    chestOffsetY_);

  // 更新约束列表
  updateConstraintList(scaledLeftHandPos,
                       incrementalLeftQuat,
                       scaledRightHandPos,
                       incrementalRightQuat,
                       Eigen::Vector3d(-0.3, 0.5, 0.16),
                       Eigen::Vector3d(-0.3, -0.5, 0.16));

  return true;
}

bool Quest3IkIncrementalROS::processChangingDataRightArm(bool rightHandCtrlModeChanged) {
  std::shared_ptr<noitom_hi5_hand_udp_python::PoseInfoList> bonePoseHandElbowPtr;
  {
    std::lock_guard<std::mutex> lock(bonePoseHandElbowMutex_);
    bonePoseHandElbowPtr = HandPoseAndElbowPositonListPtr_;
  }

  if (bonePoseHandElbowPtr == nullptr) return false;
  if (bonePoseHandElbowPtr->poses.size() < 4) return false;

  auto [incrementalLeftQuat, incrementalRightQuat, scaledLeftHandPos, scaledRightHandPos] =
      latestIncrementalResult_.getLatestIncrementalHandPose();

  // 只处理右手，左手位置保持不变
  {
    std::lock_guard<std::mutex> lock(poseConstraintListMutex_);
    scaledLeftHandPos = latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].position;
  }

  bool isRightArmCtrlModeActive = joyStickHandlerPtr_->isRightArmCtrlModeActive();

  //#########################################################################################
  // process right hand changing mode only
  //#########################################################################################
  if (rightHandCtrlModeChanged && isRightArmCtrlModeActive) {
    auto [rightMaintain, rightInstant] = rightHandSmoother_->getModeChangingState();
    if (rightInstant) {
      Eigen::Vector3d rightHandPosOnExit;
      {
        std::lock_guard<std::mutex> lock(poseConstraintListMutex_);
        rightHandPosOnExit = latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].position;
      }
      incrementalController_->updateRightArmAnchor(quest3ArmInfoTransformerPtr_->getRightHandPose(),
                                                   rightHandPosOnExit);
    }

    bool rightInstantCopy = rightInstant;
    rightHandSmoother_->processActiveModeInterpolation(
        scaledRightHandPos, rightInstantCopy, rightHandSmoother_->getDefaultPosOnExit(), "右臂");
    rightHandSmoother_->setModeChangingState(rightMaintain, rightInstantCopy);
  }

  if (rightHandCtrlModeChanged && !isRightArmCtrlModeActive) {
    auto [rightMaintain, rightInstant] = rightHandSmoother_->getModeChangingState();
    bool rightInstantCopy = rightInstant;
    rightHandSmoother_->processInactiveModeInterpolation(
        scaledRightHandPos, rightInstantCopy, rightHandSmoother_->getDefaultPosOnExit(), "右臂");
    rightHandSmoother_->setModeChangingState(rightMaintain, rightInstantCopy);
  }

  // 应用位置约束
  clipHandPositionsByAllConstraints(scaledLeftHandPos,
                                    scaledRightHandPos,
                                    robotLeftFixedShoulderPos_,
                                    robotRightFixedShoulderPos_,
                                    sphereRadiusLimit_,
                                    minReachableDistance_,
                                    leftCenter_,
                                    rightCenter_,
                                    0.2,
                                    boxMinBound_,
                                    boxMaxBound_,
                                    chestOffsetY_);

  // 更新约束列表
  updateConstraintList(scaledLeftHandPos,
                       incrementalLeftQuat,
                       scaledRightHandPos,
                       incrementalRightQuat,
                       Eigen::Vector3d(-0.3, 0.5, 0.16),
                       Eigen::Vector3d(-0.3, -0.5, 0.16));

  return true;
}

bool Quest3IkIncrementalROS::processDataLeftArm() {
  std::shared_ptr<noitom_hi5_hand_udp_python::PoseInfoList> bonePoseHandElbowPtr;
  {
    std::lock_guard<std::mutex> lock(bonePoseHandElbowMutex_);
    bonePoseHandElbowPtr = HandPoseAndElbowPositonListPtr_;
  }

  if (bonePoseHandElbowPtr == nullptr) return false;
  if (bonePoseHandElbowPtr->poses.size() < 4) return false;

  auto [incrementalLeftQuat, incrementalRightQuat, scaledLeftHandPos, scaledRightHandPos] =
      latestIncrementalResult_.getLatestIncrementalHandPose();

  {
    std::lock_guard<std::mutex> lock(poseConstraintListMutex_);
    scaledRightHandPos = latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].position;
  }

  clipHandPositionsByAllConstraints(scaledLeftHandPos,
                                    scaledRightHandPos,
                                    robotLeftFixedShoulderPos_,
                                    robotRightFixedShoulderPos_,
                                    sphereRadiusLimit_,
                                    minReachableDistance_,
                                    leftCenter_,
                                    rightCenter_,
                                    0.2,
                                    boxMinBound_,
                                    boxMaxBound_,
                                    chestOffsetY_);
  // 默认位置不要clip
  scaledLeftHandPos = joyStickHandlerPtr_->isLeftArmCtrlModeActive() ? scaledLeftHandPos : defaultLeftHandPosOnExit_;
  updateLeftConstraintList(scaledLeftHandPos, incrementalLeftQuat, Eigen::Vector3d(-0.3, 0.5, 0.16));

  return true;
}

bool Quest3IkIncrementalROS::processDataRightArm() {
  std::shared_ptr<noitom_hi5_hand_udp_python::PoseInfoList> bonePoseHandElbowPtr;
  {
    std::lock_guard<std::mutex> lock(bonePoseHandElbowMutex_);
    bonePoseHandElbowPtr = HandPoseAndElbowPositonListPtr_;
  }

  if (bonePoseHandElbowPtr == nullptr) return false;
  if (bonePoseHandElbowPtr->poses.size() < 4) return false;

  auto [incrementalLeftQuat, incrementalRightQuat, scaledLeftHandPos, scaledRightHandPos] =
      latestIncrementalResult_.getLatestIncrementalHandPose();

  // 左手位置保持不变
  {
    std::lock_guard<std::mutex> lock(poseConstraintListMutex_);
    scaledLeftHandPos = latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].position;
  }

  clipHandPositionsByAllConstraints(scaledLeftHandPos,
                                    scaledRightHandPos,
                                    robotLeftFixedShoulderPos_,
                                    robotRightFixedShoulderPos_,
                                    sphereRadiusLimit_,
                                    minReachableDistance_,
                                    leftCenter_,
                                    rightCenter_,
                                    0.2,
                                    boxMinBound_,
                                    boxMaxBound_,
                                    chestOffsetY_);

  // 默认位置不要clip
  scaledRightHandPos =
      joyStickHandlerPtr_->isRightArmCtrlModeActive() ? scaledRightHandPos : defaultRightHandPosOnExit_;
  updateRightConstraintList(scaledRightHandPos, incrementalRightQuat, Eigen::Vector3d(-0.3, -0.5, 0.16));

  return true;
}

bool Quest3IkIncrementalROS::processData() {
  std::shared_ptr<noitom_hi5_hand_udp_python::PoseInfoList> bonePoseHandElbowPtr;
  {
    std::lock_guard<std::mutex> lock(bonePoseHandElbowMutex_);
    bonePoseHandElbowPtr = HandPoseAndElbowPositonListPtr_;
  }

  if (bonePoseHandElbowPtr == nullptr) return false;
  if (bonePoseHandElbowPtr->poses.size() < 4) return false;

  //[chest, l_hand, r_hand, l_elbow, r_elbow]

  auto [incrementalLeftQuat, incrementalRightQuat, scaledLeftHandPos, scaledRightHandPos] =
      latestIncrementalResult_.getLatestIncrementalHandPose();

  clipHandPositionsByAllConstraints(scaledLeftHandPos,
                                    scaledRightHandPos,
                                    robotLeftFixedShoulderPos_,
                                    robotRightFixedShoulderPos_,
                                    sphereRadiusLimit_,
                                    minReachableDistance_,
                                    leftCenter_,
                                    rightCenter_,
                                    0.2,
                                    boxMinBound_,
                                    boxMaxBound_,
                                    chestOffsetY_);

  scaledLeftHandPos = joyStickHandlerPtr_->isLeftGrip() ? scaledLeftHandPos : leftHandSmoother_->getDefaultPosOnExit();

  scaledRightHandPos =
      joyStickHandlerPtr_->isRightGrip() ? scaledRightHandPos : rightHandSmoother_->getDefaultPosOnExit();

  updateConstraintList(scaledLeftHandPos,
                       incrementalLeftQuat,
                       scaledRightHandPos,
                       incrementalRightQuat,
                       Eigen::Vector3d(-0.3, 0.5, 0.16),
                       Eigen::Vector3d(-0.3, -0.5, 0.16));

  return true;
}

void Quest3IkIncrementalROS::solveIk() {
  std::lock_guard<std::mutex> lock(poseConstraintListMutex_);
  auto ikResult = twoStageTorsoIkPtr_->solveIK(latestPoseConstraintList_, ctrlArmIdx_, jointMidValues_);

  if (ikResult.isSuccess) {
    {
      std::lock_guard<std::mutex> lock(ikResultMutex_);
      latestIkSolution_ = ikResult.solution;
      hasValidIkSolution_ = true;
    }
  }
}

void Quest3IkIncrementalROS::processVisual() {
  auto fkCallback = [this](Eigen::Vector3d& leftPos,
                           Eigen::Quaterniond& leftQuat,
                           Eigen::Vector3d& rightPos,
                           Eigen::Quaterniond& rightQuat) {
    std::shared_ptr<kuavo_msgs::sensorsData> currentSensorData = getSensorData();
    if (currentSensorData && currentSensorData->joint_data.joint_q.size() >= 12 + jointStateSize_) {
      Eigen::VectorXd armJoints(jointStateSize_);
      for (int i = 0; i < jointStateSize_; ++i) {
        armJoints(i) = currentSensorData->joint_data.joint_q[12 + i];
      }

      auto [leftMeasuredPosition, leftMeasuredQuaternion] =
          twoStageTorsoIkPtr_->FK(armJoints, "zarm_l7_end_effector", jointStateSize_);
      auto [rightMeasuredPosition, rightMeasuredQuaternion] =
          twoStageTorsoIkPtr_->FK(armJoints, "zarm_r7_end_effector", jointStateSize_);

      leftPos = leftMeasuredPosition;
      leftQuat = leftMeasuredQuaternion;
      rightPos = rightMeasuredPosition;
      rightQuat = rightMeasuredQuaternion;
    }
  };

  {
    std::lock_guard<std::mutex> lock(poseConstraintListMutex_);
    quest3KeyFramesVisualizerPtr_->publishAllVisualizations(robotLeftFixedShoulderPos_,
                                                            robotRightFixedShoulderPos_,
                                                            sphereRadiusLimit_,
                                                            latestIncrementalResult_,
                                                            latestPoseConstraintList_,
                                                            boxMinBound_,
                                                            boxMaxBound_,
                                                            chestOffsetY_,
                                                            leftCenter_,
                                                            rightCenter_,
                                                            0.2,
                                                            fkCallback);
  }
}

void Quest3IkIncrementalROS::activateController() {
  if (controllerActivated_.load()) return;

  if (!changeMobileCtrlModeClient_.exists()) return;
  if (!humanoidArmCtrlModeClient_.exists()) return;
  if (!changeArmCtrlModeClient_.exists()) return;

  ROS_INFO("[Quest3IkIncrementalROS] Activating controller");
  kuavo_msgs::changeArmCtrlMode srv1, srv2;

  srv1.request.control_mode = static_cast<int>(MpcRefUpdateMode::ENABLED_ARM);
  srv2.request.control_mode = static_cast<int>(KuavoArmCtrlMode::EXTERNAL_CONTROL);

  controllerActivated_.store(changeMobileCtrlModeClient_.call(srv1) && srv1.response.result &&  //
                             humanoidArmCtrlModeClient_.call(srv2) && srv2.response.result &&   //
                             changeArmCtrlModeClient_.call(srv2) && srv2.response.result &&     //
                             true);
}

void Quest3IkIncrementalROS::deactivateController() {
  if (!controllerActivated_.load()) return;
  if (!changeMobileCtrlModeClient_.exists()) return;
  if (!humanoidArmCtrlModeClient_.exists()) return;
  if (!changeArmCtrlModeClient_.exists()) return;

  kuavo_msgs::changeArmCtrlMode srv1, srv2;

  srv1.request.control_mode = static_cast<int>(MpcRefUpdateMode::DISABLED_ARM);
  srv2.request.control_mode = static_cast<int>(KuavoArmCtrlMode::ARM_FIXED);

  controllerActivated_.store(!(changeMobileCtrlModeClient_.call(srv1) && srv1.response.result &&  //
                               humanoidArmCtrlModeClient_.call(srv2) && srv2.response.result &&   //
                               changeArmCtrlModeClient_.call(srv2) && srv2.response.result &&     //
                               true));
}

void Quest3IkIncrementalROS::armModeCallback(const std_msgs::Int32::ConstPtr& msg) {
  int newMode = msg->data;
  int oldMode = armControlMode_.load();

  if (oldMode != newMode) {
    lastArmControlMode_.store(oldMode);
    armControlMode_.store(newMode);
    ROS_INFO("[Quest3IkIncrementalROS] Arm control mode changed from %d to %d", oldMode, newMode);

    // 记录进入 mode 2 的时间戳（0→2 和 1→2 都需要）
    if ((oldMode == 0 || oldMode == 1) && newMode == 2) {
      std::lock_guard<std::mutex> lock(mode2EnterTimeMutex_);
      mode2EnterTime_ = ros::Time::now();
      ROS_INFO("[Quest3IkIncrementalROS] Mode 2 entered at time: %.3f, timeout duration: %.1f seconds",
               mode2EnterTime_.toSec(),
               MODE_2_TIMEOUT_DURATION);
    }
  } else {
    armControlMode_.store(newMode);
  }
}

void Quest3IkIncrementalROS::publishJointStates() {
  Eigen::VectorXd armAngleLimited;
  {
    std::lock_guard<std::mutex> lock(ikResultMutex_);
    if (!hasValidIkSolution_) return;
    if (latestIkSolution_.size() != jointStateSize_) {
      latestIkSolution_ = Eigen::VectorXd::Zero(jointStateSize_);
      ROS_WARN(
          "Joint positions size (%zu) does not match expected size (%d)", latestIkSolution_.size(), jointStateSize_);
      return;
    }
    armAngleLimited = latestIkSolution_;  // 假设已经限制过角度
  }

  sensor_msgs::JointState jointStateMsg;
  jointStateMsg.header.stamp = ros::Time::now();
  jointStateMsg.position.resize(jointStateSize_);
  jointStateMsg.velocity.resize(jointStateSize_);
  jointStateMsg.effort.resize(jointStateSize_);
  jointStateMsg.name.resize(jointStateSize_);

  for (int i = 0; i < jointStateSize_; ++i) {
    jointStateMsg.name[i] = "arm_joint_" + std::to_string(i + 1);
  }

  Eigen::VectorXd finalArmAngles = armAngleLimited;  // 默认使用目标角度

  double fhan_h = 1.0 / publishRate_;
  double fhan_h0 = fhan_h * fhan_kh0_joint_;

  for (int i = 0; i < jointStateSize_; ++i) {
    double targetAngle = finalArmAngles(i);
    if (i == 5 || i == 6 || i == 12 || i == 13) {
      targetAngle = targetAngle * deltaScaleRPY_(1);
    }
    if (i <= 6 && detectLeftGripPressed()) {
      leju_utils::fhanStepForwardWithVelLimit(q_(i),                 // 滤波后的关节角度（输出）
                                              dq_(i),                // 滤波后的关节角速度（输出）
                                              targetAngle,           // 目标角度（输入）
                                              fhan_r_joint_,         // 加速度约束
                                              fhan_h,                // 时间步长
                                              fhan_h0,               // 平滑系数
                                              max_joint_velocity_);  // 最大速度限制
    }
    if (i > 6 && i <= 13 && detectRightGripPressed()) {
      leju_utils::fhanStepForwardWithVelLimit(q_(i),                 // 滤波后的关节角度（输出）
                                              dq_(i),                // 滤波后的关节角速度（输出）
                                              targetAngle,           // 目标角度（输入）
                                              fhan_r_joint_,         // 加速度约束
                                              fhan_h,                // 时间步长
                                              fhan_h0,               // 平滑系数
                                              max_joint_velocity_);  // 最大速度限制
    }
  }
  q_(5) = std::min(std::max(q_(5), 0.85 * mec_limit_lower_(5)), 0.85 * mec_limit_upper_(5));
  q_(6) = std::min(std::max(q_(6), 0.85 * mec_limit_lower_(6)), 0.85 * mec_limit_upper_(6));

  q_(12) = std::min(std::max(q_(12), 0.85 * mec_limit_lower_(13)), 0.85 * mec_limit_upper_(13));
  q_(13) = std::min(std::max(q_(13), 0.85 * mec_limit_lower_(14)), 0.85 * mec_limit_upper_(14));

  // {
  //   for (int i = 0; i < jointStateSize_; ++i) {
  //     double targetAngle = finalArmAngles(i);
  //     if (i == 5 || i == 6) {  // 放大末端关节角度
  //       targetAngle = targetAngle * deltaScaleRPY_(1);
  //       targetAngle = std::min(std::max(targetAngle, 0.85 * mec_limit_lower_(i)), 0.85 * mec_limit_upper_(i));
  //     }

  //     if (i == 12 || i == 13) {
  //       targetAngle = targetAngle * deltaScaleRPY_(1);
  //       targetAngle = std::min(std::max(targetAngle, 0.85 * mec_limit_lower_(i + 1)), 0.85 * mec_limit_upper_(i +
  //       1));
  //     }

  //     leju_utils::fhanStepForwardWithVelLimit(q_(i),                 // 滤波后的关节角度（输出）
  //                                             dq_(i),                // 滤波后的关节角速度（输出）
  //                                             targetAngle,           // 目标角度（输入）
  //                                             fhan_r_joint_,         // 加速度约束
  //                                             fhan_h,                // 时间步长
  //                                             fhan_h0,               // 平滑系数
  //                                             max_joint_velocity_);  // 最大速度限制
  //   }

  //   // 限制末端关节角度
  //   // q_(5) = std::min(std::max(q_(5), 0.85 * mec_limit_lower_(5)), 0.85 * mec_limit_upper_(5));
  //   // q_(6) = std::min(std::max(q_(6), 0.85 * mec_limit_lower_(6)), 0.85 * mec_limit_upper_(6));
  //   // q_(12) = std::min(std::max(q_(12), 0.85 * mec_limit_lower_(12)), 0.85 * mec_limit_upper_(12));
  //   // q_(13) = std::min(std::max(q_(13), 0.85 * mec_limit_lower_(14)), 0.85 * mec_limit_upper_(14));
  // }
  for (int i = 0; i < jointStateSize_; ++i) {
    jointStateMsg.position[i] = q_(i) * 180.0 / M_PI;
    jointStateMsg.velocity[i] = dq_(i) * 180.0 / M_PI;
    jointStateMsg.effort[i] = 0.0;
  }

  kuavoArmTrajCppPublisher_.publish(jointStateMsg);

  // 同步发布左右手pose
  {
    std::lock_guard<std::mutex> lock(poseConstraintListMutex_);
    if (latestPoseConstraintList_.size() > POSE_DATA_LIST_INDEX_LEFT_HAND) {
      // 发布左手pose
      geometry_msgs::Pose leftHandPoseMsg;
      const auto& leftHandPose = latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND];
      leftHandPoseMsg.position.x = leftHandPose.position.x();
      leftHandPoseMsg.position.y = leftHandPose.position.y();
      leftHandPoseMsg.position.z = leftHandPose.position.z();

      // 将旋转矩阵转换为四元数
      Eigen::Quaterniond leftHandQuat(leftHandPose.rotation_matrix);
      leftHandPoseMsg.orientation.x = leftHandQuat.x();
      leftHandPoseMsg.orientation.y = leftHandQuat.y();
      leftHandPoseMsg.orientation.z = leftHandQuat.z();
      leftHandPoseMsg.orientation.w = leftHandQuat.w();

      leftHandPosePublisher_.publish(leftHandPoseMsg);
    }

    if (latestPoseConstraintList_.size() > POSE_DATA_LIST_INDEX_RIGHT_HAND) {
      // 发布右手pose
      geometry_msgs::Pose rightHandPoseMsg;
      const auto& rightHandPose = latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND];
      rightHandPoseMsg.position.x = rightHandPose.position.x();
      rightHandPoseMsg.position.y = rightHandPose.position.y();
      rightHandPoseMsg.position.z = rightHandPose.position.z();

      // 将旋转矩阵转换为四元数
      Eigen::Quaterniond rightHandQuat(rightHandPose.rotation_matrix);
      rightHandPoseMsg.orientation.x = rightHandQuat.x();
      rightHandPoseMsg.orientation.y = rightHandQuat.y();
      rightHandPoseMsg.orientation.z = rightHandQuat.z();
      rightHandPoseMsg.orientation.w = rightHandQuat.w();

      rightHandPosePublisher_.publish(rightHandPoseMsg);
    }
  }
}

void Quest3IkIncrementalROS::publishSensorDataArmJoints() {
  std::shared_ptr<kuavo_msgs::sensorsData> currentSensorData = getSensorData();

  if (!currentSensorData) {
    ROS_WARN("[Quest3IkIncrementalROS] sensor_data_raw is None in publishSensorDataArmJoints");
    return;
  }

  if (currentSensorData->joint_data.joint_q.size() < 12 + jointStateSize_) {
    ROS_WARN("[Quest3IkIncrementalROS] Sensor data does not contain enough joint data. Expected at least %d, got %zu",
             12 + jointStateSize_,
             currentSensorData->joint_data.joint_q.size());
    return;
  }

  sensor_msgs::JointState jointStateMsg;
  jointStateMsg.header.stamp = ros::Time::now();
  jointStateMsg.position.resize(jointStateSize_);
  jointStateMsg.name.resize(jointStateSize_);

  // 设置关节名称
  for (int i = 0; i < jointStateSize_; ++i) {
    jointStateMsg.name[i] = "arm_joint_" + std::to_string(i + 1);
  }

  // 从传感器数据提取手臂关节角（从索引12开始），并转换为角度单位
  for (int i = 0; i < jointStateSize_; ++i) {
    double jointAngleRad = currentSensorData->joint_data.joint_q[12 + i];
    jointStateMsg.position[i] = jointAngleRad * 180.0 / M_PI;  // 转换为角度
  }

  sensorDataArmJointsPublisher_.publish(jointStateMsg);
}

void Quest3IkIncrementalROS::initialize(const nlohmann::json& configJson) {
  initializeBase(configJson);

  // 初始化pose约束列表
  latestPoseConstraintList_.resize(POSE_DATA_LIST_SIZE, PoseData());

  // 初始化机器人固定位置参数
  robotRightFixedShoulderPos_ = Eigen::Vector3d(-0.017499853, -0.29269999999999996, 0.4245);
  robotLeftFixedShoulderPos_ = Eigen::Vector3d(-0.017499853, 0.29269999999999996, 0.4245);

  // 初始化机器人手臂长度参数
  upperArmLength_ = 0.2845;  // 机器人上臂长度
  lowerArmLength_ = 0.23;    // 机器人下臂长度

  //从JSON配置读取手臂关节数量
  if (configJson.contains("NUM_ARM_JOINT")) {
    jointStateSize_ = configJson["NUM_ARM_JOINT"].get<int>();
    ROS_INFO("✅ [Quest3IkIncrementalROS] Set arm joints count from JSON: %d", jointStateSize_);
  } else {
    ROS_ERROR("❌ [Quest3IkIncrementalROS] 'NUM_ARM_JOINT' field not found in JSON configuration");
    throw std::runtime_error("Missing 'NUM_ARM_JOINT' field in JSON configuration");
  }

  // 初始化关节角度fhan滤波状态
  q_.resize(jointStateSize_);
  dq_.resize(jointStateSize_);
  q_.setZero();
  dq_.setZero();

  // TEST: 初始化关节限制中间值（硬编码，从URDF中提取）
  // 关节顺序：左臂7个(zarm_l1~l7) + 右臂7个(zarm_r1~r7) = 14个
  jointMidValues_.resize(jointStateSize_);

  jointMidValues_(0) = 0;                                               // zarm_l1_joint
  jointMidValues_(1) = (-0.349065850398866 + 2.0943951023932) / 2.0;    // zarm_l2_joint
  jointMidValues_(2) = (-1.5707963267949 + 1.5707963267949) / 2.0;      // zarm_l3_joint
  jointMidValues_(3) = (-2.61799387799149 + 0.0) / 2.0;                 // zarm_l4_joint
  jointMidValues_(4) = (-1.5707963267949 + 1.5707963267949) / 2.0;      // zarm_l5_joint
  jointMidValues_(5) = (-1.30899693899575 + 0.698131700797732) / 2.0;   // zarm_l6_joint
  jointMidValues_(6) = (-0.698131700797732 + 0.698131700797732) / 2.0;  // zarm_l7_joint
  // 右臂关节中间值
  jointMidValues_(7) = 0;                                                // zarm_r1_joint
  jointMidValues_(8) = (-2.0943951023932 + 0.349065850398866) / 2.0;     // zarm_r2_joint
  jointMidValues_(9) = (-1.5707963267949 + 1.5707963267949) / 2.0;       // zarm_r3_joint
  jointMidValues_(10) = (-2.61799387799149 + 0.0) / 2.0;                 // zarm_r4_joint
  jointMidValues_(11) = (-1.5707963267949 + 1.5707963267949) / 2.0;      // zarm_r5_joint
  jointMidValues_(12) = (-0.698131700797732 + 1.30899693899575) / 2.0;   // zarm_r6_joint
  jointMidValues_(13) = (-0.698131700797732 + 0.698131700797732) / 2.0;  // zarm_r7_joint

  // 从JSON配置构建URDF路径
  std::string urdfFilePath;
  if (configJson.contains("arm_urdf")) {
    std::string kuavo_assets_path = ros::package::getPath("kuavo_assets");
    std::string arm_urdf_relative = configJson["arm_urdf"].get<std::string>();
    urdfFilePath = kuavo_assets_path + "/models/" + arm_urdf_relative;
    ROS_INFO("✅ [Quest3IkIncrementalROS] Constructed URDF path from JSON: %s", urdfFilePath.c_str());
  } else {
    ROS_ERROR("❌ [Quest3IkIncrementalROS] 'arm_urdf' field not found in JSON configuration");
    throw std::runtime_error("Missing 'arm_urdf' field in JSON configuration");
  }

  // drake initialization
  auto diagramBuilder = std::make_unique<drake::systems::DiagramBuilder<double>>();
  auto [plant, sceneGraph] = drake::multibody::AddMultibodyPlantSceneGraph(diagramBuilder.get(), 0.0);

  drake::multibody::Parser parser(&plant);
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

  // 修改关节限位
  try {
    Eigen::VectorXd limit_lower = Eigen::VectorXd::Constant(1, -1.57);
    Eigen::VectorXd limit_upper = Eigen::VectorXd::Constant(1, 1.57);
    plant.GetMutableJointByName("zarm_l6_joint").set_position_limits(limit_lower, limit_upper);
    plant.GetMutableJointByName("zarm_r6_joint").set_position_limits(limit_lower, limit_upper);

    plant.GetMutableJointByName("zarm_l7_joint").set_position_limits(limit_lower, limit_upper);
    plant.GetMutableJointByName("zarm_r7_joint").set_position_limits(limit_lower, limit_upper);
    ROS_INFO("✅ [Quest3IkIncrementalROS] Successfully updated joint limits for zarm_l7/r7_joint");
  } catch (const std::exception& e) {
    ROS_WARN("❌ [Quest3IkIncrementalROS] Failed to update joint limits: %s", e.what());
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

  // TwoStageTorsoIK initialization
  std::vector<std::string> frameNames = loadFrameNamesFromConfig(configJson);
  auto defaultIkSolverConfig = IKSolverConfig();
  twoStageTorsoIkPtr_ = std::make_unique<HighlyDynamic::TwoStageTorsoIK>(&plant, frameNames, defaultIkSolverConfig);

  // 计算并保存 ArmJoint 为全零时的双手位姿，避免运行时频繁调用 FK
  Eigen::VectorXd armJoints = Eigen::VectorXd::Zero(jointStateSize_);

  // 计算 Link6 位姿（用于 IK 约束）
  auto [leftLink6Position, leftLink6Quaternion] = twoStageTorsoIkPtr_->FK(armJoints, "zarm_l6_link", jointStateSize_);
  auto [rightLink6Position, rightLink6Quaternion] = twoStageTorsoIkPtr_->FK(armJoints, "zarm_r6_link", jointStateSize_);
  initZeroLeftLink6Position_ = leftLink6Position;
  initZeroLeftLink6Orientation_ = leftLink6Quaternion;
  initZeroRightLink6Position_ = rightLink6Position;
  initZeroRightLink6Orientation_ = rightLink6Quaternion;

  // 计算 End Effector 位姿（用于可视化等）
  auto [leftEndEffectorPosition, leftEndEffectorQuaternion] =
      twoStageTorsoIkPtr_->FK(armJoints, "zarm_l7_end_effector", jointStateSize_);
  auto [rightEndEffectorPosition, rightEndEffectorQuaternion] =
      twoStageTorsoIkPtr_->FK(armJoints, "zarm_r7_end_effector", jointStateSize_);
  initZeroLeftEndEffectorPosition_ = leftEndEffectorPosition;
  initZeroLeftEndEffectorOrientation_ = leftEndEffectorQuaternion;
  initZeroRightEndEffectorPosition_ = rightEndEffectorPosition;
  initZeroRightEndEffectorOrientation_ = rightEndEffectorQuaternion;

  ROS_INFO(
      "[Quest3IkIncrementalROS] Initialized zero joint pose - Link6: left=[%.4f, %.4f, %.4f], right=[%.4f, %.4f, %.4f]",
      initZeroLeftLink6Position_.x(),
      initZeroLeftLink6Position_.y(),
      initZeroLeftLink6Position_.z(),
      initZeroRightLink6Position_.x(),
      initZeroRightLink6Position_.y(),
      initZeroRightLink6Position_.z());
  ROS_INFO(
      "[Quest3IkIncrementalROS] Initialized zero joint pose - EndEffector: left=[%.4f, %.4f, %.4f], right=[%.4f, %.4f, "
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

  // 初始化增量控制模块
  IncrementalControlConfig incrementalConfig;
  // 使用 nodeHandle_ (命名空间为 /quest3) 来读取参数，自动跳过节点前缀
  while (!nodeHandle_.hasParam("/ik_ros_uni_cpp_node/quest3/fhan_r")) {
    ROS_WARN("[Quest3IkIncrementalROS] Waiting for /quest3/fhan_r parameter");
    ros::Duration(0.1).sleep();
  }
  PARAM_AND_PRINT_FLOAT(nodeHandle_, "/ik_ros_uni_cpp_node/quest3/fhan_r", incrementalConfig.fhan_r, 900.0, 1);

  while (!nodeHandle_.hasParam("/ik_ros_uni_cpp_node/quest3/fhan_kh0")) {
    ROS_WARN("[Quest3IkIncrementalROS] Waiting for /quest3/fhan_kh0 parameter");
    ros::Duration(0.1).sleep();
  }
  PARAM_AND_PRINT_FLOAT(nodeHandle_, "/ik_ros_uni_cpp_node/quest3/fhan_kh0", incrementalConfig.fhan_kh0, 6.0, 1);

  while (!nodeHandle_.hasParam("/ik_ros_uni_cpp_node/quest3/delta_scale_x")) {
    ROS_WARN("[Quest3IkIncrementalROS] Waiting for /quest3/delta_scale_x parameter");
    ros::Duration(0.1).sleep();
  }
  while (!nodeHandle_.hasParam("/ik_ros_uni_cpp_node/quest3/delta_scale_y")) {
    ROS_WARN("[Quest3IkIncrementalROS] Waiting for /quest3/delta_scale_y parameter");
    ros::Duration(0.1).sleep();
  }
  while (!nodeHandle_.hasParam("/ik_ros_uni_cpp_node/quest3/delta_scale_z")) {
    ROS_WARN("[Quest3IkIncrementalROS] Waiting for /quest3/delta_scale_z parameter");
    ros::Duration(0.1).sleep();
  }
  double delta_scale_x, delta_scale_y, delta_scale_z;
  PARAM_AND_PRINT_FLOAT(nodeHandle_, "/ik_ros_uni_cpp_node/quest3/delta_scale_x", delta_scale_x, 1.0, 1);
  PARAM_AND_PRINT_FLOAT(nodeHandle_, "/ik_ros_uni_cpp_node/quest3/delta_scale_y", delta_scale_y, 1.0, 1);
  PARAM_AND_PRINT_FLOAT(nodeHandle_, "/ik_ros_uni_cpp_node/quest3/delta_scale_z", delta_scale_z, 1.0, 1);
  deltaScale_ = Eigen::Vector3d(delta_scale_x, delta_scale_y, delta_scale_z);
  incrementalConfig.deltaScale = deltaScale_;

  double delta_scale_roll, delta_scale_pitch, delta_scale_yaw;
  PARAM_AND_PRINT_FLOAT(nodeHandle_, "/ik_ros_uni_cpp_node/quest3/delta_scale_roll", delta_scale_roll, 1.0, 1);
  PARAM_AND_PRINT_FLOAT(nodeHandle_, "/ik_ros_uni_cpp_node/quest3/delta_scale_pitch", delta_scale_pitch, 1.0, 1);
  PARAM_AND_PRINT_FLOAT(nodeHandle_, "/ik_ros_uni_cpp_node/quest3/delta_scale_yaw", delta_scale_yaw, 1.0, 1);
  deltaScaleRPY_ = Eigen::Vector3d(delta_scale_roll, delta_scale_pitch, delta_scale_yaw);

  PARAM_AND_PRINT_FLOAT(nodeHandle_, "/ik_ros_uni_cpp_node/quest3/max_pos_diff", incrementalConfig.maxPosDiff, 0.45, 2);

  // 读取 arm_move_threshold 参数
  while (!nodeHandle_.hasParam("/ik_ros_uni_cpp_node/quest3/arm_move_threshold")) {
    ROS_WARN("[Quest3IkIncrementalROS] Waiting for /quest3/arm_move_threshold parameter");
    ros::Duration(0.1).sleep();
  }
  PARAM_AND_PRINT_FLOAT(
      nodeHandle_, "/ik_ros_uni_cpp_node/quest3/arm_move_threshold", incrementalConfig.armMoveThreshold, 0.01, 3);
  incrementalConfig.publishRate = publishRate_;

  // 读取关节角度fhan滤波参数
  PARAM_AND_PRINT_FLOAT(nodeHandle_, "/ik_ros_uni_cpp_node/quest3/fhan_r_joint", fhan_r_joint_, 900.0, 1);
  PARAM_AND_PRINT_FLOAT(nodeHandle_, "/ik_ros_uni_cpp_node/quest3/fhan_kh0_joint", fhan_kh0_joint_, 6.0, 1);
  PARAM_AND_PRINT_FLOAT(nodeHandle_, "/ik_ros_uni_cpp_node/quest3/max_joint_velocity", max_joint_velocity_, 1.0, 3);

  // 读取手部位置约束参数
  PARAM_AND_PRINT_FLOAT(nodeHandle_, "/ik_ros_uni_cpp_node/quest3/sphere_radius_limit", sphereRadiusLimit_, 0.5, 2);
  PARAM_AND_PRINT_FLOAT(
      nodeHandle_, "/ik_ros_uni_cpp_node/quest3/min_reachable_distance", minReachableDistance_, 0.08, 3);
  PARAM_AND_PRINT_FLOAT(
      nodeHandle_, "/ik_ros_uni_cpp_node/quest3/hand_changing_mode_threshold", hand_changing_mode_threshold_, 0.055, 3);

  // 读取 box 边界参数（向量形式）
  PARAM_AND_PRINT_VECTOR3D(nodeHandle_,
                           "/quest3/box_min_bound",
                           boxMinBound_,
                           Eigen::Vector3d(0.25, -0.5, 0.1),
                           2,
                           "[Quest3IkIncrementalROS]");
  PARAM_AND_PRINT_VECTOR3D(nodeHandle_,
                           "/quest3/box_max_bound",
                           boxMaxBound_,
                           Eigen::Vector3d(1.0, 0.5, 1.0),
                           2,
                           "[Quest3IkIncrementalROS]");

  // 读取胸部中线偏移量参数
  PARAM_AND_PRINT_FLOAT(nodeHandle_, "/ik_ros_uni_cpp_node/quest3/chest_offset_y_ax", chestOffsetY_, 0.0, 3);

  // 读取圆柱体约束中心参数
  PARAM_AND_PRINT_VECTOR3D(nodeHandle_,
                           "/quest3/left_center",  //
                           leftCenter_,
                           Eigen::Vector3d(0, 0.02, 0),
                           2,
                           "[Quest3IkIncrementalROS]");
  PARAM_AND_PRINT_VECTOR3D(nodeHandle_,
                           "/quest3/right_center",  //
                           rightCenter_,
                           Eigen::Vector3d(0, -0.02, 0),
                           2,
                           "[Quest3IkIncrementalROS]");

  // 读取退出时默认手部位置参数
  PARAM_AND_PRINT_VECTOR3D(nodeHandle_,
                           "/quest3/default_left_hand_pos_on_exit",
                           defaultLeftHandPosOnExit_,
                           Eigen::Vector3d(1.0, 1.0, 1.0),
                           2,
                           "[Quest3IkIncrementalROS]");
  PARAM_AND_PRINT_VECTOR3D(nodeHandle_,
                           "/quest3/default_right_hand_pos_on_exit",
                           defaultRightHandPosOnExit_,
                           Eigen::Vector3d(1.0, -1.0, 1.0),
                           2,
                           "[Quest3IkIncrementalROS]");

  incrementalController_ = std::make_unique<IncrementalControlModule>(
      std::shared_ptr<JoyStickHandler>(joyStickHandlerPtr_.get(), [](JoyStickHandler*) {}), incrementalConfig);

  quest3ArmInfoTransformerPtr_->setDeltaScale(deltaScale_);

  double left_roll, left_pitch, left_yaw;
  double right_roll, right_pitch, right_yaw;
  PARAM_AND_PRINT_FLOAT(nodeHandle_, "/ik_ros_uni_cpp_node/quest3/left_hand_quat_offset_roll", left_roll, 0.0, 3);
  PARAM_AND_PRINT_FLOAT(nodeHandle_, "/ik_ros_uni_cpp_node/quest3/left_hand_quat_offset_pitch", left_pitch, 0.0, 3);
  PARAM_AND_PRINT_FLOAT(nodeHandle_, "/ik_ros_uni_cpp_node/quest3/left_hand_quat_offset_yaw", left_yaw, 0.0, 3);
  PARAM_AND_PRINT_FLOAT(nodeHandle_, "/ik_ros_uni_cpp_node/quest3/right_hand_quat_offset_roll", right_roll, 0.0, 3);
  PARAM_AND_PRINT_FLOAT(nodeHandle_, "/ik_ros_uni_cpp_node/quest3/right_hand_quat_offset_pitch", right_pitch, 0.0, 3);
  PARAM_AND_PRINT_FLOAT(nodeHandle_, "/ik_ros_uni_cpp_node/quest3/right_hand_quat_offset_yaw", right_yaw, 0.0, 3);
  leftHandOffsetRpy_ << left_roll, left_pitch, left_yaw;
  rightHandOffsetRpy_ << right_roll, right_pitch, right_yaw;

  // 将RPY角度转换为四元数（ZYX顺序：先绕Z轴旋转yaw，再绕Y轴旋转pitch，最后绕X轴旋转roll）
  leftHandQuatOffset_ = Eigen::AngleAxisd(leftHandOffsetRpy_(2), Eigen::Vector3d::UnitZ()) *
                        Eigen::AngleAxisd(leftHandOffsetRpy_(1), Eigen::Vector3d::UnitY()) *
                        Eigen::AngleAxisd(leftHandOffsetRpy_(0), Eigen::Vector3d::UnitX());
  rightHandQuatOffset_ = Eigen::AngleAxisd(rightHandOffsetRpy_(2), Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(rightHandOffsetRpy_(1), Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(rightHandOffsetRpy_(0), Eigen::Vector3d::UnitX());

  // clipPositionBySphere(
  //     defaultLeftHandPosOnExit_, robotLeftFixedShoulderPos_, sphereRadiusLimit_, minReachableDistance_);
  // clipPositionByBox(defaultLeftHandPosOnExit_, boxMinBound_, boxMaxBound_);
  // clipPositionByChestMidline(defaultLeftHandPosOnExit_, true, chestOffsetY_);
  // clipPositionBySphere(
  //     defaultRightHandPosOnExit_, robotRightFixedShoulderPos_, sphereRadiusLimit_, minReachableDistance_);
  // clipPositionByBox(defaultRightHandPosOnExit_, boxMinBound_, boxMaxBound_);
  // clipPositionByChestMidline(defaultRightHandPosOnExit_, false, chestOffsetY_);

  // print clip result
  {
    std::ostringstream oss_left, oss_right;
    oss_left << defaultLeftHandPosOnExit_.transpose().format(
        Eigen::IOFormat(Eigen::FullPrecision, 0, ", ", ", ", "", "", "", ""));
    oss_right << defaultRightHandPosOnExit_.transpose().format(
        Eigen::IOFormat(Eigen::FullPrecision, 0, ", ", ", ", "", "", "", ""));
    ROS_INFO("[Quest3IkIncrementalROS] Left hand clip result: %s", oss_left.str().c_str());
    ROS_INFO("[Quest3IkIncrementalROS] Right hand clip result: %s", oss_right.str().c_str());
  }

  // 使用默认手部位置初始化 latestPoseConstraintList_，确保进入增量模式时能正确初始化
  Eigen::Quaterniond defaultHandQuat = Eigen::Quaterniond::Identity();
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].position = defaultLeftHandPosOnExit_;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].rotation_matrix = defaultHandQuat.toRotationMatrix();
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].position = defaultRightHandPosOnExit_;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].rotation_matrix = defaultHandQuat.toRotationMatrix();

  // 初始化手部平滑插值器
  leftHandSmoother_ = std::make_unique<HandSmoother>("左臂", "zarm_l6_link", defaultLeftHandPosOnExit_);
  rightHandSmoother_ = std::make_unique<HandSmoother>("右臂", "zarm_r6_link", defaultRightHandPosOnExit_);

  // 更新默认位置（在clip之后）
  leftHandSmoother_->setDefaultPosOnExit(defaultLeftHandPosOnExit_);
  rightHandSmoother_->setDefaultPosOnExit(defaultRightHandPosOnExit_);

  // 初始化增量模块的手部姿态种子，避免首次进入增量模式时从单位四元数开始插值
  if (incrementalController_) {
    incrementalController_->setHandQuatSeeds(defaultHandQuat, defaultHandQuat);
  }

  ROS_INFO("[Quest3IkIncrementalROS] Interpolation system initialized successfully");
}

void Quest3IkIncrementalROS::updateConstraintList(const Eigen::Vector3d& leftHandPos,
                                                  const Eigen::Quaterniond& leftHandQuat,
                                                  const Eigen::Vector3d& rightHandPos,
                                                  const Eigen::Quaterniond& rightHandQuat,
                                                  const Eigen::Vector3d& leftElbowPos,
                                                  const Eigen::Vector3d& rightElbowPos) {
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].position = leftHandPos;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].rotation_matrix = leftHandQuat.toRotationMatrix();

  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].position = rightHandPos;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].rotation_matrix = rightHandQuat.toRotationMatrix();

  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_ELBOW].rotation_matrix = Eigen::Matrix3d::Identity();
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_ELBOW].position = leftElbowPos;

  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_ELBOW].rotation_matrix = Eigen::Matrix3d::Identity();
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_ELBOW].position = rightElbowPos;
}

void Quest3IkIncrementalROS::updateLeftConstraintList(const Eigen::Vector3d& leftHandPos,
                                                      const Eigen::Quaterniond& leftHandQuat,
                                                      const Eigen::Vector3d& leftElbowPos) {
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].position = leftHandPos;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].rotation_matrix = leftHandQuat.toRotationMatrix();
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_ELBOW].rotation_matrix =
      Eigen::AngleAxisd(-M_PI / 6.0, Eigen::Vector3d::UnitY()).toRotationMatrix();
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_ELBOW].position = leftElbowPos;
}

void Quest3IkIncrementalROS::updateRightConstraintList(const Eigen::Vector3d& rightHandPos,
                                                       const Eigen::Quaterniond& rightHandQuat,
                                                       const Eigen::Vector3d& rightElbowPos) {
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].position = rightHandPos;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].rotation_matrix = rightHandQuat.toRotationMatrix();
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_ELBOW].rotation_matrix =
      Eigen::AngleAxisd(-M_PI / 6.0, Eigen::Vector3d::UnitY()).toRotationMatrix();
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_ELBOW].position = rightElbowPos;
}

bool Quest3IkIncrementalROS::detectHumanArmMove() {
  if (incrementalController_->hasHumanArmMoved()) return true;

  incrementalController_->detectHumanArmMove(quest3ArmInfoTransformerPtr_->getLeftHandPose().position,
                                             quest3ArmInfoTransformerPtr_->getRightHandPose().position);

  return incrementalController_->hasHumanArmMoved();
}

bool Quest3IkIncrementalROS::detectLeftArmMove() {
  // 检查左臂是否已经移动
  if (incrementalController_->hasLeftArmMoved()) {
    return true;
  }

  // 检测左臂移动
  incrementalController_->detectLeftArmMove(quest3ArmInfoTransformerPtr_->getLeftHandPose().position);

  return incrementalController_->hasLeftArmMoved();
}

bool Quest3IkIncrementalROS::detectRightArmMove() {
  // 检查右臂是否已经移动
  if (incrementalController_->hasRightArmMoved()) {
    return true;
  }

  // 检测右臂移动
  incrementalController_->detectRightArmMove(quest3ArmInfoTransformerPtr_->getRightHandPose().position);

  return incrementalController_->hasRightArmMoved();
}

bool Quest3IkIncrementalROS::detectLeftGripPressed() { return joyStickHandlerPtr_->isLeftGrip(); }

bool Quest3IkIncrementalROS::detectRightGripPressed() { return joyStickHandlerPtr_->isRightGrip(); }

bool Quest3IkIncrementalROS::updateLatestIncrementalResult() {
  bool isLeftActive = joyStickHandlerPtr_->isLeftArmCtrlModeActive();
  bool isRightActive = joyStickHandlerPtr_->isRightArmCtrlModeActive();
  latestIncrementalResult_ = incrementalController_->computeIncrementalPose(
      quest3ArmInfoTransformerPtr_->getLeftHandPose(),
      quest3ArmInfoTransformerPtr_->getRightHandPose(),
      ArmPose(Eigen::Vector3d(-0.3, 0.5, 0.16), Eigen::Quaterniond::Identity()),
      ArmPose(Eigen::Vector3d(-0.3, -0.5, 0.16), Eigen::Quaterniond::Identity()),
      isLeftActive,
      isRightActive);
  return latestIncrementalResult_.isValid;
}

void Quest3IkIncrementalROS::updateSmootheIntermidiateHandPos(const Eigen::Vector3d& leftHandPos,
                                                              const Eigen::Vector3d& rightHandPos,
                                                              bool leftHandCtrlModeChanged,
                                                              bool rightHandCtrlModeChanged) {
  // print leftHandPos and rightHandPos
  std::cout << "[Quest3IkIncrementalROS] updateSmootheIntermidiateHandPos leftHandPos: " << leftHandPos.transpose()
            << std::endl;
  std::cout << "[Quest3IkIncrementalROS] updateSmootheIntermidiateHandPos   rightHandPos: " << rightHandPos.transpose()
            << std::endl;

  if (leftHandCtrlModeChanged) {
    leftHandSmoother_->resetSmoothState(latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].position);
    // print reset left
    std::cout << "\033[92m[Quest3IkIncrementalROS] Reset left hand position\033[0m" << std::endl;
  }

  if (rightHandCtrlModeChanged) {
    rightHandSmoother_->resetSmoothState(latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].position);
    // print reset right
    std::cout << "\033[92m[Quest3IkIncrementalROS] Reset right hand position\033[0m" << std::endl;
  }

  // 使用fhan插值更新平滑状态
  leftHandSmoother_->updateWithFhan(leftHandPos, 0.5, 0.01, 0.04);
  rightHandSmoother_->updateWithFhan(rightHandPos, 0.5, 0.01, 0.04);
}

bool Quest3IkIncrementalROS::updateLeftHandChangingMode(const Eigen::Vector3d& leftTargetPos) {
  std::shared_ptr<kuavo_msgs::sensorsData> currentSensorData = getSensorData();
  if (!currentSensorData || currentSensorData->joint_data.joint_q.size() < 12 + jointStateSize_) {
    ROS_WARN("[Quest3IkIncrementalROS] Sensor data not available or insufficient for FK calculation");
    return false;
  }

  Eigen::VectorXd armJoints(jointStateSize_);
  for (int i = 0; i < jointStateSize_; ++i) {
    armJoints(i) = currentSensorData->joint_data.joint_q[12 + i];
  }

  return leftHandSmoother_->updateChangingMode(
      leftTargetPos, twoStageTorsoIkPtr_.get(), armJoints, jointStateSize_, hand_changing_mode_threshold_);
}

bool Quest3IkIncrementalROS::updateRightHandChangingMode(const Eigen::Vector3d& rightTargetPos) {
  std::shared_ptr<kuavo_msgs::sensorsData> currentSensorData = getSensorData();
  if (!currentSensorData || currentSensorData->joint_data.joint_q.size() < 12 + jointStateSize_) {
    ROS_WARN("[Quest3IkIncrementalROS] Sensor data not available or insufficient for FK calculation");
    return false;
  }

  Eigen::VectorXd armJoints(jointStateSize_);
  for (int i = 0; i < jointStateSize_; ++i) {
    armJoints(i) = currentSensorData->joint_data.joint_q[12 + i];
  }

  return rightHandSmoother_->updateChangingMode(
      rightTargetPos, twoStageTorsoIkPtr_.get(), armJoints, jointStateSize_, hand_changing_mode_threshold_);
}

void Quest3IkIncrementalROS::forceDeactivateAllArmCtrlMode() {
  // 强制停用所有手臂控制模式，确保可以进入 fsmChange 流程
  if (joyStickHandlerPtr_) {
    if (joyStickHandlerPtr_->isLeftArmCtrlModeActive()) {
      joyStickHandlerPtr_->forceSetLeftArmCtrlMode(false);
    }
    if (joyStickHandlerPtr_->isRightArmCtrlModeActive()) {
      joyStickHandlerPtr_->forceSetRightArmCtrlMode(false);
    }
  }
}

void Quest3IkIncrementalROS::forceActivateAllArmCtrlMode() {
  if (joyStickHandlerPtr_) {
    if (!joyStickHandlerPtr_->isLeftArmCtrlModeActive()) {
      joyStickHandlerPtr_->forceSetLeftArmCtrlMode(true);
    }
    if (!joyStickHandlerPtr_->isRightArmCtrlModeActive()) {
      joyStickHandlerPtr_->forceSetRightArmCtrlMode(true);
    }
  }
}

void Quest3IkIncrementalROS::reset() {
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
  // 重置增量控制模块的内部状态（包括控制模式、fhan滤波状态、手臂移动检测状态等）
  if (incrementalController_) {
    incrementalController_->reset();
  }
  // 重置 grip 状态跟踪变量，避免系统重置后出现错误的上升沿检测
  lastLeftGripPressed_ = false;
  lastRightGripPressed_ = false;
  // 重置激活所有手臂控制模式的计数器，确保下次进入增量模式时可以重新执行激活逻辑
  activateAllArmCtrlModeCounter_ = 0;
  // 重置退出 mode 2 的计数器，确保下次退出时可以重新执行过渡逻辑
  exitMode2Counter_ = 0;
  // 重置进入 mode 2 时的位置重置计数器，确保下次进入时可以重新执行位置重置逻辑
  enterMode2ResetCounter_ = 0;
  // 重置IK求解结果
  {
    std::lock_guard<std::mutex> lock(ikResultMutex_);
    if (latestIkSolution_.size() != jointStateSize_) {
      latestIkSolution_.resize(jointStateSize_);
    }
    latestIkSolution_.setZero();
    hasValidIkSolution_ = false;
  }
  // 重置pose约束列表，使用默认手部位置初始化，确保进入增量模式时能正确初始化到默认位置
  {
    std::lock_guard<std::mutex> lock(poseConstraintListMutex_);
    latestPoseConstraintList_.resize(POSE_DATA_LIST_SIZE, PoseData());
    Eigen::Quaterniond defaultHandQuat = Eigen::Quaterniond::Identity();
    // 使用默认手部位置初始化，与 IncrementalControlModule 中的硬编码默认值保持一致
    latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].position = defaultLeftHandPosOnExit_;
    latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].rotation_matrix = defaultHandQuat.toRotationMatrix();
    latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].position = defaultRightHandPosOnExit_;
    latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].rotation_matrix = defaultHandQuat.toRotationMatrix();
  }
  // 重置增量控制结果
  latestIncrementalResult_ = IncrementalPoseResult();
  // 重置关节角度fhan滤波状态
  // if (q_.size() == jointStateSize_ && dq_.size() == jointStateSize_) {
  //   q_.setZero();
  //   dq_.setZero();
  // }
  // 重置mode 2进入时间戳
  {
    std::lock_guard<std::mutex> lock(mode2EnterTimeMutex_);
    mode2EnterTime_ = ros::Time(0);
  }
  // 重置后同步增量控制模块的手部姿态种子
  if (incrementalController_) {
    Eigen::Quaterniond defaultHandQuat = Eigen::Quaterniond::Identity();
    incrementalController_->setHandQuatSeeds(defaultHandQuat, defaultHandQuat);
  }
}

}  // namespace HighlyDynamic
