#include "motion_capture_ik/Quest3IkIncrementalROS.h"

#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <drake/geometry/scene_graph.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/framework/diagram.h>
#include <drake/systems/framework/diagram_builder.h>
#include <kuavo_msgs/changeArmCtrlMode.h>
#include <kuavo_msgs/Float32MultiArrayStamped.h>
#include <kuavo_msgs/twoArmHandPose.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <iomanip>
#include <cmath>

#include <leju_utils/define.hpp>
#include <leju_utils/math.hpp>
#include <leju_utils/RosMsgConvertor.hpp>

#include "motion_capture_ik/ArmControlBaseROS.h"
#include "motion_capture_ik/Quest3ArmInfoTransformer.h"
#include "motion_capture_ik/json.hpp"
#include "motion_capture_ik/IncrementalControlModule.h"
#include "motion_capture_ik/KeyFramesVisualizer.h"
#include "motion_capture_ik/JoyStickHandler.h"

#include "DrakeElbowHandPointOpt.hpp"
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
  if (!oneStageIkEndEffectorPtr_) {
    ROS_ERROR(
        "[Quest3IkIncrementalROS] oneStageIkEndEffectorPtr_ is not initialized. Please ensure it is properly created "
        "before calling run().");
    return;
  }

  ikSolveThread_ = std::thread(&Quest3IkIncrementalROS::solveIkHandElbowThreadFunction, this);
  ros::spin();
}

void Quest3IkIncrementalROS::solveIkHandElbowThreadFunction() {
  ros::Rate rate(publishRate_);
  while (!shouldStop() && ros::ok()) {
    updateSensorArmJointMeanFromSensorData();

    if ((armControlMode_ == 0 && lastArmControlMode_ == 0) || (armControlMode_ == 1 && lastArmControlMode_ == 0)) {
      reset();  // 机器人未激活 (0→0 或 0→1)，持续重置各类状态，确保进入系统时正常

      // 运行 DrakeVelocityIKSolver 测试套件
      if (leftVelocityIkSolverPtr_) {
        // 使用 left shoulder 位置作为 p0 (zarm_l2_joint 位置: -0.017500, 0.292700, 0.424500)
        // static const Eigen::Vector3d leftShoulderP0(-0.017500, 0.292700, 0.424500);
        // static DrakeVelocityIKTestSuite testSuite(leftShoulderP0, 0.2837, 0.2335);
        // testSuite.test();
      }
      rate.sleep();
      continue;  // 机器人未激活，不进行后续流程
    }

    fsmEnter();
    fsmChange();
    fsmProcess();
    fsmExit();
    publishSensorDataArmJoints();
    publishEndEffectorControlData();

    processVisual();
    publishHandPoseFromTransformer();
    activateController();
    publishJointStates();

    rate.sleep();
  }
}

void Quest3IkIncrementalROS::updateSensorArmJointMeanFromSensorData() {
  // 从 sensorData 抽取 14 维双臂关节角（rad），并进行指数均值滤波：q = 0.99*q + 0.01*qnew
  const auto currentSensorData = getSensorData();
  if (!currentSensorData) {
    return;
  }

  const int armJointStartIndex = 12 + waist_dof_;  // 考虑腰部自由度
  const size_t requiredSize = static_cast<size_t>(armJointStartIndex + SENSOR_ARM_JOINT_DIM);
  if (currentSensorData->joint_data.joint_q.size() < requiredSize) {
    return;
  }

  Eigen::VectorXd qNew = Eigen::VectorXd::Zero(SENSOR_ARM_JOINT_DIM);
  for (int i = 0; i < SENSOR_ARM_JOINT_DIM; ++i) {
    qNew(i) = currentSensorData->joint_data.joint_q[armJointStartIndex + i];
  }

  if (sensorArmJointQ_.size() != SENSOR_ARM_JOINT_DIM) {
    sensorArmJointQ_ = Eigen::VectorXd::Zero(SENSOR_ARM_JOINT_DIM);
  }

  static constexpr double kKeep = 0.92;
  static constexpr double kNew = 0.08;
  sensorArmJointQ_ = kKeep * sensorArmJointQ_ + kNew * qNew;
}

void Quest3IkIncrementalROS::computeLeftEndEffectorFK(Eigen::Vector3d& pOut, Eigen::Quaterniond& qOut) {
  // 默认值
  pOut = Eigen::Vector3d::Zero();
  qOut = Eigen::Quaterniond::Identity();

  // 使用均值滤波后的关节数据
  if (sensorArmJointQ_.size() != jointStateSize_) {
    return;
  }

  Eigen::VectorXd armJoints = sensorArmJointQ_;
  auto [l7Position, l7Quaternion] = oneStageIkEndEffectorPtr_->FK(armJoints, "zarm_l7_end_effector", jointStateSize_);
  pOut = l7Position;
  qOut = l7Quaternion;
}

void Quest3IkIncrementalROS::computeRightEndEffectorFK(Eigen::Vector3d& pOut, Eigen::Quaterniond& qOut) {
  // 默认值
  pOut = Eigen::Vector3d::Zero();
  qOut = Eigen::Quaterniond::Identity();

  // 使用均值滤波后的关节数据
  if (sensorArmJointQ_.size() != jointStateSize_) {
    return;
  }

  Eigen::VectorXd armJoints = sensorArmJointQ_;
  auto [r7Position, r7Quaternion] = oneStageIkEndEffectorPtr_->FK(armJoints, "zarm_r7_end_effector", jointStateSize_);
  pOut = r7Position;
  qOut = r7Quaternion;
}

void Quest3IkIncrementalROS::computeLeftLink4FK(Eigen::Vector3d& pOut, Eigen::Quaterniond& qOut) {
  // 默认值
  pOut = Eigen::Vector3d::Zero();
  qOut = Eigen::Quaterniond::Identity();

  // 使用均值滤波后的关节数据
  if (sensorArmJointQ_.size() != jointStateSize_) {
    return;
  }

  Eigen::VectorXd armJoints = sensorArmJointQ_;
  auto [l4Position, l4Quaternion] = oneStageIkEndEffectorPtr_->FKElbow(armJoints, "zarm_l4_link", jointStateSize_);
  pOut = l4Position;
  qOut = l4Quaternion;
}

void Quest3IkIncrementalROS::computeRightLink4FK(Eigen::Vector3d& pOut, Eigen::Quaterniond& qOut) {
  // 默认值
  pOut = Eigen::Vector3d::Zero();
  qOut = Eigen::Quaterniond::Identity();

  // 使用均值滤波后的关节数据
  if (sensorArmJointQ_.size() != jointStateSize_) {
    return;
  }

  Eigen::VectorXd armJoints = sensorArmJointQ_;
  auto [r4Position, r4Quaternion] = oneStageIkEndEffectorPtr_->FKElbow(armJoints, "zarm_r4_link", jointStateSize_);
  pOut = r4Position;
  qOut = r4Quaternion;
}

void Quest3IkIncrementalROS::computeLeftLink6FK(Eigen::Vector3d& pOut, Eigen::Quaterniond& qOut) {
  // 默认值
  pOut = Eigen::Vector3d::Zero();
  qOut = Eigen::Quaterniond::Identity();

  // 使用均值滤波后的关节数据
  if (sensorArmJointQ_.size() != jointStateSize_) {
    return;
  }

  Eigen::VectorXd armJoints = sensorArmJointQ_;
  auto [l6Position, l6Quaternion] = oneStageIkEndEffectorPtr_->FK(armJoints, "zarm_l6_link", jointStateSize_);
  pOut = l6Position;
  qOut = l6Quaternion;
}

void Quest3IkIncrementalROS::computeRightLink6FK(Eigen::Vector3d& pOut, Eigen::Quaterniond& qOut) {
  // 默认值
  pOut = Eigen::Vector3d::Zero();
  qOut = Eigen::Quaterniond::Identity();

  // 使用均值滤波后的关节数据
  if (sensorArmJointQ_.size() != jointStateSize_) {
    return;
  }

  Eigen::VectorXd armJoints = sensorArmJointQ_;
  auto [r6Position, r6Quaternion] = oneStageIkEndEffectorPtr_->FK(armJoints, "zarm_r6_link", jointStateSize_);
  pOut = r6Position;
  qOut = r6Quaternion;
}

void Quest3IkIncrementalROS::fsmEnter() {
  if ((armControlMode_ == 1 && lastArmControlMode_ == 2) || (armControlMode_ == 0 && lastArmControlMode_ == 2)) {
    enterMode2ResetCounter_ = 0;

    if (exitMode2Counter_ < EXIT_MODE_2_EXECUTION_COUNT) {
      forceDeactivateAllArmCtrlMode();

      // 进入增量模式时先用 FK 计算有效末端姿态，避免传入默认单位四元数
      Eigen::Vector3d pLeft, pRight;
      Eigen::Quaterniond qLeft, qRight;
      computeLeftEndEffectorFK(pLeft, qLeft);
      computeRightEndEffectorFK(pRight, qRight);
      Eigen::Vector3d pLeftLink4, pRightLink4;
      Eigen::Quaterniond qLeftLink4, qRightLink4;
      computeLeftLink4FK(pLeftLink4, qLeftLink4);
      computeRightLink4FK(pRightLink4, qRightLink4);
      Eigen::Vector3d pLeftLink6, pRightLink6;
      Eigen::Quaterniond qLeftLink6, qRightLink6;
      computeLeftLink6FK(pLeftLink6, qLeftLink6);
      computeRightLink6FK(pRightLink6, qRightLink6);

      std::lock_guard<std::mutex> lock(poseConstraintListMutex_);
      // 【核心修复】在进入增量模式前，先更新 latestPoseConstraintList_ 为当前 FK 计算的 Link6 位置
      // 避免使用上次退出时保存的旧位置，导致跳变
      latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].position = pLeftLink6;
      latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].rotation_matrix = qLeftLink6.toRotationMatrix();
      latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].position = pRightLink6;
      latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].rotation_matrix = qRightLink6.toRotationMatrix();

      incrementalController_->enterIncrementalModeLeftArm(
          quest3ArmInfoTransformerPtr_->getLeftHandPose(), latestPoseConstraintList_, pLeft, qLeft, qLeftLink4);

      incrementalController_->enterIncrementalModeRightArm(
          quest3ArmInfoTransformerPtr_->getRightHandPose(), latestPoseConstraintList_, pRight, qRight, qRightLink4);
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
        incrementalController_->setHandQuatSeeds(
            currentLeftHandQuat, currentRightHandQuat, useIncrementalHandOrientation_);
      }

      // 【核心修复】重置关节角度 fhan 滤波状态，避免从 t1 时刻的旧关节角度开始过渡
      if (q_.size() == jointStateSize_ && dq_.size() == jointStateSize_) {
        q_.setZero();
        dq_.setZero();
      }
      if (latest_q_.size() == jointStateSize_ && latest_dq_.size() == jointStateSize_) {
        latest_q_.setZero();
        latest_dq_.setZero();
        lowpass_dq_.setZero();
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
      const bool shouldEnterLeft =
          incrementalController_->shouldEnterIncrementalModeLeftArm(joyStickHandlerPtr_->isLeftGrip());
      const bool shouldEnterRight =
          incrementalController_->shouldEnterIncrementalModeRightArm(joyStickHandlerPtr_->isRightGrip());

      if (shouldEnterLeft || shouldEnterRight) {
        Eigen::Vector3d pLeft, pRight;
        Eigen::Quaterniond qLeft, qRight;
        Eigen::Vector3d pLeftLink4, pRightLink4;
        Eigen::Quaterniond qLeftLink4, qRightLink4;
        Eigen::Vector3d pLeftLink6, pRightLink6;
        Eigen::Quaterniond qLeftLink6, qRightLink6;
        if (shouldEnterLeft) {
          computeLeftEndEffectorFK(pLeft, qLeft);
          computeLeftLink4FK(pLeftLink4, qLeftLink4);
          computeLeftLink6FK(pLeftLink6, qLeftLink6);
        }
        if (shouldEnterRight) {
          computeRightEndEffectorFK(pRight, qRight);
          computeRightLink4FK(pRightLink4, qRightLink4);
          computeRightLink6FK(pRightLink6, qRightLink6);
        }

        // 【核心修复】在进入增量模式前，先更新 latestPoseConstraintList_ 为当前 FK 计算的 Link6 位置
        // 避免使用上次退出时保存的旧位置，导致跳变
        if (shouldEnterLeft) {
          latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].position = pLeftLink6;
          latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].rotation_matrix = qLeftLink6.toRotationMatrix();
        }
        if (shouldEnterRight) {
          latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].position = pRightLink6;
          latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].rotation_matrix = qRightLink6.toRotationMatrix();
        }

        if (shouldEnterLeft) {
          incrementalController_->enterIncrementalModeLeftArm(
              quest3ArmInfoTransformerPtr_->getLeftHandPose(), latestPoseConstraintList_, pLeft, qLeft, qLeftLink4);
        }
        if (shouldEnterRight) {
          incrementalController_->enterIncrementalModeRightArm(
              quest3ArmInfoTransformerPtr_->getRightHandPose(), latestPoseConstraintList_, pRight, qRight, qRightLink4);
        }
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

      {
        std::lock_guard<std::mutex> lock(poseConstraintListMutex_);
        latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].position = initZeroLeftLink6Position_;
        latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].rotation_matrix = Eigen::Matrix3d::Identity();
        latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].position = initZeroRightLink6Position_;
        latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].rotation_matrix = Eigen::Matrix3d::Identity();
      }

      // 重置增量控制模块，清除可能被 fsmChange/fsmProcess 更新的 fhan 滤波状态
      if (incrementalController_) {
        incrementalController_->reset();
        incrementalController_->setHandQuatSeeds(
            Eigen::Quaterniond::Identity(), Eigen::Quaterniond::Identity(), useIncrementalHandOrientation_);
      }

      // 重置关节角度 fhan 滤波状态
      if (q_.size() == jointStateSize_ && dq_.size() == jointStateSize_) {
        q_.setZero();
        dq_.setZero();
      }
      if (latest_q_.size() == jointStateSize_ && latest_dq_.size() == jointStateSize_) {
        latest_q_.setZero();
        latest_dq_.setZero();
        lowpass_dq_.setZero();
      }

      // 在超时时间内，执行进入增量模式（0→2 和 1→2 都需要）
      {
        Eigen::Vector3d pLeft, pRight;
        Eigen::Quaterniond qLeft, qRight;
        computeLeftEndEffectorFK(pLeft, qLeft);
        computeRightEndEffectorFK(pRight, qRight);
        Eigen::Vector3d pLeftLink4, pRightLink4;
        Eigen::Quaterniond qLeftLink4, qRightLink4;
        computeLeftLink4FK(pLeftLink4, qLeftLink4);
        computeRightLink4FK(pRightLink4, qRightLink4);
        Eigen::Vector3d pLeftLink6, pRightLink6;
        Eigen::Quaterniond qLeftLink6, qRightLink6;
        computeLeftLink6FK(pLeftLink6, qLeftLink6);
        computeRightLink6FK(pRightLink6, qRightLink6);

        std::lock_guard<std::mutex> lock(poseConstraintListMutex_);
        // 【核心修复】在进入增量模式前，先更新 latestPoseConstraintList_ 为当前 FK 计算的 Link6 位置
        // 避免使用上次退出时保存的旧位置，导致跳变
        latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].position = pLeftLink6;
        latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].rotation_matrix = qLeftLink6.toRotationMatrix();
        latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].position = pRightLink6;
        latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].rotation_matrix = qRightLink6.toRotationMatrix();

        incrementalController_->enterIncrementalModeLeftArm(
            quest3ArmInfoTransformerPtr_->getLeftHandPose(), latestPoseConstraintList_, pLeft, qLeft, qLeftLink4);

        incrementalController_->enterIncrementalModeRightArm(
            quest3ArmInfoTransformerPtr_->getRightHandPose(), latestPoseConstraintList_, pRight, qRight, qRightLink4);
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
      srv3.request.control_mode = static_cast<int>(kuavo_msgs::changeArmCtrlMode::Request::ik_ultra_fast_mode);
      // srv3.request.control_mode = static_cast<int>(1);
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

  // 独立处理左右臂模式切换
  bool leftProcessed = false;
  bool rightProcessed = false;

  if (leftChangingMaintainUpdated) {
    leftProcessed = processChangingDataLeftArm(leftHandCtrlModeChanged);
  }

  if (rightChangingMaintainUpdated) {
    rightProcessed = processChangingDataRightArm(rightHandCtrlModeChanged);
  }

  // 只有当至少一个臂处理成功时才继续
  if (!leftProcessed && !rightProcessed) return;

  solveIk();
  // processVisual();
  // activateController();
  // publishJointStates();

  updateLeftHandChangingMode(leftHandSmoother_->getDefaultPosOnExit());
  updateRightHandChangingMode(rightHandSmoother_->getDefaultPosOnExit());
  // 重置激活计数器，为下次 fsmChange 结束后重新激活做准备
  activateAllArmCtrlModeCounter_ = 0;
}

void Quest3IkIncrementalROS::fsmProcess() {
  processTorsoControlLoop();

  if (!incrementalController_->isIncrementalMode()) return;

  // latestHumanLeftElbowPos_ = quest3ArmInfoTransformerPtr_->getLeftElbowPose().position;
  // latestHumanRightElbowPos_ = quest3ArmInfoTransformerPtr_->getRightElbowPose().position;

  // 使用基类统一锁保护 Transformer 读写，避免与 bonePosesCallback 并发更新冲突
  ArmPose vrLeftPose;
  ArmPose vrRightPose;
  {
    std::lock_guard<std::mutex> lock(transformerDataMutex_);
    vrLeftPose = quest3ArmInfoTransformerPtr_->getLeftHandPose();  // 值拷贝，不是引用
    vrRightPose = quest3ArmInfoTransformerPtr_->getRightHandPose(); // 值拷贝，不是引用
  }

  auto [leftMaintainProcess, leftInstantProcess] = leftHandSmoother_->getModeChangingState();
  auto [rightMaintainProcess, rightInstantProcess] = rightHandSmoother_->getModeChangingState();

  // 获取当前 grip 状态
  bool currentLeftGripPressed = joyStickHandlerPtr_->isLeftGrip();
  bool currentRightGripPressed = joyStickHandlerPtr_->isRightGrip();


  // 【数据校验】3点跳变检测（仅在手臂激活时应用）
  validateVrPose(vrLeftPose, vrLeftPose, "Left", currentLeftGripPressed);
  validateVrPose(vrRightPose, vrRightPose, "Right", currentRightGripPressed);

  // 处理左手 grip 超时机制（结合移动检测）
  {
    ros::Time currentTime = ros::Time::now();
    bool leftArmMoved = incrementalController_->hasLeftArmMoved();

    if (currentLeftGripPressed) {
      // 只有在检测到移动时才开始计时
      if (leftArmMoved) {
        ros::Time startTime;
        {
          std::lock_guard<std::mutex> lock(leftGripTimeMutex_);
          // 如果还没有开始计时，则开始计时
          if (leftGripStartTime_.isZero()) {
            leftGripStartTime_ = currentTime;
            leftGripTimeoutReached_.store(false);
          }
          startTime = leftGripStartTime_;
        }

        // 检查是否达到超时
        if (!startTime.isZero()) {
          double elapsedTime = (currentTime - startTime).toSec();
          if (elapsedTime >= GRIP_TIMEOUT_DURATION && !leftGripTimeoutReached_.load()) {
            leftGripTimeoutReached_.store(true);
            ROS_INFO("[Quest3IkIncrementalROS] Left grip timeout reached (%.3f seconds)", elapsedTime);
          }
        }
      } else {
        // 未检测到移动，重置时间戳（不开始计时）
        std::lock_guard<std::mutex> lock(leftGripTimeMutex_);
        if (!leftGripStartTime_.isZero()) {
          leftGripStartTime_ = ros::Time(0);
          leftGripTimeoutReached_.store(false);
        }
      }
    } else {
      // grip 释放，重置时间戳和布尔值
      if (lastLeftGripPressed_) {
        std::lock_guard<std::mutex> lock(leftGripTimeMutex_);
        leftGripStartTime_ = ros::Time(0);
        leftGripTimeoutReached_.store(false);
      }
    }
  }

  // 处理右手 grip 超时机制（结合移动检测）
  {
    ros::Time currentTime = ros::Time::now();
    bool rightArmMoved = incrementalController_->hasRightArmMoved();

    if (currentRightGripPressed) {
      // 只有在检测到移动时才开始计时
      if (rightArmMoved) {
        ros::Time startTime;
        {
          std::lock_guard<std::mutex> lock(rightGripTimeMutex_);
          // 如果还没有开始计时，则开始计时
          if (rightGripStartTime_.isZero()) {
            rightGripStartTime_ = currentTime;
            rightGripTimeoutReached_.store(false);
          }
          startTime = rightGripStartTime_;
        }

        // 检查是否达到超时
        if (!startTime.isZero()) {
          double elapsedTime = (currentTime - startTime).toSec();
          if (elapsedTime >= GRIP_TIMEOUT_DURATION && !rightGripTimeoutReached_.load()) {
            rightGripTimeoutReached_.store(true);
            ROS_INFO("[Quest3IkIncrementalROS] Right grip timeout reached (%.3f seconds)", elapsedTime);
          }
        }
      } else {
        // 未检测到移动，重置时间戳（不开始计时）
        std::lock_guard<std::mutex> lock(rightGripTimeMutex_);
        if (!rightGripStartTime_.isZero()) {
          rightGripStartTime_ = ros::Time(0);
          rightGripTimeoutReached_.store(false);
        }
      }
    } else {
      // grip 释放，重置时间戳和布尔值
      if (lastRightGripPressed_) {
        std::lock_guard<std::mutex> lock(rightGripTimeMutex_);
        rightGripStartTime_ = ros::Time(0);
        rightGripTimeoutReached_.store(false);
      }
    }
  }

  bool leftGripRisingEdge = currentLeftGripPressed && !lastLeftGripPressed_;
  bool rightGripRisingEdge = currentRightGripPressed && !lastRightGripPressed_;

  // 更新上一帧的 grip 状态（必须在使用完之后更新）
  lastLeftGripPressed_ = currentLeftGripPressed;
  lastRightGripPressed_ = currentRightGripPressed;

  // 处理左臂 grip 上升沿：更新锚点，使增量归零
  if (leftGripRisingEdge && !leftMaintainProcess) {
    // 【核心修复】在grip激活瞬间，先通过FK计算当前真实位置，然后通过速度IK求解器优化，
    // 确保 latestPoseConstraintList_ 中的位置是当前真实位置，避免跳变
    Eigen::Vector3d pLeftLink6;
    Eigen::Quaterniond qLeftLink6;
    computeLeftLink6FK(pLeftLink6, qLeftLink6);

    // 通过FK计算末端执行器在世界系下的姿态
    Eigen::Vector3d pEndEffector;
    Eigen::Quaterniond qEndEffector;
    computeLeftEndEffectorFK(pEndEffector, qEndEffector);
    Eigen::Vector3d pLink4;
    Eigen::Quaterniond qLink4;
    computeLeftLink4FK(pLink4, qLink4);

    // 调用速度IK求解器优化手部和肘部位置，确保位置一致性
    const auto [p1Optimized, p2Optimized] =
        leftVelocityIkSolverPtr_->solve(latestHumanLeftElbowPos_,  // p1Ref: 人肘部参考位置
                                        leftElbowFixedPoint_,      // p1Fixed: 肘部固定点
                                        pLeftLink6  // p2Ref: 手部参考位置（当前FK计算的Link6位置）
        );

    // 保存优化后的结果
    latestRobotLeftElbowPos_ = p1Optimized;

    // 更新 latestPoseConstraintList_ 为优化后的位置，确保锚点设置正确
    {
      std::lock_guard<std::mutex> lock(poseConstraintListMutex_);
      latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].position = p2Optimized;
      latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].rotation_matrix = qLeftLink6.toRotationMatrix();
      latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_ELBOW].position = p1Optimized;
    }

    incrementalController_->updateLeftArmPoseAnchor(
        vrLeftPose, latestPoseConstraintList_, pEndEffector, qEndEffector, qLink4);
  }

  // 处理右臂 grip 上升沿：更新锚点，使增量归零
  if (rightGripRisingEdge && !rightMaintainProcess) {
    // 【核心修复】在grip激活瞬间，先通过FK计算当前真实位置，然后通过速度IK求解器优化，
    // 确保 latestPoseConstraintList_ 中的位置是当前真实位置，避免跳变
    Eigen::Vector3d pRightLink6;
    Eigen::Quaterniond qRightLink6;
    computeRightLink6FK(pRightLink6, qRightLink6);

    // 通过FK计算末端执行器在世界系下的姿态
    Eigen::Vector3d pEndEffector;
    Eigen::Quaterniond qEndEffector;
    computeRightEndEffectorFK(pEndEffector, qEndEffector);
    Eigen::Vector3d pLink4;
    Eigen::Quaterniond qLink4;
    computeRightLink4FK(pLink4, qLink4);

    // 调用速度IK求解器优化手部和肘部位置，确保位置一致性
    const auto [p1Optimized, p2Optimized] =
        rightVelocityIkSolverPtr_->solve(latestHumanRightElbowPos_,  // p1Ref: 人肘部参考位置
                                         rightElbowFixedPoint_,      // p1Fixed: 肘部固定点
                                         pRightLink6  // p2Ref: 手部参考位置（当前FK计算的Link6位置）
        );

    // 保存优化后的结果
    latestRobotRightElbowPos_ = p1Optimized;

    // 更新 latestPoseConstraintList_ 为优化后的位置，确保锚点设置正确
    {
      std::lock_guard<std::mutex> lock(poseConstraintListMutex_);
      latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].position = p2Optimized;
      latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].rotation_matrix = qRightLink6.toRotationMatrix();
      latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_ELBOW].position = p1Optimized;
    }

    // 【统一更新入口】使用 fsmProcess 开始时获取的 vrRightPose，而不是再次调用 getRightHandPose()
    incrementalController_->updateRightArmPoseAnchor(vrRightPose,
                                                     latestPoseConstraintList_,
                                                     pEndEffector,
                                                     qEndEffector,
                                                     qLink4);
  }

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

  // 计算ee的fk值用于实时更新
  Eigen::Vector3d pLeftEndEffector, pRightEndEffector;
  Eigen::Quaterniond qLeftEndEffector, qRightEndEffector;
  if (leftCanProcess && isLeftActive) {
    computeLeftEndEffectorFK(pLeftEndEffector, qLeftEndEffector);
  }
  if (rightCanProcess && isRightActive) {
    computeRightEndEffectorFK(pRightEndEffector, qRightEndEffector);
  }

  // 统一使用 computeIncrementalPose()，根据 leftCanProcess/rightCanProcess 决定激活哪只手臂
  if (leftCanProcess && isLeftActive) {
    latestIncrementalResult_ = incrementalController_->computeIncrementalPoseLeftArm(
        vrLeftPose, leftCanProcess && isLeftActive, qLeftEndEffector);
  }
  if (rightCanProcess && isRightActive) {
    latestIncrementalResult_ = incrementalController_->computeIncrementalPoseRightArm(
        vrRightPose, rightCanProcess && isRightActive, qRightEndEffector);
  }

  latestIncrementalResult_ = incrementalController_->getLatestIncrementalResult();

  // 独立处理左右臂数据
  bool leftProcessed = false;
  bool rightProcessed = false;

  if (leftCanProcess) {
    // print left only in green
    // std::cout << "\033[32m[Quest3IkIncrementalROS] Left arm only\033[0m" << std::endl;
    leftProcessed = processDataLeftArm();  // 左臂控制
  }

  if (rightCanProcess) {
    // print right only in green
    // std::cout << "\033[32m[Quest3IkIncrementalROS] Right arm only\033[0m" << std::endl;
    rightProcessed = processDataRightArm();  // 右臂控制
  }

  // 只有当至少一个臂处理成功时才继续
  if (!leftProcessed && !rightProcessed) {
    return;
  }

  solveIk();
  // processVisual();
  // activateController();
  // publishJointStates();
}

void Quest3IkIncrementalROS::fsmExit() {
  std::lock_guard<std::mutex> lock(poseConstraintListMutex_);
  bool shouldExitIncrementalLeftArm =
      incrementalController_->shouldExitIncrementalModeLeftArm(joyStickHandlerPtr_->isLeftGrip());
  bool shouldExitIncrementalRightArm =
      incrementalController_->shouldExitIncrementalModeRightArm(joyStickHandlerPtr_->isRightGrip());

  if (shouldExitIncrementalLeftArm) {
    Eigen::Vector3d pEndEffector;
    Eigen::Quaterniond qEndEffector;
    computeLeftEndEffectorFK(pEndEffector, qEndEffector);
    Eigen::Vector3d pLink4;
    Eigen::Quaterniond qLink4;
    computeLeftLink4FK(pLink4, qLink4);
    incrementalController_->exitIncrementalModeLeftArm(
        quest3ArmInfoTransformerPtr_->getLeftHandPose(), latestPoseConstraintList_, pEndEffector, qEndEffector, qLink4);
    dq_.head(7).setZero();
    latest_dq_.head(7).setZero();
    lowpass_dq_.head(7).setZero();
  }

  if (shouldExitIncrementalRightArm) {
    Eigen::Vector3d pEndEffector;
    Eigen::Quaterniond qEndEffector;
    computeRightEndEffectorFK(pEndEffector, qEndEffector);
    Eigen::Vector3d pLink4;
    Eigen::Quaterniond qLink4;
    computeRightLink4FK(pLink4, qLink4);
    incrementalController_->exitIncrementalModeRightArm(quest3ArmInfoTransformerPtr_->getRightHandPose(),
                                                        latestPoseConstraintList_,
                                                        pEndEffector,
                                                        qEndEffector,
                                                        qLink4);
    dq_.tail(7).setZero();
    latest_dq_.tail(7).setZero();
    lowpass_dq_.tail(7).setZero();
  }
  if (!shouldExitIncrementalLeftArm && !shouldExitIncrementalRightArm) return;
  deactivateController();
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
      latestIncrementalResult_.getLatestIncrementalHandPose(true, useIncrementalHandOrientation_, true);

  // scaledLeftHandPos = scaledLeftHandPos.cwiseProduct(deltaScale_);
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
      // 【核心修复】在模式切换瞬间，先通过FK计算当前真实位置，然后通过速度IK求解器优化，
      // 确保 latestPoseConstraintList_ 中的位置是当前真实位置，避免跳变
      Eigen::Vector3d pLeftLink6;
      Eigen::Quaterniond qLeftLink6;
      computeLeftLink6FK(pLeftLink6, qLeftLink6);

      // 通过FK计算末端执行器在世界系下的姿态
      Eigen::Vector3d pEndEffector;
      Eigen::Quaterniond qEndEffector;
      computeLeftEndEffectorFK(pEndEffector, qEndEffector);
      Eigen::Vector3d pLink4;
      Eigen::Quaterniond qLink4;
      computeLeftLink4FK(pLink4, qLink4);

      // 调用速度IK求解器优化手部和肘部位置，确保位置一致性
      const auto [p1Optimized, p2Optimized] =
          leftVelocityIkSolverPtr_->solve(latestHumanLeftElbowPos_,  // p1Ref: 人肘部参考位置
                                          leftElbowFixedPoint_,      // p1Fixed: 肘部固定点
                                          pLeftLink6  // p2Ref: 手部参考位置（当前FK计算的Link6位置）
          );

      // 保存优化后的结果
      latestRobotLeftElbowPos_ = p1Optimized;

      // 更新 latestPoseConstraintList_ 为优化后的位置，确保锚点设置正确
      {
        std::lock_guard<std::mutex> lock(poseConstraintListMutex_);
        latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].position = p2Optimized;
        latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].rotation_matrix = qLeftLink6.toRotationMatrix();
        latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_ELBOW].position = p1Optimized;
      }

      incrementalController_->updateLeftArmPoseAnchor(quest3ArmInfoTransformerPtr_->getLeftHandPose(),
                                                      latestPoseConstraintList_,
                                                      pEndEffector,
                                                      qEndEffector,
                                                      qLink4);
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

  // 通过FK计算左右肘部实时位置
  // CZJTODO ELBOW 需要平滑，否则容易导致震荡发散，因为目前hand pose 和 elbow pos的因果耦合
  Eigen::Vector3d currentLeftElbowPos, currentRightElbowPos;
  Eigen::Quaterniond qLeftElbow, qRightElbow;
  computeLeftLink4FK(currentLeftElbowPos, qLeftElbow);
  computeRightLink4FK(currentRightElbowPos, qRightElbow);

  // 应用位置约束
  // clipHandPositionsByAllConstraints(scaledLeftHandPos,
  //                                   scaledRightHandPos,
  //                                   robotLeftFixedShoulderPos_,
  //                                   robotRightFixedShoulderPos_,
  //                                   sphereRadiusLimit_,
  //                                   minReachableDistance_,
  //                                   leftCenter_,
  //                                   rightCenter_,
  //                                   0.23,
  //                                   boxMinBound_,
  //                                   boxMaxBound_,
  //                                   chestOffsetY_,
  //                                   currentLeftElbowPos,
  //                                   currentRightElbowPos,
  //                                   elbowMinDistance_,
  //                                   elbowMaxDistance_);

  // 保存优化前的手部位置（用于可视化对比）
  latestLeftHandPosBeforeOpt_ = scaledLeftHandPos;

  // 使用上一时刻的link6位置和ee位置计算期望的肘部位置
  // 通过手腕-ee向量计算期望的肘部位置
  // 向量方向：ee→手腕向量方向，长度为肘部到手腕的距离l2
  // 肘点计算：肘点 = 手腕点+向量方向*长度l2
  {
    // 计算从ee到手腕的向量（ee→手腕方向）
    Eigen::Vector3d eeToWristVec = leftLink6Position_ - leftEndEffectorPosition_;
    double vecNorm = eeToWristVec.norm();

    // 如果向量长度太小，使用默认方向（避免除零错误）
    if (vecNorm > 1e-6) {
      // 归一化向量并乘以l2_长度
      Eigen::Vector3d direction = eeToWristVec.normalized() * l2_;
      // 肘点 = 手腕点 + 向量方向*长度l2
      latestHumanLeftElbowPos_ = leftLink6Position_ + direction;
    } else {
      // 如果向量为零，使用上一时刻的肘部位置（保持连续性）
      // latestHumanLeftElbowPos_ 保持当前值不变
    }
  }

  const auto [p1Optimized, p2Optimized] =
      leftVelocityIkSolverPtr_->solve(latestHumanLeftElbowPos_,  // p1Ref: 人肘部参考位置
                                      leftElbowFixedPoint_,      // p1Fixed: 肘部固定点
                                      scaledLeftHandPos          // p2Ref: 手部参考位置
      );
  // 保存优化后的结果
  latestRobotLeftElbowPos_ = p1Optimized;
  latestLeftHandPosAfterOpt_ = p2Optimized;

  // 更新手部位置为优化后的结果（P2）
  scaledLeftHandPos = p2Optimized;

  leftLink6Position_ = scaledLeftHandPos;
  leftEndEffectorPosition_ = scaledLeftHandPos + (incrementalLeftQuat * leftEE2Link6Offset_);
  leftVirtualThumbPosition_ = scaledLeftHandPos + (incrementalLeftQuat * leftThumb2Link6Offset_);

  // 更新约束列表 - 左臂独立处理
  updateLeftConstraintList(scaledLeftHandPos, incrementalLeftQuat, p1Optimized);

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
      latestIncrementalResult_.getLatestIncrementalHandPose(true, useIncrementalHandOrientation_, true);

  // scaledRightHandPos = scaledRightHandPos.cwiseProduct(deltaScale_);
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
      // 【核心修复】在模式切换瞬间，先通过FK计算当前真实位置，然后通过速度IK求解器优化，
      // 确保 latestPoseConstraintList_ 中的位置是当前真实位置，避免跳变
      Eigen::Vector3d pRightLink6;
      Eigen::Quaterniond qRightLink6;
      computeRightLink6FK(pRightLink6, qRightLink6);

      // 通过FK计算末端执行器在世界系下的姿态
      Eigen::Vector3d pEndEffector;
      Eigen::Quaterniond qEndEffector;
      computeRightEndEffectorFK(pEndEffector, qEndEffector);
      Eigen::Vector3d pLink4;
      Eigen::Quaterniond qLink4;
      computeRightLink4FK(pLink4, qLink4);

      // 调用速度IK求解器优化手部和肘部位置，确保位置一致性
      const auto [p1Optimized, p2Optimized] =
          rightVelocityIkSolverPtr_->solve(latestHumanRightElbowPos_,  // p1Ref: 人肘部参考位置
                                           rightElbowFixedPoint_,      // p1Fixed: 肘部固定点
                                           pRightLink6  // p2Ref: 手部参考位置（当前FK计算的Link6位置）
          );

      // 保存优化后的结果
      latestRobotRightElbowPos_ = p1Optimized;

      // 更新 latestPoseConstraintList_ 为优化后的位置，确保锚点设置正确
      {
        std::lock_guard<std::mutex> lock(poseConstraintListMutex_);
        latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].position = p2Optimized;
        latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].rotation_matrix = qRightLink6.toRotationMatrix();
        latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_ELBOW].position = p1Optimized;
      }

      incrementalController_->updateRightArmPoseAnchor(quest3ArmInfoTransformerPtr_->getRightHandPose(),
                                                       latestPoseConstraintList_,
                                                       pEndEffector,
                                                       qEndEffector,
                                                       qLink4);
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

  // 通过FK计算左右肘部实时位置
  // CZJTODO ELBOW 需要平滑，否则容易导致震荡发散，因为目前hand pose 和 elbow pos的因果耦合
  Eigen::Vector3d currentLeftElbowPos, currentRightElbowPos;
  Eigen::Quaterniond qLeftElbow, qRightElbow;
  computeLeftLink4FK(currentLeftElbowPos, qLeftElbow);
  computeRightLink4FK(currentRightElbowPos, qRightElbow);

  // 应用位置约束
  // clipHandPositionsByAllConstraints(scaledLeftHandPos,
  //                                   scaledRightHandPos,
  //                                   robotLeftFixedShoulderPos_,
  //                                   robotRightFixedShoulderPos_,
  //                                   sphereRadiusLimit_,
  //                                   minReachableDistance_,
  //                                   leftCenter_,
  //                                   rightCenter_,
  //                                   0.23,
  //                                   boxMinBound_,
  //                                   boxMaxBound_,
  //                                   chestOffsetY_,
  //                                   currentLeftElbowPos,
  //                                   currentRightElbowPos,
  //                                   elbowMinDistance_,
  //                                   elbowMaxDistance_);

  // 保存优化前的手部位置（用于可视化对比）
  latestRightHandPosBeforeOpt_ = scaledRightHandPos;

  // 使用上一时刻的link6位置和ee位置计算期望的肘部位置
  // 通过手腕-ee向量计算期望的肘部位置
  // 向量方向：ee→手腕向量方向，长度为肘部到手腕的距离l2
  // 肘点计算：肘点 = 手腕点+向量方向*长度l2
  {
    // 计算从ee到手腕的向量（ee→手腕方向）
    Eigen::Vector3d eeToWristVec = rightLink6Position_ - rightEndEffectorPosition_;
    double vecNorm = eeToWristVec.norm();

    // 如果向量长度太小，使用默认方向（避免除零错误）
    if (vecNorm > 1e-6) {
      // 归一化向量并乘以l2_长度
      Eigen::Vector3d direction = eeToWristVec.normalized() * l2_;
      // 肘点 = 手腕点 + 向量方向*长度l2
      latestHumanRightElbowPos_ = rightLink6Position_ + direction;
    } else {
      // 如果向量为零，使用上一时刻的肘部位置（保持连续性）
      // latestHumanRightElbowPos_ 保持当前值不变
    }
  }

  const auto [p1Optimized, p2Optimized] =
      rightVelocityIkSolverPtr_->solve(latestHumanRightElbowPos_,  // p1Ref: 人肘部参考位置
                                       rightElbowFixedPoint_,      // p1Fixed: 肘部固定点
                                       scaledRightHandPos          // p2Ref: 手部参考位置
      );
  // 保存优化后的结果
  latestRobotRightElbowPos_ = p1Optimized;
  latestRightHandPosAfterOpt_ = p2Optimized;

  // 更新手部位置为优化后的结果（P2）
  scaledRightHandPos = p2Optimized;

  rightLink6Position_ = scaledRightHandPos;
  rightEndEffectorPosition_ = scaledRightHandPos + (incrementalRightQuat * rightEE2Link6Offset_);
  rightVirtualThumbPosition_ = scaledRightHandPos + (incrementalRightQuat * rightThumb2Link6Offset_);

  // 更新约束列表 - 右臂独立处理
  updateRightConstraintList(scaledRightHandPos, incrementalRightQuat, p1Optimized);

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
      latestIncrementalResult_.getLatestIncrementalHandPose(true, useIncrementalHandOrientation_, true);
  // scaledLeftHandPos = scaledLeftHandPos.cwiseProduct(deltaScale_);
  // 右手位置保持不变
  {
    std::lock_guard<std::mutex> lock(poseConstraintListMutex_);
    scaledRightHandPos = latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].position;
  }

  // 通过FK计算左右肘部实时位置
  // CZJTODO ELBOW 需要平滑，否则容易导致震荡发散，因为目前hand pose 和 elbow pos的因果耦合
  Eigen::Vector3d currentLeftElbowPos, currentRightElbowPos;
  Eigen::Quaterniond qLeftElbow, qRightElbow;
  computeLeftLink4FK(currentLeftElbowPos, qLeftElbow);
  computeRightLink4FK(currentRightElbowPos, qRightElbow);

  // clipHandPositionsByAllConstraints(scaledLeftHandPos,
  //                                   scaledRightHandPos,
  //                                   robotLeftFixedShoulderPos_,
  //                                   robotRightFixedShoulderPos_,
  //                                   sphereRadiusLimit_,
  //                                   minReachableDistance_,
  //                                   leftCenter_,
  //                                   rightCenter_,
  //                                   0.23,
  //                                   boxMinBound_,
  //                                   boxMaxBound_,
  //                                   chestOffsetY_,
  //                                   currentLeftElbowPos,
  //                                   currentRightElbowPos,
  //                                   elbowMinDistance_,
  //                                   elbowMaxDistance_);
  scaledLeftHandPos = joyStickHandlerPtr_->isLeftArmCtrlModeActive() ? scaledLeftHandPos : defaultLeftHandPosOnExit_;

  // 保存优化前的手部位置（用于可视化对比）
  latestLeftHandPosBeforeOpt_ = scaledLeftHandPos;

  const auto [p1Optimized, p2Optimized] =
      leftVelocityIkSolverPtr_->solve(latestHumanLeftElbowPos_,  // p1Ref: 人肘部参考位置
                                      leftElbowFixedPoint_,      // p1Fixed: 肘部固定点
                                      scaledLeftHandPos          // p2Ref: 手部参考位置
      );
  // 保存优化后的结果
  latestRobotLeftElbowPos_ = p1Optimized;
  latestLeftHandPosAfterOpt_ = p2Optimized;

  // 更新手部位置为优化后的结果（P2）
  scaledLeftHandPos = p2Optimized;

  leftLink6Position_ = scaledLeftHandPos;
  leftEndEffectorPosition_ = scaledLeftHandPos + (incrementalLeftQuat * leftEE2Link6Offset_);
  leftVirtualThumbPosition_ = scaledLeftHandPos + (incrementalLeftQuat * leftThumb2Link6Offset_);

  updateLeftConstraintList(scaledLeftHandPos, incrementalLeftQuat, p1Optimized);

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
      latestIncrementalResult_.getLatestIncrementalHandPose(true, useIncrementalHandOrientation_, true);

  // scaledRightHandPos = scaledRightHandPos.cwiseProduct(deltaScale_);
  // 左手位置保持不变
  {
    std::lock_guard<std::mutex> lock(poseConstraintListMutex_);
    scaledLeftHandPos = latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].position;
  }

  // 通过FK计算左右肘部实时位置
  // CZJTODO ELBOW 需要平滑，否则容易导致震荡发散，因为目前hand pose 和 elbow pos的因果耦合
  Eigen::Vector3d currentLeftElbowPos, currentRightElbowPos;
  Eigen::Quaterniond qLeftElbow, qRightElbow;
  computeLeftLink4FK(currentLeftElbowPos, qLeftElbow);
  computeRightLink4FK(currentRightElbowPos, qRightElbow);
  // clipHandPositionsByAllConstraints(scaledLeftHandPos,
  //                                   scaledRightHandPos,
  //                                   robotLeftFixedShoulderPos_,
  //                                   robotRightFixedShoulderPos_,
  //                                   sphereRadiusLimit_,
  //                                   minReachableDistance_,
  //                                   leftCenter_,
  //                                   rightCenter_,
  //                                   0.23,
  //                                   boxMinBound_,
  //                                   boxMaxBound_,
  //                                   chestOffsetY_,
  //                                   currentLeftElbowPos,
  //                                   currentRightElbowPos,
  //                                   elbowMinDistance_,
  //                                   elbowMaxDistance_);

  // 默认位置不要clip
  scaledRightHandPos =
      joyStickHandlerPtr_->isRightArmCtrlModeActive() ? scaledRightHandPos : defaultRightHandPosOnExit_;

  // 保存优化前的手部位置（用于可视化对比）
  latestRightHandPosBeforeOpt_ = scaledRightHandPos;

  const auto [p1Optimized, p2Optimized] =
      rightVelocityIkSolverPtr_->solve(latestHumanRightElbowPos_,  // p1Ref: 人肘部参考位置
                                       rightElbowFixedPoint_,      // p1Fixed: 肘部固定点
                                       scaledRightHandPos          // p2Ref: 手部参考位置
      );
  // 保存优化后的结果
  latestRobotRightElbowPos_ = p1Optimized;
  latestRightHandPosAfterOpt_ = p2Optimized;

  // 更新手部位置为优化后的结果（P2）
  scaledRightHandPos = p2Optimized;

  rightLink6Position_ = scaledRightHandPos;
  rightEndEffectorPosition_ = scaledRightHandPos + (incrementalRightQuat * rightEE2Link6Offset_);
  rightVirtualThumbPosition_ = scaledRightHandPos + (incrementalRightQuat * rightThumb2Link6Offset_);

  updateRightConstraintList(scaledRightHandPos, incrementalRightQuat, p1Optimized);

  return true;
}

void Quest3IkIncrementalROS::solveIk() {
  std::lock_guard<std::mutex> lock(poseConstraintListMutex_);
  auto ikResult = oneStageIkEndEffectorPtr_->solveIK(latestPoseConstraintList_, ctrlArmIdx_, jointMidValues_);

  if (ikResult.isSuccess) {
    // 在写入最新 IK 解之前，基于上一帧解做一个“跳变限制”，
    // 确保本帧解不会离上一帧解过远（IK 通道防抖/滤波）
    Eigen::VectorXd filteredSolution = ikResult.solution;

    {
      std::lock_guard<std::mutex> lock(ikResultMutex_);

      // 仅当已经存在上一帧有效解，且尺寸一致时才做跳变限制
      if (hasValidIkSolution_ && latestIkSolution_.size() == filteredSolution.size()) {
        // 单关节最大允许跳变量（单位：rad），约 20 度
        static constexpr double kMaxJointJump = 0.35;

        for (int i = 0; i < filteredSolution.size(); ++i) {
          double delta = filteredSolution[i] - latestIkSolution_[i];
          if (delta > kMaxJointJump) {
            delta = kMaxJointJump;
          } else if (delta < -kMaxJointJump) {
            delta = -kMaxJointJump;
          }
          filteredSolution[i] = latestIkSolution_[i] + delta;
        }
      }

      latestIkSolution_ = filteredSolution;
      hasValidIkSolution_ = true;
    }
  }
}

void Quest3IkIncrementalROS::processVisual() {
  auto fkCallback = [this](Eigen::Vector3d& leftPos,
                           Eigen::Quaterniond& leftQuat,
                           Eigen::Vector3d& rightPos,
                           Eigen::Quaterniond& rightQuat) {
    // 可视化使用原始传感器数据
    std::shared_ptr<kuavo_msgs::sensorsData> currentSensorData = getSensorData();
    const int armJointStartIndex = 12 + waist_dof_;  // 考虑腰部自由度
    if (currentSensorData && currentSensorData->joint_data.joint_q.size() >= armJointStartIndex + jointStateSize_) {
      Eigen::VectorXd armJoints(jointStateSize_);
      for (int i = 0; i < jointStateSize_; ++i) {
        armJoints(i) = currentSensorData->joint_data.joint_q[armJointStartIndex + i];
      }

      auto [leftMeasuredPosition, leftMeasuredQuaternion] =
          oneStageIkEndEffectorPtr_->FK(armJoints, "zarm_l7_end_effector", jointStateSize_);
      auto [rightMeasuredPosition, rightMeasuredQuaternion] =
          oneStageIkEndEffectorPtr_->FK(armJoints, "zarm_r7_end_effector", jointStateSize_);

      leftPos = leftMeasuredPosition;
      leftQuat = leftMeasuredQuaternion;
      rightPos = rightMeasuredPosition;
      rightQuat = rightMeasuredQuaternion;
    }
  };

  // 通过FK计算左右肘部实时位置（用于可视化，使用原始传感器数据）
  Eigen::Vector3d currentLeftElbowPos, currentRightElbowPos;
  Eigen::Quaterniond qLeftElbow, qRightElbow;
  std::shared_ptr<kuavo_msgs::sensorsData> currentSensorData = getSensorData();
  const int armJointStartIndex = 12 + waist_dof_;  // 考虑腰部自由度
  if (currentSensorData && currentSensorData->joint_data.joint_q.size() >= armJointStartIndex + jointStateSize_) {
    Eigen::VectorXd armJoints(jointStateSize_);
    for (int i = 0; i < jointStateSize_; ++i) {
      armJoints(i) = currentSensorData->joint_data.joint_q[armJointStartIndex + i];
    }
    auto [l4Position, l4Quaternion] = oneStageIkEndEffectorPtr_->FKElbow(armJoints, "zarm_l4_link", jointStateSize_);
    auto [r4Position, r4Quaternion] = oneStageIkEndEffectorPtr_->FKElbow(armJoints, "zarm_r4_link", jointStateSize_);
    currentLeftElbowPos = l4Position;
    qLeftElbow = l4Quaternion;
    currentRightElbowPos = r4Position;
    qRightElbow = r4Quaternion;
  } else {
    currentLeftElbowPos = Eigen::Vector3d::Zero();
    currentRightElbowPos = Eigen::Vector3d::Zero();
    qLeftElbow = Eigen::Quaterniond::Identity();
    qRightElbow = Eigen::Quaterniond::Identity();
  }
  // CZJTODO ELBOW 需要平滑，否则容易导致震荡发散，因为目前hand pose 和 elbow pos的因果耦合
  {
    std::lock_guard<std::mutex> lock(poseConstraintListMutex_);
    quest3KeyFramesVisualizerPtr_->publishAllVisualizations(robotLeftFixedShoulderPos_,
                                                            robotRightFixedShoulderPos_,
                                                            sphereRadiusLimit_,
                                                            minReachableDistance_,
                                                            latestIncrementalResult_,
                                                            latestPoseConstraintList_,
                                                            boxMinBound_,
                                                            boxMaxBound_,
                                                            chestOffsetY_,
                                                            leftCenter_,
                                                            rightCenter_,
                                                            0.2,
                                                            currentLeftElbowPos,
                                                            currentRightElbowPos,
                                                            fkCallback,
                                                            leftLink6Position_,
                                                            rightLink6Position_,
                                                            leftEndEffectorPosition_,
                                                            rightEndEffectorPosition_,
                                                            leftVirtualThumbPosition_,
                                                            rightVirtualThumbPosition_);
    // 发布机器人优化后的肘部位置可视化（墨绿色，不透明）
    quest3KeyFramesVisualizerPtr_->publishRobotElbowPosVisualization(latestRobotLeftElbowPos_,
                                                                     latestRobotRightElbowPos_);
    // 发布优化前后的手部位置可视化对比（橙色=优化前，紫色=优化后）
    quest3KeyFramesVisualizerPtr_->publishHandPosOptimizationVisualization(latestLeftHandPosBeforeOpt_,
                                                                           latestLeftHandPosAfterOpt_,
                                                                           latestRightHandPosBeforeOpt_,
                                                                           latestRightHandPosAfterOpt_);
  }
}

void Quest3IkIncrementalROS::activateController() {
  if (controllerActivated_.load()) return;
  if (!humanoidArmCtrlModeClient_.exists()) return;
  if (!changeArmCtrlModeClient_.exists()) return;

  ROS_INFO("[Quest3IkIncrementalROS] Activating controller");
  kuavo_msgs::changeArmCtrlMode srv2;
  srv2.request.control_mode = static_cast<int>(KuavoArmCtrlMode::EXTERNAL_CONTROL);

  controllerActivated_.store(
      humanoidArmCtrlModeClient_.call(srv2) && srv2.response.result &&  //
      changeArmCtrlModeClient_.call(srv2) && srv2.response.result &&    //
      true);
}

void Quest3IkIncrementalROS::deactivateController() {
  if (!changeArmModeClient_.exists()) return;
  if (!controllerActivated_.load()) return;
  if (!humanoidArmCtrlModeClient_.exists()) return;
  if (!changeArmCtrlModeClient_.exists()) return;
  if (arm_ctrl_mode_ == 1 or arm_ctrl_mode_ == 2) return; //如果当前是模式1或者模式2，不切换回去，防止进入和退出增量模式时误触发

  kuavo_msgs::changeArmCtrlMode srv1, srv2;
  srv1.request.control_mode = static_cast<int>(MpcRefUpdateMode::DISABLED_ARM);
  srv2.request.control_mode = static_cast<int>(KuavoArmCtrlMode::ARM_FIXED);

  controllerActivated_.store(!(
    changeArmModeClient_.call(srv1) && srv1.response.result &&
      humanoidArmCtrlModeClient_.call(srv2) && srv2.response.result &&  //
      changeArmCtrlModeClient_.call(srv2) && srv2.response.result &&    //
      true));
}

void Quest3IkIncrementalROS::armCtrlModeCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
  if (msg->data.size() != 2) {
    ROS_WARN("[Quest3IkIncrementalROS] Invalid arm control mode message");
    return;
  }
  arm_ctrl_mode_ = static_cast<int>(msg->data[1]); //获取手臂控制模式
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

  // 使用armAngleLimited进行FK,获得左右手的ee_pose
  // 计算左手末端执行器FK
  auto [leftEePosition, leftEeQuaternion] =
      oneStageIkEndEffectorPtr_->FK(armAngleLimited, "zarm_l7_end_effector", jointStateSize_);

  // 计算右手末端执行器FK
  auto [rightEePosition, rightEeQuaternion] =
      oneStageIkEndEffectorPtr_->FK(armAngleLimited, "zarm_r7_end_effector", jointStateSize_);

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

  // 使用armAngleLimited进行FK,获得左右手link6的pose
  // 计算左手link6 FK
  auto [leftLink6Position, leftLink6Quaternion] =
      oneStageIkEndEffectorPtr_->FK(armAngleLimited, "zarm_l6_link", jointStateSize_);

  // 计算右手link6 FK
  auto [rightLink6Position, rightLink6Quaternion] =
      oneStageIkEndEffectorPtr_->FK(armAngleLimited, "zarm_r6_link", jointStateSize_);

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

  // 使用均值滤波后的关节数据进行FK,获得左右手ee的pose
  if (sensorArmJointQ_.size() != jointStateSize_) {
    // 如果滤波数据不可用，跳过measured pose发布
  } else {
    Eigen::VectorXd armJointsMeasured = sensorArmJointQ_;

    // 计算左手末端执行器FK（基于滤波后的关节数据）
    auto [leftEePositionMeasured, leftEeQuaternionMeasured] =
        oneStageIkEndEffectorPtr_->FK(armJointsMeasured, "zarm_l7_end_effector", jointStateSize_);

    // 计算右手末端执行器FK（基于滤波后的关节数据）
    auto [rightEePositionMeasured, rightEeQuaternionMeasured] =
        oneStageIkEndEffectorPtr_->FK(armJointsMeasured, "zarm_r7_end_effector", jointStateSize_);

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

  double fhanH = 1.0 / publishRate_;
  double fhanH0 = fhanH * fhanKh0Joint_;

  for (int i = 0; i < jointStateSize_; ++i) {
    double targetAngle = finalArmAngles(i);
    if (i == 5 || i == 6 || i == 12 || i == 13) {
      targetAngle = targetAngle * deltaScaleRPY_(1);
    }
    if (i <= 6 && joyStickHandlerPtr_->isLeftGrip()) {
      leju_utils::fhanStepForwardWithVelLimit(q_(i),               // 滤波后的关节角度（输出）
                                              dq_(i),              // 滤波后的关节角速度（输出）
                                              targetAngle,         // 目标角度（输入）
                                              fhanRJoint_,         // 加速度约束
                                              fhanH,               // 时间步长
                                              fhanH0,              // 平滑系数
                                              maxJointVelocity_);  // 最大速度限制
    }
    if (i > 6 && i <= 13 && joyStickHandlerPtr_->isRightGrip()) {
      leju_utils::fhanStepForwardWithVelLimit(q_(i),               // 滤波后的关节角度（输出）
                                              dq_(i),              // 滤波后的关节角速度（输出）
                                              targetAngle,         // 目标角度（输入）
                                              fhanRJoint_,         // 加速度约束
                                              fhanH,               // 时间步长
                                              fhanH0,              // 平滑系数
                                              maxJointVelocity_);  // 最大速度限制
    }
    dq_(i) = dq_(i) > 18.0 ? 18.0 : dq_(i);
    dq_(i) = dq_(i) < -18.0 ? -18.0 : dq_(i);
  }
  // Wrist joint limits (hard-coded from URDF):
  //   src/kuavo_assets/models/biped_s49/urdf/biped_s49.urdf
  // This avoids relying on MultibodyPlant joint indexing (mec_limit_*), which can drift when the URDF changes.
  static constexpr double kZarmL5Lower = -1.5707963267949;
  static constexpr double kZarmL5Upper = 1.5707963267949;
  static constexpr double kZarmL6Lower = -1.30899693899575;
  static constexpr double kZarmL6Upper = 0.698131700797732;
  static constexpr double kZarmL7Lower = -0.698131700797732;
  static constexpr double kZarmL7Upper = 0.698131700797732;

  static constexpr double kZarmR5Lower = -1.5707963267949;
  static constexpr double kZarmR5Upper = 1.5707963267949;
  static constexpr double kZarmR6Lower = -0.698131700797732;
  static constexpr double kZarmR6Upper = 1.30899693899575;
  static constexpr double kZarmR7Lower = -0.698131700797732;
  static constexpr double kZarmR7Upper = 0.698131700797732;

  q_(4) = std::min(std::max(q_(4), 0.95 * kZarmL5Lower), 0.95 * kZarmL5Upper);
  q_(5) = std::min(std::max(q_(5), 0.85 * kZarmL6Lower), 0.85 * kZarmL6Upper);
  q_(6) = std::min(std::max(q_(6), 0.85 * kZarmL7Lower), 0.85 * kZarmL7Upper);

  q_(11) = std::min(std::max(q_(11), kZarmR5Lower), kZarmR5Upper);
  q_(12) = std::min(std::max(q_(12), 0.85 * kZarmR6Lower), 0.85 * kZarmR6Upper);
  q_(13) = std::min(std::max(q_(13), 0.85 * kZarmR7Lower), 0.85 * kZarmR7Upper);

  // 处理左手平滑过渡：根据超时时间进度逐步增大 alpha 值（仅在检测到移动时计算）
  {
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

    // if (alpha < 1.0) {
    //   std::cout << "[Quest3IkIncrementalROS] Left grip: normal (alpha=" << std::fixed << std::setprecision(3) <<
    //   alpha
    //             << ")" << std::endl;
    // }
  }

  // 处理右手平滑过渡：根据超时时间进度逐步增大 alpha 值（仅在检测到移动时计算）
  {
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

    latest_q_.tail(7) = (1.0 - alpha) * latest_q_.tail(7) + alpha * q_.tail(7);
    latest_dq_.tail(7) = (1.0 - alpha) * latest_dq_.tail(7) + alpha * dq_.tail(7);

    // if (alpha < 1.0) {
    //   std::cout << "[Quest3IkIncrementalROS] Right grip: normal (alpha=" << std::fixed << std::setprecision(3) <<
    //   alpha
    //             << ")" << std::endl;
    // }
  }

  lowpass_dq_ = lowpassDqAlpha_ * lowpass_dq_ + (1.0 - lowpassDqAlpha_) * latest_dq_;

  for (int i = 0; i < jointStateSize_; ++i) {
    jointStateMsg.position[i] = latest_q_(i) * 180.0 / M_PI;
    jointStateMsg.velocity[i] = lowpass_dq_(i) * 180.0 / M_PI;
    jointStateMsg.effort[i] = 0.0;
  }

  // 在此处fk并计算filter后的fk结果
  // 使用滤波后的关节角度 q_ 进行FK计算
  Eigen::VectorXd qRadians = q_;  // q_ 已经是弧度单位
  auto [leftEePositionFilter, leftEeQuaternionFilter] =
      oneStageIkEndEffectorPtr_->FK(qRadians, "zarm_l7_end_effector", jointStateSize_);
  auto [rightEePositionFilter, rightEeQuaternionFilter] =
      oneStageIkEndEffectorPtr_->FK(qRadians, "zarm_r7_end_effector", jointStateSize_);

  // 发布滤波后的左手末端执行器pose
  geometry_msgs::PoseStamped leftEePoseFilterMsg;
  leftEePoseFilterMsg.header.stamp = ros::Time::now();
  leftEePoseFilterMsg.header.frame_id = "base_link";
  leftEePoseFilterMsg.pose.position.x = leftEePositionFilter.x();
  leftEePoseFilterMsg.pose.position.y = leftEePositionFilter.y();
  leftEePoseFilterMsg.pose.position.z = leftEePositionFilter.z();
  leftEePoseFilterMsg.pose.orientation.w = leftEeQuaternionFilter.w();
  leftEePoseFilterMsg.pose.orientation.x = leftEeQuaternionFilter.x();
  leftEePoseFilterMsg.pose.orientation.y = leftEeQuaternionFilter.y();
  leftEePoseFilterMsg.pose.orientation.z = leftEeQuaternionFilter.z();
  leftEePoseFilterPublisher_.publish(leftEePoseFilterMsg);

  // 发布滤波后的右手末端执行器pose
  geometry_msgs::PoseStamped rightEePoseFilterMsg;
  rightEePoseFilterMsg.header.stamp = ros::Time::now();
  rightEePoseFilterMsg.header.frame_id = "base_link";
  rightEePoseFilterMsg.pose.position.x = rightEePositionFilter.x();
  rightEePoseFilterMsg.pose.position.y = rightEePositionFilter.y();
  rightEePoseFilterMsg.pose.position.z = rightEePositionFilter.z();
  rightEePoseFilterMsg.pose.orientation.w = rightEeQuaternionFilter.w();
  rightEePoseFilterMsg.pose.orientation.x = rightEeQuaternionFilter.x();
  rightEePoseFilterMsg.pose.orientation.y = rightEeQuaternionFilter.y();
  rightEePoseFilterMsg.pose.orientation.z = rightEeQuaternionFilter.z();
  rightEePoseFilterPublisher_.publish(rightEePoseFilterMsg);

  // 在此处fk并计算filter后的link6 fk结果
  // 使用滤波后的关节角度 q_ 进行FK计算
  auto [leftLink6PositionFilter, leftLink6QuaternionFilter] =
      oneStageIkEndEffectorPtr_->FK(qRadians, "zarm_l6_link", jointStateSize_);
  auto [rightLink6PositionFilter, rightLink6QuaternionFilter] =
      oneStageIkEndEffectorPtr_->FK(qRadians, "zarm_r6_link", jointStateSize_);

  // 发布滤波后的左手link6 pose
  geometry_msgs::PoseStamped leftLink6PoseFilterMsg;
  leftLink6PoseFilterMsg.header.stamp = ros::Time::now();
  leftLink6PoseFilterMsg.header.frame_id = "base_link";
  leftLink6PoseFilterMsg.pose.position.x = leftLink6PositionFilter.x();
  leftLink6PoseFilterMsg.pose.position.y = leftLink6PositionFilter.y();
  leftLink6PoseFilterMsg.pose.position.z = leftLink6PositionFilter.z();
  leftLink6PoseFilterMsg.pose.orientation.w = leftLink6QuaternionFilter.w();
  leftLink6PoseFilterMsg.pose.orientation.x = leftLink6QuaternionFilter.x();
  leftLink6PoseFilterMsg.pose.orientation.y = leftLink6QuaternionFilter.y();
  leftLink6PoseFilterMsg.pose.orientation.z = leftLink6QuaternionFilter.z();
  leftLink6PoseFilterPublisher_.publish(leftLink6PoseFilterMsg);

  // 发布滤波后的右手link6 pose
  geometry_msgs::PoseStamped rightLink6PoseFilterMsg;
  rightLink6PoseFilterMsg.header.stamp = ros::Time::now();
  rightLink6PoseFilterMsg.header.frame_id = "base_link";
  rightLink6PoseFilterMsg.pose.position.x = rightLink6PositionFilter.x();
  rightLink6PoseFilterMsg.pose.position.y = rightLink6PositionFilter.y();
  rightLink6PoseFilterMsg.pose.position.z = rightLink6PositionFilter.z();
  rightLink6PoseFilterMsg.pose.orientation.w = rightLink6QuaternionFilter.w();
  rightLink6PoseFilterMsg.pose.orientation.x = rightLink6QuaternionFilter.x();
  rightLink6PoseFilterMsg.pose.orientation.y = rightLink6QuaternionFilter.y();
  rightLink6PoseFilterMsg.pose.orientation.z = rightLink6QuaternionFilter.z();
  rightLink6PoseFilterPublisher_.publish(rightLink6PoseFilterMsg);

  // 在此处fk并计算measured后的link6 fk结果
  // 使用均值滤波后的关节数据进行FK计算
  if (sensorArmJointQ_.size() != jointStateSize_) {
    // 如果滤波数据不可用，跳过measured link6 pose发布
  } else {
    Eigen::VectorXd armJointsMeasured = sensorArmJointQ_;

    auto [leftLink6PositionMeasured, leftLink6QuaternionMeasured] =
        oneStageIkEndEffectorPtr_->FK(armJointsMeasured, "zarm_l6_link", jointStateSize_);
    auto [rightLink6PositionMeasured, rightLink6QuaternionMeasured] =
        oneStageIkEndEffectorPtr_->FK(armJointsMeasured, "zarm_r6_link", jointStateSize_);

    // 发布measured后的左手link6 pose
    geometry_msgs::PoseStamped leftLink6PoseMeasuredMsg;
    leftLink6PoseMeasuredMsg.header.stamp = ros::Time::now();
    leftLink6PoseMeasuredMsg.header.frame_id = "base_link";
    leftLink6PoseMeasuredMsg.pose.position.x = leftLink6PositionMeasured.x();
    leftLink6PoseMeasuredMsg.pose.position.y = leftLink6PositionMeasured.y();
    leftLink6PoseMeasuredMsg.pose.position.z = leftLink6PositionMeasured.z();
    leftLink6PoseMeasuredMsg.pose.orientation.w = leftLink6QuaternionMeasured.w();
    leftLink6PoseMeasuredMsg.pose.orientation.x = leftLink6QuaternionMeasured.x();
    leftLink6PoseMeasuredMsg.pose.orientation.y = leftLink6QuaternionMeasured.y();
    leftLink6PoseMeasuredMsg.pose.orientation.z = leftLink6QuaternionMeasured.z();
    leftLink6PoseMeasuredPublisher_.publish(leftLink6PoseMeasuredMsg);

    // 发布measured后的右手link6 pose
    geometry_msgs::PoseStamped rightLink6PoseMeasuredMsg;
    rightLink6PoseMeasuredMsg.header.stamp = ros::Time::now();
    rightLink6PoseMeasuredMsg.header.frame_id = "base_link";
    rightLink6PoseMeasuredMsg.pose.position.x = rightLink6PositionMeasured.x();
    rightLink6PoseMeasuredMsg.pose.position.y = rightLink6PositionMeasured.y();
    rightLink6PoseMeasuredMsg.pose.position.z = rightLink6PositionMeasured.z();
    rightLink6PoseMeasuredMsg.pose.orientation.w = rightLink6QuaternionMeasured.w();
    rightLink6PoseMeasuredMsg.pose.orientation.x = rightLink6QuaternionMeasured.x();
    rightLink6PoseMeasuredMsg.pose.orientation.y = rightLink6QuaternionMeasured.y();
    rightLink6PoseMeasuredMsg.pose.orientation.z = rightLink6QuaternionMeasured.z();
    rightLink6PoseMeasuredPublisher_.publish(rightLink6PoseMeasuredMsg);
  }

  kuavoArmTrajCppPublisher_.publish(jointStateMsg);

  // 发布 /drake_ik/eef_pose（与Python版 pub_solved_arm_eef_pose 一致）
  // 使用滤波后的 latest_q_（弧度）进行FK计算，保证与实际发送给机器人的关节角度一致
  {
    auto [latestLeftEePosition, latestLeftEeQuaternion] =
        oneStageIkEndEffectorPtr_->FK(latest_q_, "zarm_l7_end_effector", jointStateSize_);
    auto [latestRightEePosition, latestRightEeQuaternion] =
        oneStageIkEndEffectorPtr_->FK(latest_q_, "zarm_r7_end_effector", jointStateSize_);

    kuavo_msgs::twoArmHandPose eefPoseMsg;
    eefPoseMsg.header.frame_id = "torso";
    eefPoseMsg.header.stamp = ros::Time::now();

    // 左手位姿
    eefPoseMsg.left_pose.pos_xyz[0] = latestLeftEePosition.x();
    eefPoseMsg.left_pose.pos_xyz[1] = latestLeftEePosition.y();
    eefPoseMsg.left_pose.pos_xyz[2] = latestLeftEePosition.z();
    eefPoseMsg.left_pose.quat_xyzw[0] = latestLeftEeQuaternion.x();
    eefPoseMsg.left_pose.quat_xyzw[1] = latestLeftEeQuaternion.y();
    eefPoseMsg.left_pose.quat_xyzw[2] = latestLeftEeQuaternion.z();
    eefPoseMsg.left_pose.quat_xyzw[3] = latestLeftEeQuaternion.w();
    // 左臂关节角度（弧度，与Python版一致）
    const int singleArmDof = jointStateSize_ / 2;
    for (int i = 0; i < 7 && i < singleArmDof; ++i) {
      eefPoseMsg.left_pose.joint_angles[i] = latest_q_(i);
    }

    // 右手位姿
    eefPoseMsg.right_pose.pos_xyz[0] = latestRightEePosition.x();
    eefPoseMsg.right_pose.pos_xyz[1] = latestRightEePosition.y();
    eefPoseMsg.right_pose.pos_xyz[2] = latestRightEePosition.z();
    eefPoseMsg.right_pose.quat_xyzw[0] = latestRightEeQuaternion.x();
    eefPoseMsg.right_pose.quat_xyzw[1] = latestRightEeQuaternion.y();
    eefPoseMsg.right_pose.quat_xyzw[2] = latestRightEeQuaternion.z();
    eefPoseMsg.right_pose.quat_xyzw[3] = latestRightEeQuaternion.w();
    // 右臂关节角度（弧度，与Python版一致）
    for (int i = 0; i < 7 && i < singleArmDof; ++i) {
      eefPoseMsg.right_pose.joint_angles[i] = latest_q_(singleArmDof + i);
    }

    ikSolvedEefPosePublisher_.publish(eefPoseMsg);
  }

  // 发布 /drake_ik/input_pos（与Python版一致）
  // 数据格式：14个float [左手pos_xyz(3), 左手quat_xyzw(4), 右手pos_xyz(3), 右手quat_xyzw(4)]
  // C++版本使用优化后的 leftEndEffectorPosition_ / rightEndEffectorPosition_ 和对应四元数
  {
    kuavo_msgs::Float32MultiArrayStamped inputPosMsg;
    inputPosMsg.header.stamp = ros::Time::now();
    inputPosMsg.data.data.resize(14);

    // 左手：优化后的末端执行器位置 + 约束列表中的四元数
    inputPosMsg.data.data[0] = static_cast<float>(leftEndEffectorPosition_.x());
    inputPosMsg.data.data[1] = static_cast<float>(leftEndEffectorPosition_.y());
    inputPosMsg.data.data[2] = static_cast<float>(leftEndEffectorPosition_.z());
    {
      std::lock_guard<std::mutex> lock(poseConstraintListMutex_);
      Eigen::Quaterniond leftQuat(latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].rotation_matrix);
      inputPosMsg.data.data[3] = static_cast<float>(leftQuat.x());
      inputPosMsg.data.data[4] = static_cast<float>(leftQuat.y());
      inputPosMsg.data.data[5] = static_cast<float>(leftQuat.z());
      inputPosMsg.data.data[6] = static_cast<float>(leftQuat.w());

      // 右手：优化后的末端执行器位置 + 约束列表中的四元数
      inputPosMsg.data.data[7] = static_cast<float>(rightEndEffectorPosition_.x());
      inputPosMsg.data.data[8] = static_cast<float>(rightEndEffectorPosition_.y());
      inputPosMsg.data.data[9] = static_cast<float>(rightEndEffectorPosition_.z());
      Eigen::Quaterniond rightQuat(latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].rotation_matrix);
      inputPosMsg.data.data[10] = static_cast<float>(rightQuat.x());
      inputPosMsg.data.data[11] = static_cast<float>(rightQuat.y());
      inputPosMsg.data.data[12] = static_cast<float>(rightQuat.z());
      inputPosMsg.data.data[13] = static_cast<float>(rightQuat.w());
    }

    ikInputPosPublisher_.publish(inputPosMsg);
  }

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

      // 发布左手旋转矩阵（9个浮点数，按行展开）
      std_msgs::Float32MultiArray rotationMatrixMsg;
      rotationMatrixMsg.data.resize(9);
      const Eigen::Matrix3d& R = leftHandPose.rotation_matrix;
      // 按行展开：R(0,0), R(0,1), R(0,2), R(1,0), R(1,1), R(1,2), R(2,0), R(2,1), R(2,2)
      rotationMatrixMsg.data[0] = static_cast<float>(R(0, 0));
      rotationMatrixMsg.data[1] = static_cast<float>(R(0, 1));
      rotationMatrixMsg.data[2] = static_cast<float>(R(0, 2));
      rotationMatrixMsg.data[3] = static_cast<float>(R(1, 0));
      rotationMatrixMsg.data[4] = static_cast<float>(R(1, 1));
      rotationMatrixMsg.data[5] = static_cast<float>(R(1, 2));
      rotationMatrixMsg.data[6] = static_cast<float>(R(2, 0));
      rotationMatrixMsg.data[7] = static_cast<float>(R(2, 1));
      rotationMatrixMsg.data[8] = static_cast<float>(R(2, 2));
      leftHandRotationMatrixPublisher_.publish(rotationMatrixMsg);
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

      // 发布右手旋转矩阵（9个浮点数，按行展开）
      std_msgs::Float32MultiArray rightRotationMatrixMsg;
      rightRotationMatrixMsg.data.resize(9);
      const Eigen::Matrix3d& R = rightHandPose.rotation_matrix;
      // 按行展开：R(0,0), R(0,1), R(0,2), R(1,0), R(1,1), R(1,2), R(2,0), R(2,1), R(2,2)
      rightRotationMatrixMsg.data[0] = static_cast<float>(R(0, 0));
      rightRotationMatrixMsg.data[1] = static_cast<float>(R(0, 1));
      rightRotationMatrixMsg.data[2] = static_cast<float>(R(0, 2));
      rightRotationMatrixMsg.data[3] = static_cast<float>(R(1, 0));
      rightRotationMatrixMsg.data[4] = static_cast<float>(R(1, 1));
      rightRotationMatrixMsg.data[5] = static_cast<float>(R(1, 2));
      rightRotationMatrixMsg.data[6] = static_cast<float>(R(2, 0));
      rightRotationMatrixMsg.data[7] = static_cast<float>(R(2, 1));
      rightRotationMatrixMsg.data[8] = static_cast<float>(R(2, 2));
      rightHandRotationMatrixPublisher_.publish(rightRotationMatrixMsg);
    }
  }

  // 发布左右手 anchor 四元数
  if (incrementalController_) {
    geometry_msgs::Quaternion leftAnchorQuatMsg;
    geometry_msgs::Quaternion rightAnchorQuatMsg;

    Eigen::Quaterniond leftAnchorQuat = incrementalController_->getRobotLeftHandAnchorQuat();
    Eigen::Quaterniond rightAnchorQuat = incrementalController_->getRobotRightHandAnchorQuat();

    leftAnchorQuatMsg.x = leftAnchorQuat.x();
    leftAnchorQuatMsg.y = leftAnchorQuat.y();
    leftAnchorQuatMsg.z = leftAnchorQuat.z();
    leftAnchorQuatMsg.w = leftAnchorQuat.w();

    rightAnchorQuatMsg.x = rightAnchorQuat.x();
    rightAnchorQuatMsg.y = rightAnchorQuat.y();
    rightAnchorQuatMsg.z = rightAnchorQuat.z();
    rightAnchorQuatMsg.w = rightAnchorQuat.w();

    leftAnchorQuatPublisher_.publish(leftAnchorQuatMsg);
    rightAnchorQuatPublisher_.publish(rightAnchorQuatMsg);

    auto incResult = incrementalController_->getLatestIncrementalResult();
    const Eigen::Quaterniond leftDeltaQuat = incResult.getLatestRobotLeftHandQuatInc().second;
    const Eigen::Quaterniond rightDeltaQuat = incResult.getLatestRobotRightHandQuatInc().second;

    geometry_msgs::Quaternion leftDeltaQuatMsg;
    geometry_msgs::Quaternion rightDeltaQuatMsg;

    leftDeltaQuatMsg.x = leftDeltaQuat.x();
    leftDeltaQuatMsg.y = leftDeltaQuat.y();
    leftDeltaQuatMsg.z = leftDeltaQuat.z();
    leftDeltaQuatMsg.w = leftDeltaQuat.w();

    rightDeltaQuatMsg.x = rightDeltaQuat.x();
    rightDeltaQuatMsg.y = rightDeltaQuat.y();
    rightDeltaQuatMsg.z = rightDeltaQuat.z();
    rightDeltaQuatMsg.w = rightDeltaQuat.w();

    leftHandDeltaQuatPublisher_.publish(leftDeltaQuatMsg);
    rightHandDeltaQuatPublisher_.publish(rightDeltaQuatMsg);
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

  // 从传感器数据提取手臂关节角（从索引12+腰部自由度开始），并转换为角度单位
  const int armJointStartIndex = 12 + waist_dof_;  // 考虑腰部自由度
  for (int i = 0; i < jointStateSize_; ++i) {
    double jointAngleRad = currentSensorData->joint_data.joint_q[armJointStartIndex + i];
    jointStateMsg.position[i] = jointAngleRad * 180.0 / M_PI;  // 转换为角度
  }

  sensorDataArmJointsPublisher_.publish(jointStateMsg);
}

void Quest3IkIncrementalROS::publishHandPoseFromTransformer() {
  if (!quest3ArmInfoTransformerPtr_) {
    return;
  }

  // 发布来自Transformer的左手pose
  const auto& leftHandPose = quest3ArmInfoTransformerPtr_->getLeftHandPose();
  if (leftHandPose.isValid()) {
    geometry_msgs::PoseStamped leftHandPoseMsg;
    leftHandPoseMsg.header.stamp = ros::Time::now();
    leftHandPoseMsg.header.frame_id = "base_link";
    leftHandPoseMsg.pose.position.x = leftHandPose.position.x();
    leftHandPoseMsg.pose.position.y = leftHandPose.position.y();
    leftHandPoseMsg.pose.position.z = leftHandPose.position.z();
    leftHandPoseMsg.pose.orientation.w = leftHandPose.quaternion.w();
    leftHandPoseMsg.pose.orientation.x = leftHandPose.quaternion.x();
    leftHandPoseMsg.pose.orientation.y = leftHandPose.quaternion.y();
    leftHandPoseMsg.pose.orientation.z = leftHandPose.quaternion.z();
    leftHandPoseFromTransformerPublisher_.publish(leftHandPoseMsg);
  }

  // 发布来自Transformer的右手pose
  const auto& rightHandPose = quest3ArmInfoTransformerPtr_->getRightHandPose();
  if (rightHandPose.isValid()) {
    geometry_msgs::PoseStamped rightHandPoseMsg;
    rightHandPoseMsg.header.stamp = ros::Time::now();
    rightHandPoseMsg.header.frame_id = "base_link";
    rightHandPoseMsg.pose.position.x = rightHandPose.position.x();
    rightHandPoseMsg.pose.position.y = rightHandPose.position.y();
    rightHandPoseMsg.pose.position.z = rightHandPose.position.z();
    rightHandPoseMsg.pose.orientation.w = rightHandPose.quaternion.w();
    rightHandPoseMsg.pose.orientation.x = rightHandPose.quaternion.x();
    rightHandPoseMsg.pose.orientation.y = rightHandPose.quaternion.y();
    rightHandPoseMsg.pose.orientation.z = rightHandPose.quaternion.z();
    rightHandPoseFromTransformerPublisher_.publish(rightHandPoseMsg);
  }
}

void Quest3IkIncrementalROS::initialize(const nlohmann::json& configJson) {
  initializeBase(configJson);

  // 初始化pose约束列表
  latestPoseConstraintList_.resize(POSE_DATA_LIST_SIZE_PLUS, PoseData());

  // 初始化机器人固定位置参数（从 JSON 加载或使用默认值）
  loadDrakeVelocityIKGeometryFromJson(configJson);

  leftElbowFixedPoint_ = Eigen::Vector3d(-0.3, 0.5, 0.32);
  rightElbowFixedPoint_ = Eigen::Vector3d(-0.3, -0.5, 0.32);

  // 从主控制器实时订阅当前手臂控制模式
  arm_ctrl_mode_vr_sub_ = nodeHandle_.subscribe(
  "/humanoid/mpc/arm_control_mode", 1, &Quest3IkIncrementalROS::armCtrlModeCallback, this);

  //从JSON配置读取手臂关节数量
  if (configJson.contains("NUM_ARM_JOINT")) {
    jointStateSize_ = configJson["NUM_ARM_JOINT"].get<int>();
    ROS_INFO("✅ [Quest3IkIncrementalROS] Set arm joints count from JSON: %d", jointStateSize_);
  } else {
    ROS_ERROR("❌ [Quest3IkIncrementalROS] 'NUM_ARM_JOINT' field not found in JSON configuration");
    throw std::runtime_error("Missing 'NUM_ARM_JOINT' field in JSON configuration");
  }

  //从JSON配置读取腰部自由度数量
  if (configJson.contains("NUM_WAIST_JOINT")) {
    waist_dof_ = configJson["NUM_WAIST_JOINT"].get<int>();
    ROS_INFO("✅ [Quest3IkIncrementalROS] Set waist DOF from JSON: %d", waist_dof_);
  } else {
    ROS_WARN("⚠️  [Quest3IkIncrementalROS] 'NUM_WAIST_JOINT' field not found in JSON configuration, using default value: 0");
    waist_dof_ = 0;
  }

  // 初始化 sensorData 双臂关节角（14维，rad）指数均值滤波状态
  sensorArmJointQ_ = Eigen::VectorXd::Zero(SENSOR_ARM_JOINT_DIM);

  // 初始化关节角度fhan滤波状态
  q_.resize(jointStateSize_);
  dq_.resize(jointStateSize_);
  q_.setZero();
  dq_.setZero();

  // 初始化最新的关节状态
  latest_q_.resize(jointStateSize_);
  latest_dq_.resize(jointStateSize_);
  lowpass_dq_.resize(jointStateSize_);
  latest_q_.setZero();
  latest_dq_.setZero();
  lowpass_dq_.setZero();

  // 从JSON配置读取lowpass_dq_滤波因子
  if (configJson.contains("lowpass_dq_filter")) {
    const auto& filterConfig = configJson["lowpass_dq_filter"];
    if (filterConfig.is_object()) {
      if (filterConfig.contains("alpha")) {
        lowpassDqAlpha_ = filterConfig["alpha"].get<double>();
        ROS_INFO("✅ [Quest3IkIncrementalROS] Set lowpass_dq_ alpha from JSON: %.3f (beta=%.3f)",
                 lowpassDqAlpha_,
                 1.0 - lowpassDqAlpha_);
      }
    } else {
      ROS_WARN("❌ [Quest3IkIncrementalROS] 'lowpass_dq_filter' is not an object, using default value (alpha=%.3f)",
               lowpassDqAlpha_);
    }
  } else {
    ROS_INFO(
        "ℹ️  [Quest3IkIncrementalROS] 'lowpass_dq_filter' not found in JSON config, using default value "
        "(alpha=%.3f)",
        lowpassDqAlpha_);
  }

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
    // Eigen::VectorXd limit_lower = Eigen::VectorXd::Constant(1, -1.57);
    // Eigen::VectorXd limit_upper = Eigen::VectorXd::Constant(1, 1.57);

    // plant.GetMutableJointByName("zarm_l5_joint").set_position_limits(1.36 * limit_lower, 1.36 * limit_upper);
    // plant.GetMutableJointByName("zarm_r5_joint").set_position_limits(1.36 * limit_lower, 1.36 * limit_upper);

    // plant.GetMutableJointByName("zarm_l6_joint").set_position_limits(limit_lower, limit_upper);
    // plant.GetMutableJointByName("zarm_r6_joint").set_position_limits(limit_lower, limit_upper);

    // plant.GetMutableJointByName("zarm_l7_joint").set_position_limits(limit_lower, limit_upper);
    // plant.GetMutableJointByName("zarm_r7_joint").set_position_limits(limit_lower, limit_upper);
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

  // OneStageIKEndEffector initialization
  std::vector<std::string> frameNames = loadFrameNamesFromConfig(configJson);

  // Load PointTrackIKSolverConfig from JSON
  auto pointTrackConfig = loadPointTrackIKSolverConfigFromJson(configJson);
  oneStageIkEndEffectorPtr_ =
      std::make_unique<HighlyDynamic::OneStageIKEndEffector>(&plant, frameNames, pointTrackConfig);

  // 计算并保存 ArmJoint 为全零时的双手位姿，避免运行时频繁调用 FK
  Eigen::VectorXd armJoints = Eigen::VectorXd::Zero(jointStateSize_);

  // 计算 Link6 位姿（用于 IK 约束）
  auto [leftLink6Position, leftLink6Quaternion] =
      oneStageIkEndEffectorPtr_->FK(armJoints, "zarm_l6_link", jointStateSize_);
  auto [rightLink6Position, rightLink6Quaternion] =
      oneStageIkEndEffectorPtr_->FK(armJoints, "zarm_r6_link", jointStateSize_);
  initZeroLeftLink6Position_ = leftLink6Position;
  initZeroRightLink6Position_ = rightLink6Position;

  // 计算 End Effector 位姿（用于可视化等）
  auto [leftEndEffectorPosition, leftEndEffectorQuaternion] =
      oneStageIkEndEffectorPtr_->FK(armJoints, "zarm_l7_end_effector", jointStateSize_);
  auto [rightEndEffectorPosition, rightEndEffectorQuaternion] =
      oneStageIkEndEffectorPtr_->FK(armJoints, "zarm_r7_end_effector", jointStateSize_);
  initZeroLeftEndEffectorPosition_ = leftEndEffectorPosition;
  initZeroRightEndEffectorPosition_ = rightEndEffectorPosition;

  // 初始化四个点的位置（从URDF关节位置数据）
  leftLink6Position_ << -0.017300, 0.292700, -0.092700;
  rightLink6Position_ << -0.017500, -0.292700, -0.092700;
  leftEndEffectorPosition_ << -0.017300, 0.262700, -0.283700;
  rightEndEffectorPosition_ << -0.017300, -0.262700, -0.283700;

  // thumb位置待定，暂时初始化为零向量
  leftVirtualThumbPosition_ << leftEndEffectorPosition_.x() + 0.15, leftEndEffectorPosition_.y(),
      leftEndEffectorPosition_.z();
  rightVirtualThumbPosition_ << rightEndEffectorPosition_.x() + 0.15, rightEndEffectorPosition_.y(),
      rightEndEffectorPosition_.z();

  leftEE2Link6Offset_ = leftEndEffectorPosition_ - leftLink6Position_;
  rightEE2Link6Offset_ = rightEndEffectorPosition_ - rightLink6Position_;
  leftThumb2Link6Offset_ = leftVirtualThumbPosition_ - leftLink6Position_;
  rightThumb2Link6Offset_ = rightVirtualThumbPosition_ - rightLink6Position_;
  ROS_INFO(
      "[Quest3IkIncrementalROS] Initialized zero joint pose - Link6: left=[%.4f, %.4f, %.4f], right=[%.4f, %.4f, "
      "%.4f]",
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
  ikSolvedEefPosePublisher_ = nodeHandle_.advertise<kuavo_msgs::twoArmHandPose>("/ik_fk_result/eef_pose", 10);
  ikInputPosPublisher_ = nodeHandle_.advertise<kuavo_msgs::Float32MultiArrayStamped>("/ik_fk_result/input_pos", 10);

  // 初始化增量控制模块
  IncrementalControlConfig incrementalConfig;
  // 使用 nodeHandle_ (命名空间为 /quest3) 来读取参数，自动跳过节点前缀
  while (!nodeHandle_.hasParam("/ik_ros_uni_cpp_node/quest3/fhan_r")) {
    ROS_WARN("[Quest3IkIncrementalROS] Waiting for /quest3/fhan_r parameter");
    ros::Duration(0.1).sleep();
  }
  PARAM_AND_PRINT_FLOAT(nodeHandle_, "/ik_ros_uni_cpp_node/quest3/fhan_r", incrementalConfig.fhanR, 900.0, 1);

  while (!nodeHandle_.hasParam("/ik_ros_uni_cpp_node/quest3/fhan_kh0")) {
    ROS_WARN("[Quest3IkIncrementalROS] Waiting for /quest3/fhan_kh0 parameter");
    ros::Duration(0.1).sleep();
  }
  PARAM_AND_PRINT_FLOAT(nodeHandle_, "/ik_ros_uni_cpp_node/quest3/fhan_kh0", incrementalConfig.fhanKh0, 6.0, 1);

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
  PARAM_AND_PRINT_FLOAT(nodeHandle_, "/ik_ros_uni_cpp_node/quest3/fhan_r_joint", fhanRJoint_, 900.0, 1);
  PARAM_AND_PRINT_FLOAT(nodeHandle_, "/ik_ros_uni_cpp_node/quest3/fhan_kh0_joint", fhanKh0Joint_, 6.0, 1);
  PARAM_AND_PRINT_FLOAT(nodeHandle_, "/ik_ros_uni_cpp_node/quest3/max_joint_velocity", maxJointVelocity_, 30.0, 3);

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
                           Eigen::Vector3d(0, 0.06, 0),
                           2,
                           "[Quest3IkIncrementalROS]");
  PARAM_AND_PRINT_VECTOR3D(nodeHandle_,
                           "/quest3/right_center",  //
                           rightCenter_,
                           Eigen::Vector3d(0, -0.06, 0),
                           2,
                           "[Quest3IkIncrementalROS]");

  // 读取手到肘距离约束参数
  PARAM_AND_PRINT_FLOAT(nodeHandle_, "/ik_ros_uni_cpp_node/quest3/elbow_min_distance", elbowMinDistance_, 0.18, 3);
  PARAM_AND_PRINT_FLOAT(nodeHandle_, "/ik_ros_uni_cpp_node/quest3/elbow_max_distance", elbowMaxDistance_, 0.65, 3);

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

  // 读取zyx限制参数
  PARAM_AND_PRINT_VECTOR3D(nodeHandle_,
                           "/quest3/zyx_limits_final",
                           incrementalConfig.zyxLimitsFinal,
                           Eigen::Vector3d(0.95 * M_PI / 2.0, M_PI / 2.0, M_PI / 2.0),
                           2,
                           "[Quest3IkIncrementalROS]");
  PARAM_AND_PRINT_VECTOR3D(nodeHandle_,
                           "/quest3/zyx_limits_ee",
                           incrementalConfig.zyxLimitsEE,
                           Eigen::Vector3d(M_PI / 2.0, M_PI / 2.0, M_PI / 2.0),
                           2,
                           "[Quest3IkIncrementalROS]");
  PARAM_AND_PRINT_VECTOR3D(nodeHandle_,
                           "/quest3/zyx_limits_link4",
                           incrementalConfig.zyxLimitsLink4,
                           Eigen::Vector3d(M_PI / 2.0, 0.6, 0.6),
                           2,
                           "[Quest3IkIncrementalROS]");

  incrementalController_ = std::make_unique<IncrementalControlModule>(incrementalConfig);

  quest3ArmInfoTransformerPtr_->setDeltaScale(deltaScale_);

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

  // 初始化 Drake 二连杆肘手点优化求解器
  {
    // Load DrakeVelocityIKWeights from JSON
    auto drakeVelocityWeights = loadDrakeVelocityIKWeightsFromJson(configJson);
    auto drakeVelocityBounds = loadDrakeVelocityIKBoundsFromJson(configJson);

    // 初始化 leftVelocityIkSolverPtr_
    {
      // 使用从 JSON 加载的 left shoulder 位置作为 p0
      const Eigen::Vector3d& p0 = robotLeftFixedShoulderPos_;

      Eigen::Vector3d leftLb(p0.x(),                          //
                             drakeVelocityBounds.leftYLower,  //
                             drakeVelocityBounds.zLower);

      Eigen::Vector3d leftUb(p0.x() + drakeVelocityBounds.xUpperOffset,
                             p0.y() + drakeVelocityBounds.leftYUpperOffset,
                             p0.z() + drakeVelocityBounds.zUpperOffset);

      leftVelocityIkSolverPtr_ = std::make_unique<DrakeVelocityIKSolver>(p0, l1_, l2_, leftLb, leftUb);
      // 使用从 JSON 加载的权重配置
      leftVelocityIkSolverPtr_->setWeights(drakeVelocityWeights);
      ROS_INFO(
          "[Quest3IkIncrementalROS] LeftVelocityIKSolver initialized (p0=[%.3f,%.3f,%.3f], l1=%.4f, l2=%.4f, "
          "weights: q11=%.3f, q12=%.3f, q2=%.3f, qv1=%.3f, qv2=%.3f)",
          p0.x(),
          p0.y(),
          p0.z(),
          l1_,
          l2_,
          drakeVelocityWeights.q11,
          drakeVelocityWeights.q12,
          drakeVelocityWeights.q2,
          drakeVelocityWeights.qv1,
          drakeVelocityWeights.qv2);
    }

    // 初始化 rightVelocityIkSolverPtr_
    {
      // 使用从 JSON 加载的 right shoulder 位置作为 p0
      const Eigen::Vector3d& p0 = robotRightFixedShoulderPos_;

      Eigen::Vector3d rightLb(p0.x(),                                          //
                              p0.y() + drakeVelocityBounds.rightYLowerOffset,  //
                              drakeVelocityBounds.zLower);

      Eigen::Vector3d rightUb(p0.x() + drakeVelocityBounds.xUpperOffset,
                              drakeVelocityBounds.rightYUpper,
                              p0.z() + drakeVelocityBounds.zUpperOffset);

      rightVelocityIkSolverPtr_ = std::make_unique<DrakeVelocityIKSolver>(p0, l1_, l2_, rightLb, rightUb);
      // 使用从 JSON 加载的权重配置
      rightVelocityIkSolverPtr_->setWeights(drakeVelocityWeights);
      ROS_INFO(
          "[Quest3IkIncrementalROS] RightVelocityIKSolver initialized (p0=[%.3f,%.3f,%.3f], l1=%.4f, l2=%.4f, "
          "weights: q11=%.3f, q12=%.3f, q2=%.3f, qv1=%.3f, qv2=%.3f)",
          p0.x(),
          p0.y(),
          p0.z(),
          l1_,
          l2_,
          drakeVelocityWeights.q11,
          drakeVelocityWeights.q12,
          drakeVelocityWeights.q2,
          drakeVelocityWeights.qv1,
          drakeVelocityWeights.qv2);
    }
  }

  // 初始化增量模块的手部姿态种子，避免首次进入增量模式时从单位四元数开始插值
  if (incrementalController_) {
    incrementalController_->setHandQuatSeeds(defaultHandQuat, defaultHandQuat, useIncrementalHandOrientation_);
  }

  ROS_INFO("[Quest3IkIncrementalROS] Interpolation system initialized successfully");
}

void Quest3IkIncrementalROS::updateLeftConstraintList(const Eigen::Vector3d& leftHandPos,
                                                      const Eigen::Quaterniond& leftHandQuat,
                                                      const Eigen::Vector3d& leftElbowPos) {
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].position = leftHandPos;

  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].rotation_matrix = leftHandQuat.toRotationMatrix();

  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_ELBOW].position = leftElbowPos;

  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_END_EFFECTOR].position = leftEndEffectorPosition_;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_LINK6].position = leftLink6Position_;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_VIRTUAL_THUMB].position = leftVirtualThumbPosition_;
}

void Quest3IkIncrementalROS::updateRightConstraintList(const Eigen::Vector3d& rightHandPos,
                                                       const Eigen::Quaterniond& rightHandQuat,
                                                       const Eigen::Vector3d& rightElbowPos) {
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].position = rightHandPos;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].rotation_matrix = rightHandQuat.toRotationMatrix();

  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_ELBOW].position = rightElbowPos;

  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_END_EFFECTOR].position = rightEndEffectorPosition_;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_LINK6].position = rightLink6Position_;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_VIRTUAL_THUMB].position = rightVirtualThumbPosition_;
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

bool Quest3IkIncrementalROS::updateLatestIncrementalResult() {
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

  latestIncrementalResult_ =
      incrementalController_->computeIncrementalPose(quest3ArmInfoTransformerPtr_->getLeftHandPose(),
                                                     quest3ArmInfoTransformerPtr_->getRightHandPose(),
                                                     isLeftActive,
                                                     isRightActive,
                                                     qLeftEndEffector,
                                                     qRightEndEffector);
  return latestIncrementalResult_.isValid();
}

bool Quest3IkIncrementalROS::updateLeftHandChangingMode(const Eigen::Vector3d& leftTargetPos) {
  // 使用均值滤波后的关节数据
  if (sensorArmJointQ_.size() != jointStateSize_) {
    ROS_WARN("[Quest3IkIncrementalROS] Filtered joint data not available for FK calculation");
    return false;
  }

  Eigen::VectorXd armJoints = sensorArmJointQ_;
  return leftHandSmoother_->updateChangingMode(leftTargetPos,
                                               static_cast<BaseIKSolver*>(oneStageIkEndEffectorPtr_.get()),
                                               armJoints,
                                               jointStateSize_,
                                               handChangingModeThreshold_);
}

bool Quest3IkIncrementalROS::updateRightHandChangingMode(const Eigen::Vector3d& rightTargetPos) {
  // 使用均值滤波后的关节数据
  if (sensorArmJointQ_.size() != jointStateSize_) {
    ROS_WARN("[Quest3IkIncrementalROS] Filtered joint data not available for FK calculation");
    return false;
  }

  Eigen::VectorXd armJoints = sensorArmJointQ_;
  return rightHandSmoother_->updateChangingMode(rightTargetPos,
                                                static_cast<BaseIKSolver*>(oneStageIkEndEffectorPtr_.get()),
                                                armJoints,
                                                jointStateSize_,
                                                handChangingModeThreshold_);
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
    if (latestIkSolution_.size() != jointStateSize_) {
      latestIkSolution_.resize(jointStateSize_);
    }
    latestIkSolution_.setZero();
    hasValidIkSolution_ = false;
  }
  // 重置pose约束列表，使用默认手部位置初始化，确保进入增量模式时能正确初始化到默认位置
  {
    std::lock_guard<std::mutex> lock(poseConstraintListMutex_);
    latestPoseConstraintList_.resize(POSE_DATA_LIST_SIZE_PLUS, PoseData());
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
  // 重置跳变检测时间戳
  leftHandSpikeStartTime_ = ros::Time(0);
  rightHandSpikeStartTime_ = ros::Time(0);
  // 重置后同步增量控制模块的手部姿态种子
  if (incrementalController_) {
    Eigen::Quaterniond defaultHandQuat = Eigen::Quaterniond::Identity();
    incrementalController_->setHandQuatSeeds(defaultHandQuat, defaultHandQuat, useIncrementalHandOrientation_);
  }
}

PointTrackIKSolverConfig Quest3IkIncrementalROS::loadPointTrackIKSolverConfigFromJson(
    const nlohmann::json& configJson) {
  ROS_INFO("==================================================================================");
  ROS_INFO("🔧 [Quest3IkIncrementalROS] Loading PointTrackIKSolverConfig from JSON configuration");
  ROS_INFO("==================================================================================");

  PointTrackIKSolverConfig config;

  try {
    if (configJson.contains("point_track_ik_solver_config") && configJson["point_track_ik_solver_config"].is_object()) {
      const auto& ikConfig = configJson["point_track_ik_solver_config"];

      // Load PointTrackIKSolverConfig specific fields
      if (ikConfig.contains("historyBufferSize")) {
        config.historyBufferSize = ikConfig["historyBufferSize"].get<int>();
        ROS_INFO("  ✅ historyBufferSize: %d", config.historyBufferSize);
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

      ROS_INFO("✅ [Quest3IkIncrementalROS] Successfully loaded PointTrackIKSolverConfig from JSON");
    } else {
      ROS_WARN("❌ [Quest3IkIncrementalROS] 'point_track_ik_solver_config' not found in JSON config, using defaults");
    }

  } catch (const std::exception& e) {
    ROS_ERROR("❌ [Quest3IkIncrementalROS] Exception while loading PointTrackIKSolverConfig: %s", e.what());
    ROS_WARN("🔄 [Quest3IkIncrementalROS] Falling back to default configuration");
  }

  return config;
}

DrakeVelocityIKWeightConfig Quest3IkIncrementalROS::loadDrakeVelocityIKWeightsFromJson(
    const nlohmann::json& configJson) {
  ROS_INFO("==================================================================================");
  ROS_INFO("🔧 [Quest3IkIncrementalROS] Loading DrakeVelocityIKWeights from JSON configuration");
  ROS_INFO("==================================================================================");

  // Default values matching HandPriority config
  static const char* defaultName = "JSON_Loaded";
  DrakeVelocityIKWeightConfig config;
  config.name = defaultName;
  config.q11 = 0.5;
  config.q12 = 0.5;
  config.q2 = 500.0;
  config.qv1 = 0.05;
  config.qv2 = 0.001;

  try {
    if (configJson.contains("drake_velocity_ik_weights") && configJson["drake_velocity_ik_weights"].is_object()) {
      const auto& weightsConfig = configJson["drake_velocity_ik_weights"];

      if (weightsConfig.contains("q11")) {
        config.q11 = weightsConfig["q11"].get<double>();
        ROS_INFO("  ✅ q11 (P1 fixed weight): %.6e", config.q11);
      }

      if (weightsConfig.contains("q12")) {
        config.q12 = weightsConfig["q12"].get<double>();
        ROS_INFO("  ✅ q12 (P1 ref weight): %.6e", config.q12);
      }

      if (weightsConfig.contains("q2")) {
        config.q2 = weightsConfig["q2"].get<double>();
        ROS_INFO("  ✅ q2 (P2 ref weight): %.6e", config.q2);
      }

      if (weightsConfig.contains("qv1")) {
        config.qv1 = weightsConfig["qv1"].get<double>();
        ROS_INFO("  ✅ qv1 (P1 smoothness weight): %.6e", config.qv1);
      }

      if (weightsConfig.contains("qv2")) {
        config.qv2 = weightsConfig["qv2"].get<double>();
        ROS_INFO("  ✅ qv2 (P2 smoothness weight): %.6e", config.qv2);
      }

      ROS_INFO("✅ [Quest3IkIncrementalROS] Successfully loaded DrakeVelocityIKWeights from JSON");
      ROS_INFO("   Config: q11=%.6e, q12=%.6e, q2=%.6e, qv1=%.6e, qv2=%.6e",
               config.q11,
               config.q12,
               config.q2,
               config.qv1,
               config.qv2);
    } else {
      ROS_WARN(
          "❌ [Quest3IkIncrementalROS] 'drake_velocity_ik_weights' not found in JSON config, using defaults "
          "(HandPriority)");
    }

  } catch (const std::exception& e) {
    ROS_ERROR("❌ [Quest3IkIncrementalROS] Exception while loading DrakeVelocityIKWeights: %s", e.what());
    ROS_WARN("🔄 [Quest3IkIncrementalROS] Falling back to default configuration (HandPriority)");
  }

  return config;
}

DrakeVelocityIKBoundsConfig Quest3IkIncrementalROS::loadDrakeVelocityIKBoundsFromJson(
    const nlohmann::json& configJson) {
  ROS_INFO("==================================================================================");
  ROS_INFO("🔧 [Quest3IkIncrementalROS] Loading DrakeVelocityIKBounds from JSON configuration");
  ROS_INFO("==================================================================================");

  DrakeVelocityIKBoundsConfig config;

  try {
    if (configJson.contains("drake_velocity_ik_bounds") && configJson["drake_velocity_ik_bounds"].is_object()) {
      const auto& boundsConfig = configJson["drake_velocity_ik_bounds"];

      if (boundsConfig.contains("x_upper_offset")) {
        config.xUpperOffset = boundsConfig["x_upper_offset"].get<double>();
        ROS_INFO("  ✅ x_upper_offset: %.6f", config.xUpperOffset);
      }

      if (boundsConfig.contains("z_lower")) {
        config.zLower = boundsConfig["z_lower"].get<double>();
        ROS_INFO("  ✅ z_lower: %.6f", config.zLower);
      }

      if (boundsConfig.contains("z_upper_offset")) {
        config.zUpperOffset = boundsConfig["z_upper_offset"].get<double>();
        ROS_INFO("  ✅ z_upper_offset: %.6f", config.zUpperOffset);
      }

      if (boundsConfig.contains("left_y_lower")) {
        config.leftYLower = boundsConfig["left_y_lower"].get<double>();
        ROS_INFO("  ✅ left_y_lower: %.6f", config.leftYLower);
      }

      if (boundsConfig.contains("left_y_upper_offset")) {
        config.leftYUpperOffset = boundsConfig["left_y_upper_offset"].get<double>();
        ROS_INFO("  ✅ left_y_upper_offset: %.6f", config.leftYUpperOffset);
      }

      if (boundsConfig.contains("right_y_lower_offset")) {
        config.rightYLowerOffset = boundsConfig["right_y_lower_offset"].get<double>();
        ROS_INFO("  ✅ right_y_lower_offset: %.6f", config.rightYLowerOffset);
      }

      if (boundsConfig.contains("right_y_upper")) {
        config.rightYUpper = boundsConfig["right_y_upper"].get<double>();
        ROS_INFO("  ✅ right_y_upper: %.6f", config.rightYUpper);
      }

      ROS_INFO("✅ [Quest3IkIncrementalROS] Successfully loaded DrakeVelocityIKBounds from JSON");
    } else {
      ROS_WARN("❌ [Quest3IkIncrementalROS] 'drake_velocity_ik_bounds' not found in JSON config, using defaults");
    }

  } catch (const std::exception& e) {
    ROS_ERROR("❌ [Quest3IkIncrementalROS] Exception while loading DrakeVelocityIKBounds: %s", e.what());
    ROS_WARN("🔄 [Quest3IkIncrementalROS] Falling back to default configuration");
  }

  ROS_INFO(
      "   Bounds constants: x_upper_offset=%.6f, z_lower=%.6f, z_upper_offset=%.6f, left_y_lower=%.6f, "
      "left_y_upper_offset=%.6f, right_y_lower_offset=%.6f, right_y_upper=%.6f",
      config.xUpperOffset,
      config.zLower,
      config.zUpperOffset,
      config.leftYLower,
      config.leftYUpperOffset,
      config.rightYLowerOffset,
      config.rightYUpper);

  return config;
}

void Quest3IkIncrementalROS::loadDrakeVelocityIKGeometryFromJson(const nlohmann::json& configJson) {
  ROS_INFO("==================================================================================");
  ROS_INFO("🔧 [Quest3IkIncrementalROS] Loading DrakeVelocityIK Geometry from JSON configuration");
  ROS_INFO("==================================================================================");

  // Default values
  double defaultL1 = 0.2837;  // ||j2_j4||
  double defaultL2 = 0.2335;  // ||j4_j6||
  Eigen::Vector3d defaultLeftP0(-0.017499853, 0.29269999999999996, 0.4245);
  Eigen::Vector3d defaultRightP0(-0.017499853, -0.29269999999999996, 0.4245);

  // Initialize with defaults
  l1_ = defaultL1;
  l2_ = defaultL2;
  robotLeftFixedShoulderPos_ = defaultLeftP0;
  robotRightFixedShoulderPos_ = defaultRightP0;

  try {
    if (configJson.contains("drake_velocity_ik_geometry") && configJson["drake_velocity_ik_geometry"].is_object()) {
      const auto& geometryConfig = configJson["drake_velocity_ik_geometry"];

      // Load link lengths
      if (geometryConfig.contains("l1")) {
        l1_ = geometryConfig["l1"].get<double>();
        ROS_INFO("  ✅ l1 (link1 length): %.6f", l1_);
      } else {
        ROS_INFO("  ⚠️  l1 not found, using default: %.6f", defaultL1);
      }

      if (geometryConfig.contains("l2")) {
        l2_ = geometryConfig["l2"].get<double>();
        ROS_INFO("  ✅ l2 (link2 length): %.6f", l2_);
      } else {
        ROS_INFO("  ⚠️  l2 not found, using default: %.6f", defaultL2);
      }

      // Load left shoulder position (p0)
      if (geometryConfig.contains("left_p0") && geometryConfig["left_p0"].is_array() &&
          geometryConfig["left_p0"].size() == 3) {
        std::vector<double> leftP0Vec = geometryConfig["left_p0"].get<std::vector<double>>();
        robotLeftFixedShoulderPos_ = Eigen::Vector3d(leftP0Vec[0], leftP0Vec[1], leftP0Vec[2]);
        ROS_INFO("  ✅ left_p0: [%.6f, %.6f, %.6f]",
                 robotLeftFixedShoulderPos_.x(),
                 robotLeftFixedShoulderPos_.y(),
                 robotLeftFixedShoulderPos_.z());
      } else {
        ROS_INFO("  ⚠️  left_p0 not found or invalid, using default: [%.6f, %.6f, %.6f]",
                 defaultLeftP0.x(),
                 defaultLeftP0.y(),
                 defaultLeftP0.z());
      }

      // Load right shoulder position (p0)
      if (geometryConfig.contains("right_p0") && geometryConfig["right_p0"].is_array() &&
          geometryConfig["right_p0"].size() == 3) {
        std::vector<double> rightP0Vec = geometryConfig["right_p0"].get<std::vector<double>>();
        robotRightFixedShoulderPos_ = Eigen::Vector3d(rightP0Vec[0], rightP0Vec[1], rightP0Vec[2]);
        ROS_INFO("  ✅ right_p0: [%.6f, %.6f, %.6f]",
                 robotRightFixedShoulderPos_.x(),
                 robotRightFixedShoulderPos_.y(),
                 robotRightFixedShoulderPos_.z());
      } else {
        ROS_INFO("  ⚠️  right_p0 not found or invalid, using default: [%.6f, %.6f, %.6f]",
                 defaultRightP0.x(),
                 defaultRightP0.y(),
                 defaultRightP0.z());
      }

      ROS_INFO("✅ [Quest3IkIncrementalROS] Successfully loaded DrakeVelocityIK Geometry from JSON");
      ROS_INFO("   Geometry: l1=%.6f, l2=%.6f", l1_, l2_);
      ROS_INFO("   Left p0: [%.6f, %.6f, %.6f]",
               robotLeftFixedShoulderPos_.x(),
               robotLeftFixedShoulderPos_.y(),
               robotLeftFixedShoulderPos_.z());
      ROS_INFO("   Right p0: [%.6f, %.6f, %.6f]",
               robotRightFixedShoulderPos_.x(),
               robotRightFixedShoulderPos_.y(),
               robotRightFixedShoulderPos_.z());
    } else {
      ROS_WARN("❌ [Quest3IkIncrementalROS] 'drake_velocity_ik_geometry' not found in JSON config, using defaults");
      ROS_INFO("   Default geometry: l1=%.6f, l2=%.6f", defaultL1, defaultL2);
      ROS_INFO("   Default left_p0: [%.6f, %.6f, %.6f]", defaultLeftP0.x(), defaultLeftP0.y(), defaultLeftP0.z());
      ROS_INFO("   Default right_p0: [%.6f, %.6f, %.6f]", defaultRightP0.x(), defaultRightP0.y(), defaultRightP0.z());
    }

  } catch (const std::exception& e) {
    ROS_ERROR("❌ [Quest3IkIncrementalROS] Exception while loading DrakeVelocityIK Geometry: %s", e.what());
    ROS_WARN("🔄 [Quest3IkIncrementalROS] Falling back to default geometry");
  }
}

bool Quest3IkIncrementalROS::validateVrPose(const ArmPose& currentPose, ArmPose& validatedPose, const std::string& side, bool isArmActive) {
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
    ROS_ERROR("[Quest3IkIncrementalROS] Invalid side parameter: %s", side.c_str());
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
        ROS_WARN_THROTTLE(1.0, "[Quest3IkIncrementalROS] %s hand VR pose: timeout recovery triggered (%.3f seconds)",
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
      ROS_INFO_THROTTLE(1.0, "[Quest3IkIncrementalROS] %s hand VR pose: continuous spikes detected, recovering (likely fast normal motion)",
                        side.c_str());
    } else {
      // 检测到跳变，使用前一个点替代
      validatedPose.position = *prev1;
      validatedPose.quaternion = currentPose.quaternion;  // 保持当前姿态
      ROS_WARN_THROTTLE(1.0, "[Quest3IkIncrementalROS] %s hand VR pose spike detected (%d/%d), using previous position",
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
