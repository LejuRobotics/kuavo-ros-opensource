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
#include <visualization_msgs/Marker.h>

#include <leju_utils/define.hpp>
#include <leju_utils/math.hpp>
#include <leju_utils/RosMsgConvertor.hpp>

#include "motion_capture_ik/ArmControlBaseROS.h"
#include "motion_capture_ik/Quest3ArmInfoTransformer.h"
#include "motion_capture_ik/json.hpp"
#include "motion_capture_ik/IncrementalControlModule.h"
#include "motion_capture_ik/KeyFramesVisualizer.h"

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
    fsmEnter();
    fsmProcess();
    fsmExit();
    publishSensorDataArmJoints();
    publishEndEffectorControlData();
    rate.sleep();
  }
}

void Quest3IkIncrementalROS::fsmEnter() {
  if (incrementalController_->shouldEnterIncrementalMode() /* && isRunning()*/) {
    std::lock_guard<std::mutex> lock(poseConstraintListMutex_);
    incrementalController_->enterIncrementalMode(quest3ArmInfoTransformerPtr_->getLeftHandPose(),
                                                 quest3ArmInfoTransformerPtr_->getRightHandPose(),
                                                 quest3ArmInfoTransformerPtr_->getLeftElbowPose(),
                                                 quest3ArmInfoTransformerPtr_->getRightElbowPose(),
                                                 latestPoseConstraintList_);
    std::cout << "\033[92m[enterIncrementalMode] set anchor pose successfully.\033[0m" << std::endl;
  }
}

void Quest3IkIncrementalROS::fsmProcess() {
  if (armModeChanging_.load()) {
    std::cout << "\033[92m[Quest3IkIncrementalROS] Arm mode changing, not returning\033[0m" << std::endl;
  } else {
    if (!incrementalController_->isIncrementalMode() || !detectHumanArmMove() || !updateLatestIncrementalResult())
      return;
  }

  solveIkWithPoseConstraints();
  activateController();
  publishJointStates();
}

void Quest3IkIncrementalROS::fsmExit() {
  if (!incrementalController_->shouldExitIncrementalMode()) return;
  deactivateController();
  std::lock_guard<std::mutex> lock(poseConstraintListMutex_);
  incrementalController_->exitIncrementalMode(quest3ArmInfoTransformerPtr_->getLeftHandPose(),
                                              quest3ArmInfoTransformerPtr_->getRightHandPose(),
                                              quest3ArmInfoTransformerPtr_->getLeftElbowPose(),
                                              quest3ArmInfoTransformerPtr_->getRightElbowPose(),
                                              latestPoseConstraintList_);
  std::cout << "\033[93m[Quest3IkIncrementalROS] Exiting incremental mode\033[0m" << std::endl;
}

void Quest3IkIncrementalROS::solveIkWithPoseConstraints() {
  std::shared_ptr<noitom_hi5_hand_udp_python::PoseInfoList> bonePoseHandElbowPtr;
  {
    std::lock_guard<std::mutex> lock(bonePoseHandElbowMutex_);
    bonePoseHandElbowPtr = HandPoseAndElbowPositonListPtr_;
  }

  if (bonePoseHandElbowPtr == nullptr) return;
  if (bonePoseHandElbowPtr->poses.size() < 4) return;

  //[chest, l_hand, r_hand, l_elbow, r_elbow]
  {
    auto [incrementalLeftQuat, incrementalRightQuat, scaledLeftHandPos, scaledRightHandPos] =
        latestIncrementalResult_.getLatestIncrementalHandPose();

    if (armModeChanging_) {
      scaledLeftHandPos = defaultLeftHandPosOnExit_;
      incrementalLeftQuat = Eigen::Quaterniond::Identity();

      scaledRightHandPos = defaultRightHandPosOnExit_;
      incrementalRightQuat = Eigen::Quaterniond::Identity();
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

    //
    updateConstraintList(scaledLeftHandPos,
                         incrementalLeftQuat,
                         scaledRightHandPos,
                         incrementalRightQuat,
                         quest3ArmInfoTransformerPtr_->getLeftElbowPose().position,
                         quest3ArmInfoTransformerPtr_->getRightElbowPose().position);

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
        incrementalController_->updateHandFkPositions(leftMeasuredPosition, rightMeasuredPosition);

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

  std::lock_guard<std::mutex> lock(poseConstraintListMutex_);
  auto ikResult = twoStageTorsoIkPtr_->solveIK(latestPoseConstraintList_, ctrlArmIdx_, jointMidValues_);

  if (ikResult.isSuccess) {
    {
      std::lock_guard<std::mutex> lock(ikResultMutex_);
      latestIkSolution_ = ikResult.solution;
      hasValidIkSolution_ = true;
    }

    // 当处于切换模式时，使用传感器数据做 FK，判断是否到达默认目标位置
    if (armModeChanging_.load()) {
      std::shared_ptr<kuavo_msgs::sensorsData> currentSensorData = getSensorData();
      if (!currentSensorData || currentSensorData->joint_data.joint_q.size() < 12 + jointStateSize_) {
        ROS_WARN("[Quest3IkIncrementalROS] Sensor data not available or insufficient for FK calculation");
      } else {
        Eigen::VectorXd armJoints(jointStateSize_);
        for (int i = 0; i < jointStateSize_; ++i) {
          armJoints(i) = currentSensorData->joint_data.joint_q[12 + i];
        }

        auto [leftPosReached, leftQuatTmp] = twoStageTorsoIkPtr_->FK(armJoints, "zarm_l6_link", jointStateSize_);
        auto [rightPosReached, rightQuatTmp] = twoStageTorsoIkPtr_->FK(armJoints, "zarm_r6_link", jointStateSize_);

        double leftErr = (leftPosReached - defaultLeftHandPosOnExit_).norm();
        double rightErr = (rightPosReached - defaultRightHandPosOnExit_).norm();

        if (leftErr < (0.02) && rightErr < (0.02)) {
          armModeChanging_.store(false);
          ROS_INFO("[Quest3IkIncrementalROS] Default handpose reached (L=%.3f, R=%.3f), exit armModeChanging",
                   leftErr,
                   rightErr);
        } else {
          std::cout << "leftErr: " << leftErr << ", rightErr: " << rightErr << std::endl;
          std::cout << "leftPosReached: " << leftPosReached.transpose()
                    << ", rightPosReached: " << rightPosReached.transpose() << std::endl;
          std::cout << "defaultLeftHandPosOnExit_: " << defaultLeftHandPosOnExit_.transpose()
                    << ", defaultRightHandPosOnExit_: " << defaultRightHandPosOnExit_.transpose() << std::endl;
        }
      }
    }
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
  jointStateMsg.name.resize(jointStateSize_);

  for (int i = 0; i < jointStateSize_; ++i) {
    jointStateMsg.name[i] = "arm_joint_" + std::to_string(i + 1);
  }

  Eigen::VectorXd finalArmAngles = armAngleLimited;  // 默认使用目标角度

  // if (onlyHalfUpBody_) {
  //   std::shared_ptr<kuavo_msgs::sensorsData> currentSensorData = getSensorData();

  //   if (!currentSensorData) {
  //     ROS_WARN("[Quest3IkIncrementalROS] sensor_data_raw is None in publishJointStates");
  //     return;
  //   }

  //   if (armModeChanging_) {
  //     Eigen::VectorXd armCurrentState(jointStateSize_);
  //     for (int i = 0; i < jointStateSize_; ++i) {
  //       armCurrentState(i) = currentSensorData->joint_data.joint_q[12 + i];
  //     }

  //     Eigen::VectorXd deltaState = armAngleLimited - armCurrentState;
  //     double totalDistance = deltaState.norm();

  //     if (totalDistance < thresholdArmDiffHalfUpBody_rad_) {
  //       finalArmAngles = armAngleLimited;
  //       armModeChanging_ = false;
  //       ROS_INFO("[Quest3IkIncrementalROS] Interpolation completed, distance: %.4f", totalDistance);
  //     } else {
  //       // 执行插值（复现Python L666-671）
  //       double maxMove = maxSpeed_;
  //       double scale = std::clamp(maxMove / totalDistance, 0.0, 1.0);
  //       finalArmAngles = armCurrentState + deltaState * scale;

  //       if (debugPrint_) {
  //         ROS_DEBUG("[Quest3IkIncrementalROS] Interpolating: distance=%.4f, scale=%.4f", totalDistance, scale);
  //       }
  //     }
  //   }
  // }
  // finalArmAngles.setZero();

  double fhan_h = 1.0 / publishRate_;
  double fhan_h0 = fhan_h * fhan_kh0_joint_;

  if (q_.norm() < 1e-6 && dq_.norm() < 1e-6) {
    q_ = finalArmAngles;
    dq_.setZero();
  }

  for (int i = 0; i < jointStateSize_; ++i) {
    const double targetAngle = finalArmAngles(i);

    leju_utils::fhanStepForwardWithVelLimit(q_(i),                 // 滤波后的关节角度（输出）
                                            dq_(i),                // 滤波后的关节角速度（输出）
                                            targetAngle,           // 目标角度（输入）
                                            fhan_r_joint_,         // 加速度约束
                                            fhan_h,                // 时间步长
                                            fhan_h0,               // 平滑系数
                                            max_joint_velocity_);  // 最大速度限制
  }

  for (int i = 0; i < jointStateSize_; ++i) {
    jointStateMsg.position[i] = q_(i) * 180.0 / M_PI;
  }

  // for (int i = 0; i < jointStateSize_ && i < jointMidValues_.size(); ++i) {
  //   jointStateMsg.position[i] = jointMidValues_(i) * 180.0 / M_PI;  // 转换为角度
  // }
  kuavoArmTrajCppPublisher_.publish(jointStateMsg);
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

  plant.Finalize();

  diagram_ = diagramBuilder->Build();
  diagramContext_ = diagram_->CreateDefaultContext();

  // TwoStageIncrementalIK initialization
  std::vector<std::string> frameNames = loadFrameNamesFromConfig(configJson);
  auto defaultIkSolverConfig = IKSolverConfig();
  twoStageTorsoIkPtr_ =
      std::make_unique<HighlyDynamic::TwoStageIncrementalIK>(&plant, frameNames, defaultIkSolverConfig);

  kuavoArmTrajCppPublisher_ = nodeHandle_.advertise<sensor_msgs::JointState>("/kuavo_arm_traj_cpp", 2);
  sensorDataArmJointsPublisher_ = nodeHandle_.advertise<sensor_msgs::JointState>("/kuavo_arm_traj_sensor_data", 2);

  // 初始化增量控制模块
  IncrementalControlConfig incrementalConfig;
  // 使用 nodeHandle_ (命名空间为 /quest3) 来读取参数，自动跳过节点前缀
  while (!nodeHandle_.hasParam("/incremental_ik_ros_uni_node/quest3/fhan_r")) {
    ROS_WARN("[Quest3IkIncrementalROS] Waiting for /quest3/fhan_r parameter");
    ros::Duration(0.1).sleep();
  }
  PARAM_AND_PRINT_FLOAT(nodeHandle_, "/incremental_ik_ros_uni_node/quest3/fhan_r", incrementalConfig.fhan_r, 900.0, 1);

  while (!nodeHandle_.hasParam("/incremental_ik_ros_uni_node/quest3/fhan_kh0")) {
    ROS_WARN("[Quest3IkIncrementalROS] Waiting for /quest3/fhan_kh0 parameter");
    ros::Duration(0.1).sleep();
  }
  PARAM_AND_PRINT_FLOAT(
      nodeHandle_, "/incremental_ik_ros_uni_node/quest3/fhan_kh0", incrementalConfig.fhan_kh0, 6.0, 1);

  while (!nodeHandle_.hasParam("/incremental_ik_ros_uni_node/quest3/delta_scale_x")) {
    ROS_WARN("[Quest3IkIncrementalROS] Waiting for /quest3/delta_scale_x parameter");
    ros::Duration(0.1).sleep();
  }
  while (!nodeHandle_.hasParam("/incremental_ik_ros_uni_node/quest3/delta_scale_y")) {
    ROS_WARN("[Quest3IkIncrementalROS] Waiting for /quest3/delta_scale_y parameter");
    ros::Duration(0.1).sleep();
  }
  while (!nodeHandle_.hasParam("/incremental_ik_ros_uni_node/quest3/delta_scale_z")) {
    ROS_WARN("[Quest3IkIncrementalROS] Waiting for /quest3/delta_scale_z parameter");
    ros::Duration(0.1).sleep();
  }
  double delta_scale_x, delta_scale_y, delta_scale_z;
  PARAM_AND_PRINT_FLOAT(nodeHandle_, "/incremental_ik_ros_uni_node/quest3/delta_scale_x", delta_scale_x, 1.0, 1);
  PARAM_AND_PRINT_FLOAT(nodeHandle_, "/incremental_ik_ros_uni_node/quest3/delta_scale_y", delta_scale_y, 1.0, 1);
  PARAM_AND_PRINT_FLOAT(nodeHandle_, "/incremental_ik_ros_uni_node/quest3/delta_scale_z", delta_scale_z, 1.0, 1);
  deltaScale_ = Eigen::Vector3d(delta_scale_x, delta_scale_y, delta_scale_z);
  incrementalConfig.deltaScale = deltaScale_;

  PARAM_AND_PRINT_FLOAT(
      nodeHandle_, "/incremental_ik_ros_uni_node/quest3/max_pos_diff", incrementalConfig.maxPosDiff, 0.45, 2);

  // 读取 arm_move_threshold 参数
  while (!nodeHandle_.hasParam("/incremental_ik_ros_uni_node/quest3/arm_move_threshold")) {
    ROS_WARN("[Quest3IkIncrementalROS] Waiting for /quest3/arm_move_threshold parameter");
    ros::Duration(0.1).sleep();
  }
  PARAM_AND_PRINT_FLOAT(nodeHandle_,
                        "/incremental_ik_ros_uni_node/quest3/arm_move_threshold",
                        incrementalConfig.armMoveThreshold,
                        0.01,
                        3);
  incrementalConfig.publishRate = publishRate_;

  // 读取关节角度fhan滤波参数
  PARAM_AND_PRINT_FLOAT(nodeHandle_, "/incremental_ik_ros_uni_node/quest3/fhan_r_joint", fhan_r_joint_, 900.0, 1);
  PARAM_AND_PRINT_FLOAT(nodeHandle_, "/incremental_ik_ros_uni_node/quest3/fhan_kh0_joint", fhan_kh0_joint_, 6.0, 1);
  PARAM_AND_PRINT_FLOAT(
      nodeHandle_, "/incremental_ik_ros_uni_node/quest3/max_joint_velocity", max_joint_velocity_, 1.0, 3);

  // 读取手部位置约束参数
  PARAM_AND_PRINT_FLOAT(
      nodeHandle_, "/incremental_ik_ros_uni_node/quest3/sphere_radius_limit", sphereRadiusLimit_, 0.5, 2);
  PARAM_AND_PRINT_FLOAT(
      nodeHandle_, "/incremental_ik_ros_uni_node/quest3/min_reachable_distance", minReachableDistance_, 0.08, 3);

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
  PARAM_AND_PRINT_FLOAT(nodeHandle_, "/incremental_ik_ros_uni_node/quest3/chest_offset_y_ax", chestOffsetY_, 0.0, 3);

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

  incrementalController_ = IncrementalControlFactory::createCustomModule(
      std::shared_ptr<JoyStickHandler>(joyStickHandlerPtr_.get(), [](JoyStickHandler*) {}), incrementalConfig);

  quest3ArmInfoTransformerPtr_->setDeltaScale(deltaScale_);

  double left_roll, left_pitch, left_yaw;
  double right_roll, right_pitch, right_yaw;
  PARAM_AND_PRINT_FLOAT(
      nodeHandle_, "/incremental_ik_ros_uni_node/quest3/left_hand_quat_offset_roll", left_roll, 0.0, 3);
  PARAM_AND_PRINT_FLOAT(
      nodeHandle_, "/incremental_ik_ros_uni_node/quest3/left_hand_quat_offset_pitch", left_pitch, 0.0, 3);
  PARAM_AND_PRINT_FLOAT(nodeHandle_, "/incremental_ik_ros_uni_node/quest3/left_hand_quat_offset_yaw", left_yaw, 0.0, 3);
  PARAM_AND_PRINT_FLOAT(
      nodeHandle_, "/incremental_ik_ros_uni_node/quest3/right_hand_quat_offset_roll", right_roll, 0.0, 3);
  PARAM_AND_PRINT_FLOAT(
      nodeHandle_, "/incremental_ik_ros_uni_node/quest3/right_hand_quat_offset_pitch", right_pitch, 0.0, 3);
  PARAM_AND_PRINT_FLOAT(
      nodeHandle_, "/incremental_ik_ros_uni_node/quest3/right_hand_quat_offset_yaw", right_yaw, 0.0, 3);
  leftHandOffsetRpy_ << left_roll, left_pitch, left_yaw;
  rightHandOffsetRpy_ << right_roll, right_pitch, right_yaw;

  // 将RPY角度转换为四元数（ZYX顺序：先绕Z轴旋转yaw，再绕Y轴旋转pitch，最后绕X轴旋转roll）
  leftHandQuatOffset_ = Eigen::AngleAxisd(leftHandOffsetRpy_(2), Eigen::Vector3d::UnitZ()) *
                        Eigen::AngleAxisd(leftHandOffsetRpy_(1), Eigen::Vector3d::UnitY()) *
                        Eigen::AngleAxisd(leftHandOffsetRpy_(0), Eigen::Vector3d::UnitX());
  rightHandQuatOffset_ = Eigen::AngleAxisd(rightHandOffsetRpy_(2), Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(rightHandOffsetRpy_(1), Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(rightHandOffsetRpy_(0), Eigen::Vector3d::UnitX());

  clipPositionBySphere(
      defaultLeftHandPosOnExit_, robotLeftFixedShoulderPos_, sphereRadiusLimit_, minReachableDistance_);
  clipPositionByBox(defaultLeftHandPosOnExit_, boxMinBound_, boxMaxBound_);
  clipPositionByChestMidline(defaultLeftHandPosOnExit_, true, chestOffsetY_);
  clipPositionBySphere(
      defaultRightHandPosOnExit_, robotRightFixedShoulderPos_, sphereRadiusLimit_, minReachableDistance_);
  clipPositionByBox(defaultRightHandPosOnExit_, boxMinBound_, boxMaxBound_);
  clipPositionByChestMidline(defaultRightHandPosOnExit_, false, chestOffsetY_);

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

bool Quest3IkIncrementalROS::detectHumanArmMove() {
  if (incrementalController_->hasHumanArmMoved()) return true;

  incrementalController_->detectHumanArmMove(quest3ArmInfoTransformerPtr_->getLeftHandPose().position,
                                             quest3ArmInfoTransformerPtr_->getRightHandPose().position);

  return incrementalController_->hasHumanArmMoved();
}

bool Quest3IkIncrementalROS::updateLatestIncrementalResult() {
  latestIncrementalResult_ =
      incrementalController_->computeIncrementalPose(quest3ArmInfoTransformerPtr_->getLeftHandPose(),
                                                     quest3ArmInfoTransformerPtr_->getRightHandPose(),
                                                     quest3ArmInfoTransformerPtr_->getLeftElbowPose(),
                                                     quest3ArmInfoTransformerPtr_->getRightElbowPose());
  return latestIncrementalResult_.isValid;
}

}  // namespace HighlyDynamic
