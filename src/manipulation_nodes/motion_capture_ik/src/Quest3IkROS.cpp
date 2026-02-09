#include "motion_capture_ik/Quest3IkROS.h"

#include <drake/geometry/scene_graph.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/framework/diagram.h>
#include <drake/systems/framework/diagram_builder.h>
#include <kuavo_msgs/changeArmCtrlMode.h>  // 新增：手臂控制模式切换服务
#include <kuavo_msgs/headBodyPose.h>       // 新增：头部身体姿态消息
#include <ros/package.h>
#include <sensor_msgs/JointState.h>

#include <Eigen/Geometry>
#include <leju_utils/define.hpp>

#include "motion_capture_ik/ArmControlBaseROS.h"
#include "motion_capture_ik/json.hpp"
#include "motion_capture_ik/Quest3ArmInfoTransformer.h"
#include "ros/console.h"

namespace HighlyDynamic {

Quest3IkROS::Quest3IkROS(ros::NodeHandle& nodeHandle, double publishRate, bool debugPrint, ArmIdx ctrlArmIdx)
    : ArmControlBaseROS(nodeHandle, publishRate, debugPrint), ctrlArmIdx_(ctrlArmIdx) {}

Quest3IkROS::~Quest3IkROS() {
  shouldStop_ = true;

  if (ikSolveThread_.joinable()) {
    ikSolveThread_.join();
  }
  // 使用stream，因为此时ros节点可能已经关闭，不能使用ROS_INFO
  std::cout << "[Quest3IkROS] Class destroyed" << std::endl;
}

void Quest3IkROS::run() {
  ikSolveThread_ = std::thread(&Quest3IkROS::solveIkHandElbowThreadFuntion, this);
  ros::spin();
}

void Quest3IkROS::processBonePoses(const noitom_hi5_hand_udp_python::PoseInfoList::ConstPtr& msg) {
  ArmControlBaseROS::processBonePoses(msg);
  quest3DebuggerPtr_->publishBonePoseHandElbow(*HandPoseAndElbowPositonListPtr_);
}

void Quest3IkROS::solveIkHandElbowThreadFuntion() {
  ros::Rate rate(publishRate_);

  // 新增：等待服务可用并切换到手臂控制模式
  ROS_INFO("[Quest3IkROS] Waiting for arm control mode service...");
  if (changeArmCtrlModeClient_.waitForExistence(ros::Duration(10.0))) {
    ROS_INFO("[Quest3IkROS] Arm control mode service available, switching to mode 2");
    if (!changeArmCtrlMode(2)) {
      ROS_WARN("[Quest3IkROS] Failed to switch arm control mode, see the other terminal out put for details");
    }
  }

  ROS_INFO("[Quest3IkROS] Waiting for OK gesture (hold both triggers for 1-2 seconds) to start teleoperation...");
  int holdDurationSteps = 50;
  bool hasStarted = false;

  while (!shouldStop() && ros::ok()) {
    publishEndEffectorControlData();

    // 仅在启动阶段检查手柄状态（复现Python版本的行为）
    if (!hasStarted && quest3ArmInfoTransformerPtr_) {
      quest3ArmInfoTransformerPtr_->checkRunningChangeByHoldingJoy(holdDurationSteps);

      if (quest3ArmInfoTransformerPtr_->isRunning()) {
        if (!changeArmCtrlMode(2)) {
          ROS_WARN("[Quest3IkROS] Failed to switch arm control mode, see the other terminal out put for details");
        }
        hasStarted = true;
        ROS_INFO("[Quest3IkROS] OK gesture received! Starting teleoperation...");
      }
    }

    // Update running state using base class method
    updateRunningState();
    if (quest3ArmInfoTransformerPtr_) {
      // 启动后保持运行状态，不再检查手柄（复现Python版本L483行为）
      if (hasStarted) {
        isRunning_.store(true);
      } else {
        isRunning_.store(quest3ArmInfoTransformerPtr_->isRunning());
      }

      if (isRunning()) {
        publishHeadBodyPose(quest3ArmInfoTransformerPtr_->getHeadBodyPose());
      }
    } else {
      isRunning_.store(true);  // 如果transformer不可用，默认为运行状态
    }

    // 新增：如果未运行，跳过IK计算（复现Python L498-501）
    if (!isRunning()) {
      rate.sleep();
      ROS_DEBUG_THROTTLE(
          1.0, "[Quest3IkROS] Status: STOPPED - Waiting for joystick OK gesture (hold both triggers for 1-2 seconds)");
      continue;
    }

    std::shared_ptr<noitom_hi5_hand_udp_python::PoseInfoList> bonePoseHandElbowPtr;
    {
      std::lock_guard<std::mutex> lock(bonePoseHandElbowMutex_);
      bonePoseHandElbowPtr = HandPoseAndElbowPositonListPtr_;
    }
    if (twoStageTorsoIkPtr_ == nullptr) {
      ROS_WARN("TwoStageTorsoIK Solver not available");
      rate.sleep();
      continue;
    }
    if (bonePoseHandElbowPtr == nullptr) {
      ROS_WARN("No bone pose data");
      rate.sleep();
      continue;
    }

    if (bonePoseHandElbowPtr->poses.size() < 4) {
      // ROS_WARN("Insufficient pose data");
      rate.sleep();
      continue;
    }

    // [CZJ]TODO: 下面这一段代码，目前赋值是正确的，将于下一个版本优化，同时用图文说明为什么这么做
    // [CZJ]TODO: 这段代码效率很低，可读性和性能都需要进一步优化
    std::vector<PoseData> PoseConstraintList(POSE_DATA_LIST_SIZE,
                                             PoseData());  //[chest, l_hand, r_hand, l_elbow, r_elbow]

    // ############################ chest pose ############################
    // NOTES: chest pose 在weld状态下，不需要加载到drake的优化求解中
    std::shared_ptr<noitom_hi5_hand_udp_python::PoseInfoList> originalBonePosesPtr;
    {
      std::lock_guard<std::mutex> lock(bonePosesMutex_);
      originalBonePosesPtr = latestBonePosesPtr_;
    }

    if (originalBonePosesPtr && originalBonePosesPtr->poses.size() > POSE_INDEX_CHEST) {
      auto chestPosition = originalBonePosesPtr->poses[POSE_INDEX_CHEST].position;
      PoseConstraintList[POSE_DATA_LIST_INDEX_CHEST].position << chestPosition.x, chestPosition.y, chestPosition.z;

      auto chestOrientation = originalBonePosesPtr->poses[POSE_INDEX_CHEST].orientation;
      auto chestQuaternion =
          Eigen::Quaterniond(chestOrientation.w, chestOrientation.x, chestOrientation.y, chestOrientation.z);
      PoseConstraintList[POSE_DATA_LIST_INDEX_CHEST].rotation_matrix = chestQuaternion.toRotationMatrix();
    } else {
      ROS_WARN("Chest data not available, using default values");
      PoseConstraintList[POSE_DATA_LIST_INDEX_CHEST].position = Eigen::Vector3d::Zero();
      PoseConstraintList[POSE_DATA_LIST_INDEX_CHEST].rotation_matrix = Eigen::Matrix3d::Identity();
    }

    // ############################ left hand pose ############################
    // bonePoseHandElbowPtr结构: [l_hand(0), l_elbow(1), r_hand(2), r_elbow(3)]
    auto leftHandPosition = bonePoseHandElbowPtr->poses[0].position;
    PoseConstraintList[POSE_DATA_LIST_INDEX_LEFT_HAND].position << leftHandPosition.x, leftHandPosition.y,
        leftHandPosition.z;

    auto leftHandOrientation = bonePoseHandElbowPtr->poses[0].orientation;

    // 检查四元数是否有效
    double quatNorm =
        sqrt(leftHandOrientation.w * leftHandOrientation.w + leftHandOrientation.x * leftHandOrientation.x +
             leftHandOrientation.y * leftHandOrientation.y + leftHandOrientation.z * leftHandOrientation.z);

    if (quatNorm < 1e-6 || !std::isfinite(quatNorm)) {
      ROS_WARN("Invalid left hand quaternion, using identity");
      PoseConstraintList[POSE_DATA_LIST_INDEX_LEFT_HAND].rotation_matrix = Eigen::Matrix3d::Identity();
    } else {
      auto leftHandQuaternion = Eigen::Quaterniond(
          leftHandOrientation.w, leftHandOrientation.x, leftHandOrientation.y, leftHandOrientation.z);
      leftHandQuaternion.normalize();  // 确保四元数归一化
      PoseConstraintList[POSE_DATA_LIST_INDEX_LEFT_HAND].rotation_matrix = leftHandQuaternion.toRotationMatrix();
    }

    // ############################ right hand pose ############################
    auto rightHandPosition = bonePoseHandElbowPtr->poses[2].position;
    PoseConstraintList[POSE_DATA_LIST_INDEX_RIGHT_HAND].position << rightHandPosition.x, rightHandPosition.y,
        rightHandPosition.z;

    auto rightHandOrientation = bonePoseHandElbowPtr->poses[2].orientation;

    // 检查四元数是否有效
    double rightQuatNorm =
        sqrt(rightHandOrientation.w * rightHandOrientation.w + rightHandOrientation.x * rightHandOrientation.x +
             rightHandOrientation.y * rightHandOrientation.y + rightHandOrientation.z * rightHandOrientation.z);

    if (rightQuatNorm < 1e-6 || !std::isfinite(rightQuatNorm)) {
      ROS_WARN("Invalid right hand quaternion, using identity");
      PoseConstraintList[POSE_DATA_LIST_INDEX_RIGHT_HAND].rotation_matrix = Eigen::Matrix3d::Identity();
    } else {
      auto rightHandQuaternion = Eigen::Quaterniond(
          rightHandOrientation.w, rightHandOrientation.x, rightHandOrientation.y, rightHandOrientation.z);
      rightHandQuaternion.normalize();  // 确保四元数归一化
      PoseConstraintList[POSE_DATA_LIST_INDEX_RIGHT_HAND].rotation_matrix = rightHandQuaternion.toRotationMatrix();
    }

    // ############################ left elbow pose ############################
    PoseConstraintList[POSE_DATA_LIST_INDEX_LEFT_ELBOW].rotation_matrix = Eigen::Matrix3d::Identity();
    auto leftElbowPosition = bonePoseHandElbowPtr->poses[1].position;
    PoseConstraintList[POSE_DATA_LIST_INDEX_LEFT_ELBOW].position << leftElbowPosition.x, leftElbowPosition.y,
        leftElbowPosition.z;

    // ############################ right elbow pose ############################
    PoseConstraintList[POSE_DATA_LIST_INDEX_RIGHT_ELBOW].rotation_matrix = Eigen::Matrix3d::Identity();
    auto rightElbowPosition = bonePoseHandElbowPtr->poses[3].position;
    PoseConstraintList[POSE_DATA_LIST_INDEX_RIGHT_ELBOW].position << rightElbowPosition.x, rightElbowPosition.y,
        rightElbowPosition.z;

    // printPoseDataListTable(PoseConstraintList);
    auto ikResult = twoStageTorsoIkPtr_->solveIK(PoseConstraintList, ctrlArmIdx_);

    if (ikResult.isSuccess) {
      {
        std::lock_guard<std::mutex> lock(ikResultMutex_);
      }

      auto stage1Result = twoStageTorsoIkPtr_->getStage1Result();
      auto stage2Result = twoStageTorsoIkPtr_->getStage2Result();

      if (stage1Result.first && stage2Result.first) {
        quest3DebuggerPtr_->publishDebugDataPackage(*HandPoseAndElbowPositonListPtr_,
                                                    std::make_shared<Eigen::VectorXd>(stage1Result.second),
                                                    std::make_shared<Eigen::VectorXd>(stage2Result.second));
      } else {
        if (stage1Result.first) {
          quest3DebuggerPtr_->publishStage1Result(stage1Result.second);
        }
        if (stage2Result.first) {
          quest3DebuggerPtr_->publishStage2Result(stage2Result.second);
        }
      }

      publishJointStates(ikResult.solution);
    } else {
      ROS_WARN("Two-stage IK solving failed: %s", ikResult.solverLog.c_str());
    }
    rate.sleep();
  }
}

void Quest3IkROS::publishJointStates(const Eigen::VectorXd& jointPositions) {
  if (jointPositions.size() != jointStateSize_ || !sensorDataRaw_) return;

  ROS_INFO_ONCE("\033[32m[Quest3IkROS] publishJointStates Started!\033[0m");

  // drake求解时，即调用urdf中的关节限位
  Eigen::VectorXd armAngleLimited_rad = jointPositions.tail(jointStateSize_);

  sensor_msgs::JointState jointStateMsg;
  jointStateMsg.header.stamp = ros::Time::now();
  jointStateMsg.position.resize(jointStateSize_);
  jointStateMsg.name.resize(jointStateSize_);

  if (onlyHalfUpBody_ && armModeChanging_) {
    std::shared_ptr<kuavo_msgs::sensorsData> currentSensorData = getSensorData();

    const int ARM_JOINT_START = 12 + waist_dof_;  // 考虑腰部自由度

    Eigen::VectorXd armCurrentAngle_rad(jointStateSize_);

    for (int i = 0; i < jointStateSize_; ++i) {
      armCurrentAngle_rad(i) = currentSensorData->joint_data.joint_q[ARM_JOINT_START + i];
    }

    bool isFinished = false;
    armAngleLimited_rad = interpolateJointAngles(
        armCurrentAngle_rad, armAngleLimited_rad, thresholdArmDiffHalfUpBody_rad_, maxSpeed_, isFinished);

    if (isFinished) {
      armModeChanging_ = false;
    }
  }
  for (int i = 0; i < jointStateSize_; ++i) {
    jointStateMsg.name[i] = "arm_joint_" + std::to_string(i + 1);
    jointStateMsg.position[i] = armAngleLimited_rad[i] * 180.0 / M_PI;
  }

  kuavoArmTrajCppPublisher_.publish(jointStateMsg);
}

Eigen::VectorXd Quest3IkROS::interpolateJointAngles(const Eigen::VectorXd& current,
                                                    const Eigen::VectorXd& target,
                                                    double threshold,
                                                    double maxSpeed,
                                                    bool& isFinished) {
  if (current.size() != target.size() || current.size() != jointStateSize_) {
    return Eigen::VectorXd::Zero(jointStateSize_);
  }

  Eigen::VectorXd deltaState = target - current;
  double totalDistance = deltaState.norm();

  if (totalDistance < threshold) {
    isFinished = true;
    return target;
  }

  double scale = std::clamp(maxSpeed / totalDistance, 0.0, 1.0);
  Eigen::VectorXd interpolatedAngles = current + deltaState * scale;

  isFinished = false;

  ROS_INFO("[Quest3IkROS] interpolateJointAngles: distance=%.4f, scale=%.4f", totalDistance, scale);
  return interpolatedAngles;
}

void Quest3IkROS::initialize(const nlohmann::json& configJson) {
  initializeBase(configJson);

  // #############################################################
  // ########## 从JSON配置读取手臂关节数量 ########################
  // #############################################################
  if (configJson.contains("NUM_ARM_JOINT")) {
    jointStateSize_ = configJson["NUM_ARM_JOINT"].get<int>();
    ROS_INFO("✅ [Quest3IkROS] Set arm joints count from JSON: %d", jointStateSize_);
  } else {
    ROS_ERROR("❌ [Quest3IkROS] 'NUM_ARM_JOINT' field not found in JSON configuration");
    throw std::runtime_error("Missing 'NUM_ARM_JOINT' field in JSON configuration");
  }

  // #############################################################
  // ########## 从JSON配置构建URDF路径 ############################
  // #############################################################
  std::string urdfFilePath;
  if (configJson.contains("arm_urdf")) {
    std::string kuavo_assets_path = ros::package::getPath("kuavo_assets");
    std::string arm_urdf_relative = configJson["arm_urdf"].get<std::string>();
    urdfFilePath = kuavo_assets_path + "/models/" + arm_urdf_relative;
    ROS_INFO("✅ [Quest3IkROS] Constructed URDF path from JSON: %s", urdfFilePath.c_str());
  } else {
    ROS_ERROR("❌ [Quest3IkROS] 'arm_urdf' field not found in JSON configuration");
    throw std::runtime_error("Missing 'arm_urdf' field in JSON configuration");
  }

  // #############################################################
  // ########## drake initialization #############################
  // #############################################################
  auto diagramBuilder = std::make_unique<drake::systems::DiagramBuilder<double>>();
  auto [plant, sceneGraph] = drake::multibody::AddMultibodyPlantSceneGraph(diagramBuilder.get(), 0.0);

  drake::multibody::Parser parser(&plant);
  auto modelInstance = parser.AddModelFromFile(urdfFilePath);

  const auto& baseFrame = plant.GetFrameByName("base_link");
  plant.WeldFrames(plant.world_frame(), baseFrame);  // Weld base_link to world frame

  plant.Finalize();

  diagram_ = diagramBuilder->Build();
  diagramContext_ = diagram_->CreateDefaultContext();
  auto plantContext = &diagram_->GetMutableSubsystemContext(plant, diagramContext_.get());

  // #############################################################
  // ########## TwoStageTorsoIK initialization ##################
  // #############################################################
  std::vector<std::string> frameNames = loadFrameNamesFromConfig(configJson);
  auto defaultIkSolverConfig = IKSolverConfig();
  twoStageTorsoIkPtr_ = std::make_unique<HighlyDynamic::TwoStageTorsoIK>(&plant, frameNames, defaultIkSolverConfig);

  // 初始化调试器组件
  quest3DebuggerPtr_ = std::make_unique<Quest3Debugger>(nodeHandle_);
  quest3DebuggerPtr_->initialize();

  // 设置可视化回调函数（复现Python版本的即时发布逻辑）
  quest3ArmInfoTransformerPtr_->setVisualizationCallback(
      [this](const std::string& side, const std::vector<PoseData>& poses) {
        // poses 顺序: [handPose, elbowPose, shoulderPose, chestPose]
        if (poses.size() < 4) {
          ROS_WARN("[Quest3IkROS] Invalid poses vector size: %zu, expected 4", poses.size());
          return;
        }
        const Eigen::Vector3d& handPos = poses[0].position;
        const Eigen::Vector3d& elbowPos = poses[1].position;
        const Eigen::Vector3d& shoulderPos = poses[2].position;
        const Eigen::Vector3d& chestPos = poses[3].position;
        publishVisualizationMarkersForSide(side, handPos, elbowPos, shoulderPos, chestPos);
      });

  ROS_INFO("[Quest3IkROS] Interpolation system initialized successfully");
}

void Quest3IkROS::publishHeadBodyPose(const HeadBodyPose& headBodyPose) {
  if (!controlTorso_) return;

  kuavo_msgs::headBodyPose msg;
  msg.head_pitch = headBodyPose.head_pitch;
  msg.head_yaw = headBodyPose.head_yaw;
  msg.body_yaw = headBodyPose.body_yaw;
  msg.body_roll = headBodyPose.body_roll;

  // 应用pitch_ratio限制 - 对应Python版本L1111-1112
  double pitch_ratio = 0.8;
  msg.body_pitch = std::max(3.0 * M_PI / 180.0, std::min(pitch_ratio * headBodyPose.body_pitch, 40.0 * M_PI / 180.0));

  // 设置body位置 - 对应Python版本L1114-1117
  msg.body_x = 0.0;
  msg.body_y = 0.0;
  msg.body_height = headBodyPose.body_height;

  headBodyPosePublisher_.publish(msg);
}

}  // namespace HighlyDynamic
