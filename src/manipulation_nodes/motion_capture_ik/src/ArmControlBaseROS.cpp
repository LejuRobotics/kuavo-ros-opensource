#include "motion_capture_ik/ArmControlBaseROS.h"
#include <leju_utils/define.hpp>

#include "motion_capture_ik/JoyStickHandler.h"
#include "motion_capture_ik/KeyFramesVisualizer.h"
#include "motion_capture_ik/Quest3ArmInfoTransformer.h"

#include <kuavo_msgs/changeArmCtrlMode.h>
#include <kuavo_msgs/lejuClawCommand.h>
#include <kuavo_msgs/robotHandPosition.h>
#include <kuavo_msgs/headBodyPose.h>
#include <sensor_msgs/JointState.h>

namespace HighlyDynamic {

ArmControlBaseROS::ArmControlBaseROS(ros::NodeHandle& nodeHandle, double publishRate, bool debugPrint)
    : nodeHandle_(nodeHandle),
      shouldStop_(false),
      onlyHalfUpBody_(false),
      armModeChanging_(false),
      isRunning_(false),
      isRunningLast_(false),
      controllerActivated_(false),
      publishRate_(publishRate),
      debugPrint_(debugPrint),
      maxSpeed_(0.21),
      thresholdArmDiffHalfUpBody_rad_(0.2),
      controlTorso_(false),
      waist_dof_(0) {
  ROS_INFO("[ArmControlBaseROS] Base class initialized with publishRate=%.2f, debugPrint=%s",
           publishRate_,
           debugPrint_ ? "true" : "false");
}

ArmControlBaseROS::~ArmControlBaseROS() { ROS_INFO("[ArmControlBaseROS] Base class destructor called"); }

void ArmControlBaseROS::initializeBase(const nlohmann::json& configJson) {
  ROS_INFO("[ArmControlBaseROS] Initializing base ROS components...");

  // 从JSON配置读取腰部自由度数量
  if (configJson.contains("NUM_WAIST_JOINT")) {
    waist_dof_ = configJson["NUM_WAIST_JOINT"].get<int>();
    ROS_INFO("✅ [ArmControlBaseROS] Set waist DOF from JSON: %d", waist_dof_);
  } else {
    ROS_WARN("⚠️  [ArmControlBaseROS] 'NUM_WAIST_JOINT' field not found in JSON configuration, using default value: 0");
    waist_dof_ = 0;
  }

  // Initialize service client for arm control mode
  changeArmCtrlModeClient_ = nodeHandle_.serviceClient<kuavo_msgs::changeArmCtrlMode>("/change_arm_ctrl_mode");
  humanoidArmCtrlModeClient_ =
      nodeHandle_.serviceClient<kuavo_msgs::changeArmCtrlMode>("/humanoid_change_arm_ctrl_mode");
  enableWbcArmTrajectoryControlClient_ =
      nodeHandle_.serviceClient<kuavo_msgs::changeArmCtrlMode>("/enable_wbc_arm_trajectory_control");

  // Initialize service server for arm mode changing
  setArmModeChangingServer_ = nodeHandle_.advertiseService(
      "/quest3/set_arm_mode_changing", &ArmControlBaseROS::setArmModeChangingCallback, this);

  // Initialize basic subscribers
  stopRobotSubscriber_ = nodeHandle_.subscribe(
      "/stop_robot", 1, &ArmControlBaseROS::stopRobotCallback, this, ros::TransportHints().tcpNoDelay());

  sensorsDataRawSubscriber_ = nodeHandle_.subscribe(
      "/sensors_data_raw", 1, &ArmControlBaseROS::sensorDataRawCallback, this, ros::TransportHints().tcpNoDelay());

  armModeSubscriber_ = nodeHandle_.subscribe(
      "/quest3/triger_arm_mode", 10, &ArmControlBaseROS::armModeCallback, this, ros::TransportHints().tcpNoDelay());

  bonePosesSubscriber_ = nodeHandle_.subscribe(
      "/leju_quest_bone_poses", 10, &ArmControlBaseROS::bonePosesCallback, this, ros::TransportHints().tcpNoDelay());
  joystickSubscriber_ = nodeHandle_.subscribe(
      "/quest_joystick_data", 10, &ArmControlBaseROS::joystickCallback, this, ros::TransportHints().tcpNoDelay());

  sensorDataRaw_ = std::make_shared<kuavo_msgs::sensorsData>();
  latestBonePosesPtr_ = std::make_shared<noitom_hi5_hand_udp_python::PoseInfoList>();

  robotHandPositionPublisher_ =
      nodeHandle_.advertise<kuavo_msgs::robotHandPosition>("/control_robot_hand_position", 10);
  lejuClawCommandPublisher_ = nodeHandle_.advertise<kuavo_msgs::lejuClawCommand>("/leju_claw_command", 10);

  headBodyPosePublisher_ = nodeHandle_.advertise<kuavo_msgs::headBodyPose>("/kuavo_head_body_orientation_data", 10);
  kuavoArmTrajCppPublisher_ = nodeHandle_.advertise<sensor_msgs::JointState>("/kuavo_arm_traj_cpp", 2);

  // Load parameters from ROS parameter server
  loadParameters();

  // Initialize visualization components
  initializeKeyFramesVisualizer();

  joyStickHandlerPtr_ = std::make_unique<JoyStickHandler>();
  joyStickHandlerPtr_->initialize();

  quest3ArmInfoTransformerPtr_ = std::make_unique<HighlyDynamic::Quest3ArmInfoTransformer>();
  initializeArmInfoTransformerFromJson(configJson);

  HandPoseAndElbowPositonListPtr_ = std::make_shared<noitom_hi5_hand_udp_python::PoseInfoList>();

  // 初始化机器人关节状态，确保安全初始化
  ROS_INFO("[ArmControlBaseROS] Initializing arm joints for safety...");
  if (initializeArmJointsSafety()) {
    ROS_INFO("[ArmControlBaseROS] Arm joints initialized successfully for safety");
  } else {
    ROS_WARN("[ArmControlBaseROS] Arm joints initialization failed, but continuing...");
  }

  ROS_INFO("[ArmControlBaseROS] Base ROS components initialized successfully");
}

void ArmControlBaseROS::activateController() {}

void ArmControlBaseROS::deactivateController() {}

bool ArmControlBaseROS::isRunning() const { return isRunning_.load(); }

bool ArmControlBaseROS::wasRunning() const { return isRunningLast_.load(); }

bool ArmControlBaseROS::shouldStop() const { return shouldStop_.load(); }

std::shared_ptr<kuavo_msgs::sensorsData> ArmControlBaseROS::getSensorData() const {
  std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(sensorDataRawMutex_));
  return sensorDataRaw_;
}

void ArmControlBaseROS::stopRobotCallback(const std_msgs::Bool::ConstPtr& msg) {
  if (msg->data) {
    ROS_INFO("[ArmControlBaseROS] Received stop robot signal");
    shouldStop_ = true;
    ros::shutdown();
  }
}

void ArmControlBaseROS::sensorDataRawCallback(const kuavo_msgs::sensorsData::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(sensorDataRawMutex_);
  if (!sensorDataRaw_) {
    sensorDataRaw_ = std::make_shared<kuavo_msgs::sensorsData>();
  }
  *sensorDataRaw_ = *msg;
}

void ArmControlBaseROS::armModeCallback(const std_msgs::Int32::ConstPtr& msg) {
  ROS_INFO_STREAM("\033[91m[ArmControlBaseROS] armModeCallbackFunction\033[0m");
  int newMode = msg->data;
  if (newMode != 2) {
    ROS_WARN("\033[91m[ArmControlBaseROS] Reset arm mode\033[0m");
    armModeChanging_.store(false);
  } else {
    ROS_WARN("\033[91m[ArmControlBaseROS] Arm mode changing\033[0m");
    armModeChanging_.store(true);
  }
}

bool ArmControlBaseROS::initializeArmJointsSafety() {
  ROS_INFO("[ArmControlBaseROS] Initializing arm joints for safety...");

  if (!onlyHalfUpBody_) {
    ROS_INFO("[ArmControlBaseROS] onlyHalfUpBody_ is false, skipping arm joints initialization");
    return true;
  }

  std::shared_ptr<kuavo_msgs::sensorsData> currentSensorData = getSensorData();

  if (!currentSensorData) {
    ROS_WARN("[ArmControlBaseROS] sensor_data_raw is None in initializeArmJointsSafety");
    return false;
  }

  const size_t jointQSize = currentSensorData->joint_data.joint_q.size();
  const int armJointStartIndex = 12 + waist_dof_;  // 考虑腰部自由度
  const int numArmJoints = 14;

  ROS_INFO("[ArmControlBaseROS] joint_q array size: %zu, required: %d (waist_dof: %d)", 
           jointQSize, armJointStartIndex + numArmJoints, waist_dof_);

  if (jointQSize < armJointStartIndex + numArmJoints) {
    std::string errorMsg = "joint_q array too small! Size: " + std::to_string(jointQSize) +
                           ", required: " + std::to_string(armJointStartIndex + numArmJoints);
    ROS_ERROR("[ArmControlBaseROS] %s", errorMsg.c_str());
    return false;
  }

  // 执行关节状态发布
  try {
    ros::Rate rate(publishRate_);

    sensor_msgs::JointState msg;
    msg.name.resize(numArmJoints);
    for (int i = 0; i < numArmJoints; ++i) {
      msg.name[i] = "arm_joint_" + std::to_string(i + 1);
    }
    msg.header.stamp = ros::Time::now();
    msg.position.resize(numArmJoints);

    // 安全的数组访问
    for (int i = 0; i < numArmJoints; ++i) {
      const int jointIndex = armJointStartIndex + i;
      if (jointIndex < static_cast<int>(jointQSize)) {
        msg.position[i] = currentSensorData->joint_data.joint_q[jointIndex] * 180.0 / M_PI;
      } else {
        ROS_WARN("[ArmControlBaseROS] Joint index %d out of bounds, using 0.0", jointIndex);
        msg.position[i] = 0.0;
      }
    }

    // 发布20次（复现Python L1079-1081）
    for (int i = 0; i < 20; ++i) {
      kuavoArmTrajCppPublisher_.publish(msg);
      rate.sleep();
    }

    ROS_INFO("[ArmControlBaseROS] Successfully published %d joint states for safety initialization", numArmJoints);
    return true;
  } catch (const std::exception& e) {
    std::string errorMsg = "Failed to publish joint states: " + std::string(e.what());
    ROS_ERROR("[ArmControlBaseROS] %s", errorMsg.c_str());
    return false;
  }
}

bool ArmControlBaseROS::setArmModeChangingCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  ROS_INFO_STREAM("[Quest3IkROS] setArmModeChangingCallback");
  if (!initializeArmJointsSafety()) {
    return handleServiceResponse(res, false, "Failed to initialize arm joints");
  }

  // 设置arm mode changing标志
  armModeChanging_.store(true);

  return handleServiceResponse(res, true, "Arm mode changing set to True successfully");
}

bool ArmControlBaseROS::changeArmCtrlMode(int mode) {
  kuavo_msgs::changeArmCtrlMode srv;
  srv.request.control_mode = mode;

  if (changeArmCtrlModeClient_.call(srv)) {
    if (srv.response.result) {
      ROS_INFO("[ArmControlBaseROS] Successfully changed arm control mode to %d", mode);
      return true;
    } else {
      ROS_WARN("[ArmControlBaseROS] Failed to change arm control mode: %s", srv.response.message.c_str());
      return false;
    }
  } else {
    ROS_ERROR("[ArmControlBaseROS] Failed to call change_arm_ctrl_mode service");
    return false;
  }
}

void ArmControlBaseROS::loadParameters() {
  ROS_INFO("[ArmControlBaseROS] Loading parameters from ROS parameter server...");

  // Load only_half_up_body parameter
  if (nodeHandle_.hasParam("/only_half_up_body")) {
    bool onlyHalfUpBodyParam;
    nodeHandle_.getParam("/only_half_up_body", onlyHalfUpBodyParam);
    onlyHalfUpBody_ = onlyHalfUpBodyParam;
    ROS_INFO("[ArmControlBaseROS] only_half_up_body: %s", onlyHalfUpBody_ ? "true" : "false");
  }

  // Load arm movement speed parameter
  nodeHandle_.param("/arm_move_spd_half_up_body", maxSpeed_, 0.21);
  ROS_INFO("[ArmControlBaseROS] maxSpeed: %.4f rad", maxSpeed_);

  // Load arm difference threshold parameter
  nodeHandle_.param("/threshold_arm_diff_half_up_body", thresholdArmDiffHalfUpBody_rad_, 0.2);
  ROS_INFO("[ArmControlBaseROS] thresholdArmDiffHalfUpBody: %.4f rad", thresholdArmDiffHalfUpBody_rad_);

  nodeHandle_.param("/ik_ros_uni_cpp_node/control_torso", controlTorso_, false);
  ROS_INFO("[ArmControlBaseROS] controlTorso: %s", controlTorso_ ? "true" : "false");

  nodeHandle_.param("/quest3/enable_wbc_arm_trajectory", enableWbcArmTrajectory_, true);
  ROS_INFO("[ArmControlBaseROS] enableWbcArmTrajectory: %s", enableWbcArmTrajectory_ ? "true" : "false");

  ROS_INFO("[ArmControlBaseROS] Parameters loaded successfully");
}

void ArmControlBaseROS::bonePosesCallback(const noitom_hi5_hand_udp_python::PoseInfoList::ConstPtr& msg) {
  {
    std::lock_guard<std::mutex> lock(bonePosesMutex_);
    *latestBonePosesPtr_ = *msg;
  }
  processBonePoses(msg);
}

void ArmControlBaseROS::processBonePoses(const noitom_hi5_hand_udp_python::PoseInfoList::ConstPtr& msg) {
  if (!quest3ArmInfoTransformerPtr_) return;
  if (!quest3ArmInfoTransformerPtr_->updateHandPoseAndElbowPosition(*msg, *HandPoseAndElbowPositonListPtr_)) return;
}

void ArmControlBaseROS::joystickCallback(const noitom_hi5_hand_udp_python::JoySticks::ConstPtr& msg) {
  {
    std::lock_guard<std::mutex> lock(joystickMutex_);
    if (joyStickHandlerPtr_) {
      joyStickHandlerPtr_->updateJoyStickData(msg);
    }

    if (quest3ArmInfoTransformerPtr_) {
      quest3ArmInfoTransformerPtr_->updateJoystickData(
          msg->left_trigger, msg->left_grip, msg->right_trigger, msg->right_grip);
    }
  }
}

void ArmControlBaseROS::updateRunningState() {
  isRunningLast_.store(isRunning_.load());

  if (!wasRunning() && isRunning()) {
    ROS_INFO("[ArmControlBaseROS] Detected state change from stopped to running, setting armModeChanging to true");
    armModeChanging_.store(true);
  }
}

void ArmControlBaseROS::publishEndEffectorControlData() {
  if (!joyStickHandlerPtr_) {
    ROS_WARN("[ArmControlBaseROS] JoyStickHandler not initialized");
    return;
  }
  joyStickHandlerPtr_->processHandEndEffectorData();

  EndEffectorType endEffectorType = joyStickHandlerPtr_->getEndEffectorType();

  if (isHandEndEffectorType(endEffectorType)) {
    publishHandPositionData();
  } else if (isClawEndEffectorType(endEffectorType)) {
    publishClawCommandData();
  }
}

void ArmControlBaseROS::publishHandPositionData() {
  if (!joyStickHandlerPtr_) {
    ROS_WARN("[ArmControlBaseROS] JoyStickHandler not initialized for hand position data");
    return;
  }
  auto handPositionData = joyStickHandlerPtr_->getHandPositionData();
  if (handPositionData.hasValidData) {
    kuavo_msgs::robotHandPosition robotHandPosition;
    robotHandPosition.header.stamp = ros::Time::now();

    // 转换int向量到uint8向量
    robotHandPosition.left_hand_position.resize(handPositionData.leftHandPosition.size());
    robotHandPosition.right_hand_position.resize(handPositionData.rightHandPosition.size());

    for (size_t i = 0; i < handPositionData.leftHandPosition.size(); ++i) {
      robotHandPosition.left_hand_position[i] = static_cast<uint8_t>(handPositionData.leftHandPosition[i]);
    }
    for (size_t i = 0; i < handPositionData.rightHandPosition.size(); ++i) {
      robotHandPosition.right_hand_position[i] = static_cast<uint8_t>(handPositionData.rightHandPosition[i]);
    }

    robotHandPositionPublisher_.publish(robotHandPosition);
  }
}

void ArmControlBaseROS::publishClawCommandData() {
  if (!joyStickHandlerPtr_) {
    ROS_WARN("[ArmControlBaseROS] JoyStickHandler not initialized for claw command data");
    return;
  }

  auto clawCommandData = joyStickHandlerPtr_->getClawCommandData();
  if (clawCommandData.hasValidData) {
    kuavo_msgs::lejuClawCommand clawCommand;
    clawCommand.header.stamp = ros::Time::now();

    // Set claw names
    clawCommand.data.name.resize(2);
    clawCommand.data.name[0] = "left_claw";
    clawCommand.data.name[1] = "right_claw";

    // Set positions
    clawCommand.data.position.resize(2);
    if (clawCommandData.positions.size() >= 2) {
      clawCommand.data.position[0] = clawCommandData.positions[0];
      clawCommand.data.position[1] = clawCommandData.positions[1];
    } else {
      clawCommand.data.position[0] = 0.0;
      clawCommand.data.position[1] = 0.0;
    }

    // Set velocity and effort
    clawCommand.data.velocity.resize(2);
    clawCommand.data.effort.resize(2);
    if (clawCommandData.velocities.size() >= 2) {
      clawCommand.data.velocity[0] = clawCommandData.velocities[0];
      clawCommand.data.velocity[1] = clawCommandData.velocities[1];
    } else {
      clawCommand.data.velocity[0] = 90.0;
      clawCommand.data.velocity[1] = 90.0;
    }
    if (clawCommandData.efforts.size() >= 2) {
      clawCommand.data.effort[0] = clawCommandData.efforts[0];
      clawCommand.data.effort[1] = clawCommandData.efforts[1];
    } else {
      clawCommand.data.effort[0] = 1.0;
      clawCommand.data.effort[1] = 1.0;
    }

    lejuClawCommandPublisher_.publish(clawCommand);
  }
}

void ArmControlBaseROS::initializeArmInfoTransformerFromJson(const nlohmann::json& configJson) {
  ROS_INFO("==================================================================================");
  ROS_INFO("🔧 [ArmControlBaseROS] Initializing Quest3ArmInfoTransformer from JSON configuration");
  ROS_INFO("==================================================================================");

  processJsonParameter<double>(configJson, "upper_arm_length", [this](double value) {
    quest3ArmInfoTransformerPtr_->updateUpperArmLength(value);
  });

  processJsonParameter<double>(configJson, "lower_arm_length", [this](double value) {
    quest3ArmInfoTransformerPtr_->updateLowerArmLength(value);
  });

  // 初始化基座高度偏移
  processJsonParameter<double>(configJson, "base_height_offset", [this](double value) {
    quest3ArmInfoTransformerPtr_->updateBaseHeightOffset(value);
  });

  // 初始化胸部X轴偏移
  processJsonParameter<double>(configJson, "base_chest_offset_x", [this](double value) {
    quest3ArmInfoTransformerPtr_->updateBaseChestOffsetX(value);
  });

  // 初始化肩宽参数
  processJsonParameter<double>(
      configJson, "shoulder_width", [this](double value) { quest3ArmInfoTransformerPtr_->updateShoulderWidth(value); });

  ROS_INFO("🎯 [ArmControlBaseROS] Quest3ArmInfoTransformer initialization completed");
  ROS_INFO("==================================================================================");
}

template <typename T, typename UpdateFunc>
void ArmControlBaseROS::processJsonParameter(const nlohmann::json& configJson,
                                             const std::string& paramName,
                                             UpdateFunc updateFunction) {
  if (configJson.contains(paramName)) {
    T value = configJson[paramName].get<T>();
    updateFunction(value);
    ROS_INFO("✅ [ArmControlBaseROS] Initialized %s: %.4f", paramName.c_str(), static_cast<double>(value));
  } else {
    ROS_WARN("❌ [ArmControlBaseROS] '%s' not found in JSON config, using default values", paramName.c_str());
  }
}

std::vector<std::string> ArmControlBaseROS::loadFrameNamesFromConfig(const nlohmann::json& configJson) {
  ROS_INFO("==================================================================================");
  ROS_INFO("🔧 [ArmControlBaseROS] Auto-loading frame names from provided JSON configuration");
  ROS_INFO("==================================================================================");

  std::vector<std::string> frameNames;

  try {
    // 从JSON中提取end_frames_name_ik配置
    if (configJson.contains("end_frames_name_ik") && configJson["end_frames_name_ik"].is_array()) {
      frameNames = configJson["end_frames_name_ik"].get<std::vector<std::string>>();
      ROS_INFO("✅ [ArmControlBaseROS] Successfully loaded %zu frame names from JSON config:", frameNames.size());
      for (size_t i = 0; i < frameNames.size(); ++i) {
        ROS_INFO("  [%zu] %s", i, frameNames[i].c_str());
      }
    } else {
      ROS_WARN("❌ [ArmControlBaseROS] 'end_frames_name_ik' not found in JSON config, using hardcoded values");
      frameNames = {"base_link", "zarm_l7_end_effector", "zarm_r7_end_effector", "zarm_l4_link", "zarm_r4_link"};
    }

  } catch (const std::exception& e) {
    ROS_ERROR("❌ [ArmControlBaseROS] Exception while loading frame names: %s", e.what());
    ROS_WARN("🔄 [ArmControlBaseROS] Falling back to hardcoded frame names");
    frameNames = {"base_link", "zarm_l7_end_effector", "zarm_r7_end_effector", "zarm_l4_link", "zarm_r4_link"};
  }

  // 验证frame names不为空
  if (frameNames.empty()) {
    ROS_ERROR("❌ [ArmControlBaseROS] Frame names list is empty, using hardcoded values");
    frameNames = {"base_link", "zarm_l7_end_effector", "zarm_r7_end_effector", "zarm_l4_link", "zarm_r4_link"};
  }

  ROS_INFO("🎯 [ArmControlBaseROS] Final frame names configuration:");
  for (size_t i = 0; i < frameNames.size(); ++i) {
    ROS_INFO("  [%zu] %s", i, frameNames[i].c_str());
  }
  ROS_INFO("==================================================================================");

  return frameNames;
}

void ArmControlBaseROS::initializeKeyFramesVisualizer() {
  ROS_INFO("[ArmControlBaseROS] Initializing KeyFramesVisualizer...");
  quest3KeyFramesVisualizerPtr_ = std::make_unique<KeyFramesVisualizer>(nodeHandle_);
  quest3KeyFramesVisualizerPtr_->initialize();
  ROS_INFO("[ArmControlBaseROS] KeyFramesVisualizer initialized successfully");
}

void ArmControlBaseROS::publishVisualizationMarkersForSide(const std::string& side,
                                                           const Eigen::Vector3d& handPos,
                                                           const Eigen::Vector3d& elbowPos,
                                                           const Eigen::Vector3d& shoulderPos,
                                                           const Eigen::Vector3d& chestPos) {
  if (quest3KeyFramesVisualizerPtr_) {
    quest3KeyFramesVisualizerPtr_->publishVisualizationMarkersForSide(side, handPos, elbowPos, shoulderPos, chestPos);
  }
}

}  // namespace HighlyDynamic
