#include "motion_capture_ik/WheelArmControlBaseROS.h"
#include <leju_utils/define.hpp>

#include "motion_capture_ik/WheelJoyStickHandler.h"
#include "motion_capture_ik/WheelKeyFramesVisualizer.h"
#include "motion_capture_ik/Quest3ArmInfoTransformer.h"
#include "ros/console.h"

#include <fstream>
#include <ctime>

#include <kuavo_msgs/changeArmCtrlMode.h>
#include <kuavo_msgs/changeTorsoCtrlMode.h>
#include <kuavo_msgs/lejuClawCommand.h>
#include <kuavo_msgs/robotHandPosition.h>
#include <kuavo_msgs/headBodyPose.h>
#include <sensor_msgs/JointState.h>

namespace HighlyDynamic {

WheelArmControlBaseROS::WheelArmControlBaseROS(ros::NodeHandle& nodeHandle, double publishRate, bool debugPrint)
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
      controlTorso_(false) {
  ROS_INFO("[WheelArmControlBaseROS] Base class initialized with publishRate=%.2f, debugPrint=%s",
           publishRate_,
           debugPrint_ ? "true" : "false");

  // 初始化时间戳记录系统
  timestampRecords_.clear();
  timestampRecords_.reserve(10000);  // 预分配空间，避免频繁内存分配
  lastSaveTime_ = std::chrono::steady_clock::now();
  ROS_INFO("[WheelArmControlBaseROS] Timestamp recording system initialized");
}

WheelArmControlBaseROS::~WheelArmControlBaseROS() { ROS_INFO("[WheelArmControlBaseROS] Base class destructor called"); }

void WheelArmControlBaseROS::initializeBase(const nlohmann::json& configJson) {
  ROS_INFO("[WheelArmControlBaseROS] Initializing base ROS components...");

  // Initialize service client for arm control mode
  changeArmCtrlModeClient_ = nodeHandle_.serviceClient<kuavo_msgs::changeArmCtrlMode>("/change_arm_ctrl_mode");

  changeMobileCtrlModeClient_ =
      nodeHandle_.serviceClient<kuavo_msgs::changeTorsoCtrlMode>("/mobile_manipulator_mpc_control");
  humanoidArmCtrlModeClient_ =
      nodeHandle_.serviceClient<kuavo_msgs::changeArmCtrlMode>("/humanoid_change_arm_ctrl_mode");
  enableWbcArmTrajectoryControlClient_ =
      nodeHandle_.serviceClient<kuavo_msgs::changeArmCtrlMode>("/enable_wbc_arm_trajectory_control");

  // Initialize service server for arm mode changing
  setArmModeChangingServer_ = nodeHandle_.advertiseService(
      "/quest3/set_arm_mode_changing", &WheelArmControlBaseROS::setArmModeChangingCallback, this);

  // Initialize basic subscribers
  stopRobotSubscriber_ = nodeHandle_.subscribe(
      "/stop_robot", 1, &WheelArmControlBaseROS::stopRobotCallback, this, ros::TransportHints().tcpNoDelay());

  sensorsDataRawSubscriber_ = nodeHandle_.subscribe(
      "/sensors_data_raw", 1, &WheelArmControlBaseROS::sensorDataRawCallback, this, ros::TransportHints().tcpNoDelay());

  armModeSubscriber_ = nodeHandle_.subscribe(
      "/quest3/triger_arm_mode", 10, &WheelArmControlBaseROS::armModeCallback, this, ros::TransportHints().tcpNoDelay());

  bonePosesSubscriber_ = nodeHandle_.subscribe(
      "/leju_quest_bone_poses", 10, &WheelArmControlBaseROS::bonePosesCallback, this, ros::TransportHints().tcpNoDelay());
  joystickSubscriber_ = nodeHandle_.subscribe(
      "/quest_joystick_data", 10, &WheelArmControlBaseROS::joystickCallback, this, ros::TransportHints().tcpNoDelay());

  sensorDataRaw_ = std::make_shared<kuavo_msgs::sensorsData>();
  latestBonePosesPtr_ = std::make_shared<noitom_hi5_hand_udp_python::PoseInfoList>();

  robotHandPositionPublisher_ =
      nodeHandle_.advertise<kuavo_msgs::robotHandPosition>("/control_robot_hand_position", 10);
  lejuClawCommandPublisher_ = nodeHandle_.advertise<kuavo_msgs::lejuClawCommand>("/leju_claw_command", 10);

  headBodyPosePublisher_ = nodeHandle_.advertise<kuavo_msgs::headBodyPose>("/kuavo_head_body_orientation_data", 10);
  kuavoArmTrajCppPublisher_ = nodeHandle_.advertise<sensor_msgs::JointState>("/kuavo_arm_traj_cpp", 2);
  questJoystickDataPublisher_ =
      nodeHandle_.advertise<noitom_hi5_hand_udp_python::JoySticks>("/quest_joystick_data", 10);

  // Load parameters from ROS parameter server
  loadParameters();

  // Load robot joint dimension parameters from JSON configuration
  ROS_INFO("==================================================================================");
  ROS_INFO("🔧 [WheelArmControlBaseROS] Loading robot joint dimensions from JSON configuration");
  ROS_INFO("==================================================================================");

  // Load joint dimensions with default values as fallback
  numArmJoints_ = configJson.value("NUM_ARM_JOINT", 14);
  numHeadJoints_ = configJson.value("NUM_HEAD_JOINT", 2);
  numWaistJoints_ = configJson.value("NUM_WAIST_JOINT", 0);
  numTotalJoints_ = configJson.value("NUM_JOINT", 28);

  // Calculate sensor data offset: total joints - head joints - arm joints
  // Sensor data structure: [Leg joints] + [Arm joints] + [Head joints]
  //   s49: [0-11: Legs(12)] + [12-25: Arms(14)] + [26-27: Head(2)] = 28 total
  //   s60: [0-3:  Legs(4)]  + [4-17:  Arms(14)] + [18-19: Head(2)] = 20 total
  // This represents the number of leg joints before arm joints in sensor data array
  sensorDataArmOffset_ = numTotalJoints_ - numHeadJoints_ - numArmJoints_;

  // Validation: ensure the calculation is non-negative
  if (sensorDataArmOffset_ < 0) {
    ROS_ERROR("[WheelArmControlBaseROS] Invalid sensor data offset: %d (must be >= 0). Check JSON configuration!",
              sensorDataArmOffset_);
    throw std::runtime_error("Invalid joint dimension configuration in JSON");
  }

  ROS_INFO("✅ [WheelArmControlBaseROS] Loaded joint dimensions:");
  ROS_INFO("   - NUM_ARM_JOINT: %d", numArmJoints_);
  ROS_INFO("   - NUM_HEAD_JOINT: %d", numHeadJoints_);
  ROS_INFO("   - NUM_WAIST_JOINT: %d", numWaistJoints_);
  ROS_INFO("   - NUM_JOINT (total): %d", numTotalJoints_);
  ROS_INFO("   - Sensor data arm offset (calculated): %d", sensorDataArmOffset_);
  ROS_INFO("📊 [WheelArmControlBaseROS] Sensor data layout:");
  ROS_INFO("   - Leg joints:  indices [0-%d] (%d joints)", sensorDataArmOffset_ - 1, sensorDataArmOffset_);
  ROS_INFO("   - Arm joints:  indices [%d-%d] (%d joints)",
           sensorDataArmOffset_,
           sensorDataArmOffset_ + numArmJoints_ - 1,
           numArmJoints_);
  ROS_INFO("   - Head joints: indices [%d-%d] (%d joints)",
           sensorDataArmOffset_ + numArmJoints_,
           numTotalJoints_ - 1,
           numHeadJoints_);
  ROS_INFO("==================================================================================");

  // Initialize visualization components
  initializeKeyFramesVisualizer();

  // 从 JSON 配置读取摇杆阈值和低通滤波alpha系数
  double joyStickThreshold = 0.05;  // 默认阈值
  double joyStickAlpha = 0.8;       // 默认alpha系数
  if (configJson.contains("joystick_filter")) {
    const auto& joystickFilter = configJson["joystick_filter"];
    if (joystickFilter.contains("threshold")) {
      joyStickThreshold = joystickFilter["threshold"].get<double>();
    }
    if (joystickFilter.contains("alpha")) {
      joyStickAlpha = joystickFilter["alpha"].get<double>();
    }
    ROS_INFO("✅ [WheelArmControlBaseROS] Loaded joystick filter parameters: threshold=%.4f, alpha=%.4f",
             joyStickThreshold,
             joyStickAlpha);
  } else {
    ROS_WARN(
        "❌ [WheelArmControlBaseROS] 'joystick_filter' not found in JSON config, using default values: threshold=%.4f, "
        "alpha=%.4f",
        joyStickThreshold,
        joyStickAlpha);
  }
  joyStickHandlerPtr_ = std::make_unique<WheelJoyStickHandler>(joyStickThreshold, joyStickAlpha);
  joyStickHandlerPtr_->initialize();

  quest3ArmInfoTransformerPtr_ = std::make_unique<HighlyDynamic::Quest3ArmInfoTransformer>();
  initializeArmInfoTransformerFromJson(configJson);

  HandPoseAndElbowPositonListPtr_ = std::make_shared<noitom_hi5_hand_udp_python::PoseInfoList>();

  // 初始化机器人关节状态，确保安全初始化
  ROS_INFO("[WheelArmControlBaseROS] Initializing arm joints for safety...");
  if (initializeArmJointsSafety()) {
    ROS_INFO("[WheelArmControlBaseROS] Arm joints initialized successfully for safety");
  } else {
    ROS_WARN("[WheelArmControlBaseROS] Arm joints initialization failed, but continuing...");
  }

  ROS_INFO("[WheelArmControlBaseROS] Base ROS components initialized successfully");
}

void WheelArmControlBaseROS::activateController() {}

void WheelArmControlBaseROS::deactivateController() {}

bool WheelArmControlBaseROS::isRunning() const { return isRunning_.load(); }

bool WheelArmControlBaseROS::wasRunning() const { return isRunningLast_.load(); }

bool WheelArmControlBaseROS::shouldStop() const { return shouldStop_.load(); }

std::shared_ptr<kuavo_msgs::sensorsData> WheelArmControlBaseROS::getSensorData() const {
  std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(sensorDataRawMutex_));
  return sensorDataRaw_;
}

void WheelArmControlBaseROS::stopRobotCallback(const std_msgs::Bool::ConstPtr& msg) {
  if (msg->data) {
    ROS_INFO("[WheelArmControlBaseROS] Received stop robot signal");
    shouldStop_ = true;
    ros::shutdown();
  }
}

void WheelArmControlBaseROS::sensorDataRawCallback(const kuavo_msgs::sensorsData::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(sensorDataRawMutex_);
  if (!sensorDataRaw_) {
    sensorDataRaw_ = std::make_shared<kuavo_msgs::sensorsData>();
  }
  *sensorDataRaw_ = *msg;
}

void WheelArmControlBaseROS::armModeCallback(const std_msgs::Int32::ConstPtr& msg) {
  ROS_INFO_STREAM("\033[91m[WheelArmControlBaseROS] armModeCallbackFunction\033[0m");
  int newMode = msg->data;
  if (newMode != 2) {
    ROS_WARN("\033[91m[WheelArmControlBaseROS] Reset arm mode\033[0m");
    armModeChanging_.store(false);
  } else {
    ROS_WARN("\033[91m[WheelArmControlBaseROS] Arm mode changing\033[0m");
    armModeChanging_.store(true);
  }
}

bool WheelArmControlBaseROS::initializeArmJointsSafety() {
  ROS_INFO("[WheelArmControlBaseROS] Initializing arm joints for safety...");

  if (!onlyHalfUpBody_) {
    ROS_INFO("[WheelArmControlBaseROS] onlyHalfUpBody_ is false, skipping arm joints initialization");
    return true;
  }

  std::shared_ptr<kuavo_msgs::sensorsData> currentSensorData = getSensorData();

  if (!currentSensorData) {
    ROS_WARN("[WheelArmControlBaseROS] sensor_data_raw is None in initializeArmJointsSafety");
    return false;
  }

  const size_t jointQSize = currentSensorData->joint_data.joint_q.size();

  ROS_INFO(
      "[WheelArmControlBaseROS] joint_q array size: %zu, required: %d", jointQSize, sensorDataArmOffset_ + numArmJoints_);

  if (jointQSize < static_cast<size_t>(sensorDataArmOffset_ + numArmJoints_)) {
    std::string errorMsg = "joint_q array too small! Size: " + std::to_string(jointQSize) +
                           ", required: " + std::to_string(sensorDataArmOffset_ + numArmJoints_);
    ROS_ERROR("[WheelArmControlBaseROS] %s", errorMsg.c_str());
    return false;
  }

  // 执行关节状态发布
  try {
    ros::Rate rate(publishRate_);

    sensor_msgs::JointState msg;
    msg.name.resize(numArmJoints_);
    for (int i = 0; i < numArmJoints_; ++i) {
      msg.name[i] = "arm_joint_" + std::to_string(i + 1);
    }
    msg.header.stamp = ros::Time::now();
    msg.position.resize(numArmJoints_);

    // 安全的数组访问
    for (int i = 0; i < numArmJoints_; ++i) {
      const int jointIndex = sensorDataArmOffset_ + i;
      if (jointIndex < static_cast<int>(jointQSize)) {
        msg.position[i] = currentSensorData->joint_data.joint_q[jointIndex] * 180.0 / M_PI;
      } else {
        ROS_WARN("[WheelArmControlBaseROS] Joint index %d out of bounds, using 0.0", jointIndex);
        msg.position[i] = 0.0;
      }
    }

    // 发布20次（复现Python L1079-1081）
    for (int i = 0; i < 20; ++i) {
      kuavoArmTrajCppPublisher_.publish(msg);
      rate.sleep();
    }

    ROS_INFO("[WheelArmControlBaseROS] Successfully published %d joint states for safety initialization", numArmJoints_);
    return true;
  } catch (const std::exception& e) {
    std::string errorMsg = "Failed to publish joint states: " + std::string(e.what());
    ROS_ERROR("[WheelArmControlBaseROS] %s", errorMsg.c_str());
    return false;
  }
}

bool WheelArmControlBaseROS::setArmModeChangingCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  ROS_INFO_STREAM("[Quest3IkROS] setArmModeChangingCallback");
  if (!initializeArmJointsSafety()) {
    return handleServiceResponse(res, false, "Failed to initialize arm joints");
  }

  // 设置arm mode changing标志
  armModeChanging_.store(true);

  return handleServiceResponse(res, true, "Arm mode changing set to True successfully");
}

bool WheelArmControlBaseROS::changeArmCtrlMode(int mode) {
  kuavo_msgs::changeArmCtrlMode srv;
  srv.request.control_mode = mode;

  if (changeArmCtrlModeClient_.call(srv)) {
    if (srv.response.result) {
      ROS_INFO("[WheelArmControlBaseROS] Successfully changed arm control mode to %d", mode);
      return true;
    } else {
      ROS_WARN("[WheelArmControlBaseROS] Failed to change arm control mode: %s", srv.response.message.c_str());
      return false;
    }
  } else {
    ROS_ERROR("[WheelArmControlBaseROS] Failed to call change_arm_ctrl_mode service");
    return false;
  }
}

void WheelArmControlBaseROS::loadParameters() {
  ROS_INFO("[WheelArmControlBaseROS] Loading parameters from ROS parameter server...");

  // Load only_half_up_body parameter
  if (nodeHandle_.hasParam("/only_half_up_body")) {
    bool onlyHalfUpBodyParam;
    nodeHandle_.getParam("/only_half_up_body", onlyHalfUpBodyParam);
    onlyHalfUpBody_ = onlyHalfUpBodyParam;
    ROS_INFO("[WheelArmControlBaseROS] only_half_up_body: %s", onlyHalfUpBody_ ? "true" : "false");
  }

  // Load arm movement speed parameter
  nodeHandle_.param("/arm_move_spd_half_up_body", maxSpeed_, 0.21);
  ROS_INFO("[WheelArmControlBaseROS] maxSpeed: %.4f rad", maxSpeed_);

  // Load arm difference threshold parameter
  nodeHandle_.param("/threshold_arm_diff_half_up_body", thresholdArmDiffHalfUpBody_rad_, 0.2);
  ROS_INFO("[WheelArmControlBaseROS] thresholdArmDiffHalfUpBody: %.4f rad", thresholdArmDiffHalfUpBody_rad_);

  nodeHandle_.param("/ik_ros_uni_cpp_node/control_torso", controlTorso_, false);
  ROS_INFO("[WheelArmControlBaseROS] controlTorso: %s", controlTorso_ ? "true" : "false");

  nodeHandle_.param("/quest3/enable_wbc_arm_trajectory", enableWbcArmTrajectory_, true);
  ROS_INFO("[WheelArmControlBaseROS] enableWbcArmTrajectory: %s", enableWbcArmTrajectory_ ? "true" : "false");

  ROS_INFO("[WheelArmControlBaseROS] Parameters loaded successfully");
}

void WheelArmControlBaseROS::bonePosesCallback(const noitom_hi5_hand_udp_python::PoseInfoList::ConstPtr& msg) {
  {
    std::lock_guard<std::mutex> lock(bonePosesMutex_);
    *latestBonePosesPtr_ = *msg;
  }
  processBonePoses(msg);

  // 检查是否需要保存时间戳记录
  checkAndSaveTimestampRecords();
}

void WheelArmControlBaseROS::processBonePoses(const noitom_hi5_hand_udp_python::PoseInfoList::ConstPtr& msg) {
  if (!quest3ArmInfoTransformerPtr_) return;
  if (!quest3ArmInfoTransformerPtr_->updateHandPoseAndElbowPosition(*msg, *HandPoseAndElbowPositonListPtr_)) return;
}

void WheelArmControlBaseROS::joystickCallback(const noitom_hi5_hand_udp_python::JoySticks::ConstPtr& msg) {
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
  if (msg->left_trigger > 0.8 && msg->right_trigger > 0.8) {
    gripHoldCount_ = gripHoldCount_ > 1000 ? 1000 : gripHoldCount_ + 1;
  }
}

void WheelArmControlBaseROS::updateRunningState() {
  isRunningLast_.store(isRunning_.load());

  if (!wasRunning() && isRunning()) {
    ROS_INFO("[WheelArmControlBaseROS] Detected state change from stopped to running, setting armModeChanging to true");
    armModeChanging_.store(true);
  }
}

void WheelArmControlBaseROS::publishEndEffectorControlData() {
  if (!joyStickHandlerPtr_) {
    ROS_WARN("[WheelArmControlBaseROS] WheelJoyStickHandler not initialized");
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

void WheelArmControlBaseROS::publishHandPositionData() {
  if (!joyStickHandlerPtr_) {
    ROS_WARN("[WheelArmControlBaseROS] WheelJoyStickHandler not initialized for hand position data");
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

void WheelArmControlBaseROS::publishClawCommandData() {
  if (!joyStickHandlerPtr_) {
    ROS_WARN("[WheelArmControlBaseROS] WheelJoyStickHandler not initialized for claw command data");
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

void WheelArmControlBaseROS::initializeArmInfoTransformerFromJson(const nlohmann::json& configJson) {
  ROS_INFO("==================================================================================");
  ROS_INFO("🔧 [WheelArmControlBaseROS] Initializing Quest3ArmInfoTransformer from JSON configuration");
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

  ROS_INFO("🎯 [WheelArmControlBaseROS] Quest3ArmInfoTransformer initialization completed");
  ROS_INFO("==================================================================================");
}

template <typename T, typename UpdateFunc>
void WheelArmControlBaseROS::processJsonParameter(const nlohmann::json& configJson,
                                             const std::string& paramName,
                                             UpdateFunc updateFunction) {
  if (configJson.contains(paramName)) {
    T value = configJson[paramName].get<T>();
    updateFunction(value);
    ROS_INFO("✅ [WheelArmControlBaseROS] Initialized %s: %.4f", paramName.c_str(), static_cast<double>(value));
  } else {
    ROS_WARN("❌ [WheelArmControlBaseROS] '%s' not found in JSON config, using default values", paramName.c_str());
  }
}

std::vector<std::string> WheelArmControlBaseROS::loadFrameNamesFromConfig(const nlohmann::json& configJson) {
  ROS_INFO("==================================================================================");
  ROS_INFO("🔧 [WheelArmControlBaseROS] Auto-loading frame names from provided JSON configuration");
  ROS_INFO("==================================================================================");

  std::vector<std::string> frameNames;

  try {
    // 从JSON中提取end_frames_name_ik配置
    if (configJson.contains("end_frames_name_ik") && configJson["end_frames_name_ik"].is_array()) {
      frameNames = configJson["end_frames_name_ik"].get<std::vector<std::string>>();
      ROS_INFO("✅ [WheelArmControlBaseROS] Successfully loaded %zu frame names from JSON config:", frameNames.size());
      for (size_t i = 0; i < frameNames.size(); ++i) {
        ROS_INFO("  [%zu] %s", i, frameNames[i].c_str());
      }
    } else {
      ROS_WARN("❌ [WheelArmControlBaseROS] 'end_frames_name_ik' not found in JSON config, using hardcoded values");
      frameNames = {"base_link", "zarm_l7_end_effector", "zarm_r7_end_effector", "zarm_l4_link", "zarm_r4_link"};
    }

  } catch (const std::exception& e) {
    ROS_ERROR("❌ [WheelArmControlBaseROS] Exception while loading frame names: %s", e.what());
    ROS_WARN("🔄 [WheelArmControlBaseROS] Falling back to hardcoded frame names");
    frameNames = {"base_link", "zarm_l7_end_effector", "zarm_r7_end_effector", "zarm_l4_link", "zarm_r4_link"};
  }

  // 验证frame names不为空
  if (frameNames.empty()) {
    ROS_ERROR("❌ [WheelArmControlBaseROS] Frame names list is empty, using hardcoded values");
    frameNames = {"base_link", "zarm_l7_end_effector", "zarm_r7_end_effector", "zarm_l4_link", "zarm_r4_link"};
  }

  ROS_INFO("🎯 [WheelArmControlBaseROS] Final frame names configuration:");
  for (size_t i = 0; i < frameNames.size(); ++i) {
    ROS_INFO("  [%zu] %s", i, frameNames[i].c_str());
  }
  ROS_INFO("==================================================================================");

  return frameNames;
}

void WheelArmControlBaseROS::initializeKeyFramesVisualizer() {
  ROS_INFO("[WheelArmControlBaseROS] Initializing WheelKeyFramesVisualizer...");
  quest3KeyFramesVisualizerPtr_ = std::make_unique<WheelKeyFramesVisualizer>(nodeHandle_);
  quest3KeyFramesVisualizerPtr_->initialize();
  ROS_INFO("[WheelArmControlBaseROS] WheelKeyFramesVisualizer initialized successfully");
}

void WheelArmControlBaseROS::publishVisualizationMarkersForSide(const std::string& side,
                                                           const Eigen::Vector3d& handPos,
                                                           const Eigen::Vector3d& elbowPos,
                                                           const Eigen::Vector3d& shoulderPos,
                                                           const Eigen::Vector3d& chestPos) {
  if (quest3KeyFramesVisualizerPtr_) {
    quest3KeyFramesVisualizerPtr_->publishVisualizationMarkersForSide(side, handPos, elbowPos, shoulderPos, chestPos);
  }
}

void WheelArmControlBaseROS::recordTimestamp(const std::string& stepName, int64_t loopCount) {
  std::lock_guard<std::mutex> lock(timestampMutex_);

  // 获取当前时间戳（微秒）
  auto now = std::chrono::system_clock::now();
  auto duration = now.time_since_epoch();
  uint64_t timestamp = std::chrono::duration_cast<std::chrono::microseconds>(duration).count();

  // 添加记录
  timestampRecords_.push_back({stepName, timestamp, loopCount});
}

void WheelArmControlBaseROS::saveTimestampRecordsToFile() {
  std::lock_guard<std::mutex> lock(timestampMutex_);

  if (timestampRecords_.empty()) {
    return;
  }

  // 生成文件名：使用当前时间戳，保存在工作空间的logs目录
  auto now = std::chrono::system_clock::now();
  auto time_t_now = std::chrono::system_clock::to_time_t(now);
  std::tm tm_now;
  localtime_r(&time_t_now, &tm_now);

  // 创建logs目录（如果不存在）
  system("mkdir -p /root/kuavo_ws/logs");

  char filename[512];
  snprintf(filename,
           sizeof(filename),
           "/root/kuavo_ws/logs/timestamp_log_%04d%02d%02d_%02d%02d%02d.csv",
           tm_now.tm_year + 1900,
           tm_now.tm_mon + 1,
           tm_now.tm_mday,
           tm_now.tm_hour,
           tm_now.tm_min,
           tm_now.tm_sec);

  std::ofstream outFile(filename);
  if (!outFile.is_open()) {
    ROS_ERROR("[WheelArmControlBaseROS] Failed to open timestamp log file: %s", filename);
    return;
  }

  // 写入CSV头
  outFile << "loop_count,step_name,timestamp_us,timestamp_readable\n";

  // 写入所有记录
  for (const auto& record : timestampRecords_) {
    // 转换时间戳为可读格式
    auto tp = std::chrono::system_clock::time_point(std::chrono::microseconds(record.timestamp));
    auto time_t_val = std::chrono::system_clock::to_time_t(tp);
    std::tm tm_val;
    localtime_r(&time_t_val, &tm_val);

    uint64_t microseconds = record.timestamp % 1000000;

    char timeStr[64];
    snprintf(timeStr,
             sizeof(timeStr),
             "%04d-%02d-%02d %02d:%02d:%02d.%06lu",
             tm_val.tm_year + 1900,
             tm_val.tm_mon + 1,
             tm_val.tm_mday,
             tm_val.tm_hour,
             tm_val.tm_min,
             tm_val.tm_sec,
             microseconds);

    outFile << record.loopCount << "," << record.stepName << "," << record.timestamp << "," << timeStr << "\n";
  }

  outFile.close();

  ROS_INFO("[WheelArmControlBaseROS] Saved %zu timestamp records to %s", timestampRecords_.size(), filename);

  // 清空数组，重新记录
  timestampRecords_.clear();

  // 更新最后保存时间
  lastSaveTime_ = std::chrono::steady_clock::now();
}

void WheelArmControlBaseROS::checkAndSaveTimestampRecords() {
  auto now = std::chrono::steady_clock::now();
  auto elapsedSeconds = std::chrono::duration_cast<std::chrono::seconds>(now - lastSaveTime_).count();

  if (elapsedSeconds >= saveIntervalSeconds_) {
    saveTimestampRecordsToFile();
  }
}

void WheelArmControlBaseROS::publishQuestJoystickDataXAndA() {
  noitom_hi5_hand_udp_python::JoySticks msg;

  // 设置 x 轴为最大值（1.0），A 按钮（first_button_pressed）为 true
  msg.left_x = 0.0;
  msg.right_x = 0.0;
  msg.left_first_button_pressed = true;
  msg.right_first_button_pressed = true;

  // 其余所有字段设置为 0 或 false
  msg.left_y = 0.0;
  msg.left_trigger = 0.0;
  msg.left_grip = 0.0;
  msg.left_second_button_pressed = false;
  msg.left_first_button_touched = false;
  msg.left_second_button_touched = false;

  msg.right_y = 0.0;
  msg.right_trigger = 0.0;
  msg.right_grip = 0.0;
  msg.right_second_button_pressed = false;
  msg.right_first_button_touched = false;
  msg.right_second_button_touched = false;

  // 发布消息
  questJoystickDataPublisher_.publish(msg);
  ROS_INFO("[WheelArmControlBaseROS] Published quest_joystick_data  A + X button=true");
}

}  // namespace HighlyDynamic
