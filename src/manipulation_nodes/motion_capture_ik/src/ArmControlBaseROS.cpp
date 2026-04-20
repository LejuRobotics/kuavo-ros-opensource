#include "motion_capture_ik/ArmControlBaseROS.h"
#include <leju_utils/define.hpp>

#include "motion_capture_ik/JoyStickHandler.h"
#include "motion_capture_ik/KeyFramesVisualizer.h"
#include "motion_capture_ik/Quest3ArmInfoTransformer.h"
#include "ros/console.h"

#include <fstream>
#include <ctime>
#include <algorithm>
#include <cmath>
#include <ocs2_core/misc/LoadData.h>

#include <kuavo_msgs/changeArmCtrlMode.h>
#include <kuavo_msgs/changeTorsoCtrlMode.h>
#include <kuavo_msgs/lejuClawCommand.h>
#include <kuavo_msgs/robotHandPosition.h>
#include <kuavo_msgs/robotWaistControl.h>
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
      numWaistJoints_(0) {
  ROS_INFO("[ArmControlBaseROS] Base class initialized with publishRate=%.2f, debugPrint=%s",
           publishRate_,
           debugPrint_ ? "true" : "false");

  // 初始化时间戳记录系统
  timestampRecords_.clear();
  timestampRecords_.reserve(10000);  // 预分配空间，避免频繁内存分配
  lastSaveTime_ = std::chrono::steady_clock::now();
  ROS_INFO("[ArmControlBaseROS] Timestamp recording system initialized");
}

ArmControlBaseROS::~ArmControlBaseROS() { ROS_INFO("[ArmControlBaseROS] Base class destructor called"); }

void ArmControlBaseROS::initializeBase(const nlohmann::json& configJson) {
  ROS_INFO("[ArmControlBaseROS] Initializing base ROS components...");

  // 从JSON配置读取腰部自由度数量
  if (configJson.contains("NUM_WAIST_JOINT")) {
    numWaistJoints_ = configJson["NUM_WAIST_JOINT"].get<int>();
    ROS_INFO("✅ [ArmControlBaseROS] Set waist DOF from JSON: %d", numWaistJoints_);
  } else {
    ROS_WARN("⚠️  [ArmControlBaseROS] 'NUM_WAIST_JOINT' field not found in JSON configuration, using default value: 0");
    numWaistJoints_ = 0;
  }

  // Initialize service client for arm control mode
  changeArmCtrlModeClient_ = nodeHandle_.serviceClient<kuavo_msgs::changeArmCtrlMode>("/change_arm_ctrl_mode");
  changeArmModeClient_ = nodeHandle_.serviceClient<kuavo_msgs::changeArmCtrlMode>("/change_arm_ctrl_mode");
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
  questJoystickDataPublisher_ =
      nodeHandle_.advertise<noitom_hi5_hand_udp_python::JoySticks>("/quest_joystick_data", 10);
  waistMotionPublisher_ = nodeHandle_.advertise<kuavo_msgs::robotWaistControl>("/robot_waist_motion_data", 1);
  wholeTorsoCtrlPublisher_ = nodeHandle_.advertise<std_msgs::Bool>("/vr_whole_torso_ctrl", 1);
  vrWaistControlServiceClient_ = nodeHandle_.serviceClient<std_srvs::SetBool>("/humanoid/mpc/vr_waist_control");

  // Load parameters from ROS parameter server
  loadParameters();
  initializeTorsoControlFromReference();

  // Load robot joint dimension parameters from JSON configuration with fallback compatibility
  loadJointDimensionsWithFallback(configJson);

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
    ROS_INFO("✅ [ArmControlBaseROS] Loaded joystick filter parameters: threshold=%.4f, alpha=%.4f",
             joyStickThreshold,
             joyStickAlpha);
  } else {
    ROS_WARN(
        "❌ [ArmControlBaseROS] 'joystick_filter' not found in JSON config, using default values: threshold=%.4f, "
        "alpha=%.4f",
        joyStickThreshold,
        joyStickAlpha);
  }
  joyStickHandlerPtr_ = std::make_unique<JoyStickHandler>(joyStickThreshold, joyStickAlpha);
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
  const int requiredSize = sensorDataArmOffset_ + numArmJoints_;

  ROS_INFO("[ArmControlBaseROS] Joint dimension info: offset=%d, numArm=%d, required_size=%d, actual_size=%zu",
           sensorDataArmOffset_,
           numArmJoints_,
           requiredSize,
           jointQSize);
  const int armJointStartIndex = 12 + numWaistJoints_;  // 考虑腰部自由度
  const int numArmJoints = 14;

  ROS_INFO("[ArmControlBaseROS] joint_q array size: %zu, required: %d (waist_dof: %d)", 
           jointQSize, armJointStartIndex + numArmJoints, numWaistJoints_);

  if (jointQSize < static_cast<size_t>(requiredSize)) {
    std::string errorMsg = "joint_q array too small! Size: " + std::to_string(jointQSize) +
                           ", required: " + std::to_string(requiredSize) +
                           " (offset=" + std::to_string(sensorDataArmOffset_) + " + numArm=" +
                           std::to_string(numArmJoints_) + ")";
    ROS_ERROR("[ArmControlBaseROS] %s", errorMsg.c_str());
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

    // 安全的数组访问：统一使用 sensorDataArmOffset_ + i 访问手臂关节
    // This ensures compatibility with both v49 (offset=12) and v60 (offset=4) layouts
    int outOfBoundsCount = 0;
    for (int i = 0; i < numArmJoints_; ++i) {
      const int jointIndex = sensorDataArmOffset_ + i;
      if (jointIndex >= 0 && jointIndex < static_cast<int>(jointQSize)) {
        msg.position[i] = currentSensorData->joint_data.joint_q[jointIndex] * 180.0 / M_PI;
      } else {
        outOfBoundsCount++;
        msg.position[i] = 0.0;
      }
    }

    if (outOfBoundsCount > 0) {
      ROS_WARN("[ArmControlBaseROS] %d joint(s) out of bounds during safety initialization (using 0.0)",
               outOfBoundsCount);
    }

    // 发布20次（复现Python L1079-1081）
    for (int i = 0; i < 20; ++i) {
      kuavoArmTrajCppPublisher_.publish(msg);
      rate.sleep();
    }

    ROS_INFO("[ArmControlBaseROS] Successfully published %d joint states for safety initialization (offset=%d)",
             numArmJoints_,
             sensorDataArmOffset_);
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

bool ArmControlBaseROS::initializeArmControlMode() {
  kuavo_msgs::changeArmCtrlMode srv;
  srv.request.control_mode = 1;
  bool humanoidCallOk = false;
  bool vrCallOk = false;
  if (humanoidArmCtrlModeClient_.exists()) {
    humanoidCallOk = humanoidArmCtrlModeClient_.call(srv) && srv.response.result;
  } else {
    ROS_WARN("[ArmControlBaseROS] Service /humanoid_change_arm_ctrl_mode does not exist");
  }
  if (changeArmCtrlModeClient_.exists()) {
    vrCallOk = changeArmCtrlModeClient_.call(srv) && srv.response.result;
  } else {
    ROS_WARN("[ArmControlBaseROS] Service /change_arm_ctrl_mode does not exist");
  }
  if (humanoidCallOk && vrCallOk) {
    ROS_INFO("[ArmControlBaseROS] Arm control mode set to 1 during initialization");
    return true;
  } else {
    ROS_WARN("[ArmControlBaseROS] Failed to set arm control mode to 1 during initialization");
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

  // 检查是否需要保存时间戳记录
  checkAndSaveTimestampRecords();
}

void ArmControlBaseROS::processBonePoses(const noitom_hi5_hand_udp_python::PoseInfoList::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(transformerDataMutex_);
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
  if (msg->left_trigger > 0.8 && msg->right_trigger > 0.8) {
    gripHoldCount_ = gripHoldCount_ > 1000 ? 1000 : gripHoldCount_ + 1;
  }
}

void ArmControlBaseROS::updateRunningState() {
  isRunningLast_.store(isRunning_.load());

  if (!wasRunning() && isRunning()) {
    ROS_INFO("[ArmControlBaseROS] Detected state change from stopped to running, setting armModeChanging to true");
    armModeChanging_.store(true);
  }
}

void ArmControlBaseROS::initializeTorsoControlFromReference() {
  nodeHandle_.param("/robot_type", robotType_, 2);

  std::string referenceFile;
  nodeHandle_.getParam("/referenceFile", referenceFile);
  std::cout << "get referenceFile: " << referenceFile << std::endl;
  try {
    ocs2::loadData::loadCppDataType(referenceFile, "waist_yaw_max", waistYawMaxAngleDeg_);
  } catch (const std::exception& e) {
    ROS_WARN_STREAM("waist_yaw_max not found, using default: " << waistYawMaxAngleDeg_);
  }
}

void ArmControlBaseROS::callVRWaistControlSrv(bool enable) {
  std_srvs::SetBool srv;
  srv.request.data = enable;

  if (!vrWaistControlServiceClient_.waitForExistence(ros::Duration(2.0))) {
    ROS_WARN("VR waist control service not available, skipping call");
    return;
  }

  if (vrWaistControlServiceClient_.call(srv)) {
    if (srv.response.success) {
      ROS_INFO("VRWaistControlSrv call successful: %s, response: %s",
               enable ? "enabled" : "disabled",
               srv.response.message.c_str());
    } else {
      ROS_WARN("VRWaistControlSrv returned failure: %s", srv.response.message.c_str());
    }
  } else {
    ROS_ERROR("Failed to call VRWaistControlSrv");
  }
}

void ArmControlBaseROS::controlWaist(double waistYaw) {
  const double maxAngle = waistYawMaxAngleDeg_;
  waistYaw = std::max(-maxAngle, std::min(waistYaw, maxAngle));

  kuavo_msgs::robotWaistControl msg;
  msg.header.stamp = ros::Time::now();
  msg.data.data.resize(1);
  msg.data.data[0] = waistYaw;
  waistMotionPublisher_.publish(msg);
}

void ArmControlBaseROS::updateTorsoControl() {
  if (numWaistJoints_ == 0 || !joyStickHandlerPtr_) return;

  const float deadzone = 0.1f;
  float rightX = static_cast<float>(joyStickHandlerPtr_->getRightJoyStickX());
  if (std::abs(rightX) < deadzone) rightX = 0.0f;

  const float targetYaw = -1.0f * rightX * static_cast<float>(waistYawMaxAngleDeg_);
  controlWaist(targetYaw);
}

void ArmControlBaseROS::handleTorsoControlJoystick() {
  if (!joyStickHandlerPtr_) return;

  const bool leftFirstTouched = joyStickHandlerPtr_->isLeftFirstButtonTouched();
  const bool leftSecondTouched = joyStickHandlerPtr_->isLeftSecondButtonTouched();
  const bool rightSecondPressed = joyStickHandlerPtr_->isRightSecondButtonPressed();

  if (leftFirstTouched && leftSecondTouched) {
    const bool risingEdge = (!prevRightSecondButtonPressed_) && rightSecondPressed;
    if (risingEdge) {
      if (!torsoControlEnabled_ && controlTorso_) {
        torsoControlEnabled_ = true;
        torsoPitchZero_ = currentHeadBodyPose_.body_pitch;
        torsoYawZero_ = currentHeadBodyPose_.body_yaw;
        bodyHeightZero_ = currentHeadBodyPose_.body_height;
        bodyXZero_ = currentHeadBodyPose_.body_x;
        lastBodyYaw_ = currentHeadBodyPose_.body_yaw;
        accumulatedYawOffset_ = 0.0;

        std_msgs::Bool msg;
        msg.data = true;
        wholeTorsoCtrlPublisher_.publish(msg);
        callVRWaistControlSrv(true);

        ROS_INFO("[ArmControlBaseROS] Torso control enabled: yaw_zero=%.4f, pitch_zero=%.4f",
                 torsoYawZero_,
                 torsoPitchZero_);
      } else {
        torsoControlEnabled_ = false;
        std_msgs::Bool msg;
        msg.data = false;
        wholeTorsoCtrlPublisher_.publish(msg);
        callVRWaistControlSrv(false);
        ROS_INFO("[ArmControlBaseROS] Torso control disabled");
      }
    }
  }

  if (leftSecondTouched && !leftFirstTouched && !torsoControlEnabled_) {
    updateTorsoControl();
  }

  prevRightSecondButtonPressed_ = rightSecondPressed;
}

void ArmControlBaseROS::processAbsoluteTorsoControl(const HeadBodyPose& headBodyPose) {
  currentHeadBodyPose_ = headBodyPose;
  if (!torsoControlEnabled_ || numWaistJoints_ == 0) return;

  const double currentYaw = currentHeadBodyPose_.body_yaw;
  const double yawDiff = currentYaw - lastBodyYaw_;
  if (yawDiff > M_PI) {
    accumulatedYawOffset_ -= 2.0 * M_PI;
  } else if (yawDiff < -M_PI) {
    accumulatedYawOffset_ += 2.0 * M_PI;
  }
  lastBodyYaw_ = currentYaw;

  const double continuousYaw = currentYaw + accumulatedYawOffset_;
  const double relativeYaw = continuousYaw - torsoYawZero_;
  controlWaist(relativeYaw * 180.0 / M_PI);
}

void ArmControlBaseROS::processTorsoControlLoop() {
  handleTorsoControlJoystick();

  HeadBodyPose headBodyPose;
  {
    std::lock_guard<std::mutex> lock(transformerDataMutex_);
    if (!quest3ArmInfoTransformerPtr_) return;
    headBodyPose = quest3ArmInfoTransformerPtr_->getHeadBodyPose();
  }
  processAbsoluteTorsoControl(headBodyPose);
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

void ArmControlBaseROS::recordTimestamp(const std::string& stepName, int64_t loopCount) {
  std::lock_guard<std::mutex> lock(timestampMutex_);

  // 获取当前时间戳（微秒）
  auto now = std::chrono::system_clock::now();
  auto duration = now.time_since_epoch();
  uint64_t timestamp = std::chrono::duration_cast<std::chrono::microseconds>(duration).count();

  // 添加记录
  timestampRecords_.push_back({stepName, timestamp, loopCount});
}

void ArmControlBaseROS::saveTimestampRecordsToFile() {
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
    ROS_ERROR("[ArmControlBaseROS] Failed to open timestamp log file: %s", filename);
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

  ROS_INFO("[ArmControlBaseROS] Saved %zu timestamp records to %s", timestampRecords_.size(), filename);

  // 清空数组，重新记录
  timestampRecords_.clear();

  // 更新最后保存时间
  lastSaveTime_ = std::chrono::steady_clock::now();
}

void ArmControlBaseROS::checkAndSaveTimestampRecords() {
  auto now = std::chrono::steady_clock::now();
  auto elapsedSeconds = std::chrono::duration_cast<std::chrono::seconds>(now - lastSaveTime_).count();

  if (elapsedSeconds >= saveIntervalSeconds_) {
    saveTimestampRecordsToFile();
  }
}

void ArmControlBaseROS::publishQuestJoystickDataXAndA() {
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
  ROS_INFO("[ArmControlBaseROS] Published quest_joystick_data  A + X button=true");
}

void ArmControlBaseROS::loadJointDimensionsWithFallback(const nlohmann::json& configJson) {
  ROS_INFO("==================================================================================");
  ROS_INFO("🔧 [ArmControlBaseROS] Loading robot joint dimensions from JSON configuration");
  ROS_INFO("==================================================================================");

  // Step 1: Read from JSON with default fallback values (legacy v49 layout)
  int numArm = configJson.value("NUM_ARM_JOINT", 14);
  int numHead = configJson.value("NUM_HEAD_JOINT", 2);
  int numWaist = configJson.value("NUM_WAIST_JOINT", 0);
  int numTotal = configJson.value("NUM_JOINT", 28);

  // Step 2: Validate dimensions and calculate offset
  int offset = 0;
  bool isValid = validateJointDimensions(numArm, numHead, numWaist, numTotal, offset);

  // Step 3: If validation failed, fallback to legacy defaults
  if (!isValid) {
    ROS_WARN("==================================================================================");
    ROS_WARN("⚠️  [ArmControlBaseROS] Invalid joint dimension configuration detected!");
    ROS_WARN("   Falling back to legacy default layout (v49: 28 total, 14 arm, 2 head, offset=12)");
    ROS_WARN("==================================================================================");

    numArm = 14;
    numHead = 2;
    numWaist = 0;
    numTotal = 28;
    offset = numTotal - numHead - numArm;  // Should be 12

    // Re-validate fallback values (should always pass)
    if (!validateJointDimensions(numArm, numHead, numWaist, numTotal, offset)) {
      ROS_ERROR("[ArmControlBaseROS] CRITICAL: Fallback validation failed! This should never happen.");
      // Force valid values to prevent crash
      numArm = 14;
      numHead = 2;
      numWaist = 0;
      numTotal = 28;
      offset = 12;
    }
  }

  // Step 4: Assign validated values to member variables
  numArmJoints_ = numArm;
  numHeadJoints_ = numHead;
  numWaistJoints_ = numWaist;
  numTotalJoints_ = numTotal;
  sensorDataArmOffset_ = offset;

  // Step 5: Print final configuration
  ROS_INFO("✅ [ArmControlBaseROS] Final joint dimensions (after validation/fallback):");
  ROS_INFO("   - NUM_ARM_JOINT: %d", numArmJoints_);
  ROS_INFO("   - NUM_HEAD_JOINT: %d", numHeadJoints_);
  ROS_INFO("   - NUM_WAIST_JOINT: %d", numWaistJoints_);
  ROS_INFO("   - NUM_JOINT (total): %d", numTotalJoints_);
  ROS_INFO("   - Sensor data arm offset (calculated): %d", sensorDataArmOffset_);
  ROS_INFO("📊 [ArmControlBaseROS] Sensor data layout:");
  if (sensorDataArmOffset_ > 0) {
    ROS_INFO("   - Leg joints:  indices [0-%d] (%d joints)", sensorDataArmOffset_ - 1, sensorDataArmOffset_);
  } else {
    ROS_INFO("   - Leg joints:  none (offset=0)");
  }
  ROS_INFO("   - Arm joints:  indices [%d-%d] (%d joints)",
           sensorDataArmOffset_,
           sensorDataArmOffset_ + numArmJoints_ - 1,
           numArmJoints_);
  ROS_INFO("   - Head joints: indices [%d-%d] (%d joints)",
           sensorDataArmOffset_ + numArmJoints_,
           numTotalJoints_ - 1,
           numHeadJoints_);
  ROS_INFO("==================================================================================");
}

bool ArmControlBaseROS::validateJointDimensions(int& numArm, int& numHead, int& numWaist, int& numTotal, int& offset) const {
  // Validation rules:
  // 1. numArm must be > 0 (must have at least one arm joint)
  // 2. numHead must be >= 0 (head joints are optional)
  // 3. numWaist must be >= 0 (waist joints are optional)
  // 4. numTotal must be >= numArm + numHead (total must cover at least arm + head)
  // 5. offset = numTotal - numHead - numArm must be >= 0 (leg joints count, can be 0)

  bool hasError = false;
  std::string errorDetails;

  if (numArm <= 0) {
    hasError = true;
    errorDetails += "NUM_ARM_JOINT must be > 0 (got " + std::to_string(numArm) + "); ";
  }

  if (numHead < 0) {
    hasError = true;
    errorDetails += "NUM_HEAD_JOINT must be >= 0 (got " + std::to_string(numHead) + "); ";
  }

  if (numWaist < 0) {
    hasError = true;
    errorDetails += "NUM_WAIST_JOINT must be >= 0 (got " + std::to_string(numWaist) + "); ";
  }

  if (numTotal < numArm + numHead) {
    hasError = true;
    errorDetails += "NUM_JOINT (" + std::to_string(numTotal) + ") must be >= NUM_ARM_JOINT (" +
                    std::to_string(numArm) + ") + NUM_HEAD_JOINT (" + std::to_string(numHead) + "); ";
  }

  // Calculate offset
  offset = numTotal - numHead - numArm;

  if (offset < 0) {
    hasError = true;
    errorDetails += "Calculated offset (" + std::to_string(offset) + ") must be >= 0; ";
  }

  if (hasError) {
    ROS_ERROR("[ArmControlBaseROS] Joint dimension validation failed: %s", errorDetails.c_str());
    return false;
  }

  return true;
}

}  // namespace HighlyDynamic
