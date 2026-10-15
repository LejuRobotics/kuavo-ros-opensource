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

  // input_mode 必须在使用前读取：single_vr（默认）走原有单话题订阅，
  // dual_vr 走按侧话题订阅（且不订阅自己发布的 /quest_joystick_data，避免 self-loop）。
  nodeHandle_.param("/ik_ros_uni_cpp_node/quest3/input_mode", inputMode_, std::string("single_vr"));
  useDualVr_ = (inputMode_ == "dual_vr");
  ROS_INFO("[WheelArmControlBaseROS] input_mode: %s", inputMode_.c_str());

  if (useDualVr_) {
    bonePosesLeftSubscriber_ = nodeHandle_.subscribe(
        "/vr_left/leju_quest_bone_poses", 10, &WheelArmControlBaseROS::bonePosesLeftCallback, this,
        ros::TransportHints().tcpNoDelay());
    bonePosesRightSubscriber_ = nodeHandle_.subscribe(
        "/vr_right/leju_quest_bone_poses", 10, &WheelArmControlBaseROS::bonePosesRightCallback, this,
        ros::TransportHints().tcpNoDelay());
    joystickLeftSubscriber_ = nodeHandle_.subscribe(
        "/vr_left/quest_joystick_data", 10, &WheelArmControlBaseROS::joystickLeftCallback, this,
        ros::TransportHints().tcpNoDelay());
    joystickRightSubscriber_ = nodeHandle_.subscribe(
        "/vr_right/quest_joystick_data", 10, &WheelArmControlBaseROS::joystickRightCallback, this,
        ros::TransportHints().tcpNoDelay());
    ROS_INFO("[WheelArmControlBaseROS] dual_vr: subscribing /vr_left|right/leju_quest_bone_poses and "
             "/vr_left|right/quest_joystick_data");
  } else {
    bonePosesSubscriber_ = nodeHandle_.subscribe(
        "/leju_quest_bone_poses", 10, &WheelArmControlBaseROS::bonePosesCallback, this,
        ros::TransportHints().tcpNoDelay());
    joystickSubscriber_ = nodeHandle_.subscribe(
        "/quest_joystick_data", 10, &WheelArmControlBaseROS::joystickCallback, this,
        ros::TransportHints().tcpNoDelay());
  }

  sensorDataRaw_ = std::make_shared<kuavo_msgs::sensorsData>();
  latestBonePosesPtr_ = std::make_shared<noitom_hi5_hand_udp_python::PoseInfoList>();

  robotHandPositionPublisher_ =
      nodeHandle_.advertise<kuavo_msgs::robotHandPosition>("/control_robot_hand_position", 10);
  lejuClawCommandPublisher_ = nodeHandle_.advertise<kuavo_msgs::lejuClawCommand>("/leju_claw_command", 10);

  headBodyPosePublisher_ = nodeHandle_.advertise<kuavo_msgs::headBodyPose>("/kuavo_head_body_orientation_data", 10);
  kuavoArmTrajCppPublisher_ = nodeHandle_.advertise<sensor_msgs::JointState>("/kuavo_arm_traj_cpp", 2);
  questJoystickDataPublisher_ =
      nodeHandle_.advertise<noitom_hi5_hand_udp_python::JoySticks>("/quest_joystick_data", 10);
  commLatencyPublisher_ = nodeHandle_.advertise<std_msgs::Float64>("/vr_incremental/comm_latency_ms", 10);

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
  if (useDualVr_) {
    // dual_vr：两个按侧独立实例。Quest3ArmInfoTransformer 是 final 类且有状态
    // （chestPosition_ / qInitChest_ / yawOnlyQuat_ / *HandPoseRelativeToChest_ / 摇杆状态），
    // 绝不能用同一实例先 update A 再 update B。
    quest3ArmInfoTransformerLeftPtr_ = std::make_unique<HighlyDynamic::Quest3ArmInfoTransformer>();
    quest3ArmInfoTransformerRightPtr_ = std::make_unique<HighlyDynamic::Quest3ArmInfoTransformer>();
    ROS_INFO("[WheelArmControlBaseROS] dual_vr: created independent Left/Right Quest3ArmInfoTransformer");
  }
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

Quest3ArmInfoTransformer* WheelArmControlBaseROS::getArmInfoTransformerForSide(const std::string& side) const {
  if (!useDualVr_) {
    // single_vr：两侧都返回主实例，行为与改造前完全一致
    return quest3ArmInfoTransformerPtr_.get();
  }
  if (side == "Left") return quest3ArmInfoTransformerLeftPtr_.get();
  if (side == "Right") return quest3ArmInfoTransformerRightPtr_.get();
  ROS_WARN("[WheelArmControlBaseROS] getArmInfoTransformerForSide: invalid side '%s'", side.c_str());
  return nullptr;
}

// dual_vr：本侧骨骼数据的"新鲜度判定 + Chest 快照"必须在同一个临界区内完成。
// 按侧时间戳只在各侧 bone 回调里写，且与 transformer 位姿成员同持 transformerDataMutex_，
// 所以这里读同一把锁；否则可能出现"读到的是新帧的 Chest，但用的是旧帧的时间戳"这类跨帧拼接。
bool WheelArmControlBaseROS::getFreshSideChestPose(const std::string& side,
                                                   int64_t staleTimeoutMs,
                                                   Eigen::Vector3d& chestPos,
                                                   Eigen::Quaterniond* chestQuatOut) {
  std::lock_guard<std::mutex> lock(transformerDataMutex_);
  const Quest3ArmInfoTransformer* transformer = getArmInfoTransformerForSide(side);
  if (!transformer) return false;
  const std::chrono::steady_clock::time_point& recvTime =
      (side == "Right") ? rightBoneRecvTime_ : leftBoneRecvTime_;
  if (recvTime.time_since_epoch().count() == 0) {
    return false;  // 从未收到过本侧骨骼帧
  }
  const int64_t ageMs =
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - recvTime).count();
  if (ageMs > staleTimeoutMs) {
    return false;  // 本侧骨骼数据已过期（Quest 断连/丢帧）
  }
  if (!transformer->hasValidChestPose()) {
    return false;  // 本侧尚未产出过有效 Chest
  }
  chestPos = transformer->getChestPosition();
  if (chestQuatOut) {
    *chestQuatOut = transformer->getChestQuaternion();
  }
  return true;
}

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

  nodeHandle_.param("/enable_timestamp_log", enableTimestampLog_, false);
  ROS_INFO("[WheelArmControlBaseROS] enableTimestampLog: %s", enableTimestampLog_ ? "true" : "false");

  // 仅在启用日志时预分配记录空间
  if (enableTimestampLog_) {
    timestampRecords_.reserve(10000);
    lastSaveTime_ = std::chrono::steady_clock::now();
    ROS_INFO("[WheelArmControlBaseROS] Timestamp recording system initialized");
  }

  ROS_INFO("[WheelArmControlBaseROS] Parameters loaded successfully");
}

void WheelArmControlBaseROS::bonePosesCallback(const noitom_hi5_hand_udp_python::PoseInfoList::ConstPtr& msg) {
  {
    std::lock_guard<std::mutex> lock(bonePosesMutex_);
    *latestBonePosesPtr_ = *msg;
  }
  // 记录骨骼数据到达时刻，用于跨线程全链路延迟测量
  {
    std::lock_guard<std::mutex> lock(boneRecvTimeMutex_);
    boneRecvTime_ = std::chrono::steady_clock::now();
    ++boneDataSeq_;
    boneVrTimestampMs_ = msg->timestamp_ms;
  }
  // 测量通信延迟：VR节点发布时刻(timestamp_ms) → IK节点回调接收时刻
  if (commLatencyPublisher_ && msg->timestamp_ms > 0) {
    const double nowSec = ros::Time::now().toSec();
    const double vrSec = static_cast<double>(msg->timestamp_ms) / 1000.0;
    const double commLatencyMs = (nowSec - vrSec) * 1000.0;
    std_msgs::Float64 commMsg;
    commMsg.data = commLatencyMs;
    commLatencyPublisher_.publish(commMsg);
  }
  processBonePoses(msg);

  // 检查是否需要保存时间戳记录
  checkAndSaveTimestampRecords();
}

void WheelArmControlBaseROS::processBonePoses(const noitom_hi5_hand_udp_python::PoseInfoList::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(transformerDataMutex_);
  if (!quest3ArmInfoTransformerPtr_) return;
  if (!quest3ArmInfoTransformerPtr_->updateHandPoseAndElbowPosition(*msg, *HandPoseAndElbowPositonListPtr_)) return;
}

void WheelArmControlBaseROS::completeSideArmLengthMeasurementIfReady(Quest3ArmInfoTransformer* transformer,
                                                                    const std::string& side) {
  if (!transformer || transformer->isArmLengthMeasurementComplete()) return;
  if (transformer->armLengthSampleCountForSide(side) <
      static_cast<size_t>(Quest3ArmInfoTransformer::armLengthSampleTarget())) {
    return;
  }
  // 同一线程内完成，避免与 scaleArmPositions→updateMeasurement 的样本写入竞争
  transformer->completeArmLengthMeasurement();
  ROS_INFO("[WheelArmControlBaseROS] %s arm length measurement completed (dual_vr, %d samples, own side only)",
           side.c_str(),
           Quest3ArmInfoTransformer::armLengthSampleTarget());
}

void WheelArmControlBaseROS::bonePosesLeftCallback(const noitom_hi5_hand_udp_python::PoseInfoList::ConstPtr& msg) {
  // Quest-A 独占 Left：只更新 left 实例，绝不触碰 right 实例，
  // 因此 A 侧新帧不会重置/污染 B 侧的 chest、reference 状态。
  std::lock_guard<std::mutex> lock(transformerDataMutex_);
  if (!quest3ArmInfoTransformerLeftPtr_) return;
  // 只有成功处理的骨骼帧才刷新本侧接收时刻：无效帧（validateInput 失败 / computeHandPose 失败）
  // 不会产出新的 Chest，若仍刷新时间戳，getFreshSideChestPose 会把历史 Chest 误判为仍然有效。
  if (!quest3ArmInfoTransformerLeftPtr_->updateHandPoseForSide(*msg, "Left")) return;
  leftBoneRecvTime_ = std::chrono::steady_clock::now();
  completeSideArmLengthMeasurementIfReady(quest3ArmInfoTransformerLeftPtr_.get(), "Left");
}

void WheelArmControlBaseROS::bonePosesRightCallback(const noitom_hi5_hand_udp_python::PoseInfoList::ConstPtr& msg) {
  // Quest-B 独占 Right
  std::lock_guard<std::mutex> lock(transformerDataMutex_);
  if (!quest3ArmInfoTransformerRightPtr_) return;
  // 同左侧：无效帧不刷新本侧接收时刻。
  if (!quest3ArmInfoTransformerRightPtr_->updateHandPoseForSide(*msg, "Right")) return;
  rightBoneRecvTime_ = std::chrono::steady_clock::now();
  completeSideArmLengthMeasurementIfReady(quest3ArmInfoTransformerRightPtr_.get(), "Right");
}

void WheelArmControlBaseROS::joystickLeftCallback(const noitom_hi5_hand_udp_python::JoySticks::ConstPtr& msg) {
  {
    std::lock_guard<std::mutex> lock(joystickLeftMutex_);
    // 只采信 Quest-A 自己拥有的 left_* 8 个字段，右半部分一律忽略
    // （对侧 monitor 消息里的默认 0/false 不可信）
    latestLeftJoystick_.left_x = msg->left_x;
    latestLeftJoystick_.left_y = msg->left_y;
    latestLeftJoystick_.left_trigger = msg->left_trigger;
    latestLeftJoystick_.left_grip = msg->left_grip;
    latestLeftJoystick_.left_first_button_pressed = msg->left_first_button_pressed;
    latestLeftJoystick_.left_second_button_pressed = msg->left_second_button_pressed;
    latestLeftJoystick_.left_first_button_touched = msg->left_first_button_touched;
    latestLeftJoystick_.left_second_button_touched = msg->left_second_button_touched;
    leftJoystickRecvTime_ = std::chrono::steady_clock::now();
    hasLeftJoystick_ = true;
  }
  updateMergedJoystick();
}

void WheelArmControlBaseROS::joystickRightCallback(const noitom_hi5_hand_udp_python::JoySticks::ConstPtr& msg) {
  {
    std::lock_guard<std::mutex> lock(joystickRightMutex_);
    // 只采信 Quest-B 自己拥有的 right_* 8 个字段
    latestRightJoystick_.right_x = msg->right_x;
    latestRightJoystick_.right_y = msg->right_y;
    latestRightJoystick_.right_trigger = msg->right_trigger;
    latestRightJoystick_.right_grip = msg->right_grip;
    latestRightJoystick_.right_first_button_pressed = msg->right_first_button_pressed;
    latestRightJoystick_.right_second_button_pressed = msg->right_second_button_pressed;
    latestRightJoystick_.right_first_button_touched = msg->right_first_button_touched;
    latestRightJoystick_.right_second_button_touched = msg->right_second_button_touched;
    rightJoystickRecvTime_ = std::chrono::steady_clock::now();
    hasRightJoystick_ = true;
  }
  updateMergedJoystick();
}

bool WheelArmControlBaseROS::isJoystickSideStale(const std::chrono::steady_clock::time_point& recvTime,
                                                 bool hasData,
                                                 const std::chrono::steady_clock::time_point& now) const {
  if (!hasData) return true;
  const auto elapsed = now - recvTime;
  return std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() > kJoystickStaleTimeoutMs;
}

void WheelArmControlBaseROS::updateMergedJoystick() {
  // 左右 callback 可并发执行，updateMergedJoystick 也会被两个 callback 调用，
  // 因此这里先按侧取一致快照（msg + 接收时刻 + 有效标志同锁读取），
  // 再在锁外做超时判断与字段融合；绝不在锁外直接读这些共享成员。
  // 整函数由 joystickMergeMutex_ 串行化：并发进入的第二次调用会等到第一次
  // 完成"取快照→判超时→组装→发布→下发"后才开始，避免旧快照晚发布覆盖新快照。
  // 该锁为最外层，内部只顺序持有某一把下属锁，不与其嵌套。
  std::lock_guard<std::mutex> mergeLock(joystickMergeMutex_);
  noitom_hi5_hand_udp_python::JoySticks leftSnapshot;
  std::chrono::steady_clock::time_point leftRecvTime{};
  bool hasLeftSnapshot = false;
  {
    std::lock_guard<std::mutex> lock(joystickLeftMutex_);
    leftSnapshot = latestLeftJoystick_;
    leftRecvTime = leftJoystickRecvTime_;
    hasLeftSnapshot = hasLeftJoystick_;
  }

  noitom_hi5_hand_udp_python::JoySticks rightSnapshot;
  std::chrono::steady_clock::time_point rightRecvTime{};
  bool hasRightSnapshot = false;
  {
    std::lock_guard<std::mutex> lock(joystickRightMutex_);
    rightSnapshot = latestRightJoystick_;
    rightRecvTime = rightJoystickRecvTime_;
    hasRightSnapshot = hasRightJoystick_;
  }

  // 按侧超时只 neutral 自己那一侧；任一侧超时都不影响另一侧。
  // 左右统一使用同一次 now，避免两侧用不同判定时刻导致的一致性问题。
  const std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
  const bool leftStale = isJoystickSideStale(leftRecvTime, hasLeftSnapshot, now);
  const bool rightStale = isJoystickSideStale(rightRecvTime, hasRightSnapshot, now);

  noitom_hi5_hand_udp_python::JoySticks merged;
  if (!leftStale) {
    merged.left_x = leftSnapshot.left_x;
    merged.left_y = leftSnapshot.left_y;
    merged.left_trigger = leftSnapshot.left_trigger;
    merged.left_grip = leftSnapshot.left_grip;
    merged.left_first_button_pressed = leftSnapshot.left_first_button_pressed;
    merged.left_second_button_pressed = leftSnapshot.left_second_button_pressed;
    merged.left_first_button_touched = leftSnapshot.left_first_button_touched;
    merged.left_second_button_touched = leftSnapshot.left_second_button_touched;
  }
  if (!rightStale) {
    merged.right_x = rightSnapshot.right_x;
    merged.right_y = rightSnapshot.right_y;
    merged.right_trigger = rightSnapshot.right_trigger;
    merged.right_grip = rightSnapshot.right_grip;
    merged.right_first_button_pressed = rightSnapshot.right_first_button_pressed;
    merged.right_second_button_pressed = rightSnapshot.right_second_button_pressed;
    merged.right_first_button_touched = rightSnapshot.right_first_button_touched;
    merged.right_second_button_touched = rightSnapshot.right_second_button_touched;
  }

  noitom_hi5_hand_udp_python::JoySticks::ConstPtr mergedPtr =
      boost::make_shared<noitom_hi5_hand_udp_python::JoySticks>(merged);

  // dual_vr 下两个 monitor 只发布 /vr_left|right/quest_joystick_data，
  // 全局 /quest_joystick_data 没有别的发布者，而 humanoid_quest_control_with_arm
  // (QuestControlFSMNode) 等下游仍然订阅它。这里由本节点恢复唯一的 merged 发布。
  // 本函数只可能由 dual_vr 的按侧回调调用，single_vr 路径完全不受影响；
  // 同时 dual_vr 下本节点只订阅按侧话题，不会订阅自己发布的 /quest_joystick_data，
  // 不存在 self-loop。
  questJoystickDataPublisher_.publish(mergedPtr);

  {
    std::lock_guard<std::mutex> lock(joystickMutex_);
    if (joyStickHandlerPtr_) {
      joyStickHandlerPtr_->updateJoyStickData(mergedPtr);
    }
  }
  {
    // dual_vr：两个按侧实例都喂 merge 后的完整摇杆帧。
    // 此时 left_* 来自 Quest-A、right_* 来自 Quest-B，都是真实值，
    // 因此 isJoyRunningPressed()/isJoyStopPressed() 的双侧 AND 语义与 single_vr 完全一致。
    // 关键点是绝不把某一侧 monitor 的默认 0 当成对侧真实值。
    // 与 JoyStickHandler 分开加锁（joystickMutex_ 只保护 handler 状态），
    // 两把锁顺序持有，不产生嵌套。
    std::lock_guard<std::mutex> lock(transformerDataMutex_);
    if (quest3ArmInfoTransformerLeftPtr_) {
      quest3ArmInfoTransformerLeftPtr_->updateJoystickData(merged.left_trigger, merged.left_grip,
                                                           merged.right_trigger, merged.right_grip);
    }
    if (quest3ArmInfoTransformerRightPtr_) {
      quest3ArmInfoTransformerRightPtr_->updateJoystickData(merged.left_trigger, merged.left_grip,
                                                            merged.right_trigger, merged.right_grip);
    }
  }

  if (merged.left_trigger > 0.8 && merged.right_trigger > 0.8) {
    gripHoldCount_ = gripHoldCount_ > 1000 ? 1000 : gripHoldCount_ + 1;
  }
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

  // 主实例（single_vr 唯一使用；dual_vr 下仍保留，供不区分侧的既有代码路径使用）
  applyArmInfoTransformerJsonConfig(configJson, *quest3ArmInfoTransformerPtr_);

  // dual_vr：两个按侧实例必须用完全相同的机械参数初始化，否则左右臂解算尺度不一致
  if (quest3ArmInfoTransformerLeftPtr_) {
    applyArmInfoTransformerJsonConfig(configJson, *quest3ArmInfoTransformerLeftPtr_);
  }
  if (quest3ArmInfoTransformerRightPtr_) {
    applyArmInfoTransformerJsonConfig(configJson, *quest3ArmInfoTransformerRightPtr_);
  }

  ROS_INFO("🎯 [WheelArmControlBaseROS] Quest3ArmInfoTransformer initialization completed");
  ROS_INFO("==================================================================================");
}

void WheelArmControlBaseROS::applyArmInfoTransformerJsonConfig(const nlohmann::json& configJson,
                                                               Quest3ArmInfoTransformer& transformer) {
  processJsonParameter<double>(
      configJson, "upper_arm_length", [&transformer](double value) { transformer.updateUpperArmLength(value); });

  processJsonParameter<double>(
      configJson, "lower_arm_length", [&transformer](double value) { transformer.updateLowerArmLength(value); });

  // 初始化基座高度偏移
  processJsonParameter<double>(
      configJson, "base_height_offset", [&transformer](double value) { transformer.updateBaseHeightOffset(value); });

  // 初始化胸部X轴偏移
  processJsonParameter<double>(configJson, "base_chest_offset_x",
                               [&transformer](double value) { transformer.updateBaseChestOffsetX(value); });

  // 初始化肩宽参数
  processJsonParameter<double>(configJson, "shoulder_width",
                               [&transformer](double value) { transformer.updateShoulderWidth(value); });
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
  if (!enableTimestampLog_) return;

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

  // 创建当前用户 logs 目录（如果不存在）
  const char* homeDir = getenv("HOME");
  if (!homeDir) homeDir = "/tmp";
  char logDir[256];
  snprintf(logDir, sizeof(logDir), "%s/logs", homeDir);
  {
    char mkdirCmd[300];
    snprintf(mkdirCmd, sizeof(mkdirCmd), "mkdir -p %s", logDir);
    system(mkdirCmd);
  }

  char filename[512];
  snprintf(filename,
           sizeof(filename),
           "%s/timestamp_log_%04d%02d%02d_%02d%02d%02d.csv",
           logDir,
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
  if (!enableTimestampLog_) return;

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
