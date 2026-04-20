#pragma once

#include <ros/ros.h>

#include <Eigen/Dense>
#include <atomic>
#include <memory>
#include <mutex>
#include <chrono>
#include <vector>
#include <string>

#include "motion_capture_ik/json.hpp"

#include <leju_utils/define.hpp>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <kuavo_msgs/lejuClawCommand.h>
#include <kuavo_msgs/robotHandPosition.h>
#include <kuavo_msgs/robotWaistControl.h>
#include <kuavo_msgs/sensorsData.h>
#include <kuavo_msgs/headBodyPose.h>

#include <noitom_hi5_hand_udp_python/JoySticks.h>
#include <noitom_hi5_hand_udp_python/PoseInfoList.h>

namespace HighlyDynamic {

class JoyStickHandler;
class Quest3ArmInfoTransformer;
class KeyFramesVisualizer;

class ArmControlBaseROS {
 public:
  explicit ArmControlBaseROS(ros::NodeHandle& nodeHandle, double publishRate, bool debugPrint = false);

  virtual ~ArmControlBaseROS();

  virtual void initializeBase(const nlohmann::json& configJson);

  virtual void initialize(const nlohmann::json& configJson) = 0;
  virtual void run() = 0;

  bool isRunning() const;

  bool wasRunning() const;

  bool shouldStop() const;

  std::shared_ptr<kuavo_msgs::sensorsData> getSensorData() const;

  virtual void activateController();
  virtual void deactivateController();

 protected:
  // ROS node handle
  ros::NodeHandle& nodeHandle_;

  // Service clients and servers
  ros::ServiceClient changeArmCtrlModeClient_;
  ros::ServiceServer setArmModeChangingServer_;
  ros::ServiceClient changeArmModeClient_;

  ros::ServiceClient humanoidArmCtrlModeClient_;
  ros::ServiceClient enableWbcArmTrajectoryControlClient_;

  // Basic subscribers
  ros::Subscriber stopRobotSubscriber_;
  ros::Subscriber sensorsDataRawSubscriber_;
  ros::Subscriber armModeSubscriber_;
  ros::Subscriber bonePosesSubscriber_;
  ros::Subscriber joystickSubscriber_;

  // End effector control publishers
  ros::Publisher robotHandPositionPublisher_;
  ros::Publisher lejuClawCommandPublisher_;
  ros::Publisher headBodyPosePublisher_;
  ros::Publisher kuavoArmTrajCppPublisher_;
  ros::Publisher questJoystickDataPublisher_;
  ros::Publisher waistMotionPublisher_;
  ros::Publisher wholeTorsoCtrlPublisher_;
  ros::ServiceClient vrWaistControlServiceClient_;

  // Atomic state variables for thread-safe operation
  std::atomic<bool> shouldStop_;
  std::atomic<bool> onlyHalfUpBody_;
  std::atomic<bool> armModeChanging_;
  std::atomic<bool> isRunning_;
  std::atomic<bool> isRunningLast_;
  std::atomic<bool> controllerActivated_;

  // Configuration parameters
  const double publishRate_;
  const bool debugPrint_;

  // Control parameters
  double maxSpeed_;
  double thresholdArmDiffHalfUpBody_rad_;
  bool controlTorso_;
  bool enableWbcArmTrajectory_;
  bool torsoControlEnabled_ = false;
  bool prevRightSecondButtonPressed_ = false;
  int robotType_ = 2;
  double waistYawMaxAngleDeg_ = 0.0;
  HeadBodyPose currentHeadBodyPose_;
  double torsoPitchZero_ = 0.0;
  double torsoYawZero_ = 0.0;
  double bodyHeightZero_ = 0.0;
  double bodyXZero_ = 0.0;
  double lastBodyYaw_ = 0.0;
  double accumulatedYawOffset_ = 0.0;

  // Robot joint dimension parameters (loaded from JSON with fallback compatibility)
  // Sensor data structure: [Leg joints] + [Arm joints] + [Head joints]
  //
  // Compatibility strategy:
  //   - Priority: Read from JSON (NUM_ARM_JOINT, NUM_HEAD_JOINT, NUM_WAIST_JOINT, NUM_JOINT)
  //   - Fallback: If any critical field is missing or invalid, use legacy defaults:
  //     arm=14, head=2, waist=0, total=28 (v49 layout)
  //   - This ensures backward compatibility with old configurations while supporting new layouts
  //
  // Example configurations:
  //   v49: [0-11: Legs(12)] + [12-25: Arms(14)] + [26-27: Head(2)] = 28 joints total, offset=12
  //   v60: [0-3:  Legs(4)]  + [4-17:  Arms(14)] + [18-19: Head(2)] = 20 joints total, offset=4
  //
  int gripHoldCount_ = 0;
  int numArmJoints_;         // Number of arm joints (从 JSON 中的 NUM_ARM_JOINT 加载，默认14)
  int numHeadJoints_;        // Number of head joints (从 JSON 中的 NUM_HEAD_JOINT 加载，默认2)
  int numWaistJoints_;       // Number of waist joints (从 JSON 中的 NUM_WAIST_JOINT 加载，默认0)
  int numTotalJoints_;       // Total number of joints (从 JSON 中的 NUM_JOINT 加载，s49默认28，s60默认20)
  int sensorDataArmOffset_;  // Offset to arm joints in sensor data = NUM_JOINT - NUM_HEAD_JOINT - NUM_ARM_JOINT
                             // 即腿部关节数量 (v49为12，v60为4)
                             // All joint_q array accesses MUST use: sensorDataArmOffset_ + i for arm joint index i

  std::mutex sensorDataRawMutex_;
  std::shared_ptr<kuavo_msgs::sensorsData> sensorDataRaw_;

  std::mutex bonePosesMutex_;
  std::mutex joystickMutex_;
  std::mutex transformerDataMutex_;
  std::shared_ptr<noitom_hi5_hand_udp_python::PoseInfoList> latestBonePosesPtr_;
  std::unique_ptr<JoyStickHandler> joyStickHandlerPtr_;
  std::unique_ptr<Quest3ArmInfoTransformer> quest3ArmInfoTransformerPtr_;
  std::unique_ptr<KeyFramesVisualizer> quest3KeyFramesVisualizerPtr_;
  std::shared_ptr<noitom_hi5_hand_udp_python::PoseInfoList> HandPoseAndElbowPositonListPtr_;
  int loopSyncCount_ = 0;

  // 时间戳记录系统
  struct TimestampRecord {
    std::string stepName;
    uint64_t timestamp;  // 微秒级时间戳
    int64_t loopCount;   // 循环计数器
  };
  std::vector<TimestampRecord> timestampRecords_;
  std::mutex timestampMutex_;
  std::chrono::steady_clock::time_point lastSaveTime_;
  const int64_t saveIntervalSeconds_ = 20;  // 每20秒保存一次

  void recordTimestamp(const std::string& stepName, int64_t loopCount = -1);
  void saveTimestampRecordsToFile();
  void checkAndSaveTimestampRecords();

  void stopRobotCallback(const std_msgs::Bool::ConstPtr& msg);

  void sensorDataRawCallback(const kuavo_msgs::sensorsData::ConstPtr& msg);

  virtual void armModeCallback(const std_msgs::Int32::ConstPtr& msg);

  void bonePosesCallback(const noitom_hi5_hand_udp_python::PoseInfoList::ConstPtr& msg);
  void joystickCallback(const noitom_hi5_hand_udp_python::JoySticks::ConstPtr& msg);

  virtual void processBonePoses(const noitom_hi5_hand_udp_python::PoseInfoList::ConstPtr& msg);
  virtual bool setArmModeChangingCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  bool changeArmCtrlMode(int mode);

  bool initializeArmControlMode();

  bool initializeArmJointsSafety();

  virtual void loadParameters();

  virtual void fsmEnter() {}
  virtual void fsmChange() {}
  virtual void fsmProcess() {}
  virtual void fsmExit() {}

  void updateRunningState();

  template <typename ResponseType>
  bool handleServiceResponse(ResponseType& res, bool success, const std::string& message = "") {
    res.success = success;

    if (!message.empty()) {
      res.message = message;
    } else {
      res.message = success ? "Operation completed successfully" : "Operation failed";
    }

    // 记录日志
    if (success) {
      ROS_INFO("[ArmControlBaseROS] Service call succeeded: %s", res.message.c_str());
    } else {
      ROS_WARN("[ArmControlBaseROS] Service call failed: %s", res.message.c_str());
    }

    return success;
  }

  void publishEndEffectorControlData();
  void publishHandPositionData();
  void publishClawCommandData();
  void initializeTorsoControlFromReference();
  void callVRWaistControlSrv(bool enable);
  void controlWaist(double waistYaw);
  void updateTorsoControl();
  void handleTorsoControlJoystick();
  void processAbsoluteTorsoControl(const HeadBodyPose& headBodyPose);
  void processTorsoControlLoop();

  // 发布 quest_joystick_data 消息（x轴和A按钮为true，其余全零）
  void publishQuestJoystickDataXAndA();

  void initializeArmInfoTransformerFromJson(const nlohmann::json& configJson);

  std::vector<std::string> loadFrameNamesFromConfig(const nlohmann::json& configJson);

  // 可视化相关方法
  void initializeKeyFramesVisualizer();
  void publishVisualizationMarkersForSide(const std::string& side,
                                          const Eigen::Vector3d& handPos,
                                          const Eigen::Vector3d& elbowPos,
                                          const Eigen::Vector3d& shoulderPos,
                                          const Eigen::Vector3d& chestPos);

 private:
  ArmControlBaseROS(const ArmControlBaseROS&) = delete;
  ArmControlBaseROS& operator=(const ArmControlBaseROS&) = delete;

  template <typename T, typename UpdateFunc>
  void processJsonParameter(const nlohmann::json& configJson, const std::string& paramName, UpdateFunc updateFunction);

  // Dimension compatibility helpers
  void loadJointDimensionsWithFallback(const nlohmann::json& configJson);
  bool validateJointDimensions(int& numArm, int& numHead, int& numWaist, int& numTotal, int& offset) const;
};

}  // namespace HighlyDynamic
