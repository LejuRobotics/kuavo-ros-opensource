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

#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <kuavo_msgs/lejuClawCommand.h>
#include <kuavo_msgs/robotHandPosition.h>
#include <kuavo_msgs/sensorsData.h>
#include <kuavo_msgs/headBodyPose.h>

#include <noitom_hi5_hand_udp_python/JoySticks.h>
#include <noitom_hi5_hand_udp_python/PoseInfoList.h>

namespace HighlyDynamic {

class WheelJoyStickHandler;
class Quest3ArmInfoTransformer;
class WheelKeyFramesVisualizer;

class WheelArmControlBaseROS {
 public:
  explicit WheelArmControlBaseROS(ros::NodeHandle& nodeHandle, double publishRate, bool debugPrint = false);

  virtual ~WheelArmControlBaseROS();

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

  //[CZJ]TODO: 确保这些服务在不同子类中被正确初始化，调用
  ros::ServiceClient changeMobileCtrlModeClient_;
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

  // Robot joint dimension parameters (loaded from JSON)
  // Sensor data structure: [Leg joints] + [Arm joints] + [Head joints]
  //
  // Example configurations:
  //   s49: [0-11: Legs(12)] + [12-25: Arms(14)] + [26-27: Head(2)] = 28 joints total
  //   s60: [0-3:  Legs(4)]  + [4-17:  Arms(14)] + [18-19: Head(2)] = 20 joints total
  //
  int gripHoldCount_ = 0;
  int numArmJoints_;         // Number of arm joints (从 JSON 中的 NUM_ARM_JOINT 加载，默认14)
  int numHeadJoints_;        // Number of head joints (从 JSON 中的 NUM_HEAD_JOINT 加载，默认2)
  int numWaistJoints_;       // Number of waist joints (从 JSON 中的 NUM_WAIST_JOINT 加载，默认0)
  int numTotalJoints_;       // Total number of joints (从 JSON 中的 NUM_JOINT 加载，s49默认28，s60默认20)
  int sensorDataArmOffset_;  // Offset to arm joints in sensor data = NUM_JOINT - NUM_HEAD_JOINT - NUM_ARM_JOINT
                             // 即腿部关节数量 (s49为12，s60为4)

  std::mutex sensorDataRawMutex_;
  std::shared_ptr<kuavo_msgs::sensorsData> sensorDataRaw_;

  std::mutex bonePosesMutex_;
  std::mutex joystickMutex_;
  std::shared_ptr<noitom_hi5_hand_udp_python::PoseInfoList> latestBonePosesPtr_;
  std::unique_ptr<WheelJoyStickHandler> joyStickHandlerPtr_;
  std::unique_ptr<Quest3ArmInfoTransformer> quest3ArmInfoTransformerPtr_;
  std::unique_ptr<WheelKeyFramesVisualizer> quest3KeyFramesVisualizerPtr_;
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
      ROS_INFO("[WheelArmControlBaseROS] Service call succeeded: %s", res.message.c_str());
    } else {
      ROS_WARN("[WheelArmControlBaseROS] Service call failed: %s", res.message.c_str());
    }

    return success;
  }

  void publishEndEffectorControlData();
  void publishHandPositionData();
  void publishClawCommandData();

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
  WheelArmControlBaseROS(const WheelArmControlBaseROS&) = delete;
  WheelArmControlBaseROS& operator=(const WheelArmControlBaseROS&) = delete;

  template <typename T, typename UpdateFunc>
  void processJsonParameter(const nlohmann::json& configJson, const std::string& paramName, UpdateFunc updateFunction);
};

}  // namespace HighlyDynamic
