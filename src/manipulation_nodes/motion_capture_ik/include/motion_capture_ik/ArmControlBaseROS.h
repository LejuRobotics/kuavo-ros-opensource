#pragma once

#include <ros/ros.h>

#include <Eigen/Dense>
#include <atomic>
#include <memory>
#include <mutex>

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
  int waist_dof_;  // 腰部自由度数量（从JSON配置读取NUM_WAIST_JOINT）

  std::mutex sensorDataRawMutex_;
  std::shared_ptr<kuavo_msgs::sensorsData> sensorDataRaw_;

  std::mutex bonePosesMutex_;
  std::mutex joystickMutex_;
  std::shared_ptr<noitom_hi5_hand_udp_python::PoseInfoList> latestBonePosesPtr_;
  std::unique_ptr<JoyStickHandler> joyStickHandlerPtr_;
  std::unique_ptr<Quest3ArmInfoTransformer> quest3ArmInfoTransformerPtr_;
  std::unique_ptr<KeyFramesVisualizer> quest3KeyFramesVisualizerPtr_;
  std::shared_ptr<noitom_hi5_hand_udp_python::PoseInfoList> HandPoseAndElbowPositonListPtr_;

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
      ROS_INFO("[ArmControlBaseROS] Service call succeeded: %s", res.message.c_str());
    } else {
      ROS_WARN("[ArmControlBaseROS] Service call failed: %s", res.message.c_str());
    }

    return success;
  }

  void publishEndEffectorControlData();
  void publishHandPositionData();
  void publishClawCommandData();

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
};

}  // namespace HighlyDynamic
