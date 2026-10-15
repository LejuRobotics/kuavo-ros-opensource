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
#include <std_msgs/Float64.h>
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

  // 是否处于 dual_vr 输入模式（input_mode=dual_vr）
  bool isDualVrMode() const { return useDualVr_; }

  // 取本侧 Transformer：single_vr 下两侧都返回主实例 quest3ArmInfoTransformerPtr_，
  // 行为与改造前完全一致；dual_vr 下分别返回独立的 Left / Right 实例。
  Quest3ArmInfoTransformer* getArmInfoTransformerForSide(const std::string& side) const;

  // dual_vr：在 transformerDataMutex_ 内一次性完成"本侧骨骼数据新鲜度判定 + 本侧
  // Chest 快照"，保证判定用的接收时刻与读到的 Chest 来自同一个临界区（同一帧口径）。
  // 返回 false 表示：本侧 Transformer 不存在 / 本侧从未收到过骨骼帧 /
  // 距最近一帧已超过 staleTimeoutMs / 本侧尚无有效 Chest 帧。
  // chestQuatOut 非空时一并回传同一快照下的 Chest 朝向（与 position 严格同帧）。
  bool getFreshSideChestPose(const std::string& side,
                             int64_t staleTimeoutMs,
                             Eigen::Vector3d& chestPos,
                             Eigen::Quaterniond* chestQuatOut = nullptr);

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
  ros::Publisher commLatencyPublisher_;  // /vr_incremental/comm_latency_ms，VR节点→IK节点的通信延迟(ms)

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
  std::mutex transformerDataMutex_;
  // 跨线程延迟测量：记录骨骼数据到达回调的时刻，随数据传播到IK线程和发布线程
  std::mutex boneRecvTimeMutex_;
  std::chrono::steady_clock::time_point boneRecvTime_;
  // 骨骼数据序列号：每次回调递增，IK线程用它判断是否有新数据到达
  uint64_t boneDataSeq_ = 0;
  // VR端时间戳（Unix毫秒），来自PoseInfoList.timestamp_ms，用于计算VR节点→IK节点的通信延迟
  int64_t boneVrTimestampMs_ = 0;
  std::shared_ptr<noitom_hi5_hand_udp_python::PoseInfoList> latestBonePosesPtr_;
  std::unique_ptr<WheelJoyStickHandler> joyStickHandlerPtr_;
  std::unique_ptr<Quest3ArmInfoTransformer> quest3ArmInfoTransformerPtr_;
  // Dual VR（input_mode=dual_vr）：按侧独立 Transformer 实例。
  // Quest-A 的完整骨骼帧只进入 left，Quest-B 的只进入 right，两个实例各自持有
  // chestPosition_ / qInitChest_ / yawOnlyQuat_ / *HandPoseRelativeToChest_ 等有状态量，
  // 避免单实例 update A 再 update B 造成跨侧污染。
  // single_vr 下保持为空，全部逻辑仍走 quest3ArmInfoTransformerPtr_。
  std::unique_ptr<Quest3ArmInfoTransformer> quest3ArmInfoTransformerLeftPtr_;
  std::unique_ptr<Quest3ArmInfoTransformer> quest3ArmInfoTransformerRightPtr_;
  // 按侧 bone 话题订阅（仅 dual_vr 下创建）
  ros::Subscriber bonePosesLeftSubscriber_;
  ros::Subscriber bonePosesRightSubscriber_;
  // 按侧 joystick 话题订阅（仅 dual_vr 下创建，替代订阅自身发布的 /quest_joystick_data）
  ros::Subscriber joystickLeftSubscriber_;
  ros::Subscriber joystickRightSubscriber_;
  std::mutex joystickLeftMutex_;
  std::mutex joystickRightMutex_;
  // dual_vr：updateMergedJoystick() 整函数串行化。左右 joystick 回调可能并发进入
  // merge，导致旧快照晚发布；此锁保证"取快照→判超时→组装→发布→下发"整体原子。
  // 位于锁序最外层，只与 joystickLeftMutex_/joystickRightMutex_/joystickMutex_/
  // transformerDataMutex_ 分区顺序持有，不与之嵌套。
  std::mutex joystickMergeMutex_;
  // dual_vr：按侧骨骼数据接收时刻，由各侧 bone 回调在 transformerDataMutex_ 内更新。
  // 与 transformer 的位姿成员共用同一把锁，保证"快照 + 新鲜度"来自同一个临界区。
  // 默认值（epoch）表示本侧从未收到过骨骼帧，时间差必然远超阈值。
  std::chrono::steady_clock::time_point leftBoneRecvTime_{};
  std::chrono::steady_clock::time_point rightBoneRecvTime_{};
  // 最近一次收到的按侧 joystick 帧（dual_vr 下用于 merge）
  noitom_hi5_hand_udp_python::JoySticks latestLeftJoystick_;
  noitom_hi5_hand_udp_python::JoySticks latestRightJoystick_;
  std::chrono::steady_clock::time_point leftJoystickRecvTime_;
  std::chrono::steady_clock::time_point rightJoystickRecvTime_;
  bool hasLeftJoystick_ = false;
  bool hasRightJoystick_ = false;
  // input_mode: single_vr（默认）/ dual_vr
  std::string inputMode_{"single_vr"};
  bool useDualVr_{false};
  // dual_vr 按侧 joystick 超时阈值（与 Absolute Dual VR 保持一致：0.5s）
  static constexpr int64_t kJoystickStaleTimeoutMs = 500;
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
  bool enableTimestampLog_{false};           // 是否启用时间戳日志（由 ROS param enable_timestamp_log 控制）

  void recordTimestamp(const std::string& stepName, int64_t loopCount = -1);
  void saveTimestampRecordsToFile();
  void checkAndSaveTimestampRecords();

  void stopRobotCallback(const std_msgs::Bool::ConstPtr& msg);

  void sensorDataRawCallback(const kuavo_msgs::sensorsData::ConstPtr& msg);

  virtual void armModeCallback(const std_msgs::Int32::ConstPtr& msg);

  void bonePosesCallback(const noitom_hi5_hand_udp_python::PoseInfoList::ConstPtr& msg);
  void joystickCallback(const noitom_hi5_hand_udp_python::JoySticks::ConstPtr& msg);

  // Dual VR（input_mode=dual_vr）按侧回调；single_vr 下不订阅、不会被调用
  void bonePosesLeftCallback(const noitom_hi5_hand_udp_python::PoseInfoList::ConstPtr& msg);
  void bonePosesRightCallback(const noitom_hi5_hand_udp_python::PoseInfoList::ConstPtr& msg);
  void joystickLeftCallback(const noitom_hi5_hand_udp_python::JoySticks::ConstPtr& msg);
  void joystickRightCallback(const noitom_hi5_hand_udp_python::JoySticks::ConstPtr& msg);
  // dual_vr：把按侧缓存 merge 成一份完整 JoySticks 后喂给 JoyStickHandler / Transformer
  void updateMergedJoystick();
  // dual_vr：判断本侧是否已超时（超时只 neutral 自己那一侧）。
  // now 由调用方统一传入：左右两侧必须用同一个判定时刻，且该时刻与快照读取时刻无关。
  bool isJoystickSideStale(const std::chrono::steady_clock::time_point& recvTime,
                           bool hasData,
                           const std::chrono::steady_clock::time_point& now) const;
  // dual_vr：按侧实例只收到自己那一侧的数据，而 ArmLengthMeasurement 的自动完成条件
  // 要求同一实例内左右样本都 >= 30，按侧实例因此永远不会自动完成；
  // 这里在按侧回调（与样本写入同一线程）内，用本侧样本数显式完成本侧标定。
  void completeSideArmLengthMeasurementIfReady(Quest3ArmInfoTransformer* transformer, const std::string& side);

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

  // 把 JSON 中的机械参数应用到指定 Transformer 实例（dual_vr 下两个按侧实例都要应用）
  void applyArmInfoTransformerJsonConfig(const nlohmann::json& configJson, Quest3ArmInfoTransformer& transformer);

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
