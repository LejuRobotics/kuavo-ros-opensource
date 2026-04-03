#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include "motion_capture_ik/json.hpp"

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>

#include <noitom_hi5_hand_udp_python/PoseInfoList.h>
#include <leju_utils/define.hpp>
#include "motion_capture_ik/WheelArmControlBaseROS.h"
#include "motion_capture_ik/WheelOneStageIKEndEffector.h"
#include "motion_capture_ik/WheelIncrementalControlModule.h"
#include "motion_capture_ik/WheelHandSmoother.h"
#include "humanoid_wheel_interface/filters/KinemicLimitFilter.h"
#include "DrakeChestElbowHandPointOpt.hpp"

namespace HighlyDynamic {

class WheelQuest3IkIncrementalROS final : public WheelArmControlBaseROS {
 public:
  explicit WheelQuest3IkIncrementalROS(ros::NodeHandle& nodeHandle,
                                  double publishRate,
                                  bool debugPrint = false,
                                  ArmIdx ctrlArmIdx = ArmIdx::BOTH);

  ~WheelQuest3IkIncrementalROS();

  void initialize(const nlohmann::json& configJson) override;
  void run() override;

 protected:
  void activateController() override;
  void deactivateController() override;

  void armModeCallback(const std_msgs::Int32::ConstPtr& msg) override;
  std::atomic<int> armControlMode_ = 0;
  std::atomic<int> lastArmControlMode_ = 0;

  // 超时机制相关
  ros::Time mode2EnterTime_;                              // 记录进入 mode 2 的时间戳
  std::mutex mode2EnterTimeMutex_;                        // 保护时间戳的互斥锁
  static constexpr double MODE_2_TIMEOUT_DURATION = 2.0;  // 超时时间：5秒

  void fsmEnter() override;
  void fsmChange() override;
  void fsmProcess() override;
  void fsmExit() override;

  WheelPointTrackIKSolverConfig loadPointTrackIKSolverConfigFromJson(const nlohmann::json& configJson);
  DrakeChestElbowHandWeightConfig loadDrakeChestElbowHandWeightsFromJson(const nlohmann::json& configJson);
  DrakeChestElbowHandBoundsConfig loadDrakeChestElbowHandBoundsFromJson(const nlohmann::json& configJson);
  DrakeChestElbowHandPrevFilterConfig loadDrakeChestElbowHandPrevFilterConfigFromJson(const nlohmann::json& configJson);

 private:
  void chestPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

  void solveIkHandElbowThreadFunction();
  void publishJointStatesThreadFunction();
  void updateFkCacheFromSensorData();

  // 从 sensorData 抽取 14 维双臂关节角（rad），并做指数均值滤波：q = 0.99*q + 0.01*qnew
  void updateSensorArmJointMeanFromSensorData();
  void updateSensorArmJointFromSensorData();

  // FK 辅助函数：计算左手末端执行器姿态
  void computeLeftEndEffectorFK(Eigen::Vector3d& pOut, Eigen::Quaterniond& qOut);
  // FK 辅助函数：计算右手末端执行器姿态
  void computeRightEndEffectorFK(Eigen::Vector3d& pOut, Eigen::Quaterniond& qOut);
  // FK 辅助函数：计算左右臂 link4 姿态（仅用于透传到增量控制模块）
  void computeLeftLink4FK(Eigen::Vector3d& pOut, Eigen::Quaterniond& qOut);
  void computeRightLink4FK(Eigen::Vector3d& pOut, Eigen::Quaterniond& qOut);
  void computeLeftLink6FK(Eigen::Vector3d& pOut, Eigen::Quaterniond& qOut);
  void computeRightLink6FK(Eigen::Vector3d& pOut, Eigen::Quaterniond& qOut);
  // FK 辅助函数：计算左右臂 shoulder 姿态
  void computeLeftShoulderFK(Eigen::Vector3d& pOut, Eigen::Quaterniond& qOut);
  void computeRightShoulderFK(Eigen::Vector3d& pOut, Eigen::Quaterniond& qOut);
  // FK 辅助函数：计算腰部 yaw 参考点位置（用于胸部增量 anchor）
  void computeWaistYawFK(Eigen::Vector3d& pOut);
  // 启发式计算肘部参考位置
  Eigen::Vector3d computeElbow(const Eigen::Vector3d& link6Pos,
                               const Eigen::Vector3d& endEffectorPos,
                               const double link2Length);

  // 初始化关节过滤器
  void initializeFilter(std::unique_ptr<ocs2::mobile_manipulator::KinemicLimitFilter>& filterPtr,
                        int dimension,
                        double dt,
                        double velLimit,
                        double accLimit,
                        double jerkLimit,
                        const std::string& filterName = "",
                        const Eigen::VectorXd* initialState = nullptr);

  // Grip 上升沿事件处理：更新锚点，使增量归零（避免 grip 切换瞬间位置跳变）
  void handleGripRisingEdge(bool leftGripRisingEdge,
                            bool rightGripRisingEdge,
                            bool leftMaintainProcess,
                            bool rightMaintainProcess);

  void remapUpperBodyRefPoints(const Eigen::Vector3d& chestPos,
                               const Eigen::Quaterniond& chestQuat,
                               const Eigen::Vector3d& leftShoulderPos,
                               const Eigen::Vector3d& rightShoulderPos,
                               Eigen::Vector3d& leftHandRef,
                               Eigen::Vector3d& rightHandRef,
                               Eigen::Vector3d& leftElbowRef,
                               Eigen::Vector3d& rightElbowRef);

  struct WholeBodyRefInput {
    bool leftRefActive = false;
    bool rightRefActive = false;

    Eigen::Vector3d chestPosRef = Eigen::Vector3d::Zero();
    Eigen::Quaterniond chestQuatRef = Eigen::Quaterniond::Identity();  // yaw/pitch-only

    Eigen::Vector3d leftElbowRef = Eigen::Vector3d::Zero();
    Eigen::Vector3d leftHandRef = Eigen::Vector3d::Zero();
    Eigen::Quaterniond leftHandQuat = Eigen::Quaterniond::Identity();

    Eigen::Vector3d rightElbowRef = Eigen::Vector3d::Zero();
    Eigen::Vector3d rightHandRef = Eigen::Vector3d::Zero();
    Eigen::Quaterniond rightHandQuat = Eigen::Quaterniond::Identity();
  };

  static Eigen::Quaterniond computeYawPitchOnlyQuatFromRotationMatrix(const Eigen::Matrix3d& rotationMatrix);
  bool updateWholeBodyConstraintList(const WholeBodyRefInput& input);

  void solveIk();

  bool detectLeftArmMove();
  bool detectRightArmMove();

  bool updateLatestIncrementalResult();

  void updateLeftConstraintList(const Eigen::Vector3d& leftHandPos,
                                const Eigen::Quaterniond& leftHandQuat,
                                const Eigen::Vector3d& leftElbowPos);
  void updateRightConstraintList(const Eigen::Vector3d& rightHandPos,
                                 const Eigen::Quaterniond& rightHandQuat,
                                 const Eigen::Vector3d& rightElbowPos);
  bool updateLeftHandChangingMode(const Eigen::Vector3d& leftTargetPos);
  bool updateRightHandChangingMode(const Eigen::Vector3d& rightTargetPos);

  // 发布函数
  void publishJointStates();
  void publishDefaultJointStates();
  void publishZeroJointStates();
  void publishLegJointStates();         // 发布下肢关节状态（基于 IK 解）
  void publishDefaultLegJointStates();  // 发布下肢关节默认状态
  void publishAuxiliaryStates();
  void publishWholeBodyRefMarkers();  // 发布全身优化参考点（base_link下可视化）

  void reset();                          // 重置所有运行时状态，确保进入系统时正常
  void forceDeactivateAllArmCtrlMode();  // 强制停用所有手臂控制模式
  void forceActivateAllArmCtrlMode();    // 强制激活所有手臂控制模式
  // 设置轮臂快速模式服务调用函数
  // 参数：quickMode - 快速模式类型：0-关闭, 1-下肢快, 2-上肢快, 3-上下肢快
  // 返回：成功返回true，失败返回false
  bool setLbArmQuickMode(int quickMode);
  // 设置移动机械臂控制模式服务调用函数（对应lb_ctrl_api.set_control_mode）
  // 参数：targetMode - 控制模式：0-NoControl, 1-ArmOnly, 2-BaseOnly, 3-BaseArm, 4-ArmEeOnly
  // 返回：成功返回true，失败返回false
  bool setControlMode(int targetMode);

  // ============================================================================
  // Service call de-duplication / retry throttling
  // ============================================================================
  // 约束：同一模式切换请求如果已成功，不应在高频循环中重复发送服务请求；若失败则按间隔重试，避免刷服务。
  static constexpr double CONTROL_MODE_RETRY_INTERVAL_SEC = 0.5;
  static constexpr double LB_QUICK_MODE_RETRY_INTERVAL_SEC = 0.5;
  static constexpr double DEFAULT_JOINT_STATE_PUBLISH_RATE_HZ = 500.0;
  static constexpr double DEFAULT_LB_LEG_PUBLISH_RATE_MULTIPLIER = 0.25;

  std::mutex controlModeRequestMutex_;
  int lastControlModeRequested_ = -1;
  ros::Time lastControlModeRequestTime_;
  bool lastControlModeRequestSuccess_ = false;

  std::mutex lbQuickModeRequestMutex_;
  int lastLbQuickModeRequested_ = -1;
  ros::Time lastLbQuickModeRequestTime_;
  bool lastLbQuickModeRequestSuccess_ = false;

  ros::Publisher kuavoArmTrajCppPublisher_;  // 发布kuavo_arm_traj_cpp；launch中通过remap话题方式来接入当前系统
  ros::Publisher sensorDataArmJointsPublisher_;      // 发布传感器数据的手臂关节角
  ros::Publisher leftHandPosePublisher_;             // 发布左手pose
  ros::Publisher rightHandPosePublisher_;            // 发布右手pose
  ros::Publisher leftHandRotationMatrixPublisher_;   // 发布左手旋转矩阵（9个浮点数数组）
  ros::Publisher rightHandRotationMatrixPublisher_;  // 发布右手旋转矩阵（9个浮点数数组）
  ros::Publisher leftAnchorQuatPublisher_;           // 发布左手anchor四元数
  ros::Publisher rightAnchorQuatPublisher_;          // 发布右手anchor四元数
  ros::Publisher leftHandDeltaQuatPublisher_;  // 发布左手delta四元数（人手相对旋转增量，世界坐标系）
  ros::Publisher rightHandDeltaQuatPublisher_;  // 发布右手delta四元数（人手相对旋转增量，世界坐标系）
  ros::Publisher leftEePosePublisher_;          // 发布左手末端执行器pose（基于IK结果的FK计算）
  ros::Publisher rightEePosePublisher_;         // 发布右手末端执行器pose（基于IK结果的FK计算）
  ros::Publisher leftEePoseFilterPublisher_;  // 发布左手末端执行器pose（基于滤波后关节角度的FK计算）
  ros::Publisher rightEePoseFilterPublisher_;  // 发布右手末端执行器pose（基于滤波后关节角度的FK计算）
  ros::Publisher leftLink6PosePublisher_;         // 发布左手link6 pose（基于IK结果的FK计算）
  ros::Publisher rightLink6PosePublisher_;        // 发布右手link6 pose（基于IK结果的FK计算）
  ros::Publisher leftLink6PoseFilterPublisher_;   // 发布左手link6 pose（基于滤波后关节角度的FK计算）
  ros::Publisher rightLink6PoseFilterPublisher_;  // 发布右手link6 pose（基于滤波后关节角度的FK计算）
  ros::Publisher leftLink6PoseMeasuredPublisher_;  // 发布左手link6 measured pose（基于传感器数据的FK计算）
  ros::Publisher rightLink6PoseMeasuredPublisher_;  // 发布右手link6 measured pose（基于传感器数据的FK计算）
  ros::Publisher leftEePoseMeasuredPublisher_;  // 发布左手末端执行器measured pose（基于传感器数据的FK计算）
  ros::Publisher rightEePoseMeasuredPublisher_;  // 发布右手末端执行器measured pose（基于传感器数据的FK计算）
  ros::Publisher leftHandPosBeforeOptPublisher_;          // 发布优化前的左手位置
  ros::Publisher leftHandPosAfterOptPublisher_;           // 发布优化后的左手位置
  ros::Publisher rightHandPosBeforeOptPublisher_;         // 发布优化前的右手位置
  ros::Publisher rightHandPosAfterOptPublisher_;          // 发布优化后的右手位置
  ros::Publisher leftHandPoseFromTransformerPublisher_;   // 发布来自Transformer的左手pose
  ros::Publisher rightHandPoseFromTransformerPublisher_;  // 发布来自Transformer的右手pose
  ros::Publisher lbLegTrajPublisher_;
  ros::Timer lbLegTrajPublishTimer_;
  std::mutex lbLegTargetMutex_;
  Eigen::VectorXd latestLbTargetAngles_;
  bool hasLatestLbTargetAngles_ = false;
  bool lbLegTrajPublishEnabled_ = false;
  ros::Publisher cmdVelPublisher_;  // 发布底盘速度控制命令
  ros::ServiceClient enableLbArmQuickModeClient_;
  ros::ServiceClient changeTorsoCtrlModeClient_;
  ros::Subscriber chestPoseSubscriber_;  // 订阅/robot_chest_pose

  ros::Publisher wholeBodyRefMarkerArrayPublisher_;  // 全身参考点可视化（MarkerArray）

  std::thread ikSolveThread_;
  std::thread jointStatePublishThread_;
  std::mutex bonePoseHandElbowMutex_;
  std::mutex poseConstraintListMutex_;  // 保护 latestPoseConstraintList_ 的互斥锁
  std::mutex ikResultMutex_;
  std::mutex jointStateMutex_;  // 保护关节状态与滤波缓存
  std::mutex oneStageIkMutex_;  // 保护 oneStageIkEndEffectorPtr_ 的线程访问
  double jointStatePublishRateHz_ = DEFAULT_JOINT_STATE_PUBLISH_RATE_HZ;
  double lbLegPublishRateMultiplier_ = DEFAULT_LB_LEG_PUBLISH_RATE_MULTIPLIER;
  Eigen::Quaterniond chestRotationQuaternion_ = Eigen::Quaterniond::Identity();
  std::mutex chestPoseMutex_;  // 保护最新 chest pose（position）
  Eigen::Vector3d latestChestPositionInRobot_ = Eigen::Vector3d::Zero();
  bool hasChestPose_ = false;
  Eigen::Vector3d frozenRobotChestPos_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond frozenChestQuat_ = Eigen::Quaterniond::Identity();
  double frozenLeftHandHeightOffset_ = 0.0;
  double frozenRightHandHeightOffset_ = 0.0;
  double frozenLeftElbowHeightOffset_ = 0.0;
  double frozenRightElbowHeightOffset_ = 0.0;

  int drakeJointStateSize_;
  ArmIdx ctrlArmIdx_;  // 控制哪个手臂：LEFT, RIGHT, 或 BOTH

  std::unique_ptr<WheelOneStageIKEndEffector> oneStageIkEndEffectorPtr_;
  std::unique_ptr<WheelIncrementalControlModule> incrementalController_;
  std::unique_ptr<WheelHandSmoother> leftHandSmoother_;
  std::unique_ptr<WheelHandSmoother> rightHandSmoother_;

  std::unique_ptr<DrakeChestElbowHandPointOptSolver> chestElbowHandPointOptSolverPtr_;
  DrakeChestElbowHandWeightConfig chestElbowHandWeightConfig_;
  DrakeChestElbowHandBoundsConfig chestElbowHandBoundsConfig_;
  DrakeChestElbowHandPrevFilterConfig chestElbowHandPrevFilterConfig_;

  // Drake diagram and context for plant
  std::unique_ptr<drake::systems::Diagram<double>> diagram_;
  std::unique_ptr<drake::systems::Context<double>> diagramContext_;

  // IK求解结果
  Eigen::VectorXd latestIkSolution_;
  bool hasValidIkSolution_ = false;
  Eigen::VectorXd ikLowerBodyJointCommand_;  // 保存IK结果的前4个关节角度（size = 4）
  Eigen::VectorXd ikUpperBodyJointCommand_;  // 保存IK结果的前4个关节角度的指数均值滤波状态（size = 14）

  // 姿态和位置偏置
  Eigen::Vector3d deltaScale_;  // delta_scale 参数（x, y, z三轴独立）

  // 机器人结构参数
  Eigen::Vector3d robotRightFixedShoulderPos_;                     // 右肩绝对位置
  Eigen::Vector3d robotLeftFixedShoulderPos_;                      // 左肩绝对位置
  Eigen::Vector3d robotFixedWaistYawPos_;                          // 腰关节yaw位置
  Eigen::Vector3d latestWaistYawFkPos_ = Eigen::Vector3d::Zero();  // 当前关节 FK 计算的腰部 yaw 位置
  bool hasLatestWaistYawFk_ = false;
  Eigen::Vector3d chestDefaultOffset_;                                  // 胸部默认位置相对腰部yaw的偏移量
  Eigen::Vector3d latestLeftShoulderFkPos_ = Eigen::Vector3d::Zero();   // 当前关节 FK 计算的左肩位置
  Eigen::Vector3d latestRightShoulderFkPos_ = Eigen::Vector3d::Zero();  // 当前关节 FK 计算的右肩位置
  bool hasLatestLeftShoulderFk_ = false;
  bool hasLatestRightShoulderFk_ = false;
  double l1_;  // 第一段连杆长度 ||j2_j4||
  double l2_;  // 第二段连杆长度 ||j4_j6||

  // 关节状态
  Eigen::VectorXd q_;           // 滤波后的关节角度（弧度）
  Eigen::VectorXd dq_;          // 滤波后的关节角速度（弧度/秒）
  Eigen::VectorXd latest_q_;    // 最新的关节角度（弧度）
  Eigen::VectorXd latest_dq_;   // 最新的关节角速度（弧度/秒）
  Eigen::VectorXd lowpass_dq_;  // 低通滤波后的关节角速度（弧度/秒）
  Eigen::VectorXd q_init_cmd_ = Eigen::VectorXd::Zero(14);  // mode2 切入时用于平滑过渡的起始关节角
  Eigen::VectorXd jointMidValues_;  // TEST: 关节限制中间值（用于测试），存储每个关节的(limit_lower+limit_upper)/2

  // 下肢(轮臂)关节状态：与 kuavo_arm_traj 同款平滑逻辑（ruckig + alpha指数平滑 + 速度低通）
  Eigen::VectorXd lb_q_;           // 下肢滤波后的关节角度（弧度，size=4）
  Eigen::VectorXd lb_dq_;          // 下肢滤波后的关节角速度（弧度/秒，size=4）
  Eigen::VectorXd latest_lb_q_;    // 下肢最新关节角度（弧度，size=4）
  Eigen::VectorXd latest_lb_dq_;   // 下肢最新关节角速度（弧度/秒，size=4）
  Eigen::VectorXd lowpass_lb_dq_;  // 下肢低通滤波后的关节角速度（弧度/秒，size=4）
  ros::Time lbLegMoveStartTime_;   // 下肢目标变化开始时间戳（用于 alpha 从 0.01->1.0 的超时插值）
  std::mutex lbLegMoveTimeMutex_;  // 保护 lbLegMoveStartTime_ 的互斥锁
  double lbLegMoveThresholdRad_ = 0.01;  // 下肢“发生明显变化”的角度阈值（弧度）

  Eigen::VectorXd filterJointDataForDrakeFK_;
  Eigen::VectorXd jointDataForDrakeFK_;

  // 滤波参数
  double jointSpaceAccLimit_ = 100.0;   // 关节角度ruckig滤波加速度约束参数
  double jointSpaceJerkLimit_ = 600.0;  // 关节角度ruckig滤波平滑系数
  double maxJointVelocity_ = 10.0;      // 关节最大角速度限制（弧度/秒）
  double lowpassDqAlpha_ = 0.9;         // lowpass_dq_低通滤波因子（历史值权重，新值权重为1-alpha）
  std::unique_ptr<ocs2::mobile_manipulator::KinemicLimitFilter> armJointRuckigFilterPtr_;
  std::unique_ptr<ocs2::mobile_manipulator::KinemicLimitFilter> lbJointRuckigFilterPtr_;

  // 手部位置约束参数
  double sphereRadiusLimit_ = 0.5;                                  // 手部位置约束球体半径
  double minReachableDistance_ = 0.20;                              // 最小可达距离
  Eigen::Vector3d boxMinBound_ = Eigen::Vector3d(0.25, -0.5, 0.1);  // 手部位置约束边界框最小值 [x, y, z]
  Eigen::Vector3d boxMaxBound_ = Eigen::Vector3d(1.0, 0.5, 1.0);    // 手部位置约束边界框最大值 [x, y, z]
  double chestOffsetY_ = 0.0;                                 // 胸部中线偏移量，用于防止左右手过中线
  Eigen::Vector3d leftCenter_ = Eigen::Vector3d(0, 0.02, 0);  // 左手圆柱体约束中心
  Eigen::Vector3d rightCenter_ = Eigen::Vector3d(0, -0.02, 0);  // 右手圆柱体约束中心
  double elbowMinDistance_ = 0.18;                              // 手到肘最小距离约束
  double elbowMaxDistance_ = 0.65;                              // 手到肘最大距离约束
  Eigen::Vector3d defaultLeftHandPosOnExit_;                    // 退出时左手默认目标位置
  Eigen::Vector3d defaultRightHandPosOnExit_;                   // 退出时右手默认目标位置
  double handChangingModeThreshold_ = 0.055;                    // 手部模式切换时的阈值
  bool useIncrementalHandOrientation_ = true;                   // 是否使用增量式手部姿态

  // 保存 ArmJoint 为全零时的双手位姿（在 init 函数中计算，避免运行时频繁调用 FK）
  // Link6 位姿（用于 IK 约束）
  Eigen::Vector3d initZeroLeftLink6Position_;   // 全零关节角度时左手 link6 位置
  Eigen::Vector3d initZeroRightLink6Position_;  // 全零关节角度时右手 link6 位置
  // End Effector 位姿（用于可视化等）
  Eigen::Vector3d initZeroLeftEndEffectorPosition_;   // 全零关节角度时左手 end_effector 位置
  Eigen::Vector3d initZeroRightEndEffectorPosition_;  // 全零关节角度时右手 end_effector 位置

  // 状态数据
  std::vector<PoseData> latestPoseConstraintList_;  // 保存最新的pose约束列表
  WheelIncrementalPoseResult latestIncrementalResult_;

  // 保存 incrementalController_ 查询结果的手部和肘部位置
  Eigen::Vector3d latestHumanLeftElbowPos_;   // 左肘位置（通过 FK 计算）
  Eigen::Vector3d latestHumanRightElbowPos_;  // 右肘位置（通过 FK 计算）

  Eigen::Vector3d latestRobotLeftElbowPos_;
  Eigen::Vector3d latestRobotRightElbowPos_;

  // 保存优化前后的手部位置（用于可视化对比）
  Eigen::Vector3d latestLeftHandPosBeforeOpt_;   // 优化前的左手位置
  Eigen::Vector3d latestLeftHandPosAfterOpt_;    // 优化后的左手位置
  Eigen::Vector3d latestRightHandPosBeforeOpt_;  // 优化前的右手位置
  Eigen::Vector3d latestRightHandPosAfterOpt_;   // 优化后的右手位置

  // 保存优化前后的胸/肩/肘位置（用于可视化对比）
  Eigen::Vector3d latestChestPosBeforeOpt_;
  Eigen::Vector3d latestChestPosAfterOpt_;
  Eigen::Vector3d latestLeftShoulderPosBeforeOpt_;
  Eigen::Vector3d latestLeftShoulderPosAfterOpt_;
  Eigen::Vector3d latestRightShoulderPosBeforeOpt_;
  Eigen::Vector3d latestRightShoulderPosAfterOpt_;
  Eigen::Vector3d latestLeftElbowPosBeforeOpt_;
  Eigen::Vector3d latestLeftElbowPosAfterOpt_;
  Eigen::Vector3d latestRightElbowPosBeforeOpt_;
  Eigen::Vector3d latestRightElbowPosAfterOpt_;

  // chest-frame cached pose for inactive hands/elbows
  Eigen::Vector3d leftHandPosInChest_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond leftHandQuatInChest_ = Eigen::Quaterniond::Identity();
  Eigen::Vector3d rightHandPosInChest_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond rightHandQuatInChest_ = Eigen::Quaterniond::Identity();
  Eigen::Vector3d leftElbowPosInChest_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d rightElbowPosInChest_ = Eigen::Vector3d::Zero();
  bool hasLeftHandPoseInChest_ = false;
  bool hasRightHandPoseInChest_ = false;
  bool hasLeftElbowPosInChest_ = false;
  bool hasRightElbowPosInChest_ = false;

  Eigen::VectorXd mec_limit_lower_;
  Eigen::VectorXd mec_limit_upper_;
  Eigen::Vector3d deltaScaleRPY_ = Eigen::Vector3d(1.0, 1.0, 1.0);

  // Grip 状态跟踪（用于检测上升沿并更新锚点，避免频繁切换 grip 时位置跳变）
  bool lastLeftGripPressed_ = false;
  bool lastRightGripPressed_ = false;

  bool chestIncrementalUpdateEnabled_ = true;

  struct ModeChangeCycleCache {
    bool leftHandCtrlModeChanged = false;
    bool rightHandCtrlModeChanged = false;

    bool leftChangingMaintainUpdated = false;
    bool rightChangingMaintainUpdated = false;
    bool leftChangingInstantUpdated = false;
    bool rightChangingInstantUpdated = false;

    void resetAll() {
      leftHandCtrlModeChanged = false;
      rightHandCtrlModeChanged = false;
      leftChangingMaintainUpdated = false;
      rightChangingMaintainUpdated = false;
      leftChangingInstantUpdated = false;
      rightChangingInstantUpdated = false;
    }

    void resetSmootherUpdatedFlags() {
      leftChangingMaintainUpdated = false;
      rightChangingMaintainUpdated = false;
      leftChangingInstantUpdated = false;
      rightChangingInstantUpdated = false;
    }
  };
  ModeChangeCycleCache modeChangeCycle_;

  // Grip 超时机制：当连续 grip 1 秒时，布尔值为 true
  static constexpr double GRIP_TIMEOUT_DURATION = 2.0;  // 超时时间：1秒
  ros::Time leftGripStartTime_;                         // 左手 grip 开始时间戳
  ros::Time rightGripStartTime_;                        // 右手 grip 开始时间戳
  std::mutex leftGripTimeMutex_;                        // 保护左手 grip 时间戳的互斥锁
  std::mutex rightGripTimeMutex_;                       // 保护右手 grip 时间戳的互斥锁
  std::atomic<bool> leftGripTimeoutReached_ = false;    // 左手连续 grip 1 秒标志
  std::atomic<bool> rightGripTimeoutReached_ = false;   // 右手连续 grip 1 秒标志

  // 激活所有手臂控制模式的计数器：确保在 fsmChange 结束后执行指定次数，增强鲁棒性
  static constexpr int ACTIVATE_ALL_ARM_CTRL_MODE_COUNT = 1;  // 激活模式执行次数
  int activateAllArmCtrlModeCounter_ = 0;                     // 当前执行计数

  // 退出 mode 2 的计数器：确保退出过渡逻辑执行指定次数，增强鲁棒性
  static constexpr int EXIT_MODE_2_EXECUTION_COUNT = 1;  // 退出模式执行次数
  int exitMode2Counter_ = 0;                             // 当前执行计数

  // 进入 mode 2 时重置位置的计数器：确保只重置有限次，避免每个循环都重置
  static constexpr int ENTER_MODE_2_RESET_COUNT = 1;  // 进入 mode 2 时重置位置的执行次数
  int enterMode2ResetCounter_ = 0;                    // 当前执行计数

  Eigen::Vector3d leftLink6Position_;
  Eigen::Vector3d rightLink6Position_;
  Eigen::Vector3d leftEndEffectorPosition_;
  Eigen::Vector3d rightEndEffectorPosition_;
  Eigen::Vector3d leftLink4Position_;
  Eigen::Vector3d rightLink4Position_;

  Eigen::Quaterniond leftLink6Quat_ = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond rightLink6Quat_ = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond leftEndEffectorQuat_ = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond rightEndEffectorQuat_ = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond leftLink4Quat_ = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond rightLink4Quat_ = Eigen::Quaterniond::Identity();
  Eigen::Vector3d leftVirtualThumbPosition_;
  Eigen::Vector3d rightVirtualThumbPosition_;

  Eigen::Vector3d leftEE2Link6Offset_;
  Eigen::Vector3d rightEE2Link6Offset_;

  Eigen::Vector3d leftThumb2Link6Offset_;
  Eigen::Vector3d rightThumb2Link6Offset_;

  // VR手部位姿缓存（避免频繁调用 getLeftHandPose() 和 getRightHandPose()）
  ::ArmPose latestLeftHandPose_vr_;
  ::ArmPose latestRightHandPose_vr_;
};

}  // namespace HighlyDynamic
