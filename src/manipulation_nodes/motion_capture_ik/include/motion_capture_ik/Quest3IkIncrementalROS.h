#pragma once

#include <memory>
#include <mutex>
#include <thread>
#include "motion_capture_ik/json.hpp"

#include <ros/ros.h>
#include <geometry_msgs/Point.h>

#include <noitom_hi5_hand_udp_python/PoseInfoList.h>
#include <leju_utils/define.hpp>
#include "motion_capture_ik/ArmControlBaseROS.h"
#include "motion_capture_ik/OneStageIKEndEffector.h"
#include "motion_capture_ik/IncrementalControlModule.h"
#include "motion_capture_ik/HandSmoother.h"
#include "DrakeElbowHandPointOpt.hpp"
#include <std_msgs/Float64MultiArray.h>

namespace HighlyDynamic {

struct DrakeVelocityIKBoundsConfig {
  // Upper bound x component offset: ub.x = p0.x + xUpperOffset
  double xUpperOffset = 0.6;

  // Lower bound z component (absolute): lb.z = zLower
  double zLower = -0.3;

  // Upper bound z component offset: ub.z = p0.z + zUpperOffset
  double zUpperOffset = 0.1;

  // Left arm y lower bound (absolute): lb.y = leftYLower
  double leftYLower = -0.2;

  // Left arm y upper bound offset: ub.y = p0.y + leftYUpperOffset
  double leftYUpperOffset = 0.5;

  // Right arm y lower bound offset: lb.y = p0.y + rightYLowerOffset
  double rightYLowerOffset = -0.5;

  // Right arm y upper bound (absolute): ub.y = rightYUpper
  double rightYUpper = 0.2;
};

class Quest3IkIncrementalROS final : public ArmControlBaseROS {
 public:
  explicit Quest3IkIncrementalROS(ros::NodeHandle& nodeHandle,
                                  double publishRate,
                                  bool debugPrint = false,
                                  ArmIdx ctrlArmIdx = ArmIdx::BOTH);

  ~Quest3IkIncrementalROS();

  void initialize(const nlohmann::json& configJson) override;
  void run() override;

 protected:
  void activateController() override;
  void deactivateController() override;

  void armCtrlModeCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
  void armModeCallback(const std_msgs::Int32::ConstPtr& msg) override;
  std::atomic<int> armControlMode_ = 0;
  std::atomic<int> lastArmControlMode_ = 0;
  int arm_ctrl_mode_;

  // 超时机制相关
  ros::Time mode2EnterTime_;                              // 记录进入 mode 2 的时间戳
  std::mutex mode2EnterTimeMutex_;                        // 保护时间戳的互斥锁
  static constexpr double MODE_2_TIMEOUT_DURATION = 5.0;  // 超时时间：5秒

  void fsmEnter() override;
  void fsmChange() override;
  void fsmProcess() override;
  void fsmExit() override;

  PointTrackIKSolverConfig loadPointTrackIKSolverConfigFromJson(const nlohmann::json& configJson);

  DrakeVelocityIKWeightConfig loadDrakeVelocityIKWeightsFromJson(const nlohmann::json& configJson);

  DrakeVelocityIKBoundsConfig loadDrakeVelocityIKBoundsFromJson(const nlohmann::json& configJson);

  void loadDrakeVelocityIKGeometryFromJson(const nlohmann::json& configJson);

 private:
  void solveIkHandElbowThreadFunction();

  // 从 sensorData 抽取 14 维双臂关节角（rad），并做指数均值滤波：q = 0.99*q + 0.01*qnew
  void updateSensorArmJointMeanFromSensorData();

  ros::Subscriber arm_ctrl_mode_vr_sub_;

  // FK 辅助函数：计算左手末端执行器姿态
  void computeLeftEndEffectorFK(Eigen::Vector3d& pOut, Eigen::Quaterniond& qOut);
  // FK 辅助函数：计算右手末端执行器姿态
  void computeRightEndEffectorFK(Eigen::Vector3d& pOut, Eigen::Quaterniond& qOut);
  // FK 辅助函数：计算左右臂 link4 姿态（仅用于透传到增量控制模块）
  void computeLeftLink4FK(Eigen::Vector3d& pOut, Eigen::Quaterniond& qOut);
  void computeRightLink4FK(Eigen::Vector3d& pOut, Eigen::Quaterniond& qOut);
  void computeLeftLink6FK(Eigen::Vector3d& pOut, Eigen::Quaterniond& qOut);
  void computeRightLink6FK(Eigen::Vector3d& pOut, Eigen::Quaterniond& qOut);

  // 左右手独立处理模式切换数据
  bool processChangingDataLeftArm(bool leftHandCtrlModeChanged);
  bool processChangingDataRightArm(bool rightHandCtrlModeChanged);

  // 左右手独立处理正常运动数据
  bool processDataLeftArm();
  bool processDataRightArm();

  void solveIk();
  void processVisual();

  bool detectLeftArmMove();
  bool detectRightArmMove();

  // 【数据校验】3点跳变检测
  bool validateVrPose(const ArmPose& currentPose, ArmPose& validatedPose, const std::string& side, bool isArmActive);

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
  void publishSensorDataArmJoints();        // 发布传感器数据的手臂关节角
  void publishHandPosOptimizationPoints();  // 发布优化前后的手部位置点
  void publishHandPoseFromTransformer();    // 发布来自Transformer的手部pose

  void reset();                          // 重置所有运行时状态，确保进入系统时正常
  void forceDeactivateAllArmCtrlMode();  // 强制停用所有手臂控制模式
  void forceActivateAllArmCtrlMode();    // 强制激活所有手臂控制模式

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
  ros::Publisher ikSolvedEefPosePublisher_;  // 发布IK求解后的末端执行器位姿（与Python版/drake_ik/eef_pose一致）
  ros::Publisher ikInputPosPublisher_;  // 发布IK输入的目标位姿（与Python版/drake_ik/input_pos一致）

  std::thread ikSolveThread_;
  std::mutex bonePoseHandElbowMutex_;
  std::mutex poseConstraintListMutex_;  // 保护 latestPoseConstraintList_ 的互斥锁
  std::mutex ikResultMutex_;
  std::mutex transformerMutex_;  // 【新增】专门保护 Transformer 的更新和读取，避免竞态条件

  int jointStateSize_;
  int waist_dof_;      // 腰部自由度数量（从JSON配置读取NUM_WAIST_JOINT）
  ArmIdx ctrlArmIdx_;  // 控制哪个手臂：LEFT, RIGHT, 或 BOTH

  std::unique_ptr<OneStageIKEndEffector> oneStageIkEndEffectorPtr_;
  std::unique_ptr<IncrementalControlModule> incrementalController_;
  std::unique_ptr<HandSmoother> leftHandSmoother_;
  std::unique_ptr<HandSmoother> rightHandSmoother_;

  std::unique_ptr<DrakeVelocityIKSolver> leftVelocityIkSolverPtr_;
  std::unique_ptr<DrakeVelocityIKSolver> rightVelocityIkSolverPtr_;

  // Drake diagram and context for plant
  std::unique_ptr<drake::systems::Diagram<double>> diagram_;
  std::unique_ptr<drake::systems::Context<double>> diagramContext_;

  // IK求解结果
  Eigen::VectorXd latestIkSolution_;
  bool hasValidIkSolution_ = false;

  // 姿态和位置偏置
  Eigen::Vector3d deltaScale_;  // delta_scale 参数（x, y, z三轴独立）

  // 机器人结构参数
  Eigen::Vector3d robotRightFixedShoulderPos_;  // 右肩绝对位置
  Eigen::Vector3d robotLeftFixedShoulderPos_;   // 左肩绝对位置
  double l1_;                                   // 第一段连杆长度 ||j2_j4||
  double l2_;                                   // 第二段连杆长度 ||j4_j6||

  // 关节状态
  Eigen::VectorXd q_;           // 滤波后的关节角度（弧度）
  Eigen::VectorXd dq_;          // 滤波后的关节角速度（弧度/秒）
  Eigen::VectorXd latest_q_;    // 最新的关节角度（弧度）
  Eigen::VectorXd latest_dq_;   // 最新的关节角速度（弧度/秒）
  Eigen::VectorXd lowpass_dq_;  // 低通滤波后的关节角速度（弧度/秒）
  Eigen::VectorXd jointMidValues_;  // TEST: 关节限制中间值（用于测试），存储每个关节的(limit_lower+limit_upper)/2

  // 传感器数据关节角（14维，rad）：用于保存 sensorData 对应的机器人双臂关节数据（指数均值滤波后）
  static constexpr int SENSOR_ARM_JOINT_DIM = 14;
  Eigen::VectorXd sensorArmJointQ_;  // 指数均值滤波后的关节角度（弧度，长度固定为 14）

  // 滤波参数
  double fhanRJoint_ = 900.0;      // 关节角度fhan滤波加速度约束参数
  double fhanKh0Joint_ = 6.0;      // 关节角度fhan滤波平滑系数
  double maxJointVelocity_ = 1.0;  // 关节最大角速度限制（弧度/秒）
  double lowpassDqAlpha_ = 0.9;    // lowpass_dq_低通滤波因子（历史值权重，新值权重为1-alpha）

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
  Eigen::Vector3d leftElbowFixedPoint_;                         // 左肘固定点（用于 updateLeftConstraintList）
  Eigen::Vector3d rightElbowFixedPoint_;                        // 右肘固定点（用于 updateRightConstraintList）
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
  IncrementalPoseResult latestIncrementalResult_;

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

  Eigen::VectorXd mec_limit_lower_;
  Eigen::VectorXd mec_limit_upper_;
  Eigen::Vector3d deltaScaleRPY_ = Eigen::Vector3d(1.0, 1.0, 1.0);

  // Grip 状态跟踪（用于检测上升沿并更新锚点，避免频繁切换 grip 时位置跳变）
  bool lastLeftGripPressed_ = false;
  bool lastRightGripPressed_ = false;

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
  Eigen::Vector3d leftVirtualThumbPosition_;
  Eigen::Vector3d rightVirtualThumbPosition_;

  Eigen::Vector3d leftEE2Link6Offset_;
  Eigen::Vector3d rightEE2Link6Offset_;

  Eigen::Vector3d leftThumb2Link6Offset_;
  Eigen::Vector3d rightThumb2Link6Offset_;

  // 【3点跳变检测】只存储前两个点
  static constexpr double SPIKE_THRESHOLD = 0.50;  // 跳变阈值（50cm）
  static constexpr int SPIKE_RECOVERY_COUNT = 2;  // 连续跳变恢复阈值：连续N帧跳变后恢复（可能是快速正常运动）
  static constexpr double SPIKE_TIMEOUT_DURATION = 0.2;  // 超时恢复时间（秒）：如果跳变持续超过0.2秒，强制恢复
  // 左右手分别存储前两个点的位置
  Eigen::Vector3d leftHandPrev2_;      // 左手前两个点
  Eigen::Vector3d leftHandPrev1_;      // 左手前一个点
  Eigen::Vector3d rightHandPrev2_;     // 右手前两个点
  Eigen::Vector3d rightHandPrev1_;     // 右手前一个点
  int leftHandCount_ = 0;              // 左手数据计数
  int rightHandCount_ = 0;             // 右手数据计数
  int leftHandSpikeCount_ = 0;         // 左手连续跳变计数
  int rightHandSpikeCount_ = 0;        // 右手连续跳变计数
  ros::Time leftHandSpikeStartTime_;   // 左手跳变开始时间
  ros::Time rightHandSpikeStartTime_;  // 右手跳变开始时间
};

}  // namespace HighlyDynamic
