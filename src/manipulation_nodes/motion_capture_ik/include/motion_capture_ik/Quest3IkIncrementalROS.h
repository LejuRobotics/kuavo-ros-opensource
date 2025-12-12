#pragma once

#include <memory>
#include <mutex>
#include <thread>
#include "motion_capture_ik/json.hpp"

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>

#include <noitom_hi5_hand_udp_python/PoseInfoList.h>
#include <leju_utils/define.hpp>
#include "motion_capture_ik/ArmControlBaseROS.h"
#include "motion_capture_ik/TwoStageTorsoIKRec.h"
#include "motion_capture_ik/IncrementalControlModule.h"
#include "motion_capture_ik/HandSmoother.h"

namespace HighlyDynamic {

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

  void armModeCallback(const std_msgs::Int32::ConstPtr& msg) override;
  std::atomic<int> armControlMode_ = 0;
  std::atomic<int> lastArmControlMode_ = 0;

  // 超时机制相关
  ros::Time mode2EnterTime_;                              // 记录进入 mode 2 的时间戳
  std::mutex mode2EnterTimeMutex_;                        // 保护时间戳的互斥锁
  static constexpr double MODE_2_TIMEOUT_DURATION = 5.0;  // 超时时间：5秒

  void fsmEnter() override;
  void fsmChange() override;
  void fsmProcess() override;
  void fsmExit() override;

 private:
  void solveIkHandElbowThreadFuntion();

  bool processData();
  bool processChangingData(bool leftHandCtrlModeChanged, bool rightHandCtrlModeChanged);

  // 左右手独立处理模式切换数据
  bool processChangingDataLeftArm(bool leftHandCtrlModeChanged);
  bool processChangingDataRightArm(bool rightHandCtrlModeChanged);

  // 左右手独立处理正常运动数据
  bool processDataLeftArm();
  bool processDataRightArm();

  void solveIk();
  void processVisual();

  // 数据更新函数
  bool detectHumanArmMove();
  bool detectLeftArmMove();
  bool detectRightArmMove();

  bool detectLeftGripPressed();
  bool detectRightGripPressed();

  bool updateLatestIncrementalResult();
  void updateConstraintList(const Eigen::Vector3d& leftHandPos,
                            const Eigen::Quaterniond& leftHandQuat,
                            const Eigen::Vector3d& rightHandPos,
                            const Eigen::Quaterniond& rightHandQuat,
                            const Eigen::Vector3d& leftElbowPos,
                            const Eigen::Vector3d& rightElbowPos);

  void updateLeftConstraintList(const Eigen::Vector3d& leftHandPos,
                                const Eigen::Quaterniond& leftHandQuat,
                                const Eigen::Vector3d& leftElbowPos);
  void updateRightConstraintList(const Eigen::Vector3d& rightHandPos,
                                 const Eigen::Quaterniond& rightHandQuat,
                                 const Eigen::Vector3d& rightElbowPos);
  void updateSmootheIntermidiateHandPos(const Eigen::Vector3d& leftHandPos,
                                        const Eigen::Vector3d& rightHandPos,
                                        bool leftHandCtrlModeChanged,
                                        bool rightHandCtrlModeChanged);
  bool updateLeftHandChangingMode(const Eigen::Vector3d& leftTargetPos);
  bool updateRightHandChangingMode(const Eigen::Vector3d& rightTargetPos);

  // 发布函数
  void publishJointStates();
  void publishSensorDataArmJoints();  // 发布传感器数据的手臂关节角

  void reset();                          // 重置所有运行时状态，确保进入系统时正常
  void forceDeactivateAllArmCtrlMode();  // 强制停用所有手臂控制模式
  void forceActivateAllArmCtrlMode();    // 强制激活所有手臂控制模式

  ros::Publisher kuavoArmTrajCppPublisher_;  // 发布kuavo_arm_traj_cpp；launch中通过remap话题方式来接入当前系统
  ros::Publisher sensorDataArmJointsPublisher_;  // 发布传感器数据的手臂关节角
  ros::Publisher leftHandPosePublisher_;         // 发布左手pose
  ros::Publisher rightHandPosePublisher_;        // 发布右手pose

  std::thread ikSolveThread_;
  std::mutex bonePoseHandElbowMutex_;
  std::mutex poseConstraintListMutex_;  // 保护 latestPoseConstraintList_ 的互斥锁
  std::mutex ikResultMutex_;

  int jointStateSize_;
  ArmIdx ctrlArmIdx_;  // 控制哪个手臂：LEFT, RIGHT, 或 BOTH

  std::unique_ptr<TwoStageTorsoIK> twoStageTorsoIkPtr_;
  std::unique_ptr<IncrementalControlModule> incrementalController_;
  std::unique_ptr<HandSmoother> leftHandSmoother_;
  std::unique_ptr<HandSmoother> rightHandSmoother_;

  // Drake diagram and context for plant
  std::unique_ptr<drake::systems::Diagram<double>> diagram_;
  std::unique_ptr<drake::systems::Context<double>> diagramContext_;

  // IK求解结果
  Eigen::VectorXd latestIkSolution_;
  bool hasValidIkSolution_ = false;

  // 姿态和位置偏置
  Eigen::Quaterniond leftHandQuatOffset_;
  Eigen::Quaterniond rightHandQuatOffset_;
  Eigen::Vector3d leftHandOffsetRpy_;
  Eigen::Vector3d rightHandOffsetRpy_;
  Eigen::Vector3d deltaScale_;  // delta_scale 参数（x, y, z三轴独立）

  // 机器人结构参数
  Eigen::Vector3d robotRightFixedShoulderPos_;  // 右肩绝对位置
  Eigen::Vector3d robotLeftFixedShoulderPos_;   // 左肩绝对位置
  double upperArmLength_;                       // 机器人上臂长度
  double lowerArmLength_;                       // 机器人下臂长度

  // 关节状态
  Eigen::VectorXd q_;   // 滤波后的关节角度（弧度）
  Eigen::VectorXd dq_;  // 滤波后的关节角速度（弧度/秒）
  Eigen::VectorXd jointMidValues_;  // TEST: 关节限制中间值（用于测试），存储每个关节的(limit_lower+limit_upper)/2

  // 滤波参数
  double fhan_r_joint_ = 900.0;      // 关节角度fhan滤波加速度约束参数
  double fhan_kh0_joint_ = 6.0;      // 关节角度fhan滤波平滑系数
  double max_joint_velocity_ = 1.0;  // 关节最大角速度限制（弧度/秒）

  // 手部位置约束参数
  double sphereRadiusLimit_ = 0.5;                                  // 手部位置约束球体半径
  double minReachableDistance_ = 0.08;                              // 最小可达距离
  Eigen::Vector3d boxMinBound_ = Eigen::Vector3d(0.25, -0.5, 0.1);  // 手部位置约束边界框最小值 [x, y, z]
  Eigen::Vector3d boxMaxBound_ = Eigen::Vector3d(1.0, 0.5, 1.0);    // 手部位置约束边界框最大值 [x, y, z]
  double chestOffsetY_ = 0.0;                                 // 胸部中线偏移量，用于防止左右手过中线
  Eigen::Vector3d leftCenter_ = Eigen::Vector3d(0, 0.02, 0);  // 左手圆柱体约束中心
  Eigen::Vector3d rightCenter_ = Eigen::Vector3d(0, -0.02, 0);  // 右手圆柱体约束中心
  Eigen::Vector3d defaultLeftHandPosOnExit_;                    // 退出时左手默认目标位置
  Eigen::Vector3d defaultRightHandPosOnExit_;                   // 退出时右手默认目标位置
  double hand_changing_mode_threshold_ = 0.055;                 // 手部模式切换时的阈值

  // 保存 ArmJoint 为全零时的双手位姿（在 init 函数中计算，避免运行时频繁调用 FK）
  // Link6 位姿（用于 IK 约束）
  Eigen::Vector3d initZeroLeftLink6Position_;         // 全零关节角度时左手 link6 位置
  Eigen::Quaterniond initZeroLeftLink6Orientation_;   // 全零关节角度时左手 link6 姿态
  Eigen::Vector3d initZeroRightLink6Position_;        // 全零关节角度时右手 link6 位置
  Eigen::Quaterniond initZeroRightLink6Orientation_;  // 全零关节角度时右手 link6 姿态
  // End Effector 位姿（用于可视化等）
  Eigen::Vector3d initZeroLeftEndEffectorPosition_;         // 全零关节角度时左手 end_effector 位置
  Eigen::Quaterniond initZeroLeftEndEffectorOrientation_;   // 全零关节角度时左手 end_effector 姿态
  Eigen::Vector3d initZeroRightEndEffectorPosition_;        // 全零关节角度时右手 end_effector 位置
  Eigen::Quaterniond initZeroRightEndEffectorOrientation_;  // 全零关节角度时右手 end_effector 姿态

  // 状态数据
  std::vector<PoseData> latestPoseConstraintList_;  // 保存最新的pose约束列表
  IncrementalPoseResult latestIncrementalResult_;

  Eigen::VectorXd mec_limit_lower_;
  Eigen::VectorXd mec_limit_upper_;
  Eigen::Vector3d deltaScaleRPY_ = Eigen::Vector3d(1.0, 1.0, 1.0);

  // Grip 状态跟踪（用于检测上升沿并更新锚点，避免频繁切换 grip 时位置跳变）
  bool lastLeftGripPressed_ = false;
  bool lastRightGripPressed_ = false;

  // 激活所有手臂控制模式的计数器：确保在 fsmChange 结束后执行指定次数，增强鲁棒性
  static constexpr int ACTIVATE_ALL_ARM_CTRL_MODE_COUNT = 1;  // 激活模式执行次数
  int activateAllArmCtrlModeCounter_ = 0;                     // 当前执行计数

  // 退出 mode 2 的计数器：确保退出过渡逻辑执行指定次数，增强鲁棒性
  static constexpr int EXIT_MODE_2_EXECUTION_COUNT = 1;  // 退出模式执行次数
  int exitMode2Counter_ = 0;                             // 当前执行计数

  // 进入 mode 2 时重置位置的计数器：确保只重置有限次，避免每个循环都重置
  static constexpr int ENTER_MODE_2_RESET_COUNT = 1;  // 进入 mode 2 时重置位置的执行次数
  int enterMode2ResetCounter_ = 0;                    // 当前执行计数
};

}  // namespace HighlyDynamic
