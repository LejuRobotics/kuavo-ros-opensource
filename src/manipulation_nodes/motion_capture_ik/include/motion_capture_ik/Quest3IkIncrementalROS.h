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
#include "motion_capture_ik/TwoStageIncrementalIK.h"
#include "motion_capture_ik/IncrementalControlModule.h"

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

  // 重写基类的FSM方法
  void fsmEnter() override;
  void fsmProcess() override;
  void fsmExit() override;

 private:
  bool detectHumanArmMove();

  bool updateLatestIncrementalResult();

  void updateConstraintList(const Eigen::Vector3d& leftHandPos,
                            const Eigen::Quaterniond& leftHandQuat,
                            const Eigen::Vector3d& rightHandPos,
                            const Eigen::Quaterniond& rightHandQuat,
                            const Eigen::Vector3d& leftElbowPos,
                            const Eigen::Vector3d& rightElbowPos);

  ros::Publisher kuavoArmTrajCppPublisher_;  // 发布kuavo_arm_traj_cpp；launch中通过remap话题方式来接入当前系统
  ros::Publisher sensorDataArmJointsPublisher_;  // 发布传感器数据的手臂关节角

  std::thread ikSolveThread_;
  std::mutex bonePoseHandElbowMutex_;
  std::mutex poseConstraintListMutex_;  // 保护 latestPoseConstraintList_ 的互斥锁

  int jointStateSize_;
  ArmIdx ctrlArmIdx_;  // 控制哪个手臂：LEFT, RIGHT, 或 BOTH

  std::unique_ptr<TwoStageIncrementalIK> twoStageTorsoIkPtr_;

  // Drake diagram and context for plant
  std::unique_ptr<drake::systems::Diagram<double>> diagram_;
  std::unique_ptr<drake::systems::Context<double>> diagramContext_;

  std::mutex ikResultMutex_;

  // 保存最新IK求解结果的私有成员变量
  Eigen::VectorXd latestIkSolution_;
  bool hasValidIkSolution_ = false;

  // 增量控制模块
  std::unique_ptr<IncrementalControlModule> incrementalController_;

  // 末端夹爪姿态偏置旋转（四元数）
  Eigen::Quaterniond leftHandQuatOffset_;
  Eigen::Quaterniond rightHandQuatOffset_;

  Eigen::Vector3d leftHandOffsetRpy_;
  Eigen::Vector3d rightHandOffsetRpy_;

  // delta_scale 参数（x, y, z三轴独立）
  Eigen::Vector3d deltaScale_;

  // 保存最新的pose约束列表
  std::vector<PoseData> latestPoseConstraintList_;

  Eigen::Vector3d robotRightFixedShoulderPos_;  // 右肩绝对位置
  Eigen::Vector3d robotLeftFixedShoulderPos_;   // 左肩绝对位置

  // 机器人手臂长度参数
  double upperArmLength_;  // 机器人上臂长度
  double lowerArmLength_;  // 机器人下臂长度

  Eigen::VectorXd q_;   // 滤波后的关节角度（弧度）
  Eigen::VectorXd dq_;  // 滤波后的关节角速度（弧度/秒）

  // TEST: 关节限制中间值（用于测试）
  Eigen::VectorXd jointMidValues_;  // 存储每个关节的(limit_lower+limit_upper)/2

  // fhan滤波参数
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
  IncrementalPoseResult latestIncrementalResult_;

  void solveIkHandElbowThreadFuntion();

  void solveIkWithPoseConstraints();

  void publishJointStates();
  void publishSensorDataArmJoints();  // 发布传感器数据的手臂关节角
};

}  // namespace HighlyDynamic
