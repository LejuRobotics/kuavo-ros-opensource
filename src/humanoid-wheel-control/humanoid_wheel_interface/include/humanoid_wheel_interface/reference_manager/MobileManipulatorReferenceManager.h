#pragma once

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
#include <kuavo_msgs/twoArmHandPoseCmd.h>
#include <kuavo_msgs/changeTorsoCtrlMode.h>

#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include "humanoid_wheel_interface/ManipulatorModelInfo.h"
#include "ocs2_robotic_tools/common/RotationTransforms.h"
#include "ocs2_pinocchio_interface/PinocchioInterface.h"
#include "humanoid_interface/common/TopicLogger.h"

#include "humanoid_wheel_interface/common/VelocityLimiter.h"

namespace ocs2 {
namespace mobile_manipulator {

class MobileManipulatorReferenceManager : public ReferenceManager {
public:
  MobileManipulatorReferenceManager(const ManipulatorModelInfo& info, const PinocchioInterface& pinocchioInterface);
  ~MobileManipulatorReferenceManager() override = default;

  void setupSubscriptions(std::string nodeHandleName = "mobile_manipulator") override;
  
  // 服务回调函数
  bool controlModeService(kuavo_msgs::changeTorsoCtrlMode::Request& req, kuavo_msgs::changeTorsoCtrlMode::Response& res);
  bool getMpcControlModeService(kuavo_msgs::changeTorsoCtrlMode::Request& req, kuavo_msgs::changeTorsoCtrlMode::Response& res);

  // 多个约束轨迹的操作函数
  void getFirstTargetTrajectories(const TargetTrajectories& targetTrajectories);
  void getAllTargetTrajectories(const TargetTrajectories& targetTrajectories);
  void trimTargetTrajectoriesBeforeTime(scalar_t startTime);
  void publishTargetTrajectoriesLast(void);

  // 多种约束轨迹的获取函数
  const TargetTrajectories& getStateInputTargetTrajectories() const { return stateInputTargetTrajectories; }
  const TargetTrajectories& getTorsoTargetTrajectories() const { return torsoTargetTrajectories; }
  const TargetTrajectories& getEeTargetTrajectories() const { return eeTargetTrajectories; }

  // 多种约束的使能函数
  const bool getEnableEeTargetTrajectories() const { return enableEeFlag; }
  void setEnableEeTargetTrajectories(bool flag) { enableEeFlag = flag; }
  const bool getEnableArmJointTrack() const { return enableArmJointTrackFlag; }
  void setEnableArmJointTrack(bool flag) { enableArmJointTrackFlag = flag; }
  const bool getEnableBaseTrack() const { return enableBaseTrackFlag; }
  void setEnableBaseTrack(bool flag) { enableBaseTrackFlag = flag; }

  // 手臂锁住电机期望
  void setLockArmJointTarget(const vector_t& lockState)
  {
    assert(lockState.size() == lockArmState.size() && "Lock state dimension mismatch!");
    lockArmState = lockState;
  }
protected:
  virtual void modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState, TargetTrajectories& targetTrajectories,
                                ModeSchedule& modeSchedule) override;
  void generateTargetwithArmJointCmd(scalar_t initTime, double dt, const vector_t &initState,
                                     const vector_t& armJointCmd);
  void generateTargetwithVelcmdLimited(scalar_t initTime, scalar_t finalTime, const vector_t &initState,
                                  vector_t cmdVel);
  void generateTargetwithVelcmd(scalar_t initTime, scalar_t finalTime, const vector_t &initState,
                                const vector_t &cmdVel);
  void generateTargetwithPoseCmd(double initTime, const vector_t &initState,
                                 double cmdPoseTime, const vector_t &cmdPose);
  
  double targetYawPreProcess(double currentYaw, double targetYaw);
  double calcExpectedTime(const vector_t& currentPose, const vector_t& cmdPose);
  bool isVectorClose(const vector_t& v1, const vector_t& v2, const vector_t& threshold);
  
  // 辅助函数
  void updateArmEndEffectorTrajectory(scalar_t initTime, scalar_t finalTime, TargetTrajectories& targetTrajectories);
  void updateBaseTrajectory(scalar_t initTime, scalar_t finalTime, const vector_t &initState,
                           TargetTrajectories &targetTrajectories, vector_t cmdVel);
  bool getControlModeIsChange(int currentMode)
  {
    static int preMode = 0;
    bool isChange{false};
    if(preMode != currentMode) isChange = true;
    preMode = currentMode;
    return isChange;
  }

  // 获取当前末端位姿
  void getCurrentEeWorldPose(vector_t& EeState, const vector_t& initState);
  void transformLocalToWorldFrame(vector_t& localPose, const vector_t& baseState);

  // 不同控制模式的执行函数
  void updateNoControl(double initTime, const TargetTrajectories& targetTrajectories, bool isChange);
  void updateArmOnlyControl(double initTime, const vector_t& initState, bool isChange);
  void updateBaseOnlyControl(double initTime, double finalTime, const vector_t& initState, bool isChange);
  void updateBaseArmControl(double initTime, double finalTime, const vector_t& initState, bool isChange);
  void updateArmEeOnlyControl(double initTime, const vector_t& initState, bool isChange);

private:

  ros::NodeHandle nodeHandle_;
  humanoid::TopicLogger *ros_logger_ = nullptr;
  const ManipulatorModelInfo info_;
  double baseDim_ = 0;

  // 动力学库接口
  PinocchioInterface pinocchioInterface_;

  // 指令底盘速度
  bool isCmdVelUpdated_{false};
  Eigen::Vector3d cmdVel_;
  Eigen::Vector3d currentCmdVel_;
  std::mutex cmdvel_mtx_;
  ros::Subscriber targetVelocitySubscriber_;

  // 指令底盘位置
  std::vector<double> wheel_move_spd_;
  vector_t wheelReachThreshold_;
  bool isCmdPoseReach_{false};
  bool isCmdPoseUpdated_{false};
  double cmdPoseTime_ = 0;
  double currentCmdPoseTime_ = 0;
  Eigen::Vector3d cmdPose_;
  Eigen::Vector3d currentCmdPose_;
  std::mutex cmdPose_mtx_;
  ros::Subscriber targetPoseSubscriber_;
  ros::Publisher targetPoseFlagPub_;

  // 双臂末端执行器位姿指令 (x,y,z,qx,qy,qz,qw)
  vector_t left_arm_traj_pose_;
  vector_t right_arm_traj_pose_;
  vector_t currentLeftArmPose_;
  vector_t currentRightArmPose_;
  std::mutex armPose_mtx_;
  ros::Subscriber armEndEffectorSubscriber_;

  // 手臂关节轨迹指令
  vector_t arm_joint_traj_;
  std::mutex armJoint_mtx_;
  ros::Subscriber arm_joint_traj_sub_;

  // 分别保存左右臂的关节轨迹（弧度）以及是否将关节角作为期望输入
  vector_t left_arm_joint_traj_;
  vector_t right_arm_joint_traj_;
  int desire_mode_{2};  // 0: end-effector command in World Frame | 1: end-effector command in Local Frame | 2: joint command 

  // 手臂和躯干的相对位置缓存
  vector_t cachedLeftArmRelativePose_;   // 左臂相对于躯干的位姿
  vector_t cachedRightArmRelativePose_;  // 右臂相对于躯干的位姿
  vector_t cachedTorsoPose_;             // 躯干位姿缓存
  bool relativePoseCached_{false};       // 是否已缓存相对位置
  std::mutex relativePose_mtx_;          // 相对位置缓存锁

  // MPC控制模式相关
  int currentMpcControlMode_{0};  // 0: NoControl, 1: ArmOnly, 2: BaseOnly, 3: BaseArm
  std::mutex controlMode_mtx_;
  ros::ServiceServer controlModeServiceServer_;
  ros::ServiceServer getMpcControlModeServiceServer_;

  // 多种约束所需轨迹相关
  TargetTrajectories stateInputTargetTrajectories;
  TargetTrajectories torsoTargetTrajectories;
  TargetTrajectories eeTargetTrajectories;

  // 多种约束所需轨迹相关
  bool enableEeFlag{true};
  bool enableArmJointTrackFlag{false};
  bool enableBaseTrackFlag{true};

  // 锁住的手臂期望
  vector_t lockArmState;

  // 梯形插补加减速设置
    std::shared_ptr<mobile_manipulator::VelocityLimiter> velLimiter_;
};
} // namespace mobile_manipulator
} // namespace ocs2