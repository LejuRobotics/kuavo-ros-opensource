#pragma once

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int8MultiArray.h>
#include <kuavo_msgs/twoArmHandPoseCmd.h>
#include <kuavo_msgs/changeTorsoCtrlMode.h>
#include <kuavo_msgs/changeArmCtrlMode.h>

#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include "humanoid_wheel_interface/ManipulatorModelInfo.h"
#include "ocs2_robotic_tools/common/RotationTransforms.h"
#include "ocs2_pinocchio_interface/PinocchioInterface.h"
#include "humanoid_interface/common/TopicLogger.h"

#include "humanoid_wheel_interface/motion_planner/VelocityLimiter.h"
#include "humanoid_wheel_interface/motion_planner/cmdPosePlannerWithRuckig.h"
#include "humanoid_wheel_interface/motion_planner/cmdVelPlannerWithRuckig.h"

namespace ocs2 {
namespace mobile_manipulator {

enum MpcControlMode {
  NoControl = 0,    // 模式0: 使用上层下发的 targetTrajectories, 
  ArmOnly = 1,      // 模式1: 关节可动, 底盘锁住
  BaseOnly = 2,     // 模式2: 底盘可动, 下肢和手臂锁住
  BaseArm = 3,      // 模式3: 必须控制底盘, 手臂支持局部系和世界系笛卡尔和关节两种轨迹
  ArmEeOnly = 4     // 模式4: 底盘随末端移动, 不可控制, 手臂支持世界系笛卡尔轨迹
};

// 轮臂的手臂控制模式
enum LbArmControlMode {
  FalseMode = -1,   // 无效的模式
  WorldFrame = 0,   // 世界系的笛卡尔空间控制模式
  LocalFrame = 1,   // 相对浮动基座的笛卡尔空间控制模式
  JointSpace = 2,   // 关节控制模式
};

// 轮臂的服务切换模式，影响手臂的指令更新逻辑
enum LbArmControlServiceMode {
  KEEP = 0,        // 保持当前关节动作
  AUTO_SWING = 1,       // 摆动手模式
  EXTERN_CONTROL = 2,    // 外部控制模式
};

class MobileManipulatorReferenceManager : public ReferenceManager {
public:
  MobileManipulatorReferenceManager(const ManipulatorModelInfo& info, const PinocchioInterface& pinocchioInterface, const std::string& taskFile);
  ~MobileManipulatorReferenceManager() override = default;

  void setupSubscriptions(std::string nodeHandleName = "mobile_manipulator") override;

  // 配置加载函数
  void loadParamFromTaskFile(void);
  
  // 从参数服务器中更新初始期望
  void setRobotInitialArmJointTarget(ros::NodeHandle& input_nh);
  
  // 服务回调函数
  bool controlModeService(kuavo_msgs::changeTorsoCtrlMode::Request& req, kuavo_msgs::changeTorsoCtrlMode::Response& res);
  bool getMpcControlModeService(kuavo_msgs::changeTorsoCtrlMode::Request& req, kuavo_msgs::changeTorsoCtrlMode::Response& res);
  bool armControlModeSrvCallback(kuavo_msgs::changeArmCtrlMode::Request &req, kuavo_msgs::changeArmCtrlMode::Response &res);
  bool getArmControlModeCallback(kuavo_msgs::changeArmCtrlMode::Request &req, kuavo_msgs::changeArmCtrlMode::Response &res);

  // 多个约束轨迹的操作函数
  void getFirstTargetTrajectories(const TargetTrajectories& targetTrajectories);
  void getAllTargetTrajectories(const TargetTrajectories& targetTrajectories);
  void trimTargetTrajectoriesBeforeTime(scalar_t startTime);
  void publishTargetTrajectoriesNear(scalar_t initTime);
  vector_t targetTrajToPose6D(const TargetTrajectories& Traj, scalar_t initTime);

  // 多种约束轨迹的获取函数
  const TargetTrajectories& getStateInputTargetTrajectories() const { return stateInputTargetTrajectories_; }
  const TargetTrajectories& getTorsoTargetTrajectories() const { return torsoTargetTrajectories_; }
  const TargetTrajectories& getEeTargetTrajectories() const { return eeTargetTrajectories_; }

  // 多种约束的使能函数
  const bool getEnableEeTargetTrajectories() const { return enableEeFlag_; }
  void setEnableEeTargetTrajectories(bool flag) { enableEeFlag_ = flag; }
  const bool getEnableEeTargetLocalTrajectories() const { return enableEeLocalFlag_; }
  void setEnableEeTargetLocalTrajectories(bool flag) { enableEeLocalFlag_ = flag; }
  const bool getEnableArmJointTrack() const { return enableArmJointTrackFlag_; }
  void setEnableArmJointTrack(bool flag) { enableArmJointTrackFlag_ = flag; }
  const bool getEnableLegJointTrack() const { return enableLegJointTrackFlag_; }
  void setEnableLegJointTrack(bool flag) { enableLegJointTrackFlag_ = flag; }
  const bool getEnableTorsoPoseTargetTrajectories() const { return enableTorsoPoseFlag_; }
  void setEnableTorsoPoseTargetTrajectories(bool flag) { enableTorsoPoseFlag_ = flag; }
  const bool getEnableBaseTrack() const { return enableBaseTrackFlag_; }
  void setEnableBaseTrack(bool flag) { enableBaseTrackFlag_ = flag; }

protected:
  virtual void modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState, TargetTrajectories& targetTrajectories,
                                ModeSchedule& modeSchedule) override;
  
  // ruckig 轨迹生成相关
  // cmdPose
  void calcRuckigTrajWithCmdPose(double initTime, const vector_t &targetBasePose);
  void generatePoseTargetWithRuckig(double initTime, double finalTime, double dt);
  void resetCmdPoseRuckig(double initTime, const vector_t& initState, bool rePlanning);
  // cmdVel
  void calcRuckigTrajWithCmdVel(double initTime, const vector_t &targetBaseVel);
  void generateVelTargetBaseWithRuckig(double initTime, double finalTime, double dt, const vector_t &initState);
  void generateVelTargetWithRuckig(double initTime, double finalTime, double dt);
  void resetCmdVelRuckig(double initTime, const vector_t& initState, bool rePlanning);
  // cmdEePose
  void calcRuckigTrajWithEePose(double initTime, const vector_t &targetArmEePose);
  void generateDualArmEeTargetWithRuckig(double initTime, double finalTime, double dt);
  void resetDualArmRuckig(double initTime, const vector_t& initState, bool rePlanning, LbArmControlMode desireMode);
  // cmdTorsoPose
  void calcRuckigTrajWithTorsoPose(double initTime, const vector_t &targetTorsoPose);
  void generateTorsoPoseTargetWithRuckig(double initTime, double finalTime, double dt);
  void resetTorsoPoseRuckig(double initTime, const vector_t& initState, bool rePlanning);
  void resetTorsoControlPoseWithRuckig(double initTime, const vector_t& initState);
  // cmdLegJoint
  void calcRuckigTrajWithLegJoint(double initTime, const vector_t &targetLegJoint);
  void resetLegJointRuckig(double initTime, const vector_t& initState, bool rePlanning);
  // cmdArmJoint
  void calcRuckigTrajWithArmJoint(double initTime, const vector_t &targetArmJoint);
  void resetArmJointRuckig(double initTime, const vector_t& initState, bool rePlanning);
  
  double targetYawPreProcess(double currentYaw, double targetYaw);
  void setChassisControl(scalar_t initTime, scalar_t finalTime, const vector_t& initState);
  void setArmControl(scalar_t initTime, scalar_t finalTime, const vector_t& initState);
  void setTorsoControl(scalar_t initTime, scalar_t finalTime, const vector_t& initState);
  void resetAllMpcTraj(scalar_t initTime, const vector_t& initState);
  
  // 辅助函数
  bool getControlModeIsChange(int currentMode)
  {
    static int preMode = 0;
    bool isChange{false};
    if(preMode != currentMode) isChange = true;
    preMode = currentMode;
    return isChange;
  }
  bool getLbArmControlModeIsChange(LbArmControlMode desiredMode)
  {
    static LbArmControlMode preMode;
    static bool isFirstRun = true;
    if(isFirstRun)
    {
      isFirstRun = false;
      preMode = desiredMode;
      return true;
    }
    bool isChange{false};
    if(preMode != desiredMode) isChange = true;
    preMode = desiredMode;
    return isChange;
  }

  LbArmControlMode handPoseCmdFrameToLbArmMode(int frame);

  // 获取当前末端位姿
  void getCurrentEeWorldPose(vector_t& EeState, const vector_t& initState);
  void getCurrentEeBasePose(vector_t& EeState, const vector_t& initState);
  void getCurrentTorsoPoseInBase(vector_t& torsoPose, const vector_t& initState);

  // 不同控制模式的执行函数
  void updateNoControl(double initTime, const TargetTrajectories& targetTrajectories, bool isChange);
  void updateArmOnlyControl(double initTime, double finalTime, const vector_t& initState, bool isChange);
  void updateBaseOnlyControl(double initTime, double finalTime, const vector_t& initState, bool isChange);
  void updateBaseArmControl(double initTime, double finalTime, const vector_t& initState, bool isChange);
  void updateArmEeOnlyControl(double initTime, double finalTime, const vector_t& initState, bool isChange);

private:

  ros::NodeHandle nodeHandle_;
  humanoid::TopicLogger *ros_logger_ = nullptr;
  const ManipulatorModelInfo info_;
  double baseDim_ = 0;

  // 配置参数文件路径
  std::string taskFile_;
  
  // 动力学库接口
  PinocchioInterface pinocchioInterface_;

  // 指令底盘速度
  bool isCmdVelUpdated_{false};
  bool isCmdVelTimeUpdate_{false};
  double lastCmdVelTime_ = 0.0;
  Eigen::Vector3d cmdVel_;
  Eigen::Vector3d currentCmdVel_;
  std::mutex cmdvel_mtx_;
  ros::Subscriber targetVelocitySubscriber_;

  // 世界系的指令底盘速度
  bool isCmdVelWorldUpdated_{false};
  Eigen::Vector3d cmdVelWorld_;
  Eigen::Vector3d currentCmdVelWorld_;
  std::mutex cmdvelWorld_mtx_;
  ros::Subscriber targetVelocityWorldSubscriber_;

  // 指令底盘位置
  bool isCmdPoseUpdated_{false};
  Eigen::Vector3d cmdPose_;
  Eigen::Vector3d currentCmdPose_;
  std::mutex cmdPose_mtx_;
  ros::Subscriber targetPoseSubscriber_;

  // 世界系的指令底盘位置
  bool isCmdPoseWorldUpdated_{false};
  Eigen::Vector3d cmdPoseWorld_;
  std::mutex cmdPoseWorld_mtx_;
  ros::Subscriber targetPoseWorldSubscriber_;
  ros::Publisher targetCmdPoseReachTimePub_;

  // 轮臂躯干相对位姿的运动指令
  Eigen::Vector3d initialTorsoPos_;
  Eigen::Vector4d initialTorsoQuat_;
  Eigen::VectorXd cmdTorsoPose_;
  Eigen::VectorXd currentTorsoPose_;
  std::mutex cmdTorsoPose_mtx_;
  bool isCmdTorsoPoseUpdated_{false};
  ros::Subscriber targetTorsoPoseSubscriber_;
  ros::Publisher targetTorsoPoseReachTimePub_;
  bool torsoModeFlag_{true}; // true: 笛卡尔控制模式, false: 关节控制模式

  // 双臂末端执行器位姿指令 (x,y,z,qx,qy,qz,qw)
  vector_t left_arm_traj_pose_;
  vector_t right_arm_traj_pose_;
  std::mutex armPose_mtx_;
  bool isCmdDualArmPoseUpdated_{false};
  ros::Subscriber armEndEffectorSubscriber_;
  ros::Publisher armEndEffectorReachTimePub_;

  // 手臂关节轨迹指令
  bool enableQuickJointControl_{false};
  vector_t arm_joint_traj_;
  vector_t arm_init_joint_traj_;
  std::mutex armJoint_mtx_;
  ros::Subscriber arm_joint_traj_sub_;
  ros::Publisher targetArmJointReachTimePub_;

  // 躯干下肢的关节轨迹指令
  bool isCmdLegJointUpdated_{false};
  vector_t lb_leg_traj_;
  std::mutex lbLegJoint_mtx_;
  ros::Subscriber lb_leg_joint_traj_sub_;
  ros::Publisher targetLegJointReachTimePub_;

  // 分别保存左右臂的关节轨迹（弧度）以及是否将关节角作为期望输入
  vector_t left_arm_joint_traj_;
  vector_t right_arm_joint_traj_;
  LbArmControlMode desireMode_ = LbArmControlMode::JointSpace;  

  // MPC控制模式相关
  int currentMpcControlMode_{0};  // 0: NoControl, 1: ArmOnly, 2: BaseOnly, 3: BaseArm
  std::mutex controlMode_mtx_;
  ros::ServiceServer controlModeServiceServer_;
  ros::ServiceServer getMpcControlModeServiceServer_;
  ros::ServiceServer changeArmControlService_;
  ros::ServiceServer get_arm_control_mode_service_;
  ros::Publisher mpcControlModePub_;
  ros::Publisher mpcConstraintUsagePub_;

  // 关节控制默认为外部控制模式
  LbArmControlServiceMode currentArmControlMode_ = LbArmControlServiceMode::EXTERN_CONTROL; 

  // 多种约束所需轨迹相关
  TargetTrajectories stateInputTargetTrajectories_;
  TargetTrajectories torsoTargetTrajectories_;
  TargetTrajectories eeTargetTrajectories_;

  // 多种约束所需轨迹相关
  bool enableEeFlag_{true};
  bool enableEeLocalFlag_{true};
  bool enableArmJointTrackFlag_{false};
  bool enableLegJointTrackFlag_{false};
  bool enableTorsoPoseFlag_{false};
  bool enableBaseTrackFlag_{true};

  // 规划器周期
  double ruckigDt_{0.0};
  
  // cmdPose规划器
  std::shared_ptr<cmdPosePlannerWithRuckig> cmdPosePlannerRuckigPtr_;
  double plannerInitialTime_{0.0};
  Eigen::VectorXd prevTargetPose_;
  Eigen::VectorXd prevTargetVel_;
  Eigen::VectorXd prevTargetAcc_;

  // cmdVel规划器
  std::shared_ptr<cmdVelPlannerWithRuckig> cmdVelPlannerRuckigPtr_;
  double cmdVel_plannerInitialTime_{0.0};
  Eigen::VectorXd cmdVel_prevTargetPose_;
  Eigen::VectorXd cmdVel_prevTargetVel_;
  Eigen::VectorXd cmdVel_prevTargetAcc_;

  vector_t wheel_move_spd_;  // x, y, yaw
  vector_t wheel_move_acc_;  // x, y, yaw
  vector_t wheel_move_jerk_;  // x, y, yaw

  // 双臂轨迹规划器, 姿态的输入和输出均为Zyx欧拉角形式
  std::shared_ptr<cmdPosePlannerWithRuckig> cmdDualArmEePlannerRuckigPtr_;
  double cmdDualArm_plannerInitialTime_{0.0};
  Eigen::VectorXd cmdDualArm_prevTargetPose_;
  Eigen::VectorXd cmdDualArm_prevTargetVel_;
  Eigen::VectorXd cmdDualArm_prevTargetAcc_;
  
  vector_t dualArm_move_spd_;
  vector_t dualArm_move_acc_;
  vector_t dualArm_move_jerk_;

  // 躯干笛卡尔规划器, 姿态的输入和输出均为Zyx欧拉角形式
  std::shared_ptr<cmdPosePlannerWithRuckig> torsoPosePlannerRuckigPtr_;
  double torsoPose_plannerInitialTime_{0.0};
  Eigen::VectorXd torsoPose_prevTargetPose_;
  Eigen::VectorXd torsoPose_prevTargetVel_;
  Eigen::VectorXd torsoPose_prevTargetAcc_;

  vector_t torsoPose_move_spd_;
  vector_t torsoPose_move_acc_;
  vector_t torsoPose_move_jerk_;

  // 下肢关节规划器, 单位: rad
  std::shared_ptr<cmdPosePlannerWithRuckig> legJointPlannerRuckigPtr_;
  double legJoint_plannerInitialTime_{0.0};
  Eigen::VectorXd legJoint_prevTargetPose_;
  Eigen::VectorXd legJoint_prevTargetVel_;
  Eigen::VectorXd legJoint_prevTargetAcc_;

  vector_t legJoint_move_spd_;
  vector_t legJoint_move_acc_;
  vector_t legJoint_move_jerk_;

  // 上肢关节规划器, 单位: rad
  std::shared_ptr<cmdPosePlannerWithRuckig> armJointPlannerRuckigPtr_;
  double armJoint_plannerInitialTime_{0.0};
  Eigen::VectorXd armJoint_prevTargetPose_;
  Eigen::VectorXd armJoint_prevTargetVel_;
  Eigen::VectorXd armJoint_prevTargetAcc_;

  vector_t armJoint_move_spd_;
  vector_t armJoint_move_acc_;
  vector_t armJoint_move_jerk_;
};
} // namespace mobile_manipulator
} // namespace ocs2