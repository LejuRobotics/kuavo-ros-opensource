#pragma once

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <atomic>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int8MultiArray.h>
#include <kuavo_msgs/twoArmHandPoseCmd.h>
#include <kuavo_msgs/changeTorsoCtrlMode.h>
#include <kuavo_msgs/changeArmCtrlMode.h>
#include <kuavo_msgs/setRuckigPlannerParams.h>
#include <kuavo_msgs/getLbTorsoInitialPose.h>
#include <kuavo_msgs/lbTimedPosCmd.h>
#include <kuavo_msgs/lbMultiTimedOfflineTraj.h>
#include <kuavo_msgs/lbMultiTimedPosCmd.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include "humanoid_wheel_interface/ManipulatorModelInfo.h"
#include "ocs2_robotic_tools/common/RotationTransforms.h"
#include "ocs2_pinocchio_interface/PinocchioInterface.h"
#include "humanoid_interface/common/TopicLogger.h"

#include "humanoid_wheel_interface/motion_planner/VelocityLimiter.h"
#include "humanoid_wheel_interface/motion_planner/cmdPosePlannerWithRuckig.h"
#include "humanoid_wheel_interface/motion_planner/cmdVelPlannerWithRuckig.h"
#include "humanoid_wheel_interface/motion_planner/posePlannerTimedScheduler.h"

#include "humanoid_wheel_interface/estimators/CentralDifferenceDifferentiator.h"
#include "humanoid_wheel_interface/estimators/ContinuousEulerAnglesFromMatrix.h"

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

// 轮臂的基于时间的指令类型
enum LbTimedPosCmdType {
  BASE_POS_WORLD_CMD = 0,         // 底盘世界系位置运动
  BASE_POS_LOCAL_CMD,             // 底盘局部系位置运动
  TORSO_POSE_CMD,                 // 躯干笛卡尔局部系运动
  LEG_JOINT_CMD,                  // 下肢关节运动
  /****************************************/
  LEFT_ARM_WORLD_CMD,             // 双臂笛卡尔世界系运动
  RIGHT_ARM_WORLD_CMD,            // 双臂笛卡尔世界系运动
  /****************************************/
  LEFT_ARM_LOCAL_CMD,             // 双臂笛卡尔局部系运动
  RIGHT_ARM_LOCAL_CMD,            // 双臂笛卡尔局部系运动
  /****************************************/
  LEFT_ARM_JOINT_CMD,             // 左臂关节运动
  RIGHT_ARM_JOINT_CMD,            // 右臂关节运动
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
  bool resetCmdVelRuckigService(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  void resetCmdPoseRuckigFromActualState(double initTime, const vector_t& initState, bool rePlanning);
  void resetCmdVelRuckigFromActualState(double initTime, const vector_t& initState, bool rePlanning);
  bool setRuckigPlannerParamsService(kuavo_msgs::setRuckigPlannerParams::Request &req, kuavo_msgs::setRuckigPlannerParams::Response &res);
  bool getLbTorsoInitialPoseService(kuavo_msgs::getLbTorsoInitialPose::Request &req, kuavo_msgs::getLbTorsoInitialPose::Response &res);
  bool setLbTimedPosCmdService(kuavo_msgs::lbTimedPosCmd::Request &req, kuavo_msgs::lbTimedPosCmd::Response &res);
  bool setLbMultiTimedPosCmdService(kuavo_msgs::lbMultiTimedPosCmd::Request &req, kuavo_msgs::lbMultiTimedPosCmd::Response &res);
  bool setLbMultiTimedOfflineTrajService(kuavo_msgs::lbMultiTimedOfflineTraj::Request &req, kuavo_msgs::lbMultiTimedOfflineTraj::Response &res);
  bool setLbOfflineTrajEnableService(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  bool setLbResetTorsoService(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

  // 多个约束轨迹的操作函数
  void trimTargetTrajectoriesBeforeTime(scalar_t startTime);
  void publishTargetTrajectoriesNear(scalar_t initTime);
  vector_t targetTrajToPose6D(const TargetTrajectories& Traj, scalar_t initTime);
  vector_t targetTorsoTrajToPose6DContinous(const TargetTrajectories& Traj, scalar_t initTime);
  vector_t targetEeTrajToPose6DContinous(const TargetTrajectories& Traj, scalar_t initTime, int eeInx);

  // 多种约束轨迹的获取函数
  const TargetTrajectories& getStateInputTargetTrajectories() const { return stateInputTargetTrajectories_; }
  const TargetTrajectories& getTorsoTargetTrajectories() const { return torsoTargetTrajectories_; }
  // const TargetTrajectories& getEeTargetTrajectories() const { return eeTargetTrajectories_; }
  const TargetTrajectories& getEeTargetTrajectories(int armIdx) const { return eeTargetTrajectories_[armIdx]; }

  // 多种约束的使能函数 (全局setter同时设置两臂, per-arm版本支持独立控制)
  // EE世界系
  const bool getEnableEeTargetTrajectories() const { return enableEePerArm_[0] || enableEePerArm_[1]; }
  const bool getEnableEeTargetTrajectoriesForArm(int armIdx) const { return enableEePerArm_[armIdx]; }
  void setEnableEeTargetTrajectories(bool flag) { enableEePerArm_[0] = flag; enableEePerArm_[1] = flag; }
  void setEnableEeTargetTrajectoriesForArm(int armIdx, bool flag) { enableEePerArm_[armIdx] = flag; }
  // EE局部系
  const bool getEnableEeTargetLocalTrajectories() const { return enableEeLocalPerArm_[0] || enableEeLocalPerArm_[1]; }
  const bool getEnableEeTargetLocalTrajectoriesForArm(int armIdx) const { return enableEeLocalPerArm_[armIdx]; }
  void setEnableEeTargetLocalTrajectories(bool flag) { enableEeLocalPerArm_[0] = flag; enableEeLocalPerArm_[1] = flag; }
  void setEnableEeTargetLocalTrajectoriesForArm(int armIdx, bool flag) { enableEeLocalPerArm_[armIdx] = flag; }
  // 手臂关节跟踪
  const bool getEnableArmJointTrack() const { return enableArmJointTrackPerArm_[0] || enableArmJointTrackPerArm_[1]; }
  const bool getEnableArmJointTrackForArm(int armIdx) const { return enableArmJointTrackPerArm_[armIdx]; }
  void setEnableArmJointTrack(bool flag) { enableArmJointTrackPerArm_[0] = flag; enableArmJointTrackPerArm_[1] = flag; }
  void setEnableArmJointTrackForArm(int armIdx, bool flag) { enableArmJointTrackPerArm_[armIdx] = flag; }
  // 其他约束 (保持不变)
  const bool getEnableLegJointTrack() const { return enableLegJointTrackFlag_; }
  void setEnableLegJointTrack(bool flag) { enableLegJointTrackFlag_ = flag; }
  const bool getEnableTorsoPoseTargetTrajectories() const { return enableTorsoPoseFlag_; }
  void setEnableTorsoPoseTargetTrajectories(bool flag) { enableTorsoPoseFlag_ = flag; }
  const bool getEnableBaseTrack() const { return enableBaseTrackFlag_; }
  void setEnableBaseTrack(bool flag) { enableBaseTrackFlag_ = flag; }

  // 末端跟踪优先级调整的相关函数
  const bool getIsFocusEeStatus() const { return isFocusEe_; }
  void setIsFocusEeStatus(bool flag) { isFocusEe_ = flag; }

protected:
  virtual void modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState, TargetTrajectories& targetTrajectories,
                                ModeSchedule& modeSchedule) override;
  
  // ruckig 轨迹生成相关
  // cmdPose
  void calcRuckigTrajWithCmdPose(double initTime, const vector_t &targetBasePose, double desiredTime = 0.0);
  void generatePoseTargetWithRuckig(double initTime, double finalTime, double dt);
  void resetCmdPoseRuckig(double initTime, const vector_t& initState, bool rePlanning);
  // cmdVel
  void calcRuckigTrajWithCmdVel(double initTime, const vector_t &targetBaseVel);
  void generateVelTargetBaseWithRuckig(double initTime, double finalTime, double dt, const vector_t &initState);
  void generateVelTargetWithRuckig(double initTime, double finalTime, double dt);
  void resetCmdVelRuckig(double initTime, const vector_t& initState, bool rePlanning);
  // cmdEePose
  void calcRuckigTrajWithEePose(int armIdx, double initTime, const vector_t &targetArmEePose, double desiredTime = 0.0);
  void generateDualArmEeTargetWithRuckig(int armIdx, double initTime, double finalTime, double dt);
  void resetDualArmRuckig(int armIdx, double initTime, const vector_t& initState, bool rePlanning, LbArmControlMode desireMode);
  void resetDualArmRuckig(int armIdx, double initTime, const vector_t& initState, bool rePlanning, LbArmControlMode desireMode, const vector_t &targetArmEePose);
  // cmdTorsoPose
  void calcRuckigTrajWithTorsoPose(double initTime, const vector_t &targetTorsoPose, double desiredTime = 0.0);
  void generateTorsoPoseTargetWithRuckig(double initTime, double finalTime, double dt);
  void resetTorsoPoseRuckig(double initTime, const vector_t& initState, bool rePlanning);
  void resetTorsoControlPoseWithRuckig(double initTime, const vector_t& initState);
  // cmdLegJoint
  void calcRuckigTrajWithLegJoint(double initTime, const vector_t &targetLegJoint, double desiredTime = 0.0);
  void resetLegJointRuckig(double initTime, const vector_t& initState, bool rePlanning);
  // cmdArmJoint
  void calcRuckigTrajWithArmJoint(int armIdx, double initTime, const vector_t &targetArmJoint, double desiredTime = 0.0);
  void resetArmJointRuckig(int armIdx, double initTime, const vector_t& initState, bool rePlanning);
  
  double targetYawPreProcess(double currentYaw, double targetYaw);
  void setChassisControl(scalar_t initTime, scalar_t finalTime, const vector_t& initState);
  void setArmControl(int armIdx, scalar_t initTime, scalar_t finalTime, const vector_t& initState);
  void setTorsoControl(scalar_t initTime, scalar_t finalTime, const vector_t& initState);
  void resetAllMpcTraj(scalar_t initTime, const vector_t& initState);
  void resetAllMpcTrajAndTarget(scalar_t initTime, const vector_t& initState);
  void updateTimedSchedulerCurrentState(scalar_t initTime, const vector_t& initState);
  void updateTimedSchedulerTargetTraj(void);
  void updateTimedOfflineTraj(scalar_t initTime, scalar_t finalTime);
  void updateIndexRuckigPlanner(int plannerIndex, double desireTime, const Eigen::VectorXd& cmd_vec);
  
  // 辅助函数
  bool getControlModeIsChange(int currentMode)
  {
    static int preMode = 0;
    bool isChange{false};
    if(preMode != currentMode) isChange = true;
    preMode = currentMode;
    return isChange;
  }
  bool getLbArmControlModeIsChange(int armIdx, LbArmControlMode desiredMode)
  {
    static LbArmControlMode preMode[10];
    static bool isFirstRun[10] = {true};
    // 确保 armIdx 在有效范围内
    if (armIdx < 0 || armIdx >= 10) {
        return false;  // 或者处理错误
    }
    if(isFirstRun[armIdx])
    {
      isFirstRun[armIdx] = false;
      preMode[armIdx] = desiredMode;
      return true;
    }
    bool isChange = (preMode[armIdx] != desiredMode);
    preMode[armIdx] = desiredMode;
    return isChange;
  }

  int SrvRequestIndexToCmdType(int planner_index)
  {
    switch(planner_index)
    {
      case 0: 
        return static_cast<int>(LbTimedPosCmdType::BASE_POS_WORLD_CMD);
      case 2: 
        return static_cast<int>(LbTimedPosCmdType::TORSO_POSE_CMD);
      case 3:
        return static_cast<int>(LbTimedPosCmdType::LEG_JOINT_CMD);
      case 4:
        return static_cast<int>(LbTimedPosCmdType::LEFT_ARM_WORLD_CMD);
      case 5:
        return static_cast<int>(LbTimedPosCmdType::RIGHT_ARM_WORLD_CMD);
      case 6:
        return static_cast<int>(LbTimedPosCmdType::LEFT_ARM_JOINT_CMD);
      case 7:
        return static_cast<int>(LbTimedPosCmdType::RIGHT_ARM_JOINT_CMD);
      default:
        return -1;
    }
  }

  LbArmControlMode handPoseCmdFrameToLbArmMode(int frame);

  // 获取当前末端位姿
  void getCurrentEeWorldPose(vector_t& EeState, const vector_t& initState);
  void getCurrentEeBasePose(vector_t& EeState, const vector_t& initState);
  void getCurrentTorsoPoseInBase(vector_t& torsoPose, const vector_t& initState);

  // 获取当前末端位姿（采用四元数插值计算增量, 且输出为6d位姿(Zyx欧拉角)， 保证万向锁附近不跳变）
  void getCurrentEeWorldPoseContinuous(vector_t& EeState, const vector_t& initState);
  void getCurrentEeBasePoseContinuous(vector_t& EeState, const vector_t& initState);
  void getCurrentTorsoPoseInBaseContinuous(vector_t& torsoPose, const vector_t& initState);
  void getCurrentTorsoPoseInBasePitchYaw(vector_t& torsoPose, const vector_t& initState);

  // 发布所关注的笛卡尔位姿
  void publishMultiPointPose_World(const vector_t& initState);
  void publishMultiPointPose_Local(const vector_t& initState);

  // 不同控制模式的执行函数
  // void updateNoControl(double initTime, const TargetTrajectories& targetTrajectories, bool isChange);
  // void updateArmOnlyControl(double initTime, double finalTime, const vector_t& initState, bool isChange);
  // void updateBaseOnlyControl(double initTime, double finalTime, const vector_t& initState, bool isChange);
  void updateBaseArmControl(double initTime, double finalTime, const vector_t& initState, bool isChange);
  // void updateArmEeOnlyControl(double initTime, double finalTime, const vector_t& initState, bool isChange);

private:

  const int singleArmJointDim_;
  ros::NodeHandle nodeHandle_;
  humanoid::TopicLogger *ros_logger_ = nullptr;
  const ManipulatorModelInfo info_;
  double baseDim_ = 0;
  double initTime_ = 0.0;
  double finalTime_ = 0.0;
  vector_t initState_;
  // 下肢重置相关
  double resetTorsoTime_{0.0};
  double resetTorsoInitTime_{0.0};
  bool isResetTorso_{false};
  Eigen::VectorXd torsoResetMaxVel_;
  ros::ServiceServer resetTorsoStatusServiceServer_;

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
  double cmdPoseDesiredTime_{0.0};
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
  std::mutex cmdTorsoPose_mtx_;
  bool isCmdTorsoPoseUpdated_{false};
  double cmdTorsoPoseDesiredTime_{0.0};
  ros::Subscriber targetTorsoPoseSubscriber_;
  ros::Publisher targetTorsoPoseReachTimePub_;
  bool torsoModeFlag_{true}; // true: 笛卡尔控制模式, false: 关节控制模式
  ros::ServiceServer getLbTorsoInitialPoseServiceServer_;

  // 双臂末端执行器位姿指令 (x,y,z,qx,qy,qz,qw)
  vector_t cmd_arm_zyx_[2]; // [0]: 左臂, [1]: 右臂, 包含位置和欧拉角
  std::mutex armPose_mtx_[2]; // [0]: 左臂, [1]: 右臂
  bool isCmdDualArmPoseUpdated_[2]{false, false}; // [0]: 左臂, [1]: 右臂
  double cmdDualArmPoseDesiredTime_[2]{0.0, 0.0}; // [0]: 左臂, [1]: 右臂
  ros::Subscriber armEndEffectorSubscriber_;
  ros::Publisher armEndEffectorReachTimePub_[2]; // [0]: 左臂, [1]: 右臂

  // 手臂关节轨迹指令
  vector_t arm_joint_traj_[2]; // [0]: 左臂, [1]: 右臂
  vector_t arm_init_joint_traj_;
  std::mutex armJoint_mtx_[2]; // [0]: 左臂, [1]: 右臂
  bool isCmdArmJointUpdated_[2]{false, false}; // [0]: 左臂, [1]: 右臂
  double cmdArmJointDesiredTime_[2]{0.0, 0.0}; // [0]: 左臂, [1]: 右臂
  ros::Subscriber arm_joint_traj_sub_;
  ros::Publisher targetArmJointReachTimePub_[2]; // [0]: 左臂, [1]: 右臂

  // 躯干下肢的关节轨迹指令
  bool isCmdLegJointUpdated_{false};
  double cmdLegJointDesiredTime_{0.0};
  vector_t lb_leg_traj_;
  std::mutex lbLegJoint_mtx_;
  ros::Subscriber lb_leg_joint_traj_sub_;
  ros::Publisher targetLegJointReachTimePub_;

  // 用于记录末端笛卡尔模式的 focus 对象, true 为末端, false 为躯干
  bool isFocusEe_{true};
  ros::Subscriber set_focus_ee_sub_;

  // 分别保存左右臂的关节轨迹（弧度）以及是否将关节角作为期望输入
  vector_t left_arm_joint_traj_;
  vector_t right_arm_joint_traj_;
  LbArmControlMode desireMode_[2] = {LbArmControlMode::WorldFrame, 
                                     LbArmControlMode::WorldFrame}; // [0]: 左臂, [1]: 右臂

  // MPC控制模式相关
  int currentMpcControlMode_{0};  // 0: NoControl, 1: ArmOnly, 2: BaseOnly, 3: BaseArm
  std::mutex controlMode_mtx_;
  ros::ServiceServer controlModeServiceServer_;
  ros::ServiceServer getMpcControlModeServiceServer_;
  ros::ServiceServer changeArmControlService_;
  ros::ServiceServer get_arm_control_mode_service_;
  ros::ServiceServer resetCmdVelRuckigServiceServer_;
  ros::Publisher mpcControlModePub_;
  ros::Publisher mpcConstraintUsagePub_;
  ros::Publisher modifyReferenceTimePub_;

  // 速度下发开关状态
  std::atomic<bool> use_vel_control_{true};
  ros::Subscriber vel_control_state_sub_;

  // 关节控制默认为外部控制模式
  LbArmControlServiceMode currentArmControlMode_ = LbArmControlServiceMode::EXTERN_CONTROL; 

  // 多种约束所需轨迹相关
  TargetTrajectories stateInputTargetTrajectories_;
  TargetTrajectories torsoTargetTrajectories_;
  TargetTrajectories eeTargetTrajectories_[2]; // [0]: 左臂, [1]: 右臂

  // 多种约束所需轨迹相关
  // bool enableEeFlag_{true};
  // bool enableEeLocalFlag_{true};
  // bool enableArmJointTrackFlag_{false};
  bool enableEePerArm_[2]{true, true};                  // EE世界系约束 [左臂, 右臂]
  bool enableEeLocalPerArm_[2]{true, true};              // EE局部系约束 [左臂, 右臂]
  bool enableArmJointTrackPerArm_[2]{false, false};      // 手臂关节跟踪 [左臂, 右臂]
  bool enableLegJointTrackFlag_{false};
  bool enableTorsoPoseFlag_{false};
  bool enableBaseTrackFlag_{true};

  // 规划器周期
  double ruckigDt_{0.0};

  // 规划器限制更改服务
  ros::ServiceServer setRuckigPlannerParamsServiceServer_;

  // 离线轨迹跟踪相关
  ros::ServiceServer setLbMultiTimedOfflineTrajServiceServer_;
  ros::ServiceServer setLbOfflineTrajEnableServiceServer_;
  TargetTrajectories torsoOfflineTraj_;
  bool isTorsoOfflineTrajUpdate_{false};
  TargetTrajectories armEeOfflineTraj_[2];
  bool isArmEeOfflineTrajUpdate_[2]{false, false};
  bool eeOfflineTrajFrame_[2]{false, false};
  bool isOfflineTrajUpdate_{false};
  double isofflineTrajUpdateStartTime_{0.0};
  bool offlineTrajDisable_{true};
  bool trajFrameUpdate_{false};

  // 多规划器时间同步相关
  ros::ServiceServer setLbTimedPosCmdServiceServer_;
  ros::ServiceServer setLbMultiTimedPosCmdServiceServer_;
  posePlannerTimedScheduler timedPlannerScheduler_;
  std::vector<bool> isTimedPlannerUpdated_;
  std::vector<double> desireTime_;
  std::vector<Eigen::VectorXd> timedCmdVec_;
  std::vector<std::unique_ptr<std::mutex>> timedCmdVecMtx_;
  bool isUpdateTimedTarget_{false};
  
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

  // 当前实际机器人状态
  vector_t currentActualState_;
  std::mutex currentActualState_mtx_;

  vector_t wheel_move_spd_;  // x, y, yaw
  vector_t wheel_move_acc_;  // x, y, yaw
  vector_t wheel_move_jerk_;  // x, y, yaw

  // 双臂轨迹规划器, 姿态的输入和输出均为Zyx欧拉角形式
  std::shared_ptr<cmdPosePlannerWithRuckig> cmdDualArmEePlannerRuckigPtr_[2]; // [0]: 左臂, [1]: 右臂
  double cmdDualArm_plannerInitialTime_[2]{0.0, 0.0};
  Eigen::VectorXd cmdDualArm_prevTargetPose_[2];
  Eigen::VectorXd cmdDualArm_prevTargetVel_[2];
  Eigen::VectorXd cmdDualArm_prevTargetAcc_[2];
  
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
  std::shared_ptr<cmdPosePlannerWithRuckig> armJointPlannerRuckigPtr_[2]; // [0]: 左臂, [1]: 右臂
  double armJoint_plannerInitialTime_[2]{0.0, 0.0};
  Eigen::VectorXd armJoint_prevTargetPose_[2];
  Eigen::VectorXd armJoint_prevTargetVel_[2];
  Eigen::VectorXd armJoint_prevTargetAcc_[2];

  vector_t armJoint_move_spd_;
  vector_t armJoint_move_acc_;
  vector_t armJoint_move_jerk_;
};
} // namespace mobile_manipulator
} // namespace ocs2