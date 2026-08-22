#pragma once

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <atomic>
#include <algorithm>
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
#include <kuavo_msgs/accessIkSolve.h>
#include <kuavo_msgs/eePoseReachError.h>
#include <kuavo_msgs/sensorsData.h>
#include <leju_mobile_base_msgs/BaseCmdVelStatus.h>
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

#include "humanoid_wheel_interface/motion_planner/InverseKinematics.h"

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
  void setInitialTorsoPos(void);
  
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
  bool checkTargetPoseReachableService(kuavo_msgs::accessIkSolve::Request &req, kuavo_msgs::accessIkSolve::Response &res);
  bool eePoseReachErrorService(kuavo_msgs::eePoseReachError::Request &req, kuavo_msgs::eePoseReachError::Response &res);

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
  // 肩部关节的紧约束
  const bool getEnableShoulderTight() const { return enableShoulderTightFlag_[0] || enableShoulderTightFlag_[1]; }
  const bool getEnableShoulderTightForArm(int armIdx) const { return enableShoulderTightFlag_[armIdx]; }
  void setEnableShoulderTight(bool flag,int armIdx) { enableShoulderTightFlag_[armIdx] = flag; }
  void setEnableShoulderTighteningForArm(int armIdx, bool flag) { enableShoulderTightFlag_[armIdx] = flag; }
  // 肩部收紧松弛系数 alpha (1=收紧/锁肩, 0=释放), 供 shoulderTightenCost 自适应权重使用
  const scalar_t getShoulderTightAlpha(int armIdx) const { return shoulderTightAlpha_[armIdx]; }
  void setShoulderTightAlpha(int armIdx, scalar_t alpha) { shoulderTightAlpha_[armIdx] = alpha; }
  // α 低通滤波: alpha = smooth * alpha_prev + (1 - smooth) * alpha_raw, 防止权重抖动
  void updateShoulderTightAlpha(int armIdx, scalar_t alphaRaw) {
    const scalar_t raw = std::max(0.0, std::min(1.0, alphaRaw));
    shoulderTightAlpha_[armIdx] =
        shoulderTightAlphaSmooth_ * shoulderTightAlpha_[armIdx] + (1.0 - shoulderTightAlphaSmooth_) * raw;
  }
  const scalar_t getShoulderTightAlphaSmooth() const { return shoulderTightAlphaSmooth_; }
  void setShoulderTightAlphaSmooth(scalar_t smooth) { shoulderTightAlphaSmooth_ = smooth; }

  // 每周期计算肩部收紧 α: H_joint(肘腕限位余量) × H_task(末端位移) → σ 合成 → 低通滤波, 返回滤波后 α
  scalar_t computeShoulderTightAlpha(int armIdx, scalar_t initTime, const vector_t& initState);

  // 末端跟踪优先级调整的相关函数
  const bool getIsFocusEeStatus() const { return isFocusEe_; }
  void setIsFocusEeStatus(bool flag) { isFocusEe_ = flag; }
  const double getTorsoOriFocusScale() const { return torsoOriBoxScale_; }
  void setTorsoOriFocusScale(double scale) { torsoOriBoxScale_ = scale; }

  // torso/z 方向 relax barrier 的运行时参数（通过话题更新）
  scalar_t getFocusZMu() const
  {
    std::lock_guard<std::mutex> lock(focusZ_mtx_);
    return focusZMu_;
  }
  scalar_t getFocusZDelta() const
  {
    std::lock_guard<std::mutex> lock(focusZ_mtx_);
    return focusZDelta_;
  }

  /** task.info: torsoBoxSoftCost.position.focus_z_barrier.use_focus_z；为 false 时 z 与 x/y 一样用 position.unFocus_barrier */
  bool getUseFocusZ() const
  {
    std::lock_guard<std::mutex> lock(focusZ_mtx_);
    return useFocusZ_;
  }

protected:
  virtual void modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState, TargetTrajectories& targetTrajectories,
                                ModeSchedule& modeSchedule) override;

  // 3791 reset-to-state 辅助：command storage / 全部 Ruckig 锚定到指定状态（软暂停恢复干净重启）。
  // 状态与 FK 缓存全部由 resetAllMpcToState 以局部变量传入，不再经成员载体。
  void overwriteCommandStorageWithState(const vector_t& state, const vector_t& torsoPose6D, const vector_t& eeState);
  // EE ruckig 锚点仅为占位：恢复后首条 EE 指令被 forceEeReanchorOnNextCmd_ 强制重锚定，不消费此值
  void resetAllRuckigToState(scalar_t initTime, const vector_t& state, const vector_t& torsoPose6D,
                             const vector_t& eeState);

  // 3791: 软暂停恢复时整体重置 RM —— storage + 全部 Ruckig 锚定到指定状态（冻结姿态）。
  // 躯干/ee 位姿用 FK 现场计算，锚当前而非初始位。由 /mobile_manipulator_reset_to_state
  // 话题（pending 标志）触发，在 solver 重启后 modifyReferences 首拍消费。
  void resetAllMpcToState(scalar_t initTime, const vector_t& state);

  /// 锚定状态中关节保持给定值，底盘跟随当前实际位姿
  vector_t jointsWithLiveBase(const vector_t& state) const;

  // 通用 helper：将 pose 型 Ruckig 规划器重置到指定状态（current=target=state，vel/acc=0）
  template<typename PlannerPtr>
  void resetPoseRuckigToState(PlannerPtr& planner, scalar_t initTime, const vector_t& state,
                              scalar_t& plannerInitialTime,
                              vector_t& prevPose, vector_t& prevVel, vector_t& prevAcc);

  // ruckig 轨迹生成相关
  // cmdPose
  void calcRuckigTrajWithCmdPose(double initTime, const vector_t &targetBasePose, double desiredTime = 0.0);
  void generatePoseTargetWithRuckig(double initTime, double finalTime, double dt);
  void resetCmdPoseRuckig(double initTime, const vector_t& initState, bool rePlanning);
  // cmdVel
  void calcRuckigTrajWithCmdVel(double initTime, const vector_t &targetBaseVel);
  void generateVelTargetBaseWithRuckig(double initTime, double finalTime, double dt, const vector_t &initState);
  void generateVelTargetWithRuckig(double initTime, double finalTime, double dt, const vector_t &initState);
  void resetCmdVelRuckig(double initTime, const vector_t& initState, bool rePlanning);
  // cmdEePose
  void calcRuckigTrajWithEePose(int armIdx, double initTime, const vector_t &targetArmEePose, double desiredTime = 0.0);
  void generateDualArmEeTargetWithRuckig(int armIdx, double initTime, double finalTime, double dt);
  void resetDualArmRuckig(int armIdx, double initTime, const vector_t& initState, bool rePlanning, LbArmControlMode desireMode);
  void resetDualArmRuckig(int armIdx, double initTime, const vector_t& initState, bool rePlanning, LbArmControlMode desireMode, const vector_t &targetArmEePose);
  vector_t getDualArmRuckigInitialPose(LbTimedPosCmdType cmdType, const vector_t& initState, const vector_t& targetArmEePose);
  // cmdTorsoPose
  void calcRuckigTrajWithTorsoPose(double initTime, const vector_t &targetTorsoPose, double desiredTime = 0.0);
  void generateTorsoPoseTargetWithRuckig(double initTime, double finalTime, double dt);
  void resetTorsoPoseRuckig(double initTime, const vector_t& initState, bool rePlanning);
  void resetTorsoControlPoseWithRuckig(double initTime, const vector_t& initState, bool anchorOpenLoopStart);
  // torso vel/delta: integrate onto cmdTorsoPose_ (sole open-loop final target);
  // Ruckig only tracks it. dt = clamp(ΔinitTime) for sim/real period variation.
  void applyTorsoVelDeltaCommands(scalar_t initTime);
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
  void resetAllMpcTraj(scalar_t initTime, const vector_t& initState, bool anchorOpenLoopStart);
  void resetAllMpcTrajAndTarget(scalar_t initTime, const vector_t& initState, bool anchorOpenLoopStart);
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

  void computeErrorArmEeIsReachTarget(int armIdx, double initTime, double finalTime, const vector_t& initState);

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
  bool isResetTorsoRePlanning_{false};
  vector_t resetTorsoOpenLoopStart4_{vector_t::Zero(4)};  // snap at service call (#3973 VR race)
  vector_t resetTorsoMpcStart4_{vector_t::Zero(4)};       // MPC track snap at service call (#3991)
  Eigen::VectorXd torsoResetMaxVel_;
  ros::ServiceServer resetTorsoStatusServiceServer_;

  // 配置参数文件路径
  std::string taskFile_;
  
  // 动力学库接口
  PinocchioInterface pinocchioInterface_;

  // 检测末端可达性相关
  ros::ServiceServer checkTargetPoseReachableServiceServer_;

  // 声明多线程spinner
  ros::AsyncSpinner asyncSpinner_;

  // 底盘运动模式相关
  bool baseCmdVelStatus_{true};
  ros::Subscriber base_cmd_vel_status_sub_;

  // 判断末端位姿运动后的误差
  ros::Subscriber sensors_data_sub_;
  vector_t currentSensorDataJointPos_;
  bool isEeMotionComplete_[2]{false, false}; // 判断是否运动已抵达目标
  vector_t eeError_[2]; // 末端位姿误差
  ros::ServiceServer eePoseReachErrorServiceServer_;

  // 末端控制优先级相关
  bool desiredFocusEe_{false};
  bool leftArmJointTrigger_{true};   // 从其他模式切换到关节模式则触发为 true
  bool rightArmJointTrigger_{true};
  double torsoOriBoxScale_{1.0};    // 躯干的boxConstrait中姿态的上下界, 高优先级和低优先级之间需要存在倍数关系

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
  ros::Publisher shoulderTightAlphaPub_;   // 发布肩部收紧 α [左臂, 右臂] 便于 rqt 监控

  // 躯干速度 / 相对位移指令（开环最终目标只在 cmdTorsoPose_）
  // Twist 布局与 /cmd_lb_torso_pose 一致: linear.x/z, angular.z=yaw, angular.y=pitch
  // 保持语义对齐底盘 /cmd_vel：
  //   isCmdTorsoVelUpdated_ sticky; isCmdTorsoVelTimeUpdate_ refreshes lastCmdTorsoVelTime_;
  //   timeout 0.3s zeros cmdTorsoVel_; explicit zero clears sticky flag.
  Eigen::VectorXd cmdTorsoVel_;          // 4D: vx, vz, vyaw, vpitch
  Eigen::VectorXd cmdTorsoDelta_;        // 4D oneshot: dx, dz, dyaw, dpitch
  std::mutex cmdTorsoVel_mtx_;
  std::mutex cmdTorsoDelta_mtx_;
  bool isCmdTorsoVelUpdated_{false};     // sticky，对齐 isCmdVelUpdated_
  bool isCmdTorsoVelTimeUpdate_{false};  // 对齐 isCmdVelTimeUpdate_
  bool isCmdTorsoDeltaUpdated_{false};   // oneshot，用后即清
  scalar_t lastCmdTorsoVelTime_{0.0};    // 对齐 lastCmdVelTime_
  scalar_t lastTorsoVelIntegrateTime_{0.0};  // for Δt integrate (scheme A)
  bool hasTorsoVelIntegrateTime_{false};
  static constexpr scalar_t kTorsoVelTimeout_{0.3};
  ros::Subscriber targetTorsoVelSubscriber_;
  ros::Subscriber targetTorsoDeltaSubscriber_;
  ros::Publisher torsoOpenLoopStatePub_;    // /torso_open_loop_state: 4D pose [x,z,yaw,pitch]

  // 双臂末端执行器位姿指令 (x,y,z,qx,qy,qz,qw)
  vector_t cmd_arm_zyx_[2]; // [0]: 左臂, [1]: 右臂, 包含位置和欧拉角
  std::mutex armPose_mtx_[2]; // [0]: 左臂, [1]: 右臂
  bool isCmdDualArmPoseUpdated_[2]{false, false}; // [0]: 左臂, [1]: 右臂
  // 3791: resetAllMpcToState（软暂停恢复）/ resetAllMpcTrajAndTarget（home 归位）后置位，
  // 强制下一条 EE 指令走 isChange=true 重锚定（resetDualArmRuckig 用活状态+指令自身
  // 坐标系），不消费可能滞留的 EE ruckig 旧状态。免疫 preMode static 滞留、reset 时序
  // 异常、归位窗口内手臂被其他路径挪走导致的锚点帧混淆/位姿拉回（3791 甩飞、home+b 跳变）
  bool forceEeReanchorOnNextCmd_[2]{false, false}; // [0]: 左臂, [1]: 右臂
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
  vector_t initialJointTarget_;
  bool isCmdLegJointUpdated_{false};
  double cmdLegJointDesiredTime_{0.0};
  vector_t lb_leg_traj_;
  std::mutex lbLegJoint_mtx_;
  ros::Subscriber lb_leg_joint_traj_sub_;
  ros::Publisher targetLegJointReachTimePub_;

  // 用于记录末端笛卡尔模式的 focus 对象, true 为末端, false 为躯干
  bool isFocusEe_{true};
  ros::Subscriber set_focus_ee_sub_;

  // torso position z 方向 relax barrier 参数与开关（通过话题 /mobile_manipulator_focus_z 更新开关）
  mutable std::mutex focusZ_mtx_;
  scalar_t focusZMu_{0.0};
  scalar_t focusZDelta_{0.0};
  bool useFocusZ_{false};
  ros::Subscriber set_focus_z_sub_;

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

  // enable 状态（fail-safe：默认 false，未收到信号时不可控）。仅用于指令入口拒绝
  // （disable 期间丢弃一切动作指令）；冻结输出语义由 controller 承担。
  std::atomic<bool> enable_control_{false};
  ros::Subscriber enable_control_state_sub_;

  // 3791: 软暂停恢复的 reset-to-state 机制（话题写入 pending，modifyReferences 消费）
  ros::Subscriber resetToStateSub_;
  std::atomic<bool> is_reset_to_state_pending_{false};
  std::mutex reset_to_state_mtx_;
  vector_t reset_state_;
  bool isEnableControl() const { return enable_control_.load(std::memory_order_acquire); }

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
  bool enableShoulderTightFlag_[2]{false, false};                 // 肩部关节的紧约束
  ocs2::vector_t shoulderTightAlpha_{ocs2::vector_t::Constant(2, 1.0)};  // 肩部收紧松弛系数 [左臂, 右臂]
  scalar_t shoulderTightAlphaSmooth_{0.9};                      // α 低通滤波系数 (doc 建议 0.9~0.95)
  // 肩部收紧 α 调度参数 (对应 mpc_cost.md 自动松弛)
  scalar_t shoulderHThOn_{0.20};                                 // H_joint 锁肩阈值 (滞回): 余量高于此才锁肩
  scalar_t shoulderHThOff_{0.15};                                // H_joint 解锁阈值 (滞回): 余量低于此才解锁
  scalar_t shoulderTThOn_{0.25};                                 // H_task 锁肩阈值 (滞回): 位移低于此才锁肩
  scalar_t shoulderTThOff_{0.30};                                // H_task 解锁阈值 (滞回): 位移高于此才解锁
  scalar_t shoulderK1_{30.0};                                   // H_joint sigmoid 斜率
  scalar_t shoulderK2_{30.0};                                   // H_task sigmoid 斜率
  scalar_t shoulderDMax_{0.1};                                  // 典型大幅末端位移 [m]
  std::vector<std::vector<size_t>> shoulderRelaxStateIndices_;  // 每臂肘腕关节的状态下标 [左臂, 右臂]

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
  std::atomic<bool> mm_no_elbow_data_{false};

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

  // ik 可达性分析相关
  InverseKinematics ikSolverDiff_;

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