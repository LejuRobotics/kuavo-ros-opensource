/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#pragma once

#include "humanoid_interface/foot_planner/SwingTrajectoryPlanner.h"
#include "humanoid_interface/common/ModelSettings.h"

#include <ocs2_core/thread_support/Synchronized.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>

#include "humanoid_interface/foot_planner/InverseKinematics.h"
#include "humanoid_interface/foot_planner/SingleStepPlanner.h"
#include "humanoid_interface/gait/GaitSchedule.h"
#include "humanoid_interface/gait/MotionPhaseDefinition.h"

#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include "ocs2_pinocchio_interface/PinocchioInterface.h"
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include "kuavo_common/common/common.h"
#include <kuavo_msgs/changeArmCtrlMode.h>
#include <kuavo_msgs/singleStepControl.h>
#include <kuavo_msgs/changeTorsoCtrlMode.h>
#include <kuavo_msgs/ExecuteArmAction.h>
#include "kuavo_msgs/footPoseTargetTrajectoriesSrv.h"
#include "kuavo_msgs/footPose6DTargetTrajectoriesSrv.h"
#include "kuavo_msgs/kuavoModeSchedule.h"

#include <ocs2_msgs/mpc_target_trajectories.h>
#include "std_srvs/SetBool.h"
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include "humanoid_interface/common/TopicLogger.h"
#include <std_srvs/Empty.h>
#include <std_msgs/Float64.h>
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <map>

#define X_MAX_SINGLE_STEP_SIZE 0.15
#define Y_MAX_SINGLE_STEP_SIZE 0.05
#define Z_MAX_SINGLE_STEP_SIZE 0.05
#define YAW_MAX_SINGLE_STEP_SIZE 3.14/3

namespace ocs2 {
namespace humanoid {

  

  enum TorsoControlMode {
    SIX_DOF = 0,// x,y,z, yaw, pitch, roll
    ZYP, // height, yaw, pitch
    ZP // height, pitch
  };

  enum FrameType{
    Local,
    World
  };
    

/**
 * Manages the ModeSchedule and the TargetTrajectories for switched model.
 */
class SwitchedModelReferenceManager : public ReferenceManager {
 public:
  SwitchedModelReferenceManager(std::shared_ptr<GaitSchedule> gaitSchedulePtr, 
                                std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr, 
                                const PinocchioInterface& pinocchioInterface,
                                const CentroidalModelInfo& info,
                                const ModelSettings& modelSettings,
                                RobotVersion rbVersion = RobotVersion(4, 5));

  ~SwitchedModelReferenceManager() override = default;

  void setModeSchedule(const ModeSchedule& modeSchedule) override;

  contact_flag_t getContactFlags(scalar_t time) const;

  const std::shared_ptr<GaitSchedule>& getGaitSchedule() const { return gaitSchedulePtr_; }

  const std::shared_ptr<SwingTrajectoryPlanner>& getSwingTrajectoryPlanner() { return swingTrajectoryPtr_; }
  
  // 获取当前的swingplanner的所有向量数据
  std::vector<scalar_t> getSwingPlannerMultipliers() override
  {
    return swingTrajectoryPtr_->saveTrajectoryToVector();
  };
  // 从回放数据中恢复swingplanner的规划器所有向量数据，和modeschedule
  void resetReference(const std::vector<scalar_t> multipliers, const ModeSchedule& modeSchedule) override
  {
    swingTrajectoryPtr_->loadTrajectoryFromVector(multipliers, modeSchedule);
  };


  void setSwingHeight(scalar_t toe_height,  scalar_t heel_height) override;
  void observationStateCallback(const vector_t& state) override;

  void setChangeQTime(const double& time) { QTime = time; }
  void setChangeRTime(const double& time) { RTime = time; }
  double getChangeQTime(void) { return QTime; }
  double getChangeRTime(void) { return RTime; }

  void setMatrixQ(const matrix_t& Q) { gait_Q_ = Q; }
  void setMatrixR(const matrix_t& R) { gait_R_ = R; }
  matrix_t getMatrixQ(void) { return gait_Q_; }
  matrix_t getMatrixR(void) { return gait_R_; }
  bool enablePitchLimit() const { return enable_pitch_limit_; }
  void setEnablePitchLimit(bool enable) { enable_pitch_limit_ = enable; }

  inline bool getUpdatedR() const override{ return updated_R_; }
  inline bool getUpdatedQ() const override{ return updated_Q_; }
  inline void setUpdatedR(bool flag) override{ updated_R_= flag; }
  inline void setUpdatedQ(bool flag) override{ updated_Q_= flag; }

  void setMatrixQByGaitPair(const std::string &gait_name, const scalar_t &time);
  void setupSubscriptions(std::string nodeHandleName = "humanoid") override;

  vector_t getLocalPlannerVel(double initTime, const TargetTrajectories& targetTraj);
  vector_t getLocalPlannerVel(double initTime);

  void setPinocchioEndEffectorKinematics(const PinocchioEndEffectorKinematics &endEffectorKinematics)
  {
    endEffectorKinematicsPtr_.reset(endEffectorKinematics.clone());
    endEffectorKinematicsPtr_->setPinocchioInterface(pinocchioInterface_);
  }

  // VR waist control public getter
  bool isVRWaistControlEnabled() const { return vrWaistControlEnabled_; }

 private:
  double calTerrainHeight(const contact_flag_t& contact_flags, const feet_array_t<vector3_t>& feet_pos);
  void processFullBodySchedule(const vector_t& initState, FullBodySchedule& fullBodySchedule);
  void setArmTrajectory(scalar_t current_time, scalar_t startTime, FullBodySchedule& fullBodySchedule);

  TargetTrajectories generateTargetwithfullBodySchedule(scalar_t initTime, scalar_t scheduleStartTime, scalar_t scheduleEndTime, const vector_t& initState, 
                                                                     const TargetTrajectories& targetTrajectories, const FullBodySchedule& fullBodySchedule, ModeSchedule& modeSchedule);


  void modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState, TargetTrajectories& targetTrajectories,
                        ModeSchedule& modeSchedule) override;
  TargetTrajectories generateTargetwithVelcmd(scalar_t initTime, scalar_t finalTime, const vector_t &initState,
                                                                           TargetTrajectories &targetTrajectories, ModeSchedule &modeSchedule, const vector_t &cmdVel);
  TargetTrajectories generateTargetwithModeSchedule(scalar_t initTime, scalar_t finalTime, const vector_t &initState,
                                                    const TargetTrajectories &targetTrajectories, const ModeSchedule &modeSchedule);
  TargetTrajectories generateTargetwithModeScheduleWorld(scalar_t initTime, scalar_t finalTime, const vector_t &initState,
                                                         const TargetTrajectories &targetTrajectories, const ModeSchedule &modeSchedule);

  TargetTrajectories generateTargetwithPoscmd(scalar_t initTime, const vector_t &initState,
                                              TargetTrajectories &targetTrajectories, ModeSchedule &modeSchedule, const vector_t &cmdPos);

  TargetTrajectories generateTargetwithPoscmdInCurrentPose(scalar_t initTime, const vector_t &initState, TargetTrajectories &targetTrajectories, ModeSchedule &modeSchedule, const vector_t &cmdPos); 

  TargetTrajectories generateTargetAsCurrent(scalar_t initTime, scalar_t finalTime, const vector_t &initState);

  bool checkAndApplyCommandLine(scalar_t initTime, scalar_t finalTime, const vector_t& initState, vector_t& cmdVel);

  bool checkCmdPoseAndApplyCommandLine(scalar_t initTime, scalar_t finalTime, const vector_t& initState, vector_t& cmdPose);

  
  void calculateJointRef(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
              TargetTrajectories& targetTrajectories, const ModeSchedule& modeSchedule);

  bool footPoseTargetTrajectoriesSrvCallback(kuavo_msgs::footPoseTargetTrajectoriesSrv::Request &req, kuavo_msgs::footPoseTargetTrajectoriesSrv::Response &res);
  bool footPose6DTargetTrajectoriesSrvCallback(kuavo_msgs::footPose6DTargetTrajectoriesSrv::Request &req, kuavo_msgs::footPose6DTargetTrajectoriesSrv::Response &res);
  
  void processFootPose6DTargetTrajectories(const kuavo_msgs::footPose6DTargetTrajectories::ConstPtr &msg, FrameType frameType);

  bool armControlModeSrvCallback(kuavo_msgs::changeArmCtrlMode::Request &req, kuavo_msgs::changeArmCtrlMode::Response &res);

  bool torsoControlModeSrvCallback(kuavo_msgs::changeTorsoCtrlMode::Request &req, kuavo_msgs::changeTorsoCtrlMode::Response &res);

  bool enablePitchLimitCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

  bool pitchLimitStatusCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  
  // VR waist control methods
  void setVRWaistControlEnabled(bool enabled) { vrWaistControlEnabled_ = enabled; }
  bool vrWaistControlCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  
  // Height smooth transition methods
  void setHeightTransitionMaxSpeed(scalar_t speed) { heightTransitionMaxSpeed_ = speed; }
  void setHeightJumpThreshold(scalar_t threshold) { heightJumpThreshold_ = threshold; }
  void setHeightTransitionMinDuration(scalar_t duration) { heightTransitionMinDuration_ = duration; }
  void setHeightTransitionMaxDuration(scalar_t duration) { heightTransitionMaxDuration_ = duration; }
  
  // Pitch smooth transition methods
  void setPitchTransitionMaxSpeed(scalar_t speed) { pitchTransitionMaxSpeed_ = speed; }
  void setPitchJumpThreshold(scalar_t threshold) { pitchJumpThreshold_ = threshold; }
  
  // Transition status query methods
  bool isHeightTransitionActive() const { return heightSmoothTransitionActive_; }
  bool isArmExecuting() const { return isArmExecuting_; }
  scalar_t getHeightTransitionProgress() const {
    if (!heightSmoothTransitionActive_) return 1.0;
    scalar_t elapsedTime = ros::Time::now().toSec() - heightTransitionStartTime_;
    return std::min(1.0, elapsedTime / heightTransitionDuration_);
  }
  scalar_t getHeightTransitionDuration() const { return heightTransitionDuration_; }
  
  bool getArmControlModeCallback(kuavo_msgs::changeArmCtrlMode::Request &req, kuavo_msgs::changeArmCtrlMode::Response &res)
  {
    res.result = true;
    // 如果是RL控制模式下，使用newArmControlMode_，否则使用currentArmControlMode_
    if (is_rl_controller_ != 0.0) {
      res.mode = newArmControlMode_;
    } else {
      res.mode = currentArmControlMode_;
    }
    return true;
  };

  bool getCurrentGaitCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
  {
    res.success = gaitSchedulePtr_->getModeSchedule().existValidFootPose();
    if(res.success)
      res.message = "Current gait is Custom-Gait";
    else
      res.message = "Current gait is NOT Custom-Gait";
    return true;
  }
  
  bool stopSingleStepControlCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  bool singleStepControlCallback(kuavo_msgs::singleStepControl::Request &req, kuavo_msgs::singleStepControl::Response &res);

  bool loadDynamicQRCallback(kuavo_msgs::ExecuteArmAction::Request &req, kuavo_msgs::ExecuteArmAction::Response &res);

  void armTargetTrajectoriesCallback(const ocs2_msgs::mpc_target_trajectories::ConstPtr &msg);
  
  TargetTrajectories interpolateArmTarget(scalar_t startTime, const vector_t& currentArmState, const vector_t& newDesiredArmState, scalar_t maxSpeed);

  void publishFootContactPoint();

  void publishFootDesiredPoint(scalar_t time);

  std::pair<Eigen::Vector3d, Eigen::Vector3d> generate_steps(const Eigen::Vector3d& torso_pos, const double torso_yaw, const double foot_bias = 0.1);

  void checkSingleStepControlAndStop();

  void generateTargetwithTorsoMove(scalar_t initTime, const vector_t &initState, const vector_t &torsoDisplacement,
                                    const TargetTrajectories &targetTrajectories, vector_t &finalState, double &torso_max_time, const vector6_t &velocity_scale);

  std::shared_ptr<GaitSchedule> gaitSchedulePtr_;
  std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr_;

  double QTime = 0.0;
  double RTime = 0.0;

  matrix_t gait_Q_;
  matrix_t gait_R_;
  bool updated_Q_ = false;
  bool updated_R_ = false;
  // matrix_t Q_;

  void loadBaseTrackingQ(const std::string &dynamic_qr_file);
  void loadBaseTrackingR(const std::string &dynamic_qr_file);
  void loadDynamicQRMap(const std::string &dynamic_qr_file);  // 加载所有Q_dynamic_<gait_name>和R_dynamic_<gait_name>到map
  void setMatrixRByGaitPair(const std::string &gait_name, const scalar_t &time, bool all_stance);
  
  // 维度缩减函数（参考HumanoidInterface的实现）
  matrix_t initializeInputCostWeightDynamic(const std::string &taskFile, const std::string &fieldName);
  matrix_t initializeStateCostWeightDynamic(const std::string &taskFile, const std::string &fieldName);
  
  struct baseTrackingQ{
    matrix_t Stance = matrix_t::Zero(24, 24);;
    matrix_t StanceVRwaist = matrix_t::Zero(24, 24);;  // VR waist control Q matrix
    matrix_t Walk = matrix_t::Zero(24, 24);;
    matrix_t Jump = matrix_t::Zero(24, 24);;
  };

  struct baseTrackingR{
    matrix_t Stance;
    matrix_t Walk;
  };

  inline double normalizedYaw(double yaw)
  {
    while (yaw > M_PI)
      yaw -= 2*M_PI;
    while(yaw < -M_PI)
      yaw += 2*M_PI;
    return yaw;
  };

  std::vector<vector_t> getFeetPoses(const vector_t& initState);
  std::vector<vector_t> getFeetPoses(const std::vector<vector3_t> &feetPositions);

  vector3_t getComPos(const vector_t& state);

  baseTrackingQ baseTrackingQ_;
  baseTrackingR baseTrackingR_;
  std::string dynamic_qr_file_;
  bool dynamic_qr_flag_ = false;
  bool dynamic_r_set_ = false;  // Flag to indicate if R matrix was manually set via service, skip all_stance override
  matrix_t dynamic_R = matrix_t::Zero(24, 24);
  matrix_t dynamic_Q = matrix_t::Zero(24, 24);
  
  // Map存储gait_name到QR矩阵的映射，避免在回调函数中进行文件IO
  struct DynamicQRPair {
    matrix_t Q;
    matrix_t R;
  };
  std::map<std::string, DynamicQRPair> dynamic_qr_map_;
  PinocchioInterface pinocchioInterface_;
  const CentroidalModelInfo& info_;
  std::unique_ptr<PinocchioEndEffectorKinematics> endEffectorKinematicsPtr_;
  
  // 用于雅可比矩阵计算的参数
  std::vector<std::string> contactNames3DoF_;
  RobotVersion rbVersion_;

  ocs2_msgs::mpc_target_trajectories armTargetTrajectoriesMsg_;

  ros::Subscriber targetVelocitySubscriber_;
  ros::Subscriber targetPoseSubscriber_;
  ros::Subscriber targetPoseWorldSubscriber_;
  ros::Subscriber armTargetTrajectoriesSubscriber_;
  ros::Subscriber waistTargetTrajectoriesSubscriber_;
  ros::Subscriber poseTargetTrajectoriesSubscriber_;
  ros::Subscriber footPoseTargetTrajectoriesSubscriber_;
  ros::Subscriber footPoseWorldTargetTrajectoriesSubscriber_;
  ros::Subscriber footPose6DWorldTargetTrajectoriesSubscriber_;
  ros::Subscriber footPose6DTargetTrajectoriesSubscriber_;
  ros::Subscriber eef_wrench_sub_;
  ros::Subscriber fullBodyTargetTrajectoriesSubscriber_;
  ros::Subscriber estContactStateSubscriber_;
  ros::Subscriber slope_planning_sub_;
  ros::Subscriber is_rl_controller_sub_;
  ros::Publisher footContactPointPublisher_;
  ros::Publisher footDesiredPointPublisher_;
  ros::Publisher gaitTimeNamePublisher_;
  ros::Publisher armTargetCommandedPublisher_;
  ros::Publisher isCustomGaitPublisher_;
  ros::Publisher singleStepModePublisher_;
  ros::Publisher currentFootPosesPublisher_;
  ros::Publisher currentFootCenterPosePublisher_;
  ros::Publisher armTargetPublisher_;
  ros::ServiceServer change_arm_control_service_;
  ros::ServiceServer get_arm_control_mode_service_;
  ros::ServiceServer singleStepControlService_;
  ros::ServiceServer change_torso_control_service_;
  ros::ServiceServer footPoseTargetTrajectoriesService_;
  ros::ServiceServer footPose6DTargetTrajectoriesService_;
  ros::ServiceServer current_mode_service_;
  ros::ServiceServer stopSingleStepControlService_;
  ros::ServiceServer enable_pitch_limit_service_;
  ros::ServiceServer pitch_limit_status_service_;
  ros::ServiceServer vr_waist_control_service_;  // VR waist control service
  ros::ServiceServer load_dynamic_qr_service_;  // Service to load dynamic Q and R matrices based on gait name
  ros::Publisher isArmExecutingPublisher_;
  ros::Publisher modeSchedulePublisher_;

  vector_t cmdVel_;
  vector_t cmdPose_;
  vector_t cmdPoseWorld_;
  vector_t tempCmdPose_;
  TopicLogger *ros_logger_ = nullptr;
  int estContactState_ = ModeNumber::SS;
  bool isContactStateUpdated_ = false;

  scalar_t cmdHeight_;
  scalar_t cmdPitch_;
  bool velCmdUpdated_ = false;
  bool PoseCmdUpdated_ = false;
  bool PoseWorldCmdUpdated_ = false;
  bool isArmExecuting_ = false;
  bool isCmdPoseCached = false;
  bool poseTargetUpdated_ = false;
  bool armTargetUpdated_ = false;
  bool waistTargetUpdated_ = false;
  bool vrWaistControlEnabled_ = false;  // VR waist control flag
  bool vrWaistControlEnabledPrev_ = false;  // VR waist control flag  
  bool isFirstRun_ = true;
  bool isFirstVelPub_ = true;
  
  // VR高度平滑切换相关变量
  bool heightSmoothTransitionActive_ = false;  // 是否正在进行高度平滑过渡
  scalar_t heightTransitionStartTime_ = 0.0;   // 平滑过渡开始时间
  scalar_t heightTransitionDuration_ = 2.0;    // 平滑过渡持续时间（秒，动态计算）
  scalar_t heightBeforeTransition_ = 0.0;      // 过渡前的高度
  scalar_t heightAfterTransition_ = 0.0;       // 过渡后的目标高度
  scalar_t heightJumpThreshold_ = 0.05;        // 高度跳变阈值（米），超过此值触发平滑过渡
  scalar_t heightTransitionMaxSpeed_ = 0.25;    // 高度过渡最大速度（米/秒）
  scalar_t heightTransitionMinDuration_ = 0.5; // 最小过渡时间（秒）
  scalar_t heightTransitionMaxDuration_ = 5.0; // 最大过渡时间（秒）
  
  // Pitch平滑切换相关变量
  scalar_t pitchBeforeTransition_ = 0.0;       // 过渡前的pitch角度
  scalar_t pitchAfterTransition_ = 0.0;        // 过渡后的目标pitch角度
  scalar_t pitchJumpThreshold_ = 5.0 * M_PI / 180.0;  // Pitch跳变阈值（弧度），默认5度
  scalar_t pitchTransitionMaxSpeed_ = 20.0 * M_PI / 180.0;  // Pitch过渡最大角速度（弧度/秒），默认20度/秒
  ArmControlMode currentArmControlMode_ = ArmControlMode::AUTO_SWING;
  ArmControlMode newArmControlMode_ = ArmControlMode::AUTO_SWING;
  TorsoControlMode torsoControlMode_ = TorsoControlMode::SIX_DOF;
  double is_rl_controller_ = 0.0;  // RL控制器标志
  double prev_is_rl_controller_ = 0.0;  // 上一次的RL控制器标志，用于检测切换
  bool isArmControlModeChanged_ = false;
  bool isArmControlModeChangedTrigger_ = false;
  bool isCalcArmControlModeChangedTime_ = false;
  scalar_t arm_mode_change_start_time_ = -1.0;  // 模式切换开始时间，-1表示未开始切换
  scalar_t min_arm_mode_change_time_ = 0.5;  // 最小模式切换时间（秒）
  bool update_stop_single_step_ = false;

  bool begin_step_gait = false;
  scalar_t customGait_start_time = 0;
  scalar_t customGait_end_time = 0;

  vector_t TargetState_, initTargetState_;
  scalar_array_t lastTimeTrajectoryWithVel;
  vector_array_t lastStateTrajectoryWithVel;
  TargetTrajectories fullBodyTargetTrajectories_;
  TargetTrajectories fullBodyArmTargetTrajectories_;
  int feetJointNums_ = 12;
  int armJointNums_ = 10;// will replace in initialize
  int armRealDof_ = 14;
  int waistNums_ = 1;
  
  std::mutex cmdvel_mtx_;
  std::mutex cmdPose_mtx_;
  std::mutex cmdPoseWorld_mtx_;
  std::mutex armTargetCommanded_mtx_;
  std::mutex waistTargetCommanded_mtx_;

  vector_t currentCmdVel_ = vector_t::Zero(6);
  vector_t currentCmdPose_ = vector_t::Zero(6);
  vector_t cachedCmdPoseInWorldFrame_ = vector_t::Zero(6);
  vector_t currentState_ = vector_t::Zero(24);  // 存储当前状态，用于获取torso yaw角
  double currentTorsoYaw_ = 0.0; // 存储当前躯干的偏航角
  double currentTorsoRoll_ = 0.0; // 存储当前躯干的角度
  vector_t joyWaist_ = vector_t::Zero(waistNums_);
  bool ismdPoseInWorldFrameCached_ = false;

  ocs2::scalar_array_t c_relative_base_limit_{0.4, 0.15, 0.2, 0.4, 0.3, 0.4};
  // velocity_scale: x, z, pitch, yaw用0.35，y用0.15
  vector6_t torso_velocity_scale_;
  double cmd_threshold = 0.02;

  InverseKinematics inverseKinematics_;
  TargetTrajectories currentArmTargetTrajectories_;
  TargetTrajectories currentWaistTargetTrajectories_;
  TargetTrajectories currentArmTargetTrajectoriesWithAllJoints_;
  BufferedValue<TargetTrajectories> armTargetTrajectories_;
  BufferedValue<TargetTrajectories> armFullDofTargetTrajectories_;

  BufferedValue<TargetTrajectories> poseTargetTrajectories_;
  BufferedValue<vector_t> armWrenchBuffer_;

  int cntMPC_ = 0;
  ros::NodeHandle nodeHandle_;
  bool update_foot_trajectory_ = false;
  bool update_foot_world_trajectory_ = false;
  bool update_full_body_trajecory_ = false;
  FootPoseSchedule footPoseSchedule_;
  FootPoseSchedule footPoseWorldSchedule_;
  FullBodySchedule fullBodySchedule_;
  CentroidalModelRbdConversions rbdConversions_;
  ros::Time lastArmControlModeWarnTime_ = ros::Time(0);

  double arm_move_spd_{1.2};
  double waist_move_spd_{0.6};
  double terrainHeight_ = 0.0;
  double terrainHeightPrev_ = 0.0;
  double fullbodyScheduleStartTime_ = 0.0;
  double fullbodyScheduleEndTime_ = 0.0;
  std::string last_gait_name_="empty";
  bool last_all_stance_ = false;
  double vel_norm_{0};
  bool only_half_up_body_{false};

  TargetTrajectories fullBodyHeadTargetTrajectories_;  // 存储头部轨迹
  ros::Publisher headArrayPublisher_;  // 头部轨迹发布器
  
  Eigen::Vector2d lastFootCalibrationDiffXY_ = Eigen::Vector2d::Zero();
  // 处理全身轨迹
  void processFullBodyTrajectories(scalar_t initTime, scalar_t finalTime, scalar_t timeHorizon, 
                                  TargetTrajectories& targetTrajectories, const vector_t& initState,
                                  const feet_array_t<vector3_t>& feet_pos);

  // 在线校准轨迹
  void calibrateTrajectoryOnline(scalar_t initTime, scalar_t finalTime, 
                                const vector_t& currentState, 
                                TargetTrajectories& targetTrajectories,
                                const feet_array_t<vector3_t>& currentFeet);

  bool enable_slope_planning_ = false;
  bool enable_pitch_limit_ = false;
  bool is_roban_version_ = false; // 标识是否为roban版本

  vector_t last_init_target_state;
  scalar_t insert_time = 0.0;
  kuavo_msgs::kuavoModeSchedule createModeScheduleMsg(const ModeSchedule &modeSchedule, scalar_t initTime);
  SwingTrajectoryPlanner::Config swingTrajectoryPlannerConfig_;
};

}  // namespace humanoid
}  // namespace ocs2
