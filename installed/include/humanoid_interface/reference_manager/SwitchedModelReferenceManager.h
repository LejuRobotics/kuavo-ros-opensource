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

#include <ocs2_core/thread_support/Synchronized.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>

#include "humanoid_interface/foot_planner/InverseKinematics.h"
#include "humanoid_interface/foot_planner/SwingTrajectoryPlanner.h"
#include "humanoid_interface/foot_planner/SingleStepPlanner.h"
#include "humanoid_interface/gait/GaitSchedule.h"
#include "humanoid_interface/gait/MotionPhaseDefinition.h"

#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include "ocs2_pinocchio_interface/PinocchioInterface.h"
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <kuavo_msgs/changeArmCtrlMode.h>
#include <kuavo_msgs/singleStepControl.h>
#include <kuavo_msgs/changeTorsoCtrlMode.h>
#include "kuavo_msgs/footPoseTargetTrajectoriesSrv.h"
#include "kuavo_msgs/footPose6DTargetTrajectoriesSrv.h"

#include <ocs2_msgs/mpc_target_trajectories.h>
#include "std_srvs/SetBool.h"
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include "humanoid_interface/common/TopicLogger.h"
#include <std_srvs/Empty.h>
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>

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
                                const CentroidalModelInfo& info);

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
  
  bool getArmControlModeCallback(kuavo_msgs::changeArmCtrlMode::Request &req, kuavo_msgs::changeArmCtrlMode::Response &res)
  {
    res.result = true;
    res.mode = currentArmControlMode_;
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

  void armTargetTrajectoriesCallback(const ocs2_msgs::mpc_target_trajectories::ConstPtr &msg);
  TargetTrajectories interpolateArmTarget(scalar_t startTime, const vector_t& currentArmState, const vector_t& newDesiredArmState, scalar_t maxSpeed);

  void publishFootContactPoint();

  void publishFootDesiredPoint(scalar_t time);

  std::pair<Eigen::Vector3d, Eigen::Vector3d> generate_steps(const Eigen::Vector3d& torso_pos, const double torso_yaw, const double foot_bias = 0.1);

  void checkSingleStepControlAndStop();

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
  struct baseTrackingQ{
    matrix_t Stance = matrix_t::Zero(24, 24);;
    matrix_t Walk = matrix_t::Zero(24, 24);;
    matrix_t Jump = matrix_t::Zero(24, 24);;
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
  std::string dynamic_qr_file_;
  bool dynamic_qr_flag_ = false;

  PinocchioInterface pinocchioInterface_;
  const CentroidalModelInfo& info_;
  std::unique_ptr<PinocchioEndEffectorKinematics> endEffectorKinematicsPtr_;

  ocs2_msgs::mpc_target_trajectories armTargetTrajectoriesMsg_;

  ros::Subscriber targetVelocitySubscriber_;
  ros::Subscriber targetPoseSubscriber_;
  ros::Subscriber targetPoseWorldSubscriber_;
  ros::Subscriber armTargetTrajectoriesSubscriber_;
  ros::Subscriber poseTargetTrajectoriesSubscriber_;
  ros::Subscriber footPoseTargetTrajectoriesSubscriber_;
  ros::Subscriber footPoseWorldTargetTrajectoriesSubscriber_;
  ros::Subscriber footPose6DWorldTargetTrajectoriesSubscriber_;
  ros::Subscriber footPose6DTargetTrajectoriesSubscriber_;
  ros::Subscriber eef_wrench_sub_;
  ros::Subscriber fullBodyTargetTrajectoriesSubscriber_;
  ros::Subscriber estContactStateSubscriber_;
  ros::Subscriber slope_planning_sub_;
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
  bool isCmdPoseCached = false;
  bool poseTargetUpdated_ = false;
  bool armTargetUpdated_ = false;
  bool isFirstRun_ = true;
  bool isFirstVelPub_ = true;
  ArmControlMode currentArmControlMode_ = ArmControlMode::AUTO_SWING;
  ArmControlMode newArmControlMode_ = ArmControlMode::AUTO_SWING;
  TorsoControlMode torsoControlMode_ = TorsoControlMode::SIX_DOF;
  bool isArmControlModeChanged_ = false;
  bool isArmControlModeChangedTrigger_ = false;
  bool update_stop_single_step_ = false;

  vector_t TargetState_, initTargetState_;
  scalar_array_t lastTimeTrajectoryWithVel;
  vector_array_t lastStateTrajectoryWithVel;
  TargetTrajectories fullBodyTargetTrajectories_;
  TargetTrajectories fullBodyArmTargetTrajectories_;
  int feetJointNums_ = 12;
  int armJointNums_ = 10;// will replace in initialize
  int armRealDof_ = 14;
  
  std::mutex cmdvel_mtx_;
  std::mutex cmdPose_mtx_;
  std::mutex cmdPoseWorld_mtx_;
  std::mutex armTargetCommanded_mtx_;

  vector_t currentCmdVel_ = vector_t::Zero(6);
  vector_t currentCmdPose_ = vector_t::Zero(6);
  vector_t cachedCmdPoseInWorldFrame_ = vector_t::Zero(6);
  bool ismdPoseInWorldFrameCached_ = false;

  ocs2::scalar_array_t c_relative_base_limit_{0.4, 0.15, 0.3, 0.4, 0.4, 0.4};
  double cmd_threshold = 0.02;

  InverseKinematics inverseKinematics_;
  TargetTrajectories currentArmTargetTrajectories_;
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
  double terrainHeight_ = 0.0;
  double terrainHeightPrev_ = 0.0;
  double fullbodyScheduleStartTime_ = 0.0;
  double fullbodyScheduleEndTime_ = 0.0;
  std::string last_gait_name_="stance";
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

  vector_t last_init_target_state;
};

}  // namespace humanoid
}  // namespace ocs2
