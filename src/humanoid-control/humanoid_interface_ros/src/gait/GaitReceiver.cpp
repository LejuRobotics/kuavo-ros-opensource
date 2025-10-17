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

#include "humanoid_interface_ros/gait/GaitReceiver.h"
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>
#include <std_msgs/Float64MultiArray.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_msgs/mpc_flattened_controller.h>

#include "humanoid_interface_ros/gait/ModeSequenceTemplateRos.h"
#include "kuavo_msgs/gaitTimeName.h"
#include "humanoid_interface/common/TopicLogger.h"
#include <ocs2_robotic_tools/common/RotationTransforms.h>


enum JoyAxis
{
  AXIS_LEFT_STICK_Y = 0,
  AXIS_LEFT_STICK_X = 1,
  AXIS_LEFT_LT = 2, // 1 -> (-1)
  AXIS_RIGHT_STICK_YAW = 3,
  AXIS_RIGHT_STICK_Z = 4,
  AXIS_RIGHT_RT = 5, // 1 -> (-1)
  AXIS_LEFT_RIGHT_TRIGGER = 6,
  AXIS_FORWARD_BACK_TRIGGER = 7
};
#define TARGET_REACHED_THRESHOLD 0.1
#define TARGET_REACHED_THRESHOLD_YAW 0.1
#define TARGET_REACHED_FEET_THRESHOLD 0.08

namespace
{
  constexpr double TARGET_REACHED_THRESHOLD_CMD_POSE = TARGET_REACHED_THRESHOLD;     // 0.05 m
  constexpr double TARGET_REACHED_THRESHOLD_YAW_CMD_POSE = TARGET_REACHED_THRESHOLD_YAW; // 0.15 rad
}

namespace ocs2
{
  namespace humanoid
  {
    TopicLogger *ros_logger_ = nullptr;

    std::vector<size_t> mode_to_scale = {ModeNumber::SH, ModeNumber::ST, ModeNumber::HS, ModeNumber::TS};

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    GaitReceiver::GaitReceiver(::ros::NodeHandle nodeHandle, std::shared_ptr<SwitchedModelReferenceManager> referenceManagerPtr, const std::string &robotName)
        : receivedGait_({0.0, 1.0}, {ModeNumber::SS}), gaitUpdated_(false), walkModeSequenceTemplate_({0.0, 1.0}, {ModeNumber::SS})
    {
      gaitSchedulePtr_ = referenceManagerPtr->getGaitSchedule();
      swingTrajectoryPlannerPtr_ = referenceManagerPtr->getSwingTrajectoryPlanner();
      mpcModeSequenceSubscriber_ = nodeHandle.subscribe(robotName + "_mpc_mode_schedule", 1, &GaitReceiver::mpcModeSequenceCallback, this,
                                                        ::ros::TransportHints().udp());
      mpcModeScaleSequenceSubscriber_ = nodeHandle.subscribe(robotName + "_mpc_mode_scale", 1, &GaitReceiver::mpcModeScaleSequenceCallback, this,
                                                             ::ros::TransportHints().tcp());
      mpcWalkHeelScaleSubscriber_ = nodeHandle.subscribe(robotName + "_mpc_walk_heel_scale", 1, &GaitReceiver::mpcWalkHeelSacleCallback, this, ::ros::TransportHints().tcp());
      auto stopStepNumCallback = [this](const std_msgs::Int32::ConstPtr &msg)
      { 
        this->stop_step_num_ = msg->data; // 相对步数
        this->stop_step_num_ += this->gaitSchedulePtr_->getSteps(); // 转换为绝对步数
        std::cout << "[GaitReceiver]: stop step num: " << this->stop_step_num_ << std::endl;
        this->stop_step_num_updated_ = true;
        this->gaitSchedulePtr_->setStopStepIdx(this->stop_step_num_);
      };
      mpcStopStepNumSubscriber_ = nodeHandle.subscribe<std_msgs::Int32>(robotName + "_mpc_stop_step_num", 1, stopStepNumCallback);

      std::string gaitCommandFile;
      nodeHandle.getParam("/gaitCommandFile", gaitCommandFile);
      walkModeSequenceTemplate_ = loadModeSequenceTemplate(gaitCommandFile, "walk", false);
      mpcGaitTimeAndNamePublisher_ = nodeHandle.advertise<kuavo_msgs::gaitTimeName>(robotName + "_mpc_gait_time_name", 1, false);
      ros_logger_ = new TopicLogger(nodeHandle);
      joy_sub_ = nodeHandle.subscribe("/joy", 2, &GaitReceiver::joyCallback, this);

      // cmdvel subscriber
      auto targetVelocityCallback = [this](const geometry_msgs::Twist::ConstPtr &msg)
      {
        
        cmdVel_[0] = msg->linear.x;
        cmdVel_[1] = msg->linear.y;
        cmdVel_[2] = msg->linear.z;
        cmdVel_[3] = msg->angular.z;
        velCmdUpdated_ = true;
        
      };
      targetVelocitySubscriber_ =
          nodeHandle.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, targetVelocityCallback);

      // cmdvel subscriber
      auto targetPoseCallback = [this](const geometry_msgs::Twist::ConstPtr &msg)
      {
        cmdPose_[0] = msg->linear.x;
        cmdPose_[1] = msg->linear.y;
        cmdPose_[2] = msg->linear.z;
        cmdPose_[3] = msg->angular.z;
        single_step_yaw_computed_ = false;
        PoseCmdUpdated_ = true;
        PoseCmdWorldUpdated_ = false;

        // std::cout << "[GaitReceiver]: VEL_POS CALLBACK PoseCmdUpdated_  " << PoseCmdUpdated_ << std::endl;
      };
      targetPoseSubscriber_ =
          nodeHandle.subscribe<geometry_msgs::Twist>("/cmd_pose", 1, targetPoseCallback);

      // cmdPoseWorld subscriber
      auto targetPoseWorldCallback = [this](const geometry_msgs::Twist::ConstPtr &msg)
      {
        cmdPoseWorld_[0] = msg->linear.x;
        cmdPoseWorld_[1] = msg->linear.y;
        cmdPoseWorld_[2] = msg->linear.z;
        cmdPoseWorld_[3] = msg->angular.z;
        single_step_yaw_computed_ = false;
        PoseCmdWorldUpdated_ = true;
        PoseCmdUpdated_ = false;

        // std::cout << "[GaitReceiver]: VEL_POS CALLBACK PoseCmdUpdated_  " << PoseCmdUpdated_ << std::endl;
      };
      targetPoseWorldSubscriber_ =
          nodeHandle.subscribe<geometry_msgs::Twist>("/cmd_pose_world", 1, targetPoseWorldCallback);


      auto_gait_mode_service_ = nodeHandle.advertiseService(robotName + "_auto_gait",&GaitReceiver::autoGaitModeSrvCallback, this);


      // feet subscriber
      auto feetCallback = [this](const std_msgs::Float64MultiArray::ConstPtr &feet_msg)
      {
        feet_pos_measured_ = Eigen::Map<const Eigen::VectorXd>(feet_msg->data.data(), feet_msg->data.size());
      };
      feet_sub_ = nodeHandle.subscribe<std_msgs::Float64MultiArray>("/humanoid_controller/swing_leg/pos_measured", 2, feetCallback);
      
      // policy subscriber  
      auto mpcPolicyCallback = [this](const ocs2_msgs::mpc_flattened_controller::ConstPtr &msg)
      {
        const auto targetTrajSize = msg->planTargetTrajectories.stateTrajectory.size();
        auto planTargetTrajectory = msg->planTargetTrajectories.stateTrajectory[targetTrajSize - 1].value;
        std::vector<double> planTargetTrajectoryDouble(planTargetTrajectory.begin(), planTargetTrajectory.end());
        auto last_target = Eigen::Map<const vector_t>(planTargetTrajectoryDouble.data(), planTargetTrajectoryDouble.size());
        current_target_ = last_target.segment<6>(6);
      };
      policy_sub_ = nodeHandle.subscribe<ocs2_msgs::mpc_flattened_controller>(
          robotName + "_mpc_policy",                            // topic name
          2,                                                    // queue length
          mpcPolicyCallback
      );
    }

    void GaitReceiver::joyCallback(const sensor_msgs::Joy::ConstPtr &joy_msg)
    {
      current_joystick_values_.head(4) << joy_msg->axes[AXIS_LEFT_STICK_X], joy_msg->axes[AXIS_LEFT_STICK_Y], joy_msg->axes[AXIS_RIGHT_STICK_Z], joy_msg->axes[AXIS_RIGHT_STICK_YAW];

    }
    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    void GaitReceiver::preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t &currentState,
                                    const ReferenceManagerInterface &referenceManager)
    {
      bool autoGaitMode = gaitSchedulePtr_->isAutoGaitEnabled();
      // std::cout << "autoGaitMode : " << autoGaitMode << std::endl;
      const auto timeHorizon = finalTime - initTime;
      if(stop_step_num_updated_ == true && gaitSchedulePtr_->getSteps() == stop_step_num_)
      {
        stop_step_num_updated_ = false;
        gaitUpdated_ = true;
        receivedGait_.switchingTimes = {0.0, 0.5};
        receivedGait_.modeSequence = {ModeNumber::SS};
        this->gaitSchedulePtr_->setStopStepIdx(0);
        std::cout << "[GaitReceiver]: Stop step num reached, setting gait to SS" << std::endl;
      }

      if (gaitUpdated_)
      {
        std::lock_guard<std::mutex> lock(receivedGaitMutex_);
        std::cerr << "[GaitReceiver]: Setting new gait after time " << finalTime << "\n";
        std::cerr << receivedGait_;gaitSchedulePtr_->setGaitName(receivedGait_);
        
        gaitUpdated_ = false;
        current_mode_end_time_ = 0.0;
        // 判断为walk模式才enable
        mode_scale_enabled_ = (receivedGait_ == walkModeSequenceTemplate_) ? true : false;
        if (mode_scale_enabled_ && cmdVel_[0] < 0.1)
          receivedGait_.replaceModes(mode_to_scale, ModeNumber::SS); // 先移除T H
        if (receivedGait_ == gaitSchedulePtr_->getGaitMap().at("stance"))//立刻切换stance
          gaitSchedulePtr_->modifyWalkModeSequenceTemplate(receivedGait_, initTime, timeHorizon, "stance");
        else// mode时间修改有风险，立即切换有几率出现崩溃
          gaitSchedulePtr_->modifyWalkModeSequenceTemplate(receivedGait_, initTime, timeHorizon, "walk");

        // pub msg
        kuavo_msgs::gaitTimeName msg;
        msg.start_time = gaitSchedulePtr_->getGgaitTimeName().first;
        msg.gait_name = gaitSchedulePtr_->getGgaitTimeName().second;
        mpcGaitTimeAndNamePublisher_.publish(msg);

        if (gaitSchedulePtr_->getGgaitTimeName().second != "stance")
          gaitSchedulePtr_->setAutoGaitEnabled(false); // 手动切换gait时，自动切换关闭
        else
          gaitSchedulePtr_->setAutoGaitEnabled(true);
      }else if (autoGaitMode)
      {
        if(!gaitSchedulePtr_->getModeSchedule().existValidFootPose(initTime))//单步模式时不切换gait
          autoGaitCheck(initTime, finalTime, currentState, referenceManager);
      }
      if (gait_scale_updated_)
      {
        std::lock_guard<std::mutex> lock(receivedGaitMutex_);
        std::cerr << "[GaitReceiver]: Scaling old gait after time " << finalTime << "\n";
        std::cerr << receivedGait_;
        gaitSchedulePtr_->scaleModeSequenceTemplate(gait_scale_, finalTime, timeHorizon); // TODO: check if this is correct
        gait_scale_updated_ = false;
        if (mode_scale_enabled_)
        {
          std::for_each(walkModeSequenceTemplate_.switchingTimes.begin(), walkModeSequenceTemplate_.switchingTimes.end(), [&](scalar_t &time)
                    { time *= gait_scale_; });
        }
      }

      // update walk_heel_scale
      if (mode_scale_enabled_)
      {

        // cal planner_v
        auto targetTraj = referenceManager.getTargetTrajectories();
        auto state0 = targetTraj.getDesiredState(initTime);
        auto state1 = targetTraj.getDesiredState(initTime + 0.015);
        vector3_t planner_v = (state1.segment<3>(6) - state0.segment<3>(6)) / 0.015;
        const Eigen::Matrix<scalar_t, 3, 1> planner_zyx(-state0(9), 0, 0);
        Eigen::Matrix<double, 3, 3> R_WB = getRotationMatrixFromZyxEulerAngles(planner_zyx);
        planner_v = R_WB * planner_v;
        // std::cout << "[GaitRecevier] planner_v: " << planner_v << std::endl;

        scalar_t planner_vx = planner_v(0);
        ros_logger_->publishVector("/humanoid_gait_receiver/planner_v", planner_v);
        auto maxAbs = [](double a, double b) {
            return (std::abs(a) > std::abs(b)) ? a : b;
        };
        double vx_desire = maxAbs(planner_vx, cmdVel_[0]);
        

        const Eigen::Matrix<scalar_t, 3, 1> zyx(-currentState(9), 0, 0);
        R_WB = getRotationMatrixFromZyxEulerAngles(zyx);
        vector3_t vel = currentState.segment<3>(0);
        scalar_t vx = (R_WB * vel)(0);
        

        auto default_cfg = swingTrajectoryPlannerPtr_->getDefaultConfig();
        // std::cout << "[GaitRecevier] vx: " << vx << std::endl;
        double abs_vx = std::abs(vx);
        double ratio = 0.0;
        if (abs_vx < default_cfg.deadBandVelocity)
        {
          ratio = 0.0;
        }
        else if (abs_vx <= default_cfg.heelToeMaxHeightVelocity)
        {
          ratio = (abs_vx - default_cfg.deadBandVelocity) /
                  (default_cfg.heelToeMaxHeightVelocity - default_cfg.deadBandVelocity);
        }
        else
        {
          ratio = 1.0;
        }
        ros_logger_->publishValue("/gait_receiver/walk_heel_scale_ratio", ratio);
        SwingTrajectoryPlanner::Config cfg = swingTrajectoryPlannerPtr_->getConfig();

        cfg.toeSwingHeight = default_cfg.toeSwingHeight * ratio;
        cfg.heelSwingHeight = default_cfg.heelSwingHeight * ratio;

        // 选中需要缩放的mode
        ModeSequenceTemplate old_mode_template = gaitSchedulePtr_->getModeSequenceTemplate();
        ModeSequenceTemplate mode_tmp = walkModeSequenceTemplate_;

        if (vx_desire < 0)// 后退修改mode
        {
          // mode_tmp.swapModes(ModeNumber::ST, ModeNumber::SH);
          // mode_tmp.swapModes(ModeNumber::TS, ModeNumber::HS);
          mode_tmp.replaceModes({ModeNumber::ST, ModeNumber::TS}, ModeNumber::SS);
          mode_tmp.replaceModes(ModeNumber::SH, ModeNumber::ST);
          mode_tmp.replaceModes(ModeNumber::HS, ModeNumber::TS);

          std::swap(cfg.toeSwingHeight, cfg.heelSwingHeight);
        }
        swingTrajectoryPlannerPtr_->updateConfig(cfg);
       
        if (std::abs(vx_desire) < 0.1 )
          mode_tmp.replaceModes(mode_to_scale, ModeNumber::SS);
        if (old_mode_template != mode_tmp)
        {
          current_mode_end_time_ = gaitSchedulePtr_->getModeEndTime(initTime); // 记录当前mode结束时间
          std::cout << "scaled mode: " << std::endl;
          for (size_t i = 0; i < mode_tmp.switchingTimes.size(); i++)
          {
            std::cout << "mode: " << mode_tmp.modeSequence[i];
            std::cout << "time: " << mode_tmp.switchingTimes[i] << std::endl;
          }
          gaitSchedulePtr_->modifyWalkModeSequenceTemplate(mode_tmp, current_mode_end_time_, timeHorizon, "walk");
        }
      }
    }
    void GaitReceiver::autoGaitCheck(scalar_t initTime, scalar_t finalTime, const vector_t &currentState, const ReferenceManagerInterface &referenceManager)
    {
      // 自动切换gait
      // std::cout << "[GaitRecevier] PASS THROGH " << std::endl;
      auto current_gait_name = gaitSchedulePtr_->getGaitName(initTime);
      auto last_gait_name = gaitSchedulePtr_->getLastGaitName();
      auto timeHorizon = finalTime - initTime;
      double cmd_threshold = 0.018;
      vector_t cmd_vector = vector_t::Zero(3);
      ros_logger_->publishValue("/humanoid/GaitReceiver/velCmdUpdate", velCmdUpdated_);
      ros_logger_->publishValue("/humanoid/GaitReceiver/PoseCmdUpdate", PoseCmdUpdated_);
      ros_logger_->publishValue("/humanoid/GaitReceiver/PoseCmdWorldUpdate", PoseCmdWorldUpdated_);

      if(velCmdUpdated_)
      {
        cmd_vector << cmdVel_[0], cmdVel_[1], cmdVel_[3];
      }
      else if (PoseCmdUpdated_) 
      {
        cmd_vector << cmdPose_[0], cmdPose_[1], cmdPose_[3];
      }
      else if (PoseCmdWorldUpdated_)
      {
        // trans from world to base
        Eigen::Vector3d delta_pose = (Eigen::Vector3d() << (cmdPoseWorld_.head(2) - currentState.segment<2>(6)), 0).finished();
        const Eigen::Vector3d current_zyx(currentState(9), 0, 0);
        Eigen::Matrix3d R_Ws = getRotationMatrixFromZyxEulerAngles(current_zyx);
        delta_pose = R_Ws.transpose() * delta_pose;
        // const Eigen::Vector3d target_zyx(cmdPoseWorld_[3], 0, 0);
        // Eigen::Matrix3d R_Wt = getRotationMatrixFromZyxEulerAngles(target_zyx);
        // Eigen::Matrix3d  R_st = R_Ws.transpose() * R_Wt;
        // auto euler_zyx = getZyxEulerAnglesFromRotationMatrix(R_st);
        double delta_yaw = normalizedYaw(normalizedYaw(cmdPoseWorld_[3]) - normalizedYaw(currentState(9)));
        // std::cout << "euler_z: " << euler_zyx(0) << ", delta_yaw: " << delta_yaw << std::endl;
        cmd_vector << delta_pose.head(2), delta_yaw;
      }
      else return;
      double target_yaw = 0;
      ros_logger_->publishValue("/humanoid/GaitReceiver/single_step_yaw_computed_", single_step_yaw_computed_);
      ros_logger_->publishValue("/humanoid/GaitReceiver/getFinalYawSingleStepMode", swingTrajectoryPlannerPtr_->getFinalYawSingleStepMode());
      if(PoseCmdWorldUpdated_ && (!single_step_yaw_computed_ && !swingTrajectoryPlannerPtr_->getFinalYawSingleStepMode()))
      {
        if(cmd_vector.head(2).norm() > 0.3){//TODO: 这里的阈值可以调整
          target_yaw = atan2(cmd_vector(1), cmd_vector(0));//TO-DO: 转换到局部系(2025/01/17 by matthew)
          double yaw_diff = abs(normalizedYaw(normalizedYaw(target_yaw) - normalizedYaw(cmd_vector[2])));
          std::cout << "yaw_diff: " << yaw_diff << std::endl;
          if(yaw_diff > single_step_yaw_threshold_)
          {
            swingTrajectoryPlannerPtr_->setFinalYawSingleStepMode(true);
            double target_yaw_world = currentState(9) + target_yaw;
            swingTrajectoryPlannerPtr_->setTargetYaw(target_yaw_world);
            std::cout << "yaw_diff: " << yaw_diff << std::endl;
            std::cout << "target_yaw_world: " << target_yaw_world << std::endl;
          }
        }
        else{
          target_yaw = cmd_vector(2);
        }
        single_step_yaw_computed_ = true;
      }
      if(PoseCmdWorldUpdated_
          && abs(target_yaw) > single_step_yaw_threshold_ 
          && !gaitSchedulePtr_->isWalkingGait(current_gait_name))
      {
        std::cout << "target_yaw: " << target_yaw << std::endl;
        std::cout << "[GaitReceiver]: Single step mode." << std::endl;
        Eigen::Vector3d delta_pose = Eigen::Vector3d(cmd_vector(0), cmd_vector(1), target_yaw);
        // std::cout << "[GaitReceiver] delta_pose: " << delta_pose.transpose() << std::endl;
        Eigen::Vector3d xyYawTh = Eigen::Vector3d(0.0, 0.0, 1.0);
        if(abs(delta_pose(2)) > xyYawTh(2)) // 大于阈值才算单步
        {
          bool success = false;
          int retry_times = 0;
          while(!success && retry_times < 3)
          {
            success = swingTrajectoryPlannerPtr_->planSingleStep(initTime, delta_pose, xyYawTh);
            std::cout << "[GaitReceiver]: single step success[" << retry_times << "]: " << success << std::endl;
            ++retry_times;
            xyYawTh *= 0.8;
          }
          if(!success)
            ROS_ERROR("Failed to plan single step.");
          swingTrajectoryPlannerPtr_->setSingleStepPhase(success);
        }
      }
      // std::cout << "initTime:"<<initTime<< std::endl;
      // std::cout << "current_gait_name: " << current_gait_name << std::endl;
      // std::cout << "last_gait_name: " << last_gait_name << std::endl;
      auto pusblishGaittime = [&]()
      {
        kuavo_msgs::gaitTimeName msg;
        msg.start_time = gaitSchedulePtr_->getGgaitTimeName().first;
        msg.gait_name = gaitSchedulePtr_->getGgaitTimeName().second;
        mpcGaitTimeAndNamePublisher_.publish(msg);
      };

      auto targetTraj = referenceManager.getTargetTrajectories();
      auto state0 = targetTraj.getDesiredState(initTime);
      auto state1 = targetTraj.getDesiredState(initTime + 0.015);
      vector3_t planner_vec = (state1.segment<3>(6) - state0.segment<3>(6)) / 0.015;
      const Eigen::Matrix<scalar_t, 3, 1> planner_zyx(-state0(9), 0, 0);
      Eigen::Matrix<double, 3, 3> R_WB = getRotationMatrixFromZyxEulerAngles(planner_zyx);
      planner_vec = R_WB * planner_vec;
      planner_vec(2) = (state1(9) - state0(9)) / 0.015;
      ros_logger_->publishVector("/humanoid/GaitReceiver/planner_vec", planner_vec);
      if (gaitSchedulePtr_->isWalkingGait(current_gait_name))//walking gait
      {
        // std::cout << "planner_vec.norm() : " << planner_vec.norm() << std::endl;
        ros_logger_->publishValue("/humanoid/GaitReceiver/existValidFootPose_initTime", gaitSchedulePtr_->getModeSchedule().existValidFootPose(initTime));
        ros_logger_->publishValue("/humanoid/GaitReceiver/isSingleStepPhase", swingTrajectoryPlannerPtr_->isSingleStepPhase());
        ros_logger_->publishValue("/humanoid/GaitReceiver/cmd_vector_norm", cmd_vector.norm());
        ros_logger_->publishVector("/humanoid/GaitReceiver/cmd_vector", cmd_vector);

        if(!gaitSchedulePtr_->getModeSchedule().existValidFootPose(initTime) && !swingTrajectoryPlannerPtr_->isSingleStepPhase())
        if (planner_vec.norm() <= cmd_threshold|| (planner_vec.norm() <= cmd_threshold*1.5 && PoseCmdWorldUpdated_))// wait for stop
        {
          auto current_pose = currentState.segment<6>(6);
          auto current_mode = gaitSchedulePtr_->getModeSchedule().modeAtTime(initTime);
          bool is_walking = gaitSchedulePtr_->isWalkingGait(last_gait_name);
          bool reached_target = checkTargetReached(current_pose, (PoseCmdUpdated_ || PoseCmdWorldUpdated_));
          bool feet_contact = checkFeetContactPos(current_pose, current_mode);
          ros_logger_->publishValue("/humanoid/GaitReceiver/is_walking", is_walking);
          ros_logger_->publishValue("/humanoid/GaitReceiver/reached_target", reached_target);
          ros_logger_->publishValue("/humanoid/GaitReceiver/feets_contact", feet_contact);
          // std::cout << "[GaitRecevier] is_walking : "<< is_walking << " reached_target : "<< reached_target << " feet_contact : "<< feet_contact << std::endl;
          if ( is_walking && reached_target && feet_contact)// need to stop
          {
            mode_scale_enabled_ = false;
            // gaitSchedulePtr_->setGaitName(gaitSchedulePtr_->getGaitMap().at("stance"));
            gaitSchedulePtr_->modifyWalkModeSequenceTemplate(gaitSchedulePtr_->getGaitMap().at("stance"), initTime, timeHorizon,"stance");
            pusblishGaittime();

            std::cout << "[GaitReceiver]: auto gait: stop" << std::endl;
            if(PoseCmdWorldUpdated_ && swingTrajectoryPlannerPtr_->getFinalYawSingleStepMode())
            {
              std::cout << "cmd_vector: " << cmd_vector.transpose() << std::endl;
              std::cout << "[GaitReceiver]: Single step mode." << std::endl;
              Eigen::Vector3d delta_pose = Eigen::Vector3d(cmd_vector(0), cmd_vector(1), cmd_vector(2));
              double bias_time = 0.09;
              std::cout << "[GaitReceiver] start_time: " << initTime + bias_time << std::endl;
              Eigen::Vector3d xyYawTh = Eigen::Vector3d(0.0, 0.0, 1.0);
              bool success = false;
              int retry_times = 0;
              while(!success && retry_times < 30)
              {
                success = swingTrajectoryPlannerPtr_->planSingleStep(initTime + bias_time, delta_pose, xyYawTh);
                std::cout << "[GaitReceiver]: single step success[" << retry_times << "]: " << success << std::endl;
                ++retry_times;
                xyYawTh *= 0.8;
              }
              if(!success)
                ROS_ERROR("Failed to plan single step.");
              swingTrajectoryPlannerPtr_->setSingleStepPhase(success);
              std::cout << "[GaitReceiver]: setSingleStepPhase set to success in stop case" << std::endl;
            }
            PoseCmdUpdated_ = false;
            PoseCmdWorldUpdated_ = false;
            swingTrajectoryPlannerPtr_->setFinalYawSingleStepMode(false);

          }
        }
      }else// auto start
      {        
        // 指令更新 && != 正在切换
        // std::cout << "[GaitRecevier] START cmd_vector.norm() : " << cmd_vector.norm() <<  std::endl;
        // std::cout << "[GaitRecevier] PoseCmdWorldUpdated_  : " << PoseCmdWorldUpdated_ << std::endl;
        // std::cout << "[GaitRecevier] PoseCmdUpdated_ : " << PoseCmdUpdated_ << std::endl;
        // std::cout << "[GaitRecevier] !checkCmdInLimit(cmd_vector) : " << !checkCmdInLimit(cmd_vector) << std::endl;
        // std::cout << "(velCmdUpdated_ || PoseCmdUpdated_) : " << (velCmdUpdated_ || PoseCmdUpdated_) << std::endl;
        // std::cout << "[GaitRecevier]:  !gaitSchedulePtr_->isWalkingGait(last_gait_name) : " << !gaitSchedulePtr_->isWalkingGait(last_gait_name) << std::endl;
        // std::cout << "[GaitRecevier]:  !gaitSchedulePtr_->getModeSchedule().existValidFootPose(initTime) : " << !gaitSchedulePtr_->getModeSchedule().existValidFootPose(initTime) << std::endl;
        // std::cout << "[GaitRecevier]:  !swingTrajectoryPlannerPtr_->isSingleStepPhase(): " << !swingTrajectoryPlannerPtr_->isSingleStepPhase() << std::endl;
        if(!gaitSchedulePtr_->getModeSchedule().existValidFootPose(initTime) && !swingTrajectoryPlannerPtr_->isSingleStepPhase())
        // if(!swingTrajectoryPlannerPtr_->isSingleStepPhase())
        if (((velCmdUpdated_ && cmd_vector.norm() > cmd_threshold) || ((PoseCmdUpdated_ || PoseCmdWorldUpdated_) && !checkCmdInLimit(cmd_vector)))
        && !gaitSchedulePtr_->isWalkingGait(last_gait_name))
        {
          auto new_gait = walkModeSequenceTemplate_;
          std::cout << "[GaitReceiver]: new_gait temp get" << std::endl;
          if (cmd_vector[0] < 0.1)
            new_gait.replaceModes(mode_to_scale, ModeNumber::SS); // 先移除T H
          mode_scale_enabled_ = true;// 自动切换mode
          // gaitSchedulePtr_->setGaitName(walkModeSequenceTemplate_); // TODO: 注意顺序，可以改成从modifyWalkModeSequenceTemplate传入
          gaitSchedulePtr_->modifyWalkModeSequenceTemplate(new_gait, initTime, timeHorizon, "walk");
          // pub msg
          pusblishGaittime();
          std::cout << "[GaitReceiver]: auto gait: start" << std::endl;
        }
        velCmdUpdated_ = false;
        // 非walk步态且cmd_pose较小时，更新update变量
        if(cmd_vector.norm() < cmd_threshold)
        {
          PoseCmdUpdated_ = false;
          PoseCmdWorldUpdated_ = false;
        }
      }
      
    }

    bool GaitReceiver::checkTargetReached(const vector_t &currentPose, bool is_cmd_pose)
    {
      double xy_error = (currentPose.head(2) - current_target_.head(2)).norm();
      double yaw_error = std::abs(currentPose(3) - current_target_(3));
      // std::cout << "[GaitReceiver] xy_error : " << xy_error << std::endl;
      // std::cout << "[GaitReceiver] yaw_error : " << yaw_error << std::endl;
      double xy_th = is_cmd_pose ? TARGET_REACHED_THRESHOLD_CMD_POSE : TARGET_REACHED_THRESHOLD;
      double yaw_th = is_cmd_pose ? TARGET_REACHED_THRESHOLD_YAW_CMD_POSE : TARGET_REACHED_THRESHOLD_YAW;
      return xy_error < xy_th && yaw_error < yaw_th;
    }

    bool GaitReceiver::checkCmdInLimit(const vector_t &cmd_pose)
    {
      double xy_error = cmd_pose.head(2).norm();
      double yaw_error = std::abs(cmd_pose(2));
      // std::cout << "[GaitReceiver] yaw_error : " << yaw_error << std::endl;
      return xy_error < TARGET_REACHED_THRESHOLD_CMD_POSE && yaw_error < TARGET_REACHED_THRESHOLD_YAW_CMD_POSE;
    }

    bool GaitReceiver::checkFeetContactPos(const vector_t &currentPose, size_t current_mode)
    {
      vector3_t lf_pos_w = vector3_t::Zero();
      vector3_t rf_pos_w = vector3_t::Zero();

      for (int i = 0; i < 4; i++)
      {
        lf_pos_w.head(2) += feet_pos_measured_.segment<2>(i * 3) / 4;
        rf_pos_w.head(2) += feet_pos_measured_.segment<2>(i * 3 + 12) / 4;
      }

      Eigen::Matrix<scalar_t, 3, 1> zyx;
      zyx << -currentPose.tail(3)[0], 0, 0;
      vector3_t lf = getRotationMatrixFromZyxEulerAngles(zyx) * lf_pos_w;
      vector3_t rf = getRotationMatrixFromZyxEulerAngles(zyx) * rf_pos_w;
      vector3_t current_target = getRotationMatrixFromZyxEulerAngles(zyx) * current_target_.head(3);
      if (current_mode == ModeNumber::SS && std::abs(lf(0) - rf(0)) < TARGET_REACHED_FEET_THRESHOLD)
        return true;
      return false;

    }
    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    void GaitReceiver::mpcModeSequenceCallback(const ocs2_msgs::mode_schedule::ConstPtr &msg)
    {
      std::lock_guard<std::mutex> lock(receivedGaitMutex_);
      receivedGait_ = readModeSequenceTemplateMsg(*msg);
      gaitUpdated_ = true;
    }
    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    void GaitReceiver::mpcModeScaleSequenceCallback(const std_msgs::Float32::ConstPtr &msg)
    {
      std::lock_guard<std::mutex> lock(receivedGaitMutex_);
      gait_scale_ = msg->data;
      gait_scale_updated_ = true;
    }
    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    void GaitReceiver::mpcWalkHeelSacleCallback(const std_msgs::Float32::ConstPtr &msg)
    {
      std::lock_guard<std::mutex> lock(receivedGaitMutex_);
      walk_heel_scale_ = msg->data;
      walk_heel_scale_updated_ = true;
    }

  } // namespace humanoid
} // namespace ocs2
