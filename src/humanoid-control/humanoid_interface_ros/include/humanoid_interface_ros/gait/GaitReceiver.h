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

#include <mutex>

#include <ros/ros.h>

#include <ocs2_core/Types.h>
#include <ocs2_msgs/mode_schedule.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Joy.h>
#include "std_srvs/SetBool.h"

#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>

#include <humanoid_interface/gait/GaitSchedule.h>
#include <humanoid_interface/gait/ModeSequenceTemplate.h>
#include <humanoid_interface/gait/MotionPhaseDefinition.h>
#include "humanoid_interface/foot_planner/SwingTrajectoryPlanner.h"
#include <ocs2_msgs/mpc_target_trajectories.h>
#include "ocs2_ros_interfaces/common/RosMsgConversions.h"
#include "humanoid_interface/reference_manager/SwitchedModelReferenceManager.h"

namespace ocs2
{
  namespace humanoid
  {

    class GaitReceiver : public SolverSynchronizedModule
    {
    public:
      GaitReceiver(::ros::NodeHandle nodeHandle, std::shared_ptr<SwitchedModelReferenceManager> referenceManagerPtr, const std::string &robotName);

      void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t &currentState,
                        const ReferenceManagerInterface &referenceManager) override;

      void postSolverRun(const PrimalSolution &primalSolution) override {};

    private:
      void mpcModeSequenceCallback(const ocs2_msgs::mode_schedule::ConstPtr &msg);
      void mpcModeScaleSequenceCallback(const std_msgs::Float32::ConstPtr &msg);
      void mpcWalkHeelSacleCallback(const std_msgs::Float32::ConstPtr &msg);
      void joyCallback(const sensor_msgs::Joy::ConstPtr &joy_msg);
      void autoGaitCheck(scalar_t initTime, scalar_t finalTime, const vector_t &currentState, const ReferenceManagerInterface &referenceManager);
      bool checkTargetReached(const vector_t &currentPose, bool is_cmd_pose);
      bool checkCmdInLimit(const vector_t &cmd_pose);
      bool checkFeetContactPos(const vector_t &currentPose, size_t current_mode);

      bool autoGaitModeSrvCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
      {
        if (gaitSchedulePtr_)
        {
          gaitSchedulePtr_->setAutoGaitEnabled(req.data);
        } 
        res.success = true;
        res.message = "Auto Gait Mode set to: " + std::to_string(req.data);
        return true;
      }

      inline double normalizedYaw(double yaw)
      {
        while (yaw > M_PI)
          yaw -= 2*M_PI;
        while(yaw < -M_PI)
          yaw += 2*M_PI;
        return yaw;
      };

      std::shared_ptr<GaitSchedule> gaitSchedulePtr_;
      std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPlannerPtr_;

      ::ros::Subscriber mpcModeSequenceSubscriber_;
      ::ros::Subscriber mpcModeScaleSequenceSubscriber_;
      ::ros::Subscriber mpcWalkHeelScaleSubscriber_;
      ::ros::Subscriber mpcStopStepNumSubscriber_;
      ::ros::Publisher mpcGaitTimeAndNamePublisher_;
      ::ros::Subscriber joy_sub_;
      ::ros::ServiceServer auto_gait_mode_service_;
      ::ros::Subscriber targetVelocitySubscriber_;
      ::ros::Subscriber targetPoseSubscriber_;
      ::ros::Subscriber targetPoseWorldSubscriber_;

      ::ros::Subscriber feet_sub_ ;
      ::ros::Subscriber policy_sub_;
      
      vector_t feet_pos_measured_ = vector_t::Zero(24);

      std::shared_ptr<SwitchedModelReferenceManager> referenceManagerPtr_;
      
      vector_t current_joystick_values_ = vector_t::Zero(6);
      std::mutex receivedGaitMutex_;
      std::atomic_bool gaitUpdated_;
      ModeSequenceTemplate receivedGait_;
      ModeSequenceTemplate walkModeSequenceTemplate_; // only for walk mode
      float gait_scale_{1.0};
      float walk_heel_scale_{1.0};
      std::atomic_bool gait_scale_updated_{false};
      std::atomic_bool walk_heel_scale_updated_{false};
      std::atomic_bool stop_step_num_updated_{false};
      bool mode_scale_enabled_{false};
      std::vector<double> current_vx_vector_{0.0};
      double current_mode_end_time_{0.0};
      int stop_step_num_{-1};

      vector_t current_target_ = vector_t::Zero(6);
      vector_t cmdVel_ = vector_t::Zero(6);
      vector_t cmdPose_ = vector_t::Zero(6);
      vector_t cmdPoseWorld_ = vector_t::Zero(6);

      bool velCmdUpdated_ = false;
      bool PoseCmdUpdated_ = false;
      bool PoseCmdWorldUpdated_ = false;
      bool waitting_for_walk_ = false;
      bool single_step_yaw_computed_ = false;
      double single_step_yaw_threshold_ = 0.5; // rad
    };

  } // namespace humanoid
} // namespace ocs2
