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

#include <string>

#include <ros/init.h>
#include <ros/package.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_msgs/mpc_observation.h>

#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>
#include <humanoid_interface/gait/ModeSequenceTemplate.h>
#include "humanoid_interface_ros/gait/ModeSequenceTemplateRos.h"
#include "std_srvs/Trigger.h"
#include <std_msgs/Bool.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include "humanoid_interface_drake/humanoid_interface_drake.h"
#include <humanoid_interface/gait/MotionPhaseDefinition.h>

namespace ocs2
{
  using namespace humanoid;
  enum JoyButton
  {
    BUTTON_A = 0,
    BUTTON_B = 1,
    BUTTON_X = 2,
    BUTTON_Y = 3,
    BUTTON_LB = 4,
    BUTTON_RB = 5,
    BUTTON_BACK = 6,
    BUTTON_START = 7,
  };
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

#define DEAD_ZONE 0.05
#define TARGET_REACHED_THRESHOLD 0.1
#define TARGET_REACHED_THRESHOLD_YAW 0.1
#define TARGET_REACHED_FEET_THRESHOLD 0.08
  class JoyControl
  {
  public:
    JoyControl(ros::NodeHandle &nodeHandle, const std::string &robotName, bool verbose = false)
        : nodeHandle_(nodeHandle),
          targetPoseCommand_(nodeHandle, robotName)
    {
      // Get node parameters
      std::string referenceFile;
      nodeHandle.getParam("/referenceFile", referenceFile);
      RobotVersion robot_version(3, 4);
      if (nodeHandle.hasParam("/robot_version"))
      {
        int robot_version_int;
        nodeHandle.getParam("/robot_version", robot_version_int);
        int major = robot_version_int / 10;
        int minor = robot_version_int % 10;
        robot_version = RobotVersion(major, minor);
      }
      // loadData::loadCppDataType(referenceFile, "comHeight", com_height_);
      auto drake_interface_ = HighlyDynamic::HumanoidInterfaceDrake::getInstancePtr(robot_version, true, 2e-3);
      default_joint_state_ = drake_interface_->getDefaultJointState();
      com_height_ = drake_interface_->getIntialHeight();
      initial_state_ = drake_interface_->getInitialState();
      loadData::loadCppDataType(referenceFile, "targetRotationVelocity", target_rotation_velocity_);
      loadData::loadCppDataType(referenceFile, "targetDisplacementVelocity", target_displacement_velocity_);

      // gait
      std::string gaitCommandFile;
      nodeHandle.getParam("/gaitCommandFile", gaitCommandFile);
      ROS_INFO_STREAM(robotName + "_mpc_mode_schedule node is setting up ...");
      std::vector<std::string> gaitList;
      loadData::loadStdVector(gaitCommandFile, "list", gaitList, verbose);
      gait_map_.clear();
      for (const auto &gaitName : gaitList)
      {
        gait_map_.insert({gaitName, humanoid::loadModeSequenceTemplate(gaitCommandFile, gaitName, verbose)});
      }

      mode_sequence_template_publisher_ = nodeHandle.advertise<ocs2_msgs::mode_schedule>(robotName + "_mpc_mode_schedule", 10, true);
      mode_scale_publisher_ = nodeHandle.advertise<std_msgs::Float32>(robotName + "_mpc_mode_scale", 10, true);
      joy_sub_ = nodeHandle_.subscribe("joy", 10, &JoyControl::joyCallback, this);
      feet_sub_ = nodeHandle_.subscribe("/humanoid_controller/swing_leg/pos_measured", 2, &JoyControl::feetCallback, this);
      observation_sub_ = nodeHandle_.subscribe(robotName + "_mpc_observation", 10, &JoyControl::observationCallback, this);
      stop_pub_ = nodeHandle_.advertise<std_msgs::Bool>("/stop_robot", 10);
    }

    void run()
    {
      ros::Rate rate(100);
      while (ros::ok())
      {
        ros::spinOnce();
        rate.sleep();
        if (!get_observation_)
        {
          // ROS_INFO_STREAM("Waiting for observation message...");
          continue;
        }
        checkAndPublishCommandLine(joystick_origin_axis_);
      }
      return;
    }
    bool checkTargetReached()
    {
      const vector_t currentPose = observation_.state.segment<6>(6);
      double xy_error = (currentPose.head(2) - current_target_.head(2)).norm();
      double yaw_error = std::abs(currentPose(3) - current_target_(3));
      return xy_error < TARGET_REACHED_THRESHOLD && yaw_error < TARGET_REACHED_THRESHOLD_YAW;
    }

    bool checkFeetContactPos()
    {
      if (current_gait_ == "walk")
      {
        vector3_t lf_pos_w = vector3_t::Zero();
        vector3_t rf_pos_w = vector3_t::Zero();

        for (int i = 0; i < 4; i++)
        {
          lf_pos_w.head(2) += feet_pos_measured_.segment<2>(i * 3) / 4;
          rf_pos_w.head(2) += feet_pos_measured_.segment<2>(i * 3 + 12) / 4;
        }

        Eigen::Matrix<scalar_t, 3, 1> zyx;
        zyx << -observation_.state.segment<6>(6).tail(3)[0], 0, 0;
        vector3_t lf = getRotationMatrixFromZyxEulerAngles(zyx) * lf_pos_w;
        vector3_t rf = getRotationMatrixFromZyxEulerAngles(zyx) * rf_pos_w;
        vector3_t current_target = getRotationMatrixFromZyxEulerAngles(zyx) * current_target_.head(3);
        if (std::abs(lf(0) - rf(0)) < TARGET_REACHED_FEET_THRESHOLD)
          return true;
        return false;
      }
      return true;
    }

    void checkAndPublishCommandLine(const vector_t &joystick_origin_axis)
    {
      std::lock_guard<std::mutex> lock(target_mutex_);
      static bool waitting_for_walk = false;
      auto updated = commandLineToTargetTrajectories(joystick_origin_axis, observation_, current_target_traj_);
      if (!std::any_of(updated.begin(), updated.end(), [](bool x)
                       { return x; }) &&
          !waitting_for_walk)
      {
        if (auto_stance_mode_ && checkTargetReached() && !reached_target_ && checkFeetContactPos())
        {
          publishGaitTemplate("stance");
          std::cout << "publishing stance mode" << std::endl;
          reached_target_ = true;
        }
        return;
      }
      reached_target_ = false;

      if (auto_stance_mode_ && current_gait_ == "stance" && !updated[2]) // TODO: need more detailed gait check
      {
        publishGaitTemplate("walk");
        waitting_for_walk = true;
        std::cout << "publishing walk mode" << std::endl;
      }
      else if (observation_.mode != ModeNumber::SS || !auto_stance_mode_ || updated[2])
      {
        waitting_for_walk = false;
        targetPoseCommand_.publishTargetTrajectories(current_target_traj_);
      }
    }

  private:
    void feetCallback(const std_msgs::Float64MultiArray::ConstPtr &feet_msg)
    {
      feet_pos_measured_ = Eigen::Map<const Eigen::VectorXd>(feet_msg->data.data(), feet_msg->data.size());
    }
    void joyCallback(const sensor_msgs::Joy::ConstPtr &joy_msg)
    {
      joystick_origin_axis_.head(4) << joy_msg->axes[AXIS_LEFT_STICK_X], joy_msg->axes[AXIS_LEFT_STICK_Y], joy_msg->axes[AXIS_RIGHT_STICK_Z], joy_msg->axes[AXIS_RIGHT_STICK_YAW];

      vector_t button_trigger_axis = vector_t::Zero(6);
      checkGaitSwitchCommand(joy_msg);

      if (joy_msg->buttons[BUTTON_LB])
        pubModeGaitScale(0.9);
      else if (joy_msg->buttons[BUTTON_RB])
        pubModeGaitScale(1.1);
      else if (joy_msg->buttons[BUTTON_BACK])
        callTerminateSrv();
      else if (joy_msg->buttons[BUTTON_START])
        callRealInitializeSrv();
      else if (joy_msg->axes[AXIS_FORWARD_BACK_TRIGGER])
      {
        button_trigger_axis[0] = joy_msg->axes[AXIS_FORWARD_BACK_TRIGGER] / 2;
        std::cout << "tigger forward/back pressed:" << button_trigger_axis[0] << std::endl;
        checkAndPublishCommandLine(button_trigger_axis);
      }
      else if (joy_msg->axes[AXIS_LEFT_RIGHT_TRIGGER])
      {
        // button_trigger_axis[1] = joy_msg->axes[AXIS_LEFT_RIGHT_TRIGGER] / 2;
        button_trigger_axis[3] = joy_msg->axes[AXIS_LEFT_RIGHT_TRIGGER] ;
        std::cout << "tigger left/right pressed:" << button_trigger_axis[3] << std::endl;
        checkAndPublishCommandLine(button_trigger_axis);
      }
    }

    void checkGaitSwitchCommand(const sensor_msgs::Joy::ConstPtr &joy_msg)
    {
      // 检查是否有gait切换指令
      if (joy_msg->buttons[BUTTON_A])
      {
        publishGaitTemplate("stance");
      }
      else if (joy_msg->buttons[BUTTON_B])
      {
        publishGaitTemplate("trot");
      }
      else if (joy_msg->buttons[BUTTON_X])
      {
        publishGaitTemplate("jump");
      }
      else if (joy_msg->buttons[BUTTON_Y])
      {
        publishGaitTemplate("walk");
      }
      else
      {
        return;
      }

      std::cout << "gait switch to: " << current_gait_ << std::endl;
      std::cout << "turn " << (current_gait_ == "stance" ? "on " : "off ") << " auto stance mode" << std::endl;
      auto_stance_mode_ = (current_gait_ == "stance");
    }

    void publishGaitTemplate(const std::string &gaitName)
    {
      // 发布对应的gait模板
      humanoid::ModeSequenceTemplate modeSequenceTemplate = gait_map_.at(gaitName);
      mode_sequence_template_publisher_.publish(createModeSequenceTemplateMsg(modeSequenceTemplate));
      current_gait_ = gaitName;
    }

    void observationCallback(const ocs2_msgs::mpc_observation::ConstPtr &observation_msg)
    {
      observation_ = ros_msg_conversions::readObservationMsg(*observation_msg);
      get_observation_ = true;
    }

    scalar_t estimateTimeToTarget(const vector_t &desiredBaseDisplacement)
    {
      const scalar_t &dx = desiredBaseDisplacement(0);
      const scalar_t &dy = desiredBaseDisplacement(1);
      const scalar_t &dyaw = desiredBaseDisplacement(3);
      const scalar_t rotationTime = std::abs(dyaw) / target_rotation_velocity_;
      const scalar_t displacement = std::sqrt(dx * dx + dy * dy);
      const scalar_t displacementTime = displacement / target_displacement_velocity_;
      return std::max(rotationTime, displacementTime);
    }

    /**
     * Converts command line to TargetTrajectories.
     * @param [in] commad_line_target_ : [deltaX, deltaY, deltaZ, deltaYaw]
     * @param [in] observation : the current observation
     */
    std::vector<bool> commandLineToTargetTrajectories(const vector_t &joystick_origin_axis, const SystemObservation &observation, TargetTrajectories &target_traj)
    {
      std::vector<bool> updated(6, false);
      Eigen::VectorXd limit_vector(4);
      limit_vector << c_relative_base_limit_[0], c_relative_base_limit_[1], c_relative_base_limit_[2], c_relative_base_limit_[3];
      if (joystick_origin_axis.cwiseAbs().maxCoeff() < DEAD_ZONE && last_joystick_origin_axis_.cwiseAbs().maxCoeff() < DEAD_ZONE)
        return updated; // command line is zero, do nothing

      commad_line_target_.head(4) = joystick_origin_axis.head(4).cwiseProduct(limit_vector);

      vector_t currentPose = observation.state.segment<6>(6);
      // vector_t target(6);
      if (joystick_origin_axis.head(2).cwiseAbs().maxCoeff() > DEAD_ZONE)
      { // base p_x, p_y are relative to current state
        double dx = commad_line_target_(0) * cos(currentPose(3)) - commad_line_target_(1) * sin(currentPose(3));
        double dy = commad_line_target_(0) * sin(currentPose(3)) + commad_line_target_(1) * cos(currentPose(3));
        current_target_(0) = currentPose(0) + dx;
        current_target_(1) = currentPose(1) + dy;
        updated[0] = true;
        updated[1] = true;
        // std::cout << "base displacement: " << dx << ", " << dy << std::endl;
      }
      // base z relative to the default height
      if (std::abs(joystick_origin_axis(2)) > DEAD_ZONE || std::abs(last_joystick_origin_axis_(2)) > DEAD_ZONE)
      {
        updated[2] = true;
        current_target_(2) = com_height_ + commad_line_target_(2);
        std::cout << "base height: " << current_target_(2) << std::endl;
      }
      else
      {
        current_target_(2) = com_height_;
      }

      // theta_z relative to current
      if (std::abs(joystick_origin_axis(3)) > DEAD_ZONE  || std::abs(last_joystick_origin_axis_(3)) > DEAD_ZONE)
      {
        updated[3] = true;
        current_target_(3) = currentPose(3) + commad_line_target_(3) * M_PI / 180.0;
      }
      // else
      //   current_target_(3) = currentPose(3);
      // theta_y, theta_x
      current_target_(4) = initial_state_[10]; // fixed value，因为存在静差
      currentPose(4) = initial_state_[10];     // fixed value
      current_target_(5) = 0.0;

      const vector_t targetPose = current_target_;

      // target reaching duration
      const scalar_t targetReachingTime = observation.time + estimateTimeToTarget(targetPose - currentPose);

      // desired time trajectory
      const scalar_array_t timeTrajectory{observation.time, targetReachingTime};

      // desired state trajectory
      vector_array_t stateTrajectory(2, vector_t::Zero(observation.state.size()));
      stateTrajectory[0] << vector_t::Zero(6), currentPose, default_joint_state_;
      stateTrajectory[1] << vector_t::Zero(6), targetPose, default_joint_state_;

      // desired input trajectory (just right dimensions, they are not used)
      const vector_array_t inputTrajectory(2, vector_t::Zero(observation.input.size()));
      target_traj = {timeTrajectory, stateTrajectory, inputTrajectory};
      last_joystick_origin_axis_ = joystick_origin_axis;
      return updated;
    }

    inline void pubModeGaitScale(float scale)
    {
      total_mode_scale_ *= scale;
      ROS_INFO_STREAM("[JoyControl] Publish scale: " << scale << ", Total mode scale: " << total_mode_scale_);
      std_msgs::Float32 msg;
      msg.data = scale;
      mode_scale_publisher_.publish(msg);
    }
    void callRealInitializeSrv()
    {
      ros::ServiceClient client = nodeHandle_.serviceClient<std_srvs::Trigger>("/humanoid_controller/real_initial_start");
      std_srvs::Trigger srv;

      // 调用服务
      if (client.call(srv))
      {
        ROS_INFO("Service call successful");
      }
      else
      {
        ROS_ERROR("Failed to call service");
      }
    }
    void callTerminateSrv()
    {
      std::cout << "tigger callTerminateSrv" << std::endl;
      for (int i = 0; i < 5; i++)
      {
        std_msgs::Bool msg;
        msg.data = true;
        stop_pub_.publish(msg);
        ::ros::Duration(0.1).sleep();
      }
    }

  private:
    ros::NodeHandle nodeHandle_;
    TargetTrajectoriesRosPublisher targetPoseCommand_;
    ros::Subscriber joy_sub_;
    ros::Subscriber feet_sub_;
    ros::Subscriber observation_sub_;
    bool get_observation_ = false;
    vector_t current_target_ = vector_t::Zero(6);
    std::string current_gait_ = "stance";
    bool auto_stance_mode_ = true;
    bool reached_target_ = false;
    scalar_t target_displacement_velocity_;
    scalar_t target_rotation_velocity_;
    scalar_t com_height_;
    vector_t initial_state_;
    vector_t default_joint_state_ = vector_t::Zero(12);
    vector_t commad_line_target_ = vector_t::Zero(6);
    vector_t joystick_origin_axis_ = vector_t::Zero(6);
    vector_t last_joystick_origin_axis_ = vector_t::Zero(6); 

    const ocs2::scalar_array_t c_relative_base_limit_{1.0, 0.5, 0.3, 90.0};
    ocs2::SystemObservation observation_;
    ros::Publisher mode_sequence_template_publisher_;
    ros::Publisher mode_scale_publisher_;
    ros::Publisher stop_pub_;
    float total_mode_scale_{1.0};
    TargetTrajectories current_target_traj_;
    std::mutex target_mutex_;
    vector_t feet_pos_measured_ = vector_t::Zero(24);

    std::map<std::string, humanoid::ModeSequenceTemplate> gait_map_;
    std::map<JoyButton, std::string> joy_button_map_ = {{BUTTON_A, "stance"}, {BUTTON_B, "trot"}, {BUTTON_X, "jump"}, {BUTTON_Y, "walk"}};
  };
}

int main(int argc, char *argv[])
{
  const std::string robotName = "humanoid";

  // Initialize ros node
  ::ros::init(argc, argv, robotName + "_joy_command_node");
  ::ros::NodeHandle nodeHandle;

  ocs2::JoyControl joyControl(nodeHandle, robotName);
  joyControl.run();

  return 0;
}
