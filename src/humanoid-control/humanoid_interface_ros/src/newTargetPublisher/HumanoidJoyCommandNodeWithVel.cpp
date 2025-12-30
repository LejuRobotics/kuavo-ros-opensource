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

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_msgs/mpc_observation.h>

#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>
#include <humanoid_interface/gait/ModeSequenceTemplate.h>
#include "humanoid_interface_ros/gait/ModeSequenceTemplateRos.h"
#include "std_srvs/Trigger.h"
#include <std_msgs/Bool.h>
#include "humanoid_interface_drake/humanoid_interface_drake.h"
#include <geometry_msgs/Twist.h>

namespace ocs2
{
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
      cmd_vel_publisher_ = nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 10, true);
      joy_sub_ = nodeHandle_.subscribe("joy", 10, &JoyControl::joyCallback, this);
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

    void checkAndPublishCommandLine(const vector_t &joystick_origin_axis)
    {
      geometry_msgs::Twist cmdVel;
      bool updated = commandLineToTargetTrajectories(joystick_origin_axis, observation_, cmdVel);
      if (updated)
        cmd_vel_publisher_.publish(cmdVel);
    }

  private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr &joy_msg)
    {
      joystick_origin_axis_.head(4) << joy_msg->axes[AXIS_LEFT_STICK_X], joy_msg->axes[AXIS_LEFT_STICK_Y], joy_msg->axes[AXIS_RIGHT_STICK_Z], joy_msg->axes[AXIS_RIGHT_STICK_YAW];

      // gait command
      auto publish_mode_sequence_template = [&](const std::string &gaitName)
      {
        humanoid::ModeSequenceTemplate modeSequenceTemplate = gait_map_.at(gaitName);
        mode_sequence_template_publisher_.publish(createModeSequenceTemplateMsg(modeSequenceTemplate));
      };
      vector_t button_trigger_axis = vector_t::Zero(6);
      if (joy_msg->buttons[BUTTON_A])
        publish_mode_sequence_template(joy_button_map_.at(BUTTON_A));
      else if (joy_msg->buttons[BUTTON_B])
        publish_mode_sequence_template(joy_button_map_.at(BUTTON_B));
      else if (joy_msg->buttons[BUTTON_X])
        publish_mode_sequence_template(joy_button_map_.at(BUTTON_X));
      else if (joy_msg->buttons[BUTTON_Y])
        publish_mode_sequence_template(joy_button_map_.at(BUTTON_Y));
      else if (joy_msg->buttons[BUTTON_LB])
        pubModeGaitScale(0.9);
      else if (joy_msg->buttons[BUTTON_RB])
        pubModeGaitScale(1.1);
      else if (joy_msg->buttons[BUTTON_BACK])
        callTerminateSrv();
      else if (joy_msg->buttons[BUTTON_START])
        callRealInitializeSrv();
      else if (joy_msg->axes[AXIS_FORWARD_BACK_TRIGGER])
      {
        std::cout << "tigger forward/back pressed" << std::endl;
        button_trigger_axis[0] = joy_msg->axes[AXIS_FORWARD_BACK_TRIGGER] / 2;
        checkAndPublishCommandLine(button_trigger_axis);
      }
      else if (joy_msg->axes[AXIS_LEFT_RIGHT_TRIGGER])
      {
        std::cout << "tigger left/right pressed" << std::endl;
        button_trigger_axis[1] = joy_msg->axes[AXIS_LEFT_RIGHT_TRIGGER] / 2;
        checkAndPublishCommandLine(button_trigger_axis);
      }
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
    bool commandLineToTargetTrajectories(const vector_t &joystick_origin_axis, const SystemObservation &observation, geometry_msgs::Twist& cmdVel)
    {
      double dead_zone = 0.05;
      Eigen::VectorXd limit_vector(4);
      limit_vector << c_relative_base_limit_[0], c_relative_base_limit_[1], c_relative_base_limit_[2], c_relative_base_limit_[3];
      if (joystick_origin_axis.cwiseAbs().maxCoeff() < dead_zone)
        return false; // command line is zero, do nothing

      commad_line_target_.head(4) = joystick_origin_axis.head(4).cwiseProduct(limit_vector);

      // vector_t target(6);
      if (commad_line_target_.head(2).cwiseAbs().maxCoeff() > dead_zone)
      { // base p_x, p_y are relative to current state
        cmdVel.linear.x = commad_line_target_(0);
        cmdVel.angular.z = commad_line_target_(1);
      }
      // base z relative to the default height
      // current_target_(2) = com_height_ + commad_line_target_(2);
      cmdVel.linear.z = commad_line_target_(2);
      // cmdVel.linear.z = 0;
      // theta_z relative to current
      if (std::abs(commad_line_target_(3)) > dead_zone)
        cmdVel.linear.y = commad_line_target_(3);

      return true;
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
    ros::Subscriber observation_sub_;
    bool get_observation_ = false;
    scalar_t target_displacement_velocity_;
    scalar_t target_rotation_velocity_;
    scalar_t com_height_;
    vector_t default_joint_state_ = vector_t::Zero(12);
    vector_t commad_line_target_ = vector_t::Zero(6);
    vector_t joystick_origin_axis_ = vector_t::Zero(6);

    const ocs2::scalar_array_t c_relative_base_limit_{0.6, 0.8, 0.3, 0.5};
    ocs2::SystemObservation observation_;
    ros::Publisher mode_sequence_template_publisher_;
    ros::Publisher mode_scale_publisher_;
    ros::Publisher cmd_vel_publisher_;
    ros::Publisher stop_pub_;
    float total_mode_scale_{1.0};

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
