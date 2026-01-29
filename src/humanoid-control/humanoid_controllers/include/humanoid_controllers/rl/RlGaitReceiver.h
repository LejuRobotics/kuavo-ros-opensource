/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>
#include <ocs2_core/Types.h>
#include <humanoid_interface/common/Types.h>
#include <humanoid_estimation/StateEstimateBase.h>
#include <ocs2_core/misc/LoadData.h>
#include <boost/property_tree/ptree.hpp>
#include <mutex>

namespace ocs2
{
namespace humanoid
{

// CommandDataRL structure for RL controller commands
struct CommandDataRL
{
  double cmdVelLineX_;
  double cmdVelLineY_;
  double cmdVelLineZ_;
  double cmdVelAngularX_;
  double cmdVelAngularY_;
  double cmdVelAngularZ_;
  double cmdStance_;
  double cmdVelScaleLineX_;
  double cmdVelScaleLineY_;
  double cmdVelScaleLineZ_;
  double cmdVelScaleAngularX_;
  double cmdVelScaleAngularY_;
  double cmdVelScaleAngularZ_;
  double cmdScaleStance_;
  
  CommandDataRL() {
    cmdVelLineX_ = 0.0;
    cmdVelLineY_ = 0.0;
    cmdVelLineZ_ = 0.0;
    cmdVelAngularX_ = 0.0;
    cmdVelAngularY_ = 0.0;
    cmdVelAngularZ_ = 0.0;
    cmdStance_ = 1.0; // Default to stance mode
    cmdVelScaleLineX_ = 1.0;
    cmdVelScaleLineY_ = 1.0;
    cmdVelScaleLineZ_ = 1.0;
    cmdVelScaleAngularX_ = 1.0;
    cmdVelScaleAngularY_ = 1.0;
    cmdVelScaleAngularZ_ = 1.0;
    cmdScaleStance_ = 1.0;
  }
  
  void setzero()
  {
    cmdVelLineX_ = 0.0;
    cmdVelLineY_ = 0.0;
    cmdVelLineZ_ = 0.0;
    cmdVelAngularX_ = 0.0;
    cmdVelAngularY_ = 0.0;
    cmdVelAngularZ_ = 0.0;
    cmdStance_ = 1.0; // Set to stance mode when zeroing
  }
  
  void scale()
  {
    cmdVelLineX_ *= cmdVelScaleLineX_;
    cmdVelLineY_ *= cmdVelScaleLineY_;
    cmdVelLineZ_ *= cmdVelScaleLineZ_;
    cmdVelAngularX_ *= cmdVelScaleAngularX_;
    cmdVelAngularY_ *= cmdVelScaleAngularY_;
    cmdVelAngularZ_ *= cmdVelScaleAngularZ_;
    cmdStance_ *= cmdScaleStance_;
  }
  
  Eigen::VectorXd getCommandRL()
  {
    Eigen::VectorXd dynamicVector(4);
    dynamicVector << cmdVelLineX_, cmdVelLineY_, cmdVelAngularZ_, cmdStance_;
    return dynamicVector;
  }
};

class RlGaitReceiver
{
public:
  RlGaitReceiver(ros::NodeHandle& nh, CommandDataRL* initialCommand = nullptr);
  ~RlGaitReceiver() = default;

  // Main update method called from humanoidController
  void update(const ros::Time& time, const vector_t& torsostate, const vector_t& feetPositions);

  void setEnabled(bool enable);
  bool isEnabled() const;
  
  // Get current command data
  CommandDataRL getCurrentCommand() const;
  
  // Load in-place stepping configuration from config file
  void loadInPlaceStepConfig(const std::string& config_file, bool verbose = false);

private:
  // ROS callbacks
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void gaitNameCallback(const std_msgs::String::ConstPtr& msg);
  
  // Smart stop detection functions
  bool checkSmartStopConditions(const vector_t& torsostate, const vector_t& feetPositions);
  bool shouldSmartStop(const vector_t& torsostate, const vector_t& feetPositions);
  
  // Velocity processing
  double calculateVelocityMagnitude(const geometry_msgs::Twist& cmd_vel);
  geometry_msgs::Twist applyMixedMotionLimits(const geometry_msgs::Twist& cmd_vel) const;
  geometry_msgs::Twist smoothVelocityCommand(const geometry_msgs::Twist& cmd_vel, const ros::Time& current_time);
  
  // In-place stepping functions
  void startInPlaceStepping(const ros::Time& current_time);
  void stopInPlaceStepping();
  void updateInPlaceStepping(const ros::Time& current_time);
  bool isInPlaceSteppingActive() const;

private:
  ros::NodeHandle& nh_;
  
  // ROS subscribers
  ros::Subscriber cmd_vel_sub_;
  ros::Subscriber gait_name_sub_;
  
  // Current command data
  CommandDataRL currentCommand_;
  bool enabled_;
  
  // Smart stop detection parameters
  bool smart_stop_enabled_;
  double torso_velocity_threshold_;
  double feet_alignment_threshold_;
  
  // Velocity smoothing parameters
  geometry_msgs::Twist smoothed_cmd_vel_;
  geometry_msgs::Twist previous_cmd_vel_;
  double velocity_smooth_factor_;
  double max_velocity_change_;
  double velocity_smooth_time_;
  ros::Time last_velocity_update_time_;
  
  // Angular velocity smoothing parameters for turning
  double angular_velocity_smooth_factor_;      // 角速度平滑因子（默认0.5，更强的平滑）
  double angular_velocity_change_threshold_;   // 角速度变化阈值（默认0.2 rad/s）
  double angular_velocity_max_rate_;           // 最大角速度变化率（默认2.0 rad/s²）
  
  // Mixed motion limits parameters
  bool enable_mixed_mode_;                    // 是否启用混合运动模式
  double angular_vel_threshold_;              // 角速度阈值 (rad/s)
  double max_linear_vel_with_angular_;        // 有角速度时最大线速度 (m/s)
  double linear_vel_threshold_;               // 线速度阈值 (m/s)
  double max_angular_vel_with_linear_;        // 有线速度时最大角速度 (rad/s)
  bool smooth_transition_;                    // 是否启用平滑过渡
  double transition_factor_;                  // 过渡因子
  
  // In-place stepping parameters
  geometry_msgs::Twist in_place_step_velocity_;
  double in_place_step_duration_;
  bool enable_in_place_stepping_;
  bool is_in_place_stepping_;
  ros::Time in_place_step_start_time_;
  bool is_real_;
  
  // Thread safety
  mutable std::mutex command_mutex_;
};

} // namespace humanoid
} // namespace ocs2
