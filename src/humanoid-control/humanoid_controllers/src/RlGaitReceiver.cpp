/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#include "humanoid_controllers/RlGaitReceiver.h"
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <std_msgs/String.h>

namespace ocs2
{
namespace humanoid
{

RlGaitReceiver::RlGaitReceiver(ros::NodeHandle& nh, CommandDataRL* initialCommand)
  : nh_(nh)
  , enabled_(true)
  , smart_stop_enabled_(true)
  , torso_velocity_threshold_(0.05)
  , feet_alignment_threshold_(0.08)
  , velocity_smooth_factor_(0.1)
  , max_velocity_change_(0.5)
  , velocity_smooth_time_(0.1)
{
  // Initialize command data
  if (initialCommand) {
    currentCommand_ = *initialCommand;
  } else {
    currentCommand_.setzero();
    currentCommand_.cmdStance_ = 1; // Start in stance mode
  }
  currentCommand_.cmdStance_ = 1; // Start in stance mode
  
  // Initialize velocity smoothing
  smoothed_cmd_vel_.linear.x = 0.0;
  smoothed_cmd_vel_.linear.y = 0.0;
  smoothed_cmd_vel_.linear.z = 0.0;
  smoothed_cmd_vel_.angular.x = 0.0;
  smoothed_cmd_vel_.angular.y = 0.0;
  smoothed_cmd_vel_.angular.z = 0.0;
  previous_cmd_vel_ = smoothed_cmd_vel_;
  last_velocity_update_time_ = ros::Time::now();
  
  // Subscribe to cmd_vel topic
  cmd_vel_sub_ = nh_.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, 
    &RlGaitReceiver::cmdVelCallback, this);
  
  // Subscribe to gait name request topic
  gait_name_sub_ = nh_.subscribe<std_msgs::String>("/humanoid_mpc_gait_name_request", 10,
    &RlGaitReceiver::gaitNameCallback, this);
  
  ROS_INFO("[RlGaitReceiver] Initialized with smart stop detection enabled");
}

void RlGaitReceiver::update(const ros::Time& time, const vector_t& torsostate, const vector_t& feetPositions)
{
  if (!enabled_) {
    return;
  }
  double velocity_magnitude = calculateVelocityMagnitude(smoothed_cmd_vel_);
  std::lock_guard<std::mutex> lock(command_mutex_);

  if (velocity_magnitude < 0.01 && currentCommand_.cmdStance_ == 1)// 速度小并且已经站立
    return;
  
  
  // Process current velocity command
  
  if (velocity_magnitude < 0.01 && currentCommand_.cmdStance_ == 0 && smart_stop_enabled_) // 速度小并且已经行走并且智能停止启用
  {
    // Velocity is very small, check for smart stop
    if (shouldSmartStop(torsostate, feetPositions)) {
      // Smart stop conditions met, switch to stance mode
      currentCommand_.setzero();
      currentCommand_.cmdStance_ = 1;
      std::cout << "[RlGaitReceiver] Smart stop conditions met, switching to stance mode" << std::endl;
    } else {
      // Smart stop conditions not met, maintain in-place stepping
      currentCommand_.setzero();
      currentCommand_.cmdStance_ = 0; // Keep walking mode for in-place stepping
      ROS_DEBUG("[RlGaitReceiver] Smart stop conditions not met, maintaining in-place stepping");
    }
  } else {
    // Significant velocity command, switch to walking mode
    currentCommand_.cmdVelLineX_ = smoothed_cmd_vel_.linear.x;
    currentCommand_.cmdVelLineY_ = smoothed_cmd_vel_.linear.y;
    currentCommand_.cmdVelLineZ_ = smoothed_cmd_vel_.linear.z;
    currentCommand_.cmdVelAngularX_ = smoothed_cmd_vel_.angular.x;
    currentCommand_.cmdVelAngularY_ = smoothed_cmd_vel_.angular.y;
    currentCommand_.cmdVelAngularZ_ = smoothed_cmd_vel_.angular.z;
    currentCommand_.cmdStance_ = 0; // Walking mode
    
    // Reset smart stop checking when there's significant velocity command
    // resetSmartStopCheck();
  }
}

CommandDataRL RlGaitReceiver::getCurrentCommand() const
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  return currentCommand_;
}



void RlGaitReceiver::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  if (!enabled_) {
    return;
  }
  
  // Apply velocity smoothing
  ros::Time current_time = ros::Time::now();
  geometry_msgs::Twist smoothed_vel = *msg;//smoothVelocityCommand(*msg, current_time);
  
  std::lock_guard<std::mutex> lock(command_mutex_);
  smoothed_cmd_vel_ = smoothed_vel;
  
  ROS_DEBUG("[RlGaitReceiver] Received velocity command: lin(%.3f, %.3f, %.3f) ang(%.3f, %.3f, %.3f)",
            smoothed_vel.linear.x, smoothed_vel.linear.y, smoothed_vel.linear.z,
            smoothed_vel.angular.x, smoothed_vel.angular.y, smoothed_vel.angular.z);
}

void RlGaitReceiver::gaitNameCallback(const std_msgs::String::ConstPtr& msg)
{
  if (!enabled_) {
    return;
  }
  
  const std::string& gait_name = msg->data;
  ROS_INFO_STREAM("[RlGaitReceiver] Received gait name request: " << gait_name);
  
  std::lock_guard<std::mutex> lock(command_mutex_);
  
  // 根据gait名字手动切换cmdStance_状态
  if (gait_name == "stance") {
    currentCommand_.setzero();
    currentCommand_.cmdStance_ = 1.0;
    smart_stop_enabled_ = true;
    ROS_INFO("[RlGaitReceiver] Manually switched to stance mode");
  } else if (gait_name == "walk" || gait_name == "trot") {
    currentCommand_.cmdStance_ = 0.0;
    smart_stop_enabled_ = false;
    ROS_INFO_STREAM("[RlGaitReceiver] Manually switched to walking mode for gait: " << gait_name);
  } else {
    ROS_WARN("[RlGaitReceiver] Unknown gait name: %s, keeping current stance state", gait_name.c_str());
  }
}

bool RlGaitReceiver::checkSmartStopConditions(const vector_t& torsostate, const vector_t& feetPositions)
{
  // Check if we have valid data
  if (torsostate.size() < 12) {
    std::cout << "[RlGaitReceiver] Invalid torsostate data for smart stop check" << std::endl;
    return false;
  }
  
  if (feetPositions.size() < 24) {
    ROS_DEBUG("[RlGaitReceiver] Invalid feet position data size: %zu", feetPositions.size());
    return false;
  }
  
  // Extract torso state from 12D state vector: 
  // torsostate: [x, y, z, yaw, pitch, roll, vx, vy, vz, angularVx, angularVy, angularVz]
  vector3_t global_linear_velocity = torsostate.segment<3>(6);  // vx, vy, vz in global frame
  vector3_t orientation = torsostate.segment<3>(3);            // yaw, pitch, roll
  vector3_t torso_position = torsostate.segment<3>(0);         // x, y, z in global frame
  
  // Get yaw angle for coordinate transformation
  double yaw = orientation(0);
  double cos_yaw = std::cos(yaw);
  double sin_yaw = std::sin(yaw);
  
  // 1. Check torso velocity in local frame
  double forward_velocity = global_linear_velocity(0) * cos_yaw + 
                           global_linear_velocity(1) * sin_yaw;
  bool torso_slow = std::abs(forward_velocity) < torso_velocity_threshold_;
  
  
  // 2. Check feet alignment in local frame
  // Calculate average position for left foot (4 contact points)
  vector3_t lf_pos_w = vector3_t::Zero();
  for (int i = 0; i < 4; i++) {
    lf_pos_w.head(2) += feetPositions.segment<2>(i * 3) / 4.0;
  }
  
  // Calculate average position for right foot (4 contact points)  
  vector3_t rf_pos_w = vector3_t::Zero();
  for (int i = 0; i < 4; i++) {
    rf_pos_w.head(2) += feetPositions.segment<2>(i * 3 + 12) / 4.0;
  }
  
  // Transform feet positions to local frame (relative to torso)
  vector3_t lf_pos_local = lf_pos_w - torso_position;
  vector3_t rf_pos_local = rf_pos_w - torso_position;
  
  // Rotate to local frame
  vector3_t lf_pos_body;
  vector3_t rf_pos_body;
  lf_pos_body(0) = lf_pos_local(0) * cos_yaw + lf_pos_local(1) * sin_yaw;
  lf_pos_body(1) = -lf_pos_local(0) * sin_yaw + lf_pos_local(1) * cos_yaw;
  lf_pos_body(2) = lf_pos_local(2);
  
  rf_pos_body(0) = rf_pos_local(0) * cos_yaw + rf_pos_local(1) * sin_yaw;
  rf_pos_body(1) = -rf_pos_local(0) * sin_yaw + rf_pos_local(1) * cos_yaw;
  rf_pos_body(2) = rf_pos_local(2);
  
  // Check feet alignment in local x-direction (forward/backward)
  double feet_x_diff = std::abs(lf_pos_body(0) - rf_pos_body(0));
  bool feet_aligned = feet_x_diff < feet_alignment_threshold_;
  

  
  
  // Both conditions must be met for smart stop
  return torso_slow && feet_aligned;
}

bool RlGaitReceiver::shouldSmartStop(const vector_t& torsostate, const vector_t& feetPositions)
{

  // Check both conditions in local frame: torso velocity and feet alignment
  bool conditions_met = checkSmartStopConditions(torsostate, feetPositions);
  
  return conditions_met;
}


double RlGaitReceiver::calculateVelocityMagnitude(const geometry_msgs::Twist& cmd_vel)
{
  double linear_magnitude = std::sqrt(cmd_vel.linear.x * cmd_vel.linear.x + 
                                     cmd_vel.linear.y * cmd_vel.linear.y + 
                                     cmd_vel.linear.z * cmd_vel.linear.z);
  double angular_magnitude = std::sqrt(cmd_vel.angular.x * cmd_vel.angular.x + 
                                      cmd_vel.angular.y * cmd_vel.angular.y + 
                                      cmd_vel.angular.z * cmd_vel.angular.z);
  
  // Weight angular velocity less than linear velocity
  return linear_magnitude + 0.1 * angular_magnitude;
}

geometry_msgs::Twist RlGaitReceiver::smoothVelocityCommand(const geometry_msgs::Twist& cmd_vel, const ros::Time& current_time)
{
  geometry_msgs::Twist smoothed_vel;
  
  // Calculate time delta
  double dt = (current_time - last_velocity_update_time_).toSec();
  if (dt <= 0.0) {
    return previous_cmd_vel_;
  }
  
  // Apply smoothing factor
  double smooth_factor = std::min(velocity_smooth_factor_ / dt, 1.0);
  
  // Smooth linear velocities
  smoothed_vel.linear.x = previous_cmd_vel_.linear.x + smooth_factor * (cmd_vel.linear.x - previous_cmd_vel_.linear.x);
  smoothed_vel.linear.y = previous_cmd_vel_.linear.y + smooth_factor * (cmd_vel.linear.y - previous_cmd_vel_.linear.y);
  smoothed_vel.linear.z = previous_cmd_vel_.linear.z + smooth_factor * (cmd_vel.linear.z - previous_cmd_vel_.linear.z);
  
  // Smooth angular velocities
  smoothed_vel.angular.x = previous_cmd_vel_.angular.x + smooth_factor * (cmd_vel.angular.x - previous_cmd_vel_.angular.x);
  smoothed_vel.angular.y = previous_cmd_vel_.angular.y + smooth_factor * (cmd_vel.angular.y - previous_cmd_vel_.angular.y);
  smoothed_vel.angular.z = previous_cmd_vel_.angular.z + smooth_factor * (cmd_vel.angular.z - previous_cmd_vel_.angular.z);
  
  // Limit maximum change
  double linear_change = std::sqrt(std::pow(smoothed_vel.linear.x - previous_cmd_vel_.linear.x, 2) +
                                  std::pow(smoothed_vel.linear.y - previous_cmd_vel_.linear.y, 2) +
                                  std::pow(smoothed_vel.linear.z - previous_cmd_vel_.linear.z, 2));
  
  if (linear_change > max_velocity_change_) {
    double scale = max_velocity_change_ / linear_change;
    smoothed_vel.linear.x = previous_cmd_vel_.linear.x + scale * (smoothed_vel.linear.x - previous_cmd_vel_.linear.x);
    smoothed_vel.linear.y = previous_cmd_vel_.linear.y + scale * (smoothed_vel.linear.y - previous_cmd_vel_.linear.y);
    smoothed_vel.linear.z = previous_cmd_vel_.linear.z + scale * (smoothed_vel.linear.z - previous_cmd_vel_.linear.z);
  }
  
  // Update previous command and time
  previous_cmd_vel_ = smoothed_vel;
  last_velocity_update_time_ = current_time;
  
  return smoothed_vel;
}

} // namespace humanoid
} // namespace ocs2
