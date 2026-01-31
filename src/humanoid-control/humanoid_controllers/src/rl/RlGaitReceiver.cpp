/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#include "humanoid_controllers/rl/RlGaitReceiver.h"
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_core/misc/LoadData.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>
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
  , feet_alignment_threshold_(0.05)
  , velocity_smooth_factor_(0.1)
  , max_velocity_change_(0.5)
  , velocity_smooth_time_(0.1)
  , angular_velocity_smooth_factor_(0.5)      // 默认更强的角速度平滑
  , angular_velocity_change_threshold_(0.2)   // 默认角速度变化阈值 0.2 rad/s
  , angular_velocity_max_rate_(2.0)           // 默认最大角速度变化率 2.0 rad/s²
  , enable_mixed_mode_(false)                // 默认不启用混合运动模式
  , angular_vel_threshold_(0.05)             // 角速度阈值默认0.05 rad/s
  , max_linear_vel_with_angular_(0.5)         // 有角速度时最大线速度默认0.5 m/s
  , linear_vel_threshold_(0.05)              // 线速度阈值默认0.05 m/s
  , max_angular_vel_with_linear_(0.5)         // 有线速度时最大角速度默认0.5 rad/s
  , smooth_transition_(true)                 // 默认启用平滑过渡
  , transition_factor_(0.8)                   // 默认过渡因子为0.8
  , in_place_step_duration_(2.0)
  , enable_in_place_stepping_(true)
  , is_in_place_stepping_(false)
  , is_real_(false)
{
  // Get is_real parameter from ROS parameter server
  if (!nh_.getParam("/is_real", is_real_))
  {
    ROS_WARN("[RlGaitReceiver] /is_real not found in ROS params, using default: false (simulation mode)");
    is_real_ = false;
  }
  
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
  
  // Initialize in-place stepping velocities
  in_place_step_velocity_.linear.x = 0.0;
  in_place_step_velocity_.linear.y = 0.0;
  in_place_step_velocity_.linear.z = 0.0;
  in_place_step_velocity_.angular.x = 0.0;
  in_place_step_velocity_.angular.y = 0.0;
  in_place_step_velocity_.angular.z = 0.0;
  
  // Subscribe to cmd_vel topic
  cmd_vel_sub_ = nh_.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, 
    &RlGaitReceiver::cmdVelCallback, this);
  
  // Subscribe to gait name request topic
  gait_name_sub_ = nh_.subscribe<std_msgs::String>("/humanoid_mpc_gait_name_request", 10,
    &RlGaitReceiver::gaitNameCallback, this);
  
  ROS_INFO("[RlGaitReceiver] Initialized with smart stop detection enabled");
}

void RlGaitReceiver::setEnabled(bool enable)
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  enabled_ = enable;
}

bool RlGaitReceiver::isEnabled() const
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  return enabled_;
}

void RlGaitReceiver::update(const ros::Time& time, const vector_t& torsostate, const vector_t& feetPositions)
{
  if (!enabled_) {
    return;
  }
  double velocity_magnitude = calculateVelocityMagnitude(smoothed_cmd_vel_);
  std::lock_guard<std::mutex> lock(command_mutex_);

  if (velocity_magnitude < 0.01 && currentCommand_.cmdStance_ == 1) // Low velocity and already in stance mode
    return;
  
  // Update in-place stepping states
  if (isInPlaceSteppingActive())
  {
    updateInPlaceStepping(time);
  }
  
  // Process current velocity command
  
  if (velocity_magnitude < 0.01 && currentCommand_.cmdStance_ == 0) // Low velocity, already walking, and smart stop enabled
  {
    // Velocity is very small, check for smart stop
    if (smart_stop_enabled_ && shouldSmartStop(torsostate, feetPositions)) {
      // Smart stop conditions met, switch to stance mode
      currentCommand_.setzero();
      currentCommand_.cmdStance_ = 1;
      stopInPlaceStepping();
      std::cout << "[RlGaitReceiver] Smart stop conditions met, switching to stance mode" << std::endl;
    } else {
      // Smart stop conditions not met, maintain in-place stepping
      // Use configured in-place stepping velocity
      if (isInPlaceSteppingActive())
      {
        currentCommand_.cmdVelLineX_ = in_place_step_velocity_.linear.x;
        currentCommand_.cmdVelLineY_ = in_place_step_velocity_.linear.y;
        currentCommand_.cmdVelLineZ_ = in_place_step_velocity_.linear.z;
        currentCommand_.cmdVelAngularX_ = in_place_step_velocity_.angular.x;
        currentCommand_.cmdVelAngularY_ = in_place_step_velocity_.angular.y;
        currentCommand_.cmdVelAngularZ_ = in_place_step_velocity_.angular.z;
        currentCommand_.cmdStance_ = 0; // Keep walking mode for in-place stepping
      }
      else
      {
        // Start in-place stepping if enabled
        if (enable_in_place_stepping_)
        {
          startInPlaceStepping(time);
          currentCommand_.cmdVelLineX_ = in_place_step_velocity_.linear.x;
          currentCommand_.cmdVelLineY_ = in_place_step_velocity_.linear.y;
          currentCommand_.cmdVelLineZ_ = in_place_step_velocity_.linear.z;
          currentCommand_.cmdVelAngularX_ = in_place_step_velocity_.angular.x;
          currentCommand_.cmdVelAngularY_ = in_place_step_velocity_.angular.y;
          currentCommand_.cmdVelAngularZ_ = in_place_step_velocity_.angular.z;
          currentCommand_.cmdStance_ = 0; // Keep walking mode for in-place stepping
        }
        else
        {
          currentCommand_.setzero();
          currentCommand_.cmdStance_ = 0; // Keep walking mode for in-place stepping
        }
      }
      ROS_DEBUG("[RlGaitReceiver] Smart stop conditions not met, maintaining in-place stepping");
    }
  } else {
    // Significant velocity command, switch to walking mode
    // Stop in-place stepping when there's significant velocity command
    stopInPlaceStepping();
    
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
  geometry_msgs::Twist smoothed_vel = smoothVelocityCommand(*msg, current_time);
  
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
  
  // Manually switch cmdStance_ state based on gait name
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

geometry_msgs::Twist RlGaitReceiver::applyMixedMotionLimits(const geometry_msgs::Twist& cmd_vel) const
{
  if (!enable_mixed_mode_) {
    // If mixed mode is disabled, return original command
    return cmd_vel;
  }
  
  geometry_msgs::Twist limited_vel = cmd_vel;
  
  // Calculate magnitudes
  double angular_z_mag = std::abs(limited_vel.angular.z);
  double linear_xy_mag = std::sqrt(limited_vel.linear.x * limited_vel.linear.x + 
                                  limited_vel.linear.y * limited_vel.linear.y);
  
  // Check if we need to limit linear velocity due to angular velocity
  if (angular_z_mag > angular_vel_threshold_)
  {
    // When there's angular velocity, limit linear velocity to maxLinearVelWithAngular
    if (linear_xy_mag > max_linear_vel_with_angular_)
    {
      double scale_factor = max_linear_vel_with_angular_ / linear_xy_mag;
      limited_vel.linear.x *= scale_factor;
      limited_vel.linear.y *= scale_factor;
      
      ROS_DEBUG_THROTTLE(1.0, "[RlGaitReceiver] Applied linear velocity limit due to turning: scale=%.2f, angularZ=%.2f, linearXY=%.2f", 
                         scale_factor, angular_z_mag, linear_xy_mag);
    }
  }
  
  // Check if we need to limit angular velocity due to linear velocity
  if (linear_xy_mag > linear_vel_threshold_)
  {
    // When there's linear velocity, limit angular velocity to maxAngularVelWithLinear
    if (angular_z_mag > max_angular_vel_with_linear_)
    {
      double scale_factor = max_angular_vel_with_linear_ / angular_z_mag;
      limited_vel.angular.z *= scale_factor;
      
      ROS_DEBUG_THROTTLE(1.0, "[RlGaitReceiver] Applied angular velocity limit due to forward movement: scale=%.2f, linearXY=%.2f, angularZ=%.2f", 
                         scale_factor, linear_xy_mag, angular_z_mag);
    }
  }
  
  return limited_vel;
}

geometry_msgs::Twist RlGaitReceiver::smoothVelocityCommand(const geometry_msgs::Twist& cmd_vel, const ros::Time& current_time)
{
  geometry_msgs::Twist smoothed_vel = smoothed_cmd_vel_;  // 从当前平滑速度开始
  
  // Calculate time delta
  double dt = (current_time - last_velocity_update_time_).toSec();
  if (dt <= 0.0) {
    return smoothed_cmd_vel_;
  }
  
  // Calculate velocity differences
  double vel_diff_x = cmd_vel.linear.x - smoothed_vel.linear.x;
  double vel_diff_y = cmd_vel.linear.y - smoothed_vel.linear.y;
  double vel_diff_z = cmd_vel.linear.z - smoothed_vel.linear.z;
  double ang_diff_x = cmd_vel.angular.x - smoothed_vel.angular.x;
  double ang_diff_y = cmd_vel.angular.y - smoothed_vel.angular.y;
  double ang_diff_z = cmd_vel.angular.z - smoothed_vel.angular.z;
  
  // Apply smoothing factor
  double smooth_factor = std::min(velocity_smooth_factor_ / dt, 1.0);
  
  // Smooth linear velocities
  smoothed_vel.linear.x += vel_diff_x * smooth_factor;
  smoothed_vel.linear.y += vel_diff_y * smooth_factor;
  smoothed_vel.linear.z += vel_diff_z * smooth_factor;
  
  // Smooth angular velocities (x and y use normal smoothing)
  smoothed_vel.angular.x += ang_diff_x * smooth_factor;
  smoothed_vel.angular.y += ang_diff_y * smooth_factor;
  
  // Special handling for angular.z during turning (when there's linear velocity)
  double angular_z_change = std::abs(ang_diff_z);
  bool has_linear_velocity = (std::abs(cmd_vel.linear.x) > 0.01 || std::abs(cmd_vel.linear.y) > 0.01);
  
  if (has_linear_velocity && angular_z_change > angular_velocity_change_threshold_)
  {
    // Use special angular velocity smoothing and rate limiting for turning
    double angular_smooth_factor = angular_velocity_smooth_factor_;
    
    // Apply maximum angular velocity change rate limit
    double max_angular_change = angular_velocity_max_rate_ * dt;
    double limited_ang_diff_z = std::max(-max_angular_change, std::min(max_angular_change, ang_diff_z));
    
    smoothed_vel.angular.z += limited_ang_diff_z * angular_smooth_factor;
  }
  else
  {
    // Normal smoothing for angular.z when not turning
    smoothed_vel.angular.z += ang_diff_z * smooth_factor;
  }
  
  // Limit maximum linear velocity change
  double linear_change = std::sqrt(vel_diff_x * vel_diff_x + vel_diff_y * vel_diff_y + vel_diff_z * vel_diff_z);
  
  if (linear_change > max_velocity_change_) {
    double scale = max_velocity_change_ / linear_change;
    smoothed_vel.linear.x = smoothed_cmd_vel_.linear.x + scale * vel_diff_x;
    smoothed_vel.linear.y = smoothed_cmd_vel_.linear.y + scale * vel_diff_y;
    smoothed_vel.linear.z = smoothed_cmd_vel_.linear.z + scale * vel_diff_z;
  }
  
  // Apply mixed motion limits to velocity commands
  smoothed_vel = applyMixedMotionLimits(smoothed_vel);
  
  // Update previous command and time
  previous_cmd_vel_ = smoothed_vel;
  last_velocity_update_time_ = current_time;
  smoothed_cmd_vel_ = smoothed_vel;
  
  return smoothed_vel;
}

void RlGaitReceiver::loadInPlaceStepConfig(const std::string& config_file, bool verbose)
{
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(config_file, pt);
  
  // Load in-place stepping velocity configuration
  if (pt.find("inPlaceStepVelocity") != pt.not_found()) {
    loadData::loadPtreeValue(pt, in_place_step_velocity_.linear.x, "inPlaceStepVelocity.linearX", verbose);
    loadData::loadPtreeValue(pt, in_place_step_velocity_.linear.y, "inPlaceStepVelocity.linearY", verbose);
    loadData::loadPtreeValue(pt, in_place_step_velocity_.linear.z, "inPlaceStepVelocity.linearZ", verbose);
    loadData::loadPtreeValue(pt, in_place_step_velocity_.angular.x, "inPlaceStepVelocity.angularX", verbose);
    loadData::loadPtreeValue(pt, in_place_step_velocity_.angular.y, "inPlaceStepVelocity.angularY", verbose);
    loadData::loadPtreeValue(pt, in_place_step_velocity_.angular.z, "inPlaceStepVelocity.angularZ", verbose);
    
    std::cout << "[RlGaitReceiver] inPlaceStepVelocity loaded (" << (is_real_ ? "real robot" : "simulation") << " mode):" << std::endl;
    std::cout << "  linearX: " << in_place_step_velocity_.linear.x << std::endl;
    std::cout << "  linearY: " << in_place_step_velocity_.linear.y << std::endl;
    std::cout << "  angularZ: " << in_place_step_velocity_.angular.z << std::endl;
  } else {
    std::cout << "[RlGaitReceiver] Warning: inPlaceStepVelocity not found in config file, using default values" << std::endl;
  }
  
  // Load mixed motion limits parameters
  if (pt.find("mixedMotionLimits") != pt.not_found()) {
    loadData::loadPtreeValue(pt, enable_mixed_mode_, "mixedMotionLimits.enableMixedMode", verbose);
    loadData::loadPtreeValue(pt, angular_vel_threshold_, "mixedMotionLimits.angularVelThreshold", verbose);
    loadData::loadPtreeValue(pt, max_linear_vel_with_angular_, "mixedMotionLimits.maxLinearVelWithAngular", verbose);
    loadData::loadPtreeValue(pt, linear_vel_threshold_, "mixedMotionLimits.linearVelThreshold", verbose);
    loadData::loadPtreeValue(pt, max_angular_vel_with_linear_, "mixedMotionLimits.maxAngularVelWithLinear", verbose);
    loadData::loadPtreeValue(pt, smooth_transition_, "mixedMotionLimits.smoothTransition", verbose);
    loadData::loadPtreeValue(pt, transition_factor_, "mixedMotionLimits.transitionFactor", verbose);
    
    ROS_INFO("[RlGaitReceiver] Mixed motion limits loaded: enableMixedMode=%s, maxLinearVelWithAngular=%.2f, maxAngularVelWithLinear=%.2f",
             enable_mixed_mode_ ? "true" : "false", max_linear_vel_with_angular_, max_angular_vel_with_linear_);
  } else {
    ROS_WARN("[RlGaitReceiver] No mixedMotionLimits section found in config file, using default values");
  }
}

void RlGaitReceiver::startInPlaceStepping(const ros::Time& current_time)
{
  if (!enable_in_place_stepping_)
  {
    return;
  }
  
  is_in_place_stepping_ = true;
  in_place_step_start_time_ = current_time;
}

void RlGaitReceiver::stopInPlaceStepping()
{
  is_in_place_stepping_ = false;
}

void RlGaitReceiver::updateInPlaceStepping(const ros::Time& current_time)
{
  if (!is_in_place_stepping_)
    return;
  
  double elapsed_time = (current_time - in_place_step_start_time_).toSec();
  
  if (elapsed_time >= in_place_step_duration_)
  {
    stopInPlaceStepping();
  }
}

bool RlGaitReceiver::isInPlaceSteppingActive() const
{
  return is_in_place_stepping_;
}

} // namespace humanoid
} // namespace ocs2
