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
  , reuse_walk_command_in_stance_(false)
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
  nh_.param<double>("/rl_gait_receiver/robot_action_active_timeout",
                    robot_action_active_timeout_, robot_action_active_timeout_);
  robot_action_active_timeout_ = std::max(0.1, robot_action_active_timeout_);
  {
    int robot_version_int = 0;
    nh_.param("/robot_version", robot_version_int, 0);
    is_v17_ = (robot_version_int == 17);
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
  latest_cmd_vel_ = smoothed_cmd_vel_;
  previous_cmd_vel_ = smoothed_cmd_vel_;
  smoothed_cmd_pose_.linear.z = 0.0;
  smoothed_cmd_pose_.angular.y = 0.0;
  previous_cmd_pose_ = smoothed_cmd_pose_;
  latest_gait_name_ = "stance";
  trot_latched_ = false;
  last_velocity_update_time_ = ros::Time::now();
  last_pose_update_time_ = ros::Time::now();
  last_cmd_vel_msg_time_ = ros::Time::now();
  
  // Initialize in-place stepping velocities
  in_place_step_velocity_.linear.x = 0.0;
  in_place_step_velocity_.linear.y = 0.0;
  in_place_step_velocity_.linear.z = 0.0;
  in_place_step_velocity_.angular.x = 0.0;
  in_place_step_velocity_.angular.y = 0.0;
  in_place_step_velocity_.angular.z = 0.0;
  
  nh_.param("/cmd_pose_topic", cmd_pose_topic_, cmd_pose_topic_);
  if (cmd_pose_topic_.empty())
  {
    cmd_pose_topic_ = "/cmd_pose";
  }

  // Subscribe to cmd_vel topic
  cmd_vel_sub_ = nh_.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, 
    &RlGaitReceiver::cmdVelCallback, this);

  cmd_pose_sub_ = nh_.subscribe<geometry_msgs::Twist>(cmd_pose_topic_, 10,
    &RlGaitReceiver::cmdPoseCallback, this);
  
  // Subscribe to gait name request topic
  gait_name_sub_ = nh_.subscribe<std_msgs::String>("/humanoid_mpc_gait_name_request", 10,
    &RlGaitReceiver::gaitNameCallback, this);

  robot_action_state_sub_ = nh_.subscribe<humanoid_plan_arm_trajectory::RobotActionState>(
    "/robot_action_state", 10, &RlGaitReceiver::robotActionStateCallback, this);
  
  ROS_INFO("[RlGaitReceiver] Initialized with smart stop detection enabled, cmd_pose='%s'", cmd_pose_topic_.c_str());
}

void RlGaitReceiver::setEnabled(bool enable)
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  enabled_ = enable;
}

void RlGaitReceiver::resetToStance()
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  currentCommand_.setzero();
  currentCommand_.cmdStance_ = 1.0;
  smoothed_cmd_vel_.linear.x = 0.0;
  smoothed_cmd_vel_.linear.y = 0.0;
  smoothed_cmd_vel_.linear.z = 0.0;
  smoothed_cmd_vel_.angular.x = 0.0;
  smoothed_cmd_vel_.angular.y = 0.0;
  smoothed_cmd_vel_.angular.z = 0.0;
  previous_cmd_vel_ = smoothed_cmd_vel_;
  ROS_INFO("[RlGaitReceiver] resetToStance: cmdStance=1, velocity cleared");
}

bool RlGaitReceiver::isEnabled() const
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  return enabled_;
}

void RlGaitReceiver::resetCommandState(bool stance_mode)
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  currentCommand_.setzero();
  currentCommand_.cmdStance_ = stance_mode ? 1.0 : 0.0;
  smoothed_cmd_vel_.linear.x = 0.0;
  smoothed_cmd_vel_.linear.y = 0.0;
  smoothed_cmd_vel_.linear.z = 0.0;
  smoothed_cmd_vel_.angular.x = 0.0;
  smoothed_cmd_vel_.angular.y = 0.0;
  smoothed_cmd_vel_.angular.z = 0.0;
  latest_cmd_vel_ = smoothed_cmd_vel_;
  previous_cmd_vel_ = smoothed_cmd_vel_;
  smoothed_cmd_pose_.linear.z = 0.0;
  smoothed_cmd_pose_.angular.y = 0.0;
  previous_cmd_pose_ = smoothed_cmd_pose_;
  latest_gait_name_ = stance_mode ? "stance" : "walk";
  pending_gait_name_.clear();
  trot_latched_ = false;
  last_velocity_update_time_ = ros::Time::now();
  stopInPlaceStepping();
}

void RlGaitReceiver::overrideCommandState(const CommandDataRL& command)
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  currentCommand_.cmdVelLineX_ = command.cmdVelLineX_;
  currentCommand_.cmdVelLineY_ = command.cmdVelLineY_;
  currentCommand_.cmdVelLineZ_ = command.cmdVelLineZ_;
  currentCommand_.cmdVelAngularX_ = command.cmdVelAngularX_;
  currentCommand_.cmdVelAngularY_ = command.cmdVelAngularY_;
  currentCommand_.cmdVelAngularZ_ = command.cmdVelAngularZ_;
  currentCommand_.cmdPostureSquatHeight_ = command.cmdPostureSquatHeight_;
  currentCommand_.cmdPostureTrunkPitch_ = command.cmdPostureTrunkPitch_;
  currentCommand_.cmdStance_ = command.cmdStance_;

  if (currentCommand_.cmdStance_ >= 0.5)
  {
    smart_stop_enabled_ = true;
    stopInPlaceStepping();
  }
}

void RlGaitReceiver::setSwitchVelocityScale(double scale)
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  switch_velocity_scale_ = std::clamp(scale, 0.0, 1.0);
}

void RlGaitReceiver::setReuseWalkCommandInStance(bool enable)
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  reuse_walk_command_in_stance_ = enable;
}

void RlGaitReceiver::setAmpHandController(bool enable)
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  is_amp_hand_controller_ = enable;
}

void RlGaitReceiver::setCommandBufferCallback(std::function<bool()> callback)
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  command_buffer_callback_ = std::move(callback);
}

void RlGaitReceiver::resetVelocityState()
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  currentCommand_.setzero();
  smoothed_cmd_vel_.linear.x = 0.0;
  smoothed_cmd_vel_.linear.y = 0.0;
  smoothed_cmd_vel_.linear.z = 0.0;
  smoothed_cmd_vel_.angular.x = 0.0;
  smoothed_cmd_vel_.angular.y = 0.0;
  smoothed_cmd_vel_.angular.z = 0.0;
  previous_cmd_vel_ = smoothed_cmd_vel_;
  last_velocity_update_time_ = ros::Time::now();
  last_cmd_vel_msg_time_ = ros::Time::now();
  stopInPlaceStepping();
}

void RlGaitReceiver::update(const ros::Time& time, const vector_t& torsostate, const vector_t& feetPositions)
{
  if (!enabled_) {
    return;
  }
  std::function<bool()> command_buffer_callback;
  {
    std::lock_guard<std::mutex> lock(command_mutex_);

    // amp_hand: 摇杆零速发布窗口结束后，控制环仍继续把 smoothed_cmd_vel 衰减到 0.05（仅前进）
    const double cmd_vel_silent_sec = (time - last_cmd_vel_msg_time_).toSec();
    constexpr double kPassiveDecayForwardCmdXMin = 0.05;
    if (is_amp_hand_controller_ && cmd_vel_silent_sec > 0.25 &&
        smoothed_cmd_vel_.linear.x > kPassiveDecayForwardCmdXMin)
    {
      geometry_msgs::Twist zero_cmd;
      smoothed_cmd_vel_ = smoothVelocityCommand(zero_cmd, time);
    }

    command_buffer_callback = command_buffer_callback_;
  }
  const bool command_blocked = command_buffer_callback && command_buffer_callback();

  std::lock_guard<std::mutex> lock(command_mutex_);

  syncPostureToCommand();

  if (!pending_gait_name_.empty() && !command_blocked)
  {
    const std::string gait_name = pending_gait_name_;
    pending_gait_name_.clear();
    currentCommand_.cmdStance_ = 0.0;
    if (gait_name == "trot")
    {
      trot_latched_ = true;
    }
    smart_stop_enabled_ = false;
    ROS_INFO_STREAM("[RlGaitReceiver] Applied pending walking gait after controller switch: " << gait_name);
  }

  double velocity_magnitude = calculateVelocityMagnitude(smoothed_cmd_vel_);
  // sim-to-sim: 站立模式下复用行走三维指令（x/y/yaw）作为姿态命令通道。
  // 这与训练侧 vel_command_b 的复用语义一致：x->弯腰，y->保留，yaw(z)->下蹲高度。
  if (currentCommand_.cmdStance_ == 1 && reuse_walk_command_in_stance_)
  {
    // x->弯腰：仅允许正向（向前弯），屏蔽负向 cmd_x（向后弯）
    currentCommand_.cmdVelLineX_ = std::max(0.0, smoothed_cmd_vel_.linear.x);
    currentCommand_.cmdVelLineY_ = smoothed_cmd_vel_.linear.y;
    currentCommand_.cmdVelLineZ_ = smoothed_cmd_vel_.linear.z;
    currentCommand_.cmdVelAngularX_ = smoothed_cmd_vel_.angular.x;
    currentCommand_.cmdVelAngularY_ = smoothed_cmd_vel_.angular.y;
    // 站立姿态模式下，将 base 高度偏移通道复用为策略的第 3 个速度命令输入。
    currentCommand_.cmdVelAngularZ_ = smoothed_cmd_vel_.linear.z;
    currentCommand_.cmdStance_ = 1;
    return;
  }

  if (velocity_magnitude < 0.01 && currentCommand_.cmdStance_ == 1) // Low velocity and already in stance mode
  {
    // 走不停腿：动作期间允许行走时，若摇杆仍有输入则切回 walking，不卡在 stance
    if (allow_walking_during_action_)
    {
      const double latest_linear_mag = std::hypot(latest_cmd_vel_.linear.x, latest_cmd_vel_.linear.y);
      const bool has_active_cmd = (latest_linear_mag >= 0.01 ||
                                   std::abs(latest_cmd_vel_.angular.z) >= 0.01 ||
                                   std::abs(latest_cmd_vel_.linear.z) >= 0.01);
      if (has_active_cmd)
      {
        currentCommand_.cmdStance_ = 0;
        // fall through to walking transition below
      }
      else
      {
        return;
      }
    }
    else
    {
      return;
    }
  }
  
  // Update in-place stepping states
  bool was_stepping = isInPlaceSteppingActive();
  if (was_stepping)
  {
    updateInPlaceStepping(time);
  }
  // Process current velocity command

  if (velocity_magnitude < 0.01 && currentCommand_.cmdStance_ == 0) // Low velocity, already walking, and smart stop enabled
  {
    // Velocity is very small, check for smart stop
    if (smart_stop_enabled_ && !allow_walking_during_action_ && shouldSmartStop(torsostate, feetPositions)) {
      // Smart stop conditions met.
      // amp_hand_controller 保持 walking 模式 (模型内部有自然停下)，其余控制器切 stance。
      // 走不停腿：动作期间禁止 smart-stop 切 stance，避免打断行走。
      currentCommand_.setzero();
      currentCommand_.cmdStance_ = is_amp_hand_controller_ ? 0 : 1;
      stopInPlaceStepping();
      if (!is_amp_hand_controller_)
      {
        std::cout << "[RlGaitReceiver] Smart stop conditions met, switching to stance mode" << std::endl;
      }
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
    
    currentCommand_.cmdVelLineX_ = smoothed_cmd_vel_.linear.x * switch_velocity_scale_;
    currentCommand_.cmdVelLineY_ = smoothed_cmd_vel_.linear.y * switch_velocity_scale_;
    currentCommand_.cmdVelLineZ_ = smoothed_cmd_vel_.linear.z * switch_velocity_scale_;
    currentCommand_.cmdVelAngularX_ = smoothed_cmd_vel_.angular.x * switch_velocity_scale_;
    currentCommand_.cmdVelAngularY_ = smoothed_cmd_vel_.angular.y * switch_velocity_scale_;
    currentCommand_.cmdVelAngularZ_ = smoothed_cmd_vel_.angular.z * switch_velocity_scale_;
    currentCommand_.cmdStance_ = 0; // Walking mode
    
    // Reset smart stop checking when there's significant velocity command
    // resetSmartStopCheck();
  }

  // Y方向行走补偿：横向行走时注入 X/Z 偏置（原地踏步/站立分支不受影响，linear_y 为 0）
  applyYDirectionCompensation();
}

CommandDataRL RlGaitReceiver::getCurrentCommand() const
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  return currentCommand_;
}

Eigen::Vector2d RlGaitReceiver::getCurrentPostureCommand() const
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  Eigen::Vector2d posture;
  posture << currentCommand_.cmdPostureSquatHeight_, currentCommand_.cmdPostureTrunkPitch_;
  return posture;
}

void RlGaitReceiver::resetPostureCommand()
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  smoothed_cmd_pose_.linear.z = 0.0;
  smoothed_cmd_pose_.angular.y = 0.0;
  currentCommand_.cmdPostureSquatHeight_ = 0.0;
  currentCommand_.cmdPostureTrunkPitch_ = 0.0;
  ROS_DEBUG("[RlGaitReceiver] Posture command reset to zero (squat/pitch cleared)");
}

void RlGaitReceiver::syncPostureToCommand()
{
  currentCommand_.cmdPostureSquatHeight_ = smoothed_cmd_pose_.linear.z;
  currentCommand_.cmdPostureTrunkPitch_ = smoothed_cmd_pose_.angular.y;
}

CommandDataRL RlGaitReceiver::getPolicyCommand() const
{
  CommandDataRL command;
  std::function<bool()> command_buffer_callback;
  bool robot_action_active = false;
  const ros::Time now = ros::Time::now();
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    command = currentCommand_;
    command_buffer_callback = command_buffer_callback_;
    robot_action_active = isRobotActionActiveLocked(now);
  }

  if ((robot_action_active && !resolveAllowWalkingDuringAction()) ||
      (command_buffer_callback && command_buffer_callback()))
  {
    command.setzero();
  }
  return command;
}

bool RlGaitReceiver::shouldBlockCommandExecution() const
{
  std::function<bool()> command_buffer_callback;
  bool robot_action_active = false;
  const ros::Time now = ros::Time::now();
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    command_buffer_callback = command_buffer_callback_;
    robot_action_active = isRobotActionActiveLocked(now);
  }
  return (robot_action_active && !resolveAllowWalkingDuringAction()) ||
         (command_buffer_callback && command_buffer_callback());
}

geometry_msgs::Twist RlGaitReceiver::getSmoothedCmdVel() const
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  return smoothed_cmd_vel_;
}


void RlGaitReceiver::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  std::function<bool()> command_buffer_callback;
  bool robot_action_active = false;
  const ros::Time now = ros::Time::now();
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    command_buffer_callback = command_buffer_callback_;
    robot_action_active = isRobotActionActiveLocked(now);
  }

  // 走不停腿：v17 / 成员标志 / ROS param（后者由 MoRE executeArmActionCallback 设置）
  const bool allow_walking = resolveAllowWalkingDuringAction();

  if (robot_action_active && !allow_walking)
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    latest_cmd_vel_ = geometry_msgs::Twist();
    smoothed_cmd_vel_ = geometry_msgs::Twist();
    previous_cmd_vel_ = geometry_msgs::Twist();
    currentCommand_.setzero();
    pending_gait_name_.clear();
    stopInPlaceStepping();
    ROS_DEBUG_THROTTLE(1.0, "[RlGaitReceiver] Suppress cmd_vel while robot action is active");
    return;
  }

  if (command_buffer_callback && command_buffer_callback())
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    latest_cmd_vel_ = *msg;
    ROS_DEBUG_THROTTLE(1.0, "[RlGaitReceiver] Block cmd_vel until walking controller is active");
    return;
  }

  // Apply velocity smoothing
  std::lock_guard<std::mutex> lock(command_mutex_);
  ros::Time current_time = ros::Time::now();
  last_cmd_vel_msg_time_ = current_time;
  geometry_msgs::Twist smoothed_vel = smoothVelocityCommand(*msg, current_time);
  
  latest_cmd_vel_ = *msg;
  smoothed_cmd_vel_ = smoothed_vel;
  
  ROS_DEBUG("[RlGaitReceiver] Received velocity command: lin(%.3f, %.3f, %.3f) ang(%.3f, %.3f, %.3f)",
            smoothed_vel.linear.x, smoothed_vel.linear.y, smoothed_vel.linear.z,
            smoothed_vel.angular.x, smoothed_vel.angular.y, smoothed_vel.angular.z);
}

void RlGaitReceiver::cmdPoseCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  if (!enabled_)
  {
    return;
  }

  geometry_msgs::Twist pose_msg;
  pose_msg.linear.z = msg->linear.z;
  pose_msg.angular.y = msg->angular.y;

  const ros::Time current_time = ros::Time::now();
  const geometry_msgs::Twist smoothed_pose = smoothPoseCommand(pose_msg, current_time);

  std::lock_guard<std::mutex> lock(command_mutex_);
  smoothed_cmd_pose_ = smoothed_pose;
  syncPostureToCommand();

  ROS_DEBUG("[RlGaitReceiver] Received posture command: squat_z=%.3f pitch_y=%.3f",
            smoothed_pose.linear.z, smoothed_pose.angular.y);
}

void RlGaitReceiver::robotActionStateCallback(const humanoid_plan_arm_trajectory::RobotActionState::ConstPtr& msg)
{
  const bool active = msg->state == 1;
  std::lock_guard<std::mutex> lock(command_mutex_);
  if (active)
  {
    last_robot_action_active_time_ = ros::Time::now();
  }
  if (robot_action_active_ == active)
  {
    return;
  }

  robot_action_active_ = active;
  // 走不停腿：v17 / 成员标志 / ROS param
  const bool allow_walking = resolveAllowWalkingDuringAction();
  if (robot_action_active_ && !allow_walking)
  {
    currentCommand_.setzero();
    latest_cmd_vel_ = geometry_msgs::Twist();
    smoothed_cmd_vel_ = geometry_msgs::Twist();
    previous_cmd_vel_ = geometry_msgs::Twist();
    smoothed_cmd_pose_ = geometry_msgs::Twist();
    previous_cmd_pose_ = geometry_msgs::Twist();
    pending_gait_name_.clear();
    trot_latched_ = false;
    stopInPlaceStepping();
    ROS_INFO("[RlGaitReceiver] Robot action active: suppress cmd_vel and hold stance");
  }
  else if (robot_action_active_)
  {
    ROS_INFO("[RlGaitReceiver] Robot action active: walking allowed (v17 or allow_walking flag)");
  }
  else
  {
    ROS_INFO("[RlGaitReceiver] Robot action inactive: cmd_vel accepted");
  }
}

bool RlGaitReceiver::resolveAllowWalkingDuringAction() const
{
  if (is_v17_ || allow_walking_during_action_)
  {
    return true;
  }
  bool allow = false;
  ros::param::param<bool>("/allow_walking_during_arm_action", allow, false);
  return allow;
}

bool RlGaitReceiver::isRobotActionActiveLocked(const ros::Time& now) const
{
  if (!robot_action_active_)
  {
    return false;
  }
  if (!last_robot_action_active_time_.isValid())
  {
    return true;
  }
  return (now - last_robot_action_active_time_).toSec() <= robot_action_active_timeout_;
}

void RlGaitReceiver::gaitNameCallback(const std_msgs::String::ConstPtr& msg)
{
  const std::string& gait_name = msg->data;
  ROS_INFO_STREAM("[RlGaitReceiver] Received gait name request: " << gait_name);

  std::function<bool()> command_buffer_callback;
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    command_buffer_callback = command_buffer_callback_;
  }
  const bool command_blocked = command_buffer_callback && command_buffer_callback();

  std::lock_guard<std::mutex> lock(command_mutex_);
  latest_gait_name_ = gait_name;
  
  // Manually switch cmdStance_ state based on gait name
  if (gait_name == "stance") {
    pending_gait_name_.clear();
    currentCommand_.setzero();
    currentCommand_.cmdStance_ = 1.0;
    trot_latched_ = false;
    latest_cmd_vel_ = geometry_msgs::Twist();
    smoothed_cmd_vel_ = geometry_msgs::Twist();
    previous_cmd_vel_ = geometry_msgs::Twist();
    smoothed_cmd_pose_ = geometry_msgs::Twist();
    previous_cmd_pose_ = geometry_msgs::Twist();
    stopInPlaceStepping();
    smart_stop_enabled_ = true;
    ROS_INFO("[RlGaitReceiver] Manually switched to stance mode");
  } else if (gait_name == "walk" || gait_name == "trot") {
    if (command_blocked)
    {
      pending_gait_name_ = gait_name;
      ROS_INFO_STREAM("[RlGaitReceiver] Delay walking gait until walking controller is active: " << gait_name);
      return;
    }
    currentCommand_.cmdStance_ = 0.0;
    if (gait_name == "trot")
    {
      trot_latched_ = true;
    }
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
  // 姿态模式下 linear.z 是高度命令，交给 AmpWalkController 的专用起身平滑处理，
  // 不参与这里面向行走速度的通用平滑和合成限幅。
  const bool bypass_linear_z_smoothing =
      currentCommand_.cmdStance_ == 1 && reuse_walk_command_in_stance_;
  
  // Calculate time delta
  double dt = (current_time - last_velocity_update_time_).toSec();
  if (dt <= 0.0) {
    return smoothed_cmd_vel_;
  }
  
  // Calculate velocity differences
  double vel_diff_x = cmd_vel.linear.x - smoothed_vel.linear.x;
  double vel_diff_y = cmd_vel.linear.y - smoothed_vel.linear.y;
  double vel_diff_z =
      bypass_linear_z_smoothing ? 0.0 : cmd_vel.linear.z - smoothed_vel.linear.z;
  double ang_diff_x = cmd_vel.angular.x - smoothed_vel.angular.x;
  double ang_diff_y = cmd_vel.angular.y - smoothed_vel.angular.y;
  double ang_diff_z = cmd_vel.angular.z - smoothed_vel.angular.z;
  
  // Apply smoothing factor
  double smooth_factor = std::min(velocity_smooth_factor_ / dt, 1.0);
  const double speed_for_tau =
      std::max(smoothed_vel.linear.x, cmd_vel.linear.x);
  const double active_tau =
      speed_for_tau > cmd_x_decel_ema_tau_threshold_
          ? cmd_x_decel_ema_tau_high_
          : cmd_x_decel_ema_tau_low_;
  const double cmd_x_decel_ema_alpha =
      active_tau > 0.0 ? 1.0 - std::exp(-dt / active_tau) : smooth_factor;

  const auto applyAxisSmooth = [&](double& smoothed, double target, double diff) {
    smoothed += diff * smooth_factor;
  };
  const bool in_neg_cmd_x =
      cmd_vel.linear.x < -1e-9 || smoothed_cmd_vel_.linear.x < -1e-9;
  const auto applyCmdXSmooth = [&](double& smoothed, double target, double diff) {
    if (!in_neg_cmd_x && cmd_x_smooth_enabled_) {
      // 正向加/减速均使用分段 EMA 平滑
      smoothed += diff * cmd_x_decel_ema_alpha;
    } else if (!in_neg_cmd_x) {
      smoothed = target;
    } else {
      smoothed += diff * smooth_factor;
    }
  };
  
  // Smooth linear velocities
  applyCmdXSmooth(smoothed_vel.linear.x, cmd_vel.linear.x, vel_diff_x);
  applyAxisSmooth(smoothed_vel.linear.y, cmd_vel.linear.y, vel_diff_y);
  applyAxisSmooth(smoothed_vel.linear.z, cmd_vel.linear.z, vel_diff_z);
  
  // Smooth angular velocities (x and y use normal smoothing)
  applyAxisSmooth(smoothed_vel.angular.x, cmd_vel.angular.x, ang_diff_x);
  applyAxisSmooth(smoothed_vel.angular.y, cmd_vel.angular.y, ang_diff_y);
  
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
    applyAxisSmooth(smoothed_vel.angular.z, cmd_vel.angular.z, ang_diff_z);
  }
  
  // Limit maximum linear velocity change
  const double applied_diff_x = smoothed_vel.linear.x - smoothed_cmd_vel_.linear.x;
  const double applied_diff_y = smoothed_vel.linear.y - smoothed_cmd_vel_.linear.y;
  const double applied_diff_z = smoothed_vel.linear.z - smoothed_cmd_vel_.linear.z;
  double linear_change = std::sqrt(
      applied_diff_x * applied_diff_x +
      applied_diff_y * applied_diff_y +
      applied_diff_z * applied_diff_z);
  const double smoothed_linear_mag = std::sqrt(
      smoothed_cmd_vel_.linear.x * smoothed_cmd_vel_.linear.x +
      smoothed_cmd_vel_.linear.y * smoothed_cmd_vel_.linear.y +
      smoothed_cmd_vel_.linear.z * smoothed_cmd_vel_.linear.z);
  const double cmd_linear_mag = std::sqrt(
      cmd_vel.linear.x * cmd_vel.linear.x +
      cmd_vel.linear.y * cmd_vel.linear.y +
      cmd_vel.linear.z * cmd_vel.linear.z);
  const bool linear_decel = cmd_linear_mag < smoothed_linear_mag - 1e-9;
  const bool forward_pos_cmd_x =
      !in_neg_cmd_x &&
      (std::abs(smoothed_cmd_vel_.linear.x) > 1e-9 || std::abs(cmd_vel.linear.x) > 1e-9);
  const bool use_forward_cmd_x_axis_limit =
      forward_pos_cmd_x && cmd_x_smooth_enabled_ &&
      (max_velocity_change_ > 0.0 || max_velocity_change_decel_cmd_x_ > 0.0);
  const double effective_max_velocity_change =
      (max_velocity_change_neg_cmd_x_ > 0.0 && in_neg_cmd_x)
          ? max_velocity_change_neg_cmd_x_
          : max_velocity_change_;
  const bool apply_linear_change_limit =
      !use_forward_cmd_x_axis_limit &&
      !(forward_pos_cmd_x && !cmd_x_smooth_enabled_) &&
      (linear_decel || in_neg_cmd_x);

  if (linear_change > effective_max_velocity_change && apply_linear_change_limit) {
    double scale = effective_max_velocity_change / linear_change;
    smoothed_vel.linear.x = smoothed_cmd_vel_.linear.x + scale * applied_diff_x;
    smoothed_vel.linear.y = smoothed_cmd_vel_.linear.y + scale * applied_diff_y;
    smoothed_vel.linear.z = smoothed_cmd_vel_.linear.z + scale * applied_diff_z;
  }

  // 正向 cmd_x 加/减速单独限速
  if (use_forward_cmd_x_axis_limit) {
    const double prev_x = smoothed_cmd_vel_.linear.x;
    const bool accelerating =
        std::abs(cmd_vel.linear.x) > std::abs(prev_x) + 1e-9;
    const bool decelerating =
        std::abs(cmd_vel.linear.x) < std::abs(prev_x) - 1e-9;
    double limit = -1.0;
    if (accelerating && max_velocity_change_ > 0.0) {
      limit = max_velocity_change_;
    } else if (decelerating) {
      limit = max_velocity_change_decel_cmd_x_ > 0.0
                  ? max_velocity_change_decel_cmd_x_
                  : max_velocity_change_;
    }
    if (limit > 0.0) {
      const double x_diff = smoothed_vel.linear.x - prev_x;
      const double x_change = std::abs(x_diff);
      if (x_change > limit) {
        smoothed_vel.linear.x =
            prev_x + (x_diff > 0.0 ? 1.0 : -1.0) * limit;
      }
    }
  }

  // 负向 cmd_x 单独限速（后退加减速均限速）
  if (max_velocity_change_neg_cmd_x_ > 0.0 && in_neg_cmd_x) {
    const double x_diff = smoothed_vel.linear.x - smoothed_cmd_vel_.linear.x;
    const double x_change = std::abs(x_diff);
    if (x_change > max_velocity_change_neg_cmd_x_) {
      smoothed_vel.linear.x =
          smoothed_cmd_vel_.linear.x + (x_diff > 0.0 ? 1.0 : -1.0) * max_velocity_change_neg_cmd_x_;
    }
  }

  // cmd_y 单步限速
  if (max_velocity_change_cmd_y_ > 0.0) {
    const double prev_y = smoothed_cmd_vel_.linear.y;
    const double y_diff = smoothed_vel.linear.y - prev_y;
    const double y_change = std::abs(y_diff);
    if (y_change > max_velocity_change_cmd_y_) {
      smoothed_vel.linear.y =
          prev_y + (y_diff > 0.0 ? 1.0 : -1.0) * max_velocity_change_cmd_y_;
    }
  }

  if (bypass_linear_z_smoothing) {
    smoothed_vel.linear.z = cmd_vel.linear.z;
  }
  
  // Apply mixed motion limits to velocity commands
  smoothed_vel = applyMixedMotionLimits(smoothed_vel);
  
  // Update previous command and time
  previous_cmd_vel_ = smoothed_vel;
  last_velocity_update_time_ = current_time;
  smoothed_cmd_vel_ = smoothed_vel;
  
  return smoothed_vel;
}

geometry_msgs::Twist RlGaitReceiver::smoothPoseCommand(const geometry_msgs::Twist& cmd_pose,
                                                       const ros::Time& current_time)
{
  geometry_msgs::Twist smoothed = smoothed_cmd_pose_;

  double dt = (current_time - last_pose_update_time_).toSec();
  if (dt <= 0.0)
  {
    return smoothed_cmd_pose_;
  }

  const double dz = cmd_pose.linear.z - smoothed.linear.z;
  const double dy = cmd_pose.angular.y - smoothed.angular.y;
  const double max_change = max_velocity_change_ * dt / velocity_smooth_time_;
  const double max_abs_delta = std::max(std::abs(dz), std::abs(dy));

  if (max_abs_delta > max_change && max_abs_delta > 1e-9)
  {
    const double scale = max_change / max_abs_delta;
    smoothed.linear.z += scale * dz;
    smoothed.angular.y += scale * dy;
  }
  else
  {
    smoothed.linear.z = cmd_pose.linear.z;
    smoothed.angular.y = cmd_pose.angular.y;
  }

  previous_cmd_pose_ = smoothed;
  last_pose_update_time_ = current_time;
  return smoothed;
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

  // Load velocity smoothing overrides (e.g. amp_hand_param.info for v17 amp_hand_controller)
  if (pt.find("velocitySmoothing") != pt.not_found()) {
    loadData::loadPtreeValue(pt, cmd_x_smooth_enabled_, "velocitySmoothing.cmdxSmooth", verbose);
    loadData::loadPtreeValue(pt, max_velocity_change_, "velocitySmoothing.maxVelocityChange", verbose);
    loadData::loadPtreeValue(pt, max_velocity_change_decel_cmd_x_,
                             "velocitySmoothing.maxVelocityChangeDecel", verbose);
    loadData::loadPtreeValue(pt, cmd_x_decel_ema_tau_threshold_,
                             "velocitySmoothing.cmdXDecelEmaTauThreshold", verbose);
    loadData::loadPtreeValue(pt, cmd_x_decel_ema_tau_high_,
                             "velocitySmoothing.cmdXDecelEmaTauHigh", verbose);
    loadData::loadPtreeValue(pt, cmd_x_decel_ema_tau_low_,
                             "velocitySmoothing.cmdXDecelEmaTauLow", verbose);
    loadData::loadPtreeValue(pt, cmd_x_decel_ema_tau_low_,
                             "velocitySmoothing.cmdXDecelEmaTau", verbose);
    loadData::loadPtreeValue(pt, max_velocity_change_neg_cmd_x_, "velocitySmoothing.maxVelocityChangeNegCmdX", verbose);
    loadData::loadPtreeValue(pt, max_velocity_change_cmd_y_, "velocitySmoothing.maxVelocityChangeCmdY", verbose);
    std::cout << "[RlGaitReceiver] velocitySmoothing loaded: cmdxSmooth="
              << (cmd_x_smooth_enabled_ ? "true" : "false")
              << ", maxVelocityChange="
              << max_velocity_change_ << ", maxVelocityChangeDecel="
              << max_velocity_change_decel_cmd_x_ << ", cmdXDecelEmaTau(threshold/high/low)="
              << cmd_x_decel_ema_tau_threshold_ << "/"
              << cmd_x_decel_ema_tau_high_ << "/"
              << cmd_x_decel_ema_tau_low_ << ", maxVelocityChangeNegCmdX="
              << max_velocity_change_neg_cmd_x_
              << ", maxVelocityChangeCmdY=" << max_velocity_change_cmd_y_ << std::endl;
  }

  // Load Y direction compensation parameters
  if (pt.find("yDirectionCompensation") != pt.not_found()) {
    loadData::loadPtreeValue(pt, y_direction_compensation_enabled_, "yDirectionCompensation.enabled", verbose);
    loadData::loadPtreeValue(pt, y_direction_compensation_x_bias_, "yDirectionCompensation.xBias", verbose);
    loadData::loadPtreeValue(pt, y_direction_compensation_z_bias_, "yDirectionCompensation.zBias", verbose);
    loadData::loadPtreeValue(pt, y_direction_compensation_threshold_, "yDirectionCompensation.threshold", verbose);
    loadData::loadPtreeValue(pt, y_direction_compensation_separate_enabled_,
                             "yDirectionCompensation.enableSeparateCompensation", verbose);
    loadData::loadPtreeValue(pt, y_direction_compensation_x_bias_left_, "yDirectionCompensation.xBiasLeft", verbose);
    loadData::loadPtreeValue(pt, y_direction_compensation_x_bias_right_, "yDirectionCompensation.xBiasRight", verbose);
    loadData::loadPtreeValue(pt, y_direction_compensation_z_bias_left_, "yDirectionCompensation.zBiasLeft", verbose);
    loadData::loadPtreeValue(pt, y_direction_compensation_z_bias_right_, "yDirectionCompensation.zBiasRight", verbose);

    ROS_INFO("[RlGaitReceiver] Y direction compensation loaded: enabled=%s, xBias=%.4f, zBias=%.4f, threshold=%.4f, separate=%s",
             y_direction_compensation_enabled_ ? "true" : "false",
             y_direction_compensation_x_bias_, y_direction_compensation_z_bias_,
             y_direction_compensation_threshold_,
             y_direction_compensation_separate_enabled_ ? "true" : "false");
    if (y_direction_compensation_enabled_ && y_direction_compensation_separate_enabled_) {
      ROS_INFO("[RlGaitReceiver] Y direction separate compensation: left(x=%.4f,z=%.4f), right(x=%.4f,z=%.4f)",
               y_direction_compensation_x_bias_left_, y_direction_compensation_z_bias_left_,
               y_direction_compensation_x_bias_right_, y_direction_compensation_z_bias_right_);
    }
  } else {
    ROS_WARN("[RlGaitReceiver] No yDirectionCompensation section found in config file, using default values (disabled)");
  }
}

void RlGaitReceiver::applyYDirectionCompensation()
{
  if (!y_direction_compensation_enabled_)
  {
    return;
  }

  const double linear_y = currentCommand_.cmdVelLineY_;
  if (std::abs(linear_y) <= y_direction_compensation_threshold_)
  {
    return;
  }

  double x_bias = y_direction_compensation_x_bias_;
  double z_bias = y_direction_compensation_z_bias_;
  if (y_direction_compensation_separate_enabled_)
  {
    // linear_y > 0 向左行走、linear_y < 0 向右行走（cmd_vel 左为正值）
    x_bias = (linear_y > 0) ? y_direction_compensation_x_bias_right_
                            : y_direction_compensation_x_bias_left_;
    z_bias = (linear_y > 0) ? y_direction_compensation_z_bias_right_
                            : y_direction_compensation_z_bias_left_;
  }
  currentCommand_.cmdVelLineX_ += x_bias;
  currentCommand_.cmdVelAngularZ_ += z_bias;
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
    ROS_INFO("[RlGaitReceiver] In-place stepping timed out after %.1f s, stopping",
             elapsed_time);
  }
}

bool RlGaitReceiver::isInPlaceSteppingActive() const
{
  return is_in_place_stepping_;
}

bool RlGaitReceiver::isInPlaceWalkingCommand(double linear_thresh, double angular_thresh) const
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  const double linear_norm = std::hypot(latest_cmd_vel_.linear.x, latest_cmd_vel_.linear.y);
  const bool has_no_velocity_input =
      linear_norm < linear_thresh &&
      std::abs(latest_cmd_vel_.linear.z) < linear_thresh &&
      std::abs(latest_cmd_vel_.angular.z) < angular_thresh;

  return currentCommand_.cmdStance_ < 0.5 &&
         trot_latched_ &&
         has_no_velocity_input;
}

} // namespace humanoid
} // namespace ocs2
