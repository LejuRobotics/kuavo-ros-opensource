// Pinocchio must be included before Boost headers
#include <pinocchio/fwd.hpp>

#include "humanoid_controllers/rl/armController.h"
#include "humanoid_controllers/armTorqueController.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <stdexcept>

namespace humanoid_controller
{

ArmController::ArmController(ros::NodeHandle& nh, size_t joint_num, size_t joint_waist_num, size_t joint_arm_num,
                              ocs2::humanoid::TopicLogger* ros_logger)
  : nh_(nh)
  , ros_logger_(ros_logger)
  , joint_num_(joint_num)
  , joint_waist_num_(joint_waist_num)
  , joint_arm_num_(joint_arm_num)
  , arm_start_idx_(joint_num + joint_waist_num)
  , arm_control_mode_(1)
  , arm_vr_enabled_(false)
  , arm_max_tracking_velocity_(1.0)
  , arm_tracking_error_threshold_(0.1)
  , mode_interpolation_velocity_(1.0)
  , pending_arm_mode_(-1)
  , has_pending_mode_change_(false)
  , is_interpolating_(false)
  , last_cmd_stance_(-1)
  , mode2_cutoff_freq_(10.0) // 默认10Hz
{
  // 初始化当前状态
  current_arm_pos_.resize(joint_arm_num);
  current_arm_vel_.resize(joint_arm_num);
  current_arm_pos_.setZero();
  current_arm_vel_.setZero();
  
  // 初始化指令状态
  cmd_arm_pos_.resize(joint_arm_num);
  cmd_arm_vel_.resize(joint_arm_num);
  cmd_arm_pos_.setZero();
  cmd_arm_vel_.setZero();
  
  // 初始化期望状态
  desire_arm_q_.resize(joint_arm_num);
  desire_arm_v_.resize(joint_arm_num);
  desire_arm_q_.setZero();
  desire_arm_v_.setZero();
  
  // 初始化模式0相关
  mode0_fixed_pos_.resize(joint_arm_num);
  mode0_fixed_pos_.setZero();
  mode0_fixed_pos_set_ = false;
  
  // 初始化模式1相关
  default_arm_pos_.resize(joint_arm_num);
  default_arm_pos_.setZero();
  is_interpolating_to_default_ = false;
  
  // 初始化模式2相关
  raw_mode2_target_q_.resize(joint_arm_num);
  raw_mode2_target_v_.resize(joint_arm_num);
  raw_mode2_target_q_.setZero();
  raw_mode2_target_v_.setZero();
  mode2_target_q_.resize(joint_arm_num);
  mode2_target_v_.resize(joint_arm_num);
  mode2_target_q_.setZero();
  mode2_target_v_.setZero();
  mode2_target_received_ = false;
  last_mode2_input_time_valid_ = false;
  
  // 初始化控制参数向量
  joint_kp_.resize(joint_arm_num);
  joint_kd_.resize(joint_arm_num);
  torque_limits_.resize(joint_arm_num);
  
  ROS_INFO("[ArmController] Initialized: leg_joints=%zu, waist_joints=%zu, arm_joints=%zu, arm_start_idx=%zu", 
           joint_num_, joint_waist_num_, joint_arm_num_, arm_start_idx_);
}

ArmController::~ArmController()
{
}

void ArmController::reset()
{
  // 重置插值状态
  arm_control_mode_ = 1;
  arm_vr_enabled_ = false;
  is_interpolating_ = false;
  is_interpolating_to_default_ = false;
  
  // 重置缓存的cmd_arm_pos_，使用当前位置
  cmd_arm_pos_ = current_arm_pos_;
  cmd_arm_vel_ = current_arm_vel_;
  
  // 重置期望状态为当前位置
  desire_arm_q_ = current_arm_pos_;
  desire_arm_v_ = current_arm_vel_;
  
  // 重置模式0的固定位置
  mode0_fixed_pos_set_ = false;
  
  // 重置模式2的目标
  mode2_target_received_ = false;
  last_mode2_input_time_valid_ = false;
  
  // 重置滤波器
  if (arm_filter_initialized_)
  {
    arm_joint_pos_filter_.reset(current_arm_pos_);
    arm_joint_vel_filter_.reset(current_arm_vel_);
  }
  
  // 重置缓存的模式切换指令
  pending_arm_mode_ = -1;
  has_pending_mode_change_ = false;
  
  ROS_INFO("[ArmController] Reset: cleared interpolation states and cached command positions");
}

bool ArmController::initialize(const std::string& urdf_path,
                                const Eigen::VectorXd& joint_kp,
                                const Eigen::VectorXd& joint_kd)
{
  // Initialize torque controller if URDF path and kp/kd parameters are provided
  if (!urdf_path.empty() && joint_kp.size() > 0 && joint_kd.size() > 0)
  {
    // Validate parameter dimensions
    if (joint_kp.size() != joint_arm_num_ || joint_kd.size() != joint_arm_num_)
    {
      ROS_ERROR("[ArmController] Parameter dimension mismatch: kp.size()=%zu, kd.size()=%zu, expected=%zu",
                joint_kp.size(), joint_kd.size(), joint_arm_num_);
      return false;
    }
    
    try
    {
      // Store kp/kd for later use
      joint_kp_ = joint_kp;
      joint_kd_ = joint_kd;
      
      // Initialize torque controller
      arm_torque_controller_ = std::make_unique<ArmTorqueController>(
        urdf_path, joint_kp_, joint_kd_);
      ROS_INFO("[ArmController] Torque controller initialized successfully");
    }
    catch (const std::exception& e)
    {
      ROS_ERROR("[ArmController] Failed to initialize torque controller: %s", e.what());
      return false;
    }
  }
  else if (!urdf_path.empty())
  {
    ROS_DEBUG("[ArmController] URDF path provided but kp/kd not provided, torque controller not initialized");
  }
  
  // Subscribe to VR input topic
  joint_sub_ = nh_.subscribe<sensor_msgs::JointState>(
    "/kuavo_arm_traj", 3, 
    boost::bind(&ArmController::jointStateCallback, this, _1));
  
  // 滤波器参数将在首次 update() 调用时根据实际 dt 初始化
  // 这里只标记需要初始化，不预设 dt 值
  arm_filter_initialized_ = false;
  
  ROS_INFO("[ArmController] Initialization successful");
  return true;
}

void ArmController::loadSettings(double max_tracking_velocity, 
                                 double tracking_error_threshold,
                                 double mode_interpolation_velocity,
                                 const Eigen::VectorXd& default_arm_pos,
                                 double mode2_cutoff_freq)
{
  arm_max_tracking_velocity_ = max_tracking_velocity;
  arm_tracking_error_threshold_ = tracking_error_threshold;
  mode_interpolation_velocity_ = mode_interpolation_velocity;
  mode2_cutoff_freq_ = mode2_cutoff_freq;

  // 滤波器参数将在首次 update() 调用时根据实际 dt 初始化
  // 这里只更新截止频率，实际 dt 会在 update() 中设置
  // 如果已经初始化过，需要重新设置以应用新的截止频率
  if (arm_filter_initialized_ && joint_arm_num_ > 0)
  {
    arm_filter_initialized_ = false;
    ROS_INFO("[ArmController] Filter cutoff frequency updated to %.1f Hz, will re-initialize in next update()", mode2_cutoff_freq_);
  }
  
  if (default_arm_pos.size() == joint_arm_num_)
  {
    default_arm_pos_ = default_arm_pos;
    std::cout << "[ArmController] Default arm position: " << default_arm_pos.transpose() << std::endl;
  }
  else if (default_arm_pos.size() > 0)
  {
    ROS_WARN("[ArmController] Default arm position dimension mismatch: expected=%zu, got=%zu",
             joint_arm_num_, default_arm_pos.size());
  }
  
  ROS_INFO("[ArmController] Loaded settings: mode1_max_velocity=%.3f rad/s, error_threshold=%.3f rad, mode_interpolation_velocity=%.3f rad/s",
           arm_max_tracking_velocity_, arm_tracking_error_threshold_, mode_interpolation_velocity_);
}

void ArmController::update(const ros::Time& time,
                           double dt,
                           const Eigen::VectorXd& joint_pos,
                           const Eigen::VectorXd& joint_vel,
                           int cmd_stance,
                           kuavo_msgs::jointCmd& joint_cmd_msg)
{
  // 保存当前手臂位置和速度（供模式切换使用）
  current_arm_pos_ = joint_pos.segment(arm_start_idx_, joint_arm_num_);
  current_arm_vel_ = joint_vel.segment(arm_start_idx_, joint_arm_num_);

  // 1. 处理自动模式切换
  // handleAutoModeSwitch(cmd_stance);
  // 确保滤波器参数与实际控制频率匹配（仅在未初始化时使用传入的 dt 初始化）
  if (joint_arm_num_ > 0 && !arm_filter_initialized_)
  {
    Eigen::VectorXd cutoff = Eigen::VectorXd::Constant(joint_arm_num_, mode2_cutoff_freq_);
    arm_joint_pos_filter_.setParams(dt, cutoff);
    arm_joint_vel_filter_.setParams(dt, cutoff);
    arm_filter_initialized_ = true;
    ROS_INFO("[ArmController] Filter initialized: dt=%.4f s, cutoff=%.1f Hz", dt, mode2_cutoff_freq_);
  }

  // 2. 根据当前模式更新期望状态
  if (arm_control_mode_ == 0)
  {
    updateMode0(dt);
  }
  else if (arm_control_mode_ == 1)
  {
    updateMode1(time, dt, cmd_stance);
  }
  else if (arm_control_mode_ == 2)
  {
    updateMode2(dt);
  }

  // 3. 填充命令消息（根据模式决定是否填充）
  // 注意：不再对全模式应用滤波，Mode 0 和 1 的平滑插值不需要滤波
  if (arm_control_mode_ == 0 || arm_control_mode_ == 2 || 
      (arm_control_mode_ == 1 && cmd_stance == 1 && is_interpolating_to_default_))
  {
    fillJointCmdMessage(joint_cmd_msg, desire_arm_q_, desire_arm_v_,
                       joint_pos, joint_vel);
  }
  
  // 5. 发布模式状态（如果ros_logger可用）
  if (ros_logger_)
  {
    // 发布当前模式
    ros_logger_->publishValue("/arm_controller/mode", static_cast<double>(arm_control_mode_));
    

  }
    
  // 从joint_cmd_msg中提取指令位置和速度（用于保持指令一致性）
  for (size_t i = 0; i < joint_arm_num_; ++i)
  {
    size_t idx = arm_start_idx_ + i;
    if (idx < joint_cmd_msg.joint_q.size())
    {
      cmd_arm_pos_(i) = joint_cmd_msg.joint_q[idx];
    }
    else
    {
      cmd_arm_pos_(i) = current_arm_pos_(i);
    }
    if (idx < joint_cmd_msg.joint_v.size())
    {
      cmd_arm_vel_(i) = joint_cmd_msg.joint_v[idx];
    }
    else
    {
      cmd_arm_vel_(i) = current_arm_vel_(i);
    }
  }
  
}

void ArmController::fillJointCmdMessage(kuavo_msgs::jointCmd& joint_cmd_msg,
                                       const Eigen::VectorXd& target_q,
                                       const Eigen::VectorXd& target_v,
                                       const Eigen::VectorXd& joint_pos,
                                       const Eigen::VectorXd& joint_vel)
{
  // 提取当前手臂位置和速度
  Eigen::VectorXd current_arm_pos = joint_pos.segment(arm_start_idx_, joint_arm_num_);
  Eigen::VectorXd current_arm_vel = joint_vel.segment(arm_start_idx_, joint_arm_num_);
  // 计算力矩
  Eigen::VectorXd arm_tau_desired = Eigen::VectorXd::Zero(joint_arm_num_);
  if (arm_torque_controller_)
  {
    arm_torque_controller_->setMeasuredState(current_arm_pos, current_arm_vel);
    arm_tau_desired = arm_torque_controller_->computeTorque(
      target_q, target_v, Eigen::VectorXd::Zero(joint_arm_num_));
  }
  
  // 只更新joint_q、joint_v和tau，保留其他原有值
  for(size_t i = arm_start_idx_; i < arm_start_idx_ + joint_arm_num_; ++i)
  {
    size_t arm_idx = i - arm_start_idx_;
    joint_cmd_msg.joint_q[i] = target_q(arm_idx);
    joint_cmd_msg.joint_v[i] = target_v(arm_idx);
    // 如果计算了力矩，则更新tau；否则保留原有值
    if (arm_torque_controller_)
    {
      joint_cmd_msg.tau[i] = arm_tau_desired(arm_idx);
    }
    // joint_kp, joint_kd, torque_limits, control_modes 等保留原有值
  }
}

bool ArmController::changeMode(int target_mode)
{
  // 检查是否处于插值过程中
  // if (is_interpolating_ && target_mode != arm_control_mode_)
  // {
  //   // 缓存指令，待插值完成后执行
  //   pending_arm_mode_ = target_mode;
  //   has_pending_mode_change_ = true;
  //   ROS_INFO("[ArmController] Interpolation in progress, mode change command cached (current: %d -> target: %d)",
  //            arm_control_mode_, target_mode);
  //   return false;  // 返回false表示指令已缓存，未立即执行
  // }
  
  // 验证模式有效性
  if (target_mode < 0 || target_mode > 2)
  {
    ROS_WARN("[ArmController] Invalid control mode: %d", target_mode);
    return false;
  }
  
  // 执行模式切换
  applyModeChange(target_mode);
  return true;
}

void ArmController::resetInterpolationState(const ros::Time& time, const Eigen::VectorXd& start_pos, const Eigen::VectorXd& target_pos)
{
  interpolation_start_time_ = time;
  interpolation_start_pos_ = start_pos;
  
  // 根据最大关节位移和插值速度 (mode_interpolation_velocity_) 动态计算时长
  // 对于三次多项式 s = 3τ² - 2τ³，其最大导数（速度系数）为 1.5 (在 τ=0.5 时)
  // 峰值速度 V_max = 1.5 * (max_dist / T) => T = 1.5 * max_dist / V_limit
  if (start_pos.size() > 0 && start_pos.size() == target_pos.size()) {
    double max_dist = (target_pos - start_pos).lpNorm<Eigen::Infinity>();
    
    // 使用配置的插值速度计算时长，保证峰值速度不超过限制
    interpolation_duration_ = 1.5 * max_dist / mode_interpolation_velocity_;
    
    // 限制时长范围，避免极小或极大值
    interpolation_duration_ = std::max(0.2, std::min(2.0, interpolation_duration_));
  } else {
    interpolation_duration_ = 0.5;
  }
}

void ArmController::applySmoothInterpolation(const ros::Time& current_time,
                                             const Eigen::VectorXd& target_pos,
                                             const Eigen::VectorXd& target_vel)
{
  double t = (current_time - interpolation_start_time_).toSec();
  double invT = 1.0 / interpolation_duration_;
  double tau = t * invT;
  
  if (tau >= 1.0) {
    desire_arm_q_ = target_pos;
    desire_arm_v_ = target_vel;
    is_interpolating_ = false;
    if (has_pending_mode_change_) executePendingModeChange();
    return;
  }
  
  // 三次多项式插值：s = 3τ² - 2τ³
  double tau2 = tau * tau;
  double s = tau2 * (3.0 - 2.0 * tau);
  
  // 解析计算速度指令：ds/dt = (ds/dτ) * (dτ/dt)
  // ds/dτ = 6τ - 6τ²
  double ds_dtau = 6.0 * tau * (1.0 - tau);
  double ds_dt = ds_dtau * invT;
  
  desire_arm_q_ = interpolation_start_pos_ + s * (target_pos - interpolation_start_pos_);
  desire_arm_v_ = ds_dt * (target_pos - interpolation_start_pos_);
  
  is_interpolating_ = true;
}

bool ArmController::interpolateToTarget(const ros::Time& time,
                                       double dt,
                                       const Eigen::VectorXd& joint_pos,
                                       const Eigen::VectorXd& joint_vel,
                                       const Eigen::VectorXd& target_pos,
                                       kuavo_msgs::jointCmd& joint_cmd_msg,
                                       double interpolation_duration)
{
  // 保存当前手臂位置和速度
  current_arm_pos_ = joint_pos.segment(arm_start_idx_, joint_arm_num_);
  current_arm_vel_ = joint_vel.segment(arm_start_idx_, joint_arm_num_);

  // 如果还没开始插值，初始化插值状态
  if (!is_interpolating_)
  {
    // 复用 applySmoothInterpolation 的状态变量
    interpolation_start_time_ = time;
    interpolation_start_pos_ = current_arm_pos_;
    target_interpolation_target_pos_ = target_pos;
    interpolation_duration_ = interpolation_duration;
    is_interpolating_ = true;

    ROS_INFO("[ArmController] Starting smooth interpolation to target position (duration: %.2fs)", interpolation_duration);
  }

  // 复用 applySmoothInterpolation 的插值计算逻辑
  applySmoothInterpolation(time, target_interpolation_target_pos_, Eigen::VectorXd::Zero(joint_arm_num_));

  // 检查插值是否完成
  if (!is_interpolating_)
  {
    ROS_INFO("[ArmController] Smooth interpolation completed, switching to RL control");
    // 插值完成后 desire_arm_q_ 和 desire_arm_v_ 已经被设置为最终值
    fillJointCmdMessage(joint_cmd_msg, desire_arm_q_, desire_arm_v_, joint_pos, joint_vel);
    return false;  // 插值完成，返回false让RL接管
  }

  // 插值进行中，使用计算出的期望位置和速度
  fillJointCmdMessage(joint_cmd_msg, desire_arm_q_, desire_arm_v_, joint_pos, joint_vel);

  return true;  // 正在插值，使用外部控制
}

bool ArmController::changeModeCallback(kuavo_msgs::changeArmCtrlMode::Request &req,
                                      kuavo_msgs::changeArmCtrlMode::Response &res)
{
  bool success = changeMode(req.control_mode);
  
  res.result = success;
  res.mode = arm_control_mode_;
  
  return true;
}

bool ArmController::getModeCallback(kuavo_msgs::changeArmCtrlMode::Request &req,
                                   kuavo_msgs::changeArmCtrlMode::Response &res)
{
  res.result = true;
  res.mode = arm_control_mode_;
  return true;
}

void ArmController::applyModeChange(int target_mode)
{
  if (target_mode == 0)
  {
    // 使用当前手臂位置作为插值起点，插值到固定位置
    desire_arm_q_ = current_arm_pos_;
    desire_arm_v_ = current_arm_vel_;
    
    // 使用指令位置作为固定位置（保持指令一致性）
    mode0_fixed_pos_ = cmd_arm_pos_;
    
    // 初始化平滑插值状态
    resetInterpolationState(ros::Time::now(), current_arm_pos_, mode0_fixed_pos_);
    mode0_fixed_pos_set_ = true;
    
    is_interpolating_ = true;
    
    ROS_INFO("[ArmController] Switching to Mode 0: will smoothly transition to fixed position");
    // Mode 0: 固定到当前动作
    arm_control_mode_ = 0;
    arm_vr_enabled_ = false;
  }
  else if (target_mode == 1)
  {
   
    
    // 使用当前手臂位置作为插值起点，插值到默认位置
    desire_arm_q_ = current_arm_pos_;
    desire_arm_v_ = current_arm_vel_;
    
    // 初始化平滑插值状态
    resetInterpolationState(ros::Time::now(), current_arm_pos_, default_arm_pos_);
    
    std::cout << "[ArmController] applyModeChange: current_arm_pos_ " << current_arm_pos_.transpose() << std::endl;
    std::cout << "[ArmController] applyModeChange: default_arm_pos_ " << default_arm_pos_.transpose() << std::endl;
    
    // 检查当前位置和默认位置的差异
    Eigen::VectorXd error = default_arm_pos_ - current_arm_pos_;
    double error_norm = error.norm();
    
    if (error_norm > arm_tracking_error_threshold_)
    {
      // 差异较大，标记为插值状态，让模式1平滑过渡到默认位置
      is_interpolating_ = true;
      is_interpolating_to_default_ = true;  // 标记正在插值到默认位置
      ROS_INFO("[ArmController] Switching to Mode 1: will smoothly transition (Minimum Jerk) to default position (error=%.4f rad, duration=%.2f s)", 
               error_norm, interpolation_duration_);
    }
    else
    {
      // 差异较小，直接设置为默认位置
      desire_arm_q_ = default_arm_pos_;
      desire_arm_v_ = Eigen::VectorXd::Zero(joint_arm_num_);
      is_interpolating_ = false;
      is_interpolating_to_default_ = false;
      ROS_INFO("[ArmController] Switching to Mode 1: already at default position");
    }
    
    ROS_INFO("[ArmController] Switching to Mode 1: auto swing arm");
     // Mode 1: 自动摆手
     arm_control_mode_ = 1;
     arm_vr_enabled_ = false;
  }
  else if (target_mode == 2)
  {
    // 1. 设置插值起点：期望位置同步为当前实际位置
    desire_arm_q_ = current_arm_pos_;
    desire_arm_v_ = current_arm_vel_;
    
    // 2. 初始化目标：在收到外部信号前，目标保持在当前位置，防止误触发结束
    mode2_target_q_ = current_arm_pos_;
    mode2_target_v_ = current_arm_vel_;
    
    // 重置滤波器状态到当前位置
    if (arm_filter_initialized_)
    {
      arm_joint_pos_filter_.reset(current_arm_pos_);
      arm_joint_vel_filter_.reset(current_arm_vel_);
    }

    // 尚未收到外部目标
    mode2_target_received_ = false;
    
    // 3. 标记开始插值阶段
    is_interpolating_ = true;
    
    ROS_INFO("[ArmController] Switching to Mode 2: Waiting for external target to start interpolation (v_interp=%.2f)", 
             mode_interpolation_velocity_);
    
    arm_control_mode_ = 2;
    arm_vr_enabled_ = true;
  }
}

void ArmController::executePendingModeChange()
{
  if (!has_pending_mode_change_ || pending_arm_mode_ == -1)
  {
    return;
  }
  
  int target_mode = pending_arm_mode_;
  has_pending_mode_change_ = false;
  pending_arm_mode_ = -1;
  
  ROS_INFO("[ArmController] Executing cached mode change command: %d", target_mode);
  applyModeChange(target_mode);
}

void ArmController::applyRateLimitedInterpolation(double dt,
                                                 const Eigen::VectorXd& target_pos,
                                                 const Eigen::VectorXd& target_vel,
                                                 double max_velocity)
{
  // 计算当前位置与目标位置的误差
  Eigen::VectorXd error = target_pos - desire_arm_q_;
  double error_norm = error.norm();

  // 检查是否已完成插值（误差小于阈值）
  if (error_norm < arm_tracking_error_threshold_)
  {
    // 误差小于阈值，完全接管目标位置
    desire_arm_q_ = target_pos;
    desire_arm_v_ = target_vel;
    is_interpolating_ = false;
    
    // 检查是否有缓存的模式切换指令
    if (has_pending_mode_change_)
    {
      executePendingModeChange();
    }
  }
  else
  {
    // 误差大于阈值，使用限速插值
    is_interpolating_ = true;
    
    // 计算限速插值
    Eigen::VectorXd max_delta = error.cwiseAbs();
    for(int i = 0; i < joint_arm_num_; ++i)
    {
      double max_delta_pos = max_velocity * dt;
      if (max_delta(i) > max_delta_pos)
      {
        error(i) = error(i) > 0 ? max_delta_pos : -max_delta_pos;
      }
    }
    desire_arm_q_ += error;
    desire_arm_v_ = target_vel;  // 速度直接使用目标速度
  }
}

void ArmController::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  // 只在模式2时更新目标位置
  if (arm_control_mode_ == 2 && arm_vr_enabled_)
  {
    for (size_t i = 0; i < msg->name.size() && i < joint_arm_num_; ++i)
    {
      raw_mode2_target_q_(i) = msg->position[i] * M_PI / 180.0;
      if (msg->velocity.size() == joint_arm_num_)
      {
        raw_mode2_target_v_(i) = msg->velocity[i] * M_PI / 180.0;
      }
      else
      {
        // 如果没有提供速度，设置为零（后续在updateMode2中通过位置差分计算）
        raw_mode2_target_v_(i) = 0.0;
      }
    }
    
    // 标记已收到模式2输入
    mode2_target_received_ = true;
  }
}

void ArmController::updateMode0(double dt)
{
  // 模式0：固定到当前动作
  if (is_interpolating_)
  {
    // 如果正在插值（切换瞬间），执行平滑插值回到固定位置
    applySmoothInterpolation(ros::Time::now(), mode0_fixed_pos_, Eigen::VectorXd::Zero(joint_arm_num_));
  }
  else if (mode0_fixed_pos_set_)
  {
    desire_arm_q_ = mode0_fixed_pos_;
    desire_arm_v_.setZero();
  }
}

void ArmController::updateMode1(const ros::Time& time, double dt, int cmd_stance)
{
  // 模式1：自动摆手
  // 如果cmd_stance是1（站立），从指令位置通过限速插值回到默认位置，并填充fillJointCmdMessage
  // 如果cmd_stance是0（行走），不替换RL指令，保持joint_cmd_msg中的值（期望状态不更新）
  
  if (cmd_stance == 1)  // 站立
  {
    if (!is_interpolating_to_default_)
    {
      desire_arm_q_ = current_arm_pos_;
      desire_arm_v_ = current_arm_vel_;
      resetInterpolationState(time, current_arm_pos_, default_arm_pos_);
      is_interpolating_to_default_ = true;
      is_interpolating_ = true;
    }
    applySmoothInterpolation(time, default_arm_pos_, Eigen::VectorXd::Zero(joint_arm_num_));
  }
  else
  {
    is_interpolating_to_default_ = false;
    is_interpolating_ = false;
  }
}

void ArmController::updateMode2(double dt)
{
  // 模式2：外部控制模式
  // 使用和humanoidController相同的滤波逻辑：先滤波位置，然后通过位置差分计算速度，再滤波速度
  
  // 静态变量：用于速度计算（需要在函数开头声明，以便在整个函数中访问）
  static Eigen::VectorXd prev_filtered_pos;
  static bool first_call = true;
  
  // 初始化静态变量
  if (first_call && mode2_target_received_)
  {
    prev_filtered_pos = mode2_target_q_;
    first_call = false;
  }
  
  // 1. 始终更新滤波后的目标（供接管阶段使用）
  if (mode2_target_received_)
  {
    // 1.1 先对位置进行滤波（和humanoidController一致）
    mode2_target_q_ = arm_joint_pos_filter_.update(raw_mode2_target_q_);
    
    // 1.2 如果原始轨迹没有提供速度，使用滤波后的位置计算速度（和humanoidController一致）
    // 检查原始轨迹是否提供了速度（通过检查raw_mode2_target_v_是否为零向量）
    bool has_velocity = raw_mode2_target_v_.norm() > 1e-6;
    
    Eigen::VectorXd computed_vel;
    if (has_velocity)
    {
      // 如果提供了速度，先对原始速度进行滤波
      computed_vel = arm_joint_vel_filter_.update(raw_mode2_target_v_);
    }
    else
    {
      // 如果没有提供速度，通过位置差分计算速度（和humanoidController一致）
      // 确保prev_filtered_pos已初始化
      if (prev_filtered_pos.size() != joint_arm_num_)
      {
        prev_filtered_pos = mode2_target_q_;
      }
      computed_vel = (mode2_target_q_ - prev_filtered_pos) / dt;
      // 对计算出的速度再次滤波
      computed_vel = arm_joint_vel_filter_.update(computed_vel);
    }
    
    mode2_target_v_ = computed_vel;
    prev_filtered_pos = mode2_target_q_;
  }

  // 2. 核心插值/接管逻辑重构
  if (is_interpolating_)
  {
    // 插值阶段：
    // 如果还没收到第一个目标，我们保持不动，并维持 is_interpolating = true
    if (!mode2_target_received_)
    {
      desire_arm_v_.setZero();
      // 不调用 applyRateLimitedInterpolation，防止其因为误差为0而误判结束
    }
    else
    {
      // 收到目标后，向"原始目标"(raw_mode2_target_q_) 靠拢
      // 关键：不使用滤波后的目标进行插值，以确保产生足够的误差触发限速
      applyRateLimitedInterpolation(dt, raw_mode2_target_q_, Eigen::VectorXd::Zero(joint_arm_num_), mode_interpolation_velocity_);
      
      // 当误差减小到阈值，is_interpolating 会被 applyRateLimitedInterpolation 设为 false
      if (!is_interpolating_) {
          // 插值结束瞬间，重置滤波器到当前位置，确保接管瞬间无跳变
          arm_joint_pos_filter_.reset(desire_arm_q_);
          arm_joint_vel_filter_.reset(desire_arm_v_);
          // 重置静态变量，避免使用旧的prev_filtered_pos
          prev_filtered_pos = desire_arm_q_;
          first_call = false;  // 保持false，不需要重新初始化
          ROS_INFO("[ArmController] Interpolation finished, switching to filtered tracking (v_max=%.2f)", arm_max_tracking_velocity_);
      }
    }
  }
  else
  {
    // 接管阶段：正常跟踪滤波后的目标，使用最大跟踪速度
    applyRateLimitedInterpolation(dt, mode2_target_q_, mode2_target_v_, arm_max_tracking_velocity_);
  }
}

void ArmController::handleAutoModeSwitch(int cmd_stance)
{
  // 自动切换手臂控制模式：行走时模式1（RL），站定时模式0（固定到当前动作）
  
  if (cmd_stance == 0 && last_cmd_stance_ != 0 && arm_control_mode_ != 1)
  {
    // 从站立切换到行走，切换到模式1（RL控制）
    if (!is_interpolating_)
    {
      applyModeChange(1);
      ROS_INFO("[ArmController] Detected walking state, switching to RL control (Mode 1)");
    }
  }
  
  if (cmd_stance == 1 && last_cmd_stance_ != 1 && arm_control_mode_ != 0)
  {
    // 从行走切换到站立，切换到模式0（固定到当前动作）
    if (!is_interpolating_)
    {
      applyModeChange(0);
      ROS_INFO("[ArmController] Detected standing state, switching to Mode 0 (fixed to current)");
    }
  }

  last_cmd_stance_ = cmd_stance;
}

} // namespace humanoid_controller
