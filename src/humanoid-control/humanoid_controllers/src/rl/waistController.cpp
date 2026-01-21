#include "humanoid_controllers/rl/waistController.h"
#include <ros/ros.h>
#include <iostream>

namespace humanoid_controller
{

WaistController::WaistController(ros::NodeHandle& nh, size_t joint_waist_num,
                                 ocs2::humanoid::TopicLogger* ros_logger)
  : nh_(nh)
  , ros_logger_(ros_logger)
  , joint_waist_num_(joint_waist_num)
  , waist_control_mode_(1)
  , waist_control_enabled_(false)
  , waist_mode_interpolation_velocity_(1.0)
  , waist_is_interpolating_(false)
{
  // 初始化当前状态
  current_waist_pos_.resize(joint_waist_num_);
  current_waist_vel_.resize(joint_waist_num_);
  current_waist_pos_.setZero();
  current_waist_vel_.setZero();
  
  // 初始化期望状态
  desire_waist_q_.resize(joint_waist_num_);
  desire_waist_v_.resize(joint_waist_num_);
  desire_waist_q_.setZero();
  desire_waist_v_.setZero();
  
  // 模式0相关
  mode0_fixed_waist_pos_.resize(joint_waist_num_);
  mode0_fixed_waist_pos_.setZero();
  mode0_fixed_waist_pos_set_ = false;
  
  // 模式1相关
  default_waist_pos_.resize(joint_waist_num_);
  default_waist_pos_.setZero();
  
  // 模式2相关
  raw_mode2_waist_target_q_.resize(joint_waist_num_);
  raw_mode2_waist_target_q_.setZero();
  mode2_waist_target_q_.resize(joint_waist_num_);
  mode2_waist_target_q_.setZero();
  mode2_waist_target_received_ = false;
  
  // 控制参数
  waist_kp_.resize(joint_waist_num_);
  waist_kd_.resize(joint_waist_num_);
  
  // 平滑插值相关
  waist_interpolation_start_pos_.resize(joint_waist_num_);
  waist_interpolation_start_pos_.setZero();
  
  // 初始化订阅者（订阅/robot_waist_motion_data话题）
  if (joint_waist_num_ > 0)
  {
    waist_traj_sub_ = nh_.subscribe<kuavo_msgs::robotWaistControl>(
      "/robot_waist_motion_data", 10,
      boost::bind(&WaistController::waistTrajectoryCallback, this, _1));
  }
}

WaistController::~WaistController()
{
}

void WaistController::reset()
{
  // 重置插值状态
  waist_is_interpolating_ = false;
  
  // 重置期望状态为当前位置
  desire_waist_q_ = current_waist_pos_;
  desire_waist_v_ = current_waist_vel_;
  
  // 重置模式0的固定位置
  mode0_fixed_waist_pos_set_ = false;
  
  // 重置模式2的目标
  mode2_waist_target_received_ = false;
}

void WaistController::loadSettings(const Eigen::VectorXd& waist_kp,
                                   const Eigen::VectorXd& waist_kd,
                                   const Eigen::VectorXd& default_waist_pos,
                                   double waist_mode_interpolation_velocity)
{
  // PD控制参数
  if (waist_kp.size() != joint_waist_num_ || waist_kd.size() != joint_waist_num_)
  {
    ROS_WARN("[WaistController] Waist kp/kd dimension mismatch, using default values");
    waist_kp_.setConstant(80.0);  // 默认值，与MPC的waistAccelTask.kp一致
    waist_kd_.setConstant(10.0);  // 默认值，与MPC的waistAccelTask.kd一致
  }
  else
  {
    waist_kp_ = waist_kp;
    waist_kd_ = waist_kd;
  }
  
  // 默认位置
  if (default_waist_pos.size() == joint_waist_num_)
  {
    default_waist_pos_ = default_waist_pos;
  }
  else
  {
    default_waist_pos_.setZero();
  }
  
  // 模式切换时的插值速度（用于三次多项式插值）
  waist_mode_interpolation_velocity_ = waist_mode_interpolation_velocity;
}

void WaistController::update(const ros::Time& time,
                             double dt,
                             const Eigen::VectorXd& joint_pos,
                             const Eigen::VectorXd& joint_vel,
                             int cmd_stance,
                             kuavo_msgs::jointCmd& joint_cmd_msg,
                             size_t jointNumReal)
{
  if (!waist_control_enabled_ || joint_waist_num_ == 0)
  {
    ROS_WARN_THROTTLE(1.0, "[WaistController] update called but disabled: enabled=%d, joint_waist_num_=%zu", 
                      waist_control_enabled_, joint_waist_num_);
    return;
  }
  
  // 保存当前腰部位置和速度
  size_t waist_start_idx = jointNumReal;  // 腰部起始索引
  current_waist_pos_ = joint_pos.segment(waist_start_idx, joint_waist_num_);
  current_waist_vel_ = joint_vel.segment(waist_start_idx, joint_waist_num_);
  
  // 根据当前模式更新期望状态
  if (waist_control_mode_ == 0)
  {
    updateMode0(dt);
  }
  else if (waist_control_mode_ == 1)
  {
    // 模式1：RL控制，完全不处理，让RL控制器自己控制腰部
    waist_is_interpolating_ = false;
  }
  else if (waist_control_mode_ == 2)
  {
    updateMode2(dt);
  }
  
  // 填充命令消息（根据模式决定是否填充）
  // 模式0或2：总是填充；模式1：不填充，让RL控制器自己控制
  if (waist_control_mode_ == 0 || waist_control_mode_ == 2)
  {
    // 更新jointCmdMsg中的腰部部分
    // 注意：jointCmdMsg的顺序是：腿(jointNumReal) + 腰(waistNum_) + 手(armNumReal_) + 头
    size_t cmd_waist_start_idx = jointNumReal;  // 腰部在jointCmdMsg中的起始索引
    
    // 检查索引范围（检查tau和control_modes的大小，而不是joint_q）
    if (cmd_waist_start_idx + joint_waist_num_ > joint_cmd_msg.tau.size() ||
        cmd_waist_start_idx + joint_waist_num_ > joint_cmd_msg.control_modes.size())
    {
      ROS_WARN_THROTTLE(1.0, "[WaistController] Joint command message size mismatch: cmd_waist_start_idx=%zu, joint_waist_num_=%zu, tau.size()=%zu, control_modes.size()=%zu",
                        cmd_waist_start_idx, joint_waist_num_, joint_cmd_msg.tau.size(), joint_cmd_msg.control_modes.size());
      return;
    }
    
    // 计算PD控制力矩：tau = kp * (desire_q - current_q) + kd * (desire_v - current_v)
    Eigen::VectorXd pos_error = desire_waist_q_ - current_waist_pos_;
    Eigen::VectorXd vel_error = desire_waist_v_ - current_waist_vel_;
    Eigen::VectorXd tau_cmd = waist_kp_.cwiseProduct(pos_error) + waist_kd_.cwiseProduct(vel_error);
    
    // 只修改腰部关节的命令（从cmd_waist_start_idx开始，共joint_waist_num_个关节）
    for (size_t i = 0; i < joint_waist_num_; ++i)
    {
      size_t cmd_idx = cmd_waist_start_idx + i;
      
      // 确保索引在有效范围内
      if (cmd_idx >= joint_cmd_msg.tau.size() || cmd_idx >= joint_cmd_msg.control_modes.size())
      {
        ROS_WARN_THROTTLE(1.0, "[WaistController] Index out of range: cmd_idx=%zu, tau.size()=%zu, control_modes.size()=%zu",
                          cmd_idx, joint_cmd_msg.tau.size(), joint_cmd_msg.control_modes.size());
        continue;
      }
      
      // 更新力矩命令（PD控制计算出的力矩）
      joint_cmd_msg.tau[cmd_idx] = tau_cmd(i);
      
      // 更新control_modes为CST模式（力矩控制模式）
      joint_cmd_msg.control_modes[cmd_idx] = 0;  // 0 = CST (力矩控制)
    }
  }
}

bool WaistController::changeMode(int target_mode)
{
  // 验证模式有效性
  if (target_mode < 0 || target_mode > 2)
  {
    ROS_WARN("[WaistController] Invalid waist control mode: %d", target_mode);
    return false;
  }
  
  // 执行模式切换
  applyModeChange(target_mode);
  return true;
}

void WaistController::applyModeChange(int target_mode)
{
  if (target_mode == waist_control_mode_)
  {
    return;
  }
  
  int previous_mode = waist_control_mode_;
  waist_control_mode_ = target_mode;
  
  if (target_mode == 0)
  {
    // 模式0：固定到当前动作
    if (!mode0_fixed_waist_pos_set_)
    {
      mode0_fixed_waist_pos_ = current_waist_pos_;
      mode0_fixed_waist_pos_set_ = true;
    }
    if (previous_mode != 0)
    {
      resetInterpolationState(ros::Time::now(), current_waist_pos_, mode0_fixed_waist_pos_);
      waist_is_interpolating_ = true;
    }
  }
  else if (target_mode == 1)
  {
    // 模式1：RL控制
    waist_is_interpolating_ = false;  // 重置插值状态，让RL控制器完全接管
  }
  else if (target_mode == 2)
  {
    // 模式2：外部控制
    // 初始化期望位置为当前实际位置，确保插值从实际位置开始
    if (current_waist_pos_.size() == joint_waist_num_ && current_waist_pos_.norm() > 1e-6)
    {
      // 如果当前位置有效（非零），使用当前位置初始化
      desire_waist_q_ = current_waist_pos_;
      desire_waist_v_ = current_waist_vel_;
    }
    else
    {
      // 如果当前位置无效（零向量或未初始化），保持 desire_waist_q_ 不变
      // 它将在第一次 update 时从 current_waist_pos_ 初始化
    }
    waist_is_interpolating_ = true;
    mode2_waist_target_received_ = false;
  }
}

void WaistController::updateMode0(double dt)
{
  // 模式0：固定到当前位置
  if (waist_is_interpolating_)
  {
    // 如果正在插值（切换瞬间），执行平滑插值回到固定位置
    applySmoothInterpolation(ros::Time::now(), mode0_fixed_waist_pos_, Eigen::VectorXd::Zero(joint_waist_num_));
  }
  else if (mode0_fixed_waist_pos_set_)
  {
    desire_waist_q_ = mode0_fixed_waist_pos_;
    desire_waist_v_.setZero();
  }
}

void WaistController::updateMode2(double dt)
{
  // 模式2：外部控制模式
  // 使用三次多项式插值，插值结果自带速度
  
  // 1. 如果收到新目标且目标改变，重置插值状态
  if (mode2_waist_target_received_)
  {
    double target_diff = (mode2_waist_target_q_ - raw_mode2_waist_target_q_).norm();
    if (target_diff > 1e-6 || !waist_is_interpolating_)
    {
      // 确保 desire_waist_q_ 已初始化（如果还是零向量或未初始化，使用当前位置）
      if (desire_waist_q_.norm() < 1e-6 && current_waist_pos_.norm() > 1e-6)
      {
        desire_waist_q_ = current_waist_pos_;
        desire_waist_v_ = current_waist_vel_;
      }
      
      // 更新目标位置
      mode2_waist_target_q_ = raw_mode2_waist_target_q_;
      
      // 重置插值状态，开始新的插值（从当前的 desire_waist_q_ 插值到新的目标）
      resetInterpolationState(ros::Time::now(), desire_waist_q_, mode2_waist_target_q_);
      waist_is_interpolating_ = true;
    }
  }
  
  // 2. 执行三次多项式插值（自动计算速度和位置）
  if (waist_is_interpolating_ && mode2_waist_target_received_)
  {
    // 使用三次多项式插值，目标速度为0（插值结束后保持静止）
    applySmoothInterpolation(ros::Time::now(), mode2_waist_target_q_, 
                            Eigen::VectorXd::Zero(joint_waist_num_));
  }
  else if (!mode2_waist_target_received_)
  {
    // 如果还没有收到目标，保持当前位置和零速度
    if (desire_waist_q_.norm() < 1e-6 && current_waist_pos_.norm() > 1e-6)
    {
      desire_waist_q_ = current_waist_pos_;
    }
    desire_waist_v_.setZero();
  }
}

void WaistController::waistTrajectoryCallback(const kuavo_msgs::robotWaistControl::ConstPtr& msg)
{
  // 只在模式2（外部控制）时处理
  if (waist_control_mode_ != 2)
  {
    ROS_WARN_THROTTLE(1.0, "[WaistController] Waist trajectory received but control mode is %d (expected 2). Ignoring.", waist_control_mode_);
    return;
  }
  
  if (!waist_control_enabled_ || joint_waist_num_ == 0)
  {
    ROS_WARN_THROTTLE(1.0, "[WaistController] Waist trajectory received but waist control is disabled or no waist joints. enabled=%d, joint_waist_num_=%zu", 
             waist_control_enabled_, joint_waist_num_);
    return;
  }
  
  // 检查消息数据维度
  if (msg->data.data.size() != joint_waist_num_)
  {
    ROS_WARN("[WaistController] Waist trajectory dimension mismatch: expected=%zu, got=%zu",
             joint_waist_num_, msg->data.data.size());
    return;
  }
  
  // 腰部关节角度限制（与WaistKinematics和Python SDK保持一致）
  // waist_yaw_joint: [-180°, 180°] = [-π, π]
  static constexpr double WAIST_YAW_MIN_DEG = -180.0;
  static constexpr double WAIST_YAW_MAX_DEG = 180.0;
  
  // 提取目标位置（从度转换为弧度，添加角度限制）
  for (size_t i = 0; i < joint_waist_num_; ++i)
  {
    double angle_deg = msg->data.data[i];
    
    // 限制角度范围（与Python SDK保持一致：-180°到180°）
    if (angle_deg < WAIST_YAW_MIN_DEG || angle_deg > WAIST_YAW_MAX_DEG)
    {
      ROS_WARN("[WaistController] Waist joint %zu angle %.2f° exceeds limit [%.0f°, %.0f°], clamping",
               i, angle_deg, WAIST_YAW_MIN_DEG, WAIST_YAW_MAX_DEG);
      angle_deg = std::clamp(angle_deg, WAIST_YAW_MIN_DEG, WAIST_YAW_MAX_DEG);
    }
    
    raw_mode2_waist_target_q_(i) = angle_deg * M_PI / 180.0;  // 度转弧度
  }
  
  // 标记已收到模式2输入（期望速度和位置将在 updateMode2 的三次多项式插值中计算）
  mode2_waist_target_received_ = true;
}

void WaistController::applySmoothInterpolation(const ros::Time& current_time,
                                               const Eigen::VectorXd& target_pos,
                                               const Eigen::VectorXd& target_vel)
{
  // 使用三次多项式插值（与手臂控制一致）
  double elapsed = (current_time - waist_interpolation_start_time_).toSec();
  double duration = waist_interpolation_duration_;
  
  if (elapsed >= duration)
  {
    desire_waist_q_ = target_pos;
    desire_waist_v_ = target_vel;
    waist_is_interpolating_ = false;
    return;
  }
  
  // 归一化时间 [0, 1]
  double tau = elapsed / duration;
  
  // 三次多项式插值：s = 3τ² - 2τ³（与手臂控制一致）
  double tau2 = tau * tau;
  double s = tau2 * (3.0 - 2.0 * tau);
  
  // 解析计算速度指令：ds/dt = (ds/dτ) * (dτ/dt)
  // ds/dτ = 6τ - 6τ²
  double ds_dtau = 6.0 * tau * (1.0 - tau);
  double ds_dt = ds_dtau / duration;
  
  // 插值位置和速度
  desire_waist_q_ = waist_interpolation_start_pos_ + s * (target_pos - waist_interpolation_start_pos_);
  desire_waist_v_ = ds_dt * (target_pos - waist_interpolation_start_pos_);
}

void WaistController::resetInterpolationState(const ros::Time& time,
                                             const Eigen::VectorXd& start_pos,
                                             const Eigen::VectorXd& target_pos)
{
  waist_interpolation_start_time_ = time;
  waist_interpolation_start_pos_ = start_pos;
  
  // 根据最大关节位移和插值速度动态计算时长
  double max_displacement = (target_pos - start_pos).cwiseAbs().maxCoeff();
  
  // 如果位移非常小（接近0），直接设置目标值，不需要插值
  if (max_displacement < 1e-6)
  {
    waist_interpolation_duration_ = 0.0;  // 设置为0，applySmoothInterpolation会立即设置目标值
    return;
  }
  
  waist_interpolation_duration_ = max_displacement / waist_mode_interpolation_velocity_;
  
  // 限制最小和最大插值时间
  const double min_duration = 0.1;  // 最小插值时间 100ms
  const double max_duration = 2.0;  // 最大插值时间 2s
  waist_interpolation_duration_ = std::clamp(waist_interpolation_duration_, min_duration, max_duration);
}

} // namespace humanoid_controller
