#include "humanoid_controllers/rl/waistController.h"
#include <ros/ros.h>
#include <iostream>

namespace humanoid_controller
{

WaistController::WaistController(ros::NodeHandle& nh, size_t joint_waist_num,
                                 ocs2::humanoid::TopicLogger* ros_logger,
                                 bool is_real)
  : nh_(nh)
  , ros_logger_(ros_logger)
  , joint_waist_num_(joint_waist_num)
  , is_real_(is_real)
  , waist_control_mode_(1)
  , waist_control_enabled_(false)
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
  
  // 模式1相关
  default_waist_pos_.resize(joint_waist_num_);
  default_waist_pos_.setZero();
  
  // 模式2相关
  raw_mode2_waist_target_q_.resize(joint_waist_num_);
  raw_mode2_waist_target_q_.setZero();
  mode2_waist_target_received_ = false;
  
  // 控制参数
  waist_kp_.resize(joint_waist_num_);
  waist_kd_.resize(joint_waist_num_);
  
  // 初始化订阅者（订阅/robot_waist_motion_data话题）
  if (joint_waist_num_ > 0)
  {
    waist_traj_sub_ = nh_.subscribe<kuavo_msgs::robotWaistControl>(
      "/robot_waist_motion_data", 10,
      boost::bind(&WaistController::waistTrajectoryCallback, this, _1));
    
    // 订阅腰部控制使能话题
    enable_waist_control_sub_ = nh_.subscribe<std_msgs::Bool>(
      "/humanoid_controller/enable_waist_control", 10,
      boost::bind(&WaistController::enableWaistControlCallback, this, _1));
    
    ROS_INFO("[WaistController] Subscribed to /humanoid_controller/enable_waist_control (is_real=%d)", is_real_);
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
  
  // 重置模式2的目标
  mode2_waist_target_received_ = false;
  
  // 重置滤波器状态到当前位置
  if (waist_filter_initialized_)
  {
    waist_joint_pos_filter_.reset(desire_waist_q_);
    waist_joint_vel_filter_.reset(desire_waist_v_);
  }
}

void WaistController::loadSettings(const Eigen::VectorXd& waist_kp,
                                   const Eigen::VectorXd& waist_kd,
                                   const Eigen::VectorXd& default_waist_pos,
                                   double waist_mode_interpolation_velocity,
                                   double mode2_cutoff_freq)
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
  
  // 模式2外部输入的截止频率：如果传入的值无效（<=0），使用默认值5Hz
  if (mode2_cutoff_freq > 0.0)
  {
    mode2_cutoff_freq_ = mode2_cutoff_freq;
  }
  else
  {
    mode2_cutoff_freq_ = 5.0;  // 默认值5Hz
    ROS_INFO("[WaistController] Invalid or missing mode2_cutoff_freq, using default: %.1f Hz", mode2_cutoff_freq_);
  }
  
  // 如果滤波器已初始化，需要重新设置以应用新的截止频率
  if (waist_filter_initialized_ && joint_waist_num_ > 0)
  {
    waist_filter_initialized_ = false;
    ROS_INFO("[WaistController] Filter cutoff frequency updated to %.1f Hz, will re-initialize in next update()", mode2_cutoff_freq_);
  }
}

void WaistController::update(const ros::Time& time,
                             double dt,
                             const Eigen::VectorXd& joint_pos,
                             const Eigen::VectorXd& joint_vel,
                             int cmd_stance,
                             kuavo_msgs::jointCmd& joint_cmd_msg,
                             size_t jointNumReal)
{
  // if ((!waist_control_enabled_ && !waist_is_interpolating_) || joint_waist_num_ == 0)
  // {
  //   return;
  // }
  static int pre_cmd_stance = cmd_stance;
  if (pre_cmd_stance != cmd_stance)
  {
    switch (cmd_stance)
    {
      case 0:
        applyModeChange(1);
        ROS_INFO("[WaistController] Switching to Mode 1: RL control");
        break;
      case 1:
        applyModeChange(2);
        ROS_INFO("[WaistController] Switching to Mode 2: external control");
        break;
      default:
        break;
    }
    pre_cmd_stance = cmd_stance;
  }
  // 保存当前腰部位置和速度
  size_t waist_start_idx = jointNumReal;  // 腰部起始索引
  current_waist_pos_ = joint_pos.segment(waist_start_idx, joint_waist_num_);
  current_waist_vel_ = joint_vel.segment(waist_start_idx, joint_waist_num_);
  
  // 确保滤波器参数与实际控制频率匹配（仅在未初始化时使用传入的 dt 初始化）
  // 模式1和模式2共用同一组滤波器
  if (joint_waist_num_ > 0 && !waist_filter_initialized_)
  {
    Eigen::VectorXd cutoff = Eigen::VectorXd::Constant(joint_waist_num_, mode2_cutoff_freq_);
    waist_joint_pos_filter_.setParams(dt, cutoff);
    waist_joint_vel_filter_.setParams(dt, cutoff);
    waist_filter_initialized_ = true;
    ROS_INFO("[WaistController] First-order filter initialized: dt=%.4f s, cutoff=%.1f Hz", dt, mode2_cutoff_freq_);
  }
  
  // 根据当前模式更新期望状态
  if (waist_control_mode_ == 1)
  {
    updateMode1(dt);
  }
  else if (waist_control_mode_ == 2)
  {
    updateMode2(dt);
  }
  
  // 填充命令消息（根据模式决定是否填充）
  // 模式2：总是填充；模式1：只有在插值阶段填充，插值完成后让RL控制器自己控制
  if (waist_control_mode_ == 2 || waist_is_interpolating_)
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
      
      // 如果是仿真，计算前馈扭矩（PD控制）；实物不计算
      if (!is_real_)
      {
        // 计算PD控制力矩：tau = kp * (desire_q - current_q) + kd * (desire_v - current_v)
        Eigen::VectorXd pos_error = desire_waist_q_ - current_waist_pos_;
        Eigen::VectorXd vel_error = desire_waist_v_ - current_waist_vel_;
        Eigen::VectorXd tau_cmd = waist_kp_.cwiseProduct(pos_error) + waist_kd_.cwiseProduct(vel_error);
        
        // 更新力矩命令（PD控制计算出的力矩）
        joint_cmd_msg.tau[cmd_idx] = tau_cmd(i);
      }
      else
      {
        joint_cmd_msg.tau[cmd_idx] = 0;
      }      
      // 实物不计算前馈扭矩，保持原有tau值
      
      joint_cmd_msg.control_modes[cmd_idx] = 2; 
      joint_cmd_msg.joint_q[cmd_idx] = desire_waist_q_(i);
      joint_cmd_msg.joint_v[cmd_idx] = 0;
  
    }
  }
}

bool WaistController::changeMode(int target_mode)
{
  // 验证模式有效性（只支持模式1和2）
  if (target_mode != 1 && target_mode != 2)
  {
    ROS_WARN("[WaistController] Invalid waist control mode: %d (only mode 1 and 2 are supported)", target_mode);
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
  
  waist_control_mode_ = target_mode;
  
  if (target_mode == 1)
  {
    // 模式1：RL控制
    // 目标设置为默认腰部位置，通过低通滤波器平滑过渡
    // 不重置滤波器，让滤波器从当前状态平滑过渡到默认位置
    
    // 检查当前位置和默认位置的差异
    Eigen::VectorXd error = default_waist_pos_ - current_waist_pos_;
    double error_norm = error.norm();
    std::cout << "error_norm" << error_norm << std::endl;
    std::cout << "default_waist_pos_" << default_waist_pos_.transpose() << std::endl;
    std::cout << "current_waist_pos_" << current_waist_pos_.transpose() << std::endl;
    // 使用阈值（0.05 rad，约3度）来判断是否需要插值
    const double waist_tracking_error_threshold = 0.02;
    
    if (error_norm > waist_tracking_error_threshold)
    {
      // 差异较大，标记为插值状态，使用低通滤波器平滑过渡到默认位置
      waist_is_interpolating_ = true;
      ROS_INFO("[WaistController] Switching to Mode 1: will use low-pass filter to transition to default position (error=%.4f rad, cutoff=%.1f Hz)", 
               error_norm, mode2_cutoff_freq_);
    }
    else
    {
      // 差异较小，直接设置为默认位置，不需要插值
      desire_waist_q_ = default_waist_pos_;
      desire_waist_v_ = Eigen::VectorXd::Zero(joint_waist_num_);
      waist_is_interpolating_ = false;
      ROS_INFO("[WaistController] Switching to Mode 1: already at default position");
    }
    mode2_waist_target_received_ = false;
    
    ROS_INFO("[WaistController] Switching to Mode 1: RL control");
  }
  else if (target_mode == 2)
  {
    mode2_waist_target_received_ = false;
    {
      desire_waist_q_.setZero();
      desire_waist_v_.setZero();
      raw_mode2_waist_target_q_.setZero();
      waist_is_interpolating_ = true;  // 模式2始终填充命令消息
    }
    ROS_INFO("[WaistController] Switching to Mode 2: external control (cutoff=%.1f Hz, target_received=%d)", 
             mode2_cutoff_freq_, mode2_waist_target_received_);
  }
}

void WaistController::updateMode1(double dt)
{
  // 模式1：RL控制
  // 如果正在插值到默认位置，使用低通滤波器平滑过渡
  if (waist_is_interpolating_)
  {
    // 使用低通滤波器从当前位置平滑过渡到默认位置
    desire_waist_q_ = waist_joint_pos_filter_.update(default_waist_pos_);
    
    // 通过位置差分计算速度
    static Eigen::VectorXd prev_filtered_pos_mode1;
    static bool first_call_mode1 = true;
    
    if (first_call_mode1)
    {
      prev_filtered_pos_mode1 = desire_waist_q_;
      first_call_mode1 = false;
    }
    
    if (prev_filtered_pos_mode1.size() != joint_waist_num_)
    {
      prev_filtered_pos_mode1 = desire_waist_q_;
    }
    
    // 计算速度：v = (q_current - q_prev) / dt
    Eigen::VectorXd computed_vel = (desire_waist_q_ - prev_filtered_pos_mode1) / dt;
    
    // 对计算出的速度进行低通滤波
    desire_waist_v_ = waist_joint_vel_filter_.update(computed_vel);
    
    prev_filtered_pos_mode1 = desire_waist_q_;
    
    // 检查误差，如果小于阈值则结束插值，让RL控制器完全接管
    Eigen::VectorXd error = default_waist_pos_ - desire_waist_q_;
    double error_norm = error.norm();
    const double waist_tracking_error_threshold = 0.02;
    
    if (error_norm <= waist_tracking_error_threshold)
    {
      // 误差小于阈值，结束插值
      waist_joint_pos_filter_.reset(default_waist_pos_);
      desire_waist_q_ = default_waist_pos_;
      desire_waist_v_ = Eigen::VectorXd::Zero(joint_waist_num_);
      waist_is_interpolating_ = false;
      first_call_mode1 = true;  // 重置静态变量，为下次插值做准备
      ROS_INFO("[WaistController] Mode 1 interpolation completed, RL controller takes over (error=%.4f rad)", error_norm);
    }
  }
  // 插值完成后，不更新命令消息，让RL控制器完全接管
}

void WaistController::updateMode2(double dt)
{
  // 模式2：外部控制模式
  // 使用低通滤波平滑外部输入，先滤波位置，然后通过位置差分计算速度，再滤波速度
  
  // 静态变量：用于速度计算
  static Eigen::VectorXd prev_filtered_pos_mode2;
  static bool first_call_mode2 = true;
  
  // 初始化静态变量
  if (first_call_mode2 && mode2_waist_target_received_)
  {
    prev_filtered_pos_mode2 = raw_mode2_waist_target_q_;
    first_call_mode2 = false;
  }
  
  if (mode2_waist_target_received_)
  {
    // 1. 先对位置进行低通滤波
    desire_waist_q_ = waist_joint_pos_filter_.update(raw_mode2_waist_target_q_);
    
    // 2. 通过位置差分计算速度
    // 确保prev_filtered_pos_mode2已初始化
    if (prev_filtered_pos_mode2.size() != joint_waist_num_)
    {
      prev_filtered_pos_mode2 = desire_waist_q_;
    }
    
    // 计算速度：v = (q_current - q_prev) / dt
    // Eigen::VectorXd computed_vel = (desire_waist_q_ - prev_filtered_pos_mode2) / dt;
    
    // 3. 对计算出的速度进行低通滤波
    desire_waist_v_.setZero();
    
    // 更新prev_filtered_pos_mode2用于下次计算
    prev_filtered_pos_mode2 = desire_waist_q_;
  }
  else
  {

    desire_waist_q_.setZero();
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
  static constexpr double WAIST_YAW_MIN_DEG = -30.0;
  static constexpr double WAIST_YAW_MAX_DEG = 30.0;
  
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

void WaistController::enableWaistControlCallback(const std_msgs::Bool::ConstPtr& msg)
{
  bool enable = msg->data;
  
  if (enable != waist_control_enabled_)
  {
    waist_control_enabled_ = enable;
    
    if (enable)
    {
      // 启用时：切换到外部控制模式（模式2）
      if (waist_control_mode_ == 1)
      {
        changeMode(2);
      }
      ROS_INFO("[WaistController] Waist control enabled, switched to mode 2 (external control)");
    }
    else
    {
      // 禁用时：切换回RL控制模式（模式1）
      if (waist_control_mode_ != 1)
      {
        changeMode(1);
      }
      ROS_INFO("[WaistController] Waist control disabled, switched to mode 1 (RL control)");
    }
  }
}


} // namespace humanoid_controller
