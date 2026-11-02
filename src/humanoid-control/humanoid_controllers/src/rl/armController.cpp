// Pinocchio must be included before Boost headers
#include <pinocchio/fwd.hpp>

#include "humanoid_controllers/rl/armController.h"
#include "humanoid_controllers/armTorqueController.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace humanoid_controller
{
namespace
{
constexpr double kSwitchReturnPositionTolerance = 0.08;  // rad (~4.6 deg)
constexpr double kSwitchReturnVelocityTolerance = 0.20;  // rad/s
constexpr int kSwitchReturnSettledCycles = 3;
constexpr double kSwitchReturnInterpolationEpsilon = 1e-6;
}  // namespace

// 判断手臂关节是否为手腕关节（每侧 7 个关节，最后 3 个为手腕）：
// arm_idx 0-6 = zarm_l1..l7，7-13 = zarm_r1..r7 → 手腕 = l5/l6/l7 与 r5/r6/r7
static inline bool isWristJoint(int arm_idx)
{
  return (arm_idx % 7) >= 4;
}

// 判断手臂关节是否为肘关节（每侧 7 个关节的第 4 个）：
// arm_idx 0-6 = zarm_l1..l7，7-13 = zarm_r1..r7 → 肘 = l4 / r4（l1..l3 肩， l5..l7 腕）
static inline bool isElbowJoint(int arm_idx)
{
  return (arm_idx % 7) == 3;
}

// 判断手臂关节是否为肩部 pitch 关节（每侧第 1 个，绕躯干 Y 轴）：l1 / r1
static inline bool isShoulderPitchJoint(int arm_idx)
{
  return (arm_idx % 7) == 0;
}

ArmController::ArmController(ros::NodeHandle& nh, size_t joint_num, size_t joint_waist_num, size_t joint_arm_num,
                              ocs2::humanoid::TopicLogger* ros_logger)
  : nh_(nh)
  , ros_logger_(ros_logger)
  , joint_num_(joint_num)
  , joint_waist_num_(joint_waist_num)
  , joint_arm_num_(joint_arm_num)
  , arm_start_idx_(joint_num + joint_waist_num)
  , arm_control_mode_(ControlMode::kAutoSwing)
  , arm_vr_enabled_(false)
  , arm_max_tracking_velocity_(1.0)
  , arm_tracking_error_threshold_(0.1)
  , mode_interpolation_velocity_(1.0)
  , is_interpolating_(false)
  , external_cutoff_freq_(10.0) // 默认10Hz
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
  
  // 初始化锁定相关
  locked_fixed_pos_.resize(joint_arm_num);
  locked_fixed_pos_.setZero();
  locked_pos_set_ = false;
  
  // 初始化自动摆臂相关
  default_arm_pos_.resize(joint_arm_num);
  default_arm_pos_.setZero();
  auto_swing_state_ = AutoSwingState::kSwing;
  
  // 初始化外部控制相关
  raw_external_target_q_.resize(joint_arm_num);
  raw_external_target_v_.resize(joint_arm_num);
  raw_external_target_q_.setZero();
  raw_external_target_v_.setZero();
  buffered_mode2_target_q_.resize(joint_arm_num);
  buffered_mode2_target_v_.resize(joint_arm_num);
  buffered_mode2_target_q_.setZero();
  buffered_mode2_target_v_.setZero();
  buffered_mode2_target_received_ = false;
  external_target_q_.resize(joint_arm_num);
  external_target_v_.resize(joint_arm_num);
  external_target_q_.setZero();
  external_target_v_.setZero();
  filtered_pos_for_velocity_.resize(joint_arm_num);
  filtered_pos_for_velocity_.setZero();
  external_velocity_diff_valid_ = false;
  external_pause_hold_q_.resize(joint_arm_num);
  external_pause_hold_q_.setZero();
  external_pause_source_q_.resize(joint_arm_num);
  external_pause_source_q_.setZero();
  external_target_received_ = false;
  last_external_input_time_valid_ = false;
  
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
  // Keep reset atomic with respect to both external trajectory producers.
  std::lock_guard<std::mutex> input_owner_lock(external_input_owner_mutex_);
  requested_arm_mode_.store(kNoRequestedArmMode, std::memory_order_release);
  default_pose_return_for_switch_state_.store(DefaultPoseReturnState::kIdle,
                                               std::memory_order_release);
  default_pose_return_settled_cycles_ = 0;
  external_control_pause_requested_.store(false);
  external_control_pause_active_ = false;
  external_pause_resume_reacquiring_ = false;
  external_pause_source_valid_ = false;
  has_emitted_arm_command_ = false;

  // 重置插值状态
  arm_control_mode_ = ControlMode::kAutoSwing;
  arm_vr_enabled_ = false;
  is_interpolating_ = false;
  auto_swing_state_ = AutoSwingState::kSwing;
  
  // 重置缓存的cmd_arm_pos_，使用当前位置
  cmd_arm_pos_ = current_arm_pos_;
  cmd_arm_vel_ = current_arm_vel_;
  
  // 重置期望状态为当前位置
  desire_arm_q_ = current_arm_pos_;
  desire_arm_v_ = current_arm_vel_;
  
  // 重置锁定的目标位置
  locked_pos_set_ = false;
  
  // 重置外部控制的目标
  {
    std::lock_guard<std::mutex> lock(external_target_mutex_);
    raw_external_target_q_ = current_arm_pos_;
    raw_external_target_v_.setZero();
    external_target_q_ = current_arm_pos_;
    external_target_v_.setZero();
    external_target_received_ = false;
    buffered_mode2_target_q_ = current_arm_pos_;
    buffered_mode2_target_v_.setZero();
    buffered_mode2_target_received_ = false;
    last_external_input_time_valid_ = false;
  }
  external_pause_hold_q_ = current_arm_pos_;
  external_pause_source_q_ = current_arm_pos_;
  
  // 重置滤波器
  if (arm_filter_initialized_)
  {
    arm_joint_pos_filter_.reset(current_arm_pos_);
    arm_joint_vel_filter_.reset(current_arm_vel_);
  }
  resetExternalVelocityDiff();
  
  // 重置缓存的模式切换指令
  pending_arm_mode_.reset();
  offline_action_input_active_.store(false, std::memory_order_release);
  external_input_owner_epoch_.fetch_add(1, std::memory_order_acq_rel);

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
  
  // Generic external input (VR / SDK). Offline actions use a dedicated topic;
  // ArmController arbitrates the two sources before either can write a target.
  joint_sub_ = nh_.subscribe<sensor_msgs::JointState>(
    "/kuavo_arm_traj", 3,
    boost::bind(&ArmController::jointStateCallback, this, _1));
  action_traj_sub_ = nh_.subscribe<sensor_msgs::JointState>(
    "/kuavo_action_traj", 3,
    boost::bind(&ArmController::actionTrajectoryCallback, this, _1));
  
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
                                 double external_cutoff_freq)
{
  arm_max_tracking_velocity_ = max_tracking_velocity;
  arm_tracking_error_threshold_ = tracking_error_threshold;
  mode_interpolation_velocity_ = mode_interpolation_velocity;
  external_cutoff_freq_ = external_cutoff_freq;

  // 滤波器参数将在首次 update() 调用时根据实际 dt 初始化
  // 这里只更新截止频率，实际 dt 会在 update() 中设置
  // 如果已经初始化过，需要重新设置以应用新的截止频率
  if (arm_filter_initialized_ && joint_arm_num_ > 0)
  {
    arm_filter_initialized_ = false;
    ROS_INFO("[ArmController] Filter cutoff frequency updated to %.1f Hz, will re-initialize in next update()", external_cutoff_freq_);
  }
  
  if (default_arm_pos.size() == joint_arm_num_)
  {
    default_arm_pos_ = default_arm_pos;
  }
  else if (default_arm_pos.size() > 0)
  {
    ROS_WARN("[ArmController] Default arm position dimension mismatch: expected=%zu, got=%zu",
             joint_arm_num_, default_arm_pos.size());
  }
  
  ROS_INFO("[ArmController] Loaded settings: auto_swing_max_velocity=%.3f rad/s, error_threshold=%.3f rad, interpolation_velocity=%.3f rad/s",
           arm_max_tracking_velocity_, arm_tracking_error_threshold_, mode_interpolation_velocity_);
}

void ArmController::setDefaultArmPos(const Eigen::VectorXd& default_arm_pos)
{
  if (default_arm_pos.size() != static_cast<Eigen::Index>(joint_arm_num_))
  {
    ROS_WARN("[ArmController] Ignoring default arm position update: expected=%zu, got=%ld",
             joint_arm_num_, static_cast<long>(default_arm_pos.size()));
    return;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  if (default_arm_pos_.size() == default_arm_pos.size() &&
      (default_arm_pos_ - default_arm_pos).cwiseAbs().maxCoeff() < 1e-12)
  {
    return;
  }

  default_arm_pos_ = default_arm_pos;
}

void ArmController::configureImpedance(bool enabled,
                                       const Eigen::VectorXd& kp,
                                       const Eigen::VectorXd& kd,
                                       int target_source,
                                       double max_joint_vel,
                                       double force_limit,
                                       bool use_cst,
                                       bool lock_wrist)
{
  impedance_enabled_ = enabled;
  impedance_target_source_ = target_source;
  impedance_max_joint_vel_ = max_joint_vel;
  impedance_force_limit_ = force_limit;
  impedance_use_cst_ = use_cst;
  impedance_lock_wrist_ = lock_wrist;

  if (kp.size() == static_cast<int>(joint_arm_num_) && kd.size() == static_cast<int>(joint_arm_num_))
  {
    impedance_kp_ = kp;
    impedance_kd_ = kd;
  }
  else
  {
    // 兜底：默认低刚度（柔顺），避免进入模式时增益异常
    impedance_kp_ = Eigen::VectorXd::Constant(joint_arm_num_, 6.0);
    impedance_kd_ = Eigen::VectorXd::Constant(joint_arm_num_, 0.3);
    ROS_WARN("[ArmController] Impedance kp/kd dimension mismatch (kp=%ld, kd=%ld, expected=%zu), using defaults",
             kp.size(), kd.size(), joint_arm_num_);
  }

  ROS_INFO("[ArmController] Impedance configured: enabled=%d, target_source=%d, max_vel=%.2f, force_limit=%.2f, use_cst=%d, lock_wrist=%d",
           enabled ? 1 : 0, impedance_target_source_, impedance_max_joint_vel_,
           impedance_force_limit_, impedance_use_cst_ ? 1 : 0, impedance_lock_wrist_ ? 1 : 0);
}

void ArmController::configureImpedanceHandover(double handover_duration)
{
  impedance_handover_duration_ = (handover_duration > 0.0) ? handover_duration : 0.0;
  impedance_handover_active_ = false;

  if (impedance_handover_duration_ > 0.0)
  {
    ROS_INFO("[ArmController] Impedance handover enabled: duration=%.2fs "
             "(transition joint_q of ALL arm joints + kp/kd of non-wrist joints first, then release motor mode switch)",
             impedance_handover_duration_);
  }
  else
  {
    ROS_INFO("[ArmController] Impedance handover disabled (duration=%.2fs)", impedance_handover_duration_);
  }
}

void ArmController::setHandoverTargetGains(const Eigen::VectorXd& kp, const Eigen::VectorXd& kd)
{
  if (kp.size() == 0 || kp.size() != kd.size())
  {
    handover_target_kp_.resize(0);
    handover_target_kd_.resize(0);
    return;
  }
  handover_target_kp_ = kp;
  handover_target_kd_ = kd;
  ROS_INFO("[ArmController] Handover target gains set (size=%ld)", static_cast<long>(kp.size()));
}

void ArmController::configureImpedanceSwitchGuard(double guard_deg)
{
  impedance_switch_guard_deg_ = (guard_deg > 0.0) ? guard_deg : 0.0;
  if (impedance_switch_guard_deg_ > 0.0)
  {
    ROS_INFO("[ArmController] Impedance switch guard enabled: arm joints must be within %.1f deg of the "
             "impedance target pose before switching controller", impedance_switch_guard_deg_);
  }
  else
  {
    ROS_INFO("[ArmController] Impedance switch guard disabled");
  }
}

void ArmController::configureElbowFollow(bool enabled, double ratio, double max_offset_deg)
{
  elbow_follow_enabled_ = enabled && (ratio > 0.0);
  elbow_follow_ratio_ = (ratio > 0.0) ? ratio : 0.0;
  elbow_follow_max_offset_ = (max_offset_deg > 0.0) ? max_offset_deg * M_PI / 180.0 : 0.0;

  if (elbow_follow_enabled_)
  {
    ROS_INFO("[ArmController] Elbow follow enabled: k=%.3f, max_offset=%.1f deg "
             "(elbow tracks shoulder-pitch negative deviation only, CSP position mode)",
             elbow_follow_ratio_, max_offset_deg);
  }
  else
  {
    ROS_INFO("[ArmController] Elbow follow disabled");
  }
}

bool ArmController::isMeasuredArmAtDefault() const
{
  return joint_arm_num_ > 0 &&
      current_arm_pos_.size() == static_cast<int>(joint_arm_num_) &&
      current_arm_vel_.size() == static_cast<int>(joint_arm_num_) &&
      default_arm_pos_.size() == static_cast<int>(joint_arm_num_) &&
      current_arm_pos_.allFinite() && current_arm_vel_.allFinite() && default_arm_pos_.allFinite() &&
      (current_arm_pos_ - default_arm_pos_).lpNorm<Eigen::Infinity>() <=
          std::min(arm_tracking_error_threshold_, kSwitchReturnPositionTolerance) &&
      current_arm_vel_.lpNorm<Eigen::Infinity>() <= kSwitchReturnVelocityTolerance;
}

bool ArmController::isImpedanceSwitchAllowed(std::string& reason) const
{
  reason.clear();
  if (impedance_switch_guard_deg_ <= 0.0)
  {
    return true;  // 门禁关闭
  }
  if (arm_control_mode_ != ControlMode::kImpedance && arm_control_mode_ != ControlMode::kCartesianImpedance)
  {
    return true;  // 非阻抗模式不限制
  }
  if (joint_arm_num_ == 0 || current_arm_pos_.size() != static_cast<int>(joint_arm_num_) ||
      !current_arm_pos_.allFinite())
  {
    reason = "switch blocked: invalid impedance arm measurements";
    return false;
  }

  // 目标位：模式3 = 阻抗目标（默认位 / 进入时位姿）；模式4 = 进入时的关节位姿
  Eigen::VectorXd target;
  const bool is_cartesian = (arm_control_mode_ == ControlMode::kCartesianImpedance);
  if (is_cartesian)
  {
    if (cartesian_hold_joint_pos_.size() != static_cast<int>(joint_arm_num_))
    {
      reason = "switch blocked: missing Cartesian impedance reference";
      return false;
    }
    target = cartesian_hold_joint_pos_;
  }
  else if (impedance_target_source_ == 1 && impedance_hold_pos_set_ &&
           impedance_hold_pos_.size() == static_cast<int>(joint_arm_num_))
  {
    target = impedance_hold_pos_;
  }
  else
  {
    if (default_arm_pos_.size() != static_cast<int>(joint_arm_num_))
    {
      reason = "switch blocked: missing impedance arm reference";
      return false;
    }
    target = default_arm_pos_;
  }

  if (!target.allFinite())
  {
    reason = "switch blocked: non-finite impedance arm reference";
    return false;
  }

  // 只检查实际参与阻抗的关节：模式3下手腕被锁定（目标固定默认位）不参与门禁，避免误卡
  double max_dev_rad = 0.0;
  int worst_joint = -1;
  for (int i = 0; i < static_cast<int>(joint_arm_num_); ++i)
  {
    if (!is_cartesian && impedance_lock_wrist_ && isWristJoint(i))
    {
      continue;
    }
    // 肘随动（模式3）：肘以 CSP 跟随肩 pitch，其相对参考位的偏差是“被指令的”
    // （与锁定的手腕同理），不参与门禁判定，避免肩被压住时无法切出控制器。
    // 肩 pitch 自身仍参与判定，切出保护不受影响。
    if (!is_cartesian && elbow_follow_enabled_ && isElbowJoint(i))
    {
      continue;
    }
    const double dev = std::abs(current_arm_pos_(i) - target(i));
    if (dev > max_dev_rad)
    {
      max_dev_rad = dev;
      worst_joint = i;
    }
  }

  const double limit_rad = impedance_switch_guard_deg_ * M_PI / 180.0;
  if (max_dev_rad > limit_rad)
  {
    std::ostringstream oss;
    oss << "switch blocked by impedance arm-position guard: arm joint(local) " << worst_joint
        << " deviation " << (max_dev_rad * 180.0 / M_PI) << " deg > " << impedance_switch_guard_deg_
        << " deg; release the arm or wait until it returns to the target pose";
    reason = oss.str();
    return false;
  }
  return true;
}

void ArmController::applyImpedanceHandover(const ros::Time& time, kuavo_msgs::jointCmd& joint_cmd_msg)
{
  if (!impedance_handover_active_ || impedance_handover_duration_ <= 0.0 || joint_arm_num_ == 0)
  {
    impedance_handover_active_ = false;
    return;
  }

  const double alpha =
      std::clamp((time - impedance_handover_start_time_).toSec() / impedance_handover_duration_, 0.0, 1.0);
  const int cmd_size = static_cast<int>(joint_cmd_msg.control_modes.size());
  const int q_size = static_cast<int>(joint_cmd_msg.joint_q.size());
  const int v_size = static_cast<int>(joint_cmd_msg.joint_v.size());
  const int kp_size = static_cast<int>(joint_cmd_msg.joint_kp.size());
  const int kd_size = static_cast<int>(joint_cmd_msg.joint_kd.size());
  const int arm_n = static_cast<int>(joint_arm_num_);
  // 保持阻抗时使用的电机模式（非手腕关节），过渡完成后再变回目标控制器的模式
  const int hold_mode = impedance_use_cst_ ? 0 : 2;

  for (int i = 0; i < arm_n; ++i)
  {
    const int g = static_cast<int>(arm_start_idx_) + i;
    if (g < 0 || g >= cmd_size)
    {
      continue;
    }

    // ---------- 1) 位置参考过渡（所有手臂关节：默认关节位置 → 新控制器当前值） ----------
    if (g < q_size && handover_from_q_.size() == arm_n)
    {
      const double to_q = joint_cmd_msg.joint_q[g];
      const double from_q = handover_from_q_(i);
      joint_cmd_msg.joint_q[g] = from_q + (to_q - from_q) * alpha;
    }
    if (g < v_size && handover_from_v_.size() == arm_n)
    {
      const double to_v = joint_cmd_msg.joint_v[g];
      const double from_v = handover_from_v_(i);
      joint_cmd_msg.joint_v[g] = from_v + (to_v - from_v) * alpha;
    }

    // ---------- 2) 增益过渡 + 模式保持（非手腕关节；手腕在阻抗模式下未接管，故不碰） ----------
    const bool position_controlled = !impedance_handover_from_cartesian_ &&
        ((impedance_lock_wrist_ && isWristJoint(i)) ||
         (elbow_follow_enabled_ && isElbowJoint(i)));
    if (position_controlled)
    {
      continue;
    }

    double to_kp = (g < kp_size) ? joint_cmd_msg.joint_kp[g] : 0.0;
    double to_kd = (g < kd_size) ? joint_cmd_msg.joint_kd[g] : 0.0;
    if (g < static_cast<int>(handover_target_kp_.size()) && g < static_cast<int>(handover_target_kd_.size()))
    {
      to_kp = handover_target_kp_(g);
      to_kd = handover_target_kd_(g);
    }
    const double from_kp = (handover_from_kp_.size() == arm_n) ? handover_from_kp_(i) : to_kp;
    const double from_kd = (handover_from_kd_.size() == arm_n) ? handover_from_kd_(i) : to_kd;
    if (g < kp_size)
    {
      joint_cmd_msg.joint_kp[g] = from_kp + (to_kp - from_kp) * alpha;
    }
    if (g < kd_size)
    {
      joint_cmd_msg.joint_kd[g] = from_kd + (to_kd - from_kd) * alpha;
    }
    // 关键：过渡期间保持旧电机模式，不让驱动在参考/增益还没到位时就换律
    if (alpha < 1.0)
    {
      joint_cmd_msg.control_modes[g] = hold_mode;
    }
  }

  if (alpha >= 1.0)
  {
    impedance_handover_active_ = false;
    ROS_INFO("[ArmController] Impedance handover done (%.2fs): arm joint_q refs and kp/kd aligned, "
             "releasing motor mode hold (hold_mode=%d → target)",
             impedance_handover_duration_, hold_mode);
  }
}

void ArmController::configureCartesianImpedance(bool enabled,
                                                double kx,
                                                double dx,
                                                int ee_frame_mode,
                                                double force_est_gain)
{
  cartesian_impedance_enabled_ = enabled;
  cartesian_kx_ = kx;
  cartesian_dx_ = dx;
  cartesian_ee_frame_mode_ = ee_frame_mode;
  cartesian_force_est_gain_ = force_est_gain;

  ROS_INFO("[ArmController] Cartesian impedance configured: enabled=%d, kx=%.1f, dx=%.1f, ee_frame=%d, force_est_gain=%.1f",
           enabled ? 1 : 0, cartesian_kx_, cartesian_dx_, cartesian_ee_frame_mode_, cartesian_force_est_gain_);
}

void ArmController::setMeasuredJointTorque(const Eigen::VectorXd& joint_torque)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  measured_joint_torque_ = joint_torque;
  measured_joint_torque_valid_ = (joint_torque.size() > 0 && joint_torque.allFinite());
}

void ArmController::setTorqueLimits(const Eigen::VectorXd& torque_limits)
{
  if (torque_limits.size() == static_cast<int>(joint_arm_num_))
  {
    torque_limits_ = torque_limits;
    ROS_INFO("[ArmController] Torque limits set (arm): min=%.2f, max=%.2f",
             torque_limits_.minCoeff(), torque_limits_.maxCoeff());
  }
  else
  {
    ROS_WARN("[ArmController] Torque limits dimension mismatch: got=%ld, expected=%zu",
             torque_limits.size(), joint_arm_num_);
  }
}

void ArmController::setExternalCommandBufferCallback(std::function<bool()> callback)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  external_command_buffer_callback_ = std::move(callback);
}

void ArmController::setExternalControlPaused(bool paused)
{
  external_control_pause_requested_.store(paused);
}

void ArmController::clearExternalTarget()
{
  std::lock_guard<std::mutex> input_owner_lock(external_input_owner_mutex_);
  // Invalidate a generic callback that may currently be evaluating the
  // controller-routing callback without holding the owner mutex.
  external_input_owner_epoch_.fetch_add(1, std::memory_order_acq_rel);
  clearExternalTargetStateForOwnerSwitch();
}

void ArmController::setOfflineActionInputActive(bool active)
{
  std::lock_guard<std::mutex> input_owner_lock(external_input_owner_mutex_);
  const bool was_active = offline_action_input_active_.load(std::memory_order_relaxed);
  if (was_active == active)
  {
    return;
  }

  // When handing ownership to the action source, publish the new owner first so
  // the WBC-side generic-buffer check cannot apply an old generic target while
  // it is being cleared. On hand-back, clear first and expose generic ownership
  // only after the old action target has gone.
  if (active)
  {
    offline_action_input_active_.store(true, std::memory_order_release);
  }
  external_input_owner_epoch_.fetch_add(1, std::memory_order_acq_rel);
  clearExternalTargetStateForOwnerSwitch();
  if (!active)
  {
    offline_action_input_active_.store(false, std::memory_order_release);
  }

  ROS_INFO("[ArmController] External input owner -> %s",
           active ? "offline action (/kuavo_action_traj)" : "generic (/kuavo_arm_traj)");
}

void ArmController::clearExternalTargetStateForOwnerSwitch()
{
  std::lock_guard<std::mutex> lock(external_target_mutex_);
  external_target_received_ = false;
  raw_external_target_q_ = desire_arm_q_;
  raw_external_target_v_.setZero();
  external_target_q_ = desire_arm_q_;
  external_target_v_.setZero();
  buffered_mode2_target_q_ = desire_arm_q_;
  buffered_mode2_target_v_.setZero();
  buffered_mode2_target_received_ = false;
  last_external_input_time_valid_ = false;
}

void ArmController::updateInternalState(const Eigen::VectorXd& joint_pos,
                                         const Eigen::VectorXd& joint_vel,
                                         size_t arm_start_idx)
{
  current_arm_pos_ = joint_pos.segment(arm_start_idx, joint_arm_num_);
  current_arm_vel_ = joint_vel.segment(arm_start_idx, joint_arm_num_);
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

  DefaultPoseReturnState expected_return_state = DefaultPoseReturnState::kRequested;
  if (default_pose_return_for_switch_state_.compare_exchange_strong(
          expected_return_state, DefaultPoseReturnState::kStarted,
          std::memory_order_acq_rel))
  {
    default_pose_return_settled_cycles_ = 0;
    external_control_pause_requested_.store(false);
    applyModeChange(static_cast<int>(ControlMode::kAutoSwing), true);
    ROS_INFO("[ArmController] Started control-thread interpolation to default pose for controller switch");
  }

  // 1. 处理自动模式切换
  // handleAutoModeSwitch(cmd_stance);
  // 确保滤波器参数与实际控制频率匹配（仅在未初始化时使用传入的 dt 初始化）
  if (joint_arm_num_ > 0 && !arm_filter_initialized_)
  {
    Eigen::VectorXd cutoff = Eigen::VectorXd::Constant(joint_arm_num_, external_cutoff_freq_);
    arm_joint_pos_filter_.setParams(dt, cutoff);
    arm_joint_vel_filter_.setParams(dt, cutoff);
    arm_filter_initialized_ = true;
    ROS_INFO("[ArmController] Filter initialized: dt=%.4f s, cutoff=%.1f Hz", dt, external_cutoff_freq_);
  }

  // 模式回调只投递请求；所有控制状态统一在此线程修改，
  // 避免 ROS 回调与 updateAutoSwing()/updateExternalControl() 交叉写状态
  executeRequestedModeChange();

  applyExternalControlPauseState();
  if (!external_control_pause_active_)
  {
    applyBufferedMode2TargetIfReady();
  }

  // 2. 根据当前模式更新期望状态
  if (arm_control_mode_ == ControlMode::kLocked)
  {
    updateLockedPose(dt);
  }
  else if (arm_control_mode_ == ControlMode::kAutoSwing)
  {
    updateAutoSwing(time, dt, cmd_stance);
  }
  else if (arm_control_mode_ == ControlMode::kExternal)
  {
    if (external_control_pause_active_)
    {
      // Keep emitting the exact latched command while recomputing torque from
      // the current measured state in fillJointCmdMessage().
      desire_arm_q_ = external_pause_hold_q_;
      desire_arm_v_.setZero();
    }
    else if (is_returning_from_external_) {
      updateAutoSwing(time, dt, cmd_stance);
      // 回家曲线跑完（已离开 kHoming）才翻回自动摆臂 —— 判定看"曲线跑完"，不看子状态终值
      if (auto_swing_state_ != AutoSwingState::kHoming) {
        arm_control_mode_ = ControlMode::kAutoSwing;
        is_returning_from_external_ = false;
        ROS_INFO("[ArmController] External→Auto Swing interpolation complete, switched to AUTO_SWING");
      }
    } else {
      updateExternalControl(dt);
    }
  }
  else if (arm_control_mode_ == ControlMode::kImpedance)
  {
    updateImpedanceControl(dt);
  }
  else if (arm_control_mode_ == ControlMode::kCartesianImpedance)
  {
    updateCartesianImpedance(dt);
  }

  // 3. 填充命令消息（根据模式决定是否填充）
  // 注意：不再对全模式应用滤波，锁定和自动摆臂的平滑插值不需要滤波
  // 自动摆臂门禁：子状态非摆臂（回家中/按住）时，本控制器填充手臂指令，覆盖 RL
  const bool force_default_pose_output =
      default_pose_return_for_switch_state_.load(std::memory_order_acquire) !=
      DefaultPoseReturnState::kIdle;
  if (arm_control_mode_ == ControlMode::kLocked || arm_control_mode_ == ControlMode::kExternal ||
      arm_control_mode_ == ControlMode::kImpedance || arm_control_mode_ == ControlMode::kCartesianImpedance ||
      (arm_control_mode_ == ControlMode::kAutoSwing && auto_swing_state_ != AutoSwingState::kSwing) ||
      force_default_pose_output)
  {
    fillJointCmdMessage(joint_cmd_msg, desire_arm_q_, desire_arm_v_,
                       joint_pos, joint_vel);

    if (force_default_pose_output)
    {
      const bool dimensions_valid =
          desire_arm_q_.size() == default_arm_pos_.size() &&
          current_arm_pos_.size() == default_arm_pos_.size() &&
          current_arm_vel_.size() == default_arm_pos_.size();
      const double desired_error = dimensions_valid
                                       ? (desire_arm_q_ - default_arm_pos_).lpNorm<Eigen::Infinity>()
                                       : std::numeric_limits<double>::infinity();
      const double measured_error = dimensions_valid
                                        ? (current_arm_pos_ - default_arm_pos_).lpNorm<Eigen::Infinity>()
                                        : std::numeric_limits<double>::infinity();
      const double measured_velocity = dimensions_valid
                                           ? current_arm_vel_.lpNorm<Eigen::Infinity>()
                                           : std::numeric_limits<double>::infinity();
      const double position_tolerance =
          std::min(arm_tracking_error_threshold_, kSwitchReturnPositionTolerance);
      const bool settled_this_cycle =
          dimensions_valid && !is_interpolating_ &&
          desired_error <= kSwitchReturnInterpolationEpsilon &&
          measured_error <= position_tolerance &&
          measured_velocity <= kSwitchReturnVelocityTolerance;
      default_pose_return_settled_cycles_ = settled_this_cycle
                                                ? default_pose_return_settled_cycles_ + 1
                                                : 0;
      const bool return_complete =
          default_pose_return_settled_cycles_ >= kSwitchReturnSettledCycles;
      auto return_state = default_pose_return_for_switch_state_.load(std::memory_order_acquire);
      bool became_complete = false;
      if (return_complete && return_state == DefaultPoseReturnState::kStarted)
      {
        became_complete = default_pose_return_for_switch_state_.compare_exchange_strong(
            return_state, DefaultPoseReturnState::kComplete, std::memory_order_acq_rel);
      }
      else if (!return_complete && return_state == DefaultPoseReturnState::kComplete)
      {
        (void)default_pose_return_for_switch_state_.compare_exchange_strong(
            return_state, DefaultPoseReturnState::kStarted, std::memory_order_acq_rel);
      }
      if (became_complete)
      {
        ROS_INFO("[ArmController] Default-pose return complete for controller switch "
                 "(max error=%.4f rad, max velocity=%.4f rad/s)",
                 measured_error, measured_velocity);
      }
    }
  }

  // 退出阻抗时的手臂过渡：保持阻抗时的电机模式，把 joint_q（默认位参考）与 kp/kd 逐渐过渡到
  // 新控制器当前下发的值；过渡完成后才停止覆盖模式 → 驱动器在“参考/增益已匹配”时换律，避免冲击。
  applyImpedanceHandover(time, joint_cmd_msg);
  
  // 5. 发布模式状态（如果ros_logger可用）
  if (ros_logger_)
  {
    // 发布当前模式
    ros_logger_->publishValue("/arm_controller/mode", static_cast<double>(static_cast<int>(arm_control_mode_)));
    ros_logger_->publishValue("/arm_controller/external_paused",
                              external_control_pause_active_ ? 1.0 : 0.0);
    

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
  has_emitted_arm_command_ = true;
  
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
  // 笛卡尔阻抗模式：直接使用 updateCartesianImpedance 计算好的力矩（含重力补偿）
  const bool is_cartesian_impedance = (arm_control_mode_ == ControlMode::kCartesianImpedance);
  if (is_cartesian_impedance && cartesian_tau_override_valid_ &&
      cartesian_tau_override_.size() == static_cast<int>(joint_arm_num_))
  {
    arm_tau_desired = cartesian_tau_override_;
  }
  else if (arm_torque_controller_)
  {
    arm_torque_controller_->setMeasuredState(current_arm_pos, current_arm_vel);
    arm_tau_desired = arm_torque_controller_->computeTorque(
      target_q, target_v, Eigen::VectorXd::Zero(joint_arm_num_));
  }
  
  // 只更新joint_q、joint_v和tau，保留其他原有值
  const bool is_impedance_mode = (arm_control_mode_ == ControlMode::kImpedance ||
                                  arm_control_mode_ == ControlMode::kCartesianImpedance);
  for(size_t i = arm_start_idx_; i < arm_start_idx_ + joint_arm_num_; ++i)
  {
    size_t arm_idx = i - arm_start_idx_;
    joint_cmd_msg.joint_q[i] = target_q(arm_idx);
    joint_cmd_msg.joint_v[i] = target_v(arm_idx);
    // 如果计算了力矩，则更新tau；否则保留原有值
    if (arm_torque_controller_)
    {
      double tau = arm_tau_desired(arm_idx);
      // 阻抗模式：力矩钳位（软限制 impedance_force_limit_ 与硬件 torque_limits_ 取小值）
      if (is_impedance_mode)
      {
        double limit = 0.0;
        if (torque_limits_.size() == static_cast<int>(joint_arm_num_) && torque_limits_(arm_idx) > 0.0)
        {
          limit = torque_limits_(arm_idx);
        }
        if (impedance_force_limit_ > 0.0)
        {
          limit = (limit > 0.0) ? std::min(limit, impedance_force_limit_) : impedance_force_limit_;
        }
        if (limit > 0.0)
        {
          tau = std::max(-limit, std::min(limit, tau));
        }
      }
      joint_cmd_msg.tau[i] = tau;
    }
    // 阻抗模式：写入 CST 力矩模式 + 阻抗增益（覆盖 RL 的 CSP 模式）
    if (is_impedance_mode)
    {
      // 锁手腕（仅模式3）：手腕关节跳过阻抗块，保留原有 CSP(2) + motorPdo 高增益 → 位置锁定在默认位。
      // 注意：只有模式3受 lock_wrist 控制；模式4（笛卡尔阻抗）末端整体柔顺，不锁手腕。
      const bool skip_wrist = (arm_control_mode_ == ControlMode::kImpedance &&
                               impedance_lock_wrist_ && isWristJoint(static_cast<int>(arm_idx)));
      // 肘随动（仅模式3）：肘关节用 CSP(2) 位置控制跟随肩 pitch，不写 CST 阻抗块
      const bool elbow_follow_joint = (arm_control_mode_ == ControlMode::kImpedance &&
                                       elbow_follow_enabled_ && isElbowJoint(static_cast<int>(arm_idx)));
      if (elbow_follow_joint && i < joint_cmd_msg.control_modes.size())
      {
        // 2=CSP 位置模式：目标为 updateImpedanceControl 算出的 joint_q，
        // kp/kd 与 motorPdo 增益保留 RL 路径写入的值（与普通位置控制一致）
        joint_cmd_msg.control_modes[i] = 2;
      }
      if (!skip_wrist && !elbow_follow_joint)
      {
        if (i < joint_cmd_msg.control_modes.size())
        {
          // control_modes：0=CST（力矩/PD柔顺），1=CSV（速度），2=CSP（位置）
          // 阻抗必须用 CST(0)：CSP(2) 会被 RLControllerBase 的 motorPdo 覆盖成高刚度（柔顺失效）
          // RUIWO 电机固件可能不支持 CST 纯力矩；此配置项作为兜底，默认 true
          joint_cmd_msg.control_modes[i] = impedance_use_cst_ ? 0 : 2;
        }
        if (i < joint_cmd_msg.joint_kp.size() && impedance_kp_.size() == static_cast<int>(joint_arm_num_))
        {
          joint_cmd_msg.joint_kp[i] = impedance_kp_(arm_idx);
        }
        if (i < joint_cmd_msg.joint_kd.size() && impedance_kd_.size() == static_cast<int>(joint_arm_num_))
        {
          joint_cmd_msg.joint_kd[i] = impedance_kd_(arm_idx);
        }
      }
    }
    // joint_kp, joint_kd, torque_limits, control_modes 等保留原有值
  }
}

bool ArmController::requestDefaultPoseReturnForSwitch()
{
  auto expected = DefaultPoseReturnState::kIdle;
  if (!default_pose_return_for_switch_state_.compare_exchange_strong(
          expected, DefaultPoseReturnState::kRequested, std::memory_order_acq_rel))
  {
    return true;
  }

  // A paused teleoperation stream must not keep the arm latched at its last VR pose.
  external_control_pause_requested_.store(false);
  ROS_INFO("[ArmController] Queued control-thread arm return to default pose for controller switch");
  return true;
}

void ArmController::cancelDefaultPoseReturnForSwitch()
{
  default_pose_return_for_switch_state_.store(DefaultPoseReturnState::kIdle,
                                               std::memory_order_release);
}

bool ArmController::changeMode(int target_mode)
{
  // 检查是否处于插值过程中
  // if (is_interpolating_ && target_mode != arm_control_mode_)
  // {
  //   // 缓存指令，待插值完成后执行
  //   pending_arm_mode_ = target_mode;   // commented out: mode caching
  //   has_pending_mode_change_ = true;
  //   ROS_INFO("[ArmController] Interpolation in progress, mode change command cached (current: %d -> target: %d)",
  //            arm_control_mode_, target_mode);
  //   return false;  // 返回false表示指令已缓存，未立即执行
  // }
  
  // 验证模式有效性：模式 3/4（阻抗）仅在配置了对应阻抗参数后可用；
  // 未配置阻抗的控制器保持基线行为（>2 视为非法并拒绝），避免无阻抗配置的控制器
  // 因外部误发 mode3/4 而意外进入柔顺模式。
  if (target_mode < 0 || target_mode > 4 ||
      (target_mode == 3 && !impedance_enabled_) ||
      (target_mode == 4 && (!cartesian_impedance_enabled_ || !arm_torque_controller_)))
  {
    ROS_WARN("[ArmController] Invalid control mode: %d", target_mode);
    return false;
  }

  if (default_pose_return_for_switch_state_.load(std::memory_order_acquire) !=
          DefaultPoseReturnState::kIdle &&
      target_mode == static_cast<int>(ControlMode::kAutoSwing))
  {
    // The control loop owns initialization of the homing curve. A matching
    // global mode-1 acknowledgement must not restart it from a ROS callback.
    return true;
  }

  if (default_pose_return_for_switch_state_.load(std::memory_order_acquire) !=
      DefaultPoseReturnState::kIdle)
  {
    ROS_WARN_THROTTLE(1.0,
                      "[ArmController] Ignore mode %d while returning to default pose for controller switch",
                      target_mode);
    return false;
  }

  // External -> AutoSwing 归位期间 getMode() 仍为 2。重复的 mode1 请求不能
  // 重新初始化插值，否则归位曲线会在每个控制周期从头开始、永远无法完成。
  if (target_mode == static_cast<int>(ControlMode::kAutoSwing) &&
      arm_control_mode_ == ControlMode::kExternal && is_returning_from_external_)
  {
    return true;
  }
  
  // 执行模式切换
  applyModeChange(target_mode);
  return true;
}

bool ArmController::requestModeChange(int target_mode)
{
  if (target_mode < 0 || target_mode > 4 ||
      (target_mode == 3 && !impedance_enabled_) ||
      (target_mode == 4 && (!cartesian_impedance_enabled_ || !arm_torque_controller_)))
  {
    ROS_WARN("[ArmController] Invalid requested control mode: %d", target_mode);
    return false;
  }

  const auto return_state =
      default_pose_return_for_switch_state_.load(std::memory_order_acquire);
  if (return_state != DefaultPoseReturnState::kIdle)
  {
    // 与 changeMode() 的原有语义一致：归位期间接受重复 mode1，拒绝其他模式
    return target_mode == static_cast<int>(ControlMode::kAutoSwing);
  }

  requested_arm_mode_.store(target_mode, std::memory_order_release);
  return true;
}

void ArmController::executeRequestedModeChange()
{
  const int target_mode =
      requested_arm_mode_.exchange(kNoRequestedArmMode, std::memory_order_acq_rel);
  if (target_mode == kNoRequestedArmMode)
  {
    return;
  }

  // 上层可能重复投递同一模式；不能因此重置已经开始的插值
  if (target_mode == static_cast<int>(arm_control_mode_) && !is_returning_from_external_)
  {
    return;
  }

  (void)changeMode(target_mode);
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
    if (pending_arm_mode_.has_value()) executePendingModeChange();
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
  res.mode = static_cast<int>(arm_control_mode_);

  return true;
}

bool ArmController::getModeCallback(kuavo_msgs::changeArmCtrlMode::Request &req,
                                   kuavo_msgs::changeArmCtrlMode::Response &res)
{
  res.result = true;
  res.mode = static_cast<int>(arm_control_mode_);
  return true;
}

void ArmController::applyModeChange(int target_mode, bool force_interpolation_to_default)
{
  // 记录进入时的旧模式（供模式变化回调使用）
  const int old_mode = static_cast<int>(arm_control_mode_);

  // 退出阻抗模式（3/4）时：恢复原始 RL 增益，清除笛卡尔力矩覆盖
  if ((arm_control_mode_ == ControlMode::kImpedance || arm_control_mode_ == ControlMode::kCartesianImpedance) &&
      target_mode != 3 && target_mode != 4)
  {
    if (arm_torque_controller_ && joint_kp_.size() == static_cast<int>(joint_arm_num_))
    {
      arm_torque_controller_->setGains(joint_kp_, joint_kd_);
      ROS_INFO("[ArmController] Leaving impedance mode: restored original gains");
    }
    cartesian_tau_override_valid_ = false;
    impedance_hold_pos_set_ = false;
    cartesian_hold_pose_set_ = false;

    // 退出阻抗时的手臂过渡：记录起点（阻抗时的位置参考与增益），过渡完成后才放开模式切换
    if (impedance_handover_duration_ > 0.0 && joint_arm_num_ > 0)
    {
      impedance_handover_from_cartesian_ =
          arm_control_mode_ == ControlMode::kCartesianImpedance;
      // 起点位置参考：阻抗模式当前下发的参考（模式3=默认位/进入位姿，模式4=进入时位姿）
      handover_from_q_ = (desire_arm_q_.size() == static_cast<int>(joint_arm_num_))
                             ? desire_arm_q_
                             : current_arm_pos_;
      handover_from_v_.resize(joint_arm_num_);
      handover_from_v_.setZero();
      handover_from_kp_ = (impedance_kp_.size() == static_cast<int>(joint_arm_num_))
                              ? impedance_kp_
                              : Eigen::VectorXd::Zero(joint_arm_num_);
      handover_from_kd_ = (impedance_kd_.size() == static_cast<int>(joint_arm_num_))
                              ? impedance_kd_
                              : Eigen::VectorXd::Zero(joint_arm_num_);
      impedance_handover_start_time_ = ros::Time::now();
      impedance_handover_active_ = true;
      ROS_INFO("[ArmController] Impedance handover started (%.2fs): transitioning joint_q (from impedance "
               "target pose) + kp/kd before motor mode switch (arm joints=%zu)",
               impedance_handover_duration_, joint_arm_num_);
    }
  }
  else if (target_mode == 3 || target_mode == 4)
  {
    // 重新进入阻抗：取消未完成的过渡
    impedance_handover_active_ = false;
  }

  if (target_mode == 0)
  {
    // 使用当前手臂位置作为插值起点，插值到固定位置
    desire_arm_q_ = current_arm_pos_;
    desire_arm_v_ = current_arm_vel_;
    
    // 锁定目标：基线行为为“固定到切换瞬间的指令位置”（cmd_arm_pos_），
    // 仅当从阻抗模式（3/4）退出时才锁定到默认关节位置（阻抗柔顺后的安全归宿）；
    // 这样其他控制器的锁定语义与基线完全一致。
    const bool from_impedance =
        (old_mode == static_cast<int>(ControlMode::kImpedance) ||
         old_mode == static_cast<int>(ControlMode::kCartesianImpedance));
    locked_fixed_pos_ = (from_impedance &&
                         default_arm_pos_.size() == static_cast<int>(joint_arm_num_))
                            ? default_arm_pos_
                            : cmd_arm_pos_;
    
    // 初始化平滑插值状态
    resetInterpolationState(ros::Time::now(), current_arm_pos_, locked_fixed_pos_);
    locked_pos_set_ = true;
    
    is_interpolating_ = true;

    ROS_INFO("[ArmController] Switching to Locked: will smoothly transition to fixed position");
    // 锁定：固定到当前动作
    arm_control_mode_ = ControlMode::kLocked;
    arm_vr_enabled_ = false;
    is_returning_from_external_ = false;
    auto_swing_state_ = AutoSwingState::kSwing;  // 子状态仅在自动摆臂时有意义，离开时复位
  }
  else if (target_mode == 1)
  {
   
    
    // 使用当前手臂位置作为插值起点，插值到默认位置
    desire_arm_q_ = current_arm_pos_;
    desire_arm_v_ = current_arm_vel_;
    
    // 初始化平滑插值状态
    resetInterpolationState(ros::Time::now(), current_arm_pos_, default_arm_pos_);
    
    // 检查当前位置和默认位置的差异
    Eigen::VectorXd error = default_arm_pos_ - current_arm_pos_;
    double error_norm = error.norm();

    const bool needs_interpolation = force_interpolation_to_default
                                         ? error.lpNorm<Eigen::Infinity>() > kSwitchReturnInterpolationEpsilon
                                         : error_norm > arm_tracking_error_threshold_;
    if (needs_interpolation)
    {
      // 偏差大：发起回家，平滑过渡到默认位置（行走中也允许跑完，见 updateMode1）
      is_interpolating_ = true;
      auto_swing_state_ = AutoSwingState::kHoming;
      ROS_INFO("[ArmController] Switching to Auto Swing: will smoothly transition (Minimum Jerk) to default position (error=%.4f rad, duration=%.2f s)",
               error_norm, interpolation_duration_);
    }
    else
    {
      // 偏差小：直接置为默认位置，回到摆臂（站立时下一帧会由 updateMode1 重新发起回家，与基线一致）
      desire_arm_q_ = default_arm_pos_;
      desire_arm_v_ = Eigen::VectorXd::Zero(joint_arm_num_);
      is_interpolating_ = false;
      auto_swing_state_ = AutoSwingState::kSwing;
      ROS_INFO("[ArmController] Switching to Auto Swing: already at default position");
    }
    
    ROS_INFO("[ArmController] Switching to Auto Swing: auto swing arm");
    // 自动摆臂。外部控制→自动摆臂: 保持外部控制外壳，回家完成后翻回
    if (arm_control_mode_ == ControlMode::kExternal) {
      is_returning_from_external_ = true;
    } else {
      arm_control_mode_ = ControlMode::kAutoSwing;
    }
     arm_vr_enabled_ = false;
  }
  else if (target_mode == 2)
  {
    // 1. 设置插值起点：期望位置同步为当前实际位置
    desire_arm_q_ = current_arm_pos_;
    desire_arm_v_ = current_arm_vel_;
    
    // 2. 初始化目标：在收到外部信号前，目标保持在当前位置，防止误触发结束
    external_target_q_ = current_arm_pos_;
    external_target_v_ = current_arm_vel_;
    
    // 重置滤波器状态到当前位置
    if (arm_filter_initialized_)
    {
      arm_joint_pos_filter_.reset(current_arm_pos_);
      arm_joint_vel_filter_.reset(current_arm_vel_);
    }
    resetExternalVelocityDiff();

    // 尚未收到外部目标
    external_target_received_ = false;
    
    // 3. 标记开始插值阶段
    is_interpolating_ = true;
    
    ROS_INFO("[ArmController] Switching to External Control: Waiting for external target to start interpolation (v_interp=%.2f)",
             mode_interpolation_velocity_);

    arm_control_mode_ = ControlMode::kExternal;
    arm_vr_enabled_ = true;
    is_returning_from_external_ = false;
    auto_swing_state_ = AutoSwingState::kSwing;  // 子状态仅在自动摆臂时有意义，离开时复位
  }
  else if (target_mode == 3)
  {
    // 关节空间阻抗控制：从当前位置平滑过渡到阻抗目标，之后以低增益柔顺
    if (!impedance_enabled_)
    {
      ROS_WARN("[ArmController] Impedance control (mode 3) not configured, falling back to Locked");
      applyModeChange(0);
      return;
    }

    desire_arm_q_ = current_arm_pos_;
    desire_arm_v_ = current_arm_vel_;

    // 目标来源：
    //  0 = 弹簧回默认位（default_arm_pos_，交互后自动回位）
    //  1 = 保持进入时位姿（交互后停在受力位置）
    if (impedance_target_source_ == 1)
    {
      impedance_hold_pos_ = current_arm_pos_;
      impedance_hold_pos_set_ = true;
      // 平滑过渡到进入时的位姿（即当前位置，几乎无位移）
      resetInterpolationState(ros::Time::now(), current_arm_pos_, impedance_hold_pos_);
    }
    else
    {
      impedance_hold_pos_set_ = false;
      // 平滑过渡到默认位
      resetInterpolationState(ros::Time::now(), current_arm_pos_, default_arm_pos_);
    }
    is_interpolating_ = true;

    // 进入阻抗：切换为低增益（柔顺）
    if (arm_torque_controller_)
    {
      arm_torque_controller_->setGains(impedance_kp_, impedance_kd_);
      ROS_INFO("[ArmController] Impedance mode: switched to compliant gains (kp=%.2f~%.2f, kd=%.2f~%.2f)",
               impedance_kp_.minCoeff(), impedance_kp_.maxCoeff(),
               impedance_kd_.minCoeff(), impedance_kd_.maxCoeff());
    }

    ROS_INFO("[ArmController] Switching to Joint Impedance (mode 3): target_source=%d, use_cst=%d",
             impedance_target_source_, impedance_use_cst_ ? 1 : 0);
    arm_control_mode_ = ControlMode::kImpedance;
    arm_vr_enabled_ = false;
    is_returning_from_external_ = false;
    auto_swing_state_ = AutoSwingState::kSwing;
  }
  else if (target_mode == 4)
  {
    // 笛卡尔空间阻抗控制：锁定末端位姿为柔顺参考点
    if (!cartesian_impedance_enabled_ || !arm_torque_controller_)
    {
      ROS_WARN("[ArmController] Cartesian impedance control (mode 4) not configured, falling back to Locked");
      applyModeChange(0);
      return;
    }

    desire_arm_q_ = current_arm_pos_;
    desire_arm_v_ = current_arm_vel_;

    // 锁定当前末端位姿为柔顺参考
    arm_torque_controller_->setMeasuredState(current_arm_pos_, current_arm_vel_);
    cartesian_hold_pose_ = arm_torque_controller_->getEndEffectorPose(cartesian_ee_frame_mode_ == 0);
    cartesian_hold_pose_set_ = true;
    // 记录进入时的关节位姿（模式4切出时的回位门禁目标）
    cartesian_hold_joint_pos_ = current_arm_pos_;

    // 平滑过渡到当前位姿（几乎无位移，防止跳变）
    resetInterpolationState(ros::Time::now(), current_arm_pos_, current_arm_pos_);
    is_interpolating_ = true;

    // 进入笛卡尔阻抗：切换为低增益（柔顺）
    if (arm_torque_controller_)
    {
      arm_torque_controller_->setGains(impedance_kp_, impedance_kd_);
    }

    ROS_INFO("[ArmController] Switching to Cartesian Impedance (mode 4): kx=%.1f N/m, dx=%.1f N·s/m, ee_frame=%d, force_est_gain=%.1f",
             cartesian_kx_, cartesian_dx_, cartesian_ee_frame_mode_, cartesian_force_est_gain_);
    arm_control_mode_ = ControlMode::kCartesianImpedance;
    arm_vr_enabled_ = false;
    is_returning_from_external_ = false;
    auto_swing_state_ = AutoSwingState::kSwing;
  }

  // 模式变化回调（供控制器同步 RUIWO 驱动级增益等外部状态）
  if (mode_change_callback_)
  {
    mode_change_callback_(old_mode, target_mode);
  }
}

void ArmController::executePendingModeChange()
{
  if (!pending_arm_mode_.has_value())
  {
    return;
  }
  
  int target_mode = static_cast<int>(pending_arm_mode_.value());
  pending_arm_mode_.reset();

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
    if (pending_arm_mode_.has_value())
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
  if (external_control_pause_requested_.load(std::memory_order_acquire))
  {
    return;
  }

  std::uint64_t owner_epoch = 0;
  {
    std::lock_guard<std::mutex> input_owner_lock(external_input_owner_mutex_);
    if (offline_action_input_active_.load(std::memory_order_acquire))
    {
      return;
    }
    owner_epoch = external_input_owner_epoch_.load(std::memory_order_acquire);
  }

  // The routing callback can take RLControllerManager's mutex. Do not hold the
  // owner mutex across it: controller switching reaches this object in the
  // opposite direction. The epoch check below drops this message if ownership
  // changed while routing was evaluated (including action->generic ABA).
  std::function<bool()> external_command_buffer_callback;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    external_command_buffer_callback = external_command_buffer_callback_;
  }
  const bool should_buffer =
      external_command_buffer_callback && external_command_buffer_callback();

  std::lock_guard<std::mutex> input_owner_lock(external_input_owner_mutex_);
  if (offline_action_input_active_.load(std::memory_order_acquire) ||
      external_input_owner_epoch_.load(std::memory_order_acquire) != owner_epoch ||
      external_control_pause_requested_.load(std::memory_order_acquire))
  {
    return;
  }
  if (should_buffer)
  {
    std::lock_guard<std::mutex> lock(external_target_mutex_);
    if (external_control_pause_requested_.load()) return;
    storeMode2Target(*msg, buffered_mode2_target_q_, buffered_mode2_target_v_);
    buffered_mode2_target_received_ = true;
    ROS_DEBUG_THROTTLE(1.0, "[ArmController] Buffer arm trajectory until manipulation controller is active");
    return;
  }

  if (arm_control_mode_ == ControlMode::kExternal && arm_vr_enabled_)
  {
    std::lock_guard<std::mutex> lock(external_target_mutex_);
    if (external_control_pause_requested_.load() ||
        arm_control_mode_ != ControlMode::kExternal || !arm_vr_enabled_) return;
    storeMode2Target(*msg, raw_external_target_q_, raw_external_target_v_);
    external_target_received_ = true;
  }
}

void ArmController::actionTrajectoryCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  std::lock_guard<std::mutex> input_owner_lock(external_input_owner_mutex_);
  if (!offline_action_input_active_.load(std::memory_order_acquire) ||
      external_control_pause_requested_.load(std::memory_order_acquire))
  {
    return;
  }

  // Offline actions are only meaningful for the already-active manipulation
  // controller. They intentionally never enter the generic auto-switch buffer.
  if (arm_control_mode_ == ControlMode::kExternal && arm_vr_enabled_)
  {
    std::lock_guard<std::mutex> lock(external_target_mutex_);
    if (external_control_pause_requested_.load() ||
        arm_control_mode_ != ControlMode::kExternal || !arm_vr_enabled_) return;
    storeMode2Target(*msg, raw_external_target_q_, raw_external_target_v_);
    external_target_received_ = true;
  }
}

void ArmController::applyBufferedMode2TargetIfReady()
{
  // Only generic input participates in automatic controller-switch buffering.
  std::uint64_t owner_epoch = 0;
  {
    std::lock_guard<std::mutex> input_owner_lock(external_input_owner_mutex_);
    if (offline_action_input_active_.load(std::memory_order_acquire))
    {
      return;
    }
    owner_epoch = external_input_owner_epoch_.load(std::memory_order_acquire);
  }

  {
    std::lock_guard<std::mutex> lock(external_target_mutex_);
    if (!buffered_mode2_target_received_ || external_control_pause_requested_.load())
    {
      return;
    }
  }

  std::function<bool()> external_command_buffer_callback;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    external_command_buffer_callback = external_command_buffer_callback_;
  }
  if (external_command_buffer_callback && external_command_buffer_callback())
  {
    return;
  }
  if (arm_control_mode_ != ControlMode::kExternal || !arm_vr_enabled_)
  {
    return;
  }

  std::lock_guard<std::mutex> input_owner_lock(external_input_owner_mutex_);
  if (offline_action_input_active_.load(std::memory_order_acquire) ||
      external_input_owner_epoch_.load(std::memory_order_acquire) != owner_epoch)
  {
    return;
  }

  std::lock_guard<std::mutex> lock(external_target_mutex_);
  if (!buffered_mode2_target_received_ || external_control_pause_requested_.load() ||
      arm_control_mode_ != ControlMode::kExternal || !arm_vr_enabled_)
  {
    return;
  }
  raw_external_target_q_ = buffered_mode2_target_q_;
  raw_external_target_v_ = buffered_mode2_target_v_;
  external_target_received_ = true;
  buffered_mode2_target_received_ = false;
  ROS_INFO("[ArmController] Applied buffered arm trajectory after manipulation controller became active");
}

void ArmController::storeMode2Target(const sensor_msgs::JointState& msg,
                                     Eigen::VectorXd& target_q,
                                     Eigen::VectorXd& target_v) const
{
  if (target_q.size() != static_cast<int>(joint_arm_num_))
  {
    target_q = Eigen::VectorXd::Zero(joint_arm_num_);
  }
  if (target_v.size() != static_cast<int>(joint_arm_num_))
  {
    target_v = Eigen::VectorXd::Zero(joint_arm_num_);
  }

  const size_t target_size = std::min({msg.name.size(), msg.position.size(), joint_arm_num_});
  for (size_t i = 0; i < target_size; ++i)
  {
    target_q(i) = msg.position[i] * M_PI / 180.0;
    if (msg.velocity.size() == joint_arm_num_)
    {
      target_v(i) = msg.velocity[i] * M_PI / 180.0;
    }
    else
    {
      target_v(i) = 0.0;
    }
  }
}

void ArmController::updateLockedPose(double dt)
{
  // 锁定：固定到固定位置
  if (is_interpolating_)
  {
    // 如果正在插值（切换瞬间），执行平滑插值回到固定位置
    applySmoothInterpolation(ros::Time::now(), locked_fixed_pos_, Eigen::VectorXd::Zero(joint_arm_num_));
  }
  else if (locked_pos_set_)
  {
    desire_arm_q_ = locked_fixed_pos_;
    desire_arm_v_.setZero();
  }
}

void ArmController::updateAutoSwing(const ros::Time& time, double dt, int cmd_stance)
{
  // 自动摆臂（RL 持臂）内部子状态机：
  //   kSwing  --站立指令-->    kHoming   发起回家：生成当前位置→默认位的三次平滑曲线
  //   kHoming --跑完且站立-->  kHolding  到位后继续按住默认位（站立不动）
  //   kHoming --跑完且行走-->  kSwing    到位后交还 RL 策略（行走中解锁场景）
  //   kHolding --起步走-->     kSwing    交还 RL 策略摆臂
  // 注：kHolding 在站立下没有回到 kHoming 的边 —— 到位后不会再次发起回家，
  //     循环在结构上不存在。
  switch (auto_swing_state_)
  {
    case AutoSwingState::kSwing:
      if (cmd_stance == 1)
      {
        // 站立：发起回家
        desire_arm_q_ = current_arm_pos_;
        desire_arm_v_ = current_arm_vel_;
        resetInterpolationState(time, current_arm_pos_, default_arm_pos_);
        is_interpolating_ = true;
        auto_swing_state_ = AutoSwingState::kHoming;
      }
      break;

    case AutoSwingState::kHoming:
      // 回家中：跑曲线（站立或行走都跑完，不在中途打断）
      applySmoothInterpolation(time, default_arm_pos_, Eigen::VectorXd::Zero(joint_arm_num_));
      if (!is_interpolating_)
      {
        // 曲线跑完：站立按住 / 行走交还
        if (cmd_stance == 1)
          auto_swing_state_ = AutoSwingState::kHolding;
        else
          auto_swing_state_ = AutoSwingState::kSwing;
      }
      break;

    case AutoSwingState::kHolding:
      // 按住默认位：tau>=1 后 applySmoothInterpolation 每帧把期望钉在默认位
      applySmoothInterpolation(time, default_arm_pos_, Eigen::VectorXd::Zero(joint_arm_num_));
      if (cmd_stance == 0)
        auto_swing_state_ = AutoSwingState::kSwing;
      break;
  }
}

void ArmController::applyExternalControlPauseState()
{
  const bool switch_return_active =
      default_pose_return_for_switch_state_.load(std::memory_order_acquire) !=
      DefaultPoseReturnState::kIdle;
  if (switch_return_active)
  {
    // RT+Y/Pico streaming pause is subordinate to the safety return. VMP may
    // re-publish the pause request every tick while the outer mode is still 2.
    external_control_pause_requested_.store(false);
  }
  const bool pause_requested = switch_return_active
                                   ? false
                                   : external_control_pause_requested_.load();

  // The pause applies only to external control. If another mode is selected,
  // keep the request latched so a later transition to mode 2 starts held.
  if (arm_control_mode_ != ControlMode::kExternal)
  {
    external_control_pause_active_ = false;
    external_pause_resume_reacquiring_ = false;
    external_pause_source_valid_ = false;
    return;
  }

  if (pause_requested && !external_control_pause_active_)
  {
    // cmd_arm_pos_ is the exact position emitted by the previous update. On
    // the first update, use the freshly captured measured state instead of
    // the constructor's zero-initialized command.
    if (has_emitted_arm_command_ &&
        cmd_arm_pos_.size() == static_cast<int>(joint_arm_num_) &&
        cmd_arm_pos_.allFinite())
    {
      external_pause_hold_q_ = cmd_arm_pos_;
    }
    else
    {
      external_pause_hold_q_ = current_arm_pos_;
    }

    desire_arm_q_ = external_pause_hold_q_;
    desire_arm_v_.setZero();
    {
      std::lock_guard<std::mutex> lock(external_target_mutex_);
      external_pause_source_valid_ =
          external_target_received_ &&
          raw_external_target_q_.size() == static_cast<int>(joint_arm_num_) &&
          raw_external_target_q_.allFinite();
      if (external_pause_source_valid_)
      {
        external_pause_source_q_ = raw_external_target_q_;
      }
      raw_external_target_q_ = external_pause_hold_q_;
      raw_external_target_v_.setZero();
      external_target_q_ = external_pause_hold_q_;
      external_target_v_.setZero();
      external_target_received_ = false;
      buffered_mode2_target_received_ = false;
      last_external_input_time_valid_ = false;
    }
    if (arm_filter_initialized_)
    {
      arm_joint_pos_filter_.reset(external_pause_hold_q_);
      arm_joint_vel_filter_.reset(Eigen::VectorXd::Zero(joint_arm_num_));
    }
    resetExternalVelocityDiff();
    is_interpolating_ = true;
    external_pause_resume_reacquiring_ = false;
    external_control_pause_active_ = true;
    ROS_INFO("[ArmController] External control paused: holding last arm command");
    return;
  }

  if (!pause_requested && external_control_pause_active_)
  {
    // Clear every pre-pause target while serialized with the ROS callbacks.
    // A callback that arrives after this critical section is the first target
    // eligible for smooth post-resume reacquisition.
    {
      std::lock_guard<std::mutex> lock(external_target_mutex_);
      raw_external_target_q_ = external_pause_hold_q_;
      raw_external_target_v_.setZero();
      external_target_q_ = external_pause_hold_q_;
      external_target_v_.setZero();
      external_target_received_ = false;
      buffered_mode2_target_received_ = false;
      last_external_input_time_valid_ = false;
    }
    desire_arm_q_ = external_pause_hold_q_;
    desire_arm_v_.setZero();
    if (arm_filter_initialized_)
    {
      arm_joint_pos_filter_.reset(external_pause_hold_q_);
      arm_joint_vel_filter_.reset(Eigen::VectorXd::Zero(joint_arm_num_));
    }
    resetExternalVelocityDiff();
    is_interpolating_ = true;
    external_pause_resume_reacquiring_ = true;
    external_control_pause_active_ = false;
    ROS_INFO("[ArmController] External control resumed: waiting for an updated arm target");
  }
}

bool ArmController::applyStrictRateLimitedInterpolation(double dt,
                                                        const Eigen::VectorXd& target_pos,
                                                        double max_velocity)
{
  if (target_pos.size() != desire_arm_q_.size())
  {
    desire_arm_v_.setZero();
    return false;
  }

  const double max_step = std::max(0.0, max_velocity) * std::max(0.0, dt);
  bool completed = true;
  for (int i = 0; i < target_pos.size(); ++i)
  {
    const double error = target_pos(i) - desire_arm_q_(i);
    if (std::abs(error) > max_step)
    {
      if (max_step > 0.0)
      {
        desire_arm_q_(i) += std::copysign(max_step, error);
      }
      completed = false;
    }
    else
    {
      desire_arm_q_(i) = target_pos(i);
    }
  }
  desire_arm_v_.setZero();
  is_interpolating_ = !completed;
  return completed;
}

void ArmController::resetExternalVelocityDiff()
{
  external_velocity_diff_valid_ = false;
  if (filtered_pos_for_velocity_.size() != static_cast<int>(joint_arm_num_))
  {
    filtered_pos_for_velocity_.resize(joint_arm_num_);
  }
  filtered_pos_for_velocity_.setZero();
}

void ArmController::updateExternalControl(double dt)
{
  Eigen::VectorXd raw_target_q;
  bool target_received = false;
  std::uint64_t owner_epoch = 0;
  {
    // 与输入回调相同的加锁顺序：先 owner，再 target。这样读到的 epoch 和目标属于同一次切换。
    std::lock_guard<std::mutex> input_owner_lock(external_input_owner_mutex_);
    std::lock_guard<std::mutex> target_lock(external_target_mutex_);
    owner_epoch = external_input_owner_epoch_.load(std::memory_order_acquire);
    target_received = external_target_received_;
    if (target_received)
    {
      raw_target_q = raw_external_target_q_;
    }
  }
  // 输入源切换由回调递增 epoch。这里等到新源的首个目标再作废差分，避免用上一个源的位置。
  if (target_received && owner_epoch != external_velocity_diff_owner_epoch_)
  {
    resetExternalVelocityDiff();
    if (arm_filter_initialized_)
    {
      arm_joint_vel_filter_.reset(Eigen::VectorXd::Zero(joint_arm_num_));
    }
    external_velocity_diff_owner_epoch_ = owner_epoch;
  }

  // 外部控制模式
  // 先滤波位置，速度只用滤波后位置的差分，不再采用 /kuavo_arm_traj 的原始速度。
  // 原始速度是未滤波位置的帧间差分，dt 抖动会把小的位置台阶放大成速度尖峰。
  // 差分历史是控制器状态。滤波器重置后的第一拍输出 0，避免跨会话相减。

  // 1. 始终更新滤波后的目标（供接管阶段使用）
  if (target_received)
  {
    // 1.1 先对位置进行滤波（和humanoidController一致）
    external_target_q_ = arm_joint_pos_filter_.update(raw_target_q);

    // 1.2 速度 = 滤波后位置的差分，再低通。与下发的 joint_q 一致。
    Eigen::VectorXd computed_vel = Eigen::VectorXd::Zero(joint_arm_num_);
    if (!external_velocity_diff_valid_ ||
        filtered_pos_for_velocity_.size() != joint_arm_num_ ||
        dt <= 0.0)
    {
      external_velocity_diff_valid_ = true;
    }
    else
    {
      computed_vel = (external_target_q_ - filtered_pos_for_velocity_) / dt;
      computed_vel = arm_joint_vel_filter_.update(computed_vel);
    }

    external_target_v_ = computed_vel;
    filtered_pos_for_velocity_ = external_target_q_;
  }

  // 2. 核心插值/接管逻辑重构
  if (is_interpolating_)
  {
    // 插值阶段：
    // 如果还没收到第一个目标，我们保持不动，并维持 is_interpolating = true
    if (!target_received)
    {
      desire_arm_v_.setZero();
      // 不调用 applyRateLimitedInterpolation，防止其因为误差为0而误判结束
    }
    else
    {
      bool interpolation_finished = false;
      if (external_pause_resume_reacquiring_)
      {
        // Resume-specific path: clamp every joint on every tick. Unlike the
        // normal path, this cannot snap across arm_tracking_error_threshold_.
        // Xsens repeats its frozen source frame while paused. Compare against
        // that last accepted source target (not the filtered command) and
        // remain held until the source content actually changes.
        if (external_pause_source_valid_ &&
            (raw_target_q - external_pause_source_q_)
                    .lpNorm<Eigen::Infinity>() <= 1e-6)
        {
          desire_arm_q_ = external_pause_hold_q_;
          desire_arm_v_.setZero();
          is_interpolating_ = true;
        }
        else
        {
          external_pause_source_valid_ = false;
          interpolation_finished = applyStrictRateLimitedInterpolation(
              dt, raw_target_q, mode_interpolation_velocity_);
        }
        if (interpolation_finished)
        {
          external_pause_resume_reacquiring_ = false;
        }
      }
      else
      {
        // 收到目标后，向"原始目标"(raw_external_target_q_) 靠拢
        // 关键：不使用滤波后的目标进行插值，以确保产生足够的误差触发限速
        applyRateLimitedInterpolation(dt, raw_target_q, Eigen::VectorXd::Zero(joint_arm_num_), mode_interpolation_velocity_);
        interpolation_finished = !is_interpolating_;
      }

      if (interpolation_finished) {
          // 插值结束瞬间，重置滤波器到当前位置，确保接管瞬间无跳变。
          // 本拍速度已经用旧历史算过，下一拍差分重新从 0 开始。
          arm_joint_pos_filter_.reset(desire_arm_q_);
          arm_joint_vel_filter_.reset(desire_arm_v_);
          resetExternalVelocityDiff();
          ROS_INFO("[ArmController] Interpolation finished, switching to filtered tracking (v_max=%.2f)", arm_max_tracking_velocity_);
      }
    }
  }
  else
  {
    // 接管阶段：正常跟踪滤波后的目标，使用最大跟踪速度
    applyRateLimitedInterpolation(dt, external_target_q_, external_target_v_, arm_max_tracking_velocity_);
  }
}

void ArmController::updateImpedanceControl(double dt)
{
  // 关节空间阻抗控制（模式3）：
  //  1. 目标位置 = 默认位（弹簧回弹）或进入时位姿（保持）
  //  2. 平滑过渡（进入时插值，之后直接跟踪目标）
  //  3. 力矩由 ArmTorqueController 计算：τ = G + Kp·Δq + Kd·Δv（增益已在进入模式时切换为低阻抗增益）

  Eigen::VectorXd target_pos;
  if (impedance_target_source_ == 1 && impedance_hold_pos_set_)
  {
    target_pos = impedance_hold_pos_;
  }
  else
  {
    target_pos = default_arm_pos_;
  }

  // 锁手腕：阻抗控制不控制手腕关节，手腕目标强制为默认位（不参与柔顺，由 fillJointCmdMessage 以 CSP+高增益锁定）
  if (impedance_lock_wrist_ && default_arm_pos_.size() == static_cast<int>(joint_arm_num_))
  {
    for (int i = 0; i < static_cast<int>(joint_arm_num_); ++i)
    {
      if (isWristJoint(i))
      {
        target_pos(i) = default_arm_pos_(i);
      }
    }
  }

  // 肘关节随肩部 pitch 负向随动（仅模式3）：
  //   肘目标 = 肘参考位 + k · min(0, 肩pitch位移)，肩 pitch 正向偏移不动；
  //   位移量 = 肩pitch测量值 − 肩pitch参考位（与阻抗目标同一参考），即被压向负向多少。
  //   关节模式由 fillJointCmdMessage 保持 CSP(2)，不参与阻抗柔顺。
  if (elbow_follow_enabled_ &&
      current_arm_pos_.size() == static_cast<int>(joint_arm_num_) &&
      target_pos.size() == static_cast<int>(joint_arm_num_))
  {
    for (int i = 0; i < static_cast<int>(joint_arm_num_); ++i)
    {
      if (!isElbowJoint(i))
      {
        continue;
      }
      const int pitch_idx = i - 3;  // 同侧肩pitch：l4->l1(0)、r4->r1(7)
      if (pitch_idx < 0 || pitch_idx >= static_cast<int>(joint_arm_num_) || !isShoulderPitchJoint(pitch_idx))
      {
        continue;
      }
      const double pitch_offset = current_arm_pos_(pitch_idx) - target_pos(pitch_idx);
      if (pitch_offset >= 0.0)
      {
        continue;  // 肩 pitch 正向：肘不动
      }
      double follow = elbow_follow_ratio_ * pitch_offset;  // <= 0
      if (elbow_follow_max_offset_ > 0.0)
      {
        follow = std::max(follow, -elbow_follow_max_offset_);
      }
      target_pos(i) += follow;
    }
  }

  if (is_interpolating_)
  {
    // 进入阻抗模式瞬间的平滑过渡：从当前位置到目标位置
    applySmoothInterpolation(ros::Time::now(), target_pos, Eigen::VectorXd::Zero(joint_arm_num_));
  }
  else
  {
    desire_arm_q_ = target_pos;
    desire_arm_v_.setZero();
  }

  // 安全检查：关节速度超限 → 自动回退锁定（防止柔顺失控）
  if (impedance_max_joint_vel_ > 0.0 && current_arm_vel_.cwiseAbs().maxCoeff() > impedance_max_joint_vel_)
  {
    ROS_WARN_THROTTLE(1.0, "[ArmController] Impedance: arm velocity %.3f rad/s exceeds limit %.3f, falling back to Locked",
                      current_arm_vel_.cwiseAbs().maxCoeff(), impedance_max_joint_vel_);
    applyModeChange(0);
  }
}

void ArmController::updateCartesianImpedance(double dt)
{
  // 笛卡尔空间阻抗控制（模式4）：
  //  1. 参考位姿 = 进入模式时的末端位姿（锁定）
  //  2. 力矩 = Jᵀ·[Kx·Δx + Dx·(v_d − v_ee)] + G（由 ArmTorqueController 计算）
  //  3. 可选：外力估计（动量观测器）→ 力-位导纳偏移，实现外力跟随
  if (!arm_torque_controller_ || !cartesian_hold_pose_set_)
  {
    applyModeChange(0);
    return;
  }

  // 平滑过渡阶段：先让关节平滑进入（目标=当前位），完成后再进行笛卡尔阻抗
  if (is_interpolating_)
  {
    applySmoothInterpolation(ros::Time::now(), current_arm_pos_, Eigen::VectorXd::Zero(joint_arm_num_));
    // 插值结束后，将 desire_arm_q_/v_ 保持，后续力矩由笛卡尔阻抗计算
    if (!is_interpolating_)
    {
      ROS_INFO("[ArmController] Cartesian impedance: smooth entry complete, activating impedance");
    }
    return;
  }

  const bool use_left = (cartesian_ee_frame_mode_ == 0);

  // 实测力矩（用于外力估计）：从完整关节数组提取手臂段
  Eigen::VectorXd tau_meas_arm = Eigen::VectorXd::Zero(joint_arm_num_);
  if (measured_joint_torque_valid_ &&
      measured_joint_torque_.size() >= static_cast<int>(arm_start_idx_ + joint_arm_num_))
  {
    tau_meas_arm = measured_joint_torque_.segment(arm_start_idx_, joint_arm_num_);
  }

  // 目标位置：进入时的末端位姿
  const Eigen::Vector3d x_desired = cartesian_hold_pose_.translation();
  const Eigen::Vector3d v_desired = Eigen::Vector3d::Zero();

  // 外力估计 → 导纳：x_ref += F_ext / kx（让手臂"跟随"外力）
  Eigen::Vector3d x_ref = x_desired;
  if (cartesian_force_est_gain_ > 0.0 && measured_joint_torque_valid_)
  {
    arm_torque_controller_->setMeasuredState(current_arm_pos_, current_arm_vel_);
    const Eigen::Vector3d f_ext = arm_torque_controller_->estimateEndEffectorForce(tau_meas_arm, use_left);
    const double f_norm = f_ext.norm();
    if (f_norm > 0.5)  // 阈值：忽略噪声
    {
      x_ref += (f_ext / cartesian_kx_) * std::min(1.0, cartesian_force_est_gain_ * dt);
      ROS_DEBUG_THROTTLE(1.0, "[ArmController] Cartesian impedance: F_ext=%.2f N, x_ref offset=%.3f m", f_norm, (f_ext / cartesian_kx_).norm());
    }
  }

  // 计算笛卡尔阻抗力矩（内部含重力补偿）
  arm_torque_controller_->setMeasuredState(current_arm_pos_, current_arm_vel_);
  Eigen::VectorXd tau_imp = arm_torque_controller_->computeCartesianImpedanceTorque(
      x_ref, v_desired, use_left, cartesian_kx_, cartesian_dx_);

  // 力矩钳位到 torque_limits_
  if (torque_limits_.size() == static_cast<int>(joint_arm_num_))
  {
    for (int i = 0; i < joint_arm_num_; ++i)
    {
      if (torque_limits_(i) > 0.0)
      {
        tau_imp(i) = std::max(-torque_limits_(i), std::min(torque_limits_(i), tau_imp(i)));
      }
    }
  }

  // 写入期望状态：关节位置保持当前（力矩由 fillJointCmdMessage 中的 tau 施加）
  // 注意：笛卡尔阻抗通过 CST 力矩模式施加，joint_q/joint_v 仅是位置占位
  desire_arm_q_ = current_arm_pos_;
  desire_arm_v_.setZero();

  // 将计算出的力矩暂存，供 fillJointCmdMessage 使用
  // （fillJointCmdMessage 内部用 ArmTorqueController::computeTorque 会重新算 PD 力矩，
  //   这里直接覆盖：先保存到临时成员）
  cartesian_tau_override_ = tau_imp;
  cartesian_tau_override_valid_ = true;

  // 安全检查：关节速度超限 → 自动回退锁定
  if (impedance_max_joint_vel_ > 0.0 && current_arm_vel_.cwiseAbs().maxCoeff() > impedance_max_joint_vel_)
  {
    ROS_WARN_THROTTLE(1.0, "[ArmController] Cartesian impedance: arm velocity %.3f rad/s exceeds limit %.3f, falling back to Locked",
                      current_arm_vel_.cwiseAbs().maxCoeff(), impedance_max_joint_vel_);
    applyModeChange(0);
  }
}

void ArmController::handleAutoModeSwitch(int cmd_stance)
{
  // 自动切换手臂控制模式：行走→自动摆臂，站立→锁定

  if (cmd_stance == 0 && prev_cmd_stance_ != 0 && arm_control_mode_ != ControlMode::kAutoSwing)
  {
    // 从站立切换到行走 → 自动摆臂
    if (!is_interpolating_)
    {
      applyModeChange(1);
      ROS_INFO("[ArmController] Detected walking state, switching to Auto Swing");
    }
  }

  if (cmd_stance == 1 && prev_cmd_stance_ != 1 && arm_control_mode_ != ControlMode::kLocked)
  {
    // 从行走切换到站立 → 锁定当前姿态
    if (!is_interpolating_)
    {
      applyModeChange(0);
      ROS_INFO("[ArmController] Detected standing state, switching to Locked");
    }
  }

  prev_cmd_stance_ = cmd_stance;
}

} // namespace humanoid_controller
