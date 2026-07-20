#include "humanoid_controllers/SitControlManager.h"
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>
#include <kuavo_common/common/seat_config.h>  // SeatConfig = nlohmann::json
#include <kuavo_msgs/jointCmd.h>
#include <ros/ros.h>
#include <sstream>
#include <algorithm>
#include <cmath>

namespace humanoid_controller {

namespace {

struct SeatBootConfig {
  double prep_speed_deg{0.0};
  /** 第二阶段：手臂中间姿态 → sit_joint_pos 手臂的插值速度（度/秒） */
  double prep_settle_speed_deg{0.0};
  std::vector<double> leg_joint_kp;
  std::vector<double> leg_joint_kd;
  bool valid{false};
};

/** 将 calcSitState 向量写入 hardware 关节角（度）。硬件顺序：腿 | 腰 | 臂 | 头（见 hardware_plant）。 */
void fillHwPrepJointPosDegFromSitMpc(std::vector<double>& joint_pos_deg, const Eigen::VectorXd& sit_state,
                                     int waist_joints_num, int leg_joint_num, int arm_joints_num) {
  const int kStateLegOffset = static_cast<int>(SitControlManager::kMpcLegStateStart);
  const int kStateWaistOffset = static_cast<int>(SitControlManager::kMpcWaistStateIdx);
  const int kStateArmOffset = kStateWaistOffset + waist_joints_num;

  std::fill(joint_pos_deg.begin(), joint_pos_deg.end(), 0.0);

  for (int i = 0; i < leg_joint_num && i < static_cast<int>(SitControlManager::kLegDof); ++i) {
    const int src = kStateLegOffset + i;
    if (src < sit_state.size()) {
      joint_pos_deg[static_cast<size_t>(i)] = sit_state(src) * 180.0 / M_PI;
    }
  }
  if (waist_joints_num > 0 && kStateWaistOffset < sit_state.size()) {
    joint_pos_deg[static_cast<size_t>(leg_joint_num)] = sit_state(kStateWaistOffset) * 180.0 / M_PI;
  }
  const size_t arm_hw_base = static_cast<size_t>(leg_joint_num + waist_joints_num);
  for (int i = 0; i < arm_joints_num; ++i) {
    const int src = kStateArmOffset + i;
    if (src < sit_state.size() && arm_hw_base + static_cast<size_t>(i) < joint_pos_deg.size()) {
      joint_pos_deg[arm_hw_base + static_cast<size_t>(i)] = sit_state(src) * 180.0 / M_PI;
    }
  }
}

void publishHardwarePrepPlan(const SeatBootConfig& boot, const std::vector<double>& prep_sit_deg,
                             const std::vector<double>* prep_offset_deg,
                             const std::vector<double>* prep_reverse_leg_first_deg = nullptr) {
  ros::param::set("/hardware_prep_joint_pos_deg", prep_sit_deg);
  if (prep_offset_deg != nullptr) {
    ros::param::set("/hardware_prep_two_phase", true);
    ros::param::set("/hardware_prep_joint_pos_offset_deg", *prep_offset_deg);
    if (prep_reverse_leg_first_deg != nullptr) {
      ros::param::set("/hardware_prep_reverse_leg_first", true);
      ros::param::set("/hardware_prep_joint_pos_leg_first_deg", *prep_reverse_leg_first_deg);
    } else {
      ros::param::del("/hardware_prep_reverse_leg_first");
      ros::param::del("/hardware_prep_joint_pos_leg_first_deg");
    }
    const std::vector<double> moves = {boot.prep_speed_deg, boot.prep_settle_speed_deg};
    ros::param::set("/hardware_prep_moves", moves);
    if (prep_reverse_leg_first_deg != nullptr) {
      ROS_INFO("[SitControlManager] hardware prep: offset (%.1f deg/s) -> reverse legs (%.1f deg/s) -> reverse arms (%.1f deg/s).",
               boot.prep_speed_deg, boot.prep_settle_speed_deg, boot.prep_settle_speed_deg);
    } else {
      ROS_INFO("[SitControlManager] hardware prep: two-phase sit+offset (%.1f deg/s) -> sit_joint_pos (%.1f deg/s).",
               boot.prep_speed_deg, boot.prep_settle_speed_deg);
    }
  } else {
    ros::param::del("/hardware_prep_two_phase");
    ros::param::del("/hardware_prep_joint_pos_offset_deg");
    ros::param::del("/hardware_prep_reverse_leg_first");
    ros::param::del("/hardware_prep_joint_pos_leg_first_deg");
    const std::vector<double> moves = {0.0, boot.prep_speed_deg};
    ros::param::set("/hardware_prep_moves", moves);
  }
  if (!boot.leg_joint_kp.empty()) {
    ros::param::set("/hardware_prep_ec_joint_kp", boot.leg_joint_kp);
    ros::param::set("/hardware_prep_ec_joint_kd", boot.leg_joint_kd);
    ROS_INFO("[SitControlManager] hardware prep: sit_boot leg EC kp/kd (%zu dof, waist/arm EC unchanged).",
             boot.leg_joint_kp.size());
  } else if (prep_offset_deg == nullptr) {
    ROS_INFO("[SitControlManager] hardware prep: single jointMoveTo (%.1f deg/s).", boot.prep_speed_deg);
  }
}

void clearHardwarePrepParams() {
  ros::param::del("/hardware_prep_joint_pos_deg");
  ros::param::del("/hardware_prep_joint_pos_offset_deg");
  ros::param::del("/hardware_prep_joint_pos_leg_first_deg");
  ros::param::del("/hardware_prep_two_phase");
  ros::param::del("/hardware_prep_reverse_leg_first");
  ros::param::del("/hardware_prep_moves");
  ros::param::del("/hardware_prep_ec_joint_kp");
  ros::param::del("/hardware_prep_ec_joint_kd");
}

}  // namespace

SitControlManager::SitControlManager() = default;

void SitControlManager::syncBootFlags(bool use_sit_init_boot) {
  use_sit_init_boot_ = use_sit_init_boot;
  has_sit_boot_leg_gains_ = false;
  sit_boot_leg_kp_.clear();
  sit_boot_leg_kd_.clear();
  if (use_sit_init_boot_)
    loadSitBootLegGainsFromParam();
}

void SitControlManager::loadSitBootLegGainsFromParam() {
  std::vector<double> kp;
  std::vector<double> kd;
  if (!ros::param::get("/hardware_prep_ec_joint_kp", kp) || !ros::param::get("/hardware_prep_ec_joint_kd", kd) ||
      kp.size() != kLegDof || kd.size() != kLegDof) {
    return;
  }
  sit_boot_leg_kp_ = std::move(kp);
  sit_boot_leg_kd_ = std::move(kd);
  has_sit_boot_leg_gains_ = true;
  ROS_INFO("[SitControlManager] preUpdate sit_boot leg gains loaded (%zu dof).", sit_boot_leg_kp_.size());
}

bool SitControlManager::sitBootLegGain(size_t leg_index, double& kp, double& kd) const {
  if (!has_sit_boot_leg_gains_ || leg_index >= sit_boot_leg_kp_.size())
    return false;
  kp = sit_boot_leg_kp_[leg_index];
  kd = sit_boot_leg_kd_[leg_index];
  return true;
}

bool SitControlManager::configureLaunchBoot(bool use_sit_init, const RobotVersion& robot_version,
                                            const Eigen::VectorXd& sit_mpc_state, int hw_waist_joints,
                                            int hw_leg_joints, int hw_arm_joints, int hw_joint_count,
                                            const kuavo_common::SeatConfig& seat) {
  const bool enabled = use_sit_init && robotSupportsSeatControl(robot_version);
  if (!enabled) {
    if (use_sit_init && !robotSupportsSeatControl(robot_version)) {
      ROS_WARN("[SitControlManager] use_sit_init ignored: robot %s is not Kuavo5 (major 5).",
               robot_version.to_string().c_str());
    }
    clearHardwarePrepParams();
    return false;
  }
  if (hw_joint_count <= 0) {
    ROS_ERROR("[SitControlManager] invalid hw_joint_count for prep pose.");
    return false;
  }

  SeatBootConfig boot;
  const nlohmann::json* prep_obj =
      seat.contains("hardware_sit_pose_prep") && seat["hardware_sit_pose_prep"].is_object()
          ? &seat["hardware_sit_pose_prep"]
          : nullptr;
  if (!prep_obj || !prep_obj->contains("move_speed_deg_per_s")) {
    ROS_ERROR("[SitControlManager] hardware_sit_pose_prep requires move_speed_deg_per_s.");
    return false;
  }
  boot.prep_speed_deg = (*prep_obj)["move_speed_deg_per_s"].get<double>();
  if (boot.prep_speed_deg <= 0.0) {
    ROS_ERROR("[SitControlManager] hardware_sit_pose_prep move_speed_deg_per_s must be positive.");
    return false;
  }
  boot.prep_settle_speed_deg = boot.prep_speed_deg;
  if (prep_obj->contains("settle_speed_deg_per_s"))
    boot.prep_settle_speed_deg = (*prep_obj)["settle_speed_deg_per_s"].get<double>();
  if (boot.prep_settle_speed_deg <= 0.0) {
    ROS_ERROR("[SitControlManager] hardware_sit_pose_prep settle_speed_deg_per_s must be positive.");
    return false;
  }
  if (prep_obj->contains("leg_joint_kp") && prep_obj->contains("leg_joint_kd")) {
    const auto& kp = (*prep_obj)["leg_joint_kp"];
    const auto& kd = (*prep_obj)["leg_joint_kd"];
    if (kp.is_array() && kd.is_array() && kp.size() == kLegDof && kd.size() == kLegDof) {
      for (size_t i = 0; i < kLegDof; ++i) {
        boot.leg_joint_kp.push_back(kp[i].get<double>());
        boot.leg_joint_kd.push_back(kd[i].get<double>());
      }
      boot.valid = true;
    }
  } else {
    boot.valid = true;
  }

  double leg_offset_rad[kLegDof]{};
  double arm_offset_rad[kArmDof]{};
  if (seat.contains("seat_offset_leg_joint_offset_rad") && seat["seat_offset_leg_joint_offset_rad"].is_array()) {
    const auto& arr = seat["seat_offset_leg_joint_offset_rad"];
    for (size_t i = 0; i < kLegDof && i < arr.size(); ++i)
      leg_offset_rad[i] = arr[i].get<double>();
  }
  if (seat.contains("seat_offset_arm_joint_offset_rad") && seat["seat_offset_arm_joint_offset_rad"].is_array()) {
    const auto& arr = seat["seat_offset_arm_joint_offset_rad"];
    for (size_t i = 0; i < kArmDof && i < arr.size(); ++i)
      arm_offset_rad[i] = arr[i].get<double>();
  }

  std::vector<double> prep_sit_deg(static_cast<size_t>(hw_joint_count), 0.0);
  fillHwPrepJointPosDegFromSitMpc(prep_sit_deg, sit_mpc_state, hw_waist_joints, hw_leg_joints, hw_arm_joints);
  // Phase 1: sit_joint_pos + seat_offset（P3 终点，相当于反向 P3 的起始位置）
  std::vector<double> prep_offset_deg = prep_sit_deg;
  for (int i = 0; i < hw_leg_joints && i < static_cast<int>(kLegDof); ++i)
    prep_offset_deg[static_cast<size_t>(i)] += leg_offset_rad[i] * 180.0 / M_PI;
  const size_t arm_hw_base = static_cast<size_t>(hw_leg_joints + hw_waist_joints);
  for (int i = 0; i < hw_arm_joints && i < static_cast<int>(kArmDof); ++i)
    prep_offset_deg[arm_hw_base + static_cast<size_t>(i)] += arm_offset_rad[i] * 180.0 / M_PI;

  const bool reverse_leg_before_arm =
      prep_obj->contains("reverse_leg_before_arm") && (*prep_obj)["reverse_leg_before_arm"].get<bool>();
  std::vector<double> prep_leg_first_deg;
  const std::vector<double>* prep_leg_first_ptr = nullptr;
  if (reverse_leg_before_arm) {
    // 反向 offset 中间态：腿/腰回到 sit，手臂保持 offset
    prep_leg_first_deg = prep_offset_deg;
    for (int i = 0; i < hw_leg_joints && i < static_cast<int>(kLegDof); ++i)
      prep_leg_first_deg[static_cast<size_t>(i)] = prep_sit_deg[static_cast<size_t>(i)];
    for (int i = 0; i < hw_waist_joints; ++i)
      prep_leg_first_deg[static_cast<size_t>(hw_leg_joints + i)] =
          prep_sit_deg[static_cast<size_t>(hw_leg_joints + i)];
    prep_leg_first_ptr = &prep_leg_first_deg;
  }

  // Phase 2/3: sit_joint_pos（P3 起点；reverse_leg_first 时先腿后手）
  publishHardwarePrepPlan(boot, prep_sit_deg, &prep_offset_deg, prep_leg_first_ptr);
  ROS_INFO(
      "[SitControlManager] use_sit_init hardware prep: phase1=sit+offset, reverse=%s; "
      "leg[0..%d) waist[%d] arm[%d..%d) (deg).",
      reverse_leg_before_arm ? "legs then arms" : "all joints",
      hw_leg_joints, hw_leg_joints, hw_leg_joints + hw_waist_joints,
      hw_leg_joints + hw_waist_joints + hw_arm_joints);
  return true;
}

Eigen::VectorXd SitControlManager::makeBootStartState(const Eigen::VectorXd& sit_state,
                                                      const Eigen::VectorXd& squat_state,
                                                      size_t wbc_state_dim,
                                                      size_t mpc_leg_waist_dim) const {
  Eigen::VectorXd boot = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(wbc_state_dim));
  if (use_sit_init_boot_) {
    const size_t n = std::min(static_cast<size_t>(sit_state.size()), wbc_state_dim);
    if (n > 0) {
      boot.head(static_cast<Eigen::Index>(n)) = sit_state.head(static_cast<Eigen::Index>(n));
    }
    return boot;
  }
  (void)sit_state;
  if (static_cast<size_t>(squat_state.size()) >= mpc_leg_waist_dim) {
    boot.head(static_cast<Eigen::Index>(mpc_leg_waist_dim)) =
        squat_state.head(static_cast<Eigen::Index>(mpc_leg_waist_dim));
  }
  return boot;
}

bool SitControlManager::loadConfig() {
  config_ok_ = false;

  if (!seat_config_) {
    ROS_ERROR("[SitControlManager] SeatConfig not available (non-Kuavo5?); cannot load seat config.");
    return false;
  }
  const auto& seat = *seat_config_;

  // ── seat_offset ──
  if (!seat.contains("seat_offset_leg_joint_offset_rad") || !seat["seat_offset_leg_joint_offset_rad"].is_array() || seat["seat_offset_leg_joint_offset_rad"].size() != kLegDof) {
    ROS_ERROR("[SitControlManager] missing or invalid seat_offset_leg_joint_offset_rad (size %zu).", kLegDof);
    return false;
  }
  for (size_t i = 0; i < kLegDof; ++i)
    leg_offset_rad_[i] = seat["seat_offset_leg_joint_offset_rad"][i].get<double>();
  if (!seat.contains("seat_offset_arm_joint_offset_rad") || !seat["seat_offset_arm_joint_offset_rad"].is_array() || seat["seat_offset_arm_joint_offset_rad"].size() != kArmDof) {
    ROS_ERROR("[SitControlManager] missing or invalid seat_offset_arm_joint_offset_rad (size %zu).", kArmDof);
    return false;
  }
  for (size_t i = 0; i < kArmDof; ++i)
    arm_offset_rad_[i] = seat["seat_offset_arm_joint_offset_rad"][i].get<double>();
  offset_duration_sec_ = seat.value("seat_offset_duration_seconds", 0.0);
  if (offset_duration_sec_ <= 0.0) {
    ROS_ERROR("[SitControlManager] seat_offset_duration_seconds must be positive.");
    return false;
  }
  offset_smoothstep_ = (seat.value("seat_offset_alpha_profile", "linear") != "linear");

  // ── seat_leg_action (P4) ──
  leg_action_config_ok_ = false;
  leg_action_smoothstep_ = (seat.value("seat_leg_action_alpha_profile", "linear") != "linear");
  {
    bool steps_ok = true;
    for (size_t s = 0; s < kLegActionSteps; ++s) {
      const std::string dur_key = "seat_leg_action_step" + std::to_string(s + 1) + "_duration_seconds";
      const std::string joint_key = "seat_leg_action_step" + std::to_string(s + 1) + "_leg_joint_rad";
      leg_action_durations_[s] = seat.value(dur_key, -1.0);
      if (leg_action_durations_[s] <= 0.0 ||
          !seat.contains(joint_key) || !seat[joint_key].is_array() || seat[joint_key].size() != kLegDof) {
        steps_ok = false;
        break;
      }
    }
    if (steps_ok) {
      for (size_t s = 0; s < kLegActionSteps; ++s) {
        const std::string joint_key = "seat_leg_action_step" + std::to_string(s + 1) + "_leg_joint_rad";
        for (size_t di = 0; di < kLegDof; ++di)
          leg_action_step_rad_[s][di] = seat[joint_key][di].get<double>();
      }
      leg_action_config_ok_ = true;
      // seat_leg_action_enabled=false 时跳过 P4 收脚（与旧 isSeatLegActionEnabled 判断一致）
      const bool leg_action_enabled = seat.value("seat_leg_action_enabled", true);
      if (!leg_action_enabled) {
        leg_action_config_ok_ = false;
        ROS_INFO("[SitControlManager] seat_leg_action_enabled=false; P4 leg sequence skipped (hold at P3 CSP).");
      }
    } else {
      ROS_WARN("[SitControlManager] seat_leg_action config missing; P4 leg sequence disabled.");
    }
  }

  // ── stand_up_from_seat（sit→stand 速度 + preUpdate timing）──
  const nlohmann::json* stand_up_cfg =
      seat.contains("stand_up_from_seat") && seat["stand_up_from_seat"].is_object() ? &seat["stand_up_from_seat"]
                                                                                    : nullptr;
  if (stand_up_cfg) {
    sit_to_stand_com_velocity_mps_ =
        stand_up_cfg->value("sit_to_stand_com_velocity_mps", sit_to_stand_com_velocity_mps_);
    mpc_init_delay_after_stand_up_sec_ =
        stand_up_cfg->value("mpc_init_delay_after_stand_up_seconds", mpc_init_delay_after_stand_up_sec_);
    contact_protect_grace_after_stand_up_sec_ = stand_up_cfg->value(
        "contact_protect_grace_after_stand_up_seconds", contact_protect_grace_after_stand_up_sec_);
    hold_at_sit_pose_before_stand_up_sec_ = stand_up_cfg->value("hold_at_sit_pose_before_stand_up_seconds",
                                                                hold_at_sit_pose_before_stand_up_sec_);
  }
  if (mpc_init_delay_after_stand_up_sec_ < 0.0) mpc_init_delay_after_stand_up_sec_ = 0.0;
  if (contact_protect_grace_after_stand_up_sec_ < 0.0) contact_protect_grace_after_stand_up_sec_ = 0.0;
  if (hold_at_sit_pose_before_stand_up_sec_ < 0.0) hold_at_sit_pose_before_stand_up_sec_ = 0.0;

  // ── hardware_sit_pose_prep（use_sit_init 腿 EC 增益，P3/P4 CSP 锁定复用）──
  const nlohmann::json* prep_obj =
      seat.contains("hardware_sit_pose_prep") && seat["hardware_sit_pose_prep"].is_object()
          ? &seat["hardware_sit_pose_prep"]
          : nullptr;
  if (prep_obj) {
    if (prep_obj->contains("move_speed_deg_per_s"))
      boot_prep_speed_deg_ = (*prep_obj)["move_speed_deg_per_s"].get<double>();
    if (prep_obj->contains("leg_joint_kp") && prep_obj->contains("leg_joint_kd") &&
        (*prep_obj)["leg_joint_kp"].is_array() && (*prep_obj)["leg_joint_kd"].is_array() &&
        (*prep_obj)["leg_joint_kp"].size() == kLegDof && (*prep_obj)["leg_joint_kd"].size() == kLegDof) {
      sit_boot_leg_kp_.resize(kLegDof);
      sit_boot_leg_kd_.resize(kLegDof);
      for (size_t i = 0; i < kLegDof; ++i) {
        sit_boot_leg_kp_[i] = (*prep_obj)["leg_joint_kp"][i].get<double>();
        sit_boot_leg_kd_[i] = (*prep_obj)["leg_joint_kd"][i].get<double>();
      }
      has_sit_boot_leg_gains_ = true;
      ROS_INFO("[SitControlManager] hardware_sit_pose_prep leg EC kp/kd loaded for CSP hold.");
    }
  }

  // ── 踝关节堵转阈值 ──
  stall_torque_threshold_ = seat.value("seat_stall_ankle_torque_nm", 20.0);
  stall_consecutive_limit_ = static_cast<int>(seat.value("seat_stall_consecutive_frames", 10));
  if (stall_consecutive_limit_ < 1) stall_consecutive_limit_ = 1;

  config_ok_ = true;
  ROS_INFO("[SitControlManager] seat_offset loaded (duration=%.2fs, profile=%s).", offset_duration_sec_,
           offset_smoothstep_ ? "smoothstep" : "linear");
  if (leg_action_config_ok_)
    ROS_INFO("[SitControlManager] seat_leg_action loaded (%zu steps: %.2fs/%.2fs/%.2fs, profile=%s).",
             kLegActionSteps, leg_action_durations_[0], leg_action_durations_[1], leg_action_durations_[2],
             leg_action_smoothstep_ ? "smoothstep" : "linear");
  ROS_INFO("[SitControlManager] stand_up_from_seat: sit_to_stand_vel=%.3f m/s, mpc_init_delay=%.2fs, "
           "contact_grace=%.2fs, hold_at_sit=%.2fs.",
           sit_to_stand_com_velocity_mps_, mpc_init_delay_after_stand_up_sec_,
           contact_protect_grace_after_stand_up_sec_, hold_at_sit_pose_before_stand_up_sec_);
  return true;
}

void SitControlManager::setupRos(ros::NodeHandle& nh) {
  if (!loadConfig())
    return;

  ros::param::param("seat_freeze_mrt_on_sit_complete", freeze_mrt_on_sit_complete_, true);
  publishPhaseParam();

  sub_sit_complete_ = nh.subscribe("/bot_sit_down_complete", 1, &SitControlManager::onSitDownComplete, this);
  sub_unfreeze_ = nh.subscribe("/bot_sit_unfreeze", 1, &SitControlManager::onSitUnfreeze, this);
}

bool SitControlManager::isSeatOffsetActive() const {
  return state_ == State::OFFSET_RUNNING || state_ == State::OFFSET_DONE;
}

SitControlManager::Phase SitControlManager::getPhase() const {
  switch (state_) {
    case State::FROZEN:
      return Phase::P2_SIT_HOLD;
    case State::OFFSET_RUNNING:
    case State::OFFSET_DONE:
      return Phase::P3_OFFSET;
    case State::LEG_ACTION_STEP0_RUNNING:
    case State::LEG_ACTION_STEP1_RUNNING:
    case State::LEG_ACTION_STEP2_RUNNING:
    case State::LEG_ACTION_DONE:
      return Phase::P4_LEG_ACTION;
    default:
      return Phase::P1_STAND;
  }
}

void SitControlManager::publishPhaseParam() const {
  ros::param::set("/seat_control_phase", static_cast<int>(getPhase()));
  ros::param::set("/seat_p3_csp_ready", state_ == State::OFFSET_DONE || isSeatLegActionActive());
  ros::param::set("/seat_sit_sequence_complete", sit_sequence_complete_);
}

void SitControlManager::markSitSequenceComplete() {
  if (sit_sequence_complete_)
    return;
  sit_sequence_complete_ = true;
  sit_auto_advance_active_ = false;
  publishPhaseParam();
  ROS_INFO("[SitControlManager] Full sit sequence complete (final CSP hold). Next sit_down may power off.");
}

void SitControlManager::releaseSeatResources() {
  resetOffset();
  resetLegAction();
  resetAnkleStallDetector();
  frozen_hold_counter_ = 0;
  frozen_mpc_state_.resize(0);
  frozen_mpc_input_.resize(0);
  frozen_mpc_mode_ = 0;
  p3_end_targets_ = {};
  leg_action_leg_start_.fill(0.0);
  sit_auto_advance_active_ = false;
  sit_sequence_complete_ = false;
  state_ = State::IDLE;
  publishPhaseParam();
}

bool SitControlManager::tickAutoAdvance() {
  if (!sit_auto_advance_active_ || sit_sequence_complete_)
    return false;

  if (state_ == State::FROZEN) {
    std::string err;
    if (!config_ok_) {
      ROS_ERROR("[SitControlManager] auto-advance P3 skipped: seat_offset config not loaded.");
      sit_auto_advance_active_ = false;
      return false;
    }
    if (!startOffset(err)) {
      ROS_WARN("[SitControlManager] auto-advance P3 failed: %s", err.c_str());
      return false;
    }
    ROS_INFO("[SitControlManager] auto-advance: P2 -> P3 seat_offset.");
    return true;
  }

  if (state_ == State::OFFSET_DONE && !isSeatLegActionActive()) {
    if (!leg_action_config_ok_) {
      markSitSequenceComplete();
      return true;
    }
    std::string err;
    if (!startLegAction(err)) {
      ROS_WARN("[SitControlManager] auto-advance P4 failed: %s", err.c_str());
      return false;
    }
    ROS_INFO("[SitControlManager] auto-advance: P3 CSP -> P4 leg action.");
    return true;
  }
  return false;
}

bool SitControlManager::captureStandUpFromSeatStartSnapshot(StandUpFromSeatStartSnapshot& out,
                                                            std::string& err) const {
  out = {};
  if (isSeatOffsetRunning() || isSeatLegActionRunning()) {
    err = "seat motion still running; wait for final CSP hold";
    return false;
  }
  if (!sit_sequence_complete_) {
    err = "sit sequence not at final hold; wait for P3/P4 auto sequence to finish";
    return false;
  }
  if (state_ != State::OFFSET_DONE && state_ != State::LEG_ACTION_DONE) {
    err = "must be at final CSP hold (P3 or P4 complete)";
    return false;
  }

  if (state_ == State::LEG_ACTION_DONE)
    out.csp_hold_joint_targets = buildLegActionTargets(1.0);
  else
    out.csp_hold_joint_targets = buildTargets(1.0);
  if (!out.csp_hold_joint_targets.valid) {
    err = "final CSP hold joint targets unavailable";
    return false;
  }

  out.reverse_seat_offset_duration_sec = offset_duration_sec_;
  out.use_smoothstep = offset_smoothstep_;
  out.valid = true;
  err.clear();
  return true;
}

bool SitControlManager::releaseSeatHoldForStandUp(std::string& err) {
  if (isSeatOffsetRunning() || isSeatLegActionRunning()) {
    err = "seat motion still running; wait for final CSP hold";
    return false;
  }
  if (!sit_sequence_complete_) {
    err = "sit sequence not at final hold; wait for P3/P4 auto sequence to finish";
    return false;
  }
  if (state_ != State::OFFSET_DONE && state_ != State::LEG_ACTION_DONE) {
    err = "must be at final CSP hold (P3 or P4 complete)";
    return false;
  }

  releaseSeatResources();
  ROS_INFO("[SitControlManager] Released final seat hold for preUpdate re-entry (MPC remains paused until preUpdate completes).");
  err.clear();
  return true;
}

double SitControlManager::smoothstep01(double t) {
  t = std::max(0.0, std::min(1.0, t));
  return t * t * (3.0 - 2.0 * t);
}

double SitControlManager::alphaFromElapsed(double elapsed_sec, double duration_sec, bool use_smoothstep) const {
  if (duration_sec <= 1e-6)
    return 1.0;
  const double t = std::min(elapsed_sec / duration_sec, 1.0);
  if (!use_smoothstep)
    return t;
  return smoothstep01(t);
}

SitControlManager::SeatPolicy SitControlManager::policy() const {
  SeatPolicy p;
  p.active = isSeatOffsetActive() || isSeatLegActionActive();
  p.wbc_bypass = p.active;
  p.offset_running = state_ == State::OFFSET_RUNNING;
  p.csp_hold = isSeatCspHold();
  p.policy_frozen = isPolicyFrozen();
  p.use_frozen_ref = isPolicyFrozen() || shouldHoldFrozenMpcReference();
  p.offset_alpha = alpha_;
  return p;
}

void SitControlManager::tickFrame(double time, const Eigen::VectorXd& obs_state,
                                  const Eigen::VectorXd& cur_input, size_t mode) {
  cur_mpc_time_ = time;
  cur_mpc_state_ = obs_state;
  cur_mpc_input_ = cur_input;
  cur_mpc_mode_ = mode;
  tickFrozenHoldCounter();
  tickAutoAdvance();
}

SitControlManager::SeatJointTargets SitControlManager::buildTargets(double alpha) const {
  SeatJointTargets t;
  if (static_cast<size_t>(frozen_mpc_state_.size()) < kMpcLegStateStart + kLegDof)
    return t;
  t.valid = true;
  t.alpha = alpha;
  for (size_t i = 0; i < kLegDof; ++i) {
    const auto idx = static_cast<Eigen::Index>(kMpcLegStateStart + i);
    t.leg[i] = frozen_mpc_state_(idx) + alpha * leg_offset_rad_[i];
  }
  if (static_cast<size_t>(frozen_mpc_state_.size()) > kMpcWaistStateIdx)
    t.waist = frozen_mpc_state_(static_cast<Eigen::Index>(kMpcWaistStateIdx));
  for (size_t i = 0; i < kArmDof; ++i)
    t.arm[i] = arm_pos_[i] + alpha * arm_offset_rad_[i];
  return t;
}

bool SitControlManager::startOffset(std::string& err) {
  if (state_ == State::OFFSET_RUNNING) {
    err = "seat offset already running";
    return false;
  }
  if (static_cast<size_t>(frozen_mpc_state_.size()) < kMpcLegStateStart + kLegDof) {
    err = "frozen MPC state unavailable";
    return false;
  }
  // P3 臂参考 = arm_pos_ + α·offset；须在 P3 起点从冻结落座态取臂角，否则 arm_pos_ 默认为 0 会导致首帧大幅跳变。
  constexpr size_t kMpcArmStart = kMpcWaistStateIdx + 1;
  for (size_t i = 0; i < kArmDof; ++i) {
    const auto idx = static_cast<Eigen::Index>(kMpcArmStart + i);
    arm_pos_[i] = (idx < frozen_mpc_state_.size()) ? frozen_mpc_state_(idx) : 0.0;
  }
  state_ = State::OFFSET_RUNNING;
  alpha_ = 0.0;
  offset_t0_ = std::chrono::steady_clock::now();
  publishPhaseParam();
  ROS_INFO("[SitControlManager] P3 arm baseline(0): %.3f %.3f ... (from frozen sit, +alpha*offset)",
           arm_pos_[0], arm_pos_[1]);
  return true;
}

void SitControlManager::resetOffset() {
  alpha_ = 0.0;
}

void SitControlManager::resetLegAction() {
  leg_action_alpha_ = 0.0;
}

SitControlManager::SeatJointTargets SitControlManager::buildLegActionTargets(double alpha) const {
  SeatJointTargets t = p3_end_targets_;
  if (!t.valid)
    return {};
  t.alpha = alpha;
  const int target_idx = legActionTargetStepIndex();
  if (target_idx < 0)
    return {};
  const auto& step_target = leg_action_step_rad_[static_cast<size_t>(target_idx)];
  for (size_t i = 0; i < kLegDof; ++i)
    t.leg[i] = leg_action_leg_start_[i] + alpha * (step_target[i] - leg_action_leg_start_[i]);
  return t;
}

int SitControlManager::legActionStepIndex() const {
  switch (state_) {
    case State::LEG_ACTION_STEP0_RUNNING:
      return 0;
    case State::LEG_ACTION_STEP1_RUNNING:
      return 1;
    case State::LEG_ACTION_STEP2_RUNNING:
      return 2;
    default:
      return -1;
  }
}

int SitControlManager::legActionTargetStepIndex() const {
  if (state_ == State::LEG_ACTION_DONE)
    return static_cast<int>(kLegActionSteps) - 1;
  return legActionStepIndex();
}

bool SitControlManager::startLegAction(std::string& err) {
  if (!leg_action_config_ok_) {
    err = "seat_leg_action config not loaded";
    return false;
  }
  if (state_ != State::OFFSET_DONE) {
    err = "must be P3 CSP hold (OFFSET_DONE)";
    return false;
  }
  if (isSeatLegActionRunning()) {
    err = "leg action already running";
    return false;
  }
  p3_end_targets_ = buildTargets(1.0);
  if (!p3_end_targets_.valid) {
    err = "P3 end targets unavailable";
    return false;
  }
  leg_action_leg_start_ = p3_end_targets_.leg;
  leg_action_alpha_ = 0.0;
  leg_action_t0_ = std::chrono::steady_clock::now();
  state_ = State::LEG_ACTION_STEP0_RUNNING;
  publishPhaseParam();
  ROS_INFO("[SitControlManager] P4 step1: leg action started (%.2fs).", leg_action_durations_[0]);
  return true;
}

void SitControlManager::advanceLegActionStep() {
  const int step_idx = legActionStepIndex();
  if (step_idx < 0 || step_idx >= static_cast<int>(kLegActionSteps))
    return;
  leg_action_leg_start_ = leg_action_step_rad_[static_cast<size_t>(step_idx)];
  leg_action_alpha_ = 0.0;
  leg_action_t0_ = std::chrono::steady_clock::now();
  if (step_idx + 1 >= static_cast<int>(kLegActionSteps)) {
    leg_action_alpha_ = 1.0;
    state_ = State::LEG_ACTION_DONE;
    publishPhaseParam();
    if (sit_auto_advance_active_)
      markSitSequenceComplete();
    ROS_INFO("[SitControlManager] P4 step%zu complete -> CSP hold at final leg pose.", kLegActionSteps);
    return;
  }
  state_ = static_cast<State>(static_cast<int>(State::LEG_ACTION_STEP0_RUNNING) + step_idx + 1);
  publishPhaseParam();
  ROS_INFO("[SitControlManager] P4 step%d complete -> step%d (%.2fs).", step_idx + 1, step_idx + 2,
           leg_action_durations_[static_cast<size_t>(step_idx + 1)]);
}

SitControlManager::SeatJointTargets SitControlManager::updateLegAction() {
  if (!isSeatLegActionActive())
    return {};
  if (state_ == State::LEG_ACTION_DONE)
    return buildLegActionTargets(1.0);

  const int step_idx = legActionStepIndex();
  if (step_idx < 0)
    return buildLegActionTargets(1.0);
  const double elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - leg_action_t0_).count();
  leg_action_alpha_ =
      alphaFromElapsed(elapsed, leg_action_durations_[static_cast<size_t>(step_idx)], leg_action_smoothstep_);
  if (leg_action_alpha_ >= 1.0) {
    leg_action_alpha_ = 1.0;
    advanceLegActionStep();
    if (state_ == State::LEG_ACTION_DONE)
      return buildLegActionTargets(1.0);
    return buildLegActionTargets(0.0);
  }
  return buildLegActionTargets(leg_action_alpha_);
}

void SitControlManager::seedArmBaselineFromHardware(const Eigen::VectorXd& arm_hw_pos) {
  const size_t n = std::min(static_cast<size_t>(arm_hw_pos.size()), kArmDof);
  for (size_t i = 0; i < n; ++i)
    arm_pos_[i] = arm_hw_pos(static_cast<Eigen::Index>(i));
}

void SitControlManager::writeSeatTargetsWbc(Eigen::Ref<Eigen::VectorXd> joint_q, Eigen::Ref<Eigen::VectorXd> joint_v,
                                            const SeatJointTargets& targets, size_t leg_dof, size_t waist_num,
                                            size_t arm_num) const {
  if (!targets.valid)
    return;
  for (size_t i = 0; i < kLegDof && i < leg_dof; ++i) {
    joint_q(static_cast<Eigen::Index>(i)) = targets.leg[i];
    joint_v(static_cast<Eigen::Index>(i)) = 0.0;
  }
  if (waist_num > 0) {
    const auto w = static_cast<Eigen::Index>(leg_dof);
    joint_q(w) = targets.waist;
    joint_v(w) = 0.0;
  }
  const size_t arm_base = leg_dof + waist_num;
  for (size_t i = 0; i < kArmDof && i < arm_num; ++i) {
    const auto j = static_cast<Eigen::Index>(arm_base + i);
    joint_q(j) = targets.arm[i];
    joint_v(j) = 0.0;
  }
}

void SitControlManager::writeSeatTargetsHw(Eigen::Ref<Eigen::VectorXd> hw_pos, Eigen::Ref<Eigen::VectorXd> hw_vel,
                                           Eigen::Ref<Eigen::VectorXd> hw_tau, const SeatJointTargets& targets,
                                           size_t leg_dof, size_t waist_num, size_t arm_num) const {
  if (!targets.valid)
    return;
  for (size_t i = 0; i < kLegDof && i < leg_dof; ++i) {
    const auto j = static_cast<Eigen::Index>(i);
    hw_pos(j) = targets.leg[i];
    hw_vel(j) = 0.0;
    hw_tau(j) = 0.0;
  }
  if (waist_num > 0) {
    const auto w = static_cast<Eigen::Index>(leg_dof);
    hw_pos(w) = targets.waist;
    hw_vel(w) = 0.0;
    hw_tau(w) = 0.0;
  }
  const size_t arm_base = leg_dof + waist_num;
  for (size_t i = 0; i < kArmDof && i < arm_num; ++i) {
    const auto j = static_cast<Eigen::Index>(arm_base + i);
    hw_pos(j) = targets.arm[i];
    hw_vel(j) = 0.0;
    hw_tau(j) = 0.0;
  }
}

SitControlManager::SeatOffsetStep SitControlManager::stepSeatOffset(double dt, Eigen::Ref<Eigen::VectorXd> wbc_joint_q,
                                                                    Eigen::Ref<Eigen::VectorXd> wbc_joint_v,
                                                                    Eigen::Ref<Eigen::VectorXd> hw_pos,
                                                                    Eigen::Ref<Eigen::VectorXd> hw_vel,
                                                                    Eigen::Ref<Eigen::VectorXd> hw_tau,
                                                                    size_t leg_dof, size_t waist_num,
                                                                    size_t arm_num) {
  SeatOffsetStep step;
  if (isSeatLegActionActive()) {
    step.active = true;
    step.running = isSeatLegActionRunning();
    step.targets = updateLegAction();
    if (!step.targets.valid)
      return step;
    writeSeatTargetsHw(hw_pos, hw_vel, hw_tau, step.targets, leg_dof, waist_num, arm_num);
    step.skip_wbc = true;
    return step;
  }

  if (!isSeatOffsetActive())
    return step;
  step.active = true;
  step.running = isSeatOffsetRunning();
  step.targets = update();
  if (!step.targets.valid)
    return step;
  if (step.running)
    writeSeatTargetsWbc(wbc_joint_q, wbc_joint_v, step.targets, leg_dof, waist_num, arm_num);
  else {
    writeSeatTargetsHw(hw_pos, hw_vel, hw_tau, step.targets, leg_dof, waist_num, arm_num);
    step.skip_wbc = true;
  }
  return step;
}

SitControlManager::SeatJointTargets SitControlManager::update() {
  if (!isSeatOffsetActive())
    return {};
  if (state_ == State::OFFSET_DONE)
    return buildTargets(1.0);

  const double elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - offset_t0_).count();
  alpha_ = alphaFromElapsed(elapsed, offset_duration_sec_, offset_smoothstep_);
  if (state_ == State::OFFSET_RUNNING && alpha_ >= 1.0) {
    state_ = State::OFFSET_DONE;
    alpha_ = 1.0;
    publishPhaseParam();
    ROS_INFO("[SitControlManager] P3 offset complete -> CSP hold.");
    tickAutoAdvance();
  }
  return buildTargets(alpha_);
}

void SitControlManager::onSitDownComplete(const std_msgs::Int8::ConstPtr& msg) {
  (void)msg;
  if (!freeze_mrt_on_sit_complete_)
    return;
  if (state_ != State::IDLE)
    return;
  frozen_mpc_state_ = cur_mpc_state_;
  frozen_mpc_input_ = cur_mpc_input_;
  frozen_mpc_mode_ = cur_mpc_mode_;
  state_ = State::FROZEN;
  sit_auto_advance_active_ = true;
  sit_sequence_complete_ = false;
  publishPhaseParam();
  if (mrt_) {
    mrt_->pauseResumeMpcNode(true);
    ROS_INFO("[SitControlManager] P2: MPC solver paused (policy frozen on controller).");
  }
  ROS_INFO("[SitControlManager] P2: MPC frozen at sit endpoint (auto P3/P4 armed).");
}

void SitControlManager::onSitUnfreeze(const std_msgs::Int8::ConstPtr& msg) {
  // 自动串联状态下，起立统一经 humanoidController preUpdate 重入，不再走 MPC 反向 lerp。
  // 保留本回调仅供兼容/应急 unfreeze；正常 stand_up 不再发布 /bot_sit_unfreeze。
  const bool from_p4 = isSeatLegActionActive();
  const bool from_p3 = (state_ == State::OFFSET_RUNNING || state_ == State::OFFSET_DONE);
  const bool p2_to_p1 = (msg->data == 1 && !from_p3 && !from_p4);
  resetOffset();
  resetLegAction();
  if (from_p4) {
    state_ = State::OFFSET_DONE;
    alpha_ = 1.0;
  } else {
    state_ = from_p3 ? State::FROZEN : State::IDLE;
  }
  sit_auto_advance_active_ = false;
  publishPhaseParam();
  if (p2_to_p1)
    frozen_hold_counter_ = kFrozenHoldAwaitingPolicy;
  resumeMpc();
  if (from_p4)
    ROS_INFO("[SitControlManager] stand_up (legacy unfreeze): P4 -> P3 CSP hold.");
  else
    ROS_INFO("[SitControlManager] stand_up (legacy unfreeze): %s -> %s", from_p3 ? "P3" : "P2", from_p3 ? "P2" : "P1");
}

void SitControlManager::resetMpcAfterSitPause(const char* reason) {
  if (!mrt_ || cur_mpc_state_.size() == 0) {
    ROS_WARN("[SitControlManager] resetMpc skipped (%s): current MRT state not ready.", reason);
    return;
  }
  // 与 humanoidController RL->MPC 切换相同：spinMRT + 当前观测态 + 零输入 reset
  mrt_->spinMRT();
  Eigen::VectorXd zero_input = Eigen::VectorXd::Zero(cur_mpc_input_.size());
  const ocs2::scalar_array_t timeTrajectory{cur_mpc_time_};
  const ocs2::vector_array_t stateTrajectory{cur_mpc_state_};
  const ocs2::vector_array_t inputTrajectory{zero_input};
  ocs2::TargetTrajectories target(timeTrajectory, stateTrajectory, inputTrajectory);
  mrt_->resetMpcNode(target);
  ROS_INFO("[SitControlManager] resetMpcNode at t=%.3f (%s) [RL->MPC pattern].", cur_mpc_time_, reason);
}

void SitControlManager::resumeMpc() {
  if (!mrt_)
    return;
  mrt_->pauseResumeMpcNode(false);
  resetMpcAfterSitPause("seat_resume");
}

void SitControlManager::notifyMpcPolicyUpdated() {
  if (frozen_hold_counter_ == kFrozenHoldAwaitingPolicy)
    frozen_hold_counter_ = kFrozenHoldFrames;
}

void SitControlManager::tickFrozenHoldCounter() {
  if (frozen_hold_counter_ > 0)
    --frozen_hold_counter_;
}

// ── P3/P4 实机 CSP ──

void SitControlManager::applyRealHardwareCspHold(kuavo_msgs::jointCmd& msg,
                                                 const Eigen::VectorXd& leg_waist_kp,
                                                 const Eigen::VectorXd& leg_waist_kd, int leg_waist_count,
                                                 int arm_start, int arm_count) const {
  for (int i = 0; i < leg_waist_count; ++i) {
    msg.control_modes[static_cast<size_t>(i)] = 2;
    msg.tau[static_cast<size_t>(i)] = 0.0;
    msg.joint_v[static_cast<size_t>(i)] = 0.0;
    double kp = leg_waist_kp[i];
    double kd = leg_waist_kd[i];
    if (i < static_cast<int>(kLegDof) && has_sit_boot_leg_gains_ &&
        static_cast<size_t>(i) < sit_boot_leg_kp_.size()) {
      kp = sit_boot_leg_kp_[static_cast<size_t>(i)];
      kd = sit_boot_leg_kd_[static_cast<size_t>(i)];
    }
    msg.joint_kp[static_cast<size_t>(i)] = kp;
    msg.joint_kd[static_cast<size_t>(i)] = kd;
  }
  for (int i = 0; i < arm_count; ++i) {
    const size_t j = static_cast<size_t>(arm_start + i);
    msg.control_modes[j] = 2;
    msg.tau[j] = 0.0;
    msg.joint_v[j] = 0.0;
  }
}

bool SitControlManager::checkAnkleStall(const Eigen::Ref<const Eigen::VectorXd>& ankle_torques) {
  if (!isSeatCspHold())
    return false;
  const double max_tau = std::max({std::abs(ankle_torques(0)), std::abs(ankle_torques(1)),
                                    std::abs(ankle_torques(2)), std::abs(ankle_torques(3))});
  if (max_tau > stall_torque_threshold_) {
    ++stall_consecutive_count_;
    if (stall_consecutive_count_ >= stall_consecutive_limit_) {
      ROS_ERROR("[SitControlManager] ankle stall: max_tau=%.1f > %.1f N·m x %d frames — STOP!",
                max_tau, stall_torque_threshold_, stall_consecutive_count_);
      return true;
    }
  } else {
    stall_consecutive_count_ = 0;
  }
  return false;
}

void SitControlManager::resetAnkleStallDetector() {
  stall_consecutive_count_ = 0;
}

}  // namespace humanoid_controller
