// 从 humanoidController 抽出的「坐起身」控制器实现。
// 逻辑与原 humanoidController::beginStandUpFromSeat / preUpdate 段0 / tickAwaitStandUpStart /
// publishSeatCspHoldFromTargets / preUpdateStandUpHeadPitchRad 逐段对应，仅改成员归属。

#include "humanoid_controllers/SitUpController.h"

#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>
#include <humanoid_interface_drake/humanoid_interface_drake.h>
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_srvs/Trigger.h>
#include <thread>

namespace humanoid_controller {

namespace {
constexpr const char* kLogTag = "[SitUp]";
}

SitUpController::SitUpController(ros::NodeHandle& nh, SitControlManager& scm)
    : nh_(nh), scm_(scm) {}

void SitUpController::setJointSpecs(int leg_num, int waist_num, int arm_num, int head_num,
                                    const Eigen::VectorXd& joint_kp, const Eigen::VectorXd& joint_kd,
                                    const std::vector<double>& max_current,
                                    bool is_real) {
  leg_n_ = leg_num;
  waist_n_ = waist_num;
  arm_n_ = arm_num;
  head_n_ = head_num;
  joint_kp_ = joint_kp;
  joint_kd_ = joint_kd;
  max_current_ = max_current;
  is_real_ = is_real;
}

void SitUpController::setMeasuredStart(const Eigen::VectorXd& jointPosWBC) {
  measured_start_ = jointPosWBC;
}

void SitUpController::setupRos() {
  seat_return_reverse_prep_client_ =
      nh_.serviceClient<std_srvs::Trigger>("hardware/seat_return_reverse_prep");
}

void SitUpController::publishSeatCspHoldFromTargets(const SeatJointTargets& targets,
                                                    kuavo_msgs::jointCmd& out_cmd) {
  // 只填 out_cmd，发布由主类统一（publishControlCommands / replaceDefaultEcMotorPdoGait 留主类）。
  const double head_pitch = scm_.headDownPitchRad();
  if (!scm_.fillSeatCspHoldCmd(out_cmd, targets, joint_kp_, joint_kd_, max_current_,
                               leg_n_, waist_n_, arm_n_, head_n_, is_real_, head_pitch)) {
    ROS_ERROR_THROTTLE(1.0, "%s fillSeatCspHoldCmd failed.", kLogTag);
  }
}

void SitUpController::clearPlan() {
  snapshot_ = {};
  setHwSeatReverse(HwSeatReverse::Idle);
  hw_seat_reverse_err_.clear();
  ros::param::set("/hardware/seat_reverse_prep_active", false);
}

void SitUpController::markAborted() {
  aborted_ = true;
  phase_ = Phase::IDLE;
}

void SitUpController::resetOnComplete() {
  phase_ = Phase::IDLE;
  reverse_seat_offset_done_ = false;
  awaiting_stand_up_start_ = false;
  aborted_ = false;
  reverse_seat_settle_stage_ = 0;
  reverse_seat_offset_start_time_sec_ = 0.0;
  hold_at_sit_complete_time_sec_ = 0.0;
  setHwSeatReverse(HwSeatReverse::Idle);
  hw_seat_reverse_err_.clear();
  snapshot_ = {};
  robot_stand_up_complete_time_ = 0.0;
}

double SitUpController::headDownPitchRadAt(const ros::Time& time) const {
  // 座椅 sit_up / use_sit_init_boot 起身全程低头；进 MPC 站立时由主类清 keep_head_down 并抬头。
  (void)time;
  if (!scm_.useSitInitBoot() && phase_ == Phase::IDLE)
    return 0.0;
  return scm_.headDownPitchRad();
}

void SitUpController::kickHwReverseThread() {
  // 原 beginStandUpFromSeat 末尾：detached 线程调 hardware/seat_return_reverse_prep。
  // 捕获 this 安全：reverse prep 期间主循环停（preUpdate 接管），对象存活到 resetOnComplete。
  std::thread([this]() {
    std_srvs::Trigger srv;
    const bool ok = seat_return_reverse_prep_client_.call(srv) && srv.response.success;
    if (!ok) {
      hw_seat_reverse_err_ =
          srv.response.message.empty() ? "hardware/seat_return_reverse_prep failed"
                                        : srv.response.message;
      setHwSeatReverse(HwSeatReverse::Failed);
      ROS_ERROR("%s seat reverse prep HW failed: %s", kLogTag, hw_seat_reverse_err_.c_str());
    } else {
      setHwSeatReverse(HwSeatReverse::Done);
      ROS_INFO("%s seat reverse prep HW complete: %s", kLogTag, srv.response.message.c_str());
    }
  }).detach();
}

bool SitUpController::beginStandUpFromSeat(std::string& err) {
  // —— 门禁（与 releaseSeatHoldForStandUp 一致：须最终 CSP hold）——
  if (!scm_.isSeatOffsetRunning() && !scm_.isSeatLegActionRunning()) {
    // ok
  } else {
    err = "seat motion still running; wait for final CSP hold";
    return false;
  }
  if (!scm_.isSitSequenceComplete()) {
    err = "sit sequence not at final hold; wait for P3/P4 auto sequence to finish";
    return false;
  }
  if (!scm_.isSeatCspHold()) {
    err = "must be at final CSP hold (P3 or P4 complete)";
    return false;
  }
  if (!drake_) {
    err = "drake_interface unavailable";
    return false;
  }

  snapshot_ = {};
  snapshot_.reverse_leg_before_arm = scm_.prepReverseLegBeforeArm();
  snapshot_.reverse_arm_clear = scm_.reverseArmClearEnabled();
  snapshot_.valid = true;

  // 段0 起点：release hold 前实测（由主类 setMeasuredStart 注入，begin 时即建，与原码一致）
  if (measured_start_.size() < leg_n_ + waist_n_ + arm_n_) {
    err = "measured_start (jointPosWBC) not injected before beginStandUpFromSeat";
    return false;
  }
  snapshot_.measured_start_joint_targets =
      SitControlManager::seatTargetsFromHwJoints(measured_start_,
                                                  static_cast<size_t>(leg_n_),
                                                  static_cast<size_t>(waist_n_),
                                                  static_cast<size_t>(arm_n_));
  if (!snapshot_.measured_start_joint_targets.valid) {
    err = "measured jointPosWBC_ unavailable for stand_up phase0 start";
    clearPlan();
    return false;
  }

  // 段0 终点：配置 sit_joint_pos → Drake getSitInitialState()（与段1 sit 起点同源）
  {
    const vector_t& sit_mpc = drake_->getSitInitialState();
    const Eigen::Index joint_dim = static_cast<Eigen::Index>(leg_n_ + waist_n_ + arm_n_);
    if (sit_mpc.size() < 12 + joint_dim) {
      err = "getSitInitialState too small for sit_joint_pos joints";
      clearPlan();
      return false;
    }
    snapshot_.sit_joint_targets =
        SitControlManager::seatTargetsFromHwJoints(sit_mpc.segment(12, joint_dim),
                                                    static_cast<size_t>(leg_n_),
                                                    static_cast<size_t>(waist_n_),
                                                    static_cast<size_t>(arm_n_));
    if (!snapshot_.sit_joint_targets.valid) {
      err = "failed to map getSitInitialState joints to sit targets";
      clearPlan();
      return false;
    }
  }

  if (is_real_) {
    // 实机段0：交给 hardware_node jointMoveToPrepGoal（与 use_sit_init phase2a/2b 同路径）
    const vector_t& sit_mpc = drake_->getSitInitialState();
    const int hw_joint_count = leg_n_ + waist_n_ + arm_n_ + head_n_;
    if (!scm_.publishSeatReturnReversePrepParams(sit_mpc, measured_start_,
                                                  waist_n_, leg_n_, arm_n_, hw_joint_count, err)) {
      clearPlan();
      return false;
    }
    if (!seat_return_reverse_prep_client_.exists()) {
      err = "hardware/seat_return_reverse_prep service unavailable";
      clearPlan();
      return false;
    }
    setHwSeatReverse(HwSeatReverse::Handoff);
    hw_seat_reverse_err_.clear();
    ros::param::set("/hardware/seat_reverse_prep_active", false);
  } else {
    // 仿真无 hardware_node：CSP 复刻同一套 reverse 轨迹（非实机失败回退）
    const auto& measured = snapshot_.measured_start_joint_targets;
    const auto& sit = snapshot_.sit_joint_targets;
    const double move_deg_s = scm_.prepMoveSpeedDegPerS();
    const double settle_deg_s = scm_.prepSettleSpeedDegPerS();

    auto dur_from_rad = [](double max_abs_dq_rad, double speed_deg_s) {
      const double min_d = SitControlManager::kJointMoveToMinDurationSec;
      if (speed_deg_s <= 1e-6)
        return min_d;
      const double d = (max_abs_dq_rad * 180.0 / M_PI) / speed_deg_s;
      return (d < min_d) ? min_d : d;
    };

    if (snapshot_.reverse_leg_before_arm) {
      snapshot_.leg_first_mid_targets =
          SitControlManager::makeLegFirstMidTargets(measured, sit);
      if (!snapshot_.leg_first_mid_targets.valid) {
        err = "failed to build leg_first mid targets for phase0a";
        clearPlan();
        return false;
      }
      using SCM = SitControlManager;
      const double dq_lw = SCM::maxAbsDq(measured, snapshot_.leg_first_mid_targets, true, false);
      snapshot_.phase0a_duration_sec = dur_from_rad(dq_lw, move_deg_s);
      if (snapshot_.reverse_arm_clear) {
        snapshot_.arm_clear_mid_targets = SCM::makeArmClearMidTargets(
            measured, sit, scm_.reverseArmClearOffsetRad());
        if (!snapshot_.arm_clear_mid_targets.valid) {
          err = "failed to build arm_clear mid targets for phase0b";
          clearPlan();
          return false;
        }
        const double dq_clear =
            SCM::maxAbsDq(snapshot_.leg_first_mid_targets, snapshot_.arm_clear_mid_targets, false, true);
        const double dq_sit =
            SCM::maxAbsDq(snapshot_.arm_clear_mid_targets, sit, false, true);
        snapshot_.phase0b_duration_sec = dur_from_rad(dq_clear, settle_deg_s);
        snapshot_.phase0c_duration_sec = dur_from_rad(dq_sit, settle_deg_s);
        ROS_INFO("%s phase0 sim CSP settle: 0a move=%.1f deg/s (%.2fs), "
                 "0b clear=%.1f deg/s (%.2fs), 0c sit=%.1f deg/s (%.2fs).",
                 kLogTag, move_deg_s, snapshot_.phase0a_duration_sec,
                 settle_deg_s, snapshot_.phase0b_duration_sec,
                 settle_deg_s, snapshot_.phase0c_duration_sec);
      } else {
        const double dq_arm = SCM::maxAbsDq(snapshot_.leg_first_mid_targets, sit, false, true);
        snapshot_.phase0b_duration_sec = dur_from_rad(dq_arm, settle_deg_s);
        snapshot_.phase0c_duration_sec = 0.0;
        ROS_INFO("%s phase0 sim CSP settle: 0a move=%.1f deg/s (%.2fs), "
                 "0b settle=%.1f deg/s (%.2fs).",
                 kLogTag, move_deg_s, snapshot_.phase0a_duration_sec,
                 settle_deg_s, snapshot_.phase0b_duration_sec);
      }
    } else {
      const double max_abs_dq =
          SitControlManager::maxAbsDq(measured, sit, true, true);
      snapshot_.phase0a_duration_sec = dur_from_rad(max_abs_dq, settle_deg_s);
      snapshot_.phase0b_duration_sec = 0.0;
      snapshot_.phase0c_duration_sec = 0.0;
      ROS_INFO("%s phase0 sim CSP: all joints settle=%.1f deg/s (%.2fs).",
               kLogTag, settle_deg_s, snapshot_.phase0a_duration_sec);
    }
  }

  // 状态机进入段0（kick HW / done pub 由主类做；CSP hold 延到 preUpdate 段0 再 release）
  phase_ = Phase::PHASE0_REVERSE;
  reverse_seat_offset_done_ = false;
  awaiting_stand_up_start_ = false;
  reverse_seat_settle_stage_ = 0;
  reverse_seat_offset_start_time_sec_ = 0.0;
  hold_at_sit_complete_time_sec_ = 0.0;
  aborted_ = false;

  ROS_INFO("%s begin: phase0=%s, then await start, phase1=sit->stand.",
           kLogTag, is_real_ ? "HW jointMoveToPrepGoal" : "sim CSP replica");
  err.clear();
  return true;
}

void SitUpController::kickHwReverseIfNeeded() {
  if (is_real_ && hwSeatReverse() == HwSeatReverse::Handoff)
    kickHwReverseThread();
}

bool SitUpController::tickAwaitStandUpStart(bool with_sit_csp_hold, kuavo_msgs::jointCmd& out_cmd) {
  const bool want_await = scm_.awaitStartBeforeSitToStand();
  if (!want_await) {
    reverse_seat_offset_done_ = true;
    return false;
  }
  if (!awaiting_stand_up_start_) {
    awaiting_stand_up_start_ = true;
    ros::param::set("/hardware/is_ready", 0);
    ros::param::set("/hardware/ready_to_start", 1);
  }
  if (with_sit_csp_hold)
    publishSeatCspHoldFromTargets(snapshot_.sit_joint_targets, out_cmd);

  int is_ready = 0;
  if (ros::param::get("/hardware/is_ready", is_ready) && is_ready == 1) {
    awaiting_stand_up_start_ = false;
    reverse_seat_offset_done_ = true;
    ROS_INFO("%s stand_up start received (is_ready=1); start sit->stand.", kLogTag);
    return false;
  }
  return true;
}

bool SitUpController::runHwReversePhase0(const ros::Time& time, const Eigen::VectorXd& jointPosWBC,
                                         kuavo_msgs::jointCmd& out_cmd) {
  using Hw = HwSeatReverse;
  const auto st = hwSeatReverse();
  if (st == Hw::Failed) {
    ROS_ERROR("%s abort stand_up: HW reverse prep failed: %s", kLogTag, hw_seat_reverse_err_.c_str());
    markAborted();
    return false;  // 主类发 stand_up failed 并 return
  }
  if (st == Hw::Handoff || st == Hw::Running) {
    if (st == Hw::Handoff) {
      bool hw_active = false;
      if (ros::param::get("/hardware/seat_reverse_prep_active", hw_active) && hw_active) {
        setHwSeatReverse(Hw::Running);
        ROS_INFO("%s HW seat reverse prep active; stop CSP bridge.", kLogTag);
      } else {
        const auto live = SitControlManager::seatTargetsFromHwJoints(
            jointPosWBC, static_cast<size_t>(leg_n_), static_cast<size_t>(waist_n_),
            static_cast<size_t>(arm_n_));
        if (!live.valid) {
          hw_seat_reverse_err_ = "jointPosWBC_ invalid during CSP bridge handoff";
          setHwSeatReverse(Hw::Failed);
          ROS_ERROR("%s %s", kLogTag, hw_seat_reverse_err_.c_str());
          return true;  // 仍 true：桥接失败但未 abort，等下帧
        }
        // 填 CSP hold cmd → 主类统一发布
        publishSeatCspHoldFromTargets(live, out_cmd);
      }
    }
    return true;  // 桥接/运行中：本帧 return
  }
  if (st == Hw::Done) {
    // 对齐仿真：段0 完成后先 hold_at_sit_pose_before_stand_up_sec 静持，再 await start
    if (hold_at_sit_complete_time_sec_ < 1e-6)
      hold_at_sit_complete_time_sec_ = time.toSec();
    const double hold_delay = scm_.holdAtSitPoseBeforeStandUpSec();
    const bool hold_elapsed = time.toSec() >= hold_at_sit_complete_time_sec_ + hold_delay;
    if (!hold_elapsed) {
      publishSeatCspHoldFromTargets(snapshot_.sit_joint_targets, out_cmd);
      return true;
    }
    const bool was_awaiting = awaiting_stand_up_start_;
    const bool still_waiting = tickAwaitStandUpStart(true, out_cmd);
    if (still_waiting && !was_awaiting) {
      ROS_INFO("%s phase0 HW reverse complete (hold=%.2fs); waiting for start "
               "(/hardware/is_ready=1 via 'o' or /humanoid_controller/real_initial_start).",
               kLogTag, hold_delay);
    } else if (!still_waiting && !was_awaiting) {
      ROS_INFO("%s phase0 HW reverse complete (hold=%.2fs); start sit->stand.", kLogTag, hold_delay);
    }
    return still_waiting;  // true=仍在等 → return；false=切段1
  }
  return true;
}

bool SitUpController::runSimReversePhase0(const ros::Time& time, kuavo_msgs::jointCmd& out_cmd) {
  using SCM = SitControlManager;
  const auto& snap = snapshot_;
  if (!snap.measured_start_joint_targets.valid) {
    ROS_ERROR_THROTTLE(1.0, "%s sim phase0 measured_start_joint_targets missing.", kLogTag);
    markAborted();
    return false;
  }

  const int final_stage = SCM::phase0FinalStage(snap);
  const double stage_end =
      reverse_seat_offset_start_time_sec_ + SCM::phase0StageDurationSec(snap, reverse_seat_settle_stage_);
  if (reverse_seat_settle_stage_ < final_stage && time.toSec() > stage_end) {
    const int next = reverse_seat_settle_stage_ + 1;
    reverse_seat_settle_stage_ = next;
    reverse_seat_offset_start_time_sec_ = time.toSec();
    if (next == 1 && snap.reverse_arm_clear) {
      ROS_INFO("%s phase0a done; start phase0b arms->clear (%.2fs).", kLogTag, snap.phase0b_duration_sec);
    } else if (next == 1) {
      ROS_INFO("%s phase0a done; start phase0b arms->sit (%.2fs).", kLogTag, snap.phase0b_duration_sec);
    } else if (next == 2) {
      ROS_INFO("%s phase0b done; start phase0c arms->sit (%.2fs).", kLogTag, snap.phase0c_duration_sec);
    }
  }
  const double active_end =
      reverse_seat_offset_start_time_sec_ + SCM::phase0StageDurationSec(snap, reverse_seat_settle_stage_);
  const bool holding = (reverse_seat_settle_stage_ >= final_stage) && time.toSec() > active_end;

  const double startTime = reverse_seat_offset_start_time_sec_;
  const double endTime = active_end;
  const double standUpDuration = endTime - startTime;
  double standUpBlend = 1.0;
  if (!holding && standUpDuration > 1e-6) {
    const double t = std::min(std::max((time.toSec() - startTime) / standUpDuration, 0.0), 1.0);
    standUpBlend = SCM::jointMoveToCosine01(t);  // 段0 对齐 HW calcCos
  }

  SeatJointTargets targets;
  if (holding) {
    targets = snap.sit_joint_targets;
  } else if (snap.reverse_leg_before_arm && reverse_seat_settle_stage_ == 0) {
    targets = SCM::lerpJointTargets(snap.measured_start_joint_targets, snap.leg_first_mid_targets, standUpBlend);
  } else if (snap.reverse_leg_before_arm && reverse_seat_settle_stage_ == 1 && snap.reverse_arm_clear) {
    targets = SCM::lerpJointTargets(snap.leg_first_mid_targets, snap.arm_clear_mid_targets, standUpBlend);
  } else if (snap.reverse_leg_before_arm && reverse_seat_settle_stage_ == 1) {
    targets = SCM::lerpJointTargets(snap.leg_first_mid_targets, snap.sit_joint_targets, standUpBlend);
  } else if (snap.reverse_leg_before_arm && reverse_seat_settle_stage_ == 2) {
    targets = SCM::lerpJointTargets(snap.arm_clear_mid_targets, snap.sit_joint_targets, standUpBlend);
  } else {
    targets = SCM::lerpJointTargets(snap.measured_start_joint_targets, snap.sit_joint_targets, standUpBlend);
  }
  if (!targets.valid) {
    ROS_ERROR_THROTTLE(1.0, "%s phase0 measured->sit CSP targets invalid.", kLogTag);
    return true;  // 暂跳过本帧
  }

  const double head_pitch = headDownPitchRadAt(time);
  if (!scm_.fillSeatCspHoldCmd(out_cmd, targets, joint_kp_, joint_kd_, max_current_,
                                leg_n_, waist_n_, arm_n_, head_n_, is_real_, head_pitch)) {
    ROS_ERROR_THROTTLE(1.0, "%s phase0 fillSeatCspHoldCmd failed.", kLogTag);
    return true;
  }

  if (holding) {
    // 段0 settle 完成 → 先 hold_at_sit_pose_before_stand_up_sec 静持，再 await start
    // （原 humanoidController preUpdate 2978 settle_end + hold_delay）
    if (hold_at_sit_complete_time_sec_ < 1e-6)
      hold_at_sit_complete_time_sec_ = time.toSec();  // 首帧记录 settle 完成时刻
    const double hold_delay = scm_.holdAtSitPoseBeforeStandUpSec();
    const bool hold_elapsed = time.toSec() >= hold_at_sit_complete_time_sec_ + hold_delay;
    if (!hold_elapsed) {
      // 静持期：持续发 sit CSP，本帧 return
      return true;
    }
    // hold_delay 满 → 进入/轮询 await start
    const bool still_waiting = tickAwaitStandUpStart(true, out_cmd);
    return still_waiting;
  }
  return true;
}

bool SitUpController::runPhase0(const ros::Time& time, const Eigen::VectorXd& jointPosWBC,
                                kuavo_msgs::jointCmd& out_cmd) {
  if (phase_ == Phase::IDLE)
    return false;

  // 同线程延后 release：勿在 ROS 回调里先清 CSP，否则主循环可能发一帧站立 WBC。
  if (scm_.isSeatCspHold()) {
    std::string rel_err;
    if (!scm_.releaseSeatHoldForStandUp(rel_err)) {
      ROS_ERROR("%s deferred releaseSeatHoldForStandUp failed: %s", kLogTag, rel_err.c_str());
      markAborted();
      return false;
    }
  }

  bool still;
  if (is_real_)
    still = runHwReversePhase0(time, jointPosWBC, out_cmd);
  else
    still = runSimReversePhase0(time, out_cmd);

  if (still)
    return true;
  if (aborted_)
    return false;
  // reverse done：保持 phase_ 非 IDLE（会话仍 active，供段1 /bot_stand_up_complete 判定）
  return false;
}

}  // namespace humanoid_controller
