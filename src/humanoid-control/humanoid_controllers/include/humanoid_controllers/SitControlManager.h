#pragma once

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_srvs/Trigger.h>
#include <array>
#include <chrono>
#include <Eigen/Dense>
#include <string>
#include "kuavo_common/common/common.h"
#include "kuavo_common/common/seat_config.h"
#include "kuavo_msgs/jointCmd.h"

namespace ocs2 {
class MRT_ROS_Interface;
}

namespace humanoid_controller {

/** P1 落座 / P2 落座终点冻结 / P3 关节偏置 / P4 收脚。仅 sit_down 序列使用。 */
class SitControlManager {
 public:
  static constexpr size_t kLegDof = 12;
  static constexpr size_t kArmDof = 14;
  static constexpr size_t kMpcLegStateStart = 12;
  static constexpr size_t kMpcWaistStateIdx = kMpcLegStateStart + kLegDof;
  static constexpr size_t kLegActionSteps = 3;

  /** Kuavo5 代际（主版本 5，如 50–59）支持座椅 P1/P2/P3/P4 与坐姿启动 */
  static bool robotSupportsSeatControl(const RobotVersion& version) { return version.start_with(5); }

  enum class Phase : int { P1_STAND = 1, P2_SIT_HOLD = 2, P3_OFFSET = 3, P4_LEG_ACTION = 4 };

  struct SeatJointTargets {
    bool valid = false;
    double alpha = 0.0;
    std::array<double, kLegDof> leg{};
    double waist = 0.0;
    std::array<double, kArmDof> arm{};
  };

  SitControlManager();
  void setMrtInterface(ocs2::MRT_ROS_Interface* mrt) { mrt_ = mrt; }
  /**
   * use_sit_init 实机启动：从 hardware_sit_pose_prep 发布 /hardware_prep_*（在 setupRos 之前调用）。
   * @return 是否实际启用坐姿启动（use_sit_init 且 Kuavo5）
   */
  static bool configureLaunchBoot(bool use_sit_init, const RobotVersion& robot_version,
                                  const Eigen::VectorXd& sit_mpc_state, int hw_waist_joints,
                                  int hw_leg_joints, int hw_arm_joints, int hw_joint_count,
                                  const kuavo_common::SeatConfig& seat);
  /** 与 configureLaunchBoot 结果同步实例标志（humanoidController 在 SCM 创建后调用） */
  void setSeatConfig(const kuavo_common::SeatConfig& seat) { seat_config_ = &seat; }
  void syncBootFlags(bool use_sit_init_boot);
  void setupRos(ros::NodeHandle& nh);

  bool useSitInitBoot() const { return use_sit_init_boot_; }
  bool hasSitBootLegGains() const { return has_sit_boot_leg_gains_; }
  bool sitBootLegGain(size_t leg_index, double& kp, double& kd) const;

  /** preUpdate 起立插值起点：坐姿启动用 sit，否则用 squat（仅填充前 12+jointNum 维） */
  Eigen::VectorXd makeBootStartState(const Eigen::VectorXd& sit_state, const Eigen::VectorXd& squat_state,
                                     size_t wbc_state_dim, size_t mpc_leg_waist_dim) const;

  /** 将 t∈[0,1] 映射为 smoothstep：3t² - 2t³（与 P3 seat_offset 一致） */
  static double smoothstep01(double t);

  double sitToStandComVelocityMps() const { return sit_to_stand_com_velocity_mps_; }

  /** 起立轨迹结束后、init MPC 前的稳定等待（秒） */
  double mpcInitDelayAfterStandUpSec() const { return mpc_init_delay_after_stand_up_sec_; }

  /** 起立接触保护：robotStandUpCompleteTime_ 后的额外容忍时间（秒） */
  double contactProtectGraceAfterStandUpSec() const { return contact_protect_grace_after_stand_up_sec_; }

  /** 反向 seat_offset 完成后、sit→stand 前的 sit 姿态保持时间（秒）；0=立即起立 */
  double holdAtSitPoseBeforeStandUpSec() const { return hold_at_sit_pose_before_stand_up_sec_; }

  /** P3 首帧：用硬件/仿真实测 14 臂角覆盖基准（补全 MPC 冻结态可能不足的臂维） */
  void seedArmBaselineFromHardware(const Eigen::VectorXd& arm_hw_pos);

  SeatJointTargets update();

  /** 每帧控制策略（一次性查询，供 humanoidController 决策） */
  struct SeatPolicy {
    bool active{false};           // P3 offset 或 P4 leg action 激活
    bool wbc_bypass{false};       // 跳过手臂轨迹 + 滤波
    bool offset_running{false};   // P3 仍在 WBC 插值中，选 sitDownWbc_
    bool csp_hold{false};         // CSP 锁定中，跳过踝力控 / 行走 KpKd / 头部反馈
    bool policy_frozen{false};    // MPC 策略已冻结，用 frozen 参考代 MPC 输出
    bool use_frozen_ref{false};   // 当前帧是否应使用 frozen 参考（含 hold counter 过渡）
    double offset_alpha{0.0};     // P3 偏置进度 α∈[0,1]
  };
  SeatPolicy policy() const;

  /** P3/P4 CSP hold 踝关节堵转检测（每帧调，在 jointCmd 填充后、发送前）。
   *  @param ankle_torques 4 维：左踝(4,5) + 右踝(10,11) 出力扭矩
   *  @return true 表示应立即停机 + 解冻 MPC */
  bool checkAnkleStall(const Eigen::Ref<const Eigen::VectorXd>& ankle_torques);
  void resetAnkleStallDetector();
  double stallTorqueThreshold() const { return stall_torque_threshold_; }

  /** 恢复 MPC（unfreeze 后调用） */
  void resumeMpc();

  /** P1 落座完成后是否已到达最终 CSP hold（P3 无 P4，或 P4 完成） */
  bool isSitSequenceComplete() const { return sit_sequence_complete_; }

  /** stand_up 开始前：快照 CSP 锁定姿关节目标，供反向 seat_offset 插值 */
  struct StandUpFromSeatStartSnapshot {
    bool valid = false;
    SeatJointTargets csp_hold_joint_targets{};
    /** 反向 seat_offset 插值时长（秒），通常等于 seat_offset_duration_seconds */
    double reverse_seat_offset_duration_sec = 1.0;
    bool use_smoothstep = true;
  };

  /**
   * 在 releaseSeatResources 之前调用：捕获最终 CSP 锁定姿关节目标。
   * @return false 时 err 说明拒绝原因（与 releaseSeatHoldForStandUp 前置条件一致）
   */
  bool captureStandUpFromSeatStartSnapshot(StandUpFromSeatStartSnapshot& out, std::string& err) const;

  /**
   * 落座序列到达最终 CSP 锁定姿后，释放座椅资源，供 humanoidController 进入 stand_up preUpdate。
   * @return false 时 err 说明拒绝原因（序列未完成或仍在运动中）
   */
  bool releaseSeatHoldForStandUp(std::string& err);

  /** 每帧存储当前观测 + 推进冻结 hold counter（替代 setCurrentMrtState + tickFrozenHoldCounter） */
  void tickFrame(double time, const Eigen::VectorXd& obs_state, const Eigen::VectorXd& cur_input, size_t mode);

  // ── P3/P4 实机 CSP（mode=2 + EC PDO kp/kd；腿关节优先 seat_boot 刚度；仿真不特殊处理）──
  void applyRealHardwareCspHold(kuavo_msgs::jointCmd& msg, const Eigen::VectorXd& leg_waist_kp,
                                const Eigen::VectorXd& leg_waist_kd, int leg_waist_count, int arm_start,
                                int arm_count) const;

  /** P3/P4：更新偏置或腿部序列；RUNNING 写 WBC 关节参考，CSP 阶段写 output 并 skip_wbc=true */
  struct SeatOffsetStep {
    SeatJointTargets targets{};
    bool active{false};
    bool running{false};
    bool skip_wbc{false};
  };
  SeatOffsetStep stepSeatOffset(double dt, Eigen::Ref<Eigen::VectorXd> wbc_joint_q,
                                Eigen::Ref<Eigen::VectorXd> wbc_joint_v, Eigen::Ref<Eigen::VectorXd> hw_pos,
                                Eigen::Ref<Eigen::VectorXd> hw_vel, Eigen::Ref<Eigen::VectorXd> hw_tau,
                                size_t leg_dof, size_t waist_num, size_t arm_num);

  bool isSeatOffsetRunning() const { return state_ == State::OFFSET_RUNNING; }
  bool isSeatLegActionRunning() const {
    return state_ == State::LEG_ACTION_STEP0_RUNNING || state_ == State::LEG_ACTION_STEP1_RUNNING ||
           state_ == State::LEG_ACTION_STEP2_RUNNING;
  }
  /** P3 偏置 WBC 完成后 CSP 锁位 */
  bool isSeatOffsetCspHold() const { return state_ == State::OFFSET_DONE; }
  /** P3 CSP 或 P4 全程（含三步插值与 P4 完成） */
  bool isSeatCspHold() const {
    return state_ == State::OFFSET_DONE || state_ == State::LEG_ACTION_STEP0_RUNNING ||
           state_ == State::LEG_ACTION_STEP1_RUNNING || state_ == State::LEG_ACTION_STEP2_RUNNING ||
           state_ == State::LEG_ACTION_DONE;
  }
  bool isSeatLegActionActive() const {
    return state_ == State::LEG_ACTION_STEP0_RUNNING || state_ == State::LEG_ACTION_STEP1_RUNNING ||
           state_ == State::LEG_ACTION_STEP2_RUNNING || state_ == State::LEG_ACTION_DONE;
  }
  bool isSeatOffsetActive() const;
  /** P2/P3/P4 均使用落座终点冻结的 MPC 参考（evaluatePolicy 不更新） */
  bool isPolicyFrozen() const {
    return state_ == State::FROZEN || isSeatOffsetActive() || isSeatLegActionActive();
  }
  /** P2→P1 unfreeze 后、新 policy 到达前继续用冻结参考，避免 WBC 跳变 */
  bool shouldHoldFrozenMpcReference() const { return frozen_hold_counter_ != 0; }
  void notifyMpcPolicyUpdated();
  void tickFrozenHoldCounter();
  Phase getPhase() const;
  double getOffsetAlpha() const { return alpha_; }
  double getLegActionAlpha() const { return leg_action_alpha_; }
  bool isLegActionEnabled() const { return leg_action_config_ok_; }

  const Eigen::VectorXd& frozenMpcState() const { return frozen_mpc_state_; }
  const Eigen::VectorXd& frozenMpcInput() const { return frozen_mpc_input_; }
  size_t frozenMpcMode() const { return frozen_mpc_mode_; }

  /** P3 规划用：α∈[0,1] 时 frozen + α·offset 的关节目标（与 update/buildTargets 一致） */
  SeatJointTargets offsetTargetsAt(double alpha) const { return buildTargets(alpha); }

 private:
  enum class State {
    IDLE,
    FROZEN,
    OFFSET_RUNNING,
    OFFSET_DONE,
    LEG_ACTION_STEP0_RUNNING,
    LEG_ACTION_STEP1_RUNNING,
    LEG_ACTION_STEP2_RUNNING,
    LEG_ACTION_DONE
  };

  bool loadConfig();
  SeatJointTargets buildTargets(double alpha) const;
  SeatJointTargets buildLegActionTargets(double alpha) const;
  void writeSeatTargetsWbc(Eigen::Ref<Eigen::VectorXd> joint_q, Eigen::Ref<Eigen::VectorXd> joint_v,
                           const SeatJointTargets& targets, size_t leg_dof, size_t waist_num, size_t arm_num) const;
  void writeSeatTargetsHw(Eigen::Ref<Eigen::VectorXd> hw_pos, Eigen::Ref<Eigen::VectorXd> hw_vel,
                          Eigen::Ref<Eigen::VectorXd> hw_tau, const SeatJointTargets& targets, size_t leg_dof,
                          size_t waist_num, size_t arm_num) const;
  double alphaFromElapsed(double elapsed_sec, double duration_sec, bool use_smoothstep) const;
  bool startOffset(std::string& err);
  bool startLegAction(std::string& err);
  void resetOffset();
  void resetLegAction();
  void publishPhaseParam() const;
  void resetMpcAfterSitPause(const char* reason);
  void releaseSeatResources();
  void markSitSequenceComplete();
  bool tickAutoAdvance();
  SeatJointTargets updateLegAction();
  void advanceLegActionStep();
  int legActionStepIndex() const;
  int legActionTargetStepIndex() const;

  void onSitDownComplete(const std_msgs::Int8::ConstPtr& msg);
  void onSitUnfreeze(const std_msgs::Int8::ConstPtr& msg);

  bool freeze_mrt_on_sit_complete_{true};
  /** P1 落座冻结后自动串联 P3/P4（单次 sit_down） */
  bool sit_auto_advance_active_{false};
  bool sit_sequence_complete_{false};
  bool config_ok_{false};
  bool leg_action_config_ok_{false};
  bool use_sit_init_boot_{false};
  bool has_sit_boot_leg_gains_{false};
  double boot_prep_speed_deg_{8.0};
  double sit_to_stand_com_velocity_mps_{0.08};
  double mpc_init_delay_after_stand_up_sec_{0.8};
  double contact_protect_grace_after_stand_up_sec_{0.5};
  double hold_at_sit_pose_before_stand_up_sec_{0.0};
  std::vector<double> sit_boot_leg_kp_;
  std::vector<double> sit_boot_leg_kd_;
  State state_{State::IDLE};
  static constexpr int kFrozenHoldAwaitingPolicy = -1;   ///< 等 MPC 生产新 policy
  static constexpr int kFrozenHoldFrames = 8;             ///< policy 到达后额外保持帧数
  /** 0=不保持; kFrozenHoldAwaitingPolicy=等 policy; >0=policy 后剩余保持帧数 */
  int frozen_hold_counter_{0};

  void loadSitBootLegGainsFromParam();

  double alpha_{0.0};
  double offset_duration_sec_{0.0};
  bool offset_smoothstep_{false};
  std::chrono::steady_clock::time_point offset_t0_{};

  double leg_action_alpha_{0.0};
  std::array<double, kLegActionSteps> leg_action_durations_{};
  bool leg_action_smoothstep_{true};
  std::array<std::array<double, kLegDof>, kLegActionSteps> leg_action_step_rad_{};
  std::array<double, kLegDof> leg_action_leg_start_{};
  SeatJointTargets p3_end_targets_{};
  std::chrono::steady_clock::time_point leg_action_t0_{};

  Eigen::VectorXd frozen_mpc_state_;
  Eigen::VectorXd frozen_mpc_input_;
  size_t frozen_mpc_mode_{0};
  double cur_mpc_time_{0.0};
  Eigen::VectorXd cur_mpc_state_;
  Eigen::VectorXd cur_mpc_input_;
  size_t cur_mpc_mode_{0};

  std::array<double, kArmDof> arm_pos_{};
  std::array<double, kLegDof> leg_offset_rad_{};
  std::array<double, kArmDof> arm_offset_rad_{};

  ros::Subscriber sub_sit_complete_;
  ros::Subscriber sub_unfreeze_;

  ocs2::MRT_ROS_Interface* mrt_{nullptr};
  const kuavo_common::SeatConfig* seat_config_{nullptr};

  // ── 踝关节堵转检测 ──
  double stall_torque_threshold_{20.0};      // N·m，超过此值认为堵转
  int stall_consecutive_limit_{10};           // 连续超阈值帧数触发停机（~0.02s at 500Hz）
  int stall_consecutive_count_{0};
};

}  // namespace humanoid_controller
