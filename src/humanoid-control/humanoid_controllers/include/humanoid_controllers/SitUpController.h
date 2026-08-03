#pragma once

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_srvs/Trigger.h>
#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <Eigen/Dense>

#include "kuavo_msgs/jointCmd.h"
#include "humanoid_controllers/SitControlManager.h"
#include "humanoid_wbc/WbcBase.h"

// 前向声明，避免头里拖入 MRT / Drake 重依赖（.cpp 内 #include 实体头）
namespace ocs2 { class MRT_ROS_Interface; }
namespace HighlyDynamic { class HumanoidInterfaceDrake; }

namespace humanoid_controller {

// SitControlManager 通过 ocs2 命名空间继承 vector_t / scalar_t；这里带入作用域。
using ocs2::vector_t;

/** 从坐姿 CSP hold 起身（段0 reverse → await start → 段1 sit→stand）。
 *
 * ROS 入口留主类。调用序：beginStandUpFromSeat（门禁需 hold）→ 主类重入 preUpdate /
 * release hold / pause MPC → kickHwReverseIfNeeded。/bot_stand_up_complete 由主类发布。 */
class SitUpController {
 public:
  using Snapshot = SitControlManager::StandUpFromSeatStartSnapshot;
  using SeatJointTargets = SitControlManager::SeatJointTargets;

  SitUpController(ros::NodeHandle& nh, SitControlManager& scm);

  /** 仅建 hardware/seat_return_reverse_prep client。对外 ROS 入口留主类。 */
  void setupRos();

  /** 依赖注入：humanoidController init 后调一次（MRT / Drake / 起立 WBC 由主类持有，本类借用，不夺所有权）。 */
  void setMrt(ocs2::MRT_ROS_Interface* mrt) { mrt_ = mrt; }
  void setDrake(HighlyDynamic::HumanoidInterfaceDrake* drake) { drake_ = drake; }
  void setWbc(std::shared_ptr<ocs2::humanoid::WbcBase> wbc) { standUpWbc_ = std::move(wbc); }

  /** preUpdate 重入回调：主类注入 beginSeatReturnToPreUpdate，
   *  供 seatReturnToPreUpdateCallback 服务在 beginStandUpFromSeat 成功后调用。 */
  void setSeatReturnPreUpdateHook(std::function<void()> hook) { seat_return_hook_ = std::move(hook); }

  /** 段0 起点：release CSP hold 前的实测关节（主类 jointPosWBC_）。须在 beginStandUpFromSeat
   *  前注入——begin 时即用其建 measured_start + leg_first/arm_clear mid + dur，与原码一致。 */
  void setMeasuredStart(const Eigen::VectorXd& jointPosWBC);

  /** 关节规格（leg/waist/arm/head 数量 + kp/kd/tau_max，从 humanoidController 注入只读副本）。
   *  control_modes 段0/段1 均为 mode=2（CSP/位置控制），不需注入。 */
  void setJointSpecs(int leg_num, int waist_num, int arm_num, int head_num,
                      const Eigen::VectorXd& joint_kp, const Eigen::VectorXd& joint_kd,
                      const std::vector<double>& max_current,
                      bool is_real);

  /** 段1 sit→stand 完成所需：站立完成时间 + MPC init 延迟 + head_down pitch（均借 SCM 配置）。 */
  void setRobotStandUpCompleteTime(double t) { robot_stand_up_complete_time_ = t; }
  double robotStandUpCompleteTime() const { return robot_stand_up_complete_time_; }

  // ── 主入口（humanoidController::preUpdate 顶部调）──
  /** 座椅起身会话进行中（段0/await/段1），等价旧 stand_up_from_seat_active_。 */
  bool isActive() const { return phase_ != Phase::IDLE; }
  bool isReverseDone() const { return reverse_seat_offset_done_; }
  bool isAwaitingStart() const { return awaiting_stand_up_start_; }
  bool isAborted() const { return aborted_; }

  /** 段0 reverse + await-start。
   *  @return true=仍在段0/await；false=结束（查 isAborted：DONE 时 isReverseDone，phase 保持至 resetOnComplete）。 */
  bool runPhase0(const ros::Time& time, const Eigen::VectorXd& jointPosWBC,
                 kuavo_msgs::jointCmd& out_cmd);

  /** 门禁+规划+进 PHASE0_REVERSE。不 kick HW、不发 done（主类 release/pause 后再 kick）。 */
  bool beginStandUpFromSeat(std::string& err);
  /** 主类 release hold 后调用：实机启动 HW reverse。 */
  void kickHwReverseIfNeeded();
  /** 主类 release 失败时回滚。 */
  void clearPlan();

  void resetOnComplete();

  /** 起身期低头 pitch（rad，正=低头）。座椅 sit_up / use_sit_init_boot 全程低头；
   *  进 MPC 站立时由主类清 keep_head_down 并抬头。 */
  double headDownPitchRadAt(const ros::Time& time) const;

  /** 段0 abort（HW reverse 失败）：主类发布 /bot_stand_up_complete(-1) 后 return。
   *  本类只置 aborted_ 标志并归 IDLE，不直接发 topic（核心 publish 留主类，避免出问题找不到）。 */
  void markAborted();

 private:
  enum class Phase : int {
    IDLE,
    PHASE0_REVERSE,      // 段0：reverse 回 sit（实机 HW / 仿真 CSP 复刻）
    PHASE0_HOLD_AWAIT,  // 段0 完成 hold，等 /hardware/is_ready
  };

  /** 实机段0 HW reverse 状态（仿真不用）。 */
  enum class HwSeatReverse : int { Idle = 0, Handoff = 1, Running = 2, Done = 3, Failed = 4 };

  bool runHwReversePhase0(const ros::Time& time, const Eigen::VectorXd& jointPosWBC,
                          kuavo_msgs::jointCmd& out_cmd);  // is_real_==true
  bool runSimReversePhase0(const ros::Time& time, kuavo_msgs::jointCmd& out_cmd);  // is_real_==false

  /** 段0 完成后：进入/轮询 await start。
   *  @return true 仍在等待（本帧应 return）；false 已切到 sit→stand（reverse_done 已置）。 */
  bool tickAwaitStandUpStart(bool with_sit_csp_hold, kuavo_msgs::jointCmd& out_cmd);

  void publishSeatCspHoldFromTargets(const SeatJointTargets& targets,
                                     kuavo_msgs::jointCmd& out_cmd);

  HwSeatReverse hwSeatReverse() const { return static_cast<HwSeatReverse>(hw_seat_reverse_state_.load()); }
  void setHwSeatReverse(HwSeatReverse s) { hw_seat_reverse_state_.store(static_cast<int>(s)); }

  void kickHwReverseThread();

  ros::NodeHandle& nh_;
  SitControlManager& scm_;
  ocs2::MRT_ROS_Interface* mrt_{nullptr};
  HighlyDynamic::HumanoidInterfaceDrake* drake_{nullptr};
  std::shared_ptr<ocs2::humanoid::WbcBase> standUpWbc_;

  // preUpdate 重入钩子：主类注入 beginSeatReturnToPreUpdate，
  // 供 /humanoid_controller/seat_return_to_preupdate 服务在 beginStandUpFromSeat 成功后调用。
  std::function<void()> seat_return_hook_;

  ros::ServiceClient seat_return_reverse_prep_client_;

  // ── 段0 状态（从 humanoidController 迁入，语义不变）──
  Phase phase_{Phase::IDLE};
  Snapshot snapshot_;                          // stand_up_from_seat_start_snapshot_
  bool reverse_seat_offset_done_{false};        // 段0 reverse 完成，可进段1
  bool awaiting_stand_up_start_{false};         // 段0 完成 hold，等 /hardware/is_ready
  bool aborted_{false};                         // 段0 HW reverse 失败（一次性）
  int  reverse_seat_settle_stage_{0};           // 段0 子阶段：0=0a,1=0b,2=0c（仿真 CSP 路径）
  double reverse_seat_offset_start_time_sec_{0.0};
  double hold_at_sit_complete_time_sec_{0.0};   // 段0 settle 完成时刻，供 hold_delay 计时
  std::atomic<int> hw_seat_reverse_state_{static_cast<int>(HwSeatReverse::Idle)};
  std::string hw_seat_reverse_err_;
  double robot_stand_up_complete_time_{0.0};

  // ── 关节规格（只读副本，避免每帧传参）──
  int leg_n_{12}, waist_n_{1}, arm_n_{0}, head_n_{2};
  Eigen::VectorXd joint_kp_, joint_kd_, measured_start_;  // measured_start_ = 注入的 jointPosWBC_
  std::vector<double> max_current_;
  bool is_real_{false};
};

}  // namespace humanoid_controller
