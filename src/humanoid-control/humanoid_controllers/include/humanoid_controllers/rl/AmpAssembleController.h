#pragma once
#include "humanoid_controllers/rl/RLControllerBase.h"
#include "humanoid_controllers/rl/AmpFamilyPolicy.h"
#include <array>
#include "humanoid_controllers/rl/RunThermalGuard.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <kuavo_msgs/robotWaistControl.h>
#include <kuavo_msgs/ControllerSwitchEvent.h>
#include <humanoid_plan_arm_trajectory/RobotActionState.h>

namespace humanoid_controller {
// Owns policy selection and child lifetime. Joint-command blending is owned by
// humanoidController; a child is never updated twice by this wrapper.
class AmpAssembleController final : public RLControllerBase {
 public:
  using Role = AmpFamilyPolicy::Role;
  AmpAssembleController(const std::string&, const std::string&, ros::NodeHandle&,
                        ocs2::humanoid::TopicLogger*);
  ~AmpAssembleController() override;
  bool initialize() override;
  bool reload() override;
  void start() override;
  void stop() override;
  void pause() override;
  void resume() override;
  void resumeWarm() override;
  void reset() override;
  void advance(bool outer_blend_active);
  bool ownsMember(const std::string&) const;
  bool requestMember(const std::string&);
  // nullopt rejects without side effects; otherwise value reports exit readiness.
  std::optional<bool> requestFamilyExit(bool to_rl = false);
  bool canSwitchToMpc() const;
  void cancelFamilyExit();
  bool familyExitReady() const;
  void notifyExternalControl();
  bool walkingAllowed() const { return walking_allowed_.load(); }
  bool externalAllowed() const { return external_allowed_.load(); }
  RLControllerBase* getExecutionController() override { return child(); }
  SwitchBlendPair takeSwitchBlendPair() override;
  void finishSwitchBlend(bool success) override;
  std::string activeMemberName() const;
  void setCommandBufferCallback(std::function<bool()> callback) override;
  void setExternalCommandBufferCallback(std::function<bool()> callback) override;
  void updateVelocityLimitsParam(ros::NodeHandle& nh) override;
  bool supportsExternalControlRequests() const override { return member(Role::Manipulate) != nullptr; }
  bool requestArmControlMode(int mode) override;
  int getArmControlModeOnControllerActivate() const override { return 1; }
  void onInterpolationComplete() override { child()->onInterpolationComplete(); }
  void waitForNextCycle() override { child()->waitForNextCycle(); }
  bool supportsWalkingSwitch() const override { return child()->supportsWalkingSwitch(); }
  bool isReadyForInterpolation() const override { return child()->isReadyForInterpolation(); }
  bool requiresEmergencyStop() const override { return child()->requiresEmergencyStop(); }
  bool requestToExit() const override { return child()->requestToExit(); }
  bool isAllowToExit() const override { return child()->isAllowToExit(); }
  bool supportsWalkingPhaseSyncSwitch() const override { return child()->supportsWalkingPhaseSyncSwitch(); }
  bool hasValidWalkingPhase() const override { return child()->hasValidWalkingPhase(); }
  double getWalkingPhaseRad() const override { return child()->getWalkingPhaseRad(); }
  double getWalkingFrequencyHz() const override { return child()->getWalkingFrequencyHz(); }
  void setExternalPhaseOverride(bool on, double s, double c, double f) override { child()->setExternalPhaseOverride(on,s,c,f); }
  void resetGaitCommandState(bool stance = true) override { child()->resetGaitCommandState(stance); }
  bool getGaitCommandState(ocs2::humanoid::CommandDataRL& c) const override { return child()->getGaitCommandState(c); }
  void setGaitCommandState(const ocs2::humanoid::CommandDataRL& c) override { child()->setGaitCommandState(c); }
  void setSwitchVelocityScale(double scale) override { child()->setSwitchVelocityScale(scale); }
  bool hasNearZeroGaitCommand(double l, double a) const override { return child()->hasNearZeroGaitCommand(l,a); }
  bool isInPlaceSteppingActive() const override { return child()->isInPlaceSteppingActive(); }
  bool isInPlaceWalkingCommand(double l, double a) const override { return child()->isInPlaceWalkingCommand(l,a); }
  bool allowsWalkingDuringArmAction() const noexcept override { return child()->allowsWalkingDuringArmAction(); }
  bool hasActiveArmActionSession() const noexcept override { return child()->hasActiveArmActionSession(); }
  void abortActiveArmActionSessionForSafety() override { child()->abortActiveArmActionSessionForSafety(); }
  ArmController* getArmController() override { return child()->getArmController(); }
  const ArmController* getArmController() const override { return child()->getArmController(); }
  const Eigen::VectorXd& getInitialState() const override { return child()->getInitialState(); }
  Eigen::VectorXd getDefaultJointPos() const override { return child()->getDefaultJointPos(); }
  Eigen::VectorXd getDefaultBaseState() const override { return child()->getDefaultBaseState(); }
  double getDefaultBaseHeightControl() const override { return child()->getDefaultBaseHeightControl(); }
  double getDefaultBaseXOffsetControl() const override { return child()->getDefaultBaseXOffsetControl(); }
  Eigen::VectorXd getCurrentJointReference() const override { return child()->getCurrentJointReference(); }
  const Eigen::VectorXd& getJointKpVector() const override { return child()->getJointKpVector(); }
  const Eigen::VectorXd& getJointKdVector() const override { return child()->getJointKdVector(); }
  const Eigen::VectorXd& getJointTorqueLimits() const override { return child()->getJointTorqueLimits(); }
  const Eigen::VectorXd& getJointControlModes() const override { return child()->getJointControlModes(); }
  const Eigen::VectorXd& getJointPdModes() const override { return child()->getJointPdModes(); }
  bool getUseInterpolateFromMPC() const override { return child()->getUseInterpolateFromMPC(); }
  bool getUseDefaultMotorCspKpkd() const override { return child()->getUseDefaultMotorCspKpkd(); }
  double getControlFrequency() const override { return child()->getControlFrequency(); }
 protected:
  bool loadConfig(const std::string&) override;
  bool updateImpl(const ros::Time&, const SensorData&, const Eigen::VectorXd&, kuavo_msgs::jointCmd&) override;
  void preprocessSensorData(SensorData&) override {} // Child filters exactly once.
 private:
  RLControllerBase* child() const { return active_.load(); }
  RLControllerBase* member(Role r) const { return members_[static_cast<size_t>(r)].get(); }
  AmpFamilyPolicy::Input inputSnapshot() const;
  void refreshCommandGates(const AmpFamilyPolicy::Input&);
  void publishMember(const std::string& source, bool transitioning);
  void publishStatus(const AmpFamilyPolicy::Input&, const char* reason = nullptr, bool force = false);
  void manualCycleCallback(const std_msgs::Empty::ConstPtr&);
  void softCallback(const std_msgs::Empty::ConstPtr&);
  // Restore only an impedance mode that this family left for a cancelled exit.
  std::optional<int> soft_exit_arm_mode_;
  std::array<std::unique_ptr<RLControllerBase>,5> members_;
  std::atomic<RLControllerBase*> active_{nullptr};
  mutable std::mutex policy_mutex_, input_mutex_;
  // Serializes member preparation with ROS-side pause/reset. Never held during update().
  mutable std::recursive_mutex lifecycle_mutex_;
  AmpFamilyPolicy policy_;
  void thermalTick(const ros::WallTimerEvent&);
  mutable std::mutex thermal_mutex_;
  RunThermalGuard thermal_;
  bool thermal_enabled_=false;
  std::vector<int> thermal_indices_;
  double thermal_walk_speed_=0.6, thermal_exit_timeout_=5;
  std::atomic<double> thermal_exit_since_{-1};
  std::atomic<bool> thermal_timed_out_{false};
  double thermal_last_tick_=-1;
  std::atomic<bool> thermal_running_{false};
  ros::WallTimer thermal_timer_, thermal_status_timer_;
  ros::Publisher thermal_pub_;
  std::array<int, 2> elbow_joint_indices_{{16, 23}};
  // Protected by input_mutex_; receipt only advances for a distinct sensor timestamp.
  double elbow_sample_time_ = 0.0, elbow_received_time_ = 0.0;
  std::array<double, 2> elbow_position_{{std::numeric_limits<double>::quiet_NaN(),
                                       std::numeric_limits<double>::quiet_NaN()}};
  SwitchBlendPair pending_pair_;
  RLControllerBase* blend_source_ = nullptr;
  std::atomic<bool> walking_allowed_{false}, external_allowed_{false};
  geometry_msgs::Twist command_;
  ros::Time command_stamp_, gait_stamp_, external_stamp_, action_stamp_;
  bool gait_walk_ = false, action_active_ = false;
  double command_timeout_ = 0.5, external_timeout_ = 0.5;
  double switch_duration_ = 0.5;
  double stop_speed_ = 0.3;
  std::atomic<bool> pending_external_mode_{false};
  std::atomic<bool> external_handoff_pending_{false};
  std::function<bool()> upstream_walk_gate_, upstream_external_gate_;
  std::vector<ros::Subscriber> subscribers_;
  ros::ServiceServer walk_waq_service_;
  ros::Publisher member_pub_, member_event_pub_, status_pub_, controller_type_pub_;
  ros::WallTime last_status_publish_;
};
} // namespace humanoid_controller
