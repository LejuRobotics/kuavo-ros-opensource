#include <pinocchio/fwd.hpp>
#include "humanoid_controllers/rl/AmpAssembleController.h"
#include "humanoid_controllers/rl/AmpFamilyMemberController.h"
#include "humanoid_controllers/rl/AmpWaqController.h"
#include "humanoid_controllers/rl/AmpWildImpController.h"
#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp>
#include "humanoid_controllers/rl/AmpFamilyConfig.h"
#include <kuavo_common/common/json.hpp>
#include <algorithm>
#include <iterator>
#include <limits>
#include <chrono>

namespace humanoid_controller {
namespace {
double thermalNow() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
bool recent(const ros::Time& stamp, const ros::Time& now, double duration) {
  return !stamp.isZero() && now >= stamp && (now - stamp).toSec() <= duration;
}
}
AmpAssembleController::AmpAssembleController(const std::string& name, const std::string& config,
    ros::NodeHandle& nh, ocs2::humanoid::TopicLogger* logger)
    : RLControllerBase(name, RLControllerType::AMP_CONTROLLER, config, nh, logger) {
  // The child has already performed arm/waist replacement and PDO gain mapping.
  arm_command_replacement_enabled_ = waist_command_replacement_enabled_ = false;
  use_default_motor_csp_kpkd_ = true;
}
AmpAssembleController::~AmpAssembleController() { stop(); }
bool AmpAssembleController::loadConfig(const std::string& file) {
  const auto config = AmpFamilyConfig::parse(YAML::LoadFile(file), name_);
  policy_ = AmpFamilyPolicy(config.switching);
  thermal_ = RunThermalGuard(config.thermal);
  thermal_enabled_=config.thermal.enabled; thermal_indices_=config.thermal_indices;
  thermal_walk_speed_=config.thermal_walk_speed; thermal_exit_timeout_=config.thermal_exit_timeout;
  elbow_joint_indices_ = config.switching.elbow_joint_indices;
  stop_speed_ = config.switching.manipulation_max_speed;
  command_timeout_ = config.command_timeout;
  external_timeout_ = config.external_timeout;
  switch_duration_ = config.blend_duration;
  const auto directory = boost::filesystem::path(file).parent_path();
  for (size_t i = 0; i < members_.size(); ++i) {
    const auto& spec = config.members[i];
    if (!spec.enabled) continue;
    const auto& name = spec.name;
    auto path = boost::filesystem::path(spec.config_file);
    if (!path.is_absolute()) path = directory / path;
    std::unique_ptr<AmpFamilyMemberController> model;
    if (i == static_cast<size_t>(Role::Soft))
      model = std::make_unique<AmpWildImpController>(name, path.string(), nh_, ros_logger_);
    else if (i == static_cast<size_t>(Role::WalkWaq))
      model = std::make_unique<AmpWaqController>(name, path.string(), nh_, ros_logger_);
    else
      model = std::make_unique<AmpFamilyMemberController>(name, path.string(), nh_, ros_logger_);
    model->setFamilyObservationOwner([this, child = model.get()]() { return active_.load() == child; });
    if (!model->initialize())
      throw std::runtime_error("AMP member initialization failed: " + name);
    model->setRobotActionStateEnabled(config.switching.enabled[static_cast<size_t>(Role::Manipulate)]);
    if (i == static_cast<size_t>(Role::Run)) {
      model->setForwardVelocitySmoothing(config.run_decel_tau, config.run_accel_tau);
      model->configureRunElbowInterpolation(config.elbow_interpolation, elbow_joint_indices_);
    }
    members_[i] = std::move(model);
  }
  if (config.switching.run_to_walk_elbow_guard) {
    const int first_arm = jointNum_ + waistNum_;
    const int end_arm = first_arm + jointArmNum_;
    if (std::any_of(elbow_joint_indices_.begin(), elbow_joint_indices_.end(),
                    [first_arm, end_arm](int index) { return index < first_arm || index >= end_arm; }))
      throw std::invalid_argument("AMP elbow guard index must refer to an arm joint");
  }
  active_.store(member(Role::Walk));
  return true;
}
bool AmpAssembleController::initialize() {
  try {
    if (!loadConfig(config_file_)) return false;
    member_pub_ = nh_.advertise<std_msgs::String>("/humanoid_controller/amp_family/active_member", 1, true);
    status_pub_ = nh_.advertise<std_msgs::String>("/humanoid_controller/amp_family/status", 1, true);
    controller_type_pub_ = nh_.advertise<std_msgs::Int8>("/humanoid_controller/amp_family/controller_type", 1, true);
    member_event_pub_ = nh_.advertise<kuavo_msgs::ControllerSwitchEvent>("/humanoid_controller/amp_family/switch_event", 10);
    subscribers_.push_back(nh_.subscribe<geometry_msgs::Twist>("/cmd_vel", 10,
      [this](const geometry_msgs::Twist::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(input_mutex_);
        if (!std::isfinite(msg->linear.x) || !std::isfinite(msg->linear.y) || !std::isfinite(msg->angular.z)) {
          command_ = geometry_msgs::Twist(); command_stamp_ = ros::Time(); return;
        }
        command_ = *msg; command_stamp_ = ros::Time::now();
      }));
    subscribers_.push_back(nh_.subscribe<std_msgs::String>("/humanoid_mpc_gait_name_request", 10,
      [this](const std_msgs::String::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(input_mutex_);
        gait_walk_ = msg->data == "walk" || msg->data == "trot";
        gait_stamp_ = ros::Time::now();
      }));
    subscribers_.push_back(nh_.subscribe<std_msgs::Float64MultiArray>("/humanoid/mpc/arm_control_mode", 10,
      [this](const std_msgs::Float64MultiArray::ConstPtr& msg) {
        if (!msg->data.empty() && (msg->data.front() == 2.0 || (msg->data.size() > 1 && msg->data[1] == 2.0)))
          notifyExternalControl();
      }));
    subscribers_.push_back(nh_.subscribe<kuavo_msgs::robotWaistControl>("/robot_waist_motion_data", 10,
      [this](const kuavo_msgs::robotWaistControl::ConstPtr&) { notifyExternalControl(); }));
    subscribers_.push_back(nh_.subscribe<std_msgs::Bool>("/humanoid_controller/enable_waist_control", 10,
      [this](const std_msgs::Bool::ConstPtr& msg) { if (msg->data) notifyExternalControl(); }));
    subscribers_.push_back(nh_.subscribe<humanoid_plan_arm_trajectory::RobotActionState>("/robot_action_state", 10,
      [this](const humanoid_plan_arm_trajectory::RobotActionState::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(input_mutex_);
        action_active_ = msg->state == 1; action_stamp_ = ros::Time::now();
      }));
    subscribers_.push_back(nh_.subscribe<std_msgs::Empty>("/humanoid_controller/soft_switch_request", 1,
      &AmpAssembleController::softCallback, this));
    subscribers_.push_back(nh_.subscribe<std_msgs::Empty>("/humanoid_controller/amp_family/cycle_request", 1,
      &AmpAssembleController::manualCycleCallback, this));
    walk_waq_service_ = nh_.advertiseService<std_srvs::Trigger::Request, std_srvs::Trigger::Response>(
      "/humanoid_controller/amp_family/switch_to_walk_waq",
      [this](std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
        auto* target = member(Role::WalkWaq);
        res.success = isActive() && target && requestMember(target->getName());
        res.message = res.success ? "WALK_WAQ request accepted; observe amp_family/status for completion"
                                  : "AMP family must be active and WALK_WAQ enabled, with no pending family exit";
        return true;
      });
    if(thermal_enabled_) {
      const auto cfg=AmpFamilyConfig::parse(YAML::LoadFile(config_file_),name_);
      thermal_pub_=nh_.advertise<std_msgs::String>("/humanoid_controller/amp_family/run_thermal_status",1,true);
      subscribers_.push_back(nh_.subscribe<std_msgs::Float64MultiArray>(cfg.thermal_topic,1,
        [this](const std_msgs::Float64MultiArray::ConstPtr& msg) {
          std::vector<double> values;
          std::transform(thermal_indices_.begin(), thermal_indices_.end(), std::back_inserter(values),
            [&msg](int i) {
              return static_cast<size_t>(i) < msg->data.size()
                  ? msg->data[i] : std::numeric_limits<double>::quiet_NaN();
            });
          std::lock_guard<std::mutex> lock(thermal_mutex_); thermal_.sample(values,thermalNow());
        }));
      thermal_timer_=nh_.createWallTimer(ros::WallDuration(0.05),&AmpAssembleController::thermalTick,this);
      // User-facing telemetry is 1 Hz; protection continues at 20 Hz.
      thermal_status_timer_=nh_.createWallTimer(ros::WallDuration(1.0),
        [this](const ros::WallTimerEvent&) {
          std_msgs::String status;
          {
            std::lock_guard<std::mutex> lock(thermal_mutex_);
            status.data=RunThermalGuard::statusMessage(
              thermal_.status(),thermal_timed_out_.load());
          }
          thermal_pub_.publish(status);
        });
    }
    setCommandBufferCallback({}); setExternalCommandBufferCallback({});
    initialized_ = true; state_ = ControllerState::PAUSED;
    publishStatus(inputSnapshot(), "inactive", true);
    return true;
  } catch (const std::exception& e) {
    ROS_ERROR("[%s] Cannot initialize AMP family: %s", name_.c_str(), e.what());
    stop(); return false;
  }
}
bool AmpAssembleController::reload() {
  ROS_WARN("[%s] Reload the complete AMP family by restarting the controller node", name_.c_str());
  return false; // Keep borrowed blend pointers stable throughout the node lifetime.
}
void AmpAssembleController::start() {
  std::lock_guard<std::recursive_mutex> lifecycle_lock(lifecycle_mutex_);
  for (auto& m : members_) if (m) m->start();
  state_ = ControllerState::PAUSED;
}
void AmpAssembleController::stop() {
  std::lock_guard<std::recursive_mutex> lifecycle_lock(lifecycle_mutex_);
  thermal_timer_.stop();
  thermal_status_timer_.stop();
  walk_waq_service_.shutdown();
  subscribers_.clear();
  for (auto& m : members_) if (m) m->stop();
  state_ = ControllerState::STOPPED;
  if (child()) publishStatus(inputSnapshot(), "inactive", true);
}
void AmpAssembleController::pause() {
  thermal_running_=false; thermal_exit_since_=-1; thermal_timed_out_=false;
  std::lock_guard<std::recursive_mutex> lifecycle_lock(lifecycle_mutex_);
  soft_exit_arm_mode_.reset();
  state_ = ControllerState::PAUSED;
  walking_allowed_ = external_allowed_ = false;
  { std::lock_guard<std::mutex> lock(input_mutex_); elbow_sample_time_ = elbow_received_time_ = 0.0; }
  for (auto& m : members_) if (m) m->pause();
  {
    std::lock_guard<std::mutex> lock(policy_mutex_);
    policy_.finishTransition(false, ros::Time::now().toSec());
    active_.store(member(policy_.activeRole()));
    pending_pair_ = {}; blend_source_ = nullptr;
  }
  if (child()) publishStatus(inputSnapshot(), "inactive", true);
}
void AmpAssembleController::reset() {
  thermal_running_=false; thermal_exit_since_=-1; thermal_timed_out_=false;
  std::lock_guard<std::recursive_mutex> lifecycle_lock(lifecycle_mutex_);
  soft_exit_arm_mode_.reset();
  for (auto& m : members_) if (m) { m->setGaitExecutionGate(false, true); m->reset(); }
  { std::lock_guard<std::mutex> input_lock(input_mutex_);
    gait_walk_ = false; gait_stamp_ = ros::Time();
    elbow_sample_time_ = elbow_received_time_ = 0.0;
  }
  std::lock_guard<std::mutex> lock(policy_mutex_);
  policy_.reset(ros::Time::now().toSec()); active_.store(member(Role::Walk));
  pending_pair_ = {}; blend_source_ = nullptr;
  pending_external_mode_ = false;
  external_handoff_pending_ = false;
}
void AmpAssembleController::resume() {
  std::lock_guard<std::recursive_mutex> lifecycle_lock(lifecycle_mutex_);
  reset(); child()->resume(); state_ = ControllerState::RUNNING;
  publishMember("", false);
  publishStatus(inputSnapshot(), nullptr, true);
}
void AmpAssembleController::resumeWarm() {
  std::lock_guard<std::recursive_mutex> lifecycle_lock(lifecycle_mutex_);
  // Actual entry uses resume() and resets to WALK. This path revives only a
  // prepared outgoing family for blending; SOFT must not re-enter impedance.
  if (child()->getType() == RLControllerType::AMP_WILD_IMP_CONTROLLER)
    static_cast<AmpFamilyMemberController*>(child())->AmpFamilyMemberController::resumeWarm();
  else child()->resumeWarm();
  state_ = ControllerState::RUNNING;
  publishStatus(inputSnapshot(), nullptr, true);
}
AmpFamilyPolicy::Input AmpAssembleController::inputSnapshot() const {
  AmpFamilyPolicy::Input in;
  const auto now = ros::Time::now(); in.now = now.toSec();
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    const bool action = member(Role::Manipulate) && action_active_ && recent(action_stamp_, now, external_timeout_);
    in.command_fresh = recent(command_stamp_, now, command_timeout_) && !action;
    in.command_x = command_.linear.x; in.command_y = command_.linear.y; in.command_yaw = command_.angular.z;
    in.walking_intent = gait_walk_ && !action; // Gait requests are latched until an explicit stance request.
    in.elbow_sample_time = elbow_sample_time_; in.elbow_received_time = elbow_received_time_;
    in.elbow_position = elbow_position_;
    in.external_control = recent(external_stamp_, now, external_timeout_) || action ||
                          pending_external_mode_.load() || external_handoff_pending_.load();
  }
  in.command_limits = static_cast<const AmpFamilyMemberController*>(child())->getFamilyCommandLimits();
  in.stance = child()->isAllowToExit();
  in.phase_valid = child()->hasValidWalkingPhase();
  in.body_speed = baseStateRL_.size() >= 9 ? baseStateRL_.segment<3>(6).norm() : std::numeric_limits<double>::quiet_NaN();
  if(thermal_enabled_) {
    std::lock_guard<std::mutex> lock(thermal_mutex_);
    const auto& status=thermal_.status();
    const bool fresh=status.valid && thermalNow() - thermal_last_tick_ <= 0.2;
    in.thermal_can_enter=status.can_enter && fresh;
    in.thermal_force_exit=status.force_exit || !fresh;
  }
  return in;
}
void AmpAssembleController::notifyExternalControl() {
  if (!isActive() || !member(Role::Manipulate)) return;
  std::lock_guard<std::mutex> lock(input_mutex_);
  external_stamp_ = ros::Time::now();
  // One-shot waist/arm targets must survive dwell and RUN -> WALK -> MANIPULATE.
  if (!external_allowed_) external_handoff_pending_ = true;
}
bool AmpAssembleController::ownsMember(const std::string& name) const {
  return std::any_of(members_.begin(), members_.end(),
                     [&name](const auto& m) { return m && m->getName() == name; });
}
std::vector<std::string> AmpAssembleController::getMemberNames() const {
  // Membership is immutable after initialization; reload requires a node restart.
  std::vector<std::string> names;
  for (const auto& model : members_)
    if (model) names.push_back(model->getName());
  return names;
}
bool AmpAssembleController::requestMember(const std::string& name) {
  for (size_t i = 0; i < members_.size(); ++i) if (members_[i] && members_[i]->getName() == name) {
    const auto role = static_cast<Role>(i);
    if (role == Role::Run && !inputSnapshot().thermal_can_enter) return false;
    const bool arm_action = child()->hasActiveArmActionSession();
    // Keep admission and latch cleanup atomic with member requests and input callbacks.
    std::lock_guard<std::mutex> policy_lock(policy_mutex_);
    std::lock_guard<std::mutex> input_lock(input_mutex_);
    const auto now = ros::Time::now();
    const bool action = member(Role::Manipulate) && action_active_ && recent(action_stamp_, now, external_timeout_);
    const bool external = recent(external_stamp_, now, external_timeout_) || action ||
        pending_external_mode_.load() || external_handoff_pending_.load();
    // SOFT ignores background operation intent: allow its explicit exit to
    // discard that latch, but never cancel a real action or manipulation handoff.
    const bool soft_exit = policy_.activeRole() == Role::Soft && !policy_.blending() &&
        !policy_.exiting() && policy_.requestedRole() != Role::Manipulate &&
        !action && !arm_action && !pending_external_mode_.load();
    if (role != Role::Manipulate && external && !soft_exit) return false;
    const bool accepted = policy_.request(role);
    if (accepted && role != Role::Manipulate) {
      external_handoff_pending_ = pending_external_mode_ = false;
      external_stamp_ = ros::Time();
    }
    return accepted;
  }
  return false;
}
std::optional<bool> AmpAssembleController::requestFamilyExit(bool to_rl) {
  std::lock_guard<std::mutex> lock(policy_mutex_);
  // Check and request under the same lock as transition commit: a concurrent
  // WALK -> RUN must not slip between permission checking and queuing the exit.
  if (policy_.blocksOrdinaryFamilyExit()) return std::nullopt;
  policy_.requestExit(to_rl);
  if (!policy_.directRlExit()) walking_allowed_ = false;
  external_allowed_ = false;
  external_handoff_pending_ = pending_external_mode_ = false;
  return policy_.exitReady();
}
void AmpAssembleController::cancelFamilyExit() {
  std::lock_guard<std::mutex> lock(policy_mutex_); policy_.cancelExit();
}
bool AmpAssembleController::familyExitReady() const {
  std::lock_guard<std::mutex> lock(policy_mutex_);
  const auto* arm = child()->getArmController();
  return policy_.exitReady() && (!arm || (arm->getMode() == 1 &&
      !arm->isImpedanceHandoverActive() && !arm->isHomingToDefault() &&
      (policy_.activeRole() != Role::Soft || arm->isMeasuredArmAtDefault())));
}
bool AmpAssembleController::canSwitchToMpc() const {
  // Raw intent remains visible even when the exit gate has zeroed execution.
  // Stale nonzero input does not become permission to exit.
  {
    std::lock_guard<std::mutex> lock(policy_mutex_);
    if (policy_.blending()) return false;
  }
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    if (!(std::hypot(command_.linear.x, command_.linear.y) <= 0.05 &&
          std::abs(command_.angular.z) <= 0.05)) return false;
  }
  return child() && child()->isAllowToExit() && child()->hasNearZeroGaitCommand(0.05, 0.05);
}
void AmpAssembleController::refreshCommandGates(const AmpFamilyPolicy::Input& in) {
  std::lock_guard<std::mutex> lock(policy_mutex_);
  walking_allowed_ = isActive() && policy_.walkingAllowed(in);
  external_allowed_ = isActive() && policy_.externalAllowed();
  // A manipulation handoff holds stance independently of residual body speed;
  // the physical entry-speed check remains in the policy transition guard.
  const bool hold_stance = in.body_speed <= stop_speed_ || policy_.manipulationStanceRequired(in);
  for (auto& model : members_) if (model)
  {
    const bool inactive = model.get() != child() && model.get() != blend_source_;
    const bool source_stopping = model.get() != child() && in.external_control;
    model->setGaitExecutionGate(!walking_allowed_.load() || inactive || source_stopping, hold_stance);
  }
}
void AmpAssembleController::advance(bool outer_blend_active) {
  std::lock_guard<std::recursive_mutex> lifecycle_lock(lifecycle_mutex_);
  if (!isActive()) return;
  thermal_running_=child()==member(Role::Run) || (blend_source_ && blend_source_==member(Role::Run));
  auto in = inputSnapshot();
  // Control-side watchdog bounds the wait for permission to leave RUN.
  if(thermal_enabled_ && thermal_running_) {
    if(in.thermal_force_exit && thermal_exit_since_<0) {
      thermal_exit_since_=thermalNow();
      std::lock_guard<std::mutex> lock(thermal_mutex_); thermal_.latchExit();
    }
    if(thermal_exit_since_>=0) in.thermal_force_exit=true;
    const bool blending_out_of_run=blend_source_==member(Role::Run) && child()!=member(Role::Run);
    thermal_timed_out_=thermal_exit_since_>=0 && RunThermalGuard::exitTimedOut(
      blending_out_of_run,thermalNow()-thermal_exit_since_.load(),thermal_exit_timeout_);
  } else { thermal_exit_since_=-1; thermal_timed_out_=false; }
  const double thermal_limit = RunThermalGuard::exitVelocityLimit(
      thermal_enabled_, thermal_running_.load(), in.thermal_force_exit,
      thermal_timed_out_.load(), thermal_walk_speed_);
  for(auto& m:members_) if(m)
    static_cast<AmpFamilyMemberController*>(m.get())->setThermalVelocityLimit(thermal_limit);
  if (auto* run = member(Role::Run)) {
    AmpFamilyPolicy snapshot;
    { std::lock_guard<std::mutex> lock(policy_mutex_); snapshot = policy_; }
    // Preview intent without advancing policy timers; update even while the
    // outer blender owns RUN -> WALK so the live RUN source stays capped.
    snapshot.evaluate(in);
    static_cast<AmpFamilyMemberController*>(run)->setRunToWalkSpeedLimit(snapshot.runToWalkSpeedLimit());
    static_cast<AmpFamilyMemberController*>(run)->requestRunElbowInterpolation(
        snapshot.runToWalkRequested(), child() == run || blend_source_ == run);
  }
  refreshCommandGates(in);
  if (outer_blend_active) { publishStatus(in, "outer_blend"); return; }
  auto* source = child();
  auto* arm = source->getArmController();
  if (arm) {
    std::string reason;
    in.arm_ready = arm->isImpedanceSwitchAllowed(reason);
    Role role; bool exiting, passive_run_exit, staying_soft;
    { std::lock_guard<std::mutex> lock(policy_mutex_);
      role = policy_.activeRole(); exiting = policy_.exiting();
      passive_run_exit = role == Role::Run && policy_.config().run_to_walk_elbow_guard;
      const auto requested = policy_.requestedRole();
      staying_soft = role == Role::Soft && !exiting && !policy_.blending() &&
          (!requested || *requested == Role::Soft);
    }
    if (soft_exit_arm_mode_ && staying_soft &&
        !arm->isHomingToDefault() && !arm->isImpedanceHandoverActive()) {
      if (arm->getMode() == 1 || arm->getRequestedMode() == 1)
        source->requestArmControlMode(*soft_exit_arm_mode_);
      soft_exit_arm_mode_.reset();
    }
    // Return the current arm using its own balance policy before leaving it.
    if ((role == Role::Soft || role == Role::Manipulate || (exiting && !passive_run_exit)) && in.arm_ready) {
      bool wants_exit;
      { std::lock_guard<std::mutex> lock(policy_mutex_); wants_exit = policy_.exiting() || policy_.evaluate(in).has_value(); }
      if (wants_exit && arm->getRequestedMode() != 1) {
        if (role == Role::Soft && (arm->getMode() == 3 || arm->getMode() == 4))
          soft_exit_arm_mode_ = arm->getMode();
        source->requestArmControlMode(1);
      }
      if (wants_exit) in.arm_ready = arm->getMode() == 1 && !arm->isImpedanceHandoverActive() &&
          !arm->isHomingToDefault() && (role != Role::Soft || arm->isMeasuredArmAtDefault());
    }
  }
  if (source->hasActiveArmActionSession()) { publishStatus(in, "arm_action"); return; }
  std::optional<Role> next;
  { std::lock_guard<std::mutex> lock(policy_mutex_); next = policy_.evaluate(in); }
  if (!next) {
    if (external_allowed_ && pending_external_mode_) {
      if (source->requestArmControlMode(2)) pending_external_mode_ = false;
    }
    publishStatus(in);
    return;
  }
  auto* target = member(*next);
  if (!target || !target->isInitialized()) { publishStatus(in, "target_unavailable"); return; }
  target->prepareForActivation();
  if (*next == Role::Soft || *next == Role::Manipulate || in.stance)
    target->resetGaitCommandState(true);
  else
    // Every family member is an AmpFamilyMemberController (including the SOFT subtype).
    static_cast<AmpFamilyMemberController*>(target)->inheritGaitCommandState(*static_cast<AmpFamilyMemberController*>(source));
  target->resumeWarm();
  if (!target->isActive()) { publishStatus(in, "target_not_active"); return; }
  bool committed = false;
  auto confirmed_input = inputSnapshot();
  if (arm) {
    std::string reason;
    confirmed_input.arm_ready = arm->isImpedanceSwitchAllowed(reason);
    if (source == member(Role::Soft) || source == member(Role::Manipulate))
      confirmed_input.arm_ready = confirmed_input.arm_ready && arm->getMode() == 1 &&
          !arm->isImpedanceHandoverActive() && !arm->isHomingToDefault();
    if (source == member(Role::Soft))
      confirmed_input.arm_ready = confirmed_input.arm_ready && arm->isMeasuredArmAtDefault();
  }
  const char* rejection_reason = "request_changed";
  {
    std::lock_guard<std::mutex> lock(policy_mutex_);
    // A ROS request can replace the intent while target preparation runs.
    const auto confirmed = policy_.evaluate(confirmed_input);
    if (!confirmed) rejection_reason = policy_.status().reason;
    if (confirmed && *confirmed == *next) {
      policy_.beginTransition(*next, confirmed_input.now);
      soft_exit_arm_mode_.reset();
      active_.store(target); blend_source_ = source;
      pending_pair_ = {source, target, source->getName(), target->getName(), switch_duration_,
                       *next == Role::Run || source == member(Role::Run), true};
      committed = true;
    }
  }
  if (!committed) { target->pause(); publishStatus(confirmed_input, rejection_reason, true); return; }
  if(source==member(Role::Run) && *next==Role::Walk && thermal_exit_since_>=0) {
    thermal_timed_out_=false;
    for(auto& model:members_) if(model)
      static_cast<AmpFamilyMemberController*>(model.get())->setThermalVelocityLimit(thermal_walk_speed_);
  }
  refreshCommandGates(in);
  target->updateVelocityLimitsParam(nh_);
  publishMember(source->getName(), true);
  publishStatus(in, "blending", true);
}
RLControllerBase::SwitchBlendPair AmpAssembleController::takeSwitchBlendPair() {
  std::lock_guard<std::mutex> lock(policy_mutex_);
  auto pair = pending_pair_; pending_pair_ = {}; return pair;
}
void AmpAssembleController::finishSwitchBlend(bool success) {
  std::lock_guard<std::recursive_mutex> lifecycle_lock(lifecycle_mutex_);
  RLControllerBase *old_source, *old_target;
  {
    std::lock_guard<std::mutex> lock(policy_mutex_);
    if (!policy_.blending()) return;
    old_source = blend_source_; old_target = child();
    policy_.finishTransition(success, ros::Time::now().toSec());
    active_.store(member(policy_.activeRole()));
    blend_source_ = nullptr; pending_pair_ = {};
  }
  if(success && old_source==member(Role::Run) && child()==member(Role::Walk)) {
    const bool thermal_exit=thermal_exit_since_>=0;
    thermal_running_=false; thermal_exit_since_=-1; thermal_timed_out_=false;
    for(auto& model:members_) if(model)
      static_cast<AmpFamilyMemberController*>(model.get())->setThermalVelocityLimit(
        std::numeric_limits<double>::infinity());
    if(thermal_exit)
      static_cast<AmpFamilyMemberController*>(child())->resumeExternalVelocityAfterThermalExit();
  }
  if (!success && old_target && old_target != child()) old_target->pause();
  if (success && old_source && old_source != child()) old_source->pause();
  if (!success && isActive()) child()->resumeWarm();
  refreshCommandGates(inputSnapshot());
  child()->updateVelocityLimitsParam(nh_);
  publishMember("", false);
  publishStatus(inputSnapshot(), success ? "completed" : "rolled_back", true);
}
std::string AmpAssembleController::activeMemberName() const { return child() ? child()->getName() : ""; }
void AmpAssembleController::publishMember(const std::string& source, bool transitioning) {
  if (!member_pub_) return;
  std_msgs::String state; state.data = activeMemberName(); member_pub_.publish(state);
  if (!source.empty()) {
    kuavo_msgs::ControllerSwitchEvent event; event.header.stamp = ros::Time::now();
    event.from_controller = source; event.to_controller = activeMemberName(); member_event_pub_.publish(event);
  }
  ROS_INFO("[%s] member=%s phase=%s", name_.c_str(), state.data.c_str(), transitioning ? "blending" : "active");
}
void AmpAssembleController::publishStatus(const AmpFamilyPolicy::Input& in, const char* reason, bool force) {
  if (!status_pub_) return;
  const auto now = ros::WallTime::now();
  if (!force && !last_status_publish_.isZero() && (now - last_status_publish_).toSec() < 0.2) return;
  last_status_publish_ = now;
  // Same selection semantics as active_member: changes at blend start, and
  // returns to the source on rollback. Waiting at a gate keeps the current value.
  std_msgs::Int8 controller_type;
  const auto* selected = child();
  controller_type.data = 0;
  if (isActive() && selected) {
    for (size_t i = 0; i < members_.size(); ++i)
      if (selected == members_[i].get()) {
        controller_type.data = AmpFamilyPolicy::controllerType(static_cast<Role>(i));
        break;
      }
  }
  controller_type_pub_.publish(controller_type);
  // Diagnostics evaluate a copy: observing status must never advance a dwell timer.
  AmpFamilyPolicy snapshot;
  {
    std::lock_guard<std::mutex> lock(policy_mutex_);
    snapshot = policy_;
  }
  snapshot.evaluate(in);
  if (reason && std::string(reason) == "outer_blend" && snapshot.blending()) reason = "blending";
  const auto& status = snapshot.status();
  const auto& cfg = snapshot.config();
  const auto roleName = [&](Role role) -> std::string {
    const auto* model = member(role);
    return model ? model->getName() : "";
  };
  nlohmann::json member_config = nlohmann::json::array();
  for (size_t i = 0; i < members_.size(); ++i) if (members_[i])
    member_config.push_back({{"name", members_[i]->getName()},
                             {"controller_type", AmpFamilyPolicy::controllerType(static_cast<Role>(i))},
                             {"allow_direct_rl_switch", cfg.allow_direct_rl_switch[i]}});
  nlohmann::json enabled = nlohmann::json::array();
  for (const auto& model : members_) if (model) enabled.push_back(model->getName());
  nlohmann::json manual_cycle = nlohmann::json::array();
  for (Role role : AmpFamilyPolicy::manual_cycle)
    if (member(role)) manual_cycle.push_back(roleName(role));
  const auto requested = snapshot.requestedRole();
  RunToWalkElbowInterpolation elbow_interpolation;
  if (auto* run = member(Role::Run))
    elbow_interpolation = static_cast<AmpFamilyMemberController*>(run)->runElbowInterpolationSnapshot();
  nlohmann::json data = {
    {"manual_cycle", manual_cycle},
    {"controller_type", controller_type.data},
    {"anchor_member", roleName(Role::Walk)}, {"members", member_config},
    {"direct_rl_exit", snapshot.directRlExit()}, {"stamp", in.now}, {"family_active", isActive()}, {"active_member", activeMemberName()},
    {"requested_member", requested ? roleName(*requested) : ""},
    {"desired_member", roleName(status.desired)}, {"next_member", roleName(status.next)},
    {"blending", snapshot.blending()}, {"exiting", snapshot.exiting()},
    {"reason", reason ? reason : status.reason}, {"enabled_members", enabled},
    {"command_x", in.command_x}, {"command_yaw", in.command_yaw},
    {"command_threshold_units", cfg.command_threshold_ratios ? "ratio" : "velocity"},
    {"command_ratio", AmpFamilyPolicy::commandRatios(in)}, // x, y, yaw; may exceed 1 for external publishers.
    {"command_limits", in.command_limits},
    {"command_fresh", in.command_fresh}, {"body_speed", in.body_speed},
    {"stance", in.stance}, {"phase_valid", in.phase_valid}, {"arm_ready", in.arm_ready},
    {"walking_intent", in.walking_intent}, {"external_control", in.external_control},
    {"walking_allowed", walking_allowed_.load()}, {"external_allowed", external_allowed_.load()},
    {"dwell_remaining", status.dwell_remaining}, {"intent_remaining", status.intent_remaining},
    {"walkTorun", cfg.walk_to_run}, {"runTowalk", cfg.run_to_walk},
    {"runToWalkSpeedLimit", cfg.run_to_walk_speed_limit},
    {"run_to_walk_speed_limit_active", std::isfinite(snapshot.runToWalkSpeedLimit())},
    {"run_to_walk_elbow_interpolation", {
      {"enabled", elbow_interpolation.enabled()}, {"active", elbow_interpolation.active()},
      {"phase", elbow_interpolation.phaseName()}, {"reference", elbow_interpolation.reference()}}},
    {"run_max_angular", cfg.run_max_angular}, {"run_min_body_speed", cfg.run_min_body_speed},
    {"run_walk_elbow_guard", {
      {"enabled", cfg.run_to_walk_elbow_guard},
      {"applicable", snapshot.activeRole() == Role::Run && status.next == Role::Walk && !snapshot.blending()},
      {"joint_indices", cfg.elbow_joint_indices}, {"position", in.elbow_position},
      {"min_position", cfg.elbow_min}, {"max_position", cfg.elbow_max},
      {"sample_time", in.elbow_sample_time}, {"received_time", in.elbow_received_time},
      {"sample_fresh", status.elbow_sample_fresh},
      {"in_range", status.elbows_in_range}, {"ready", status.elbows_ready},
      {"hold_time", cfg.elbow_hold_time}, {"hold_remaining", status.elbow_hold_remaining},
      {"max_sample_age", cfg.elbow_sample_timeout}
    }}
  };
  std_msgs::String message; message.data = data.dump(); status_pub_.publish(message);
}
void AmpAssembleController::thermalTick(const ros::WallTimerEvent&) {
  const double now=thermalNow();
  {
    std::lock_guard<std::mutex> lock(thermal_mutex_);
    thermal_.update(now,thermal_running_.load());
    thermal_last_tick_=now;
  }
  if(thermal_timed_out_) ROS_ERROR_THROTTLE(1,"Thermal RUN exit timed out: command reduced to zero; inspect physical switch guard");
}

void AmpAssembleController::manualCycleCallback(const std_msgs::Empty::ConstPtr&) {
  std::lock_guard<std::recursive_mutex> lifecycle_lock(lifecycle_mutex_);
  if (!isActive()) return;
  std::optional<Role> next;
  {
    std::lock_guard<std::mutex> lock(policy_mutex_);
    next = policy_.nextManualRole();
  }
  if (!next) {
    ROS_WARN_THROTTLE(1.0, "[%s] Manual cycle unavailable: blending/exiting or no enabled alternative", name_.c_str());
    return;
  }
  auto* target = member(*next);
  if (target && requestMember(target->getName())) {
    ROS_INFO("[%s] Manual cycle requested %s; waiting for normal switch gates", name_.c_str(), target->getName().c_str());
    publishStatus(inputSnapshot(), nullptr, true);
  }
}
void AmpAssembleController::softCallback(const std_msgs::Empty::ConstPtr&) {
  if (!isActive()) return;
  Role requested;
  { std::lock_guard<std::mutex> lock(policy_mutex_);
    requested = policy_.activeRole() == Role::Soft ? Role::Walk : Role::Soft;
  }
  auto* target = member(requested);
  if (!target) {
    ROS_WARN_THROTTLE(1.0, "[%s] SOFT is disabled in amp_family.yaml", name_.c_str());
    return;
  }
  requestMember(target->getName());
}
void AmpAssembleController::setCommandBufferCallback(std::function<bool()> callback) {
  upstream_walk_gate_ = std::move(callback);
  for (auto& m : members_) if (m) m->setCommandBufferCallback([this]() {
    return !walking_allowed_.load() || (upstream_walk_gate_ && upstream_walk_gate_());
  });
}
void AmpAssembleController::setExternalCommandBufferCallback(std::function<bool()> callback) {
  upstream_external_gate_ = std::move(callback);
  for (auto& m : members_) if (m) {
    auto* model = m.get();
    m->setExternalCommandBufferCallback([this, model]() {
      return model != active_.load() || !external_allowed_.load() ||
             (upstream_external_gate_ && upstream_external_gate_());
    });
  }
}
bool AmpAssembleController::requestArmControlMode(int mode) {
  if (mode == 2) {
    if (!member(Role::Manipulate)) {
      ROS_WARN_THROTTLE(1.0, "[%s] External arm control requires enabled MANIPULATE", name_.c_str());
      return false;
    }
    {
      std::lock_guard<std::mutex> lock(policy_mutex_);
      // An explicit mode-2 request may leave SOFT; background VR/waist traffic
      // must not release its latch merely by calling notifyExternalControl().
      if (policy_.activeRole() == Role::Soft && !policy_.request(Role::Manipulate))
        return false;
    }
    notifyExternalControl();
    if (!external_allowed_) { pending_external_mode_ = true; return true; }
  } else pending_external_mode_ = false;
  return child()->requestArmControlMode(mode);
}
void AmpAssembleController::updateVelocityLimitsParam(ros::NodeHandle& nh) { child()->updateVelocityLimitsParam(nh); }
bool AmpAssembleController::updateImpl(const ros::Time& time, const SensorData& data,
    const Eigen::VectorXd& state, kuavo_msgs::jointCmd& command) {
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    const double stamp = data.timeStamp_.toSec();
    if (stamp != elbow_sample_time_) {
      elbow_sample_time_ = stamp;
      elbow_received_time_ = ros::Time::now().toSec();
    }
    for (size_t i = 0; i < elbow_joint_indices_.size(); ++i) {
      const int index = elbow_joint_indices_[i];
      elbow_position_[i] = index >= 0 && index < data.jointPos_.size()
          ? data.jointPos_[index] : std::numeric_limits<double>::quiet_NaN();
    }
  }
  auto* current = child();
  current->applyBaseState(baseStateRL_); current->applyFeetPositions(feetPositionsRL_);
  const bool success = current->update(time, data, state, command);
  if (success && external_allowed_ && (!upstream_external_gate_ || !upstream_external_gate_()))
    external_handoff_pending_ = false;
  return success;
}
} // namespace humanoid_controller
