#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <limits>
#include <stdexcept>

namespace humanoid_controller {
// No ROS or model state: all time and physical readiness comes from the caller.
class AmpFamilyPolicy {
 public:
  enum class Role { Manipulate, Walk, Run, Soft, WalkWaq };
  // Public logging codes: fixed by role, independent of enabled-member order.
  // 0 is reserved for an inactive family or an unknown selection.
  static constexpr int controllerType(Role role) {
    switch (role) {
      case Role::Manipulate: return 1;
      case Role::Walk: return 2;
      case Role::Run: return 3;
      case Role::Soft: return 4;
      case Role::WalkWaq: return 5;
    }
    return 0;
  }
  struct Config {
    std::array<bool, 5> enabled{{true, true, true, true, false}};
    std::array<bool, 5> allow_direct_rl_switch{{true, true, false, true, true}};
    bool command_threshold_ratios = false; // Legacy configs remain in velocity units.
    double walk_to_run = 1.4;
    double run_to_walk = 1.2;
    double run_to_walk_speed_limit = 0.0; // Forward target before smoothing; 0 disables.
    double run_max_angular = 0.3;
    double run_min_body_speed = 0.5;
    double manipulation_max_speed = 0.3;
    double walk_enter = 0.05;
    double intent_hold = 0.4;
    double min_dwell = 1.0;
    // Opt in per family; legacy families and other controller types are unchanged.
    bool run_to_walk_elbow_guard = false;
    std::array<int, 2> elbow_joint_indices{{16, 23}};
    std::array<double, 2> elbow_min{{-0.8, -0.8}}, elbow_max{{0.2, 0.2}};
    double elbow_hold_time = 0.1, elbow_sample_timeout = 0.1;
  };
  struct Input {
    double now = 0.0;
    double command_x = 0.0;
    double command_y = 0.0;
    double command_yaw = 0.0;
    std::array<double,3> command_limits{{1.,1.,1.}}; // Current member: x, y, yaw.
    double body_speed = 0.0;
    bool command_fresh = false;
    bool walking_intent = false;
    bool external_control = false;
    bool stance = false;
    bool phase_valid = false;
    bool arm_ready = true;
    bool thermal_can_enter = true, thermal_force_exit = false;
    double elbow_sample_time = 0.0, elbow_received_time = 0.0;
    std::array<double, 2> elbow_position{{std::numeric_limits<double>::quiet_NaN(),
                                         std::numeric_limits<double>::quiet_NaN()}};
  };
  struct Status {
    Role desired = Role::Walk;
    Role next = Role::Walk;
    const char* reason = "idle";
    double dwell_remaining = 0.0;
    double intent_remaining = 0.0;
    bool elbow_sample_fresh = false, elbows_in_range = false, elbows_ready = false;
    double elbow_hold_remaining = 0.0;
  };
  static std::array<double,3> commandRatios(const Input& in) {
    std::array<double,3> values{{in.command_x,in.command_y,in.command_yaw}};
    for(size_t i=0;i<values.size();++i)
      values[i]=std::isfinite(in.command_limits[i]) && in.command_limits[i]>0
          ? values[i]/in.command_limits[i] : std::numeric_limits<double>::quiet_NaN();
    return values;
  }
  bool enabled(Role role) const { return config_.enabled.at(static_cast<size_t>(role)); }
  const Status& status() const { return status_; }
  std::optional<Role> requestedRole() const { return requested_; }
  const Config& config() const { return config_; }
  AmpFamilyPolicy() : AmpFamilyPolicy(Config{}) {}
  explicit AmpFamilyPolicy(Config config) : config_(config) {
    if (config.command_threshold_ratios && (config.walk_to_run >= 1.0 ||
        config.run_to_walk > 1.0 || config.run_max_angular > 1.0 || config.walk_enter > 1.0))
      throw std::invalid_argument("AMP ratio thresholds must be within full scale; walkTorun must be < 1");
    if (!enabled(Role::Walk) || !std::isfinite(config.walk_to_run) || !std::isfinite(config.run_to_walk) ||
        !std::isfinite(config.run_to_walk_speed_limit) || config.run_to_walk_speed_limit < 0.0 ||
        config.run_to_walk < 0.0 || config.walk_to_run <= 0.0 ||
        !std::isfinite(config.min_dwell) || config.min_dwell < 0.0 ||
        !std::isfinite(config.intent_hold) || config.intent_hold < 0.0 ||
        !std::isfinite(config.run_max_angular) || config.run_max_angular <= 0.0 ||
        !std::isfinite(config.run_min_body_speed) || config.run_min_body_speed < 0.0 ||
        !std::isfinite(config.manipulation_max_speed) || config.manipulation_max_speed < 0.0 ||
        !std::isfinite(config.walk_enter) || config.walk_enter < 0.0)
      throw std::invalid_argument("invalid AMP family switching thresholds");
    if (!std::isfinite(config.elbow_hold_time) || config.elbow_hold_time < 0 ||
        !std::isfinite(config.elbow_sample_timeout) || config.elbow_sample_timeout <= 0 ||
        config.elbow_joint_indices[0] < 0 || config.elbow_joint_indices[1] < 0 ||
        config.elbow_joint_indices[0] == config.elbow_joint_indices[1])
      throw std::invalid_argument("invalid AMP elbow guard timing or joint indices");
    for (size_t i = 0; i < 2; ++i)
      if (!std::isfinite(config.elbow_min[i]) || !std::isfinite(config.elbow_max[i]) ||
          config.elbow_min[i] >= config.elbow_max[i])
        throw std::invalid_argument("invalid AMP elbow guard position range");
  }
  static Role nextRole(Role from, Role requested) {
    return from != requested && from != Role::Walk && requested != Role::Walk
               ? Role::Walk : requested;
  }
  void clearElbowGuardHistory() {
    elbow_since_.reset(); elbow_last_sample_.reset();
    elbow_received_since_.reset(); elbow_last_received_.reset();
  }
  void reset(double now) {
    clearElbowGuardHistory();
    active_ = source_ = Role::Walk;
    requested_.reset(); candidate_.reset(); run_walk_requested_ = false;
    auto_run_armed_ = true; automatic_run_exit_ = false;
    blending_ = exiting_ = direct_rl_exit_ = false; last_transition_ = now; status_ = {};
  }
  // Manual ring: automatic RUN and action-driven MANIPULATE are not extra slots.
  inline static constexpr std::array<Role, 3> manual_cycle{{Role::Walk, Role::WalkWaq, Role::Soft}};
  std::optional<Role> nextManualRole() const {
    if (blending_ || exiting_) return {};
    const Role current = requested_.value_or(active_);
    const auto it = std::find(manual_cycle.begin(), manual_cycle.end(), current);
    // RUN/MANIPULATE behave like the WALK slot, selecting WAQ first.
    const size_t start = it == manual_cycle.end() ? 0 : std::distance(manual_cycle.begin(), it);
    for (size_t step = 1; step < manual_cycle.size(); ++step) {
      const Role next = manual_cycle[(start + step) % manual_cycle.size()];
      if (enabled(next)) return next;
    }
    return {};
  }
  bool request(Role role) {
    if (exiting_ || !enabled(role)) return false;
    if (!requested_ || *requested_ != role) clearElbowGuardHistory();
    requested_ = role; candidate_.reset(); run_walk_requested_ = false;
    return true;
  }
  void requestExit(bool to_rl = false) { run_walk_requested_ = false; direct_rl_exit_ = to_rl; exiting_ = true; requested_.reset(); candidate_.reset(); }
  void cancelExit() { if (exiting_) clearElbowGuardHistory(); exiting_ = false; candidate_.reset(); }
  bool directRlExit() const { return exiting_ && direct_rl_exit_ && config_.allow_direct_rl_switch[static_cast<size_t>(active_)]; }
  bool exitReady() const { return exiting_ && (active_ == Role::Walk || directRlExit()) && !blending_; }
  bool exiting() const { return exiting_; }
  bool blending() const { return blending_; }
  // Selection changes at blend start; the RUN source still executes until completion.
  bool blocksOrdinaryFamilyExit() const {
    return active_ == Role::Run || (blending_ && source_ == Role::Run);
  }
  bool runToWalkRequested() const {
    return (active_ == Role::Run && status_.desired == Role::Walk) ||
           (blending_ && source_ == Role::Run && active_ == Role::Walk);
  }
  double runToWalkSpeedLimit() const {
    return runToWalkRequested() && config_.run_to_walk_speed_limit > 0.0
        ? config_.run_to_walk_speed_limit : std::numeric_limits<double>::infinity();
  }
  Role activeRole() const { return active_; }
  bool externalAllowed() const { return active_ == Role::Manipulate && !blending_ && !exiting_; }
  bool walkingAllowed(const Input& in) const {
    if (active_ == Role::Soft || (exiting_ && !directRlExit())) return false;
    if (in.external_control && enabled(Role::Manipulate) && active_ != Role::Manipulate) return false;
    return !requested_ || *requested_ == active_ ||
           (*requested_ != Role::Manipulate && *requested_ != Role::Soft);
  }
  std::optional<Role> evaluate(const Input& in) {
    automatic_run_exit_ = false;
    status_ = {}; status_.desired = status_.next = active_;
    const auto wait = [this](const char* reason) -> std::optional<Role> {
      status_.reason = reason; return {};
    };
    if (blending_) return wait("blending");
    const auto ratios = commandRatios(in);
    if (config_.command_threshold_ratios &&
        (!std::isfinite(ratios[0]) || !std::isfinite(ratios[1]) || !std::isfinite(ratios[2]))) {
      clearElbowGuardHistory(); return wait("invalid_command_range");
    }
    const double command_x = config_.command_threshold_ratios ? ratios[0] : in.command_x;
    const double command_y = config_.command_threshold_ratios ? ratios[1] : in.command_y;
    const double command_yaw = config_.command_threshold_ratios ? ratios[2] : in.command_yaw;
    if (!std::isfinite(in.now) || !std::isfinite(in.body_speed)) {
      clearElbowGuardHistory(); return wait("invalid_state");
    }
    if (in.now < last_transition_) { last_transition_ = in.now; candidate_.reset(); run_walk_requested_ = false; }
    const bool thermal_exit = in.thermal_force_exit && active_ == Role::Run;
    if (requested_ == Role::Run && !in.thermal_can_enter) requested_.reset();
    Role desired = active_;
    const bool external = in.external_control && enabled(Role::Manipulate);
    const bool explicit_request = exiting_ || requested_.has_value();
    const char* idle_reason = "idle";
    if (thermal_exit) { desired = Role::Walk; }
    else if (exiting_) { desired = directRlExit() ? active_ : Role::Walk; idle_reason = "exit_ready"; }
    else if (requested_) desired = *requested_;
    else if (active_ == Role::Soft) return wait("soft_latched");
    else if (active_ == Role::WalkWaq && !external) return wait("walk_waq_selected");
    else if (external) desired = Role::Manipulate;
    else {
      const bool moving_command = in.command_fresh &&
          (std::hypot(command_x, command_y) > config_.walk_enter ||
           std::abs(command_yaw) > config_.walk_enter);
      if (active_ == Role::Manipulate && (moving_command || in.walking_intent)) desired = Role::Walk;
      if (active_ == Role::Walk) {
        // With overlapping raw thresholds, an unchanged /cmd_vel must not
        // repeatedly re-enter RUN after retiring to WALK. A fresh command
        // back below the entry boundary rearms automatic entry. Joystick
        // publishers normally satisfy this when adopting the WALK range.
        if (in.command_fresh && command_x <= config_.walk_to_run)
          auto_run_armed_ = true;
        if (!enabled(Role::Run)) idle_reason = "run_disabled";
        else if (!in.command_fresh) idle_reason = "command_stale";
        else if (!auto_run_armed_) idle_reason = "waiting_run_rearm";
        else if (command_x <= config_.walk_to_run) idle_reason = "run_command_below_threshold";
        else if (std::abs(command_yaw) >= config_.run_max_angular) idle_reason = "run_turn_rate_high";
        else desired = Role::Run;
      }
      if (active_ == Role::Run) {
        // Retain a valid low-speed request while waiting for the elbows. A
        // quiet command publisher must not turn this into a wait for stance.
        // Fresh commands outside the exit range explicitly cancel the request.
        if (config_.run_to_walk_elbow_guard && in.command_fresh)
          run_walk_requested_ = command_x < config_.run_to_walk;
        if ((!in.command_fresh && in.stance) ||
            (in.command_fresh && command_x < config_.run_to_walk) ||
            (config_.run_to_walk_elbow_guard && run_walk_requested_))
          desired = Role::Walk;
      }
    }
    automatic_run_exit_ = active_ == Role::Run && desired == Role::Walk &&
        !thermal_exit && !explicit_request && !external && command_x > config_.walk_to_run;
    status_.desired = desired;
    if (desired == active_) {
      // Keep SOFT latched by role, not by an indefinitely pending request.
      if (requested_ && !exiting_) requested_.reset();
      candidate_.reset(); clearElbowGuardHistory(); return wait(idle_reason);
    }
    const Role next = nextRole(active_, desired);
    status_.next = next;
    // Ordinary guarded RUN -> WALK starts as soon as the configured elbow
    // guard passes. Other destinations and family exits retain their gates.
    const bool immediate_run_walk = config_.run_to_walk_elbow_guard &&
        active_ == Role::Run && desired == Role::Walk && !exiting_;
    // Debounce the requested motion, independently of physical readiness.
    // Per-stride speed/phase fluctuations must not erase a held stick intent.
    // Every physical guard below is still checked at the instant of switching.
    if (!thermal_exit && !immediate_run_walk && !explicit_request && !external) {
      if (!candidate_ || *candidate_ != next) { candidate_ = next; candidate_since_ = in.now; }
      status_.intent_remaining = std::max(0.0, config_.intent_hold - (in.now - candidate_since_));
    }
    if (config_.run_to_walk_elbow_guard && active_ == Role::Run && next == Role::Walk) {
      if (const char* reason = checkElbows(in)) return wait(reason);
    } else clearElbowGuardHistory();
    if (next == Role::Soft && !in.stance) return wait("waiting_stance");
    if (next == Role::Manipulate) {
      if (!in.stance) return wait("waiting_stance");
      if (in.body_speed > config_.manipulation_max_speed) return wait("waiting_body_slow");
    }
    if (next == Role::Run) {
      if (!in.thermal_can_enter) return wait("run_thermal_blocked");
      if (in.stance || !in.phase_valid || in.body_speed < config_.run_min_body_speed) {
        return wait(in.stance ? "waiting_walking" : !in.phase_valid ? "waiting_phase" : "waiting_run_body_speed");
      }
    }
    if ((active_ == Role::Soft || active_ == Role::Manipulate) && !in.arm_ready) return wait("waiting_arm");
    if (!thermal_exit && active_ == Role::Run && (exiting_ || desired == Role::Manipulate || desired == Role::Soft)) {
      if (!in.stance) return wait("waiting_stance");
      if (in.body_speed > config_.manipulation_max_speed) return wait("waiting_body_slow");
    }
    status_.dwell_remaining = immediate_run_walk ? 0.0 :
        std::max(0.0, config_.min_dwell - (in.now - last_transition_));
    if (!thermal_exit && status_.dwell_remaining > 0) return wait("waiting_dwell");
    if (!thermal_exit && status_.intent_remaining > 0) return wait("waiting_intent");
    status_.reason = "ready";
    return next;
  }
  void beginTransition(Role target, double now) {
    if (!enabled(target) || blending_ || target == active_ || nextRole(active_, target) != target)
      throw std::logic_error("invalid or overlapping AMP family transition");
    clearElbowGuardHistory();
    if (automatic_run_exit_ && active_ == Role::Run && target == Role::Walk &&
        config_.run_to_walk >= config_.walk_to_run)
      auto_run_armed_ = false;
    source_ = active_; active_ = target; blending_ = true; run_walk_requested_ = false;
    candidate_.reset(); last_transition_ = now;
  }
  void finishTransition(bool success, double now) {
    clearElbowGuardHistory();
    if (!blending_) return;
    if (!success) active_ = source_;
    if (active_ == Role::Run) auto_run_armed_ = true;
    blending_ = false; last_transition_ = now;
    if (success && requested_ && *requested_ == active_) requested_.reset();
  }
 private:
  const char* checkElbows(const Input& in) {
    status_.elbow_hold_remaining = config_.elbow_hold_time;
    const double stamp = in.elbow_sample_time, received = in.elbow_received_time;
    // The sensor clock may differ from ROS time (e.g. paused/slow MuJoCo).
    // Age is measured against the first reception of this distinct sensor frame.
    const bool valid = std::isfinite(stamp) && stamp > 0 &&
        std::isfinite(received) && received > 0 && received <= in.now &&
        in.now - received <= config_.elbow_sample_timeout &&
        std::isfinite(in.elbow_position[0]) && std::isfinite(in.elbow_position[1]);
    if (!valid) { clearElbowGuardHistory(); return "waiting_run_walk_elbow_sample"; }
    if ((elbow_last_sample_ && stamp < *elbow_last_sample_) ||
        (elbow_last_received_ && received < *elbow_last_received_)) {
      clearElbowGuardHistory(); elbow_last_sample_ = stamp; elbow_last_received_ = received;
      return "waiting_run_walk_elbow_sample";
    }
    status_.elbow_sample_fresh = true;
    if ((elbow_last_sample_ && stamp - *elbow_last_sample_ > config_.elbow_sample_timeout) ||
        (elbow_last_received_ && received - *elbow_last_received_ > config_.elbow_sample_timeout)) {
      elbow_since_.reset(); elbow_received_since_.reset();
    }
    elbow_last_sample_ = stamp; elbow_last_received_ = received;
    status_.elbows_in_range = true;
    for (size_t i = 0; i < 2; ++i)
      status_.elbows_in_range = status_.elbows_in_range &&
          in.elbow_position[i] >= config_.elbow_min[i] && in.elbow_position[i] <= config_.elbow_max[i];
    if (!status_.elbows_in_range) {
      elbow_since_.reset(); elbow_received_since_.reset(); return "waiting_run_walk_elbow_range";
    }
    if (!elbow_since_) { elbow_since_ = stamp; elbow_received_since_ = received; }
    // Both clocks must advance: replayed frames and fast sensor clocks cannot satisfy dwell.
    const double held = std::min(stamp - *elbow_since_, received - *elbow_received_since_);
    status_.elbow_hold_remaining = std::max(0.0, config_.elbow_hold_time - held);
    status_.elbows_ready = status_.elbow_hold_remaining <= 1e-9;
    return status_.elbows_ready ? nullptr : "waiting_run_walk_elbow_hold";
  }
  std::optional<double> elbow_since_, elbow_last_sample_, elbow_received_since_, elbow_last_received_;
  Config config_;
  Status status_;
  Role active_ = Role::Walk;
  Role source_ = Role::Walk;
  std::optional<Role> requested_, candidate_;
  bool blending_ = false, exiting_ = false, direct_rl_exit_ = false;
  bool run_walk_requested_ = false;
  bool auto_run_armed_ = true, automatic_run_exit_ = false;
  double candidate_since_ = 0.0, last_transition_ = 0.0;
};
} // namespace humanoid_controller
