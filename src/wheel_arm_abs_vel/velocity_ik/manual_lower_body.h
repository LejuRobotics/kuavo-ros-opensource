#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>

namespace velocity_ik {
// This transaction never writes/integrates robot q. The production supervisor
// consumes its desired velocity and retains sole ownership of q/dq/ddq.
class ManualLowerBody {
 public:
  using Joints = std::array<double, 4>;
  using Key = std::array<uint64_t, 7>;
  enum State : uint8_t { Idle, Queued, Moving, Settled, Braking, Aborted };
  static constexpr double kMaxVelocity = 12.0 * 3.14159265358979323846 / 180.0;
  static constexpr double kMaxAcceleration = 15.0 * 3.14159265358979323846 / 180.0;
  struct Snapshot {
    double now{0.0};
    Key key{};  // session, generation, body/base epochs, source, arm/torso operations
    bool allowed{false}, stationary{false};
    std::string reason{"IK has not provided a live execution snapshot"};
    Joints q{}, dq{}, measured{}, measured_dq{}, lower{}, upper{};
  };

  bool busy() const { return state == Queued || state == Moving || state == Braking; }
  bool ready(double now) const {
    for (std::size_t j=0;j<4;++j)
      if (!std::isfinite(snapshot.q[j]) || !std::isfinite(snapshot.measured[j]) ||
          !std::isfinite(snapshot.lower[j]) || !std::isfinite(snapshot.upper[j]) ||
          snapshot.lower[j] > snapshot.upper[j]) return false;
    return std::isfinite(now) && now >= snapshot.now && now - snapshot.now <= .1 &&
           snapshot.now > 0 && snapshot.allowed && snapshot.stationary && !busy();
  }
  bool request(const Joints& goal, double now, std::string& reason) {
    if (!ready(now)) {
      reason = busy() ? "A lower-body adjustment is already running" :
          (!snapshot.allowed ? snapshot.reason :
           (!snapshot.stationary ? "Wait for arms and lower body to stop" : "IK snapshot is stale"));
      return false;
    }
    double travel = 0.;
    for (std::size_t j=0;j<4;++j) {
      if (!std::isfinite(goal[j]) || goal[j] < snapshot.lower[j] || goal[j] > snapshot.upper[j]) {
        reason = "Target is non-finite or outside the model limits (joint " + std::to_string(j+1) + ")";
        return false;
      }
      travel = std::max(travel, std::abs(goal[j]-snapshot.q[j]));
    }
    target = goal; key_ = snapshot.key; ++id;
    deadline_ = now + 10. + 4. * travel / kMaxVelocity;
    settled_s_ = 0.; state = Queued; message = "Waiting for IK admission";
    reason = message;
    return true;
  }
  bool cancel(uint64_t request_id, std::string& reason) {
    if (!busy() || request_id != id) { reason = "No matching active adjustment"; return false; }
    abort("Cancelled by TUI"); reason = message; return true;
  }
  void abort(const std::string& reason) {
    if (state == Moving || state == Queued) {
      state = Braking; message = reason; settled_s_ = 0.;
    }
  }
  void update(const Snapshot& next, double dt) {
    snapshot = next;
    if (state == Moving || state == Queued) {
      if (!next.allowed) abort(next.reason);
      else if (next.key != key_) abort("VR session or operator mode changed");
      else if (!std::isfinite(dt) || dt <= 0. || dt > .05 || next.now > deadline_)
        abort("Adjustment timed out or IK timing became invalid");
      else {
        state = Moving; message = "Adjusting lower body; arms and torso following remain locked";
        bool settled = true;
        for (std::size_t j=0;j<4;++j)
          settled = settled && std::abs(target[j]-next.q[j]) <= 5.e-4 &&
              std::abs(target[j]-next.measured[j]) <= .02 &&
              std::abs(next.dq[j]) <= 1.e-3 && std::abs(next.measured_dq[j]) <= .01;
        settled_s_ = settled ? settled_s_ + dt : 0.;
        if (settled_s_ >= .1) {
          state = Settled; message = "Target reached and held; use X+B to resume arms (X+A prepares)";
        }
      }
    }
    if (state == Braking && next.stationary) state = Aborted;
  }
  Joints velocity() const {
    Joints result{};
    if (state == Moving)
      for (std::size_t j=0;j<4;++j)
        result[j] = std::clamp(target[j]-snapshot.q[j], -kMaxVelocity, kMaxVelocity);
    return result;
  }

  Snapshot snapshot;
  State state{Idle};
  uint64_t id{0};
  Joints target{};
  std::string message{"No adjustment requested"};
 private:
  Key key_{};
  double deadline_{0.}, settled_s_{0.};
};
}  // namespace velocity_ik
