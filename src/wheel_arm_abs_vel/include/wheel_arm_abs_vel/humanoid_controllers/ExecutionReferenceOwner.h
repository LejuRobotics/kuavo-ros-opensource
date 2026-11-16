#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

namespace velocity_ik_execution {
// WBC-thread-only owner. The normal producer sample is passed through exactly.
// On loss, advance one trapezoidal stopping segment per nominal control tick.
// Recovery is stop-first and uses a two-way handshake; no extrapolated producer
// state accumulated during the outage may replace the stopping trajectory.
template <class Reference>
class ExecutionReferenceOwner {
 public:
  enum class Phase { Producer, Braking, StoppedAnchor };
  explicit ExecutionReferenceOwner(std::string instance=""):instance_(std::move(instance)) {}
  bool resolve(const Reference& input, bool fresh, bool& owned,
               int64_t steady_ns, int64_t timeout_ns, double dt,
               Reference& output) {
    const bool gap = last_tick_ns_ && steady_ns - last_tick_ns_ > timeout_ns;
    last_tick_ns_ = steady_ns;
    if (!owned && !degrading_) { have_state_ = false; return false; }
    if (!have_state_) {
      if (!fresh || input.execution_epoch != 0 ||
          (!input.execution_owner_instance.empty() && input.execution_owner_instance!=instance_)) { owned = false; return false; }
      state_ = input; have_state_ = true;
    }
    if (!degrading_ && (!fresh || gap || input.execution_epoch != epoch_ ||
        input.producer_instance != state_.producer_instance ||
        (!input.execution_owner_instance.empty() && input.execution_owner_instance != instance_))) {
      degrading_ = true;
      ++epoch_;
      anchor_ = 0;
      state_.execution_epoch = epoch_;
      state_.recovery_anchor_seq = 0;
    }
    if (degrading_) {
      owned = true;
      // A stopped snapshot's identity remains stable while waiting. Reject
      // a delayed first candidate: only an exact zero-velocity echo can rejoin.
      if (anchor_ && fresh && input.takeover_active && input.execution_epoch == epoch_ &&
          input.recovery_anchor_seq == anchor_ &&
          input.execution_owner_instance == instance_ &&
          (input.q - state_.q).cwiseAbs().maxCoeff() <= 1e-10 &&
          input.dq.cwiseAbs().maxCoeff() == 0.0 &&
          input.ddq.cwiseAbs().maxCoeff() == 0.0) {
        degrading_ = false;
        state_ = input;
        granted_until_ns_ = input.producer_lease_deadline_ns;
      } else {
        // A sent producer lease is irrevocable until its shared-clock deadline.
        // Repeat the last adopted snapshot while revocation is in flight.
        // This is not a second integration step.
        if (!anchor_ && steady_ns >= granted_until_ns_) {
          const auto previous = state_.dq;
          for (int i = 0; i < state_.dq.size(); ++i) {
            const double delta = state_.stopping_acceleration(i) * dt;
            state_.dq(i) = std::copysign(
                std::max(0.0, std::abs(previous(i)) - delta), previous(i));
          }
          state_.ddq = (state_.dq - previous) / dt;
          state_.q += 0.5 * dt * (previous + state_.dq);
          ++state_.reference_seq;
          if (state_.dq.cwiseAbs().maxCoeff() == 0.0 &&
              state_.ddq.cwiseAbs().maxCoeff() == 0.0)
            anchor_ = state_.reference_seq;
        }
        state_.execution_epoch = epoch_;
        state_.recovery_anchor_seq = anchor_;
        state_.safety_braking = !anchor_;
        state_.takeover_active = true;
      }
    } else {
      state_ = input;
      granted_until_ns_ = input.producer_lease_deadline_ns;
    }
    state_.execution_owner_instance = instance_;
    output = state_;
    return true;
  }
  Phase phase() const {
    return !degrading_ ? Phase::Producer
        : (anchor_ ? Phase::StoppedAnchor : Phase::Braking);
  }
  bool degrading() const { return degrading_; }
  uint64_t epoch() const { return epoch_; }
  uint64_t anchor() const { return anchor_; }
  int64_t producerLeaseUntil() const { return degrading_ ? 0 : granted_until_ns_; }
 private:
  Reference state_;
  std::string instance_;
  int64_t granted_until_ns_{0};
  bool have_state_{false}, degrading_{false};
  uint64_t epoch_{0}, anchor_{0};
  int64_t last_tick_ns_{0};
};
}  // namespace velocity_ik_execution
