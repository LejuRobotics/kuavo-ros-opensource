#pragma once
#include <cmath>
#include <cstdint>

namespace velocity_ik {
// Fixed PREPARE is an explicit joint-space transaction, not a live pose task.
// This class carries command freshness only; it never owns q/dq/ddq.
class FixedPrepareTransaction {
 public:
  bool observe(uint64_t session, uint64_t generation, uint64_t coordinate_epoch,
               uint32_t seq, double source_s, double receive_ns,
               double source_age_s, double timeout_s, bool prepare,
               bool control_valid, bool admission_pose_valid) {
    if (!std::isfinite(source_s) || source_s <= 0 ||
        !std::isfinite(receive_ns) || receive_ns <= 0 ||
        !std::isfinite(source_age_s) || source_age_s < 0 ||
        !std::isfinite(timeout_s) || timeout_s <= 0) {
      return false;
    }
    // Packet reception never renews an old command. A changed identity also
    // cannot make a delayed packet newer in this process's source clock.
    if (seen_ && (source_s <= source_s_ || receive_ns < receive_ns_)) return false;
    const bool same = seen_ && session == session_ && generation == generation_ &&
                      coordinate_epoch == coordinate_epoch_;
    const uint32_t advance = seq - seq_;
    if (same && (advance == 0 || advance >= 0x80000000u)) return false;
    const bool continuing = same && admitted_;
    seen_ = true;
    source_s_ = source_s; receive_ns_ = receive_ns; seq_ = seq;
    session_ = session; generation_ = generation; coordinate_epoch_ = coordinate_epoch;
    // Changed pose source/confidence cannot authorize initial admission.
    // Cancellation requires a fresh explicit lifecycle/control command.
    admitted_ = prepare && control_valid && source_age_s <= timeout_s &&
                (continuing || admission_pose_valid);
    deadline_ns_ = receive_ns + (timeout_s - source_age_s) * 1e9;
    return admitted_;
  }

  bool fresh(uint64_t session, uint64_t generation, uint64_t coordinate_epoch,
             double now_ns) const {
    return admitted_ && session == session_ && generation == generation_ &&
           coordinate_epoch == coordinate_epoch_ && std::isfinite(now_ns) &&
           now_ns >= receive_ns_ && now_ns <= deadline_ns_;
  }

 private:
  bool seen_ = false, admitted_ = false;
  uint64_t session_ = 0, generation_ = 0, coordinate_epoch_ = 0;
  uint32_t seq_ = 0;
  double source_s_ = 0, receive_ns_ = 0, deadline_ns_ = 0;
};
}  // namespace velocity_ik
