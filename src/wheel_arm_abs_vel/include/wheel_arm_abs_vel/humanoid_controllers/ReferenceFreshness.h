#pragma once
#include <cstdint>

namespace velocity_ik_execution {
// Producer/header age is compared only inside the ROS clock domain.
// A newly accepted sample's remaining lease is then timed only by steady time.
// Call under the reference snapshot mutex. Receipt alone never renews a lease.
class ReferenceFreshness {
 public:
  enum class Result { NewSample, DuplicateOrReordered, OldStamp, ExpiredOrFuture };
  Result accept(uint64_t session, uint64_t generation, uint64_t sequence,
                int64_t publish_ros_ns, int64_t receive_ros_ns,
                int64_t timeout_ns) {
    if (publish_ros_ns <= 0 || receive_ros_ns < publish_ros_ns ||
        receive_ros_ns - publish_ros_ns > timeout_ns)
      return Result::ExpiredOrFuture;
    if (initialized_) {
      if (session == session_ &&
          (generation < generation_ ||
           (generation == generation_ && sequence <= sequence_)))
        return Result::DuplicateOrReordered;
      // Applies across generation/session changes as well: replaying an old
      // epoch cannot supersede a newer published reference.
      if (publish_ros_ns <= publish_ros_ns_) return Result::OldStamp;
    }
    initialized_ = true;
    session_ = session; generation_ = generation; sequence_ = sequence;
    publish_ros_ns_ = publish_ros_ns;
    return Result::NewSample;
  }
 private:
  bool initialized_{false};
  uint64_t session_{0}, generation_{0}, sequence_{0};
  int64_t publish_ros_ns_{0};
};
}  // namespace velocity_ik_execution
