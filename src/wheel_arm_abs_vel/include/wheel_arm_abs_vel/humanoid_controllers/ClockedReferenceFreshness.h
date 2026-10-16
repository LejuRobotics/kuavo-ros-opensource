#pragma once
#include <cstdint>
#include <cmath>
#include <string>
namespace velocity_ik_execution {
// Contract: producer and receiver share a verified Linux boot/steady-clock
// domain. ROS is used only for ROS-age and same-epoch stamp checks. A local ROS
// discontinuity cannot reset monotonic ordering, source identity or sequences.
class ClockedReferenceFreshness {
 public:
  enum class Result { NewSample, WrongClockDomain, MissingIdentity,
    DuplicateOrReordered, OldStamp, ExpiredOrFuture, ReceiverClockRegression };
  explicit ClockedReferenceFreshness(std::string domain):domain_(std::move(domain)) {}
  Result accept(const std::string& domain,const std::string& instance,
      uint64_t session,uint64_t generation,uint64_t sequence,
      int64_t publish_steady_ns,int64_t receive_steady_ns,
      int64_t publish_ros_ns,int64_t receive_ros_ns,int64_t timeout_ns) {
    if(domain_.empty() || domain!=domain_) return Result::WrongClockDomain;
    if(instance.empty()) return Result::MissingIdentity;
    if(timeout_ns<=0 || receive_steady_ns<=0 || receive_ros_ns<=0)
      return Result::ExpiredOrFuture;
    if(have_receive_) {
      if(receive_steady_ns<receive_steady_ns_) return Result::ReceiverClockRegression;
      const long double step=(static_cast<long double>(receive_ros_ns)-receive_ros_ns_)
          -(static_cast<long double>(receive_steady_ns)-receive_steady_ns_);
      // Use the existing lease, not a new physical safety threshold. Smaller
      // clock adjustments may cause a bounded rejection until ROS catches up.
      if(std::abs(step)>timeout_ns) ++ros_epoch_;
    }
    have_receive_=true;receive_steady_ns_=receive_steady_ns;receive_ros_ns_=receive_ros_ns;
    if(publish_steady_ns<=0 || receive_steady_ns<publish_steady_ns ||
       receive_steady_ns-publish_steady_ns>timeout_ns ||
       publish_ros_ns<=0 || receive_ros_ns<publish_ros_ns ||
       receive_ros_ns-publish_ros_ns>timeout_ns) return Result::ExpiredOrFuture;
    if(initialized_) {
      if(instance==instance_ && session==session_ &&
         (generation<generation_ || (generation==generation_ && sequence<=sequence_)))
        return Result::DuplicateOrReordered;
      if(publish_steady_ns<=publish_steady_ns_) return Result::OldStamp;
      if(accepted_ros_epoch_==ros_epoch_ && publish_ros_ns<=publish_ros_ns_)
        return Result::OldStamp;
    }
    initialized_=true;instance_=instance;session_=session;generation_=generation;sequence_=sequence;
    publish_steady_ns_=publish_steady_ns;publish_ros_ns_=publish_ros_ns;accepted_ros_epoch_=ros_epoch_;
    return Result::NewSample;
  }
 private:
  std::string domain_,instance_;
  bool initialized_=false,have_receive_=false;
  uint64_t session_=0,generation_=0,sequence_=0,ros_epoch_=0,accepted_ros_epoch_=0;
  int64_t publish_steady_ns_=0,publish_ros_ns_=0,receive_steady_ns_=0,receive_ros_ns_=0;
};
}
