#pragma once
#include <cstdint>
#include <string>
namespace velocity_ik_execution {
// ROS-free producer half of the existing stop-first handshake. The adapter
// verifies source age/sequence first and validates/copies the exact q/v/a anchor.
class ExecutionProducerLease {
 public:
  enum class Phase { Active, Braking, Hold };
  enum class Result { Ignored, Heartbeat, InstallAnchor, Granted, OwnerRestart };
  struct Feedback {
    std::string producer,owner;
    uint64_t epoch=0,anchor=0;
    int64_t generated_ns=0,lease_until_ns=0;
    Phase phase=Phase::Braking;
    bool valid=false;
  };
  explicit ExecutionProducerLease(std::string producer):producer_(std::move(producer)) {}
  Result observe(const Feedback& f,bool valid_stopped,bool equals_local_stopped) {
    if(terminal_ || restart_required_ || f.producer!=producer_ || f.owner.empty())
      return Result::Ignored;
    if(owner_.empty()) {
      if(!f.valid)return Result::Ignored;
      owner_=f.owner;
    } else if(owner_!=f.owner) {
      restart_required_=true;waiting_=true;initial_=false;lease_until_ns_=0;anchor_=0;
      return Result::OwnerRestart;
    }
    if(!f.valid || f.epoch<epoch_)return Result::Ignored;
    if(f.epoch!=epoch_) {
      if(f.phase==Phase::Active)return Result::Ignored;
      epoch_=f.epoch;anchor_=0;waiting_=true;initial_=false;lease_until_ns_=0;
    }
    if(f.phase==Phase::Braking) {
      waiting_=true;initial_=false;lease_until_ns_=0;
      return Result::Heartbeat;
    }
    if(f.phase==Phase::Hold) {
      if(!f.anchor || !valid_stopped || (anchor_ && anchor_!=f.anchor))return Result::Ignored;
      const bool install=anchor_==0;
      anchor_=f.anchor;waiting_=true;initial_=false;lease_until_ns_=0;
      return install?Result::InstallAnchor:Result::Heartbeat;
    }
    if(f.lease_until_ns<=f.generated_ns)return Result::Ignored;
    if(initial_) {
      if(f.epoch!=0 || !equals_local_stopped)return Result::Ignored;
      initial_=false;waiting_=false;lease_until_ns_=f.lease_until_ns;
      return Result::Granted;
    }
    if(waiting_) {
      if(!anchor_ || f.anchor!=anchor_ || !equals_local_stopped)return Result::Ignored;
      waiting_=false;lease_until_ns_=f.lease_until_ns;
      return Result::Granted;
    }
    lease_until_ns_=f.lease_until_ns;
    return Result::Heartbeat;
  }
  bool canAdvance(int64_t now_ns,int64_t segment_ns) {
    if(terminal_ || restart_required_ || initial_ || waiting_)return false;
    if(segment_ns<0 || now_ns>lease_until_ns_ || segment_ns>lease_until_ns_-now_ns) {
      waiting_=true;anchor_=0;lease_until_ns_=0;return false;
    }
    return true;
  }
  void terminalStop(){terminal_=true;waiting_=true;lease_until_ns_=0;}
  bool initialPending()const{return initial_;}
  bool waiting()const{return waiting_;}
  bool restartRequired()const{return restart_required_;}
  uint64_t epoch()const{return epoch_;}
  uint64_t anchor()const{return anchor_;}
  int64_t leaseUntil()const{return lease_until_ns_;}
  const std::string& owner()const{return owner_;}
 private:
  std::string producer_,owner_;
  bool initial_=true,waiting_=false,terminal_=false,restart_required_=false;
  uint64_t epoch_=0,anchor_=0;
  int64_t lease_until_ns_=0;
};
}
