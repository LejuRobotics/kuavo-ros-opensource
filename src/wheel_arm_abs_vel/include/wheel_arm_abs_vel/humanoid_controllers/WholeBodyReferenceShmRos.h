#pragma once
#include "WholeBodyReferenceShm.h"
#include <wheel_arm_abs_vel/WholeBodyIkReference.h>
#include <ros/ros.h>
#include <cmath>
#include <stdexcept>
#include <algorithm>
namespace kuavo_reference_shm {
struct Payload {
static constexpr uint64_t ABI_TAG=0xabb458e7489e217aULL;
uint32_t schema{1}, header_seq{};
uint64_t header_stamp_ns{};
char header_frame[128]{};
uint32_t dof_layout_version{};
char dof_names[21][48]{};
uint8_t dof_groups[21]{};
uint64_t session_id{};
uint64_t generation{};
uint64_t reference_seq{};
uint8_t authority_state{};
uint8_t valid{};
uint8_t takeover_requested{};
uint8_t takeover_active{};
char world_frame[128]{};
char session_frame[128]{};
uint8_t session_anchor_valid{};
uint8_t base_feedback_valid{};
double q[21]{};
double dq[21]{};
double ddq[21]{};
double task_scale{};
uint8_t safety_braking{};
uint8_t emergency_dynamics_used{};
uint64_t execution_epoch{};
uint64_t recovery_anchor_seq{};
double stopping_acceleration[21]{};
double position_lower[21]{};
double position_upper[21]{};
char execution_clock_domain[128]{};
char producer_instance[128]{};
char execution_owner_instance[128]{};
int64_t generated_monotonic_ns{};
uint64_t owner_feedback_seq{};
int64_t producer_lease_deadline_ns{};
};
template<std::size_t N> inline bool copyString(char (&to)[N],const std::string& from){
 if(from.size()>=N)return false;std::memset(to,0,N);std::memcpy(to,from.data(),from.size());return true;
}
template<std::size_t N> inline bool readString(std::string& to,const char (&from)[N]){
 const void* end=std::memchr(from,0,N);if(!end)return false;to.assign(from,static_cast<const char*>(end)-from);return true;
}
inline bool encode(const wheel_arm_abs_vel::WholeBodyIkReference& m,Payload& p){
p=Payload{};p.header_seq=m.header.seq;p.header_stamp_ns=m.header.stamp.toNSec();
if(!copyString(p.header_frame,m.header.frame_id))return false;
p.dof_layout_version=m.dof_layout_version;
if(m.dof_names.size()!=21)return false;for(int i=0;i<21;++i)if(!copyString(p.dof_names[i],m.dof_names[i]))return false;
if(m.dof_groups.size()!=21)return false;for(int i=0;i<21;++i)p.dof_groups[i]=m.dof_groups[i];
p.session_id=m.session_id;
p.generation=m.generation;
p.reference_seq=m.reference_seq;
p.authority_state=m.authority_state;
p.valid=m.valid;
p.takeover_requested=m.takeover_requested;
p.takeover_active=m.takeover_active;
if(!copyString(p.world_frame,m.world_frame))return false;
if(!copyString(p.session_frame,m.session_frame))return false;
p.session_anchor_valid=m.session_anchor_valid;
p.base_feedback_valid=m.base_feedback_valid;
if(m.q.size()!=21)return false;for(int i=0;i<21;++i)p.q[i]=m.q[i];
for(double x:p.q)if(!std::isfinite(x))return false;
if(m.dq.size()!=21)return false;for(int i=0;i<21;++i)p.dq[i]=m.dq[i];
for(double x:p.dq)if(!std::isfinite(x))return false;
if(m.ddq.size()!=21)return false;for(int i=0;i<21;++i)p.ddq[i]=m.ddq[i];
for(double x:p.ddq)if(!std::isfinite(x))return false;
p.task_scale=m.task_scale;
if(!std::isfinite(p.task_scale))return false;
p.safety_braking=m.safety_braking;
p.emergency_dynamics_used=m.emergency_dynamics_used;
p.execution_epoch=m.execution_epoch;
p.recovery_anchor_seq=m.recovery_anchor_seq;
if(m.stopping_acceleration.size()!=21)return false;for(int i=0;i<21;++i)p.stopping_acceleration[i]=m.stopping_acceleration[i];
for(double x:p.stopping_acceleration)if(!std::isfinite(x))return false;
if(m.position_lower.size()!=21)return false;for(int i=0;i<21;++i)p.position_lower[i]=m.position_lower[i];
for(double x:p.position_lower)if(!std::isfinite(x))return false;
if(m.position_upper.size()!=21)return false;for(int i=0;i<21;++i)p.position_upper[i]=m.position_upper[i];
for(double x:p.position_upper)if(!std::isfinite(x))return false;
if(!copyString(p.execution_clock_domain,m.execution_clock_domain))return false;
if(!copyString(p.producer_instance,m.producer_instance))return false;
if(!copyString(p.execution_owner_instance,m.execution_owner_instance))return false;
p.generated_monotonic_ns=m.generated_monotonic_ns;
p.owner_feedback_seq=m.owner_feedback_seq;
p.producer_lease_deadline_ns=m.producer_lease_deadline_ns;
return true;
}
inline bool decode(const Payload& p,wheel_arm_abs_vel::WholeBodyIkReference& m){
if(p.schema!=1)return false;m.header.seq=p.header_seq;m.header.stamp.fromNSec(p.header_stamp_ns);
if(!readString(m.header.frame_id,p.header_frame))return false;
m.dof_layout_version=p.dof_layout_version;
m.dof_names.resize(21);for(int i=0;i<21;++i)if(!readString(m.dof_names[i],p.dof_names[i]))return false;
m.dof_groups.assign(p.dof_groups,p.dof_groups+21);
m.session_id=p.session_id;
m.generation=p.generation;
m.reference_seq=p.reference_seq;
m.authority_state=p.authority_state;
if(p.valid>1)return false;m.valid=p.valid;
if(p.takeover_requested>1)return false;m.takeover_requested=p.takeover_requested;
if(p.takeover_active>1)return false;m.takeover_active=p.takeover_active;
if(!readString(m.world_frame,p.world_frame))return false;
if(!readString(m.session_frame,p.session_frame))return false;
if(p.session_anchor_valid>1)return false;m.session_anchor_valid=p.session_anchor_valid;
if(p.base_feedback_valid>1)return false;m.base_feedback_valid=p.base_feedback_valid;
m.q.assign(p.q,p.q+21);
for(double x:p.q)if(!std::isfinite(x))return false;
m.dq.assign(p.dq,p.dq+21);
for(double x:p.dq)if(!std::isfinite(x))return false;
m.ddq.assign(p.ddq,p.ddq+21);
for(double x:p.ddq)if(!std::isfinite(x))return false;
m.task_scale=p.task_scale;
if(!std::isfinite(p.task_scale))return false;
if(p.safety_braking>1)return false;m.safety_braking=p.safety_braking;
if(p.emergency_dynamics_used>1)return false;m.emergency_dynamics_used=p.emergency_dynamics_used;
m.execution_epoch=p.execution_epoch;
m.recovery_anchor_seq=p.recovery_anchor_seq;
m.stopping_acceleration.assign(p.stopping_acceleration,p.stopping_acceleration+21);
for(double x:p.stopping_acceleration)if(!std::isfinite(x))return false;
m.position_lower.assign(p.position_lower,p.position_lower+21);
for(double x:p.position_lower)if(!std::isfinite(x))return false;
m.position_upper.assign(p.position_upper,p.position_upper+21);
for(double x:p.position_upper)if(!std::isfinite(x))return false;
if(!readString(m.execution_clock_domain,p.execution_clock_domain))return false;
if(!readString(m.producer_instance,p.producer_instance))return false;
if(!readString(m.execution_owner_instance,p.execution_owner_instance))return false;
m.generated_monotonic_ns=p.generated_monotonic_ns;
m.owner_feedback_seq=p.owner_feedback_seq;
m.producer_lease_deadline_ns=p.producer_lease_deadline_ns;
return true;
}
class RosTransport {
 public:
  void configure(const ros::NodeHandle& nh,const std::string& topic,bool producer){
   nh.param(topic+"/transport",mode_,std::string("ros"));
   int refkey=344421,ownerkey=344422;
   nh.param(topic+"/shm_reference_key",refkey,refkey);nh.param(topic+"/shm_owner_key",ownerkey,ownerkey);
   if(mode_!="ros" && mode_!="shm")throw std::runtime_error("reference transport must be ros or shm");
   if(mode_=="ros")return;
   preparePool(reference_pool_);preparePool(owner_pool_);
   if(refkey==ownerkey || refkey<=0 || ownerkey<=0)throw std::runtime_error("independent positive reference/owner SHM keys required");
   using RefRole=Snapshot<Payload,1>::Role;using OwnerRole=Snapshot<Payload,2>::Role;
   if(!reference_.initialize(refkey,producer?RefRole::Writer:RefRole::Reader))
     throw std::runtime_error("reference SHM startup: "+reference_.error());
   if(!owner_.initialize(ownerkey,producer?OwnerRole::Reader:OwnerRole::Writer))
     throw std::runtime_error("owner SHM startup: "+owner_.error());
   ROS_INFO("[WholeBodyReferenceTransport] %s SHM ref_key=%d owner_key=%d ABI=%zu",
            producer?"producer":"consumer",refkey,ownerkey,sizeof(Payload));
  }
  bool shm()const{return mode_=="shm";}
  bool writeReference(const wheel_arm_abs_vel::WholeBodyIkReference& msg){
   Payload p{};return encode(msg,p)&&reference_.write(p);
  }
  bool readReference(wheel_arm_abs_vel::WholeBodyIkReference::ConstPtr& msg){
   Payload p{};return reference_.readIfUpdated(p)&&decodePool(p,reference_pool_,msg);
  }
  bool writeOwner(const wheel_arm_abs_vel::WholeBodyIkReference& msg){
   Payload p{};return encode(msg,p)&&owner_.write(p);
  }
  bool readOwner(wheel_arm_abs_vel::WholeBodyIkReference::ConstPtr& msg){
   Payload p{};return owner_.readIfUpdated(p)&&decodePool(p,owner_pool_,msg);
  }
  uint64_t referenceRead()const{return reference_.lastRead();}
  uint64_t referenceWritten()const{return reference_.lastWrite();}
 private:
  using Pool=std::array<wheel_arm_abs_vel::WholeBodyIkReference::Ptr,3>;
  static void preparePool(Pool& pool){
   for(auto& slot:pool){slot.reset(new wheel_arm_abs_vel::WholeBodyIkReference());
    auto& m=*slot;m.header.frame_id.reserve(127);
    m.dof_names.resize(21);for(auto& str:m.dof_names)str.reserve(47);
m.dof_groups.resize(21);
m.world_frame.reserve(127);
m.session_frame.reserve(127);
m.q.resize(21);
m.dq.resize(21);
m.ddq.resize(21);
m.stopping_acceleration.resize(21);
m.position_lower.resize(21);
m.position_upper.resize(21);
m.execution_clock_domain.reserve(127);
m.producer_instance.reserve(127);
m.execution_owner_instance.reserve(127);
   }
  }
  static bool decodePool(const Payload& p,Pool& pool,wheel_arm_abs_vel::WholeBodyIkReference::ConstPtr& out){
   // Never overwrite a message retained by the current owner/producer cache.
   // Three startup-preallocated slots give bounded, allocation-free decoding.
   for(auto& slot:pool)if(slot.use_count()==1){
    if(!decode(p,*slot))return false;out=slot;return true;
   }
   return false;
  }
  Pool reference_pool_{},owner_pool_{};
  std::string mode_{"ros"};Snapshot<Payload,1> reference_;Snapshot<Payload,2> owner_;
};
} // namespace kuavo_reference_shm
