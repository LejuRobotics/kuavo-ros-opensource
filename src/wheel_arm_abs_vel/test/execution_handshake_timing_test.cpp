#include "wheel_arm_abs_vel/humanoid_controllers/ClockedReferenceFreshness.h"
#include "wheel_arm_abs_vel/humanoid_controllers/ExecutionProducerLease.h"
#include "wheel_arm_abs_vel/humanoid_controllers/ExecutionReferenceOwner.h"
#include <Eigen/Core>
#include <cassert>
#include <memory>
#include <fstream>
#include <iostream>
#include <vector>
struct Ref {
 Eigen::VectorXd q=Eigen::VectorXd::Zero(21),dq=q,ddq=q,
   stopping_acceleration=Eigen::VectorXd::Constant(21,25);
 std::string producer_instance="p1",execution_owner_instance;
 uint64_t reference_seq=1,execution_epoch=0,recovery_anchor_seq=0;
 int64_t producer_lease_deadline_ns=0,generated_ns=0,ros_ns=0;
 bool safety_braking=false,takeover_active=true;
};
using P=velocity_ik_execution::ExecutionProducerLease;
using O=velocity_ik_execution::ExecutionReferenceOwner<Ref>;
using G=velocity_ik_execution::ClockedReferenceFreshness;
constexpr int64_t S=1000000000000LL,L=50000000,H=2000000,ROS=1700000000000000000LL;
struct Packet {P::Feedback f;Ref state;uint64_t seq=0;int64_t ros=0;};
struct Case {std::string name;bool drop_hold=false,drop_ack=false,echo_finite=false,
 echo_permanent=false,restart_producer=false,restart_owner=false,clock_back=false,
 clock_forward=false,late_old=false,terminal=false;};
int main(){
 std::vector<Case> cases;
 Case a;a.name="drop_one_HOLD";a.drop_hold=true;cases.push_back(a);
 a={};a.name="drop_one_ACTIVE_ACK";a.drop_ack=true;cases.push_back(a);
 a={};a.name="finite_echo_outage_forward_normal";a.echo_finite=true;cases.push_back(a);
 a={};a.name="permanent_echo_outage_forward_normal";a.echo_permanent=true;cases.push_back(a);
 a={};a.name="producer_restart";a.restart_producer=true;cases.push_back(a);
 a={};a.name="owner_restart_requires_coordinated_restart";a.restart_owner=true;cases.push_back(a);
 a={};a.name="ROS_back_60_seconds";a.clock_back=true;cases.push_back(a);
 a={};a.name="ROS_forward_then_return";a.clock_forward=true;cases.push_back(a);
 a={};a.name="late_old_epoch_anchor";a.late_old=true;cases.push_back(a);
 a={};a.name="normal_exit_during_stop_first";a.terminal=true;cases.push_back(a);
 for(const auto& c:cases){
  std::string producer="p1",owner="wbc1";
  auto p=std::make_unique<P>(producer);auto o=std::make_unique<O>(owner);
  auto fg=std::make_unique<G>("boot");G source_gate("boot");
  Ref local,cached,out,previous;bool have_cache=false,owned=false,have_out=false,have_packet=false;
  Packet packet,saved;bool have_saved=false,dropped_hold=false,dropped_ack=false,terminal=false;
  uint64_t fbseq=0;int producer_advances=0,owner_advances=0,post_fault_advances=0,anchors=0,rejected_old=0;
  std::ofstream trace(c.name+".csv");
  trace<<"tick,producer_advance,owner_advance,epoch,anchor,initial,waiting,restart,q_producer,q_owner,v_owner,grant_until\n";
  for(int k=0;k<400;++k){
   const int64_t t=S+k*H;
   int64_t ros=ROS+k*H;
   if(c.clock_back && k>=50)ros-=60000000000LL;
   if(c.clock_forward && k>=50 && k<100)ros+=60000000000LL;
   if(c.restart_producer && k==50){
    producer="p2";p=std::make_unique<P>(producer);fg=std::make_unique<G>("boot");
    local=Ref{};
   }
   if(c.restart_owner && k==50){
    owner="wbc2";o=std::make_unique<O>(owner);fbseq=0;have_out=false;
   }
   if(c.terminal && k==76){terminal=true;p->terminalStop();}
   if(have_packet){
    const bool echo_out=((c.echo_finite && k>=50 && k<130) || (c.echo_permanent && k>=50));
    bool drop=echo_out;
    if(c.drop_hold && !dropped_hold && packet.f.phase==P::Phase::Hold){drop=true;dropped_hold=true;}
    if(c.drop_ack && !dropped_ack && packet.f.epoch && packet.f.phase==P::Phase::Active){drop=true;dropped_ack=true;}
    if(!drop && packet.f.producer==producer){
     auto accepted=fg->accept("boot",packet.f.owner,1,1,packet.seq,packet.f.generated_ns,t,packet.ros,ros,L);
     if(accepted==G::Result::NewSample){
      const bool stopped=packet.state.q.allFinite() && packet.state.dq.isZero(0) && packet.state.ddq.isZero(0);
      const bool equal=stopped && local.dq.isZero(0) && local.ddq.isZero(0) &&
        (local.q-packet.state.q).cwiseAbs().maxCoeff()<=1e-10;
      auto action=p->observe(packet.f,stopped,equal);
      if(action==P::Result::InstallAnchor){
       local.q=packet.state.q;local.dq=packet.state.dq;local.ddq=packet.state.ddq;++anchors;
      }
     }
    }
    if(!have_saved && packet.f.phase==P::Phase::Hold){saved=packet;have_saved=true;}
   }
   if(c.late_old && k==220 && have_saved){
    const auto rejected=fg->accept("boot",saved.f.owner,1,1,saved.seq,saved.f.generated_ns,t,saved.ros,ros,L);
    assert(rejected!=G::Result::NewSample);++rejected_old;
   }
   const bool advance=p->canAdvance(t,H);
   assert(!terminal || !advance);
   bool send=false;
   if(advance){
    const auto v0=local.dq;
    local.dq=(local.dq.array()+.05).min(.1).matrix();
    local.ddq=(local.dq-v0)/.002;local.q+=.001*(v0+local.dq);
    ++local.reference_seq;send=true;++producer_advances;if(k>200)++post_fault_advances;
   }else if(!terminal && p->initialPending()){
    send=true; // same seed sequence and generation time on repetitions
   }else if(!terminal && p->waiting() && p->anchor()){
    assert(local.dq.isZero(0) && local.ddq.isZero(0));
    ++local.reference_seq;send=true;
   }
   local.producer_instance=producer;local.execution_owner_instance=p->owner();
   local.execution_epoch=p->epoch();local.recovery_anchor_seq=p->anchor();
   if(send && (!local.generated_ns || advance || (p->waiting()&&p->anchor()))){
    local.generated_ns=t;local.ros_ns=ros;local.producer_lease_deadline_ns=t+L;
   }
   const bool forced_forward_loss=(c.drop_hold||c.drop_ack||c.late_old||c.terminal) && k>=50 && k<90;
   if(send && !forced_forward_loss){
    auto accepted=source_gate.accept("boot",producer,1,1,local.reference_seq,local.generated_ns,t,local.ros_ns,ros,L);
    if(accepted==G::Result::NewSample){cached=local;have_cache=true;owned=true;}
   }
   const bool fresh=have_cache && t-cached.generated_ns<=L;
   const Ref old=out;const bool was_out=have_out;
   const bool usable=o->resolve(cached,fresh,owned,t,L,.002,out);
   const bool owner_advance=usable && o->degrading() && was_out && out.reference_seq!=old.reference_seq;
   if(owner_advance){
    assert((out.q-old.q-.001*(old.dq+out.dq)).cwiseAbs().maxCoeff()<1e-12);
    assert((out.ddq-(out.dq-old.dq)/.002).cwiseAbs().maxCoeff()<1e-12);
    ++owner_advances;
   }
   assert(!(advance && owner_advance));
   if(usable)have_out=true;
   packet={};packet.f.producer=have_cache?cached.producer_instance:producer;
   packet.f.owner=owner;packet.f.epoch=o->epoch();packet.f.anchor=o->anchor();
   packet.f.generated_ns=t;packet.f.lease_until_ns=o->producerLeaseUntil();
   packet.f.valid=usable;packet.seq=++fbseq;packet.ros=ros;
   packet.f.phase=o->phase()==O::Phase::Producer?P::Phase::Active:
     (o->phase()==O::Phase::StoppedAnchor?P::Phase::Hold:P::Phase::Braking);
   packet.state=usable?out:cached;have_packet=have_cache;
   trace<<k<<','<<advance<<','<<owner_advance<<','<<p->epoch()<<','<<p->anchor()<<','<<p->initialPending()<<','<<p->waiting()<<','<<p->restartRequired()<<','<<local.q[0]<<','<<(usable?out.q[0]:0)<<','<<(usable?out.dq[0]:0)<<','<<p->leaseUntil()<<'\n';
  }
  if(c.echo_permanent){assert(p->waiting() && o->anchor() && out.dq.isZero(0));assert(post_fault_advances==0);}
  else if(c.restart_owner){assert(p->restartRequired() && p->waiting());assert(post_fault_advances==0);}
  else if(c.terminal){assert(p->waiting());assert(post_fault_advances==0);}
  else{assert(post_fault_advances>0 && !p->waiting());}
  if(c.drop_hold)assert(dropped_hold);
  if(c.drop_ack)assert(dropped_ack);
  if(c.late_old)assert(rejected_old==1);
  std::cout<<"PASS "<<c.name<<" producer_steps="<<producer_advances<<" owner_steps="<<owner_advances<<" anchors="<<anchors<<" resume_steps="<<post_fault_advances<<std::endl;
 }
}
