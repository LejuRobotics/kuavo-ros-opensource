#include "wheel_arm_abs_vel/humanoid_controllers/ReferenceFreshness.h"
#include "wheel_arm_abs_vel/humanoid_controllers/ExecutionReferenceOwner.h"
#include <Eigen/Core>
#include <cassert>
#include <iostream>
struct Ref {
 Eigen::VectorXd q,dq,ddq,stopping_acceleration,position_lower,position_upper;
 uint64_t reference_seq=1,execution_epoch=0,recovery_anchor_seq=0;
 bool safety_braking=false,takeover_active=true;
 std::string producer_instance,execution_owner_instance;
 int64_t producer_lease_deadline_ns=0;
};
int main() {
 using Gate=velocity_ik_execution::ReferenceFreshness; using Result=Gate::Result;
 Gate gate; const int64_t t=1000000000, lease=50000000;
 assert(gate.accept(1,1,1,t,t+1000000,lease)==Result::NewSample);
 assert(gate.accept(1,1,1,t,t+49000000,lease)!=Result::NewSample);
 assert(gate.accept(1,1,1,t+49000000,t+49000000,lease)!=Result::NewSample);
 assert(gate.accept(1,1,2,t,t+1000000,lease)==Result::OldStamp);
 assert(gate.accept(1,1,2,t-1000000000,t,lease)!=Result::NewSample);
 assert(gate.accept(1,1,2,t+100000000,t+1000000,lease)==Result::ExpiredOrFuture);
 assert(gate.accept(1,1,2,t+2000000,t+3000000,lease)==Result::NewSample);
 assert(gate.accept(1,0,3,t+4000000,t+5000000,lease)!=Result::NewSample);
 assert(gate.accept(1,2,1,t+4000000,t+5000000,lease)==Result::NewSample);
 assert(gate.accept(2,1,1,t+1000000,t+5000000,lease)==Result::OldStamp);
 assert(gate.accept(2,1,1,t+6000000,t+7000000,lease)==Result::NewSample);
 int segments=0;
 for (double v0: {0.0,0.01,0.17948516,0.45,0.8,2.0,3.0}) {
  for (double brake: {1.0,1.5,3.0,8.0,18.0,25.0}) {
   for (double sign: {-1.0,1.0}) {
    constexpr double h=.002;
    Ref in; in.dq=Eigen::VectorXd::Constant(21,sign*v0);
    in.ddq=Eigen::VectorXd::Zero(21);
    in.stopping_acceleration=Eigen::VectorXd::Constant(21,brake);
    in.q=Eigen::VectorXd::Constant(21,-sign*(v0*v0/(2*brake)+brake*h*h/8+1e-8));
    velocity_ik_execution::ExecutionReferenceOwner<Ref> owner;
    bool owned=true; Ref out; int64_t ns=1000000000;
    assert(owner.resolve(in,true,owned,ns,lease,h,out));
    // Repeated consumption is an unchanged sample, not an extra IK segment.
    for(int i=0;i<4;++i) {
     ns+=2000000; assert(owner.resolve(in,true,owned,ns,lease,h,out));
     assert((out.q-in.q).norm()==0 && (out.dq-in.dq).norm()==0);
    }
    Ref previous=out;
    // A WBC gap is independently detected even if the producer is still fresh.
    ns+=lease+1;
    assert(owner.resolve(in,true,owned,ns,lease,h,out) && owner.degrading());
    for(int i=0; i<10000;++i) {
     assert((out.q-previous.q-.5*h*(previous.dq+out.dq)).cwiseAbs().maxCoeff()<1e-12);
     assert((out.ddq-(out.dq-previous.dq)/h).cwiseAbs().maxCoeff()<1e-12);
     assert(out.dq.cwiseAbs().maxCoeff()<=previous.dq.cwiseAbs().maxCoeff()+1e-15);
     assert(out.ddq.cwiseAbs().maxCoeff()<=brake+1e-10);
     assert(sign*out.q.maxCoeff()<=1e-10);
     ++segments;
     if(owner.anchor()) break;
     previous=out;ns+=2000000;
     // A fresh original producer sample must not reclaim the output.
     assert(owner.resolve(in,true,owned,ns,lease,h,out));
    }
    assert(owner.anchor() && out.dq.norm()==0 && out.ddq.norm()==0);
    const Ref stopped=out;
    const auto anchor=owner.anchor();
    for(int i=0;i<10;++i) {
     ns+=2000000; owner.resolve(in,true,owned,ns,lease,h,out);
     assert(owner.degrading() && owner.anchor()==anchor && (out.q-stopped.q).norm()==0);
    }
    Ref bad=stopped;bad.q.array()+=.001;
    ns+=2000000;owner.resolve(bad,true,owned,ns,lease,h,out);
    assert(owner.degrading());
    Ref echo=stopped;echo.reference_seq+=100;
    ns+=2000000;owner.resolve(echo,true,owned,ns,lease,h,out);
    assert(!owner.degrading() && (out.q-stopped.q).norm()==0);
   }
  }
 }
 std::cout<<"PASS freshness counterexamples; stopping/hold/handshake segments="<<segments<<"\n";
}
