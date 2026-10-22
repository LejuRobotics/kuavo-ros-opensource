#include "continuous_reference.h"
#include <cassert>
#include <iostream>
#include <limits>
using namespace velocity_ik;
using V=Eigen::Vector3d;
int main() {
  InnovationLimits limits;
  assert(limits.valid());InnovationLimits invalid=limits;invalid.jerk=0.;assert(!invalid.valid());
  // Fixed-bandwidth, geometrically consistent motion: the correction must
  // not chase the intentional nominal velocity-filter lag.
  InnovationVectorReference r;r.reset(V::Zero());V target=V::Zero();
  double peak_c=0.;
  for(int i=0;i<3000;++i) {
    const V input(i<1000?.6:(i<2000?-.6:0.),0.,0.);target+=.002*input;
    r.step(target,input,.002,12.,limits);peak_c=std::max(peak_c,r.correction.v.norm());
    assert(r.p.allFinite() && r.v.allFinite());
  }
  assert(peak_c<1e-9);assert((r.p-target).norm()<1e-9);
  // True observation position steps cannot jump the output position/velocity;
  // correction budgets constrain actual state, not a clipped publish value.
  r.reset(V::Zero());target=V(.006,0,0);double peak=0.;
  V prev_a=V::Zero(),prev_v=V::Zero(),prev_p=V::Zero();
  for(int i=0;i<2500;++i) {
    r.step(target,V::Zero(),.002,12.,limits);peak=std::max(peak,r.v.norm());
    assert(r.correction.v.norm()<=limits.speed+1e-9);
    assert(r.correction.a.norm()<=limits.acceleration+1e-9);
    assert((r.correction.a-prev_a).norm()<=limits.jerk*.002+1e-8);
    const V integral=prev_v*.002+.5*prev_a*.002*.002+(r.correction.a-prev_a)*(.002*.002/6.);
    assert((r.p-prev_p-integral).norm()<1e-12);
    prev_a=r.correction.a;prev_v=r.v;prev_p=r.p;
  }
  std::cerr<<"six-mm step peak "<<peak<<" final error "<<(r.p-target).norm()<<std::endl;
  assert(peak<.14);assert((r.p-target).norm()<1e-8);
  // Saturation/reversal, no hidden velocity or acceleration windup.
  InnovationCorrection c;V last_a=V::Zero();
  for(int i=0;i<15000;++i) {
    double dt=(i%11==0)?.004:.002;
    const double sign=((i/317)%2)?1.:-1.;
    c.step(V(sign*3.,std::sin(i*.1)*2.,-sign*3.),dt,limits);
    assert(c.v.norm()<=limits.speed+1e-8);
    assert(c.a.norm()<=limits.acceleration+1e-8);
    assert((c.a-last_a).norm()<=limits.jerk*dt+1e-8);last_a=c.a;
  }
  // Rotation continuity, signs, multi-axis finite states and settling.
  InnovationRotationReference rot;rot.reset(Eigen::Quaterniond::Identity());
  const auto q=innovationExp(V(.03,-.02,.01));
  for(int i=0;i<4000;++i) {
    Eigen::Quaterniond input=q;if(i%2)input.coeffs()*=-1.;
    rot.step(input,V::Zero(),.002,12.,InnovationReferenceSettings{}.orientation);
    assert(std::abs(rot.q.norm()-1.)<1e-12);
  }
  assert(innovationLog(q*rot.q.conjugate()).norm()<1e-7);
  // New-frame audit evaluates the two anchors at the same current time.
  ReferenceInnovationAudit audit;FramePrediction fp;FramePredictionSettings pc;
  const double origin=1e12;fp.update(origin,origin,1.,origin,pc);
  audit.update(fp,pc,origin,V::Zero(),Eigen::Quaterniond::Identity(),V(1,0,0),V::Zero());
  fp.update(origin+14e6,origin+14e6,1.014,origin+16e6,pc);
  audit.update(fp,pc,origin+16e6,V(.014,0,0),Eigen::Quaterniond::Identity(),V(1,0,0),V::Zero());
  assert(audit.dp.norm()<1e-9);
  fp.update(origin+28e6,origin+28e6,1.028,origin+30e6,pc);
  audit.update(fp,pc,origin+30e6,V(.034,0,0),Eigen::Quaterniond::Identity(),V(1,0,0),V::Zero());
  assert((audit.dp-V(.006,0,0)).norm()<1e-9);
  // Full adaptive, ideal high-activity 1Hz motion: gain and phase acceptance.
  AdaptiveReferenceSettings frequency_cfg;frequency_cfg.innovation.enabled=true;
  AdaptiveVectorReference frequency_ref;double sin_sum=0.,cos_sum=0.;int count=0;
  for(int i=0;i<10000;++i) {
    const double t=.002*i,phase=2.*3.14159265358979323846*t;
    frequency_ref.stepAdaptive(V(.1*std::sin(phase),0,0),V(.2*3.14159265358979323846*std::cos(phase),0,0),.002,12.,true,frequency_cfg);
    if(i>=2500){sin_sum+=frequency_ref.p.x()*std::sin(phase);cos_sum+=frequency_ref.p.x()*std::cos(phase);++count;}
  }
  const double gain=2.*std::hypot(sin_sum,cos_sum)/count/.1;
  const double delay=-std::atan2(cos_sum,sin_sum)/(2.*3.14159265358979323846);
  assert(gain>.95 && gain<1.05);assert(delay>=0. && delay<.04);
  // Same fast start/reversal fixture as the existing reference regression.
  AdaptiveReferenceSettings previous_cfg,new_cfg;new_cfg.innovation.enabled=true;
  AdaptiveVectorReference previous,current;
  double position=0.,onset_old=-1.,onset_new=-1.,reverse_old=-1.,reverse_new=-1.;
  for(int i=0;i<2500;++i) {
    const double speed=i<1500?0.:(i<1600?.6:-.6);position+=speed*.002;
    previous.stepAdaptive(V(position,0,0),V(speed,0,0),.002,12.,true,previous_cfg);
    current.stepAdaptive(V(position,0,0),V(speed,0,0),.002,12.,true,new_cfg);
    if(i>=1500 && onset_old<0. && previous.v.x()>.54)onset_old=(i-1500)*.002;
    if(i>=1500 && onset_new<0. && current.v.x()>.54)onset_new=(i-1500)*.002;
    if(i>=1600 && reverse_old<0. && previous.v.x()<-.54)reverse_old=(i-1600)*.002;
    if(i>=1600 && reverse_new<0. && current.v.x()<-.54)reverse_new=(i-1600)*.002;
  }
  std::cout<<"motion 90% onset old/new "<<onset_old<<"/"<<onset_new
           <<" reversal old/new "<<reverse_old<<"/"<<reverse_new<<std::endl;
  assert(onset_new>=0. && onset_new-onset_old<=.012001);
  assert(reverse_new>=0. && reverse_new-reverse_old<=.012001);
  // Low activity remains filtered, without a static-intent detector/deadband.
  for(double hz:{.5,1.}) {
    AdaptiveVectorReference quiet;double xx=0.,yy=0.;
    for(int i=0;i<12000;++i) {
      const double phase=2.*3.14159265358979323846*hz*i*.002;
      const double x=.001*std::sin(phase),vx=.001*2.*3.14159265358979323846*hz*std::cos(phase);
      quiet.stepAdaptive(V(x,0,0),V(vx,0,0),.002,12.,true,new_cfg);
      if(i>=2000){xx+=x*x;yy+=quiet.p.x()*quiet.p.x();}
    }
    const double small_gain=std::sqrt(yy/xx);
    std::cout<<"small 1mm "<<hz<<"Hz gain="<<small_gain<<std::endl;
    assert(small_gain>.1 && small_gain<(hz<1.?.85:.65));
  }
  // Adaptive parameter changes preserve state; nonfinite input is rejected.
  AdaptiveReferenceSettings cfg;cfg.innovation.enabled=true;
  AdaptiveVectorReference ar;
  for(int i=0;i<3000;++i) {
    double t=.002*i;V vel(.4*std::cos(t*6.),0,0),pos(.4/6.*std::sin(t*6.),0,0);
    ar.stepAdaptive(pos,vel,.002,12.,true,cfg);assert(ar.p.allFinite() && ar.v.allFinite());
  }
  ar.valid=false;ar.stepAdaptive(V(2,0,0),V::Zero(),.002,12.,true,cfg);
  assert((ar.p-V(2,0,0)).norm()<1e-12 && ar.v.norm()<1e-12);
  bool threw=false;try {r.step(V::Zero(),V::Zero(),0.,12.,limits);}catch(const std::invalid_argument&){threw=true;}assert(threw);
  threw=false;try {r.step(V::Constant(std::numeric_limits<double>::quiet_NaN()),V::Zero(),.002,12.,limits);}catch(const std::invalid_argument&){threw=true;}assert(threw);
  std::cout<<"innovation reference tests passed; ideal correction peak="<<peak_c
           <<" six-mm correction speed peak="<<peak<<" 1Hz gain="<<gain<<" lag_s="<<delay<<std::endl;
}
