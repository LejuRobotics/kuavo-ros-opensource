#include "continuous_reference.h"
#include <cassert>
#include <iostream>
using namespace velocity_ik;
using V=Eigen::Vector3d;
int main() {
  AdaptiveReferenceSettings cfg;cfg.innovation.enabled=true;
  cfg.innovation.orientation.frequency=3.;
  // True large pose steps; zero input velocity. Include an arbitrary 3D axis.
  for(double angle : {.1,.6,1.,1.5,2.5}) {
    AdaptiveRotationReference r;
    const auto identity=Eigen::Quaterniond::Identity();
    r.stepAdaptive(identity,V::Zero(),.002,6.,true,cfg);
    const auto target=innovationExp(angle*V(1.,-.7,.4).normalized());
    double peak=0.,tail=0.,last_unsettled=0.,late_error=0.;
    for(int i=0;i<4000;++i) {
      r.stepAdaptive(target,V::Zero(),.002,6.,true,cfg);
      const double e=innovationLog(target*r.q.conjugate()).norm();
      assert(r.q.coeffs().allFinite() && r.v.allFinite());
      assert(std::abs(r.q.norm()-1.)<1e-12);
      assert((r.v-r.innovation.motion.v-r.innovation.correction.v).norm()<1e-12);
      assert(r.innovation.correction.v.norm()<=cfg.innovation.orientation.speed+1e-8);
      peak=std::max(peak,r.v.norm());
      if(e>.01 || r.v.norm()>.05)last_unsettled=i*.002;
      if(i>=1500){tail=std::max(tail,r.v.norm());late_error=std::max(late_error,e);}
    }
    std::cout<<"step="<<angle<<" peak="<<peak<<" settled="<<last_unsettled
             <<" after3s_speed="<<tail<<" after3s_error="<<late_error<<std::endl;
    assert(last_unsettled<3.);assert(tail<.01);assert(late_error<.01);
  }
  // Multi-axis motion, variable frame spacing (12-22 ms), a missing-frame
  // interval, repeated reversals, then a fixed target with zero velocity.
  AdaptiveRotationReference left,right;
  Eigen::Quaterniond observed=Eigen::Quaterniond::Identity();
  V input=V::Zero(); double last_frame=0.,next_frame=0.,peak=0.,tail=0.;
  for(int i=0;i<6000;++i) {
    const double t=i*.002;
    if(t>=next_frame) {
      if(t<6.) {
        input=V(1.2*std::cos(4*t),.8*std::sin(3*t),.5*std::cos(2*t));
        observed=(innovationExp(input*(t-last_frame))*observed).normalized();
      } else input.setZero();
      last_frame=t;next_frame=t+((i%5==0)?.022:.014);
      if(t>3. && t<3.01)next_frame=t+.07;
    }
    const auto target=innovationExp(input*(t-last_frame))*observed;
    left.stepAdaptive(target,input,.002,6.,true,cfg);
    right.stepAdaptive(target,input,.002,6.,true,cfg);
    assert(left.q.angularDistance(right.q)<1e-12);
    assert((left.v-right.v).norm()<1e-12);
    peak=std::max(peak,left.v.norm());
    if(t>=9.)tail=std::max(tail,left.v.norm());
    assert(left.q.coeffs().allFinite());
  }
  std::cout<<"3D repeated motion peak="<<peak<<" stopped tail="<<tail<<std::endl;
  assert(tail<.01);
  assert(innovationLog(observed*left.q.conjugate()).norm()<.01);
  // Hold noisy source angular velocity at 72Hz; output shape must improve.
  double rough[2]={0.,0.};
  for(int mode=0;mode<2;++mode) {
    AdaptiveRotationReference r;auto test=cfg;
    test.innovation.orientation.frequency=mode?3.:12.;
    Eigen::Quaterniond observation=Eigen::Quaterniond::Identity();V w=V::Zero(),prev=V::Zero();
    double next=0.,stamp=0.;
    for(int i=0;i<6000;++i) {
      double t=i*.002;
      if(t>=next) {
        stamp=t;next=t+1./72.;
        double a=.35*std::sin(2*M_PI*t)+.006*std::sin(30*M_PI*t);
        w=V(.35*2*M_PI*std::cos(2*M_PI*t)+.006*30*M_PI*std::cos(30*M_PI*t),0,0);
        observation=innovationExp(V(a,0,0));
      }
      r.stepAdaptive(innovationExp(w*(t-stamp))*observation,w,.002,mode?6.:12.,true,test);
      if(i>1000)rough[mode]+=(r.v-prev).squaredNorm();
      prev=r.v;
    }
  }
  std::cout<<"angular velocity increment energy baseline/new "<<rough[0]<<" / "<<rough[1]<<std::endl;
  assert(rough[1]<.8*rough[0]);
}
