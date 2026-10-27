#include "continuous_reference.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
using namespace velocity_ik;
constexpr double pi=3.14159265358979323846, dt=.002;

// Also serves as the offline bag-replay runner: the exact production shaper,
// without ROS, publishers, hardware, or an alternative Python implementation.
int main(int argc,char** argv) {
  AdaptiveReferenceSettings cfg;
  if(argc==2) {
    const bool adaptive=std::string(argv[1])=="adaptive";
    const double hz=adaptive ? 12. : std::stod(argv[1]);
    AdaptiveVectorReference p;AdaptiveRotationReference q;
    Eigen::Vector3d target,velocity,omega;double x,y,z,w,h;
    std::cout.precision(12);
    while(std::cin>>h>>target.x()>>target.y()>>target.z()
          >>velocity.x()>>velocity.y()>>velocity.z()>>x>>y>>z>>w
          >>omega.x()>>omega.y()>>omega.z()) {
      const Eigen::Quaterniond rotation(w,x,y,z);
      p.stepAdaptive(target,velocity,h,hz,adaptive,cfg);
      q.stepAdaptive(rotation,omega,h,hz,adaptive,cfg);
      std::cout<<p.p.transpose()<<' '<<p.v.transpose()<<' '
               <<q.q.coeffs().transpose()<<' '<<q.v.transpose()<<' '
               <<p.gate.activity<<' '<<p.gate.frequency<<' '
               <<q.gate.activity<<' '<<q.gate.frequency<<'\n';
    }
    return std::cin.eof() ? 0 : 2;
  }
  assert(cfg.valid(12.));
  auto invalid=cfg;invalid.position.error_full=invalid.position.error_start;
  assert(!invalid.valid(12.));invalid=cfg;invalid.min_frequency=13.;assert(!invalid.valid(12.));
  invalid=cfg;invalid.orientation.speed_full=std::numeric_limits<double>::quiet_NaN();assert(!invalid.valid(12.));
  invalid=cfg;invalid.attack_time_constant=0.;assert(!invalid.valid(12.));

  // Disabled is exactly the legacy fixed-12Hz path with 0.1 velocity guide.
  ContinuousVector legacy;AdaptiveVectorReference disabled;
  ContinuousRotation legacy_q;AdaptiveRotationReference disabled_q;
  for(int i=0;i<3000;++i) {
    const double t=i*dt;Eigen::Vector3d p(.1*std::sin(t),.02*std::cos(3*t),0);
    Eigen::Vector3d v(.1*std::cos(t),-.06*std::sin(3*t),0),w(0,0,.2*std::cos(t));
    Eigen::Quaterniond q(Eigen::AngleAxisd(.2*std::sin(t),Eigen::Vector3d::UnitZ()));
    legacy.step(p,.1*v,dt,12.);disabled.stepAdaptive(p,v,dt,12.,false,cfg);
    legacy_q.step(q,.1*w,dt,12.);disabled_q.stepAdaptive(q,w,dt,12.,false,cfg);
    assert((legacy.p-disabled.p).norm()<1e-12 && (legacy.v-disabled.v).norm()<1e-12);
    assert(legacy_q.q.angularDistance(disabled_q.q)<1e-12);
  }

  // Low-amplitude motion is attenuated; there is no hard dead zone.
  for(double freq:{.5,1.}) {
    AdaptiveVectorReference r;double xx=0.,yy=0.;
    for(int i=0;i<12000;++i) {
      const double t=i*dt,x=.001*std::sin(2*pi*freq*t),v=.001*2*pi*freq*std::cos(2*pi*freq*t);
      r.stepAdaptive(Eigen::Vector3d(x,0,0),Eigen::Vector3d(v,0,0),dt,12.,true,cfg);
      if(i>2000) {xx+=x*x;yy+=r.p.x()*r.p.x();}
      assert(r.p.allFinite() && r.v.allFinite());
    }
    const double gain=std::sqrt(yy/xx);std::cout<<"small amplitude "<<freq<<"Hz gain "<<gain<<'\n';
    assert(gain>.1 && gain<(freq<1. ? .85 : .6));
  }

  // 1Hz, 10cm purposeful motion: amplitude and phase remain close to 12Hz.
  AdaptiveVectorReference r;double ss=0.,cc=0.,min_turn_hz=12.;int n=0;
  for(int i=0;i<8000;++i) {
    const double t=i*dt,x=.1*std::sin(2*pi*t),v=.1*2*pi*std::cos(2*pi*t);
    r.stepAdaptive(Eigen::Vector3d(x,0,0),Eigen::Vector3d(v,0,0),dt,12.,true,cfg);
    if(i>=2000) {ss+=r.p.x()*std::sin(2*pi*t);cc+=r.p.x()*std::cos(2*pi*t);++n;
      if(std::abs(v)<.01) min_turn_hz=std::min(min_turn_hz,r.gate.frequency);}
  }
  const double gain=2*std::hypot(ss,cc)/n/.1,lag=-std::atan2(cc,ss)/(2*pi);
  std::cout<<"large 1Hz gain "<<gain<<" lag_ms "<<lag*1000<<" turn_hz "<<min_turn_hz<<'\n';
  assert(gain>.95 && gain<1.05 && lag<.040 && lag>0. && min_turn_hz>8.);

  // Slow ramp eventually tracks at full speed, not a fixed fraction thereof.
  r=AdaptiveVectorReference{};
  for(int i=0;i<10000;++i)r.stepAdaptive(Eigen::Vector3d(.002*i*dt,0,0),Eigen::Vector3d(.002,0,0),dt,12.,true,cfg);
  assert(std::abs(r.v.x()-.002)<1e-6 && .002*9999*dt-r.p.x()<.004);

  // A slow but large accumulated position error also releases bandwidth.
  AdaptiveReferenceGate g;
  for(int i=0;i<1500;++i)g.update(0.,Eigen::Vector3d::Zero(),dt,12.,cfg,cfg.position,true);
  assert(g.frequency<1.01);
  for(int i=0;i<15;++i)g.update(.02,Eigen::Vector3d::Zero(),dt,12.,cfg,cfg.position,true);
  assert(g.frequency>11.);

  // No cross-arm/channel coupling. Reset does not retain a sluggish gate.
  AdaptiveVectorReference left,right;
  for(int i=0;i<2000;++i) {
    left.stepAdaptive(Eigen::Vector3d(i*dt,0,0),Eigen::Vector3d::UnitX(),dt,12.,true,cfg);
    right.stepAdaptive(Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),dt,12.,true,cfg);
  }
  assert(left.gate.frequency>11.9 && right.gate.frequency<1.01);
  right.valid=false;right.stepAdaptive(Eigen::Vector3d::Ones(),Eigen::Vector3d::Zero(),dt,12.,true,cfg);
  assert(right.gate.frequency>11.8 && (right.p-Eigen::Vector3d::Ones()).norm()<1e-12);

  // Quaternion sign/wrap and rotation equivariance of the vector gate.
  AdaptiveRotationReference rot;AdaptiveVectorReference a,b;
  const Eigen::Quaterniond R(Eigen::AngleAxisd(.7,Eigen::Vector3d(1,2,3).normalized()));
  for(int i=0;i<6000;++i) {
    double t=i*dt;Eigen::Quaterniond q(Eigen::AngleAxisd(3.+.1*t,Eigen::Vector3d::UnitZ()));
    if(i%2)q.coeffs()*=-1.;
    rot.stepAdaptive(q,Eigen::Vector3d(0,0,.1),dt,12.,true,cfg);
    assert(std::abs(rot.q.norm()-1.)<1e-12 && rot.v.allFinite());
    Eigen::Vector3d p(.01*std::sin(t),.02*std::cos(t),.001*t),v(.01*std::cos(t),-.02*std::sin(t),.001);
    a.stepAdaptive(p,v,dt,12.,true,cfg);b.stepAdaptive(R*p,R*v,dt,12.,true,cfg);
    assert((R*a.p-b.p).norm()<1e-10 && std::abs(a.gate.frequency-b.gate.frequency)<1e-8);
  }
  std::cout<<"adaptive reference contracts passed\n";
}
