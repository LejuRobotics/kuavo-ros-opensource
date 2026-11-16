#include "fixed_bandwidth_reference.h"
#include <cassert>
#include <iostream>
using namespace velocity_ik;
int main() {
  constexpr double dt=.002,pi=3.14159265358979323846;
  // Step response: no overshoot and settle to a stopped target.
  AdaptiveVectorReference p; p.reset(Eigen::Vector3d::Zero());
  AdaptiveRotationReference q; q.reset(Eigen::Quaterniond::Identity());
  const Eigen::Quaterniond goal(Eigen::AngleAxisd(.5,Eigen::Vector3d::UnitY()));
  double prev=0.;
  for(int k=0;k<1500;++k) {
    stepFixedVector(p,Eigen::Vector3d::UnitX(),dt,4.);
    stepFixedRotation(q,goal,dt,3.);
    assert(p.p.x()>=prev-1.e-12 && p.p.x()<=1.+1.e-12);prev=p.p.x();
    assert(q.v.y()>=-1.e-9 && q.q.angularDistance(Eigen::Quaterniond::Identity())<=.5+1.e-9);
    assert(std::abs(q.q.norm()-1.)<1.e-12);
  }
  assert(p.v.norm()<1.e-9 && q.v.norm()<1.e-9 && q.q.angularDistance(goal)<1.e-9);
  // Measured gain at the cutoff, with 72Hz ZOH input and 500Hz output.
  for(int rotation=0;rotation<2;++rotation) {
    const double fc=rotation?3.:4.;double input=0.,energy=0.,time_next=0.;int count=0;
    p.valid=false;q.valid=false;
    for(int k=0;k<10000;++k) {
      double t=k*dt;
      if(t+1.e-12>=time_next){input=.01*std::sin(2.*pi*fc*t);time_next+=1./72.;}
      stepFixedVector(p,Eigen::Vector3d(input,0.,0.),dt,fc);
      stepFixedRotation(q,Eigen::Quaterniond(Eigen::AngleAxisd(input,Eigen::Vector3d::UnitX())),dt,fc);
      double output=rotation?fixedRotationError(q.q,Eigen::Quaterniond::Identity()).x():p.p.x();
      if(k>=5000){energy+=output*output;++count;}
    }
    double gain=std::sqrt(2.*energy/count)/.01;
    assert(std::abs(gain-std::sqrt(.5))<.015);
    std::cout<<"cutoff="<<fc<<" rotation="<<rotation<<" gain="<<gain<<'\n';
  }
  // q and -q are the same observation; no artificial long-way rotation.
  auto neg=goal;neg.coeffs()*=-1.;q.reset(goal);
  stepFixedRotation(q,neg,dt,3.);assert(q.v.norm()<1.e-10);
  std::cout<<"PASS fixed bandwidth step, stop, gain and quaternion sign\n";
}
