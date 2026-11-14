#include "third_order_position_reference.h"
#include "fixed_bandwidth_reference.h"
#include <cassert>
#include <fstream>
#include <iomanip>
#include <iostream>
using namespace velocity_ik;
int main(int argc,char**argv) {
  if(argc==3) {
    std::ifstream in(argv[1]);std::ofstream out(argv[2]);
    if(!in || !out)return 2;
    ContinuousVector r;double dt;Eigen::Vector3d target;
    out<<std::setprecision(17);
    while(in>>dt>>target.x()>>target.y()>>target.z()) {
      stepThirdOrderPosition(r,target,dt,5.);
      out<<r.p.transpose()<<' '<<r.v.transpose()<<' '<<r.a.transpose()<<'\n';
    }
    return in.eof()?0:3;
  }
  constexpr double dt=.002,pi=3.14159265358979323846;
  ContinuousVector r,old;r.reset(Eigen::Vector3d::Zero());old=r;
  double prev=0.,peak_a=0.,peak_delta=0.,old_peak_delta=0.;
  Eigen::Vector3d last_a=Eigen::Vector3d::Zero(),old_last=last_a;
  for(int k=0;k<1000;++k) {
    stepThirdOrderPosition(r,Eigen::Vector3d(.1,0,0),dt,5.);
    stepFixedVector(old,Eigen::Vector3d(.1,0,0),dt,5.);
    assert(r.p.x()>=prev-1e-13 && r.p.x()<=.1+1e-13);
    peak_a=std::max(peak_a,r.a.norm());
    peak_delta=std::max(peak_delta,(r.a-last_a).norm());
    old_peak_delta=std::max(old_peak_delta,(old.a-old_last).norm());
    prev=r.p.x();last_a=r.a;old_last=old.a;
  }
  assert((r.p-Eigen::Vector3d(.1,0,0)).norm()<1e-12);
  assert(r.v.norm()<1e-10 && r.a.norm()<1e-9);
  assert(peak_a<100. && peak_delta<.25*old_peak_delta);
  // Observation corrections cannot instantaneously change acceleration.
  r.p=Eigen::Vector3d(.1,-.2,.3);r.v=Eigen::Vector3d(.4,.1,-.2);
  r.a=Eigen::Vector3d(.3,-.2,.4);const auto before=r;
  stepThirdOrderPosition(r,before.p+Eigen::Vector3d(.1,0,0),1e-8,5.);
  assert((r.a-before.a).norm()<.001);
  assert((r.v-before.v).norm()<1e-7);
  // Exact ZOH stepping must be independent of tick subdivision.
  auto one=before,two=before;const Eigen::Vector3d target(-.3,.4,.7);
  stepThirdOrderPosition(one,target,.004,5.);
  stepThirdOrderPosition(two,target,.002,5.);stepThirdOrderPosition(two,target,.002,5.);
  assert((one.p-two.p).norm()<1e-12 && (one.v-two.v).norm()<1e-11 && (one.a-two.a).norm()<1e-9);
  // Filtering must preserve control geometry under rigid frame transforms.
  const Eigen::Matrix3d R=Eigen::AngleAxisd(.7,Eigen::Vector3d(1,2,3).normalized()).toRotationMatrix();
  const Eigen::Vector3d offset(.5,.2,-.1);auto a=before,b=before;
  b.p=R*a.p+offset;b.v=R*a.v;b.a=R*a.a;
  for(int k=0;k<200;++k){
    const Eigen::Vector3d u(.1*std::sin(k*.1),.2*std::cos(k*.07),.3);
    stepThirdOrderPosition(a,u,dt,5.);stepThirdOrderPosition(b,R*u+offset,dt,5.);
    assert((b.p-(R*a.p+offset)).norm()<1e-12);
    assert((b.v-R*a.v).norm()<1e-11 && (b.a-R*a.a).norm()<1e-9);
  }
  // Measure actual cutoff with 120Hz observations and 500Hz output.
  r.valid=false;double next=0.,input=0.,energy=0.;int count=0;
  for(int k=0;k<10000;++k){
    const double t=k*dt;
    if(t+1e-12>=next){input=.01*std::sin(2*pi*5.*t);next+=1./120.;}
    stepThirdOrderPosition(r,Eigen::Vector3d(input,0,0),dt,5.);
    if(k>=5000){energy+=r.p.x()*r.p.x();++count;}
  }
  const double gain=std::sqrt(2*energy/count)/.01;
  assert(std::abs(gain-std::sqrt(.5))<.015);
  r.reset(Eigen::Vector3d(.2,.3,.4));assert(r.v.isZero() && r.a.isZero());
  std::cout<<"PASS third-order position: no overshoot, stop, reset, continuous acceleration, ZOH subdivision, frame equivariance, 120Hz/500Hz cutoff gain="<<gain
           <<" step_peak_acceleration="<<peak_a<<" acceleration_change_ratio="<<peak_delta/old_peak_delta<<'\n';
}
