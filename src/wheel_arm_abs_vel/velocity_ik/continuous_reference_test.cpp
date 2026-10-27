#include "continuous_reference.h"
#include <cassert>
#include <iostream>
int main(){
  using namespace velocity_ik;
  ContinuousVector r;r.reset(Eigen::Vector3d::Zero());
  const double dt=.002;
  for(int i=1;i<=1000;++i)r.step(Eigen::Vector3d(i*dt,0,0),Eigen::Vector3d(1,0,0),dt,6.);
  assert(std::abs(r.p.x()-2.)<1e-9 && std::abs(r.v.x()-1.)<1e-9);
  // Production guide weight does not cap steady tracking speed to 10%.
  r.reset(Eigen::Vector3d::Zero());
  for(int i=1;i<=1000;++i)r.step(Eigen::Vector3d(i*dt,0,0),Eigen::Vector3d(.1,0,0),dt,6.);
  // Sampled position target with a 0.1 guide has a small inter-tick ripple.
  assert(std::abs(r.v.x()-1.)<1e-3);
  assert(2.-r.p.x()>0. && 2.-r.p.x()<.055);
  r.reset(Eigen::Vector3d::Zero());double old=0.;
  for(int i=0;i<2000;++i){r.step(Eigen::Vector3d::Ones(),Eigen::Vector3d::Zero(),dt,1.2);
    assert(r.p.x()>=old-1e-12 && r.p.x()<=1.+1e-12);old=r.p.x();}
  ContinuousRotation q;q.reset(Eigen::Quaterniond::Identity());
  for(int i=1;i<=2000;++i){q.step(Eigen::Quaterniond(Eigen::AngleAxisd(.1*i*dt,Eigen::Vector3d::UnitZ())),Eigen::Vector3d(0,0,.1),dt,6.);assert(q.q.coeffs().allFinite());}
  assert(std::abs(q.q.norm()-1.)<1e-12);
  std::cout<<"continuous reference contracts passed\n";
}
