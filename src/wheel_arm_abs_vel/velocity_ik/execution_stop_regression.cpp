#include "stop_reserve.h"
#include <cassert>
#include <iostream>
int main() {
  constexpr double h=.002;
  const double q=-6.82309758509743e-06, v=.0045243009446560905, b=1.5;
  assert(q+velocity_ik::stopDistance(v,0,0,b)<1e-9); // recorded QP numerical residual
  const double ideal_q=-velocity_ik::stopDistance(v,0,0,b);
  assert(ideal_q+velocity_ik::discreteStopDistance(v,0,0,b,h)>7.49e-7);
  assert(q+velocity_ik::discreteStopDistance(v,0,0,b,h)>7.49e-7);
  const auto crossing=velocity_ik::trapezoidSegmentBounds(-1e-7,.001,-.001,h);
  assert(crossing.second>0 && crossing.first<0);
  unsigned cases=0;
  for(double brake : {.4,1.,1.5,3.,8.,25.})
    for(int i=0;i<3001;++i) {
      const double start=i*.001003;
      double speed=start,distance=0;
      while(speed>0) {
        const double next=std::max(0.,speed-brake*h);
        distance+=.5*h*(speed+next); speed=next;
      }
      assert(std::abs(distance-velocity_ik::discreteStopDistance(start,0,0,brake,h))<1e-10);
      ++cases;
    }
  for(int i=-30;i<=30;++i) for(int j=-30;j<=30;++j) {
    const double a=i*.01, z=j*.01;
    const auto bounds=velocity_ik::trapezoidSegmentBounds(.2,a,z,h);
    for(int k=0;k<=1000;++k) {
      const double t=h*k/1000.;
      const double position=.2+a*t+.5*(z-a)/h*t*t;
      assert(position>=bounds.first-1e-14 && position<=bounds.second+1e-14);
    }
    ++cases;
  }
  std::cout<<"PASS exact discrete stopping and reversal extremum; cases="<<cases<<"\n";
}
