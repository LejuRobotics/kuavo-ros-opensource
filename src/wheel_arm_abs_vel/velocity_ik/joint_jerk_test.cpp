#include "joint_jerk.h"
#include <cassert>
#include <limits>
#include <random>
#include <iostream>
int main() {
 using velocity_ik::jointJerkInterval;
 for(double A:{50.,100.})for(double T:{.008,.012}) {
  const double h=.002,J=2*A/T;double v=0.,a=A;int steps=0;
  for(;steps<int(std::lround(T/h));++steps) {
   const auto b=jointJerkInterval(v-A*h,v+A*h,v,a,h,J);
   assert(b.valid&&!b.safety_relaxed);
   const double next=std::clamp(v-A*h,b.lower,b.upper),na=(next-v)/h;
   assert(std::abs(na-a)<=J*h+1e-9);v=next;a=na;
  }
  assert(std::abs(a+A)<1e-8);
 }
 std::mt19937 g(7);std::uniform_real_distribution<double> r(-1.,1.);
 for(int k=0;k<20000;++k) {
  double h=.001+(.5+.5*r(g))*.003,v=3*r(g),a=100*r(g),J=5000+20000*(.5+.5*r(g));
  double lo=v-.2+1.*r(g),hi=lo+.2*(.5+.5*r(g));
  auto b=jointJerkInterval(lo,hi,v,a,h,J);assert(b.valid&&b.lower>=lo&&b.upper<=hi&&b.lower<=b.upper);
  const double center=v+h*a,radius=J*h*h;
  const bool overlap=std::max(lo,center-radius)<=std::min(hi,center+radius);
  assert(b.safety_relaxed==!overlap);
  for(double x:{b.lower,b.upper}) {
   const double j=((x-v)/h-a)/h;
   if(overlap)assert(std::abs(j)<=J+1e-7);
   else assert(x==std::clamp(center,lo,hi));
  }
 }
 const double nan=std::numeric_limits<double>::quiet_NaN(),inf=std::numeric_limits<double>::infinity();
 for(double bad:{0.,-1.,nan,inf}) {
  assert(!jointJerkInterval(-1,1,0,0,bad,1).valid);
  assert(!jointJerkInterval(-1,1,0,0,.002,bad).valid);
 }
 assert(!jointJerkInterval(1,-1,0,0,.002,1).valid);
 assert(!jointJerkInterval(-1,1,nan,0,.002,1).valid);
 assert(!jointJerkInterval(-1,1,0,inf,.002,1).valid);
 assert(!jointJerkInterval(-1,1,0,1e308,1e308,1e308).valid);
 std::cout<<"PASS 8/12ms full reversals, 20000 variable-step safety intersections, invalid inputs\n";
}
