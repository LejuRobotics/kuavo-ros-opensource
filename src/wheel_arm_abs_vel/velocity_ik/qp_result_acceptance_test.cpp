#include "weighted_qp.h"
#include <cassert>
#include <iostream>
#include <limits>
using namespace velocity_ik;
int main(){
 Eigen::MatrixXd H=Eigen::MatrixXd::Identity(1,1),C=H;
 Eigen::VectorXd lo(1),hi(1),x(1),z(1),g(1);lo<<-1.;hi<<1.;int count=0;
 const auto test=[&](const char* label,double xx,double zz,double residual,bool expected){
  x<<xx;z<<zz;g<<-xx-zz+residual;const auto r=checkQpResult(H,g,C,lo,hi,x,z);
  assert(r.accepted==expected);++count;
  std::cout<<label<<" accepted="<<r.accepted<<" violation="<<r.violation<<" stationarity="<<r.stationarity<<" complementarity="<<r.complementarity<<'\n';
 };
 test("valid",1.,1.,0.,true);
 test("complementarity_4e-6",1.-1e-9,4000.,0.,true);
 test("complementarity_8e-6",1.-1e-9,8000.,0.,false);
 test("hard_violation_1e-7",1.+1e-7,1.,0.,false);
 test("stationarity_3e-7",1.,1.,3e-7,false);
 test("nonfinite",std::numeric_limits<double>::quiet_NaN(),1.,0.,false);
 WeightedQp qp;Eigen::MatrixXd empty(0,1);Eigen::VectorXd ev(0),solution;g<<-.5;int code;WeightedQp::Stats stats;
 bool ok=qp.solve(H,g,lo,hi,empty,ev,ev,&solution,&code,&stats);
 assert(ok&&code==0&&stats.native_status==0&&stats.primal<=1e-6);++count;
 Eigen::VectorXd cl(1),cu(1);cl<<2.;cu<<3.;
 ok=qp.solve(H,g,lo,hi,C,cl,cu,&solution,&code,&stats);
 assert(!ok && code!=0);++count;
 std::cout<<"infeasible rejected="<<!ok<<" native="<<stats.native_status<<" primal="<<stats.primal<<" code="<<code<<'\n';
 std::cout<<"PASS "<<count<<" production-linked gate cases\n";
}
