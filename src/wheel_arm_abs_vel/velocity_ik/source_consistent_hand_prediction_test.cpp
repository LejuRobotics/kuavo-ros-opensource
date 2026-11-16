
#include "source_consistent_hand_prediction.h"
#include <iostream>
#include <cassert>
using namespace velocity_ik;
int main(){
 const Eigen::Vector3d v(1.,-.4,.2),w(0.,0.,.7); SourceConsistentHandPrediction a;
 a.reset(0.,Eigen::Vector3d::Zero(),Eigen::Quaterniond::Identity(),v,w);
 double max_p=0.,max_w=0.;
 for(double t:{.008,.017,.025,.033,.050,.058,.067,.083,.092}) {
   auto q=fixedRotationAdvance(Eigen::Quaterniond::Identity(),t*w);
   a.observe(t,t*v,q,.02);
   for(double age:{0.,.002,.007,.016}) {
    max_p=std::max(max_p,(t*v+age*a.linear-(t+age)*v).norm());
    max_w=std::max(max_w,(a.angular-w).norm());
   }
 }
 assert(max_p<1e-12 && max_w<1e-12);
 auto previous=a.linear;a.observe(a.stamp,10.*v,Eigen::Quaterniond::Identity(),.02);
 assert((a.linear-previous).norm()==0.);
 a.reset(1.,Eigen::Vector3d::Zero(),Eigen::Quaterniond::Identity(),Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero());
 a.observe(1.01,Eigen::Vector3d::Zero(),Eigen::Quaterniond::Identity(),.02);
 assert(a.linear.norm()==0. && a.angular.norm()==0.);
 ContinuousVector r;for(int i=0;i<1000;++i)stepFixedVector(r,Eigen::Vector3d::Zero(),.002,12.);
 assert(r.p.norm()==0. && r.v.norm()==0.);
 std::cout<<"constant_velocity_prediction_max_m="<<max_p<<" constant_omega_error="<<max_w<<" duplicate_and_reset_passed=1"<<std::endl;
}
