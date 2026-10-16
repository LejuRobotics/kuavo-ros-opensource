#include "weighted_qp.h"
#include "stop_reserve.h"
#include "elbow_feedforward.h"
#include "channel_observation.h"
#include "bounded_prediction.h"
#include "async_diagnostics.h"
#include <vector>
#include <iostream>
#include <stdexcept>
static void require(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
int main(){
 using namespace velocity_ik;using Eigen::Vector3d;const double nan=std::numeric_limits<double>::quiet_NaN();
 {
  WeightedQp bounded;bounded.prepareWorkspaces({{2,0}});
  Eigen::MatrixXd H=Eigen::MatrixXd::Identity(2,2),C(0,2);
  Eigen::VectorXd g=Eigen::VectorXd::Zero(2),lo=Eigen::VectorXd::Constant(2,-1),hi=-lo,empty(0),x;
  WeightedQp::Stats stats;int status;
  require(bounded.solve(H,g,lo,hi,C,empty,empty,&x,&status,&stats),"preallocated family solves");
  require(!stats.workspace_created,"no workspace created inside configured solve");
  C=Eigen::MatrixXd::Identity(2,2);
  require(!bounded.solve(H,g,lo,hi,C,lo,hi,&x,&status,&stats) && status==-1005,"unknown family rejected");
  bool rejected=false;try{bounded.prepareWorkspaces({{2,2}});}catch(const std::logic_error&){rejected=true;}
  require(rejected,"running workspace set immutable");
 }

 for(double placeholder:{nan,99.0}){
  Vector3d a(1,2,3),b(1.01,2,3),bad=Vector3d::Constant(placeholder),ff=Vector3d::Zero();
  ff=continuousElbowFeedforward(true,true,true,a,b,ff,.01,.5);require((ff-Vector3d(.5,0,0)).norm()<1e-12,"valid EMA");
  ff=continuousElbowFeedforward(true,false,true,b,bad,ff,.01,.5);require(ff.isZero()&&ff.allFinite(),"invalid reset");
  ff=continuousElbowFeedforward(false,true,true,bad,a,ff,.01,.5);require(ff.isZero()&&ff.allFinite(),"recovery seeds history");
  ff=continuousElbowFeedforward(true,true,true,a,b,ff,.01,.5);require(ff.allFinite()&&(ff-Vector3d(.5,0,0)).norm()<1e-12,"valid path resumes");
  require(continuousElbowFeedforward(true,true,false,a,b,ff,.01,.5).isZero(),"source discontinuity");
 }
 // Cross-channel counterexample: replay identical left-hand observations
 // with a moving versus a stationary right hand. Left estimates must agree.
 for(bool right_moves:{false,true}) {
  ChannelObservation left,right,elbow;
  for(int k=0;k<8;++k) {
   const double stamp=1.+.014*k,host=stamp*1e9;
   const Vector3d lp(k==0?0.:.01,0,0),rp(right_moves?.01*k:0.,0,0);
   left=observeChannel(left,lp,Eigen::Quaterniond::Identity(),true,true,stamp,host,.01,1e-10,1e-10);
   right=observeChannel(right,rp,Eigen::Quaterniond::Identity(),true,true,stamp,host,.01,1e-10,1e-10);
   elbow=observeChannel(elbow,Vector3d::Constant(k==2?nan:.01*k),Eigen::Quaterniond::Identity(),k!=2,true,stamp,host,.01,1e-10,1e-10);
   if(k==2 || k==3) require(elbow.linear.isZero(),"elbow invalid and recovery seed independently");
   if(k>=1) require(std::abs(left.linear.x()-.01/.024)<1e-10,"other channel cannot advance left EMA");
  }
  require(left.updates==2 && right.updates==(right_moves?8:1),"each channel counts its content observations");
  require(channelDecay(left,1.2e9,.025,.05)<.05,"duplicates do not hold velocity forever");
 }
 // The same sample arriving in new packets keeps its original epoch.
 {
  auto first=observeChannel(ChannelObservation{},Vector3d::Zero(),Eigen::Quaterniond::Identity(),true,true,1.,1e9,.005,1e-10,1e-10);
  auto repeat=observeChannel(first,Vector3d::Zero(),Eigen::Quaterniond::Identity(),true,true,1.014,1.014e9,.005,1e-10,1e-10);
  require(std::abs(channelPredictionAge(repeat,1.016e9,.001)-.017)<1e-12,"duplicate preserves prediction epoch");
  auto fresh=observeChannel(repeat,Vector3d(.001,0,0),Eigen::Quaterniond::Identity(),true,true,1.028,1.028e9,.005,1e-10,1e-10);
  require(std::abs(channelPredictionAge(fresh,1.030e9,.001)-.003)<1e-12,"new content moves only its own epoch");
  require(std::abs(channelPredictionAge(repeat,1.030e9,.001)-.031)<1e-12,"another channel cannot reset epoch");
  require(channelPredictionAge(ChannelObservation{},1e9,.001)==0.,"invalid channel has no prediction");
 }
 // Fixed-motion cap crossing remains continuous, and unrelated channels
 // cannot reset its displacement or alter its feedforward.
 for (double speed : {.2, 2., 20.}) {
  const Vector3d velocity(speed,0,0), original=velocity;
  for(double boundary : {.020, .020/speed}) {
   const auto before=boundedPredictionStep(velocity,boundary-1e-9,.020);
   const auto after=boundedPredictionStep(velocity,boundary+1e-9,.020);
   require((after-before).norm()<1e-6,"continuous time and space caps");
   require(after.norm()<=.020+1e-12 && after.x()+1e-12>=before.x(),"bounded non-retreating fixed motion");
  }
  const auto left=boundedPredictionStep(velocity,.015,.020);
  for(double right_speed : {0.,1000.}) {
   const auto right=boundedPredictionStep(Vector3d(right_speed,0,0),.015,.020);
   require(right.norm()<=.020+1e-12,"independent right cap");
   require((boundedPredictionStep(velocity,.015,.020)-left).norm()==0,"right cap cannot reset left");
  }
  require(velocity==original,"cap does not modify FF");
 }
 require(boundedPredictionStep(Vector3d(nan,0,0),.01,.02).isZero(),"invalid prediction is finite");
 require(boundedPredictionStep(Vector3d::Ones(),-.001,.02).isZero(),"negative horizon rejected");
 require(std::abs(boundedBodyPredictionStep(Eigen::Vector4d(10,-10,.1,0),.015)[2]-.0015)<1e-12,"body components independent");
 // Integrate an adversarial delayed acceleration followed by guaranteed
 // braking. Outward, zero, reverse (checked on the opposite boundary), and
 // narrow-interval feasibility are independent of the QP implementation.
 for(double v0 : {0.0, 0.01, 0.5, 2.0}) for(double delay : {0.0, .02, .045}) {
  const double aout=40., brake=25., h=1e-5; double q=0.,v=v0;
  for(int k=0;k<int(std::round(delay/h));++k){q+=v*h+.5*aout*h*h;v+=aout*h;}
  while(v>0){double dt=std::min(h,v/brake);q+=v*dt-.5*brake*dt*dt;v=std::max(0.,v-brake*dt);}
  require(std::abs(q-stopDistance(v0,delay,aout,brake))<1e-9,"delayed physical braking distance");
 }
 require(stopDistance(.5,.045,25.,25.)>.01,"narrow outward interval rejected");
 require(stopDistance(0.,.045,25.,25.)>.05,"zero velocity retains delay acceleration reserve");
 require(std::abs(stopDistance(2.,0.,0.,25.)-.08)<1e-12,"normal zero delay stop");
 Eigen::MatrixXd H=Eigen::MatrixXd::Identity(2,2),C=H;Eigen::VectorXd g=Eigen::VectorXd::Zero(2),lo=Eigen::VectorXd::Constant(2,-1),hi=-lo,x=g,z=g;
 require(checkQpResult(H,g,C,lo,hi,x,z).accepted,"finite result");
 auto b=x;b(0)=nan;require(!checkQpResult(H,g,C,lo,hi,b,z).accepted,"NaN primal");
 b=z;b(0)=nan;require(!checkQpResult(H,g,C,lo,hi,x,b).accepted,"NaN dual");
 require(!checkQpResult(H,g,C,lo,hi,Eigen::VectorXd::Zero(1),z).accepted,"primal size");
 require(!checkQpResult(H,g,C,lo,hi,x,Eigen::VectorXd::Zero(1)).accepted,"dual size");
 x.setConstant(1e308);C.setConstant(1e308);require(!checkQpResult(H,g,C,lo,hi,x,z).accepted,"Cx overflow");
 C.setIdentity();H.setConstant(1e308);require(!checkQpResult(H,g,C,lo,hi,x,z).accepted,"stationarity overflow");
 WeightedQp qp;Eigen::VectorXd result;int status;WeightedQp::Stats stats;
 H.setIdentity();x.setZero();Eigen::MatrixXd empty(0,2);Eigen::VectorXd e(0);
 require(qp.solve(H,g,lo,hi,empty,e,e,&result,&status,&stats)&&result.allFinite()&&stats.accepted,"finite QP after recovery");
 std::atomic<unsigned> accepted{0},executed{0};AsyncDiagnostics diagnostics;
 std::vector<std::thread> producers;
 for(unsigned k=0;k<3;++k)producers.emplace_back([&]{for(unsigned i=0;i<2000;++i)if(diagnostics.submit([&]{++executed;}))++accepted;});
 for(auto& producer:producers)producer.join();diagnostics.stop();
 require(executed==accepted,"all accepted diagnostic jobs drain exactly once");
 require(accepted+diagnostics.dropped()==6000,"bounded queue reports every drop");
 std::cout<<"diagnostic accepted="<<accepted<<" executed="<<executed<<" full="<<diagnostics.dropped()<<"\n";
 std::cout<<"PASS elbow NaN/finite invalid recovery; normal EMA; QP finite/size/overflow checks; actual QP\n";
}
