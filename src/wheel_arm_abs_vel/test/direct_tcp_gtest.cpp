#include "wheel_arm_abs_vel/direct_tcp/mapper.h"
#include <gtest/gtest.h>
#include <fstream>
#include <cstring>

using namespace wheel_arm_abs_vel::direct_tcp;
namespace {
Sample stationary(const Vec& p=Vec::Zero(),const Rot& r=Rot::Identity()) {
  Pose t=Pose::Identity();t.translation()=p;t.linear()=r;return {t,Vec::Zero(),Vec::Zero()};
}
struct Fixture : testing::Test {
  Config config{std::getenv("DIRECT_TCP_CONFIG")};RobotModel model{config};
  ArmConfig c=config.arms[0];WristGeometry g=model.geometry()[0];
  std::vector<std::string> names;std::vector<double> seed;
  RobotArm robot;Sample ca;
  void SetUp()override {
    // Existing contracts exercise the disabled default. Feature-specific
    // tests below explicitly enable the candidate, independent of trial config.
    c.feasible_reference_fast_cutoff=0.;
    for(const auto& x:config.document.at("seed_joints").items()){names.push_back(x.key());seed.push_back(x.value());}
    robot=model.evaluate(names,seed,std::vector<double>(seed.size(),0.))[0];ca=controller(model.joint7()[0]);
  }
  Sample controller(const Sample& wrist)const {
    const Pose p=wrist.pose*c.controller_to_virtual.inverse();
    return {p,wrist.linear-wrist.angular.cross(p.linear()*c.controller_to_virtual.translation()),wrist.angular};
  }
  Sample input(int joint,double delta,double rate=0.) {
    auto q=seed;std::vector<double> dq(q.size(),0.);const auto name="zarm_l"+std::to_string(joint)+"_joint";
    const auto k=std::find(names.begin(),names.end(),name)-names.begin();q[k]+=delta;dq[k]=rate;
    model.evaluate(names,q,dq);return controller(model.joint7()[0]);
  }
  EngagedMapper mapper(){return EngagedMapper(c,g,ca,robot);}
  void derivative(const Sample& s,double elapsed=1.) {
    const double e=1.e-6;auto advance=[&](double t){auto x=s;x.pose.translation()+=t*s.linear;
      const double w=s.angular.norm();if(w>0.)x.pose.linear()=Eigen::AngleAxisd(t*w,s.angular/w).toRotationMatrix()*s.pose.linear();return x;};
    auto m=mapper(),mp=mapper(),mn=mapper();const auto a=m.map(s,robot,elapsed),p=mp.map(advance(e),robot,elapsed+e),n=mn.map(advance(-e),robot,elapsed-e);
    EXPECT_LT(((p.command.pose.translation()-n.command.pose.translation())/(2*e)-a.command.linear).norm(),2.e-7);
    Eigen::AngleAxisd aa(p.command.pose.linear()*n.command.pose.linear().transpose());
    EXPECT_LT((aa.axis()*aa.angle()/(2*e)-a.command.angular).norm(),2.e-7);
    EXPECT_LT(((p.elbow.pose.translation()-n.elbow.pose.translation())/(2*e)-a.elbow.linear).norm(),2.e-7);
  }
};
TEST(Inputs,QuaternionSign) {
  const auto a=sample({{0.1,0.2,0.3,0,0,0,1}},Vec::Zero(),Vec::Zero()),b=sample({{0.1,0.2,0.3,0,0,0,-1}},Vec::Zero(),Vec::Zero());
  EXPECT_TRUE(a.pose.matrix()==b.pose.matrix());
}
TEST(Inputs,OnlyThreeControlStatesNoHoldOrPrepare) {
  EXPECT_EQ(controlFromWire(0),Control::Inactive);EXPECT_EQ(controlFromWire(1),Control::Armed);EXPECT_EQ(controlFromWire(2),Control::Active);
  EXPECT_DEATH(controlFromWire(3),"only INACTIVE/ARMED/ACTIVE");
  EXPECT_DEATH(controlFromWire(4),"only INACTIVE/ARMED/ACTIVE");
}
TEST(Inputs,RejectInvalidWithoutRepair) {
  EXPECT_THROW(sample({0,0,0,0,0,0,0},Vec::Zero(),Vec::Zero()),std::invalid_argument);
  EXPECT_THROW(sample({NAN,0,0,0,0,0,1},Vec::Zero(),Vec::Zero()),std::invalid_argument);
  EXPECT_THROW(sample({0,0,0,0,0,0,2},Vec::Zero(),Vec::Zero()),std::invalid_argument);
}
TEST(Inputs,TransformContract) {
  Json j={{1,0,0,0},{0,1,0,0},{0,0,1,.17},{0,0,0,1}};EXPECT_DOUBLE_EQ(transform(j).translation().z(),.17);
  j[0][0]=2;EXPECT_THROW(transform(j),std::invalid_argument);
}
TEST_F(Fixture,IndependentUrdfSingleJointRoundTrip) {
  for(int joint:{4,5,6,7}) {
    auto m=mapper();const auto s=input(joint,.2,.05);const auto expected=model.evaluate(names,seed,std::vector<double>(seed.size(),0.));
    (void)expected; // regenerate the independent, perturbed FK below
    input(joint,.2,.05);const auto fk=model.joint7()[0];const Pose tcp=fk.pose*g.j7_to_tcp;
    const auto out=m.map(s,robot,1.);
    EXPECT_LT((out.command.pose.matrix()-tcp.matrix()).norm(),1.e-10)<<joint;
    EXPECT_LT((out.command.linear-fk.linear-fk.angular.cross(fk.pose.linear()*g.j7_to_tcp.translation())).norm(),1.e-10)<<joint;
    EXPECT_LT((out.command.angular-fk.angular).norm(),1.e-10)<<joint;
    EXPECT_LT((out.elbow.pose.translation()-robot.elbow.pose.translation()).norm(),1.e-10)<<joint;
  }
}
TEST_F(Fixture,RigidOffsetPivotAndAnalyticDerivative) {
  auto m=mapper();const Vec r=m.virtualPoint(stationary()).pose.translation();
  const Vec pivot(.3,.1,.2),omega(.4,-.2,.3);const Rot R=rotation(0,.5);
  auto s=stationary(pivot-R*r,R);s.angular=omega;s.linear=-omega.cross(R*r);
  const auto v=m.virtualPoint(s);EXPECT_LT((v.pose.translation()-pivot).norm(),1.e-14);EXPECT_LT(v.linear.norm(),1.e-14);
}
TEST_F(Fixture,ConfiguredPointIsWristNotTcp) {
  auto m=mapper();const auto s=input(7,.3);const auto out=m.map(s,robot,1.);const auto j7=model.joint7()[0];
  const auto rawj7=out.raw.pose*g.j7_to_tcp.inverse();EXPECT_LT((rawj7.translation()-j7.pose.translation()).norm(),1.e-12);
  EXPECT_GT((out.raw.pose.translation()-robot.tcp.pose.translation()).norm(),.04);
  EXPECT_LT((rawj7.translation()-(robot.tcp.pose*g.j7_to_tcp.inverse()).translation()).norm(),1.e-12);
}
TEST_F(Fixture,AxisAlignment) {
  c.alignment=rotation(2,.4);auto m=mapper();auto s=ca;s.pose.translation().x()+=.01;
  const auto out=m.map(s,robot,1.);EXPECT_LT((out.raw.pose.translation()-robot.tcp.pose.translation()-c.alignment*Vec(.01,0,0)).norm(),1.e-14);
}
TEST_F(Fixture,BodyAndCurrentArmPerturbationsBitwiseIdentical) {
  auto m=mapper();auto baseline=m;const auto a=baseline.map(ca,robot,1.);auto unrelated=robot;
  for(int k=0;k<100;++k) {
    unrelated.forearm=rotation(0,k*.01);unrelated.forearm_angular=Vec(k,2*k,3*k);unrelated.elbow.pose.translation()=Vec(k,-k,4*k);
    auto independent=m;const auto b=independent.map(ca,unrelated,1.);
    EXPECT_EQ(std::memcmp(a.command.pose.matrix().data(),b.command.pose.matrix().data(),16*sizeof(double)),0);
    EXPECT_EQ(std::memcmp(a.command.linear.data(),b.command.linear.data(),3*sizeof(double)),0);
    EXPECT_EQ(std::memcmp(a.elbow.pose.matrix().data(),b.elbow.pose.matrix().data(),16*sizeof(double)),0);
  }
}
TEST(Projection,ScalarC1BothEdgesAndReturn) {
  for(double x:{-1.1,-.9,.9,1.1}) {const double e=1.e-7;const auto y=softProject(x,-1,1,.1);
    EXPECT_NEAR((softProject(x+e,-1,1,.1).value-softProject(x-e,-1,1,.1).value)/(2*e),y.derivative,1.e-6);}
  EXPECT_DOUBLE_EQ(softProject(0,-1,1,.1).value,0.);EXPECT_DOUBLE_EQ(softProject(2,-1,1,.1).value,1.);
}
TEST_F(Fixture,WorkspaceAndToolDerivative) {
  auto s=ca;s.pose.translation()+=Vec(.3,.1,-.2);s.linear=Vec(.1,-.05,.02);s.angular=Vec(0,.1,0);derivative(s);
}
TEST_F(Fixture,ShoulderLimitProjectionRegeneratesCompleteChain) {
  auto m=mapper();const auto s=input(1,.9,.1);const auto out=m.map(s,robot,1.);
  const Vec upper=out.elbow.pose.translation()-robot.shoulder.pose.translation();
  const Vec lower=out.elbow.pose.linear()*g.forearm_vector;
  const double q4=std::atan2(g.forearm_vector.x(),-g.forearm_vector.z())-
      std::atan2(g.upper_vector.x(),-g.upper_vector.z())-std::atan2(upper.cross(lower).norm(),upper.dot(lower));
  const Rot r3=robot.shoulder_parent.transpose()*out.elbow.pose.linear()*rotation(1,-q4);
  const Vec shoulder(std::atan2(r3(0,2),r3(2,2)),std::atan2(-r3(1,2),std::hypot(r3(1,0),r3(1,1))),std::atan2(r3(1,0),r3(1,1)));
  std::array<double,7> angles{shoulder[0],shoulder[1],shoulder[2],q4,out.applied_wrist[0],out.applied_wrist[1],out.applied_wrist[2]};
  auto q=seed;
  for(int j=0;j<7;++j){const auto name="zarm_l"+std::to_string(j+1)+"_joint";q[std::find(names.begin(),names.end(),name)-names.begin()]=angles[j];}
  for(int j=0;j<4;++j){EXPECT_GE(angles[j],g.upstream_lower[j]-1.e-10);EXPECT_LE(angles[j],g.upstream_upper[j]+1.e-10);}
  const auto independent=model.evaluate(names,q,std::vector<double>(q.size(),0.))[0];
  EXPECT_LT((independent.tcp.pose.matrix()-out.command.pose.matrix()).norm(),1.e-10);
  EXPECT_LT((independent.elbow.pose.translation()-out.elbow.pose.translation()).norm(),1.e-10);
  EXPECT_GT((out.command.pose.translation()-out.raw.pose.translation()).norm(),.01);
  derivative(s);
}
TEST_F(Fixture,ShoulderBoundaryContinuousDerivativeAndReturn) {
  for(double delta:{.20,.23,.26,.30,.6}){auto s=input(1,delta,.08);derivative(s);}
  auto m=mapper();m.advance(ca,robot,0.);const auto far=input(1,.9);
  for(int k=1;k<=240;++k)m.advance(far,robot,k/120.);
  Output out;for(int k=241;k<=600;++k)out=m.advance(ca,robot,k/120.);
  EXPECT_LT((out.command.pose.matrix()-robot.tcp.pose.matrix()).norm(),1.e-9);
  EXPECT_LT(out.command.linear.norm(),1.e-8);
}
TEST_F(Fixture,ShoulderEngageNearBoundaryDoesNotClipAnchor) {
  const auto k=std::find(names.begin(),names.end(),"zarm_l1_joint")-names.begin();auto q=seed;
  q[k]=g.upstream_upper[0]-.005;robot=model.evaluate(names,q,std::vector<double>(q.size(),0.))[0];ca=controller(model.joint7()[0]);
  auto m=mapper();const auto out=m.advance(ca,robot,0.);
  EXPECT_LT((out.command.pose.matrix()-robot.tcp.pose.matrix()).norm(),1.e-10);
  EXPECT_LT((out.elbow.pose.translation()-robot.elbow.pose.translation()).norm(),1.e-10);
  EXPECT_LT(out.command.linear.norm(),1.e-9);
  for(double speed:{-.02,.02}){std::vector<double> dq(q.size(),0.);dq[k]=speed;
    robot=model.evaluate(names,q,dq)[0];ca=controller(model.joint7()[0]);auto moving=mapper();const auto first=moving.advance(ca,robot,0.);
    EXPECT_LT((first.command.pose.matrix()-robot.tcp.pose.matrix()).norm(),1.e-10);
    EXPECT_LT((first.command.linear-robot.tcp.linear).norm(),1.e-9);
    EXPECT_LT((first.command.angular-robot.tcp.angular).norm(),1.e-9);
  }
  q[k]=g.upstream_upper[0]+.01;robot=model.evaluate(names,q,std::vector<double>(q.size(),0.))[0];ca=controller(model.joint7()[0]);
  EXPECT_THROW(mapper(),std::domain_error);
}
TEST_F(Fixture,AllUpstreamFinalRatesRespectOriginalStoppingDomain) {
  c.reference_fast_cutoff=6.;
  c.feasible_reference_fast_cutoff=12.;
  auto m=mapper();m.advance(ca,robot,0.);
  for(int k=1;k<=1200;++k){
    const auto in=input(1,.9*std::sin(k*.025));const auto out=m.advance(in,robot,k/120.);
    for(int j=0;j<4;++j){
      const double up=std::max(0.,out.upstream_rate[j]),down=std::max(0.,-out.upstream_rate[j]);
      EXPECT_GE(out.upstream[j],g.upstream_lower[j]+g.upstream_stopping_linear[j]*down+g.upstream_stopping_quadratic[j]*down*down-1.e-9);
      EXPECT_LE(out.upstream[j],g.upstream_upper[j]-g.upstream_stopping_linear[j]*up-g.upstream_stopping_quadratic[j]*up*up+1.e-9);
      EXPECT_LE(std::abs(out.upstream_rate[j]),g.upstream_vmax[j]+1.e-9);
    }
    if(k%100==0){auto q=seed;std::vector<double> dq(q.size(),0.);
      for(int j=0;j<7;++j){const auto name="zarm_l"+std::to_string(j+1)+"_joint";const auto i=std::find(names.begin(),names.end(),name)-names.begin();q[i]=j<4?out.upstream[j]:out.reference_wrist[j-4];dq[i]=j<4?out.upstream_rate[j]:out.reference_wrist_rate[j-4];}
      const auto independent=model.evaluate(names,q,dq)[0];
      EXPECT_LT((independent.tcp.pose.matrix()-out.command.pose.matrix()).norm(),1.e-10);
      EXPECT_LT((independent.tcp.linear-out.command.linear).norm(),1.e-9);
      EXPECT_LT((independent.tcp.angular-out.command.angular).norm(),1.e-9);
      EXPECT_LT((independent.elbow.linear-out.elbow.linear).norm(),1.e-9);
    }
    if(k==300){auto a=m,b=m;const double e=1.e-6;const auto plus=a.advance(in,robot,k/120.+e),minus=b.advance(in,robot,k/120.);
      EXPECT_LT(((plus.command.pose.translation()-minus.command.pose.translation())/e-minus.command.linear).norm(),2.e-4);}
  }
}
TEST_F(Fixture,FastBandwidthDoesNotStepAndReturnsToBaseline) {
  c.reference_fast_cutoff=6.;auto m=mapper();m.advance(ca,robot,0.);
  const auto far=input(1,.7);const auto before=m.advance(far,robot,.01);
  const auto same=m.advance(ca,robot,.01);
  EXPECT_LT((before.command.pose.matrix()-same.command.pose.matrix()).norm(),1.e-12);
  EXPECT_LT((before.command.linear-same.command.linear).norm(),1.e-12);
  double fastest=0.;Output out;
  for(int k=2;k<=180;++k){out=m.advance(far,robot,k/120.);fastest=std::max(fastest,out.position_cutoff_hz);}
  EXPECT_GT(fastest,4.);EXPECT_LE(fastest,6.);
  for(int k=181;k<=600;++k)out=m.advance(ca,robot,k/120.);
  EXPECT_DOUBLE_EQ(out.position_cutoff_hz,c.reference_cutoff);
  EXPECT_LT((out.command.pose.matrix()-robot.tcp.pose.matrix()).norm(),1.e-9);
}
TEST_F(Fixture,FeasibleBandwidthReducesReachableTrackingErrorWithoutRawChange) {
  const auto target=input(1,-.6);auto ordinary=mapper();c.feasible_reference_fast_cutoff=8.;auto faster=mapper();
  const auto witness=faster.map(target,robot,0.);
  ASSERT_LT(witness.position_projection.norm(),1.e-6);ASSERT_LT(witness.orientation_projection.norm(),1.e-6);
  ordinary.advance(target,robot,0.);faster.advance(target,robot,0.);double base_error=0.,new_error=0.,max_hz=0.;
  for(int k=1;k<=240;++k){const double t=k/120.;const auto a=ordinary.advance(target,robot,t),b=faster.advance(target,robot,t);
    EXPECT_TRUE(a.raw.pose.matrix()==b.raw.pose.matrix());base_error+=(a.command.pose.translation()-a.raw.pose.translation()).norm();new_error+=(b.command.pose.translation()-b.raw.pose.translation()).norm();max_hz=std::max(max_hz,b.position_cutoff_hz);
  }
  // A large step also exercises unchanged joint rate constraints. Check
  // an actual reduction without assuming a fixed percentage through them.
  EXPECT_GT(max_hz,4.);EXPECT_LE(max_hz,8.);EXPECT_LT(new_error,base_error);
}
TEST_F(Fixture,FeasibleBandwidthDoesNotBoostLargeStaticProjection) {
  c.feasible_reference_fast_cutoff=8.;auto m=mapper();const auto target=input(1,.9);
  ASSERT_GT(m.map(target,robot,0.).position_projection.norm(),.01);
  m.advance(target,robot,0.);
  for(int k=1;k<=120;++k){const auto o=m.advance(target,robot,k/120.);EXPECT_LE(o.position_cutoff_hz,std::max(c.reference_fast_cutoff,c.reference_cutoff)+1.e-12);}
}
TEST_F(Fixture,FeasibleBandwidthDoesNotAcceleratePureWristRotation) {
  c.feasible_reference_fast_cutoff=12.;auto m=mapper();m.advance(ca,robot,0.);
  for(int k=1;k<=240;++k){const auto target=input(6,.25*std::sin(k*.025),.25*.025*120.*std::cos(k*.025));const auto out=m.advance(target,robot,k/120.);
    EXPECT_LE(out.position_cutoff_hz,std::max(c.reference_cutoff,c.reference_fast_cutoff)+1.e-10);
  }
}
TEST_F(Fixture,FeasibleBandwidthActiveDerivativeAndSameTimeUpdate) {
  c.feasible_reference_fast_cutoff=12.;auto m=mapper();const auto target=input(1,-.4);m.advance(target,robot,0.);m.advance(target,robot,.01);
  auto a=m,b=m,d=m;const double t=.012,h=1.e-6;
  const auto center=a.advance(target,robot,t),plus=b.advance(target,robot,t+h),minus=d.advance(target,robot,t-h);
  ASSERT_GT(center.position_cutoff_hz,4.);
  EXPECT_LT(((plus.command.pose.translation()-minus.command.pose.translation())/(2*h)-center.command.linear).norm(),1.e-5);
  const Eigen::AngleAxisd aa(plus.command.pose.linear()*minus.command.pose.linear().transpose());
  EXPECT_LT((aa.angle()*aa.axis()/(2*h)-center.command.angular).norm(),1.e-4);
  const auto updated=a.advance(ca,robot,t);
  EXPECT_LT((updated.command.pose.matrix()-center.command.pose.matrix()).norm(),1.e-12);
  EXPECT_LT((updated.command.linear-center.command.linear).norm(),1.e-12);
}
TEST_F(Fixture,IntegrationConvergesUnderIndependentTimeSubdivision) {
  c.reference_fast_cutoff=0.;
  for(int joint:{1,6})for(double displacement:{.25,.9}){
    auto coarse=mapper(),fine=mapper();const auto target=input(joint,displacement);
    coarse.advance(target,robot,0.);fine.advance(target,robot,0.);
    for(int k=1;k<=40;++k){const double time=k*.01;
      const auto a=coarse.advance(target,robot,time);Output b;
      for(int n=1;n<=20;++n)b=fine.advance(target,robot,(k-1)*.01+n*.0005);
      EXPECT_LT((a.command.pose.matrix()-b.command.pose.matrix()).norm(),1.e-7);
      EXPECT_LT((a.command.linear-b.command.linear).norm(),1.e-6);
      EXPECT_LT((a.command.angular-b.command.angular).norm(),1.e-6);
    }
  }
}
TEST_F(Fixture,WristProjectionPreservesUpstreamGeometry) {
  auto m=mapper();auto s=input(6,.65,.2);const auto out=m.map(s,robot,1.);
  EXPECT_NEAR(out.applied_wrist[1],g.upper[1],1.e-12);EXPECT_GT(out.orientation_projection.norm(),.1);
  EXPECT_LT((out.elbow.pose.translation()-robot.elbow.pose.translation()).norm(),1.e-11);
  const Rot r6=out.elbow.pose.linear()*rotation(2,out.applied_wrist[0])*rotation(0,out.applied_wrist[1]);
  const Vec p6=(out.command.pose*g.j7_to_tcp.inverse()).translation()-r6*g.j6_to_j7;
  const Rot initialr6=robot.forearm;const Vec p6anchor=(robot.tcp.pose*g.j7_to_tcp.inverse()).translation()-initialr6*g.j6_to_j7;
  EXPECT_LT((p6-p6anchor).norm(),1.e-11);EXPECT_GT(out.position_projection.norm(),.02);derivative(s);
}
TEST_F(Fixture,ZeroConflictHintLengthsAndDerivative) {
  auto s=ca;s.pose.translation()+=Vec(.02,.04,-.01);s.linear=Vec(.02,.03,-.04);s.angular=Vec(.1,.2,.3);
  auto m=mapper();const auto out=m.map(s,robot,1.);const Rot r6=out.elbow.pose.linear()*rotation(2,out.applied_wrist[0])*rotation(0,out.applied_wrist[1]);
  const Vec p6=(out.command.pose*g.j7_to_tcp.inverse()).translation()-r6*g.j6_to_j7;
  EXPECT_NEAR((out.elbow.pose.translation()-robot.shoulder.pose.translation()).norm(),g.upper_length,1.e-12);
  EXPECT_NEAR((p6-out.elbow.pose.translation()).norm(),g.forearm_length,1.e-12);derivative(s);
}
TEST_F(Fixture,LargeRotationWrapContinuous) {
  auto m=mapper();Output last;
  for(int k=0;k<=400;++k){last=m.map(input(5,.01*k),robot,1.);EXPECT_LE(last.applied_wrist[0],g.upper[0]+1.e-12);}
  const auto next=m.map(input(5,4.01),robot,1.);EXPECT_LT((last.command.pose.linear()-next.command.pose.linear()).norm(),1.e-10);
}
TEST_F(Fixture,EngageAndReengageC1AtRest) {
  auto m=mapper();auto s=input(7,.2,.1);const auto out=m.advance(s,robot,0.);
  EXPECT_EQ(out.command.linear.norm(),0.);EXPECT_EQ(out.command.angular.norm(),0.);EXPECT_LT((out.command.pose.matrix()-robot.tcp.pose.matrix()).norm(),1.e-14);
  input(4,.1);const auto next=model.evaluate(names,seed,std::vector<double>(seed.size(),0.))[0];
  EngagedMapper re(c,g,s,next);const auto b=re.advance(s,next,0.);EXPECT_LT((b.command.pose.matrix()-next.tcp.pose.matrix()).norm(),1.e-14);
}
TEST_F(Fixture,MovingReferenceEngageC1) {
  auto moving=robot;moving.tcp.linear=Vec(.02,-.01,.03);moving.tcp.angular=Vec(.1,-.05,.02);
  EngagedMapper m(c,g,ca,moving);const auto out=m.advance(ca,robot,0.);
  EXPECT_LT((out.command.linear-moving.tcp.linear).norm(),1.e-10);EXPECT_LT((out.command.angular-moving.tcp.angular).norm(),1.e-10);
  EXPECT_LT((out.command.pose.matrix()-moving.tcp.pose.matrix()).norm(),1.e-10);
}
TEST_F(Fixture,EngageDerivative) {
  const double t=.07;auto s=ca;s.pose.translation().y()+=t*.1;s.pose.linear()=rotation(1,t*.2)*s.pose.linear();s.linear=Vec(0,.1,0);s.angular=Vec(0,.2,0);derivative(s,t);
}
TEST_F(Fixture,SingularTargetExplicitFailure) {
  auto m=mapper();const auto s=input(6,std::acos(-1.)/2);EXPECT_THROW(m.map(s,robot,1.),std::domain_error);
}
TEST_F(Fixture,ReferenceExactDerivativeAndNoStepAtObservation) {
  auto m=mapper();const auto start=m.advance(ca,robot,0.);const auto s=input(6,.3);
  const auto update=m.advance(s,robot,.05); // previous forcing was the anchor
  EXPECT_LT((update.command.pose.matrix()-start.command.pose.matrix()).norm(),1.e-10);
  auto a=m,p=m,n=m;const double t=.072,e=1.e-6;
  const auto center=a.advance(s,robot,t),plus=p.advance(s,robot,t+e),minus=n.advance(s,robot,t-e);
  EXPECT_LT(((plus.command.pose.translation()-minus.command.pose.translation())/(2*e)-center.command.linear).norm(),1.e-7);
  Eigen::AngleAxisd aa(plus.command.pose.linear()*minus.command.pose.linear().transpose());
  EXPECT_LT((aa.axis()*aa.angle()/(2*e)-center.command.angular).norm(),1.e-7);
  EXPECT_LT(((plus.elbow.pose.translation()-minus.elbow.pose.translation())/(2*e)-center.elbow.linear).norm(),1.e-7);
  EXPECT_GT((center.command.pose.matrix()-update.command.pose.matrix()).norm(),.001);
}
TEST_F(Fixture,ReferenceUnitDcGainAndReturnNoStall) {
  auto m=mapper();m.advance(ca,robot,0.);const auto s=input(7,.25);const auto desired=mapper().map(s,robot,1.);
  Output out;for(int k=1;k<=240;++k)out=m.advance(s,robot,k/120.);
  EXPECT_LT((out.command.pose.matrix()-desired.command.pose.matrix()).norm(),1.e-10);
  for(int k=241;k<=480;++k)out=m.advance(ca,robot,k/120.);
  EXPECT_LT((out.command.pose.matrix()-robot.tcp.pose.matrix()).norm(),1.e-10);EXPECT_LT(out.command.linear.norm(),1.e-10);
  EXPECT_THROW(m.advance(ca,robot,1.),std::invalid_argument);
}
TEST_F(Fixture,ReferenceMovingAnchorTangent) {
  auto moving=robot;moving.tcp.linear=Vec(.02,-.01,.03);moving.tcp.angular=Vec(.1,-.05,.02);
  EngagedMapper m(c,g,ca,moving);const auto out=m.advance(ca,moving,0.);
  EXPECT_LT((out.command.linear-moving.tcp.linear).norm(),1.e-10);EXPECT_LT((out.command.angular-moving.tcp.angular).norm(),1.e-10);
  EXPECT_LT((out.command.pose.matrix()-moving.tcp.pose.matrix()).norm(),1.e-10);
}
TEST_F(Fixture,DynamicPreviewBoundaryDerivativeAndObservationC1) {
  auto m=mapper();m.advance(ca,robot,0.);const auto s=input(6,.65);m.advance(s,robot,.05);
  for(int k=7;k<=18;++k)m.advance(s,robot,k/120.);
  auto a=m,p=m,n=m;const double t=.153,e=1.e-6;
  const auto center=a.advance(s,robot,t),plus=p.advance(s,robot,t+e),minus=n.advance(s,robot,t-e);
  EXPECT_LT(center.dynamic_upper[1],g.upper[1]-.01);
  EXPECT_LT(((plus.command.pose.translation()-minus.command.pose.translation())/(2*e)-center.command.linear).norm(),1.e-7);
  Eigen::AngleAxisd aa(plus.command.pose.linear()*minus.command.pose.linear().transpose());EXPECT_LT((aa.axis()*aa.angle()/(2*e)-center.command.angular).norm(),1.e-7);
  auto changed=a.advance(ca,robot,t);EXPECT_LT((changed.command.pose.matrix()-center.command.pose.matrix()).norm(),1.e-12);EXPECT_LT((changed.command.angular-center.command.angular).norm(),1.e-12);
}
TEST_F(Fixture,SingleProjectionEquilibriumHasNoRestReserve) {
  auto m=mapper();m.advance(ca,robot,0.);const auto source=input(6,g.upper[1]-.02);
  auto diagnostic=mapper().map(source,robot,1.);Output out;
  for(int k=1;k<=1200;++k)out=m.advance(source,robot,k/120.);
  EXPECT_LT((out.command.pose.matrix()-diagnostic.projected.pose.matrix()).norm(),1.e-9);
  EXPECT_LT(out.reference_wrist_rate.norm(),1.e-9);
  for(int j=0;j<3;++j){EXPECT_NEAR(out.final_domain_lower[j],g.lower[j],1.e-9);EXPECT_NEAR(out.final_domain_upper[j],g.upper[j],1.e-9);}
}
TEST_F(Fixture,FinalRateDomainThroughBoundaryAndReturn) {
  auto m=mapper();m.advance(ca,robot,0.);Output out;
  for(int k=1;k<=1200;++k){const double t=k/120.;const double angle=t<5.?.75*.5*(1.-std::cos(2*M_PI*t/5.)):0.;
    out=m.advance(input(6,angle),robot,t);
    for(int j=0;j<3;++j){EXPECT_LE(out.reference_wrist[j],out.final_domain_upper[j]+1.e-10);EXPECT_GE(out.reference_wrist[j],out.final_domain_lower[j]-1.e-10);}
    const Rot r6=out.elbow.pose.linear()*rotation(2,out.reference_wrist[0])*rotation(0,out.reference_wrist[1]);
    const Vec p6=(out.command.pose*g.j7_to_tcp.inverse()).translation()-r6*g.j6_to_j7;
    EXPECT_NEAR((p6-out.elbow.pose.translation()).norm(),g.forearm_length,1.e-11);
  }
  EXPECT_LT((out.command.pose.matrix()-robot.tcp.pose.matrix()).norm(),1.e-9);
}
TEST_F(Fixture,SettleMotionKeepsActualDerivative) {
  auto m=mapper();m.advance(ca,robot,0.);const auto target=input(7,.3);
  for(int k=1;k<=12;++k)m.advance(target,robot,k/120.);
  const double t=.108,eps=1.e-6;auto p=m,n=m;const auto out=m.advance(target,robot,t),plus=p.advance(target,robot,t+eps),minus=n.advance(target,robot,t-eps);
  EXPECT_GT(out.command.linear.norm(),.001);
  EXPECT_LT(((plus.command.pose.translation()-minus.command.pose.translation())/(2*eps)-out.command.linear).norm(),1.e-7);
  Eigen::AngleAxisd aa(plus.command.pose.linear()*minus.command.pose.linear().transpose());EXPECT_LT((aa.axis()*aa.angle()/(2*eps)-out.command.angular).norm(),1.e-7);
}
TEST_F(Fixture,RawIndependentOfReferenceAndConstraintChoices) {
  auto a=mapper();auto alternative=c;alternative.reference_cutoff=12.;alternative.wrist_reference_cutoff=9.;auto limits=g;
  for(int k=0;k<3;++k){limits.lower[k]+=.03;limits.upper[k]-=.03;}
  EngagedMapper b(alternative,limits,ca,robot);
  for(int k=1;k<=30;++k){const auto source=input(6,.01*k);const auto x=a.advance(source,robot,k/120.),y=b.advance(source,robot,k/120.);
    EXPECT_EQ(std::memcmp(x.raw.pose.matrix().data(),y.raw.pose.matrix().data(),16*sizeof(double)),0);
    EXPECT_EQ(std::memcmp(x.raw.linear.data(),y.raw.linear.data(),3*sizeof(double)),0);
  }
}
TEST(Configuration,MissingFileFails) {EXPECT_THROW(Config("/missing/direct_tcp.json"),std::invalid_argument);}
TEST(Configuration,ModelAndMissingFields) {
  const char* path=std::getenv("DIRECT_TCP_CONFIG");ASSERT_NE(path,nullptr);Config c(path);RobotModel model(c);EXPECT_GT(model.geometry()[0].upper_length,.2);
  auto j=c.document;j.erase("position_budget_m");std::ofstream("/tmp/direct_tcp_missing_field.json")<<j;EXPECT_THROW(Config("/tmp/direct_tcp_missing_field.json"),Json::out_of_range);
  j=c.document;j["model_sha256"]="wrong";std::ofstream("/tmp/direct_tcp_wrong_sha.json")<<j;EXPECT_THROW(Config("/tmp/direct_tcp_wrong_sha.json"),std::invalid_argument);
  j=c.document;j["arms"][0]["shoulder_rotation_weights"][1]=0.;std::ofstream("/tmp/direct_tcp_zero_metric.json")<<j;EXPECT_THROW(Config("/tmp/direct_tcp_zero_metric.json"),std::invalid_argument);
  j=c.document;j["arms"][0].erase("shoulder_rotation_weights");std::ofstream("/tmp/direct_tcp_missing_metric.json")<<j;EXPECT_THROW(Config("/tmp/direct_tcp_missing_metric.json"),Json::out_of_range);
  j=c.document;j.erase("mapping_contract");std::ofstream("/tmp/direct_tcp_missing_contract.json")<<j;EXPECT_THROW(Config("/tmp/direct_tcp_missing_contract.json"),Json::out_of_range);
}
}  // namespace

TEST_F(Fixture,SourceUpdatePreservesAccelerationFreeAndNearBoundary) {
  for(double angle:{.15,.65}) {
    auto m=mapper();m.advance(ca,robot,0.);const auto source=input(6,angle);
    for(int k=1;k<=18;++k)m.advance(source,robot,k/120.);
    auto changed=m;const auto old_center=m.advance(source,robot,.15);
    const auto new_center=changed.advance(ca,robot,.15);
    // Richardson one-sided derivatives avoid confusing finite probe duration
    // with a discontinuity at the observation update itself.
    const auto acceleration=[&](EngagedMapper state,const Sample& forcing,const Output& center) {
      const double h=1.e-5;auto full=state,half=state;
      const auto a=full.advance(forcing,robot,.15+h),b=half.advance(forcing,robot,.15+h/2.);
      return std::pair<Vec,Vec>{(4.*(b.command.linear-center.command.linear)-(a.command.linear-center.command.linear))/h,
        (4.*(b.command.angular-center.command.angular)-(a.command.angular-center.command.angular))/h};
    };
    const auto left=acceleration(m,source,old_center),right=acceleration(changed,ca,new_center);
    EXPECT_LT((left.first-right.first).norm(),1.e-3);
    EXPECT_LT((left.second-right.second).norm(),1.e-3);
  }
}
