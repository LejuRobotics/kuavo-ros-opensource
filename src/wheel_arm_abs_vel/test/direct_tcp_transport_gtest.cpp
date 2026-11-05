#include "wheel_arm_abs_vel/direct_tcp/transport.h"
#include <gtest/gtest.h>
#include <cstdlib>
namespace d=wheel_arm_abs_vel::direct_tcp;
d::Config hardware(){return d::Config(std::getenv("DIRECT_TCP_HARDWARE_CONFIG"));}
TEST(Transport,InverseOfExistingIkShoulderAlignment) {
 auto c=hardware();d::RobotModel m(c);std::vector<std::string> names;std::vector<double> q;
 for(const auto& x:c.document.at("seed_joints").items()){names.push_back(x.key());q.push_back(x.value());}
 for(int step=0;step<30;++step){auto angles=q;for(std::size_t k=0;k<names.size();++k)if(names[k]=="waist_pitch_joint")angles[k]=.01*step;
 auto robot=m.evaluate(names,angles,std::vector<double>(q.size(),0.));auto torso=m.torsoSample();
 for(int a=0;a<2;++a){const d::Vec sh=torso.pose.inverse()*robot[a].shoulder.pose.translation(),anchor(0,a==0?.15:-.15,.42);
  auto expected=robot[a].tcp;expected.pose.translation()+=d::Vec(.02,-.01,.015);expected.linear=d::Vec(.2,.3,-.1);expected.angular=d::Vec(.1,-.2,.4);
  auto wire=d::sourceCoordinates(expected,torso,sh,anchor);d::Pose alignment=d::Pose::Identity();alignment.translation()=sh-anchor;
  const auto reconstructed=torso.pose*alignment*wire.pose;
  EXPECT_TRUE(reconstructed.matrix().isApprox(expected.pose.matrix(),1e-12));
  EXPECT_TRUE((torso.pose.linear()*wire.linear).isApprox(expected.linear,1e-12));
  EXPECT_TRUE((torso.pose.linear()*wire.angular).isApprox(expected.angular,1e-12));
  auto elbow=d::sourceCoordinates(robot[a].elbow,torso,sh,anchor);EXPECT_TRUE((torso.pose*alignment*elbow.pose).matrix().isApprox(robot[a].elbow.pose.matrix(),1e-12));
 }
 }
}
TEST(Transport,MovingTorsoDerivative) {
 d::Sample torso{d::Pose::Identity(),d::Vec::Zero(),d::Vec::Zero()},base=torso;torso.pose.translation()=d::Vec(.1,.2,.3);torso.linear=d::Vec(.02,.03,-.01);torso.angular=d::Vec(0,.4,0);
 base.pose.translation()=d::Vec(.8,.1,.4);base.linear=d::Vec(.1,.2,-.2);base.angular=d::Vec(.3,0,.1);
 const double h=1e-7;auto nextT=torso,nextB=base;
 nextT.pose.translation()+=h*torso.linear;nextB.pose.translation()+=h*base.linear;
 nextT.pose.linear()=Eigen::AngleAxisd(h*torso.angular.norm(),torso.angular.normalized()).toRotationMatrix()*torso.pose.linear();
 nextB.pose.linear()=Eigen::AngleAxisd(h*base.angular.norm(),base.angular.normalized()).toRotationMatrix()*base.pose.linear();
 const auto s=d::sourceCoordinates(base,torso,d::Vec(.1,.2,.3),d::Vec(0,.15,.42));
 const auto n=d::sourceCoordinates(nextB,nextT,d::Vec(.1,.2,.3),d::Vec(0,.15,.42));const auto derivative=d::poseDerivative(n,s,h);
 EXPECT_LT((derivative.linear-s.linear).norm(),1e-7);EXPECT_LT((derivative.angular-s.angular).norm(),1e-7);
 EXPECT_THROW(d::poseDerivative(n,s,0),std::invalid_argument);
}
TEST(Transport,Actual400062ToolAndWristDomain) {
 auto c=hardware();d::RobotModel m(c);EXPECT_FALSE(c.simulation_only);
 for(const auto& g:m.geometry()){EXPECT_NEAR(g.j7_to_tcp.translation().z(),-.17,1e-12);EXPECT_NEAR(g.j6_to_j7.norm(),.021,1e-12);EXPECT_GT(g.upper_length,.25);EXPECT_LT(g.upper_length,.30);}
 std::cout<<m.audit().dump(2)<<std::endl;
}

TEST(Transport,FloatQuaternionRoundoffPreservesRotation) {
 const Eigen::Quaterniond expected(Eigen::AngleAxisd(.73,d::Vec(1,2,3).normalized()));
 const double scale=std::sqrt(.9999989434730134); // norm2 observed in real Quest capture
 std::array<double,7> raw{{.1,.2,.3,expected.x()*scale,expected.y()*scale,expected.z()*scale,expected.w()*scale}};
 EXPECT_THROW(d::sample(raw,d::Vec::Zero(),d::Vec::Zero()),std::invalid_argument);
 const auto got=d::questTransportSample(raw);
 EXPECT_TRUE(got.pose.linear().isApprox(expected.toRotationMatrix(),1e-12));
 EXPECT_TRUE(got.pose.translation().isApprox(d::Vec(.1,.2,.3),1e-12));
}
TEST(Transport,RejectCorruptPoseWithoutManufacturingOrientation) {
 EXPECT_THROW(d::questTransportSample({0,0,0,0,0,0,0}),std::invalid_argument);
 EXPECT_THROW(d::questTransportSample({0,0,0,0,0,0,.5}),std::invalid_argument);
 EXPECT_THROW(d::questTransportSample({NAN,0,0,0,0,0,1}),std::invalid_argument);
 EXPECT_THROW(d::questTransportSample({0,0,0,0,INFINITY,0,1}),std::invalid_argument);
}
