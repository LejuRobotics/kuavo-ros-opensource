#define VELOCITY_IK_OFFLINE_TEST
#include "../src/pinocchio_velocity_ik_node.cpp"
#include <cassert>
struct VelocityIkOfflineTest {
 static void run(PinocchioVelocityIk& n) {
  n.orientation_reference_mode_="fixed_bandwidth";
  n.continuous_reference_enabled_=true;
  PinocchioVelocityIk::TargetSnapshot input;
  input.valid=true;input.estimated_twists_valid=true;
  input.receive_monotonic_ns=1.e12;
  for(int arm=0;arm<2;++arm) {
   input.arm[arm].hand_orientation=Eigen::Quaterniond::Identity();
   input.arm[arm].hand_position.setZero();
   input.arm[arm].elbow_position.setZero();input.arm[arm].elbow_valid=true;
   n.hand_reference_[arm].valid=false;n.hand_rotation_reference_[arm].valid=false;
   n.elbow_reference_[arm].valid=false;
  }
  auto world=input;n.shapeContinuousTarget(world,input,1.e12,.002);
  for(int k=1;k<=1500;++k) {
   for(int arm=0;arm<2;++arm) {
    input.arm[arm].hand_position=Eigen::Vector3d(.1,0.,0.);
    input.arm[arm].hand_orientation=Eigen::Quaterniond(Eigen::AngleAxisd(.3,Eigen::Vector3d::UnitY()));
    // Intentionally inconsistent/noisy FF must not bypass the reference.
    input.arm[arm].hand_linear_ff=Eigen::Vector3d::Constant((k % 2) ?100.:-100.);
    input.arm[arm].hand_angular_ff=input.arm[arm].hand_linear_ff;
   }
   world=input;n.shapeContinuousTarget(world,input,1.e12+k*2.e6,.002);
   assert((world.arm[0].hand_position-world.arm[1].hand_position).norm()<1.e-12);
   assert(world.arm[0].hand_orientation.angularDistance(world.arm[1].hand_orientation)<1.e-12);
   for(int arm=0;arm<2;++arm) {
    assert(world.arm[arm].hand_position.x()>=0. && world.arm[arm].hand_position.x()<=.1+1.e-12);
    assert(!n.hand_reference_[arm].innovation.valid && !n.hand_rotation_reference_[arm].innovation.valid);
    assert(world.arm[arm].hand_linear_ff.norm()<2. && world.arm[arm].hand_angular_ff.norm()<5.);
   }
  }
  assert(world.arm[0].hand_linear_ff.norm()<1.e-9 && world.arm[0].hand_angular_ff.norm()<1.e-9);
  // Non-estimated/legacy input must also use the same reference.
  input.estimated_twists_valid=false;
  n.host_target_reconstruction_=true;
  world=input;
  n.prepareTargetForTracking(world,input,1.e12+2.e7,.02,0.,true,true);
  assert((world.arm[0].hand_position-input.arm[0].hand_position).norm()<1.e-12);
  assert(world.arm[0].hand_orientation.angularDistance(input.arm[0].hand_orientation)<1.e-12);
  n.shapeContinuousTarget(world,input,1.e12+2.e7,.002);
  assert(world.arm[0].hand_angular_ff.norm()<1.e-8);
  std::cout<<"PASS actual node fixed references: both arms, no FF/age bypass, stationary settling, legacy input\n";
 }
};
int main(int argc,char**argv) {
 ros::init(argc,argv,"joint_jerk_test",ros::init_options::NoSigintHandler);
 try{PinocchioVelocityIk n;VelocityIkOfflineTest::run(n);}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
