#!/usr/bin/env python3
"""Exercise the production wheel selection block and limiter, without ROS/hardware."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
wheel = root.parent/'humanoid-wheel-control/humanoid_wheel_interface'
source = (root/'src/absolute/humanoidController_wheel_absolute.cpp').read_text()
begin = source.index('    if (fixed_velocity_ik_pipeline_ && velocity_ik_base_control_)')
end = source.index('    else if (wholeBodyIkAuthorityOwned &&', begin)
block = source[begin:end]
program = r"""
#include <cassert>
#include <string>
#define ROS_INFO_STREAM(...)
#include <chrono>
#include <atomic>
#include <limits>
#include <memory>
#include <Eigen/Core>
#include "wheel_arm_abs_vel/humanoid_controllers/VelocityIkBaseCommand.h"
#include "humanoid_wheel_interface/motion_planner/VelocityLimiter.h"
namespace geometry_msgs {
struct Vector { double x=0, y=0, z=0; };
struct Twist { Vector linear, angular; };
}
namespace wheel_arm_abs_vel { struct WholeBodyIkReference { enum { AUTHORITY_INACTIVE=0, AUTHORITY_ACTIVE=2 }; }; }
using namespace humanoidController_wheel_wbc;
using Twist=geometry_msgs::Twist;
VelocityIkBaseCommand<Twist> velocity_ik_base_command_;
struct { bool safety_braking=false; int authority_state=2; } wholeBodyIkReference;
std::string velocity_ik_base_gate_reason_;
Eigen::VectorXd fixed_pipeline_last_q=Eigen::VectorXd::Zero(18);
struct { int armDim=18; } info;
bool fixed_velocity_ik_pipeline_=true, velocity_ik_base_control_=true;
bool base_locked_=false, enable_mpc_=false, baseCmdVelStatus_=true;
bool wholeBodyIkReferenceApplied=true, wholeBodyIkReferenceFresh=true;
bool wholeBodyIkAuthorityOwned=true, fixedPipelineIdleHold=false;
enum class SoftPauseState { IDLE, PAUSED };
std::atomic<SoftPauseState> soft_pause_state_{SoftPauseState::IDLE};
auto velLimiter_=std::make_shared<ocs2::mobile_manipulator::VelocityLimiter>(3);
std::unique_ptr<const ocs2::mobile_manipulator::VelocityLimiter> base_velocity_limiter_zero_;
double now() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
Twist select() {
 Twist velCmdMsg;
""" + block + r"""
 return velCmdMsg;
}
void send(double age=0) { Twist t; t.linear.x=.4; t.linear.y=-.4; t.angular.z=.4; velocity_ik_base_command_.update(t,now()-age); }
bool zero(Twist t) { return t.linear.x==0 && t.linear.y==0 && t.angular.z==0; }
int main() {
 velLimiter_->setAccelerationLimits(Eigen::Vector3d::Constant(1.2), Eigen::Vector3d::Constant(1.2));
 velLimiter_->setAccelerationDt(.002);
 base_velocity_limiter_zero_=std::make_unique<const ocs2::mobile_manipulator::VelocityLimiter>(*velLimiter_);
 assert(zero(select()));
 send(); auto v=select(); assert(std::abs(v.linear.x-.0024)<1e-12);
 assert(v.linear.y<0 && v.angular.z>0);
 for(int i=0;i<20;++i) select();
 send(.3); assert(zero(select()));
 send(); assert(std::abs(select().linear.x-.0024)<1e-12); // reset on expiry
 for(int gate=0;gate<5;++gate) {
   send();
   if(gate==0) base_locked_=true;
   if(gate==1) enable_mpc_=true;
   if(gate==2) baseCmdVelStatus_=false;
   if(gate==3) fixed_pipeline_last_q.resize(0);
   if(gate==4) soft_pause_state_=SoftPauseState::PAUSED;
   assert(zero(select()));
   base_locked_=false;enable_mpc_=false;baseCmdVelStatus_=true;
   fixed_pipeline_last_q=Eigen::VectorXd::Zero(18);soft_pause_state_=SoftPauseState::IDLE;
   assert(zero(select()));send();assert(std::abs(select().linear.x-.0024)<1e-12);
 }
 // Body reference loss and every arm lifecycle phase are independent of wheels.
 wholeBodyIkReferenceApplied=false;wholeBodyIkReferenceFresh=false;
 wholeBodyIkReference.safety_braking=true;
 for(int state : {0,1,2,3,4}) {
   wholeBodyIkReference.authority_state=state;
   send();assert(!zero(select()));
 }
 Twist invalid; invalid.linear.x=std::numeric_limits<double>::quiet_NaN();
 velocity_ik_base_command_.update(invalid,now()); assert(zero(select()));
 invalid=Twist{}; invalid.linear.z=1;
 velocity_ik_base_command_.update(invalid,now()); assert(zero(select()));
 send(-1); assert(zero(select()));
}
"""
with tempfile.TemporaryDirectory() as d:
    path=Path(d)
    (path/'ros').mkdir()
    (path/'ros/ros.h').write_text('// VelocityLimiter only includes ROS; it does not call it.\n')
    (path/'test.cpp').write_text(program)
    subprocess.run(['g++', '-std=c++17', '-O2', '-pthread', '-I'+str(path),
                    '-I/usr/include/eigen3', '-I'+str(root/'include'), '-I'+str(wheel/'include'),
                    str(path/'test.cpp'), str(wheel/'src/motion_planner/VelocityLimiter.cpp'),
                    '-o', str(path/'test')], check=True)
    subprocess.run([str(path/'test')], check=True)
print('PASS: production base selection, all authority gates, stale/invalid input, limiter reset')
