#!/usr/bin/env python3
"""Compile the production pre-owner selection block without ROS or hardware.

Exercise measured initialization, stopped release, and a new controller instance.
This is not a hardware/controller integration test.
"""
from pathlib import Path
import subprocess
import tempfile

source = Path(__file__).resolve().parents[1] / 'src/absolute/humanoidController_wheel_absolute.cpp'
text = source.read_text()
begin = text.index('    if (fixed_velocity_ik_pipeline_ && !wholeBodyIkAuthorityOwned && !enable_mpc_ && fixed_pipeline_last_q.size() == 0)')
end = text.index('    // Only the formal owner', begin)
block = text[begin:end]
startup_begin = text.index('    const auto& info = manipulatorModelInfo_;', text.index('  bool humanoidControllerWheelWbc::preUpdate('))
startup_validation_end = text.index('    if (is_real_) {', startup_begin)
startup_hold_begin = text.index('    optimizedState_mrt_ = observation_wheel_.state;', startup_validation_end)
startup_hold_end = text.index('    if (headNum_ > 0', startup_hold_begin)
startup = text[startup_begin:startup_validation_end] + text[startup_hold_begin:startup_hold_end]
# Initialization action has its own test; this checks the subsequent hold.
# Exercise the retained reference-twist fallback. The current joystick branch
# has its own production-block regression in velocity_ik_base_command_test.py.
base_begin = text.index('    else if (wholeBodyIkAuthorityOwned && wholeBodyIkReferenceApplied && baseCmdVelStatus_)')
base_end = text.index('    if(true &&', base_begin)
base = '    geometry_msgs::Twist velCmdMsg;\n' + text[base_begin:base_end].replace('else if', 'if', 1)

assert 'static vector_t fixed_pipeline_last_q' not in text
assert 'applyFinalArmJointLimitVelocityDamper' not in text
program = r'''
#include <Eigen/Core>
#define ROS_ERROR_THROTTLE(...)
#include <cassert>
#include <stdexcept>
#include <limits>
using vector_t = Eigen::VectorXd;
struct { int armDim=18, stateDim=21, inputDim=21; } manipulatorModelInfo_;
auto& info=manipulatorModelInfo_;
struct { vector_t state=vector_t::Zero(21), input=vector_t::Zero(21); } observation_wheel_;
struct { vector_t q=vector_t::Zero(21), dq=vector_t::Zero(21); } wholeBodyIkReference;
vector_t fixed_pipeline_last_q, target_qpos, target_qvel;
vector_t optimizedState_mrt_, optimizedInput_mrt_, optimizedState_mrt_limit_, optimizedInput_mrt_limit_;
namespace geometry_msgs { struct Twist { struct XYZ {double x=0,y=0,z=0;} linear,angular; }; }
bool baseCmdVelStatus_=true;
Eigen::Vector3d cmdVelWorldToBody(const Eigen::Vector3d& v, double) {return v;}
bool fixed_velocity_ik_pipeline_=true, legacy_called=false;
bool enable_mpc_=false, wholeBodyIkAuthorityOwned=false, wholeBodyIkReferenceApplied=false;
void stopDynamics(const char*) { throw std::runtime_error("invalid hold"); }
void updateUserJointCmd(int,vector_t&,vector_t&) {
  assert(!fixed_velocity_ik_pipeline_); legacy_called=true;
}
bool initialize() {
''' + startup + r'''
  return true;
}
void select() {
  int time=0;
  target_qpos=vector_t::Zero(18); target_qvel=vector_t::Zero(18);
''' + block + r'''
}
int main() {
  observation_wheel_.state(7)=0.2589;
  observation_wheel_.state(10)=-0.52453;
  observation_wheel_.state(14)=0.26005;
  observation_wheel_.state(17)=-0.52224;
  const vector_t ready=observation_wheel_.state.tail(18);
  observation_wheel_.input.setConstant(.123);
  initialize();
  assert(optimizedState_mrt_.isApprox(observation_wheel_.state));
  assert(optimizedInput_mrt_.isZero());
  assert(observation_wheel_.input.isApprox(vector_t::Constant(21,.123)));
  select(); assert(target_qpos.isApprox(ready,1e-12)); assert(target_qvel.isZero());
  for(int i=0;i<1500;++i) {
    observation_wheel_.state.array()+=1e-5;
    select(); assert(target_qpos.isApprox(ready,1e-12)); assert(target_qvel.isZero());
  }
  wholeBodyIkAuthorityOwned=true; wholeBodyIkReferenceApplied=true;
  wholeBodyIkReference.q.tail(18)=ready.array()+.1;
  wholeBodyIkReference.dq.tail(18).setConstant(.2);
  wholeBodyIkReference.dq.head(3) << .2,.1,.05;
  select(); assert(target_qpos.isApprox(wholeBodyIkReference.q.tail(18)));
  assert(target_qvel.isApprox(wholeBodyIkReference.dq.tail(18)));
  // Owned stopping tuples must pass through, not become instantaneous holds.
  wholeBodyIkReferenceApplied=false;
  wholeBodyIkReference.dq.tail(18).setConstant(.05);
  select(); assert(target_qvel.isApprox(wholeBodyIkReference.dq.tail(18)));
  wholeBodyIkReference.q.tail(18).array()+=.01;
  wholeBodyIkReference.dq.setZero(); select();
  const vector_t stopped=target_qpos;
  wholeBodyIkAuthorityOwned=false; observation_wheel_.state.setZero();
  for(int i=0;i<1500;++i) {
    select(); assert(target_qpos.isApprox(stopped,1e-12)); assert(target_qvel.isZero());
  }
  // A newly constructed controller must latch its own measured posture.
  fixed_pipeline_last_q.resize(0);
  observation_wheel_.state.setConstant(.3);
  select(); assert(target_qpos.isApprox(observation_wheel_.state.tail(18)));
  assert(target_qvel.isZero()); assert(!legacy_called);
  observation_wheel_.state(0)=std::numeric_limits<double>::quiet_NaN();
  bool rejected=false;
  try {initialize();} catch(const std::runtime_error&) {rejected=true;}
  assert(rejected);
}
'''
with tempfile.TemporaryDirectory(prefix='fixed-hold-test-') as directory:
    root=Path(directory)
    (root/'test.cpp').write_text(program)
    subprocess.run(['g++','-std=c++17','-O2','-I/usr/include/eigen3',str(root/'test.cpp'),'-o',str(root/'test')],check=True)
    subprocess.run([str(root/'test')],check=True)
print('PASS: production measured startup, zero desired speed, no recapture, owned braking, stopped release, base ownership, invalid startup rejection; instance restart')
