#!/usr/bin/env python3
"""Compile production startup interpolation and verify initialization wiring."""
from pathlib import Path
import subprocess
import tempfile
root=Path(__file__).resolve().parents[1]
source=(root/'src/absolute/humanoidController_wheel_absolute.cpp').read_text()
pre=source[source.index('  bool humanoidControllerWheelWbc::preUpdate('):source.index('  void humanoidControllerWheelWbc::update(')]
assert pre.index('checkDynamicsSensor')<pre.index('performSimpleActions(time)')<pre.index('isPreUpdateComplete = true')
assert 'preTargetActions_ = startActions_ + delta' in pre
assert 'starting configured arm preparation' in pre and 'preparation completed' in pre
hardware=(root.parent/'kuavo-ros-control-lejulib/hardware_node/src/hardware_node.cc').read_text()
assert 'StandUpLoop();' in hardware and '"/hardware/is_ready", 1' in hardware
start=source[source.index('  bool humanoidControllerWheelWbc::starting('):source.index('  bool humanoidControllerWheelWbc::preUpdate(')]
assert 'isHardwareReady != 1' in start
# Defined dev action, not a new made-up posture.
legacy=(root.parent/'humanoid-control/humanoid_controllers/src/humanoidController_wheel_wbc.cpp').read_text()
for line in ('targetAction.tail(armNum_)[4] = startAction.tail(armNum_)[4] - 0.5236;',
             'targetAction.tail(armNum_/2)[4] = startAction.tail(armNum_/2)[4] + 0.5236;'):
    assert line in legacy and line in source
code=r"""
#include <cassert>
#include "wheel_arm_abs_vel/humanoid_controllers/StartupArmAction.h"
int main() {
 using namespace wheel_absolute_control;
 Eigen::VectorXd q=Eigen::VectorXd::Constant(21,.2),delta=Eigen::VectorXd::Zero(21);
 delta(11)=-.5236;delta(18)=.5236;
 auto first=sampleStartupArmAction(q,delta,0,1.5);
 assert(first.q.isApprox(q)&&first.v.isZero()&&!first.complete);
 auto mid=sampleStartupArmAction(q,delta,.75,1.5);
 assert(mid.q.isApprox(q+delta)&&mid.v.isZero());
 for(int i=1;i<1500;++i) {
   auto s=sampleStartupArmAction(q,delta,i*.001,1.5);
   assert(s.q.head(7).isApprox(q.head(7))&&s.v.head(7).isZero());
   assert(s.v.cwiseAbs().maxCoeff()<1.31);
   auto prev=sampleStartupArmAction(q,delta,(i-1)*.001,1.5);
   assert((s.q-prev.q).cwiseAbs().maxCoeff()<.00131);
 }
 auto end=sampleStartupArmAction(q,delta,1.5,1.5);
 assert(end.q.isApprox(q)&&end.v.isZero()&&end.complete);
 assert(sampleStartupArmAction(q,delta,2.,1.5).q.isApprox(q));
 bool bad=false;try{sampleStartupArmAction(q,delta,-1,1.5);}catch(...){bad=true;}assert(bad);
}
"""
with tempfile.TemporaryDirectory() as d:
    p=Path(d);(p/'test.cpp').write_text(code)
    subprocess.run(['g++','-std=c++17','-I/usr/include/eigen3','-I'+str(root/'include'),str(p/'test.cpp'),'-o',str(p/'test')],check=True)
    subprocess.run([str(p/'test')],check=True)
print('PASS: o/hardware-ready wiring, original arm action, continuous out-and-back reference')
