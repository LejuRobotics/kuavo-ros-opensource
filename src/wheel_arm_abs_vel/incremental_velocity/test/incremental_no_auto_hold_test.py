#!/usr/bin/env python3
"""Production dual input and no-auto-HOLD policy, without ROS or hardware."""
import ast
import copy
import math
from pathlib import Path
import subprocess
import tempfile
from types import SimpleNamespace as NS
import unittest
import xml.etree.ElementTree as ET

import numpy as np

ROOT=Path(__file__).resolve().parents[1]


def joystick(**kwargs):
    fields={side+'_'+name:0. for side in ('left','right')
            for name in ('x','y','trigger','grip')}
    fields.update({side+'_'+name:False for side in ('left','right')
                   for name in ('first_button_pressed','second_button_pressed',
                                'first_button_touched','second_button_touched')})
    fields.update(kwargs)
    return NS(**fields)


def identity_quaternion(rotation):
    # Translation-only fixtures; rotation conversion is unchanged by this patch.
    np.testing.assert_allclose(rotation,np.eye(3),atol=1.e-12)
    return np.array([0.,0.,0.,1.])


scope=dict(copy=copy,math=math,np=np,JoySticks=joystick,DualQuestSample=NS,
           rospy=NS(Time=NS(now=lambda:0.)),matrix_to_quaternion=identity_quaternion)
for relative,name in [('quest_input_buffer.py','SourceClockGate'),
                      ('tools/lightweight_transforms.py','quaternion_to_matrix'),
                      ('incremental_dual_quest.py','DualQuest')]:
    path=ROOT/'scripts'/relative
    tree=ast.parse(path.read_text())
    node=next(n for n in tree.body if getattr(n,'name',None)==name)
    exec(compile(ast.Module(body=[node],type_ignores=[]),str(path),'exec'),scope)
DualQuest=scope['DualQuest']


def bones(stamp, chest=0., left=.3, right=-.3):
    def pose(x):
        return NS(position=NS(x=x,y=0.,z=0.),orientation=NS(x=0.,y=0.,z=0.,w=1.))
    poses=[pose(chest) for _ in range(24)]
    poses[4]=pose(chest+left);poses[5]=pose(chest+right)
    return NS(timestamp_ms=stamp,poses=poses,is_hand_tracking=False,is_high_confidence=True)


class DualInputTest(unittest.TestCase):
    def test_each_side_updates_without_pairing_or_age_hold(self):
        for owner in ('left','right'):
            d=DualQuest(owner)
            self.assertIsNone(d.skeleton(0,bones(10000,chest=2.),10.))
            self.assertTrue(all(d.skeleton(1,bones(50000,chest=5.),10.005).side_available))
            # Keep the right snapshot. Neither a short gap nor a prolonged
            # skeleton-only gap invents a right clutch operation.
            for elapsed in (.015,.060,.300,.800):
                p=d.skeleton(0,bones(10000+int(1000*elapsed),chest=2.,left=elapsed),10.+elapsed)
                self.assertIsNotNone(p)
                self.assertEqual(p.side_available,[True,True])
                chest=p.skeleton.poses[23].position.x
                self.assertAlmostEqual(p.skeleton.poses[4].position.x-chest,elapsed)
                self.assertAlmostEqual(p.skeleton.poses[5].position.x-chest,-.3)
                self.assertEqual(p.owner_chest_fresh,owner=='left' or elapsed<.5)

    def test_beta_joystick_timeout_neutralizes_only_its_own_side(self):
        d=DualQuest('left')
        d.joystick(0,joystick(left_grip=.9,right_grip=.1),10.)
        p=d.joystick(1,joystick(right_grip=.8,left_grip=.2),10.005)
        self.assertEqual((p.left_grip,p.right_grip),(.9,.8))
        p=d.joystick(1,joystick(right_grip=.8),10.510)
        self.assertEqual((p.left_grip,p.right_grip),(0.,.8))

    def test_device_clocks_remain_independent_and_output_monotonic(self):
        d=DualQuest('left')
        d.skeleton(0,bones(100000),20.)
        first=d.skeleton(1,bones(1000),20.001)
        second=d.skeleton(0,bones(100010),20.010)
        self.assertGreater(second.skeleton.timestamp_ms,first.skeleton.timestamp_ms)
        self.assertEqual(second.source_time_ms,[100010,1000])
        self.assertIsNone(d.skeleton(0,bones(100010),20.020))
        self.assertIsNone(d.skeleton(0,bones(100005),20.030))
        third=d.skeleton(1,bones(1040),20.041)
        self.assertGreater(third.skeleton.timestamp_ms,second.skeleton.timestamp_ms)
        self.assertEqual(third.side_available,[True,True])


class PolicyTest(unittest.TestCase):
    def test_automatic_hold_mechanism_is_removed_from_shared_runtime(self):
        removed=('automatic_hold_enabled', 'hold_on_input_timeout', 'hold_on_timeout',
                 'confirm_reacquire', 'confirmation_required', 'confirm_ready_',
                 'force_confirmation_', 'automaticHoldEnabled',
                 'baseTrackingRequiresHold', 'executionTrackingRequiresHold',
                 'reference_tracking_fault_latched_', 'reference_tracking_fault_elapsed_',
                 'reference_tracking_recovery_elapsed_')
        for relative in ('scripts/ik_ros_velocity.py', 'src/pinocchio_velocity_ik_node.cpp',
                         'velocity_ik/teleop_continuity.h'):
            source=(ROOT/relative).read_text()
            for name in removed:
                self.assertNotIn(name,source,(relative,name))

    def test_retained_target_cannot_override_explicit_prepare_or_execution_validity(self):
        solver=(ROOT/'src/pinocchio_velocity_ik_node.cpp').read_text()
        def statement(prefix):
            begin=solver.index(prefix)
            return solver[begin:solver.index(';',begin)+1]
        # Compile the actual solver predicates and PREPARE completion branch,
        # with only the unavailable ROS/Pinocchio inputs replaced by a fixture.
        fresh=statement('const bool source_target_fresh =')
        admitted=statement('const bool source_target_admissible =')
        dispatch_begin=solver.index('if (!contract_active) input_continuity_.suspend();')
        dispatch_end=solver.index('\n\n',dispatch_begin)
        dispatch=solver[dispatch_begin:dispatch_end]
        tracking=statement('const bool tracking = !execution_waiting_')
        complete=statement('state = preparation_complete_ ? ShadowState::kHold')
        program='''
#include "teleop_continuity.h"
#include <cassert>
#include <limits>
enum class ShadowState { kHold, kPreparing };
struct Fixture {
  struct { bool valid=true; } target;
  bool target_source_active=true, takeover_started_=true, reference_initialized_=true;
  bool execution_waiting_=false, robot_fresh=true, mode_ok=true;
  bool startup_bootstrap_active_=false, dynamic_fault_latched_=false;
  bool last_admitted=false;
  double target_timeout_=.1;
  velocity_ik::InputContinuity input_continuity_;
  bool tracks(double target_age, bool contract_active) {
''' + fresh + admitted + dispatch + '''
    last_admitted=source_target_admissible;
    const bool target_fresh=source_target_admissible; // All geometry valid in this fixture.
''' + tracking + '''
    return tracking;
  }
  ShadowState preparationState(bool preparation_complete_) {
    ShadowState state;
''' + complete + '''
    return state;
  }
};
int main() {
  Fixture f;
  f.takeover_started_=false;
  assert(!f.tracks(1.,true) && !f.last_admitted);
  assert(!f.tracks(.01,true) && f.last_admitted); // Still needs execution acquisition.
  f.takeover_started_=true;
  for(double age : {.01,.16,.55,2.,60.}) {
    assert(f.tracks(age,true) && f.input_continuity_.budget==1.);
  }
  assert(f.preparationState(false)==ShadowState::kPreparing);
  // 30 simulated seconds of fresh targets while X+A remains PREPARE:
  // completion parks the solver and cannot authorize normal following.
  for(int tick=0;tick<15000;++tick) {
    assert(!f.tracks(.001,false) && f.input_continuity_.budget==0.);
    assert(f.preparationState(true)==ShadowState::kHold);
  }
  assert(f.tracks(.001,true)); // Only an explicit ACTIVE contract resumes.
  for(bool* gate : {&f.robot_fresh, &f.mode_ok, &f.reference_initialized_,
                    &f.target_source_active, &f.target.valid}) {
    *gate=false;assert(!f.tracks(.001,true));*gate=true;
    assert(f.tracks(.001,true));
  }
  for(bool* blocker : {&f.execution_waiting_, &f.dynamic_fault_latched_,
                       &f.startup_bootstrap_active_}) {
    *blocker=true;assert(!f.tracks(.001,true));*blocker=false;
    assert(f.tracks(.001,true));
  }
  for(double age : {-1.,std::numeric_limits<double>::infinity(),
                    std::numeric_limits<double>::quiet_NaN()}) {
    assert(!f.tracks(age,true));
  }
}
'''
        with tempfile.TemporaryDirectory(prefix='incremental-policy-') as directory:
            folder=Path(directory);cpp=folder/'test.cpp';binary=folder/'test'
            cpp.write_text(program)
            subprocess.run(['g++','-std=c++17','-I'+str(ROOT/'velocity_ik'),str(cpp),'-o',str(binary)],check=True)
            subprocess.run([str(binary)],check=True)


if __name__=='__main__':
    unittest.main()
