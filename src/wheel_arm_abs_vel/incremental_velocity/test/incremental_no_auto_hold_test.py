#!/usr/bin/env python3
"""Production dual input and no-auto-HOLD policy, without ROS or hardware."""
import ast
import copy
import math
from enum import IntEnum
from pathlib import Path
import subprocess
import tempfile
import threading
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


class StartupTest(unittest.TestCase):
    def setUp(self):
        path = ROOT / 'scripts/ik_ros_velocity.py'
        tree = ast.parse(path.read_text(), filename=str(path))
        states = ('AbsoluteEnableState', 'AbsoluteEnableReason')
        definitions = [node for node in tree.body if getattr(node, 'name', None) in states]
        mapper = next(node for node in tree.body if getattr(node, 'name', None) == 'IkRos')
        methods = ('_advance_fixed_request', '_queue_fixed_request_locked', 'fixed_arm_operation_callback',
                   '_fixed_operation_mode', '_fixed_arm_operation', '_set_incremental_entry_pose',
                   '_fixed_capture_input_error_locked', '_follow_input_time_locked')
        definitions.extend(node for node in mapper.body if getattr(node, 'name', None) in methods)
        self.now = 10.
        self.params = {}
        scope = dict(IntEnum=IntEnum, time=NS(monotonic=lambda: self.now),
                     rospy=NS(set_param=self.params.__setitem__),
                     TeleopTcpTarget=NS(SOURCE_QUEST_HAND_TRACKING=2),
                     changeArmCtrlModeResponse=lambda ok, mode, message: NS(result=ok, mode=mode))
        exec(compile(ast.Module(body=definitions, type_ignores=[]), str(path), 'exec'), scope)
        self.state = scope['AbsoluteEnableState']
        self.mapper = NS(
            enable_absolute_target_alignment=True, enable_velocity_ik_takeover=True,
            _clutches=[object(), object()], _reset_to_default=True,
            _absolute_lock=threading.RLock(), _fixed_operation_mutex=threading.Lock(),
            _absolute_state=self.state.WAITING,
            _absolute_session_id=0, _absolute_generation=0, _body_coordinate_epoch=0,
            _fixed_stop_epoch=0, _fixed_control_enabled=True, _fixed_pending_operation=None,
            _fixed_operation_error=False, _fixed_operation_block=False,
            _fixed_prepare_bootstrap=False, _fixed_dependency_timeout=12.,
            _fixed_acquisition_timeout=5., _absolute_mode_status=(0, 0, False),
            _absolute_fault_hold_pending=False, _tracking_source_hold_latched=False,
            _absolute_stop_pending=False, _absolute_external_mode_reset_pending=False,
            _absolute_latest_measured_q=np.tile([.26, 0., 0., -.5, 0., 0., 0.], 2),
            _tracking_last_accepted_source=1,
            absolute_enable_max_vr_age=.25, absolute_enable_max_joint_state_age=.1,
            quest3_arm_info_transformer=NS(is_stop_gesture=lambda: False),
            _configured_tracking_source=lambda: 1, _wheel_mpc_stable_for_mm_cmd=lambda: True,
            _publish_fixed_operation_state=lambda: None, _authorize_fixed_follow_locked=lambda: None,
            _start_velocity_session_locked=self.start_session)
        for name in methods:
            setattr(self.mapper, name, scope[name].__get__(self.mapper))
        self.fresh_feedback()

    def fresh_feedback(self):
        self.mapper._last_valid_tcp_receive = self.now
        self.mapper._joystick_receive_monotonic = self.now
        self.mapper._absolute_latest_measured_q_time = self.now

    def start_session(self, reason):
        self.mapper._absolute_state = self.state.SWITCHING
        self.mapper._absolute_session_id += 1

    def test_fresh_quest_and_robot_feedback_do_not_queue_prepare(self):
        # Represent the old automatic start condition after 30 accepted poses per arm.
        self.mapper._initial_arm_samples = [0, 0]
        for _ in range(120):
            self.now += .01
            self.fresh_feedback()
            self.mapper._advance_fixed_request()
            self.assertIsNone(self.mapper._fixed_pending_operation)
            self.assertEqual(self.mapper._absolute_state, self.state.WAITING)
            self.assertEqual(self.mapper._absolute_session_id, 0)
        self.assertFalse(self.mapper._fixed_prepare_bootstrap)
        self.assertFalse(self.mapper._fixed_operation_block)

    def test_first_xa_keeps_existing_follow_entry(self):
        # The wheel FSM maps reported operation mode 1 to follow on X+A.
        self.assertEqual(self.mapper._fixed_operation_mode(), 1)
        response = self.mapper.fixed_arm_operation_callback(NS(control_mode=2))
        self.assertTrue(response.result)
        self.assertEqual(response.mode, 2)
        self.assertTrue(self.params['/absolute_velocity_ik/use_incremental_entry_pose'])
        self.mapper._advance_fixed_request()
        self.assertEqual(self.mapper._absolute_state, self.state.SWITCHING)
        self.assertTrue(self.mapper._fixed_prepare_bootstrap)
        self.assertTrue(self.mapper._absolute_resume_after_prepare)


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

    def test_prepare_settle_accepts_reached_pose_with_raw_velocity_chatter(self):
        solver=(ROOT/'src/pinocchio_velocity_ik_node.cpp').read_text()
        settle=solver[solver.index('const bool settled ='):solver.index('preparation_settle_cycles_ = settled')]
        self.assertIn('reference_velocity <= preparation_velocity_tolerance_', settle)
        self.assertIn('measured_velocity_settled', settle)
        self.assertNotIn('selected_velocity', settle)
        # Bag 2026-11-25-15-26-09_19, session 67: the arm sat on the entry pose
        # for 18s. Reference speed was ~0 and position error was <0.01 rad, but
        # wrist joint_v stayed above 0.030 rad/s often enough that 50 consecutive
        # samples never happened (longest run 49). Chatter of 0.14 rad/s is
        # inside the raw allowance and must count; a joint still moving at
        # 0.40 rad/s must not.
        reference_tol, measured_tol, cycles = 0.030, 0.150, 50
        def settles(reference_speed, measured_speed, samples):
            run=0
            for _ in range(samples):
                ok=(reference_speed<=reference_tol and measured_speed<=measured_tol)
                run=run+1 if ok else 0
                if run>=cycles:
                    return True
            return False
        self.assertTrue(settles(0.0, 0.14, cycles))
        self.assertFalse(settles(0.0, 0.14, cycles-1))
        self.assertFalse(settles(0.05, 0.0, cycles*2))
        self.assertFalse(settles(0.0, 0.40, cycles*2))


if __name__=='__main__':
    unittest.main()
