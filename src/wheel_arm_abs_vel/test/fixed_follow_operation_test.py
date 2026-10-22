#!/usr/bin/env python3
"""Production AST lifecycle tests: deterministic clock, no ROS or actuators."""
import ast
import math
from enum import IntEnum
from pathlib import Path
from types import SimpleNamespace as NS
import threading
import unittest
import importlib.util
class Clock:
    now=10.
    def monotonic(self): return self.now
clock=Clock()
ns=dict(IntEnum=IntEnum,time=clock,math=math,
    rospy=NS(loginfo=lambda *a:None,logwarn=lambda *a:None,loginfo_throttle=lambda *a:None),
    changeArmCtrlModeResponse=lambda ok,mode,message:NS(result=ok,mode=mode,message=message),
    TeleopTcpTarget=NS(CONTROL_PREPARE=4),WholeBodyIkReference=NS(AUTHORITY_ACTIVE=2),
    Int32=lambda **kw:NS(**kw))
source=Path(__file__).resolve().parents[1]/'scripts/ik_ros_velocity.py'
torso_spec=importlib.util.spec_from_file_location('torso',source.parent/'tools/torso_controls.py')
torso_module=importlib.util.module_from_spec(torso_spec);torso_spec.loader.exec_module(torso_module)
tree=ast.parse(source.read_text())
methods={'_controller_start_request','_interrupt_fixed_prepare_locked','_fixed_operation_mode','_fixed_arm_operation','_fixed_capture_input_error_locked',
    '_authorize_fixed_follow_locked','_queue_absolute_capture_locked','_handle_absolute_target_and_gesture',
    'whole_body_velocity_ik_status_callback','_queue_fixed_request_locked','_advance_fixed_request',
    '_fixed_consumer_ack_locked','_fixed_owner_callback','_fixed_enable_callback',
    '_publish_wheel_base_command', '_torso_input_ready_locked',
    '_fill_subsystem_permissions_locked', '_bump_absolute_generation_locked',
    '_latch_tracking_fault_locked', '_pause_absolute_session', '_finish_absolute_session'}
selected=[]
for item in tree.body:
    if not isinstance(item,ast.ClassDef):continue
    if item.name in ('AbsoluteEnableState','AbsoluteEnableReason','TrackingSourceMode'):selected.append(item)
    elif any(isinstance(m,ast.FunctionDef) and m.name=='_fixed_arm_operation' for m in item.body):
        selected.append(ast.ClassDef(name='Mapper',bases=[],keywords=[],decorator_list=[],
            body=[m for m in item.body if isinstance(m,ast.FunctionDef) and m.name in methods]))
exec(compile(ast.fix_missing_locations(ast.Module(body=selected,type_ignores=[])),str(source),'exec'),ns)
Mapper,State=ns['Mapper'],ns['AbsoluteEnableState']
class FixedFollowTest(unittest.TestCase):
    def setUp(self):
        clock.now=10.;n=self.n=Mapper();n._absolute_lock=threading.RLock();n._fixed_operation_mutex=threading.Lock()
        n.enable_absolute_target_alignment=n.enable_velocity_ik_takeover=True
        n._absolute_state=State.WAITING;n._absolute_mode_status=(2,2,False)
        for name in ('_absolute_gesture_running','_fixed_operation_block','_fixed_prepare_bootstrap',
             '_fixed_operation_error','_absolute_fault_hold_pending','_tracking_source_hold_latched',
             '_absolute_stop_pending','_absolute_pause_pending','_absolute_external_mode_reset_pending',
             '_absolute_resume_after_prepare','_absolute_prepare_complete'):setattr(n,name,False)
        n._fixed_pending_operation=n._fixed_consumer_status=n._fixed_owner_status=None
        n._fixed_dependency_timeout=12.;n._fixed_acquisition_timeout=5.;n._fixed_prepare_timeout=20.
        n._fixed_operation_phase=0;n._body_coordinate_epoch=0
        n._fixed_control_enabled=True;n._fixed_stop_epoch=0
        n._absolute_latest_target={'receive_time':10.,'source':1}
        n._absolute_latest_measured_q=[0.]*14
        n._absolute_latest_measured_q_time=n._joystick_receive_monotonic=10.
        n._absolute_latest_measured_q_source='sensor'
        n.absolute_enable_max_vr_age=.25;n.absolute_enable_max_joint_state_age=.1
        n._absolute_generation=n._absolute_session_id=1
        n.tracking_source_mode=ns['TrackingSourceMode'].CONTROLLER
        n._configured_tracking_source=lambda:1
        n._clone_absolute_target=lambda t:None if t is None else t.copy()
        n._publish_absolute_enable_status=n._publish_fixed_operation_state=lambda:None
        n._wheel_mpc_stable_for_mm_cmd=lambda:True
        n.quest3_arm_info_transformer=NS(is_runing=False,ok_gesture_counts=0,shot_gesture_counts=49,is_stop_gesture=lambda:False)
        def bump():n._absolute_generation+=1;return n._absolute_generation
        n._bump_absolute_generation_locked=bump
        def finish(reason,request_hold,message):
            n._absolute_state=State.WAIT_RELEASE;n._absolute_generation+=1
        n._finish_absolute_session=finish
        n._request_absolute_hold_mode=lambda:True
        n._pause_absolute_session=lambda:setattr(n,'_absolute_state',State.RETURNING_TO_PREP)
    def fresh(self):
        n=self.n;n._absolute_latest_target={'receive_time':clock.now,'source':1}
        n._absolute_latest_measured_q_time=n._joystick_receive_monotonic=clock.now
    def ack(self,prepared=False):
        n=self.n
        m=NS(session_id=n._absolute_session_id,generation=n._absolute_generation,
            valid=True,takeover_active=True,control_state=4,preparation_complete=prepared)
        owner=NS(**vars(m),authority_state=2,safety_braking=False,execution_owner_instance='owner',
            generated_monotonic_ns=int(clock.now*1e9),producer_lease_deadline_ns=int((clock.now+.05)*1e9))
        n._fixed_owner_callback(owner);n.whole_body_velocity_ik_status_callback(m)
    def start(self,mode=2):
        r=self.n._fixed_arm_operation(mode);self.n._advance_fixed_request();return r
    def test_cold_start_double_triggers_and_xa_five_trials_each(self):
        for staggered in (False,True):
            for trial in range(5):
                self.setUp();n=self.n;n._absolute_latest_measured_q=None
                if staggered:
                    n._controller_start_request(NS(left_trigger=.9,right_trigger=0.))
                    self.assertIsNone(n._fixed_pending_operation)
                n._controller_start_request(NS(left_trigger=.9,right_trigger=.9))
                self.assertIsNotNone(n._fixed_pending_operation)
                deadline=n._fixed_pending_operation['deadline']
                n._controller_start_request(NS(left_trigger=.9,right_trigger=.9))
                self.assertEqual(n._fixed_pending_operation['deadline'],deadline)
                n._advance_fixed_request();self.assertEqual(n._absolute_state,State.WAITING)
                clock.now+=.05;self.fresh();n._absolute_latest_measured_q=[0.]*14
                n._advance_fixed_request();self.assertEqual(n._absolute_state,State.CAPTURED)
        for trial in range(5):
            self.setUp();self.n._absolute_latest_measured_q=None
            self.assertTrue(self.start().result)
            self.n._absolute_latest_measured_q=[0.]*14;self.fresh();self.n._advance_fixed_request()
            self.assertEqual(self.n._absolute_state,State.CAPTURED)

    def test_accept_is_not_completion(self):
        self.assertTrue(self.start().result);self.assertEqual(self.n._absolute_state,State.CAPTURED)
        self.n._absolute_state=State.ACTIVE;self.n._advance_fixed_request()
        self.assertEqual(self.n._fixed_operation_phase,2)
        self.ack();self.n._advance_fixed_request();self.assertEqual(self.n._fixed_operation_phase,4)

    def test_startup_follow_preserves_default_but_revocations_cancel_it(self):
        for source_name in ('quest', 'pico'):
            for event in ('initial_follow', 'prepare', 'hold', 'fault', 'session_stop', 'session_prepare'):
                with self.subTest(source=source_name, event=event):
                    self.setUp();n=self.n
                    p=n._torso_controls=torso_module.TorsoControls(source_name)
                    if event == 'prepare': n._fixed_arm_operation(1)
                    elif event == 'hold': n._fixed_arm_operation(0)
                    elif event == 'fault':
                        n._tracking_source_hold_count=0
                        n._latch_tracking_fault_locked(ns['AbsoluteEnableReason'].NONE, False)
                    elif event == 'session_stop':
                        n._reset_tracking_session_source_locked=lambda:None
                        Mapper._finish_absolute_session(n, ns['AbsoluteEnableReason'].NONE, False, 'test stop')
                    elif event == 'session_prepare': Mapper._pause_absolute_session(n)
                    # Exercise both production first-follow request and its
                    # later authorization; neither may rearm a cancelled default.
                    if event == 'initial_follow': self.assertTrue(self.start().result)
                    n._authorize_fixed_follow_locked()
                    raw=NS(**{side+'_'+key:False for side in ('left','right') for key in
                              ('first_button_pressed','second_button_pressed','first_button_touched','second_button_touched')},
                           left_trigger=0.,right_trigger=0.,left_grip=0.,right_grip=0.)
                    p.update(raw,raw,10.,True);p.update(raw,raw,10.01,True)
                    self.assertEqual(p.enabled,event=='initial_follow')
    def test_missing_joint_and_mpc_wait_then_automatically_start(self):
        n=self.n;n._absolute_latest_measured_q=None;n._wheel_mpc_stable_for_mm_cmd=lambda:False
        self.assertTrue(self.start().result);self.assertEqual(n._absolute_state,State.WAITING)
        clock.now+=2;self.fresh();n._absolute_latest_measured_q=[0.]*14
        n._advance_fixed_request();self.assertEqual(n._absolute_state,State.WAITING)
        n._wheel_mpc_stable_for_mm_cmd=lambda:True;n._advance_fixed_request()
        self.assertEqual(n._absolute_state,State.CAPTURED)
    def test_duplicate_does_not_extend_deadline_or_capture(self):
        self.start();p=self.n._fixed_pending_operation.copy();gen=self.n._absolute_generation
        self.assertTrue(self.n._fixed_arm_operation(2).result)
        self.assertEqual(self.n._fixed_pending_operation,p);self.assertEqual(self.n._absolute_generation,gen)
    def test_dependency_timeout(self):
        self.n._absolute_latest_measured_q=None;self.start();clock.now+=12.1;self.fresh()
        self.n._advance_fixed_request();self.assertEqual(self.n._fixed_operation_phase,5)
        self.assertIsNone(self.n._fixed_pending_operation)
    def test_mode_ack_without_consumer_times_out(self):
        self.start();self.n._absolute_state=State.ACTIVE;clock.now+=5.1;self.fresh()
        self.n._advance_fixed_request();self.assertEqual(self.n._fixed_operation_phase,5)
    def test_wrong_generation_and_expired_owner_not_ack(self):
        self.start();self.ack();self.n._fixed_owner_status[1].generation-=1
        self.assertFalse(self.n._fixed_consumer_ack_locked(clock.now))
        self.ack();clock.now+=.06;self.assertFalse(self.n._fixed_consumer_ack_locked(clock.now))
    def test_cancel_stop_epoch_source_generation_and_stream_loss(self):
        for change in [lambda n:setattr(n,'_absolute_stop_pending',True),
                       lambda n:setattr(n,'_body_coordinate_epoch',1),
                       lambda n:setattr(n,'_configured_tracking_source',lambda:2),
                       lambda n:setattr(n,'_absolute_generation',99),
                       lambda n:setattr(n,'_absolute_latest_target',{'receive_time':9.,'source':1}),
                       lambda n:setattr(n,'_joystick_receive_monotonic',9.)]:
            self.setUp();self.start();change(self.n);self.n._advance_fixed_request()
            self.assertIsNone(self.n._fixed_pending_operation);self.assertEqual(self.n._fixed_operation_phase,5)
    def test_hold_cancels_queued_request(self):
        self.start();self.assertTrue(self.n._fixed_arm_operation(0).result)
        self.assertIsNone(self.n._fixed_pending_operation)
    def test_disable_then_reenable_cannot_resurrect_request(self):
        self.start();self.n._fixed_enable_callback(NS(data=False))
        self.n._fixed_enable_callback(NS(data=True));self.n._advance_fixed_request()
        self.assertIsNone(self.n._fixed_pending_operation)
        self.assertEqual(self.n._fixed_operation_phase,5)
    def test_not_enabled_waits_without_capture(self):
        self.n._fixed_control_enabled=False;self.start()
        self.assertEqual(self.n._absolute_state,State.WAITING)
        self.n._fixed_enable_callback(NS(data=True));self.n._advance_fixed_request()
        self.assertEqual(self.n._absolute_state,State.CAPTURED)
    def test_held_stop_cannot_authorize(self):
        self.n.quest3_arm_info_transformer.is_stop_gesture=lambda:True
        self.assertFalse(self.n._fixed_arm_operation(2).result)
    def test_stale_joystick_cannot_queue(self):
        self.n._joystick_receive_monotonic=9.;self.assertFalse(self.n._fixed_arm_operation(2).result)
    def test_fault_guard_preserved(self):
        self.n._absolute_fault_hold_pending=True;self.assertFalse(self.n._fixed_arm_operation(2).result)
        self.n._absolute_fault_hold_pending=False;self.n._tracking_source_hold_latched=True
        self.n.arm_mode_callback=lambda msg:None;self.assertFalse(self.n._fixed_arm_operation(2).result)
    def test_second_xa_interrupts_and_late_complete_cannot_undo_capture(self):
        n=self.n;self.start(1);n._absolute_state=State.RETURNING_TO_PREP
        self.ack();n._advance_fixed_request()
        old=n._fixed_consumer_status[1];generation=n._absolute_generation
        self.assertTrue(n._fixed_arm_operation(2).result)
        self.assertEqual(n._absolute_state,State.CAPTURED)
        self.assertGreater(n._absolute_generation,generation)
        self.assertFalse(n._absolute_resume_after_prepare)
        old.preparation_complete=True;n.whole_body_velocity_ik_status_callback(old)
        self.assertEqual(n._absolute_state,State.CAPTURED)
        pending=n._fixed_pending_operation.copy()
        self.assertTrue(n._fixed_arm_operation(2).result)
        self.assertEqual(n._fixed_pending_operation,pending)
        n._absolute_state=State.ACTIVE;self.ack();n._advance_fixed_request()
        self.assertEqual(n._fixed_operation_phase,4)

    def test_third_xa_replaces_follow_acquisition_with_prepare(self):
        n=self.n;self.start(1);n._absolute_state=State.RETURNING_TO_PREP
        self.ack();self.assertTrue(n._fixed_arm_operation(2).result)
        self.assertEqual(n._absolute_state,State.CAPTURED)
        generation=n._absolute_generation
        self.assertTrue(n._fixed_arm_operation(1).result)
        self.assertEqual(n._fixed_pending_operation['mode'],1)
        self.assertTrue(n._fixed_prepare_bootstrap and n._fixed_operation_block)
        self.assertEqual(n._absolute_generation,generation)

    def test_first_trigger_retries_mutex_contention_without_new_edge(self):
        n=self.n;held=NS(left_trigger=.9,right_trigger=.9)
        n._fixed_operation_mutex.acquire()
        n._controller_start_request(held);self.assertIsNone(n._fixed_pending_operation)
        n._fixed_operation_mutex.release()
        n._controller_start_request(held);self.assertIsNotNone(n._fixed_pending_operation)

    def test_prepare_interrupt_rejects_unhealthy_input_or_transaction(self):
        changes=[lambda n:setattr(n,'_fixed_control_enabled',False),
                 lambda n:setattr(n,'_absolute_stop_pending',True),
                 lambda n:setattr(n,'_absolute_fault_hold_pending',True),
                 lambda n:setattr(n,'_absolute_external_mode_reset_pending',True),
                 lambda n:setattr(n,'_fixed_stop_epoch',1),
                 lambda n:setattr(n,'_body_coordinate_epoch',1),
                 lambda n:setattr(n,'_configured_tracking_source',lambda:2),
                 lambda n:setattr(n.quest3_arm_info_transformer,'is_stop_gesture',lambda:True),
                 lambda n:setattr(n,'_joystick_receive_monotonic',9.),
                 lambda n:setattr(n,'_absolute_latest_measured_q_time',9.),
                 lambda n:n._absolute_latest_target.update(receive_time=9.),
                 lambda n:n._fixed_pending_operation.update(generation=99),
                 lambda n:n._fixed_pending_operation.update(deadline=9.),
                 lambda n:setattr(n,'_fixed_owner_status',None)]
        for change in changes:
            self.setUp();n=self.n;self.start(1);n._absolute_state=State.RETURNING_TO_PREP
            self.ack();change(n)
            self.assertFalse(n._fixed_arm_operation(2).result)
            self.assertEqual(n._absolute_state,State.RETURNING_TO_PREP)

    def test_follow_before_prepare_acquisition_replaces_intent(self):
        n=self.n;n._fixed_control_enabled=False
        self.assertTrue(n._fixed_arm_operation(1).result)
        deadline=n._fixed_pending_operation['deadline']
        self.assertTrue(n._fixed_arm_operation(2).result)
        self.assertEqual(n._fixed_pending_operation['mode'],2)
        self.assertEqual(n._fixed_pending_operation['deadline'],deadline)
        self.assertFalse(n._fixed_prepare_bootstrap)

    def test_stale_second_xa_does_not_queue_follow(self):
        n=self.n;self.start(1);n._absolute_state=State.RETURNING_TO_PREP
        n._joystick_receive_monotonic=9.
        self.assertFalse(n._fixed_arm_operation(2).result)
        self.assertFalse(n._absolute_resume_after_prepare)
    def test_follow_after_completed_prepare(self):
        n=self.n;self.start(1);n._absolute_state=State.RETURNING_TO_PREP
        self.ack(True);n._advance_fixed_request()
        self.assertEqual(n._absolute_state,State.PREPARED)
        self.assertTrue(n._fixed_arm_operation(2).result);n._advance_fixed_request()
        self.assertEqual(n._absolute_state,State.CAPTURED)
    def test_callback_while_waiting_for_lock_is_not_stream_loss(self):
        self.start(1);n=self.n
        class CallbackBeforeLock:
            def __enter__(inner):
                clock.now+=.001
                n._joystick_receive_monotonic=clock.now
                n._absolute_latest_target['receive_time']=clock.now
            def __exit__(inner,*args):pass
        n._absolute_lock=CallbackBeforeLock()
        n._advance_fixed_request()
        self.assertIsNotNone(n._fixed_pending_operation)
        self.assertNotEqual(n._fixed_operation_phase,5)
    def test_prepare_runs_without_held_buttons(self):
        self.start(1);n=self.n;n._absolute_state=State.RETURNING_TO_PREP
        n.joySticks_data=NS(left_first_button_pressed=False,right_first_button_pressed=False)
        for _ in range(100):
            clock.now+=.02;self.fresh();self.ack();n._advance_fixed_request()
            self.assertIsNotNone(n._fixed_pending_operation)
        self.ack(True);n._advance_fixed_request()
        self.assertEqual(n._fixed_operation_phase,4)
    def test_prepare_timeout(self):
        self.start(1);self.n._absolute_state=State.RETURNING_TO_PREP;self.ack();self.n._advance_fixed_request()
        clock.now+=20.1;self.fresh();self.n._advance_fixed_request();self.assertEqual(self.n._fixed_operation_phase,5)
    def test_follow_authority_and_real_stop_preserved(self):
        self.start();n=self.n;self.assertTrue(n.quest3_arm_info_transformer.is_runing)
        n._absolute_state=State.ACTIVE;self.ack();n._advance_fixed_request();gen=n._absolute_generation
        n._handle_absolute_target_and_gesture(n._absolute_latest_target,True)
        self.assertEqual(n._absolute_generation,gen)
        n._handle_absolute_target_and_gesture(n._absolute_latest_target,False)
        self.assertTrue(n._absolute_stop_pending)
if __name__=='__main__':unittest.main()
