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
    TeleopTcpTarget=NS(CONTROL_PREPARE=4, SOURCE_UNKNOWN=0,
                      SOURCE_QUEST_CONTROLLER=1, SOURCE_QUEST_HAND_TRACKING=2),
    WholeBodyIkReference=NS(AUTHORITY_ACTIVE=2),
    Int32=lambda **kw:NS(**kw))
source=Path(__file__).resolve().parents[1]/'scripts/ik_ros_velocity.py'
torso_spec=importlib.util.spec_from_file_location('torso',source.parent/'tools/torso_controls.py')
torso_module=importlib.util.module_from_spec(torso_spec);torso_spec.loader.exec_module(torso_module)
tree=ast.parse(source.read_text())
methods={'arm_mode_callback','_admit_velocity_gesture_locked','fixed_arm_operation_callback','_tracking_source_policy_locked','_start_velocity_session_locked','_velocity_session_cycle','_controller_start_request','_interrupt_fixed_prepare_locked','_fixed_operation_mode','_fixed_arm_operation','_fixed_capture_input_error_locked',
    '_follow_input_time_locked', '_maybe_start_optical_follow', '_tracking_source_policy',
    '_authorize_fixed_follow_locked','_queue_absolute_capture_locked','_handle_absolute_target_and_gesture','_handle_velocity_gesture_edge',
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
    def test_velocity_gesture_matches_existing_lifecycle(self):
        fields = ('_absolute_state', '_absolute_generation', '_absolute_stop_pending',
                  '_absolute_gesture_running', '_fixed_operation_block', '_tracking_control_running')
        for state in (State.WAITING, State.SWITCHING, State.ACTIVE,
                      State.RETURNING_TO_PREP, State.PREPARED):
            for previous, running in ((False, True), (True, False), (True, True)):
                results = []
                for method in ('_handle_absolute_target_and_gesture', '_handle_velocity_gesture_edge'):
                    self.setUp()
                    self.n._absolute_state = state
                    self.n._absolute_gesture_running = previous
                    self.n._absolute_frozen_target = None
                    self.n._reset_tracking_session_source_locked = lambda: None
                    self.n._clone_absolute_target = lambda x: x
                    self.n._publish_absolute_enable_status = lambda: None
                    if method == '_handle_absolute_target_and_gesture':
                        getattr(self.n, method)(None, running)
                    else:
                        getattr(self.n, method)(running)
                    results.append(tuple(getattr(self.n, k, None) for k in fields))
                self.assertEqual(results[0], results[1], (state, previous, running))

    def setUp(self):
        clock.now=10.;n=self.n=Mapper();n._absolute_lock=threading.RLock();n._fixed_operation_mutex=threading.Lock()
        n.enable_absolute_target_alignment=n.enable_velocity_ik_takeover=True
        n._absolute_state=State.WAITING;n._absolute_mode_status=(2,2,False)
        n._controller_start_consumed=False
        n._controller_start_completed=True
        n._torso_controls=torso_module.TorsoControls("quest")
        for name in ('_absolute_gesture_running','_fixed_operation_block','_fixed_prepare_bootstrap',
             '_fixed_operation_error','_absolute_fault_hold_pending','_tracking_source_hold_latched',
             '_absolute_stop_pending','_absolute_pause_pending','_absolute_external_mode_reset_pending',
             '_absolute_resume_after_prepare','_absolute_prepare_complete'):setattr(n,name,False)
        n._fixed_pending_operation=n._fixed_consumer_status=n._fixed_owner_status=None
        n._fixed_dependency_timeout=12.;n._fixed_acquisition_timeout=5.;n._fixed_prepare_timeout=20.
        n._fixed_operation_phase=0;n._body_coordinate_epoch=0
        n._fixed_control_enabled=True;n._fixed_stop_epoch=0
        n._absolute_latest_target=None;n._last_valid_tcp_receive=10.
        n._tracking_last_accepted_source=1;n._absolute_mode_status_time=10.;n.absolute_enable_mode_timeout=5.
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
        n=self.n;n._last_valid_tcp_receive=clock.now
        n._absolute_latest_measured_q_time=n._joystick_receive_monotonic=clock.now
    def ack(self,prepared=False):
        n=self.n
        m=NS(session_id=n._absolute_session_id,generation=n._absolute_generation,
            valid=True,takeover_active=True,session_bound=True,control_state=4,preparation_complete=prepared)
        owner=NS(**vars(m),authority_state=2,safety_braking=False,execution_owner_instance='owner',
            generated_monotonic_ns=int(clock.now*1e9),producer_lease_deadline_ns=int((clock.now+.05)*1e9))
        n._fixed_owner_callback(owner);n.whole_body_velocity_ik_status_callback(m)
    def start(self,mode=2):
        r=self.n._fixed_arm_operation(mode);self.n._advance_fixed_request();return r
    def test_cold_start_double_triggers_and_xa_five_trials_each(self):
        for staggered in (False,True):
            for trial in range(5):
                self.setUp();n=self.n;n._controller_start_completed=False;n._absolute_latest_measured_q=None
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
                n._advance_fixed_request();self.assertEqual(n._absolute_state,State.SWITCHING)
        for trial in range(5):
            self.setUp();self.n._absolute_latest_measured_q=None
            self.assertTrue(self.start().result)
            self.n._absolute_latest_measured_q=[0.]*14;self.fresh();self.n._advance_fixed_request()
            self.assertEqual(self.n._absolute_state,State.SWITCHING)


    def test_accept_is_not_completion(self):
        self.assertTrue(self.start().result)
        n=self.n
        self.assertEqual(n._absolute_state,State.SWITCHING)
        self.assertIsNone(n._absolute_latest_target)
        n._absolute_mode_request_time=clock.now
        n._fixed_consumer_status=(clock.now,NS(session_id=n._absolute_session_id,
            generation=n._absolute_generation,session_bound=False,valid=False,takeover_active=False))
        n._velocity_session_cycle(n._absolute_state,n._absolute_generation)
        self.assertEqual(n._absolute_state,State.SWITCHING)
        # Bound is sufficient even before execution ACTIVE; completion still needs owner ack.
        n._fixed_consumer_status[1].session_bound=True
        n._velocity_session_cycle(n._absolute_state,n._absolute_generation)
        self.assertEqual(n._absolute_state,State.ACTIVE)
        n._advance_fixed_request();self.assertEqual(n._fixed_operation_phase,2)
        self.ack();n._advance_fixed_request();self.assertEqual(n._fixed_operation_phase,4)

    def test_triggers_cannot_reopen_after_acknowledged_follow(self):
        for state in (State.WAITING, State.WAIT_RELEASE, State.PREPARED, State.PAUSED, State.ACTIVE):
            self.setUp(); n=self.n; n._controller_start_completed=False
            n._controller_start_request(NS(left_trigger=.9,right_trigger=.9))
            n._advance_fixed_request(); n._absolute_state=State.ACTIVE
            self.ack(); n._advance_fixed_request()
            self.assertTrue(n._controller_start_completed)
            n._fixed_pending_operation=None; n._absolute_state=state
            for _ in range(3):
                n._controller_start_request(NS(left_trigger=0.,right_trigger=0.))
                n._controller_start_request(NS(left_trigger=.9,right_trigger=.9))
                self.assertIsNone(n._fixed_pending_operation)

    def test_rejected_first_trigger_does_not_spend_admission(self):
        n=self.n;n._controller_start_completed=False;n._absolute_fault_hold_pending=True
        held=NS(left_trigger=.9,right_trigger=.9)
        n._controller_start_request(held)
        self.assertFalse(n._controller_start_consumed)
        n._absolute_fault_hold_pending=False
        n._controller_start_request(NS(left_trigger=0.,right_trigger=0.))
        n._controller_start_request(held)
        self.assertIsNotNone(n._fixed_pending_operation)
        self.assertTrue(n._controller_start_consumed)


    def test_xa_replacing_pending_prepare_spends_trigger_admission(self):
        n=self.n
        self.assertTrue(n._fixed_arm_operation(1).result)
        self.assertFalse(n._controller_start_consumed)
        self.assertTrue(n._fixed_arm_operation(2).result)
        self.assertTrue(n._controller_start_consumed)
        self.assertEqual(n._fixed_pending_operation['mode'],2)

    def test_timeout_before_first_follow_keeps_trigger_retry(self):
        n=self.n;n._controller_start_completed=False;n._absolute_latest_measured_q=None
        held=NS(left_trigger=.9,right_trigger=.9)
        n._controller_start_request(held)
        clock.now+=12.1;self.fresh();n._advance_fixed_request()
        n._controller_start_request(NS(left_trigger=0.,right_trigger=0.))
        n._controller_start_request(held)
        self.assertIsNotNone(n._fixed_pending_operation)
        self.assertFalse(n._fixed_arm_operation(2).result)
        self.assertIsNotNone(n._fixed_pending_operation)


    def test_explicit_follow_restores_default_after_revocation(self):
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
                    # Revocation alone cannot restore permission. The admitted
                    # explicit follow request restores the selected mode.
                    if event != 'initial_follow':
                        self.assertFalse(p.enabled)
                        self.assertFalse(p.startup_enable_pending)
                    if event == 'initial_follow': self.assertTrue(self.start().result)
                    n._authorize_fixed_follow_locked()
                    raw=NS(**{side+'_'+key:False for side in ('left','right') for key in
                              ('first_button_pressed','second_button_pressed','first_button_touched','second_button_touched')},
                           left_trigger=0.,right_trigger=0.,left_grip=0.,right_grip=0.)
                    p.update(raw,raw,10.,True);p.update(raw,raw,10.01,True)
                    self.assertTrue(p.enabled)
    def test_prepare_resume_clears_manual_torso_switches(self):
        for source_name in ('quest', 'pico'):
            self.setUp();n=self.n
            n._absolute_state=State.ACTIVE
            p=n._torso_controls=torso_module.TorsoControls(source_name)
            p.selected_enabled=False  # 腰部总开关手动关闭
            p.position_enabled=False  # 腰部细分（位置保持）手动打开
            p.enabled=True
            n._fixed_arm_operation(1)
            self.assertFalse(p.enabled)
            self.assertTrue(p.position_enabled)
            self.assertTrue(p.selected_enabled)
            n._authorize_fixed_follow_locked()
            raw=NS(**{side+'_'+key:False for side in ('left','right') for key in
                      ('first_button_pressed','second_button_pressed','first_button_touched','second_button_touched')},
                   left_trigger=0.,right_trigger=0.,left_grip=0.,right_grip=0.)
            p.update(raw,raw,10.,True);p.update(raw,raw,10.01,True)
            self.assertTrue(p.enabled)
            self.assertTrue(p.position_enabled)
            self.assertTrue(p.selected_enabled)

    def test_ready_pose_return_clears_manual_torso_switches(self):
        n=self.n
        p=n._torso_controls
        p.selected_enabled=False
        p.position_enabled=False
        p.enabled=True
        Mapper._pause_absolute_session(n)
        self.assertFalse(p.enabled)
        self.assertFalse(p.startup_enable_pending)
        self.assertTrue(p.position_enabled)
        self.assertTrue(p.selected_enabled)
        n._authorize_fixed_follow_locked()
        raw=NS(**{side+'_'+key:False for side in ('left','right') for key in
                  ('first_button_pressed','second_button_pressed','first_button_touched','second_button_touched')},
               left_trigger=0.,right_trigger=0.,left_grip=0.,right_grip=0.)
        p.update(raw,raw,10.,True);p.update(raw,raw,10.01,True)
        self.assertTrue(p.enabled)
        self.assertTrue(p.position_enabled)

    def test_pending_follow_without_dependencies_cannot_rearm_torso(self):
        n=self.n
        p=n._torso_controls
        p.reset()
        n._absolute_latest_measured_q=None
        self.assertTrue(n._fixed_arm_operation(2).result)
        self.assertFalse(p.enabled)
        self.assertFalse(p.startup_enable_pending)
        n._advance_fixed_request()
        self.assertEqual(n._absolute_state,State.WAITING)
        self.assertFalse(p.startup_enable_pending)
        n._absolute_latest_measured_q=[0.]*14
        self.fresh()
        n._advance_fixed_request()
        self.assertEqual(n._absolute_state,State.SWITCHING)
        self.assertTrue(p.startup_enable_pending)
        self.assertFalse(p.enabled)  # ready/fresh joystick still gates motion

    def test_missing_joint_and_mpc_wait_then_automatically_start(self):
        n=self.n;n._absolute_latest_measured_q=None;n._wheel_mpc_stable_for_mm_cmd=lambda:False
        self.assertTrue(self.start().result);self.assertEqual(n._absolute_state,State.WAITING)
        clock.now+=2;self.fresh();n._absolute_latest_measured_q=[0.]*14
        n._advance_fixed_request();self.assertEqual(n._absolute_state,State.WAITING)
        n._wheel_mpc_stable_for_mm_cmd=lambda:True;n._advance_fixed_request()
        self.assertEqual(n._absolute_state,State.SWITCHING)
    def test_duplicate_does_not_extend_deadline_or_capture(self):
        self.start();p=self.n._fixed_pending_operation.copy();gen=self.n._absolute_generation
        self.assertTrue(self.n._fixed_arm_operation(2).result)
        self.assertEqual(self.n._fixed_pending_operation,p);self.assertEqual(self.n._absolute_generation,gen)
    def test_dependency_timeout(self):
        self.n._absolute_latest_measured_q=None;self.start();clock.now+=12.1;self.fresh()
        self.n._advance_fixed_request();self.assertEqual(self.n._fixed_operation_phase,5)
        self.assertIsNone(self.n._fixed_pending_operation)
    def _record_failed_hold_requests(self):
        n=self.n;calls=[]
        n._Mapper__send_srv=True
        n._reset_tracking_session_source_locked=lambda:None
        n._finish_absolute_session=lambda *args:Mapper._finish_absolute_session(n,*args)
        n._request_absolute_hold_mode=lambda:calls.append('hold') or False
        return calls

    def test_disabled_dependency_timeout_cancels_without_hold_fault(self):
        n=self.n;n._fixed_control_enabled=False;n._absolute_mode_status=(0,0,False)
        calls=self._record_failed_hold_requests()
        self.assertTrue(self.start().result)
        generation=n._absolute_generation
        clock.now+=12.1;self.fresh();n._advance_fixed_request()
        self.assertEqual(calls,[])
        self.assertFalse(n._absolute_fault_hold_pending)
        self.assertIsNone(n._fixed_pending_operation)
        self.assertEqual(n._fixed_operation_phase,5)
        self.assertGreater(n._absolute_generation,generation)
        for _ in range(120):n._advance_fixed_request()
        self.assertEqual(calls,[])
        # A new explicit request can succeed once the original dependencies
        # become ready; the timed-out request itself never resumes.
        n._fixed_control_enabled=True
        self.assertTrue(self.start().result)
        self.assertEqual(n._absolute_state,State.SWITCHING)  # beta starts acquisition in this tick

    def test_acquisition_timeout_keeps_existing_hold_handshake(self):
        n=self.n;n._absolute_mode_status=(0,0,False)
        calls=self._record_failed_hold_requests();self.start()
        clock.now+=5.1;self.fresh();n._advance_fixed_request()
        self.assertEqual(calls,['hold'])
        self.assertTrue(n._absolute_fault_hold_pending)

    def test_dependency_timeout_with_existing_mode2_keeps_hold_handshake(self):
        n=self.n;n._fixed_control_enabled=False;n._absolute_mode_status=(2,2,False)
        calls=self._record_failed_hold_requests();self.start()
        clock.now+=12.1;self.fresh();n._advance_fixed_request()
        self.assertEqual(calls,['hold'])
        self.assertTrue(n._absolute_fault_hold_pending)

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
                       lambda n:setattr(n,'_last_valid_tcp_receive',9.),
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
        self.assertEqual(self.n._absolute_state,State.SWITCHING)
    def test_held_stop_cannot_authorize(self):
        self.n.quest3_arm_info_transformer.is_stop_gesture=lambda:True
        self.assertFalse(self.n._fixed_arm_operation(2).result)
    def test_stale_joystick_cannot_queue(self):
        self.n._joystick_receive_monotonic=9.;self.assertFalse(self.n._fixed_arm_operation(2).result)
    def test_fault_guard_preserved(self):
        self.n._absolute_fault_hold_pending=True;self.assertFalse(self.n._fixed_arm_operation(2).result)
        self.n._absolute_fault_hold_pending=False;self.n._tracking_source_hold_latched=True
        self.n.arm_mode_callback=lambda msg, **kw:None;self.assertFalse(self.n._fixed_arm_operation(2).result)
    def test_second_xa_interrupts_and_late_complete_cannot_undo_capture(self):
        n=self.n;self.start(1);n._absolute_state=State.RETURNING_TO_PREP
        self.ack();n._advance_fixed_request()
        old=n._fixed_consumer_status[1];generation=n._absolute_generation
        self.assertTrue(n._fixed_arm_operation(2).result)
        self.assertEqual(n._absolute_state,State.SWITCHING)
        self.assertGreater(n._absolute_generation,generation)
        self.assertFalse(n._absolute_resume_after_prepare)
        old.preparation_complete=True;n.whole_body_velocity_ik_status_callback(old)
        self.assertEqual(n._absolute_state,State.SWITCHING)
        pending=n._fixed_pending_operation.copy()
        self.assertTrue(n._fixed_arm_operation(2).result)
        self.assertEqual(n._fixed_pending_operation,pending)
        n._absolute_state=State.ACTIVE;self.ack();n._advance_fixed_request()
        self.assertEqual(n._fixed_operation_phase,4)

    def test_third_xa_replaces_follow_acquisition_with_prepare(self):
        n=self.n;self.start(1);n._absolute_state=State.RETURNING_TO_PREP
        self.ack();self.assertTrue(n._fixed_arm_operation(2).result)
        self.assertEqual(n._absolute_state,State.SWITCHING)
        generation=n._absolute_generation
        self.assertTrue(n._fixed_arm_operation(1).result)
        self.assertEqual(n._fixed_pending_operation['mode'],1)
        self.assertTrue(n._fixed_prepare_bootstrap and n._fixed_operation_block)
        self.assertEqual(n._absolute_generation,generation)

    def test_first_trigger_retries_mutex_contention_without_new_edge(self):
        n=self.n;n._controller_start_completed=False;held=NS(left_trigger=.9,right_trigger=.9)
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
                 lambda n:setattr(n,'_last_valid_tcp_receive',9.),
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
        self.assertEqual(n._absolute_state,State.SWITCHING)
    def test_callback_while_waiting_for_lock_is_not_stream_loss(self):
        self.start(1);n=self.n
        class CallbackBeforeLock:
            def __enter__(inner):
                clock.now+=.001
                n._joystick_receive_monotonic=clock.now
                n._last_valid_tcp_receive=clock.now
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
