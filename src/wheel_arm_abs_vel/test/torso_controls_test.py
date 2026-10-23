#!/usr/bin/env python3
"""Torso gesture arbitration and production mapper admission, without ROS."""
import ast
import copy
import sys
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
import importlib.util
spec=importlib.util.spec_from_file_location('torso_controls',ROOT/'scripts/tools/torso_controls.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
TorsoControls=module.TorsoControls

def joy(**kw):
    fields = {side+'_'+key: False for side in ('left', 'right') for key in
              ('first_button_pressed','second_button_pressed','first_button_touched','second_button_touched')}
    fields.update({side+'_'+key: 0. for side in ('left','right') for key in ('trigger','grip')})
    fields.update(kw)
    return NS(**fields)

class ControlsTest(unittest.TestCase):
    def setUp(self): self.now=10.
    def sample(self, p, ready=True, **kw):
        self.now+=.01
        raw=joy(**kw); before=copy.copy(raw)
        result=p.update(raw, raw, self.now, ready)
        self.assertEqual(vars(raw),vars(before))
        return result
    def chord(self, p):
        return (dict(left_first_button_touched=True,left_second_button_touched=True,right_second_button_pressed=True)
                if p.source=='quest' else dict(left_second_button_pressed=True,right_trigger=.8))
    def enable(self,p):
        self.sample(p); self.sample(p,**self.chord(p)); self.assertTrue(p.enabled)
    def test_default_locked_xa_and_xb_cannot_enable(self):
        for source in ('quest','pico'):
            p=TorsoControls(source, enable_on_first_follow=False)
            for button in ('right_first_button_pressed','right_second_button_pressed'):
                self.sample(p);self.sample(p,left_first_button_pressed=True,**{button:True})
                self.assertFalse(p.enabled)
    def test_toggle_edge_and_release_for_both_devices(self):
        for source in ('quest','pico'):
            p=TorsoControls(source, enable_on_first_follow=False);self.enable(p)
            for _ in range(40):self.sample(p,**self.chord(p));self.assertTrue(p.enabled)
            self.sample(p);self.sample(p,**self.chord(p));self.assertFalse(p.enabled)
    def test_quest_modifier_and_b_are_consumed(self):
        p=TorsoControls('quest', enable_on_first_follow=False);self.sample(p)
        for kw in [dict(left_first_button_touched=True,left_second_button_touched=True),self.chord(p),dict(right_second_button_pressed=True),{}]:
            out=self.sample(p,**kw)
            self.assertFalse(out.left_first_button_touched or out.left_second_button_touched or out.right_second_button_pressed)
        out=self.sample(p,left_first_button_touched=True,right_second_button_pressed=True)
        self.assertTrue(out.left_first_button_touched and out.right_second_button_pressed)
    def test_quest_requires_b_press_while_modifier_is_present(self):
        p=TorsoControls('quest', enable_on_first_follow=False);self.sample(p);self.sample(p,right_second_button_pressed=True)
        self.sample(p,**self.chord(p));self.assertFalse(p.enabled)
        # Keep XY touched and release/repress B, matching the original FSM.
        modifier=dict(left_first_button_touched=True,left_second_button_touched=True)
        self.sample(p,**modifier);self.assertFalse(p.enabled)
        self.sample(p,**self.chord(p));self.assertTrue(p.enabled)
        self.sample(p,**modifier);self.assertTrue(p.enabled)
        self.sample(p,**self.chord(p));self.assertFalse(p.enabled)
    def test_quest_retouch_while_b_held_does_not_toggle(self):
        p=TorsoControls('quest', enable_on_first_follow=False);self.enable(p)
        for _ in range(5):
            self.sample(p,right_second_button_pressed=True)
            self.sample(p,**self.chord(p));self.assertTrue(p.enabled)
        self.sample(p,left_first_button_touched=True,left_second_button_touched=True)
        self.sample(p,**self.chord(p));self.assertFalse(p.enabled)
    def test_quest_xb_release_x_cannot_become_torso_enable(self):
        p=TorsoControls('quest', enable_on_first_follow=False);self.sample(p)
        self.sample(p,**dict(self.chord(p),left_first_button_pressed=True))
        self.sample(p,**self.chord(p));self.assertFalse(p.enabled)
        self.sample(p,left_first_button_touched=True,left_second_button_touched=True)
        self.sample(p,**self.chord(p));self.assertTrue(p.enabled)
    def test_quest_modifier_alone_never_toggles(self):
        p=TorsoControls('quest', enable_on_first_follow=False);self.sample(p)
        for _ in range(5):
            self.sample(p,left_first_button_touched=True,left_second_button_touched=True)
            self.sample(p);self.assertFalse(p.enabled)
    def test_xb_keeps_torso_permission_but_xa_and_stop_revoke_it(self):
        for source in ('quest','pico'):
            p=TorsoControls(source, enable_on_first_follow=False);self.enable(p)
            self.sample(p,left_first_button_pressed=True,right_second_button_pressed=True)
            self.assertTrue(p.enabled)
            self.sample(p);self.assertTrue(p.enabled)
            self.sample(p,left_first_button_pressed=True,right_first_button_pressed=True)
            self.assertFalse(p.enabled)
            self.enable(p)
            self.sample(p,left_first_button_pressed=True,left_second_button_pressed=True)
            self.assertFalse(p.enabled)
    def test_pico_y_first_has_no_grip_or_freeze_side_effect(self):
        p=TorsoControls('pico', enable_on_first_follow=False);self.sample(p)
        for kw in [dict(left_second_button_pressed=True),dict(left_second_button_pressed=True,right_trigger=.3),self.chord(p),dict(right_trigger=.8),{},{}]:
            out=self.sample(p,**kw)
            self.assertFalse(out.left_second_button_pressed)
            self.assertEqual(out.right_trigger,0.)
        self.assertTrue(p.enabled)
        self.sample(p,right_trigger=.7)
        self.now+=.16
        self.assertEqual(self.sample(p,right_trigger=.7).right_trigger,.7)
    def test_pico_rt_first_short_window_has_no_gripper_close(self):
        p=TorsoControls('pico', enable_on_first_follow=False);self.sample(p)
        self.assertEqual(self.sample(p,right_trigger=.6).right_trigger,0.)
        for kw in [self.chord(p),dict(left_second_button_pressed=True),{}]:
            out=self.sample(p,**kw)
            self.assertFalse(out.left_second_button_pressed);self.assertEqual(out.right_trigger,0.)
        self.assertTrue(p.enabled)
    def test_long_standalone_rt_resumes_after_window_and_late_y_holds_value(self):
        p=TorsoControls('pico', enable_on_first_follow=False);self.sample(p)
        self.assertEqual(self.sample(p,right_trigger=.8).right_trigger,0.)
        self.now+=.16
        self.assertEqual(self.sample(p,right_trigger=.8).right_trigger,.8)
        out=self.sample(p,**self.chord(p))
        self.assertTrue(p.enabled);self.assertEqual(out.right_trigger,.8)
        self.assertFalse(self.sample(p).left_second_button_pressed)

    def test_pico_y_alone_pulses_once_on_release(self):
        p=TorsoControls('pico', enable_on_first_follow=False);self.sample(p)
        for _ in range(30):self.assertFalse(self.sample(p,left_second_button_pressed=True).left_second_button_pressed)
        self.assertTrue(self.sample(p).left_second_button_pressed)
        self.assertFalse(self.sample(p).left_second_button_pressed)
    def test_hold_prepare_fault_stale_and_reset_require_fresh_chord(self):
        for source in ('quest','pico'):
            for failure in ('not_ready','stale','reset','stop','arm','nan'):
                p=TorsoControls(source, enable_on_first_follow=False);self.enable(p)
                if failure=='not_ready': self.sample(p,ready=False,**self.chord(p))
                elif failure=='stale': self.now+=1.;self.assertFalse(p.authorized(self.now,True));self.sample(p,**self.chord(p))
                elif failure=='reset':p.reset();self.sample(p,**self.chord(p))
                elif failure=='stop':self.sample(p,left_first_button_pressed=True,left_second_button_pressed=True)
                elif failure=='arm':self.sample(p,left_first_button_pressed=True,right_first_button_pressed=True)
                else:self.sample(p,**dict(self.chord(p),left_grip=float('nan')))
                self.assertFalse(p.enabled,failure)
                self.sample(p);self.sample(p,**self.chord(p));self.assertTrue(p.enabled,failure)
    def test_new_stream_with_held_chord_is_not_an_edge(self):
        for source in ('quest','pico'):
            p=TorsoControls(source, enable_on_first_follow=False)
            for _ in range(4):self.sample(p,**self.chord(p));self.assertFalse(p.enabled)
            self.sample(p);self.sample(p,**self.chord(p));self.assertTrue(p.enabled)
    def test_pico_y_event_is_consumed_before_slow_hand_timer(self):
        tree=ast.parse((ROOT/'scripts/ik_ros_velocity.py').read_text())
        cls=next(x for x in tree.body if isinstance(x,ast.ClassDef) and x.name=='IkRos')
        fn=next(x for x in cls.body if isinstance(x,ast.FunctionDef) and x.name=='_consume_torso_end_effector_edges')
        env={name:name for name in ('QIANGNAO','QIANGNAO_TOUCH','REVO2','LINKER_HAND','LEJUCLAW')}
        exec(compile(ast.Module(body=[fn],type_ignores=[]),'<production>','exec'),env)
        fn=env['_consume_torso_end_effector_edges']
        for kind in env.copy():
            if kind not in ('QIANGNAO','QIANGNAO_TOUCH','REVO2','LINKER_HAND','LEJUCLAW'):continue
            n=NS(_torso_controls=TorsoControls('pico', enable_on_first_follow=False),sg100_enabled=False,end_effector_type=kind,__freeze_finger=False)
            event=joy(left_second_button_pressed=True);fn(n,event)
            self.assertTrue(getattr(n,'__freeze_finger'));self.assertFalse(event.left_second_button_pressed)
            fn(n,event);self.assertTrue(getattr(n,'__freeze_finger'))

    def test_actual_sg100_touch_gate_is_suppressed_during_torso_modifier(self):
        import math
        import threading
        tree=ast.parse((ROOT/'scripts/ik_ros_velocity.py').read_text())
        cls=next(x for x in tree.body if isinstance(x,ast.ClassDef) and x.name=='IkRos')
        fn=next(x for x in cls.body if isinstance(x,ast.FunctionDef) and x.name=='_publish_sg100_triggers')
        spec=importlib.util.spec_from_file_location('sg100',ROOT/'scripts/tools/sg100_vr_gesture.py')
        sg=importlib.util.module_from_spec(spec);spec.loader.exec_module(sg)
        gate=sg.GestureTouchGate();events=[]
        env={'math':math,'Vector3Stamped':lambda:NS(header=NS(),vector=NS(x=0.,y=0.,z=0.)),
             'rospy':NS(Time=NS(now=lambda:0))}
        exec(compile(ast.Module(body=[fn],type_ignores=[]),'<production>','exec'),env)
        p=TorsoControls('quest', enable_on_first_follow=False);self.sample(p,left_first_button_touched=True)
        n=NS(_torso_controls=p,sg100_trigger_pub=NS(publish=lambda m:None),stop_event=threading.Event(),
             _absolute_lock=threading.RLock(),_tracking_source_hold_latched=False,
             quest3_arm_info_transformer=NS(is_hand_tracking=False,left_finger_joints=[],right_finger_joints=[],is_stop_gesture=lambda:False),
             sg100_gesture_client=NS(update=lambda msg,valid:events.append(gate.update(msg,valid,self.now))))
        for _ in range(60):
            msg=self.sample(p,**dict(self.chord(p),right_second_button_touched=True))
            env['_publish_sg100_triggers'](n,msg)
        self.assertTrue(p.enabled);self.assertFalse(any(events))

    def test_actual_mapper_readiness(self):
        tree=ast.parse((ROOT/'scripts/ik_ros_velocity.py').read_text())
        cls=next(x for x in tree.body if isinstance(x,ast.ClassDef) and x.name=='IkRos')
        fn=next(x for x in cls.body if isinstance(x,ast.FunctionDef) and x.name=='_torso_input_ready_locked')
        env={'AbsoluteEnableState':NS(ACTIVE=3)}
        exec(compile(ast.Module(body=[fn],type_ignores=[]),'<production>','exec'),env)
        good=dict(enable_velocity_ik_takeover=True,_absolute_state=3,_fixed_control_enabled=True,
                  _absolute_mode_status=(2,2,False),_fixed_pending_operation=None,_fixed_operation_error='',
                  _fixed_operation_block='',_absolute_stop_pending=False,_absolute_fault_hold_pending=False,
                  _tracking_source_hold_latched=False,_absolute_external_mode_reset_pending=False,
                  _absolute_latest_target={'receive_time':10.},absolute_enable_max_vr_age=.2)
        fn=env['_torso_input_ready_locked'];self.assertTrue(fn(NS(**good),10.1))
        for key,value in dict(enable_velocity_ik_takeover=False,_absolute_state=2,_fixed_control_enabled=False,
             _absolute_mode_status=(1,2,True),_fixed_pending_operation=2,_fixed_operation_error='fault',
             _fixed_operation_block='hold',_absolute_stop_pending=True,_absolute_fault_hold_pending=True,
             _tracking_source_hold_latched=True,_absolute_external_mode_reset_pending=True,_absolute_latest_target=None).items():
            self.assertFalse(fn(NS(**dict(good,**{key:value})),10.1),key)
        self.assertFalse(fn(NS(**good),10.3));self.assertFalse(fn(NS(**good),9.9))


class StartupTorsoControlsTest(unittest.TestCase):
    setUp = ControlsTest.setUp
    sample = ControlsTest.sample
    chord = ControlsTest.chord

    def test_default_waits_for_first_ready_fresh_input(self):
        for source in ('quest', 'pico'):
            p = TorsoControls(source)
            self.assertFalse(p.authorized(self.now, False))
            for _ in range(5):
                self.sample(p, ready=False)
                self.assertFalse(p.enabled)
            self.sample(p)
            self.assertTrue(p.enabled)
            self.assertTrue(p.authorized(self.now, True))
            self.assertFalse(p.startup_enable_pending)

    def test_manual_off_stays_off_and_same_chord_reenables(self):
        for source in ('quest', 'pico'):
            p = TorsoControls(source)
            self.sample(p, ready=False); self.sample(p)
            self.sample(p, **self.chord(p)); self.assertFalse(p.enabled)
            for _ in range(30):
                self.sample(p); self.assertFalse(p.enabled)
            self.sample(p, **self.chord(p)); self.assertTrue(p.enabled)

    def test_prepare_stop_stale_and_fault_do_not_reapply_startup(self):
        for source in ('quest', 'pico'):
            for event in ('prepare', 'stop', 'stale', 'not_ready', 'reset'):
                p = TorsoControls(source)
                self.sample(p, ready=False); self.sample(p)
                if event == 'prepare':
                    self.sample(p, left_first_button_pressed=True, right_first_button_pressed=True)
                elif event == 'stop':
                    self.sample(p, left_first_button_pressed=True, left_second_button_pressed=True)
                elif event == 'stale':
                    self.now += 1.; self.assertFalse(p.authorized(self.now, True))
                elif event == 'not_ready': self.sample(p, ready=False)
                else: p.reset()
                self.sample(p); self.sample(p)
                self.assertFalse(p.enabled, (source,event))
                self.sample(p, **self.chord(p)); self.assertTrue(p.enabled)

    def test_revoke_before_first_follow_cancels_pending_default(self):
        for source in ('quest', 'pico'):
            for event in ('reset', 'stop', 'invalid', 'stale_update', 'stale_poll'):
                with self.subTest(source=source, event=event):
                    p = TorsoControls(source)
                    self.sample(p, ready=False)
                    if event == 'reset': p.reset()
                    elif event == 'prepare':
                        self.sample(p, ready=False, left_first_button_pressed=True, right_first_button_pressed=True)
                    elif event == 'stop':
                        self.sample(p, ready=False, left_first_button_pressed=True, left_second_button_pressed=True)
                    elif event == 'invalid': self.sample(p, ready=False, right_trigger=float('nan'))
                    else:
                        self.now += 1.
                        if event == 'stale_poll': self.assertFalse(p.authorized(self.now, False))
                        else: self.sample(p, ready=False)
                    self.assertFalse(p.startup_enable_pending)
                    for _ in range(5):
                        self.sample(p); self.assertFalse(p.enabled)
                    self.sample(p, **self.chord(p)); self.assertTrue(p.enabled)

    def test_explicit_follow_after_prepare_restores_selected_mode(self):
        for source in ('quest', 'pico'):
            for manual_off in (False, True):
                with self.subTest(source=source, manual_off=manual_off):
                    p = TorsoControls(source)
                    self.sample(p, ready=False); self.sample(p)
                    if manual_off:
                        self.sample(p, **self.chord(p)); self.sample(p)
                    p.reset()  # production mapper enters PREPARE
                    self.sample(p, ready=False)
                    p.request_follow()  # admitted X+A resume, not readiness alone
                    for _ in range(3):
                        self.sample(p, ready=False, left_first_button_pressed=True,
                                    right_first_button_pressed=True)
                        self.assertFalse(p.enabled)
                    self.sample(p, ready=False); self.sample(p)
                    self.assertEqual(p.enabled, not manual_off)
                    self.sample(p, **self.chord(p))
                    self.assertEqual(p.enabled, manual_off)

    def test_cold_prepare_then_first_follow_uses_default_torso(self):
        for source in ('quest', 'pico'):
            p = TorsoControls(source)
            self.sample(p, ready=False)
            p.reset()
            self.sample(p, ready=False, left_first_button_pressed=True,
                        right_first_button_pressed=True)
            self.assertFalse(p.enabled)
            p.request_follow()
            self.sample(p, ready=False)
            self.sample(p)
            self.assertTrue(p.enabled)

    def test_fault_after_follow_request_cancels_pending_permission(self):
        for source in ('quest', 'pico'):
            for event in ('stop', 'stale', 'invalid'):
                p = TorsoControls(source)
                self.sample(p, ready=False); p.request_follow()
                if event == 'stop':
                    self.sample(p, ready=False, left_first_button_pressed=True,
                                left_second_button_pressed=True)
                elif event == 'invalid': self.sample(p, right_trigger=float('nan'))
                else: self.now += 1.; p.authorized(self.now, False)
                self.sample(p); self.sample(p)
                self.assertFalse(p.enabled, (source, event))

    def test_initial_wait_without_packets_preserves_default(self):
        for source in ('quest', 'pico'):
            p = TorsoControls(source)
            for _ in range(5):
                self.now += 1.
                self.assertFalse(p.authorized(self.now, False))
                self.assertTrue(p.startup_enable_pending)
            self.sample(p); self.assertFalse(p.enabled)
            self.sample(p); self.assertTrue(p.enabled)

    def test_ready_loss_before_first_update_cancels_default(self):
        for source in ('quest', 'pico'):
            p = TorsoControls(source)
            self.sample(p, ready=False)
            self.assertFalse(p.authorized(self.now, True))
            self.assertFalse(p.authorized(self.now, False))
            self.sample(p); self.sample(p)
            self.assertFalse(p.enabled)
            self.assertFalse(p.startup_enable_pending)

    def test_first_ready_chord_turns_default_off_without_reenable(self):
        for source in ('quest', 'pico'):
            p = TorsoControls(source)
            self.sample(p, ready=False)
            self.sample(p, **self.chord(p)); self.assertFalse(p.enabled)
            self.sample(p); self.sample(p); self.assertFalse(p.enabled)

    def test_chord_held_before_ready_is_not_a_new_toggle(self):
        for source in ('quest', 'pico'):
            p = TorsoControls(source)
            self.sample(p, ready=False, **self.chord(p))
            self.sample(p, **self.chord(p)); self.assertTrue(p.enabled)
            self.sample(p); self.sample(p, **self.chord(p))
            self.assertFalse(p.enabled)

if __name__=='__main__':unittest.main()
