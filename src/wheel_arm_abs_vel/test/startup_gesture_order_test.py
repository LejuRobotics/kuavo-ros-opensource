#!/usr/bin/env python3
"""Production entrypoints: initial controller chord, acknowledged follow, restart."""
import unittest
from types import SimpleNamespace as NS
import fixed_follow_operation_test as fixed

class StartupGestureOrderTest(unittest.TestCase):
    def setUp(self):
        self.f=fixed.FixedFollowTest(); self.f.setUp(); self.n=self.f.n
        self.n._controller_start_acquired=False
        self.n._controller_start_completed=False
    def triggers(self, left=.8, right=.8):
        self.n._controller_start_request(NS(left_trigger=left,right_trigger=right))
    def service(self, mode):
        return self.n.fixed_arm_operation_callback(NS(control_mode=mode))
    def test_first_xa_cannot_prepare_or_follow(self):
        for mode in (1,2):
            before=(self.n._absolute_state,self.n._absolute_generation)
            self.assertFalse(self.service(mode).result)
            self.assertEqual(before,(self.n._absolute_state,self.n._absolute_generation))
            self.assertIsNone(self.n._fixed_pending_operation)
    def test_first_chord_needs_both_front_triggers(self):
        self.triggers(.8,0.); self.assertIsNone(self.n._fixed_pending_operation)
        self.triggers(0.,.8); self.assertIsNone(self.n._fixed_pending_operation)
        self.triggers(); self.assertEqual(self.n._fixed_pending_operation['mode'],2)
        self.assertFalse(self.n._controller_start_completed)
        self.assertFalse(self.service(1).result)
    def test_failed_first_attempt_does_not_unlock_xa(self):
        self.n._absolute_latest_measured_q=None
        self.triggers(); fixed.clock.now+=13
        self.n._advance_fixed_request()
        self.assertFalse(self.n._controller_start_completed)
        self.assertFalse(self.service(2).result)
        self.f.fresh(); self.triggers(0.,0.); self.triggers()
        self.assertIsNotNone(self.n._fixed_pending_operation)
    def test_acknowledged_follow_permanently_switches_to_service_restart(self):
        self.triggers(); self.n._advance_fixed_request()
        self.n._absolute_state=fixed.State.ACTIVE
        self.n._advance_fixed_request()
        self.assertFalse(self.n._controller_start_completed)  # no execution ack yet
        self.f.ack(); self.n._advance_fixed_request()
        self.assertTrue(self.n._controller_start_acquired)
        self.n._advance_fixed_request()  # The next supervisor tick observes transition completion.
        self.assertTrue(self.n._controller_start_completed)
        for state in (fixed.State.WAITING,fixed.State.WAIT_RELEASE,fixed.State.PREPARED):
            self.n._absolute_state=state; self.n._fixed_pending_operation=None
            self.triggers(0.,0.); self.triggers()
            self.assertIsNone(self.n._fixed_pending_operation)
        self.f.fresh()
        self.assertTrue(self.service(2).result)
    def test_source_selection_does_not_bypass_first_service_gate(self):
        self.n.tracking_source_mode=fixed.ns['TrackingSourceMode'].AUTO
        self.n._configured_tracking_source=lambda:0
        self.n._tracking_last_accepted_source=2
        self.assertFalse(self.service(2).result)

    def test_core_operation_rejects_unmarked_first_start(self):
        for state in (fixed.State.WAITING, fixed.State.WAIT_RELEASE, fixed.State.PREPARED,
                      fixed.State.RETURNING_TO_PREP, fixed.State.PAUSED):
            for mode in (1, 2):
                self.n._absolute_state=state
                self.assertFalse(self.n._fixed_arm_operation(mode).result)
                self.assertIsNone(self.n._fixed_pending_operation)

    def test_legacy_mode_topic_cannot_start_or_resume(self):
        fixed.ns['rospy'].logwarn_throttle=lambda *a:None
        for completed in (False, True):
            for state in (fixed.State.WAITING, fixed.State.WAIT_RELEASE, fixed.State.PREPARED,
                          fixed.State.RETURNING_TO_PREP, fixed.State.PAUSED):
                for mode in (1, 2):
                    self.n._controller_start_completed=completed; self.n._absolute_state=state
                    before=(self.n._absolute_generation,self.n._absolute_session_id)
                    self.n.arm_mode_callback(NS(data=mode))
                    self.assertEqual(self.n._absolute_state,state)
                    self.assertEqual(before,(self.n._absolute_generation,self.n._absolute_session_id))
                    self.assertIsNone(self.n._fixed_pending_operation)

    def test_raw_running_latch_cannot_start_any_source(self):
        for completed in (False, True):
            for source in (1, 2):
                for method in ('_handle_velocity_gesture_edge', '_handle_absolute_target_and_gesture'):
                    self.setUp(); n=self.n
                    n._controller_start_completed=completed
                    n.tracking_source_mode=fixed.ns['TrackingSourceMode'].AUTO
                    n._tracking_last_accepted_source=source
                    if method=='_handle_velocity_gesture_edge':
                        n._handle_velocity_gesture_edge(True)
                    else:
                        n._handle_absolute_target_and_gesture(None, True)
                    self.assertIsNone(n._fixed_pending_operation)
                    self.assertFalse(n._absolute_gesture_running)

    def test_enable_status_and_owner_feedback_do_not_create_intent(self):
        self.n._fixed_enable_callback(NS(data=True))
        self.f.ack(); self.n._advance_fixed_request()
        self.assertIsNone(self.n._fixed_pending_operation)
        self.assertEqual(self.n._absolute_state, fixed.State.WAITING)
        self.assertFalse(self.n._controller_start_completed)

    def test_xb_cannot_create_a_new_session(self):
        for completed in (False, True):
            for state in (fixed.State.WAITING, fixed.State.WAIT_RELEASE, fixed.State.PREPARED):
                self.n._controller_start_completed=completed; self.n._absolute_state=state
                for mode in (0, 2):
                    self.assertFalse(self.n.fixed_arm_operation_callback(NS(control_mode=mode), arm_hold_only=True).result)
                    self.assertIsNone(self.n._fixed_pending_operation)

    def test_controller_triggers_do_not_admit_hand_source(self):
        self.n.tracking_source_mode=fixed.ns['TrackingSourceMode'].AUTO
        self.n._tracking_last_accepted_source=2
        self.triggers()
        self.assertIsNone(self.n._fixed_pending_operation)

    def test_pending_first_trigger_cannot_turn_into_optical_start(self):
        self.n.tracking_source_mode=fixed.ns['TrackingSourceMode'].AUTO
        self.n._configured_tracking_source=lambda:0
        self.triggers(); self.assertIsNotNone(self.n._fixed_pending_operation)
        self.n._tracking_last_accepted_source=2
        self.n._advance_fixed_request()
        self.assertIsNone(self.n._fixed_pending_operation)
        self.assertFalse(self.n._controller_start_completed)

    def test_xb_preserves_existing_arm_hold_without_new_generation(self):
        self.n._controller_start_acquired=self.n._controller_start_completed=True
        self.n._absolute_state=fixed.State.ACTIVE
        self.f.ack()
        generation=self.n._absolute_generation
        self.assertTrue(self.n.fixed_arm_operation_callback(NS(control_mode=0), arm_hold_only=True).result)
        self.assertTrue(self.n._fixed_arm_hold)
        self.assertTrue(self.n.fixed_arm_operation_callback(NS(control_mode=2), arm_hold_only=True).result)
        self.assertFalse(self.n._fixed_arm_hold)
        self.assertEqual(self.n._absolute_generation,generation)
        self.assertIsNone(self.n._fixed_pending_operation)

if __name__=='__main__':unittest.main()
