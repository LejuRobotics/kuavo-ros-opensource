#!/usr/bin/env python3
"""Exercise production hand-source admission and owner acquisition without ROS."""
import unittest
import fixed_follow_operation_test as fixed


class OpticalFollowEntryTest(unittest.TestCase):
    def setUp(self):
        self.f = fixed.FixedFollowTest(); self.f.setUp()
        self.n = n = self.f.n
        n.tracking_source_mode = fixed.ns['TrackingSourceMode'].AUTO
        n.tracking_source_mode_name = 'auto'
        n._configured_tracking_source = lambda: 0
        n._tracking_session_source = n._tracking_last_observed_source = 1
        n._tracking_matching_source_streak = n._tracking_rejected_source_count = 0
        n._tracking_source_interrupted = False
        n._absolute_state = fixed.State.PREPARED
        n._fixed_operation_block = True
        n._controller_start_consumed = True
        n._joystick_receive_monotonic = None
        n.quest3_arm_info_transformer.is_runing = True  # old controller latch
        n.quest3_arm_info_transformer.is_runing_gesture = lambda: True
        fixed.ns['rospy'].logwarn_throttle = lambda *a: None

    def frame(self, accepted=True, source=2):
        n = self.n; fixed.clock.now += .01
        n._absolute_latest_measured_q_time = fixed.clock.now
        n._maybe_start_optical_follow(source, accepted)
        policy = n._tracking_source_policy(source, n.quest3_arm_info_transformer.is_runing)
        if accepted and policy['source_allowed']:
            n._handle_absolute_target_and_gesture(
                {'source': source, 'receive_time': fixed.clock.now},
                n.quest3_arm_info_transformer.is_runing)

    def test_prepare_to_optical_requires_fifty_fresh_ok_frames_and_new_owner(self):
        n = self.n; old_generation = n._absolute_generation
        for _ in range(49): self.frame()
        self.assertEqual(n._absolute_state, fixed.State.PREPARED)
        self.assertIsNone(n._fixed_pending_operation)
        self.frame()
        self.assertGreater(n._absolute_generation, old_generation)
        self.assertEqual(n._fixed_pending_operation['phase'], 'dependencies')
        self.assertEqual(n._absolute_latest_target['source'], 2)
        n._advance_fixed_request()
        self.assertEqual(n._absolute_state, fixed.State.CAPTURED)
        self.assertEqual(n._absolute_pending_capture['target']['source'], 2)
        self.assertIsNone(n._joystick_receive_monotonic)
        n._absolute_state = fixed.State.ACTIVE
        n._advance_fixed_request()
        self.assertIsNotNone(n._fixed_pending_operation)  # acceptance is not completion
        self.f.ack(); n._advance_fixed_request()
        self.assertIsNone(n._fixed_pending_operation)
        self.assertEqual(n._fixed_operation_phase, 4)
        self.assertTrue(n._controller_start_consumed)

    def test_rejected_or_non_ok_or_gapped_frames_restart_count(self):
        for event in ('invalid', 'neutral', 'gap', 'controller'):
            self.setUp()
            for _ in range(30): self.frame()
            if event == 'invalid': self.frame(False)
            elif event == 'neutral':
                self.n.quest3_arm_info_transformer.is_runing_gesture = lambda: False
                self.frame()
                self.n.quest3_arm_info_transformer.is_runing_gesture = lambda: True
            elif event == 'gap': fixed.clock.now += 1.
            else: self.frame(source=1)
            for _ in range(20): self.frame()
            self.assertIsNone(self.n._fixed_pending_operation, event)

    def test_active_fixed_hold_reset_in_progress_and_fault_cannot_handoff(self):
        changes = [lambda n: setattr(n, '_absolute_state', fixed.State.ACTIVE),
                   lambda n: setattr(n, '_absolute_state', fixed.State.RETURNING_TO_PREP),
                   lambda n: setattr(n, '_absolute_fault_hold_pending', True),
                   lambda n: setattr(n, '_tracking_source_hold_latched', True),
                   lambda n: setattr(n, '_absolute_stop_pending', True),
                   lambda n: setattr(n, '_fixed_operation_error', True),
                   lambda n: setattr(n, 'tracking_source_mode', fixed.ns['TrackingSourceMode'].CONTROLLER)]
        for change in changes:
            self.setUp(); change(self.n)
            for _ in range(60): self.frame()
            self.assertIsNone(self.n._fixed_pending_operation)

    def test_hand_follow_still_requires_fresh_joints_and_pose(self):
        for _ in range(50): self.frame()
        n = self.n; n._absolute_latest_measured_q_time = 0.
        n._advance_fixed_request()
        self.assertEqual(n._fixed_pending_operation['phase'], 'dependencies')
        fixed.clock.now += .6
        n._advance_fixed_request()
        self.assertEqual(n._fixed_operation_phase, 5)
        self.assertIsNone(n._fixed_pending_operation)

    def test_auto_locks_active_source_but_allows_controller_recapture_after_prepare(self):
        n = self.n; n._tracking_session_source = 2
        n._absolute_state = fixed.State.ACTIVE
        self.assertFalse(n._tracking_source_policy(1, True)['source_allowed'])
        n._absolute_state = fixed.State.PREPARED
        self.assertTrue(n._tracking_source_policy(1, True)['source_allowed'])
        n._absolute_latest_target = {'source': 1, 'receive_time': fixed.clock.now}
        n._joystick_receive_monotonic = fixed.clock.now
        self.assertTrue(n._fixed_arm_operation(2).result)
        n._advance_fixed_request()
        self.assertEqual(n._absolute_pending_capture['target']['source'], 1)
        self.assertEqual(n._tracking_session_source, 1)


if __name__ == '__main__':
    unittest.main()
