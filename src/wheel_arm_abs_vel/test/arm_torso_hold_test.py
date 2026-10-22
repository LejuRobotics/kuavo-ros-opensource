#!/usr/bin/env python3
"""Production Mapper/gesture tests for an arm-only pause in torso mode."""
from types import SimpleNamespace as NS
import unittest
import fixed_follow_operation_test as fixed
from torso_controls_test import TorsoControls


class ArmTorsoHoldTest(unittest.TestCase):
    def setUp(self):
        self.fixture = fixed.FixedFollowTest()
        self.fixture.setUp()
        self.n = n = self.fixture.n
        n._absolute_state = fixed.State.ACTIVE
        n._fixed_arm_hold = False
        n._fixed_arm_operation_seq = 0
        n._absolute_gesture_running = True
        n.quest3_arm_info_transformer.is_runing = True
        n._torso_controls = TorsoControls('quest')
        n._torso_controls.enabled = True
        n._torso_controls.last_time = fixed.clock.now
        self.hold_calls = []
        n._request_absolute_hold_mode = lambda: self.hold_calls.append(True) or True
        self.fixture.ack()

    def pause(self):
        result = self.n._fixed_arm_operation(0)
        self.assertTrue(result.result, result.message)
        self.assertEqual(result.mode, 0)
        self.assertTrue(self.n._fixed_arm_hold)

    def test_pause_keeps_active_session_owner_and_torso(self):
        n = self.n
        identity = (n._absolute_session_id, n._absolute_generation)
        self.pause()
        self.assertEqual(n._absolute_state, fixed.State.ACTIVE)
        self.assertTrue(n._torso_controls.enabled)
        self.assertEqual((n._absolute_session_id, n._absolute_generation), identity)
        self.assertIsNone(n._fixed_pending_operation)
        self.assertFalse(self.hold_calls)
        self.assertTrue(n._fixed_consumer_ack_locked(fixed.clock.now))

    def test_resume_retains_mapping_session_and_publishes_new_operation(self):
        n = self.n
        self.pause()
        msg = NS()
        n._fill_subsystem_permissions_locked(msg, fixed.clock.now)
        self.assertTrue(msg.arm_hold and msg.torso_control_enabled)
        self.assertEqual(msg.arm_operation_seq, 1)
        result = n._fixed_arm_operation(2)
        self.assertTrue(result.result)
        self.assertEqual(result.mode, 2)
        self.assertFalse(n._fixed_arm_hold)
        self.assertTrue(n._torso_controls.enabled)
        self.assertEqual(n._absolute_generation, 1)
        self.assertIsNone(n._fixed_pending_operation)
        n._fill_subsystem_permissions_locked(msg, fixed.clock.now)
        self.assertFalse(msg.arm_hold)
        self.assertTrue(msg.torso_control_enabled)
        self.assertEqual(msg.arm_operation_seq, 2)

    def test_duplicate_hold_does_not_toggle_or_extend_operation(self):
        self.pause()
        self.pause()
        self.assertEqual(self.n._fixed_arm_operation_seq, 1)
        self.assertTrue(self.n._torso_controls.enabled)

    def test_torso_can_be_disabled_while_arms_stay_held(self):
        self.pause()
        self.n._torso_controls.reset()
        msg = NS()
        self.n._fill_subsystem_permissions_locked(msg, fixed.clock.now)
        self.assertTrue(msg.arm_hold)
        self.assertFalse(msg.torso_control_enabled)
        self.assertTrue(self.n._fixed_arm_operation(2).result)
        self.assertFalse(self.n._torso_controls.enabled)

    def test_xa_from_arm_hold_enters_prepare_and_revokes_torso(self):
        self.pause()
        self.assertTrue(self.n._fixed_arm_operation(1).result)
        self.assertFalse(self.n._fixed_arm_hold)
        self.assertFalse(self.n._torso_controls.enabled)
        self.assertEqual(self.n._absolute_state, fixed.State.RETURNING_TO_PREP)
        self.assertEqual(self.n._fixed_operation_mode(), 1)

    def test_resume_rejects_stale_fault_disabled_and_lost_owner(self):
        changes = [lambda n: setattr(n, '_fixed_owner_status', None),
                   lambda n: setattr(n, '_absolute_fault_hold_pending', True),
                   lambda n: setattr(n, '_tracking_source_hold_latched', True),
                   lambda n: setattr(n, '_fixed_control_enabled', False),
                   lambda n: setattr(n, '_absolute_stop_pending', True),
                   lambda n: setattr(n, '_absolute_mode_status', (0, 0, False)),
                   lambda n: setattr(n, '_joystick_receive_monotonic', 9.),
                   lambda n: setattr(n, '_absolute_latest_measured_q_time', 9.),
                   lambda n: n._absolute_latest_target.update(receive_time=9.),
                   lambda n: setattr(n.quest3_arm_info_transformer, 'is_stop_gesture', lambda: True)]
        for change in changes:
            self.setUp()
            self.pause()
            change(self.n)
            self.assertFalse(self.n._fixed_arm_operation(2).result)
            self.assertTrue(self.n._fixed_arm_hold)
            self.assertEqual(self.n._fixed_arm_operation_seq, 1)

    def test_unhealthy_pause_uses_existing_whole_session_hold(self):
        self.n._fixed_owner_status = None
        self.assertTrue(self.n._fixed_arm_operation(0).result)
        self.assertFalse(self.n._fixed_arm_hold)
        self.assertFalse(self.n._torso_controls.enabled)
        self.assertEqual(len(self.hold_calls), 1)

    def test_without_torso_xb_keeps_session_and_allows_later_torso_enable(self):
        self.n._torso_controls.enabled = False
        self.pause()
        self.assertEqual(self.n._absolute_state, fixed.State.ACTIVE)
        self.assertFalse(self.hold_calls)
        self.n._torso_controls.enabled = True
        msg = NS()
        self.n._fill_subsystem_permissions_locked(msg, fixed.clock.now)
        self.assertTrue(msg.arm_hold and msg.torso_control_enabled)

    def test_session_generation_change_clears_subsystem_pause(self):
        self.pause()
        fixed.Mapper._bump_absolute_generation_locked(self.n)
        self.assertFalse(self.n._fixed_arm_hold)
        self.assertEqual(self.n._absolute_generation, 2)


if __name__ == '__main__':
    unittest.main()
