#!/usr/bin/env python3

import os
import sys
import unittest

import numpy as np


SCRIPT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scripts"))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from tools.absolute_ik_safety import AbsoluteIkExecutionSafety  # noqa: E402


class AbsoluteIkExecutionSafetyTest(unittest.TestCase):
    def make_safety(self, recovery_successes=3):
        return AbsoluteIkExecutionSafety(
            dof=14,
            recovery_successes=recovery_successes,
            target_max_step=0.12,
            recovery_max_offset=0.12,
            recovery_stability_step=0.03,
            max_velocity=1.5,
            max_acceleration=6.0,
            joint_lower=np.full(14, -1.0),
            joint_upper=np.full(14, 1.0),
            min_joint_limit_margin=0.03,
        )

    def test_repeated_bounded_candidates_cannot_move_a_held_command(self):
        safety = self.make_safety()
        zero = np.zeros(14)
        safety.reset(zero)
        safety.enter_hold(zero)

        for offset in (0.10, 0.20, 0.30, 0.40):
            decision = safety.observe_hard_candidate(
                np.full(14, offset), zero
            )
            self.assertFalse(decision.accepted)
            np.testing.assert_allclose(safety.command, zero)

        self.assertTrue(safety.hold_active)
        self.assertEqual(safety.recovery_success_count, 0)

    def test_recorded_incident_candidate_is_outside_recovery_envelope(self):
        previous = np.array([
            -0.192, 0.360, -0.087, -1.550, 0.503, -0.112, 0.206,
            -0.181, -0.378, 0.204, -1.632, -0.196, -0.046, 0.165,
        ])
        recorded_soft_candidate = np.array([
            -0.592, 0.760, 0.313, -1.950, 0.103, 0.288, -0.194,
            -0.581, 0.022, -0.196, -2.032, -0.596, -0.446, -0.235,
        ])
        safety = AbsoluteIkExecutionSafety(
            dof=14,
            recovery_successes=3,
            target_max_step=0.12,
            recovery_max_offset=0.12,
            recovery_stability_step=0.03,
            max_velocity=1.5,
            max_acceleration=6.0,
        )
        safety.reset(previous)
        safety.enter_hold(previous)

        decision = safety.observe_hard_candidate(
            recorded_soft_candidate, previous
        )

        self.assertFalse(decision.accepted)
        self.assertEqual(decision.reason, "recovery_offset")
        self.assertAlmostEqual(decision.max_trusted_offset, 0.4, places=6)
        np.testing.assert_allclose(safety.command, previous)

    def test_recovery_requires_consecutive_stable_hard_candidates(self):
        safety = self.make_safety(recovery_successes=3)
        zero = np.zeros(14)
        safety.reset(zero)
        safety.enter_hold(zero)

        decisions = [
            safety.observe_hard_candidate(np.full(14, value), zero)
            for value in (0.050, 0.055, 0.052)
        ]
        self.assertEqual([item.accepted for item in decisions], [False, False, True])
        self.assertEqual(decisions[-1].reason, "recovered")
        self.assertFalse(safety.hold_active)

    def test_hard_target_continuity_is_independent_of_command_lag(self):
        safety = self.make_safety()
        zero = np.zeros(14)
        safety.reset(zero)

        first = safety.observe_hard_candidate(np.full(14, 0.50), zero)
        self.assertTrue(first.accepted)
        self.assertEqual(first.reason, "acquired")
        lagging_command = safety.limit_accepted_candidate(
            np.full(14, 0.50), 0.01
        )
        self.assertLess(np.max(lagging_command), 0.01)

        second = safety.observe_hard_candidate(
            np.full(14, 0.52), lagging_command
        )
        self.assertTrue(second.accepted)
        self.assertEqual(second.reason, "tracking")
        self.assertAlmostEqual(second.max_trusted_offset, 0.02, places=6)

    def test_discontinuous_hard_target_enters_hold(self):
        safety = self.make_safety()
        zero = np.zeros(14)
        safety.reset(zero)
        self.assertTrue(safety.observe_hard_candidate(zero, zero).accepted)

        decision = safety.observe_hard_candidate(
            np.full(14, 0.40), zero
        )

        self.assertFalse(decision.accepted)
        self.assertEqual(decision.reason, "target_discontinuity")
        self.assertTrue(safety.hold_active)
        np.testing.assert_allclose(safety.command, zero)

    def test_joint_limit_margin_enters_hold(self):
        safety = self.make_safety()
        zero = np.zeros(14)
        safety.reset(zero)
        candidate = zero.copy()
        candidate[4] = 0.98

        decision = safety.observe_hard_candidate(candidate, zero)

        self.assertFalse(decision.accepted)
        self.assertEqual(decision.reason, "joint_limit_margin")
        self.assertTrue(safety.hold_active)
        np.testing.assert_allclose(safety.command, zero)

    def test_final_limiter_respects_velocity_and_acceleration(self):
        safety = self.make_safety()
        command = np.zeros(14)
        safety.reset(command)
        target = np.full(14, 0.4)
        previous_velocity = np.zeros(14)

        for _ in range(50):
            previous_command = command.copy()
            command = safety.limit_accepted_candidate(target, 0.01)
            velocity = (command - previous_command) / 0.01
            self.assertLessEqual(np.max(np.abs(velocity)), 1.5 + 1.0e-12)
            acceleration = (velocity - previous_velocity) / 0.01
            self.assertLessEqual(np.max(np.abs(acceleration)), 6.0 + 1.0e-9)
            previous_velocity = velocity

        self.assertLessEqual(np.max(command), 0.4 + 1.0e-12)

    def test_final_limiter_respects_acceleration_after_target_reversal(self):
        safety = self.make_safety()
        command = np.zeros(14)
        safety.reset(command)
        previous_velocity = np.zeros(14)

        for cycle in range(60):
            target = (
                np.full(14, 0.8)
                if cycle < 30
                else np.full(14, -0.8)
            )
            previous_command = command.copy()
            command = safety.limit_accepted_candidate(target, 0.01)
            velocity = (command - previous_command) / 0.01
            acceleration = (velocity - previous_velocity) / 0.01
            self.assertLessEqual(np.max(np.abs(velocity)), 1.5 + 1.0e-12)
            self.assertLessEqual(np.max(np.abs(acceleration)), 6.0 + 1.0e-9)
            previous_velocity = velocity


if __name__ == "__main__":
    unittest.main()
