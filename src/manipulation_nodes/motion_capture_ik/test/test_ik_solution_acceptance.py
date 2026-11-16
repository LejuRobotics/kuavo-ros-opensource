#!/usr/bin/env python3
import os
import sys
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'scripts')))
from ik_ros_uni import IkRos, IkSolutionFrame


def make_node():
    node = IkRos.__new__(IkRos)
    node._ik_condition = threading.Condition()
    node._ik_solution_lock = threading.Lock()
    node._ik_control_generation = 0
    node._ik_target_generation = 1
    node._latest_ik_solution = None
    node._last_published_arm_q = np.zeros(14)
    node.trigger_reset_mode = False
    node.stop_event = threading.Event()
    node._IkRos__as_mc_ik = True
    node._vr_error = False
    node.ik_solution_timeout_s = 0.2
    node._ik_stale_solution_drop_count = 0
    node.enable_vr_latency_diagnostics = False
    return node


def solution(generation=1, age=0.01):
    now = time.perf_counter()
    return IkSolutionFrame(generation, SimpleNamespace(), np.zeros(14),
                           120.0, time.time(), now, now - age)


class IkSolutionAcceptanceTest(unittest.TestCase):
    def test_new_targets_during_solve_do_not_starve_output(self):
        node = make_node()
        for generation in range(1, 101):
            # Input has advanced while this round was solving.
            node._ik_target_generation = generation + 5
            result = solution(generation)
            self.assertTrue(node._accept_ik_solution(result, 0))
            self.assertIs(node._latest_ik_solution, result)
        self.assertEqual(node._ik_stale_solution_drop_count, 0)

    def test_reset_invalidates_inflight_result_even_after_reset_flag_clears(self):
        node = make_node()
        result = solution()
        node._clear_latest_ik_solution(clear_published_command=True)
        node.trigger_reset_mode = False
        self.assertFalse(node._accept_ik_solution(result, 0))
        self.assertIsNone(node._latest_ik_solution)
        self.assertIsNone(node._last_published_arm_q)
        self.assertTrue(node._accept_ik_solution(solution(2), 1))

    def test_pending_reset_stop_and_invalid_vr_reject_results(self):
        for field, value in [('trigger_reset_mode', True), ('_vr_error', True)]:
            node = make_node()
            setattr(node, field, value)
            self.assertFalse(node._accept_ik_solution(solution(), 0))
        node = make_node()
        node.stop_event.set()
        self.assertFalse(node._accept_ik_solution(solution(), 0))

    def test_entering_external_mode_invalidates_inflight_result_once(self):
        node = make_node()
        node._last_arm_mode_for_velocity = 1
        node._arm_velocity_lock = threading.Lock()
        node.only_half_up_body = False
        result = solution()
        node.arm_mode_callback(SimpleNamespace(data=2))
        self.assertFalse(node._accept_ik_solution(result, 0))
        self.assertEqual(node._ik_control_generation, 1)
        node.arm_mode_callback(SimpleNamespace(data=2))
        self.assertEqual(node._ik_control_generation, 1)

    def test_slow_solve_cannot_refresh_age_of_an_old_target(self):
        node = make_node()
        self.assertFalse(node._accept_ik_solution(solution(age=0.25), 0))
        self.assertIsNone(node._latest_ik_solution)

    def test_older_or_duplicate_result_cannot_replace_newer_result(self):
        node = make_node()
        latest = solution(3)
        self.assertTrue(node._accept_ik_solution(latest, 0))
        self.assertFalse(node._accept_ik_solution(solution(2), 0))
        self.assertFalse(node._accept_ik_solution(solution(3), 0))
        self.assertIs(node._latest_ik_solution, latest)

    def test_transient_vr_error_during_solve_invalidates_inflight_result(self):
        """Error then recovery before solve ends must still reject the old result.

        Reproduces the race where the input thread commits an error frame and
        then a recovery frame while IK is still solving the pre-error target.
        Control generation must advance on the error commit itself so the
        in-flight solution cannot sneak through after `_vr_error` clears.
        """
        node = make_node()
        inflight = solution(1)
        with node._ik_condition:
            node._apply_vr_error_on_target_commit_locked(True)
            self.assertTrue(node._vr_error)
            self.assertEqual(node._ik_control_generation, 1)
            # Recovery frame arrives before the pre-error solve finishes.
            node._apply_vr_error_on_target_commit_locked(False)
            self.assertFalse(node._vr_error)
            # Recovery must not roll control generation backwards.
            self.assertEqual(node._ik_control_generation, 1)
        self.assertFalse(node._accept_ik_solution(inflight, 0))
        self.assertIsNone(node._latest_ik_solution)
        # A solve started after the error (control gen 1) is still acceptable.
        self.assertTrue(node._accept_ik_solution(solution(2), 1))

    def test_manual_trajectory_keeps_existing_timeout_exemption(self):
        node = make_node()
        result = IkSolutionFrame(1, None, np.zeros(14), 120.0,
                                 time.time(), time.perf_counter())
        self.assertTrue(node._accept_ik_solution(result, 0))

    def test_publisher_does_not_extend_expired_target_age(self):
        node = make_node()
        node.controller_dt = 0.01
        node._latest_ik_solution = solution(age=0.21)
        node._ik_publish_timeout_count = 0
        node.publish_joint_states = MagicMock()
        rate = MagicMock()
        rate.sleep.side_effect = node.stop_event.set
        with patch('ik_ros_uni.rospy.Rate', return_value=rate), \
                patch('ik_ros_uni.rospy.is_shutdown', return_value=False), \
                patch('ik_ros_uni.rospy.logwarn_throttle'):
            node.ik_fixed_rate_publish_thread()
        node.publish_joint_states.assert_not_called()
        self.assertEqual(node._ik_publish_timeout_count, 1)


if __name__ == '__main__':
    unittest.main()
