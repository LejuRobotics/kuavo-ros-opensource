"""Regression tests for Task 3's accepted object-relative second IK."""

from pathlib import Path
import unittest

import numpy as np

from utils.task3_second_ik import Task3SecondIKPlanner


SCENE = Path(__file__).resolve().parents[1] / (
    "models/biped_s400062/xml/"
    "task3.xml")
ABOVE_RIGHT = (
    -0.333097, -0.339038, 0.502965, -1.551866,
    1.119605, 0.118366, 0.468295,
)


class Task3SecondIKTest(unittest.TestCase):

    def test_nominal_target_matches_accepted_pose(self):
        planner = Task3SecondIKPlanner(SCENE)
        result = planner.solve(
            base_pose=(0.0, 0.0, 0.0),
            ring_position=(0.620, -0.230, 0.650),
            measured_arm=(0.0,) * 7 + ABOVE_RIGHT,
        )

        self.assertIsNotNone(result)
        np.testing.assert_allclose(
            result.hand_position_m,
            (0.521805, -0.231697, 0.860822),
            atol=5e-4,
        )
        self.assertTrue(all(depth > 0.0
                            for depth in result.fingertip_depths_m))
        self.assertEqual(len(result.right_waypoints), 2)

    def test_target_translates_with_ring_position(self):
        planner = Task3SecondIKPlanner(SCENE)
        ring = np.asarray((0.625, -0.227, 0.652), dtype=float)
        result = planner.solve(
            base_pose=(0.0, 0.0, 0.0),
            ring_position=ring,
            measured_arm=(0.0,) * 7 + ABOVE_RIGHT,
        )

        self.assertIsNotNone(result)
        np.testing.assert_allclose(
            result.hand_position_m,
            ring + planner.TARGET_HAND_OFFSET_FROM_RING_M,
            atol=5e-4,
        )

    def test_feedforward_command_matches_task1_bias_pattern(self):
        planner = Task3SecondIKPlanner(SCENE)
        ring = np.asarray((0.620, -0.230, 0.650), dtype=float)
        desired_position, desired_rotation = planner.reference_pose(ring)
        command_position, command_rotation = (
            planner.feedforward_command_pose(ring))

        np.testing.assert_allclose(
            command_position,
            desired_position - planner.TRACKING_POSITION_BIAS_WORLD_M,
        )
        self.assertAlmostEqual(
            (desired_rotation.inv() * command_rotation).magnitude(),
            planner.tracking_rotation.magnitude(),
        )

    def test_feedforward_pose_has_a_one_shot_ik_solution(self):
        planner = Task3SecondIKPlanner(SCENE)
        result = planner.solve_feedforward(
            base_pose=(0.0, 0.0, 0.0),
            ring_position=(0.620, -0.230, 0.650),
            measured_arm=(0.0,) * 7 + ABOVE_RIGHT,
        )

        self.assertIsNotNone(result)

if __name__ == "__main__":
    unittest.main()
