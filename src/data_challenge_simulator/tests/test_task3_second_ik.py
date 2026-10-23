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
        self.assertTrue(all(depth < planner.ring_height
                            for depth in result.fingertip_depths_m))
        self.assertTrue(all(radius < planner.inner_radius
                            for radius in result.fingertip_radii_m))
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


def test_live_descent_remains_reachable_at_recorded_parking_error():
    """Regression: collect layout 457576801 failed at the first insertion."""
    import json
    from scipy.spatial.transform import Rotation
    planner = Task3SecondIKPlanner(SCENE)
    ring = np.asarray((0.604299, -0.155438, 0.649998))
    measured = (0.0,) * 7 + (
        -0.637447, -0.364747, 0.999605, -0.698163,
        0.697585, -0.555188, 0.276539)
    config = json.loads((SCENE.parents[3] / "config/task3_grasp.json").read_text())
    desired_position, desired_rotation = planner.reference_pose(ring)
    target, rotation = planner.feedforward_command_pose(ring)
    target += np.asarray((-0.00023, 0.00027, 0.00036))
    target[2] -= config["descent_distance_m"]
    rotation = desired_rotation * Rotation.from_quat(
        (0.49759645, 0.49969572, 0.50109182, 0.50160637)).inv() * rotation
    result = planner.solve_target(
        (-0.059, 0.138, 0.0), ring, measured, target, rotation,
        validate_clearance=False)
    assert result is not None
    assert all(0 < depth < planner.ring_height for depth in result.fingertip_depths_m)
