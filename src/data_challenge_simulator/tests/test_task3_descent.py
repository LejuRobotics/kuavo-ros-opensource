"""Unit tests for Task 3's one-shot pre-descent error assessment."""

import unittest

from utils.task3_descent import Task3DescentPlanner


class Task3DescentAssessmentTest(unittest.TestCase):

    def test_assessment_computes_xy_and_highest_tip_z_correction(self):
        assessment = Task3DescentPlanner.assessment_from_tips(
            tips=(
                (0.650, -0.240, 0.725),
                (0.640, -0.220, 0.718),
                (0.630, -0.230, 0.710),
            ),
            ring_position=(0.620, -0.230, 0.650),
            ring_height=0.060,
        )

        expected_xy_errors = (
            (-0.030, 0.010), (-0.020, -0.010), (-0.010, 0.0))
        for actual, expected in zip(
                assessment.fingertip_xy_errors_m, expected_xy_errors):
            for actual_axis, expected_axis in zip(actual, expected):
                self.assertAlmostEqual(actual_axis, expected_axis)
        self.assertAlmostEqual(
            assessment.center_xy_correction_m[0], -0.020)
        self.assertAlmostEqual(
            assessment.center_xy_correction_m[1], 0.0)
        self.assertAlmostEqual(
            assessment.vertical_correction_m, -0.016)

    def test_assessment_rejects_non_three_tip_input(self):
        with self.assertRaisesRegex(ValueError, "three finite XYZ"):
            Task3DescentPlanner.assessment_from_tips(
                tips=((0.0, 0.0, 0.0),),
                ring_position=(0.0, 0.0, 0.0),
                ring_height=0.060,
            )


if __name__ == "__main__":
    unittest.main()
