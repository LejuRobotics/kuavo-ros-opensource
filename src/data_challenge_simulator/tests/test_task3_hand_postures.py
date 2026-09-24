"""Unit tests for Task 3 independent, force-free finger commands."""

import unittest

from utils.hand_postures import (
    JOINT_SUFFIXES,
    advance_internal_expansions,
    internal_independent_expansion_pose,
)


class Task3HandPostureTest(unittest.TestCase):

    def test_default_mode_changes_only_four_authorized_joints(self):
        compact = internal_independent_expansion_pose(0, 0, 0, "r")
        expanded = internal_independent_expansion_pose(0.2, 0.3, 0.4, "r")
        nonzero = {
            name[2:] for name, value in compact.items()
            if value != 0.0
        }
        changed = {
            name[2:] for name in compact
            if compact[name] != expanded[name]
        }
        self.assertEqual(
            nonzero, {"index_j2", "middle_j1", "little_j1", "little_j2"})
        self.assertEqual(changed, {"index_j2", "middle_j1", "little_j2"})
        self.assertEqual(compact["r_little_j3"], 0.0)
        self.assertEqual(expanded["r_little_j3"], 0.0)
        self.assertEqual(len(compact), len(JOINT_SUFFIXES))

    def test_optional_fifth_joint_is_explicit(self):
        compact = internal_independent_expansion_pose(
            0, 0, 0, "r", little_j3_compact=-0.25,
            little_j3_expanded=0.15)
        expanded = internal_independent_expansion_pose(
            0, 0, 0.5, "r", little_j3_compact=-0.25,
            little_j3_expanded=0.15)
        self.assertAlmostEqual(compact["r_little_j3"], -0.25)
        self.assertAlmostEqual(expanded["r_little_j3"], -0.05)

    def test_latched_finger_stays_fixed_while_others_advance(self):
        current = (0.10, 0.20, 0.30)
        following = advance_internal_expansions(
            current, latch_mask=0b010, step=0.05, limits=(0.5, 0.5, 0.32))
        for actual, expected in zip(following, (0.15, 0.20, 0.32)):
            self.assertAlmostEqual(actual, expected)

    def test_all_latched_means_no_command_progress(self):
        current = (0.10, 0.20, 0.30)
        self.assertEqual(
            advance_internal_expansions(current, 0b111, 0.05), current)


if __name__ == "__main__":
    unittest.main()
