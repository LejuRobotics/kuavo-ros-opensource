"""Exercise unchanged normal input and the two discontinuity cases without ROS."""
import os
from pathlib import Path
import sys
import unittest

import numpy as np

os.environ.setdefault("KUAVO_MAPPER_NUMERIC_BACKEND", "python")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts" / "tools"))
from pose_velocity_kalman import exp_so3
from quest_skeleton_jump import QuestSkeletonJumpCorrection


def skeleton():
    result = {i: np.eye(4) for i in (*QuestSkeletonJumpCorrection.indices, 22, 23)}
    result[23][:3, 3] = (0., 0., 1.)
    for i, xyz in {
        0: (0., .20, 1.1), 1: (.20, .20, .98), 4: (.41, .20, .96),
        2: (0., -.20, 1.1), 3: (.20, -.20, .98), 5: (.41, -.20, .96),
    }.items():
        result[i][:3, 3] = xyz
    return result


class QuestSkeletonJumpTest(unittest.TestCase):
    def setUp(self):
        self.guard = QuestSkeletonJumpCorrection()
        self.initial = skeleton()
        self.assertIs(self.guard.update(self.initial, 0., 1), self.initial)

    def test_normal_pose_and_reversal_pass_on_the_current_stamp(self):
        for stamp, dx in ((.008, .010), (.016, .020), (.024, .010), (.032, 0.)):
            frame = skeleton()
            frame[5][0, 3] += dx
            frame[5][:3, :3] = exp_so3(np.array([dx*6., 0., 0.]))
            self.assertIs(self.guard.update(frame, stamp, 1), frame)

    def test_local_jump_corrects_one_arm_and_rejoins(self):
        frame = skeleton()
        for i in (3, 5):
            frame[i][0, 3] += .067
        amended = self.guard.update(frame, .008, 1)
        self.assertTrue(self.guard.correcting[1])
        self.assertFalse(self.guard.correcting[0])
        self.assertFalse(self.guard.rebased)
        self.assertIs(amended[4], frame[4])
        self.assertIs(amended[23], frame[23])
        for i in (3, 5):
            self.assertAlmostEqual(amended[i][0, 3] - self.initial[i][0, 3], .0064)
        self.assertEqual(frame[5][0, 3], self.initial[5][0, 3] + .067)
        self.assertIs(self.guard.update(frame, .008, 1), amended)
        for tick in range(2, 13):
            output = self.guard.update(frame, tick*.008, 1)
        self.assertFalse(self.guard.correcting.any())
        self.assertIs(output, frame)

    def test_shared_frame_change_is_removed_at_the_source_stamp(self):
        jump = np.eye(4)
        jump[:3, :3] = exp_so3(np.array([0., 0., .69]))
        jump[:3, 3] = (.92, .36, 0.)
        frame = {i: jump @ pose for i, pose in skeleton().items()}
        corrected = self.guard.update(frame, .008, 1)
        self.assertTrue(self.guard.rebased)
        self.assertFalse(self.guard.correcting.any())
        for i, pose in corrected.items():
            np.testing.assert_allclose(pose, self.initial[i], atol=1.e-12)
        advanced = skeleton()
        advanced[4][0, 3] += .005
        corrected = self.guard.update({i: jump @ pose for i, pose in advanced.items()}, .016, 1)
        self.assertFalse(self.guard.rebased)
        np.testing.assert_allclose(corrected[4], advanced[4], atol=1.e-12)

    def test_different_session_restarts_without_old_correction(self):
        frame = skeleton()
        frame[5][0, 3] += .067
        self.guard.update(frame, .008, 1)
        self.assertTrue(self.guard.correcting[1])
        self.assertIs(self.guard.update(frame, .016, 2), frame)
        self.assertFalse(self.guard.correcting.any())


if __name__ == "__main__":
    unittest.main()
