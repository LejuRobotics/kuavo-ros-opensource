#!/usr/bin/env python3
"""Exercise production target geometry without ROS or robot motion."""
import ast
import math
import unittest
from pathlib import Path
from types import SimpleNamespace

source = Path(__file__).resolve().parents[1] / 'scripts/head_control_manager.py'
tree = ast.parse(source.read_text())
cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'HeadControlManager')
method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'calculate_head_pose_from_hand')
namespace = {'math': math}
exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), 'exec'), namespace)
calculate = namespace['calculate_head_pose_from_hand']


class RearTargetTest(unittest.TestCase):
    def owner(self, enabled=True):
        return SimpleNamespace(hold_rear_targets=enabled, rear_target_held=False,
                               min_track_distance_xy=.08, target_yaw=12., target_pitch=5.,
                               yaw_limit=[-85, 85], pitch_limit=[-25, 25])

    def test_bag_rear_midpoint_never_flips_yaw(self):
        owner = self.owner()
        for y in [-.00012, .00256, -.00411, .00609]*10:
            self.assertEqual(calculate(owner, [-.105, y, -.90], [0, 0, 0]), (None, None))

    def test_front_target_tracks_normally(self):
        yaw, pitch = calculate(self.owner(), [.4, .1, -.2], [0, 0, 0])
        self.assertAlmostEqual(yaw, math.degrees(math.atan2(.1, .4)))
        self.assertLessEqual(pitch, 25.)

    def test_rear_front_hysteresis(self):
        owner = self.owner()
        calculate(owner, [-.1, .2, 0], [0, 0, 0])
        for x in [.001, -.001, .029]:
            self.assertEqual(calculate(owner, [x, .2, 0], [0, 0, 0]), (None, None))
        self.assertNotEqual(calculate(owner, [.03, .2, 0], [0, 0, 0]), (None, None))
        self.assertNotEqual(calculate(owner, [.01, .2, 0], [0, 0, 0]), (None, None))
        self.assertEqual(calculate(owner, [0., .2, 0], [0, 0, 0]), (None, None))

    def test_legacy_monitor_behavior_preserved(self):
        self.assertEqual(calculate(self.owner(False), [-.1, .001, -.9], [0, 0, 0]), (85, 25))


if __name__ == '__main__':
    unittest.main()
