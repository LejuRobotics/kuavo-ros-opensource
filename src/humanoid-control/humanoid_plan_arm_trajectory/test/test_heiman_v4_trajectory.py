#!/usr/bin/env python3
"""Exercise V4 wheel joint extraction and service routing without ROS nodes."""
import importlib.util
import os
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

SCRIPT = Path(__file__).resolve().parents[1] / 'script' / 'arm_trajectory_bezier_process.py'
spec = importlib.util.spec_from_file_location('heiman_v4_bezier', SCRIPT)
bezier = importlib.util.module_from_spec(spec)
try:
    spec.loader.exec_module(bezier)
except ImportError as error:
    raise unittest.SkipTest('Source the built ROS workspace: ' + str(error))


class InitializationReachedControllerLookup(Exception):
    pass


class V4TrajectoryTest(unittest.TestCase):
    def test_wheel_actions_remain_compatible(self):
        for version in (410062, 410063):
            for action_version in (62, 63, 400062, 400063, 410062, 410063):
                self.assertTrue(bezier.is_tact_robot_type_compatible(action_version, version))
            for action_version in (14, 41, 45, 52, 55):
                self.assertFalse(bezier.is_tact_robot_type_compatible(action_version, version))

    def test_launch_version_selects_wheel_layout_and_service(self):
        for version in (400062, 400063, 410062, 410063):
            with self.subTest(version=version):
                demo = bezier.ArmTrajectoryBezierDemo.__new__(bezier.ArmTrajectoryBezierDemo)
                # Stop before node/service initialization; the real constructor sets
                # the version flags and message state before controller discovery.
                with patch.dict(os.environ, ROBOT_VERSION='45'), \
                     patch.object(bezier.rospy, 'get_param', return_value=version), \
                     patch.object(bezier.ArmTrajectoryBezierDemo, 'get_current_control_mode',
                                  side_effect=InitializationReachedControllerLookup):
                    with self.assertRaises(InitializationReachedControllerLookup):
                        demo.__init__()
                self.assertEqual(demo.robot_version.version_number(), version)
                self.assertTrue(demo.has_waist)
                self.assertEqual(demo._get_extractor_key(), 'KUAVO_WHEELED')
                q = list(range(20))
                self.assertEqual(demo._extract_kuavo_wheeled(q, [50]*12),
                                 list(range(4, 18)) + [50]*12 + [18, 19, 3])
                with patch.object(bezier.rospy, 'wait_for_service') as wait, \
                     patch.object(bezier.rospy, 'ServiceProxy') as proxy:
                    proxy.return_value.return_value = SimpleNamespace(result=True)
                    self.assertTrue(demo.call_change_arm_ctrl_mode_service(2))
                    wait.assert_called_once_with('/wheel_arm_change_arm_ctrl_mode', timeout=0.5)
                    proxy.assert_called_once_with('/wheel_arm_change_arm_ctrl_mode', bezier.changeArmCtrlMode)


if __name__ == '__main__':
    unittest.main()
