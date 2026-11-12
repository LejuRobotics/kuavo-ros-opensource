#!/usr/bin/env python3
"""Parse Quest launch configurations without starting ROS nodes or hardware."""
from pathlib import Path
import unittest
import roslaunch
from roslaunch.substitution_args import resolve_args

PACKAGE = Path(__file__).resolve().parents[1]
ENTRIES = ('launch_quest3_ik.launch', 'launch_dual_quest3_ik.launch',
           'launch_quest3_ik_videostream_orbbec.launch')
HEIMAN = (400062, 400063, 410062, 410063)


class HeimanLaunchTest(unittest.TestCase):
    def load(self, entry, version, incremental=True, extra=()):
        resolved = Path(resolve_args('$(find noitom_hi5_hand_udp_python)/launch/' + entry)).resolve()
        self.assertEqual(resolved, (PACKAGE / 'launch' / entry).resolve())
        args = ['robot_version:=' + str(version),
                'use_cpp_incremental_ik:=' + str(incremental).lower()]
        if entry == 'launch_dual_quest3_ik.launch':
            args += ['left_ip_address:=127.0.0.1', 'right_ip_address:=127.0.0.2']
        return roslaunch.config.load_config_default([(str(resolved), args + list(extra))],
                                                   None, verbose=False)

    def test_heiman_cpp_uses_wheel_ik_and_hand_type(self):
        for entry in ENTRIES:
            for version in HEIMAN:
                with self.subTest(entry=entry, version=version):
                    config = self.load(entry, version)
                    self.assertTrue(config.params['/wheel_ik'].value)
                    self.assertEqual(config.params['/end_effector_type'].value, 'heiman')
                    self.assertEqual(config.params['/control_finger_type'].value, 0)
                    types = [node.type for node in config.nodes]
                    self.assertIn('wheel_ik_ros_uni_cpp_node', types)
                    self.assertNotIn('ik_ros_uni_cpp_node', types)

    def test_python_mapper_receives_heiman_type(self):
        for entry in ENTRIES:
            for version in (410062, 410063):
                with self.subTest(entry=entry, version=version):
                    config = self.load(entry, version, incremental=False)
                    mapper = next(node for node in config.nodes if node.type == 'ik_ros_uni.py')
                    self.assertIn('--ee_type heiman', mapper.args.replace('--ee_type=', '--ee_type '))
                    self.assertEqual(config.params['/end_effector_type'].value, 'heiman')
                    self.assertTrue(config.params['/wheel_ik'].value)

    def test_other_hands_and_explicit_overrides_are_preserved(self):
        for entry in ENTRIES:
            for version, expected in ((210062, 'lejuclaw'), (310062, 'qiangnao')):
                config = self.load(entry, version)
                self.assertTrue(config.params['/wheel_ik'].value)
                self.assertEqual(config.params['/end_effector_type'].value, expected)
            for incremental in (False, True):
                config = self.load(entry, 410062, incremental=incremental, extra=['ee_type:=qiangnao'])
                self.assertEqual(config.params['/end_effector_type'].value, 'qiangnao')

    def test_v4_standard_quest_scales_match_existing_wheel_profile(self):
        for entry in (ENTRIES[0], ENTRIES[2]):
            expected = self.load(entry, 400062)
            for version in (410062, 410063):
                actual = self.load(entry, version)
                for key in ('/vr_cmd_vel/linear_scale_x', '/vr_cmd_vel/linear_scale_y',
                            '/vr_cmd_vel/angular_scale_z'):
                    self.assertEqual(actual.params[key].value, expected.params[key].value)

    def test_arm_only_ik_keeps_global_hand_parameters(self):
        for version in (410062, 410063):
            config = self.load(ENTRIES[0], version, extra=['use_arm_only_ik:=true'])
            self.assertEqual(config.params['/end_effector_type'].value, 'heiman')
            self.assertEqual(config.params['/control_finger_type'].value, 0)
            types = [node.type for node in config.nodes]
            self.assertIn('wheel_arm_only_ik_ros_uni_cpp_node', types)
            self.assertNotIn('wheel_ik_ros_uni_cpp_node', types)

    def test_biped_keeps_original_ik_path(self):
        config = self.load(ENTRIES[0], 45)
        self.assertFalse(config.params['/wheel_ik'].value)
        self.assertEqual(config.params['/end_effector_type'].value, 'qiangnao')
        self.assertIn('ik_ros_uni_cpp_node', [node.type for node in config.nodes])


if __name__ == '__main__':
    unittest.main()
