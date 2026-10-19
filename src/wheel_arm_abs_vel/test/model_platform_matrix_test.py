#!/usr/bin/env python3
"""Offline model/topology matrix. Does not start ROS or qualify target hardware."""
import json
from pathlib import Path
import re
import sys
import unittest
import xml.etree.ElementTree as ET

import runtime_topology_test as fixtures
partition, BASELINE, ROOT = fixtures.partition, fixtures.BASELINE, fixtures.ROOT
sys.path.insert(0, str(ROOT/'scripts'))
from tools.full_model_contract import full_model_relative_path, require_full_model

ASSETS = ROOT.parent/'kuavo_assets'
PLATFORMS = {
    'aaeon_soft': ('x86_64', '0-15', '2,3,7,14,15', 'soft'),
    'aaeon_hard': ('x86_64', '0-15', '2,3,5,7', 'hard'),
    'aaeon_14_soft': ('x86_64', '0-13', '2,3,7', 'soft'),
    'aaeon_14_hard': ('x86_64', '0-13', '2,3,5,7', 'hard'),
    'agx_soft': ('aarch64', '0-11', '2,7', 'soft'),
    'agx_hard': ('aarch64', '0-11', '2,3,7', 'hard'),
}


class ModelPlatformMatrix(unittest.TestCase):
    setUp = fixtures.Topology.setUp
    fixture = fixtures.Topology.fixture

    def check_combination(self, version, platform):
        machine, online, isolated, mode = PLATFORMS[platform]
        self.fixture(online, isolated)
        profile, _ = partition.select_profile({}, machine, self.root,
            cpu_model='Intel(R) Core(TM) Ultra 7 255H')
        if machine == 'x86_64':
            self.assertEqual(profile['ik_isolation_mode'], mode)
            self.assertEqual(profile['wbc_cpus'], [2, 3])
            self.assertEqual(profile['driver_reserved_cpus'], [6, 7])
            self.assertEqual(profile['ik_cpus'], [5])
            self.assertEqual(profile['mapper_cpus'], [4])
            self.assertEqual(profile['sidecar_cpus'], [8, 9, 10, 11, 12, 13] if online == '0-13' else [14, 15])
            self.assertEqual(profile['background_cpus'], [0, 1, 8, 9, 10, 11, 12, 13])
            self.assertEqual(profile['ik_fifo_priority'], 40)
            self.assertEqual(bool(partition.profile_warnings(profile)), mode == 'soft')
        else:
            self.assertEqual(profile['ik_isolation_mode'], mode)
            self.assertEqual(profile['wbc_cpus'], [2])
            self.assertEqual(profile['driver_reserved_cpus'], [7])
            self.assertEqual(profile['ik_cpus'], [3])
            self.assertEqual(profile['mapper_cpus'], [4])
            self.assertEqual(profile['sidecar_cpus'], [10, 11])
            self.assertEqual(profile['background_cpus'], [0, 1, 5, 6, 8, 9, 10, 11])
            self.assertEqual(profile['ik_fifo_priority'], 40)
            self.assertEqual(bool(partition.profile_warnings(profile)), mode == 'soft')
        cfg = json.loads((ROOT/f'config/model_s{version}.json').read_text())
        full = require_full_model(version, str(ASSETS), cfg)
        self.assertEqual(full, ASSETS/'models'/full_model_relative_path(version))
        joints = {j.get('name'): j for j in ET.parse(full).getroot().findall('joint')}
        fixed = cfg['velocity_ik_fixed_joints']
        self.assertTrue(set(fixed) <= joints.keys())
        active = {name for name,j in joints.items() if j.get('type') != 'fixed' and name not in fixed}
        expected = {'knee_joint', 'leg_joint', 'waist_yaw_joint', 'waist_pitch_joint'}
        expected |= {f'zarm_{side}{i}_joint' for side in ('l', 'r') for i in range(1, 8)}
        self.assertEqual(active, expected)
        task = ROOT.parent/f'humanoid-wheel-control/humanoid_wheel_interface/config/kuavo_s{version}/task.info'
        removal = task.read_text().split('removeJoints {')[1].split('}')[0]
        self.assertEqual(set(re.findall(r'"([^"]+)"', removal)), set(fixed))
        for launch in ('load_kuavo_real_velocity_ik.launch', 'load_kuavo_absolute_sim_base.launch'):
            tree = ET.parse(ROOT/'launch'/launch).getroot()
            args = {arg.get('name'): arg for arg in tree.findall('arg')}
            path = args['urdfFile'].get('value').replace('$(find kuavo_assets)', str(ASSETS)).replace('$(arg robot_version)', str(version))
            self.assertEqual(Path(path), full)
            if launch == 'load_kuavo_real_velocity_ik.launch':
                # HardwareNodelet executes inside this manager, not the loader
                # or Mapper process. Bind its environment to the explicit arg.
                managers = tree.findall("node[@name='nodelet_manager']")
                self.assertEqual(len(managers), 1)
                self.assertEqual(managers[0].get('args'), 'manager')
                version_env = managers[0].findall("env[@name='ROBOT_VERSION']")
                self.assertEqual(len(version_env), 1)
                self.assertEqual(version_env[0].get('value'), '$(arg robot_version)')
                self.assertEqual(version_env[0].get('value').replace(
                    '$(arg robot_version)', str(version)), str(version))
            # Evaluate the actual launch gate with safe fixture arguments, not a
            # reimplementation. Full recursive ROS launch parsing has its own test.
            values = dict(robot_version=version, robot_type=1, nodelet_robot_type=1,
                          wheel_control_type=1, estimator_type=1, with_ik=False,
                          use_whole_body_ik_reference=True, use_external_mpc=False,
                          real=launch.startswith('load_kuavo_real'))
            expression = args['absolute_launch_contract'].get('value')[7:-1]
            self.assertEqual(eval(expression, {'__builtins__': {}, 'int': int, 'arg': values.__getitem__}), 'wheel_arm_abs_vel_s62')

    def test_all_versions_reject_missing_contract_wrong_model_and_drake_fallback(self):
        for version in (62, 63, 400062):
            for cfg in ({}, {'velocity_ik_urdf': ''}, {'velocity_ik_urdf': None},
                        {'velocity_ik_urdf': 7}, {'arm_urdf': full_model_relative_path(version)},
                        {'velocity_ik_urdf': f'biped_s{version}/urdf/drake/biped_v3_arm.urdf'},
                        {'velocity_ik_urdf': full_model_relative_path(63 if version != 63 else 62)}):
                with self.subTest(version=version, config=cfg), self.assertRaises((ValueError, FileNotFoundError)):
                    require_full_model(version, str(ASSETS), cfg)

    def test_unknown_version_rejected(self):
        for version in (15, 60, 64, 0):
            with self.subTest(version=version), self.assertRaises(ValueError):
                full_model_relative_path(version)

    def test_robot_state_server_uses_sidecar_wrapper(self):
        tree = ET.parse(ROOT/'launch/quest_velocity_ik.launch').getroot()
        node = tree.find("node[@name='robot_state_server']")
        self.assertIn('whole_body_ik_runtime.sh background /usr/bin/python3', node.get('launch-prefix'))
        self.assertTrue(node.get('launch-prefix').endswith('/scripts/sidecar_runtime.py'))


for version in (62, 63, 400062):
    for platform in PLATFORMS:
        def case(self, version=version, platform=platform):
            self.check_combination(version, platform)
        setattr(ModelPlatformMatrix, f'test_v{version}_{platform}', case)

if __name__ == '__main__':
    unittest.main()
