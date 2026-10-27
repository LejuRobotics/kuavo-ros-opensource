#!/usr/bin/env python3
"""Offline startup identity and shell dispatch tests; never execute ROS."""
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('identity', ROOT/'scripts/absolute_robot_version.py')
identity = importlib.util.module_from_spec(spec)
spec.loader.exec_module(identity)


class IdentityTest(unittest.TestCase):
    def test_all_models_explicit_and_environment(self):
        for version in ('62', '63', '400062'):
            self.assertEqual(identity.resolve([], {'ROBOT_VERSION': version})[0], version)
            self.assertEqual(identity.resolve(['robot_version:=' + version],
                             {'ROBOT_VERSION': 'bad'})[0], version)

    def test_missing_invalid_or_conflicting_environment_fails(self):
        for args, env in [([], {}), ([], {'ROBOT_VERSION': ''}),
                          ([], {'ROBOT_VERSION': '40'}),
                          (['robot_version:=62', 'robot_version:=63'], {}),
                          (['robot_version:=bad'], {'ROBOT_VERSION': '62'})]:
            with self.subTest(args=args, env=env), self.assertRaises(ValueError):
                identity.resolve(args, env)

    def test_saved_shell_files_do_not_override_or_supply_identity(self):
        with tempfile.TemporaryDirectory() as tmp:
            (Path(tmp)/'.bashrc').write_text('export ROBOT_VERSION=63')
            env = dict(os.environ, HOME=tmp)
            env.pop('ROBOT_VERSION', None)
            result = subprocess.run(['python3', str(ROOT/'scripts/absolute_robot_version.py')],
                                    env=env, capture_output=True, text=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertEqual(result.stdout, '')
            env['ROBOT_VERSION'] = '62'
            result = subprocess.run(['python3', str(ROOT/'scripts/absolute_robot_version.py')],
                                    env=env, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0)
            self.assertEqual(result.stdout.strip(), '62')

    def wrapper(self, args, version):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            for name, body in [
                ('rospack', '#!/bin/sh\nprintf "%s\\n" "' + str(ROOT) + '"\n'),
                ('roslaunch', '#!/usr/bin/python3\nimport json, os, sys\nprint(json.dumps([os.environ.get("ROBOT_VERSION"), sys.argv[1:]]))\n')]:
                f = folder/name
                f.write_text(body)
                f.chmod(0o755)
            env = dict(os.environ, PATH=str(folder)+':/usr/bin:/bin',
                       HOME=tmp, KUAVO_RUNTIME_PARTITION='0', ROBOT_VERSION=version)
            return subprocess.run(['bash', str(ROOT/'scripts/platform_roslaunch.sh')] + args,
                                  env=env, capture_output=True, text=True)

    def test_wrapper_autodetect_and_explicit_override(self):
        for version in ('62', '63', '400062'):
            for entry in ('quest_velocity_ik.launch', 'load_kuavo_real_velocity_ik.launch',
                          'load_kuavo_mujoco_velocity_ik.launch',
                          'load_kuavo_absolute_sim_base.launch', 'control.launch'):
                args = ['wheel_arm_abs_vel', entry]
                if entry == 'control.launch':
                    args += ['controller_type:=wheel_arm_abs_vel', 'real:=false']
                with self.subTest(version=version, entry=entry):
                    result = self.wrapper(args, version)
                    self.assertEqual(result.returncode, 0, result.stderr)
                    self.assertEqual(json.loads(result.stdout), [version, args])
                    result = self.wrapper(args + ['robot_version:=' + version], 'bad')
                    self.assertEqual(result.returncode, 0, result.stderr)
                    self.assertEqual(json.loads(result.stdout)[0], version)

    def test_wrapper_failure_cannot_fall_through_to_roslaunch(self):
        result = self.wrapper(['wheel_arm_abs_vel', 'load_kuavo_real_velocity_ik.launch'], 'bad')
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(result.stdout, '')
        self.assertIn('unsupported ROBOT_VERSION', result.stderr)

    def test_legacy_dispatch_unchanged(self):
        for args in [['humanoid_controllers', 'control.launch'],
                     ['wheel_arm_abs_vel', 'control.launch']]:
            result = self.wrapper(args, 'bad')
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(json.loads(result.stdout), ['bad', args])
            self.assertEqual(result.stderr, '')


if __name__ == '__main__':
    unittest.main()
