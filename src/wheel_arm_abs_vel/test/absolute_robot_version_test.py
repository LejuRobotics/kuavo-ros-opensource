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
            self.assertEqual(identity.resolve([], {'ROBOT_VERSION': version}, [])[0], version)
            self.assertEqual(identity.resolve(['robot_version:=' + version],
                             {'ROBOT_VERSION': 'bad'}, [])[0], version)

    def test_literal_persistent_configuration(self):
        with tempfile.TemporaryDirectory() as tmp:
            f = Path(tmp)/'.bashrc'
            for text, version in [('export ROBOT_VERSION=62', '62'),
                                  (' export ROBOT_VERSION="63" # machine', '63'),
                                  ("ROBOT_VERSION='400062'", '400062')]:
                f.write_text(text)
                self.assertEqual(identity.resolve([], {}, [f]), (version, str(f)))
            f.write_text('export ROBOT_VERSION=62')
            self.assertEqual(identity.resolve([], {'ROBOT_VERSION': '63'}, [f])[0], '63')

    def test_unknown_invalid_or_conflicting_configuration_fails(self):
        for args, env in [([], {}), ([], {'ROBOT_VERSION': ''}),
                          ([], {'ROBOT_VERSION': '40'}),
                          (['robot_version:=62', 'robot_version:=63'], {}),
                          (['robot_version:=bad'], {'ROBOT_VERSION': '62'})]:
            with self.subTest(args=args, env=env), self.assertRaises(ValueError):
                identity.resolve(args, env, [])
        with tempfile.TemporaryDirectory() as tmp:
            f = Path(tmp)/'.bashrc'
            g = Path(tmp)/'environment'
            f.write_text('export ROBOT_VERSION=62')
            g.write_text('ROBOT_VERSION=63')
            with self.assertRaisesRegex(ValueError, 'conflicting'):
                identity.resolve([], {}, [f, g])
            for text in ['export ROBOT_VERSION=$(touch /tmp/never)',
                         'export ROBOT_VERSION=$OTHER',
                         'export ROBOT_VERSION=62; echo unsafe']:
                f.write_text(text)
                with self.assertRaisesRegex(ValueError, 'nonliteral'):
                    identity.resolve([], {}, [f])

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
            args = ['wheel_arm_abs_vel', 'quest_velocity_ik.launch']
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
