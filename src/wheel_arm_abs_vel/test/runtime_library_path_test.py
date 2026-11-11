#!/usr/bin/env python3
"""Exercise the real wrapper without ROS, drivers or actuator access."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class RuntimeLibraries(unittest.TestCase):
    def test_sdk_driver_and_workspace_dependencies(self):
        with tempfile.TemporaryDirectory() as directory:
            workspace = Path(directory)
            package = workspace / 'src/wheel_arm_abs_vel'
            package.mkdir(parents=True)
            sdk = workspace / 'src/kuavo-ros-control-lejulib/hardware_plant/lib'
            deps = workspace / '.absolute-deps'
            deps.mkdir()
            (deps / 'setup.bash').write_text('export LD_LIBRARY_PATH=/pinned-deps/lib:${LD_LIBRARY_PATH:-}\n')
            bins = workspace / 'bin'
            bins.mkdir()
            (bins / 'rospack').write_text('#!/bin/sh\nprintf "%s\\n" "$TEST_PACKAGE"\n')
            (bins / 'uname').write_text('#!/bin/sh\nprintf "%s\\n" "$TEST_ARCH"\n')
            for tool in bins.iterdir():
                tool.chmod(0o755)
            for arch, relative in (('aarch64', 'EC_Master_arm64/Bin/Linux/aarch64'),
                                   ('x86_64', 'EC_Master/lib')):
                expected = sdk / relative
                expected.mkdir(parents=True)
                env = dict(os.environ, PATH=str(bins)+':'+os.environ['PATH'],
                           TEST_PACKAGE=str(package), TEST_ARCH=arch,
                           KUAVO_SCHED_APPLIED='1', LD_LIBRARY_PATH='/old-driver/lib')
                output = subprocess.check_output(
                    ['bash', str(ROOT / 'scripts/whole_body_ik_runtime.sh'),
                     'controller', '/usr/bin/env'], env=env, text=True)
                values = dict(line.split('=', 1) for line in output.splitlines() if '=' in line)
                paths = values['LD_LIBRARY_PATH'].split(':')
                self.assertEqual(Path(paths[0]).resolve(), expected.resolve())
                self.assertEqual(paths[1:], ['/pinned-deps/lib', '/old-driver/lib'])


if __name__ == '__main__':
    unittest.main()
