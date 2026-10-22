#!/usr/bin/env python3
"""Startup topology contracts; fixtures are not AGX hardware qualification."""
import copy
from collections import namedtuple
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('partition', ROOT/'scripts/runtime_partition.py')
partition = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(partition)
BASELINE = json.loads((ROOT/'config/runtime_partition.json').read_text())


class Topology(unittest.TestCase):
    def setUp(self):
        # Topology fixtures must not inherit the CPU model of the test host.
        model = patch.object(partition, 'cpu_model_name', return_value='fixture legacy x86')
        model.start()
        self.addCleanup(model.stop)
        # Darwin lacks these Linux APIs. Tests always mock calls; never change
        # the test host's scheduling. Fixtures remain the Linux topology source.
        api = {'SCHED_OTHER': 0, 'sched_param': namedtuple('sched_param', 'sched_priority'),
               'sched_setaffinity': None, 'sched_setscheduler': None, 'sched_getaffinity': None}
        for name, value in api.items():
            if not hasattr(os, name):
                replacement = patch.object(os, name, value, create=True)
                replacement.start()
                self.addCleanup(replacement.stop)
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)

    def fixture(self, online='0-11', isolated='2,3,7', siblings=None):
        (self.root/'online').write_text(online)
        (self.root/'isolated').write_text(isolated)
        for cpu in partition.cpu_list(online):
            root = self.root/('cpu%d' % cpu)
            (root/'topology').mkdir(parents=True, exist_ok=True)
            (root/'topology/thread_siblings_list').write_text((siblings or {}).get(cpu, str(cpu)))
        return self.root

    def select(self, **kwargs):
        return partition.select_profile(environ={}, machine='aarch64', sysroot=self.root, **kwargs)[0]

    def test_eight_cpu_arm_preserves_shared_driver_and_wbc_contracts(self):
        self.fixture('0-7')
        profile = self.select()
        self.assertEqual(profile['wbc_cpus'], [2])
        self.assertEqual(profile['driver_reserved_cpus'], [7])
        self.assertEqual(profile['ik_cpus'], [3])
        self.assertEqual(profile['mapper_cpus'], [4])
        self.assertEqual(profile['background_cpus'], [0, 1, 5, 6])
        self.assertLess(max(sum([profile[k] for k in profile if k.endswith('_cpus')], [])), 8)

    def test_twelve_cpu_arm_uses_capacity_and_keeps_background(self):
        self.fixture(isolated='2,3,7,8')
        (self.root/'cpu8/cpu_capacity').write_text('1024')
        (self.root/'cpu4/cpu_capacity').write_text('1024')
        profile = self.select()
        self.assertEqual(profile['ik_cpus'], [3])
        self.assertEqual(profile['mapper_cpus'], [4])
        self.assertEqual(profile['background_cpus'], [0, 1, 5, 6, 9, 10, 11])

    def test_sparse_online_cpu_ids_are_used_instead_of_cpu_count(self):
        self.fixture('0-2,4-7,10', '2,7,10')
        self.assertEqual(self.select()['ik_cpus'], [10])

    def test_missing_isolated_ik_soft_fallback(self):
        self.fixture('0-7', '2,7')
        profile = self.select()
        self.assertEqual(profile['ik_isolation_mode'], 'soft')
        self.assertEqual(profile['ik_cpus'], [3])
        self.assertTrue(partition.profile_warnings(profile))
        with self.assertRaisesRegex(ValueError, 'strict mode'):
            partition.select_profile({'KUAVO_REQUIRE_IK_HARD_ISOLATION': 'true'},
                                     'aarch64', self.root)

    def test_missing_wbc_isolation_rejected(self):
        self.fixture('0-7', '3,7')
        with self.assertRaisesRegex(ValueError, 'WBC CPU 2'):
            self.select()

    def test_power_mode_with_offline_driver_cpu_is_not_silently_accepted(self):
        self.fixture('0-5', '2,3')
        with self.assertRaisesRegex(ValueError, 'must be online and allowed'):
            self.select()

    def test_cpuset_restriction_is_checked(self):
        self.fixture()
        with self.assertRaisesRegex(ValueError, 'must be online and allowed'):
            self.select(allowed={0, 1, 2, 3, 4, 5, 6})
        with self.assertRaisesRegex(ValueError, 'soft isolation requires'):
            self.select(allowed={0, 1, 2, 4, 5, 6, 7})

    def test_physical_siblings_cannot_be_shared_between_roles(self):
        self.fixture('0-7', '2,3,7', {2: '2,3', 3: '2,3'})
        with self.assertRaisesRegex(ValueError, 'soft isolation requires'):
            self.select()

    def test_mapper_cannot_consume_last_background_core(self):
        self.fixture('0,2,3,7')
        with self.assertRaisesRegex(ValueError, 'Mapper needs'):
            self.select()

    def test_x86_aaeon_default_is_exactly_the_original_profile(self):
        self.fixture('0-15', '2,3,7,14,15')
        profile, source = partition.select_profile({}, 'x86_64', self.root)
        self.assertEqual(profile, BASELINE)
        self.assertEqual(source, str(partition.DEFAULT_PROFILE))

    def test_fourteen_cpu_x86_without_separate_isolated_ik_is_rejected(self):
        self.fixture('0-13', '2,3,7')
        with self.assertRaisesRegex(ValueError, 'unavailable CPUs: ik_cpus'):
            partition.select_profile({}, 'x86_64', self.root)

    def test_x86_explicit_profile_keeps_precedence(self):
        self.fixture('0-15', '2,3,7,14,15')
        custom = copy.deepcopy(BASELINE)
        custom['mapper_nice'] = 0
        path = self.root/'profile.json'
        path.write_text(json.dumps(custom))
        profile, source = partition.select_profile({'KUAVO_RUNTIME_PROFILE': str(path)}, 'x86_64', self.root)
        self.assertEqual(profile, custom)
        self.assertEqual(source, str(path))

    def test_arm_explicit_profile_precedence_and_fixed_driver_validation(self):
        self.fixture()
        custom = self.select()
        custom['mapper_cpus'], custom['background_cpus'] = [4], [0, 1, 8, 9, 10, 11]
        path = self.root/'profile.json'
        path.write_text(json.dumps(custom))
        env = {'KUAVO_RUNTIME_PROFILE': str(path)}
        self.assertEqual(partition.select_profile(env, 'arm64', self.root)[0], custom)
        custom['wbc_cpus'] = [3]
        custom['ik_cpus'] = [2]
        path.write_text(json.dumps(custom))
        with self.assertRaisesRegex(ValueError, 'controller WBC CPU 2'):
            partition.select_profile(env, 'arm64', self.root)
        custom['wbc_cpus'], custom['ik_cpus'] = [2], [3]
        custom['driver_reserved_cpus'] = [5, 6]
        path.write_text(json.dumps(custom))
        with self.assertRaisesRegex(ValueError, 'reserve configured driver'):
            partition.select_profile(env, 'arm64', self.root)

    def test_preflight_does_not_change_affinity_or_scheduler_or_exec(self):
        self.fixture()
        with patch('os.sched_getaffinity', return_value=set(range(12))), \
             patch.object(partition, 'cgroup_cpu_limit', return_value=None), \
             patch.object(partition, 'topology', return_value=partition.topology(self.root)), \
             patch.object(partition, 'select_profile', return_value=(self.select(), 'fixture')), \
             patch('os.sched_setaffinity', side_effect=AssertionError('affinity mutation')), \
             patch('os.sched_setscheduler', side_effect=AssertionError('scheduler mutation')), \
             patch('os.execv', side_effect=AssertionError('exec mutation')), \
             patch('builtins.print'):
            self.assertEqual(partition.inspect_profile(), 0)

    def test_wrapper_exports_selected_arm_profile_to_child_without_remapping_drivers(self):
        self.fixture('0-7')
        profile = self.select()
        with patch('platform.machine', return_value='aarch64'), \
             patch.object(partition, 'cgroup_cpu_limit', return_value=None), \
             patch.object(partition, 'select_profile', return_value=(profile, 'fixture')), \
             patch.object(sys, 'argv', ['partition.py', '/wrapper', 'ik', '/node']), \
             patch.dict(os.environ, {}, clear=True), \
             patch('os.sched_setscheduler') as scheduler, \
             patch('os.sched_setaffinity') as affinity, \
             patch('os.execv') as execute:
            partition.main()
            affinity.assert_called_once_with(0, {0, 1, 5, 6})
            self.assertEqual(os.environ['KUAVO_IK_CPUS'], '3')
            self.assertEqual(json.loads(os.environ['KUAVO_RUNTIME_PARTITION_JSON']), profile)
            self.assertEqual(os.environ['KUAVO_SCHED_APPLIED'], '1')
            self.assertEqual(scheduler.call_count, 1)
            execute.assert_called_once_with('/wrapper', ['/wrapper', 'ik', '/node'])


    def test_parent_launcher_forwards_strict_and_driver_settings_before_roslaunch(self):
        scripts = self.root/'scripts'
        scripts.mkdir()
        (scripts/'absolute_robot_version.py').write_text(
            (ROOT/'scripts/absolute_robot_version.py').read_text())
        (scripts/'runtime_partition.py').write_text(
            "import os,sys,json\nprint(json.dumps([sys.argv[1:], "
            "os.environ['KUAVO_REQUIRE_IK_HARD_ISOLATION'], "
            "os.environ['KUAVO_ARM_DRIVER_RESERVED_CPUS']]))\n")
        uname = self.root/'uname'
        uname.write_text('#!/bin/sh\necho aarch64\n')
        uname.chmod(0o755)
        rospack = self.root/'rospack'
        rospack.write_text('#!/bin/sh\nprintf "%s\\n" "' + str(self.root) + '"\n')
        rospack.chmod(0o755)
        env = dict(os.environ, PATH=str(self.root)+':'+os.environ['PATH'],
                   KUAVO_RUNTIME_PARTITION='1', ROBOT_VERSION='62')
        args = ['wheel_arm_abs_vel', 'control.launch', 'controller_type:=wheel_arm_abs_vel',
                'require_ik_hard_isolation:=true', 'arm_driver_reserved_cpus:=5-7']
        result = subprocess.check_output(
            ['bash', str(ROOT/'scripts/platform_roslaunch.sh')] + args, env=env, text=True)
        argv, strict, drivers = json.loads(result)
        self.assertEqual(argv, ['--launch', '/usr/bin/env', 'roslaunch'] + args)
        self.assertEqual((strict, drivers), ('true', '5-7'))

    def test_parent_launcher_x86_is_plain_roslaunch(self):
        for name, script in (
                ("uname", "echo x86_64"),
                ("roslaunch", "printf '%s\\n' \"$@\""),
                ("rospack", "exit 99")):
            path = self.root/name
            path.write_text("#!/bin/sh\n" + script + "\n")
            path.chmod(0o755)
        result = subprocess.check_output(
            ["bash", str(ROOT/"scripts/platform_roslaunch.sh"), "pkg", "file.launch"],
            env=dict(os.environ, PATH=str(self.root)+":"+os.environ["PATH"]), text=True)
        self.assertEqual(result.splitlines(), ["pkg", "file.launch"])

    def test_quest3_entries_join_the_fence_without_version_resolution(self):
        """Quest3 遥控入口要进分区，但不做绝对链路的 ROBOT_VERSION 解析。"""
        scripts = self.root/'scripts'
        scripts.mkdir()
        (scripts/'absolute_robot_version.py').write_text(
            "import sys\nsys.exit('must not be called for quest3 entries')\n")
        (scripts/'runtime_partition.py').write_text(
            "import os,sys,json\nprint(json.dumps([sys.argv[1:], "
            "os.environ.get('ROBOT_VERSION')]))\n")
        for name, script in (("uname", "echo aarch64"),
                             ("rospack", 'printf "%s\\n" "' + str(self.root) + '"'),
                             ("roslaunch", 'printf "PLAIN %s\\n" "$@"')):
            path = self.root/name
            path.write_text("#!/bin/sh\n" + script + "\n")
            path.chmod(0o755)
        env = dict(os.environ, PATH=str(self.root)+":"+os.environ["PATH"],
                   KUAVO_RUNTIME_PARTITION='1')
        env.pop('ROBOT_VERSION', None)
        for launch in ("launch_quest3_ik.launch",
                       "launch_quest3_ik_videostream_orbbec.launch",
                       "launch_dual_quest3_ik.launch"):
            with self.subTest(launch=launch):
                args = ["noitom_hi5_hand_udp_python", launch]
                result = subprocess.check_output(
                    ["bash", str(ROOT/"scripts/platform_roslaunch.sh")] + args,
                    env=env, text=True)
                argv, robot_version = json.loads(result)
                self.assertEqual(argv, ['--launch', '/usr/bin/env', 'roslaunch'] + args)
                self.assertIsNone(robot_version)

    def test_wrapper_self_locates_when_package_is_not_built(self):
        """wheel_arm_abs_vel 未进 devel 空间时 rospack 返回非零，包装脚本要能自定位。

        本机只构建运行需要的包，wheel_arm_abs_vel 不在构建列表里。若包装脚本死守
        `rospack find`，`set -e` 会让 Quest3 入口永远拉不起来，绑核通道也就永远
        拿不到 KUAVO_IK_CPUS。
        """
        scripts = self.root/'scripts'
        scripts.mkdir()
        (scripts/'platform_roslaunch.sh').write_text(
            (ROOT/'scripts/platform_roslaunch.sh').read_text())
        (scripts/'runtime_partition.py').write_text(
            "import json,sys\nprint(json.dumps(sys.argv[1:]))\n")
        for name, script in (("rospack", "exit 99"),
                             ("roslaunch", 'printf "PLAIN %s\\n" "$@"')):
            path = self.root/name
            path.write_text("#!/bin/sh\n" + script + "\n")
            path.chmod(0o755)
        args = ["noitom_hi5_hand_udp_python", "launch_quest3_ik.launch"]
        result = subprocess.check_output(
            ["bash", str(scripts/"platform_roslaunch.sh")] + args,
            env=dict(os.environ, PATH=str(self.root)+":"+os.environ["PATH"],
                     KUAVO_RUNTIME_PARTITION='1'), text=True)
        self.assertEqual(json.loads(result),
                         ['--launch', '/usr/bin/env', 'roslaunch'] + args)

    def test_control_tree_stays_out_of_the_fence(self):
        """运控树是独立 roslaunch，套围栏只会缩小 nodelet 的核池，故保持直通。"""
        for name, script in (("uname", "echo aarch64"),
                             ("roslaunch", 'printf "%s\\n" "$@"'),
                             ("rospack", "exit 99")):
            path = self.root/name
            path.write_text("#!/bin/sh\n" + script + "\n")
            path.chmod(0o755)
        args = ["humanoid_controllers", "load_kuavo_real_wheel.launch",
                "joystick_type:=h12", "start_way:=auto"]
        result = subprocess.check_output(
            ["bash", str(ROOT/"scripts/platform_roslaunch.sh")] + args,
            env=dict(os.environ, PATH=str(self.root)+":"+os.environ["PATH"],
                     KUAVO_RUNTIME_PARTITION='1', ROBOT_VERSION='62'), text=True)
        self.assertEqual(result.splitlines(), args)


    def test_arm_parent_launcher_leaves_legacy_and_unrelated_commands_untouched(self):
        for name, script in (("uname", "echo aarch64"),
                             ("roslaunch", "printf '%s\\n' \"$@\""),
                             ("rospack", "exit 99")):
            path = self.root/name
            path.write_text("#!/bin/sh\n" + script + "\n")
            path.chmod(0o755)
        for args in (["wheel_arm_abs_vel", "control.launch"],
                     ["wheel_arm_abs_vel", "control.launch", "controller_type:=legacy"],
                     ["wheel_arm_abs_vel", "control.launch", "controller_type:=wheel_arm_abs_vel",
                      "controller_type:=legacy"],
                     ["humanoid_controllers", "load_kuavo_real.launch"]):
            with self.subTest(args=args):
                result = subprocess.check_output(
                    ["bash", str(ROOT/"scripts/platform_roslaunch.sh")] + args,
                    env=dict(os.environ, PATH=str(self.root)+":"+os.environ["PATH"]), text=True)
                self.assertEqual(result.splitlines(), args)

    def cgroup_fixture(self, membership, mountinfo, files):
        proc = self.root/'proc'
        proc.mkdir()
        (proc/'cgroup').write_text(membership)
        (proc/'mountinfo').write_text(mountinfo)
        for name, value in files.items():
            file = self.root/name
            file.parent.mkdir(parents=True, exist_ok=True)
            file.write_text(value)
        return proc

    def test_cgroup_v2_nested_cpuset_is_authoritative(self):
        proc = self.cgroup_fixture('0::/robot\n',
            '36 25 0:32 / /sys/fs/cgroup rw - cgroup2 cgroup rw\n',
            {'sys/fs/cgroup/robot/cpuset.cpus.effective': '0-2,4-11',
             'sys/fs/cgroup/cpuset.cpus.effective': '0-11'})
        self.assertEqual(partition.cgroup_cpu_limit(proc, self.root), {0, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11})

    def test_cgroup_v1_mounted_subtree_and_inheritance(self):
        proc = self.cgroup_fixture('3:cpuset:/robot/child\n',
            '36 25 0:32 /robot /sys/fs/cgroup/cpuset rw - cgroup cgroup rw,cpuset\n',
            {'sys/fs/cgroup/cpuset/child/cpuset.cpus': '',
             'sys/fs/cgroup/cpuset/cpuset.cpus': '0-7'})
        self.assertEqual(partition.cgroup_cpu_limit(proc, self.root), set(range(8)))

    def test_unresolvable_cgroup_does_not_fall_back_to_all_cpus(self):
        proc = self.cgroup_fixture('0::/missing\n',
            '36 25 0:32 / /sys/fs/cgroup rw - cgroup2 cgroup rw\n',
            {'sys/fs/cgroup/cpuset.cpus.effective': '0-11'})
        with self.assertRaisesRegex(ValueError, 'cannot resolve'):
            partition.cgroup_cpu_limit(proc, self.root)

    def test_no_cpuset_controller_means_no_extra_limit(self):
        proc = self.cgroup_fixture('3:cpu:/robot\n', '', {})
        self.assertIsNone(partition.cgroup_cpu_limit(proc, self.root))

    def test_twelve_online_ten_inherited_cpus_still_allow_explicit_rt_binding(self):
        self.fixture('0-11', '2,7')
        profile = self.select(allowed=set(range(12)))
        # The supplied AGX screenshot reports 12 online but nproc=10. A narrowed
        # launcher mask must not be confused with the hard cgroup cpuset ceiling.
        inherited = set(range(12)) - {2, 7}
        with patch('platform.machine', return_value='aarch64'), \
             patch('os.sched_getaffinity', return_value=inherited), \
             patch.object(partition, 'cgroup_cpu_limit', return_value=set(range(12))), \
             patch.object(partition, 'select_profile', return_value=(profile, 'fixture')) as select, \
             patch.object(sys, 'argv', ['partition.py', '/wrapper', 'controller']), \
             patch.dict(os.environ, {}, clear=True), \
             patch('os.sched_setscheduler'), patch('os.sched_setaffinity'), patch('os.execv'):
            partition.main()
            select.assert_called_once_with(allowed=set(range(12)))
            self.assertEqual(os.environ['KUAVO_IK_CPUS'], '3')


    def test_reported_agx_orin_soft_and_hard_layouts(self):
        # User-supplied 2026-10-17 AGX output: 12 single-thread cores, only 2/7
        # isolated, equal capacity 1024 and max frequency 2201600 kHz.
        self.fixture('0-11', '2,7')
        for cpu in range(12):
            root = self.root/('cpu%d' % cpu)
            (root/'cpu_capacity').write_text('1024')
            (root/'cpufreq').mkdir()
            (root/'cpufreq/cpuinfo_max_freq').write_text('2201600')
        soft = self.select(allowed=set(range(12)))
        self.assertEqual(soft['ik_isolation_mode'], 'soft')
        self.assertEqual(soft['background_cpus'], [0, 1, 5, 6, 8, 9, 10, 11])
        self.assertEqual(soft['ik_cpus'], [3])
        self.assertEqual(soft['mapper_cpus'], [4])
        # Alternative pre-existing host isolation, not a host mutation.
        (self.root/'isolated').write_text('2,3,7')
        profile = self.select(allowed=set(range(12)))
        self.assertEqual(profile['wbc_cpus'], [2])
        self.assertEqual(profile['ik_cpus'], [3])
        self.assertEqual(profile['mapper_cpus'], [4])
        self.assertEqual(profile['driver_reserved_cpus'], [7])
        self.assertEqual(profile['background_cpus'], [0, 1, 5, 6, 8, 9, 10, 11])


    def test_driver_reservation_extension(self):
        self.fixture(isolated='2,7')
        profile, _ = partition.select_profile(
            {'KUAVO_ARM_DRIVER_RESERVED_CPUS': '5-7'}, 'aarch64', self.root)
        self.assertEqual(profile['driver_reserved_cpus'], [5, 6, 7])
        self.assertEqual(profile['background_cpus'], [0, 1, 8, 9, 10, 11])

    def test_cpuset_can_reduce_background_without_losing_rt_roles(self):
        self.fixture(isolated='2,7')
        profile = self.select(allowed={0, 2, 3, 4, 7, 8})
        self.assertEqual(profile['background_cpus'], [0, 8])
        with self.assertRaisesRegex(ValueError, 'soft isolation requires'):
            self.select(allowed={0, 2, 4, 7, 8})

    def test_hard_isolation_preferred_and_strict_accepts_existing_third_core(self):
        self.fixture(isolated='2,3,7')
        p, _ = partition.select_profile(
            {'KUAVO_REQUIRE_IK_HARD_ISOLATION': 'true'}, 'aarch64', self.root)
        self.assertEqual(p['ik_cpus'], [3])
        self.assertEqual(p['ik_isolation_mode'], 'hard')
        self.assertEqual(partition.profile_warnings(p), [])

    def test_explicit_soft_profile_cannot_bypass_strict_mode(self):
        self.fixture(isolated='2,7')
        path = self.root/'custom.json'
        path.write_text(json.dumps(self.select()))
        with self.assertRaisesRegex(ValueError, 'kernel-isolated'):
            partition.select_profile({'KUAVO_RUNTIME_PROFILE': str(path),
                                      'KUAVO_REQUIRE_IK_HARD_ISOLATION': 'true'},
                                     'aarch64', self.root)

    def test_x86_ignores_arm_only_knobs(self):
        self.fixture('0-15', '2,3,7,14,15')
        profile, _ = partition.select_profile(
            {'KUAVO_REQUIRE_IK_HARD_ISOLATION': 'nonsense',
             'KUAVO_ARM_DRIVER_RESERVED_CPUS': 'invalid'}, 'x86_64', self.root)
        self.assertEqual(profile, BASELINE)

    def test_background_child_resets_inherited_critical_affinity_to_arm_sidecar(self):
        spec = importlib.util.spec_from_file_location('fence',
                    ROOT/'scripts/tools/background_fence.py')
        fence = importlib.util.module_from_spec(spec); spec.loader.exec_module(fence)
        self.fixture(isolated='2,7')
        profile = self.select()
        with patch.dict(os.environ, {'KUAVO_RUNTIME_PARTITION_JSON': json.dumps(profile)}), \
             patch('platform.machine', return_value='aarch64'), \
             patch('os.sched_setscheduler') as scheduler, patch('os.sched_setaffinity') as affinity:
            fence.apply_background_fence()
            affinity.assert_called_once_with(0, {10, 11})
            scheduler.assert_called_once()
        with patch('platform.machine', return_value='x86_64'), \
             patch.dict(os.environ, {}, clear=True), \
             patch('os.sched_setaffinity', side_effect=AssertionError('x86 changed')):
            fence.apply_background_fence()


if __name__ == '__main__':
    unittest.main()
