#!/usr/bin/env python3
"""255H P/E/LP-E allocation and unrelated-x86 compatibility tests."""
import copy
import importlib.util
import json
import os
import sys
import unittest
from unittest.mock import patch
from types import ModuleType

import runtime_topology_test as fixtures
partition, BASELINE, ROOT = fixtures.partition, fixtures.BASELINE, fixtures.ROOT


class X86Topology(unittest.TestCase):
    setUp = fixtures.Topology.setUp
    fixture = fixtures.Topology.fixture

    def select(self, allowed=None, env=None):
        return partition.select_profile(env or {}, 'x86_64', self.root, allowed)

    def test_original_sixteen_cpu_profile(self):
        self.fixture('0-15', '2,3,7,14,15')
        profile, source = self.select()
        self.assertEqual(profile, BASELINE)
        self.assertEqual(source, str(partition.DEFAULT_PROFILE))
        self.assertEqual(profile['driver_reserved_cpus'], [5, 6, 7])
        self.assertEqual(profile['ik_cpus'], [14])
        self.assertEqual(profile['mapper_cpus'], [4])
        self.assertEqual(profile['background_cpus'], [0, 1, 8, 9, 10, 11, 12, 13])

    def test_fourteen_cpu_does_not_move_ik_to_another_isolated_cpu(self):
        self.fixture('0-13', '2,3,4,7')
        with self.assertRaisesRegex(ValueError, 'unavailable CPUs: ik_cpus'):
            self.select()

    def test_capacity_and_cpuset_do_not_rewrite_original_profile(self):
        self.fixture('0-15', '2,3,7,14,15')
        (self.root/'cpu10/cpu_capacity').write_text('1024')
        # The legacy x86 selector does not consume the ARM cpuset input. The
        # kernel still enforces its own affinity limits at the original syscall.
        self.assertEqual(self.select(allowed={0, 1})[0], BASELINE)

    def test_explicit_profile_retains_original_precedence(self):
        self.fixture('0-15', '2,3,7,14,15')
        custom = copy.deepcopy(BASELINE)
        custom['mapper_nice'] = 0
        path = self.root/'profile.json'
        path.write_text(json.dumps(custom))
        profile, source = self.select(env={'KUAVO_RUNTIME_PROFILE': str(path)})
        self.assertEqual(profile, custom)
        self.assertEqual(source, str(path))

    def test_invalid_explicit_profile_is_not_repaired(self):
        self.fixture('0-15', '2,3,7,14,15')
        custom = dict(BASELINE, ik_cpus=[17])
        path = self.root/'profile.json'
        path.write_text(json.dumps(custom))
        with self.assertRaisesRegex(ValueError, 'unavailable CPUs: ik_cpus'):
            self.select(env={'KUAVO_RUNTIME_PROFILE': str(path)})

    def test_original_isolation_and_smt_rejections(self):
        for isolated, siblings, message in (
                ('2,3,7', None, 'kernel-isolated'),
                ('2,3,7,14,15', {4: '4,8', 8: '4,8'}, 'physical core shared')):
            with self.subTest(isolated=isolated, siblings=siblings):
                self.fixture('0-15', isolated, siblings)
                with self.assertRaisesRegex(ValueError, message):
                    self.select()

    def test_arm_options_do_not_affect_x86(self):
        self.fixture('0-15', '2,3,7,14,15')
        self.assertEqual(self.select(env={'KUAVO_REQUIRE_IK_HARD_ISOLATION': 'bad',
            'KUAVO_ARM_DRIVER_RESERVED_CPUS': 'bad'})[0], BASELINE)

    def test_x86_wrapper_preserves_syscalls_and_exported_profile(self):
        with patch('platform.machine', return_value='x86_64'), \
             patch.object(partition, 'cgroup_cpu_limit', side_effect=AssertionError('new x86 dependency')), \
             patch.object(partition, 'select_profile', return_value=(BASELINE, 'fixture')), \
             patch.object(sys, 'argv', ['partition.py', '/wrapper', 'ik', '/node']), \
             patch.dict(os.environ, {}, clear=True), \
             patch('os.sched_setscheduler', create=True) as scheduler, \
             patch('os.sched_setaffinity', create=True) as affinity, \
             patch('os.execv') as execute:
            partition.main()
            scheduler.assert_called_once_with(0, os.SCHED_OTHER, os.sched_param(0))
            affinity.assert_called_once_with(0, set(BASELINE['background_cpus']))
            self.assertEqual(json.loads(os.environ['KUAVO_RUNTIME_PARTITION_JSON']), BASELINE)
            self.assertEqual(os.environ['KUAVO_IK_CPUS'], '14')
            self.assertEqual(os.environ['KUAVO_IK_PRIORITY'], '40')
            self.assertEqual(os.environ['KUAVO_SCHED_APPLIED'], '1')
            execute.assert_called_once_with('/wrapper', ['/wrapper', 'ik', '/node'])

    def test_background_fence_is_noop_even_with_old_adapted_environment(self):
        spec = importlib.util.spec_from_file_location('fence', ROOT/'scripts/tools/background_fence.py')
        fence = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(fence)
        with patch('platform.machine', return_value='x86_64'), \
             patch.dict(os.environ, {'KUAVO_RUNTIME_PARTITION_JSON': json.dumps(dict(BASELINE, topology_adapted=True))}), \
             patch('os.sched_setaffinity', create=True, side_effect=AssertionError('x86 affinity changed')), \
             patch('os.sched_setscheduler', create=True, side_effect=AssertionError('x86 policy changed')):
            fence.apply_background_fence()


class Aaeon255h(unittest.TestCase):
    setUp = fixtures.Topology.setUp
    fixture = fixtures.Topology.fixture

    def select(self, allowed=None, env=None):
        return partition.select_profile(env or {}, 'x86_64', self.root, allowed,
                                        cpu_model='Intel(R) Core(TM) Ultra 7 255H')

    def test_soft_and_hard_keep_ik_on_p_core_5(self):
        for isolated, mode in [('2,3,7,14,15', 'soft'), ('2,3,5,7', 'hard')]:
            with self.subTest(isolated=isolated):
                self.fixture('0-15', isolated)
                p, source = self.select()
                self.assertEqual(source, str(partition.AAEON_PROFILE))
                self.assertEqual(p['wbc_cpus'], [2, 3])
                self.assertEqual(p['ik_cpus'], [5])
                self.assertEqual(p['ik_fifo_priority'], 40)
                self.assertEqual(p['mapper_cpus'], [4])
                self.assertEqual(p['driver_reserved_cpus'], [6, 7])
                self.assertEqual(p['background_cpus'], [0, 1, 8, 9, 10, 11, 12, 13])
                self.assertEqual(p['sidecar_cpus'], [14, 15])
                self.assertEqual(p['ik_isolation_mode'], mode)
                self.assertEqual(bool(partition.profile_warnings(p)), mode == 'soft')

    def test_proc_model_identification_does_not_depend_on_broken_frequency_sysfs(self):
        self.fixture('0-15', '2,3,7,14,15')
        info = self.root/'cpuinfo'
        info.write_text('processor : 0\nmodel name\t: Intel(R) Core(TM) Ultra 7 255H\n')
        # setUp deliberately mocks host detection; exercise the real reader via
        # a separate import, with fixture content rather than this test host.
        spec = importlib.util.spec_from_file_location('reader', ROOT/'scripts/runtime_partition.py')
        reader = importlib.util.module_from_spec(spec); spec.loader.exec_module(reader)
        detected = reader.cpu_model_name(info)
        self.assertEqual(partition.select_profile({}, 'x86_64', self.root,
                         cpu_model=detected)[0]['ik_cpus'], [5])
        self.assertEqual(partition.select_profile({}, 'x86_64', self.root,
                         cpu_model='Intel(R) Core(TM) Ultra 7 255HX')[0], BASELINE)

    def test_explicit_255h_config_and_explicit_legacy_override(self):
        self.fixture('0-15', '2,3,7,14,15')
        p, _ = partition.select_profile({'KUAVO_RUNTIME_PROFILE': str(partition.AAEON_PROFILE)},
                                        'x86_64', self.root, cpu_model='unknown')
        self.assertEqual(p['ik_cpus'], [5])
        old, _ = self.select(env={'KUAVO_RUNTIME_PROFILE': str(partition.DEFAULT_PROFILE)})
        self.assertEqual(old, BASELINE)

    def test_critical_cpuset_or_isolation_missing_rejected(self):
        self.fixture('0-15', '2,3,7,14,15')
        for cpu in (2, 3, 4, 5, 6, 7):
            with self.subTest(cpu=cpu), self.assertRaisesRegex(ValueError, 'critical CPUs'):
                self.select(allowed=set(range(16))-{cpu})
        self.fixture('0-15', '2,3,14,15')
        with self.assertRaisesRegex(ValueError, 'driver 7'):
            self.select()

    def test_lpe_unavailable_uses_only_background_e_cores(self):
        self.fixture('0-15', '2,3,7,14,15')
        p, _ = self.select(allowed=set(range(14)))
        self.assertEqual(p['sidecar_cpus'], [8, 9, 10, 11, 12, 13])
        self.assertEqual(p['ik_cpus'], [5])
        p, _ = self.select(allowed=set(range(15)))
        self.assertEqual(p['sidecar_cpus'], [14])
        self.fixture('0-13', '2,3,7')
        self.assertEqual(self.select()[0]['sidecar_cpus'], [8, 9, 10, 11, 12, 13])

    def test_no_available_noncontrol_cpu_does_not_silently_use_p_core(self):
        self.fixture('0-15', '2,3,7,14,15')
        with self.assertRaisesRegex(ValueError, 'sidecar_cpus'):
            self.select(allowed=set(range(8)))

    def test_strict_is_explicit_opt_in(self):
        self.fixture('0-15', '2,3,7,14,15')
        self.select()  # Stock boot must succeed without isolcpus=5.
        with self.assertRaisesRegex(ValueError, 'kernel-isolated'):
            self.select(env={'KUAVO_REQUIRE_IK_HARD_ISOLATION': 'true'})
        self.fixture('0-15', '2,3,5,7')
        self.select(env={'KUAVO_REQUIRE_IK_HARD_ISOLATION': 'true'})

    def test_smt_collision_with_critical_thread_is_rejected(self):
        self.fixture('0-15', '2,3,7,14,15', {3: '3,5', 5: '3,5'})
        with self.assertRaisesRegex(ValueError, 'physical core shared'):
            self.select()
        self.fixture('0-15', '2,3,7,14,15', {3: '3', 5: '5', 14: '5,14'})
        with self.assertRaisesRegex(ValueError, 'sidecar shares'):
            self.select()

    def test_255h_config_cannot_redirect_sidecars_to_background_p_cores(self):
        self.fixture('0-15', '2,3,7,14,15')
        config = json.loads(partition.AAEON_PROFILE.read_text())
        config['sidecar_cpus'] = [0, 1]
        path = self.root/'bad-sidecar.json'
        path.write_text(json.dumps(config))
        with self.assertRaisesRegex(ValueError, 'LP-E sidecar'):
            self.select(env={'KUAVO_RUNTIME_PROFILE': str(path)})

    def test_wrapper_fences_parent_and_exports_p_core_ik(self):
        self.fixture('0-15', '2,3,7,14,15')
        profile = self.select()[0]
        with patch('platform.machine', return_value='x86_64'), \
             patch.object(partition, 'select_profile', return_value=(profile, 'fixture')), \
             patch.object(partition, 'cgroup_cpu_limit', return_value=None) as limit, \
             patch.object(sys, 'argv', ['partition.py', '--launch', '/usr/bin/env', 'roslaunch', 'wheel_arm_abs_vel', 'quest_velocity_ik.launch']), \
             patch.dict(os.environ, {}, clear=True), \
             patch('os.sched_setscheduler') as scheduler, patch('os.sched_setaffinity') as affinity, \
             patch.object(sys, 'stderr') as stderr, \
             patch('os.execv', side_effect=lambda *_: self.assertTrue(stderr.flush.called)) as execute:
            partition.main()
            limit.assert_called_once()
            self.assertIn('[WARN]', ''.join(call.args[0] for call in stderr.write.call_args_list))
            scheduler.assert_called_once_with(0, os.SCHED_OTHER, os.sched_param(0))
            affinity.assert_called_once_with(0, set(profile['background_cpus']))
            self.assertEqual(os.environ['KUAVO_IK_CPUS'], '5')
            self.assertEqual(json.loads(os.environ['KUAVO_RUNTIME_PARTITION_JSON'])['sidecar_cpus'], [14, 15])
            execute.assert_called_once_with('/usr/bin/env', sys.argv[2:])

    def test_other_x86_parent_executes_without_affinity_changes(self):
        with patch('platform.machine', return_value='x86_64'), \
             patch.object(partition, 'select_profile', return_value=(BASELINE, 'fixture')), \
             patch.object(sys, 'argv', ['partition.py', '--launch', '/usr/bin/env', 'roslaunch', 'pkg', 'launch']), \
             patch('os.sched_setaffinity', side_effect=AssertionError('legacy parent changed')), \
             patch('os.sched_setscheduler', side_effect=AssertionError('legacy parent changed')), \
             patch('os.execv') as execute:
            partition.main()
            execute.assert_called_once_with('/usr/bin/env', sys.argv[2:])

    def test_sidecar_fence_applies_lpe_and_agx_pools(self):
        spec = importlib.util.spec_from_file_location('fence', ROOT/'scripts/tools/background_fence.py')
        fence = importlib.util.module_from_spec(spec); spec.loader.exec_module(fence)
        self.fixture('0-15', '2,3,7,14,15')
        aaeon = self.select()[0]
        self.fixture('0-11', '2,7')
        arm = partition.select_profile({}, 'aarch64', self.root)[0]
        for p, expected in [(aaeon, {14, 15}), (arm, {10, 11})]:
            with self.subTest(platform=p['platform']), \
                 patch.dict(os.environ, {'KUAVO_RUNTIME_PARTITION_JSON': json.dumps(p)}), \
                 patch('os.sched_setscheduler') as scheduler, patch('os.sched_setaffinity') as affinity:
                fence.apply_background_fence()
                scheduler.assert_called_once_with(0, os.SCHED_OTHER, os.sched_param(0))
                affinity.assert_called_once_with(0, expected)
        with patch.dict(os.environ, {'KUAVO_RUNTIME_PARTITION_JSON': json.dumps(dict(aaeon, sidecar_cpus=[5]))}), \
             patch('os.sched_setaffinity', side_effect=AssertionError('protected CPU touched')), \
             self.assertRaisesRegex(ValueError, 'invalid background fence'):
            fence.apply_background_fence()

    def test_embedded_state_publisher_fences_before_publish_loop(self):
        import ast
        from types import SimpleNamespace
        source = ast.parse((ROOT/'scripts/robot_state_server_velocity.py').read_text())
        cls = next(n for n in source.body if isinstance(n, ast.ClassDef) and n.name == 'RobotStateServer')
        method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '_publish_loop')
        events = []
        namespace = {'rospy': SimpleNamespace(Rate=lambda hz: events.append(('rate', hz)))}
        exec(compile(ast.Module(body=[method], type_ignores=[]), 'state-publisher', 'exec'), namespace)
        # Keep this unit test independent of tools/__init__.py's ROS package lookup.
        fence = ModuleType('tools.background_fence')
        fence.apply_background_fence = lambda: events.append('fence')
        with patch.dict(sys.modules, {'tools': ModuleType('tools'), 'tools.background_fence': fence}):
            namespace['_publish_loop'](SimpleNamespace(publish_rate=20, running=False))
        self.assertEqual(events, ['fence', ('rate', 20)])

    def test_standalone_sidecar_fences_before_exec(self):
        spec = importlib.util.spec_from_file_location('fence', ROOT/'scripts/tools/background_fence.py')
        fence = importlib.util.module_from_spec(spec); spec.loader.exec_module(fence)
        with patch.dict(sys.modules, {'tools': ModuleType('tools'), 'tools.background_fence': fence}):
            spec = importlib.util.spec_from_file_location('sidecar', ROOT/'scripts/sidecar_runtime.py')
            sidecar = importlib.util.module_from_spec(spec); spec.loader.exec_module(sidecar)
        events = []
        with patch.object(sidecar, 'apply_background_fence', side_effect=lambda: events.append('fence') or True), \
             patch('os.setpriority', side_effect=lambda *args: events.append('nice')), \
             patch('os.execvp', side_effect=lambda *args: events.append(('exec', args))), \
             patch.object(sys, 'argv', ['sidecar.py', '/state_server', '--flag']):
            sidecar.main()
        self.assertEqual(events, ['fence', 'nice', ('exec', ('/state_server', ['/state_server', '--flag']))])

    def test_agx_uses_cpu3_soft_even_when_other_isolated_core_exists(self):
        self.fixture('0-11', '2,7,8')
        p, _ = partition.select_profile({}, 'aarch64', self.root)
        self.assertEqual(p['ik_cpus'], [3])
        self.assertEqual(p['ik_isolation_mode'], 'soft')
        self.assertEqual(p['sidecar_cpus'], [10, 11])


if __name__ == '__main__':
    unittest.main()
