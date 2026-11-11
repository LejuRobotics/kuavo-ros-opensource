#!/usr/bin/env python3
import json
import os
import pathlib
import sys
import unittest
import types
import tempfile
from unittest.mock import patch
ROOT=pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
sys.path.insert(0,str(ROOT/'scripts/tools'))
from runtime_partition import arm_profile, select_profile
from mapper_input_scheduling import MapperInputScheduler, configure_mapper_interpreter, _place_control_writer
PROFILE=dict(mapper_cpus=[4],wbc_cpus=[2,3],ik_cpus=[5],driver_reserved_cpus=[6,7],
             background_cpus=[0,1],mapper_fifo_priority=35,ik_fifo_priority=70,
             mapper_nice=-10,python_switch_interval_sec=.001)
class SchedulingTests(unittest.TestCase):
    def test_agx_interpreter_inherits_mapper_core_without_rt(self):
        profile=dict(PROFILE,platform='agx_arm',mapper_interpreter_affinity=True)
        with patch.dict(os.environ,KUAVO_RUNTIME_PARTITION_JSON=json.dumps(profile)), \
             patch('mapper_input_scheduling._place_control_writer'), \
             patch('os.sched_setaffinity') as affinity, patch('os.sched_setscheduler') as policy:
            self.assertTrue(configure_mapper_interpreter())
            affinity.assert_called_once_with(0,{4})
            self.assertEqual(policy.call_args.args[:2],(0,os.SCHED_OTHER))
            self.assertEqual(policy.call_args.args[2].sched_priority,0)
    def test_other_platform_and_opt_out_unchanged(self):
        for profile in (dict(PROFILE,platform='aaeon_16',mapper_interpreter_affinity=True),dict(PROFILE,platform='agx_arm',mapper_interpreter_affinity=False)):
            with patch.dict(os.environ,KUAVO_RUNTIME_PARTITION_JSON=json.dumps(profile)), \
                 patch('os.sched_setaffinity') as affinity:
                self.assertFalse(configure_mapper_interpreter());affinity.assert_not_called()
    def test_interpreter_rejects_control_core_collision(self):
        with patch.dict(os.environ,KUAVO_RUNTIME_PARTITION_JSON=json.dumps(dict(PROFILE,platform='agx_arm',mapper_interpreter_affinity=True,mapper_cpus=[2]))), \
             patch('os.sched_setaffinity') as affinity:
            with self.assertRaises(ValueError):configure_mapper_interpreter()
            affinity.assert_not_called()
    def test_only_target_socket_writer_moves_and_original_queue_runs(self):
        calls=[]
        class FakeQueue:
            def _run(self):calls.append(self._connection.name);return 'unchanged'
        module=types.ModuleType('rospy.impl.tcpros_pubsub');module.QueuedConnection=FakeQueue
        with patch.dict(sys.modules,{'rospy.impl.tcpros_pubsub':module}), \
             patch('os.sched_setaffinity') as affinity, patch('os.sched_setscheduler') as policy, patch('ctypes.CDLL'):
            _place_control_writer(PROFILE)
            ordinary=FakeQueue();ordinary._connection=types.SimpleNamespace(name='/sensors_data_raw')
            self.assertEqual(ordinary._run(),'unchanged');affinity.assert_not_called()
            target=FakeQueue();target._connection=types.SimpleNamespace(name='/mm/teleop_tcp_target')
            self.assertEqual(target._run(),'unchanged');affinity.assert_called_once_with(0,{0,1})
            self.assertEqual(policy.call_args.args[:2],(0,os.SCHED_OTHER))
            self.assertEqual(calls,['/sensors_data_raw','/mm/teleop_tcp_target'])
    def exercise(self, denied=False, bad_affinity=False):
        with patch.dict(os.environ,KUAVO_RUNTIME_PARTITION_JSON=json.dumps(PROFILE)), \
             patch('pathlib.Path.read_text',return_value='2-3,5'), \
             patch('os.sched_getaffinity',return_value={0,1}), \
             patch('os.sched_setaffinity',side_effect=PermissionError() if bad_affinity else None), \
             patch('os.sched_getscheduler',return_value=os.SCHED_OTHER), \
             patch('os.sched_getparam',return_value=os.sched_param(0)), \
             patch('os.getpriority',return_value=0), patch('ctypes.CDLL'), \
             patch('os.sched_setscheduler',side_effect=PermissionError() if denied else None) as fifo, \
             patch('os.setpriority') as nice:
            scheduler=MapperInputScheduler();result=scheduler.apply_current_thread()
            self.assertIsNone(scheduler.apply_current_thread())
            if bad_affinity:
                fifo.assert_not_called();nice.assert_called_once_with(os.PRIO_PROCESS,0,-10)
            elif denied:
                self.assertEqual(fifo.call_args.args[2].sched_priority,35)
                nice.assert_called_once_with(os.PRIO_PROCESS,0,-10)
                self.assertTrue(any('falling back' in error for error in result['errors']))
            else:
                self.assertEqual(fifo.call_args.args[2].sched_priority,35);nice.assert_not_called()
    def test_fifo_only_after_affinity(self):self.exercise()
    def test_permission_falls_back(self):self.exercise(denied=True)
    def test_no_fifo_when_affinity_fails(self):self.exercise(bad_affinity=True)


class RuntimeProfileIntegration(unittest.TestCase):
    """Exercise production profile selection; mock only OS scheduling/ROS I/O."""
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.cpu = pathlib.Path(self.temp.name) / 'cpu'
        self.cpu.mkdir()
        self.fixture()

    def fixture(self, cpus=range(12), isolated='2,7'):
        (self.cpu / 'online').write_text(','.join(map(str, cpus)))
        (self.cpu / 'isolated').write_text(isolated)
        for cpu in cpus:
            path = self.cpu / ('cpu%d/topology' % cpu)
            path.mkdir(parents=True, exist_ok=True)
            (path / 'thread_siblings_list').write_text(str(cpu))

    def select(self, environ=None, machine='aarch64', **kwargs):
        return select_profile(environ={} if environ is None else environ,
                              machine=machine, sysroot=self.cpu, **kwargs)[0]

    def explicit(self, profile, **kwargs):
        path = pathlib.Path(self.temp.name) / 'profile.json'
        path.write_text(json.dumps(profile))
        return self.select({'KUAVO_RUNTIME_PROFILE': str(path)}, **kwargs)

    def apply_profile(self, profile, enabled, writer_enabled=True):
        with patch.dict(os.environ, KUAVO_RUNTIME_PARTITION_JSON=json.dumps(profile)), \
             patch('mapper_input_scheduling._place_control_writer') as writer, \
             patch('os.sched_setaffinity') as affinity, \
             patch('os.sched_setscheduler') as policy:
            self.assertEqual(configure_mapper_interpreter(), enabled)
            if enabled:
                affinity.assert_called_once_with(0, set(profile['mapper_cpus']))
                policy.assert_called_once_with(0, os.SCHED_OTHER, os.sched_param(0))
            else:
                affinity.assert_not_called()
                policy.assert_not_called()
            if enabled and writer_enabled:
                writer.assert_called_once_with(profile)
            else:
                writer.assert_not_called()

    def test_arm_profile_output_enables_both_optimizations(self):
        profile = arm_profile(sysroot=self.cpu)
        self.assertEqual(profile['platform'], 'agx_arm')
        self.apply_profile(profile, True)

    def test_auto_arm_profile_with_soft_hard_and_restricted_cpuset(self):
        for isolated, allowed in [('2,7', set(range(12))),
                                  ('2,3,7', set(range(12))),
                                  ('2,7', set(range(10)))]:
            with self.subTest(isolated=isolated, allowed=allowed):
                self.fixture(isolated=isolated)
                self.apply_profile(self.select(allowed=allowed), True)

    def test_explicit_legacy_arm_json_is_normalized_and_enabled(self):
        profile = self.select()
        profile['platform'] = 'agx_12'
        profile.pop('mapper_interpreter_affinity')
        profile.pop('mapper_control_writer_background')
        selected = self.explicit(profile)
        self.assertEqual(selected['platform'], 'agx_arm')
        self.apply_profile(selected, True)

    def test_explicit_opt_out_is_preserved_for_each_optimization(self):
        for interpreter, writer in [(False, True), (True, False), (False, False)]:
            with self.subTest(interpreter=interpreter, writer=writer):
                profile = self.select()
                profile.update(mapper_interpreter_affinity=interpreter,
                               mapper_control_writer_background=writer)
                selected = self.explicit(profile)
                self.assertEqual(selected['mapper_interpreter_affinity'], interpreter)
                self.assertEqual(selected['mapper_control_writer_background'], writer)
                self.apply_profile(selected, interpreter, writer)

    def test_other_arm_topologies_do_not_inherit_twelve_core_tuning(self):
        for cpus in [range(8), range(16), list(range(11)) + [13]]:
            with self.subTest(cpus=list(cpus)):
                self.fixture(cpus)
                profile = self.select()
                self.assertFalse(profile['mapper_interpreter_affinity'])
                self.assertFalse(profile['mapper_control_writer_background'])
                self.apply_profile(profile, False)
                # A copied twelve-core JSON must not widen the audited scope.
                profile.update(mapper_interpreter_affinity=True,
                               mapper_control_writer_background=True)
                selected = self.explicit(profile)
                self.assertFalse(selected['mapper_interpreter_affinity'])
                self.assertFalse(selected['mapper_control_writer_background'])
                self.apply_profile(selected, False)

    def test_legacy_quest_entry_does_not_enable_absolute_tuning(self):
        profile = self.select({'KUAVO_RUNTIME_ENTRY': 'quest3'})
        self.assertNotIn('mapper_interpreter_affinity', profile)
        self.assertNotIn('mapper_control_writer_background', profile)
        self.apply_profile(profile, False)

    def test_non_arm_profile_is_unchanged_even_with_tuning_flags(self):
        self.fixture(isolated='2,3,7')
        profile = self.select()
        profile['platform'] = 'aaeon_16'
        selected = self.explicit(profile, machine='x86_64')
        self.assertEqual(selected, profile)
        self.apply_profile(selected, False)

    def test_auto_aaeon_profile_keeps_its_existing_scheduling(self):
        self.fixture(range(16), isolated='2,3,7')
        profile = self.select(machine='x86_64', cpu_model='Intel Core Ultra 7 255H')
        self.assertEqual(profile['platform'], 'aaeon_255h')
        self.apply_profile(profile, False)

    def test_missing_capability_does_not_enable_interpreter(self):
        profile = self.select()
        profile.pop('mapper_interpreter_affinity')
        self.apply_profile(profile, False)

if __name__=='__main__':unittest.main()
