#!/usr/bin/env python3
import json
import os
import pathlib
import sys
import unittest
from unittest.mock import patch
ROOT=pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts/tools'))
from mapper_input_scheduling import MapperInputScheduler
PROFILE=dict(mapper_cpus=[4],wbc_cpus=[2,3],ik_cpus=[5],driver_reserved_cpus=[6,7],
             background_cpus=[0,1],mapper_fifo_priority=35,ik_fifo_priority=70,
             mapper_nice=-10,python_switch_interval_sec=.001)
class SchedulingTests(unittest.TestCase):
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
if __name__=='__main__':unittest.main()
