#!/usr/bin/env python3
"""Guard the private simulator's upstream-source boundary and lifecycle contract."""
from pathlib import Path
import sys
import unittest
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from prepare_absolute_sim import prepare

class PrivateSimulationSourceTest(unittest.TestCase):
    def setUp(self):
        self.path = ROOT.parent / "mujoco/src/mujoco_node.cc"
        self.source = self.path.read_text()

    def test_source_stays_unchanged_and_cleanup_follows_both_workers(self):
        generated = prepare(self.source)
        self.assertEqual(self.path.read_text(), self.source)
        self.assertNotIn("mujocolcm.startLCMThread();", generated)
        end = generated[generated.index("  sim->RenderLoop();"):]
        self.assertLess(end.index("physicsthreadhandle.join()"), end.index("mj_deleteData"))
        self.assertLess(end.index("g_dexhand_node->stop()"), end.index("mj_deleteData"))
        self.assertLess(end.index("absolute_stop_callbacks()"), end.index("mj_deleteData"))
        self.assertLess(end.index("sim.reset()"), end.index("mj_deleteData"))
        self.assertIn("signal(SIGTERM, signalHandler)", generated)

    def test_servo_snapshot_uses_command_mutex(self):
        generated = prepare(self.source)
        callback = generated[generated.index("void jointCmdCallback"):]
        self.assertLess(callback.index("lock(queueMutex)"), callback.index("absolute_servo.encode"))
        self.assertIn("absolute_servo.configure", generated)

    def test_upstream_command_drift_fails_closed(self):
        with self.assertRaises(ValueError):
            prepare(self.source.replace("  // controlCommands.push(tau);", ""))

    def test_upstream_lifecycle_drift_fails_closed(self):
        with self.assertRaises(ValueError):
            prepare(self.source.replace("  physicsthreadhandle.join();", "  physicsthreadhandle.detach();"))

if __name__ == "__main__":
    unittest.main()
