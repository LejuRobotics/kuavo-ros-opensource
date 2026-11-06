#!/usr/bin/env python3
"""VR startup head-mode policy, independent of ROS."""
import ast
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "scripts/monitor_quest3.py"
TREE = ast.parse(SOURCE.read_text())
CLASS = next(n for n in TREE.body if isinstance(n, ast.ClassDef)
             and n.name == "Quest3BoneFramePublisher")


class ServiceUnavailable(Exception):
    pass


def extract(name, scope):
    func = next(n for n in CLASS.body if isinstance(n, ast.FunctionDef) and n.name == name)
    exec(compile(ast.Module(body=[func], type_ignores=[]), str(SOURCE), "exec"), scope)
    return scope[name]


class StartupHeadModeTest(unittest.TestCase):
    def setUp(self):
        self.client = Mock(return_value=SimpleNamespace(success=True, message="ok"))
        self.ros = SimpleNamespace(
            wait_for_service=Mock(), ServiceProxy=Mock(return_value=self.client),
            ROSException=ServiceUnavailable, ServiceException=ServiceUnavailable,
            loginfo=Mock(), logwarn=Mock(), logwarn_throttle=Mock())
        scope = {"rospy": self.ros, "SetHeadControlMode": object}
        self.receive = extract("_head_startup_mode_callback", scope)
        self.tick = extract("_try_startup_head_mode", scope)
        self.node = SimpleNamespace(_head_startup_pending=True,
            _head_startup_current_mode=None, _head_startup_attempts=0,
            _head_startup_target="vr_follow", _head_startup_track_target="right_hand")

    def mode(self, mode):
        self.receive(self.node, SimpleNamespace(mode=mode))

    def test_unknown_state_never_calls_service(self):
        self.tick(self.node, None)
        self.client.assert_not_called()
        self.ros.wait_for_service.assert_not_called()

    def test_fixed_switches_once_even_with_repeated_status(self):
        self.mode("fixed")
        self.tick(self.node, None)
        self.mode("fixed")
        self.tick(self.node, None)
        self.client.assert_called_once_with(mode="vr_follow", fixed_hand="")

    def test_existing_modes_and_later_fixed_are_preserved(self):
        for mode in ("vr_follow", "right_hand", "active_hand", "both_hands_midpoint", "track_hand"):
            with self.subTest(mode=mode):
                self.node._head_startup_pending = True
                self.mode(mode)
                self.tick(self.node, None)
                self.mode("fixed")
                self.tick(self.node, None)
        self.client.assert_not_called()

    def test_mode_changes_while_waiting_do_not_call_service(self):
        self.mode("fixed")
        self.ros.wait_for_service.side_effect = lambda *args, **kwargs: self.mode("right_hand")
        self.tick(self.node, None)
        self.client.assert_not_called()

    def test_service_unavailable_retries_then_succeeds(self):
        self.mode("fixed")
        self.ros.wait_for_service.side_effect = [ServiceUnavailable("not ready"), None]
        self.tick(self.node, None)
        self.assertTrue(self.node._head_startup_pending)
        self.tick(self.node, None)
        self.client.assert_called_once()
        self.assertFalse(self.node._head_startup_pending)

    def test_retries_are_bounded(self):
        self.mode("fixed")
        self.ros.wait_for_service.side_effect = ServiceUnavailable("not ready")
        for _ in range(12):
            self.tick(self.node, None)
        self.assertEqual(self.ros.wait_for_service.call_count, 10)
        self.assertFalse(self.node._head_startup_pending)

    def test_rejected_mode_is_not_repeated(self):
        self.client.return_value = SimpleNamespace(success=False, message="invalid mode")
        self.mode("fixed")
        self.tick(self.node, None)
        self.tick(self.node, None)
        self.client.assert_called_once()
        self.assertFalse(self.node._head_startup_pending)

    def test_secondary_monitor_never_switches(self):
        self.node._head_startup_pending = False
        self.mode("fixed")
        self.tick(self.node, None)
        self.client.assert_not_called()

    def test_track_target_is_passed_to_service(self):
        self.node._head_startup_target = "track_hand"
        self.mode("fixed")
        self.tick(self.node, None)
        self.client.assert_called_once_with(mode="track_hand", fixed_hand="right_hand")


class LaunchDefaultsTest(unittest.TestCase):
    def test_empty_head_startup_mode_resolves_to_fixed(self):
        source = ROOT / "scripts/head_control_node.py"
        tree = ast.parse(source.read_text())
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "HeadTrackHandNode")
        init = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "__init__")
        assignment = next(n for n in init.body if isinstance(n, ast.Assign)
                          and any(isinstance(t, ast.Name) and t.id == "mode_name" for t in n.targets))
        for value, expected in (("", "fixed"), (None, "fixed"), ("  ", "fixed"),
                                ("track_hand", "track_hand"), ("vr_follow", "vr_follow")):
            with self.subTest(value=value):
                scope = {"rospy": SimpleNamespace(get_param=lambda *args: value)}
                exec(compile(ast.Module(body=[assignment], type_ignores=[]), str(source), "exec"), scope)
                self.assertEqual(scope["mode_name"], expected)

    def test_machine_and_vr_launch_defaults(self):
        machine_launch = ROOT.parents[1] / "humanoid-control/humanoid_controllers/launch"
        for name in ("load_kuavo_real", "load_kuavo_mujoco_sim", "load_kuavo_real_wheel",
                     "load_kuavo_mujoco_sim_wheel", "load_kuavo_real_wheel_vr", "load_kuavo_mujoco_sim_wheel_vr"):
            arg = ET.parse(machine_launch / (name + ".launch")).getroot().find("arg[@name='head_control_mode']")
            self.assertEqual(arg.get("default"), "")
        for path in (ROOT / "launch").glob("*.launch"):
            if 'type="monitor_quest3.py"' in path.read_text():
                arg = ET.parse(path).getroot().find("arg[@name='head_control_mode']")
                self.assertEqual(arg.get("default"), "vr_follow", str(path))


if __name__ == "__main__":
    unittest.main()
