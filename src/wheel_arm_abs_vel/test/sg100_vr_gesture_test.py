#!/usr/bin/env python3
import ast
import importlib.util
import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS, ModuleType, MethodType
from unittest.mock import patch

source = Path(__file__).resolve().parents[1] / 'scripts/tools/sg100_vr_gesture.py'
spec = importlib.util.spec_from_file_location('sg100_vr_gesture', source)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

incremental_source = source.parents[2] / 'incremental_velocity/scripts/tools/sg100_vr_gesture.py'
incremental_spec = importlib.util.spec_from_file_location('incremental_sg100_vr_gesture', incremental_source)
incremental_module = importlib.util.module_from_spec(incremental_spec)
incremental_spec.loader.exec_module(incremental_module)


def packet(**changes):
    values = dict(left_first_button_touched=False, left_first_button_pressed=False,
                  left_second_button_touched=True, left_second_button_pressed=False,
                  right_first_button_touched=True, right_first_button_pressed=False,
                  right_second_button_touched=False, right_second_button_pressed=False,
                  left_x=0., left_y=0., right_x=0., right_y=0.,
                  left_trigger=0., right_trigger=0.)
    values.update(changes)
    return NS(**values)


def released():
    return packet(left_second_button_touched=False, right_first_button_touched=False)


class LegacyGate:
    """Run the actual legacy callback without importing ROS or commanding hardware."""
    def __init__(self):
        path = source.parents[3] / 'manipulation_nodes/motion_capture_ik/scripts/ik_ros_uni.py'
        tree = ast.parse(path.read_text())
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'IkRos')
        fn = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '_check_sg100_gesture_switch')
        self.now = 0.
        env = dict(math=math, time=NS(monotonic=lambda: self.now),
                   rospy=NS(loginfo=lambda *a: None, ServiceException=RuntimeError))
        exec(compile(ast.Module(body=[fn], type_ignores=[]), str(path), 'exec'), env)
        self.fn = env[fn.name]
        self._sg100_pending_dir = 0
        self._sg100_debounce = False
        self._sg100_confirm_sec = .8
        self.result = 0

    def _sg100_step(self, direction, side):
        self.result = direction
        return NS(success=True, current_id=1)

    def update(self, msg, valid, now):
        self.now, self.result = now, 0
        self.fn(self, msg)
        return self.result


class GestureTouchTest(unittest.TestCase):
    def gates(self):
        return (module.GestureTouchGate(), incremental_module.GestureTouchGate(), LegacyGate())

    def hold(self, gate, msg, start=0):
        return [gate.update(msg, True, start+i*.1) for i in range(10)]

    def test_next_and_previous_after_800ms(self):
        for msg, direction in ((packet(), 1), (packet(right_first_button_touched=False,
                right_second_button_touched=True), -1)):
            for gate in self.gates():
                out = self.hold(gate, msg)
                self.assertEqual(out[:8], [0]*8)
                self.assertEqual(sum(out), direction)

    def test_held_touch_and_partial_release_do_not_rearm(self):
        for gate in self.gates():
            self.hold(gate, packet())
            self.assertEqual(sum(self.hold(gate, packet(), 1)), 0)
            self.hold(gate, packet(left_second_button_touched=False), 2)
            self.assertEqual(sum(self.hold(gate, packet(right_first_button_touched=False,
                right_second_button_touched=True), 3)), 0)
            gate.update(released(), True, 4)
            self.assertEqual(sum(self.hold(gate, packet(), 4.1)), 1)

    def test_any_press_x_touch_or_ambiguous_touch_blocks_until_full_release(self):
        for field in ('left_first_button_pressed', 'left_second_button_pressed',
                      'right_first_button_pressed', 'right_second_button_pressed',
                      'left_first_button_touched', 'right_second_button_touched'):
            for gate in self.gates():
                with self.subTest(field=field, gate=type(gate).__name__):
                    gate.update(packet(), True, 0)
                    self.assertEqual(sum(self.hold(gate, packet(**{field: True}), .1)), 0)
                    self.assertEqual(sum(self.hold(gate, packet(), 1.1)), 0)
                    gate.update(released(), True, 2.1)
                    self.assertEqual(sum(self.hold(gate, packet(), 2.2)), 1)

    def test_old_x_combo_never_switches(self):
        for gate in self.gates():
            msg = packet(left_first_button_touched=True, left_second_button_touched=False)
            self.assertEqual(sum(self.hold(gate, msg)), 0)

    def test_invalid_or_stale_input_requires_release(self):
        for valid in (False, True):
            gate = module.GestureTouchGate()
            gate.update(packet(), True, 0)
            gate.update(packet(), valid, .4)
            self.assertEqual(sum(self.hold(gate, packet(), .5)), 0)
            gate.update(released(), True, 1.5)
            self.assertEqual(sum(self.hold(gate, packet(), 1.6)), 1)

    def test_direction_change_before_confirmation_restarts_timer(self):
        for gate in self.gates():
            for i in range(5):
                self.assertEqual(gate.update(packet(), True, i*.1), 0)
            previous = packet(right_first_button_touched=False, right_second_button_touched=True)
            self.assertEqual(sum(self.hold(gate, previous, .5)), -1)

    def test_axis_movement_or_invalid_axis_cancels_until_release(self):
        for axis in ('left_x', 'left_y', 'right_x', 'right_y'):
            for value in (.11, -.11, float('nan'), float('inf')):
                for gate in self.gates():
                    with self.subTest(axis=axis, value=value, gate=type(gate).__name__):
                        gate.update(packet(), True, 0)
                        self.assertEqual(sum(self.hold(gate, packet(**{axis: value}), .1)), 0)
                        self.assertEqual(sum(self.hold(gate, packet(), 1.1)), 0)
                        gate.update(released(), True, 2.1)
                        self.assertEqual(sum(self.hold(gate, packet(), 2.2)), 1)

    def test_neutral_deadzone_allows_small_axis_noise(self):
        for gate in self.gates():
            self.assertEqual(sum(self.hold(gate, packet(left_x=.1, left_y=-.1,
                                                      right_x=.05, right_y=-.05))), 1)


class LegacyHandRoutingTest(unittest.TestCase):
    def make_node(self, robot_type, end_effector_type):
        path = source.parents[3] / 'manipulation_nodes/motion_capture_ik/scripts/ik_ros_uni.py'
        tree = ast.parse(path.read_text())
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'IkRos')
        init = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '__init__')
        start = next(i for i, n in enumerate(init.body) if isinstance(n, ast.Assign)
                     and any(isinstance(t, ast.Attribute) and t.attr == '_sg100_abs_vr'
                             for t in n.targets))
        end = next(i for i in range(start, len(init.body)) if isinstance(init.body[i], ast.If)
                   and any(isinstance(n, ast.Name) and n.id == 'HEIMAN'
                           for n in ast.walk(init.body[i].test)))
        messages, requests = [], []
        def step(direction, side):
            requests.append((direction, side))
            return NS(success=True, current_id=3)
        ros = NS(Publisher=lambda *a, **kw: NS(publish=messages.append),
                 ServiceProxy=lambda *a: step, loginfo=lambda *a: None,
                 logerr=lambda *a: None, ServiceException=RuntimeError)
        node = NS(robot_type=robot_type, end_effector_type=end_effector_type)
        env = dict(self=node, HEIMAN='heiman', rospy=ros, math=math,
                   Float32MultiArray=lambda: NS(data=[]), time=NS(monotonic=lambda: node.now))
        srv = ModuleType('kuavo_msgs.srv')
        srv.SG100StepGesture = object
        pkg = ModuleType('kuavo_msgs')
        pkg.srv = srv
        with patch.dict(sys.modules, {'kuavo_msgs': pkg, 'kuavo_msgs.srv': srv}):
            exec(compile(ast.Module(body=init.body[start:end+1], type_ignores=[]), str(path), 'exec'), env)
        for name in ('pub_robot_end_hand', '_publish_sg100_from_joystick', '_check_sg100_gesture_switch'):
            fn = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == name)
            exec(compile(ast.Module(body=[fn], type_ignores=[]), str(path), 'exec'), env)
            setattr(node, name, MethodType(env[name], node))
        vars(node).update(__freeze_finger=False, __button_y_last=False, now=0.)
        return node, messages, requests

    def test_humanoid_and_wheel_enable_trigger_and_gesture_route(self):
        for robot_type in (0, 1):
            with self.subTest(robot_type=robot_type):
                node, messages, requests = self.make_node(robot_type, 'heiman')
                self.assertTrue(node._sg100_abs_vr)
                for i in range(10):
                    node.now = i*.1
                    node.pub_robot_end_hand(packet(left_trigger=.25, right_trigger=.75))
                self.assertEqual(len(messages), 10)
                self.assertEqual(messages[-1].data, [.25, .75])
                self.assertEqual(requests, [(1, 2)])
                node.pub_robot_end_hand(packet(left_trigger=-.2, right_trigger=1.2))
                self.assertEqual(messages[-1].data, [0., 1.])

    def test_other_end_effectors_do_not_enable_sg100_route(self):
        for robot_type in (0, 1):
            node, messages, requests = self.make_node(robot_type, 'qiangnao')
            self.assertFalse(node._sg100_abs_vr)
            self.assertIsNone(node._sg100_trigger_pub)
            self.assertIsNone(node._sg100_step)
            self.assertEqual(messages, [])
            self.assertEqual(requests, [])


if __name__ == '__main__':
    unittest.main()
