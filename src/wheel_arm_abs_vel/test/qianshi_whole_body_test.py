#!/usr/bin/env python3
"""Whole-body launch and glove input checks without starting ROS nodes."""

import sys
import unittest
from pathlib import Path

import roslaunch
from kuavo_msgs.msg import JoySticks

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from glove_quest_input import GloveQuestInput


def load(*args, version='400062', launch='quest_velocity_ik.launch'):
    config = roslaunch.config.ROSLaunchConfig()
    roslaunch.xmlloader.XmlLoader().load(
        str(ROOT / 'launch' / launch), config,
        argv=['robot_version:=' + version, *args], verbose=False)
    return config


class QianshiLaunchTest(unittest.TestCase):
    def test_quest_only_has_no_glove_nodes(self):
        config = load()
        self.assertTrue(any(node.name == 'sg100_mapper_bridge' for node in config.nodes))
        self.assertFalse(any(node.package == 'sg100_glove_bridge' for node in config.nodes))

    def test_single_backend_keeps_whole_body_solver_monitor_and_fsm(self):
        for backend in ('linux_sdk', 'windows_udp'):
            for mode in ('incremental', 'absolute'):
                with self.subTest(backend=backend, mode=mode):
                    config = load('hand_control_source:=qianshi', 'teleop_mode:=' + mode,
                                  'qianshi_backend:=' + backend, 'qianshi_use_mujoco:=true',
                                  'qianshi_udp_left_module_map:=[C,B,A,F,E]',
                                  'qianshi_left_id:=17', 'qianshi_right_id:=18')
                    nodes = {node.name: node for node in config.nodes}
                    self.assertEqual(len(nodes), len(config.nodes))
                    self.assertEqual(nodes['absolute_velocity_ik'].type,
                                     'pinocchio_incremental_velocity_ik_node' if mode == 'incremental'
                                     else 'pinocchio_velocity_ik_node')
                    self.assertEqual(nodes['ik_ros_uni'].type, 'run_incremental_velocity.py'
                                     if mode == 'incremental' else 'ik_ros_velocity.py')
                    self.assertEqual(nodes['humanoid_quest_control_with_arm'].type,
                                     'humanoid_quest_absolute_control')
                    self.assertEqual(nodes['monitor_quest3'].type, 'monitor_quest3_velocity.py')
                    self.assertIn('glove_gesture_node', nodes)
                    self.assertIn('sg100_mujoco_viewer', nodes)
                    self.assertNotIn('sg100_mapper_bridge', nodes)
                    self.assertFalse(config.params['/ik_ros_uni/sg100_enabled'].value)
                    self.assertTrue(config.params['/monitor_quest3/qianshi_gestures'].value)
                    if backend == 'linux_sdk':
                        self.assertNotIn('sg100_windows_udp', nodes)
                        self.assertTrue(nodes['sg100_glove_both'].launch_prefix.endswith(' glove_sdk'))
                        self.assertEqual(config.params['/sg100_glove_both/left_id'].value, 17)
                        self.assertEqual(config.params['/sg100_glove_both/right_id'].value, 18)
                    else:
                        self.assertNotIn('sg100_glove_both', nodes)
                        self.assertTrue(nodes['sg100_windows_udp'].launch_prefix.endswith(' background'))
                        self.assertEqual(config.params['/sg100_windows_udp/left_module_map'].value,
                                         list('CBAFE'))

    def test_windows_preset_matches_explicit_whole_body_configuration(self):
        def snapshot(config):
            nodes = [tuple(getattr(node, field) for field in node.__slots__
                           if field not in ('filename', 'machine'))
                     for node in config.nodes]
            return nodes, {key: value.value for key, value in config.params.items()}

        for version in ('400062', '400063', '410062', '410063'):
            for mode in ('incremental', 'absolute'):
                with self.subTest(version=version, mode=mode):
                    args = ['ip_address:=192.0.2.20', 'teleop_mode:=' + mode]
                    preset = load(*args, version=version, launch='quest_windows_udp.launch')
                    explicit = load(
                        *args, 'hand_control_source:=qianshi', 'qianshi_backend:=windows_udp',
                        'qianshi_hand_side:=both', 'qianshi_glove_gesture_enable:=true',
                        'qianshi_udp_left_module_map:=[A,B,C,D,E]',
                        'qianshi_udp_right_module_map:=[A,B,C,D,E]', version=version)
                    self.assertEqual(snapshot(preset), snapshot(explicit))
                    nodes = {node.name: node for node in preset.nodes}
                    self.assertTrue(nodes['sg100_windows_udp'].launch_prefix.endswith(' background'))
                    self.assertTrue(nodes['glove_gesture_node'].launch_prefix.endswith(' background'))
                    self.assertEqual(preset.params['/sg100_windows_udp/right_port'].value, 18002)
                    self.assertEqual(preset.params['/sg100_windows_udp/left_port'].value, 18003)
                    self.assertTrue(preset.params['/sg100_windows_udp/emit_gesture_poses'].value)
                    self.assertTrue(preset.params['/monitor_quest3/qianshi_gestures'].value)
                    self.assertNotIn('sg100_mapper_bridge', nodes)
                    self.assertNotIn('sg100_glove_both', nodes)

    def test_windows_preset_defaults_and_explicit_overrides(self):
        config = load('ip_address:=192.0.2.20', launch='quest_windows_udp.launch')
        nodes = {node.name: node for node in config.nodes}
        self.assertEqual(nodes['absolute_velocity_ik'].type, 'pinocchio_velocity_ik_node')
        self.assertEqual(nodes['ik_ros_uni'].type, 'ik_ros_velocity.py')
        for side in ('left', 'right'):
            self.assertEqual(config.params['/sg100_windows_udp/' + side + '_module_map'].value,
                             list('ABCDE'))
        for flag in ('qianshi_hand_side:=left', 'qianshi_hand_side:=right',
                     'qianshi_glove_gesture_enable:=false'):
            with self.subTest(flag=flag):
                config = load('ip_address:=192.0.2.20', flag,
                              'qianshi_udp_right_port:=28002', 'qianshi_udp_left_port:=28003',
                              'qianshi_udp_left_module_map:=[C,B,A,F,E]',
                              launch='quest_windows_udp.launch')
                self.assertFalse(any(node.name == 'glove_gesture_node' for node in config.nodes))
                self.assertFalse(config.params['/monitor_quest3/qianshi_gestures'].value)
                self.assertEqual(config.params['/sg100_windows_udp/right_port'].value, 28002)
                self.assertEqual(config.params['/sg100_windows_udp/left_port'].value, 28003)
                self.assertEqual(config.params['/sg100_windows_udp/left_module_map'].value, list('CBAFE'))
        with self.assertRaises(roslaunch.xmlloader.XmlParseException):
            load(launch='quest_windows_udp.launch')

    def test_bad_backend_input_and_non_sg100_model_rejected_before_launch(self):
        for flag in ('qianshi_backend:=wrong', 'qianshi_hand_side:=wrong',
                     'use_cpp_monitor:=true', 'input_source:=pico',
                     'enable_velocity_ik_takeover:=false', 'tracking_source_mode:=hand'):
            with self.subTest(flag=flag), self.assertRaises(roslaunch.xmlloader.XmlParseException):
                load('hand_control_source:=qianshi', flag)
        with self.assertRaises(roslaunch.xmlloader.XmlParseException):
            load('hand_control_source:=qianshi', version='62')

    def test_single_hand_selects_one_backend_without_glove_gesture_injection(self):
        for backend in ('linux_sdk', 'windows_udp'):
            for side in ('left', 'right'):
                with self.subTest(backend=backend, side=side):
                    config = load('hand_control_source:=qianshi', 'qianshi_backend:=' + backend,
                                  'qianshi_hand_side:=' + side)
                    glove_nodes = [n for n in config.nodes if n.package == 'sg100_glove_bridge']
                    expected_name = ('sg100_glove_' + side if backend == 'linux_sdk'
                                     else 'sg100_windows_udp')
                    self.assertEqual([n.name for n in glove_nodes], [expected_name])
                    self.assertEqual(config.params['/' + expected_name + '/side'].value, side)
                    self.assertFalse(config.params['/monitor_quest3/qianshi_gestures'].value)
                    self.assertTrue(config.params['/ik_ros_uni/qianshi_enabled'].value)


class GloveInputTest(unittest.TestCase):
    def setUp(self):
        self.states = []
        self.input = GloveQuestInput(0.5, self.states.append)

    def test_pinch_requires_interaction_and_preserves_physical_input(self):
        self.input.on_buttons(0x05, 0.0)
        self.assertFalse(self.input.apply(JoySticks(), 0.01).right_first_button_pressed)
        self.input.on_buttons(0x10, 0.02)
        self.input.on_buttons(0x10, 0.03)
        self.input.on_buttons(0x05, 0.04)
        joy = self.input.apply(JoySticks(left_y=0.25, right_grip=0.85), 0.05)
        self.assertTrue(joy.left_first_button_pressed and joy.right_first_button_pressed)
        self.assertTrue(joy.left_first_button_touched and joy.right_first_button_touched)
        self.assertEqual((joy.left_y, joy.right_grip), (0.25, 0.85))
        self.input.expire(0.55)
        self.assertEqual(self.states, [True, False])
        self.assertFalse(self.input.apply(JoySticks(), 0.56).right_first_button_pressed)

    def test_physical_right_action_cancels_synthetic_stop_hold(self):
        self.input.on_buttons(0x10, 0.0)
        self.input.on_buttons(0x03, 0.1)
        joy = self.input.apply(JoySticks(right_first_button_pressed=True), 0.35)
        self.assertTrue(joy.right_first_button_pressed)
        self.assertFalse(joy.left_second_button_pressed)
        self.input.on_buttons(0x03, 0.36)
        self.assertFalse(self.input.apply(JoySticks(), 0.37).left_second_button_pressed)
        self.input.on_buttons(0x03, 0.57)
        self.assertTrue(self.input.apply(JoySticks(), 0.58).left_second_button_pressed)

    def test_overlapping_glove_pinches_keep_xa_without_stop_chord(self):
        self.input.on_buttons(0x10, 0.0)
        self.input.on_buttons(0x07, 0.1)  # glove X + Y + A
        joy = self.input.apply(JoySticks(), 0.11)
        self.assertTrue(joy.left_first_button_pressed)
        self.assertFalse(joy.left_second_button_pressed)
        self.assertFalse(joy.left_second_button_touched)
        self.assertTrue(joy.right_first_button_pressed)
        self.input.on_buttons(0x02, 0.2)  # Y alone remains available
        joy = self.input.apply(JoySticks(), 0.21)
        self.assertTrue(joy.left_second_button_pressed)
        self.assertFalse(joy.left_first_button_pressed)

    def test_glove_stop_chord_requires_a_continuous_hold(self):
        self.input.on_buttons(0x10, 0.0)
        self.input.on_buttons(0x03, 0.1)
        short = self.input.apply(JoySticks(), 0.29)
        self.assertFalse(short.left_first_button_pressed)
        self.assertFalse(short.left_second_button_pressed)
        held = self.input.apply(JoySticks(), 0.31)
        self.assertTrue(held.left_first_button_pressed)
        self.assertTrue(held.left_second_button_pressed)

    def test_right_action_cancels_and_restarts_glove_stop_hold(self):
        self.input.on_buttons(0x10, 0.0)
        self.input.on_buttons(0x03, 0.1)
        self.input.on_buttons(0x07, 0.25)  # X + Y + A prefers X + A.
        xa = self.input.apply(JoySticks(), 0.26)
        self.assertTrue(xa.left_first_button_pressed)
        self.assertFalse(xa.left_second_button_pressed)
        self.assertTrue(xa.right_first_button_pressed)
        self.input.on_buttons(0x03, 0.3)
        restarted = self.input.apply(JoySticks(), 0.49)
        self.assertFalse(restarted.left_first_button_pressed)
        self.assertFalse(restarted.left_second_button_pressed)
        held = self.input.apply(JoySticks(), 0.51)
        self.assertTrue(held.left_first_button_pressed)
        self.assertTrue(held.left_second_button_pressed)

    def test_xyb_prefers_xb_and_never_arms_stop(self):
        self.input.on_buttons(0x10, 0.0)
        self.input.on_buttons(0x0b, 0.1)
        result = self.input.apply(JoySticks(), 0.4)
        self.assertTrue(result.left_first_button_pressed)
        self.assertFalse(result.left_second_button_pressed)
        self.assertTrue(result.right_second_button_pressed)

    def test_stop_hold_resets_on_timeout_and_interaction_exit(self):
        self.input.on_buttons(0x10, 0.0)
        self.input.on_buttons(0x03, 0.1)
        self.input.expire(0.61)
        self.input.on_buttons(0x10, 0.7)
        self.input.on_buttons(0x03, 0.8)
        after_timeout = self.input.apply(JoySticks(), 0.99)
        self.assertFalse(after_timeout.left_first_button_pressed)
        self.input.on_buttons(0x10, 1.0)
        self.input.on_buttons(0x10, 1.01)
        self.input.on_buttons(0x03, 1.02)
        after_exit = self.input.apply(JoySticks(), 1.21)
        self.assertFalse(after_exit.left_first_button_pressed)

    def test_glove_cannot_complete_a_stop_chord_with_one_physical_button(self):
        self.input.on_buttons(0x10, 0.0)
        for physical, other, buttons in (('left_first', 'left_second', 0x02),
                                         ('left_second', 'left_first', 0x01)):
            with self.subTest(physical=physical):
                self.input.on_buttons(buttons, 0.1)
                joy = JoySticks()
                setattr(joy, physical + '_button_pressed', True)
                result = self.input.apply(joy, 0.11)
                self.assertTrue(getattr(result, physical + '_button_pressed'))
                self.assertFalse(getattr(result, other + '_button_pressed'))
                self.assertFalse(getattr(result, other + '_button_touched'))

    def test_physical_stop_chord_is_preserved_during_glove_interaction(self):
        self.input.on_buttons(0x10, 0.0)
        self.input.on_buttons(0x07, 0.1)
        joy = JoySticks(left_first_button_pressed=True, left_second_button_pressed=True,
                       left_first_button_touched=True, left_second_button_touched=True)
        result = self.input.apply(joy, 0.11)
        self.assertTrue(result.left_first_button_pressed and result.left_second_button_pressed)
        self.assertTrue(result.left_first_button_touched and result.left_second_button_touched)

    def test_fist_expires_and_first_ack_releases_interaction(self):
        self.input.on_buttons(0x10, 0.0)
        self.input.on_buttons(0x20, 0.1)
        self.input.on_buttons(0, 0.11)
        self.assertEqual(self.input.apply(JoySticks(), 0.12).right_trigger, 1.0)
        self.assertEqual(self.input.apply(JoySticks(), 0.26).right_trigger, 0.0)
        self.input.on_buttons(0x20, 0.28)
        self.assertEqual(self.input.apply(JoySticks(), 0.30).left_trigger, 1.0)
        self.input.on_following(True)
        self.input.on_buttons(0x10, 0.32)
        self.input.on_following(True)
        self.input.on_buttons(0x20, 0.35)
        self.assertEqual(self.input.apply(JoySticks(), 0.36).left_trigger, 0.0)
        self.assertEqual(self.states, [True, False, True])
        self.input.on_following(False)
        self.input.on_following(True)
        self.assertEqual(self.states, [True, False, True, False])


if __name__ == '__main__':
    unittest.main()
