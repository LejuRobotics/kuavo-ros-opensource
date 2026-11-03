#!/usr/bin/env python3
"""ROS wire conversion, lifecycle and launch parity; no hardware or ROS master."""
import sys
from pathlib import Path
import unittest
from unittest.mock import Mock, patch
from types import SimpleNamespace
import queue
import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / 'scripts'))
import legacy_dependencies
import roslaunch
from tf.transformations import euler_matrix, quaternion_matrix
from noitom_hi5_hand_udp_python.msg import PoseInfoList
from monitor_pico import PicoQuestReceiver, quest_event
from monitor_quest3_velocity import Quest3BoneFramePublisher
from pico_input.adapter import PoseAdapter, BONES, P, QUEST_CHEST_REFERENCE
from pico_input import body_tracking_extended_pb2 as proto
from quest_input_buffer import SourceClockGate, LatestSample


def body_packet(seconds=100):
    msg = proto.VRData()
    msg.header.timestamp.seconds = seconds
    for i in range(24):
        p = msg.full_body.full_body.add(rot_qw=1.)
        # Nondegenerate upper arms/forearms, including across hand offsets.
        p.pos_x = i * .08
        p.pos_y = 1.
        p.pos_z = .1
    for side in ('left', 'right'):
        getattr(msg.controller.controllers, side).thumbstick.extend([0., 0.])
    return msg


class PicoParityTest(unittest.TestCase):
    def setUp(self):
        patcher=patch("monitor_pico.rospy.logwarn_throttle")
        patcher.start();self.addCleanup(patcher.stop)

    def receiver(self):
        r = PicoQuestReceiver.__new__(PicoQuestReceiver)
        r.adapter = PoseAdapter()
        r.peer = ''
        r.connected = False
        r.feedback = SimpleNamespace(peer=None)
        r._pose_mailbox = LatestSample()
        r._aux_mailbox = LatestSample()
        r._control_requests = queue.Queue(maxsize=32)
        r.joysticks_pub = Mock()
        r.gates = {k: SourceClockGate() for k in ('body', 'joy', 'command')}
        r.last_fresh = {k: 0. for k in r.gates}
        r.require_button_release = False
        return r

    def test_discovery_without_legacy_head_manager(self):
        import threading
        r = self.receiver()
        r._input_max_age = .15
        r._discovery_stop = threading.Event()
        r.advertiser = Mock()
        r.discovery_pairs = [("192.0.2.1", "192.0.2.255")]
        with patch("monitor_pico.rospy.is_shutdown", return_value=False), \
             patch("monitor_pico.rospy.get_param", return_value="qiangnao"), \
             patch("monitor_pico.announce", side_effect=lambda *a: r._discovery_stop.set()) as announce:
            r._publish_discovery()
        announce.assert_called_once()

    def test_shared_conversion_preserves_arm_and_chest_and_adds_head(self):
        adapter = PoseAdapter()
        body = adapter.from_full_body(body_packet().full_body.full_body)
        self.assertEqual(len(body), 26)
        converted = PoseInfoList()
        receiver = Quest3BoneFramePublisher.__new__(Quest3BoneFramePublisher)
        receiver.process_pose_data(quest_event(body, 100000000000), converted, None)
        for expected, pose in zip(body, converted.poses):
            np.testing.assert_allclose([pose.position.x, pose.position.y, pose.position.z], expected[:3, 3], atol=1e-12)
            q = pose.orientation
            np.testing.assert_allclose(quaternion_matrix([q.x, q.y, q.z, q.w])[:3,:3], expected[:3,:3], atol=1e-12)
        # Regression against the pre-head-extension conversion, using only its slots.
        matrices = {k: np.eye(4) for k in BONES}
        for k in matrices: matrices[k][:3, 3] = body[k][:3, 3]
        original = adapter.from_robot_matrices(matrices)
        matrices[24] = np.eye(4)
        matrices[25] = np.eye(4)
        with_head = adapter.from_robot_matrices(matrices)
        np.testing.assert_allclose(original, with_head[:24])

    def test_head_relative_rotation_and_global_body_rotation(self):
        adapter = PoseAdapter()
        packet = body_packet()
        base = adapter.from_full_body(packet.full_body.full_body)
        matrices = {k: np.eye(4) for k in BONES}
        for k in matrices: matrices[k][:3, 3] = base[k][:3, 3]
        matrices[24] = np.eye(4)
        matrices[25] = euler_matrix(0, 0, .2)
        body = adapter.from_robot_matrices(matrices)
        relative = body[23,:3,:3].T @ body[25,:3,:3]
        expected = QUEST_CHEST_REFERENCE.T @ matrices[25][:3,:3] @ QUEST_CHEST_REFERENCE
        np.testing.assert_allclose(relative, expected, atol=1e-12)
        world = euler_matrix(.4, -.1, .3)
        moved = adapter.from_robot_matrices({k:world@v for k,v in matrices.items()})
        np.testing.assert_allclose(moved[23,:3,:3].T @ moved[25,:3,:3], relative, atol=1e-12)

    def test_invalid_head_preserves_valid_arms(self):
        packet = body_packet()
        packet.full_body.full_body[15].rot_qw = 0.
        body = PoseAdapter().from_full_body(packet.full_body.full_body)
        self.assertEqual(len(body), 24)

    @patch('monitor_pico.rospy.set_param')
    @patch('monitor_pico.rospy.Time.now')
    def test_edges_separate_channels_and_no_invented_touch(self, now, set_param):
        r = self.receiver()
        packet = body_packet()
        packet.controller.controllers.left.primary_button = True
        r.handle(packet.SerializeToString(), ('127.0.0.1', 5555), 1.)
        self.assertTrue(r.joysticks_pub.publish.call_args[0][0].left_first_button_pressed)
        self.assertFalse(r.joysticks_pub.publish.call_args[0][0].left_first_button_touched)
        packet.ClearField('controller')
        packet.header.timestamp.nanos = 10000000
        r.handle(packet.SerializeToString(), ('127.0.0.1', 5555), 1.01)
        self.assertEqual(r.joysticks_pub.publish.call_count, 1)  # Body packet is not a button release.
        self.assertEqual(len(r._pose_mailbox.take()[0].poses), 26)

    @patch('monitor_pico.rospy.set_param')
    @patch('monitor_pico.rospy.Time.now')
    def test_stale_input_requires_release_before_reenable(self, now, set_param):
        r = self.receiver()
        packet = body_packet()
        r.handle(packet.SerializeToString(), ('127.0.0.1', 5555), 1.)
        packet.header.timestamp.seconds = 99
        packet.controller.controllers.right.primary_button = True
        r.handle(packet.SerializeToString(), ('127.0.0.1', 5555), 1.01)
        self.assertFalse(r.connected)
        self.assertIsNone(r._pose_mailbox.take(0))
        packet.header.timestamp.seconds = 101
        r.handle(packet.SerializeToString(), ('127.0.0.1', 5555), 2.)
        self.assertFalse(r.joysticks_pub.publish.call_args[0][0].right_first_button_pressed)
        packet.controller.controllers.right.primary_button = False
        packet.header.timestamp.nanos = 10000000
        r.handle(packet.SerializeToString(), ('127.0.0.1', 5555), 2.01)
        packet.controller.controllers.right.primary_button = True
        packet.header.timestamp.nanos = 20000000
        r.handle(packet.SerializeToString(), ('127.0.0.1', 5555), 2.02)
        self.assertTrue(r.joysticks_pub.publish.call_args[0][0].right_first_button_pressed)

    @patch('monitor_pico.rospy.set_param')
    @patch('monitor_pico.rospy.Time.now')
    def test_body_disconnect_preserves_joy_and_reconnect_does_not_invent_neutral(self, now, set_param):
        r=self.receiver();packet=body_packet()
        packet.controller.controllers.left.thumbstick[1]=1.
        r.handle(packet.SerializeToString(),('127.0.0.1',5555),1.)
        calls=r.joysticks_pub.publish.call_count
        r._disconnect('body')
        self.assertEqual(r.joysticks_pub.publish.call_count,calls)
        packet.ClearField('full_body');packet.header.timestamp.nanos=10000000
        r.handle(packet.SerializeToString(),('127.0.0.1',5555),1.01)
        self.assertEqual(r.joysticks_pub.publish.call_args[0][0].left_y,1.)
        r._disconnect('joy')
        self.assertTrue(np.isnan(r.joysticks_pub.publish.call_args[0][0].left_y))
        packet.controller.controllers.right.primary_button=True
        packet.header.timestamp.nanos=20000000
        r.handle(packet.SerializeToString(),('127.0.0.1',5555),1.02)
        result=r.joysticks_pub.publish.call_args[0][0]
        self.assertEqual(result.left_y,1.)
        self.assertFalse(result.right_first_button_pressed)

    def test_thumb_long_press_single_fire_and_chord_priority(self):
        from pico_input.thumb_buttons import ThumbButtons
        from kuavo_msgs.msg import JoySticks
        state = ThumbButtons()
        msg = JoySticks(left_first_button_pressed=True)
        for i in range(6): result = state.update(msg, i*.1)
        self.assertTrue(result.left_first_button_touched)
        self.assertFalse(msg.left_first_button_touched)  # Actual input never rewritten.
        for i in range(6, 20): self.assertTrue(state.update(msg, i*.1).left_first_button_touched)
        state.update(JoySticks(), 2.)
        for i in range(21, 27): result = state.update(msg, i*.1)
        self.assertFalse(result.left_first_button_touched)
        state.update(JoySticks(), 2.7)
        chord = JoySticks(left_first_button_pressed=True, right_first_button_pressed=True)
        for i in range(28, 40): result = state.update(chord, i*.1)
        self.assertFalse(result.left_first_button_touched)
        self.assertFalse(result.right_first_button_touched)
        for i in range(40, 50): result = state.update(msg, i*.1)
        self.assertFalse(result.left_first_button_touched)  # Partial chord release is not a new press.

    def test_thumb_disconnection_and_initialization_do_not_toggle(self):
        from pico_input.thumb_buttons import ThumbButtons
        from kuavo_msgs.msg import JoySticks
        msg = JoySticks(right_first_button_pressed=True)
        state = ThumbButtons()
        for i in range(10): result = state.update(msg, i*.1, enabled=False)
        self.assertFalse(result.right_first_button_touched)
        for i in range(10, 20): result = state.update(msg, i*.1, enabled=True)
        self.assertFalse(result.right_first_button_touched)
        state.update(JoySticks(), 2.)
        state.update(msg, 2.1)
        self.assertFalse(state.update(msg, 3.).right_first_button_touched)
        for i in range(31, 40): result = state.update(msg, i*.1)
        self.assertFalse(result.right_first_button_touched)

    def test_launch_control_parameters_and_overrides_are_equal(self):
        for version in (62, 63, 400062):
            args = ['robot_version:='+str(version), 'ee_type:=none', 'enable_terminal_start:=false',
                    'tracking_source_mode:=controller',
                    'hand_reference_frequency:=9.0', 'enable_head_control:=true',
                    'absolute_enable_mode_timeout:=4.0']
            configs = [roslaunch.config.load_config_default(
                [(str(PACKAGE/'launch'/name), args)], None, verbose=False)
                for name in ('quest_velocity_ik.launch', 'pico_velocity_ik.launch')]
            def controls(cfg):
                return {k:v.value for k,v in cfg.params.items()
                        if not k.startswith(('/monitor_quest3/', '/monitor_pico/', '/vr_input_source'))}
            self.assertEqual(controls(configs[0]), controls(configs[1]))
            self.assertFalse(any(n.name=='absolute_terminal_start' for n in configs[1].nodes))
            self.assertTrue(configs[1].params['/monitor_pico/enable_head_control'].value)
            node = next(n for n in configs[1].nodes if n.name == 'monitor_pico')
            self.assertTrue(node.launch_prefix.endswith(" background"))
        cfg = roslaunch.config.load_config_default([(str(PACKAGE/'launch/pico_velocity_ik.launch'),
              ['robot_version:=62','enable_head_control:=false'])],None,verbose=False)
        self.assertFalse(cfg.params['/monitor_pico/enable_head_control'].value)
        self.assertFalse(any(n.name=='absolute_terminal_start' for n in cfg.nodes))
        cfg = roslaunch.config.load_config_default([(str(PACKAGE/'launch/pico_velocity_ik.launch'),
              ['robot_version:=62','enable_terminal_start:=true'])],None,verbose=False)
        self.assertTrue(any(n.name=='absolute_terminal_start' for n in cfg.nodes))

    def test_quest_optical_default_and_explicit_source_locks_reach_both_nodes(self):
        for entry, default in (('quest_velocity_ik.launch', 'auto'),
                               ('pico_velocity_ik.launch', 'controller')):
            for override in (None, 'controller', 'hand', 'auto'):
                args = ['robot_version:=62']
                if override is not None: args.append('tracking_source_mode:='+override)
                cfg = roslaunch.config.load_config_default(
                    [(str(PACKAGE/'launch'/entry), args)], None, verbose=False)
                for node in ('ik_ros_uni', 'absolute_velocity_ik'):
                    self.assertEqual(cfg.params['/'+node+'/tracking_source_mode'].value,
                                     override or default)


if __name__ == '__main__':
    unittest.main()
