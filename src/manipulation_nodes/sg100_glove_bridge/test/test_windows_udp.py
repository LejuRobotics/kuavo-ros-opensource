#!/usr/bin/env python3
"""Offline protocol and loopback tests for the Windows SG100 UDP backend."""

import importlib.util
import math
from pathlib import Path
import socket
import struct
import sys
import time
import types
import unittest
from unittest import mock


class Message:
    def __init__(self, data=None):
        self.data = [] if data is None else data


class Publisher:
    def __init__(self, topic, _message_type, queue_size=1):
        self.topic = topic
        self.queue_size = queue_size
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class SG100HandCommand:
    MODE_JOINT_POSITION = 7

    def __init__(self):
        self.header = types.SimpleNamespace(stamp=None)
        self.control_mode = 0
        self.left_hand_positions = []
        self.right_hand_positions = []
        self.left_enable_mask = 0
        self.right_enable_mask = 0
        self.source_id = 0


def load_node():
    publishers = {}
    parameters = {}
    rospy = types.ModuleType("rospy")
    rospy.get_param = lambda name, default=None: parameters.get(name, default)

    def make_publisher(topic, message_type, queue_size=1):
        publisher = Publisher(topic, message_type, queue_size)
        publishers[topic] = publisher
        return publisher

    rospy.Publisher = make_publisher
    rospy.Subscriber = lambda *args, **kwargs: object()
    rospy.on_shutdown = lambda callback: None
    rospy.is_shutdown = lambda: False
    rospy.init_node = lambda name: None
    rospy.loginfo = lambda *args, **kwargs: None
    rospy.logwarn = lambda *args, **kwargs: None
    rospy.logerr = lambda *args, **kwargs: None
    rospy.Time = types.SimpleNamespace(now=lambda: 123.0)
    sys.modules["rospy"] = rospy
    sys.modules.setdefault("rospkg", types.ModuleType("rospkg"))
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.Bool = Message
    std_msgs_msg.Float32MultiArray = Message
    sys.modules["std_msgs"] = std_msgs
    sys.modules["std_msgs.msg"] = std_msgs_msg
    kuavo_msgs = types.ModuleType("kuavo_msgs")
    kuavo_msgs_msg = types.ModuleType("kuavo_msgs.msg")
    kuavo_msgs_msg.SG100HandCommand = SG100HandCommand
    sys.modules["kuavo_msgs"] = kuavo_msgs
    sys.modules["kuavo_msgs.msg"] = kuavo_msgs_msg

    package = Path(__file__).resolve().parents[1]
    base_path = package / "scripts/sg100_glove_node.py"
    base_spec = importlib.util.spec_from_file_location("sg100_glove_node", base_path)
    base = importlib.util.module_from_spec(base_spec)
    sys.modules["sg100_glove_node"] = base
    base_spec.loader.exec_module(base)

    node_path = package / "scripts/sg100_windows_udp_node.py"
    node_spec = importlib.util.spec_from_file_location(
        "sg100_windows_udp_node_under_test", node_path)
    node = importlib.util.module_from_spec(node_spec)
    node_spec.loader.exec_module(node)
    return node, parameters, publishers


node, PARAMS, PUBLISHERS = load_node()


def free_udp_ports(count):
    sockets = []
    try:
        for _ in range(count):
            udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            udp_socket.bind(("127.0.0.1", 0))
            sockets.append(udp_socket)
        return [udp_socket.getsockname()[1] for udp_socket in sockets]
    finally:
        for udp_socket in sockets:
            udp_socket.close()


def ik_packet(frame, source_time, joints):
    body = " ".join(
        [str(frame), str(source_time)] + [str(value) for value in joints])
    return ("<SG100>" + body + "</SG100>\r\n").encode("utf-8")


def dof_packet(frame, offset=0.0):
    seconds = time.time()
    milliseconds = int(round(seconds * 1000.0))
    modules = []
    for module_index in range(6):
        modules.extend([
            offset + module_index,
            offset + module_index + 0.1,
            offset + module_index + 0.2,
            1.0, 0.0, 0.0, 0.0,
        ])
    return node.DOF_STRUCT.pack(
        node.DOF_MAGIC, node.DOF_VERSION, frame, node.DOF_FLAGS,
        seconds, *modules, milliseconds)


class ProtocolTest(unittest.TestCase):
    def test_ik_packet_requires_exact_tags_crlf_and_13_numbers(self):
        joints = [index / 10.0 for index in range(11)]
        for frame in (7, "7.00000000"):
            with self.subTest(frame=frame):
                self.assertEqual(node.parse_ik_packet(ik_packet(frame, 1.25, joints)),
                                 (7, 1.25, joints))
        bad_packets = [
            ik_packet(7, 1.25, joints).replace(b"\r\n", b"\n"),
            ik_packet(7, 1.25, joints).replace(b"</SG100>", b"</BAD>"),
            ik_packet(7, 1.25, joints) + b"extra",
            b"<SG100>\xff</SG100>\r\n",
            ik_packet(-1, 1.25, joints),
            ik_packet(7, -1.0, joints),
            ik_packet(7, 1.25, [math.inf] * 11),
            ik_packet(7, 1.25, joints + [0.0]),
            b" <SG100>7 1 " + b"0 " * 10 + b"0</SG100>\r\n",
            ik_packet("7.10000000", 1.25, joints),
            ik_packet("7.00000000000000000001", 1.25, joints),
            b"<SG100>7 1 " + b"0 " * 10 + b"</SG100>\r\n",
        ]
        for payload in bad_packets:
            with self.subTest(payload=payload):
                with self.assertRaises(ValueError):
                    node.parse_ik_packet(payload)

    def test_send_dof_layout_and_validity(self):
        self.assertEqual(node.DOF_STRUCT.format, "<IIIId42fq")
        self.assertEqual(node.DOF_STRUCT.size, 200)
        payload = dof_packet(9)
        self.assertEqual(struct.unpack_from("<IIII", payload, 0),
                         (node.DOF_MAGIC, node.DOF_VERSION, 9, node.DOF_FLAGS))
        frame, seconds, milliseconds, poses = node.parse_dof_packet(payload)
        self.assertEqual(frame, 9)
        self.assertLess(abs(seconds * 1000.0 - milliseconds), 2.0)
        self.assertEqual(len(poses), 6)
        self.assertEqual(poses[0][3:], [1.0, 0.0, 0.0, 0.0])
        for index, wrong in ((0, 0), (1, 3), (3, 1), (4, -1.0),
                             (5, math.nan), (6, math.inf), (-1, -1)):
            changed = list(node.DOF_STRUCT.unpack(payload))
            changed[index] = wrong
            with self.subTest(field=index, value=wrong):
                with self.assertRaises(ValueError):
                    node.parse_dof_packet(node.DOF_STRUCT.pack(*changed))
        with self.assertRaises(ValueError):
            node.parse_dof_packet(payload[:-1])
        with self.assertRaises(ValueError):
            node.parse_dof_packet(payload + b"\x00")
        unpacked = list(node.DOF_STRUCT.unpack(payload))
        unpacked[5 + 3] = 0.0
        with self.assertRaises(ValueError):
            node.parse_dof_packet(node.DOF_STRUCT.pack(*unpacked))
        unpacked = list(node.DOF_STRUCT.unpack(payload))
        unpacked[4] = math.inf
        with self.assertRaises(ValueError):
            node.parse_dof_packet(node.DOF_STRUCT.pack(*unpacked))
        unpacked = list(node.DOF_STRUCT.unpack(payload))
        unpacked[-1] += 10000
        with self.assertRaises(ValueError):
            node.parse_dof_packet(node.DOF_STRUCT.pack(*unpacked))

        with self.assertRaises(ValueError):
            node.parse_ik_packet(ik_packet(1, math.inf, [0.0] * 11))
        with self.assertRaises(ValueError):
            node.parse_ik_packet(ik_packet(1, 0.0, [math.nan] + [0.0] * 10))

    def test_module_mapping_is_explicit_and_unambiguous(self):
        self.assertEqual(node.parse_module_map([], "map"), [])
        self.assertEqual(node.parse_module_map("A,C,E,B,D", "map"),
                         ["A", "C", "E", "B", "D"])
        for value in (["A"] * 5, ["A", "B", "C", "D"],
                      ["A", "B", "C", "D", "G"]):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    node.parse_module_map(value, "map")

    def test_sequence_gate_rejects_duplicates_and_old_epoch_packets(self):
        gate = node.SequenceGate(restart_timeout=2.0, relative_time=True)
        self.assertTrue(gate.accept(100, 50.0, 0.0))
        self.assertFalse(gate.accept(100, 50.0, 0.1))
        self.assertFalse(gate.accept(1, 0.1, 1.9))
        self.assertTrue(gate.accept(2, 0.2, 2.1))
        self.assertFalse(gate.accept(101, 51.0, 2.2))
        self.assertTrue(gate.accept(3, 0.3, 2.3))

    def test_sequence_gate_recovers_continuous_restart_after_timeout(self):
        gate = node.SequenceGate(restart_timeout=2.0, relative_time=True)
        self.assertTrue(gate.accept(10000, 500.0, 0.0))
        self.assertFalse(gate.accept(0, 0.0, 0.01))
        accepted = []
        for frame in range(1, 202):
            arrival = 0.01 + frame * 0.01
            if gate.accept(frame, frame * 0.01, arrival):
                accepted.append(frame)
        self.assertEqual(accepted, [199, 200, 201])
        self.assertFalse(gate.accept(10001, 501.0, 2.02))
        self.assertTrue(gate.accept(202, 2.02, 2.03))

    def test_single_delayed_low_packet_does_not_poison_current_stream(self):
        gate = node.SequenceGate(restart_timeout=2.0, relative_time=True)
        self.assertTrue(gate.accept(1000, 10.0, 0.0))
        self.assertFalse(gate.accept(2, 0.02, 0.1))
        self.assertTrue(gate.accept(1001, 10.01, 0.2))
        self.assertTrue(gate.accept(1002, 10.02, 0.3))

    def test_restart_stream_crossing_old_time_and_frame_recovers(self):
        gate = node.SequenceGate(restart_timeout=2.0, relative_time=True)
        self.assertTrue(gate.accept(100, 1.0, 0.0))
        self.assertFalse(gate.accept(0, 0.0, 0.01))
        accepted = []
        for frame in range(1, 202):
            arrival = 0.01 + frame * 0.01
            if gate.accept(frame, frame * 0.01, arrival):
                accepted.append((frame, arrival))
        self.assertTrue(accepted)
        self.assertLessEqual(accepted[0][1], 2.02)

    def test_restart_recovers_when_initial_low_frames_were_lost(self):
        for relative in (True, False):
            with self.subTest(relative_time=relative):
                gate = node.SequenceGate(restart_timeout=2.0, relative_time=relative)
                self.assertTrue(gate.accept(10000, 500.0, 0.0))
                new_time = 20.0 if relative else 520.0
                self.assertFalse(gate.accept(2000, new_time, 20.0))
                self.assertTrue(gate.accept(2001, new_time + .01, 20.01))
                self.assertFalse(gate.accept(2001, new_time + .01, 20.02))
                self.assertFalse(gate.accept(1999, new_time - .01, 20.03))
                self.assertTrue(gate.accept(2002, new_time + .02, 20.04))

    def test_send_dof_sequence_is_independent_and_handles_counter_reset(self):
        gate = node.SequenceGate(restart_timeout=2.0, relative_time=False)
        self.assertTrue(gate.accept(1000, 1700000000.0, 0.0))
        self.assertFalse(gate.accept(1000, 1700000000.01, .1))
        self.assertFalse(gate.accept(999, 1700000000.02, .2))
        self.assertFalse(gate.accept(1001, 1699999999.0, .3))
        self.assertTrue(gate.accept(1001, 1700000000.1, .4))
        self.assertFalse(gate.accept(0, 1700000003.0, 3.0))
        self.assertTrue(gate.accept(1, 1700000003.01, 3.01))
        self.assertTrue(gate.accept(2, 1700000003.02, 3.02))


class LoopbackTest(unittest.TestCase):
    def setUp(self):
        PUBLISHERS.clear()
        PARAMS.clear()
        self.ports = dict(zip(("right", "left"), free_udp_ports(2)))
        PARAMS.update({
            "~bind_address": "127.0.0.1",
            "~right_port": self.ports["right"],
            "~left_port": self.ports["left"],
            "~right_module_map": ["C", "B", "A", "D", "F"],
            "~left_module_map": ["A", "B", "C", "D", "E"],
            "~publish_real": False,
        })
        self.bridge = node.SG100WindowsUdpNode()
        self.sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def tearDown(self):
        self.sender.close()
        self.bridge.close()

    def send(self, hand, payload):
        self.sender.sendto(payload, ("127.0.0.1", self.ports[hand]))

    def test_single_hand_binds_only_its_port_and_enables_only_its_target(self):
        for side, other in (("right", "left"), ("left", "right")):
            with self.subTest(side=side):
                self.bridge.close()
                PUBLISHERS.clear()
                PARAMS.update({"~side": side, "~publish_real": True})
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as inactive:
                    inactive.bind(("127.0.0.1", self.ports[other]))
                    self.bridge = node.SG100WindowsUdpNode()
                    self.assertEqual(len(self.bridge._sockets), 1)
                    self.send(side, ik_packet(1, 0.01, [0.2] * 11))
                    self.send(side, dof_packet(1))
                    self.assertEqual(self.bridge.poll_once(0.2), 2)
                    real = PUBLISHERS["/sg100_hand_command"].messages[-1]
                    self.assertEqual(getattr(real, side + "_enable_mask"), 0x07FF)
                    self.assertEqual(getattr(real, side + "_hand_positions"), [0.2] * 11)
                    self.assertEqual(getattr(real, other + "_hand_positions"), [0.0] * 11)
                    self.assertEqual(getattr(real, other + "_enable_mask"), 0)
                    self.assertEqual(real.source_id, 1)
                    self.assertIn("/qianshi/glove_" + side + "/fingertips", PUBLISHERS)
                    self.assertNotIn("/qianshi/glove_" + other + "/fingertips", PUBLISHERS)

    def test_mixed_streams_freeze_joint_targets_and_do_not_replay_dof(self):
        self.bridge.close()
        PARAMS["~publish_real"] = True
        self.bridge = node.SG100WindowsUdpNode()
        right_q = [1.0 + index for index in range(11)]
        left_q = [2.0 + index for index in range(11)]
        self.send("right", ik_packet(1, 0.01, right_q))
        self.send("left", dof_packet(1, 20.0))
        self.send("right", dof_packet(1, 10.0))
        self.send("left", ik_packet("1.00000000", 0.01, left_q))
        self.assertEqual(self.bridge.poll_once(0.2), 4)

        right_joint_pub = PUBLISHERS["/sg100_sim_hand_command"]
        left_joint_pub = PUBLISHERS["/sg100_sim_left_hand_command"]
        right_pose_pub = PUBLISHERS["/qianshi/glove_right/fingertips"]
        left_pose_pub = PUBLISHERS["/qianshi/glove_left/fingertips"]
        real_pub = PUBLISHERS["/sg100_hand_command"]
        self.assertEqual(right_joint_pub.messages[-1].data, right_q)
        self.assertEqual(left_joint_pub.messages[-1].data, left_q)
        self.assertEqual(len(right_pose_pub.messages[-1].data), 35)
        self.assertEqual(len(left_pose_pub.messages[-1].data), 35)
        # Right thumb maps to module C while left thumb maps to module A.
        self.assertAlmostEqual(right_pose_pub.messages[-1].data[0], 12.0)
        self.assertAlmostEqual(left_pose_pub.messages[-1].data[0], 20.0)

        self.bridge._on_interaction_mode(Message(True))
        newest_right_q = [9.0 + index for index in range(11)]
        newest_left_q = [8.0 + index for index in range(11)]
        self.send("right", ik_packet(2, 0.02, newest_right_q))
        self.send("left", ik_packet(2, 0.02, newest_left_q))
        self.send("right", dof_packet(2, 30.0))
        self.assertEqual(self.bridge.poll_once(0.2), 3)
        self.assertEqual(right_joint_pub.messages[-1].data, right_q)
        self.assertEqual(left_joint_pub.messages[-1].data, left_q)
        self.assertEqual(real_pub.messages[-1].right_hand_positions, right_q)
        self.assertEqual(real_pub.messages[-1].left_hand_positions, left_q)
        self.assertEqual(len(right_pose_pub.messages), 2)

        joint_count = len(right_joint_pub.messages)
        pose_count = len(right_pose_pub.messages)
        self.assertEqual(self.bridge.poll_once(0.01), 0)
        self.assertEqual(len(right_joint_pub.messages), joint_count)
        self.assertEqual(len(right_pose_pub.messages), pose_count)

        self.bridge._on_interaction_mode(Message(False))
        self.assertEqual(right_joint_pub.messages[-1].data, newest_right_q)
        self.assertEqual(left_joint_pub.messages[-1].data, newest_left_q)
        self.assertEqual(real_pub.messages[-1].right_hand_positions, newest_right_q)
        self.assertEqual(real_pub.messages[-1].left_hand_positions, newest_left_q)

        # A valid packet with an empty map is consumed but never repeats or
        # fabricates a fingertip publication.
        self.bridge.module_maps["right"] = []
        self.send("right", dof_packet(3, 40.0))
        self.assertEqual(self.bridge.poll_once(0.2), 1)
        self.assertEqual(len(right_pose_pub.messages), pose_count)

    def test_first_ik_during_interaction_waits_for_exit(self):
        self.bridge.close()
        PARAMS["~publish_real"] = True
        self.bridge = node.SG100WindowsUdpNode()
        self.bridge._on_interaction_mode(Message(True))
        self.send("right", ik_packet(1, 0.01, [0.2] * 11))
        self.send("left", ik_packet(1, 0.01, [0.3] * 11))
        self.send("left", dof_packet(1))
        self.assertEqual(self.bridge.poll_once(0.2), 3)
        self.assertFalse(PUBLISHERS["/sg100_sim_hand_command"].messages)
        self.assertFalse(PUBLISHERS["/sg100_sim_left_hand_command"].messages)
        real_pub = PUBLISHERS["/sg100_hand_command"]
        self.assertFalse(real_pub.messages)
        self.assertEqual(len(PUBLISHERS["/qianshi/glove_left/fingertips"].messages), 1)

        self.send("right", ik_packet(2, 0.02, [0.4] * 11))
        self.assertEqual(self.bridge.poll_once(0.2), 1)
        self.assertFalse(real_pub.messages)

        self.bridge._on_interaction_mode(Message(False))
        self.assertEqual(real_pub.messages[-1].right_hand_positions, [0.4] * 11)
        self.assertEqual(real_pub.messages[-1].left_hand_positions, [0.3] * 11)

    def test_two_sockets_keep_latest_ik_and_dof_independently(self):
        self.assertEqual({hand: sock.getsockname()[1] for hand, sock in
                          self.bridge._sockets.items()}, self.ports)
        # One valid text packet is exactly 200 bytes, like SendDof.
        text = ik_packet(2, 0.02, [0.2] * 11)
        text = text.replace(b"</SG100>", b" " * (200 - len(text)) + b"</SG100>")
        self.assertEqual(len(text), 200)
        self.send("right", ik_packet(1, 0.01, [0.1] * 11))
        self.send("right", dof_packet(91, 10.0))
        self.send("right", text)
        self.send("right", dof_packet(92, 20.0))
        self.send("right", ik_packet(3, 0.03, [0.3] * 11))
        self.send("left", dof_packet(3, 30.0))
        self.send("left", ik_packet(80, 0.80, [0.8] * 11))
        self.send("left", ik_packet(81, 0.81, [0.81] * 11))
        self.send("left", dof_packet(4, 40.0))
        self.assertEqual(self.bridge.poll_once(0.2), 4)
        for hand, joint_topic, q, x in (
                ("right", "/sg100_sim_hand_command", 0.3, 22.0),
                ("left", "/sg100_sim_left_hand_command", 0.81, 40.0)):
            joint_messages = PUBLISHERS[joint_topic].messages
            pose_messages = PUBLISHERS["/qianshi/glove_" + hand + "/fingertips"].messages
            self.assertEqual([m.data for m in joint_messages], [[q] * 11])
            self.assertEqual(len(pose_messages), 1)
            self.assertEqual(pose_messages[0].data[0], x)
        self.assertEqual(self.bridge._bad_lines, 0)

        self.send("right", ik_packet(3, 0.03, [0.9] * 11))
        self.send("right", ik_packet(2, 0.02, [0.8] * 11))
        self.send("right", dof_packet(92))
        self.send("right", dof_packet(91))
        self.assertEqual(self.bridge.poll_once(0.2), 0)
        # The socket queue was drained and neither stream is replayed.
        self.assertEqual(self.bridge.poll_once(0.01), 0)

    def test_busy_port_yields_to_other_port(self):
        for frame in range(1, 7):
            self.send("left", ik_packet(frame, frame * .01, [float(frame)] * 11))
        self.send("right", ik_packet(1, .01, [0.3] * 11))
        self.send("right", dof_packet(100))
        with mock.patch.object(node, "MAX_DRAIN_PER_SOCKET", 3):
            self.assertEqual(self.bridge.poll_once(0.2), 3)
            left = PUBLISHERS["/sg100_sim_left_hand_command"]
            right = PUBLISHERS["/sg100_sim_hand_command"]
            self.assertEqual([m.data for m in left.messages], [[3.0] * 11])
            self.assertEqual([m.data for m in right.messages], [[0.3] * 11])
            self.assertEqual(self.bridge.poll_once(0.2), 1)
            self.assertEqual(left.messages[-1].data, [6.0] * 11)

    def test_one_protocol_restarts_while_the_other_stays_live(self):
        self.send("right", ik_packet(1000, 10.0, [0.1] * 11))
        self.send("right", dof_packet(500))
        with mock.patch.object(node.time, "monotonic", return_value=10.0):
            self.assertEqual(self.bridge.poll_once(0.2), 2)
        self.send("right", ik_packet(0, 0.0, [0.2] * 11))
        self.send("right", dof_packet(501))
        self.send("right", ik_packet(1, 0.01, [0.3] * 11))
        with mock.patch.object(node.time, "monotonic", return_value=13.0):
            self.assertEqual(self.bridge.poll_once(0.2), 2)
        self.assertEqual(self.bridge._gates[("right", "ik")].last_frame, 1)
        self.assertEqual(self.bridge._gates[("right", "dof")].last_frame, 501)
        joints = PUBLISHERS["/sg100_sim_hand_command"]
        self.assertEqual([m.data for m in joints.messages], [[0.1] * 11, [0.3] * 11])

    def test_listen_ports_must_be_configured_and_distinct(self):
        for value in (None, 0, 65536, self.ports["left"]):
            with self.subTest(port=value):
                PARAMS["~right_port"] = value
                with self.assertRaises(ValueError):
                    node.SG100WindowsUdpNode()

    def test_unrepresentable_ik_value_is_dropped_and_node_stays_alive(self):
        self.send("right", ik_packet(1, 0.01, [1.0e300] * 11))
        self.assertEqual(self.bridge.poll_once(0.2), 0)
        publisher = PUBLISHERS["/sg100_sim_hand_command"]
        self.assertEqual(publisher.messages, [])
        self.send("right", ik_packet(1, 0.02, [1.0] * 11))
        self.assertEqual(self.bridge.poll_once(0.2), 1)
        self.assertEqual(publisher.messages[-1].data, [1.0] * 11)

    def test_malformed_packets_do_not_block_either_protocol(self):
        damaged = list(node.DOF_STRUCT.unpack(dof_packet(1)))
        damaged[0] = 0
        self.send("right", node.DOF_STRUCT.pack(*damaged))
        damaged[0] = node.DOF_MAGIC
        damaged[1] = 3
        self.send("right", node.DOF_STRUCT.pack(*damaged))
        self.send("right", ik_packet(1, 0.01, [0.1] * 11)[:-1])
        self.send("right", dof_packet(1))
        self.send("right", ik_packet(1, 0.01, [0.1] * 11))
        self.assertEqual(self.bridge.poll_once(0.2), 2)
        self.assertEqual(self.bridge._bad_lines, 3)
        publisher = PUBLISHERS["/qianshi/glove_right/fingertips"]
        self.assertEqual(len(publisher.messages), 1)
        self.send("right", dof_packet(1))
        self.send("right", dof_packet(0))
        self.assertEqual(self.bridge.poll_once(0.2), 0)
        self.assertEqual(len(publisher.messages), 1)

    def test_poll_returns_cleanly_when_shutdown_closes_sockets(self):
        def close_then_fail(_read, _write, _error, _timeout):
            self.bridge.close()
            raise OSError(9, "Bad file descriptor")

        with mock.patch.object(node.select, "select", side_effect=close_then_fail):
            self.assertEqual(self.bridge.poll_once(0.2), 0)

    def test_recv_returns_cleanly_when_shutdown_closes_ready_socket(self):
        bridge = self.bridge

        class ClosedDuringRecv:
            def recvfrom(self, _size):
                bridge._closed = True
                raise OSError(9, "Bad file descriptor")

        broken_socket = ClosedDuringRecv()
        self.bridge._socket_hands[broken_socket] = "right"
        try:
            with mock.patch.object(
                    node.select, "select",
                    return_value=([broken_socket], [], [])):
                self.assertEqual(self.bridge.poll_once(0.2), 0)
        finally:
            self.bridge._closed = False
            self.bridge._socket_hands.pop(broken_socket, None)

    def test_second_instance_fails_on_port_conflict(self):
        with self.assertRaises(OSError):
            node.SG100WindowsUdpNode()
        # The failed constructor closed only its own partial sockets.
        self.send("right", ik_packet(1, 0.01, [3.0] * 11))
        self.assertEqual(self.bridge.poll_once(0.2), 1)

    def test_nonfinite_period_parameters_are_rejected(self):
        for parameter in ("~restart_timeout", "~diagnostic_period"):
            previous = PARAMS.get(parameter)
            for value in (math.nan, math.inf):
                PARAMS[parameter] = value
                with self.subTest(parameter=parameter, value=value):
                    with self.assertRaises(ValueError):
                        node.SG100WindowsUdpNode()
            if previous is None:
                PARAMS.pop(parameter, None)
            else:
                PARAMS[parameter] = previous

    def test_real_command_caches_both_sides_and_sets_enable_masks(self):
        self.bridge.close()
        PARAMS["~publish_real"] = True
        PUBLISHERS.clear()
        self.bridge = node.SG100WindowsUdpNode()
        left = [1.0] * 11
        right = [2.0] * 11
        self.bridge._publish_joint("left", left)
        real_publisher = PUBLISHERS["/sg100_hand_command"]
        first = real_publisher.messages[-1]
        self.assertEqual(first.left_hand_positions, left)
        self.assertEqual(first.right_hand_positions, [0.0] * 11)
        self.assertEqual(first.right_enable_mask, 0)
        self.assertEqual(first.source_id, 1)
        self.bridge._publish_joint("right", right)
        second = real_publisher.messages[-1]
        self.assertEqual(second.left_hand_positions, left)
        self.assertEqual(second.right_hand_positions, right)
        self.assertEqual(second.left_enable_mask, 0x07FF)
        self.assertEqual(second.right_enable_mask, 0x07FF)
        self.assertEqual(second.control_mode, SG100HandCommand.MODE_JOINT_POSITION)


if __name__ == "__main__":
    unittest.main()
