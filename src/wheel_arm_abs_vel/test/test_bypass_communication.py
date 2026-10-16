#!/usr/bin/env python3
"""Offline round-1 regressions: no ROS master, robot, or network needed."""
import ast
import importlib.util
import io
import math
from pathlib import Path
import socket
import sys
import threading
import types
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT.parent / "motion_capture_ik" / "scripts"))
import robot_state_server_velocity as states
import udp_server_velocity as udp
import monitor_quest3_velocity as monitor
from tools import noncontrol_publish_process as noncontrol
from tools import arm_marker_process as markers
from kuavo_msgs.msg import sensorsData
from noitom_hi5_hand_udp_python.msg import PoseInfoList, PoseInfo
from geometry_msgs.msg import Point, Quaternion


class BypassTest(unittest.TestCase):
    def subscriber(self):
        with patch.object(states.RobotSubscriber, "_setup_subscription"):
            return states.RobotSubscriber(ee_type="qiangnao", subscribe_sensor_data=True)

    def test_latest_sensor_converts_only_at_consumption(self):
        sub = self.subscriber()
        with patch.object(states.robot_state_pb2, "JointState", wraps=states.robot_state_pb2.JointState) as ctor:
            for i in range(500):
                msg = sensorsData()
                msg.joint_data.joint_q = [float(i)]
                msg.joint_data.joint_v = [2.0]
                msg.joint_data.joint_torque = [3.0]
                sub._sensors_callback(msg)
            self.assertEqual(ctor.call_count, 0)
            self.assertIs(sub._latest_state["joint"], msg)
            valid, result = sub.get_joint_state()
            self.assertTrue(valid)
            self.assertEqual(list(result.position), [499.0])
            self.assertEqual(list(result.velocity), [2.0])
            self.assertEqual(list(result.torque), [3.0])
            self.assertEqual(ctor.call_count, 1)
            self.assertEqual(sub.get_joint_state(), (False, None))

    def test_mailbox_take_does_not_clear_later_arrival(self):
        sub = self.subscriber()
        first, second = object(), object()
        sub._sensors_callback(first)
        taken = sub._take_state("joint")
        sub._sensors_callback(second)
        self.assertIs(taken, first)
        self.assertIs(sub._take_state("joint"), second)

    def test_end_effector_and_force_layout(self):
        sub = self.subscriber()
        sub._dexhand_callback(types.SimpleNamespace(position=[1.0], velocity=[], effort=[]))
        valid, result = sub.get_ee_state()
        self.assertTrue(valid)
        self.assertEqual(list(result.position), [1.0])
        self.assertEqual(len(result.velocity), 12)
        sub._arm_force_callback(types.SimpleNamespace(data=[1., 2., 3., 99., 99., 99., 4., 5., 6.]))
        valid, result = sub.get_arm_force_feedback()
        self.assertTrue(valid)
        self.assertAlmostEqual(result.max_force[0], sub._normalize_force(3.), places=4)
        self.assertAlmostEqual(result.max_force[1], sub._normalize_force(6.), places=4)

    def test_transaction_fifo_not_coalesced(self):
        with patch.object(states.RobotSubscriber, "_setup_subscription"), patch.object(
                states.rospy, "get_param", side_effect=lambda name, default: default):
            server = states.RobotStateServer()
        responses = [object() for _ in range(20)]
        for response in responses:
            server.add_item_mass_force_response(response)
        for response in responses:
            self.assertIs(server._process_item_mass_force_response(), response)
        self.assertIsNone(server._process_item_mass_force_response())

    def test_udp_one_serialization_and_receive_lease(self):
        server = udp.UDPServer(client_timeout=60, force_port=False)
        sent = []
        server.socket = types.SimpleNamespace(sendto=lambda data, addr: sent.append((data, addr)))
        server.clients = {
            "old": server.ClientInfo(("127.0.0.1", 1), 0.),
            "a": server.ClientInfo(("127.0.0.1", 2), 59.),
            "b": server.ClientInfo(("127.0.0.1", 3), 59.),
        }
        count = []
        frame = types.SimpleNamespace(SerializeToString=lambda: count.append(1) or b"frame")
        with patch.object(udp.time, "monotonic", return_value=61.):
            server.broadcast(frame)
        self.assertNotIn("old", server.clients)
        self.assertEqual(len(count), 1)
        self.assertEqual(len(sent), 2)
        self.assertIs(sent[0][0], sent[1][0])
        self.assertEqual(server.clients["a"].last_rx, 59.)
        with patch.object(udp.time, "monotonic", return_value=120.):
            server.broadcast(frame)
        self.assertEqual(server.clients, {})
        self.assertEqual(len(count), 1)

    def test_udp_hello_and_endpoint_heartbeat(self):
        server = udp.UDPServer(force_port=False)
        addr = ("127.0.0.1", 42)
        packets = iter([
            (b'{"action":"Hello","client_id":"a"}', addr),
            (b"heartbeat", addr),
            (b"heartbeat", ("127.0.0.1", 43)),
        ])
        snapshots = []
        def receive(size):
            snapshots.append(server.clients.get("a").last_rx if server.clients else None)
            try:
                return next(packets)
            except StopIteration:
                server.running = False
                raise socket.timeout()
        server.socket = types.SimpleNamespace(
            settimeout=lambda value: None, recvfrom=receive, sendto=lambda data, endpoint: len(data))
        server.running = True
        with patch.object(udp.time, "monotonic", side_effect=[10., 20., 30.]):
            server._receive_loop()
        self.assertEqual(snapshots, [None, 10., 20., 20.])
        self.assertEqual(server.clients["a"].last_rx, 20.)

    def test_noncontrol_gate_precedes_serialization_and_is_per_channel(self):
        proc = noncontrol.DiagnosticProcess.__new__(noncontrol.DiagnosticProcess)
        proc.sent = proc.dropped = proc.errors = proc.attempts = 0
        proc.closed = proc.failed = False
        proc.period, proc.last_submit = .01, {}
        proc.gate_lock = threading.Lock()
        packets, conversions = [], []
        proc.socket = types.SimpleNamespace(send=lambda data, flags: packets.append(data) or len(data))
        message = types.SimpleNamespace(serialize=lambda out: conversions.append(1) or out.write(b"x"))
        with patch.object(noncontrol.time, "monotonic", side_effect=[1., 1.002, 1.002, 1.011]):
            self.assertTrue(proc.submit(0, message))
            self.assertFalse(proc.submit(0, message))
            self.assertTrue(proc.submit(1, message))
            self.assertTrue(proc.submit(0, message))
        self.assertEqual(len(conversions), 3)
        proc.period = 0.
        with patch.object(noncontrol.time, "monotonic", return_value=2.):
            for _ in range(10):
                self.assertTrue(proc.submit(0, message))

    def worker_output(self, coalesce):
        left, right = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        output = {}
        class Message:
            def deserialize(self, payload):
                self.payload = payload
                return self
        def publisher(topic, *args, **kwargs):
            output[topic] = []
            return types.SimpleNamespace(
                get_num_connections=lambda: 1, publish=output[topic].append)
        topics = [spec[0] for spec in noncontrol.CHANNELS]
        now = noncontrol.time.monotonic_ns()
        for index, payload in [(0, b"old"), (1, b"other"), (0, b"new")]:
            left.send(noncontrol._HEADER.pack(noncontrol._MAGIC, index, now) + payload)
        with patch("rospy.init_node"), patch("rospy.signal_shutdown"), patch(
                "rospy.is_shutdown", side_effect=[False, True]), patch(
                "rospy.Publisher", side_effect=publisher), patch(
                "roslib.message.get_message_class", return_value=Message), patch(
                "os.sched_setscheduler"), patch("os.setpriority"):
            noncontrol.worker(right.detach(), topics, coalesce)
        left.close()
        return {topic: [msg.payload for msg in output[topic]] for topic in topics}

    def test_noncontrol_worker_coalesces_each_channel(self):
        result = self.worker_output(True)
        self.assertEqual(result[noncontrol.CHANNELS[0][0]], [b"new"])
        self.assertEqual(result[noncontrol.CHANNELS[1][0]], [b"other"])

    def test_noncontrol_worker_audit_keeps_all_packets(self):
        result = self.worker_output(False)
        self.assertEqual(result[noncontrol.CHANNELS[0][0]], [b"old", b"new"])
        self.assertEqual(result[noncontrol.CHANNELS[1][0]], [b"other"])

    def monitor(self, rate):
        obj = monitor.Quest3BoneFramePublisher.__new__(monitor.Quest3BoneFramePublisher)
        obj.bone_names = [str(i) for i in range(26)]
        obj.observation_tf_rate_hz = rate
        obj._last_observation_tf = float("-inf")
        obj.control_finger_tf_enabled = False
        obj._input_max_age = .12
        obj.messages = []
        obj.bone_tf_pub = types.SimpleNamespace(publish=obj.messages.append)
        return obj

    def poses(self):
        result = PoseInfoList()
        for i in range(26):
            result.poses.append(PoseInfo(
                position=Point(x=float(i), y=2., z=3.),
                orientation=Quaternion(w=1.)))
        return result

    def test_tf_gates_observation_but_preserves_head_and_chest(self):
        obj, poses = self.monitor(30.), self.poses()
        for i in range(120):
            now = i / 120.
            with patch.object(monitor.time, "monotonic", return_value=now):
                obj.publish_pose_tf(poses, monitor.rospy.Time(), now, 0.)
        self.assertEqual(len(obj.messages), 120)
        full = [m for m in obj.messages if len(m.transforms) == 26]
        self.assertGreaterEqual(len(full), 25)
        self.assertLessEqual(len(full), 31)
        for msg in obj.messages:
            names = {t.child_frame_id for t in msg.transforms}
            self.assertTrue({"23", "25"} <= names)
            self.assertEqual(msg.transforms[-1].transform.translation.z, 9.)

    def test_optional_gesture_control_tf_stays_full_rate(self):
        obj, poses = self.monitor(30.), self.poses()
        obj.control_finger_tf_enabled = True
        obj._last_observation_tf = 1.
        with patch.object(monitor.time, "monotonic", return_value=1.002):
            obj.publish_pose_tf(poses, monitor.rospy.Time(), 1.002, 0.)
        names = {t.child_frame_id for t in obj.messages[0].transforms}
        self.assertEqual(names, {str(i) for i in range(4, 22)} | {"23", "25"})

    def test_tf_stale_invalid_and_audit(self):
        obj, poses = self.monitor(0.), self.poses()
        with patch.object(monitor.time, "monotonic", return_value=1.):
            obj.publish_pose_tf(poses, monitor.rospy.Time(), 0., 0.)
            self.assertEqual(obj.messages, [])
            for _ in range(5):
                obj.publish_pose_tf(poses, monitor.rospy.Time(), 1., 0.)
            self.assertEqual([len(m.transforms) for m in obj.messages], [26]*5)
            poses.poses[0].position.x = float("nan")
            obj.publish_pose_tf(poses, monitor.rospy.Time(), 1., 0.)
            self.assertEqual(len(obj.messages[-1].transforms), 25)

    def test_chest_marker_once_per_frame(self):
        counts = {}
        names = ["marker_pub", "marker_pub_elbow", "marker_pub_shoulder",
                 "marker_pub_right", "marker_pub_elbow_right",
                 "marker_pub_shoulder_right", "marker_pub_chest"]
        pubs = {n: types.SimpleNamespace(publish=lambda msg, n=n: counts.__setitem__(n, counts.get(n, 0)+1))
                for n in names}
        with patch.object(markers, "point_marker", side_effect=lambda *args: args):
            jobs = [(side, (1,2,3), (2,3,4), (3,4,5), (4,5,6)) for side in ("Left", "Right")]
            markers.publish_jobs(pubs, jobs)
        self.assertEqual(counts, dict.fromkeys(names, 1))


if __name__ == "__main__":
    unittest.main()
