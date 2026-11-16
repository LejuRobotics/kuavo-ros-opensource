#!/usr/bin/env python3
"""Pico transport adapter for the shared absolute Quest input pipeline."""
import legacy_dependencies
import json
import queue
import socket
import threading
import time
from types import SimpleNamespace

import rospy
from kuavo_msgs.msg import JoySticks
from tf.transformations import quaternion_from_matrix

from monitor_quest3_velocity import Quest3BoneFramePublisher
from quest_input_buffer import SourceClockGate
from pico_input.adapter import PoseAdapter, controller_values, decode_datagram
from pico_input.robot_feedback import PicoRobotFeedback
from pico_input.discovery import discovery_pairs, announce


def quest_event(body, stamp_ns):
    """Inverse of the Quest coordinate conversion, retaining double precision.

    No source hand tracking or touch observations are synthesized. The shared
    processor reconstructs the existing Pico ROS arm/chest matrices exactly.
    """
    poses = []
    for matrix in body:
        x, y, z = matrix[:3, 3]
        qx, qy, qz, qw = quaternion_from_matrix(matrix)
        poses.append(SimpleNamespace(
            position=SimpleNamespace(x=-y, y=z, z=-x),
            quaternion=SimpleNamespace(x=-qy, y=qz, z=-qx, w=qw)))
    return SimpleNamespace(poses=poses, timestamp=stamp_ns // 1000000,
                           IsHandTracking=False, IsDataHighConfidence=True)


class PicoQuestReceiver(Quest3BoneFramePublisher):
    def __init__(self):
        super().__init__()
        self.adapter = PoseAdapter(rospy.get_param("~left_hand_offset", [0.] * 6),
                                   rospy.get_param("~right_hand_offset", [0.] * 6))
        self.peer = rospy.get_param("~peer_ip", "")
        self.connected = False
        self._closed = False
        self._discovery_stop = threading.Event()
        self._discovery_thread = None
        self.advertise_ip = rospy.get_param("~advertise_ip", "")
        self.broadcast_ip = rospy.get_param("~broadcast_ip", "")
        if bool(self.advertise_ip) != bool(self.broadcast_ip):
            raise ValueError("advertise_ip and broadcast_ip must both be supplied or both empty")
        self.discovery_pairs = discovery_pairs(self.advertise_ip, self.broadcast_ip)
        self.advertiser = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.advertiser.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        rospy.loginfo("Pico discovery targets: %s", self.discovery_pairs)
        if not self.discovery_pairs:
            rospy.logwarn("No active IPv4 broadcast interface; restart Pico input after connecting LAN.")
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 262144)
        self.sock.bind((rospy.get_param("~bind_address", "0.0.0.0"),
                        int(rospy.get_param("~port", 12345))))
        self.sock.settimeout(0.02)
        self.feedback = PicoRobotFeedback()
        self.set_robot_state_server(self.feedback)
        rospy.on_shutdown(self.close)
        rospy.loginfo("Pico controller input: shared Quest pose/head pipeline; "
                      "button touch and bare-hand tracking are unsupported.")

    def _disconnect(self, channel=None):
        if channel in (None, 'body'):
            self._pose_mailbox.clear()
            self._aux_mailbox.clear()
            self.feedback.peer = None
            if self.connected:
                self.connected = False
                rospy.set_param("/quest3/connected", False)
        if channel in (None, 'joy'):
            # An invented neutral sample would arm the base neutral gate on
            # reconnect. Invalid axes stop it without granting that permission.
            invalid = JoySticks()
            invalid.left_y = float('nan')
            self.joysticks_pub.publish(invalid)
            self.require_button_release = True
        rospy.logwarn_throttle(1., '[PicoInput] %s stream unavailable', channel or 'all')

    def _fresh(self, channel, stamp_ns, now):
        gate = self.gates[channel]
        if gate.accept(stamp_ns / 1e6, now):
            self.last_fresh[channel] = now
            return True
        self._disconnect(channel)
        # Unlike Quest, Pico pushes without a hi handshake. Require a fresh
        # stream and button release after a device-clock restart; never recenter.
        if (gate.latest_source is not None and stamp_ns / 1e9 < gate.latest_source
                and now - self.last_fresh[channel] > 0.5):
            gate.reset()
        return False

    def handle(self, raw, address, now):
        if self.peer and address[0] != self.peer:
            return
        msg, stamp_ns, _capture = decode_datagram(raw)
        body = joy = None
        if msg.HasField("full_body"):
            try:
                body = self.adapter.from_full_body(msg.full_body.full_body)
            except (ValueError, IndexError) as error:
                self._disconnect('body')
                rospy.logwarn_throttle(2., 'Invalid Pico body (joy remains independent): %s', error)
        if msg.HasField("controller"):
            joy = controller_values(msg.controller.controllers)
        if body is None and joy is None and not msg.HasField("vr_command"):
            return
        if not self.peer:
            self.peer = address[0]
        if joy is not None and self._fresh("joy", stamp_ns, now):
            if self.require_button_release:
                self.require_button_release = not (
                    joy["left_trigger"] < 0.1 and joy["right_trigger"] < 0.1
                    and not any(v for k, v in joy.items() if k.endswith("_pressed")))
            hand = JoySticks(**joy)
            if self.require_button_release:
                # Buttons wait for release; preserve real stick axes so a held
                # stick cannot be mistaken for a neutral reconnect sample.
                for name in joy:
                    if name.endswith(('_pressed', '_trigger', '_grip')):
                        setattr(hand, name, False if name.endswith('_pressed') else 0.)
            self.joysticks_pub.publish(hand)
        if body is not None and self._fresh("body", stamp_ns, now):
            gate = self.gates["body"]
            self._pose_mailbox.put((quest_event(body, stamp_ns), rospy.Time.now(), now,
                                    gate.relative_age, gate.rejected, gate.duplicates))
            self.feedback.peer = address
            if not self.connected:
                self.connected = True
                rospy.set_param("/quest3/connected", True)
        if msg.HasField("vr_command") and self.connected:
            command = msg.vr_command
            if command.HasField("item_mass_force_request") and self._fresh("command", stamp_ns, now):
                try:
                    self._control_requests.put_nowait(command.item_mass_force_request)
                except queue.Full:
                    rospy.logwarn_throttle(1., "Pico configuration request queue full")

    def _receive_packets(self):
        self.gates = {key: SourceClockGate(self._input_max_age) for key in ("body", "joy", "command")}
        self.last_fresh = {key: time.monotonic() for key in self.gates}
        self.require_button_release = False
        # Discovery and ROS parameter RPCs must never delay controller/body input.
        self._discovery_thread = threading.Thread(
            target=self._publish_discovery, name="pico_discovery", daemon=True)
        self._discovery_thread.start()
        timed_out = set()
        while not rospy.is_shutdown():
            try:
                raw, address = self.sock.recvfrom(65535)
                self.handle(raw, address, time.monotonic())
            except socket.timeout:
                pass
            except OSError:
                if rospy.is_shutdown() or self._closed:
                    break
                raise
            except Exception as error:
                rospy.logwarn_throttle(2., "Pico malformed/unsupported packet skipped: %s", error)
            now = time.monotonic()
            # Check channels independently: controller-only traffic must not keep
            # stale body input alive, or vice versa. Match Quest's one-second loss.
            for key in ("body", "joy"):
                if now - self.last_fresh[key] > 1.:
                    if key not in timed_out:
                        self._disconnect(key)
                        self.gates[key].reset()
                        timed_out.add(key)
                else:
                    timed_out.discard(key)

    def _publish_discovery(self):
        while not rospy.is_shutdown() and not self._discovery_stop.is_set():
            try:
                # Preserve live end-effector discovery, outside the UDP receiver.
                announce(self.advertiser, self.discovery_pairs,
                         rospy.get_param("/end_effector_type", "qiangnao"),
                         lambda local, error: rospy.logwarn_throttle(
                             5., "Pico discovery broadcast failed on %s: %s", local, error))
            except Exception as error:
                if not self._discovery_stop.is_set():
                    rospy.logwarn_throttle(5., "Pico discovery update failed: %s", error)
            self._discovery_stop.wait(1.)

    def close(self):
        self._discovery_stop.set()
        self._closed = True
        rospy.set_param("/quest3/connected", False)
        self.feedback.close()
        self.sock.close()
        if self._discovery_thread and self._discovery_thread is not threading.current_thread():
            self._discovery_thread.join(timeout=1.)
        if self.advertiser:
            self.advertiser.close()


if __name__ == "__main__":
    PicoQuestReceiver().run()
