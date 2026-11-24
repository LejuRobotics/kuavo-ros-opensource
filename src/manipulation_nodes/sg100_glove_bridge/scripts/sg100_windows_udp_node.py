#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Windows SG100 UDP -> ROS bridge.

This backend deliberately reuses only the ROS publication/interaction-mode
parts of :mod:`sg100_glove_node`.  It never constructs or starts the Linux SDK
node, and consequently never scans or opens a serial device.
"""

import importlib.util
import math
import os
import re
import select
import socket
import struct
import threading
import time

import rospy
from std_msgs.msg import Bool, Float32MultiArray


# A catkin devel-space relay executes the source in a private dictionary and
# therefore is not a normal importable module.  Load the sibling source file
# explicitly; ``__file__`` points at the real source for a relay, and both
# installed scripts also live in the same libexec directory.
_LINUX_BRIDGE_PATH = os.path.join(
    os.path.dirname(os.path.realpath(__file__)), "sg100_glove_node.py")
_LINUX_BRIDGE_SPEC = importlib.util.spec_from_file_location(
    "_sg100_linux_bridge_for_windows_udp", _LINUX_BRIDGE_PATH)
if _LINUX_BRIDGE_SPEC is None or _LINUX_BRIDGE_SPEC.loader is None:
    raise ImportError("cannot load sibling sg100_glove_node.py")
linux_bridge = importlib.util.module_from_spec(_LINUX_BRIDGE_SPEC)
_LINUX_BRIDGE_SPEC.loader.exec_module(linux_bridge)


IK_HEADER = "<SG100>"
IK_PREFIX = IK_HEADER.encode("ascii")
IK_FOOTER = "</SG100>\r\n"
IK_VALUE_COUNT = 13
DOF_MAGIC = 0x4C313047
DOF_VERSION = 4
DOF_FLAGS = 0
DOF_PREFIX = struct.pack("<II", DOF_MAGIC, DOF_VERSION)
DOF_STRUCT = struct.Struct("<IIIId42fq")
DOF_SIZE = 200
MODULE_NAMES = tuple("ABCDEF")
MAX_DRAIN_PER_SOCKET = 256
RESTART_MAX_FRAME = 10
RESTART_MAX_RELATIVE_TIME = 2.0
TIMESTAMP_TOLERANCE_MS = 2000.0
QUATERNION_NORM_MIN = 0.5
QUATERNION_NORM_MAX = 1.5
FLOAT32_MAX = 3.4028234663852886e38
_IK_FRAME_PATTERN = re.compile(r"([0-9]+)(?:\.0+)?")


def parse_ik_packet(payload):
    """Decode one exact CRLF-terminated IKHand datagram.

    Returns ``(frame, relative_time, joints)``.  No unit conversion, filtering,
    scaling, or zero-offset is applied.
    """
    try:
        text = payload.decode("utf-8")
    except (AttributeError, UnicodeDecodeError) as exc:
        raise ValueError("IK packet is not UTF-8 bytes") from exc
    if not text.startswith(IK_HEADER) or not text.endswith(IK_FOOTER):
        raise ValueError("IK packet tags or CRLF terminator are invalid")
    body = text[len(IK_HEADER):-len(IK_FOOTER)]
    fields = body.split()
    if len(fields) != IK_VALUE_COUNT:
        raise ValueError("IK packet must contain exactly 13 numbers")
    # Windows formats the frame as a fixed-point number with a zero fraction.
    frame_field = _IK_FRAME_PATTERN.fullmatch(fields[0])
    if frame_field is None:
        raise ValueError("IK frame must be a non-negative integer")
    frame = int(frame_field.group(1))
    try:
        source_time = float(fields[1])
        joints = [float(value) for value in fields[2:]]
    except ValueError as exc:
        raise ValueError("IK packet contains a non-number") from exc
    values = [source_time] + joints
    if source_time < 0.0 or not all(math.isfinite(value) for value in values):
        raise ValueError("IK time and joints must be finite and non-negative in time")
    if any(abs(value) > FLOAT32_MAX for value in joints):
        raise ValueError("IK joint exceeds Float32 ROS message range")
    return frame, source_time, joints


def parse_dof_packet(payload):
    """Decode and validate one exact 200-byte SendDof V4 datagram."""
    if not isinstance(payload, bytes) or len(payload) != DOF_SIZE:
        raise ValueError("SendDof packet must be exactly 200 bytes")
    magic, version, frame, flags, seconds, *tail = DOF_STRUCT.unpack(payload)
    modules = tail[:42]
    milliseconds = tail[42]
    if (magic, version, flags) != (DOF_MAGIC, DOF_VERSION, DOF_FLAGS):
        raise ValueError("SendDof header is invalid")
    if seconds < 0.0 or milliseconds < 0 or not math.isfinite(seconds):
        raise ValueError("SendDof timestamp is invalid")
    if abs(seconds * 1000.0 - float(milliseconds)) > TIMESTAMP_TOLERANCE_MS:
        raise ValueError("SendDof second/millisecond timestamps disagree")
    if not all(math.isfinite(value) for value in modules):
        raise ValueError("SendDof module data must be finite")
    poses = [list(modules[index:index + 7]) for index in range(0, 42, 7)]
    for pose in poses:
        norm = math.sqrt(sum(value * value for value in pose[3:7]))
        if not QUATERNION_NORM_MIN <= norm <= QUATERNION_NORM_MAX:
            raise ValueError("SendDof quaternion norm is invalid")
    return frame, seconds, milliseconds, poses


def parse_module_map(value, parameter_name):
    """Validate a thumb/index/middle/ring/pinky mapping to modules A--F."""
    if value in (None, ""):
        return []
    if isinstance(value, str):
        value = [item.strip() for item in value.split(",") if item.strip()]
    if not isinstance(value, (list, tuple)):
        raise ValueError("%s must be a list" % parameter_name)
    result = [str(item).strip().upper() for item in value]
    if not result:
        return []
    if (len(result) != 5 or len(set(result)) != 5
            or any(item not in MODULE_NAMES for item in result)):
        raise ValueError(
            "%s must contain five distinct module names selected from A-F" % parameter_name)
    return result


class SequenceGate:
    """Monotonic frame gate with a bounded, conservative restart recovery."""

    def __init__(self, restart_timeout, relative_time):
        self.restart_timeout = float(restart_timeout)
        self.relative_time = bool(relative_time)
        self.last_frame = None
        self.last_source_time = None
        self.last_arrival = None
        self._guard_until = None
        self._old_frame = None
        self._old_source_time = None
        self._candidate_frame = None
        self._candidate_source_time = None
        self._candidate_count = 0

    def accept(self, frame, source_time, arrival):
        if self.last_frame is None:
            self._record(frame, source_time, arrival)
            return True

        in_guard = self._guard_until is not None and arrival < self._guard_until
        if in_guard:
            # Packets buffered from the previous process epoch must not be
            # mistaken for a forward jump in the freshly restarted stream.
            if frame >= self._old_frame:
                return False
            if self.relative_time and source_time >= self._old_source_time:
                return False
        elif self._guard_until is not None:
            self._guard_until = None
            self._old_frame = None
            self._old_source_time = None

        # A valid forward packet from the established epoch always wins.  A
        # single delayed low packet must not poison the live stream.  Without
        # a sender session ID, a restarted stream that has already crossed both
        # old counters is indistinguishable and is safely treated as forward.
        if frame > self.last_frame and source_time >= self.last_source_time:
            self._clear_candidate()
            self._record(frame, source_time, arrival)
            return True

        if self._candidate_frame is not None:
            if (frame > self._candidate_frame
                    and source_time >= self._candidate_source_time):
                self._candidate_frame = frame
                self._candidate_source_time = source_time
                self._candidate_count += 1
                if (self._candidate_count >= 2
                        and arrival - self.last_arrival >= self.restart_timeout):
                    old_frame = self.last_frame
                    old_source_time = self.last_source_time
                    self._record(frame, source_time, arrival)
                    self._guard_until = arrival + self.restart_timeout
                    self._old_frame = old_frame
                    self._old_source_time = old_source_time
                    self._clear_candidate()
                    return True
            return False

        # UDP may lose the first packets after restart. After the established
        # stream has timed out, allow any lower counter to seed the same
        # two-frame confirmation, instead of waiting to see frame 0..10.
        idle = arrival - self.last_arrival >= self.restart_timeout
        low_frame = (frame < self.last_frame
                     and (frame <= RESTART_MAX_FRAME or idle))
        if self.relative_time:
            source_reset = ((source_time <= RESTART_MAX_RELATIVE_TIME or idle)
                            and source_time < self.last_source_time)
        else:
            # SendDof uses wall-clock time, so a restarted sender resets only
            # FrameId while its timestamp remains monotonic.
            source_reset = source_time >= self.last_source_time
        if low_frame and source_reset:
            self._candidate_frame = frame
            self._candidate_source_time = source_time
            self._candidate_count = 1
        return False

    def _record(self, frame, source_time, arrival):
        self.last_frame = frame
        self.last_source_time = source_time
        self.last_arrival = arrival

    def _clear_candidate(self):
        self._candidate_frame = None
        self._candidate_source_time = None
        self._candidate_count = 0


class SG100WindowsUdpNode(linux_bridge.SG100GloveNode):
    """Receive IKHand and SendDof on one UDP socket per enabled hand."""

    def __init__(self):
        # Do not call SG100GloveNode.__init__: that constructor resolves and
        # prepares the Linux SDK/serial runtime.
        requested_side = rospy.get_param("~side", "both")
        if requested_side not in linux_bridge.SUPPORTED_SIDES:
            raise ValueError("~side must be left, right, or both")
        self.active_hands = ("left", "right") if requested_side == "both" else (requested_side,)
        # Use the side-tagged SDK line format consistently, even with one hand.
        self.side = "both"
        self.hand_board_ids = {hand: board for hand, board in (("left", 0), ("right", 1))
                               if hand in self.active_hands}
        self.command_topic = rospy.get_param(
            "~command_topic", "/sg100_sim_hand_command")
        self.left_command_topic = rospy.get_param(
            "~left_command_topic", "/sg100_sim_left_hand_command")
        self.real_command_topic = rospy.get_param(
            "~real_command_topic", "/sg100_hand_command")
        self.publish_real = bool(rospy.get_param("~publish_real", True))
        # ROS parameters survive a launch process while the master remains up.
        # Require the dedicated launch entry marker as well, so a stale private
        # parameter cannot silently change legacy Windows entry behaviour.
        self.windows_entry_safety = (
            os.environ.get("KUAVO_WINDOWS_GLOVE_ENTRY") == "1"
            and bool(rospy.get_param("~windows_entry_safety", False)))
        self.ik_freshness_timeout = 0.5
        if self.windows_entry_safety:
            try:
                self.ik_freshness_timeout = float(
                    rospy.get_param("~ik_freshness_timeout", 0.5))
            except (TypeError, ValueError) as exc:
                raise ValueError(
                    "~ik_freshness_timeout must be finite and positive") from exc
            if (not math.isfinite(self.ik_freshness_timeout)
                    or self.ik_freshness_timeout <= 0.0):
                raise ValueError(
                    "~ik_freshness_timeout must be finite and positive")
        self.emit_gesture_poses = bool(
            rospy.get_param("~emit_gesture_poses", True))
        self.calibration_capture_all_dof = bool(
            rospy.get_param("~calibration_capture_all_dof", False))
        self.bind_address = rospy.get_param("~bind_address", "0.0.0.0")
        self.restart_timeout = float(rospy.get_param("~restart_timeout", 2.0))
        if not math.isfinite(self.restart_timeout) or self.restart_timeout <= 0.0:
            raise ValueError("~restart_timeout must be finite and positive")
        self.diagnostics = bool(rospy.get_param("~diagnostics", False))
        self.diagnostic_period = float(rospy.get_param("~diagnostic_period", 2.0))
        if not math.isfinite(self.diagnostic_period) or self.diagnostic_period <= 0.0:
            raise ValueError("~diagnostic_period must be finite and positive")

        self.module_maps = {
            hand: parse_module_map(rospy.get_param("~%s_module_map" % hand, []),
                                   "~%s_module_map" % hand)
            for hand in self.active_hands
        }
        ports = {}
        for hand in self.active_hands:
            parameter = "~" + hand + "_port"
            port = rospy.get_param(parameter, None)
            if type(port) is not int or not 1 <= port <= 65535:
                raise ValueError("%s must be configured as an integer in 1..65535" % parameter)
            ports[hand] = port
        if len(set(ports.values())) != len(ports):
            raise ValueError("enabled UDP ports must be distinct")
        self.ports = ports

        joint_topics = {"left": self.left_command_topic, "right": self.command_topic}
        self.joint_publishers = {
            hand: rospy.Publisher(joint_topics[hand], Float32MultiArray, queue_size=1)
            for hand in self.active_hands}
        self.real_positions = {"left": None, "right": None}
        self.real_publisher = None
        if self.publish_real:
            if linux_bridge.SG100HandCommand is None:
                rospy.logerr(
                    "[sg100_windows_udp] cannot import SG100HandCommand; "
                    "real-hand output is disabled")
            else:
                self.real_publisher = rospy.Publisher(
                    self.real_command_topic, linux_bridge.SG100HandCommand,
                    queue_size=1)
        self.pose_publishers = {}
        if self.emit_gesture_poses:
            for hand in self.active_hands:
                self.pose_publishers[hand] = rospy.Publisher(
                    linux_bridge.FINGERTIP_TOPICS[hand],
                    Float32MultiArray,
                    queue_size=1000 if self.calibration_capture_all_dof else 1)

        self._frames = 0
        self._pose_frames = 0
        self._bad_lines = 0
        self._joint_lock = threading.Lock()
        self._publish_lock = threading.RLock()
        self._interaction_mode = False
        self._freeze_q = {}
        self._latest_joint = {}
        self._ik_arrivals = {}
        self._closed = False
        self._last_diagnostic = {"left": 0.0, "right": 0.0}
        self._last_module_xyz = {"left": None, "right": None}
        self._gates = {
            (hand, stream): SequenceGate(self.restart_timeout, stream == "ik")
            for hand in self.active_hands for stream in ("ik", "dof")
        }
        self._sockets = {}
        self._socket_hands = {}
        try:
            for hand, port in ports.items():
                udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                try:
                    udp_socket.setblocking(False)
                    udp_socket.bind((self.bind_address, port))
                except Exception:
                    udp_socket.close()
                    raise
                self._sockets[hand] = udp_socket
                self._socket_hands[udp_socket] = hand
        except Exception:
            self.close()
            raise
        self._interaction_sub = rospy.Subscriber(
            "/glove_gesture/interaction_mode", Bool,
            self._on_interaction_mode, queue_size=1)
        rospy.on_shutdown(self.close)

    def _on_interaction_mode(self, msg):
        with self._publish_lock:
            was_enabled = self._interaction_mode
            linux_bridge.SG100GloveNode._on_interaction_mode(self, msg)
            if was_enabled and not self._interaction_mode:
                # UDP keeps consuming packets while frozen.  Resume immediately
                # with the newest target, even if the sender pauses at this point.
                if self.windows_entry_safety:
                    now = time.monotonic()
                    self._sweep_expired_ik(now)
                    fresh_targets = {}
                    for hand in self.active_hands:
                        latest = self._latest_joint.get(hand)
                        if latest is not None and self._ik_is_fresh(hand, now):
                            fresh_targets[hand] = latest
                        else:
                            self._drop_hand_cache(hand)
                    # Populate every fresh target before the first combined
                    # real command.  This avoids one side briefly replaying an
                    # older frozen target while the other side is restored.
                    for hand, latest in fresh_targets.items():
                        self.real_positions[hand] = list(latest)
                    for hand, latest in fresh_targets.items():
                        self._publish_joint(hand, latest)
                    return
                for hand in self.active_hands:
                    latest = self._latest_joint.get(hand)
                    if latest is not None:
                        self._publish_joint(hand, latest)

    def _ik_is_fresh(self, hand, now):
        arrival = self._ik_arrivals.get(hand)
        if arrival is None:
            return False
        age = now - arrival
        return 0.0 <= age <= self.ik_freshness_timeout

    def _drop_hand_cache(self, hand):
        self._latest_joint.pop(hand, None)
        self._ik_arrivals.pop(hand, None)
        self._freeze_q.pop(hand, None)
        self.real_positions[hand] = None

    def _accept_ik_target(self, hand, joints, arrival):
        if not self._ik_is_fresh(hand, arrival):
            # A reconnect starts a new target epoch.  Never carry an old
            # frozen target or real-hand cache into that epoch.
            self._drop_hand_cache(hand)
        self._ik_arrivals[hand] = arrival
        self._latest_joint[hand] = list(joints)
        self._publish_joint(hand, joints)

    def _expire_stale_hands(self, now):
        expired = [
            hand for hand in self.active_hands
            if hand in self._ik_arrivals and not self._ik_is_fresh(hand, now)
        ]
        for hand in expired:
            self._drop_hand_cache(hand)
        return bool(expired)

    def _sweep_expired_ik(self, now=None):
        if not self.windows_entry_safety:
            return False
        if now is None:
            now = time.monotonic()
        if not self._expire_stale_hands(now):
            return False
        self._publish_real_state(now)
        return True

    def _publish_real_state(self, now):
        if self.real_publisher is None or linux_bridge.SG100HandCommand is None:
            return
        cmd = linux_bridge.SG100HandCommand()
        cmd.header.stamp = rospy.Time.now()
        cmd.control_mode = linux_bridge.SG100HandCommand.MODE_JOINT_POSITION
        left = self.real_positions["left"]
        right = self.real_positions["right"]
        # The driver validates both arrays before applying enable masks.
        cmd.left_hand_positions = (
            left if left is not None else [0.0] * linux_bridge.JOINT_COUNT)
        cmd.right_hand_positions = (
            right if right is not None else [0.0] * linux_bridge.JOINT_COUNT)
        cmd.source_id = 1
        cmd.left_enable_mask = (
            linux_bridge.REAL_HAND_ENABLE_MASK
            if left is not None and self._ik_is_fresh("left", now) else 0)
        cmd.right_enable_mask = (
            linux_bridge.REAL_HAND_ENABLE_MASK
            if right is not None and self._ik_is_fresh("right", now) else 0)
        self.real_publisher.publish(cmd)

    def _publish_real_command(self, hand, q):
        if not self.windows_entry_safety:
            return linux_bridge.SG100GloveNode._publish_real_command(self, hand, q)
        if self.real_publisher is None or linux_bridge.SG100HandCommand is None:
            return
        if hand not in self.real_positions:
            return
        self.real_positions[hand] = list(q)
        now = time.monotonic()
        # Fold an expired peer into this same combined command, instead of
        # first publishing a redundant all-zero release and then the new hand.
        self._expire_stale_hands(now)
        self._publish_real_state(now)

    def _publish_joint(self, hand, joints):
        # An IK stream can start after DoF gestures have already entered
        # interaction mode. Wait for exit if there is no target to hold.
        if self._interaction_mode and hand not in self._freeze_q:
            return
        board_id = self.hand_board_ids[hand]
        fields = ["J", hand, str(board_id), "0"] + [str(value) for value in joints]
        linux_bridge.SG100GloveNode._handle_joint_line(self, fields)

    def _publish_pose(self, hand, frame, poses):
        mapping = self.module_maps[hand]
        if not mapping:
            return
        flattened = []
        for module_name in mapping:
            flattened.extend(poses[MODULE_NAMES.index(module_name)])
        fields = (["P", hand, str(self.hand_board_ids[hand]), str(frame)]
                  + [str(value) for value in flattened])
        linux_bridge.SG100GloveNode._handle_pose_line(self, fields)

    def _decode(self, hand, payload, arrival):
        stream = "unknown"
        try:
            if payload.startswith(IK_PREFIX):
                stream = "ik"
                frame, source_time, data = parse_ik_packet(payload)
            elif payload.startswith(DOF_PREFIX):
                stream = "dof"
                frame, source_time, _milliseconds, poses = parse_dof_packet(payload)
                data = (frame, poses)
            else:
                raise ValueError("expected SG100 text tag or SendDof V4 header")
            if self._gates[(hand, stream)].accept(frame, source_time, arrival):
                return stream, data
        except ValueError as exc:
            self._bad_lines += 1
            if self._bad_lines % 100 == 1:
                rospy.logwarn("[sg100_windows_udp] rejected %s/%s packet: %s (total=%d)",
                              hand, stream, exc, self._bad_lines)
        return None

    def poll_once(self, timeout=0.02):
        """Drain ports fairly; normally publish newest, or every accepted calibration DoF."""
        if self._closed or not self._socket_hands:
            return 0
        try:
            ready, _, _ = select.select(list(self._socket_hands), [], [], timeout)
        except (OSError, ValueError):
            if self._closed or rospy.is_shutdown():
                return 0
            raise
        published = 0
        for udp_socket in ready:
            hand = self._socket_hands.get(udp_socket)
            if hand is None:
                if self._closed or rospy.is_shutdown():
                    return published
                continue
            newest = {}
            newest_ik_arrival = None
            calibration_dof = []
            for _ in range(MAX_DRAIN_PER_SOCKET):
                try:
                    payload, _peer = udp_socket.recvfrom(65535)
                except BlockingIOError:
                    break
                except OSError:
                    if self._closed or rospy.is_shutdown():
                        return published
                    raise
                arrival = time.monotonic()
                decoded = self._decode(hand, payload, arrival)
                if decoded is not None:
                    stream, data = decoded
                    if stream == "dof" and self.calibration_capture_all_dof:
                        calibration_dof.append(data)
                    else:
                        newest[stream] = data
                        if stream == "ik":
                            newest_ik_arrival = arrival
            if not newest and not calibration_dof:
                continue
            with self._publish_lock:
                for stream, data in newest.items():
                    if stream == "ik":
                        if self.windows_entry_safety:
                            self._accept_ik_target(
                                hand, data, newest_ik_arrival)
                        else:
                            self._latest_joint[hand] = list(data)
                            self._publish_joint(hand, data)
                    else:
                        frame, poses = data
                        self._publish_pose(hand, frame, poses)
                        self._diagnose_modules(hand, poses)
                # Calibration needs the raw accepted cadence.  SequenceGate
                # already removed duplicates/reorders; publish each remaining
                # DoF exactly once and never alter the joint/real-hand path.
                for frame, poses in calibration_dof:
                    self._publish_pose(hand, frame, poses)
                    self._diagnose_modules(hand, poses)
            published += len(newest) + len(calibration_dof)
        if self.windows_entry_safety:
            with self._publish_lock:
                self._sweep_expired_ik()
        return published

    def _diagnose_modules(self, hand, poses):
        if not self.diagnostics:
            return
        now = time.monotonic()
        if now - self._last_diagnostic[hand] < self.diagnostic_period:
            return
        xyz = [tuple(pose[:3]) for pose in poses]
        previous = self._last_module_xyz[hand]
        deltas = None if previous is None else [
            tuple(current[i] - old[i] for i in range(3))
            for current, old in zip(xyz, previous)
        ]
        labelled_xyz = dict(zip(MODULE_NAMES, xyz))
        labelled_deltas = (None if deltas is None
                           else dict(zip(MODULE_NAMES, deltas)))
        rospy.loginfo("[sg100_windows_udp] %s modules xyz=%s delta=%s",
                      hand, labelled_xyz, labelled_deltas)
        self._last_module_xyz[hand] = xyz
        self._last_diagnostic[hand] = now

    def run(self):
        for hand in self.active_hands:
            rospy.loginfo("[sg100_windows_udp] %s listening on %s:%d (IKHand + SendDof)",
                          hand, self.bind_address, self.ports[hand])
            mapping = self.module_maps[hand]
            if mapping:
                rospy.loginfo(
                    "[sg100_windows_udp] %s fingertips mapping "
                    "thumb/index/middle/ring/pinky=%s",
                    hand, ",".join(mapping))
            else:
                rospy.logwarn(
                    "[sg100_windows_udp] %s IK is available, fingertips are "
                    "disabled; configure ~%s_module_map with five distinct A-F modules",
                    hand, hand)
        try:
            while not rospy.is_shutdown() and not self._closed:
                self.poll_once()
        finally:
            self.close()

    def close(self):
        if getattr(self, "_closed", False):
            return
        self._closed = True
        for udp_socket in list(getattr(self, "_sockets", {}).values()):
            udp_socket.close()
        self._sockets = {}
        self._socket_hands = {}


def main():
    rospy.init_node("sg100_windows_udp_bridge")
    node = SG100WindowsUdpNode()
    node.run()


if __name__ == "__main__":
    main()
