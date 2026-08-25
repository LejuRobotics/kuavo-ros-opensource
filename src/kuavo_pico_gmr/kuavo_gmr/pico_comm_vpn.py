#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Authoritative teleoperation session manager and gated PICO UDP receiver."""

import ipaddress
import json
import math
import os
import queue
import re
import socket
import sys
import threading
import time
import xmlrpc.client
from dataclasses import dataclass
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional

def _read_upper_wg_ip():
    """Read upper computer WG IP from file written by vpn_register.py"""
    try:
        with open("/etc/wireguard/upper_peer_ip") as f:
            ip = f.read().strip()
        if re.match(r"^\d+\.\d+\.\d+\.\d+$", ip):
            return ip
    except Exception:
        pass
    return "10.77.0.13"  # fallback for old robots

UPPER_WG_IP = _read_upper_wg_ip()


import requests
import rosgraph
import rospy
from kuavo_msgs.msg import ControllerSwitchEvent, JoySticks, robotHeadMotionData
from kuavo_msgs.srv import getControllerList
from kuavo_ros_interfaces.msg import robotHandPosition
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float64MultiArray, String
from std_srvs.srv import SetBool, SetBoolRequest, Trigger, TriggerRequest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import body_tracking_extended_pb2 as proto
from pico_comm_minimal import MinimalPicoReceiver
from pico_diagnostic_runtime import ReceivedDatagram, monotonic_ns


IDLE_AMP = "IDLE_AMP"
PICO_JOYSTICK_STARTING = "PICO_JOYSTICK_STARTING"
PICO_JOYSTICK_ACTIVE = "PICO_JOYSTICK_ACTIVE"
PICO_JOYSTICK_STOPPING = "PICO_JOYSTICK_STOPPING"
STARTING = "STARTING"
SWITCHING_TO_VMP = "SWITCHING_TO_VMP"
STARTING_RECORDERS = "STARTING_RECORDERS"
TELEOPERATING = "TELEOPERATING"
STOPPING_INPUT = "STOPPING_INPUT"
STOPPING_RECORDERS = "STOPPING_RECORDERS"
SWITCHING_TO_AMP = "SWITCHING_TO_AMP"
FINALIZING = "FINALIZING"
ERROR = "ERROR"

AMP_FAMILY_CONTROLLERS = frozenset(("amp_controller", "amp_wild_controller"))


class _TimeoutTransport(xmlrpc.client.Transport):
    """Bound ROS Master XML-RPC calls used by bridge readiness checks."""

    def __init__(self, timeout_s):
        super().__init__()
        self._timeout_s = timeout_s

    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = self._timeout_s
        return connection


def utc_now():
    return datetime.utcnow().isoformat(timespec="seconds") + "Z"


@dataclass
class SessionCommand:
    action: str
    task_id: str
    vr_ip: Optional[str]
    reason: str
    response: "queue.Queue"


class SessionManager:
    """All state transitions run on one worker; callbacks only enqueue events."""

    def __init__(self):
        self.http_host = rospy.get_param("~http_host", "0.0.0.0")
        self.http_port = int(rospy.get_param("~http_port", 9090))
        self.allowed_vr_network = ipaddress.ip_network(
            rospy.get_param("~allowed_vr_network", "10.77.0.0/24")
        )
        self.upper_precheck_url = rospy.get_param(
            "~upper_precheck_url",
            f"http://{UPPER_WG_IP}:9090/api/teleop/teleop_check",
        )
        self.recorder_base_url = rospy.get_param(
            "~recorder_base_url", f"http://{UPPER_WG_IP}:9092"
        ).rstrip("/")
        self.http_timeout = float(rospy.get_param("~peer_http_timeout", 10.0))
        self.controller_timeout = float(
            rospy.get_param("~controller_timeout", 15.0)
        )
        self.require_switch_event = bool(
            rospy.get_param("~require_switch_event", True)
        )
        self.teleop_state_topic = rospy.get_param(
            "~teleop_state_topic", "/pico/teleop_state"
        )
        self.require_upper_precheck = bool(
            rospy.get_param("~require_upper_precheck", True)
        )
        self.dry_run = bool(rospy.get_param("~dry_run", False))
        self.teleop_idle_timeout = float(
            rospy.get_param("~teleop_idle_timeout", 5.0)
        )
        self.pico_joystick_idle_timeout = float(
            rospy.get_param("~pico_joystick_idle_timeout", 5.0)
        )
        self.pico_joystick_command_timeout = float(
            rospy.get_param("~pico_joystick_command_timeout", 0.3)
        )
        self.pico_joystick_stop_neutral_duration = float(
            rospy.get_param("~pico_joystick_stop_neutral_duration", 0.5)
        )
        self.pico_joystick_gait_timeout = float(
            rospy.get_param("~pico_joystick_gait_timeout", 5.0)
        )
        self.authorized_joy_topic = rospy.get_param(
            "~authorized_joy_topic", "/pico/joy_amp_authorized"
        )
        self.converted_joy_topic = rospy.get_param(
            "~converted_joy_topic", "/pico/joy_converted"
        )
        self.g12_joy_topic = rospy.get_param("~g12_joy_topic", "/joy")
        self.g12_axis_deadzone = float(
            rospy.get_param("~g12_axis_deadzone", 0.05)
        )
        self.g12_input_timeout = float(
            rospy.get_param("~g12_input_timeout", 0.2)
        )
        self.g12_neutral_hold = float(
            rospy.get_param("~g12_neutral_hold", 0.3)
        )
        self.bridge_output_timeout = float(
            rospy.get_param("~bridge_output_timeout", 1.0)
        )
        self.bridge_start_timeout = float(
            rospy.get_param("~bridge_start_timeout", 3.0)
        )
        self.bridge_master_timeout = float(
            rospy.get_param("~bridge_master_timeout", 0.5)
        )
        self.bridge_graph_check_interval = float(
            rospy.get_param("~bridge_graph_check_interval", 1.0)
        )
        self.pico_converter_node = rospy.get_param(
            "~pico_converter_node", "/pico_gmr_joy_converter"
        )
        self.auto_gait_node = rospy.get_param(
            "~auto_gait_node", "/humanoid_joy_control_auto_gait_with_vel"
        )

        self._lock = threading.RLock()
        self._state = IDLE_AMP
        self._active_task_id = None
        self._active_vr_ip = None
        self._allowed_vr_ip = None
        self._started_at = None
        self._last_activity_at = None
        self._last_packet_time = 0.0
        self._last_joystick_at = None
        self._last_joystick_packet_time = 0.0
        self._joystick_zeroed = True
        self._last_joystick_command = {
            "left_x": 0.0,
            "left_y": 0.0,
            "right_x": 0.0,
            "right_y": 0.0,
        }
        self._disconnect_reason = None
        self._last_task_id = None
        self._last_error = ""
        self._recorder_started = False
        self._last_switch_event = None
        self._active_controller_name = None
        self._rl_walking = None
        self._last_rl_command_state_time = 0.0
        self._last_converted_joy_time = 0.0
        self._last_bridge_graph_check_time = 0.0
        self._bridge_ready = False
        self._bridge_error = "not checked"
        self._g12_has_intent = False
        self._last_g12_input_time = 0.0
        self._last_g12_active_time = 0.0

        self._commands = queue.Queue()
        self._shutdown = threading.Event()
        self._http_server = None
        self.teleop_state_pub = rospy.Publisher(
            self.teleop_state_topic, Bool, queue_size=1, latch=True
        )
        self.amp_switch_pub = rospy.Publisher(
            "/humanoid_controller/nav_switch_rl_controller_by_name",
            String,
            queue_size=1,
        )
        self.authorized_joy_pub = rospy.Publisher(
            self.authorized_joy_topic, JoySticks, queue_size=1
        )
        self.gait_name_pub = rospy.Publisher(
            "/humanoid_mpc_gait_name_request", String, queue_size=1
        )
        self.teleop_state_pub.publish(Bool(False))
        rospy.Subscriber(
            "/humanoid_controller/controller_switch_event",
            ControllerSwitchEvent,
            self._on_switch_event,
            queue_size=10,
        )
        rospy.Subscriber(
            "/rl_controller/InputData/command_scalar_state",
            Float64MultiArray,
            self._on_rl_command_state,
            queue_size=10,
        )
        rospy.Subscriber(
            self.converted_joy_topic,
            Joy,
            self._on_converted_joy,
            queue_size=10,
        )
        rospy.Subscriber(
            self.g12_joy_topic,
            Joy,
            self._on_g12_joy,
            queue_size=10,
        )

        if not self.dry_run:
            try:
                self._get_active_controller()
            except Exception as exc:
                rospy.logwarn("Initial controller query failed: %s", exc)

        self._worker = threading.Thread(
            target=self._command_loop, daemon=True, name="teleop-session-worker"
        )
        self._worker.start()
        self._start_http_server()

        # A newly started manager always begins closed. This does not switch modes.
        self._allowed_vr_ip = None
        if not self.dry_run:
            try:
                self._pause_pico_input(True)
            except Exception as exc:
                rospy.logwarn("Initial Pico input gate could not be closed: %s", exc)

    def _on_switch_event(self, message):
        with self._lock:
            self._active_controller_name = str(message.to_controller).strip()
            if self._active_controller_name == "amp_controller":
                self._rl_walking = False
            self._last_switch_event = {
                "from": message.from_controller,
                "to": message.to_controller,
                "at": utc_now(),
            }

    def _on_rl_command_state(self, message):
        # command_scalar_state = [vx, vy, wz, 1 - cmdStance].
        if len(message.data) < 4:
            return
        with self._lock:
            self._rl_walking = float(message.data[3]) >= 0.5
            self._last_rl_command_state_time = time.monotonic()

    def _on_converted_joy(self, _message):
        with self._lock:
            self._last_converted_joy_time = time.monotonic()

    def _on_g12_joy(self, message):
        now = time.monotonic()
        valid_layout = len(message.axes) == 8 and len(message.buttons) == 11
        axes_valid = valid_layout and all(
            math.isfinite(float(value)) and -1.001 <= float(value) <= 1.001
            for value in message.axes
        )
        buttons_valid = valid_layout and all(
            int(value) in (0, 1) for value in message.buttons
        )
        has_intent = bool(
            axes_valid
            and buttons_valid
            and (
                any(abs(float(value)) > self.g12_axis_deadzone for value in message.axes)
                or any(int(value) != 0 for value in message.buttons)
            )
        )
        with self._lock:
            self._last_g12_input_time = now
            self._g12_has_intent = has_intent
            if has_intent:
                self._last_g12_active_time = now

    def _g12_active_locked(self, now=None):
        now = time.monotonic() if now is None else now
        return bool(
            self._g12_has_intent
            and self._last_g12_input_time > 0.0
            and now - self._last_g12_input_time < self.g12_input_timeout
        )

    def _g12_blocks_transition_locked(self, now=None):
        now = time.monotonic() if now is None else now
        if self._g12_active_locked(now):
            return True
        return bool(
            self._last_g12_active_time > 0.0
            and now - self._last_g12_active_time < self.g12_neutral_hold
        )

    def _set_state(self, state, error=""):
        with self._lock:
            self._state = state
            if error:
                self._last_error = error

    def status(self):
        with self._lock:
            pico_joystick_active = self._state == PICO_JOYSTICK_ACTIVE
            teleop_active = self._state == TELEOPERATING
            packet_age = None
            joystick_age = None
            converted_joy_age = None
            g12_input_age = None
            now = time.monotonic()
            if self._last_packet_time > 0.0:
                packet_age = max(0.0, now - self._last_packet_time)
            if self._last_joystick_packet_time > 0.0:
                joystick_age = max(0.0, now - self._last_joystick_packet_time)
            if self._last_converted_joy_time > 0.0:
                converted_joy_age = max(
                    0.0, now - self._last_converted_joy_time
                )
            if self._last_g12_input_time > 0.0:
                g12_input_age = max(0.0, now - self._last_g12_input_time)
            return {
                "code": 0,
                "state": self._state,
                "input_mode": (
                    "PICO_JOYSTICK"
                    if pico_joystick_active
                    else "TELEOP" if teleop_active else "NONE"
                ),
                "pico_joystick_active": pico_joystick_active,
                "teleop_active": teleop_active,
                "authorized": (
                    teleop_active
                    and self._allowed_vr_ip == self._active_vr_ip
                ),
                "pico_joystick_authorized": (
                    pico_joystick_active
                    and self._allowed_vr_ip == self._active_vr_ip
                ),
                "activeController": self._active_controller_name,
                "picoJoystickController": self._active_controller_name,
                "ampFamilyActive": (
                    self._active_controller_name in AMP_FAMILY_CONTROLLERS
                ),
                "picoJoystickWalking": self._rl_walking,
                "activeTaskId": self._active_task_id,
                "vr_device_ip": self._active_vr_ip,
                "startedAt": self._started_at,
                "lastActivityAt": self._last_activity_at,
                "lastJoystickAt": self._last_joystick_at,
                "lastJoystickCommand": dict(self._last_joystick_command),
                "packetIdleSeconds": packet_age,
                "joystickIdleSeconds": joystick_age,
                "authorizedJoyTopic": self.authorized_joy_topic,
                "convertedJoyTopic": self.converted_joy_topic,
                "convertedJoyIdleSeconds": converted_joy_age,
                "bridgeReady": self._bridge_ready,
                "bridgeError": self._bridge_error,
                "g12Active": self._g12_active_locked(now),
                "g12BlocksTransition": self._g12_blocks_transition_locked(now),
                "g12InputIdleSeconds": g12_input_age,
                "disconnectReason": self._disconnect_reason,
                "lastTaskId": self._last_task_id,
                "lastError": self._last_error,
                "lastSwitchEvent": self._last_switch_event,
                "safety": {
                    "picoHeartbeat": (
                        "active"
                        if pico_joystick_active or teleop_active
                        else "inactive"
                    ),
                    "joystickCommandZeroed": self._joystick_zeroed,
                    "hardwareEstop": "reserved",
                    "fallDetection": "reserved",
                    "standingCheck": "reserved",
                },
            }

    def input_mode(self, source_ip):
        with self._lock:
            authorized = (
                self._state in (PICO_JOYSTICK_ACTIVE, TELEOPERATING)
                and self._allowed_vr_ip is not None
                and source_ip == self._allowed_vr_ip
            )
            if not authorized:
                return None
            self._last_activity_at = utc_now()
            self._last_packet_time = time.monotonic()
            return self._state

    def input_allowed(self, source_ip):
        """Backward-compatible full teleoperation gate."""
        return self.input_mode(source_ip) == TELEOPERATING

    def submit(self, action, task_id, vr_ip=None, reason="", timeout=60.0):
        response = queue.Queue(maxsize=1)
        self._commands.put(
            SessionCommand(action, task_id, vr_ip, reason, response)
        )
        try:
            return response.get(timeout=timeout)
        except queue.Empty:
            return 504, {
                "code": 504,
                "teleop_active": False,
                "message": "%s timed out" % action,
            }

    def _command_loop(self):
        while not self._shutdown.is_set() and not rospy.is_shutdown():
            try:
                command = self._commands.get(timeout=0.1)
            except queue.Empty:
                self._check_session_timeouts()
                continue
            try:
                if command.action == "start":
                    result = self._start(command.task_id, command.vr_ip)
                elif command.action == "stop":
                    result = self._stop(command.task_id, command.reason)
                elif command.action == "pico_joystick_start":
                    result = self._start_pico_joystick(
                        command.task_id, command.vr_ip
                    )
                elif command.action == "pico_joystick_stop":
                    result = self._stop_pico_joystick(
                        command.task_id, command.reason
                    )
                else:
                    result = (400, {"code": 400, "message": "unknown action"})
            except Exception as exc:
                rospy.logerr("Unhandled session transition error: %s", exc)
                self._set_state(ERROR, str(exc))
                result = (
                    500,
                    {
                        "code": 500,
                        "teleop_active": False,
                        "message": str(exc),
                    },
                )
            command.response.put(result)

    def _check_session_timeouts(self):
        with self._lock:
            state = self._state
            task_id = self._active_task_id
            now = time.monotonic()
            packet_idle = now - self._last_packet_time
            joystick_idle = now - self._last_joystick_packet_time
            should_zero_joystick = (
                state == PICO_JOYSTICK_ACTIVE
                and not self._joystick_zeroed
                and joystick_idle > self.pico_joystick_command_timeout
            )
            should_stop_joystick = (
                state == PICO_JOYSTICK_ACTIVE
                and joystick_idle > self.pico_joystick_idle_timeout
            )
            should_stop_teleop = (
                state == TELEOPERATING
                and packet_idle > self.teleop_idle_timeout
            )
            g12_interrupt = (
                state == PICO_JOYSTICK_ACTIVE
                and self._g12_active_locked(now)
            )

        if g12_interrupt:
            rospy.logwarn("G12 input took priority; revoking PICO joystick lease")
            self._interrupt_pico_joystick_for_g12(task_id)
            return

        if state == PICO_JOYSTICK_ACTIVE:
            bridge_error = self._bridge_runtime_error()
            self._set_bridge_status(bridge_error)
            if bridge_error:
                rospy.logerr("PICO Joy bridge lost: %s", bridge_error)
                self._stop_pico_joystick(task_id, "pico_joy_bridge_lost")
                return

        if should_zero_joystick:
            rospy.logwarn(
                "PICO joystick command timeout after %.3fs, publishing neutral Joy",
                joystick_idle,
            )
            self._publish_neutral_joy()
            with self._lock:
                if self._state == PICO_JOYSTICK_ACTIVE:
                    self._joystick_zeroed = True

        if should_stop_joystick:
            rospy.logwarn(
                "PICO joystick idle timeout after %.1fs (limit %.1fs), "
                "auto-stopping",
                joystick_idle,
                self.pico_joystick_idle_timeout,
            )
            self._stop_pico_joystick(task_id, "pico_joystick_idle_timeout")
            return

        if not should_stop_teleop:
            return
        rospy.logwarn(
            "Teleoperation idle timeout after %.1fs (limit %.1fs), auto-stopping",
            packet_idle,
            self.teleop_idle_timeout,
        )
        self._stop(task_id, "vr_idle_timeout")

    @staticmethod
    def _validated_pico_axis(value):
        value = float(value)
        if not math.isfinite(value) or value < -1.001 or value > 1.001:
            raise ValueError("PICO joystick axis is non-finite or out of range")
        return max(-1.0, min(1.0, value))

    def handle_pico_joystick(
        self, source_ip, left_x, left_y, right_x, right_y
    ):
        """Forward axes only while the matching PICO joystick lease is active."""
        try:
            axes = tuple(
                self._validated_pico_axis(value)
                for value in (left_x, left_y, right_x, right_y)
            )
        except (TypeError, ValueError) as exc:
            rospy.logwarn_throttle(1.0, "Reject invalid PICO joystick sample: %s", exc)
            self._publish_neutral_joy()
            return

        with self._lock:
            if not (
                self._state == PICO_JOYSTICK_ACTIVE
                and source_ip == self._allowed_vr_ip
            ):
                return
            left_x, left_y, right_x, right_y = axes
            self._last_joystick_packet_time = time.monotonic()
            self._last_joystick_at = utc_now()
            self._joystick_zeroed = (
                left_x == 0.0
                and left_y == 0.0
                and right_x == 0.0
                and right_y == 0.0
            )
            self._last_joystick_command = {
                "left_x": left_x,
                "left_y": left_y,
                "right_x": right_x,
                "right_y": right_y,
            }
            if not self.dry_run:
                self.authorized_joy_pub.publish(
                    self._make_authorized_joy(left_x, left_y, right_x, right_y)
                )

    def _start_pico_joystick(self, task_id, vr_ip):
        with self._lock:
            if (
                self._state == PICO_JOYSTICK_ACTIVE
                and task_id == self._active_task_id
                and vr_ip == self._active_vr_ip
            ):
                return 200, self._success("PICO joystick already started")
            if self._state != IDLE_AMP:
                return 409, self._failure(
                    "SESSION_CONFLICT",
                    "another session or transition is active",
                )
            if self._g12_blocks_transition_locked():
                return 409, self._failure(
                    "G12_ACTIVE",
                    "G12 input is active or has not been neutral long enough",
                )

        try:
            self._require_amp_family_active()
        except Exception as exc:
            return 409, self._failure("AMP_FAMILY_REQUIRED", str(exc))
        try:
            self._wait_bridge_ready()
        except Exception as exc:
            return 503, self._failure("PICO_JOY_BRIDGE_NOT_READY", str(exc))

        with self._lock:
            self._state = PICO_JOYSTICK_STARTING
            self._active_task_id = task_id
            self._active_vr_ip = vr_ip
            self._allowed_vr_ip = None
            self._started_at = utc_now()
            self._last_activity_at = self._started_at
            self._last_packet_time = time.monotonic()
            self._last_joystick_at = None
            self._last_joystick_packet_time = self._last_packet_time
            self._joystick_zeroed = True
            self._disconnect_reason = None
            self._last_error = ""
            self._recorder_started = False

        try:
            # Keep VMP input frozen. PICO joystick mode never releases the
            # full body/hand/head teleoperation pipeline.
            self._pause_pico_input(True)
            self._require_g12_neutral()
            self._require_amp_family_active()
            self._wait_bridge_ready()
            self._publish_neutral_joy()

            with self._lock:
                self._allowed_vr_ip = vr_ip
                self._state = PICO_JOYSTICK_ACTIVE
                self._last_activity_at = utc_now()
                self._last_packet_time = time.monotonic()
                self._last_joystick_packet_time = self._last_packet_time
            self.teleop_state_pub.publish(Bool(False))
            rospy.loginfo(
                "PICO joystick active: task=%s vr=%s", task_id, vr_ip
            )
            return 200, self._success("PICO joystick started")
        except Exception as exc:
            rospy.logerr("PICO joystick START failed: %s", exc)
            rollback_errors = self._rollback_pico_joystick_start()
            message = str(exc)
            if rollback_errors:
                message += "; rollback: " + "; ".join(rollback_errors)
            return 500, self._failure("PICO_JOYSTICK_START_FAILED", message)

    def _rollback_pico_joystick_start(self):
        errors = []
        with self._lock:
            self._allowed_vr_ip = None
        try:
            self._publish_neutral_joy(
                self.pico_joystick_stop_neutral_duration
            )
        except Exception as exc:
            errors.append("neutral Joy: %s" % exc)
        with self._lock:
            failed_task = self._active_task_id
            self._last_task_id = failed_task
            self._active_task_id = None
            self._active_vr_ip = None
            self._started_at = None
            self._last_activity_at = utc_now()
            self._last_joystick_at = None
            self._last_joystick_packet_time = 0.0
            self._joystick_zeroed = True
            self._disconnect_reason = "pico_joystick_start_failed"
            self._last_error = "; ".join(errors)
            self._state = IDLE_AMP if not errors else ERROR
        return errors

    def _stop_pico_joystick(self, task_id, reason):
        with self._lock:
            if self._state == IDLE_AMP:
                if task_id == self._last_task_id:
                    return 200, self._success("PICO joystick already stopped")
                return 409, self._failure(
                    "NO_MATCHING_SESSION", "no active PICO joystick session"
                )
            if self._state != PICO_JOYSTICK_ACTIVE:
                return 409, self._failure(
                    "SESSION_CONFLICT", "PICO joystick is not active"
                )
            if task_id != self._active_task_id:
                return 409, self._failure(
                    "TASK_MISMATCH",
                    "late or unrelated STOP cannot end the active session",
                )
            self._state = PICO_JOYSTICK_STOPPING
            self._allowed_vr_ip = None
            self._disconnect_reason = reason or "requested"

            g12_active = self._g12_active_locked()

        if g12_active:
            return self._interrupt_pico_joystick_for_g12(task_id)

        errors = []
        try:
            self._stop_pico_joystick_motion()
        except Exception as exc:
            errors.append("neutral/stance: %s" % exc)

        with self._lock:
            self._last_task_id = task_id
            self._active_task_id = None
            self._active_vr_ip = None
            self._started_at = None
            self._last_activity_at = utc_now()
            self._last_joystick_at = None
            self._last_joystick_packet_time = 0.0
            self._joystick_zeroed = True
            self._last_error = "; ".join(errors)
            self._state = IDLE_AMP if not errors else ERROR

        if errors:
            return 500, self._failure(
                "PICO_JOYSTICK_STOP_FAILED", "; ".join(errors)
            )
        rospy.loginfo("PICO joystick stopped: task=%s", task_id)
        return 200, self._success("PICO joystick stopped")

    def _start(self, task_id, vr_ip):
        with self._lock:
            if (
                self._state == TELEOPERATING
                and task_id == self._active_task_id
                and vr_ip == self._active_vr_ip
            ):
                return 200, self._success("already started")
            handoff_from_joystick = self._state == PICO_JOYSTICK_ACTIVE
            if handoff_from_joystick and (
                task_id != self._active_task_id
                or vr_ip != self._active_vr_ip
            ):
                return 409, self._failure(
                    "SESSION_MISMATCH",
                    "PICO joystick and teleoperation must use the same task/IP",
                )
            if not handoff_from_joystick and self._state != IDLE_AMP:
                return 409, self._failure(
                    "SESSION_CONFLICT",
                    "another session or transition is active",
                )
            if self._g12_blocks_transition_locked():
                return 409, self._failure(
                    "G12_ACTIVE",
                    "G12 input is active or has not been neutral long enough",
                )
            self._state = (
                PICO_JOYSTICK_STOPPING
                if handoff_from_joystick
                else STARTING
            )
            if not handoff_from_joystick:
                self._active_task_id = task_id
                self._active_vr_ip = vr_ip
                self._started_at = utc_now()
            self._allowed_vr_ip = None
            self._last_activity_at = utc_now()
            self._last_packet_time = time.monotonic()
            self._last_joystick_packet_time = 0.0
            self._joystick_zeroed = True
            self._disconnect_reason = None
            self._last_error = ""
            self._recorder_started = False

        try:
            self._pause_pico_input(True)
            # Revoke walking authority and normalize every AMP-family entry,
            # including direct intervention while patrol left AMP_WILD active.
            self._require_g12_neutral()
            self._normalize_amp_for_vmp()
            self._set_state(STARTING)
            if self.require_upper_precheck:
                self._check_upper_preconditions()
            self._require_g12_neutral()
            if self._get_active_controller() != "amp_controller":
                raise RuntimeError("robot is not in active AMP controller")

            self._set_state(SWITCHING_TO_VMP)
            self._switch_to_vmp()
            self._wait_controller("vmp_controller", True)

            self._set_state(STARTING_RECORDERS)
            self._recorder_start(task_id)
            self._recorder_started = True

            # State and whitelist must both match before UDP is accepted.
            with self._lock:
                self._allowed_vr_ip = vr_ip
                self._state = TELEOPERATING
                self._last_activity_at = utc_now()
                self._last_packet_time = time.monotonic()
            self._pause_pico_input(False)
            self.teleop_state_pub.publish(Bool(True))
            rospy.loginfo("Teleoperation active: task=%s vr=%s", task_id, vr_ip)
            return 200, self._success("started")
        except Exception as exc:
            rospy.logerr("Teleoperation START failed: %s", exc)
            rollback_errors = self._rollback_start(task_id)
            message = str(exc)
            if rollback_errors:
                message += "; rollback: " + "; ".join(rollback_errors)
            return 500, self._failure("START_FAILED", message)

    def _rollback_start(self, task_id):
        errors = []
        with self._lock:
            self._allowed_vr_ip = None
        self.teleop_state_pub.publish(Bool(False))
        try:
            self._pause_pico_input(True)
        except Exception as exc:
            errors.append("input gate: %s" % exc)
        if self._recorder_started:
            try:
                self._recorder_stop(task_id)
            except Exception as exc:
                errors.append("recorder: %s" % exc)
        try:
            if self._controller_active("vmp_controller"):
                self._switch_to_amp()
            self._wait_controller("amp_controller", True)
            self._publish_neutral_joy(
                self.pico_joystick_stop_neutral_duration
            )
            try:
                head_pub = rospy.Publisher(
                    "/robot_head_motion_data", robotHeadMotionData,
                    queue_size=1, latch=True)
                head_pub.publish(joint_data=[0.0, 0.0])
                rospy.loginfo("[VPN PICO] 头部已回正")
                # 手指回零位
                hand_pub = rospy.Publisher(
                    "/control_robot_hand_position", robotHandPosition,
                    queue_size=1, latch=True)
                hand_pub.publish(left_hand_position=[0]*6, right_hand_position=[0]*6)
                rospy.loginfo("[VPN PICO] 手指已回零位")
            except Exception as e:
                rospy.logwarn("[VPN PICO] 头部/手指回位失败: %s" % e)
        except Exception as exc:
            errors.append("AMP restore: %s" % exc)

        with self._lock:
            failed_task = self._active_task_id
            self._last_task_id = failed_task
            self._active_task_id = None
            self._active_vr_ip = None
            self._started_at = None
            self._last_activity_at = utc_now()
            self._last_joystick_at = None
            self._last_joystick_packet_time = 0.0
            self._joystick_zeroed = True
            self._disconnect_reason = "start_failed"
            self._recorder_started = False
            self._last_error = "; ".join(errors)
            self._state = IDLE_AMP if not errors else ERROR
        if errors:
            rospy.logerr("Teleoperation rollback errors: %s", "; ".join(errors))
        return errors

    def _stop(self, task_id, reason):
        with self._lock:
            if self._state == PICO_JOYSTICK_ACTIVE:
                # The same generic STOP remains a safe fallback for clients
                # that do not call the dedicated PICO joystick endpoint.
                pass_to_pico_joystick_stop = True
            else:
                pass_to_pico_joystick_stop = False
        if pass_to_pico_joystick_stop:
            return self._stop_pico_joystick(task_id, reason)

        with self._lock:
            if self._state == IDLE_AMP:
                if task_id == self._last_task_id:
                    return 200, self._success("already stopped")
                return 409, self._failure(
                    "NO_MATCHING_SESSION", "no active session for task"
                )
            if task_id != self._active_task_id:
                return 409, self._failure(
                    "TASK_MISMATCH",
                    "late or unrelated STOP cannot end the active session",
                )
            self._state = STOPPING_INPUT
            self._allowed_vr_ip = None
            self._disconnect_reason = reason or "requested"

        errors = []
        self.teleop_state_pub.publish(Bool(False))
        try:
            self._pause_pico_input(True)
        except Exception as exc:
            errors.append("input gate: %s" % exc)

        self._set_state(STOPPING_RECORDERS)
        if self._recorder_started:
            try:
                self._recorder_stop(task_id)
            except Exception as exc:
                errors.append("recorder: %s" % exc)

        self._set_state(SWITCHING_TO_AMP)
        try:
            self._switch_to_amp()
            self._wait_controller("amp_controller", True)
            self._publish_neutral_joy(
                self.pico_joystick_stop_neutral_duration
            )
            try:
                head_pub = rospy.Publisher(
                    "/robot_head_motion_data", robotHeadMotionData,
                    queue_size=1, latch=True)
                head_pub.publish(joint_data=[0.0, 0.0])
                rospy.loginfo("[VPN PICO] 头部已回正")
                # 手指回零位
                hand_pub = rospy.Publisher(
                    "/control_robot_hand_position", robotHandPosition,
                    queue_size=1, latch=True)
                hand_pub.publish(left_hand_position=[0]*6, right_hand_position=[0]*6)
                rospy.loginfo("[VPN PICO] 手指已回零位")
            except Exception as e:
                rospy.logwarn("[VPN PICO] 头部/手指回位失败: %s" % e)
        except Exception as exc:
            errors.append("AMP restore: %s" % exc)

        self._set_state(FINALIZING)
        with self._lock:
            self._last_task_id = task_id
            self._active_task_id = None
            self._active_vr_ip = None
            self._started_at = None
            self._last_activity_at = utc_now()
            self._last_joystick_at = None
            self._last_joystick_packet_time = 0.0
            self._joystick_zeroed = True
            self._recorder_started = False
            self._last_error = "; ".join(errors)
            self._state = IDLE_AMP if not errors else ERROR

        if errors:
            return 500, self._failure("STOP_FAILED", "; ".join(errors))
        return 200, self._success("stopped")

    def _check_upper_preconditions(self):
        if self.dry_run:
            return
        response = requests.get(self.upper_precheck_url, timeout=self.http_timeout)
        body = response.json()
        if response.status_code != 200 or body.get("passed") is not True:
            raise RuntimeError(
                "upper precheck failed: %s"
                % body.get("reason", "HTTP %s" % response.status_code)
            )

    def _recorder_start(self, task_id):
        if self.dry_run:
            return
        response = requests.post(
            self.recorder_base_url + "/api/teleop/recorder/start",
            json={"teleopTaskId": task_id},
            timeout=max(self.http_timeout, 40.0),
        )
        body = response.json()
        if response.status_code != 200 or body.get("recording") is not True:
            raise RuntimeError(
                "recorder start failed: %s"
                % body.get("message", "HTTP %s" % response.status_code)
            )

    def _recorder_stop(self, task_id):
        if self.dry_run:
            return
        response = requests.post(
            self.recorder_base_url + "/api/teleop/recorder/stop",
            json={"teleopTaskId": task_id},
            timeout=max(self.http_timeout, 70.0),
        )
        body = response.json()
        if response.status_code != 200 or body.get("ok") is not True:
            raise RuntimeError(
                "recorder stop failed: %s"
                % body.get("message", "HTTP %s" % response.status_code)
            )

    def _pause_pico_input(self, paused):
        if self.dry_run:
            return
        service_name = "/vmp/pico_stream_control"
        rospy.wait_for_service(service_name, timeout=5.0)
        proxy = rospy.ServiceProxy(service_name, SetBool)
        response = proxy(SetBoolRequest(data=paused))
        if not response.success:
            raise RuntimeError(
                "%s rejected data=%s: %s"
                % (service_name, paused, response.message)
            )

    @staticmethod
    def _make_authorized_joy(left_x=0.0, left_y=0.0, right_x=0.0, right_y=0.0):
        joy = JoySticks()
        joy.left_x = float(left_x)
        joy.left_y = float(left_y)
        joy.right_x = float(right_x)
        joy.right_y = float(right_y)
        return joy

    def _publish_neutral_joy(self, duration=0.0):
        with self._lock:
            self._joystick_zeroed = True
            self._last_joystick_command = {
                "left_x": 0.0,
                "left_y": 0.0,
                "right_x": 0.0,
                "right_y": 0.0,
            }
        if self.dry_run:
            return
        duration = max(0.0, float(duration))
        deadline = time.monotonic() + duration
        neutral = self._make_authorized_joy()
        while True:
            self.authorized_joy_pub.publish(neutral)
            if rospy.is_shutdown() or time.monotonic() >= deadline:
                break
            time.sleep(0.05)

    def _interrupt_pico_joystick_for_g12(self, task_id):
        with self._lock:
            if task_id != self._active_task_id:
                return 409, self._failure(
                    "TASK_MISMATCH", "G12 interruption no longer matches the active task"
                )
            self._state = PICO_JOYSTICK_STOPPING
            self._allowed_vr_ip = None
            self._disconnect_reason = "g12_override"

        # G12 owns the converter output now. Revoke the remote lease without
        # issuing stance commands that would fight the local operator.
        self._publish_neutral_joy()
        with self._lock:
            self._last_task_id = task_id
            self._active_task_id = None
            self._active_vr_ip = None
            self._started_at = None
            self._last_activity_at = utc_now()
            self._last_joystick_at = None
            self._last_joystick_packet_time = 0.0
            self._joystick_zeroed = True
            self._last_error = ""
            self._state = IDLE_AMP
        rospy.logwarn("PICO joystick session interrupted by G12: task=%s", task_id)
        return 200, self._success("PICO joystick interrupted by G12")

    def _require_g12_neutral(self):
        with self._lock:
            if self._g12_blocks_transition_locked():
                raise RuntimeError(
                    "G12 input is active or has not been neutral long enough"
                )

    def _bridge_graph_error(self):
        with xmlrpc.client.ServerProxy(
            rosgraph.get_master_uri(),
            transport=_TimeoutTransport(self.bridge_master_timeout),
            allow_none=True,
        ) as master:
            code, message, state = master.getSystemState(rospy.get_name())
        if code != 1:
            return "ROS Master getSystemState failed: %s" % message

        publications, subscriptions, _services = state

        def nodes_for(entries, topic):
            resolved = rospy.resolve_name(topic)
            for registered_topic, nodes in entries:
                if registered_topic == resolved:
                    return set(nodes)
            return set()

        authorized_subscribers = nodes_for(
            subscriptions, self.authorized_joy_topic
        )
        converted_publishers = nodes_for(publications, self.converted_joy_topic)
        converted_subscribers = nodes_for(
            subscriptions, self.converted_joy_topic
        )
        if self.pico_converter_node not in authorized_subscribers:
            return "%s is not subscribed to %s" % (
                self.pico_converter_node,
                self.authorized_joy_topic,
            )
        if self.pico_converter_node not in converted_publishers:
            return "%s is not publishing %s" % (
                self.pico_converter_node,
                self.converted_joy_topic,
            )
        if self.auto_gait_node not in converted_subscribers:
            return "%s is not subscribed to %s" % (
                self.auto_gait_node,
                self.converted_joy_topic,
            )
        return ""

    def _bridge_runtime_error(self):
        if self.dry_run:
            return ""
        if self.authorized_joy_pub.get_num_connections() == 0:
            return "authorized Joy publisher has no converter subscriber"
        with self._lock:
            converted_time = self._last_converted_joy_time
        if converted_time <= 0.0:
            return "no converted Joy output observed"
        age = time.monotonic() - converted_time
        if age > self.bridge_output_timeout:
            return "converted Joy output is stale (%.3fs)" % age
        now = time.monotonic()
        with self._lock:
            graph_check_due = (
                now - self._last_bridge_graph_check_time
                >= self.bridge_graph_check_interval
            )
            if graph_check_due:
                self._last_bridge_graph_check_time = now
        if graph_check_due:
            try:
                return self._bridge_graph_error()
            except Exception as exc:
                return "ROS graph probe failed: %s" % exc
        return ""

    def _set_bridge_status(self, error):
        with self._lock:
            self._bridge_ready = not error
            self._bridge_error = error

    def _wait_bridge_ready(self):
        if self.dry_run:
            self._set_bridge_status("")
            return
        deadline = time.monotonic() + self.bridge_start_timeout
        last_error = "bridge readiness not checked"
        while time.monotonic() < deadline and not rospy.is_shutdown():
            try:
                last_error = self._bridge_graph_error()
            except Exception as exc:
                last_error = "ROS graph probe failed: %s" % exc
            if not last_error:
                with self._lock:
                    self._last_bridge_graph_check_time = time.monotonic()
                last_error = self._bridge_runtime_error()
            self._set_bridge_status(last_error)
            if not last_error:
                return
            time.sleep(0.1)
        raise RuntimeError(last_error)

    def _set_gait(self, gait_name):
        if gait_name not in ("walk", "stance"):
            raise ValueError("unsupported gait: %s" % gait_name)
        if self.dry_run:
            return

        if rospy.is_shutdown():
            # Best effort during node shutdown; services may already be
            # unavailable, but one stance request is still safer than none.
            self.gait_name_pub.publish(String(data=gait_name))
            return

        deadline = time.monotonic() + self.pico_joystick_gait_timeout
        while (
            self.gait_name_pub.get_num_connections() == 0
            and time.monotonic() < deadline
            and not rospy.is_shutdown()
        ):
            time.sleep(0.05)
        if self.gait_name_pub.get_num_connections() == 0:
            raise RuntimeError(
                "gait topic has no subscriber: "
                "/humanoid_mpc_gait_name_request"
            )

        request_started = time.monotonic()
        expected_walking = gait_name == "walk"
        last_publish = 0.0
        last_walking = None
        while time.monotonic() < deadline and not rospy.is_shutdown():
            now = time.monotonic()
            if now - last_publish >= 0.5:
                self.gait_name_pub.publish(String(data=gait_name))
                last_publish = now
            with self._lock:
                last_walking = self._rl_walking
                state_time = self._last_rl_command_state_time
            if (
                state_time >= request_started
                and last_walking is expected_walking
            ):
                return
            time.sleep(0.1)
        raise RuntimeError(
            "RL gait did not become %s (walking=%s, state_fresh=%s)"
            % (
                gait_name,
                last_walking,
                self._last_rl_command_state_time >= request_started,
            )
        )

    def _controller_active(self, controller_name):
        if self.dry_run:
            return controller_name == "amp_controller" or self._state in (
                SWITCHING_TO_VMP,
                STARTING_RECORDERS,
                TELEOPERATING,
                STOPPING_INPUT,
                STOPPING_RECORDERS,
            )
        service_name = "/humanoid_controllers/%s/isActive" % controller_name
        rospy.wait_for_service(service_name, timeout=5.0)
        proxy = rospy.ServiceProxy(service_name, Trigger)
        return bool(proxy(TriggerRequest()).success)

    def _get_active_controller(self):
        if self.dry_run:
            with self._lock:
                return self._active_controller_name or "amp_controller"
        service_name = "/humanoid_controller/get_controller_list"
        rospy.wait_for_service(service_name, timeout=5.0)
        proxy = rospy.ServiceProxy(service_name, getControllerList)
        response = proxy(proxy.request_class())
        if not response.success:
            raise RuntimeError(
                "controller list query rejected: %s" % response.message
            )
        controller_name = str(response.current_controller).strip()
        if not controller_name:
            raise RuntimeError("controller list returned an empty active controller")
        with self._lock:
            self._active_controller_name = controller_name
            if controller_name == "amp_controller":
                self._rl_walking = False
        return controller_name

    def _require_amp_family_active(self):
        controller_name = self._get_active_controller()
        if controller_name not in AMP_FAMILY_CONTROLLERS:
            raise RuntimeError(
                "active controller %s is not in the allowed AMP family %s"
                % (controller_name, sorted(AMP_FAMILY_CONTROLLERS))
            )
        return controller_name

    def _wait_controller(self, controller_name, expected):
        if self.dry_run:
            return
        deadline = time.time() + self.controller_timeout
        while time.time() < deadline and not rospy.is_shutdown():
            if self._controller_active(controller_name) is expected:
                if expected:
                    with self._lock:
                        self._active_controller_name = controller_name
                return
            time.sleep(0.2)
        raise RuntimeError(
            "%s active did not become %s" % (controller_name, expected)
        )

    def _switch_to_vmp(self):
        if self.dry_run:
            return
        service_name = "/humanoid_controller/switch_to_vmp_controller"
        with self._lock:
            self._last_switch_event = None
        rospy.wait_for_service(service_name, timeout=5.0)
        response = rospy.ServiceProxy(service_name, Trigger)(TriggerRequest())
        if not response.success:
            raise RuntimeError("VMP switch rejected: %s" % response.message)
        self._wait_switch_event("vmp_controller")

    def _switch_to_rl_controller(self, controller_name):
        if self.dry_run:
            return
        with self._lock:
            self._last_switch_event = None
        deadline = time.time() + 5.0
        while (
            self.amp_switch_pub.get_num_connections() == 0
            and time.time() < deadline
            and not rospy.is_shutdown()
        ):
            time.sleep(0.05)
        if self.amp_switch_pub.get_num_connections() == 0:
            raise RuntimeError(
                "AMP switch topic has no subscriber: "
                "/humanoid_controller/nav_switch_rl_controller_by_name"
            )
        self.amp_switch_pub.publish(String(data=controller_name))
        self._wait_switch_event(controller_name)

    def _switch_to_amp(self):
        self._switch_to_rl_controller("amp_controller")

    def _stop_pico_joystick_motion(self):
        """Revoke Pico motion and reach stance without changing AMP family member."""
        self._publish_neutral_joy(
            self.pico_joystick_stop_neutral_duration
        )
        controller_name = self._require_amp_family_active()
        # AMP is the manipulation/stance member. AMP_WILD owns the walking
        # receiver and must explicitly acknowledge a fresh stance request.
        if controller_name == "amp_wild_controller":
            self._set_gait("stance")
        return controller_name

    def _normalize_amp_for_vmp(self):
        """Reach stance, then normalize AMP_WILD to AMP before VMP entry."""
        controller_name = self._stop_pico_joystick_motion()
        self._require_g12_neutral()
        if controller_name == "amp_wild_controller":
            self._switch_to_amp()
            self._wait_controller("amp_controller", True)
        elif controller_name != "amp_controller":
            raise RuntimeError(
                "cannot normalize unexpected controller %s" % controller_name
            )
        if self._get_active_controller() != "amp_controller":
            raise RuntimeError("AMP normalization did not reach amp_controller")

    def _wait_switch_event(self, target_controller):
        if self.dry_run or not self.require_switch_event:
            return
        deadline = time.time() + self.controller_timeout
        last_controller_query = 0.0
        while time.time() < deadline and not rospy.is_shutdown():
            with self._lock:
                event = self._last_switch_event
            if event and str(event["to"]).strip() == target_controller:
                return
            now = time.monotonic()
            if now - last_controller_query >= 0.2:
                last_controller_query = now
                try:
                    if self._get_active_controller() == target_controller:
                        rospy.logwarn(
                            "Controller %s became current before its switch event was observed",
                            target_controller,
                        )
                        return
                except Exception:
                    pass
            time.sleep(0.05)
        raise RuntimeError(
            "controller switch event to %s was not observed" % target_controller
        )

    def _success(self, message):
        result = self.status()
        result.update({"code": 0, "message": message})
        return result

    def _failure(self, error_code, message):
        result = self.status()
        result.update(
            {
                "code": 1,
                "errorCode": error_code,
                "message": message,
            }
        )
        return result

    def _start_http_server(self):
        manager = self

        class Handler(BaseHTTPRequestHandler):
            def _respond(self, code, body):
                payload = json.dumps(body, ensure_ascii=False).encode("utf-8")
                self.send_response(code)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(payload)))
                self.end_headers()
                self.wfile.write(payload)

            def _body(self):
                length = int(self.headers.get("Content-Length", "0"))
                raw = self.rfile.read(length)
                try:
                    return json.loads(raw.decode("utf-8")) if raw else {}
                except Exception as exc:
                    raise ValueError("invalid JSON: %s" % exc)

            @staticmethod
            def _task_id(body):
                task_id = body.get("teleopTaskId")
                if not isinstance(task_id, str) or not task_id.strip():
                    raise ValueError("teleopTaskId must be a non-empty string")
                return task_id

            @staticmethod
            def _vr_ip(body):
                vr_ip = body.get("vr_device_ip")
                try:
                    parsed_ip = ipaddress.ip_address(vr_ip)
                except ValueError:
                    raise ValueError("vr_device_ip must be a valid IP")
                if parsed_ip not in manager.allowed_vr_network:
                    raise ValueError(
                        "vr_device_ip is outside %s"
                        % manager.allowed_vr_network
                    )
                return str(parsed_ip)

            def do_GET(self):
                if self.path.split("?", 1)[0] == "/api/teleop/status":
                    self._respond(200, manager.status())
                else:
                    self._respond(404, {"code": 404, "message": "not found"})

            def do_POST(self):
                path = self.path.split("?", 1)[0]
                try:
                    body = self._body()
                    task_id = self._task_id(body)
                    if path == "/api/teleop/start":
                        vr_ip = self._vr_ip(body)
                        code, result = manager.submit(
                            "start",
                            task_id,
                            vr_ip=vr_ip,
                            timeout=90.0,
                        )
                    elif path == "/api/teleop/pico_joystick/start":
                        vr_ip = self._vr_ip(body)
                        code, result = manager.submit(
                            "pico_joystick_start",
                            task_id,
                            vr_ip=vr_ip,
                            timeout=30.0,
                        )
                    elif path == "/api/teleop/pico_joystick/stop":
                        reason = body.get("reason", "requested")
                        if not isinstance(reason, str):
                            raise ValueError("reason must be a string")
                        code, result = manager.submit(
                            "pico_joystick_stop",
                            task_id,
                            reason=reason,
                            timeout=30.0,
                        )
                    elif path == "/api/teleop/stop":
                        reason = body.get("reason", "requested")
                        if not isinstance(reason, str):
                            raise ValueError("reason must be a string")
                        code, result = manager.submit(
                            "stop",
                            task_id,
                            reason=reason,
                            timeout=120.0,
                        )
                    else:
                        code, result = 404, {
                            "code": 404,
                            "message": "not found",
                        }
                except ValueError as exc:
                    code, result = 400, {
                        "code": 400,
                        "errorCode": "INVALID_ARGUMENT",
                        "message": str(exc),
                    }
                self._respond(code, result)

            def log_message(self, fmt, *args):
                rospy.logdebug("Teleop HTTP: " + fmt, *args)

        self._http_server = ThreadingHTTPServer(
            (self.http_host, self.http_port), Handler
        )
        threading.Thread(
            target=self._http_server.serve_forever,
            daemon=True,
            name="teleop-http",
        ).start()
        rospy.loginfo(
            "Teleop session API listening on %s:%s",
            self.http_host,
            self.http_port,
        )

    def shutdown(self):
        if self._shutdown.is_set():
            return
        self._shutdown.set()
        if self._http_server:
            self._http_server.shutdown()
        with self._lock:
            task_id = self._active_task_id
        if task_id:
            self._stop(task_id, "node_shutdown")
        else:
            with self._lock:
                self._allowed_vr_ip = None
            if not self.dry_run:
                try:
                    self._pause_pico_input(True)
                except Exception:
                    pass


class PicoReceiverVPN(MinimalPicoReceiver):
    def __init__(self, session_manager, host, port, publish_tf, **diagnostic_kwargs):
        super().__init__(
            host=host,
            port=port,
            publish_tf=publish_tf,
            enable_ip_broadcast=False,
            **diagnostic_kwargs,
        )
        self.session_manager = session_manager
        self.joystick_queue = queue.Queue(maxsize=1)

    @staticmethod
    def _controller_axes(controller_data):
        if not controller_data.HasField("controllers"):
            return None
        states = controller_data.controllers

        def axes(state):
            if state is None:
                return 0.0, 0.0
            thumbstick = state.thumbstick
            x_value = float(thumbstick[0]) if len(thumbstick) > 0 else 0.0
            y_value = float(thumbstick[1]) if len(thumbstick) > 1 else 0.0
            return x_value, y_value

        left_state = states.left if states.HasField("left") else None
        right_state = states.right if states.HasField("right") else None
        left_x, left_y = axes(left_state)
        right_x, right_y = axes(right_state)
        return left_x, left_y, right_x, right_y

    def _process_pico_joystick_thread(self):
        rospy.loginfo("PICO joystick-only processing thread started")
        while self.running and not rospy.is_shutdown():
            try:
                datagram = self.joystick_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            payload_fields = []
            handled_controller = False
            try:
                if isinstance(datagram, ReceivedDatagram):
                    data = datagram.payload
                    if datagram.process_start_monotonic_ns <= 0:
                        datagram.process_start_monotonic_ns = monotonic_ns()
                else:
                    data = datagram

                message = proto.VRData()
                message.ParseFromString(data)
                if isinstance(datagram, ReceivedDatagram):
                    datagram.protobuf_parse_done_monotonic_ns = monotonic_ns()
                payload_fields = self._message_payload_fields(message)
                self.diagnostics.record_vrdata_fields(
                    datagram, payload_fields, parse_ok=True
                )

                if message.HasField("controller"):
                    values = self._controller_axes(message.controller)
                    if values is not None:
                        source_ip = datagram.addr[0]
                        self.session_manager.handle_pico_joystick(
                            source_ip, *values
                        )
                        handled_controller = True

                self.diagnostics.record_processing_timing(
                    datagram,
                    payload_fields,
                    process_done_monotonic_ns=monotonic_ns(),
                    handled_controller=handled_controller,
                    handled_full_body=False,
                    process_ok=True,
                )
            except Exception as exc:
                rospy.logerr_throttle(
                    1.0, "PICO joystick packet parse failed: %s", exc
                )
                if isinstance(datagram, ReceivedDatagram):
                    self.diagnostics.record_parse_failure(datagram, str(exc))
                    self.diagnostics.record_processing_timing(
                        datagram,
                        payload_fields,
                        process_done_monotonic_ns=monotonic_ns(),
                        handled_controller=handled_controller,
                        handled_full_body=False,
                        process_ok=False,
                        error=str(exc),
                    )

    def _enqueue_latest(self, target_queue, datagram):
        try:
            target_queue.put_nowait(datagram)
        except queue.Full:
            try:
                dropped = target_queue.get_nowait()
                self.diagnostics.record_queue_drop(dropped)
                target_queue.put_nowait(datagram)
            except queue.Empty:
                pass

    def start(self):
        self.running = True
        self.diagnostics.start()
        self.process_thread = threading.Thread(
            target=self._process_data_thread, daemon=True
        )
        self.process_thread.start()
        self.joystick_thread = threading.Thread(
            target=self._process_pico_joystick_thread,
            daemon=True,
            name="pico-joystick-processor",
        )
        self.joystick_thread.start()
        rospy.loginfo("Gated PICO UDP listening on %s:%s", self.host, self.port)
        while self.running and not rospy.is_shutdown():
            try:
                data, address = self.socket.recvfrom(65535)
                recv_ns = monotonic_ns()
            except socket.timeout:
                continue
            except OSError:
                break
            except Exception as exc:
                rospy.logerr("PICO UDP receive error: %s", exc)
                continue

            input_mode = self.session_manager.input_mode(address[0])
            if input_mode is None:
                self.diagnostics.record_unauthorized_drop(address, len(data))
                rospy.logwarn_throttle(
                    5.0, "Dropped unauthorized PICO UDP from %s", address[0]
                )
                continue
            datagram = self.diagnostics.decode_datagram(data, address, recv_ns)
            if datagram is None:
                continue
            datagram.enqueue_monotonic_ns = monotonic_ns()
            if input_mode == PICO_JOYSTICK_ACTIVE:
                self._enqueue_latest(self.joystick_queue, datagram)
            elif input_mode == TELEOPERATING:
                self._enqueue_latest(self.data_queue, datagram)


def main():
    rospy.init_node("pico_receiver_vpn", anonymous=False)
    manager = SessionManager()
    receiver = PicoReceiverVPN(
        manager,
        host=rospy.get_param("~host", "0.0.0.0"),
        port=int(rospy.get_param("~udp_port", 12345)),
        publish_tf=bool(rospy.get_param("~publish_tf", True)),
        enable_diagnostics=bool(rospy.get_param("~enable_diagnostics", True)),
        diagnostic_log_enable=bool(rospy.get_param("~diagnostic_log_enable", False)),
        diagnostic_log_dir=rospy.get_param("~diagnostic_log_dir", "~/.ros/pico_diagnostics"),
        diagnostic_log_max_file_mb=float(rospy.get_param("~diagnostic_log_max_file_mb", 300.0)),
        diagnostic_log_compress=bool(rospy.get_param("~diagnostic_log_compress", True)),
        diagnostic_udp_reply=bool(rospy.get_param("~diagnostic_udp_reply", True)),
        enable_time_sync=bool(rospy.get_param("~enable_time_sync", True)),
        diagnostic_publish_hz=float(rospy.get_param("~diagnostic_publish_hz", 1.0)),
        diagnostic_allow_legacy=bool(rospy.get_param("~diagnostic_allow_legacy", True)),
        diagnostic_ping_enable=bool(rospy.get_param("~diagnostic_ping_enable", True)),
        diagnostic_ping_lower_ip=rospy.get_param("~diagnostic_ping_lower_ip", ""),
        diagnostic_ping_interval_sec=float(rospy.get_param("~diagnostic_ping_interval_sec", 5.0)),
        diagnostic_ping_timeout_sec=float(rospy.get_param("~diagnostic_ping_timeout_sec", 1.0)),
        diagnostic_sync_max_valid_rtt_ms=float(rospy.get_param("~diagnostic_sync_max_valid_rtt_ms", 200.0)),
        diagnostic_sync_max_valid_residual_ms=float(rospy.get_param("~diagnostic_sync_max_valid_residual_ms", 10.0)),
    )
    rospy.on_shutdown(manager.shutdown)
    try:
        receiver.start()
    finally:
        receiver.stop()
        manager.shutdown()


if __name__ == "__main__":
    main()
