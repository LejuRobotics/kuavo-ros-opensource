#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Authoritative teleoperation session manager and gated PICO UDP receiver."""

import ipaddress
import re
import json
import os
import queue
import socket
import sys
import threading
import time
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
import rospy
from kuavo_msgs.msg import ControllerSwitchEvent, robotHeadMotionData
from kuavo_ros_interfaces.msg import robotHandPosition
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, SetBoolRequest, Trigger, TriggerRequest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pico_comm_minimal import MinimalPicoReceiver
from pico_diagnostic_runtime import monotonic_ns


IDLE_AMP = "IDLE_AMP"
STARTING = "STARTING"
SWITCHING_TO_VMP = "SWITCHING_TO_VMP"
STARTING_RECORDERS = "STARTING_RECORDERS"
TELEOPERATING = "TELEOPERATING"
STOPPING_INPUT = "STOPPING_INPUT"
STOPPING_RECORDERS = "STOPPING_RECORDERS"
SWITCHING_TO_AMP = "SWITCHING_TO_AMP"
FINALIZING = "FINALIZING"
ERROR = "ERROR"


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

        self._lock = threading.RLock()
        self._state = IDLE_AMP
        self._active_task_id = None
        self._active_vr_ip = None
        self._allowed_vr_ip = None
        self._started_at = None
        self._last_activity_at = None
        self._last_packet_time = 0.0
        self._disconnect_reason = None
        self._last_task_id = None
        self._last_error = ""
        self._recorder_started = False
        self._last_switch_event = None

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
        self.teleop_state_pub.publish(Bool(False))
        rospy.Subscriber(
            "/humanoid_controller/controller_switch_event",
            ControllerSwitchEvent,
            self._on_switch_event,
            queue_size=10,
        )

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
            self._last_switch_event = {
                "from": message.from_controller,
                "to": message.to_controller,
                "at": utc_now(),
            }

    def _set_state(self, state, error=""):
        with self._lock:
            self._state = state
            if error:
                self._last_error = error

    def status(self):
        with self._lock:
            return {
                "code": 0,
                "state": self._state,
                "teleop_active": self._state == TELEOPERATING,
                "authorized": (
                    self._state == TELEOPERATING
                    and self._allowed_vr_ip == self._active_vr_ip
                ),
                "activeTaskId": self._active_task_id,
                "vr_device_ip": self._active_vr_ip,
                "startedAt": self._started_at,
                "lastActivityAt": self._last_activity_at,
                "disconnectReason": self._disconnect_reason,
                "lastTaskId": self._last_task_id,
                "lastError": self._last_error,
                "lastSwitchEvent": self._last_switch_event,
                "safety": {
                    "picoHeartbeat": "reserved",
                    "hardwareEstop": "reserved",
                    "fallDetection": "reserved",
                    "standingCheck": "reserved",
                },
            }

    def input_allowed(self, source_ip):
        with self._lock:
            allowed = (
                self._state == TELEOPERATING
                and self._allowed_vr_ip is not None
                and source_ip == self._allowed_vr_ip
            )
            if allowed:
                self._last_activity_at = utc_now()
                self._last_packet_time = time.monotonic()
            return allowed

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
                command = self._commands.get(timeout=0.5)
            except queue.Empty:
                self._check_idle_timeout()
                continue
            try:
                if command.action == "start":
                    result = self._start(command.task_id, command.vr_ip)
                elif command.action == "stop":
                    result = self._stop(command.task_id, command.reason)
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

    def _check_idle_timeout(self):
        with self._lock:
            if self._state != TELEOPERATING:
                return
            task_id = self._active_task_id
            idle = time.monotonic() - self._last_packet_time
            if idle <= self.teleop_idle_timeout:
                return
        rospy.logwarn(
            "Teleoperation idle timeout after %.1fs (limit %.1fs), auto-stopping",
            idle, self.teleop_idle_timeout,
        )
        self._stop(task_id, "vr_idle_timeout")

    def _start(self, task_id, vr_ip):
        with self._lock:
            if (
                self._state == TELEOPERATING
                and task_id == self._active_task_id
                and vr_ip == self._active_vr_ip
            ):
                return 200, self._success("already started")
            if self._state != IDLE_AMP:
                return 409, self._failure(
                    "SESSION_CONFLICT",
                    "another session or transition is active",
                )
            self._state = STARTING
            self._active_task_id = task_id
            self._active_vr_ip = vr_ip
            self._allowed_vr_ip = None
            self._started_at = utc_now()
            self._last_activity_at = self._started_at
            self._last_packet_time = time.monotonic()
            self._disconnect_reason = None
            self._last_error = ""
            self._recorder_started = False

        try:
            self._pause_pico_input(True)
            if self.require_upper_precheck:
                self._check_upper_preconditions()
            if not self._controller_active("amp_controller"):
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
            self._disconnect_reason = "start_failed"
            self._recorder_started = False
            self._state = IDLE_AMP if not errors else ERROR
        return errors

    def _stop(self, task_id, reason):
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

    def _wait_controller(self, controller_name, expected):
        if self.dry_run:
            return
        deadline = time.time() + self.controller_timeout
        while time.time() < deadline and not rospy.is_shutdown():
            if self._controller_active(controller_name) is expected:
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

    def _switch_to_amp(self):
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
        self.amp_switch_pub.publish(String(data="amp_controller"))
        self._wait_switch_event("amp_controller")

    def _wait_switch_event(self, target_controller):
        if self.dry_run or not self.require_switch_event:
            return
        deadline = time.time() + self.controller_timeout
        while time.time() < deadline and not rospy.is_shutdown():
            with self._lock:
                event = self._last_switch_event
            if event and target_controller in event["to"]:
                return
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
                        code, result = manager.submit(
                            "start",
                            task_id,
                            vr_ip=str(parsed_ip),
                            timeout=90.0,
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

    def start(self):
        self.running = True
        self.diagnostics.start()
        self.process_thread = threading.Thread(
            target=self._process_data_thread, daemon=True
        )
        self.process_thread.start()
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

            if not self.session_manager.input_allowed(address[0]):
                self.diagnostics.record_unauthorized_drop(address, len(data))
                rospy.logwarn_throttle(
                    5.0, "Dropped unauthorized PICO UDP from %s", address[0]
                )
                continue
            datagram = self.diagnostics.decode_datagram(data, address, recv_ns)
            if datagram is None:
                continue
            datagram.enqueue_monotonic_ns = monotonic_ns()
            try:
                self.data_queue.put_nowait(datagram)
            except queue.Full:
                try:
                    dropped = self.data_queue.get_nowait()
                    self.diagnostics.record_queue_drop(dropped)
                    self.data_queue.put_nowait(datagram)
                except queue.Empty:
                    pass


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
        diagnostic_allow_legacy=bool(rospy.get_param("~diagnostic_allow_legacy", False)),
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
