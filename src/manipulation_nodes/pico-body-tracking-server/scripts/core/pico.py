"""
Pico VR device integration core functionality.
"""

import sys
import socket
import queue
import threading
import argparse
import signal
import time
import json
from collections import deque
import rospy
from enum import Enum
import numpy as np
import tf
from core.pico_diagnostic_runtime import (
    JsonlDiagnosticLogger,
    PicoDiagnosticsRuntime,
    ReceivedDatagram,
    monotonic_ns,
)
from core.ros.pico import (
    KuavoPicoNodeManager,
    KuavoPicoNode,
    RobotMatrixPublisher,
    ControlMode
)
from core.ros.pico_utils import RobotInfoBroadcaster
from core.ros.robot_data import RobotDataServer
from core.ros.system_identification import SystemIdentification
from common.logger import SDKLogger
from .ros import body_tracking_extended_pb2 as proto
from .ros import hand_wrench_srv_pb2
from .config.pico_vr_config import HandWrenchConfig

def signal_handler(self):
    """Handle signal for graceful exit."""
    SDKLogger.info('Exiting gracefully...')
    if self.socket:
        self.socket.close()
    sys.exit(0)

def parse_args():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--record_bag', type=bool, default=False, help='Record bag file')
    parser.add_argument('--host', type=str, default='0.0.0.0', help='UDP server host')
    parser.add_argument('--port', type=int, default=12345, help='UDP server port')
    return parser.parse_args()

class PicoError(Exception):
    """Base exception class for Pico related errors."""
    pass

class ConnectionError(PicoError):
    """Exception raised for connection related errors."""
    pass

class DataProcessingError(PicoError):
    """Exception raised for data processing related errors."""
    pass

class ConfigurationError(PicoError):
    """Exception raised for configuration related errors."""
    pass

class ErrorState(Enum):
    """Enum for tracking error states."""
    NORMAL = 0
    CONNECTION_ERROR = 1
    DATA_PROCESSING_ERROR = 2
    CONFIGURATION_ERROR = 3


class LatestDatagramQueue:
    """Small receive queue that coalesces pending datagrams by stream."""

    def __init__(self, maxsize=5, latest_by_stream=True, drop_callback=None):
        self.maxsize = max(1, int(maxsize))
        self.latest_by_stream = bool(latest_by_stream)
        self.drop_callback = drop_callback
        self._fifo = queue.Queue(maxsize=self.maxsize)
        self._lock = threading.Lock()
        self._not_empty = threading.Condition(self._lock)
        self._pending = {}
        self._order = deque()

    @staticmethod
    def _key(datagram):
        header = getattr(datagram, "header", None)
        if header:
            return ("stream", int(header.stream))
        return ("legacy", 0)

    def _record_drop(self, datagram):
        if self.drop_callback:
            self.drop_callback(datagram)

    def put_latest(self, datagram):
        if not self.latest_by_stream:
            try:
                self._fifo.put(datagram, block=False)
            except queue.Full:
                try:
                    dropped = self._fifo.get_nowait()
                    self._record_drop(dropped)
                    self._fifo.put_nowait(datagram)
                except queue.Empty:
                    pass
            return

        key = self._key(datagram)
        with self._not_empty:
            dropped = self._pending.get(key)
            if dropped is not None:
                self._record_drop(dropped)
                self._pending[key] = datagram
                self._not_empty.notify()
                return

            if len(self._order) >= self.maxsize:
                old_key = self._order.popleft()
                dropped = self._pending.pop(old_key, None)
                self._record_drop(dropped)

            self._pending[key] = datagram
            self._order.append(key)
            self._not_empty.notify()

    def get(self, timeout=None):
        if not self.latest_by_stream:
            return self._fifo.get(timeout=timeout)

        end_time = None if timeout is None else time.monotonic() + timeout
        with self._not_empty:
            while not self._order:
                if timeout is None:
                    self._not_empty.wait()
                    continue
                remaining = end_time - time.monotonic()
                if remaining <= 0:
                    raise queue.Empty
                self._not_empty.wait(remaining)

            key = self._order.popleft()
            return self._pending.pop(key)

class KuavoPicoServer:
    """Server class for Pico VR device integration."""
    
    def __init__(self, *args, **kwargs):
        """Initialize the Pico server."""
        self.host = kwargs.get('host', '0.0.0.0')
        self.port = kwargs.get('port', 12345)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((self.host, self.port))
        self.socket.settimeout(1)

        # New protocol streams are processed independently.  The fallback queue
        # keeps legacy or unspecified packets on the original serial path.
        self.data_queue = None
        self.skeleton_queue = None
        self.controller_queue = None
        self.process_threads = []
        self.robot_matrix_publisher = RobotMatrixPublisher()

        self.tf_publish_enable = bool(self._get_pico_param("tf_publish_enable", False))
        self.tf_publish_hz = float(self._get_pico_param("tf_publish_hz", 100.0))
        if self.tf_publish_hz <= 0.0:
            self.tf_publish_hz = 100.0
        self._tf_publish_interval_ns = int(1e9 / self.tf_publish_hz)
        self._last_tf_publish_monotonic_ns = 0
        
        KuavoPicoNodeManager.get_instance('kuavo_pico_node')
        
        # Default configuration
        self.DEFAULT_CONFIG = {
            "head_motion_range": {
                "pitch": [-30, 30],  # Pitch range in degrees
                "yaw": [-60, 60]     # Yaw range in degrees
            }
        }
        self.config = self.DEFAULT_CONFIG.copy()
        
        # Error handling
        self.error_state = ErrorState.NORMAL
        self.error_count = 0
        self.max_retries = 3
        self.retry_delay = 1.0  # seconds
        
        # Initialize PicoNode
        self.pico_node = None

        self.process_thread = None
        
        # Start robot information broadcast thread
        self.broadcaster = RobotInfoBroadcaster()
        self.broadcast_thread = threading.Thread(target=self.broadcaster.broadcast, daemon=True)
        self.broadcast_thread.start()

        # 上传机器人手臂/手指电流, 接收下发控制指令
        self.robot_data_server = RobotDataServer(self.broadcaster.eef_type)
        self.robot_data_server_thread = threading.Thread(target=self.robot_data_server.start, daemon=True)
        self.robot_data_server_thread.start()

        self.system_identification = SystemIdentification()

        diagnostic_log_enable = bool(self._get_diag_param("log_enable", False))
        self.diagnostic_detail_timing_enabled = diagnostic_log_enable
        self.diagnostic_logger = JsonlDiagnosticLogger(
            enabled=diagnostic_log_enable,
            log_dir=str(self._get_diag_param("log_dir", "~/.ros/pico_diagnostics")),
            max_file_size_mb=float(self._get_diag_param("log_max_file_size_mb", 300.0)),
            compress_closed_files=bool(self._get_diag_param("log_compress", True)),
        )
        self.diagnostics = PicoDiagnosticsRuntime(
            sock=self.socket,
            logger=self.diagnostic_logger,
            enabled=bool(self._get_diag_param("enable", True)),
            udp_reply_enabled=bool(self._get_diag_param("udp_reply_enable", True)),
            sync_enabled=bool(self._get_diag_param("enable_time_sync", True)),
            diagnostic_publish_hz=float(self._get_diag_param("publish_hz", 1.0)),
            allow_legacy_vrdata=bool(self._get_diag_param("allow_legacy", True)),
            ping_enabled=diagnostic_log_enable and bool(self._get_diag_param("ping_enable", True)),
            ping_interval_sec=float(self._get_diag_param("ping_interval_sec", 5.0)),
            ping_timeout_sec=float(self._get_diag_param("ping_timeout_sec", 1.0)),
            sync_max_valid_rtt_ms=float(self._get_diag_param("sync_max_valid_rtt_ms", 20.0)),
            sync_max_valid_residual_ms=float(self._get_diag_param("sync_max_valid_residual_ms", 5.0)),
        )
        diagnostic_queue_size = int(self._get_diag_param("queue_size", 5))
        diagnostic_latest_queue_enable = bool(self._get_diag_param("latest_queue_enable", True))
        self.data_queue = LatestDatagramQueue(
            maxsize=diagnostic_queue_size,
            latest_by_stream=diagnostic_latest_queue_enable,
            drop_callback=self.diagnostics.record_queue_drop,
        )
        self.skeleton_queue = LatestDatagramQueue(
            maxsize=diagnostic_queue_size,
            latest_by_stream=diagnostic_latest_queue_enable,
            drop_callback=self.diagnostics.record_queue_drop,
        )
        self.controller_queue = LatestDatagramQueue(
            maxsize=diagnostic_queue_size,
            latest_by_stream=diagnostic_latest_queue_enable,
            drop_callback=self.diagnostics.record_queue_drop,
        )
        SDKLogger.info(
            "PICO receive queues configured: latest_by_stream=%s, size=%s, streams=skeleton/controller/legacy"
            % (diagnostic_latest_queue_enable, diagnostic_queue_size)
        )
        SDKLogger.info(
            "PICO TF publishing configured: enable=%s, hz=%.1f"
            % (self.tf_publish_enable, self.tf_publish_hz)
        )

    @staticmethod
    def _get_diag_param(name, default):
        private_name = "~diagnostic_%s" % name
        global_name = "/pico_diagnostic_%s" % name
        return rospy.get_param(private_name, rospy.get_param(global_name, default))

    @staticmethod
    def _get_pico_param(name, default):
        private_name = "~pico_%s" % name
        global_name = "/pico_%s" % name
        return rospy.get_param(private_name, rospy.get_param(global_name, default))

    def _should_publish_tf(self, now_ns):
        if not self.tf_publish_enable:
            return False
        if (
            self._last_tf_publish_monotonic_ns == 0
            or now_ns - self._last_tf_publish_monotonic_ns >= self._tf_publish_interval_ns
        ):
            self._last_tf_publish_monotonic_ns = now_ns
            return True
        return False
        
    def get_head_motion_range(self):
        """Get head motion range from configuration."""
        return self.config["head_motion_range"]

    def send_initial_message(self):
        """Send initial message to establish connection."""
        message = b'hi'
        max_retries = 200
        for attempt in range(max_retries):
            try:
                self.socket.sendto(message, (self.host, self.port))
                data, addr = self.socket.recvfrom(1024)
                print(f"\033[92mAcknowledgment From PICO received on attempt {attempt + 1}, start to receiving data...\033[0m")
                return True
            except socket.timeout:
                SDKLogger.warning(f"Pico timeout: Attempt {attempt + 1} timed out. Retrying...")
            except KeyboardInterrupt:
                SDKLogger.info("Force quit by Ctrl-c.")
                return False
        SDKLogger.error("Failed to send message after 200 attempts.")
        return False

    def clean_up(self):
        """Cleanup when the object is destroyed."""
        if self.socket:
            self.socket.close()
        try:
            self.diagnostics.stop()
            self.diagnostic_logger.close()
        except Exception:
            pass
        self.broadcaster.quit()    
        SDKLogger.info("Stopped UDP server")

    def handle_error(self, error, error_state):
        """Handle errors and determine if should continue."""
        self.error_state = error_state
        self.error_count += 1
        
        if self.error_count > self.max_retries:
            SDKLogger.error(f"Max retries exceeded, stopping server: {error}")
            return False
        
        SDKLogger.warning(f"Error occurred (attempt {self.error_count}/{self.max_retries}): {error}")
        time.sleep(self.retry_delay)
        return True

    @staticmethod
    def _message_payload_fields(message):
        fields = []
        for field_name in [
            "full_body",
            "upper_body",
            "controller",
            "robot_data",
            "vr_command",
            "delayed_diagnosis_command",
        ]:
            try:
                if message.HasField(field_name):
                    fields.append(field_name)
            except ValueError:
                pass
        return fields

    @staticmethod
    def _duration_ns(end_ns, start_ns):
        return end_ns - start_ns if end_ns > 0 and start_ns > 0 and end_ns >= start_ns else None

    def process_data(self, datagram):
        """Process data."""
        if not isinstance(datagram, ReceivedDatagram):
            datagram = ReceivedDatagram(
                raw_data=datagram,
                payload=datagram,
                addr=("", 0),
                recv_monotonic_ns=monotonic_ns(),
                legacy=True,
            )
        detail_timing = self.diagnostic_detail_timing_enabled
        if detail_timing and datagram.process_start_monotonic_ns <= 0:
            datagram.process_start_monotonic_ns = monotonic_ns()

        payload_fields = []
        handled_controller = False
        handled_full_body = False
        process_ok = True
        error = ""
        process_done_ns = 0

        try:
            data_str = datagram.payload
            message = proto.VRData()
            message.ParseFromString(data_str)
            if detail_timing:
                datagram.protobuf_parse_done_monotonic_ns = monotonic_ns()
            payload_fields = self._message_payload_fields(message)
            self.diagnostics.record_vrdata_fields(datagram, payload_fields, parse_ok=True)
        except Exception as exc:
            process_ok = False
            error = str(exc)
            self.diagnostics.record_parse_failure(datagram, error)
            if detail_timing:
                process_done_ns = monotonic_ns()
                self.diagnostics.record_processing_timing(
                    datagram,
                    payload_fields,
                    process_done_monotonic_ns=process_done_ns,
                    handled_controller=handled_controller,
                    handled_full_body=handled_full_body,
                    process_ok=process_ok,
                    error=error,
                )
            raise

        try:
            # 延迟诊断命令
            if message.HasField('delayed_diagnosis_command'):
                SDKLogger.info("---------------- receive delayed diagnosis command ------------------")
                SDKLogger.info(f"delayed diagnosis command: {message.delayed_diagnosis_command}")
                self.process_delayed_diagnosis_command(message.delayed_diagnosis_command)
                # 延迟诊断中不处理其他数据
                return

            if self.system_identification.is_running():
                # 延迟诊断中不处理数据
                return

            if self.is_play_mode():
                SDKLogger.debug("Skipping all data processing in play mode")
                return
            if message.HasField('full_body'):
                self.process_body_tracking_data(message.full_body, datagram, payload_fields)
                handled_full_body = True

            if message.HasField('upper_body'):
                # TODO 处理半身模式
                pass

            if message.HasField('controller'):
                self.process_pico_joy_data(message.controller)
                handled_controller = True

            if message.HasField('vr_command'):
                self.process_vr_command(message.vr_command)
        except Exception as exc:
            process_ok = False
            error = str(exc)
            raise
        finally:
            if detail_timing:
                process_done_ns = monotonic_ns()
                self.diagnostics.record_processing_timing(
                    datagram,
                    payload_fields,
                    process_done_monotonic_ns=process_done_ns,
                    handled_controller=handled_controller,
                    handled_full_body=handled_full_body,
                    process_ok=process_ok,
                    error=error,
                )
    def is_play_mode(self):
        """Check if the system is in play mode."""
        return hasattr(self.pico_node, '_play_mode') and self.pico_node._play_mode

    def process_delayed_diagnosis_command(self, command):
        """Process delayed diagnosis command."""
        # 停止延迟诊断
        if not command.run:
            SDKLogger.warning("收到停止延迟诊断命令，停止延迟诊断")
            self.system_identification.stop()
            return
        # 启动延迟诊断
        if not self.system_identification.is_running():
            SDKLogger.info("收到启动延迟诊断命令，启动延迟诊断")
            self.system_identification.start()
        else:
            # SDKLogger.warning("延迟诊断正在进行中，不会处理其他数据")
            return
        
    def process_body_tracking_data(self, full_body_data, datagram=None, payload_fields=None):
        """Process data."""
        # 控制遥操解锁/锁定
        if not self.pico_node.toggle_teleop_unlock:
            self.diagnostic_logger.log(
                "pico_body_tracking_skip",
                reason="teleop_locked",
                packet_sequence=datagram.header.packet_sequence if datagram and datagram.header else 0,
                stream_sequence=datagram.header.stream_sequence if datagram and datagram.header else 0,
            )
            return

        detail_timing = self.diagnostic_detail_timing_enabled
        matrix_start_ns = monotonic_ns() if detail_timing else 0
        robot_urdf_matrices, current_time = self.pico_node.pico_info_transformer.get_robot_urdf_matrix_from_proto(full_body_data)
        matrix_done_ns = monotonic_ns() if detail_timing else 0
        if robot_urdf_matrices is None:
            self.diagnostic_logger.log(
                "pico_body_tracking_skip",
                reason="invalid_matrices",
                packet_sequence=datagram.header.packet_sequence if datagram and datagram.header else 0,
                stream_sequence=datagram.header.stream_sequence if datagram and datagram.header else 0,
            )
            return

        publish_matrices_start_ns = monotonic_ns() if detail_timing else 0
        self.robot_matrix_publisher.publish_matrices(robot_urdf_matrices, current_time)
        publish_matrices_done_ns = monotonic_ns() if detail_timing else 0

        # TF is optional and independently rate-limited so it cannot run on the
        # main bone-pose path when visualization or downstream consumers do not need it.
        tf_start_ns = 0
        tf_done_ns = 0
        if self._should_publish_tf(monotonic_ns()):
            tf_start_ns = monotonic_ns() if detail_timing else 0
            self.pico_node.pico_info_transformer.publish_tf_transforms(robot_urdf_matrices, current_time)
            tf_done_ns = monotonic_ns() if detail_timing else 0

        trace_base = None
        if detail_timing:
            trace_base = {
                "stage": "pico_body_tracking",
                "stamp_ms": int(current_time.to_sec() * 1000),
                "stamp_ns": int(current_time.secs) * 1000000000 + int(current_time.nsecs),
                "source_ip": datagram.addr[0] if datagram else "",
                "source_port": datagram.addr[1] if datagram else 0,
                "legacy": datagram.legacy if datagram else True,
                "fields": list(payload_fields or []),
                "recv_monotonic_ns": datagram.recv_monotonic_ns if datagram else 0,
                "enqueue_monotonic_ns": datagram.enqueue_monotonic_ns if datagram else 0,
                "process_start_monotonic_ns": datagram.process_start_monotonic_ns if datagram else 0,
                "protobuf_parse_done_monotonic_ns": datagram.protobuf_parse_done_monotonic_ns if datagram else 0,
                "matrix_start_monotonic_ns": matrix_start_ns,
                "matrix_done_monotonic_ns": matrix_done_ns,
                "publish_matrices_start_monotonic_ns": publish_matrices_start_ns,
                "publish_matrices_done_monotonic_ns": publish_matrices_done_ns,
                "tf_start_monotonic_ns": tf_start_ns,
                "tf_done_monotonic_ns": tf_done_ns,
                "session_uuid": datagram.header.session_uuid.hex() if datagram and datagram.header else "",
                "stream": datagram.header.stream if datagram and datagram.header else 0,
                "packet_sequence": datagram.header.packet_sequence if datagram and datagram.header else 0,
                "stream_sequence": datagram.header.stream_sequence if datagram and datagram.header else 0,
                "send_monotonic_ns": datagram.header.send_monotonic_ns if datagram and datagram.header else 0,
            }

        if self.pico_node.pico_info_transformer.control_mode == 'WholeBody':
            # Publish local poses
            bone_publish_start_ns = monotonic_ns() if detail_timing else 0
            self.pico_node.pico_info_transformer.publish_local_poses(robot_urdf_matrices, current_time)
            bone_publish_done_ns = monotonic_ns() if detail_timing else 0
            if detail_timing:
                self._record_bone_publish_trace(trace_base, bone_publish_start_ns, bone_publish_done_ns)

            # Process foot pose for stepping using parallel detection
            self.pico_node.pico_info_transformer.process_foot_poses_parallel(robot_urdf_matrices)

        elif self.pico_node.pico_info_transformer.control_mode == 'UpperBody':
            # Process body pose for stepping 
            bone_publish_start_ns = monotonic_ns() if detail_timing else 0
            self.pico_node.pico_info_transformer.publish_local_poses(robot_urdf_matrices, current_time)
            bone_publish_done_ns = monotonic_ns() if detail_timing else 0
            if detail_timing:
                self._record_bone_publish_trace(trace_base, bone_publish_start_ns, bone_publish_done_ns)
            
        elif self.pico_node.pico_info_transformer.control_mode == 'LowerBody':
            # Process foot pose for stepping using parallel detection
            self.pico_node.pico_info_transformer.process_foot_poses_parallel(robot_urdf_matrices)

    def _record_bone_publish_trace(self, trace_base, bone_publish_start_ns, bone_publish_done_ns):
        trace = dict(trace_base)
        trace.update({
            "bone_topic": "/leju_pico_bone_poses",
            "bone_publish_start_monotonic_ns": bone_publish_start_ns,
            "bone_publish_done_monotonic_ns": bone_publish_done_ns,
            "udp_recv_to_bone_publish_done_ns": self._duration_ns(bone_publish_done_ns, trace_base.get("recv_monotonic_ns", 0)),
            "receiver_start_to_bone_publish_done_ns": self._duration_ns(bone_publish_done_ns, trace_base.get("process_start_monotonic_ns", 0)),
            "protobuf_parse_to_bone_publish_done_ns": self._duration_ns(bone_publish_done_ns, trace_base.get("protobuf_parse_done_monotonic_ns", 0)),
            "matrix_process_ns": self._duration_ns(trace_base.get("matrix_done_monotonic_ns", 0), trace_base.get("matrix_start_monotonic_ns", 0)),
            "publish_matrices_ns": self._duration_ns(trace_base.get("publish_matrices_done_monotonic_ns", 0), trace_base.get("publish_matrices_start_monotonic_ns", 0)),
            "tf_publish_ns": self._duration_ns(trace_base.get("tf_done_monotonic_ns", 0), trace_base.get("tf_start_monotonic_ns", 0)),
            "bone_publish_ns": self._duration_ns(bone_publish_done_ns, bone_publish_start_ns),
        })
        if self.pico_node:
            self.pico_node.record_pico_bone_trace(trace)
        self.diagnostic_logger.log("pico_bone_publish_timing", **trace)
    
    def process_pico_joy_data(self, controller_data):
        """Process pico joy data and print to console."""
        try:
            self.pico_node.pico_info_transformer.publish_pico_joys(controller_data)
        except Exception as e:
            SDKLogger.error(f"Error processing pico joy data: {e}")

    def _process_item_mass_force_request(self, req: hand_wrench_srv_pb2.ItemMassForceRequest):
        """Process item mass force."""
        if req.operation == hand_wrench_srv_pb2.ItemMassForceOperation.GET:
            # 返回末端力
            self.robot_data_server.push_item_mass_force_config()
            print("get item mass force config")
        elif req.operation == hand_wrench_srv_pb2.ItemMassForceOperation.SET:
            # 设置末端力
            hand_wrench_config = HandWrenchConfig(  
                default=False,
                description=req.data.description,
                itemMass=req.data.item_mass,
                lforceX=req.data.lforce_x,
                lforceY=req.data.lforce_y,
                lforceZ=req.data.lforce_z
            )
            self.pico_node.set_item_mass_force(req.data.case_name, hand_wrench_config)
            res = hand_wrench_srv_pb2.ItemMassForceResponse()
            res.operation = req.operation
            res.description = "success"
            res.status = hand_wrench_srv_pb2.ItemMassForceResponse.OperationStatus.SUCCESS
            self.robot_data_server.add_item_mass_force_response(res)
        else:
            SDKLogger.error(f"Invalid operation: {req.operation}")

    def process_vr_command(self, vr_command: proto.VrCommandData):
        """Process VR command from RobotData protobuf message."""
        try:
            if vr_command.HasField('item_mass_force_request'):
                req = vr_command.item_mass_force_request
                self._process_item_mass_force_request(req)
            if vr_command.HasField('control_mode'):
                control_mode = vr_command.control_mode.control_mode
                if control_mode == "mobile_mpc":
                    self.pico_node.pico_info_transformer.is_mobile_mpc = True
                    self.pico_node._set_control_mode(ControlMode.MOBILE_MPC_MODE.value)
                elif control_mode == "follow":
                    self.pico_node.pico_info_transformer.is_mobile_mpc = False
                    self.pico_node.pico_info_transformer.control_torso_mode = False
                    self.pico_node._set_control_mode(ControlMode.FOLLOW_MODE.value)
                elif control_mode == "incremental":
                    self.pico_node.pico_info_transformer.is_mobile_mpc = False
                    self.pico_node.pico_info_transformer.control_torso_mode = False
                    self.pico_node._set_control_mode(ControlMode.INCREMENTAL_MODE.value)
                self.robot_data_server.control_mode = control_mode
        except Exception as e:
            SDKLogger.error(f"Error processing VR command from protobuf: {e}")

    def process_data_thread(self):
        """Process legacy and unspecified packets on the compatibility path."""
        self._process_queue_thread(self.data_queue, "legacy")

    def _process_queue_thread(self, data_queue, queue_name):
        """Consume one stream queue without blocking other stream workers."""
        SDKLogger.info("PICO %s data processing thread started", queue_name)
        while not KuavoPicoNodeManager.get_ros_is_shutdown():
            try:
                datagram = data_queue.get(timeout=0.1)
                self.process_data(datagram)
            except queue.Empty:
                continue
            except Exception as e:
                SDKLogger.error("Error processing %s data: %s", queue_name, e)

    def _enqueue_datagram(self, datagram):
        """Route fixed-header streams while preserving the legacy queue."""
        header = getattr(datagram, "header", None)
        if header is not None and int(header.stream) == 1:
            self.skeleton_queue.put_latest(datagram)
        elif header is not None and int(header.stream) == 2:
            self.controller_queue.put_latest(datagram)
        else:
            self.data_queue.put_latest(datagram)

    def start(self):
        """Start server."""
        SDKLogger.info("Starting PicoServer...")
        self.pico_node = KuavoPicoNode()
        self.pico_node.set_diagnostic_logger(
            self.diagnostic_logger if self.diagnostic_detail_timing_enabled else None
        )
        self.diagnostics.start()

        # Start independent stream workers after ROS node and diagnostics are ready.
        self.process_threads = [
            threading.Thread(
                target=self._process_queue_thread,
                args=(self.skeleton_queue, "skeleton"),
                daemon=True,
                name="pico-skeleton-worker",
            ),
            threading.Thread(
                target=self._process_queue_thread,
                args=(self.controller_queue, "controller"),
                daemon=True,
                name="pico-controller-worker",
            ),
            threading.Thread(
                target=self.process_data_thread,
                daemon=True,
                name="pico-legacy-worker",
            ),
        ]
        self.process_thread = self.process_threads[0]
        for thread in self.process_threads:
            thread.start()
        
        self.socket.settimeout(1)
        
        SDKLogger.info(f"PICO UDP Server listening on {self.host}:{self.port}")
        
        if not self.send_initial_message():
            SDKLogger.error("Failed to establish initial connection")
            return

        SDKLogger.info("Waiting for data...")
        while not KuavoPicoNodeManager.get_ros_is_shutdown():
            try:
                data, addr = self.socket.recvfrom(65535)
                recv_ns = monotonic_ns()
                if(self.robot_data_server.target_client_addr != addr):
                    SDKLogger.debug(f"get new client: {addr}")
                    self.robot_data_server.target_client_addr = addr
                datagram = self.diagnostics.decode_datagram(data, addr, recv_ns)
                if datagram is None:
                    continue
                if datagram.legacy:
                    self.diagnostics.set_peer(addr)
                datagram.enqueue_monotonic_ns = monotonic_ns()
                self._enqueue_datagram(datagram)
            except socket.timeout:
                continue
            except Exception as e:
                SDKLogger.error(f"Error in main loop: {e}")
                if not self.handle_error(e, ErrorState.CONNECTION_ERROR):
                    break
    
if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    args = parse_args()
    try:
        server = KuavoPicoServer(**vars(args))
        server.start()
    except Exception as e:
        server.clean_up()
        SDKLogger.error(f"Node interrupted: {e}")
