"""Allowlisted diagnostic ROS publication in a separate interpreter.

Only immutable serialized snapshots cross bounded nonblocking IPC. No target,
hand, head/body command, or lifecycle acknowledgement topic is accepted here.
"""
import io
import json
import os
from pathlib import Path
import socket
import struct
import subprocess
import sys
import time
import math
import threading

# Stable channel IDs are local IPC only; ROS types and topic payloads are unchanged.
CHANNELS = [
    ("/whole_body_velocity_ik/diagnostics/mapper_timing", "std_msgs/Float64MultiArray", 32),
    ("/quest3_debug/chest_axis", "std_msgs/Float32MultiArray", 10),
    ("/quest3_debug/shoulder_angle", "std_msgs/Float32MultiArray", 10),
    ("/ik_debug/quest_tcp_mapping", "wheel_arm_abs_vel/QuestTcpMappingDebug", 50),
    ("/ik_visualization_markers", "visualization_msgs/MarkerArray", 10),
    ("/visualization_marker/human_array_left", "visualization_msgs/MarkerArray", 10),
    ("/visualization_marker/human_array_right", "visualization_msgs/MarkerArray", 10),
    ("/visualization_marker/shoulder_quest3", "visualization_msgs/Marker", 10),
    ("/visualization_marker_right/shoulder_quest3", "visualization_msgs/Marker", 10),
    ("/visualization_marker/upper_arm_constraint_left", "visualization_msgs/Marker", 10),
    ("/visualization_marker/upper_arm_constraint_right", "visualization_msgs/Marker", 10),
    ("/joint_states", "sensor_msgs/JointState", 10),
]
_HEADER = struct.Struct("<4sHQ")
_MAGIC = b"KDP1"
_MAX_PAYLOAD = 65536
_dispatcher = None

class DiagnosticProcess:
    def __init__(self, topics, rate_hz=100.0, coalesce=True):
        self.sent = self.dropped = self.errors = self.attempts = 0
        self.closed = self.failed = False
        self.process = None
        self.period = 0.0 if rate_hz <= 0 else 1.0 / rate_hz
        self.last_submit = {}
        self.gate_lock = threading.Lock()
        parent, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        self.socket = parent
        try:
            parent.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 131072)
            parent.setblocking(False)
            self.process = subprocess.Popen(
                [sys.executable, str(Path(__file__).resolve()), "--worker",
                 str(child.fileno()), json.dumps(topics), str(int(coalesce))], pass_fds=(child.fileno(),),
                close_fds=True, start_new_session=True)
        except Exception:
            self.failed = True
            self.errors += 1
            parent.close()
        finally:
            child.close()

    def submit(self, channel, message):
        self.attempts += 1
        if self.closed or self.failed:
            self.dropped += 1
            return False
        with self.gate_lock:
            now = time.monotonic()
            if now - self.last_submit.get(channel, float("-inf")) < self.period:
                return False
            self.last_submit[channel] = now
        try:
            # Gate before serialization/IPC. Subsequent mutation cannot leak.
            data = io.BytesIO()
            message.serialize(data)
            payload = data.getvalue()
            if len(payload) > _MAX_PAYLOAD:
                self.dropped += 1
                return False
            packet = _HEADER.pack(_MAGIC, channel, time.monotonic_ns()) + payload
            if self.socket.send(packet, socket.MSG_DONTWAIT | socket.MSG_NOSIGNAL) != len(packet):
                self.errors += 1
                self.dropped += 1
                return False
        except BlockingIOError:
            self.dropped += 1
            return False
        except OSError:
            self.failed = True
            self.errors += 1
            self.dropped += 1
            return False
        except Exception:
            self.errors += 1
            self.dropped += 1
            return False
        self.sent += 1
        return True

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.socket.close()
        if self.process is not None:
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.process.terminate()
                try:
                    self.process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait(timeout=2)

class DiagnosticPublisher:
    def __init__(self, process, channel, data_class):
        self.process, self.channel, self.data_class = process, channel, data_class

    def publish(self, message):
        if not isinstance(message, self.data_class):
            raise TypeError("Diagnostic publisher expects its declared ROS message")
        return self.process.submit(self.channel, message)

class RatePublisher:
    """Synchronous fallback uses the same configurable observation rate."""
    def __init__(self, pub, rate):
        self.pub = pub
        self.period = 0.0 if rate <= 0 else 1.0 / rate
        self.last = float("-inf")

    def publish(self, message):
        now = time.monotonic()
        if now - self.last < self.period or self.pub.get_num_connections() == 0:
            return False
        self.last = now
        self.pub.publish(message)
        return True

    def get_num_connections(self):
        return self.pub.get_num_connections()

def publisher(topic, data_class, queue_size=10):
    """Create during node initialization; never initialize a process in a callback."""
    global _dispatcher
    import rospy
    name = "/"+topic.lstrip("/")
    channel = next((i for i, spec in enumerate(CHANNELS) if spec[0] == name), None)
    if channel is None or CHANNELS[channel][1] != data_class._type or CHANNELS[channel][2] != queue_size:
        raise ValueError("Non-control publisher topic/type/queue is not allowlisted: "+name)
    rate = float(rospy.get_param("~noncontrol_publish_rate_hz", 100.0))
    if not math.isfinite(rate):
        raise ValueError("noncontrol_publish_rate_hz must be finite")
    if not rospy.get_param("~noncontrol_publish_async", False):
        return RatePublisher(rospy.Publisher(topic, data_class, queue_size=queue_size), rate)
    if _dispatcher is None:
        _dispatcher = DiagnosticProcess(
            [rospy.resolve_name(t) for t, _, _ in CHANNELS], rate,
            bool(rospy.get_param("~noncontrol_publish_coalesce", True)))
        rospy.on_shutdown(_dispatcher.close)
        if _dispatcher.failed:
            rospy.logwarn("Diagnostic publication process failed; diagnostic output disabled")
    return DiagnosticPublisher(_dispatcher, channel, data_class)

def diagnostics():
    d = _dispatcher
    if d is None:
        return []
    return [dict(pid=d.process.pid if d.process else None,
                 returncode=d.process.poll() if d.process else None,
                 sent=d.sent, dropped=d.dropped, errors=d.errors, attempts=d.attempts,
                 closed=d.closed, failed=d.failed)]

def worker(fd, topics, coalesce=True):
    try:
        from .background_fence import apply_background_fence
    except ImportError:
        from background_fence import apply_background_fence
    apply_background_fence()
    import select
    import rospy
    from roslib.message import get_message_class
    from std_msgs.msg import String
    if len(topics) != len(CHANNELS):
        raise ValueError("Diagnostic topic count")
    channel = socket.socket(fileno=fd)
    channel.setblocking(False)
    os.sched_setscheduler(0, os.SCHED_OTHER, os.sched_param(0))
    os.setpriority(os.PRIO_PROCESS, 0, 10)
    rospy.init_node("mapper_noncontrol_worker", anonymous=True)
    classes = [get_message_class(spec[1]) for spec in CHANNELS]
    if any(c is None for c in classes):
        raise RuntimeError("Missing diagnostic ROS message class")
    pubs = [rospy.Publisher(t, c, queue_size=spec[2])
            for t, c, spec in zip(topics, classes, CHANNELS)]
    health = rospy.Publisher("/whole_body_velocity_ik/diagnostics/mapper_noncontrol_worker_v1",
                             String, queue_size=1, latch=True)
    received = published = stale = invalid = errors = 0
    next_health = 0
    disconnected = False
    try:
        while not rospy.is_shutdown() and not disconnected:
            if select.select([channel], [], [], .1)[0]:
                pending = {} if coalesce else []
                for _ in range(64):
                    try:
                        packet = channel.recv(_HEADER.size+_MAX_PAYLOAD+1)
                    except BlockingIOError:
                        break
                    if not packet:
                        disconnected = True
                        break
                    received += 1
                    try:
                        if not _HEADER.size <= len(packet) <= _HEADER.size+_MAX_PAYLOAD:
                            raise ValueError("diagnostic packet size")
                        magic, index, stamp = _HEADER.unpack_from(packet)
                        if magic != _MAGIC or index >= len(pubs):
                            raise ValueError("diagnostic packet header")
                        if not 0 <= time.monotonic_ns()-stamp <= 250_000_000:
                            stale += 1
                            continue
                        if coalesce:
                            pending[index] = packet[_HEADER.size:]
                        else:
                            pending.append((index, packet[_HEADER.size:]))
                    except Exception:
                        invalid += 1
                        continue
                # Bounded drain, latest per channel: one noisy observation
                # channel cannot force dozens of redundant deserialize/publish calls.
                items = pending.items() if coalesce else pending
                for index, payload in items:
                    if pubs[index].get_num_connections() == 0:
                        continue
                    try:
                        message = classes[index]().deserialize(payload)
                        pubs[index].publish(message)
                        published += 1
                    except Exception:
                        errors += 1
            if time.monotonic() >= next_health:
                health.publish(String(data=json.dumps(dict(version=1, pid=os.getpid(),
                    monotonic_ns=time.monotonic_ns(), received=received, published=published,
                    stale=stale, invalid=invalid, errors=errors))))
                next_health = time.monotonic()+1
    finally:
        channel.close()
        rospy.signal_shutdown("diagnostic IPC closed")

if __name__ == "__main__":
    if len(sys.argv) != 5 or sys.argv[1] != "--worker":
        raise SystemExit("Started only by the Mapper diagnostic publisher")
    worker(int(sys.argv[2]), json.loads(sys.argv[3]), bool(int(sys.argv[4])))
