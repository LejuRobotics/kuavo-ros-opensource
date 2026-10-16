"""Nonblocking visual-only IPC. No input, retargeting, filter, or control logic."""
import json
import os
from pathlib import Path
import socket
import struct
import subprocess
import sys
import time
import weakref

_HEADER = struct.Struct("<4sQQB")
_JOB = struct.Struct("<B12d")
_MAGIC = b"KMV1"
_MAX_PACKET = _HEADER.size + 2*_JOB.size
_instances = weakref.WeakSet()

TOPICS = {
    "marker_pub": "visualization_marker",
    "marker_pub_right": "visualization_marker_right",
    "marker_pub_elbow": "visualization_marker/elbow",
    "marker_pub_elbow_right": "visualization_marker_right/elbow",
    "marker_pub_shoulder": "visualization_marker/shoulder",
    "marker_pub_shoulder_right": "visualization_marker_right/shoulder",
    "marker_pub_chest": "visualization_marker_chest",
}

def encode(jobs, sequence, monotonic_ns):
    if not 1 <= len(jobs) <= 2:
        raise ValueError("expected one or two visual arm jobs")
    data = [_HEADER.pack(_MAGIC, sequence, monotonic_ns, len(jobs))]
    for side, hand, elbow, shoulder, chest in jobs:
        if side not in ("Left", "Right"):
            raise ValueError("invalid visual side")
        points = list(hand)+list(elbow)+list(shoulder)+[
            chest.position.x, chest.position.y, chest.position.z]
        data.append(_JOB.pack(0 if side == "Left" else 1, *points))
    return b"".join(data)

def decode(data):
    if len(data) < _HEADER.size:
        raise ValueError("short visual packet")
    magic, sequence, stamp, count = _HEADER.unpack_from(data)
    if magic != _MAGIC or count not in (1, 2) or len(data) != _HEADER.size+count*_JOB.size:
        raise ValueError("invalid visual packet")
    jobs = []
    for i in range(count):
        row = _JOB.unpack_from(data, _HEADER.size+i*_JOB.size)
        if row[0] not in (0, 1):
            raise ValueError("invalid visual side")
        jobs.append(("Left" if row[0] == 0 else "Right", row[1:4], row[4:7], row[7:10], row[10:13]))
    return sequence, stamp, jobs

class MarkerProcess:
    """Bounded SOCK_SEQPACKET, one best-effort send per bone callback.

    Full queues or a dead visual process drop visuals only. No blocking send,
    join, ROS publish, retry, or child restart is performed by submit().
    """
    def __init__(self, topics, rate=30.0):
        if not 1.0 <= float(rate) <= 120.0:
            raise ValueError("visual rate must be in [1, 120] Hz")
        self.sent = self.dropped = self.errors = self.attempts = 0
        self.closed = False
        self.failed = False
        self.process = None
        parent, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        self.socket = parent
        try:
            parent.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 8192)
            parent.setblocking(False)
            command = [sys.executable, str(Path(__file__).resolve()), "--worker",
                       str(child.fileno()), json.dumps(topics), str(float(rate))]
            # No preexec_fn / multiprocessing fork of a live rospy interpreter.
            self.process = subprocess.Popen(command, pass_fds=(child.fileno(),),
                                            close_fds=True, start_new_session=True)
        except BaseException:
            parent.close()
            raise
        finally:
            child.close()
        _instances.add(self)

    def submit(self, jobs):
        if not jobs:
            return True
        self.attempts += 1
        if self.closed or self.failed:
            self.dropped += 1
            return False
        try:
            data = encode(jobs, self.attempts, time.monotonic_ns())
            # SOCK_SEQPACKET preserves the whole bounded packet or fails.
            if self.socket.send(data, socket.MSG_DONTWAIT | socket.MSG_NOSIGNAL) != len(data):
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
        self.socket.close()  # EOF stops worker even on unexpected parent exit.
        if self.process is not None:
            try:
                self.process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self.process.terminate()
                try:
                    self.process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait(timeout=2.0)

def diagnostics():
    return [dict(pid=x.process.pid, returncode=x.process.poll(), sent=x.sent,
                 dropped=x.dropped, errors=x.errors, attempts=x.attempts,
                 closed=x.closed, failed=x.failed) for x in list(_instances) if x.process is not None]

def point_marker(point, scale=0.05, alpha=0.3, color=(0, 0, 1)):
    # Match Quest3ArmInfoTransformer.construct_point_marker, including publish-time stamp.
    import rospy
    from visualization_msgs.msg import Marker
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.scale.x = marker.scale.y = marker.scale.z = scale
    marker.color.a = alpha
    marker.color.r, marker.color.g, marker.color.b = color
    marker.header.stamp = rospy.Time.now()
    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = point
    marker.pose.orientation.w = 1.0
    return marker

def publish_jobs(publishers, jobs):
    for side, hand, elbow, shoulder, chest in jobs:
        suffix = "" if side == "Left" else "_right"
        publishers["marker_pub"+suffix].publish(point_marker(hand, .08, .9, (1, 0, 0)))
        publishers["marker_pub_elbow"+suffix].publish(point_marker(elbow, .1, .3, (0, 1, 0)))
        publishers["marker_pub_shoulder"+suffix].publish(point_marker(shoulder, .1, .3, (0, 0, 1)))
    if jobs:
        publishers["marker_pub_chest"].publish(point_marker(jobs[0][4], .1, .8))

def worker(fd, topics, rate):
    import select
    import rospy
    from visualization_msgs.msg import Marker
    from std_msgs.msg import Float64MultiArray

    channel = socket.socket(fileno=fd)
    channel.setblocking(False)
    # Lower only this visual process; never touch robot/input/IK scheduling.
    os.sched_setscheduler(0, os.SCHED_OTHER, os.sched_param(0))
    os.setpriority(os.PRIO_PROCESS, 0, 10)
    rospy.init_node("quest_arm_marker_worker", anonymous=True)
    publishers = {name: rospy.Publisher(topic, Marker, queue_size=1) for name, topic in topics.items()}
    health = rospy.Publisher("/whole_body_velocity_ik/diagnostics/arm_marker_worker_v1",
                             Float64MultiArray, queue_size=1)
    period = 1.0/rate
    next_publish = time.monotonic()
    next_health = next_publish
    received = rendered = coalesced = stale = invalid = 0
    latest = None
    last_sequence = last_source_ns = 0
    disconnected = False
    try:
        while not rospy.is_shutdown() and not disconnected:
            now = time.monotonic()
            timeout = min(.1, max(0.0, next_publish-now))
            if select.select([channel], [], [], timeout)[0]:
                # Bound work per pass; an unlimited producer must not starve publish/shutdown.
                for _ in range(64):
                    try:
                        packet = channel.recv(_MAX_PACKET+1)
                    except BlockingIOError:
                        break
                    if not packet:
                        disconnected = True
                        break
                    try:
                        value = decode(packet)
                    except (ValueError, struct.error):
                        invalid += 1
                        continue
                    received += 1
                    if latest is not None:
                        coalesced += 1
                    latest = value
            now = time.monotonic()
            if now >= next_publish:
                if latest is not None:
                    sequence, stamp, jobs = latest
                    latest = None
                    if 0 <= time.monotonic_ns()-stamp <= 250_000_000:
                        publish_jobs(publishers, jobs)
                        rendered += 1
                        last_sequence, last_source_ns = sequence, stamp
                    else:
                        stale += 1
                next_publish = now+period
            if now >= next_health:
                health.publish(Float64MultiArray(data=[
                    1, os.getpid(), time.monotonic_ns(), received, rendered,
                    coalesced, stale, invalid, last_sequence, last_source_ns]))
                next_health = now+1.0
    finally:
        channel.close()
        rospy.signal_shutdown("visual IPC closed")

if __name__ == "__main__":
    if len(sys.argv) != 5 or sys.argv[1] != "--worker":
        raise SystemExit("This visual helper is started by the Mapper")
    worker(int(sys.argv[2]), json.loads(sys.argv[3]), float(sys.argv[4]))
