"""Bounded asynchronous observation. No control/mapping/filter decisions."""
import functools
import json
import sys
import os
import queue
import resource
import threading
import time
from std_msgs.msg import Float64MultiArray, String
import rospy

_local = threading.local()

def point(stage):
    frame = getattr(_local, "frame", None)
    if frame is None:
        return
    if len(frame["points"]) >= 96:
        frame["overflow"] = 1
        return
    wall = time.monotonic_ns()
    cpu = time.thread_time_ns()
    usage = resource.getrusage(resource.RUSAGE_THREAD)
    frame["points"].append((stage, wall, cpu, usage.ru_nvcsw, usage.ru_nivcsw,
                            usage.ru_minflt, usage.ru_majflt))

def identity(sequence, session, generation):
    frame = getattr(_local, "frame", None)
    if frame is not None:
        frame["sequence"], frame["session"], frame["generation"] = sequence, session, generation

def thread_identities():
    """Low-rate diagnostic identity only; never invoked by the bone callback.

    Stacks identify business functions at sample time, not GIL ownership over
    an interval. Do not inspect user data, repr objects, or install thread hooks.
    """
    started = time.monotonic_ns()
    cpu_started = time.thread_time_ns()
    frames = sys._current_frames()
    rows = []
    threads = threading.enumerate()
    try:
        for thread in threads[:128]:
            target = getattr(thread, "_target", None)
            row = dict(tid=thread.native_id, name=thread.name,
                       target=getattr(target, "__qualname__", ""))
            objects = [getattr(target, "__self__", None)]
            objects.extend(getattr(thread, "_args", ())[:4])
            bindings = []
            for obj in objects:
                values = getattr(obj, "__dict__", {})
                connection = values.get("_connection")
                if connection is not None:
                    values = getattr(connection, "__dict__", {})
                name = values.get("name")
                if isinstance(name, str):
                    bindings.append(dict(type=type(obj).__name__, topic=name[:160]))
            row["bindings"] = bindings
            stack = []
            frame = frames.get(thread.ident)
            while frame is not None and len(stack) < 12:
                stack.append([os.path.basename(frame.f_code.co_filename),
                              frame.f_code.co_name, frame.f_lineno])
                frame = frame.f_back
            row["stack"] = stack
            rows.append(row)
    finally:
        # Do not retain suspended frames and their locals.
        frames.clear()
        frame = None
    marker_module = sys.modules.get("tools.arm_marker_process")
    marker_processes = marker_module.diagnostics() if marker_module is not None else []
    diagnostic_module = sys.modules.get("tools.noncontrol_publish_process")
    noncontrol_processes = diagnostic_module.diagnostics() if diagnostic_module is not None else []
    return dict(version=1, pid=os.getpid(), monotonic_ns=started, marker_processes=marker_processes,
                noncontrol_processes=noncontrol_processes,
                wall_ns=time.monotonic_ns()-started,
                cpu_ns=time.thread_time_ns()-cpu_started,
                thread_count=len(threads), truncated=len(threads)>128,
                threads=rows)


class Recorder:
    def __init__(self):
        self.enabled = rospy.get_param("~runtime_probe_enabled", True)
        self.dropped = 0
        self.errors = 0
        self.serial = 0
        self.queue = queue.Queue(maxsize=128)
        self.stop = threading.Event()
        self.external = None
        if self.enabled and os.environ.get("KUAVO_RUNTIME_PARTITION_JSON"):
            from tools.mapper_probe_process import ProbeProcess
            self.external = ProbeProcess()
            rospy.on_shutdown(self.external.close)
        if self.enabled and self.external is None:
            self.publisher = rospy.Publisher(
                "/whole_body_velocity_ik/diagnostics/mapper_runtime_probe_v1",
                Float64MultiArray, queue_size=32)
            self.input_publisher = rospy.Publisher(
                "/whole_body_velocity_ik/diagnostics/mapper_input_gap_v1",
                Float64MultiArray, queue_size=32)
            self.identity_publisher = rospy.Publisher(
                "/whole_body_velocity_ik/diagnostics/mapper_thread_identity_v1",
                String, queue_size=1, latch=True)
            self.next_identity = 0.0
            self.worker = threading.Thread(target=self._run, name="mapper_probe_writer", daemon=True)
            self.worker.start()
            rospy.on_shutdown(self.stop.set)

    def _run(self):
        while not self.stop.is_set():
            now = time.monotonic()
            if now >= self.next_identity:
                self.next_identity = now + 2.0
                try:
                    if self.identity_publisher.get_num_connections():
                        self.identity_publisher.publish(String(data=json.dumps(thread_identities())))
                except Exception:
                    self.errors += 1
            try:
                frame = self.queue.get(timeout=0.05)
            except queue.Empty:
                continue
            try:
                data = [1, os.getpid(), frame["tid"], frame["serial"],
                        frame["session"], frame["generation"], frame["sequence"],
                        frame["source_ms"], self.dropped, self.errors, frame["overflow"],
                        len(frame["points"])]
                for sample in frame["points"]:
                    data.extend(sample)
                self.publisher.publish(Float64MultiArray(data=data))
                self.input_publisher.publish(Float64MultiArray(data=input_gap_row(frame, self.dropped)))
            except Exception:
                self.errors += 1

def input_entry(msg, previous):
    """Intervals only: Quest source time and host monotonic have different epochs.

    source_kind: 1=header.stamp, 2=PoseInfoList.timestamp_ms, 0=unavailable.
    gap CPU is receive-thread CPU between callbacks (incl. deserialization),
    not proof of network delay or GIL ownership. Correlate with kernel trace.
    """
    wall, cpu = time.monotonic_ns(), time.thread_time_ns()
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is not None:
        kind, source = 1, stamp.to_nsec()
    elif hasattr(msg, "timestamp_ms"):
        kind, source = 2, float(msg.timestamp_ms)*1e6
    else:
        kind, source = 0, float("nan")
    end, end_cpu, old_kind, old_source = previous or (0, 0, 0, float("nan"))
    return [wall, cpu, end, (wall-end)*1e-6 if end else float("nan"),
            kind, source, (source-old_source)*1e-6 if kind and kind == old_kind else float("nan"),
            end_cpu, (cpu-end_cpu)*1e-6 if end else float("nan")]


def input_gap_row(frame, dropped):
    return [1, os.getpid(), frame["tid"], frame["serial"], *frame["entry"], dropped]


def callback(function):
    @functools.wraps(function)
    def wrapped(self, msg):
        recorder = getattr(self, "_runtime_probe", None)
        if recorder is None or not recorder.enabled:
            return function(self, msg)
        histories = getattr(_local, "input_history", None)
        if histories is None:
            histories = _local.input_history = {}
        key = (id(recorder), function)
        entry = input_entry(msg, histories.get(key))
        recorder.serial += 1
        frame = dict(tid=threading.get_native_id(), serial=recorder.serial,
                     source_ms=msg.timestamp_ms, sequence=-1, session=-1,
                     generation=-1, overflow=0, points=[], entry=entry)
        previous = getattr(_local, "frame", None)
        _local.frame = frame
        point(0)
        try:
            return function(self, msg)
        finally:
            point(99)
            _local.frame = previous
            if recorder.external is not None:
                recorder.external.submit(frame, recorder.dropped, recorder.errors)
            else:
                try:
                    recorder.queue.put_nowait(frame)
                except queue.Full:
                    recorder.dropped += 1
            # Includes probe handoff overhead in the preceding callback, rather
            # than falsely charging it to the next inter-callback gap.
            histories[key] = (time.monotonic_ns(), time.thread_time_ns(), entry[4], entry[5])
    return wrapped
