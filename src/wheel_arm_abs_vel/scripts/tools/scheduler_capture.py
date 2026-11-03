#!/usr/bin/env python3
"""FOLLOW-gated kernel flight recorder. Observes only; never starts control.
ROS imports are lazy so parser/state-machine tests need no ROS master.
"""
import argparse
import collections
import json
import os
from pathlib import Path
import re
import shutil
import signal
import threading
import time

PREFIX = "/whole_body_velocity_ik/diagnostics/"
TRACE_ROOT = Path("/sys/kernel/tracing")

def read(path):
    try:
        return Path(path).read_text().strip()
    except (OSError, ProcessLookupError):
        return ""

def write(path, value):
    Path(path).write_text(str(value))

def process_identity(pid, role):
    p = Path("/proc") / str(pid)
    names = [os.path.basename(x.decode(errors="replace"))
             for x in (p / "cmdline").read_bytes().split(b"\0")[:3]]
    expected = {"ik": "pinocchio_velocity_ik_node", "mapper": "ik_ros_uni.py"}
    if expected[role] not in names:
        raise ValueError("Unexpected process for " + role)
    # field 22; comm can contain spaces/parentheses.
    start = (p / "stat").read_text().rsplit(")", 1)[1].split()[19]
    return {"pid": pid, "start_ticks": int(start), "argv": names,
            "exe": os.readlink(str(p / "exe"))}

def proc_inventory(focus_pids):
    rows = []
    for proc in Path("/proc").glob("[0-9]*"):
        for task in (proc / "task").glob("[0-9]*"):
            tid = int(task.name)
            row = dict(pid=int(proc.name), tid=tid, comm=read(task / "comm"))
            if int(proc.name) in focus_pids:
                try:
                    row.update(policy=os.sched_getscheduler(tid),
                               priority=os.sched_getparam(tid).sched_priority,
                               affinity=sorted(os.sched_getaffinity(tid)),
                               nice=os.getpriority(os.PRIO_PROCESS, tid))
                except ProcessLookupError:
                    continue
                row.update(schedstat=read(task / "schedstat"), wchan=read(task / "wchan"))
            rows.append(row)
    return rows

def parse_probe(role, data):
    h, count_index, cap = (12, 11, 96) if role == "mapper" else (10, 8, 64)
    if len(data) < h or data[0] != 1:
        raise ValueError("probe header")
    n = int(data[count_index])
    if not 2 <= n <= cap or len(data) != h + 7*n:
        raise ValueError("probe length")
    pts = [tuple(int(x) for x in data[i:i+7]) for i in range(h, len(data), 7)]
    if pts[0][0] != 0 or pts[-1][0] != 99:
        raise ValueError("probe endpoints")
    if any(a[1] > b[1] for a, b in zip(pts, pts[1:])):
        raise ValueError("probe clock order")
    return dict(role=role, pid=int(data[1]), tid=int(data[2]), serial=int(data[3]),
                session=int(data[4]), generation=int(data[5]), sequence=int(data[6]),
                dropped=int(data[8] if role == "mapper" else data[7]),
                overflow=int(data[10] if role == "mapper" else data[9]),
                points=pts, begin=pts[0][1], end=pts[-1][1])

class State:
    """Callbacks only update bounded memory; filesystem/trace I/O is on main thread."""
    def __init__(self):
        self.lock = threading.Lock()
        self.latest = {}
        self.reference = None
        self.active_key = None
        self.pending = {}
        self.identities = None
        self.inputs = collections.deque(maxlen=1024)
        self.errors = 0
        self.missing = collections.Counter()
        self.stats = collections.Counter()
        self.input_overflow = 0

    def reference_update(self, msg, now=None):
        now = time.monotonic_ns() if now is None else now
        with self.lock:
            self.reference = (now, int(msg.session_id), int(msg.generation),
                              msg.control_state == 2 and msg.state == 1
                              and bool(msg.valid) and bool(msg.takeover_active))

    def key(self, now):
        r = self.reference
        if not r or not r[3] or not 0 <= now-r[0] < 500_000_000:
            return None
        if set(self.latest) != {"ik", "mapper"}:
            return None
        a, b = self.latest["ik"], self.latest["mapper"]
        for p in (a, b):
            if not 0 <= now-p["end"] < 500_000_000:
                return None
            if (p["session"], p["generation"]) != r[1:3]:
                return None
        return (a["pid"], a["tid"], b["pid"], b["tid"], r[1], r[2])

    def set_active(self, key):
        with self.lock:
            self.active_key = key
            self.pending.clear()

    def probe(self, role, data, now=None):
        try:
            p = parse_probe(role, data)
        except (ValueError, TypeError, OverflowError):
            with self.lock:
                self.errors += 1
            return
        now = time.monotonic_ns() if now is None else now
        with self.lock:
            previous = self.latest.get(role)
            self.latest[role] = p
            self.stats[role+"_received"] += 1
            same = previous is not None and all(p[k] == previous[k] for k in
                    ("pid", "tid", "session", "generation"))
            consecutive = same and p["serial"] == previous["serial"]+1
            if same and p["serial"] > previous["serial"]+1:
                self.missing[role] += p["serial"]-previous["serial"]-1
            if self.active_key is None or self.key(now) != self.active_key:
                return
            candidates = []
            stage = max(zip(p["points"], p["points"][1:]), key=lambda x: x[1][1]-x[0][1])
            duration = stage[1][1]-stage[0][1]
            if duration >= (3_000_000 if role == "ik" else 5_000_000):
                candidates.append((role+"_stage", duration, stage[0][1]))
            if consecutive:
                pub = 81 if role == "ik" else 51
                old = [x[1] for x in previous["points"] if x[0] == pub]
                new = [x[1] for x in p["points"] if x[0] == pub]
                if old and new and new[-1]-old[-1] >= (8_000_000 if role == "ik" else 25_000_000):
                    candidates.append((role+"_gap", new[-1]-old[-1], old[-1]))
            for category, duration, begin in candidates:
                event = dict(category=category, duration_ns=duration, begin=begin,
                             end=p["end"], observed_ns=now, probe=p,
                             previous_probe=previous if consecutive else None)
                self.stats[category+"_detected"] += 1
                if category not in self.pending or duration > self.pending[category]["duration_ns"]:
                    self.pending[category] = event

    def identity(self, text):
        try:
            value = json.loads(text)
            if value["version"] != 1 or len(value["threads"]) > 128:
                raise ValueError("identity schema")
            with self.lock:
                self.identities = value
        except (ValueError, KeyError, TypeError):
            with self.lock:
                self.errors += 1

    def input(self, data):
        with self.lock:
            if self.active_key:
                if len(self.inputs) == self.inputs.maxlen:
                    self.input_overflow += 1
                self.inputs.append((time.monotonic_ns(), list(data)))

class FlightTrace:
    def __init__(self, directory, tids, mapper_pid, buffer_kb=512):
        self.directory = Path(directory)
        self.path = TRACE_ROOT / "instances" / ("kuavo_flight_" + str(os.getpid()))
        self.enabled = []
        self.filters = {}
        self.tids = list(tids)
        self.mapper_pid = mapper_pid
        self.started_ns = time.monotonic_ns()
        self.path.mkdir()
        try:
            write(self.path/"tracing_on", 0)
            write(self.path/"trace_clock", "mono")
            write(self.path/"buffer_size_kb", buffer_kb)
            for option in ("record-tgid", "record-cmd"):
                if (self.path/"options"/option).exists():
                    write(self.path/"options"/option, 1)
            # Continuous CPU timeline, not just the first successor of a focus thread.
            self.enable("sched/sched_switch")
            focus = " || ".join("pid == "+str(tid) for tid in tids)
            for ev in ("sched_wakeup", "sched_wakeup_new", "sched_migrate_task"):
                self.enable("sched/"+ev, focus)
            for ev in ("sched_process_fork", "sched_process_exec", "sched_process_exit"):
                self.enable("sched/"+ev)
            for ev in ("irq_handler_entry", "irq_handler_exit", "softirq_entry", "softirq_exit"):
                self.enable("irq/"+ev, required=False)
            focused = " || ".join("common_pid == "+str(tid) for tid in tids)
            # Only the two producer/callback threads, never every IK BLAS worker.
            for call in ("futex", "nanosleep", "clock_nanosleep", "recvfrom", "recvmsg",
                         "sendto", "sendmsg", "poll", "ppoll", "select", "pselect6"):
                for edge in ("enter", "exit"):
                    self.enable("syscalls/sys_"+edge+"_"+call, focused, required=False)
            # Extra GIL-wake syscalls from mapper threads use verified addresses.
            self.refresh_mapper_threads()
            # Allocate snapshot storage while tracing is off.
            write(self.path/"snapshot", 1)
            write(self.path/"trace", "")
            write(self.path/"tracing_on", 1)
            self.started_ns = time.monotonic_ns()
        except BaseException:
            self.close()
            raise

    def enable(self, event, filt=None, required=True):
        path = self.path/"events"/event
        if not path.exists():
            if required:
                raise RuntimeError("Missing event "+event)
            return
        if filt:
            write(path/"filter", filt)
            self.filters[event] = read(path/"filter")
        write(path/"enable", 1)
        self.enabled.append(event)

    def refresh_mapper_threads(self):
        # Optional exact binding is supplied only by a matching audited Python ELF.
        binding = self.directory.parent/"gil_binding.json"
        if not binding.exists():
            return
        evidence = json.loads(binding.read_text())
        exe = Path("/proc")/str(self.mapper_pid)/"exe"
        import hashlib
        if not hasattr(self, "gil_verified"):
            digest = hashlib.sha256(exe.read_bytes()).hexdigest()
            self.gil_verified = digest == evidence["python_sha256"]
        if not self.gil_verified:
            return
        tids = sorted(int(p.name) for p in (Path("/proc")/str(self.mapper_pid)/"task").glob("[0-9]*"))
        if tids == getattr(self, "mapper_tids", None):
            return
        self.mapper_tids = tids
        target = " || ".join("common_pid == "+str(t) for t in self.tids)
        mapper = " || ".join("common_pid == "+str(t) for t in tids) or "common_pid == -1"
        addresses = " || ".join("uaddr == "+str(int(v, 16)) for v in evidence["addresses"])
        # Keep all focus futex calls; other mapper threads only GIL address events.
        filt = "(" + target + ") || ((" + mapper + ") && (" + addresses + "))"
        write(self.path/"events/syscalls/sys_enter_futex/filter", filt)
        self.filters["syscalls/sys_enter_futex"] = read(self.path/"events/syscalls/sys_enter_futex/filter")

    def snapshot(self, stem, event):
        started = time.monotonic_ns()
        write(self.path/"snapshot", 1)  # atomic swap; running ring continues, no trace_pipe consumer
        snapshot_ns = time.monotonic_ns()
        minimum = max(self.started_ns, event["begin"]-100_000_000)
        maximum = snapshot_ns
        trace = self.directory/(stem+".trace")
        ranges = {}
        read_bytes = saved_bytes = lost_lines = 0
        limited = False
        pat = re.compile(r"\[(\d+)\].*?\s(\d+)\.(\d+): ")
        with (self.path/"snapshot").open(errors="replace") as source, trace.open("w") as dest:
            for line in source:
                read_bytes += len(line)
                if read_bytes > 96*1024**2:
                    limited = True
                    break
                match = pat.search(line)
                if "LOST" in line:
                    lost_lines += 1
                    dest.write(line)
                if not match:
                    if line.startswith("#"):
                        dest.write(line)
                    continue
                cpu = int(match[1])
                stamp = int(match[2])*1_000_000_000+int(match[3].ljust(9, "0"))
                bounds = ranges.setdefault(cpu, [stamp, stamp])
                bounds[0] = min(bounds[0], stamp)
                bounds[1] = max(bounds[1], stamp)
                if minimum <= stamp <= maximum:
                    dest.write(line)
                    saved_bytes += len(line)
                    if saved_bytes > 16*1024**2:
                        limited = True
                        break
        meta = dict(trigger=event, capture_monotonic_ns=started, snapshot_monotonic_ns=snapshot_ns,
                    save_end_monotonic_ns=time.monotonic_ns(), requested_start_ns=minimum,
                    requested_end_ns=maximum, read_bytes=read_bytes, saved_bytes=saved_bytes,
                    size_truncated=limited, lost_lines=lost_lines, retained_cpu_ranges=ranges,
                    ring_stats={p.parent.name: read(p) for p in (self.path/"per_cpu").glob("cpu*/stats")},
                    filters=self.filters, enabled=self.enabled,
                    saved_cmdlines=read(self.path/"saved_cmdlines"),
                    saved_tgids=read(self.path/"saved_tgids"),
                    note="Ring overwrite is expected; certify event coverage from retained CPU ranges. "
                         "Exclude observer snapshot/save intervals in later performance claims.")
        return meta

    def close(self):
        if self.path.exists():
            write(self.path/"tracing_on", 0)
            for ev in self.enabled:
                write(self.path/"events"/ev/"enable", 0)
            self.path.rmdir()

def run(args):
    import rospy
    from std_msgs.msg import Float64MultiArray, String
    from wheel_arm_abs_vel.msg import VelocityIkShadow

    if os.geteuid() != 0:
        raise RuntimeError("Run the observer with sudo, without starting any robot node")
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=False)
    # Copy audited binding next to per-run directories, only if provided.
    if args.gil_binding:
        shutil.copyfile(args.gil_binding, out/"gil_binding.json")
    # Observer threads inherit lower CPU priority; robot process settings are never changed.
    os.nice(10)
    from tools.scheduler_master_guard import master_pid
    initial_master_pid = master_pid(os.environ.get("ROS_MASTER_URI", "http://localhost:11311"))
    state = State()
    rospy.init_node("kuavo_scheduler_flight_observer", anonymous=True, disable_signals=True)
    observer_status = rospy.Publisher(PREFIX+"scheduler_observer_v1", String, queue_size=1, latch=True)
    subscriptions = [
        rospy.Subscriber(PREFIX+"reference", VelocityIkShadow, state.reference_update, queue_size=1, tcp_nodelay=True),
        rospy.Subscriber(PREFIX+"mapper_thread_identity_v1", String, lambda m: state.identity(m.data), queue_size=1)]
    for role in ("ik", "mapper"):
        subscriptions.append(rospy.Subscriber(PREFIX+role+"_runtime_probe_v1", Float64MultiArray,
            lambda msg, role=role: state.probe(role, msg.data), queue_size=16, buff_size=2**20, tcp_nodelay=True))
    subscriptions.append(rospy.Subscriber("/quest3/input_timing", Float64MultiArray,
                                         lambda m: state.input(m.data), queue_size=128))
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    global_before = {k: read(TRACE_ROOT/k) for k in ("tracing_on", "current_tracer", "trace_clock", "events/enable")}
    manifest = dict(version=2, boot_id=read("/proc/sys/kernel/random/boot_id"), global_before=global_before,
                    max_wait_seconds=args.wait, max_follow_seconds_per_identity=args.duration,
                    max_total_bytes=args.max_mib*1024**2, strategy="FOLLOW gated ring, event snapshots",
                    start_realtime_ns=time.time_ns(), start_monotonic_ns=time.monotonic_ns(),
                    runs=[], status="WAITING_FOR_FOLLOW", errors=[])
    (out/"manifest.json").write_text(json.dumps(manifest, indent=2))
    observer_status.publish(String(data=json.dumps(dict(version=1, status="WAITING_FOR_FOLLOW", output=str(out), pid=os.getpid()))))
    rospy.loginfo("[sched-observer] WAITING_FOR_FOLLOW: %s (no pre-test wait timeout)", out)
    print("WAITING_FOR_FOLLOW "+str(out), flush=True)
    trace = None
    key = None
    histories = {}
    last_health = 0
    last_save = 0
    total_bytes = 0
    deadline = time.monotonic()+args.wait if args.wait else None
    next_identity_check = 0
    observed_identity = None
    last_identity_stamp = 0
    try:
        with (out/"quest_input_timing.jsonl").open("w") as inputs, (out/"status.jsonl").open("w") as status:
            while not stop.is_set() and not rospy.is_shutdown() and (deadline is None or time.monotonic() < deadline):
                now = time.monotonic_ns()
                with state.lock:
                    next_key = state.key(now)
                    latest = dict(state.latest)
                    identity = state.identities
                    values = list(state.inputs)
                    state.inputs.clear()
                for item in values:
                    line = json.dumps(item)+"\n"
                    inputs.write(line)
                    total_bytes += len(line)
                if next_key != key:
                    if trace:
                        trace.close()
                        trace = None
                    state.set_active(None)
                    key = next_key
                    observed_identity = None
                    if key:
                        try:
                            observed_identity = (process_identity(key[0], "ik"), process_identity(key[2], "mapper"))
                            for pid, tid in ((key[0], key[1]), (key[2], key[3])):
                                if not (Path("/proc")/str(pid)/"task"/str(tid)).is_dir():
                                    raise ValueError("TID does not belong to process")
                        except (OSError, ValueError) as error:
                            status.write(json.dumps(dict(time_ns=now, rejected=str(error)))+"\n")
                            key = None
                            stop.wait(.1)
                            continue
                        # Use CPUs outside both observed threads' allowed masks when possible.
                        # Change only this observer's threads, including rospy receive workers.
                        focus_cpus = set(os.sched_getaffinity(key[1])) | set(os.sched_getaffinity(key[3]))
                        outside = set(range(os.cpu_count())) - focus_cpus
                        if outside:
                            for task in (Path("/proc")/str(os.getpid())/"task").glob("[0-9]*"):
                                try:
                                    os.sched_setaffinity(int(task.name), outside)
                                except ProcessLookupError:
                                    pass
                        identity_key = key + tuple(x["start_ticks"] for x in observed_identity)
                        history = histories.setdefault(identity_key, dict(seconds=0., saved=collections.Counter()))
                        run_dir = out/("run_%03d"%len(manifest["runs"]))
                        run_dir.mkdir()
                        entry = dict(key=key, processes=observed_identity, start_ns=now, path=run_dir.name,
                                     observer_nice=os.getpriority(os.PRIO_PROCESS, 0),
                                     observer_affinity=sorted(os.sched_getaffinity(0)))
                        manifest["runs"].append(entry)
                        if history["seconds"] < args.duration:
                            trace = FlightTrace(run_dir, (key[1], key[3]), key[2])
                            state.set_active(key)
                            rospy.loginfo("[sched-observer] CAPTURING_FOLLOW: %s identity=%s", run_dir, key)
                            print("FOLLOW_CAPTURE "+str(run_dir)+" "+str(key), flush=True)
                        last_tick = time.monotonic()
                    else:
                        print("WAITING_FOR_FOLLOW", flush=True)
                if trace:
                    elapsed = time.monotonic()
                    history["seconds"] += elapsed-last_tick
                    last_tick = elapsed
                    if history["seconds"] >= args.duration:
                        trace.close()
                        trace = None
                        state.set_active(None)
                        entry["stop_reason"] = "follow_duration_complete"
                        print("RUN_COMPLETE waiting for a new session or process", flush=True)
                    elif now >= next_identity_check:
                        try:
                            actual = (process_identity(key[0], "ik"), process_identity(key[2], "mapper"))
                            if actual != observed_identity:
                                raise ValueError("process restart")
                            trace.refresh_mapper_threads()
                        except (OSError, ValueError) as error:
                            entry["stop_reason"] = str(error)
                            trace.close()
                            trace = None
                            state.set_active(None)
                            key = None
                        next_identity_check = now+2_000_000_000
                if trace and now-last_save >= 5_000_000_000:
                    with state.lock:
                        pending = dict(state.pending)
                        state.pending.clear()
                    # At least two windows per category; no one category consumes all slots.
                    candidates = [p for p in pending.values() if history["saved"][p["category"]] < 3
                                  and now-p["end"] < 250_000_000 and p["begin"] >= trace.started_ns]
                    if candidates:
                        event = max(candidates, key=lambda p: p["duration_ns"])
                        category = event["category"]
                        stem = "%s_%02d"%(category, history["saved"][category])
                        meta = trace.snapshot(stem, event)
                        meta["thread_identity"] = identity if identity and identity.get("pid") == key[2] and 0 <= now-identity.get("monotonic_ns", 0) < 4_000_000_000 else None
                        meta["processes"] = observed_identity
                        meta["proc_threads"] = proc_inventory({key[0], key[2]})
                        meta["latest_probes"] = latest
                        (run_dir/(stem+".json")).write_text(json.dumps(meta, indent=2))
                        total_bytes += (run_dir/(stem+".trace")).stat().st_size+(run_dir/(stem+".json")).stat().st_size
                        history["saved"][category] += 1
                        last_save = time.monotonic_ns()
                        meta["observer_busy_until_ns"] = time.monotonic_ns()
                        (run_dir/(stem+".json")).write_text(json.dumps(meta, indent=2))
                        print("SAVED "+str(run_dir/stem), flush=True)
                if now-last_health >= 2_000_000_000:
                    if identity and identity.get("monotonic_ns") != last_identity_stamp:
                        last_identity_stamp = identity.get("monotonic_ns")
                        with (out/"mapper_thread_identities.jsonl").open("a") as identities_file:
                            line = json.dumps(identity)+"\n"
                            identities_file.write(line)
                            total_bytes += len(line)
                        (out/"mapper_threads_latest.json").write_text(json.dumps(identity, indent=2))
                    with state.lock:
                        health = dict(monotonic_ns=now, key=key, tracing=bool(trace),
                                      stats=dict(state.stats), missing=dict(state.missing), errors=state.errors,
                                      input_overflow=state.input_overflow, total_bytes=total_bytes)
                    status.write(json.dumps(health)+"\n")
                    status.flush()
                    inputs.flush()
                    manifest["status"] = ("CAPTURING_FOLLOW" if trace else
                        "FOLLOW_BUDGET_COMPLETE" if key and history["seconds"] >= args.duration else
                        "WAITING_FOR_FOLLOW")
                    manifest["last_health"] = health
                    observer_status.publish(String(data=json.dumps(dict(version=1, status=manifest["status"],
                        output=str(out), pid=os.getpid(), **health))))
                    (out/"manifest.json").write_text(json.dumps(manifest, indent=2))
                    last_health = now
                    try:
                        current_master = master_pid(os.environ.get("ROS_MASTER_URI", "http://localhost:11311"))
                    except (OSError, RuntimeError):
                        current_master = None
                    if current_master != initial_master_pid:
                        manifest["status"] = "MASTER_CHANGED"
                        break
                    if shutil.disk_usage(out).free < 2*1024**3 or total_bytes > args.max_mib*1024**2:
                        manifest["status"] = "DISK_OR_TOTAL_SIZE_LIMIT"
                        break
                stop.wait(.02)
    except BaseException as error:
        manifest["errors"].append(repr(error))
        raise
    finally:
        if trace:
            trace.close()
        for sub in subscriptions:
            sub.unregister()

        manifest.update(end_monotonic_ns=time.monotonic_ns(), end_realtime_ns=time.time_ns(),
                        global_after={k: read(TRACE_ROOT/k) for k in global_before},
                        total_saved_bytes=total_bytes)
        if manifest["status"] not in ("DISK_OR_TOTAL_SIZE_LIMIT", "MASTER_CHANGED"):
            manifest["status"] = "ERROR" if manifest["errors"] else "STOPPED"
        (out/"manifest.json").write_text(json.dumps(manifest, indent=2))
        try:
            observer_status.publish(String(data=json.dumps(dict(version=1, status=manifest["status"], output=str(out), pid=os.getpid(), errors=manifest["errors"]))))
        except Exception:
            pass
        rospy.loginfo("[sched-observer] %s: %s", manifest["status"], out)
        rospy.signal_shutdown("observer finished")
    return 75 if manifest["status"] == "MASTER_CHANGED" else 0

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--output", required=True)
    ap.add_argument("--wait", type=float, default=7200)
    ap.add_argument("--duration", type=float, default=180)
    ap.add_argument("--max-mib", type=int, default=256)
    ap.add_argument("--gil-binding")
    arguments = ap.parse_args()
    if not 0 < arguments.duration <= 300 or not 0 <= arguments.wait <= 7200:
        ap.error("duration <=300 and wait in [0,7200] (0 waits until shutdown) required")
    raise SystemExit(run(arguments))
