"""Interrupt-safe teardown for the simulator entry points.

Every long-lived child is started in a session of its own.  A terminal Ctrl+C
therefore never reaches it: the entry point has to tear it down from its own
``finally`` blocks.

That teardown has two weaknesses this module closes:

* A second Ctrl+C lands *inside* the teardown and aborts it half way, so
  whatever had not been signalled yet is simply left running.  ``ShutdownGuard``
  turns repeated Ctrl+C into a no-op while the teardown runs, so the teardown
  always finishes.
* A process-group leader can exit before its descendants.  ``ShutdownGuard``
  therefore saves owned PGIDs, snapshots their descendants before shutdown,
  and checks process birth times before signalling individual PIDs.

Name-based cleanup is deliberately limited to stale *round children* before a
new round starts.  A supervisor must never kill another supervisor by name:
the host wrapper serializes entry-point replacement before launching the next
one.
"""

import os
import re
import signal
import subprocess
import time

# The three nodes whose survival is what makes the next round refuse to start.
NODELET_NODES = ("/nodelet_manager", "/nodelet_mujoco", "/nodelet_controller")

# How long to let a signalled process group exit before escalating to SIGKILL.
SWEEP_GRACE = 3.0


def sweep_patterns(task_id):
    """Command-line patterns for stale children of one task round.

    The leading character of each match is bracketed so the pattern cannot
    match the ``ps``/``pkill`` command line that is looking for it.

    Top-level supervisors are intentionally absent.  Killing ``model_entry``
    from a newly started ``helperfunc`` used to make the retiring model's own
    cleanup kill the new helper in return.
    """
    return (
        r"[t]ask{}\w*\.py".format(task_id),
        r"[t]ask_scorer\.py",
        r"[r]osbag\s+(?:record|play)\b",
        r"[r]oslaunch\s+data_challenge_simulator",
        r"[s]ource_bin_latch_v2\.py",
        r"[r]osbag_compat_publisher\.py",
        r"[t]opic_repeater\.py",
        r"[n]odelet_manager",
        r"[n]odelet_mujoco",
        r"[n]odelet_controller",
    )


def _ppid_of(pid):
    """Return the parent pid of ``pid``, or None if it cannot be read."""
    try:
        with open("/proc/{}/stat".format(pid), "rb") as stream:
            data = stream.read()
    except OSError:
        return None
    # comm can contain spaces and parentheses, so split after the *last* ')'.
    # What follows is "state ppid ...".
    try:
        fields = data[data.rindex(b")") + 2:].split()
        return int(fields[1])
    except (ValueError, IndexError):
        return None


def _protected_pids():
    """This process and its ancestors -- never signal these.

    Ancestors rather than the whole process group: a task script started
    without ``setsid`` shares this process group, and it is exactly the kind of
    orphan the sweep is meant to catch.
    """
    protected = set()
    pid = os.getpid()
    while pid and pid not in protected:
        protected.add(pid)
        pid = _ppid_of(pid)
    return protected


def _iter_processes():
    """Yield ``(pid, cmdline)`` for every readable process."""
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        try:
            with open("/proc/{}/cmdline".format(entry), "rb") as stream:
                raw = stream.read()
        except OSError:
            continue
        cmdline = raw.replace(b"\0", b" ").decode("utf-8", "replace").strip()
        if not cmdline:
            continue  # kernel thread, or a zombie with nothing left to kill
        yield int(entry), cmdline


def _signal_matching_processes(task_id, signum, log):
    patterns = [re.compile(pattern) for pattern in sweep_patterns(task_id)]
    protected = _protected_pids()
    victims = []
    for pid, cmdline in _iter_processes():
        if pid in protected:
            continue
        if any(pattern.search(cmdline) for pattern in patterns):
            victims.append((pid, cmdline))
    for pid, cmdline in victims:
        try:
            os.kill(pid, signum)
        except OSError:
            continue
        log("[INFO] 已结束进程 {}: {}".format(pid, cmdline[:120]))
    return victims


def clear_stale_processes(task_id, grace=SWEEP_GRACE, log=print):
    """Remove processes from an earlier run before starting a new one."""
    found = _signal_matching_processes(task_id, signal.SIGTERM, log)
    if found:
        time.sleep(grace)
        _signal_matching_processes(task_id, signal.SIGKILL, log)



def clear_dead_simulator_nodes(log=print):
    """Remove only known simulator registrations whose local API has closed."""
    import rosgraph
    import rosnode
    from xmlrpc.client import ServerProxy, Transport, Error

    class ProbeTransport(Transport):
        def make_connection(self, host):
            connection = super().make_connection(host)
            connection.timeout = 1.0
            return connection

    master = rosgraph.Master('/simulator_cleanup')
    for name in NODELET_NODES:
        try:
            uri = master.lookupNode(name)
            try:
                with ServerProxy(uri, transport=ProbeTransport()) as api:
                    api.getPid('/simulator_cleanup')
            except ConnectionRefusedError:
                # Never erase a replacement node or a merely slow live node.
                if master.lookupNode(name) == uri:
                    rosnode.cleanup_master_blacklist(master, [name])
                    log("[INFO] 已移除退出节点的失效注册: {}".format(name))
        except (OSError, Error, rosgraph.MasterError, rosgraph.MasterFailure):
            continue

def clear_simulator_nodes(timeout=10.0, log=print):
    """Kill simulator nodes left over from an interrupted round, then return.

    An interrupted round leaves its nodelets registered with the master.  They
    used to make the next round refuse to start; clearing them here means the
    next round always gets a clean graph.  Returns quietly once the nodes are
    gone, or once ``timeout`` has passed.
    """
    import rosnode

    deadline = time.time() + timeout
    while time.time() < deadline:
        clear_dead_simulator_nodes(log=log)
        try:
            active = set(rosnode.get_node_names())
        except Exception:
            return
        conflicts = sorted(active.intersection(NODELET_NODES))
        if not conflicts:
            return
        log("[INFO] 清除上一轮遗留的仿真进程: {}".format(", ".join(conflicts)))
        subprocess.call(
            ["rosnode", "kill"] + conflicts,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(1.0)


class ShutdownGuard(object):
    """Reap owned process trees on normal exit and on INT/TERM/HUP."""

    SIGNALS = (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)

    def __init__(self, task_id, log=print):
        self.task_id = task_id
        self.log = log
        self._children = []
        self._process_groups = {}
        self._graceful = set()
        self._tracked = {}
        self._interrupts = 0
        self._cleaning = False
        self._swept = False
        self._previous_handlers = {}

    def __enter__(self):
        self._previous_handlers = {
            sig: signal.signal(sig, self._on_interrupt) for sig in self.SIGNALS}
        return self

    def __exit__(self, *_exc):
        try:
            self.sweep()
        finally:
            for sig, handler in self._previous_handlers.items():
                signal.signal(sig, handler)
        return False

    def _on_interrupt(self, signum, _frame):
        if self._cleaning or self._interrupts:
            self.log("[INFO] 正在清理本次资源，忽略重复退出信号。")
            return
        self._interrupts += 1
        self.log("[INFO] 收到退出信号 {}，开始清理。".format(signum))
        raise KeyboardInterrupt

    def add(self, process, graceful=False, process_group=False):
        if process is not None:
            if process_group:
                try:
                    pgid = os.getpgid(process.pid)
                except ProcessLookupError:
                    pgid = None
                if pgid == os.getpgrp():
                    raise ValueError(
                        "refusing to manage the supervisor's own process group")
                self._process_groups[process] = process.pid
            self._children.append(process)
            if graceful:
                self._graceful.add(process)
            self._swept = False
        return process

    @staticmethod
    def _process_snapshot():
        result = {}
        for name in os.listdir("/proc"):
            if not name.isdigit():
                continue
            try:
                with open("/proc/{}/stat".format(name), "rb") as stream:
                    raw = stream.read()
                fields = raw[raw.rindex(b")") + 2:].split()
                # State, PPID, PGID, start time; birth time prevents PID reuse.
                if fields[0] != b"Z":
                    result[int(name)] = (
                        int(fields[1]), int(fields[2]), int(fields[19]))
            except (OSError, ValueError, IndexError):
                pass
        return result

    def _alive_children(self):
        """Snapshot descendants before signalling parents, including setsid children."""
        snapshot = self._process_snapshot()
        protected = _protected_pids()
        owners = {
            pid: owner for pid, (birth, owner) in self._tracked.items()
            if pid in snapshot and snapshot[pid][2] == birth}
        groups = {pgid: process for process, pgid in self._process_groups.items()}
        for process in self._children:
            # poll() also reaps exited direct children.
            if process.poll() is None and process.pid in snapshot:
                owners[process.pid] = process
        for pid, (_, pgid, _) in snapshot.items():
            if pgid in groups and pid not in protected:
                owners.setdefault(pid, groups[pgid])
        while True:
            found = {pid: owners[ppid] for pid, (ppid, _, _) in snapshot.items()
                     if ppid in owners and pid not in owners and pid not in protected}
            if not found:
                break
            owners.update(found)
        for pid, owner in owners.items():
            self._tracked[pid] = (snapshot[pid][2], owner)
        return owners

    def _signal_children(self, signum, graceful_only=False, roots_only=False):
        for pid, owner in self._alive_children().items():
            if graceful_only and owner not in self._graceful:
                continue
            if roots_only and pid != owner.pid:
                continue
            # Check again immediately before signalling a saved PID.
            try:
                with open("/proc/{}/stat".format(pid), "rb") as stream:
                    raw = stream.read()
                birth = int(raw[raw.rindex(b")") + 2:].split()[19])
                if birth == self._tracked[pid][0]:
                    os.kill(pid, signum)
            except (OSError, ValueError, IndexError):
                pass

    def _wait_for_children(self, seconds, graceful_only=False):
        deadline = time.monotonic() + seconds
        while True:
            alive = self._alive_children()
            if graceful_only:
                alive = {pid: owner for pid, owner in alive.items()
                         if owner in self._graceful}
            if not alive or time.monotonic() >= deadline:
                return alive
            time.sleep(0.1)

    def sweep(self, grace=10.0):
        if self._swept:
            return
        if not self._children:
            self._swept = True
            return
        self._cleaning = True
        handlers = {sig: signal.signal(sig, self._on_interrupt) for sig in self.SIGNALS}
        try:
            self.log("[INFO] 清理本次资源：先完成录包，再关闭仿真与辅助进程。")
            self._alive_children()
            if self._graceful:
                self._signal_children(signal.SIGINT, graceful_only=True, roots_only=True)
                self._wait_for_children(grace, graceful_only=True)
            # Let roslaunch send ROS-aware shutdown signals and unregister
            # nodes before escalating to descendants that failed to follow it.
            self._signal_children(signal.SIGTERM, roots_only=True)
            remaining = self._wait_for_children(grace)
            if remaining:
                self._signal_children(signal.SIGTERM)
                remaining = self._wait_for_children(2.0)
            if remaining:
                self.log("[WARNING] 退出超时，强制回收本次进程：{}".format(sorted(remaining)))
                self._signal_children(signal.SIGKILL)
                remaining = self._wait_for_children(2.0)
            if remaining:
                raise RuntimeError("本次资源清理未完成，残留 PID：{}".format(sorted(remaining)))
            self._children.clear()
            self._process_groups.clear()
            self._graceful.clear()
            self._tracked.clear()
            self._swept = True
            self.log("[INFO] 本次资源清理完成：已核验进程树无运行残留。")
        finally:
            self._cleaning = False
            for sig, handler in handlers.items():
                signal.signal(sig, handler)
