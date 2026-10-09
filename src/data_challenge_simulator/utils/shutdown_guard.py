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
  therefore saves every owned PGID when the child is registered and signals
  that saved group during teardown.

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
    """Make Ctrl+C teardown finish and stop only owned children.

    Use it as a context manager around the whole run and register every child
    process group with :meth:`add`.  The first Ctrl+C raises
    ``KeyboardInterrupt`` as usual, so the entry point's own ``finally`` blocks
    still run their graceful shutdown; any further Ctrl+C is ignored so that
    teardown cannot be interrupted half way.  :meth:`sweep` signals only the
    child PIDs and process groups registered by this guard.
    """

    def __init__(self, task_id, log=print):
        self.task_id = task_id
        self.log = log
        self._children = []
        self._process_groups = {}
        self._graceful = set()
        self._interrupts = 0
        self._swept = False
        self._previous_handler = None

    def __enter__(self):
        self._previous_handler = signal.getsignal(signal.SIGINT)
        signal.signal(signal.SIGINT, self._on_interrupt)
        return self

    def __exit__(self, exc_type, _exc_value, _traceback):
        # An interrupt that lands *before* the run reaches its own teardown --
        # during topic waits, or between rounds -- unwinds straight through
        # here, with the simulator still up.  Sweep on the way out so no path
        # out of the run leaves a live process group behind.
        if exc_type is not None and not self._swept:
            self.sweep()
        signal.signal(signal.SIGINT, self._previous_handler)
        return False

    def _on_interrupt(self, _signum, _frame):
        self._interrupts += 1
        if self._interrupts == 1:
            raise KeyboardInterrupt
        self.log(
            "[INFO] 正在退出，忽略后续 Ctrl+C。"
            "若长时间没有反应，用 Ctrl+\\ 强制结束。")

    def add(self, process, graceful=False, process_group=False):
        """Register a child for :meth:`sweep` to signal.

        ``graceful`` sends SIGINT first, which is how rosbag finishes writing
        its file; plain SIGTERM would leave the bag unfinalized.

        Set ``process_group`` only for a process created with
        ``start_new_session=True``.  Its PGID is captured now, while the group
        leader is known to exist, so descendants can still be terminated if
        the leader exits before teardown.  A child which shares our process
        group is rejected: signalling that group would kill the supervisor
        that is responsible for completing teardown.
        """
        if process is not None:
            self._children.append(process)
            if graceful:
                self._graceful.add(process)
            if process_group:
                # start_new_session=True makes the child a session and process
                # group leader, so PGID == PID by contract.  Do not look it up
                # to derive the stored value: a short-lived child may exit
                # before add() runs, but its descendants can still retain this
                # process group.  A best-effort lookup is only a misuse check.
                pgid = process.pid
                try:
                    actual_pgid = os.getpgid(process.pid)
                except ProcessLookupError:
                    actual_pgid = None
                if actual_pgid == os.getpgrp():
                    raise ValueError(
                        "refusing to manage the supervisor's own process group")
                self._process_groups[process] = pgid
        return process

    def sweep(self, grace=SWEEP_GRACE):
        """Signal everything this guard owns.

        Registered independent groups are signalled directly.  Ordinary child
        processes are signalled by PID, so the supervisor's own process group
        is never a teardown target.  Stale processes from an older run are the
        host wrapper's responsibility and must not be guessed by name here.
        """
        self.log("[INFO] 清理本轮遗留的进程 ...")
        self._swept = True
        self._signal_children(signal.SIGTERM, graceful_signal=signal.SIGINT)
        self._wait_for_children(grace)
        self._signal_children(signal.SIGKILL)

    # -- internals -------------------------------------------------------

    def _signal_children(self, signum, graceful_signal=None):
        for process in self._children:
            if graceful_signal is not None and process in self._graceful:
                signum_for_process = graceful_signal
            else:
                signum_for_process = signum
            pgid = self._process_groups.get(process)
            if pgid is not None:
                # Do not skip a group merely because its leader exited.  The
                # descendants retain the original PGID and are the processes
                # teardown most needs to catch.
                try:
                    os.killpg(pgid, signum_for_process)
                except OSError:
                    pass
            elif process.poll() is None:
                try:
                    process.send_signal(signum_for_process)
                except OSError:
                    pass

    @staticmethod
    def _group_is_alive(pgid):
        try:
            os.killpg(pgid, 0)
        except ProcessLookupError:
            return False
        except PermissionError:
            return True
        return True

    def _child_is_alive(self, process):
        # poll() also reaps a direct child which has exited.
        process_alive = process.poll() is None
        pgid = self._process_groups.get(process)
        if pgid is None:
            return process_alive
        return self._group_is_alive(pgid)

    def _wait_for_children(self, seconds):
        deadline = time.time() + seconds
        while time.time() < deadline:
            if not any(self._child_is_alive(process)
                       for process in self._children):
                return
            time.sleep(0.1)
