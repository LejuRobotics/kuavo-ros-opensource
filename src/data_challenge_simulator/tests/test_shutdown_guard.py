"""Process-level regression tests for interrupt-safe teardown."""

from pathlib import Path
import subprocess
import sys
import time

import pytest


PACKAGE_DIR = Path(__file__).resolve().parents[1]
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))

from utils.shutdown_guard import ShutdownGuard, clear_stale_processes


SLEEP_CODE = "import time; time.sleep(60)"


def _process_is_running(pid):
    try:
        status = Path("/proc/{}/status".format(pid)).read_text()
    except OSError:
        return False
    return not any(line.startswith("State:\tZ") for line in status.splitlines())


def _wait_until_stopped(pid, timeout=2.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if not _process_is_running(pid):
            return True
        time.sleep(0.02)
    return not _process_is_running(pid)


def test_guard_refuses_to_signal_its_own_process_group():
    child = subprocess.Popen([sys.executable, "-c", SLEEP_CODE])
    try:
        guard = ShutdownGuard(9, log=lambda _message: None)
        with pytest.raises(ValueError, match="supervisor's own process group"):
            guard.add(child, process_group=True)
    finally:
        child.terminate()
        child.wait(timeout=2.0)


def test_direct_child_is_terminated_without_signalling_parent_group():
    child = subprocess.Popen([sys.executable, "-c", SLEEP_CODE])
    guard = ShutdownGuard(9, log=lambda _message: None)
    guard.add(child)

    guard.sweep(grace=0.1)

    assert child.wait(timeout=2.0) == -15


def test_saved_process_group_kills_descendant_after_leader_exits():
    leader_code = (
        "import subprocess, sys; "
        "child = subprocess.Popen([sys.executable, '-c', {!r}]); "
        "print(child.pid, flush=True)"
    ).format(SLEEP_CODE)
    leader = subprocess.Popen(
        [sys.executable, "-c", leader_code],
        stdout=subprocess.PIPE,
        text=True,
        start_new_session=True,
    )
    guard = ShutdownGuard(9, log=lambda _message: None)
    guard.add(leader, process_group=True)
    descendant_pid = int(leader.stdout.readline().strip())
    leader.wait(timeout=2.0)
    assert _process_is_running(descendant_pid)

    guard.sweep(grace=0.1)

    assert _wait_until_stopped(descendant_pid)


def test_next_run_removes_a_stale_task_process():
    stale = subprocess.Popen([
        sys.executable,
        "-c",
        SLEEP_CODE,
        "task9_orphan.py",
    ])
    try:
        clear_stale_processes(9, grace=0.1, log=lambda _message: None)
        assert stale.wait(timeout=2.0) == -15
    finally:
        if stale.poll() is None:
            stale.kill()
            stale.wait(timeout=2.0)


def test_stale_round_cleanup_does_not_kill_another_supervisor():
    helper = subprocess.Popen([
        sys.executable,
        "-c",
        SLEEP_CODE,
        "helperfunc.py",
    ])
    model = subprocess.Popen([
        sys.executable,
        "-c",
        SLEEP_CODE,
        "model_entry.py",
    ])
    try:
        clear_stale_processes(9, grace=0.1, log=lambda _message: None)
        assert helper.poll() is None
        assert model.poll() is None
    finally:
        for process in (helper, model):
            if process.poll() is None:
                process.terminate()
            process.wait(timeout=2.0)


def test_guard_does_not_kill_unowned_processes_by_name():
    unrelated = subprocess.Popen([
        sys.executable,
        "-c",
        SLEEP_CODE,
        "task9_unrelated.py",
    ])
    try:
        guard = ShutdownGuard(9, log=lambda _message: None)
        guard.sweep(grace=0.1)
        assert unrelated.poll() is None
    finally:
        unrelated.terminate()
        unrelated.wait(timeout=2.0)
