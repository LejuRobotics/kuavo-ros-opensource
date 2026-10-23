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


def test_normal_exit_and_guard_reuse():
    guard = ShutdownGuard(9, log=lambda _: None)
    for _ in range(2):
        with guard:
            child = guard.add(subprocess.Popen(
                [sys.executable, "-c", SLEEP_CODE], start_new_session=True),
                process_group=True)
        assert child.poll() is not None


def test_detached_descendant_is_reaped():
    code = (
        "import subprocess, sys, time; "
        "p=subprocess.Popen([sys.executable, '-c', %r], start_new_session=True); "
        "print(p.pid, flush=True); time.sleep(60)" % SLEEP_CODE)
    with ShutdownGuard(9) as guard:
        leader = guard.add(subprocess.Popen(
            [sys.executable, "-c", code], stdout=subprocess.PIPE,
            text=True, start_new_session=True), process_group=True)
        descendant = int(leader.stdout.readline())
    assert leader.poll() is not None
    assert _wait_until_stopped(descendant)


@pytest.mark.parametrize("signum", [2, 15, 1])
def test_exit_signals_run_finally_and_clean_children(signum, tmp_path):
    import os
    ready = tmp_path / "ready"
    done = tmp_path / "done"
    code = """
import sys, time, subprocess
from pathlib import Path
sys.path.insert(0, %r)
from utils.shutdown_guard import ShutdownGuard
try:
    with ShutdownGuard(9):
        from utils.task2_randomization import task_scene
        with task_scene(1, 28) as scene, ShutdownGuard(9) as guard:
            child=guard.add(subprocess.Popen(
                [sys.executable, '-c', 'import time; time.sleep(60)'],
                start_new_session=True), process_group=True)
            Path(%r).write_text(str(child.pid)+'\\n'+scene)
            time.sleep(60)
except KeyboardInterrupt:
    Path(%r).write_text('clean')
""" % (str(PACKAGE_DIR), str(ready), str(done))
    parent = subprocess.Popen([sys.executable, "-c", code], start_new_session=True,
                              stdout=subprocess.DEVNULL)
    try:
        deadline = time.monotonic() + 5
        while not ready.exists() and time.monotonic() < deadline:
            time.sleep(.02)
        assert ready.exists()
        pid, scene = ready.read_text().splitlines()
        os.kill(parent.pid, signum)
        assert parent.wait(timeout=15) == 0
        assert done.exists()
        assert not Path(scene).exists()
        assert _wait_until_stopped(int(pid))
    finally:
        if parent.poll() is None:
            parent.kill()
            parent.wait()


def test_recording_finishes_before_simulator_stops(tmp_path):
    marker = tmp_path / "bag-closed"
    result = tmp_path / "sim-stopped"
    recorder_code = """
import signal, time
from pathlib import Path
def stop(*_):
    time.sleep(.25)
    Path(%r).write_text('closed')
    raise SystemExit
signal.signal(signal.SIGINT, stop)
print('ready', flush=True)
while True: time.sleep(.1)
""" % str(marker)
    sim_code = """
import signal, time
from pathlib import Path
def stop(*_):
    Path(%r).write_text(str(Path(%r).exists()))
    raise SystemExit
signal.signal(signal.SIGTERM, stop)
print('ready', flush=True)
while True: time.sleep(.1)
""" % (str(result), str(marker))
    with ShutdownGuard(9) as guard:
        for code, graceful in [(recorder_code, True), (sim_code, False)]:
            proc = guard.add(subprocess.Popen(
                [sys.executable, '-c', code], stdout=subprocess.PIPE,
                text=True, start_new_session=True),
                graceful=graceful, process_group=True)
            assert proc.stdout.readline().strip() == "ready"
    assert result.read_text() == "True"


def test_stubborn_child_is_killed_and_waited():
    code = ("import signal, time; signal.signal(signal.SIGTERM, signal.SIG_IGN); "
            "print('ready', flush=True); time.sleep(60)")
    guard = ShutdownGuard(9)
    child = guard.add(subprocess.Popen(
        [sys.executable, "-c", code], stdout=subprocess.PIPE,
        text=True, start_new_session=True), process_group=True)
    assert child.stdout.readline().strip() == "ready"
    guard.sweep(grace=.1)
    assert child.poll() == -9


def test_parent_can_unregister_children_before_descendant_fallback(tmp_path):
    marker = tmp_path / "child-exit-signal"
    child_code = """
import signal, time
from pathlib import Path
def stop(sig, _):
    Path(%r).write_text(str(sig))
    raise SystemExit
signal.signal(signal.SIGINT, stop)
signal.signal(signal.SIGTERM, stop)
print('ready', flush=True)
while True: time.sleep(.1)
""" % str(marker)
    parent_code = """
import signal, subprocess, sys, time
p=subprocess.Popen([sys.executable, '-c', %r], stdout=subprocess.PIPE,
                   text=True, start_new_session=True)
p.stdout.readline()
def stop(*_):
    time.sleep(.1)
    p.send_signal(signal.SIGINT)
    p.wait()
    raise SystemExit
signal.signal(signal.SIGTERM, stop)
print('ready', flush=True)
while True: time.sleep(.1)
""" % child_code
    with ShutdownGuard(9) as guard:
        parent = guard.add(subprocess.Popen(
            [sys.executable, '-c', parent_code], stdout=subprocess.PIPE,
            text=True, start_new_session=True), process_group=True)
        assert parent.stdout.readline().strip() == "ready"
    assert marker.read_text() == "2"


def test_repeated_signal_during_cleanup_does_not_abort_it(tmp_path):
    import os
    ready = tmp_path / 'cleaning'
    code = """
import signal, subprocess, sys, time
from pathlib import Path
sys.path.insert(0, %r)
from utils.shutdown_guard import ShutdownGuard
def log(message):
    if '先完成录包' in message: Path(%r).write_text('cleaning')
child_code = "import signal,time; signal.signal(signal.SIGTERM, lambda *a: (time.sleep(.5), exit(0))); print('ready',flush=True); time.sleep(60)"
with ShutdownGuard(9, log=log) as guard:
    child=guard.add(subprocess.Popen([sys.executable,'-c',child_code],
        stdout=subprocess.PIPE, text=True, start_new_session=True),process_group=True)
    child.stdout.readline()
print(child.poll(), flush=True)
""" % (str(PACKAGE_DIR), str(ready))
    parent = subprocess.Popen([sys.executable, '-c', code], stdout=subprocess.PIPE,
                              text=True, start_new_session=True)
    try:
        deadline=time.monotonic()+5
        while not ready.exists() and time.monotonic()<deadline: time.sleep(.01)
        assert ready.exists()
        for sig in (2,15,1): os.kill(parent.pid,sig)
        assert parent.communicate(timeout=10)[0].strip() == "0"
        assert parent.returncode == 0
    finally:
        if parent.poll() is None:
            parent.kill()
            parent.wait()


@pytest.mark.parametrize("state,expected", [
    ("refused", True), ("timeout", False), ("live", False), ("replacement", False)])
def test_only_dead_unchanged_node_registration_is_removed(monkeypatch, state, expected):
    import rosgraph
    import rosnode
    import xmlrpc.client
    from utils.shutdown_guard import clear_dead_simulator_nodes
    removed = []
    calls = []
    class Master:
        def lookupNode(self, name):
            if name != "/nodelet_manager":
                raise rosgraph.MasterError("unknown node")
            calls.append(name)
            if state == "replacement" and len(calls) > 1:
                return "http://localhost:2222/"
            return "http://localhost:1111/"
    class Proxy:
        def __enter__(self): return self
        def __exit__(self, *args): pass
        def getPid(self, caller):
            if state in ("refused", "replacement"):
                raise ConnectionRefusedError()
            if state == "timeout":
                raise TimeoutError()
            return [1, "", 123]
    monkeypatch.setattr(rosgraph, "Master", lambda _: Master())
    monkeypatch.setattr(xmlrpc.client, "ServerProxy", lambda *a, **kw: Proxy())
    monkeypatch.setattr(rosnode, "cleanup_master_blacklist",
                        lambda master, nodes: removed.extend(nodes))
    clear_dead_simulator_nodes(log=lambda _: None)
    assert removed == (["/nodelet_manager"] if expected else [])
