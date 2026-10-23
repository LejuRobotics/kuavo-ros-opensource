"""Bounded RPC workers and task-independent wait/recovery behaviour."""
import ast
from pathlib import Path
import socket
import subprocess
import sys
import threading
import time
from types import SimpleNamespace

import pytest

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
from utils import init_notification


@pytest.mark.parametrize("exit_code", [0, 1])
def test_worker_response_is_acknowledgement_only_on_success(monkeypatch, exit_code):
    monkeypatch.setattr(init_notification, "INIT_CALL", "raise SystemExit(%d)" % exit_code)
    assert init_notification.notify_init(lambda: False) is (exit_code == 0)


@pytest.mark.parametrize("cancel", [False, True])
def test_blocked_workers_are_bounded_and_reaped(monkeypatch, cancel):
    monkeypatch.setattr(init_notification, "INIT_CALL", "import time; time.sleep(60)")
    popen = subprocess.Popen
    children = []
    def start(*args, **kwargs):
        child = popen(*args, **kwargs)
        children.append(child)
        return child
    monkeypatch.setattr(init_notification.subprocess, "Popen", start)
    for _ in range(3):
        began = time.monotonic()
        cancelled = lambda: cancel and time.monotonic() - began > .1
        assert not init_notification.notify_init(cancelled, timeout=.3)
        assert time.monotonic() - began < 1.5
        assert all(child.poll() is not None for child in children)


def test_keyboard_interrupt_also_reaps_worker(monkeypatch):
    monkeypatch.setattr(init_notification, "INIT_CALL", "import time; time.sleep(60)")
    popen = subprocess.Popen
    children = []
    def start(*args, **kwargs):
        child = popen(*args, **kwargs)
        children.append(child)
        return child
    def cancel():
        if children: raise KeyboardInterrupt
        return False
    monkeypatch.setattr(init_notification.subprocess, "Popen", start)
    with pytest.raises(KeyboardInterrupt):
        init_notification.notify_init(cancel)
    assert children[0].poll() is not None


@pytest.mark.parametrize("task_id", [1, 2, 3])
def test_missing_peer_then_recovery_keeps_round_and_logs_once(task_id):
    tree = ast.parse((PACKAGE / "scripts/model_entry.py").read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef)
               and n.name == "EvaluationProtocol")
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef)
                  and n.name == "_notify_initialized")
    replies = iter([False, False, False, True])
    logs = []
    ns = dict(rospy=SimpleNamespace(is_shutdown=lambda: False,
                                   loginfo=lambda *a: logs.append(a)),
              notify_init=lambda cancelled: next(replies))
    exec(compile(ast.Module(body=[method], type_ignores=[]), "notify", "exec"), ns)
    state = SimpleNamespace(task_id=task_id, round_seed=30, evaluation_started=False,
                            relaunch=threading.Event(), _init_wait_logged=False, _reset_running=False,
                            reset_handoff=SimpleNamespace(wait_for=lambda *a, **kw: True,
                                                         pending_seed=lambda: 30))
    assert [ns["_notify_initialized"](state) for _ in range(4)] == [False]*3+[True]
    assert state.round_seed == 30 and not state.relaunch.is_set()
    assert len(logs) == 2
    state.relaunch.set()
    assert not ns["_notify_initialized"](state)  # No further callback on reset.


@pytest.mark.parametrize("response", ["success", "reject", "hang", "absent"])
def test_real_ros_rpc(response, monkeypatch):
    pytest.importorskip("rosgraph")
    from utils.model_master import ModelMaster
    with socket.socket() as reservation:
        reservation.bind(("127.0.0.1", 0))
        port = reservation.getsockname()[1]
    monkeypatch.setenv("ROS_MASTER_URI", "http://127.0.0.1:%d" % port)
    with ModelMaster():
        peer = None
        try:
            if response != "absent":
                code = """
import rospy, time
from std_srvs.srv import Trigger, TriggerResponse
rospy.init_node('init_notification_test', disable_signals=True)
def callback(_):
    if %r == 'hang': time.sleep(60)
    return TriggerResponse(success=%r == 'success', message='probe')
service=rospy.Service('/simulator/init', Trigger, callback)
print('READY', flush=True)
rospy.spin()
""" % (response, response)
                peer = subprocess.Popen([sys.executable, "-c", code], stdout=subprocess.PIPE,
                                        text=True, start_new_session=True)
                assert peer.stdout.readline().strip() == "READY"
            began = time.monotonic()
            assert init_notification.notify_init(lambda: False, timeout=1.5) is (response == "success")
            assert time.monotonic() - began < 2.5
        finally:
            if peer is not None:
                peer.terminate()
                peer.wait(timeout=5)
