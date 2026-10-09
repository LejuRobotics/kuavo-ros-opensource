"""Real ROS master lifecycle checks on a private port; no simulator cleanup."""

import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
from xmlrpc.client import ServerProxy

import pytest

pytest.importorskip('rosgraph')
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from utils.model_master import ModelMaster


@pytest.fixture
def master_uri(monkeypatch):
    with socket.socket() as reservation:
        reservation.bind(('127.0.0.1', 0))
        port = reservation.getsockname()[1]
    uri = 'http://127.0.0.1:{}'.format(port)
    monkeypatch.setenv('ROS_MASTER_URI', uri)
    return uri


def test_master_survives_two_round_launches_and_borrower_exit(master_uri, tmp_path):
    launch = tmp_path / 'round.launch'
    launch.write_text('<launch><param name="round_probe" value="ready"/></launch>')
    owner = ModelMaster()
    with owner:
        pid = owner.pid
        for _ in range(2):
            with ModelMaster() as borrower:
                assert borrower.process is None
                assert borrower.pid == pid
                with ServerProxy(master_uri) as api:
                    api.deleteParam('/test', '/round_probe')
                child = subprocess.Popen(
                    ['roslaunch', str(launch)], start_new_session=True,
                    stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
                try:
                    deadline = time.monotonic() + 10
                    while time.monotonic() < deadline:
                        with ServerProxy(master_uri) as api:
                            if api.getParam('/test', '/round_probe')[0] == 1:
                                break
                        time.sleep(0.05)
                    else:
                        pytest.fail('round roslaunch did not reach the session master')
                finally:
                    if child.poll() is None:
                        os.killpg(child.pid, signal.SIGINT)
                    child.wait(timeout=10)
                owner.check()
            owner.check()
    assert owner._probe() is None


def test_exception_cleans_up_owned_master(master_uri):
    owner = ModelMaster()
    with pytest.raises(RuntimeError, match='test failure'):
        with owner:
            raise RuntimeError('test failure')
    assert owner._probe() is None


def test_unavailable_remote_master_is_not_replaced(master_uri, monkeypatch):
    monkeypatch.setattr('utils.model_master.is_local_address', lambda host: False)
    owner = ModelMaster()
    with pytest.raises(RuntimeError, match='remote master'):
        with owner:
            pass
    assert owner.process is None


def test_master_loss_requires_session_restart(master_uri):
    with ModelMaster() as owner:
        with ModelMaster() as borrower:
            owner.close()
            with pytest.raises(RuntimeError, match='restart the evaluation session'):
                borrower.check()


def test_startup_timeout_cleans_up_child(master_uri, monkeypatch):
    original_popen = subprocess.Popen
    children = []

    def stalled_roscore(*args, **kwargs):
        child = original_popen(
            [sys.executable, '-c', 'import time; time.sleep(60)'],
            start_new_session=True, stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL)
        children.append(child)
        return child

    monkeypatch.setattr('utils.model_master.subprocess.Popen', stalled_roscore)
    owner = ModelMaster(timeout=0.2)
    with pytest.raises(RuntimeError, match='timed out starting'):
        with owner:
            pass
    assert len(children) == 1
    assert children[0].poll() is not None
    assert owner.process is None
