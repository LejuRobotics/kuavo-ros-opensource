import sys
from types import SimpleNamespace
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
if str(PACKAGE) not in sys.path:
    sys.path.insert(0, str(PACKAGE))

import pytest
import rospy

from utils import arm_state


def _message(joints):
    return SimpleNamespace(
        joint_data=SimpleNamespace(joint_q=list(joints)))


def test_waits_for_message_and_extracts_wheel_arm_joint_slice(monkeypatch):
    joints = [0.1 * index for index in range(28)]
    calls = []

    def wait_for_message(topic, message_type, timeout):
        calls.append((topic, message_type, timeout))
        return _message(joints)

    monkeypatch.setattr(rospy, "wait_for_message", wait_for_message)

    assert arm_state.wait_for_first_arm_state(timeout=2.5) == joints[4:18]
    assert calls == [(
        "/sensors_data_raw", arm_state.sensorsData, 2.5)]


def test_does_not_accept_the_sdk_cache_when_no_message_arrives(monkeypatch):
    def timeout(*_args, **_kwargs):
        raise rospy.ROSException("no message")

    monkeypatch.setattr(rospy, "wait_for_message", timeout)

    with pytest.raises(RuntimeError, match="first real arm state"):
        arm_state.wait_for_first_arm_state(timeout=0.1)


def test_rejects_short_or_non_finite_sensor_messages(monkeypatch):
    monkeypatch.setattr(
        rospy, "wait_for_message",
        lambda *_args, **_kwargs: _message([0.0] * 17))
    with pytest.raises(RuntimeError, match="expected at least 18"):
        arm_state.wait_for_first_arm_state()

    joints = [0.0] * 28
    joints[11] = float("nan")
    monkeypatch.setattr(
        rospy, "wait_for_message",
        lambda *_args, **_kwargs: _message(joints))
    with pytest.raises(RuntimeError, match="non-finite"):
        arm_state.wait_for_first_arm_state()
