"""State-driven transitions must not advance on old or unfinished motion."""
import ast
import threading
from pathlib import Path
from types import SimpleNamespace
import math
import numpy as np
import pytest

PACKAGE = Path(__file__).resolve().parents[1]


def load_function(filename, name, namespace, class_name=None):
    tree = ast.parse((PACKAGE / "examples" / filename).read_text())
    nodes = tree.body
    if class_name:
        nodes = next(node.body for node in nodes if isinstance(node, ast.ClassDef)
                     and node.name == class_name)
    node = next(node for node in nodes if isinstance(node, ast.FunctionDef)
                and node.name == name)
    exec(compile(ast.Module(body=[node], type_ignores=[]), filename, "exec"), namespace)
    return namespace[name]


def test_hands_require_new_settled_samples_and_fail_on_missing_state():
    clock = [0.0]
    samples = iter([0.1, 0, 0.1, 0, 0, 0, 0])
    value = [0.1]
    def sample(topic, msg_type, timeout):
        if "/l_hand" in topic:
            value[0] = next(samples)
        return SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(
            x=value[0], y=0, z=0)))
    def sleep(dt):
        clock[0] += dt
    ros = SimpleNamespace(is_shutdown=lambda: False, sleep=sleep,
        loginfo=lambda *args: None, wait_for_message=sample)
    ns = dict(time=SimpleNamespace(monotonic=lambda: clock[0]), rospy=ros,
              np=np, PoseStamped=object)
    wait = load_function("task2_pick.py", "wait_for_hand_targets", ns)
    wait(np.zeros((2,3)))
    assert clock[0] == pytest.approx(0.12)
    samples = iter([0.1]*100)
    with pytest.raises(RuntimeError, match="did not settle"):
        wait(np.zeros((2,3)), timeout=0.1)
    def unavailable(*args, **kwargs):
        raise RuntimeError("no fresh state")
    ros.wait_for_message = unavailable
    with pytest.raises(RuntimeError, match="no fresh state"):
        wait(np.zeros((2,3)))


def test_skip_opening_only_when_both_hands_are_already_open():
    ns = dict(math=math, box_gripper_targets=lambda side, value: [0.0]*11)
    opened = load_function("task2_pick.py", "box_grippers_are_open", ns)
    state = {"left": [0.0]*11, "right": [0.0]*11}
    gripper = SimpleNamespace(measured_positions=lambda side: state[side])
    assert opened(gripper, 0)
    for bad in (None, [], [0.5]*11, [float("nan")]*11):
        state["right"] = bad
        assert not opened(gripper, 0)


@pytest.mark.parametrize("fresh,moving,enabled,early", [
    (True, False, True, True), (False, False, True, False),
    (True, True, True, False), (True, False, False, False)])
def test_chassis_early_settle_requires_new_stopped_measurements(fresh, moving, enabled, early):
    clock = [0.0]
    motion = SimpleNamespace(_lock=threading.Lock(), _state_sequence=1,
        _velocity=(0, 0, 0, 0, 0, 0), settle_on_velocity=enabled,
        publish_rate=100, publish=lambda: None)
    def tick():
        clock[0] += 0.01
        if fresh:
            motion._state_sequence += 1
            motion._velocity = (0.1 if moving else 0, 0, 0, 0, 0, 0)
    ns = dict(math=math, time=SimpleNamespace(monotonic=lambda: clock[0]),
        rospy=SimpleNamespace(is_shutdown=lambda: False,
            Rate=lambda rate: SimpleNamespace(sleep=tick), loginfo=lambda *args: None))
    stop = load_function("task2_base_motion.py", "stop", ns, "ChassisMotion")
    stop(motion)
    assert (clock[0] < 0.6) == early
    assert clock[0] >= 0.1
