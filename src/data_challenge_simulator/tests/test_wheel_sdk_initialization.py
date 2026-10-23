"""Wheel SDK startup must check wheel capabilities and retain real failures."""
from types import SimpleNamespace

import pytest

from kuavo_humanoid_sdk.kuavo.core.ros import control, param


@pytest.mark.parametrize("robot_type", [1, "1", 2])
def test_missing_standing_height_is_only_an_error_for_bipeds(monkeypatch, robot_type):
    errors = []
    monkeypatch.setattr(param.rospy, "has_param", lambda _: False)
    monkeypatch.setattr(param.rospy, "get_param", lambda name, default=None: robot_type)
    monkeypatch.setattr(param.rospy, "logerr", errors.append)
    height = param.RosParameter().init_stand_height()
    if robot_type in (1, "1"):
        assert height == pytest.approx(0.8328437523948975)
        assert errors == []
    else:
        assert height == pytest.approx(0.8328437523948975)
        assert errors == ["com_height parameter not found"]


def test_explicit_standing_height_is_preserved(monkeypatch):
    monkeypatch.setattr(param.rospy, "has_param", lambda _: True)
    monkeypatch.setattr(param.rospy, "get_param", lambda _: 0.75)
    assert param.RosParameter().init_stand_height() == 0.75


@pytest.mark.parametrize("wheel,connected", [(True, True), (True, False), (False, True)])
def test_motion_connection_checks_match_robot_type(monkeypatch, wheel, connected):
    errors, checked = [], []

    class Publisher:
        def __init__(self, name):
            self.name = name

        def get_num_connections(self):
            checked.append(self.name)
            return int(connected)

    class Stamp:
        def __sub__(self, _):
            return SimpleNamespace(to_sec=lambda: 2.0)

    monkeypatch.setattr(control, "rospy", SimpleNamespace(
        Time=SimpleNamespace(now=Stamp)))
    monkeypatch.setattr(control, "kuavo_ros_param", SimpleNamespace(
        is_wheel_arm_robot=lambda: wheel))
    monkeypatch.setattr(control.SDKLogger, "error", errors.append)
    motion = control.ControlRobotMotion.__new__(control.ControlRobotMotion)
    names = ("cmd_vel", "cmd_pose", "step_ctrl", "switch_gait", "cmd_pose_world")
    for name in names:
        setattr(motion, "_pub_" + name, Publisher(name))

    assert motion.connect(timeout=1.0) == connected
    assert checked == (["cmd_vel"] if wheel else list(names))
    if connected:
        assert errors == []
    else:
        assert len(errors) == 1 and "cmd_vel" in errors[0]


def test_optional_arm_interface_is_reported_without_failing_startup(monkeypatch):
    errors, warnings = [], []

    class Stamp:
        def __sub__(self, _):
            return SimpleNamespace(to_sec=lambda: 2.0)

    monkeypatch.setattr(control, "rospy", SimpleNamespace(Time=SimpleNamespace(now=Stamp)))
    monkeypatch.setattr(control.SDKLogger, "error", errors.append)
    monkeypatch.setattr(control.SDKLogger, "warning", warnings.append)
    arm = control.ControlRobotArm.__new__(control.ControlRobotArm)
    arm._pub_ctrl_arm_traj = SimpleNamespace(
        name="/kuavo_arm_traj", get_num_connections=lambda: 1)
    arm._pub_ctrl_arm_target_poses = SimpleNamespace(
        name="/kuavo_arm_target_poses", get_num_connections=lambda: 0)
    assert arm.connect()
    assert errors == []
    assert len(warnings) == 1 and "/kuavo_arm_target_poses" in warnings[0]


@pytest.mark.parametrize("controller_ready", [False, True])
def test_task2_waits_for_controller_before_sdk(monkeypatch, tmp_path, controller_ready):
    import importlib.util
    from pathlib import Path

    examples = Path(__file__).resolve().parents[1] / "examples"
    monkeypatch.syspath_prepend(str(examples))
    spec = importlib.util.spec_from_file_location("task2_startup", examples / "task2.py")
    task = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(task)
    calls = []

    def wait(topic, message_type, timeout):
        calls.append("controller")
        assert topic == "/mobile_manipulator_wbc_observation/state"
        if not controller_ready:
            raise RuntimeError("controller unavailable")

    def initialize(**kwargs):
        calls.append("sdk")
        raise RuntimeError("stop after initialization barrier")

    monkeypatch.setattr(task, "RESULT_PATH", tmp_path / "result")
    monkeypatch.setattr(task.rospy, "init_node", lambda *a, **k: None)
    monkeypatch.setattr(task.rospy, "wait_for_message", wait)
    monkeypatch.setattr(task.KuavoSDK, "Init", staticmethod(initialize))
    with pytest.raises(RuntimeError):
        task.main()
    assert calls == (["controller", "sdk"] if controller_ready else ["controller"])
