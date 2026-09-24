"""Behavioral tests for model-only chassis initialization and handoff."""

import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace


PACKAGE = Path(__file__).resolve().parents[1]
MODULE_PATH = PACKAGE / "utils/model_base_motion.py"


class FakeEndpoint(object):
    def __init__(self):
        self.unregistered = False

    def unregister(self):
        self.unregistered = True


class FakeChassis(object):
    instances = []

    def __init__(self, **_kwargs):
        self.calls = []
        self.current_pose = (0.0, 0.0, 0.0)
        self._publisher = FakeEndpoint()
        self._subscriber = FakeEndpoint()
        self.__class__.instances.append(self)

    def wait_until_ready(self, timeout):
        self.calls.append(("ready", timeout))

    def move_open_loop(self, x, y):
        self.current_pose = (x, y, self.current_pose[2])
        self.calls.append(("move_open_loop", x, y))
        return {"target": (x, y)}

    def rotate_open_loop(self, angle):
        start = self.current_pose
        self.current_pose = (
            start[0], start[1], start[2] + angle)
        self.calls.append(("rotate_open_loop", angle))
        return {"start": start}

    def wait_until_stopped(self, speed_threshold, timeout):
        self.calls.append(("wait_stopped", speed_threshold, timeout))
        return True

    def pose(self):
        return self.current_pose

    def stop(self):
        self.calls.append(("stop",))


def _module(monkeypatch):
    FakeChassis.instances = []

    rospy = ModuleType("rospy")
    rospy.loginfo = lambda *_args, **_kwargs: None
    monkeypatch.setitem(sys.modules, "rospy", rospy)

    base_motion = ModuleType("task2_base_motion")
    base_motion.ChassisMotion = FakeChassis
    base_motion.normalize_angle = lambda angle: angle
    monkeypatch.setitem(sys.modules, "task2_base_motion", base_motion)

    task3_randomization = ModuleType("utils.task3_randomization")
    task3_randomization.Task3RandomizationPlanner = lambda: SimpleNamespace(
        plan=lambda _seed: SimpleNamespace(rings=(
            SimpleNamespace(name="ring_1", docking_base=(1.0, 0.0, 0.0)),
            SimpleNamespace(name="ring_2", docking_base=(2.0, 0.0, 0.0)),
        )))
    monkeypatch.setitem(
        sys.modules, "utils.task3_randomization", task3_randomization)

    name = "model_base_motion_under_test"
    spec = importlib.util.spec_from_file_location(name, MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_task1_initialization_opens_no_cmd_vel_endpoint(monkeypatch):
    module = _module(monkeypatch)
    initializer = module.Task1ModelBaseInitializer(seed=3)

    initializer.prepare()
    initializer.close()

    assert initializer.chassis is None
    assert FakeChassis.instances == []


def test_task2_initialization_opens_no_cmd_vel_endpoint(monkeypatch):
    module = _module(monkeypatch)
    initializer = module.Task2ModelBaseInitializer(seed=3)

    initializer.prepare()
    initializer.close()

    assert initializer.chassis is None
    assert FakeChassis.instances == []


def test_task3_initialization_stops_at_first_ring(monkeypatch):
    module = _module(monkeypatch)
    initializer = module.Task3ModelBaseInitializer(seed=4)
    chassis = initializer.chassis

    initializer.prepare()

    move_targets = [call[1:3] for call in chassis.calls
                    if call[0] == "move_open_loop"]
    assert move_targets == [(1.0, 0.0)]


def test_close_stops_and_unregisters_chassis_endpoints(monkeypatch):
    module = _module(monkeypatch)
    initializer = module.Task3ModelBaseInitializer(seed=4)
    chassis = initializer.chassis

    initializer.close()

    assert chassis.calls == [("stop",)]
    assert chassis._publisher.unregistered
    assert chassis._subscriber.unregistered
    assert initializer.chassis is None


def test_fixed_initializers_release_their_chassis_endpoints():
    entry = (PACKAGE / "scripts/model_entry.py").read_text()
    task1 = (PACKAGE / "examples/task1_v2_initialize.py").read_text()
    assert entry.count("close_chassis(chassis)") == 2
    assert "close_chassis(chassis)" in task1


def test_model_entry_closes_initializer_before_ready():
    source = (PACKAGE / "scripts/model_entry.py").read_text()
    start = source.index("def _initialize(self):")
    end = source.index("    def publish_success(self, value):")
    body = source[start:end]
    assert body.index("base_initializer.prepare()") < body.index(
        "base_initializer.close()")
    assert body.index("base_initializer.close()") < body.index(
        "self.ready_publisher.publish(Bool(data=True))")


def test_model_entry_has_no_episode_time_base_worker():
    source = (PACKAGE / "scripts/model_entry.py").read_text()
    assert "base_motion.start()" not in source
    assert "base_motion.error" not in source
    assert "_drop_base_motion" not in source


def test_model_initializer_contains_no_episode_route_or_trigger():
    source = MODULE_PATH.read_text()
    assert "base_move_trigger" not in source
    assert "def run(" not in source
    assert "def start(" not in source
    assert "self._wait(" not in source
    assert not (PACKAGE / "utils/base_move_trigger.py").exists()


def test_model_initialization_never_uses_closed_loop_pose_arrival():
    source = MODULE_PATH.read_text()
    assert "move_to_pose" not in source
    assert "translate_relative" not in source
    assert "rotate_relative" not in source
