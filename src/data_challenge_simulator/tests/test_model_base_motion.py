"""Regression tests for model-only chassis cleanup and handoff."""

import importlib.util
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
MODULE_PATH = PACKAGE / "utils/model_base_motion.py"


class FakeEndpoint(object):
    def __init__(self):
        self.unregistered = False

    def unregister(self):
        self.unregistered = True


class FakeChassis(object):
    def __init__(self):
        self.calls = []
        self._publisher = FakeEndpoint()
        self._subscriber = FakeEndpoint()

    def stop(self):
        self.calls.append(("stop",))


def _module():
    name = "model_base_motion_under_test"
    spec = importlib.util.spec_from_file_location(name, MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_close_stops_and_unregisters_chassis_endpoints():
    module = _module()
    chassis = FakeChassis()

    module.close_chassis(chassis)

    assert chassis.calls == [("stop",)]
    assert chassis._publisher.unregistered
    assert chassis._subscriber.unregistered


def test_close_accepts_absent_chassis():
    module = _module()
    assert module.close_chassis(None) is None


def test_fixed_initializers_release_their_chassis_endpoints():
    entry = (PACKAGE / "scripts/model_entry.py").read_text()
    task1 = (PACKAGE / "examples/task1_v2_initialize.py").read_text()
    assert entry.count("close_chassis(chassis)") == 2
    assert "close_chassis(chassis)" in task1


def test_model_entry_has_no_post_initialization_base_motion():
    source = (PACKAGE / "scripts/model_entry.py").read_text()
    start = source.index("def _initialize(self):")
    end = source.index("    def publish_success(self, value):")
    body = source[start:end]
    assert "base_initializer" not in body
    assert "model_base_motion" not in body
    assert "docking_base" not in body
    assert "move_open_loop" not in body


def test_model_entry_has_no_episode_time_base_worker():
    source = (PACKAGE / "scripts/model_entry.py").read_text()
    assert "base_motion.start()" not in source
    assert "base_motion.error" not in source
    assert "_drop_base_motion" not in source


def test_model_base_helper_contains_no_navigation_logic():
    source = MODULE_PATH.read_text()
    assert "Task3RandomizationPlanner" not in source
    assert "from task2_base_motion import" not in source
    assert "ChassisMotion(" not in source
    assert "docking_base" not in source
    assert "move_open_loop" not in source
    assert "rotate_open_loop" not in source
    assert "move_to_pose" not in source
    assert "base_move_trigger" not in source
    assert not (PACKAGE / "utils/base_move_trigger.py").exists()
