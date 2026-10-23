"""Publishing races with ROS shutdown; normal publish failures must surface."""
import ast
from pathlib import Path
from types import SimpleNamespace
from threading import Event, Lock

import pytest


def test_shutdown_during_publish():
    source = Path(__file__).resolve().parents[1] / "utils/gripper_controller.py"
    tree = ast.parse(source.read_text())
    method = next(n for n in ast.walk(tree)
                  if isinstance(n, ast.FunctionDef) and n.name == "_publish_loop")
    for shutdown in (True, False):
        stopped = [False]
        class PublishError(Exception):
            pass
        def publish(message):
            stopped[0] = shutdown
            raise PublishError("closed topic")
        ros = SimpleNamespace(is_shutdown=lambda: stopped[0],
                              Rate=lambda hz: SimpleNamespace(sleep=lambda: None),
                              ROSException=PublishError,
                              ROSInterruptException=KeyboardInterrupt)
        namespace = {"rospy": ros}
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), "exec"), namespace)
        event = Event()
        event.set()
        instance = SimpleNamespace(publish_frequency=100, is_running=True,
                                   command_initialized=event, command_lock=Lock(),
                                   current_left_positions=[0], current_right_positions=[0],
                                   command_pub=SimpleNamespace(publish=publish),
                                   _message=lambda left, right: (left, right))
        if shutdown:
            namespace["_publish_loop"](instance)
        else:
            with pytest.raises(PublishError):
                namespace["_publish_loop"](instance)
