"""Observe shutdown ordering without starting ROS or a simulator."""
import ast
import threading
from pathlib import Path
from types import SimpleNamespace
import pytest

@pytest.mark.parametrize("fallback", [False, True])
def test_publish_timers_finish_before_topics_close(fallback):
    path = Path(__file__).resolve().parents[1] / "scripts/rosbag_compat_publisher.py"
    tree = ast.parse(path.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef)
               and n.name == "RosbagCompatPublisher")
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef)
                  and n.name == "_stop_timers")
    scope = {"threading": threading}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), "exec"), scope)
    events = []
    def timer(name):
        return SimpleNamespace(shutdown=lambda: events.append(("stop", name)),
                               join=lambda: events.append(("join", name)))
    obj = SimpleNamespace(timer=timer("pose"), cmd_vel_timer=timer("velocity") if fallback else None)
    scope["_stop_timers"](obj)
    names = ["pose", "velocity"] if fallback else ["pose"]
    assert events == [("stop", n) for n in names] + [("join", n) for n in names]
    assert "rospy.on_shutdown(self._stop_timers)" in path.read_text()
