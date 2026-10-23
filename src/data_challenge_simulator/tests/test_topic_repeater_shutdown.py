"""A timer publishing during ROS teardown must exit without masking live errors."""
import ast
from pathlib import Path
from types import SimpleNamespace
import pytest

@pytest.mark.parametrize("shutdown", [False, True])
def test_publisher_closure_race(shutdown):
    path=Path(__file__).resolve().parents[1]/"utils/topic_repeater.py"
    cls=next(n for n in ast.parse(path.read_text()).body if isinstance(n,ast.ClassDef))
    callback=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=="timer_callback")
    state={"stopping":False}
    class RosError(Exception): pass
    def publish(message):
        state["stopping"]=shutdown
        raise RosError("publish() to a closed topic")
    ros=SimpleNamespace(core=SimpleNamespace(is_shutdown_requested=lambda:state["stopping"]),
                        ROSException=RosError, Time=SimpleNamespace(now=lambda:1))
    scope={"rospy":ros,"JointState":SimpleNamespace}
    exec(compile(ast.Module(body=[callback],type_ignores=[]),str(path),"exec"),scope)
    relay=SimpleNamespace(last_cmd_twist=None,
        last_joint_state=SimpleNamespace(header=SimpleNamespace(),name=[],position=[],velocity=[],effort=[]),
        joint_state_pub=SimpleNamespace(publish=publish))
    if shutdown:
        scope["timer_callback"](relay,None)
    else:
        with pytest.raises(RosError):scope["timer_callback"](relay,None)
