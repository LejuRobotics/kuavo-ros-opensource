"""A moved destination must not put the scripted return through table legs."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace
import pytest

def test_transfer_stays_in_front_corridor():
    path=Path(__file__).resolve().parents[1]/"examples/task3_descend.py"
    node=next(n for n in ast.parse(path.read_text()).body
              if isinstance(n,ast.FunctionDef) and n.name=="table_transfer_waypoints")
    namespace={"math": math, "docking_is_clear": lambda *args: False}
    exec(compile(ast.Module(body=[node],type_ignores=[]),str(path),"exec"),namespace)
    waypoints=namespace["table_transfer_waypoints"]
    for dy in (-.1,0,.1):
        model=SimpleNamespace(
            body=lambda name:SimpleNamespace(pos=(.62,
                                                 .85+dy if "destination" in name else 0,0)),
            geom=lambda name:SimpleNamespace(size=(.20,.36,.03)))
        current=(-.196,1.309,0)
        target=(.052,.061,0)
        points=waypoints(current,target,model,.03)
        assert points[-1]==target
        for point in points:
            if point[1]!=current[1]:
                assert max(current[0],point[0])<=1e-9
            current=point

    # A lateral destination already on the corridor is one move, not two
    # identical destinations with a stop between them.
    points=waypoints((-.1,0,0),(-.1,.9,0),model,.03)
    assert points == [(-.1,.9,0)]

    # Clear straight segments need no intermediate stop.
    namespace["docking_is_clear"] = lambda *args: True
    assert waypoints((.05,.54,0),(-.1,1.1,0),model,.03) == [(-.1,1.1,0)]
