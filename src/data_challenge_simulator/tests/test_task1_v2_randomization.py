import math
from utils.task1_v2_randomization import DROP_CENTER_OFFSETS, Task1V2RandomizationPlanner

def test_seed_does_not_wrap_ten_layouts():
    planner=Task1V2RandomizationPlanner()
    first=planner.plan(28)
    assert first==planner.plan(28)
    assert first.cylinders!=planner.plan(38).cylinders
    assert len(set(DROP_CENTER_OFFSETS))==3
    for i,a in enumerate(first.cylinders):
        assert .55<=a.position[0]<=.61
        assert a.position[2]==.685
        for b in first.cylinders[i+1:]:
            assert math.dist(a.position[:2],b.position[:2])>=.075
