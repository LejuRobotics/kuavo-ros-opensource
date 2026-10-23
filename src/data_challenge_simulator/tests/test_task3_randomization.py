import math
from utils.task3_randomization import RING_NAMES, Task3RandomizationPlanner

def test_generated_rings_do_not_wrap_and_are_separated():
    planner=Task3RandomizationPlanner()
    assert planner.plan(28)==planner.plan(28)
    assert planner.plan(28).rings!=planner.plan(38).rings
    for seed in range(100):
        plan=planner.plan(seed)
        assert tuple(r.name for r in plan.rings)==RING_NAMES
        assert plan.initial_base==(0,0,0)
        for i,a in enumerate(plan.rings):
            assert .50<=a.position[0]<=.74
            assert -.29<=a.position[1]<=.29
            assert a.position[2]==.65
            for b in plan.rings[i+1:]:
                assert math.dist(a.position[:2],b.position[:2])>=.16


def test_target_table_shift_does_not_block_pickup_docking():
    from utils.task3_randomization import docking_is_clear
    # Previously the left front wheel hit the nearer target table leg.
    assert not docking_is_clear((.047757, .369057, 0), (.522456, .85, 0))
    for seed in range(28, 33):
        plan = Task3RandomizationPlanner().plan(seed)
        assert plan.destination_position[0] == .62
        assert abs(plan.destination_position[1]-.85) <= .1
        assert all(docking_is_clear(r.docking_base, plan.destination_position)
                   for r in plan.rings)
