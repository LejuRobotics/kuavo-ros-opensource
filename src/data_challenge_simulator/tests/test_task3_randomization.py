import json
import math

from utils.task3_randomization import (
    MIN_DOCKED_JOINT_MARGIN_RAD,
    MIN_RING_SEPARATION_M,
    RING_NAMES,
    Task3RandomizationPlanner,
)


def test_saved_layouts_have_three_verified_separated_rings():
    planner = Task3RandomizationPlanner()
    for seed in range(1, len(planner.layouts) + 1):
        plan = planner.plan(seed)
        assert tuple(ring.name for ring in plan.rings) == RING_NAMES
        for index, ring in enumerate(plan.rings):
            assert ring.joint_margin_rad >= MIN_DOCKED_JOINT_MARGIN_RAD
            for other in plan.rings[index + 1:]:
                assert math.dist(
                    ring.position[:2], other.position[:2]
                ) >= MIN_RING_SEPARATION_M


def test_seed_selection_is_reproducible_and_wraps_catalogue():
    planner = Task3RandomizationPlanner()
    first = planner.plan(1)
    repeated = planner.plan(1)
    wrapped = planner.plan(1 + len(planner.layouts))
    assert first == repeated
    assert first.catalogue_seed == wrapped.catalogue_seed
    assert first.initial_base == wrapped.initial_base
    assert first.rings == wrapped.rings


def test_collect_layout_override_does_not_replace_saved_catalogue(
        tmp_path, monkeypatch):
    saved = Task3RandomizationPlanner()
    layout = dict(saved.layouts[0])
    layout["seed"] = 999
    collect_file = tmp_path / "collect.json"
    collect_file.write_text(
        json.dumps({"layouts": [layout]}), encoding="utf-8")
    monkeypatch.setenv("TASK3_LAYOUT_FILE", str(collect_file))

    collect = Task3RandomizationPlanner()
    assert collect.plan(1).catalogue_seed == 999
    assert saved.plan(1).catalogue_seed == 1
