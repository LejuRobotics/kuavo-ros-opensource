import json
import math

from utils.task2_pick_planner import BOX_NAMES
from utils.task2_randomization import Task2RandomizationPlanner


BOX_X_RANGE = (0.74, 0.78)
BOX_2_Y_RANGE = (0.02, 0.30)
FIXED_BOX_1_Y = -0.42
FRONT_STANDOFF_M = 0.650


def test_saved_layouts_keep_both_boxes_inside_the_declared_ranges():
    planner = Task2RandomizationPlanner()
    for seed in range(1, len(planner.layouts) + 1):
        plan = planner.plan(seed)
        assert tuple(box.name for box in plan.boxes) == BOX_NAMES
        by_name = {box.name: box for box in plan.boxes}
        for box in plan.boxes:
            assert BOX_X_RANGE[0] <= box.position[0] <= BOX_X_RANGE[1]
        assert math.isclose(
            by_name["box_1"].position[1], FIXED_BOX_1_Y, abs_tol=1e-9)
        assert (BOX_2_Y_RANGE[0] <= by_name["box_2"].position[1]
                <= BOX_2_Y_RANGE[1])


def test_docking_base_is_the_box_centre_pulled_back_by_the_standoff():
    planner = Task2RandomizationPlanner()
    for seed in range(1, len(planner.layouts) + 1):
        for box in planner.plan(seed).boxes:
            assert math.isclose(
                box.docking_base[0],
                box.position[0] - FRONT_STANDOFF_M, abs_tol=1e-9)
            assert math.isclose(
                box.docking_base[1], box.position[1], abs_tol=1e-9)
            assert box.docking_base[2] == 0.0


def test_seed_selection_is_reproducible_and_wraps_catalogue():
    planner = Task2RandomizationPlanner()
    first = planner.plan(1)
    repeated = planner.plan(1)
    wrapped = planner.plan(1 + len(planner.layouts))
    assert first == repeated
    assert first.catalogue_seed == wrapped.catalogue_seed
    assert first.initial_base == wrapped.initial_base
    assert first.boxes == wrapped.boxes


def test_collect_layout_override_does_not_replace_saved_catalogue(
        tmp_path, monkeypatch):
    saved = Task2RandomizationPlanner()
    layout = dict(saved.layouts[0])
    layout["seed"] = 999
    collect_file = tmp_path / "collect.json"
    collect_file.write_text(
        json.dumps({"layouts": [layout]}), encoding="utf-8")
    monkeypatch.setenv("TASK2_LAYOUT_FILE", str(collect_file))

    collect = Task2RandomizationPlanner()
    assert collect.plan(1).catalogue_seed == 999
    assert saved.plan(1).catalogue_seed == 1
