import json
from utils.task1_v2_randomization import (
    DROP_CENTER_OFFSETS,
    MIN_OBJECT_TRANSVERSE_SEPARATION,
    OBJECT_COUNT,
    Task1V2RandomizationPlanner,
)


def test_saved_layouts_have_three_transversely_separated_cylinders():
    planner = Task1V2RandomizationPlanner()
    for seed in range(1, len(planner.layouts) + 1):
        plan = planner.plan(seed)
        assert len(plan.cylinders) == OBJECT_COUNT == 3
        for first in range(len(plan.cylinders)):
            for second in range(first + 1, len(plan.cylinders)):
                assert abs(
                    plan.cylinders[first].position[1]
                    - plan.cylinders[second].position[1]
                ) >= MIN_OBJECT_TRANSVERSE_SEPARATION


def test_seed_selection_is_reproducible_and_wraps_catalogue():
    planner = Task1V2RandomizationPlanner()
    first = planner.plan(1)
    repeated = planner.plan(1)
    wrapped = planner.plan(1 + len(planner.layouts))
    assert first == repeated
    assert first.catalogue_seed == wrapped.catalogue_seed
    assert first.initial_base == wrapped.initial_base
    assert first.task_base == wrapped.task_base
    assert first.cylinders == wrapped.cylinders


def test_collect_layout_override_does_not_replace_saved_catalogue(
        tmp_path, monkeypatch):
    saved = Task1V2RandomizationPlanner()
    saved_first_catalogue_seed = int(saved.layouts[0]["seed"])
    document = {
        "layouts": [{
            "seed": 999,
            "initial_base": [0.1, 0.2, 0.0],
            "task_base": [0.1, 0.2, 0.0],
            "cylinders": [
                {"position": [0.5, -0.2, 0.685], "yaw": 0.0},
                {"position": [0.6, -0.1, 0.685], "yaw": 0.0},
                {"position": [0.5, 0.0, 0.685], "yaw": 0.0},
            ],
        }],
    }
    collect_file = tmp_path / "collect.json"
    collect_file.write_text(json.dumps(document), encoding="utf-8")
    monkeypatch.setenv("TASK1_V2_LAYOUT_FILE", str(collect_file))

    collect = Task1V2RandomizationPlanner()
    assert collect.plan(1).catalogue_seed == 999
    assert saved.plan(1).catalogue_seed == saved_first_catalogue_seed


def test_generation_ranges_allow_forward_spread():
    planner = Task1V2RandomizationPlanner()
    with planner.layout_file.open("r", encoding="utf-8") as stream:
        generation = json.load(stream)["generation"]

    assert generation["robot_initial_x_range"] == [-0.133, -0.088165]
    assert generation["robot_task_x_range"] == [-0.03, 0.015]
    assert generation["object_regions"][0]["x"] == [0.54, 0.65]
    assert generation["object_regions"][1]["x"] == [0.54, 0.65]
    assert generation["object_regions"][2]["x"] == [0.55, 0.63]

    for layout in planner.layouts:
        assert -0.133 <= layout["initial_base"][0] <= -0.088165
        assert -0.03 <= layout["task_base"][0] <= 0.015
        blue_x = layout["cylinders"][2]["position"][0]
        assert 0.55 <= blue_x <= 0.63


def test_saved_catalogue_is_not_forced_into_one_forward_line():
    planner = Task1V2RandomizationPlanner()
    forward_spans = []
    for layout in planner.layouts:
        forward_positions = [
            cylinder["position"][0]
            for cylinder in layout["cylinders"]
        ]
        forward_spans.append(max(forward_positions) - min(forward_positions))

    assert max(forward_spans) >= 0.05


def test_three_drop_slots_are_distinct():
    assert len(DROP_CENTER_OFFSETS) == OBJECT_COUNT == 3
    assert len(set(DROP_CENTER_OFFSETS)) == OBJECT_COUNT
