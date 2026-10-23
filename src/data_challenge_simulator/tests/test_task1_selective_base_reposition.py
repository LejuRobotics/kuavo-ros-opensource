import json
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
CONFIG = PACKAGE / "config/task1_v2_layout_seeds.json"
GENERATOR = PACKAGE / "tools/generate_task1_v2_layout_seeds.py"
TASK = PACKAGE / "examples/task1.py"


def test_generation_profiles_separate_direct_and_move_required_regions():
    document = json.loads(CONFIG.read_text())
    generation = document["generation"]

    assert generation["direct_task_base"] == [0.015, 0.0, 0.0]
    assert generation["blue_left_shift_range"] == [0.01, 0.10]
    assert generation["blue_left_shift_margin"] == 0.04
    assert len(generation["object_region_profiles"]) >= 2
    for profile in generation["object_region_profiles"]:
        assert len(profile["regions"]) == 3
        blue = profile["regions"][2]
        assert 0.625 <= blue["x"][0] <= blue["x"][1] <= 0.64
        assert -0.10 <= blue["y"][0] <= blue["y"][1] <= -0.02

    for layout in document["layouts"]:
        task_base = layout["task_base"]
        grasp_bases = layout["grasp_bases"]
        assert grasp_bases[0] == task_base
        assert grasp_bases[1] == task_base
        assert grasp_bases[2][0] == task_base[0]
        assert grasp_bases[2][1] > task_base[1]
        commanded_shift = grasp_bases[2][1] - task_base[1]
        assert commanded_shift <= 0.14
        assert commanded_shift == (
            layout["blue_minimum_feasible_shift_m"]
            + layout["blue_shift_margin_m"])
        assert layout["blue_shift_margin_m"] == 0.04


def test_generator_requires_red_yellow_direct_and_blue_move():
    source = GENERATOR.read_text()

    assert "for index in (0, 1):" in source
    assert "if self.source_plan(task_base, cylinders[2], 2) is not None:" in source
    assert 'self.generation["blue_left_shift_range"]' in source
    assert 'self.generation["blue_left_shift_margin"]' in source
    assert "commanded_shift = minimum_feasible_shift + shift_margin" in source
    assert '"grasp_bases": grasp_bases' in source
    assert "MAX_SOURCE_ORIENTATION_ERROR_DEG = 8.0" in source


def test_runtime_keeps_base_stationary_with_generated_objects():
    from utils.task1_v2_randomization import Task1V2RandomizationPlanner
    for seed in (28,29):
        plan=Task1V2RandomizationPlanner().plan(seed)
        assert plan.initial_base==plan.task_base==(.015,0,0)
        assert not plan.requires_base_motion
