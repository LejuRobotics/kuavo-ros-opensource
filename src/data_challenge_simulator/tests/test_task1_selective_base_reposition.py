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


def test_runtime_moves_only_for_layout_marked_grasp_base():
    source = TASK.read_text()

    assert "planned_grasp_base = randomization_plan.grasp_bases[index]" in source
    assert "common_task_base_measured[axis] + grasp_base_delta[axis]" in source
    assert "requires_grasp_base_move = any(" in source
    assert '"{}.grasp_base_move".format(name)' in source
    assert '"{}.loaded_base_return".format(name)' in source
    assert "BLUE_RUNTIME_LEFT_SHIFT_MARGIN_M = 0.02" in source
    assert "grasp_base_delta[1] + BLUE_RUNTIME_LEFT_SHIFT_MARGIN_M" in source
