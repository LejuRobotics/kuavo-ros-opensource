import json
import os
from pathlib import Path

import mujoco
import pytest

from utils.task2_randomization import (
    BOX_NAMES, DEFAULT_SCENE, Task2RandomizationPlanner,
    active_scene, seed_for_round, task_scene,
)


def test_seeded_scenes_fit_table_and_do_not_cycle_a_catalogue(monkeypatch):
    monkeypatch.setenv("TASK2_LAYOUT_FILE", "/does/not/exist.json")
    planner = Task2RandomizationPlanner()
    assert [seed_for_round(i) for i in range(1, 6)] == [28, 29, 30, 31, 32]
    samples = [planner.plan(seed) for seed in range(1, 1001)]
    assert planner.plan(28) == planner.plan(28)
    assert len({p.boxes for p in samples}) == 1000
    assert samples[0].boxes != samples[10].boxes
    for plan in samples:
        assert plan.initial_base == (0.0, 0.0, 0.0)
        assert tuple(b.name for b in plan.boxes) == BOX_NAMES
        assert -0.10 <= plan.conveyor_x_offset <= 0.10
        assert plan.conveyor_position[1:] == (1.10, 0.0)
        assert plan.boxes[1].position[1] - plan.boxes[0].position[1] - 0.44 >= 0.12 - 1e-12
        for box in plan.boxes:
            x, y, z = box.position
            assert x - 0.32 == pytest.approx(0.43)
            assert -0.64 + 0.02 <= y - 0.22
            assert y + 0.22 <= 0.52 - 0.02
            assert z == 0.60 and box.yaw == 0
            assert box.docking_base[1] == y


def test_generated_scene_used_by_simulation_and_ik_and_cleaned_on_failure():
    from utils.task2_pick_planner import Task2PickPlanner
    baseline = DEFAULT_SCENE.read_bytes()
    plan = Task2RandomizationPlanner().plan(28)
    with pytest.raises(RuntimeError, match="test failure"):
        with task_scene(2, 28) as filename:
            assert active_scene() == Path(filename)
            model = mujoco.MjModel.from_xml_path(filename)
            data = mujoco.MjData(model)
            mujoco.mj_forward(model, data)
            belt = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "destination_conveyor")
            assert tuple(data.xpos[belt]) == pytest.approx(plan.conveyor_position)
            assert model.vis.headlight.active == 1
            assert tuple(model.vis.headlight.diffuse) == pytest.approx(plan.lighting["headlight_diffuse"])
            assert tuple(model.vis.headlight.ambient) == pytest.approx((0.25, 0.25, 0.25))
            assert tuple(model.vis.headlight.specular) == (0, 0, 0)
            assert model.nlight == 1 and model.light_directional[0]
            assert tuple(model.light_pos[0]) == (0, 0, 7)
            assert tuple(model.light_dir[0]) == (0, 0, -1)
            assert tuple(model.light_diffuse[0]) == pytest.approx((0.7, 0.7, 0.7))
            for box in plan.boxes:
                body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, box.name)
                assert tuple(data.xpos[body]) == pytest.approx(box.position)
            planner = Task2PickPlanner()
            assert tuple(planner.model.body_pos[belt]) == pytest.approx(plan.conveyor_position)
            raise RuntimeError("test failure")
    assert not Path(filename).exists()
    assert "TASK2_SCENE_FILE" not in os.environ
    assert DEFAULT_SCENE.read_bytes() == baseline


def test_first_five_layouts_have_solvable_downward_grasp_and_lift():
    from utils.task2_pick_planner import Task2PickPlanner
    from utils.task2_randomization import PACKAGE_DIR
    import numpy as np
    cfg = json.loads((PACKAGE_DIR / "config/task2_pick.json").read_text())
    for seed in range(28, 33):
        with task_scene(2, seed):
            layout = Task2RandomizationPlanner().plan(seed)
            planner = Task2PickPlanner(ik_config=cfg["ik"])
            arm_seed = np.concatenate([planner.model.qpos0[planner.arm_qpos_addresses[s]] for s in ("l", "r")])
            poses = {b.name: (b.position, (0, 0, 0, 1)) for b in layout.boxes}
            for box in layout.boxes:
                plan = planner.plan(box.name, box.docking_base, poses, arm_seed, cfg["grasp"])
                assert len(plan.lift_waypoints) == cfg["grasp"]["lift_waypoint_count"]


def test_task_collect_metadata_and_model_reset_share_round_seeds():
    import ast
    import random
    from types import SimpleNamespace
    from utils.task2_randomization import PACKAGE_DIR
    helper = ast.parse((PACKAGE_DIR / "examples/helperfunc.py").read_text())
    funcs = [node for node in helper.body if isinstance(node, ast.FunctionDef)
             and node.name in ("round_seed", "score_metadata")]
    namespace = {"seed_for_round": seed_for_round}
    exec(compile(ast.Module(body=funcs, type_ignores=[]), "helperfunc.py", "exec"), namespace)
    for round_id in range(1, 6):
        assert namespace["score_metadata"](2, round_id) == {
            "task_id": 2, "round": round_id, "seed": 27 + round_id}
    assert namespace["round_seed"](1, 5) == 32
    assert namespace["round_seed"](3, 5) == 32
    entry = ast.parse((PACKAGE_DIR / "scripts/model_entry.py").read_text())
    cls = next(n for n in entry.body if isinstance(n, ast.ClassDef) and n.name == "EvaluationProtocol")
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "_next_reset_seed")
    namespace = {"random": random}
    exec(compile(ast.Module(body=[method], type_ignores=[]), "model_entry.py", "exec"), namespace)
    state = SimpleNamespace(task_id=2, round_seed=28, score_file=None)
    assert namespace["_next_reset_seed"](state) == 28  # Retry before first start.
    state.score_file = "score.txt"
    for seed in range(28, 32):
        state.round_seed = seed
        assert namespace["_next_reset_seed"](state) == seed + 1


def test_bright_pastel_headlights_and_fixed_scene_light():
    planner = Task2RandomizationPlanner()
    expected = {"light_red": (0.9, 0.7, 0.7), "light_green": (0.7, 0.9, 0.7),
                "light_blue": (0.7, 0.7, 0.9)}
    samples = [planner.plan(seed).lighting for seed in range(28, 128)]
    assert {p["headlight_color"] for p in samples} == set(expected)
    assert planner.plan(28).lighting == planner.plan(28).lighting
    for lighting in samples:
        assert lighting["headlight_diffuse"] == expected[lighting["headlight_color"]]
        assert lighting["headlight_ambient"] == (0.25, 0.25, 0.25)
    for seed in (28, 29, 30, 31, 32):
        with task_scene(2, seed) as filename:
            model = mujoco.MjModel.from_xml_path(filename)
            assert tuple(model.vis.headlight.diffuse) == pytest.approx(planner.plan(seed).lighting["headlight_diffuse"])
            assert tuple(model.vis.headlight.ambient) == pytest.approx((0.25, 0.25, 0.25))
            assert tuple(model.light_diffuse[0]) == pytest.approx((0.7, 0.7, 0.7))
            assert tuple(model.light_specular[0]) == pytest.approx((0.3, 0.3, 0.3))
            assert tuple(model.light_ambient[0]) == (0, 0, 0)
