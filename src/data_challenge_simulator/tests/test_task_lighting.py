"""Static contracts for deterministic per-round task lighting."""

from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
SOURCE = PACKAGE.parent
HELPER = PACKAGE / "examples/helperfunc.py"
MODEL_ENTRY = PACKAGE / "scripts/model_entry.py"
LAUNCH = PACKAGE / "launch"
MUJOCO_NODE = SOURCE / "mujoco/src/mujoco_node.cc"


def test_collect_passes_round_seed_to_lighting():
    source = HELPER.read_text()
    launch_builder = source[source.index("def simulator_launch_command"):
                            source.index("def ensure_clean_simulator_graph")]
    assert "'task_light_seed:={}'.format(round_seed(task_id, round_id))" in launch_builder


def test_model_passes_round_seed_to_lighting():
    source = MODEL_ENTRY.read_text()
    assert '"task_light_seed:={}".format(seed)' in source


def test_task_and_model_launches_route_the_light_seed():
    for task_id in (1, 2, 3):
        task_name = ("load_kuavo_mujoco_sim1.launch" if task_id == 1
                     else "load_kuavo_mujoco_sim{}.launch".format(task_id))
        task_source = (LAUNCH / task_name).read_text()
        assert '<arg name="task_light_seed" default="-1"/>' in task_source
        assert (
            '<param name="task_light_seed" value="$(arg task_light_seed)"/>'
            in task_source)
        assert '<param name="task_light_profile" value="{}"/>'.format(
            task_id) in task_source

        model_source = (LAUNCH / "load_kuavo_mujoco_model{}.launch".format(
            task_id)).read_text()
        assert '<arg name="task_light_seed" default="-1"/>' in model_source
        assert (
            '<arg name="task_light_seed" value="$(arg task_light_seed)"/>'
            in model_source)


def test_mujoco_uses_upstream_headlight_ranges_only():
    source = MUJOCO_NODE.read_text()
    start = source.index("void randomizeTaskLighting")
    end = source.index("void PhysicsThread", start)
    body = source[start:end]

    assert 'getParam("task_light_seed", seed)' in body
    assert 'getParam("task_light_profile", profile)' in body
    assert "channel == tint ? 0.9f : 0.7f" in body
    assert "ambient[channel] = 0.25f" in body
    assert "specular[channel] = 0.0f" in body
    assert "std::seed_seq seed_sequence{seed, profile}" in body
    assert "model->light_diffuse[channel] =" not in body
    assert "model->vis.headlight.diffuse[channel]" in body
    assert "model->vis.headlight.ambient[channel]" in body
    assert "model->vis.headlight.specular[channel]" in body
    assert "source=scene_xml" in body
    assert "task2_scene_seed" in body


def test_all_tasks_use_identical_task2_lights():
    import xml.etree.ElementTree as ET
    import mujoco
    import numpy as np
    from utils.task2_randomization import task_scene, DEFAULT_SCENE

    for seed in (1, 2, 28):
        expected = None
        for task_id in (2, 1, 3):
            original = ET.parse(DEFAULT_SCENE.with_name('task%d.xml' % task_id))
            with task_scene(task_id, seed) as filename:
                generated = ET.parse(filename)
                lights = (generated.find('visual/headlight').attrib,
                          [light.attrib for light in generated.findall('.//worldbody//light')])
                if expected is None:
                    expected = lights
                assert lights == expected
                model = mujoco.MjModel.from_xml_path(filename)
                assert model.nlight == 1
                assert np.allclose(model.light_diffuse, [[0.7, 0.7, 0.7]])

        assert not Path(filename).exists()
