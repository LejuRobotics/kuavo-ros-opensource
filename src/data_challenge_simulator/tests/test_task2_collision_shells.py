"""Task 2 full robot contact coverage and physical blocking."""
import copy
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest

PACKAGE = Path(__file__).resolve().parents[1]
SCENES = PACKAGE / "models/biped_s400062/xml"
ROBOT = ET.parse(SCENES / "biped_s400062_icra.xml").getroot()


@pytest.mark.parametrize("mesh", [
    "torso", "base_link", "l_forearm", "r_forearm",
    "l_hand_base", "r_hand_base", "l_index_link1", "l_index_link2",
    "l_index_link3", "r_thumb_link2",
])
def test_shell_physically_blocks_a_pushed_object(mesh):
    """Use the actual STL shell and task masks, with a 20 N box probe."""
    root = ET.Element("mujoco")
    ET.SubElement(root, "compiler", meshdir=str(
        (SCENES / ROBOT.find("compiler").get("meshdir")).resolve()))
    ET.SubElement(root, "option", gravity="0 0 0", timestep="0.002",
                  integrator="implicitfast")
    asset = ET.SubElement(root, "asset")
    asset.append(copy.deepcopy(ROBOT.find("./asset/mesh[@name='%s']" % mesh)))
    world = ET.SubElement(root, "worldbody")
    shell = copy.deepcopy(ROBOT.find(
        ".//geom[@name='task2_%s_collision']" % mesh))
    assert shell is not None
    shell.set("name", "shell")
    world.append(shell)
    body = ET.SubElement(world, "body", name="probe", pos="1 0 0")
    ET.SubElement(body, "joint", type="slide", axis="1 0 0")
    ET.SubElement(body, "inertial", pos="0 0 0", mass="2",
                  diaginertia="0.001 0.001 0.001")
    ET.SubElement(body, "geom", name="probe", type="box", size="0.02 0.02 0.02",
                  contype="1", conaffinity="11", solref="0.01 1",
                  solimp="0.95 0.99 0.001")
    model = mujoco.MjModel.from_xml_string(ET.tostring(root).decode())
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    mesh_id = model.geom_dataid[0]
    vertices = model.mesh_vert[model.mesh_vertadr[mesh_id]:
                              model.mesh_vertadr[mesh_id] + model.mesh_vertnum[mesh_id]]
    points = vertices @ data.geom_xmat[0].reshape(3, 3).T + data.geom_xpos[0]
    low, high = points.min(axis=0), points.max(axis=0)
    model.body_pos[1] = (low + high) / 2
    model.body_pos[1, 0] = high[0] + 0.10
    mujoco.mj_forward(model, data)
    data.qfrc_applied[0] = -20
    for _ in range(1000):
        mujoco.mj_step(model, data)
    assert data.ncon > 0, mesh
    assert min(c.dist for c in data.contact[:data.ncon]) > -0.002, mesh
    assert data.xpos[1, 0] > low[0], mesh
    assert abs(data.qvel[0]) < 0.01, mesh


def test_task_objects_and_tables_collide_with_robot_shells():
    for task, names in (
            (1, ("block_1_geom", "target_bin_floor", "source_bin_floor")),
            (3, ("task3_destination_table_top", "task3_source_table_top"))):
        model = mujoco.MjModel.from_xml_path(str(SCENES / ("task%d.xml" % task)))
        shell = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM,
                                "task2_base_link_collision")
        for name in names:
            geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
            assert geom >= 0, name
            assert (model.geom_contype[shell] & model.geom_conaffinity[geom]
                    or model.geom_contype[geom] & model.geom_conaffinity[shell]), name


def test_box_mass_and_robot_inertias_are_preserved_by_shells():
    model = mujoco.MjModel.from_xml_path(str(SCENES / "task2.xml"))
    for name in ("box_1", "box_2"):
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        assert model.body_mass[bid] == pytest.approx(1.0)
    # Compile the same robot without the new shells and compare every inertia.
    robot = copy.deepcopy(ROBOT)
    for parent in robot.iter():
        for child in list(parent):
            if child.tag == "geom" and child.get("name", "").startswith("task2_"):
                parent.remove(child)
    robot.find("compiler").set("meshdir", str(
        (SCENES / ROBOT.find("compiler").get("meshdir")).resolve()))
    original = mujoco.MjModel.from_xml_string(ET.tostring(robot).decode())
    for bid in range(1, original.nbody):
        name = mujoco.mj_id2name(original, mujoco.mjtObj.mjOBJ_BODY, bid)
        new_bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        np.testing.assert_array_equal(model.body_mass[new_bid], original.body_mass[bid])
        np.testing.assert_array_equal(model.body_inertia[new_bid], original.body_inertia[bid])



def test_loaded_docking_positions_leave_room_for_a_quarter_turn():
    import json
    import math
    from utils.task2_randomization import Task2RandomizationPlanner

    config = json.loads((PACKAGE / "config/task2_pick.json").read_text())
    model = mujoco.MjModel.from_xml_path(str(SCENES / "task2.xml"))
    data = mujoco.MjData(model)
    base_geom = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_GEOM, "task2_base_link_collision")
    for seed in (1, 2):
        for box in Task2RandomizationPlanner().plan(seed).boxes:
            for degrees in range(91):
                angle = math.radians(degrees)
                data.qpos[:] = model.qpos0
                data.qpos[:2] = (
                    box.position[0] - config["docking"]["front_standoff_m"],
                    box.position[1])
                data.qpos[3:7] = (
                    math.cos(angle / 2), 0, 0, math.sin(angle / 2))
                mujoco.mj_forward(model, data)
                assert all(
                    contact.dist >= 0 for contact in data.contact
                    if base_geom in (contact.geom1, contact.geom2)
                ), (seed, box.name, degrees)
