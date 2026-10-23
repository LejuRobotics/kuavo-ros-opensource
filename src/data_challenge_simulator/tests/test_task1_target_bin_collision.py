"""Dynamic regression for cylinders becoming embedded in the target bin."""
import copy
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

SCENE = Path(__file__).resolve().parents[1] / "models/biped_s400062/xml/task1.xml"


def target_bin_model():
    source = ET.parse(SCENE).getroot()
    root = ET.Element("mujoco")
    root.append(copy.deepcopy(source.find("option")))
    root.append(copy.deepcopy(source.find("default")))
    assets = ET.SubElement(root, "asset")
    assets.append(copy.deepcopy(source.find("asset/mesh[@name='task1_cylinder_collision']")))
    world = ET.SubElement(root, "worldbody")
    for name in ("target_bin", "cylinder_1"):
        body = copy.deepcopy(source.find(".//body[@name='%s']" % name))
        for geom in body.findall(".//geom"):
            geom.attrib.pop("material", None)
        world.append(body)
    return mujoco.MjModel.from_xml_string(ET.tostring(root).decode())


def test_random_oriented_drops_do_not_remain_embedded_in_bin():
    model = target_bin_model()
    rng = np.random.default_rng(17)
    for case in range(30):
        data = mujoco.MjData(model)
        data.qpos[:3] = [rng.uniform(0.2375, 0.3775),
                         rng.uniform(-0.5552, -0.5008), 0.82]
        quaternion = rng.normal(size=4)
        data.qpos[3:7] = quaternion / np.linalg.norm(quaternion)
        for _ in range(1000):
            mujoco.mj_step(model, data)
        depth = min((contact.dist for contact in data.contact), default=0.0)
        assert depth > -0.001, (case, depth)
        x, y, z = data.qpos[:3]
        assert 0.2 < x < 0.415 and -0.6 < y < -0.46 and 0.515 < z < 0.61, (
            case, data.qpos[:3])


def test_near_source_wall_spawn_settles_without_floor_penetration():
    source = ET.parse(SCENE).getroot()
    root = ET.Element("mujoco")
    root.append(copy.deepcopy(source.find("option")))
    root.append(copy.deepcopy(source.find("default")))
    assets = ET.SubElement(root, "asset")
    assets.append(copy.deepcopy(source.find("asset/mesh[@name='task1_cylinder_collision']")))
    world = ET.SubElement(root, "worldbody")
    for name in ("source_bin", "cylinder_3"):
        body = copy.deepcopy(source.find(".//body[@name='%s']" % name))
        if name == "source_bin":
            body.set("pos", "0.62 -0.196 0.65")
            body.attrib.pop("gravcomp")
            body.remove(body.find("joint"))
        for geom in body.findall("geom"):
            geom.attrib.pop("material", None)
        world.append(body)
    model = mujoco.MjModel.from_xml_string(ET.tostring(root).decode())
    data = mujoco.MjData(model)
    # Seed 3's original spawn overlaps both the wall and floor slightly.
    data.qpos[:] = [0.629341, -0.020399, 0.685,
                    0.90515438, 0.0, 0.0, -0.425082986]
    for _ in range(1000):
        mujoco.mj_step(model, data)
    assert min(contact.dist for contact in data.contact) > -0.0001
    assert abs(data.qpos[2] - 0.686) < 0.0001


def test_embedded_target_corner_pose_recovers_under_contact_forces():
    model = target_bin_model()
    data = mujoco.MjData(model)
    # Real seed-4 release: no-slip post-processing previously held this
    # tilted cylinder 3.23 mm inside the bin indefinitely.
    data.qpos[:] = [0.32133106264, -0.5017920564, 0.5449638347,
                    0.17652349, -0.34426549, -0.17830285, -0.90472583]
    for _ in range(1000):
        mujoco.mj_step(model, data)
    assert min(contact.dist for contact in data.contact) > -0.0001
    assert 0.535 < data.qpos[2] < 0.55
