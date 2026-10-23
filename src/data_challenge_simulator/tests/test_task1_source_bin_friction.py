"""The source-bin interior dissipates a cylinder's spin through contact."""
import copy
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

SCENE = Path(__file__).resolve().parents[1] / "models/biped_s400062/xml/task1.xml"


def test_source_bin_contact_stops_spin_without_joint_damping():
    scene = ET.parse(SCENE).getroot()
    root = ET.Element("mujoco")
    for tag in ("option", "default"):
        root.append(copy.deepcopy(scene.find(tag)))
    asset = ET.SubElement(root, "asset")
    asset.append(copy.deepcopy(scene.find("asset/mesh[@name='task1_cylinder_collision']")))
    world = ET.SubElement(root, "worldbody")
    for name in ("source_bin", "cylinder_1"):
        body = copy.deepcopy(scene.find(".//body[@name='%s']" % name))
        if name == "source_bin":
            body.set("pos", "0.62 -0.196 0.65")
            body.attrib.pop("gravcomp")
            body.remove(body.find("joint"))
        for geom in body.findall("geom"):
            geom.attrib.pop("material", None)
        world.append(body)
    model = mujoco.MjModel.from_xml_string(ET.tostring(root).decode())
    # Isolate contact friction: no global damping is allowed to mask failure.
    model.dof_damping[:] = 0
    for geom in world.find("body[@name='source_bin']").findall("geom"):
        index = model.geom(geom.get("name")).id
        assert model.geom_condim[index] == 6
        np.testing.assert_allclose(model.geom_friction[index], [1.2, 0.02, 0.001])
    for y in (-0.196, -0.022):
        data = mujoco.MjData(model)
        data.qpos[:3] = [0.62, y, 0.686]
        for _ in range(250):
            mujoco.mj_step(model, data)
        assert data.ncon > 0
        assert all(c.dim == 6 and c.friction[2] >= 0.02 for c in data.contact)
        data.qvel[5] = 20.0  # a brief external spin impulse
        for _ in range(250):
            mujoco.mj_step(model, data)
        assert np.linalg.norm(data.qvel[3:]) < 0.05, data.qvel
        assert min(c.dist for c in data.contact) > -0.0001
