#!/usr/bin/env python3
"""Offline S63 model selection, Mapper geometry and operating-domain regression."""
import json
from pathlib import Path
import re
import sys
import unittest
import xml.etree.ElementTree as ET
import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
ASSETS = PACKAGE.parent / "kuavo_assets"
sys.path.insert(0, str(PACKAGE / "scripts"))
from tools.full_model_contract import require_full_model
from tools.target_geometry_contract import neutral_retarget_geometry, model_geometry


class S63ModelTest(unittest.TestCase):
    def setUp(self):
        self.overlay = json.loads((PACKAGE / "config/model_s63.json").read_text())
        self.cfg = json.loads((ASSETS / "config/kuavo_v63/kuavo.json").read_text())
        self.cfg.update(self.overlay)
        self.urdf = ASSETS / "models" / self.overlay["velocity_ik_urdf"]

    def test_full_model_selection(self):
        require_full_model(63, str(ASSETS), self.overlay)
        self.assertEqual(self.urdf, ASSETS / "models/biped_s63/urdf/biped_s63.urdf")
        for relative in ("biped_s62/urdf/biped_s62.urdf",
                         "biped_s63/urdf/drake/biped_v3_arm.urdf", None):
            with self.assertRaises(ValueError):
                require_full_model(63, str(ASSETS), {"velocity_ik_urdf": relative})
        with self.assertRaises(FileNotFoundError):
            require_full_model(63, str(ASSETS), {"velocity_ik_urdf": "missing.urdf"})
        with self.assertRaises(ValueError):
            require_full_model(62, "", {})

    def test_mapper_uses_s63_neutral_geometry(self):
        geom = neutral_retarget_geometry(self.urdf, "base_link",
                self.cfg["shoulder_frame_names"], self.cfg["end_frames_name_ik"][3:5])
        np.testing.assert_allclose(geom["left_shoulder"], [.11075, .253, 1.19601], atol=1e-12, rtol=0)
        np.testing.assert_allclose(geom["right_shoulder"], [.11075, -.253, 1.19601], atol=1e-12, rtol=0)
        self.assertAlmostEqual(geom["upper_arm_length"], np.hypot(.02, .2837), 12)
        self.assertNotEqual(geom["shoulder_width"], .2527)

    def test_tool_transform_once(self):
        geom = model_geometry(self.urdf, self.cfg["end_frames_name_ik"][1:3],
                              {side: np.eye(4) for side in ("Left", "Right")})
        for side in ("Left", "Right"):
            self.assertAlmostEqual(geom["arms"][side]["joint7_to_tcp"][2][3], -.17, 12)

    def test_reduction_matches_wbc_s63(self):
        task = PACKAGE.parent / "humanoid-wheel-control/humanoid_wheel_interface/config/kuavo_s63/task.info"
        block = task.read_text().split("removeJoints {")[1].split("}")[0]
        self.assertEqual(re.findall(r'"([^"]+)"', block), self.overlay["velocity_ik_fixed_joints"])
        joints = {j.get("name"): j for j in ET.parse(self.urdf).getroot().findall("joint")}
        active = [n for n,j in joints.items() if j.get("type") != "fixed"
                  and n not in self.overlay["velocity_ik_fixed_joints"]]
        self.assertEqual(len(active), 18)
        self.assertTrue(all(joints[n].get("type") == "revolute" for n in active))

    def test_s63_operating_intersection(self):
        # Historical S63 limits are evidence only; production never loads this model.
        historical = ASSETS / "models/biped_s63/urdf/drake/biped_v3_arm.urdf"
        full, old = [{j.get("name"): j for j in ET.parse(path).getroot().findall("joint")}
                     for path in (self.urdf, historical)]
        for name,joint in full.items():
            if joint.get("type") != "revolute" or name in self.overlay["velocity_ik_fixed_joints"]:
                continue
            bounds = [float(joint.find("limit").get(k)) for k in ("lower", "upper")]
            prior = [float(old[name].find("limit").get(k)) for k in ("lower", "upper")]
            cap = self.overlay["velocity_ik_position_limits"].get(name, bounds)
            self.assertEqual([max(bounds[0], cap[0]), min(bounds[1], cap[1])],
                             [max(bounds[0], prior[0]), min(bounds[1], prior[1])], name)
        self.assertNotIn("knee_joint", self.overlay["velocity_ik_position_limits"])


if __name__ == "__main__":
    unittest.main()
