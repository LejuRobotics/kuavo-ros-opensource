"""S63 directory synchronization preserves model roles and calibrated arm FK."""

import hashlib
from pathlib import Path
import shutil
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation


CALIB = Path(__file__).resolve().parents[1]
REPO = CALIB.parents[2]
sys.path.insert(0, str(CALIB))
import urdf_model_sync as sync


def fk(data, side, positions):
    root = ET.fromstring(data)
    result = np.eye(4)
    for i, position in enumerate(positions, 1):
        joint = root.find("joint[@name='zarm_%s%d_joint']" % (side, i))
        origin = joint.find("origin")
        transform = np.eye(4)
        transform[:3, :3] = Rotation.from_euler("xyz", np.fromstring(origin.get("rpy"), sep=" ")).as_matrix()
        transform[:3, 3] = np.fromstring(origin.get("xyz"), sep=" ")
        motion = np.eye(4)
        motion[:3, :3] = Rotation.from_rotvec(np.fromstring(joint.find("axis").get("xyz"), sep=" ") * position).as_matrix()
        result = result @ transform @ motion
    return result


def without_arm_origins_and_metadata(data):
    data = sync._METADATA.sub(b"", data)
    for start, end in sorted(sync._origin_spans(data).values(), reverse=True):
        data = data[:start] + data[end:]
    # Inserting a metadata comment introduces exactly one newline + indent.
    return data.replace(b">\n  \n", b">\n", 1)


class ModelSyncTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="mocap_model_sync_")
        self.addCleanup(self.temporary.cleanup)
        self.directory = Path(self.temporary.name) / "urdf"
        shutil.copytree(REPO / "src/kuavo_assets/models/biped_s63/urdf", self.directory)
        # Build an independent candidate; tests do not require a saved lab fit.
        self.candidate = Path(self.temporary.name) / "candidate.urdf"
        tree = ET.parse(self.directory / "biped_s63.urdf")
        root = tree.getroot()
        root.insert(0, ET.Comment(' mocap_kin_calib:{"schema_version":1} '))
        for side, sign in (("l", 1), ("r", -1)):
            for i in range(1, 8):
                origin = root.find("joint[@name='zarm_%s%d_joint']/origin" % (side, i))
                xyz = np.fromstring(origin.get("xyz"), sep=" ")
                xyz += sign * np.array([0.0003, -0.0002, 0.0001]) * i
                origin.set("xyz", " ".join(map(str, xyz)))
                origin.set("rpy", " ".join(map(str, sign * np.array([0.001, -0.002, 0.0015]) * i)))
        tree.write(self.candidate, encoding="utf-8", xml_declaration=True)

    def build(self, **kwargs):
        return sync.build_model_updates(self.directory, self.candidate, **kwargs)

    def test_all_models_share_candidate_torso_to_j7_fk(self):
        updates = self.build()
        candidate = self.candidate.read_bytes()
        poses = [np.zeros(7), np.array([0.3, -0.8, 0.4, 1.2, -0.6, 0.2, 0.5])]
        poses.extend(np.random.default_rng(63).uniform(-1.2, 1.2, (20, 7)))
        for path, data in updates.items():
            if path.suffix != ".urdf":
                continue
            sides = "l" if path.name == "Larm.urdf" else "r" if path.name == "Rarm.urdf" else "lr"
            for side in sides:
                for positions in poses:
                    with self.subTest(model=path.name, side=side):
                        np.testing.assert_allclose(fk(data, side, positions), fk(candidate, side, positions), atol=2e-14, rtol=0)

    def test_no_writes_and_non_arm_content_is_preserved(self):
        before = {p: hashlib.sha256(p.read_bytes()).digest() for p in self.directory.rglob("*") if p.is_file()}
        updates = self.build()
        self.assertEqual(set(updates), {self.directory / p for p in sync._KNOWN_MODELS})
        self.assertEqual(updates[self.directory / sync._MAIN], self.candidate.read_bytes())
        self.assertEqual(updates[self.directory / sync._XACRO], (self.directory / sync._XACRO).read_bytes())
        for relative in sync._VARIANTS:
            path = self.directory / relative
            self.assertEqual(without_arm_origins_and_metadata(updates[path]), without_arm_origins_and_metadata(path.read_bytes()))
        self.assertEqual(before, {p: hashlib.sha256(p.read_bytes()).digest() for p in before})

    def test_legacy_arms_are_complete_standard_s63_models(self):
        candidate = ET.parse(self.candidate).getroot()
        updates = self.build()
        for relative, side in sync._SINGLE_ARMS.items():
            data = updates[self.directory / relative]
            root = sync._parse(data, relative)
            links, joints = sync._validate_tree(root, relative)
            self.assertEqual(len(links), 8)
            self.assertEqual(len(joints), 7)
            self.assertIn("waist_yaw_link", links)
            self.assertIsNone(root.find("mujoco"))
            self.assertIsNone(root.find("transmission"))
            for i in range(1, 8):
                for tag, name in (("link", "zarm_%s%d_link" % (side, i)), ("joint", "zarm_%s%d_joint" % (side, i))):
                    expected = candidate.find("%s[@name='%s']" % (tag, name))
                    actual = root.find("%s[@name='%s']" % (tag, name))
                    self.assertEqual(ET.tostring(expected).strip(), ET.tostring(actual).strip())
            self.assertEqual(sync._metadata_comment(data, relative), sync._metadata_comment(self.candidate.read_bytes(), self.candidate))

    def test_explicit_legacy_opt_out_preserves_fragments(self):
        updates = self.build(rebuild_legacy_arms=False)
        for relative in sync._SINGLE_ARMS:
            path = self.directory / relative
            self.assertEqual(updates[path], path.read_bytes())

    def test_nominal_is_not_runtime_and_unexpected_models_fail(self):
        nominal = self.directory / "biped_s63.nominal.urdf"
        nominal.write_bytes(b"stable original model")
        self.assertNotIn(nominal, self.build())
        extra = self.directory / "drake/surprise.urdf"
        extra.write_bytes(b"<robot/>")
        with self.assertRaisesRegex(ValueError, "drake/surprise.urdf"):
            self.build()

    def test_incompatible_axis_and_duplicate_joint_fail_closed(self):
        path = self.directory / "drake/biped_v3_full.urdf"
        original = path.read_bytes()
        root = ET.fromstring(original)
        joint = root.find("joint[@name='zarm_l5_joint']")
        joint.find("axis").set("xyz", "1 0 0")
        path.write_bytes(ET.tostring(root))
        with self.assertRaisesRegex(ValueError, "incompatible axis.*zarm_l5_joint"):
            self.build()
        root = ET.fromstring(original)
        root.append(root.find("joint[@name='zarm_l5_joint']"))
        path.write_bytes(ET.tostring(root))
        with self.assertRaisesRegex(ValueError, "duplicate joint"):
            self.build()

    def test_second_sync_replaces_metadata_and_is_idempotent(self):
        first = self.build()
        for path, data in first.items():
            path.write_bytes(data)
        second = sync.build_model_updates(self.directory / "biped_s63.urdf", self.candidate)
        self.assertEqual(first, second)


if __name__ == "__main__":
    unittest.main()
