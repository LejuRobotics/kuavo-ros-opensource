"""Offline S56 wrist-only mesh checks; never connects to ROS or hardware."""
import hashlib
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
import cam_proj_validation as projection

S56 = HERE.parents[1] / "kuavo_assets" / "models" / "biped_s56"


class NoHandProjectionTests(unittest.TestCase):
    def test_real_s56_hand_is_removed_without_cutting_or_mutating_wrist(self):
        for side, original_count, kept_count, wrist_z in (
            ("l", 10000, 3019, -0.069379),
            ("r", 9999, 2288, -0.067054),
        ):
            with self.subTest(side=side):
                path = S56 / "meshes" / f"{side}_hand_pitch_nohand.STL"
                original_hash = hashlib.sha256(path.read_bytes()).hexdigest()
                vertices, faces = projection.load_stl_mesh(path)
                original_vertices, original_faces = vertices.copy(), faces.copy()
                self.assertEqual(len(faces), original_count)
                self.assertLess(vertices[:, 2].min(), -0.24)

                wrist_vertices, wrist_faces, info = projection.strip_s56_hand_mesh(
                    path, vertices, faces)
                self.assertEqual(len(wrist_faces), kept_count)
                wrist_triangles = wrist_vertices[wrist_faces]
                self.assertAlmostEqual(wrist_triangles[:, :, 2].min(), wrist_z, places=6)
                self.assertGreaterEqual(wrist_triangles[:, :, 2].min(), -0.0696)
                # Each retained triangle must be an intact original CAD face.
                original_triangles = {triangle.tobytes() for triangle in vertices[faces]}
                self.assertTrue(all(triangle.tobytes() in original_triangles
                                    for triangle in wrist_triangles))
                self.assertIsInstance(info, dict)
                self.assertTrue(info)
                np.testing.assert_array_equal(vertices, original_vertices)
                np.testing.assert_array_equal(faces, original_faces)
                self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), original_hash)

    def test_only_seven_arm_links_and_joint_seven_tips_ignore_attached_hands(self):
        tree = ET.parse(S56 / "urdf" / "biped_s56.urdf")
        root = tree.getroot()
        for side in ("l", "r"):
            name = f"test_{side}_attached_hand"
            link = ET.SubElement(root, "link", name=name)
            geometry = ET.SubElement(ET.SubElement(link, "visual"), "geometry")
            ET.SubElement(geometry, "mesh", filename=f"/nonexistent/{name}.STL")
            joint = ET.SubElement(root, "joint", name=f"{name}_joint", type="fixed")
            ET.SubElement(joint, "parent", link=f"zarm_{side}7_link")
            ET.SubElement(joint, "child", link=name)
            ET.SubElement(joint, "origin", xyz="0 0 -0.1", rpy="0 0 0")

        with tempfile.TemporaryDirectory() as directory:
            urdf = Path(directory) / "candidate_with_attached_hands.urdf"
            tree.write(urdf)
            for arms, sides in (("both", ("l", "r")), ("left", ("l",)), ("right", ("r",))):
                with self.subTest(arms=arms):
                    # Loading succeeds only when the nonexistent hand meshes are excluded.
                    model = projection.ProjectionModel(urdf, arms=arms, points_per_link=8)
                    expected = {f"zarm_{side}{i}_link" for side in sides for i in range(1, 8)}
                    self.assertEqual({name for name, _, _ in model.meshes}, expected)
                    self.assertEqual(len(model.meshes), len(expected))
                    self.assertEqual({name for name, _ in model.clouds}, expected)
                    self.assertEqual(model.tips, [f"zarm_{side}7_link" for side in sides])

    def test_same_mesh_name_for_another_robot_is_not_filtered(self):
        source = S56 / "meshes" / "l_hand_pitch_nohand.STL"
        vertices, faces = projection.load_stl_mesh(source)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "biped_other" / "meshes" / source.name
            path.parent.mkdir(parents=True)
            path.write_bytes(source.read_bytes())
            unchanged_vertices, unchanged_faces, info = projection.strip_s56_hand_mesh(
                path, vertices, faces)
            self.assertIs(unchanged_vertices, vertices)
            self.assertIs(unchanged_faces, faces)
            self.assertIsNone(info)

    def test_changed_s56_mesh_is_rejected_before_using_known_geometry_filter(self):
        source = S56 / "meshes" / "l_hand_pitch_nohand.STL"
        vertices, faces = projection.load_stl_mesh(source)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "biped_s56" / "meshes" / source.name
            path.parent.mkdir(parents=True)
            changed = bytearray(source.read_bytes())
            changed[0] ^= 1  # Valid binary STL geometry, but no longer the audited asset.
            path.write_bytes(changed)
            with self.assertRaises(ValueError):
                projection.strip_s56_hand_mesh(path, vertices, faces)


if __name__ == "__main__":
    unittest.main()
