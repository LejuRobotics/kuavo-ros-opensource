"""Offline checks; no ROS master, camera, or robot connection is needed."""
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
import xml.etree.ElementTree as ET

import numpy as np

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
import cam_proj_validation as projection
from plot_board_error_from_csv import fk_base_to_tip, find_chain_joint_names

try:
    import cv2
except ImportError:
    cv2 = None


class ProjectionOfflineTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.layout = projection.JointLayout("56")
        cls.urdf = HERE / "output" / "urdf_final.urdf"
        cls.model = projection.ProjectionModel(cls.urdf, points_per_link=128)
        cls.q = cls.layout.decode(np.linspace(-0.25, 0.35, cls.layout.count))

    def test_s56_feedback_mapping_and_invalid_feedback(self):
        self.assertEqual(self.layout.count, 29)
        for side, start in (("l", 13), ("r", 20)):
            self.assertEqual([self.layout.indices[f"zarm_{side}{i}_joint"]
                              for i in range(1, 8)], list(range(start, start + 7)))
        self.assertEqual(self.layout.indices["waist_yaw_joint"], 12)
        self.assertEqual(self.layout.indices["zhead_1_joint"], 27)
        self.assertEqual(self.layout.indices["zhead_2_joint"], 28)
        decoded = self.layout.decode(np.arange(29, dtype=float))
        self.assertEqual(decoded["zarm_l1_joint"], 13.0)
        self.assertEqual(decoded["zhead_2_joint"], 28.0)
        for values in (np.zeros(28), np.full(29, np.nan)):
            with self.assertRaises(ValueError):
                self.layout.decode(values)

    def test_cached_fk_matches_existing_fk_for_camera_and_both_arms(self):
        poses = self.model.fk(self.q)
        for name in [self.model.camera_link] + [n for n, _ in self.model.clouds] + self.model.tips:
            chain = find_chain_joint_names(self.model.joints, "base_link", name)
            expected = fk_base_to_tip(self.model.joints, chain, self.q)
            np.testing.assert_allclose(poses[name], expected, atol=1e-12)
        missing = dict(self.q)
        del missing["zhead_2_joint"]
        with self.assertRaisesRegex(ValueError, "zhead_2_joint"):
            self.model.fk(missing)

    def test_candidate_camera_and_arm_origins_are_used_independently(self):
        original = self.model.fk(self.q)
        for joint_name, affected, unchanged in (
            ("zarm_l1_joint", "zarm_l7_link", "head_camera_base"),
            ("head_camera_base_joint", "head_camera_base", "zarm_l7_link"),
        ):
            tree = ET.parse(self.urdf)
            origin = tree.find(f"./joint[@name='{joint_name}']/origin")
            xyz = np.fromstring(origin.get("xyz"), sep=" ")
            xyz[0] += 0.02
            origin.set("xyz", " ".join(map(str, xyz)))
            with tempfile.TemporaryDirectory() as directory:
                candidate = Path(directory) / "candidate.urdf"
                tree.write(candidate)
                changed = projection.ProjectionModel(candidate, points_per_link=8).fk(self.q)
            self.assertAlmostEqual(np.linalg.norm(changed[affected][:3, 3]
                                                  - original[affected][:3, 3]), 0.02)
            np.testing.assert_allclose(changed[unchanged], original[unchanged], atol=1e-12)

    def test_real_mesh_cache_is_bounded(self):
        self.assertEqual(len(self.model.clouds), 14)
        for name, points in self.model.clouds:
            self.assertGreater(len(points), 0, name)
            self.assertLessEqual(len(points), 128, name)
            self.assertEqual(points.shape[1], 3)
            self.assertTrue(np.isfinite(points).all(), name)
        self.assertLessEqual(sum(len(p) for _, p in self.model.clouds), 14 * 128)

    def test_static_transform_composition_inverse_and_disconnected_frames(self):
        transforms = projection.StaticTransforms()
        parent_middle = np.eye(4)
        parent_middle[:3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
        parent_middle[:3, 3] = [1, 2, 3]
        middle_child = np.eye(4)
        middle_child[:3, 3] = [0.2, 0.3, 0.4]
        transforms.update("/anchor", "/middle", parent_middle)
        transforms.update("middle", "optical", middle_child)
        expected = parent_middle @ middle_child
        np.testing.assert_allclose(transforms.lookup("anchor", "optical"), expected)
        np.testing.assert_allclose(transforms.lookup("/optical", "/anchor"),
                                   np.linalg.inv(expected), atol=1e-12)
        np.testing.assert_allclose(transforms.lookup("anchor", "anchor"), np.eye(4))
        with self.assertRaisesRegex(ValueError, "No fixed"):
            transforms.lookup("anchor", "unconnected")

    def test_joint_history_nearest_bounded_skew_and_clock_reset(self):
        history = projection.JointHistory(size=2)
        with self.assertRaises(ValueError):
            history.nearest(1.0, 0.1)
        for stamp in (1.0, 1.1, 1.2):
            history.append(stamp, {"q": stamp})
        self.assertEqual(len(history.samples), 2)
        value, stamp, skew = history.nearest(1.18, 0.03)
        self.assertEqual(value, {"q": 1.2})
        self.assertEqual(stamp, 1.2)
        self.assertAlmostEqual(skew, 0.02)
        with self.assertRaisesRegex(ValueError, "mismatch"):
            history.nearest(1.5, 0.03)
        history.append(0.2, {"q": 7})
        self.assertEqual(list(history.samples), [(0.2, {"q": 7})])

    @unittest.skipUnless(cv2 is not None, "OpenCV is required for pixel projection")
    def test_camera_info_must_match_raw_image_geometry(self):
        info = SimpleNamespace(
            header=SimpleNamespace(frame_id="camera_optical"), height=480, width=640,
            K=[100., 0., 320., 0., 120., 240., 0., 0., 1.], D=[0.] * 5,
            distortion_model="plumb_bob", binning_x=0, binning_y=0,
            roi=SimpleNamespace(x_offset=0, y_offset=0, width=0, height=0))
        frame = np.zeros((480, 640, 3), np.uint8)
        K, D, model = projection.camera_parameters(info, frame, "camera_optical")
        self.assertEqual(K.shape, (3, 3))
        self.assertEqual(D.shape, (5,))
        self.assertEqual(model, "plumb_bob")
        with self.assertRaisesRegex(ValueError, "frame_id"):
            projection.camera_parameters(info, frame, "other_camera")
        with self.assertRaisesRegex(ValueError, "sizes"):
            projection.camera_parameters(info, frame[:200], "camera_optical")
        info.roi.width = 320
        with self.assertRaisesRegex(ValueError, "cropped"):
            projection.camera_parameters(info, frame, "camera_optical")
        info.roi.width = 0
        info.K[0] = 0
        with self.assertRaisesRegex(ValueError, "uncalibrated"):
            projection.camera_parameters(info, frame, "camera_optical")

    @unittest.skipUnless(cv2 is not None, "OpenCV is required for pixel projection")
    def test_known_pixels_and_behind_camera_rejection(self):
        K = np.array([[100., 0., 320.], [0., 120., 240.], [0., 0., 1.]])
        xyz = np.array([[0., 0., 2.], [0.2, -0.2, 2.],
                        [0., 0., -1.], [0., 0., 0.], [100., 0., 1.],
                        [np.nan, 0., 1.]])
        pixels = projection.project_points(xyz, K, np.zeros(5), "plumb_bob", 640, 480)
        np.testing.assert_array_equal(pixels, [[320, 240], [330, 228]])
        empty = projection.project_points(xyz[2:], K, np.zeros(5), "plumb_bob", 640, 480)
        self.assertEqual(empty.shape, (0, 2))


if __name__ == "__main__":
    unittest.main()
