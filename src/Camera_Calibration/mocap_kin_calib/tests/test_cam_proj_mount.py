"""Candidate geometry and camera-driver joins; no ROS or hardware required."""
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
import numpy as np

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
import cam_proj_validation as p


class CameraMountTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.path = p.CALIB_ROOT.parent / 'kuavo_assets/models/biped_s56/urdf/biped_s56.urdf'
        cls.model = p.ProjectionModel(cls.path, points_per_link=8)
        cls.wheel_model = p.ProjectionModel(
            p.CALIB_ROOT.parent / 'kuavo_assets/models/biped_s63/urdf/biped_s63.urdf',
            points_per_link=8)
        # Both wheel feedback versions default to the S63 model geometry.
        cls.cases = [('56', cls.model, .014312),
                     ('62', cls.wheel_model, .014311),
                     ('63', cls.wheel_model, .014311)]

    def driver(self):
        graph = p.StaticTransforms()
        graph.update('head_camera_link', 'head_camera_depth_frame', np.eye(4))
        depth_color = p.pose_matrix([-.000032, -.023727, .000126, -.000815, -.000221, -.002421, .999997])
        color_optical = p.pose_matrix([0, 0, 0, -.5, .5, -.5, .5])
        graph.update('head_camera_depth_frame', 'head_camera_color_frame', depth_color)
        graph.update('head_camera_color_frame', 'head_camera_color_optical_frame', color_optical)
        return graph, depth_color @ color_optical

    def test_nominal_mount_keeps_candidate_offset_and_live_factory_extrinsics(self):
        graph, driver_optical = self.driver()
        for version, model, mount_z in self.cases:
            with self.subTest(version=version):
                extrinsics = p.CameraExtrinsics(model, version)
                actual, source = extrinsics.resolve(graph, 'head_camera_color_optical_frame')
                mount = np.eye(4)
                mount[:3, 3] = [.011715, .0475, mount_z]
                np.testing.assert_allclose(actual, mount @ driver_optical, atol=1e-12)
                self.assertIn(f'S{version} nominal', source)
                # The inverse must recover each 3-D point, not merely a translation.
                point = np.array([.12, -.17, .7, 1.])
                np.testing.assert_allclose(p.rigid_inverse(actual) @ (actual @ point), point, atol=1e-12)

    def test_published_mount_replaces_nominal_identity(self):
        graph, driver_optical = self.driver()
        mount_driver = p.pose_matrix([.01, -.02, .03, 0, .05, 0, 1])
        graph.update('head_camera_depth', 'head_camera_link', mount_driver)
        for version, model, _ in self.cases:
            with self.subTest(version=version):
                ext = p.CameraExtrinsics(model, version)
                actual, source = ext.resolve(graph, 'head_camera_color_optical_frame')
                expected = ext.fixed.lookup('head_camera_base', 'head_camera_depth') @ mount_driver @ driver_optical
                np.testing.assert_allclose(actual, expected, atol=1e-12)
                self.assertEqual(source, 'published mount TF')

    def test_candidate_installation_change_is_not_overridden_by_old_static_urdf(self):
        graph, driver_optical = self.driver()
        graph.update('head_camera_base', 'head_camera_depth', np.eye(4))
        tree = ET.parse(self.path)
        origin = tree.find("./joint[@name='head_camera_depth_joint']/origin")
        origin.set('xyz', '.02 .03 .04')
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'candidate.urdf'
            tree.write(path)
            model = p.ProjectionModel(path, points_per_link=8)
            actual, _ = p.CameraExtrinsics(model, '56').resolve(graph, 'head_camera_color_optical_frame')
        expected = np.eye(4)
        expected[:3, 3] = [.02, .03, .04]
        np.testing.assert_allclose(actual, expected @ driver_optical, atol=1e-12)

    def test_unknown_robot_and_missing_driver_tf_fail_explicitly(self):
        graph, _ = self.driver()
        with self.assertRaisesRegex(ValueError, 'published camera mount'):
            p.CameraExtrinsics(self.model, '99').resolve(graph, 'head_camera_color_optical_frame')
        for version, model, _ in self.cases:
            with self.subTest(version=version):
                with self.assertRaisesRegex(ValueError, 'No fixed'):
                    p.CameraExtrinsics(model, version).resolve(p.StaticTransforms(), 'head_camera_color_optical_frame')
                with self.assertRaisesRegex(ValueError, 'published camera mount'):
                    p.CameraExtrinsics(model, version, mount_frame='head_camera_base').resolve(
                        graph, 'head_camera_color_optical_frame')

    def test_complete_optical_override_tf_still_supported(self):
        graph = p.StaticTransforms()
        expected = p.pose_matrix([.01, .02, .03, -.5, .5, -.5, .5])
        graph.update('head_camera_base', 'other_optical', expected)
        actual, source = p.CameraExtrinsics(self.model, '56').resolve(graph, 'other_optical')
        np.testing.assert_allclose(actual, expected, atol=1e-12)
        self.assertIn('/tf_static', source)

    def test_head_motion_changes_relative_arm_pose_and_waist_cancels(self):
        layout = p.JointLayout('56')
        q = layout.decode(np.zeros(29))
        ext, _ = p.CameraExtrinsics(self.model, '56').resolve(self.driver()[0], 'head_camera_color_optical_frame')
        def relative(values):
            pose = self.model.fk(values)
            return p.rigid_inverse(pose[self.model.camera_link] @ ext) @ pose['zarm_l7_link']
        original = relative(q)
        q['waist_yaw_joint'] = .4
        np.testing.assert_allclose(relative(q), original, atol=1e-12)
        q['zhead_2_joint'] = .3
        self.assertGreater(np.linalg.norm(relative(q) - original), .1)


if __name__ == '__main__':
    unittest.main()
