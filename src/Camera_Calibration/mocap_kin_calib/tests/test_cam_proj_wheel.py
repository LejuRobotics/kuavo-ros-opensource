"""Wheel-arm projection checks without ROS, network access, or motion commands."""
import os
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
import cam_proj_validation as p
from plot_board_error_from_csv import find_chain_joint_names, fk_base_to_tip

try:
    import cv2
except ImportError:
    cv2 = None


class WheelProjectionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.urdf = p.CALIB_ROOT.parent / 'kuavo_assets/models/biped_s63/urdf/biped_s63.urdf'
        cls.model = p.ProjectionModel(cls.urdf, points_per_link=32, root_link='waist_yaw_link')
        cls.base_model = p.ProjectionModel(cls.urdf, points_per_link=32)
        cls.prefix = ('knee_joint', 'leg_joint', 'waist_pitch_joint', 'waist_yaw_joint')
        cls.optical = p.pose_matrix([.011715, .0237, .014311, -.5, .5, -.5, .5])
        cls.K = np.array([[610., 0., 640.], [0., 610., 360.], [0., 0., 1.]])

    def pose(self):
        q = p.JointLayout('62').decode(np.zeros(20))
        for side in ('l', 'r'):
            q[f'zarm_{side}1_joint'] = -.8
            q[f'zarm_{side}4_joint'] = -.7
        q['zhead_2_joint'] = .4
        return q

    def test_default_model_and_explicit_candidate_keep_feedback_version(self):
        for version in ('62', '63'):
            with self.subTest(version=version), patch.dict(os.environ, ROBOT_VERSION=version):
                args = p.parse_args([])
                self.assertEqual(args.urdf, self.urdf)
                self.assertEqual(args.robot_version, version)
                self.assertEqual(p.JointLayout(args.robot_version).path.parent.name,
                                 f'kuavo_v{version}')
                candidate = Path('/tmp/user_candidate.urdf')
                args = p.parse_args(['--urdf', str(candidate)])
                self.assertEqual(args.urdf, candidate)
                self.assertEqual(args.robot_version, version)
        with patch.dict(os.environ, ROBOT_VERSION='63'):
            args = p.parse_args(['--robot-version', '56'])
            self.assertEqual(args.robot_version, '56')
            self.assertEqual(args.urdf, HERE / 'output/urdf_final.urdf')

    def test_full_feedback_maps_sixteen_required_arm_and_head_joints(self):
        expected = {f'zarm_{side}{i}_joint': start + i - 1
                    for side, start in (('l', 4), ('r', 11)) for i in range(1, 8)}
        expected.update(zhead_1_joint=18, zhead_2_joint=19)
        self.assertEqual(self.model.required_joints, set(expected))
        self.assertTrue(set(self.prefix).isdisjoint(self.model.required_joints))
        for version in ('62', '63'):
            with self.subTest(version=version):
                layout = p.JointLayout(version)
                self.assertEqual(layout.count, 20)
                self.assertEqual(layout.indices, expected)
                q = layout.decode(np.arange(20, dtype=float))
                self.assertEqual(q, {name: float(index) for name, index in expected.items()})
                self.model.fk(q)
                invalid_prefix = np.zeros(20)
                invalid_prefix[0] = np.nan
                for values in (np.zeros(16), np.zeros(19), np.zeros(21), invalid_prefix):
                    with self.assertRaises(ValueError):
                        layout.decode(values)
        with self.assertRaisesRegex(ValueError, 'Missing measured joints'):
            self.base_model.fk(self.pose())

    def test_relative_pose_matches_independent_full_base_chain(self):
        rng = np.random.default_rng(62)
        for _ in range(10):
            q = {name: value + rng.uniform(-.15, .15) for name, value in self.pose().items()}
            full_q = dict(q, **dict(zip(self.prefix, rng.uniform(.1, .5, 4))))
            reduced = self.model.fk(q)
            camera_chain = find_chain_joint_names(self.model.joints, 'base_link', self.model.camera_link)
            full_camera = fk_base_to_tip(self.model.joints, camera_chain, full_q) @ self.optical
            for name, _ in self.model.clouds:
                chain = find_chain_joint_names(self.model.joints, 'base_link', name)
                expected = np.linalg.inv(full_camera) @ fk_base_to_tip(self.model.joints, chain, full_q)
                actual = np.linalg.inv(reduced[self.model.camera_link] @ self.optical) @ reduced[name]
                np.testing.assert_allclose(actual, expected, rtol=0, atol=1e-12)

    @unittest.skipUnless(cv2 is not None, 'OpenCV is required for pixel projection')
    def test_mesh_pixels_match_independent_full_base_chain(self):
        q = self.pose()
        full_q = dict(q, **dict(zip(self.prefix, (.2, -.3, .4, -.5))))
        reduced = self.model.fk(q)
        chain = find_chain_joint_names(self.model.joints, 'base_link', self.model.camera_link)
        full_camera = fk_base_to_tip(self.model.joints, chain, full_q) @ self.optical
        total_visible = 0
        for name, vertices in self.model.clouds:
            chain = find_chain_joint_names(self.model.joints, 'base_link', name)
            reference = np.linalg.inv(full_camera) @ fk_base_to_tip(self.model.joints, chain, full_q)
            actual = np.linalg.inv(reduced[self.model.camera_link] @ self.optical) @ reduced[name]
            pixels = []
            for transform in (reference, actual):
                xyz = vertices @ transform[:3, :3].T + transform[:3, 3]
                pixels.append(p.project_points(xyz, self.K, np.zeros(5), 'plumb_bob', 1280, 720))
            np.testing.assert_array_equal(pixels[0], pixels[1])
            total_visible += len(pixels[0])
        self.assertGreater(total_visible, 0)

    def test_head_and_arm_motion_change_relative_pose(self):
        def relative(q):
            poses = self.model.fk(q)
            return np.linalg.inv(poses[self.model.camera_link]) @ poses['zarm_l7_link']

        q = self.pose()
        original = relative(q)
        for joint in ('zhead_1_joint', 'zhead_2_joint', 'zarm_l2_joint'):
            with self.subTest(joint=joint):
                changed = dict(q)
                changed[joint] += .3
                self.assertGreater(np.linalg.norm(relative(changed) - original), .1)

    @unittest.skipUnless(cv2 is not None, 'OpenCV is required for rendering')
    def test_render_metadata_names_the_actual_fk_root(self):
        q = self.pose()
        full_q = dict(q, **dict(zip(self.prefix, (.2, -.3, .4, -.5))))
        frame = np.zeros((180, 320, 3), dtype=np.uint8)
        K = self.K.copy()
        K[:2] /= 4
        for model, values in ((self.model, q), (self.base_model, full_q)):
            with self.subTest(root=model.root_link):
                _, stats = model.render(frame, values, self.optical, K, np.zeros(5))
                self.assertEqual(stats['fk_root'], model.root_link)
                expected = model.fk(values)[model.camera_link] @ self.optical
                np.testing.assert_allclose(stats['T_root_optical'], expected, rtol=0, atol=1e-12)
                if model.root_link == 'base_link':
                    np.testing.assert_allclose(stats['T_base_optical'], expected, rtol=0, atol=1e-12)
                else:
                    self.assertNotIn('T_base_optical', stats)

    def test_viewer_selects_wheel_root_with_real_model_and_layout(self):
        modules = {
            'rospy': SimpleNamespace(Publisher=Mock(), Subscriber=Mock(), loginfo=Mock()),
            'cv2': SimpleNamespace(setNumThreads=Mock()),
            'cv_bridge': SimpleNamespace(CvBridge=Mock()),
            'kuavo_msgs.msg': SimpleNamespace(sensorsData=object),
            'sensor_msgs.msg': SimpleNamespace(CameraInfo=object, CompressedImage=object, Image=object),
            'tf2_msgs.msg': SimpleNamespace(TFMessage=object),
        }
        for version in ('62', '63'):
            with self.subTest(version=version), patch.dict(sys.modules, modules):
                viewer = p.ProjectionViewer(p.parse_args(['--robot-version', version]))
                self.assertEqual(viewer.model.root_link, 'waist_yaw_link')
                self.assertEqual(viewer.model.path, self.urdf)
                self.assertEqual(viewer.layout.path.parent.name, f'kuavo_v{version}')
                self.assertEqual(viewer.model.required_joints, set(viewer.layout.indices))


if __name__ == '__main__':
    unittest.main()
