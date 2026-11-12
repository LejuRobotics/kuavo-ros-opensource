#!/usr/bin/env python3
"""Validate SG100 V4 asset integration without starting robot hardware.

Run: python3 -m unittest discover -s src/kuavo_assets/tests -v
MuJoCo checks run when its Python bindings and NumPy are available.
"""
import hashlib
import json
import os
from pathlib import Path
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

ASSETS = Path(__file__).resolve().parents[1]
MODELS = ASSETS / 'models'
V4 = MODELS / 'heiman_sg100' / 'v4'
VERSIONS = ('410062', '410063')


def hand(side):
    label = 'left' if side == 'l' else 'right'
    return ET.fromstring('<robot>' + (V4 / 'urdf' / f'heiman_{label}_hand.urdf').read_text() + '</robot>')


def values(element):
    """Compare XML semantics independently of exporter whitespace."""
    return (element.tag, dict(element.attrib), [values(child) for child in element])


def numbers(text):
    return [float(value) for value in text.split()]


class HeimanV4ModelsTest(unittest.TestCase):
    def test_imported_meshes_and_vendor_mass(self):
        manifest = json.loads((V4 / 'source_manifest.json').read_text())
        for side, expected_mass in [('l', 0.2808614), ('r', 0.28133254558633874)]:
            root = hand(side)
            self.assertEqual(len(root.findall('link')), 12)
            mass = sum(float(link.find('inertial/mass').get('value')) for link in root.findall('link'))
            self.assertAlmostEqual(mass, expected_mass, places=12)
            for filename, digest in manifest[side]['meshes_sha256'].items():
                self.assertEqual(hashlib.sha256((V4 / 'meshes' / filename).read_bytes()).hexdigest(), digest)
        self.assertEqual(hand('l').find('joint[@name="l_thumb_j1"]/origin').get('xyz'),
                         '-0.0349576946890685 -0.0204910969115199 0.0221499999999999')
        self.assertEqual(hand('r').find('joint[@name="r_thumb_j2"]/limit').get('lower'), '-2.18')

    def test_hand_parameters_in_every_urdf(self):
        for version in VERSIONS:
            for path in (MODELS / f'biped_s{version}' / 'urdf').rglob('*.urdf'):
                root = ET.parse(path).getroot()
                old = ET.parse(str(path).replace(version, version.replace('4100', '4000'))).getroot()
                for side in ('l', 'r'):
                    for expected in hand(side):
                        actual = root.find(f'{expected.tag}[@name="{expected.get("name")}"]')
                        if actual is None:
                            self.assertIn(path.name, ('Larm.urdf', 'Rarm.urdf'))
                            continue
                        with self.subTest(version=version, file=path.name, name=actual.get('name')):
                            previous = old.find(f'{expected.tag}[@name="{expected.get("name")}"]')
                            if expected.get('name') == f'{side}_hand_base_joint':
                                self.assertEqual(values(actual), values(previous))
                            elif expected.tag == 'link':
                                self.assertEqual(values(actual.find('inertial')), values(expected.find('inertial')))
                                self.assertEqual(values(actual.find('visual/origin')), values(expected.find('visual/origin')))
                                self.assertEqual(Path(actual.find('visual/geometry/mesh').get('filename')).name,
                                                 Path(expected.find('visual/geometry/mesh').get('filename')).name)
                            else:
                                self.assertEqual(actual.get('type'), previous.get('type'))
                                for tag in ('origin', 'axis', 'parent', 'child', 'limit'):
                                    self.assertEqual(values(actual.find(tag)), values(expected.find(tag)))

    def test_tree_resources_and_body_parameters(self):
        for version in VERSIONS:
            for path in (MODELS / f'biped_s{version}' / 'urdf').rglob('*.urdf'):
                root = ET.parse(path).getroot()
                old = ET.parse(str(path).replace(version, version.replace('4100', '4000'))).getroot()
                links = [link.get('name') for link in root.findall('link')]
                children = [joint.find('child').get('link') for joint in root.findall('joint')]
                self.assertEqual(len(links), len(set(links)))
                self.assertEqual(len(children), len(set(children)))
                if path.name not in ('Larm.urdf', 'Rarm.urdf'):
                    self.assertEqual(len(set(links) - set(children)), 1, str(path))
                self.assertEqual(set(children) - set(links), set())
                self.assertEqual([j.get('name') for j in root.findall('joint')],
                                 [j.get('name') for j in old.findall('joint')])
                hand_names = {el.get('name') for s in ('l', 'r') for el in hand(s)}
                for el in root:
                    if el.tag not in ('link', 'joint') or el.get('name') in hand_names:
                        continue
                    prev = old.find(f'{el.tag}[@name="{el.get("name")}"]')
                    if el.tag == 'joint':
                        self.assertEqual(values(el), values(prev))
                    elif el.find('inertial') is not None:
                        self.assertEqual(values(el.find('inertial')), values(prev.find('inertial')))
                for mesh in root.findall('.//mesh'):
                    filename = mesh.get('filename')
                    resolved = ASSETS.parent / filename[len('package://'):] if filename.startswith('package://') else path.parent / filename
                    self.assertTrue(resolved.is_file(), str(resolved))
            xml = MODELS / f'biped_s{version}' / 'xml' / f'biped_s{version}.xml'
            root = ET.parse(xml).getroot()
            meshdir = xml.parent / root.find('compiler').get('meshdir')
            for mesh in root.findall('asset/mesh'):
                self.assertTrue((meshdir / mesh.get('file')).is_file())
            config = json.loads((ASSETS / 'config' / f'kuavo_v{version}' / 'kuavo.json').read_text())
            self.assertTrue((MODELS / config['arm_urdf']).is_file())

    def test_wheel_launch_keeps_explicit_model_version(self):
        try:
            import roslaunch
            import rospkg
            directory = Path(rospkg.RosPack().get_path('humanoid_controllers')) / 'launch'
        except ImportError as error:
            self.skipTest('Source the ROS workspace: ' + str(error))
        except rospkg.ResourceNotFound as error:
            self.skipTest('Source the ROS workspace: ' + str(error))
        for version in VERSIONS:
            for launch in ('load_kuavo_real_wheel.launch', 'load_kuavo_mujoco_sim_wheel.launch'):
                with self.subTest(version=version, launch=launch), patch.dict(os.environ, ROBOT_VERSION='17'):
                    config = roslaunch.config.ROSLaunchConfig()
                    roslaunch.xmlloader.XmlLoader().load(str(directory / launch), config,
                                                       argv=['robot_version:=' + version], verbose=False)
                    self.assertEqual(config.params['/robot_version'].value, int(version))
                    self.assertIn('biped_s' + version, config.params['/urdfFile'].value)
                    self.assertIn('kuavo_s' + version + '/task.info', config.params['/taskFile'].value)
                    self.assertIn('heiman_sg100/v4/meshes/', config.params['/humanoid_description'].value)

    def test_mujoco_kinematics_and_inertia(self):
        try:
            import mujoco
            import numpy as np
        except ImportError:
            self.skipTest('MuJoCo Python bindings and NumPy are required')

        def rotation(rpy):
            roll, pitch, yaw = numbers(rpy)
            sr, sp, sy = np.sin([roll, pitch, yaw])
            cr, cp, cy = np.cos([roll, pitch, yaw])
            return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                             [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                             [-sp, cp*sr, cp*cr]])

        for version in VERSIONS:
            model = mujoco.MjModel.from_xml_path(str(MODELS / f'biped_s{version}' / 'xml' / f'biped_s{version}.xml'))
            data = mujoco.MjData(model)
            for moving in (False, True):
                data.qpos[:] = model.qpos0
                for side in ('l', 'r'):
                    expected_poses = {f'{side}_hand_base': (np.zeros(3), np.eye(3))}
                    for joint in hand(side).findall('joint'):
                        if joint.get('type') == 'fixed':
                            continue
                        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint.get('name'))
                        self.assertGreaterEqual(jid, 0)
                        limits = joint.find('limit')
                        angle = 0.35 * (float(limits.get('upper')) + float(limits.get('lower'))) if moving else 0.0
                        data.qpos[model.jnt_qposadr[jid]] = angle
                        parent_pos, parent_rot = expected_poses[joint.find('parent').get('link')]
                        origin = joint.find('origin')
                        pos = parent_pos + parent_rot @ numbers(origin.get('xyz'))
                        axis = np.array(numbers(joint.find('axis').get('xyz')))
                        skew = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
                        turn = np.eye(3) + np.sin(angle)*skew + (1-np.cos(angle))*(skew @ skew)
                        expected_poses[joint.find('child').get('link')] = (pos, parent_rot @ rotation(origin.get('rpy')) @ turn)
                    mujoco.mj_forward(model, data)
                    base = model.body(f'{side}_hand_base').id
                    base_rot = data.xmat[base].reshape(3, 3)
                    for name, (pos, rot) in expected_poses.items():
                        bid = model.body(name).id
                        np.testing.assert_allclose(base_rot.T @ (data.xpos[bid]-data.xpos[base]), pos, atol=1e-10)
                        np.testing.assert_allclose(base_rot.T @ data.xmat[bid].reshape(3, 3), rot, atol=1e-10)
                        source = hand(side).find(f'link[@name="{name}"]/inertial')
                        self.assertAlmostEqual(model.body_mass[bid], float(source.find('mass').get('value')), places=12)
                        inertia = source.find('inertia')
                        expected = np.array([[float(inertia.get(f'i{a}{b}' if a <= b else f'i{b}{a}')) for b in 'xyz'] for a in 'xyz'])
                        matrix = np.zeros(9)
                        mujoco.mju_quat2Mat(matrix, model.body_iquat[bid])
                        mat = matrix.reshape(3, 3)
                        np.testing.assert_allclose(mat @ np.diag(model.body_inertia[bid]) @ mat.T, expected, atol=1e-10)
            mujoco.mj_step(model, data, nstep=10)
            self.assertTrue(np.isfinite(data.qpos).all())


if __name__ == '__main__':
    unittest.main()
