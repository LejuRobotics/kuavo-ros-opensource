#!/usr/bin/env python3
"""Robot reach, rotating tool and analytic velocity regressions (no ROS)."""
import json
import ast
import contextlib
import io
from pathlib import Path
import sys
import types
import unittest
import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / "scripts/tools"))
sys.path.insert(0, str(PACKAGE / "scripts"))
from target_geometry_contract import model_geometry, map_model_forearm, neutral_retarget_geometry, LEGACY, WRIST, MODEL_FOREARM
from pose_velocity_kalman import mapped_rates, exp_so3
# Only ROS package discovery is stubbed; production helper modules stay intact.
tools_package = types.ModuleType('tools')
tools_package.__path__ = [str(PACKAGE / 'scripts/tools')]
sys.modules['tools'] = tools_package
from tools.full_model_contract import resolve_model, require_full_model

def rpy_to_matrix(rpy):
    roll, pitch, yaw = rpy
    return exp_so3([0., 0., yaw]) @ exp_so3([0., pitch, 0.]) @ exp_so3([roll, 0., 0.])

VERSIONS = ('60', '61', '62', '63', '200062', '210062', '300062', '310062', '400062', '400063')


class MappingTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        assets = PACKAGE.parent / "kuavo_assets"
        c = json.loads((assets / "config/kuavo_v62/kuavo.json").read_text())
        c.update(json.loads((PACKAGE / "config/model_s62.json").read_text()))
        urdf = assets / "models" / c["velocity_ik_urdf"]
        cls.model = model_geometry(urdf, c["end_frames_name_ik"][1:3],
                                  {s: np.eye(4) for s in ("Left", "Right")})
        cls.upper = neutral_retarget_geometry(urdf, "base_link", c["shoulder_frame_names"],
                                             c["end_frames_name_ik"][3:5])["upper_arm_length"]

    def test_straight_wrist_and_rotating_tool(self):
        for arm in self.model["arms"].values():
            direction = np.array([1., 0., 0.])
            elbow = self.upper * direction
            for angle in np.linspace(-np.pi, np.pi, 65):
                rotation = exp_so3([0., angle, 0.])
                tcp, wrist = map_model_forearm(elbow, direction, rotation, arm)
                tool = np.asarray(arm["joint7_to_tcp"])
                np.testing.assert_allclose(tcp - rotation @ tool[:3, :3].T @ tool[:3, 3], wrist, atol=1e-12)
                np.testing.assert_allclose(wrist, direction * (self.upper + arm["neutral_elbow_to_joint7_m"]), atol=1e-12)
                self.assertLessEqual(np.linalg.norm(wrist), arm["shoulder_to_joint7_necessary_upper_m"])
            self.assertLess(arm["neutral_elbow_to_joint7_m"], .30)

    def startup(self, version, params=None):
        # Run both complete production geometry blocks, including the resolver
        # that used to fail before the old test's mode-selection slice began.
        tree = ast.parse((PACKAGE / 'scripts/ik_ros_velocity.py').read_text())
        blocks = [n for n in ast.walk(tree) if isinstance(n, ast.If)
                  and isinstance(n.test, ast.Name)
                  and n.test.id == 'velocity_ik_takeover_requested']
        def imports(block, name):
            return any(isinstance(n, ast.ImportFrom)
                       and any(a.name == name for a in n.names) for n in ast.walk(block))
        neutral = next(n for n in blocks if imports(n, 'neutral_retarget_geometry'))
        geometry = next(n for n in blocks if imports(n, 'resolve_velocity_tcp_geometry'))
        code = compile(ast.Module(body=[neutral, geometry], type_ignores=[]),
                       'actual_mapper_geometry_startup', 'exec')
        class Params:
            def __init__(self, values): self.p = dict(values)
            def get_param(self, key, default=None): return self.p.get(key, default)
            def set_param(self, key, value): self.p[key] = value
            def logwarn(self, *args): pass
        assets = PACKAGE.parent / 'kuavo_assets'
        wheel = PACKAGE.parent / 'humanoid-wheel-control/humanoid_wheel_interface'
        overlay_file = PACKAGE / ('config/model_s' + version + '.json')
        overlay = json.loads(overlay_file.read_text()) if overlay_file.exists() else None
        contract = resolve_model(version, assets, wheel, overlay)
        config = json.loads((assets / ('config/kuavo_v' + version) / 'kuavo.json').read_text())
        config.update({k: v for k, v in contract.items() if k.startswith('velocity_ik_')})
        ros = Params(params or {})
        ns = dict(rospy=ros, robot_version=version, model_config=config,
                  velocity_ik_takeover_requested=True, neutral_robot_geometry=None,
                  model_file=str(require_full_model(version, assets, config)),
                  end_frames_name=config['end_frames_name_ik'],
                  shoulder_frame_names=config['shoulder_frame_names'],
                  np=np, rpy_to_matrix=rpy_to_matrix)
        with contextlib.redirect_stdout(io.StringIO()):
            exec(code, ns)
        return ros.p['/quest3/teleop_tcp/geometry_binding'], ros.p

    def test_all_model_startups_without_legacy_scalar(self):
        for version in VERSIONS:
            with self.subTest(version=version):
                geometry, params = self.startup(version)
                self.assertEqual(geometry['operation_mode'], MODEL_FOREARM)
                self.assertFalse(geometry['forearm_retarget_length_applied'])
                self.assertNotIn('/quest3/teleop_tcp/forearm_retarget_length', params)
                self.assertEqual(geometry['robot_version'], version)
                self.assertIn('biped_s' + version + '/', geometry['ik_model']['model_path'])
                self.assertGreater(geometry['neutral_robot_geometry']['upper_arm_length'], 0.)
                for side, arm in geometry['ik_model']['arms'].items():
                    self.assertEqual(geometry['forearm_to_joint7_m'][side],
                                     arm['neutral_elbow_to_joint7_m'])
                self.assertEqual([geometry['ik_model']['arms'][s]['elbow_frame']
                                  for s in ('Left', 'Right')], geometry['robot_elbow_frames'])

    def test_old_scalar_does_not_change_any_model_geometry(self):
        for version in VERSIONS:
            with self.subTest(version=version):
                clean, _ = self.startup(version)
                stale, _ = self.startup(version, {
                    '~velocity_tcp_forearm_length': .45,
                    '~velocity_tcp_geometry_id': 'OLD_CONFIGURED_FOREARM',
                    '/quest3/teleop_tcp/forearm_retarget_length': .45})
                self.assertEqual(stale['ignored_legacy_forearm_length'], .45)
                self.assertEqual(stale['ik_model'], clean['ik_model'])
                self.assertEqual(stale['forearm_to_joint7_m'], clean['forearm_to_joint7_m'])
                self.assertFalse(stale['forearm_retarget_length_applied'])

    def test_explicit_legacy_and_wrist_modes_keep_scalar_validation(self):
        for version in VERSIONS:
            for mode in (LEGACY, WRIST):
                with self.subTest(version=version, mode=mode):
                    params = {'~velocity_target_geometry_mode': mode}
                    with self.assertRaisesRegex(ValueError, 'explicit forearm'):
                        self.startup(version, params)
                    params.update({'~velocity_tcp_forearm_length': .45,
                                   '~velocity_tcp_geometry_id': 'EXPLICIT_TEST_GEOMETRY'})
                    geometry, result = self.startup(version, params)
                    self.assertEqual(geometry['operation_mode'], mode)
                    self.assertEqual(result['/quest3/teleop_tcp/forearm_retarget_length'], .45)
                    params['~velocity_tcp_forearm_length'] = float('nan')
                    with self.assertRaisesRegex(ValueError, 'finite and positive'):
                        self.startup(version, params)

    def test_unknown_mode_is_rejected_before_scalar_resolution(self):
        with self.assertRaisesRegex(ValueError, 'Unknown velocity target geometry mode'):
            self.startup('62', {'~velocity_target_geometry_mode': 'unknown'})

    def test_all_model_rotating_tools_use_each_urdf(self):
        for version in VERSIONS:
            geometry, _ = self.startup(version, {
                '/absolute_velocity_ik/left_tcp_translation': [.01, -.02, .03],
                '/absolute_velocity_ik/right_tcp_rpy': [.1, -.2, .3]})
            upper = geometry['neutral_robot_geometry']['upper_arm_length']
            direction = np.array([1., 0., 0.])
            for side, arm in geometry['ik_model']['arms'].items():
                for angle in np.linspace(-np.pi, np.pi, 17):
                    with self.subTest(version=version, side=side, angle=angle):
                        rotation = exp_so3([0., angle, 0.])
                        tcp, wrist = map_model_forearm(upper * direction, direction, rotation, arm)
                        tool = np.asarray(arm['joint7_to_tcp'])
                        np.testing.assert_allclose(
                            tcp - rotation @ tool[:3, :3].T @ tool[:3, 3], wrist, atol=1e-12)
                        np.testing.assert_allclose(
                            wrist, direction * (upper + arm['neutral_elbow_to_joint7_m']), atol=1e-12)

    def test_actual_mapper_uses_per_arm_geometry_without_old_scalars(self):
        from skeleton_filter_alignment_test import mapper_class, make_mapper
        cls = mapper_class(PACKAGE / 'scripts/tools/quest3_utils_velocity.py')
        for version in VERSIONS:
            geometry, _ = self.startup(version)
            for side in ('Left', 'Right'):
                with self.subTest(version=version, side=side):
                    mapper = make_mapper(cls)
                    mapper.tcp_geometry_mode = MODEL_FOREARM
                    mapper.tcp_geometry_binding = geometry
                    mapper.tcp_model_forearm = geometry['ik_model']['arms']
                    mapper.tcp_mapping_diagnostics = {}
                    mapper.is_hand_tracking = False
                    mapper.init_R_wC = np.eye(3)
                    mapper.tcp_source_to_tcp = {'controller': {side: np.eye(4)}}
                    # Deliberately wrong legacy scalars must not affect targets.
                    mapper.upper_arm_length = 9.
                    mapper.tcp_forearm_retarget_length = 8.
                    mapper.shoulder_width = 7.
                    chest, shoulder, elbow, palm = [np.eye(4) for _ in range(4)]
                    shoulder[:3, 3] = [0., .2, 0.]
                    elbow[:3, 3] = [.2, .2, 0.]
                    palm[:3, 3] = [.2, .5, 0.]
                    rotation = exp_so3([.2, -.3, .1])
                    target, hint = mapper._compute_unified_tcp_target(
                        side, chest, shoulder, elbow, palm, None, rotation)
                    neutral = geometry['neutral_robot_geometry']
                    expected_hint = np.asarray(neutral['shoulder_positions'][side]) + np.array(
                        [neutral['upper_arm_lengths'][side], 0., 0.])
                    expected_tcp, _ = map_model_forearm(
                        expected_hint, [0., 1., 0.], rotation, geometry['ik_model']['arms'][side])
                    np.testing.assert_allclose(hint, expected_hint, atol=1e-12)
                    np.testing.assert_allclose(target[0], expected_tcp, atol=1e-12)

    def test_velocity_matches_finite_difference_with_chest_and_tool_rotation(self):
        rng = np.random.RandomState(71)
        for arm0 in self.model["arms"].values():
            # Nontrivial tool rotation catches a wrong TCP-vs-joint7 offset frame.
            arm = dict(arm0); tool = np.asarray(arm["joint7_to_tcp"])
            tool[:3, :3] = exp_so3([.2, -.3, .1]); arm["joint7_to_tcp"] = tool
            adapter = exp_so3([-.1, .2, .3]); axes = exp_so3([.3, .1, -.4])
            for _ in range(30):
                states = []
                for i in range(4):
                    T = np.eye(4); T[:3, :3] = exp_so3(rng.randn(3)); T[:3, 3] = rng.randn(3)
                    states.append((T, rng.randn(3), rng.randn(3)))
                def position(ss):
                    c, s, e, h = [x[0] for x in ss]; remove = axes @ c[:3, :3].T
                    upper, lower = e[:3, 3]-s[:3, 3], h[:3, 3]-e[:3, 3]
                    hint = self.upper * remove @ (upper / np.linalg.norm(upper))
                    return map_model_forearm(hint, remove @ (lower / np.linalg.norm(lower)),
                                             remove @ h[:3, :3] @ adapter, arm)[0]
                v, _, _ = mapped_rates(*states, axes, self.upper, arm["neutral_elbow_to_joint7_m"],
                                       states[3][0][:3, :3], tool[:3, 3], None,
                                       adapter @ tool[:3, :3].T)
                shifted = []; dt = 1e-7
                for T, vel, omega in states:
                    U = T.copy(); U[:3, 3] += dt * vel; U[:3, :3] = exp_so3(dt * omega) @ T[:3, :3]
                    shifted.append((U, vel, omega))
                np.testing.assert_allclose(v, (position(shifted)-position(states))/dt, atol=1e-5, rtol=1e-5)


if __name__ == "__main__":
    unittest.main()
