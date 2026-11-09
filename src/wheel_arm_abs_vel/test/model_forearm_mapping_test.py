#!/usr/bin/env python3
"""Robot reach, rotating tool and analytic velocity regressions (no ROS)."""
import json
import ast
from pathlib import Path
import sys
import unittest
import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / "scripts/tools"))
from target_geometry_contract import model_geometry, map_model_forearm, neutral_retarget_geometry, LEGACY, WRIST, MODEL_FOREARM
from pose_velocity_kalman import mapped_rates, exp_so3


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

    def test_actual_startup_mode_selection(self):
        tree=ast.parse((PACKAGE/'scripts/ik_ros_velocity.py').read_text())
        block=next(n for n in ast.walk(tree) if isinstance(n,ast.If) and
                   any(isinstance(x,ast.Assign) and 'operation_mode' in ast.dump(x.targets[0])
                       for x in n.body))
        begin=next(i for i,x in enumerate(block.body) if isinstance(x,ast.Assign) and
                   'operation_mode' in ast.dump(x.targets[0]))
        end=next(i for i,x in enumerate(block.body) if isinstance(x,ast.Assign) and
                 'source_origin_status' in ast.dump(x.targets[0]))
        code=compile(ast.Module(body=block.body[begin:end],type_ignores=[]),'actual_startup','exec')
        class Params:
            def __init__(self,p):self.p=p
            def get_param(self,k,default=None):return self.p.get(k,default)
            def logwarn(self,*args):pass
        def run(version,p):
            ns=dict(rospy=Params(p),robot_version=version,LEGACY=LEGACY,WRIST=WRIST,
                    MODEL_FOREARM=MODEL_FOREARM,velocity_geometry={'ik_model':self.model})
            exec(code,ns);return ns['velocity_geometry']
        for version in ('62', '200062', '210062', '300062', '310062', '400062'):
            self.assertEqual(run(version,{})['operation_mode'],MODEL_FOREARM)
        self.assertEqual(run('63',{})['operation_mode'],LEGACY)
        self.assertEqual(run('62',{'~velocity_target_geometry_mode':LEGACY,
                                  '~velocity_tcp_forearm_length':.45})['operation_mode'],LEGACY)
        stale=run('62',{'~velocity_tcp_forearm_length':.45})
        self.assertEqual(stale['operation_mode'],MODEL_FOREARM)
        self.assertFalse(stale['forearm_retarget_length_applied'])
        self.assertEqual(stale['ignored_legacy_forearm_length'],.45)
        for length in stale['forearm_to_joint7_m'].values():
            self.assertAlmostEqual(length,.2552846450533208)
        with self.assertRaises(ValueError):run('62',{'~velocity_target_geometry_mode':'unknown'})

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
