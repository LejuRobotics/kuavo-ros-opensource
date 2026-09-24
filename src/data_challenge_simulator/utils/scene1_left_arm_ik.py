"""Scene 1 left-arm IK for pulling the lever with the inverted-L hook."""

import math
from pathlib import Path

import mujoco
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R


class Scene1LeftArmIK:
    """Solve only the left arm against the actual Scene 1 MuJoCo model."""

    # Centre of the index/middle/little fingertip-pad row in lever_hook_pose,
    # measured from the same formal Scene 1 model loaded below.
    HOOK_FINGER_ROW_CENTER_IN_R7 = np.array(
        [-0.003000, -0.095882, -0.230496])
    READY = np.array([-0.9, 0.265, -1.0, -0.8, -0.58, 0.4, 0.35])

    def __init__(self):
        package_dir = Path(__file__).resolve().parents[1]
        xml_path = package_dir / "models/biped_s400062/xml/task1.xml"
        self.model = mujoco.MjModel.from_xml_path(str(xml_path))
        self.data = mujoco.MjData(self.model)

        base_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        base_joint_id = self.model.body_jntadr[base_body_id]
        self.base_qpos_address = self.model.jnt_qposadr[base_joint_id]
        self.base_translation_world = np.zeros(3)
        self.base_yaw_world = 0.0

        joint_names = ["zarm_l{}_joint".format(i) for i in range(1, 8)]
        self.joint_ids = np.array([
            mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            for name in joint_names
        ])
        self.qpos_addresses = np.array([
            self.model.jnt_qposadr[joint_id]
            for joint_id in self.joint_ids
        ])
        self.lower = self.model.jnt_range[self.joint_ids, 0].copy() + 1e-5
        self.upper = self.model.jnt_range[self.joint_ids, 1].copy() - 1e-5
        self.r7_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "zarm_l7_link")

    def set_base_translation_world(self, translation_world):
        translation = np.asarray(translation_world, dtype=float)
        if translation.shape != (3,):
            raise ValueError("base translation must contain exactly 3 values")
        self.base_translation_world = translation.copy()

    def set_base_pose_world(self, translation_world, yaw_world):
        """Set the planar base pose used by offline FK and IK.

        Scene 1 currently keeps the base yaw at zero during the right-arm
        phase.  The explicit yaw is needed when measuring a later lever stance
        that the mobile chassis can reach.
        """
        self.set_base_translation_world(translation_world)
        self.base_yaw_world = float(yaw_world)

    def _forward(self, left_q, point_in_r7=None):
        self.data.qpos[:] = self.model.qpos0
        base_slice = slice(
            self.base_qpos_address, self.base_qpos_address + 3)
        self.data.qpos[base_slice] += self.base_translation_world
        half_yaw = 0.5 * self.base_yaw_world
        self.data.qpos[
            self.base_qpos_address + 3:self.base_qpos_address + 7
        ] = (math.cos(half_yaw), 0.0, 0.0, math.sin(half_yaw))
        self.data.qpos[self.qpos_addresses] = np.asarray(left_q, dtype=float)
        mujoco.mj_kinematics(self.model, self.data)
        rotation = self.data.xmat[self.r7_body_id].reshape(3, 3).copy()
        r7_position = self.data.xpos[self.r7_body_id].copy()
        if point_in_r7 is None:
            point_in_r7 = self.HOOK_FINGER_ROW_CENTER_IN_R7
        tcp = r7_position + rotation @ np.asarray(point_in_r7, dtype=float)
        return tcp, r7_position, rotation

    def tcp_world(self, left_q, point_in_r7=None):
        return self._forward(left_q, point_in_r7)[0].tolist()

    def solve(self, target_world, orientation_world, seed_left_q,
              point_in_r7=None, max_position_error=0.005,
              joint_limit_margin_fraction=0.0):
        target = np.asarray(target_world, dtype=float)
        target_rotation = R.from_quat(orientation_world)
        target_matrix = target_rotation.as_matrix()
        margin_fraction = float(joint_limit_margin_fraction)
        if not 0.0 <= margin_fraction < 0.5:
            raise ValueError(
                "joint_limit_margin_fraction must be in [0.0, 0.5)")
        margin = margin_fraction * (self.upper - self.lower)
        solve_lower = self.lower + margin
        solve_upper = self.upper - margin
        seed = np.clip(
            np.asarray(seed_left_q, dtype=float), solve_lower, solve_upper)
        if point_in_r7 is None:
            point_in_r7 = self.HOOK_FINGER_ROW_CENTER_IN_R7
        point_in_r7 = np.asarray(point_in_r7, dtype=float)

        def residual(left_q):
            tcp, _, rotation = self._forward(left_q, point_in_r7)
            orientation_error = (
                target_rotation.inv() * R.from_matrix(rotation)).as_rotvec()
            return np.concatenate((
                100.0 * (tcp - target),
                2.0 * (rotation[:, 0] - target_matrix[:, 0]),
                4.0 * (rotation[:, 1] - target_matrix[:, 1]),
                0.1 * orientation_error,
                0.1 * (left_q - seed),
            ))

        starts = (seed, self.READY, np.zeros(7))
        candidates = []
        for start in starts:
            result = least_squares(
                residual,
                np.clip(start, solve_lower, solve_upper),
                bounds=(solve_lower, solve_upper),
                max_nfev=600,
                xtol=1e-10,
                ftol=1e-10,
                gtol=1e-10,
            )
            tcp, r7_position, rotation = self._forward(
                result.x, point_in_r7)
            position_error = float(np.linalg.norm(tcp - target))
            orientation_error = float(np.linalg.norm(
                (target_rotation.inv() * R.from_matrix(rotation)).as_rotvec()))
            row_axis_error = float(np.degrees(np.arccos(np.clip(
                np.dot(rotation[:, 0], target_matrix[:, 0]), -1.0, 1.0))))
            palm_normal_error = float(np.degrees(np.arccos(np.clip(
                np.dot(rotation[:, 1], target_matrix[:, 1]), -1.0, 1.0))))
            candidates.append((
                float(result.cost), position_error, orientation_error,
                row_axis_error, palm_normal_error,
                result.x.copy(), tcp, r7_position))

        best = min(candidates, key=lambda item: (item[0], item[1]))
        print(
            "Scene1 left IK: target={} finger_row={} r7={} "
            "position_error={:.6f} m orientation_error={:.3f} deg".format(
                np.round(target, 6).tolist(),
                np.round(best[6], 6).tolist(),
                np.round(best[7], 6).tolist(),
                best[1], np.degrees(best[2])))
        if best[1] > max_position_error:
            raise RuntimeError(
                "left-arm lever IK position error {:.6f} m exceeds {:.6f} m"
                .format(best[1], max_position_error))
        print(
            "Scene1 left IK: finger-row axis error={:.3f} deg "
            "palm-normal error={:.3f} deg".format(best[3], best[4]))
        return best[5].tolist()

    def solve_lever(self, target_world, seed_left_q,
                    joint_limit_margin_fraction=0.10,
                    max_position_error=0.005):
        """Solve the lever TCP without constraining the palm roll.

        The fingertip row remains parallel to the world-Y handle.  Rotation
        about that row is deliberately free so contact reaction cannot fight a
        fixed palm-down wrist command.
        """
        target = np.asarray(target_world, dtype=float)
        target_row_axis = np.array([0.0, -1.0, 0.0])
        margin_fraction = float(joint_limit_margin_fraction)
        if not 0.0 <= margin_fraction < 0.5:
            raise ValueError(
                "joint_limit_margin_fraction must be in [0.0, 0.5)")
        margin = margin_fraction * (self.upper - self.lower)
        solve_lower = self.lower + margin
        solve_upper = self.upper - margin
        seed = np.clip(
            np.asarray(seed_left_q, dtype=float), solve_lower, solve_upper)

        def residual(left_q):
            tcp, _, rotation = self._forward(left_q)
            return np.concatenate((
                100.0 * (tcp - target),
                2.0 * (rotation[:, 0] - target_row_axis),
                0.2 * (left_q - seed),
            ))

        candidates = []
        for start in (seed, self.READY, np.zeros(7)):
            result = least_squares(
                residual,
                np.clip(start, solve_lower, solve_upper),
                bounds=(solve_lower, solve_upper),
                max_nfev=600,
                xtol=1e-10,
                ftol=1e-10,
                gtol=1e-10,
            )
            tcp, r7_position, rotation = self._forward(result.x)
            position_error = float(np.linalg.norm(tcp - target))
            row_axis_error = float(np.degrees(np.arccos(np.clip(
                np.dot(rotation[:, 0], target_row_axis), -1.0, 1.0))))
            candidates.append((
                float(result.cost), position_error, row_axis_error,
                result.x.copy(), tcp, r7_position))

        best = min(candidates, key=lambda item: (item[0], item[1]))
        print(
            "Scene1 lever IK: target={} finger_row={} r7={} "
            "position_error={:.6f} m row_axis_error={:.3f} deg".format(
                np.round(target, 6).tolist(),
                np.round(best[4], 6).tolist(),
                np.round(best[5], 6).tolist(),
                best[1], best[2]))
        if best[1] > max_position_error:
            raise RuntimeError(
                "left-arm lever IK position error {:.6f} m exceeds {:.6f} m"
                .format(best[1], max_position_error))
        return best[3].tolist()
