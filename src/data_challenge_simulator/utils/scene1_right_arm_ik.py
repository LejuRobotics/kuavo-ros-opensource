"""Scene 1 right-arm IK for the SG100 grasp centre."""

from pathlib import Path

import mujoco
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R


class Scene1RightArmIK:
    """Solve the right arm against the actual Scene 1 MuJoCo kinematics."""

    PALM_CENTER_IN_R7 = np.array([0.000067, 0.016679, -0.132586])
    OPEN_GRASP_CENTER_IN_EEF = np.array([0.034313, 0.087794, -0.022232])
    OPEN_GRASP_CENTER_IN_R7 = PALM_CENTER_IN_R7 + OPEN_GRASP_CENTER_IN_EEF
    # Midpoint of the two active fingertip collision pads at full closure.
    # Unlike the open midpoint, this is the point that must coincide with the
    # cylinder centre after the fixed gripper-closing action.
    CLOSED_GRASP_CENTER_IN_R7 = np.array([0.036002, 0.152276, -0.156807])
    READY = np.array([-0.9, -0.265, 1.0, -0.8, 0.58, -0.4, 0.35])
    USER_GRASP = np.array([-0.809, -0.515, 1.11, -0.969, 0.66, -0.317, 0.384])

    def __init__(self):
        package_dir = Path(__file__).resolve().parents[1]
        xml_path = package_dir / (
            "models/biped_s400062/xml/task1.xml")
        self.model = mujoco.MjModel.from_xml_path(str(xml_path))
        self.data = mujoco.MjData(self.model)
        base_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        base_joint_id = self.model.body_jntadr[base_body_id]
        self.base_qpos_address = self.model.jnt_qposadr[base_joint_id]
        self.base_translation_world = np.zeros(3)
        joint_names = ["zarm_r{}_joint".format(i) for i in range(1, 8)]
        self.joint_ids = np.array([
            mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            for name in joint_names
        ])
        self.qpos_addresses = np.array([
            self.model.jnt_qposadr[joint_id]
            for joint_id in self.joint_ids
        ])
        # Stay a few floating-point ulps inside the same model limits so the
        # SDK does not report an identical boundary value as being clipped on
        # every 100 Hz command frame.
        self.lower = self.model.jnt_range[self.joint_ids, 0].copy() + 1e-5
        self.upper = self.model.jnt_range[self.joint_ids, 1].copy() - 1e-5
        self.r7_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "zarm_r7_link")
        hand_joint_names = (
            "r_thumb_j1", "r_thumb_j2", "r_thumb_j3",
            "r_index_j1", "r_index_j2", "r_index_j3",
            "r_middle_j1", "r_middle_j2",
            "r_little_j1", "r_little_j2", "r_little_j3",
        )
        hand_joint_ids = [
            mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            for name in hand_joint_names
        ]
        self.hand_qpos_addresses = np.array([
            self.model.jnt_qposadr[joint_id]
            for joint_id in hand_joint_ids
        ])
        self.thumb_pad_geom_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM,
            "r_thumb_fingertip_collision")
        self.index_pad_geom_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM,
            "r_index_fingertip_collision")

    def set_base_translation_world(self, translation_world):
        """Use the simulator's fixed, identity-oriented base translation."""
        translation = np.asarray(translation_world, dtype=float)
        if translation.shape != (3,):
            raise ValueError("base translation must contain exactly 3 values")
        self.base_translation_world = translation.copy()

    def _forward(self, right_q, point_in_r7=None):
        self.data.qpos[:] = self.model.qpos0
        base_slice = slice(
            self.base_qpos_address, self.base_qpos_address + 3)
        self.data.qpos[base_slice] += self.base_translation_world
        self.data.qpos[self.qpos_addresses] = right_q
        mujoco.mj_kinematics(self.model, self.data)
        rotation = self.data.xmat[self.r7_body_id].reshape(3, 3).copy()
        r7_position = self.data.xpos[self.r7_body_id].copy()
        if point_in_r7 is None:
            point_in_r7 = self.OPEN_GRASP_CENTER_IN_R7
        grasp_center = r7_position + rotation @ point_in_r7
        palm_center = r7_position + rotation @ self.PALM_CENTER_IN_R7
        return grasp_center, palm_center, rotation

    def eef_quaternion_world(self, right_q):
        _, _, rotation = self._forward(np.asarray(right_q, dtype=float))
        return R.from_matrix(rotation).as_quat().tolist()

    def eef_quaternion_world_with_yaw(self, right_q, yaw_rad):
        """Return the EEF attitude after a world-Z yaw adjustment."""
        orientation = R.from_quat(self.eef_quaternion_world(right_q))
        return (R.from_euler("z", yaw_rad) * orientation).as_quat().tolist()

    def grasp_center_world(self, right_q):
        grasp_center, _, _ = self._forward(np.asarray(right_q, dtype=float))
        return grasp_center.tolist()

    def measured_pad_geometry(self, right_q, right_hand_q):
        """Evaluate actual thumb/index pad centres from measured joint state."""
        self.data.qpos[:] = self.model.qpos0
        base_slice = slice(
            self.base_qpos_address, self.base_qpos_address + 3)
        self.data.qpos[base_slice] += self.base_translation_world
        self.data.qpos[self.qpos_addresses] = np.asarray(right_q, dtype=float)
        self.data.qpos[self.hand_qpos_addresses] = np.asarray(
            right_hand_q, dtype=float)
        mujoco.mj_kinematics(self.model, self.data)
        thumb = self.data.geom_xpos[self.thumb_pad_geom_id].copy()
        index = self.data.geom_xpos[self.index_pad_geom_id].copy()
        return {
            "thumb": thumb.tolist(),
            "index": index.tolist(),
            "midpoint": (0.5 * (thumb + index)).tolist(),
            "gap": float(np.linalg.norm(index - thumb)),
        }

    def solve(
            self, target_world, orientation_world, seed_right_q,
            point_in_r7=None):
        target = np.asarray(target_world, dtype=float)
        target_rotation = R.from_quat(orientation_world)
        seed = np.clip(
            np.asarray(seed_right_q, dtype=float), self.lower, self.upper)
        if point_in_r7 is None:
            point_in_r7 = self.OPEN_GRASP_CENTER_IN_R7
        point_in_r7 = np.asarray(point_in_r7, dtype=float)

        def residual(right_q):
            grasp_center, _, rotation = self._forward(right_q, point_in_r7)
            orientation_error = (
                target_rotation.inv() * R.from_matrix(rotation)).as_rotvec()
            return np.concatenate((
                100.0 * (grasp_center - target),
                0.5 * orientation_error,
                0.05 * (right_q - seed),
            ))

        starts = (seed, self.READY, self.USER_GRASP)
        candidates = []
        for start in starts:
            result = least_squares(
                residual,
                np.clip(start, self.lower, self.upper),
                bounds=(self.lower, self.upper),
                max_nfev=600,
                xtol=1e-10,
                ftol=1e-10,
                gtol=1e-10,
            )
            grasp_center, palm_center, rotation = self._forward(
                result.x, point_in_r7)
            position_error = float(np.linalg.norm(grasp_center - target))
            orientation_error = float(np.linalg.norm(
                (target_rotation.inv() * R.from_matrix(rotation)).as_rotvec()))
            candidates.append((
                float(result.cost),
                position_error,
                orientation_error,
                result.x.copy(),
                grasp_center,
                palm_center,
            ))

        best = min(candidates, key=lambda item: (item[0], item[1]))
        print(
            "Scene1 IK: target={} grasp_center={} palm={} "
            "position_error={:.6f} m orientation_error={:.3f} deg".format(
                np.round(target, 6).tolist(),
                np.round(best[4], 6).tolist(),
                np.round(best[5], 6).tolist(),
                best[1],
                np.degrees(best[2]),
            ))
        return best[3].tolist()
