"""Accepted Task 1 right-arm IK implementation for the canonical MJCF."""

from pathlib import Path

import mujoco
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R

from utils.scene1_right_arm_ik import Scene1RightArmIK


class Scene1V2RightArmIK(Scene1RightArmIK):
    """Reuse the accepted solver while loading task1.xml."""

    # Mean centre of the right index/middle/little collision pads in the
    # Task1 V2 lever-hook posture, expressed in zarm_r7_link.  Its little
    # distal joint is 1.30 rad so that all three fingertip pads align.
    HOOK_FINGER_ROW_CENTER_IN_R7 = np.array(
        [-0.003000, 0.095481, -0.233601])

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
        base_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        self.robot_body_ids = {base_body_id}
        for body_id in range(1, self.model.nbody):
            if int(self.model.body_parentid[body_id]) in self.robot_body_ids:
                self.robot_body_ids.add(body_id)
        self.ignored_task_contact_geoms = {
            "floor",
            "source_bin_floor",
            "target_bin_floor",
            "block_1_visual",
            "block_1_geom",
            "block_2_visual",
            "block_2_geom",
        }

    def closed_grasp_center_world(self, right_q):
        centre, _, _ = self._forward(
            np.asarray(right_q, dtype=float),
            self.CLOSED_GRASP_CENTER_IN_R7)
        return centre.tolist()

    def task_obstacle_contacts(self, right_q, right_hand_q):
        """Return robot/task contacts, excluding support floors and cylinders."""
        self.data.qpos[:] = self.model.qpos0
        base_slice = slice(
            self.base_qpos_address, self.base_qpos_address + 3)
        self.data.qpos[base_slice] += self.base_translation_world
        self.data.qpos[self.qpos_addresses] = np.asarray(
            right_q, dtype=float)
        self.data.qpos[self.hand_qpos_addresses] = np.asarray(
            right_hand_q, dtype=float)
        mujoco.mj_forward(self.model, self.data)

        contacts = set()
        for contact_index in range(self.data.ncon):
            contact = self.data.contact[contact_index]
            geom1 = int(contact.geom1)
            geom2 = int(contact.geom2)
            body1 = int(self.model.geom_bodyid[geom1])
            body2 = int(self.model.geom_bodyid[geom2])
            robot1 = body1 in self.robot_body_ids
            robot2 = body2 in self.robot_body_ids
            if robot1 == robot2:
                continue
            name1 = mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_GEOM, geom1)
            name2 = mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_GEOM, geom2)
            task_name = name2 if robot1 else name1
            if task_name in self.ignored_task_contact_geoms:
                continue
            contacts.add((name1, name2))
        return tuple(sorted(contacts))

    def hook_tcp_world(self, right_q):
        hook_center, _, _ = self._forward(
            np.asarray(right_q, dtype=float),
            self.HOOK_FINGER_ROW_CENTER_IN_R7)
        return hook_center.tolist()

    def eef_rotation_world(self, right_q):
        """Return the right wrist rotation matrix in world coordinates."""
        _, _, rotation = self._forward(np.asarray(right_q, dtype=float))
        return rotation.copy()

    @staticmethod
    def rotate_about_world_y(reference_rotation_world, angle_rad):
        """Rotate a reference wrist attitude with the lever about world Y."""
        reference = np.asarray(reference_rotation_world, dtype=float)
        if reference.shape != (3, 3):
            raise ValueError("reference rotation must be a 3x3 matrix")
        return R.from_euler("y", float(angle_rad)).as_matrix() @ reference

    @staticmethod
    def interpolate_rotation_world(start_rotation_world,
                                   end_rotation_world, fraction):
        """Interpolate two world rotations along their shortest arc."""
        phase = min(1.0, max(0.0, float(fraction)))
        start = R.from_matrix(np.asarray(start_rotation_world, dtype=float))
        end = R.from_matrix(np.asarray(end_rotation_world, dtype=float))
        correction = start.inv() * end
        return (
            start
            * R.from_rotvec(phase * correction.as_rotvec())).as_matrix()

    def solve_lever(self, target_world, seed_right_q,
                    joint_limit_margin_fraction=0.10,
                    max_position_error=0.005,
                    target_rotation_world=None):
        """Solve the right hook row on the world-Y horizontal handle."""
        target = np.asarray(target_world, dtype=float)
        target_row_axis = np.array([0.0, 1.0, 0.0])
        target_rotation = None
        if target_rotation_world is not None:
            target_rotation_matrix = np.asarray(
                target_rotation_world, dtype=float)
            if target_rotation_matrix.shape != (3, 3):
                raise ValueError("target rotation must be a 3x3 matrix")
            target_rotation = R.from_matrix(target_rotation_matrix)
        margin_fraction = float(joint_limit_margin_fraction)
        if not 0.0 <= margin_fraction < 0.5:
            raise ValueError(
                "joint_limit_margin_fraction must be in [0.0, 0.5)")
        margin = margin_fraction * (self.upper - self.lower)
        solve_lower = self.lower + margin
        solve_upper = self.upper - margin
        seed = np.clip(
            np.asarray(seed_right_q, dtype=float),
            solve_lower, solve_upper)

        def residual(right_q):
            tcp, _, rotation = self._forward(
                right_q, self.HOOK_FINGER_ROW_CENTER_IN_R7)
            if target_rotation is None:
                orientation_residual = 2.0 * (
                    rotation[:, 0] - target_row_axis)
            else:
                orientation_residual = 2.0 * (
                    target_rotation.inv()
                    * R.from_matrix(rotation)).as_rotvec()
            return np.concatenate((
                100.0 * (tcp - target),
                orientation_residual,
                0.2 * (right_q - seed),
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
            tcp, _, rotation = self._forward(
                result.x, self.HOOK_FINGER_ROW_CENTER_IN_R7)
            position_error = float(np.linalg.norm(tcp - target))
            row_axis_error = float(np.degrees(np.arccos(np.clip(
                np.dot(rotation[:, 0], target_row_axis), -1.0, 1.0))))
            orientation_error = (
                None if target_rotation is None else float(np.linalg.norm(
                    (target_rotation.inv()
                     * R.from_matrix(rotation)).as_rotvec())))
            candidates.append((
                float(result.cost), position_error, row_axis_error,
                orientation_error,
                result.x.copy(), tcp))

        best = min(candidates, key=lambda item: (item[0], item[1]))
        print(
            "Scene1 V2 right lever IK: target={} finger_row={} "
            "position_error={:.6f} m row_axis_error={:.3f} deg "
            "orientation_error={}".format(
                np.round(target, 6).tolist(),
                np.round(best[5], 6).tolist(), best[1], best[2],
                "n/a" if best[3] is None else "{:.3f} deg".format(
                    np.degrees(best[3]))))
        if best[1] > max_position_error:
            raise RuntimeError(
                "right-arm lever IK position error {:.6f} m exceeds "
                "{:.6f} m".format(best[1], max_position_error))
        return best[4].tolist()
