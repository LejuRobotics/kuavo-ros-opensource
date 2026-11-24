"""Hand kinematics, joint limits and fingertip pose fitting."""

from pathlib import Path

import numpy as np
import pinocchio as pin

from .ik_helpers import (
    golden_section_minimize,
    quat_xyz_angles_wxyz,
    quat_yz_plane_x_angle_wxyz,
    root_flexion_weight,
)
from .ik_position import solve_position_ik
from .types import FingerSpec, SolverConfig


class SG100IK:
    def __init__(self, urdf_path, fingers=None, cfg=None):
        self.urdf_path = Path(urdf_path)
        self.cfg = cfg or SolverConfig()
        self.fingers = fingers or default_fingers()
        self.model = pin.buildModelFromUrdf(str(self.urdf_path))
        self.nq = self.model.nq
        self.nv = self.model.nv
        self.lower = np.asarray(self.model.lowerPositionLimit, dtype=float)
        self.upper = np.asarray(self.model.upperPositionLimit, dtype=float)
        self._validate_model()
        self._active_q_idx = np.array(
            [self.qidx(jn) for f in self.fingers for jn in f.joints], dtype=int
        )
        self.n_active = self._active_q_idx.size
        self._neutral = self.make_neutral()
        self._tip_rotation_offsets = self._calibrate_tip_rotation_offsets()
        for f in self.fingers:
            w = self.cfg.finger_weights.get(f.name, {})
            f.pos_weight = float(w.get("pos", f.pos_weight))
            f.rot_weight = float(w.get("rot", f.rot_weight))
            f.active_q_idx = np.array([self.qidx(jn) for jn in f.joints], dtype=int)
            f.tip_frame_id = self._add_fingertip_frame(f)
        self.data = self.model.createData()
        self._palm_frame_id = self.model.getFrameId("base_link")
        self._mcp_aa_axis_tasks = self._build_mcp_aa_axis_tasks()
        self._init_mcp_aa_zero_rotations()
        self._bend_orientation_tasks = self._build_bend_orientation_tasks()
        self.last_position_ik_q = None
        self.last_pre_fuse_q = None
        self.last_side_orientation_fit = {}
        self.last_terminal_orientation_fit = {}

    def _add_fingertip_frame(self, finger):
        link_frame_id = self.model.getFrameId(finger.tip_frame)
        link_frame = self.model.frames[link_frame_id]
        link_to_tip = pin.SE3(
            self._tip_rotation_offsets[finger.name],
            np.asarray(finger.tip_offset_xyz, dtype=float).reshape(3),
        )
        tip_frame = pin.Frame(
            f"{finger.name}_stl_tip",
            link_frame.parentJoint,
            link_frame_id,
            link_frame.placement * link_to_tip,
            pin.FrameType.OP_FRAME,
        )
        return self.model.addFrame(tip_frame)

    def _calibrate_tip_rotation_offsets(self):
        configured = self.cfg.tip_reference_world_wxyz
        if configured is None:
            return {f.name: np.eye(3) for f in self.fingers}
        reference_q = self.clip(np.zeros(self.nq))
        reference_data = self.model.createData()
        pin.forwardKinematics(self.model, reference_data, reference_q)
        pin.updateFramePlacements(self.model, reference_data)
        offsets = {}
        per_finger = self.cfg.tip_reference_world_wxyz_per_finger
        for finger in self.fingers:
            quat = np.asarray(per_finger.get(finger.name, configured), dtype=float).reshape(4)
            norm = np.linalg.norm(quat)
            if norm <= 1e-12:
                raise ValueError(f"tip_reference_world_wxyz for {finger.name} must be non-zero")
            quat = quat / norm
            desired_world_rotation = pin.Quaternion(*quat).matrix()
            link_frame_id = self.model.getFrameId(finger.tip_frame)
            world_from_link = reference_data.oMf[link_frame_id].rotation
            offsets[finger.name] = world_from_link.T @ desired_world_rotation
        return offsets

    def _validate_model(self):
        missing_joints = [
            jn for f in self.fingers for jn in f.joints if not self.model.existJointName(jn)
        ]
        missing_frames = [
            f.tip_frame
            for f in self.fingers
            if self.model.getFrameId(f.tip_frame) >= len(self.model.frames)
        ]
        if missing_joints or missing_frames:
            raise ValueError(
                f"URDF does not match SG100 IK. missing_joints={missing_joints}, missing_frames={missing_frames}"
            )

    def qidx(self, joint_name):
        jid = self.model.getJointId(joint_name)
        return self.model.joints[jid].idx_q

    def _build_mcp_aa_axis_tasks(self):
        direct_cfg = self.cfg.mcp_aa_direct
        fit_cfg = self.cfg.side_orientation_ik
        direct_enabled = bool(direct_cfg.get("enabled", False))
        fit_enabled = bool(fit_cfg.get("enabled", False))
        if direct_enabled and fit_enabled:
            raise ValueError("mcp_aa_direct and side_orientation_ik cannot both be enabled")
        if not direct_enabled and not fit_enabled:
            return []
        active_cfg = direct_cfg if direct_enabled else fit_cfg
        requested = set(active_cfg.get("fingers", ["index", "little"]))
        joint_by_finger = {"index": "index_j1", "little": "little_j1"}
        unknown = requested - set(joint_by_finger)
        if unknown:
            raise ValueError(
                f"MCP_AA fingers contains fingers without an SG100 MCP_AA joint: {sorted(unknown)}"
            )
        tasks = []
        for fi, f in enumerate(self.fingers):
            if f.name in requested:
                joint_name = joint_by_finger[f.name]
                tasks.append(
                    dict(
                        finger_index=fi,
                        finger_name=f.name,
                        joint_name=joint_name,
                        joint_id=self.model.getJointId(joint_name),
                    )
                )
        return tasks

    def _init_mcp_aa_zero_rotations(self):
        q_zero = self.clip(np.zeros(self.nq))
        pin.forwardKinematics(self.model, self.data, q_zero)
        for task in self._mcp_aa_axis_tasks:
            jid = task["joint_id"]
            task["zero_rotation"] = self.data.oMi[jid].rotation.copy()

    def _build_bend_orientation_tasks(self):
        task_joints = {
            "thumb": ["thumb_j2", "thumb_j3"],
            "index": ["index_j2", "index_j3"],
            "middle": ["middle_j1", "middle_j2"],
            "little": ["little_j2", "little_j3"],
        }
        tasks = {}
        for finger in self.fingers:
            joint_names = task_joints[finger.name]
            axis_joint_id = self.model.getJointId(joint_names[0])
            tasks[finger.name] = dict(
                axis_v_index=self.model.joints[axis_joint_id].idx_v,
                active_q_indices=np.array([self.qidx(jn) for jn in joint_names], dtype=int),
            )
        return tasks

    def make_neutral(self):
        q = np.zeros(self.nq)
        for i in self._active_q_idx:
            lo, hi = self.lower[i], self.upper[i]
            if np.isfinite(lo) and np.isfinite(hi):
                q[i] = 0.5 * (lo + hi)
        for f in self.fingers:
            if f.name in ("index", "little"):
                jn = f"{f.name}_j1"
                qi = self.qidx(jn)
                q[qi] = 0.0
        return self.clip(q)

    def clip(self, q):
        c = self.cfg
        q = np.asarray(q, dtype=float).copy()
        q[self._active_q_idx] = np.clip(
            q[self._active_q_idx],
            self.lower[self._active_q_idx] + c.bounds_eps,
            self.upper[self._active_q_idx] - c.bounds_eps,
        )
        return q

    def joint_dict(self, q):
        return {jn: float(q[self.qidx(jn)]) for f in self.fingers for jn in f.joints}

    def fingertip_poses(self, q):
        q = self.clip(np.asarray(q, dtype=float))
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        pos = np.zeros((len(self.fingers), 3))
        quat = np.zeros((len(self.fingers), 4))
        for i, f in enumerate(self.fingers):
            M = self.data.oMf[f.tip_frame_id]
            pos[i] = M.translation
            qn = pin.Quaternion(M.rotation)
            quat[i] = np.array([qn.w, qn.x, qn.y, qn.z])
        return pos, quat

    def get_palm_quat(self, q):
        q = self.clip(np.asarray(q, dtype=float))
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        qn = pin.Quaternion(self.data.oMf[self._palm_frame_id].rotation)
        return np.array([qn.w, qn.x, qn.y, qn.z])

    def _refit_finger_position_locked_joints(self, q, target_pos, finger, locked_values, cfg):

        locked_cols = {self.qidx(name) for name in locked_values}
        position_cols = np.array(
            [self.qidx(name) for name in finger.joints if self.qidx(name) not in locked_cols],
            dtype=int,
        )
        if position_cols.size == 0:
            return q
        target = np.asarray(target_pos, dtype=float).reshape(3)
        work = self.clip(q)
        for name, value in locked_values.items():
            work[self.qidx(name)] = value
        max_iters = max(int(cfg.get("position_refit_iterations", 8)), 1)
        eta = float(cfg.get("position_refit_step_eta", 1.0))
        damping = max(float(cfg.get("position_refit_damping", 0.001)), 0.0)
        max_step = max(float(cfg.get("position_refit_max_step", 0.2)), 0.0)
        for _ in range(max_iters):
            for name, value in locked_values.items():
                work[self.qidx(name)] = value
            pin.forwardKinematics(self.model, self.data, work)
            pin.updateFramePlacements(self.model, self.data)
            current = self.data.oMf[finger.tip_frame_id].translation.copy()
            error = target - current
            if float(np.linalg.norm(error)) < self.cfg.pos_tol:
                break
            jacobian_full = pin.computeFrameJacobian(
                self.model, self.data, work, finger.tip_frame_id, pin.LOCAL_WORLD_ALIGNED
            )
            jacobian = jacobian_full[:3, position_cols]
            hessian = jacobian.T @ jacobian + damping * np.eye(position_cols.size)
            try:
                step = eta * np.linalg.solve(hessian, jacobian.T @ error)
            except np.linalg.LinAlgError:
                break
            step_norm = float(np.linalg.norm(step))
            if max_step > 0.0 and step_norm > max_step:
                step *= max_step / step_norm
            work[position_cols] += step
            work = self.clip(work)
            for name, value in locked_values.items():
                work[self.qidx(name)] = value
        return self.clip(work)

    def _fit_terminal_finger_orientation(
        self, q, target_quat_wxyz, finger, terminal_name, cfg, position_q=None
    ):

        q_col = self.qidx(terminal_name)
        lower = float(self.lower[q_col] + self.cfg.bounds_eps)
        upper = float(self.upper[q_col] - self.cfg.bounds_eps)
        if upper <= lower:
            return q
        target = np.asarray(target_quat_wxyz, dtype=float).reshape(4)
        norm = float(np.linalg.norm(target))
        if norm <= 1e-12:
            return q
        target = target / norm
        target_rotation = pin.Quaternion(*target).matrix()

        def objective(candidate):
            work = q.copy()
            work[q_col] = float(candidate)
            work = self.clip(work)
            pin.forwardKinematics(self.model, self.data, work)
            pin.updateFramePlacements(self.model, self.data)
            error = pin.log3(target_rotation @ self.data.oMf[finger.tip_frame_id].rotation.T)
            return float(error @ error)

        samples = max(int(cfg.get("coarse_samples", 21)), 3)
        iterations = max(int(cfg.get("refine_iterations", 10)), 0)
        grid = np.linspace(lower, upper, samples)
        costs = np.array([objective(value) for value in grid], dtype=float)
        best = int(np.argmin(costs))
        fitted = self._golden_section_minimize(
            objective,
            float(grid[max(best - 1, 0)]),
            float(grid[min(best + 1, samples - 1)]),
            iterations,
        )
        gain = float((cfg.get("output_gain", {}) or {}).get(finger.name, 1.0))
        position_q = q if position_q is None else position_q
        root_weight = self._root_flexion_weight(position_q, finger.name, "terminal")
        blend_weight = gain * root_weight
        if (self.cfg.root_flexion_weight or {}).get("enabled", False):
            blend_weight = float(np.clip(blend_weight, 0.0, 1.0))
        position_value = float(position_q[q_col])
        out = q.copy()
        out[q_col] = np.clip(
            position_value + blend_weight * (fitted - position_value), lower, upper
        )
        self.last_terminal_orientation_fit[finger.name] = dict(
            position_ik_q=position_value,
            fitted_q=float(fitted),
            root_weight=root_weight,
            blend_weight=blend_weight,
            output_q=float(out[q_col]),
        )
        return self.clip(out)

    def _fit_mcp_aa_with_pin(self, q, target_quat_wxyz, valid_mask):

        fit_cfg = self.cfg.side_orientation_ik or {}
        if not self._mcp_aa_axis_tasks:
            return q
        out = q.copy()
        work = out.copy()
        max_abs = np.deg2rad(abs(float(fit_cfg.get("max_abs_angle_deg", 30.0))))
        coarse_samples = max(int(fit_cfg.get("coarse_samples", 25)), 3)
        refine_iterations = max(int(fit_cfg.get("refine_iterations", 12)), 0)
        gains = fit_cfg.get("output_gain", {}) or {}
        for task in self._mcp_aa_axis_tasks:
            fi = task["finger_index"]
            finger_name = task["finger_name"]
            if not valid_mask[fi]:
                continue
            jid = task["joint_id"]
            q_col = self.model.joints[jid].idx_q
            lower = max(float(self.lower[q_col] + self.cfg.bounds_eps), -max_abs)
            upper = min(float(self.upper[q_col] - self.cfg.bounds_eps), max_abs)
            if upper <= lower:
                continue
            target = np.asarray(target_quat_wxyz[fi], dtype=float).reshape(4)
            target_norm = float(np.linalg.norm(target))
            if target_norm <= 1e-12:
                continue
            target = target / target_norm
            target_rotation = pin.Quaternion(*target).matrix()
            finger = self.fingers[fi]

            def objective(candidate):
                work[q_col] = float(candidate)
                pin.forwardKinematics(self.model, self.data, work)
                pin.updateFramePlacements(self.model, self.data)
                error = pin.log3(target_rotation @ self.data.oMf[finger.tip_frame_id].rotation.T)
                return float(error @ error)

            grid = np.linspace(lower, upper, coarse_samples)
            costs = np.array([objective(value) for value in grid], dtype=float)
            best_index = int(np.argmin(costs))
            bracket_lower = float(grid[max(0, best_index - 1)])
            bracket_upper = float(grid[min(coarse_samples - 1, best_index + 1)])
            fitted = self._golden_section_minimize(
                objective, bracket_lower, bracket_upper, refine_iterations
            )
            fitted_cost = objective(fitted)
            gain = max(float(gains.get(finger_name, 1.0)), 0.0)
            root_weight = self._root_flexion_weight(q, finger_name, "side")
            unweighted = float(np.clip(gain * fitted, lower, upper))
            output_value = float(np.clip(root_weight * unweighted, lower, upper))
            out[q_col] = output_value
            work[q_col] = output_value
            self.last_side_orientation_fit[finger_name] = dict(
                position_ik_q=float(q[q_col]),
                fitted_q=float(fitted),
                root_weight=root_weight,
                unweighted_output_q=float(unweighted),
                output_q=output_value,
                residual_rad=float(np.sqrt(fitted_cost)),
            )
        return self.clip(out)

    def _solve_four_finger_flexion_then_position(
        self, q, target_quat_wxyz, target_pos, valid_mask, position_q=None
    ):

        cfg = self.cfg.four_finger_flexion_orientation_ik
        if not cfg.get("enabled", False):
            return q
        requested = set(cfg.get("fingers", ["index", "middle", "little"]))
        terminal_map = cfg.get(
            "terminal_joint", {"index": "index_j3", "middle": "middle_j2", "little": "little_j3"}
        )
        side_map = cfg.get("side_joint", {"index": "index_j1", "little": "little_j1"})
        out = self.clip(q)
        position_q = q if position_q is None else position_q
        for fi, finger in enumerate(self.fingers):
            name = finger.name
            if name not in requested or not valid_mask[fi]:
                continue
            terminal_name = terminal_map.get(name)
            if terminal_name is None or terminal_name not in finger.joints:
                continue
            side_name = side_map.get(name)
            locked_names = [terminal_name]
            if side_name is not None and side_name in finger.joints:
                locked_names.append(side_name)
            out = self._fit_terminal_finger_orientation(
                out, target_quat_wxyz[fi], finger, terminal_name, cfg, position_q=position_q
            )
            locked_values = {
                joint_name: float(out[self.qidx(joint_name)]) for joint_name in locked_names
            }
            out = self._refit_finger_position_locked_joints(
                out, target_pos[fi], finger, locked_values, cfg
            )
            for joint_name, value in locked_values.items():
                out[self.qidx(joint_name)] = value
        return self.clip(out)

    def _fuse_mcp_aa_direct_angles(self, q, mcp_aa_angles, valid_mask):
        cfg = self.cfg.mcp_aa_direct or {}
        gains = cfg.get("output_gain", {}) or {}
        signs = cfg.get("sign", {}) or {}
        out = q.copy()
        for task in self._mcp_aa_axis_tasks:
            fi, name = task["finger_index"], task["finger_name"]
            if not valid_mask[fi] or name not in mcp_aa_angles:
                continue
            q_col = self.model.joints[task["joint_id"]].idx_q
            gain = max(float(gains.get(name, 1.0)), 0.0)
            sign = -1.0 if float(signs.get(name, 1.0)) < 0.0 else 1.0
            unweighted = float(
                np.clip(
                    sign * gain * float(mcp_aa_angles[name]),
                    self.lower[q_col] + self.cfg.bounds_eps,
                    self.upper[q_col] - self.cfg.bounds_eps,
                )
            )
            out[q_col] = self._root_flexion_weight(q, name, "side") * unweighted
        return self.clip(out)

    def get_palm_input_yz_x_debug(self, q, quat_wxyz):
        q = self.clip(np.asarray(q, dtype=float).reshape(self.nq))
        quat_wxyz = np.asarray(quat_wxyz, dtype=float).reshape(len(self.fingers), 4)
        palm_q = self.get_palm_quat(q)
        palm_x = self._quat_yz_plane_x_angle_wxyz(palm_q)
        input_x = {
            finger.name: self._quat_yz_plane_x_angle_wxyz(quat_wxyz[fi])
            for fi, finger in enumerate(self.fingers)
        }
        return palm_x, input_x

    def solve(
        self,
        pos,
        quat_wxyz=None,
        q_init=None,
        valid_mask=None,
        mcp_aa_angles=None,
        mcp_aa_target_quat_wxyz=None,
    ):
        """Solve fingertip targets and update the per-frame solver state."""
        return solve_position_ik(
            self, pos, quat_wxyz, q_init, valid_mask, mcp_aa_angles, mcp_aa_target_quat_wxyz
        )

    @staticmethod
    def _quat_xyz_angles_wxyz(quat_wxyz):
        return quat_xyz_angles_wxyz(quat_wxyz)

    @staticmethod
    def _quat_yz_plane_x_angle_wxyz(quat_wxyz):
        return quat_yz_plane_x_angle_wxyz(quat_wxyz)

    @staticmethod
    def _golden_section_minimize(function, lower, upper, iterations):
        return golden_section_minimize(function, lower, upper, iterations)

    def _root_flexion_weight(self, position_q, finger_name, component):
        return root_flexion_weight(self, position_q, finger_name, component)


def default_fingers(hand="right"):
    if hand == "left":
        finger_tip = (-0.00345086, 0.05645613, -0.01457561)
        thumb_tip = (-0.02051656, 0.05175681, 0.01457563)
    elif hand == "right":
        finger_tip = (-0.0034578, 0.0564561, 0.0144244)
        thumb_tip = (0.0210782, 0.0517568, 0.0144244)
    else:
        raise ValueError("hand must be 'right' or 'left'")
    return [
        FingerSpec(
            "thumb",
            ["thumb_j1", "thumb_j2", "thumb_j3"],
            "thumb_link3",
            thumb_tip,
            rot_weight=0.03,
        ),
        FingerSpec("index", ["index_j1", "index_j2", "index_j3"], "index_link3", finger_tip),
        FingerSpec("middle", ["middle_j1", "middle_j2"], "middle_link2", finger_tip),
        FingerSpec("little", ["little_j1", "little_j2", "little_j3"], "little_link3", finger_tip),
    ]
