"""Damped least-squares position and bend-orientation solve."""

import numpy as np
import pinocchio as pin


def solve_position_ik(
    self,
    pos,
    quat_wxyz=None,
    q_init=None,
    valid_mask=None,
    mcp_aa_angles=None,
    mcp_aa_target_quat_wxyz=None,
):
    c = self.cfg
    pos = np.asarray(pos, dtype=float).reshape(len(self.fingers), 3)
    use_rot = quat_wxyz is not None
    if use_rot:
        quat_wxyz = np.asarray(quat_wxyz, dtype=float).reshape(len(self.fingers), 4)
    if valid_mask is None:
        valid_mask = np.ones(len(self.fingers), dtype=bool)
    else:
        valid_mask = np.asarray(valid_mask, dtype=bool).reshape(len(self.fingers))
    q = self._neutral.copy() if q_init is None else self.clip(np.asarray(q_init, dtype=float))
    q_ref = q.copy()
    staged_cfg = c.four_finger_flexion_orientation_ik or {}
    orientation_first = (
        use_rot
        and bool(staged_cfg.get("enabled", False))
        and bool(staged_cfg.get("skip_initial_position_ik", True))
    )
    deferred = (
        set(staged_cfg.get("fingers", ["index", "middle", "little"])) - {"thumb"}
        if orientation_first
        else set()
    )
    cols = np.asarray(
        [self.qidx(jn) for f in self.fingers if f.name not in deferred for jn in f.joints],
        dtype=int,
    )
    desired = []
    for i in range(len(self.fingers)):
        if use_rot:
            qw, qx, qy, qz = quat_wxyz[i]
            nq = max(float(np.linalg.norm(quat_wxyz[i])), 1e-12)
            Rdes = pin.Quaternion(qw / nq, qx / nq, qy / nq, qz / nq).matrix()
        else:
            Rdes = np.eye(3)
        desired.append(pin.SE3(Rdes, pos[i]))
    last_step_norm = float("inf")
    for _ in range(int(c.max_iters)):
        if cols.size == 0:
            break
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        pin.computeJointJacobians(self.model, self.data, q)
        pos_errs, pos_jacs, bend_errs, bend_jacs = [], [], [], []
        for i, f in enumerate(self.fingers):
            if not valid_mask[i] or f.name in deferred:
                continue
            oMf = self.data.oMf[f.tip_frame_id]
            e_pos = desired[i].translation - oMf.translation
            J = pin.computeFrameJacobian(
                self.model, self.data, q, f.tip_frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
            )
            pos_errs.append(f.pos_weight * e_pos)
            pos_jacs.append(f.pos_weight * J[:3, cols])
            four_stage = bool((c.four_finger_flexion_orientation_ik or {}).get("enabled", False))
            if use_rot and f.rot_weight > 0.0 and (not four_stage or f.name == "thumb"):
                e_rot = pin.log3(desired[i].rotation @ oMf.rotation.T)
                task = self._bend_orientation_tasks[f.name]
                bend_axis = np.asarray(J[3:, task["axis_v_index"]], dtype=float).reshape(3)
                axis_norm = float(np.linalg.norm(bend_axis))
                if axis_norm > 1e-12:
                    bend_axis /= axis_norm
                    bend_jac = bend_axis @ J[3:, cols]
                    allowed = np.isin(cols, task["active_q_indices"])
                    bend_jac = np.where(allowed, bend_jac, 0.0)
                    rot_weight = float(f.rot_weight)
                    bend_errs.append(rot_weight * np.array([np.dot(e_rot, bend_axis)]))
                    bend_jacs.append(rot_weight * bend_jac.reshape(1, -1))
        errs = list(pos_errs) + list(bend_errs)
        jacs = list(pos_jacs) + list(bend_jacs)
        if c.smooth_weight > 0:
            sw = np.sqrt(c.smooth_weight)
            errs.append(sw * (q_ref[cols] - q[cols]))
            jacs.append(sw * np.eye(cols.size))
        if c.posture_weight > 0:
            pw = np.sqrt(c.posture_weight)
            errs.append(pw * (self._neutral[cols] - q[cols]))
            jacs.append(pw * np.eye(cols.size))
        if not errs:
            break
        e = np.concatenate(errs)
        Jb = np.vstack(jacs)
        JtJ = Jb.T @ Jb
        lam = c.damping_lambda0 * (1.0 + np.trace(JtJ) / max(JtJ.shape[0], 1))
        dq = np.linalg.solve(JtJ + lam * np.eye(cols.size), Jb.T @ e)
        dq *= c.step_eta
        last_step_norm = float(np.linalg.norm(dq))
        if last_step_norm > c.max_step:
            dq *= c.max_step / last_step_norm
        q[cols] = q[cols] + dq
        q = self.clip(q)
        if last_step_norm < c.pos_tol:
            break
    position_q = q.copy()
    self.last_position_ik_q = position_q.copy()
    self.last_side_orientation_fit = {}
    self.last_terminal_orientation_fit = {}
    if use_rot:
        if (c.mcp_aa_direct or {}).get("enabled", False) and mcp_aa_angles:
            q = self._fuse_mcp_aa_direct_angles(q, mcp_aa_angles, valid_mask)
        if (c.side_orientation_ik or {}).get(
            "enabled", False
        ) and mcp_aa_target_quat_wxyz is not None:
            targets = np.asarray(mcp_aa_target_quat_wxyz, dtype=float).reshape(len(self.fingers), 4)
            q = self._fit_mcp_aa_with_pin(q, targets, valid_mask)
        q = self._solve_four_finger_flexion_then_position(
            q, quat_wxyz, pos, valid_mask, position_q=position_q
        )
    self.last_pre_fuse_q = q.copy()
    return q
