"""Stateful fingertip target filtering and joint-output stabilization."""

import numpy as np


class TargetStateFilter:
    def __init__(self, cfg):
        self.cfg = dict(cfg or {})
        self.enabled = bool(self.cfg.get("enabled", False))
        self.x = None
        self.v = None
        self.last_t = None
        self.last_meas = None
        self.static = False
        self.static_count = 0

    def update(self, measured, t_now, valid_mask=None):
        measured = np.asarray(measured, dtype=float).reshape(5, 3)
        if not self.enabled:
            self.last_meas = measured.copy()
            self.last_t = float(t_now)
            return measured.copy(), "disabled"
        if valid_mask is None:
            valid_mask = np.ones(measured.shape[0], dtype=bool)
        else:
            valid_mask = np.asarray(valid_mask, dtype=bool).reshape(measured.shape[0])
        if self.x is None or self.v is None or self.last_t is None:
            self.x = measured.copy()
            self.v = np.zeros_like(measured)
            self.last_t = float(t_now)
            self.last_meas = measured.copy()
            return self.x.copy(), "init"
        dt = max(float(t_now) - self.last_t, 1e-4)
        if self.cfg.get("reset_on_invalid", True) and not np.all(valid_mask):
            self.x[~valid_mask] = measured[~valid_mask]
            self.v[~valid_mask] = 0.0
        x_pred = self.x + self.v * dt
        residual = measured - x_pred
        step = measured - self.last_meas
        max_step = (
            float(np.max(np.linalg.norm(step[valid_mask], axis=1))) if np.any(valid_mask) else 0.0
        )
        max_residual = (
            float(np.max(np.linalg.norm(residual[valid_mask], axis=1)))
            if np.any(valid_mask)
            else 0.0
        )
        max_speed = (
            float(np.max(np.linalg.norm(self.v[valid_mask], axis=1))) if np.any(valid_mask) else 0.0
        )
        enter_step = float(self.cfg.get("static_enter_threshold_m", 0.0003))
        exit_step = float(self.cfg.get("static_exit_threshold_m", 0.00075))
        residual_thr = float(self.cfg.get("static_residual_threshold_m", 0.0006))
        speed_thr = float(self.cfg.get("static_speed_threshold_mps", 0.006))
        if self.static:
            if (
                max_step > exit_step
                or max_residual > residual_thr * 1.5
                or max_speed > speed_thr * 1.5
            ):
                self.static = False
                self.static_count = 0
        else:
            if max_step < enter_step and max_residual < residual_thr and max_speed < speed_thr:
                self.static_count += 1
            else:
                self.static_count = 0
            if self.static_count >= int(self.cfg.get("static_enter_frames", 4)):
                self.static = True
        if self.static:
            alpha = float(self.cfg.get("alpha_static", 0.08))
            beta = float(self.cfg.get("beta_static", 0.002))
            state = "static"
        else:
            alpha = float(self.cfg.get("alpha_moving", 0.85))
            beta = float(self.cfg.get("beta_moving", 0.08))
            state = "moving"
        self.x = x_pred + alpha * residual
        self.v = self.v + (beta / dt) * residual
        vmax = float(self.cfg.get("max_velocity_mps", 0.5))
        speed = np.linalg.norm(self.v, axis=1, keepdims=True)
        self.v *= np.minimum(1.0, vmax / np.clip(speed, 1e-12, None))
        self.x[~valid_mask] = measured[~valid_mask]
        self.v[~valid_mask] = 0.0
        self.last_meas = measured.copy()
        self.last_t = float(t_now)
        return self.x.copy(), state


def output_guard(q_raw, q_prev, state, cfg):
    if not cfg.get("enabled", False) or q_prev is None or state != "static":
        return q_raw.copy()
    q_raw = np.asarray(q_raw, dtype=float)
    q_prev = np.asarray(q_prev, dtype=float)
    dq = q_raw - q_prev
    deadband = float(cfg.get("static_deadband_rad", 0.0008))
    max_dq = float(cfg.get("static_max_dq_rad", 0.004))
    q_out = q_raw.copy()
    small = np.abs(dq) < deadband
    q_out[small] = q_prev[small]
    return q_prev + np.clip(q_out - q_prev, -max_dq, max_dq)
