#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
统一解算器 GUI：由 `ValidationSpec` 驱动。

核心原则
--------
1. 画图数据**全部**走 solver 的 ``motor_to_joint_position`` / ``motor_to_joint_velocity``
   接口——GUI 的真正目的就是"验证 solver 给回的 q/dq 能不能复现 MuJoCo 的真值"。
2. 任何 solver 调用失败都立即抛出（fail-fast）；不使用 try/except 静默 fallback。
3. Tab 按 spec.channels 的 side 分组（left / right / shared）；如果机构只有一组则退化为单 Tab。
4. 每个 Tab 共用 4 张图：``pos``、``vel``、``pos_err``、``vel_err``；每条通道独占颜色。
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

import numpy as np

from mujoco_gui_common import (
    _ErrStats,
    _Series,
    apply_plot_defaults,
    require_qt_pg,
    set_fixed_history_window,
)

from solver_validation_spec import ChannelPlan, ValidationSpec


# 10-色色板（复用 matplotlib tab10）
_COLORS = [
    (31, 119, 180), (255, 127, 14), (44, 160, 44), (214, 39, 40),
    (148, 103, 189), (140, 86, 75), (227, 119, 194), (127, 127, 127),
    (188, 189, 34), (23, 190, 207),
]


def _axes_y_auto_x_manual(pw: Any) -> None:
    vb = pw.getViewBox()
    xa = getattr(vb, "XAxis", 0)
    ya = getattr(vb, "YAxis", 1)
    vb.enableAutoRange(axis=xa, enable=False)
    vb.enableAutoRange(axis=ya, enable=True)


@dataclass
class _ChannelState:
    plan: ChannelPlan
    color_idx: int
    s_q_mj: _Series = field(default_factory=lambda: _Series([], []))
    s_q_sv: _Series = field(default_factory=lambda: _Series([], []))
    s_dq_mj: _Series = field(default_factory=lambda: _Series([], []))
    s_dq_sv: _Series = field(default_factory=lambda: _Series([], []))
    s_e_q: _Series = field(default_factory=lambda: _Series([], []))
    s_e_dq: _Series = field(default_factory=lambda: _Series([], []))
    st_q: _ErrStats = field(default_factory=_ErrStats)
    st_dq: _ErrStats = field(default_factory=_ErrStats)

    c_q_mj: Any = None
    c_q_sv: Any = None
    c_dq_mj: Any = None
    c_dq_sv: Any = None
    c_e_q: Any = None
    c_e_dq: Any = None


def _group_channels(channels: List[ChannelPlan]) -> List[str]:
    sides: List[str] = []
    for ch in channels:
        if ch.side not in sides:
            sides.append(ch.side)
    return sides


def run_unified_gui(
    *,
    spec: ValidationSpec,
    solver: Any,
    mujoco_module: Any,
    model: Any,
    data: Any,
    viewer: Any,
    max_points: int = 800,
    window_seconds: float = 10.0,
    sample_every_n_steps: int = 5,
    render_hz: float = 20.0,
    title: Optional[str] = None,
) -> None:
    """
    启动一个 spec-driven GUI 窗口，同时驱动一个 MuJoCo passive viewer。

    调用约定与旧的 `run_*_gui` 保持一致：
    - `viewer` 由外层 `mujoco.viewer.launch_passive(...)` 产生；
    - 当用户关闭 viewer 时，GUI 也会退出。
    """
    if not spec.channels:
        raise ValueError(f"ValidationSpec(module={spec.module}) 没有可绘制的 channels")
    for ch in spec.channels:
        if ch.q_index is None:
            raise ValueError(f"channel {ch.key} 未配置 q_index，GUI 无法从 solver 画关节曲线")

    QtCore, QtWidgets, pg = require_qt_pg()
    mujoco_module.mj_forward(model, data)

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    win = QtWidgets.QMainWindow()
    win.setWindowTitle(title or f"Kuavo Unified GUI - {spec.label}")

    root_w = QtWidgets.QWidget()
    root = QtWidgets.QVBoxLayout(root_w)
    win.setCentralWidget(root_w)

    grp = QtWidgets.QGroupBox("显示曲线")
    gl = QtWidgets.QHBoxLayout(grp)
    cb_pos = QtWidgets.QCheckBox("位置 (rad/m)")
    cb_vel = QtWidgets.QCheckBox("速度 (rad-m/s)")
    cb_pos.setChecked(True)
    cb_vel.setChecked(True)
    gl.addWidget(cb_pos); gl.addWidget(cb_vel); gl.addStretch(1)
    root.addWidget(grp)

    info_lbl = QtWidgets.QLabel(f"module={spec.module}  token={spec.token}  mjcf={spec.mjcf_path}")
    info_lbl.setStyleSheet("QLabel{font-family: monospace; color:#555;}")
    root.addWidget(info_lbl)

    tabs = QtWidgets.QTabWidget()
    root.addWidget(tabs, 1)

    tab_states: Dict[str, Dict[str, Any]] = {}
    channel_states: List[_ChannelState] = []

    sides = _group_channels(spec.channels)
    for side in sides:
        page = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(page)

        p_pos = pg.PlotWidget()
        p_vel = pg.PlotWidget()
        p_epos = pg.PlotWidget()
        p_evel = pg.PlotWidget()
        apply_plot_defaults(p_pos, p_vel, p_epos, p_evel)
        p_pos.setLabel("left", "q", units="rad/m"); p_pos.setTitle(f"{side}: joint_pos mj vs solver(motor_to_joint_position)")
        p_vel.setLabel("left", "dq", units="rad|m/s"); p_vel.setTitle(f"{side}: joint_vel mj vs solver(motor_to_joint_velocity)")
        p_epos.setLabel("left", "err", units="rad/m"); p_epos.setTitle(f"{side}: position error (mj - solver)")
        p_evel.setLabel("left", "err", units="rad|m/s"); p_evel.setTitle(f"{side}: velocity error (mj - solver)")
        for pw in (p_pos, p_vel, p_epos, p_evel):
            _axes_y_auto_x_manual(pw)

        v.addWidget(p_pos, 1)
        v.addWidget(p_vel, 1)
        v.addWidget(p_epos, 1)
        v.addWidget(p_evel, 1)

        lbl = QtWidgets.QLabel()
        lbl.setStyleSheet("QLabel{font-family: monospace; font-size: 12px;}")
        v.addWidget(lbl, 0)

        tabs.addTab(page, side)
        tab_states[side] = {
            "p_pos": p_pos, "p_vel": p_vel, "p_epos": p_epos, "p_evel": p_evel,
            "lbl": lbl, "channels": [],
        }

    color_cursor = 0
    for ch in spec.channels:
        st = _ChannelState(plan=ch, color_idx=color_cursor % len(_COLORS))
        color_cursor += 1
        col = _COLORS[st.color_idx]
        pen_mj = pg.mkPen(color=col, width=2)
        pen_sv = pg.mkPen(color=col, width=1, style=QtCore.Qt.DashLine)
        pen_err = pg.mkPen(color=col, width=2)

        tab = tab_states[ch.side]
        st.c_q_mj  = tab["p_pos"].plot([], [], pen=pen_mj, name=f"{ch.label}_mj")
        st.c_q_sv  = tab["p_pos"].plot([], [], pen=pen_sv, name=f"{ch.label}_sv")
        st.c_dq_mj = tab["p_vel"].plot([], [], pen=pen_mj, name=f"{ch.label}_mj")
        st.c_dq_sv = tab["p_vel"].plot([], [], pen=pen_sv, name=f"{ch.label}_sv")
        st.c_e_q   = tab["p_epos"].plot([], [], pen=pen_err, name=f"{ch.label}_err")
        st.c_e_dq  = tab["p_evel"].plot([], [], pen=pen_err, name=f"{ch.label}_err")
        tab["channels"].append(st)
        channel_states.append(st)

    def _apply_visibility() -> None:
        sp = bool(cb_pos.isChecked())
        sv = bool(cb_vel.isChecked())
        for tab in tab_states.values():
            tab["p_pos"].setVisible(sp)
            tab["p_epos"].setVisible(sp)
            tab["p_vel"].setVisible(sv)
            tab["p_evel"].setVisible(sv)
    cb_pos.toggled.connect(lambda _=False: _apply_visibility())
    cb_vel.toggled.connect(lambda _=False: _apply_visibility())
    _apply_visibility()

    last_render_wall = time.time()
    render_period = 1.0 / max(float(render_hz), 1.0)
    step = 0

    def _sample_and_store() -> None:
        t = float(data.time)
        q_mj = np.asarray(spec.read_q_from_mj(model, data), dtype=float).reshape((spec.dim_joint,))
        p_mj = np.asarray(spec.read_p_from_mj(model, data), dtype=float).reshape((spec.dim_motor,))
        dq_mj = np.asarray(spec.read_dq_from_mj(model, data), dtype=float).reshape((spec.dim_joint,))
        dp_mj = np.asarray(spec.read_dp_from_mj(model, data), dtype=float).reshape((spec.dim_motor,))

        # ⚠️ 下面两个调用如果失败就直接抛（fail-fast，不吞异常）
        q_solver = spec.motor_to_joint_position(solver, p_mj)
        dq_solver = spec.motor_to_joint_velocity(solver, q_mj, p_mj, dp_mj)

        for st in channel_states:
            idx = int(st.plan.q_index)
            q_r = float(q_mj[idx]); q_h = float(q_solver[idx])
            dq_r = float(dq_mj[idx]); dq_h = float(dq_solver[idx])
            e_q = q_r - q_h; e_dq = dq_r - dq_h

            st.s_q_mj.append(t, q_r, max_points)
            st.s_q_sv.append(t, q_h, max_points)
            st.s_dq_mj.append(t, dq_r, max_points)
            st.s_dq_sv.append(t, dq_h, max_points)
            st.s_e_q.append(t, e_q, max_points)
            st.s_e_dq.append(t, e_dq, max_points)
            st.st_q.update(e_q)
            st.st_dq.update(e_dq)

    def _render() -> None:
        # 先把曲线刷上
        t_max = 0.0
        for st in channel_states:
            if st.s_q_mj.t:
                t_max = max(t_max, float(st.s_q_mj.t[-1]))
            if cb_pos.isChecked():
                st.c_q_mj.setData(st.s_q_mj.t, st.s_q_mj.y)
                st.c_q_sv.setData(st.s_q_sv.t, st.s_q_sv.y)
                st.c_e_q.setData(st.s_e_q.t, st.s_e_q.y)
            if cb_vel.isChecked():
                st.c_dq_mj.setData(st.s_dq_mj.t, st.s_dq_mj.y)
                st.c_dq_sv.setData(st.s_dq_sv.t, st.s_dq_sv.y)
                st.c_e_dq.setData(st.s_e_dq.t, st.s_e_dq.y)

        # 统一时间轴
        if t_max > 0.0:
            for tab in tab_states.values():
                set_fixed_history_window(
                    tab["p_pos"], tab["p_vel"], tab["p_epos"], tab["p_evel"],
                    t_max=t_max, window_seconds=window_seconds,
                )

        # 每个 Tab 的 err 摘要 label
        for tab in tab_states.values():
            lines = []
            for st in tab["channels"]:
                lines.append(
                    f"{st.plan.label:14s} pos err max={st.st_q.max_abs:.3e} rmse={st.st_q.rmse:.3e}  "
                    f"vel err max={st.st_dq.max_abs:.3e} rmse={st.st_dq.rmse:.3e}"
                )
            tab["lbl"].setText("\n".join(lines))

    def _tick() -> None:
        nonlocal step, last_render_wall
        if not viewer.is_running():
            win.close()
            QtCore.QTimer.singleShot(0, app.quit)
            return
        mujoco_module.mj_step(model, data)
        step += 1
        if step % max(int(sample_every_n_steps), 1) == 0:
            _sample_and_store()
            now = time.time()
            if (now - last_render_wall) >= render_period:
                last_render_wall = now
                _render()
        viewer.sync()

    win.resize(1100, 900)
    win.show()

    timer = QtCore.QTimer()
    timer.setTimerType(QtCore.Qt.PreciseTimer)
    timer.timeout.connect(_tick)
    timer.start(0)
    app.exec()
