#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Textual TUI for live MuJoCo vs solver comparison (replaces PyQt/pyqtgraph viewer).

MuJoCo 3D window still opens via ``launch_passive``; this terminal UI shows
per-channel numeric comparison and error stats in a ``DataTable``.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from mujoco_gui_common import _ErrStats
from solver_validation_spec import ChannelPlan, ValidationSpec

try:
    from textual.app import App, ComposeResult
    from textual.containers import Horizontal, Vertical
    from textual.widgets import DataTable, Footer, Header, Static, Checkbox
except ImportError as e:  # pragma: no cover
    raise RuntimeError(
        "unified_solver_textual requires textual: pip install textual"
    ) from e


@dataclass
class _ChannelLive:
    plan: ChannelPlan
    st_q: _ErrStats = field(default_factory=_ErrStats)
    st_dq: _ErrStats = field(default_factory=_ErrStats)
    mj_pos: float = 0.0
    sv_pos: float = 0.0
    mj_vel: float = 0.0
    sv_vel: float = 0.0
    err_pos: float = 0.0
    err_vel: float = 0.0


@dataclass
class _Snapshot:
    sim_time: float = 0.0
    channels: List[_ChannelLive] = field(default_factory=list)
    running: bool = True
    error: Optional[str] = None


def _sample_channels(
    *,
    spec: ValidationSpec,
    solver: Any,
    model: Any,
    data: Any,
    states: List[_ChannelLive],
) -> None:
    q_mj = np.asarray(spec.read_q_from_mj(model, data), dtype=float).reshape((spec.dim_joint,))
    p_mj = np.asarray(spec.read_p_from_mj(model, data), dtype=float).reshape((spec.dim_motor,))
    dq_mj = np.asarray(spec.read_dq_from_mj(model, data), dtype=float).reshape((spec.dim_joint,))
    dp_mj = np.asarray(spec.read_dp_from_mj(model, data), dtype=float).reshape((spec.dim_motor,))

    q_solver = spec.motor_to_joint_position(solver, p_mj)
    dq_solver = spec.motor_to_joint_velocity(solver, q_mj, p_mj, dp_mj)
    p_solver = spec.joint_to_motor_position(solver, q_mj)
    dp_solver = spec.joint_to_motor_velocity(solver, q_mj, p_mj, dq_mj)

    for st in states:
        if st.plan.q_index is not None:
            idx = int(st.plan.q_index)
            mj_p = float(q_mj[idx])
            sv_p = float(q_solver[idx])
            mj_v = float(dq_mj[idx])
            sv_v = float(dq_solver[idx])
        else:
            idx = int(st.plan.p_index)
            mj_p = float(p_mj[idx])
            sv_p = float(p_solver[idx])
            mj_v = float(dp_mj[idx])
            sv_v = float(dp_solver[idx])
        e_p = mj_p - sv_p
        e_v = mj_v - sv_v
        st.mj_pos, st.sv_pos, st.mj_vel, st.sv_vel = mj_p, sv_p, mj_v, sv_v
        st.err_pos, st.err_vel = e_p, e_v
        st.st_q.update(e_p)
        st.st_dq.update(e_v)


def run_unified_textual(
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
    if not spec.channels:
        raise ValueError(f"ValidationSpec(module={spec.module}) has no channels")

    snap_lock = threading.Lock()
    snap = _Snapshot(channels=[_ChannelLive(plan=ch) for ch in spec.channels])
    stop_flag = threading.Event()
    paused = threading.Event()

    def _sim_loop() -> None:
        mujoco_module.mj_forward(model, data)
        step = 0
        try:
            while viewer.is_running() and not stop_flag.is_set():
                mujoco_module.mj_step(model, data)
                step += 1
                if step % max(int(sample_every_n_steps), 1) == 0:
                    if not paused.is_set():
                        _sample_channels(
                            spec=spec, solver=solver, model=model, data=data, states=snap.channels
                        )
                        with snap_lock:
                            snap.sim_time = float(data.time)
                viewer.sync()
        except Exception as e:
            with snap_lock:
                snap.error = str(e)
        finally:
            with snap_lock:
                snap.running = False

    sim_thread = threading.Thread(target=_sim_loop, name="mujoco-sim", daemon=True)
    sim_thread.start()

    win_title = title or f"Solver viewer — {spec.label}"
    period = 1.0 / max(float(render_hz), 1.0)
    _paused = [False]
    _show_vel = [True]

    class ViewerApp(App):
        TITLE = "Kuavo Solver Viewer"
        SUB_TITLE = win_title

        BINDINGS = [
            ("q", "quit", "Quit"),
            ("Q", "quit", "Quit"),
            ("p", "pause_resume", "Pause/Resume"),
            ("r", "reset_stats", "Reset stats"),
            ("s", "screenshot", "Screenshot"),
        ]

        CSS = """
        Screen { layout: vertical; }
        #info { height: 3; padding: 0 1; }
        #header-status { height: 1; padding: 0 1; }
        #footer-summary { height: 1; padding: 0 1; }
        DataTable { height: 1fr; }
        """

        def compose(self) -> ComposeResult:
            yield Header()
            yield Static("", id="header-status")
            yield Static(
                f"module={spec.module}  token={spec.token}\n"
                f"MJCF: {spec.mjcf_path}\n"
                "Close the MuJoCo window or press Q to exit.",
                id="info",
            )
            yield Horizontal(
                Checkbox("Show velocity columns", id="chk-show-vel", value=True),
            )
            yield DataTable(id="table", zebra_stripes=True)
            yield Static("", id="footer-summary")
            yield Footer()

        def on_mount(self) -> None:
            table = self.query_one("#table", DataTable)
            table.add_columns(
                "channel",
                "side",
                "mj_pos",
                "sv_pos",
                "err_pos",
                "pos_max",
                "pos_rmse",
                "mj_vel",
                "sv_vel",
                "err_vel",
                "vel_max",
                "vel_rmse",
            )
            self.set_interval(period, self._refresh)

        def _refresh(self) -> None:
            with snap_lock:
                if snap.error:
                    self.exit(message=snap.error)
                    return
                if not snap.running and not viewer.is_running():
                    self.exit()
                    return
                rows = list(snap.channels)
                t = snap.sim_time

            info = self.query_one("#info", Static)
            info.update(
                f"module={spec.module}  token={spec.token}  t={t:.3f}s\n"
                f"MJCF: {spec.mjcf_path}\n"
                "P=pause/resume R=reset S=screenshot Q=quit"
            )

            paused_str = "PAUSED" if _paused[0] else "running"
            self.query_one("#header-status", Static).update(
                f"Status: {paused_str}  |  Channels: {len(rows)}"
            )

            # Compute worst error across all channels
            worst_pos_abs = max((abs(st.err_pos) for st in rows), default=0.0)
            worst_vel_abs = max((abs(st.err_vel) for st in rows), default=0.0)
            worst_pos_rmse = max((st.st_q.rmse for st in rows), default=0.0)
            worst_vel_rmse = max((st.st_dq.rmse for st in rows), default=0.0)
            footer_text = (
                f"Worst err — pos: max={worst_pos_abs:.3e} rmse={worst_pos_rmse:.3e}  "
                f"vel: max={worst_vel_abs:.3e} rmse={worst_vel_rmse:.3e}"
            )
            self.query_one("#footer-summary", Static).update(footer_text)

            show_vel = _show_vel[0]
            table = self.query_one("#table", DataTable)
            table.clear()
            for st in rows:
                cols = [
                    st.plan.label,
                    st.plan.side,
                    f"{st.mj_pos:.5f}",
                    f"{st.sv_pos:.5f}",
                    f"{st.err_pos:+.3e}",
                    f"{st.st_q.max_abs:.3e}",
                    f"{st.st_q.rmse:.3e}",
                ]
                if show_vel:
                    cols += [
                        f"{st.mj_vel:.5f}",
                        f"{st.sv_vel:.5f}",
                        f"{st.err_vel:+.3e}",
                        f"{st.st_dq.max_abs:.3e}",
                        f"{st.st_dq.rmse:.3e}",
                    ]
                table.add_row(*cols)

        def action_quit(self) -> None:
            stop_flag.set()
            self.exit()

        def action_pause_resume(self) -> None:
            _paused[0] = not _paused[0]
            if _paused[0]:
                paused.set()
            else:
                paused.clear()
            self.notify(
                "Paused" if _paused[0] else "Resumed",
                severity="information",
            )

        def action_reset_stats(self) -> None:
            with snap_lock:
                for st in snap.channels:
                    st.st_q = _ErrStats()
                    st.st_dq = _ErrStats()
                    st.mj_pos = st.sv_pos = st.mj_vel = st.sv_vel = 0.0
                    st.err_pos = st.err_vel = 0.0
            self.notify("Stats reset", severity="information")

        def action_screenshot(self) -> None:
            ts = time.strftime("%Y%m%d_%H%M%S")
            self.save_screenshot(f"solver_viewer_{ts}.svg")
            self.notify(f"Screenshot saved to solver_viewer_{ts}.svg", severity="information")

        def on_checkbox_changed(self, event: Checkbox.Changed) -> None:
            if event.checkbox.id == "chk-show-vel":
                _show_vel[0] = bool(event.value)

    ViewerApp().run()
    stop_flag.set()
    sim_thread.join(timeout=2.0)