#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GUI 通用工具（被 unified_solver_textual.py 复用）。

历史说明
--------
老版本里包含各机构的 ``*_solver_compare`` 函数（`ankle/knee/waist/arm`），
伴随 try/except 静默 fallback。新架构统一把"如何调用 solver 画图"的逻辑收敛到
`solver_validation_spec.ValidationSpec`，所以这里只保留通用的 Qt/plot 工具。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Tuple


@dataclass
class _Series:
    t: list[float]
    y: list[float]

    def append(self, t: float, y: float, max_points: int) -> None:
        self.t.append(float(t))
        self.y.append(float(y))
        if len(self.t) > max_points:
            extra = len(self.t) - max_points
            del self.t[:extra]
            del self.y[:extra]


@dataclass
class _ErrStats:
    max_abs: float = 0.0
    sum_sq: float = 0.0
    n: int = 0

    def update(self, e: float) -> None:
        ae = abs(float(e))
        self.max_abs = max(self.max_abs, ae)
        self.sum_sq += float(e) * float(e)
        self.n += 1

    @property
    def rmse(self) -> float:
        if self.n <= 0:
            return 0.0
        return math.sqrt(self.sum_sq / float(self.n))


def require_qt_pg() -> Tuple[Any, Any, Any]:
    """
    返回 (QtCore, QtWidgets, pg)，并确保 QApplication 已初始化。
    """
    try:
        from PySide6 import QtCore, QtWidgets  # type: ignore
    except Exception as e:  # pragma: no cover
        raise RuntimeError("缺少 PySide6：python3 -m pip install PySide6 pyqtgraph") from e

    try:
        import pyqtgraph as pg  # type: ignore
    except Exception as e:  # pragma: no cover
        raise RuntimeError("缺少 pyqtgraph：python3 -m pip install pyqtgraph") from e

    _app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    _ = _app  # keep reference
    pg.setConfigOptions(antialias=True)
    return QtCore, QtWidgets, pg


def apply_plot_defaults(*plot_widgets: Any) -> None:
    """Grid + legend + 秒单位的 x 轴。"""
    for pw in plot_widgets:
        try:
            pw.showGrid(x=True, y=True, alpha=0.25)
        except Exception:
            pass
        try:
            pw.setLabel("bottom", "t", units="s")
        except Exception:
            pass
        try:
            pw.addLegend()
        except Exception:
            pass


def set_fixed_history_window(*plot_widgets: Any, t_max: float, window_seconds: float) -> None:
    """固定 x 轴宽度为 window_seconds；t_max < window 时仍显示 [0, window]。"""
    t1 = float(t_max)
    t0 = float(t1 - float(window_seconds))
    if t0 < 0.0:
        t0 = 0.0
        t1 = float(window_seconds)
    for pw in plot_widgets:
        try:
            pw.setXRange(float(t0), float(t1), padding=0.0)
        except Exception:
            pass
