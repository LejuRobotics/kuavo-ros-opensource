#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
脚踝可行域评估（bar 限位 → pitch/roll 可行域散点图）。

口径：
- 仅调用 kuavo_solver 的原始接口 AxisOffsetAnkleSolver.motor_to_joint_position(p4)
  （通过 solver_validation_spec.build_spec 构造同一套 4D 约定）。
- 不做 silent fallback：求解异常会被统计并输出（脚本不中断）。

4D 约定：
  p4 = [L_l_bar, L_r_bar, R_l_bar, R_r_bar]
  q4 = [L_pitch, L_roll, R_pitch, R_roll]

用法示例：
  python3 ankle_feasible_region_plot.py --version 70 --grid 121
  python3 ankle_feasible_region_plot.py --version 70 --legs left --grid 61 --no-summary-json
  python3 ankle_feasible_region_plot.py --version 70 --symmetry_check --save-npz
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
import xml.etree.ElementTree as ET
from collections import Counter
from dataclasses import dataclass
from itertools import product
from typing import Any, Dict, List, Optional, Tuple

import numpy as np


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_SCRIPTS_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
for _sub in ("lib", "validation"):
    _p = os.path.join(_SCRIPTS_ROOT, _sub)
    if _p not in sys.path:
        sys.path.insert(0, _p)
if _SCRIPTS_ROOT not in sys.path:
    sys.path.insert(0, _SCRIPTS_ROOT)


# 与 mujoco_unified_cli.py 一致：`scripts/tools` → scripts → kuavo_solver → …/kuavo-ros-control
KUAVO_SOLVER_ROOT_FOR_PY = os.path.abspath(os.path.join(_SCRIPTS_ROOT, ".."))
_REPO_ROOT_FOR_PY = os.path.abspath(os.path.join(KUAVO_SOLVER_ROOT_FOR_PY, "..", ".."))


def _setup_solver_path() -> None:
    """
    与 ``mujoco_unified_cli._setup_solver_path`` 等价：在无 ROS/source 时也尽量挂载
    ``build/kuavo_solver/python``、devel/install、以及在 build 下浅搜 kuavo_solver_py.so。
    """
    try:
        import kuavo_solver_py as _  # type: ignore  # noqa: F401

        return
    except Exception:
        pass

    def _add(p: str) -> None:
        if p and os.path.isdir(p) and p not in sys.path:
            sys.path.insert(0, p)

    # 与本包 CMake 编译产物（常见于 kuavo-ros-control/build/kuavo_solver/python）
    _add(os.path.join(_REPO_ROOT_FOR_PY, "build", "kuavo_solver", "python"))
    # 部分工作区把 binding 建在仓库旁（沿用历史路径）
    _add(os.path.abspath(os.path.join(_REPO_ROOT_FOR_PY, "..", "build_kuavo_solver", "python")))
    _add(KUAVO_SOLVER_ROOT_FOR_PY)
    _add(os.path.join(KUAVO_SOLVER_ROOT_FOR_PY, "python"))

    for base in ("devel", "install"):
        root = os.path.join(_REPO_ROOT_FOR_PY, base)
        for rel in (
            "lib/python3/dist-packages",
            "lib/python3.8/dist-packages",
            "lib/python3.10/dist-packages",
            "lib/python3/site-packages",
            "lib/python3.8/site-packages",
            "lib/python3.10/site-packages",
        ):
            _add(os.path.join(root, rel))

    max_depth = 6
    for start in (os.path.join(_REPO_ROOT_FOR_PY, "build"), _REPO_ROOT_FOR_PY):
        if not os.path.isdir(start):
            continue
        for cur, dirs, _files in os.walk(start):
            depth = cur[len(start) :].count(os.sep)
            if depth > max_depth:
                dirs[:] = []
                continue
            if os.path.basename(cur) not in ("site-packages", "dist-packages"):
                continue
            try:
                entries = os.listdir(cur)
            except Exception:
                continue
            if any(e.startswith("kuavo_solver_py") for e in entries):
                _add(cur)


def _import_solver_py():
    _setup_solver_path()
    try:
        import kuavo_solver_py as solver_py  # type: ignore

        # 便于对照「运行时到底加载哪一份 .so」；若与刚编译的路径不一致请先检查 PYTHONPATH / source setup.bash。
        sf = getattr(solver_py, "__file__", None)
        print(f"[ankle_feasible] kuavo_solver_py: {sf}", flush=True)
        return solver_py
    except ImportError as e:
        tried = []
        for cand in (
            os.path.join(_REPO_ROOT_FOR_PY, "build", "kuavo_solver", "python"),
            os.path.join(KUAVO_SOLVER_ROOT_FOR_PY, "python"),
            os.path.join(_REPO_ROOT_FOR_PY, "devel", "lib", "python3", "dist-packages"),
        ):
            tried.append(f"  {'EXISTS' if os.path.isdir(cand) else 'MISS'} {cand}")
        hint = "\n".join(tried[:8])
        raise RuntimeError(
            "无法 import kuavo_solver_py。请先在本仓库构建 Python 绑定，例如在 kuavo-ros-control 目录下编译 "
            "`kuavo_solver`（生成 build/kuavo_solver/python/kuavo_solver_py*.so），或 source 已就绪的 ROS 环境。\n"
            f"当前 python={sys.executable}\n"
            "以下为常见候选路径是否存在（MISS 则说明尚未构建或未装到此处）：\n"
            + hint
        ) from e


def _bar_axis_delta(lo: float, hi: float) -> float:
    """在限位区间内取一小扰动用于自检。"""
    a, b = float(lo), float(hi)
    lo2, hi2 = (a, b) if a <= b else (b, a)
    span = hi2 - lo2
    return max(2e-3, min(0.08, 0.04 * span))


def _verify_solver_p4_four_bars_are_independent(
    solver: object, bl: BarLimits, *, atol: float = 1e-8
) -> None:
    """
    与 AnkleSolver 的 p4 ↔ leg12 对齐：左腿两根杆、右腿两根杆须在 motor→joint 中独立起效。

    旧版 kuavo_solver_py 曾把左腿第二杆写错位，导致 Pick 读到的 L_r_bar 恒为 0。
    此时「只动 L_l_bar」左翼会变，「只动 L_r_bar」左翼关节几乎不变 —— 本函数专门检测该模式。
    若本检查抛错，请重新编译 kuavo_solver 的 Python 扩展后再跑本脚本。
    """
    ll0, lr0, rl0, rr0 = bl.mids()
    p_mid = np.array([ll0, lr0, rl0, rr0], dtype=float)

    def _q_LR(p: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        q = np.asarray(solver.motor_to_joint_position(np.asarray(p, dtype=float)), dtype=float).reshape((4,))
        return np.array([q[0], q[1]], dtype=float), np.array([q[2], q[3]], dtype=float)

    def _clip(side: str, v: float) -> float:
        if side == "ll":
            lo, hi = bl.l_l
        elif side == "lr":
            lo, hi = bl.l_r
        elif side == "rl":
            lo, hi = bl.r_l
        elif side == "rr":
            lo, hi = bl.r_r
        else:
            raise ValueError(side)
        if lo <= hi:
            return float(min(hi - 1e-9, max(lo + 1e-9, v)))
        return float(min(lo - 1e-9, max(hi + 1e-9, v)))

    d_ll = _bar_axis_delta(bl.l_l[0], bl.l_l[1])
    d_lr = _bar_axis_delta(bl.l_r[0], bl.l_r[1])

    pll = p_mid.copy()
    pll[0] = _clip("ll", float(pll[0] + d_ll))

    plr = p_mid.copy()
    plr[1] = _clip("lr", float(plr[1] + d_lr))

    qLl_m, qRl_m = _q_LR(p_mid)
    qLl_a, qRl_a = _q_LR(pll)
    qLl_b, qRl_b = _q_LR(plr)

    d_left_from_ll = float(np.linalg.norm(qLl_a - qLl_m))
    d_left_from_lr = float(np.linalg.norm(qLl_b - qLl_m))
    d_right_spurious_ll = float(np.linalg.norm(qRl_a - qRl_m))
    d_right_spurious_lr = float(np.linalg.norm(qRl_b - qRl_m))

    # 典型旧绑定：左翼对 L_r_bar 无响应（或極弱），只对 L_l_bar 响应
    if d_left_from_ll > 1e-6 and d_left_from_lr <= atol:
        raise RuntimeError(
            "kuavo_solver_py 自检失败：左翼 pitch/roll 对「只增加 L_r_bar」几乎无响应，"
            "但对「只增加 L_l_bar」有明显响应。\n"
            "这与旧版绑定把左腿第二杆未写入 leg12[6] 的现象一致。**请重新编译并确保运行的 .so "
            "为新版**（参见 `python/kuavo_solver_pybind.cpp` 内 MakeP12FromP4）。\n"
            f"d_left(||Δq|| from ΔL_l)={d_left_from_ll:.6g}, d_left(||Δq|| from ΔL_r)={d_left_from_lr:.6g}"
        )

    if d_left_from_lr < atol and d_left_from_ll < atol:
        raise RuntimeError(
            "kuavo_solver_py 自检失败：沿 L_l_bar、L_r_bar 微扰时左翼均无响应，请检查扩展是否损坏或指针错误。"
        )

    # 只调左翼两根杆时，右翼数值解一般变化很小（双踝 kinematically 分开）；仅巨大串扰告警
    if d_right_spurious_ll > max(0.25, d_left_from_ll * 0.3) or d_right_spurious_lr > max(
        0.25, max(d_left_from_lr, d_left_from_ll) * 0.3
    ):
        print(
            "[ankle_feasible] warning: 微调左翼杆时令右翼 pitch/roll 变化偏大："
            f"ΔR(ll)={d_right_spurious_ll:.4g}, ΔR(lr)={d_right_spurious_lr:.4g}"
        )


@dataclass(frozen=True)
class BarLimits:
    l_l: Tuple[float, float]
    l_r: Tuple[float, float]
    r_l: Tuple[float, float]
    r_r: Tuple[float, float]

    def mids(self) -> Tuple[float, float, float, float]:
        return (
            0.5 * (self.l_l[0] + self.l_l[1]),
            0.5 * (self.l_r[0] + self.l_r[1]),
            0.5 * (self.r_l[0] + self.r_l[1]),
            0.5 * (self.r_r[0] + self.r_r[1]),
        )


@dataclass
class ScanResult:
    ok_points: np.ndarray  # shape (K,2) [pitch, roll]  (仅表示 solver 正常返回且 finite 的点)
    in_limits_mask: np.ndarray  # shape (K,) 是否落在 (pitch_limits, roll_limits) 内
    total: int
    ok: int
    fail: int
    exc_counter: Counter
    exc_samples: Dict[str, List[str]]
    sample_pitch: np.ndarray  # shape (Np,)
    sample_roll: np.ndarray   # shape (Nr,)
    feasible_grid: np.ndarray  # shape (Np,Nr) bool

    def in_design_fraction(self) -> Optional[float]:
        """在全部成功采样点中，落在 ankle_pitch/roll 设计范围内的比例。"""
        if self.ok_points.shape[0] == 0 or self.in_limits_mask.size == 0:
            return None
        return float(np.mean(self.in_limits_mask.astype(float)))


def _empty_scan_result() -> ScanResult:
    return ScanResult(
        ok_points=np.zeros((0, 2), dtype=float),
        in_limits_mask=np.zeros((0,), dtype=bool),
        total=0,
        ok=0,
        fail=0,
        exc_counter=Counter(),
        exc_samples={},
        sample_pitch=np.zeros((0,), dtype=float),
        sample_roll=np.zeros((0,), dtype=float),
        feasible_grid=np.zeros((0, 0), dtype=bool),
    )


def _ankle_joint_limits_from_params_yaml(*, params_yaml: str, token: str) -> Tuple[Tuple[float, float], Tuple[float, float]]:
    """
    从 axisoffsetanklesolver.yaml 中读取指定 token 的踝 pitch/roll 限位。

    这是“可行域”的物理口径：motor→joint 的数学解如果超限，应视为不可行点而不是绘图点。
    缺字段直接报错（不回退）。
    """
    if not params_yaml or not os.path.isfile(params_yaml):
        raise FileNotFoundError(f"params_yaml 不存在: {params_yaml}")
    try:
        import yaml
    except Exception as e:
        raise RuntimeError("缺少 PyYAML（yaml），无法读取 solver params_yaml") from e
    with open(params_yaml, "r", encoding="utf-8") as f:
        root = yaml.safe_load(f) or {}
    if not isinstance(root, dict):
        raise RuntimeError(f"params_yaml 格式非法（非 dict）: {params_yaml}")
    variants = root.get("variants") or {}
    if not isinstance(variants, dict) or token not in variants:
        raise RuntimeError(f"params_yaml 中找不到 variants[{token!r}]；params_yaml={params_yaml}")
    v = variants[token] or {}
    if not isinstance(v, dict):
        raise RuntimeError(f"variants[{token!r}] 非 dict；params_yaml={params_yaml}")
    pitch_limits = v.get("ankle_pitch_limits")
    roll_limits = v.get("ankle_roll_limits")
    if not (isinstance(pitch_limits, (list, tuple)) and len(pitch_limits) == 2):
        raise RuntimeError(f"ankle_pitch_limits 缺失或格式错误；token={token} params_yaml={params_yaml}")
    if not (isinstance(roll_limits, (list, tuple)) and len(roll_limits) == 2):
        raise RuntimeError(f"ankle_roll_limits 缺失或格式错误；token={token} params_yaml={params_yaml}")
    return (float(pitch_limits[0]), float(pitch_limits[1])), (float(roll_limits[0]), float(roll_limits[1]))


def _in_range(x: float, lo: float, hi: float, eps: float = 1e-9) -> bool:
    lo2, hi2 = (lo, hi) if lo <= hi else (hi, lo)
    return (x >= lo2 - eps) and (x <= hi2 + eps)


def _polar_envelope(
    points_pitch_roll: np.ndarray,
    *,
    center_pitch_roll: Optional[Tuple[float, float]] = None,
    angle_bins: int = 720,
    r_quantile: float = 0.995,
) -> np.ndarray:
    """
    方案 A：按角度分桶，在每个桶内取 r 的高分位数作为外包络。

    返回边界点数组 shape (M,2)，每行 [pitch, roll]，按角度顺序排列（首尾不强制相同）。
    """
    pts = np.asarray(points_pitch_roll, dtype=float).reshape((-1, 2))
    if pts.shape[0] < 10:
        return np.zeros((0, 2), dtype=float)
    if not (0.0 < float(r_quantile) <= 1.0):
        raise ValueError("r_quantile must be in (0,1]")
    if int(angle_bins) < 36:
        raise ValueError("angle_bins too small (suggest >= 360)")

    if center_pitch_roll is None:
        c = np.median(pts, axis=0)
        c_pitch, c_roll = float(c[0]), float(c[1])
    else:
        c_pitch, c_roll = float(center_pitch_roll[0]), float(center_pitch_roll[1])

    dp = pts[:, 0] - c_pitch
    dr = pts[:, 1] - c_roll
    theta = np.arctan2(dp, dr)  # angle in [pitch,roll] plane (x=roll, y=pitch)
    r = np.sqrt(dp * dp + dr * dr)

    # map theta ∈ [-pi, pi) → bin [0..angle_bins-1]
    t01 = (theta + np.pi) / (2.0 * np.pi)
    b = np.floor(t01 * float(angle_bins)).astype(int)
    b = np.clip(b, 0, int(angle_bins) - 1)

    env_pts: List[Tuple[float, float]] = []
    for bi in range(int(angle_bins)):
        idx = np.nonzero(b == bi)[0]
        if idx.size == 0:
            continue
        r_q = float(np.quantile(r[idx], float(r_quantile)))
        th = (float(bi) + 0.5) * (2.0 * np.pi / float(angle_bins)) - np.pi
        # x=roll, y=pitch
        roll = c_roll + r_q * np.cos(th)
        pitch = c_pitch + r_q * np.sin(th)
        env_pts.append((pitch, roll))

    if not env_pts:
        return np.zeros((0, 2), dtype=float)
    return np.asarray(env_pts, dtype=float).reshape((-1, 2))


def _envelope_center(points_pitch_roll: np.ndarray) -> Tuple[float, float]:
    pts = np.asarray(points_pitch_roll, dtype=float).reshape((-1, 2))
    if pts.shape[0] == 0:
        return 0.0, 0.0
    c = np.median(pts, axis=0)
    return float(c[0]), float(c[1])


def _fit_min_symmetric_inscribed_rhombus(
    points_pitch_roll: np.ndarray,
    *,
    design_pitch_limits: Optional[Tuple[float, float]] = None,
    design_roll_limits: Optional[Tuple[float, float]] = None,
    roll0_eps_ratio: float = 0.01,
    roll_edge_eps_ratio: float = 0.01,
    ray_perp_tol_ratio: float = 0.02,
    envelope_angle_bins: int = 1440,
    envelope_r_quantile: float = 0.995,
    min_points: int = 10,
) -> Dict[str, object]:
    """
    先拟合外包络线，再取射线与外包络线交点构造“最大内接四边形”（对称于 roll=0）：

    - 上/下顶点：roll=0 附近的 pitch 最大/最小值
    - 边A：设计边 roll=design_roll_max（design_roll_limits[1]）
    - 用外包络线与 roll=roll_max 求交，得到 Ct/Cb（仍在 roll=roll_max 上）
    - 由 Vt->Ct 与 Vb->Cb 发射射线；分别求射线与外包络线的“更远交点”
    - 右顶点：取上下射线“更远交点”的平均；左顶点做 roll 镜像
    """
    pts = np.asarray(points_pitch_roll, dtype=float).reshape((-1, 2))
    if pts.shape[0] < min_points:
        return {"method": "max_inscribed_quadrilateral_envelope", "note": "too_few_points"}

    pitch = pts[:, 0]
    roll = pts[:, 1]

    if design_roll_limits is None:
        return {"method": "max_inscribed_quadrilateral_envelope", "note": "missing_design_roll_limits"}

    roll_max = float(design_roll_limits[1])
    roll_min = float(design_roll_limits[0])
    roll_span = float(abs(roll_max - roll_min))
    if not np.isfinite(roll_span) or roll_span < 1e-12:
        roll_span = float(max(1e-6, np.max(np.abs(roll))))

    design_used = {"pitch_limits": design_pitch_limits, "roll_limits": design_roll_limits}

    roll0_eps = float(max(1e-9, roll_span * roll0_eps_ratio))
    near_zero = np.abs(roll) <= roll0_eps
    if int(np.sum(near_zero)) < 3:
        p_top = float(np.max(pitch))
        p_bot = float(np.min(pitch))
    else:
        p_top = float(np.max(pitch[near_zero]))
        p_bot = float(np.min(pitch[near_zero]))

    if not np.isfinite(p_top) or not np.isfinite(p_bot) or p_top <= p_bot + 1e-12:
        return {"method": "max_inscribed_quadrilateral_envelope", "note": "invalid_roll0_vertices"}

    c_pitch = 0.5 * (p_top + p_bot)

    env = _polar_envelope(
        pts,
        center_pitch_roll=(c_pitch, 0.0),
        angle_bins=int(envelope_angle_bins),
        r_quantile=float(envelope_r_quantile),
    )
    env = np.asarray(env, dtype=float).reshape((-1, 2))
    if env.shape[0] < 3:
        half_width = float(np.max(np.abs(roll)))
        vertices = [[p_top, 0.0], [c_pitch, half_width], [p_bot, 0.0], [c_pitch, -half_width]]
        area = float(2.0 * (p_top - c_pitch) * half_width)
        return {
            "method": "max_inscribed_quadrilateral_envelope",
            "pitch_min": float(p_bot),
            "pitch_max": float(p_top),
            "center_pitch_roll": [float(c_pitch), 0.0],
            "vertices_pitch_roll": vertices,
            "area": area,
            "design_used": design_used,
            "note": "envelope_degenerate",
        }

    M = int(env.shape[0])

    def _cross2(a: np.ndarray, b: np.ndarray) -> float:
        return float(a[0] * b[1] - a[1] * b[0])

    def _intersections_with_roll_const(poly: np.ndarray, roll_const: float) -> List[float]:
        pitches: List[float] = []
        eps = float(1e-12)
        for i in range(M - 1):
            p1, r1 = float(poly[i, 0]), float(poly[i, 1])
            p2, r2 = float(poly[i + 1, 0]), float(poly[i + 1, 1])
            denom = (r2 - r1)
            if abs(denom) <= eps:
                if abs(r1 - roll_const) <= eps:
                    pitches.append(p1)
                continue
            if (r1 - roll_const) * (r2 - roll_const) <= 0.0:
                t = (roll_const - r1) / denom
                if -eps <= t <= 1.0 + eps:
                    pitches.append(p1 + t * (p2 - p1))
        # close
        p1, r1 = float(poly[M - 1, 0]), float(poly[M - 1, 1])
        p2, r2 = float(poly[0, 0]), float(poly[0, 1])
        denom = (r2 - r1)
        if abs(denom) > eps:
            if (r1 - roll_const) * (r2 - roll_const) <= 0.0:
                t = (roll_const - r1) / denom
                if -eps <= t <= 1.0 + eps:
                    pitches.append(p1 + t * (p2 - p1))
        if not pitches:
            return []
        # dedup
        ps_sorted = sorted(pitches)
        dedup: List[float] = []
        for p in ps_sorted:
            if not dedup or abs(p - dedup[-1]) > 1e-9:
                dedup.append(p)
        return dedup

    # roll=roll_max 与包络求交得到 Ct/Cb
    pitches_on_edge = _intersections_with_roll_const(env, roll_max)
    if len(pitches_on_edge) >= 2:
        p_edge_top = float(np.max(pitches_on_edge))
        p_edge_bot = float(np.min(pitches_on_edge))
    elif len(pitches_on_edge) == 1:
        p_edge_top = float(pitches_on_edge[0])
        p_edge_bot = float(pitches_on_edge[0])
    else:
        # 退化：回退到离散点近似
        edge_eps = float(max(1e-9, roll_span * roll_edge_eps_ratio))
        dif = np.abs(roll - roll_max)
        near_edge = dif <= edge_eps
        if int(np.sum(near_edge)) < 3:
            near_edge = dif <= (float(np.min(dif)) + edge_eps)
        p_edge_top = float(np.max(pitch[near_edge]))
        p_edge_bot = float(np.min(pitch[near_edge]))

    Vt = np.asarray([p_top, 0.0], dtype=float)
    Vb = np.asarray([p_bot, 0.0], dtype=float)
    Ct = np.asarray([p_edge_top, roll_max], dtype=float)
    Cb = np.asarray([p_edge_bot, roll_max], dtype=float)

    ray_up = Ct - Vt
    ray_dn = Cb - Vb

    def _ray_poly_intersections(ray_origin: np.ndarray, ray_dir: np.ndarray) -> List[Tuple[float, np.ndarray]]:
        out: List[Tuple[float, np.ndarray]] = []
        eps = float(1e-12)
        d = np.asarray(ray_dir, dtype=float).reshape((2,))
        if np.linalg.norm(d) < 1e-12:
            return out
        for i in range(M - 1):
            a = env[i]
            b = env[i + 1]
            s = np.asarray(b - a, dtype=float).reshape((2,))
            denom = _cross2(d, s)
            if abs(denom) <= eps:
                continue
            t = _cross2(a - ray_origin, s) / denom
            u = _cross2(a - ray_origin, d) / denom
            if t >= -1e-10 and -1e-10 <= u <= 1.0 + 1e-10:
                pt = ray_origin + t * d
                out.append((float(t), pt))
        # close segment
        a = env[M - 1]
        b = env[0]
        s = np.asarray(b - a, dtype=float).reshape((2,))
        denom = _cross2(d, s)
        if abs(denom) > eps:
            t = _cross2(a - ray_origin, s) / denom
            u = _cross2(a - ray_origin, d) / denom
            if t >= -1e-10 and -1e-10 <= u <= 1.0 + 1e-10:
                pt = ray_origin + t * d
                out.append((float(t), pt))
        return out

    ints_up = _ray_poly_intersections(Vt, ray_up)
    ints_dn = _ray_poly_intersections(Vb, ray_dn)

    # 关键：四边形右顶点应位于“上射线 与 下射线”的交点（这样四边形边才会沿射线）。
    d1 = Ct - Vt
    d2 = Cb - Vb
    den = _cross2(d1, d2)
    if abs(den) < 1e-14:
        # 两射线平行：无法得到唯一焦点；返回一个退化但仍保证 roll=0 上下顶点正确的四边形。
        p_right = float(0.5 * (p_edge_top + p_edge_bot))
        r_right = float(abs(roll_max))
        note_parallel = True
    else:
        Vdiff = Vb - Vt
        s = _cross2(Vdiff, d2) / den
        I = Vt + s * d1
        p_right = float(I[0])
        r_right = float(I[1])
        if r_right < 0:
            r_right = -r_right
        note_parallel = False

    vertices = [[p_top, 0.0], [p_right, r_right], [p_bot, 0.0], [p_right, -r_right]]
    x = np.asarray([v[0] for v in vertices], dtype=float)
    y = np.asarray([v[1] for v in vertices], dtype=float)
    area = 0.5 * float(abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))))

    return {
        "method": "max_inscribed_quadrilateral_envelope_intersection",
        "pitch_min": float(p_bot),
        "pitch_max": float(p_top),
        "center_pitch_roll": [float(c_pitch), 0.0],
        "vertices_pitch_roll": vertices,
        "area": area,
        "design_used": design_used,
        "ray_used": {
            "Vt": [float(Vt[0]), float(Vt[1])],
            "Vb": [float(Vb[0]), float(Vb[1])],
            "Ct": [float(Ct[0]), float(Ct[1])],
            "Cb": [float(Cb[0]), float(Cb[1])],
            "p_right": float(p_right),
            "r_right": float(r_right),
            "ray_intersection_note": "parallel_rays" if note_parallel else "intersection_point",
            "ints_up_count": int(len(ints_up)) if ints_up else 0,
            "ints_dn_count": int(len(ints_dn)) if ints_dn else 0,
        },
    }


def _rhombus_curve_points(fit: Dict[str, object]) -> np.ndarray:
    vs = np.asarray(fit.get("vertices_pitch_roll") or [], dtype=float).reshape((-1, 2))
    if vs.shape[0] != 4:
        return np.zeros((0, 2), dtype=float)
    return np.vstack([vs, vs[0:1]])


def _fit_max_inscribed_axis_aligned_rectangle(
    feasible_grid: np.ndarray,
    pitch_samples: np.ndarray,
    roll_samples: np.ndarray,
) -> Dict[str, object]:
    """
    在可行域布尔栅格中求面积最大的轴对齐内接矩形（pitch-roll 坐标系）。
    """
    g = np.asarray(feasible_grid, dtype=bool)
    ps = np.asarray(pitch_samples, dtype=float).reshape((-1,))
    rs = np.asarray(roll_samples, dtype=float).reshape((-1,))
    if g.ndim != 2 or g.size == 0 or ps.size != g.shape[0] or rs.size != g.shape[1]:
        return {"method": "max_inscribed_axis_aligned_rectangle", "note": "invalid_grid"}

    n_pitch, n_roll = g.shape
    heights = np.zeros((n_roll,), dtype=int)
    best = {
        "area_cells": 0,
        "top": 0,
        "bottom": 0,
        "left": 0,
        "right": 0,
    }

    for i in range(n_pitch):
        heights = np.where(g[i], heights + 1, 0)
        stack: List[int] = []
        j = 0
        while j <= n_roll:
            cur_h = 0 if j == n_roll else int(heights[j])
            if not stack or cur_h >= int(heights[stack[-1]]):
                stack.append(j)
                j += 1
                continue
            top_idx = stack.pop()
            h = int(heights[top_idx])
            if h <= 0:
                continue
            left_bound = 0 if not stack else (stack[-1] + 1)
            right_bound = j - 1
            w = right_bound - left_bound + 1
            area_cells = h * w
            if area_cells > int(best["area_cells"]):
                best["area_cells"] = int(area_cells)
                best["bottom"] = int(i)
                best["top"] = int(i - h + 1)
                best["left"] = int(left_bound)
                best["right"] = int(right_bound)

    if int(best["area_cells"]) <= 0:
        return {"method": "max_inscribed_axis_aligned_rectangle", "note": "no_feasible_cell"}

    it, ib = int(best["top"]), int(best["bottom"])
    jl, jr = int(best["left"]), int(best["right"])
    pitch_min = float(min(ps[it], ps[ib]))
    pitch_max = float(max(ps[it], ps[ib]))
    roll_min = float(min(rs[jl], rs[jr]))
    roll_max = float(max(rs[jl], rs[jr]))
    area = float((pitch_max - pitch_min) * (roll_max - roll_min))
    vertices = [
        [pitch_max, roll_min],
        [pitch_max, roll_max],
        [pitch_min, roll_max],
        [pitch_min, roll_min],
    ]
    return {
        "method": "max_inscribed_axis_aligned_rectangle",
        "area": area,
        "area_cells": int(best["area_cells"]),
        "pitch_min": pitch_min,
        "pitch_max": pitch_max,
        "roll_min": roll_min,
        "roll_max": roll_max,
        "vertices_pitch_roll": vertices,
    }


def _rectangle_curve_points(fit: Dict[str, object]) -> np.ndarray:
    vs = np.asarray(fit.get("vertices_pitch_roll") or [], dtype=float).reshape((-1, 2))
    if vs.shape[0] != 4:
        return np.zeros((0, 2), dtype=float)
    return np.vstack([vs, vs[0:1]])


def _limits_from_mjcf(*, mjcf_path: str, bar_joint_names: Tuple[str, str, str, str]) -> BarLimits:
    """
    从 MJCF/XML 中提取 bar 的限位。

    优先级：
    1) <actuator> 下任意子元素的 ctrlrange（且 joint=bar_name）
    2) <joint name=bar_name range="lo hi">

    若缺失则 fail-fast 抛异常（不做 fallback）。
    """
    if not os.path.isfile(mjcf_path):
        raise FileNotFoundError(f"MJCF 不存在: {mjcf_path}")

    try:
        root = ET.parse(mjcf_path).getroot()
    except Exception as e:
        raise RuntimeError(f"无法解析 MJCF: {mjcf_path}") from e

    def _parse_pair(s: str) -> Tuple[float, float]:
        parts = (s or "").strip().split()
        if len(parts) != 2:
            raise ValueError(f"invalid range string: {s!r}")
        return float(parts[0]), float(parts[1])

    # 1) actuator ctrlrange
    act_limits: Dict[str, Tuple[float, float]] = {}
    actuator = root.find("actuator")
    if actuator is not None:
        for elem in list(actuator):
            j = elem.get("joint")
            if not j:
                continue
            if j not in bar_joint_names:
                continue
            cr = elem.get("ctrlrange")
            if cr:
                act_limits[j] = _parse_pair(cr)

    # 2) joint range
    joint_limits: Dict[str, Tuple[float, float]] = {}
    for jname in bar_joint_names:
        node = root.find(f".//joint[@name='{jname}']")
        if node is None:
            continue
        r = node.get("range")
        if r:
            joint_limits[jname] = _parse_pair(r)

    def _get(name: str) -> Tuple[float, float]:
        if name in act_limits:
            return act_limits[name]
        if name in joint_limits:
            return joint_limits[name]
        raise RuntimeError(
            f"无法在 MJCF 中找到 bar={name!r} 的限位：未找到 actuator ctrlrange，也未找到 joint range；mjcf={mjcf_path}"
        )

    l_l, l_r, r_l, r_r = (_get(n) for n in bar_joint_names)
    return BarLimits(l_l=l_l, l_r=l_r, r_l=r_l, r_r=r_r)


def _linspace(lo: float, hi: float, n: int) -> np.ndarray:
    if n < 2:
        return np.array([0.5 * (lo + hi)], dtype=float)
    return np.linspace(float(lo), float(hi), int(n), dtype=float)


def _nan_or_inf(x: np.ndarray) -> bool:
    return not np.all(np.isfinite(np.asarray(x, dtype=float)))


def _scan_leg(
    *,
    leg: str,
    solver: object,
    bar_limits: BarLimits,
    grid: int,
    fixed_other_leg_mids: Tuple[float, float],
    joint_limits_pitch: Tuple[float, float],
    joint_limits_roll: Tuple[float, float],
    pitch_sampling_limits: Optional[Tuple[float, float]] = None,
    roll_sampling_limits: Optional[Tuple[float, float]] = None,
    max_exc_samples_per_type: int = 5,
    show_progress: bool = False,
    progress_every: Optional[int] = None,
) -> ScanResult:
    """
    扫描单腿（关节空间采样）：在该腿 ankle joint 的 pitch/roll 设计限位内做网格采样，
    对每个 (pitch, roll) 构造 q4（另一条腿固定在其 bar_mid 对应的姿态），
    调用 joint_to_motor_position(q4) 得到 p4，并判断目标腿的两根 bar 是否落在 bar_limits 内。

    返回：
    - ok_points：满足 bar_limits 的 (pitch, roll) 可行点
    - in_limits_mask：与 ok_points 同长度（关节采样本身在 limits 内，因此通常全 True）
    """

    leg = (leg or "").strip().lower()
    if leg not in ("left", "right"):
        raise ValueError("leg must be 'left' or 'right'")

    pitch_lo, pitch_hi = float((pitch_sampling_limits or joint_limits_pitch)[0]), float(
        (pitch_sampling_limits or joint_limits_pitch)[1]
    )
    roll_lo, roll_hi = float((roll_sampling_limits or joint_limits_roll)[0]), float(
        (roll_sampling_limits or joint_limits_roll)[1]
    )
    xs = _linspace(pitch_lo, pitch_hi, grid)
    ys = _linspace(roll_lo, roll_hi, grid)

    other_l, other_r = fixed_other_leg_mids
    # 通过“另一条腿固定在 bar mid 对应姿态”来消除另一条腿自由变化带来的混淆。
    # joint_to_motor_position / motor_to_joint_position 在 ankle 两侧求解是相互独立的，
    # 因此这里不影响目标腿求解的正确性。
    if leg == "left":
        # 固定右腿：p4 = [0,0, R_l_mid, R_r_mid]
        p4_fixed_other = np.array([0.0, 0.0, float(other_l), float(other_r)], dtype=float)
        q4_fixed_other = np.asarray(solver.motor_to_joint_position(p4_fixed_other), dtype=float).reshape((4,))
        other_pitch, other_roll = float(q4_fixed_other[2]), float(q4_fixed_other[3])
    else:
        # 固定左腿：p4 = [L_l_mid, L_r_mid, 0,0]
        p4_fixed_other = np.array([float(other_l), float(other_r), 0.0, 0.0], dtype=float)
        q4_fixed_other = np.asarray(solver.motor_to_joint_position(p4_fixed_other), dtype=float).reshape((4,))
        other_pitch, other_roll = float(q4_fixed_other[0]), float(q4_fixed_other[1])

    total = int(xs.size * ys.size)
    ok_pts: List[Tuple[float, float]] = []
    in_mask: List[bool] = []
    exc_counter: Counter = Counter()
    exc_samples: Dict[str, List[str]] = {}
    feasible = np.zeros((xs.size, ys.size), dtype=bool)

    step = progress_every
    if show_progress and step is None:
        step = max(1, total // 40) if total > 2500 else 0
    t0 = time.perf_counter()

    for idx, (pitch, roll) in enumerate(product(xs, ys)):
        ip = idx // ys.size
        ir = idx % ys.size
        if step and idx > 0 and idx % step == 0:
            dt = time.perf_counter() - t0
            rate = idx / dt if dt > 0 else 0.0
            print(
                f"  [{leg}] scan progress {idx}/{total} ({100.0 * idx / max(total, 1):.1f}%)  ~{rate:.0f} pts/s",
                flush=True,
            )
        try:
            if leg == "left":
                q4 = np.array([float(pitch), float(roll), other_pitch, other_roll], dtype=float)
            else:
                q4 = np.array([other_pitch, other_roll, float(pitch), float(roll)], dtype=float)

            p4 = np.asarray(solver.joint_to_motor_position(q4), dtype=float).reshape((4,))
            if _nan_or_inf(p4):
                raise FloatingPointError(f"solver returned non-finite p4: {p4.tolist()}")

            if leg == "left":
                ok = _in_range(float(p4[0]), bar_limits.l_l[0], bar_limits.l_l[1]) and _in_range(
                    float(p4[1]), bar_limits.l_r[0], bar_limits.l_r[1]
                )
            else:
                ok = _in_range(float(p4[2]), bar_limits.r_l[0], bar_limits.r_l[1]) and _in_range(
                    float(p4[3]), bar_limits.r_r[0], bar_limits.r_r[1]
                )

            if not ok:
                continue

            feasible[ip, ir] = True
            in_mask.append(
                _in_range(pitch, joint_limits_pitch[0], joint_limits_pitch[1])
                and _in_range(roll, joint_limits_roll[0], joint_limits_roll[1])
            )
            ok_pts.append((float(pitch), float(roll)))
        except Exception as e:
            et = type(e).__name__
            exc_counter[et] += 1
            if et not in exc_samples:
                exc_samples[et] = []
            if len(exc_samples[et]) < int(max_exc_samples_per_type):
                exc_samples[et].append(f"q4_pitch_roll=({float(pitch)},{float(roll)}) err={e}")

    if show_progress and total > 0:
        dt = time.perf_counter() - t0
        print(f"  [{leg}] scan done {total}/{total} in {dt:.2f}s", flush=True)

    ok_arr = np.asarray(ok_pts, dtype=float).reshape((-1, 2)) if ok_pts else np.zeros((0, 2), dtype=float)
    mask_arr = np.asarray(in_mask, dtype=bool).reshape((-1,)) if in_mask else np.zeros((0,), dtype=bool)
    return ScanResult(
        ok_points=ok_arr,
        in_limits_mask=mask_arr,
        total=total,
        ok=int(ok_arr.shape[0]),
        fail=int(total - ok_arr.shape[0]),
        exc_counter=exc_counter,
        exc_samples=exc_samples,
        sample_pitch=np.asarray(xs, dtype=float),
        sample_roll=np.asarray(ys, dtype=float),
        feasible_grid=feasible,
    )


def _scan_leg_bar(
    *,
    leg: str,
    solver: object,
    bar_limits: BarLimits,
    grid: int,
    fixed_other_leg_mids: Tuple[float, float],
    joint_limits_pitch: Tuple[float, float],
    joint_limits_roll: Tuple[float, float],
    no_joint_limit_clip: bool,
    show_progress: bool = False,
    progress_every: Optional[int] = None,
) -> ScanResult:
    """
    扫描单腿（执行器 bar 空间采样）：在目标腿的 bar ctrlrange 内做网格采样，
    构造完整 p4（另一条腿固定在其 bar mid），调用 motor_to_joint_position(p4) 得到目标腿的 (pitch, roll)。

    返回 ok_points 为映射后的 pitch/roll 点云；in_limits_mask 用于设计关节限位上色（可选关闭）。

    注意：这里“工作空间”直接来自执行器空间，不再对 pitch/roll 做规则网格采样。
    """
    leg = (leg or "").strip().lower()
    if leg not in ("left", "right"):
        raise ValueError("leg must be 'left' or 'right'")

    other_l_mid, other_r_mid = fixed_other_leg_mids

    if leg == "left":
        xs = _linspace(bar_limits.l_l[0], bar_limits.l_l[1], grid)
        ys = _linspace(bar_limits.l_r[0], bar_limits.l_r[1], grid)
        total = int(xs.size * ys.size)
        # p4 = [L_l_bar, L_r_bar, R_l_bar, R_r_bar]
        def _p4(ll: float, lr: float) -> np.ndarray:
            return np.asarray([ll, lr, float(other_l_mid), float(other_r_mid)], dtype=float)
        def _extract(q4: np.ndarray) -> Tuple[float, float]:
            # q4 = [L_pitch, L_roll, R_pitch, R_roll]
            return float(q4[0]), float(q4[1])
    else:
        xs = _linspace(bar_limits.r_l[0], bar_limits.r_l[1], grid)
        ys = _linspace(bar_limits.r_r[0], bar_limits.r_r[1], grid)
        total = int(xs.size * ys.size)
        # p4 = [L_l_bar, L_r_bar, R_l_bar, R_r_bar]
        def _p4(rl: float, rr: float) -> np.ndarray:
            return np.asarray([float(other_l_mid), float(other_r_mid), rl, rr], dtype=float)
        def _extract(q4: np.ndarray) -> Tuple[float, float]:
            # q4 = [L_pitch, L_roll, R_pitch, R_roll]
            return float(q4[2]), float(q4[3])

    ok_pts: List[Tuple[float, float]] = []
    in_mask: List[bool] = []
    exc_counter: Counter = Counter()
    exc_samples: Dict[str, List[str]] = {}

    # bar 扫描模式下先不做规则 feasible_grid（后续拟合可再扩展）
    feasible_dummy = np.zeros((grid, grid), dtype=bool)

    step = progress_every
    if show_progress and step is None:
        step = max(1, total // 40) if total > 2500 else 0
    t0 = time.perf_counter()

    for idx, (a, b) in enumerate(product(xs, ys)):
        if step and idx > 0 and idx % step == 0:
            dt = time.perf_counter() - t0
            rate = idx / dt if dt > 0 else 0.0
            print(f"  [{leg}] scan progress {idx}/{total} ({100.0 * idx / max(total, 1):.1f}%)  ~{rate:.0f} pts/s", flush=True)

        try:
            p4 = _p4(float(a), float(b))
            if _nan_or_inf(p4):
                raise FloatingPointError(f"non-finite p4: {p4.tolist()}")
            q4 = np.asarray(solver.motor_to_joint_position(p4), dtype=float).reshape((4,))
            if _nan_or_inf(q4):
                raise FloatingPointError(f"solver returned non-finite q4: {q4.tolist()}")

            pitch, roll = _extract(q4)
            ok_pts.append((pitch, roll))

            if no_joint_limit_clip:
                # 关闭上色：不生成 in/out mask
                pass
            else:
                in_mask.append(
                    _in_range(pitch, joint_limits_pitch[0], joint_limits_pitch[1])
                    and _in_range(roll, joint_limits_roll[0], joint_limits_roll[1])
                )
        except Exception as e:
            et = type(e).__name__
            exc_counter[et] += 1
            if et not in exc_samples:
                exc_samples[et] = []
            if len(exc_samples[et]) < 5:
                exc_samples[et].append(f"p4=({float(a)},{float(b)}) err={e}")

    ok_arr = np.asarray(ok_pts, dtype=float).reshape((-1, 2)) if ok_pts else np.zeros((0, 2), dtype=float)
    if no_joint_limit_clip:
        mask_arr = np.zeros((0,), dtype=bool)
    else:
        mask_arr = np.asarray(in_mask, dtype=bool).reshape((-1,)) if in_mask else np.zeros((0,), dtype=bool)

    ok_n = int(ok_arr.shape[0])
    fail_n = int(total - ok_n)

    return ScanResult(
        ok_points=ok_arr,
        in_limits_mask=mask_arr,
        total=total,
        ok=ok_n,
        fail=fail_n,
        exc_counter=exc_counter,
        exc_samples=exc_samples,
        sample_pitch=np.asarray(xs, dtype=float),
        sample_roll=np.asarray(ys, dtype=float),
        feasible_grid=feasible_dummy,
    )


def _print_scan_summary(title: str, res: ScanResult) -> None:
    print(f"\n=== {title} ===")
    print(f"total={res.total} ok={res.ok} fail={res.fail} ok_rate={res.ok / max(res.total, 1):.3f}")
    if res.ok_points.size:
        pmin = np.min(res.ok_points[:, 0])
        pmax = np.max(res.ok_points[:, 0])
        rmin = np.min(res.ok_points[:, 1])
        rmax = np.max(res.ok_points[:, 1])
        print(f"pitch_range=[{pmin:.6f}, {pmax:.6f}] roll_range=[{rmin:.6f}, {rmax:.6f}]")
        if res.in_limits_mask.size:
            inside = int(np.sum(res.in_limits_mask))
            outside = int(res.in_limits_mask.size - inside)
            frac = inside / max(res.ok, 1)
            print(f"in_design_limits={inside} out_of_limits={outside}  frac(in_limits|solver_ok)={frac:.4f}")
    if res.exc_counter:
        print("exceptions:")
        for k, v in res.exc_counter.most_common(10):
            print(f"  - {k}: {v}")
        for k in list(res.exc_samples.keys())[:5]:
            for s in res.exc_samples[k][:2]:
                print(f"  sample({k}): {s}")


def _plot_points(
    *,
    out_png: str,
    title: str,
    points_pitch_roll: np.ndarray,
    in_limits_mask: Optional[np.ndarray] = None,
    pitch_limits: Optional[Tuple[float, float]] = None,
    roll_limits: Optional[Tuple[float, float]] = None,
    axis_pitch_limits: Optional[Tuple[float, float]] = None,
    axis_roll_limits: Optional[Tuple[float, float]] = None,
    envelope_points_pitch_roll: Optional[np.ndarray] = None,
    envelope_fit_points_pitch_roll: Optional[np.ndarray] = None,
    envelope_label: str = "envelope",
    envelope_fit_label: str = "envelope_fit",
    x_axis: str = "roll",
    close_envelope_loop: bool = True,
) -> None:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as e:
        raise RuntimeError("缺少 matplotlib，无法画图；请安装 matplotlib 后重试") from e

    pts = np.asarray(points_pitch_roll, dtype=float).reshape((-1, 2))
    mask = None
    if in_limits_mask is not None:
        mask = np.asarray(in_limits_mask, dtype=bool).reshape((-1,))
        if mask.shape[0] != pts.shape[0]:
            raise ValueError("in_limits_mask shape mismatch with points")
    pitch = pts[:, 0] if pts.size else np.zeros((0,), dtype=float)
    roll = pts[:, 1] if pts.size else np.zeros((0,), dtype=float)

    if x_axis not in ("roll", "pitch"):
        raise ValueError("x_axis must be 'roll' or 'pitch'")

    if x_axis == "roll":
        x, y = roll, pitch
        xlabel, ylabel = "roll", "pitch"
    else:
        x, y = pitch, roll
        xlabel, ylabel = "pitch", "roll"

    fig = plt.figure(figsize=(7.2, 6.0), dpi=150)
    ax = fig.add_subplot(1, 1, 1)
    if mask is None or pts.size == 0:
        ax.scatter(x, y, s=3.0, alpha=0.7, linewidths=0.0, label=f"solutions (n={int(pts.shape[0])})")
    else:
        inside = mask
        outside = ~mask
        ax.scatter(x[outside], y[outside], s=3.0, alpha=0.35, linewidths=0.0, label=f"out_of_limits (n={int(np.sum(outside))})")
        ax.scatter(x[inside], y[inside], s=3.0, alpha=0.75, linewidths=0.0, label=f"in_limits (n={int(np.sum(inside))})")

    # 设计限位框（画在 pitch-roll 坐标系；若 x_axis 交换则相应交换）
    if pitch_limits is not None and roll_limits is not None:
        plo, phi = float(pitch_limits[0]), float(pitch_limits[1])
        rlo, rhi = float(roll_limits[0]), float(roll_limits[1])
        if x_axis == "roll":
            # x=roll, y=pitch
            xs = [rlo, rhi, rhi, rlo, rlo]
            ys = [plo, plo, phi, phi, plo]
        else:
            # x=pitch, y=roll
            xs = [plo, phi, phi, plo, plo]
            ys = [rlo, rlo, rhi, rhi, rlo]
        ax.plot(xs, ys, linewidth=1.5, label="design_limits")

    # 关键：显式设定坐标轴范围（避免 Matplotlib 只按点云 extents 自动缩放导致“看起来裁剪”）。
    # x/y 轴与 x_axis 参数一致映射：
    #   x_axis="roll"  => x=roll,  y=pitch
    #   x_axis="pitch" => x=pitch, y=roll
    ap = axis_pitch_limits if axis_pitch_limits is not None else pitch_limits
    ar = axis_roll_limits if axis_roll_limits is not None else roll_limits
    if ap is not None and ar is not None:
        plo, phi = float(ap[0]), float(ap[1])
        rlo, rhi = float(ar[0]), float(ar[1])
        if x_axis == "roll":
            ax.set_xlim(rlo, rhi)
            ax.set_ylim(plo, phi)
        else:
            ax.set_xlim(plo, phi)
            ax.set_ylim(rlo, rhi)

    def _pitch_roll_to_xy(env2d: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        env = np.asarray(env2d, dtype=float).reshape((-1, 2))
        if not env.size:
            return np.zeros((0,), dtype=float), np.zeros((0,), dtype=float)
        ep = env[:, 0]
        er = env[:, 1]
        if x_axis == "roll":
            return er, ep
        return ep, er

    # 外包络边界（可选首尾闭合便于视觉识别）
    if envelope_points_pitch_roll is not None:
        env = np.asarray(envelope_points_pitch_roll, dtype=float).reshape((-1, 2))
        if env.size:
            if close_envelope_loop and env.shape[0] >= 3:
                env = np.vstack([env, env[:1]])
            ex, ey = _pitch_roll_to_xy(env)
            ax.plot(ex, ey, linewidth=1.6, label=envelope_label)
    if envelope_fit_points_pitch_roll is not None:
        env2 = np.asarray(envelope_fit_points_pitch_roll, dtype=float).reshape((-1, 2))
        if env2.size:
            if close_envelope_loop and env2.shape[0] >= 3:
                env2 = np.vstack([env2, env2[:1]])
            ex, ey = _pitch_roll_to_xy(env2)
            ax.plot(ex, ey, linewidth=1.8, linestyle="--", label=envelope_fit_label)
    ax.set_title(title)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.25)
    ax.set_aspect("equal" if pts.size else "auto", adjustable="box")
    ax.legend(
        loc="best",
        framealpha=0.8,
        fontsize=8,
        markerscale=0.8,
        handlelength=1.0,
        borderpad=0.3,
        labelspacing=0.2,
        borderaxespad=0.3,
    )
    fig.tight_layout()
    _d = os.path.dirname(os.path.abspath(out_png))
    if _d:
        os.makedirs(_d, exist_ok=True)
    fig.savefig(out_png)
    plt.close(fig)


def _plot_overlay(
    *,
    out_png: str,
    title: str,
    left_points_pitch_roll: np.ndarray,
    right_points_pitch_roll: np.ndarray,
    left_in_limits_mask: Optional[np.ndarray],
    right_in_limits_mask: Optional[np.ndarray],
    pitch_limits: Optional[Tuple[float, float]],
    roll_limits: Optional[Tuple[float, float]],
    axis_pitch_limits: Optional[Tuple[float, float]] = None,
    axis_roll_limits: Optional[Tuple[float, float]] = None,
    left_envelope_pitch_roll: Optional[np.ndarray],
    right_envelope_pitch_roll: Optional[np.ndarray],
    left_envelope_fit_pitch_roll: Optional[np.ndarray],
    right_envelope_fit_pitch_roll: Optional[np.ndarray],
    envelope_label: str,
    envelope_fit_label: str,
    mirror_right_roll: bool,
    x_axis: str = "roll",
    close_envelope_loop: bool = True,
) -> None:
    """
    叠加对比图：把右脚点云（可选 roll 取负）叠加到同一坐标系下，便于看“镜像对称性”。
    """
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as e:
        raise RuntimeError("缺少 matplotlib，无法画图；请安装 matplotlib 后重试") from e

    L = np.asarray(left_points_pitch_roll, dtype=float).reshape((-1, 2))
    R = np.asarray(right_points_pitch_roll, dtype=float).reshape((-1, 2))
    mL = None
    mR = None
    if left_in_limits_mask is not None:
        mL = np.asarray(left_in_limits_mask, dtype=bool).reshape((-1,))
        if mL.shape[0] != L.shape[0]:
            raise ValueError("left_in_limits_mask shape mismatch")
    if right_in_limits_mask is not None:
        mR = np.asarray(right_in_limits_mask, dtype=bool).reshape((-1,))
        if mR.shape[0] != R.shape[0]:
            raise ValueError("right_in_limits_mask shape mismatch")
    if mirror_right_roll and R.size:
        R = R.copy()
        R[:, 1] *= -1.0

    def _xy(pts: np.ndarray):
        pitch = pts[:, 0] if pts.size else np.zeros((0,), dtype=float)
        roll = pts[:, 1] if pts.size else np.zeros((0,), dtype=float)
        if x_axis == "roll":
            return roll, pitch, "roll", "pitch"
        return pitch, roll, "pitch", "roll"

    xL, yL, xlabel, ylabel = _xy(L)
    xR, yR, _xl, _yl = _xy(R)

    fig = plt.figure(figsize=(7.2, 6.0), dpi=150)
    ax = fig.add_subplot(1, 1, 1)
    # left
    if mL is None or L.size == 0:
        ax.scatter(xL, yL, s=3.0, alpha=0.55, linewidths=0.0, label=f"left (n={int(L.shape[0])})")
    else:
        ax.scatter(xL[~mL], yL[~mL], s=3.0, alpha=0.25, linewidths=0.0, label=f"left out_of_limits (n={int(np.sum(~mL))})")
        ax.scatter(xL[mL], yL[mL], s=3.0, alpha=0.65, linewidths=0.0, label=f"left in_limits (n={int(np.sum(mL))})")
    # right
    if mR is None or R.size == 0:
        right_label = f"right{' (roll mirrored)' if mirror_right_roll else ''} (n={int(R.shape[0])})"
        ax.scatter(xR, yR, s=3.0, alpha=0.55, linewidths=0.0, label=right_label)
    else:
        right_label0 = f"right{' (roll mirrored)' if mirror_right_roll else ''} out_of_limits (n={int(np.sum(~mR))})"
        right_label1 = f"right{' (roll mirrored)' if mirror_right_roll else ''} in_limits (n={int(np.sum(mR))})"
        ax.scatter(xR[~mR], yR[~mR], s=3.0, alpha=0.25, linewidths=0.0, label=right_label0)
        ax.scatter(xR[mR], yR[mR], s=3.0, alpha=0.65, linewidths=0.0, label=right_label1)
    # design limits box
    if pitch_limits is not None and roll_limits is not None:
        plo, phi = float(pitch_limits[0]), float(pitch_limits[1])
        rlo, rhi = float(roll_limits[0]), float(roll_limits[1])
        if x_axis == "roll":
            xs = [rlo, rhi, rhi, rlo, rlo]
            ys = [plo, plo, phi, phi, plo]
        else:
            xs = [plo, phi, phi, plo, plo]
            ys = [rlo, rlo, rhi, rhi, rlo]
        ax.plot(xs, ys, linewidth=1.5, label="design_limits")

    def _close_env_row(env: np.ndarray) -> np.ndarray:
        e = np.asarray(env, dtype=float).reshape((-1, 2))
        if close_envelope_loop and e.shape[0] >= 3:
            return np.vstack([e, e[:1]])
        return e

    # envelopes
    if left_envelope_pitch_roll is not None:
        envL = _close_env_row(left_envelope_pitch_roll)
        if envL.size:
            ep, er = envL[:, 0], envL[:, 1]
            ex, ey = (er, ep) if x_axis == "roll" else (ep, er)
            ax.plot(ex, ey, linewidth=1.6, label=f"left {envelope_label}")
    if right_envelope_pitch_roll is not None:
        envR = np.asarray(right_envelope_pitch_roll, dtype=float).reshape((-1, 2))
        if envR.size:
            envR = envR.copy()
            if mirror_right_roll:
                envR[:, 1] *= -1.0
            envR = _close_env_row(envR)
            ep, er = envR[:, 0], envR[:, 1]
            ex, ey = (er, ep) if x_axis == "roll" else (ep, er)
            ax.plot(ex, ey, linewidth=1.6, label=f"right {envelope_label}{' (mirrored)' if mirror_right_roll else ''}")
    if left_envelope_fit_pitch_roll is not None:
        envL = _close_env_row(left_envelope_fit_pitch_roll)
        if envL.size:
            ep, er = envL[:, 0], envL[:, 1]
            ex, ey = (er, ep) if x_axis == "roll" else (ep, er)
            ax.plot(ex, ey, linewidth=1.8, linestyle="--", label=f"left {envelope_fit_label}")
    if right_envelope_fit_pitch_roll is not None:
        envR = np.asarray(right_envelope_fit_pitch_roll, dtype=float).reshape((-1, 2))
        if envR.size:
            envR = envR.copy()
            if mirror_right_roll:
                envR[:, 1] *= -1.0
            envR = _close_env_row(envR)
            ep, er = envR[:, 0], envR[:, 1]
            ex, ey = (er, ep) if x_axis == "roll" else (ep, er)
            ax.plot(ex, ey, linewidth=1.8, linestyle="--", label=f"right {envelope_fit_label}{' (mirrored)' if mirror_right_roll else ''}")
    ax.set_title(title)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)

    # 同样显式设定坐标范围，保证“完整工作空间”不会因为点云 extents 变化而被裁剪。
    ap = axis_pitch_limits if axis_pitch_limits is not None else pitch_limits
    ar = axis_roll_limits if axis_roll_limits is not None else roll_limits
    if ap is not None and ar is not None:
        plo, phi = float(ap[0]), float(ap[1])
        rlo, rhi = float(ar[0]), float(ar[1])
        if x_axis == "roll":
            ax.set_xlim(rlo, rhi)
            ax.set_ylim(plo, phi)
        else:
            ax.set_xlim(plo, phi)
            ax.set_ylim(rlo, rhi)

    ax.grid(True, alpha=0.25)
    ax.legend(
        loc="best",
        framealpha=0.8,
        fontsize=8,
        markerscale=0.8,
        handlelength=1.0,
        borderpad=0.3,
        labelspacing=0.2,
        borderaxespad=0.3,
    )
    fig.tight_layout()
    _d = os.path.dirname(os.path.abspath(out_png))
    if _d:
        os.makedirs(_d, exist_ok=True)
    fig.savefig(out_png)
    plt.close(fig)


def _exc_counter_to_plain(c: Counter) -> Dict[str, int]:
    return {str(k): int(v) for k, v in c.items()}


def _write_feasible_summary_json(
    path: str,
    *,
    args: argparse.Namespace,
    spec: Any,
    bar_limits: BarLimits,
    pitch_limits: Tuple[float, float],
    roll_limits: Tuple[float, float],
    legs_mode: str,
    left_res: ScanResult,
    right_res: ScanResult,
    left_fit: Dict[str, object],
    right_fit: Dict[str, object],
    left_quad_fit: Optional[Dict[str, object]] = None,
    right_quad_fit: Optional[Dict[str, object]] = None,
    envelope_quantile: float,
    envelope_angle_bins: int,
    envelope_order: int,
) -> None:
    doc: Dict[str, Any] = {
        "tool": "ankle_feasible_region_plot",
        "version_arg": str(args.version),
        "solver_token": getattr(spec, "token", ""),
        "params_yaml": str(getattr(spec, "params_yaml", "") or ""),
        "mjcf_path": str(getattr(spec, "mjcf_path", "") or ""),
        "joint_limits": {"pitch": list(pitch_limits), "roll": list(roll_limits)},
        "bar_limits": {
            "l_l_bar": list(bar_limits.l_l),
            "l_r_bar": list(bar_limits.l_r),
            "r_l_bar": list(bar_limits.r_l),
            "r_r_bar": list(bar_limits.r_r),
        },
        "scan": {
            "grid": int(args.grid),
            "scan_space": str(getattr(args, "scan_space", "pitchroll")),
            "legs_mode": legs_mode,
            "progress_disabled": bool(getattr(args, "no_progress", False)),
        },
        "left": {
            "total": left_res.total,
            "solver_ok": left_res.ok,
            "solver_fail": left_res.fail,
            "exc_types": _exc_counter_to_plain(left_res.exc_counter),
            "fraction_in_design_limits": left_res.in_design_fraction(),
        },
        "right": {
            "total": right_res.total,
            "solver_ok": right_res.ok,
            "solver_fail": right_res.fail,
            "exc_types": _exc_counter_to_plain(right_res.exc_counter),
            "fraction_in_design_limits": right_res.in_design_fraction(),
        },
        "envelope": {
            "quantile": float(envelope_quantile),
            "angle_bins": int(envelope_angle_bins),
            "fourier_order": int(envelope_order),
            "fit_method": "max_inscribed_axis_aligned_rectangle",
            "left_rect_area": left_fit.get("area"),
            "right_rect_area": right_fit.get("area"),
            "left_rect": {
                "pitch_min": left_fit.get("pitch_min"),
                "pitch_max": left_fit.get("pitch_max"),
                "roll_min": left_fit.get("roll_min"),
                "roll_max": left_fit.get("roll_max"),
            },
            "right_rect": {
                "pitch_min": right_fit.get("pitch_min"),
                "pitch_max": right_fit.get("pitch_max"),
                "roll_min": right_fit.get("roll_min"),
                "roll_max": right_fit.get("roll_max"),
            },
        },
    }

    if left_quad_fit is not None or right_quad_fit is not None:
        doc["max_inscribed_quadrilateral"] = {
            "left": left_quad_fit,
            "right": right_quad_fit,
        }
    _dd = os.path.dirname(os.path.abspath(path))
    if _dd:
        os.makedirs(_dd, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(doc, f, ensure_ascii=False, indent=2)


def _parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="kuavo_solver 脚踝可行域评估并画图（bar→pitch/roll）")
    p.add_argument("--version", type=str, default="70", help="solver version（在 solver_version_index.yaml 中可解析）")
    p.add_argument("--legs", type=str, default="both", choices=("both", "left", "right"), help="扫描左腿、右腿或两腿（双摄时可只扫一条以节省时间）")
    p.add_argument(
        "--scan_space",
        type=str,
        default="bar",
        choices=("bar", "pitchroll"),
        help="采样空间：bar=执行器 ctrlrange 上采样并映射到 pitch/roll；pitchroll=在关节 pitch/roll 上采样再用 bar 限位过滤（旧逻辑）。",
    )

    # 若不提供 bar 限位，则默认从 spec.mjcf_path 解析（actuator ctrlrange / joint range）
    p.add_argument("--l_l_bar_lo", type=float, default=None)
    p.add_argument("--l_l_bar_hi", type=float, default=None)
    p.add_argument("--l_r_bar_lo", type=float, default=None)
    p.add_argument("--l_r_bar_hi", type=float, default=None)
    p.add_argument("--r_l_bar_lo", type=float, default=None)
    p.add_argument("--r_l_bar_hi", type=float, default=None)
    p.add_argument("--r_r_bar_lo", type=float, default=None)
    p.add_argument("--r_r_bar_hi", type=float, default=None)

    p.add_argument(
        "--grid",
        type=int,
        default=200,
        help="采样点数：每个腿二维网格各采样 grid 个点（单腿总点数 grid×grid；bar 模式为 bar×bar，pitchroll 模式为 pitch×roll）。",
    )
    p.add_argument(
        "--no-progress",
        action="store_true",
        help="关闭扫描进度输出（单腿 grid² 较大时默认会周期性打印进度）",
    )
    p.add_argument(
        "--no_joint_limit_clip",
        action="store_true",
        help="工作空间检测时不使用关节 pitch/roll 限位做采样范围（并关闭设计限位框/内外上色）。",
    )
    p.add_argument(
        "--joint_limit_margin_ratio",
        type=float,
        default=0.25,
        help="当 --no_joint_limit_clip 开启时，采样范围在 joint limits 基础上向外扩展的比例（按半跨度扩展）。",
    )
    p.add_argument(
        "--no-summary-json",
        action="store_true",
        help="不写运行摘要 feasible_region_summary.json（默认写入 --outdir）",
    )
    p.add_argument("--outdir", type=str, default=os.path.join(SCRIPT_DIR, "out_feasible_region"))
    p.add_argument("--x_axis", type=str, default="roll", choices=("roll", "pitch"), help="散点图 x 轴选择")
    p.add_argument("--save_npz", action="store_true", help="同时保存 npz 原始数据")
    p.add_argument("--envelope_angle_bins", type=int, default=1440, help="包络角度分桶数（方案A）")
    p.add_argument("--envelope_quantile", type=float, default=0.995, help="包络半径分位数（方案A）")
    p.add_argument("--envelope_fourier_order", type=int, default=12, help="包络 r(theta) 傅里叶拟合阶数 K")
    p.add_argument("--envelope_export_json", action="store_true", help="导出包络几何表达（json）到 outdir")
    p.add_argument(
        "--symmetry_check",
        action="store_true",
        help="做左右镜像对称性验证（右腿基准：rr=-ll, rl=-lr）并输出误差热力图与 symmetry_check.json",
    )
    p.add_argument("--symmetry_samples", type=int, default=61, help="对称性验证采样密度（每根 bar 采样点数）")
    return p.parse_args(argv)


def _symmetry_check(
    *,
    solver: object,
    bar_limits: BarLimits,
    outdir: str,
    samples: int,
) -> Optional[Dict[str, Any]]:
    """
    验证“左右镜像”对称性（严格口径，右腿基准）：
      r_l_bar = -l_r_bar
      r_r_bar = -l_l_bar

    对每个左脚 (l_l, l_r)：
      - 计算左脚解 (Lp, Lr)
      - 构造镜像右脚输入 (r_l, r_r) 并计算右脚解 (Rp, Rr)

    严格误差定义（不做经验 bias 修正）：
      err_pitch = Lp - Rp
      err_roll  = Lr + Rr
    """
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    xs = np.linspace(bar_limits.l_l[0], bar_limits.l_l[1], int(samples), dtype=float)
    ys = np.linspace(bar_limits.l_r[0], bar_limits.l_r[1], int(samples), dtype=float)
    n = int(samples)

    Lp = np.full((n, n), np.nan, dtype=float)
    Lr = np.full((n, n), np.nan, dtype=float)
    Rp = np.full((n, n), np.nan, dtype=float)
    Rr = np.full((n, n), np.nan, dtype=float)

    # mids for the "other" ankle bars (not under test)
    l_l_mid, l_r_mid, r_l_mid, r_r_mid = bar_limits.mids()

    for i, ll in enumerate(xs):
        for j, lr in enumerate(ys):
            # left: set right ankle to mid
            p4_left = np.array([ll, lr, r_l_mid, r_r_mid], dtype=float)
            # right mirrored: set left ankle to mid
            rl = -lr
            rr = -ll
            p4_right = np.array([l_l_mid, l_r_mid, rl, rr], dtype=float)
            try:
                q4L = np.asarray(solver.motor_to_joint_position(p4_left), dtype=float).reshape((4,))
                q4R = np.asarray(solver.motor_to_joint_position(p4_right), dtype=float).reshape((4,))
                if not np.all(np.isfinite(q4L)) or not np.all(np.isfinite(q4R)):
                    continue
                Lp[i, j] = float(q4L[0])
                Lr[i, j] = float(q4L[1])
                Rp[i, j] = float(q4R[2])
                Rr[i, j] = float(q4R[3])
            except Exception:
                continue

    valid = np.isfinite(Lp) & np.isfinite(Lr) & np.isfinite(Rp) & np.isfinite(Rr)
    if int(np.sum(valid)) < 10:
        print("symmetry_check: too few valid points, skip")
        return None

    err_pitch = Lp - Rp
    err_roll = Lr + Rr

    # stats
    ep = err_pitch[valid]
    er = err_roll[valid]

    stats: Dict[str, Any] = {
        "samples_per_axis": int(n),
        "valid_cells": int(np.sum(valid)),
        "grid_cells": int(n * n),
        "mirror_mapping": {
            "r_l_bar": "-l_r_bar",
            "r_r_bar": "-l_l_bar",
        },
        "pitch_err_abs_quantiles": {
            "p50": float(np.quantile(np.abs(ep), 0.5)),
            "p95": float(np.quantile(np.abs(ep), 0.95)),
            "p99": float(np.quantile(np.abs(ep), 0.99)),
        },
        "roll_err_abs_quantiles_strict": {
            "p50": float(np.quantile(np.abs(er), 0.5)),
            "p95": float(np.quantile(np.abs(er), 0.95)),
            "p99": float(np.quantile(np.abs(er), 0.99)),
        },
    }

    def _heatmap(path: str, Z: np.ndarray, title: str, vlim: Optional[float] = None) -> None:
        fig = plt.figure(figsize=(7.2, 6.0), dpi=150)
        ax = fig.add_subplot(1, 1, 1)
        # imshow expects [row, col] => [x_idx, y_idx]
        data = np.array(Z, dtype=float)
        data[~valid] = np.nan
        vmax = float(vlim) if vlim is not None else float(np.nanquantile(np.abs(data), 0.99))
        im = ax.imshow(
            data.T,
            origin="lower",
            aspect="auto",
            extent=[float(xs[0]), float(xs[-1]), float(ys[0]), float(ys[-1])],
            vmin=-vmax,
            vmax=vmax,
            cmap="coolwarm",
        )
        ax.set_title(title)
        ax.set_xlabel("l_l_bar")
        ax.set_ylabel("l_r_bar")
        fig.colorbar(im, ax=ax, shrink=0.85)
        fig.tight_layout()
        fig.savefig(path)
        plt.close(fig)

    os.makedirs(outdir, exist_ok=True)
    pp = os.path.join(outdir, "symmetry_pitch_error_heatmap.png")
    pr = os.path.join(outdir, "symmetry_roll_error_heatmap.png")
    _heatmap(pp, err_pitch, "Symmetry error (strict): L_pitch - R_pitch")
    _heatmap(pr, err_roll, "Symmetry error (strict): L_roll + R_roll")

    jp = os.path.join(outdir, "symmetry_check.json")
    stats["outputs"] = {"pitch_heatmap_png": pp, "roll_heatmap_png": pr, "summary_json": jp}
    with open(jp, "w", encoding="utf-8") as f:
        json.dump(stats, f, ensure_ascii=False, indent=2)

    print(
        "symmetry_check stats:\n"
        f"  valid={int(np.sum(valid))}/{n*n}\n"
        f"  pitch_err abs(p50/p95/p99)={[float(np.quantile(np.abs(ep), q)) for q in (0.5,0.95,0.99)]}\n"
        f"  roll_err  abs(p50/p95/p99)={[float(np.quantile(np.abs(er), q)) for q in (0.5,0.95,0.99)]}\n"
        f"  written: {jp}\n"
        f"  heatmaps: {pp}\n"
        f"           {pr}\n"
    )
    return stats


def main(argv: Optional[List[str]] = None) -> int:
    args = _parse_args(argv)

    outdir = os.path.abspath(args.outdir)
    os.makedirs(outdir, exist_ok=True)

    solver_py = _import_solver_py()
    from solver_validation_spec import build_spec  # local import to keep sys.path changes above effective

    spec = build_spec(module="ankle", version=str(args.version), solver_module=solver_py)
    solver = spec.construct_solver(solver_py)

    pitch_limits, roll_limits = _ankle_joint_limits_from_params_yaml(
        params_yaml=str(spec.params_yaml or ""),
        token=str(spec.token),
    )
    print(f"ankle joint limits from params_yaml (token={spec.token}): pitch={pitch_limits} roll={roll_limits}")

    scan_space = str(args.scan_space)
    if bool(args.no_joint_limit_clip):
        if scan_space == "pitchroll":
            # 旧逻辑：关节 pitch/roll 扩展采样范围，并用扩展范围绘制 workspace box（同时关闭 in/out 上色）。
            p_lo, p_hi = float(pitch_limits[0]), float(pitch_limits[1])
            r_lo, r_hi = float(roll_limits[0]), float(roll_limits[1])
            p_span = abs(p_hi - p_lo)
            r_span = abs(r_hi - r_lo)
            p_margin = float(args.joint_limit_margin_ratio) * p_span
            r_margin = float(args.joint_limit_margin_ratio) * r_span
            p_lo2, p_hi2 = (min(p_lo, p_hi) - p_margin, max(p_lo, p_hi) + p_margin)
            r_lo2, r_hi2 = (min(r_lo, r_hi) - r_margin, max(r_lo, r_hi) + r_margin)
            pitch_sampling_limits: Optional[Tuple[float, float]] = (p_lo2, p_hi2)
            roll_sampling_limits: Optional[Tuple[float, float]] = (r_lo2, r_hi2)
            plot_pitch_limits: Optional[Tuple[float, float]] = (float(p_lo2), float(p_hi2))
            plot_roll_limits: Optional[Tuple[float, float]] = (float(r_lo2), float(r_hi2))
            plot_in_mask_left: Optional[np.ndarray] = None
            plot_in_mask_right: Optional[np.ndarray] = None
            print(
                f"[ankle_feasible] no_joint_limit_clip: sampling pitch={pitch_sampling_limits} roll={roll_sampling_limits} (workspace box shown, no in/out coloring by joint limits)"
            )
        else:
            # bar 扫描：采样范围天然来自执行器 ctrlrange，这里不再对关节范围做扩展。
            pitch_sampling_limits = None
            roll_sampling_limits = None
            plot_pitch_limits = None  # scan 后根据点云 extents 补齐
            plot_roll_limits = None
            plot_in_mask_left = None
            plot_in_mask_right = None
            print("[ankle_feasible] no_joint_limit_clip: bar scan -> no in/out coloring; workspace bbox from mapped pitch/roll points")
    else:
        pitch_sampling_limits = None
        roll_sampling_limits = None
        plot_pitch_limits = pitch_limits
        plot_roll_limits = roll_limits
        plot_in_mask_left = None  # filled after scan
        plot_in_mask_right = None

    bar_args = (
        args.l_l_bar_lo,
        args.l_l_bar_hi,
        args.l_r_bar_lo,
        args.l_r_bar_hi,
        args.r_l_bar_lo,
        args.r_l_bar_hi,
        args.r_r_bar_lo,
        args.r_r_bar_hi,
    )
    if all(v is None for v in bar_args):
        bar_limits = _limits_from_mjcf(
            mjcf_path=str(spec.mjcf_path),
            bar_joint_names=("l_l_bar", "l_r_bar", "r_l_bar", "r_r_bar"),
        )
        print(
            "bar limits loaded from MJCF:\n"
            f"  l_l_bar={bar_limits.l_l}\n"
            f"  l_r_bar={bar_limits.l_r}\n"
            f"  r_l_bar={bar_limits.r_l}\n"
            f"  r_r_bar={bar_limits.r_r}\n"
            f"  mjcf={spec.mjcf_path}"
        )
    elif any(v is None for v in bar_args):
        raise RuntimeError("bar 限位参数必须要么全部提供，要么全部不提供（自动从 MJCF 读取）")
    else:
        bar_limits = BarLimits(
            l_l=(float(args.l_l_bar_lo), float(args.l_l_bar_hi)),
            l_r=(float(args.l_r_bar_lo), float(args.l_r_bar_hi)),
            r_l=(float(args.r_l_bar_lo), float(args.r_l_bar_hi)),
            r_r=(float(args.r_r_bar_lo), float(args.r_r_bar_hi)),
        )

    l_l_mid, l_r_mid, r_l_mid, r_r_mid = bar_limits.mids()

    _verify_solver_p4_four_bars_are_independent(solver, bar_limits)

    legs_mode = str(args.legs)
    per_leg_total = int(args.grid) * int(args.grid)
    show_prog = (not bool(args.no_progress)) and (per_leg_total > 1600)

    if legs_mode in ("both", "left"):
        if scan_space == "bar":
            left_res = _scan_leg_bar(
                leg="left",
                solver=solver,
                bar_limits=bar_limits,
                grid=int(args.grid),
                fixed_other_leg_mids=(r_l_mid, r_r_mid),
                joint_limits_pitch=pitch_limits,
                joint_limits_roll=roll_limits,
                no_joint_limit_clip=bool(args.no_joint_limit_clip),
                show_progress=show_prog,
            )
        else:
            left_res = _scan_leg(
                leg="left",
                solver=solver,
                bar_limits=bar_limits,
                grid=int(args.grid),
                fixed_other_leg_mids=(r_l_mid, r_r_mid),
                joint_limits_pitch=pitch_limits,
                joint_limits_roll=roll_limits,
                pitch_sampling_limits=pitch_sampling_limits,
                roll_sampling_limits=roll_sampling_limits,
                show_progress=show_prog,
            )
    else:
        left_res = _empty_scan_result()

    if legs_mode in ("both", "right"):
        if scan_space == "bar":
            right_res = _scan_leg_bar(
                leg="right",
                solver=solver,
                bar_limits=bar_limits,
                grid=int(args.grid),
                fixed_other_leg_mids=(l_l_mid, l_r_mid),
                joint_limits_pitch=pitch_limits,
                joint_limits_roll=roll_limits,
                no_joint_limit_clip=bool(args.no_joint_limit_clip),
                show_progress=show_prog,
            )
        else:
            right_res = _scan_leg(
                leg="right",
                solver=solver,
                bar_limits=bar_limits,
                grid=int(args.grid),
                fixed_other_leg_mids=(l_l_mid, l_r_mid),
                joint_limits_pitch=pitch_limits,
                joint_limits_roll=roll_limits,
                pitch_sampling_limits=pitch_sampling_limits,
                roll_sampling_limits=roll_sampling_limits,
                show_progress=show_prog,
            )
    else:
        right_res = _empty_scan_result()

    if left_res.total > 0:
        _print_scan_summary(f"LEFT (version={args.version}, token={spec.token})", left_res)
    if right_res.total > 0:
        _print_scan_summary(f"RIGHT (version={args.version}, token={spec.token})", right_res)

    # 只有在“应用关节限位裁剪”模式下才需要 in_limits 上色/设计限位框。
    if not bool(args.no_joint_limit_clip):
        plot_in_mask_left = left_res.in_limits_mask if left_res.total > 0 else None
        plot_in_mask_right = right_res.in_limits_mask if right_res.total > 0 else None

    # bar 扫描：no_joint_limit_clip 时 plot_pitch_limits/plot_roll_limits 先设为 None，
    # 这里根据映射后的 pitch/roll 点云推导 workspace bbox，保证坐标轴“完整显示工作空间”。
    if scan_space == "bar":
        all_pts: List[np.ndarray] = []
        if left_res.total > 0 and left_res.ok_points.size:
            all_pts.append(np.asarray(left_res.ok_points, dtype=float).reshape((-1, 2)))
        if right_res.total > 0 and right_res.ok_points.size:
            all_pts.append(np.asarray(right_res.ok_points, dtype=float).reshape((-1, 2)))
        if all_pts:
            pts_cat = np.concatenate(all_pts, axis=0)
            plot_pitch_limits = (float(np.min(pts_cat[:, 0])), float(np.max(pts_cat[:, 0])))
            plot_roll_limits = (float(np.min(pts_cat[:, 1])), float(np.max(pts_cat[:, 1])))
        else:
            # 无有效点时回退到设计限位（避免 plt 轴不设导致裁剪）。
            plot_pitch_limits = pitch_limits
            plot_roll_limits = roll_limits

    angle_bins = int(args.envelope_angle_bins)
    q = float(args.envelope_quantile)
    left_quad_fit_obj: Optional[Dict[str, object]] = None
    right_quad_fit_obj: Optional[Dict[str, object]] = None

    if scan_space == "pitchroll":
        left_center = _envelope_center(left_res.ok_points)
        right_center = _envelope_center(right_res.ok_points)
        left_env = _polar_envelope(
            left_res.ok_points, center_pitch_roll=left_center, angle_bins=angle_bins, r_quantile=q
        )
        right_env = _polar_envelope(
            right_res.ok_points, center_pitch_roll=right_center, angle_bins=angle_bins, r_quantile=q
        )

        left_fit = _fit_max_inscribed_axis_aligned_rectangle(
            left_res.feasible_grid,
            left_res.sample_pitch,
            left_res.sample_roll,
        )
        right_fit = _fit_max_inscribed_axis_aligned_rectangle(
            right_res.feasible_grid,
            right_res.sample_pitch,
            right_res.sample_roll,
        )
        left_fit_curve = _rectangle_curve_points(left_fit)
        right_fit_curve = _rectangle_curve_points(right_fit)
    else:
        # bar 扫描：工作空间用执行器空间采样得到的点云直接画出来；
        # 但设计限位/最大内接四边形/最大内接矩形作为“后续处理”，这里额外做一次 pitch/roll 可行域栅格化拟合。
        left_env = None
        right_env = None
        left_fit = {"method": "max_inscribed_axis_aligned_rectangle", "note": "not_computed"}
        right_fit = {"method": "max_inscribed_axis_aligned_rectangle", "note": "not_computed"}
        left_fit_curve = None
        right_fit_curve = None

        # 拟合用的轴范围：取 bar->(pitch,roll) 映射后的点云 bbox（避免再次裁剪）。
        fit_pitch_limits = plot_pitch_limits
        fit_roll_limits = plot_roll_limits

        if left_res.total > 0 and fit_pitch_limits is not None and fit_roll_limits is not None:
            left_fit_scan = _scan_leg(
                leg="left",
                solver=solver,
                bar_limits=bar_limits,
                grid=int(args.grid),
                fixed_other_leg_mids=(r_l_mid, r_r_mid),
                joint_limits_pitch=pitch_limits,
                joint_limits_roll=roll_limits,
                pitch_sampling_limits=fit_pitch_limits,
                roll_sampling_limits=fit_roll_limits,
                show_progress=False,
            )
            left_quad_fit_obj = _fit_min_symmetric_inscribed_rhombus(
                left_fit_scan.ok_points,
                design_pitch_limits=pitch_limits,
                design_roll_limits=roll_limits,
                envelope_angle_bins=int(angle_bins),
                envelope_r_quantile=float(q),
            )
            left_env = _rhombus_curve_points(left_quad_fit_obj)
            left_fit = _fit_max_inscribed_axis_aligned_rectangle(
                left_fit_scan.feasible_grid, left_fit_scan.sample_pitch, left_fit_scan.sample_roll
            )
            left_fit_curve = _rectangle_curve_points(left_fit)

        if right_res.total > 0 and fit_pitch_limits is not None and fit_roll_limits is not None:
            right_fit_scan = _scan_leg(
                leg="right",
                solver=solver,
                bar_limits=bar_limits,
                grid=int(args.grid),
                fixed_other_leg_mids=(l_l_mid, l_r_mid),
                joint_limits_pitch=pitch_limits,
                joint_limits_roll=roll_limits,
                pitch_sampling_limits=fit_pitch_limits,
                roll_sampling_limits=fit_roll_limits,
                show_progress=False,
            )
            right_quad_fit_obj = _fit_min_symmetric_inscribed_rhombus(
                right_fit_scan.ok_points,
                design_pitch_limits=pitch_limits,
                design_roll_limits=roll_limits,
                envelope_angle_bins=int(angle_bins),
                envelope_r_quantile=float(q),
            )
            right_env = _rhombus_curve_points(right_quad_fit_obj)
            right_fit = _fit_max_inscribed_axis_aligned_rectangle(
                right_fit_scan.feasible_grid, right_fit_scan.sample_pitch, right_fit_scan.sample_roll
            )
            right_fit_curve = _rectangle_curve_points(right_fit)

    exported: List[str] = []

    if bool(args.envelope_export_json):
        if legs_mode in ("both", "left"):
            lp = os.path.join(outdir, "left_envelope_fit.json")
            with open(lp, "w", encoding="utf-8") as f:
                json.dump(left_fit, f, ensure_ascii=False, indent=2)
            exported.append(lp)
        if legs_mode in ("both", "right"):
            rp = os.path.join(outdir, "right_envelope_fit.json")
            with open(rp, "w", encoding="utf-8") as f:
                json.dump(right_fit, f, ensure_ascii=False, indent=2)
            exported.append(rp)
        print(
            "envelope fit exported (max_inscribed_axis_aligned_rectangle) "
            f"left_area={left_fit.get('area')} right_area={right_fit.get('area')}"
        )
        for ep in exported:
            print(f"  - {ep}")

    left_png = os.path.join(outdir, "left_pitch_roll.png")
    right_png = os.path.join(outdir, "right_pitch_roll.png")
    overlay_png = os.path.join(outdir, "overlay_left_vs_right.png")
    overlay_mirror_png = os.path.join(outdir, "overlay_left_vs_right_mirror_roll.png")

    if legs_mode in ("both", "left"):
        _plot_points(
            out_png=left_png,
            title=f"LEFT ankle feasible region (version={args.version}, token={spec.token})",
            points_pitch_roll=left_res.ok_points,
            in_limits_mask=plot_in_mask_left,
            pitch_limits=pitch_limits,
            roll_limits=roll_limits,
            axis_pitch_limits=plot_pitch_limits,
            axis_roll_limits=plot_roll_limits,
            envelope_points_pitch_roll=left_env,
            envelope_fit_points_pitch_roll=left_fit_curve,
            envelope_label="max_inscribed_quadrilateral (roll=0 pitch max/min)"
            if scan_space == "bar"
            else f"envelope(q={q}, bins={angle_bins})",
            envelope_fit_label="max_inscribed_rectangle (virtual joints)"
            if scan_space == "bar"
            else "max_inscribed_rectangle",
            x_axis=str(args.x_axis),
            close_envelope_loop=(scan_space == "pitchroll"),
        )

    if legs_mode in ("both", "right"):
        _plot_points(
            out_png=right_png,
            title=f"RIGHT ankle feasible region (version={args.version}, token={spec.token})",
            points_pitch_roll=right_res.ok_points,
            in_limits_mask=plot_in_mask_right,
            pitch_limits=pitch_limits,
            roll_limits=roll_limits,
            axis_pitch_limits=plot_pitch_limits,
            axis_roll_limits=plot_roll_limits,
            envelope_points_pitch_roll=right_env,
            envelope_fit_points_pitch_roll=right_fit_curve,
            envelope_label="max_inscribed_quadrilateral (roll=0 pitch max/min)"
            if scan_space == "bar"
            else f"envelope(q={q}, bins={angle_bins})",
            envelope_fit_label="max_inscribed_rectangle (virtual joints)"
            if scan_space == "bar"
            else "max_inscribed_rectangle",
            x_axis=str(args.x_axis),
            close_envelope_loop=(scan_space == "pitchroll"),
        )

    do_overlay = bool(left_res.ok_points.size and right_res.ok_points.size)
    if do_overlay:
        _plot_overlay(
            out_png=overlay_png,
            title=f"Overlay LEFT vs RIGHT (no mirror) (version={args.version}, token={spec.token})",
            left_points_pitch_roll=left_res.ok_points,
            right_points_pitch_roll=right_res.ok_points,
            left_in_limits_mask=plot_in_mask_left,
            right_in_limits_mask=plot_in_mask_right,
            pitch_limits=pitch_limits,
            roll_limits=roll_limits,
            axis_pitch_limits=plot_pitch_limits,
            axis_roll_limits=plot_roll_limits,
            left_envelope_pitch_roll=left_env,
            right_envelope_pitch_roll=right_env,
            left_envelope_fit_pitch_roll=left_fit_curve,
            right_envelope_fit_pitch_roll=right_fit_curve,
            envelope_label="max_inscribed_quadrilateral (roll=0 pitch max/min)"
            if scan_space == "bar"
            else f"envelope(q={q})",
            envelope_fit_label="max_inscribed_rectangle (virtual joints)"
            if scan_space == "bar"
            else "fit(max_rectangle)",
            mirror_right_roll=False,
            x_axis=str(args.x_axis),
            close_envelope_loop=(scan_space == "pitchroll"),
        )
        _plot_overlay(
            out_png=overlay_mirror_png,
            title=f"Overlay LEFT vs RIGHT (mirror right roll) (version={args.version}, token={spec.token})",
            left_points_pitch_roll=left_res.ok_points,
            right_points_pitch_roll=right_res.ok_points,
            left_in_limits_mask=plot_in_mask_left,
            right_in_limits_mask=plot_in_mask_right,
            pitch_limits=pitch_limits,
            roll_limits=roll_limits,
            axis_pitch_limits=plot_pitch_limits,
            axis_roll_limits=plot_roll_limits,
            left_envelope_pitch_roll=left_env,
            right_envelope_pitch_roll=right_env,
            left_envelope_fit_pitch_roll=left_fit_curve,
            right_envelope_fit_pitch_roll=right_fit_curve,
            envelope_label="max_inscribed_quadrilateral (roll=0 pitch max/min)"
            if scan_space == "bar"
            else f"envelope(q={q})",
            envelope_fit_label="max_inscribed_rectangle (virtual joints)"
            if scan_space == "bar"
            else "fit(max_rectangle)",
            mirror_right_roll=True,
            x_axis=str(args.x_axis),
            close_envelope_loop=(scan_space == "pitchroll"),
        )
    else:
        print("跳过左右叠加图：需要先完成左右两条腿的扫描并得到非空点云（如需叠加请 --legs both）")

    summary_path = os.path.join(outdir, "feasible_region_summary.json")
    if not bool(args.no_summary_json):
        _write_feasible_summary_json(
            summary_path,
            args=args,
            spec=spec,
            bar_limits=bar_limits,
            pitch_limits=pitch_limits,
            roll_limits=roll_limits,
            legs_mode=legs_mode,
            left_res=left_res,
            right_res=right_res,
            left_fit=left_fit,
            right_fit=right_fit,
            left_quad_fit=left_quad_fit_obj,
            right_quad_fit=right_quad_fit_obj,
            envelope_quantile=q,
            envelope_angle_bins=angle_bins,
            envelope_order=int(args.envelope_fourier_order),
        )

    if bool(args.save_npz):
        np.savez_compressed(
            os.path.join(outdir, "ankle_feasible_region.npz"),
            legs_mode=np.array([str(legs_mode)], dtype=object),
            version=str(args.version),
            token=str(spec.token),
            pitch_limits_design=np.asarray(pitch_limits, dtype=float),
            roll_limits_design=np.asarray(roll_limits, dtype=float),
            bar_limits=np.array(
                [
                    [bar_limits.l_l[0], bar_limits.l_l[1]],
                    [bar_limits.l_r[0], bar_limits.l_r[1]],
                    [bar_limits.r_l[0], bar_limits.r_l[1]],
                    [bar_limits.r_r[0], bar_limits.r_r[1]],
                ],
                dtype=float,
            ),
            left_points_pitch_roll=left_res.ok_points,
            right_points_pitch_roll=right_res.ok_points,
            left_in_limits_mask=left_res.in_limits_mask,
            right_in_limits_mask=right_res.in_limits_mask,
            left_total=np.array([left_res.total, left_res.ok, left_res.fail], dtype=int),
            right_total=np.array([right_res.total, right_res.ok, right_res.fail], dtype=int),
        )

    out_lines = ["outputs:"]
    if legs_mode in ("both", "left"):
        out_lines.append(f"  - {left_png}")
    if legs_mode in ("both", "right"):
        out_lines.append(f"  - {right_png}")
    if do_overlay:
        out_lines.append(f"  - {overlay_png}")
        out_lines.append(f"  - {overlay_mirror_png}")
    if not bool(args.no_summary_json):
        out_lines.append(f"  - {summary_path}")
    if bool(args.save_npz):
        out_lines.append(f"  - {os.path.join(outdir, 'ankle_feasible_region.npz')}")
    print("\n" + "\n".join(out_lines))

    if bool(args.symmetry_check):
        _symmetry_check(
            solver=solver,
            bar_limits=bar_limits,
            outdir=outdir,
            samples=int(args.symmetry_samples),
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

