#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import numpy as np


@dataclass
class StrictThreshold:
    abs_max: float = 1.0e-4
    rel_l2: float = 2.0e-3
    rmse: float = 5.0e-5


def normalize_version(v: str) -> str:
    vv = (v or "").strip()
    if vv.isdigit():
        return f"s{vv}"
    return vv


def default_mjcf_for(*, package_root: str, module: str, token: str) -> str:
    """
    根据 solver token 解析默认 MJCF 路径（不针对特定代次定制）。
    - ``package_root``：kuavo_solver 包根（含 ``robot-descriptions`` 的目录）。
    - ankles: biped_ankles_<token>/mjcf/biped_ankles.xml 或 biped_knees.xml
    - waists: biped_waist_<token>/mjcf/biped_waist.xml
    - arms:   biped_arms_<token>/mjcf/biped_elbows.xml / biped_wrists.xml
    """
    t = (token or "").strip()
    if not t:
        raise ValueError("empty token for mjcf resolve")
    root = os.path.join(os.path.abspath(package_root), "robot-descriptions")
    if module == "ankle":
        return os.path.join(root, "ankles", f"biped_ankles_{t}", "mjcf", "biped_ankles.xml")
    if module == "knee":
        return os.path.join(root, "ankles", f"biped_ankles_{t}", "mjcf", "biped_knees.xml")
    if module == "waist":
        return os.path.join(root, "waists", f"biped_waist_{t}", "mjcf", "biped_waist.xml")
    if module == "arm_elbow":
        return os.path.join(root, "arms", f"biped_arms_{t}", "mjcf", "biped_elbows.xml")
    if module == "arm_wrist":
        return os.path.join(root, "arms", f"biped_arms_{t}", "mjcf", "biped_wrists.xml")
    raise ValueError(f"unsupported module for mjcf resolve: {module!r}")


def vector_err(a: np.ndarray, b: np.ndarray) -> Dict[str, float]:
    diff = np.asarray(a, dtype=float) - np.asarray(b, dtype=float)
    abs_max = float(np.max(np.abs(diff)))
    rmse = float(np.sqrt(np.mean(diff * diff)))
    rel = float(np.linalg.norm(diff) / max(np.linalg.norm(a), 1.0e-12))
    return {"abs_max": abs_max, "rmse": rmse, "rel_l2": rel}


def pass_threshold(err: Dict[str, float], th: StrictThreshold) -> bool:
    return err["abs_max"] <= th.abs_max and err["rmse"] <= th.rmse and err["rel_l2"] <= th.rel_l2


def cosine_similarity(a: np.ndarray, b: np.ndarray) -> float:
    aa = np.asarray(a, dtype=float).reshape((-1,))
    bb = np.asarray(b, dtype=float).reshape((-1,))
    na = float(np.linalg.norm(aa))
    nb = float(np.linalg.norm(bb))
    if na < 1.0e-12 or nb < 1.0e-12:
        return 1.0
    return float(np.dot(aa, bb) / (na * nb))


_POSITION_ERR_KEYS = (
    "err_solver_rt",
    "err_q_mujoco",
    "err_p_mujoco",
)
_JACOBIANISH_ERR_KEYS = (
    "constraint_vel_err",
    "err_vec",
    "err_j_len_ankle",
    "err_j_len_act",
    "err_j_constraint",
    "err_j_actuator",
    "err_dp",
)


def _pick_worst_err(worst_cases: List[Dict[str, Any]]) -> Tuple[Optional[str], Optional[Dict[str, float]]]:
    """
    从 worst_cases 中挑一个“最能代表失败原因”的误差字典，用于生成简短结论。
    返回 (err_name, err_dict)；找不到则返回 (None, None)。
    """
    if not worst_cases:
        return None, None
    c0 = worst_cases[0] or {}
    all_keys = _POSITION_ERR_KEYS + _JACOBIANISH_ERR_KEYS
    cand: List[Tuple[str, Dict[str, float]]] = []
    for k in all_keys:
        v = c0.get(k)
        if isinstance(v, dict) and all(x in v for x in ("abs_max", "rmse", "rel_l2")):
            cand.append(
                (
                    k,
                    {
                        "abs_max": float(v["abs_max"]),
                        "rmse": float(v["rmse"]),
                        "rel_l2": float(v["rel_l2"]),
                    },
                )
            )
    if not cand:
        return None, None
    best_k, best_d = cand[0]
    for nk, nv in cand[1:]:
        if nv["abs_max"] > best_d["abs_max"]:
            best_k, best_d = nk, nv
    return best_k, best_d


def format_conclusion(*, summary: Dict[str, Any], worst_cases: List[Dict[str, Any]]) -> str:
    """
    生成高可读性结论（单行/少行），用于 CLI 默认输出。
    """
    th = (summary.get("threshold") or {}) if isinstance(summary, dict) else {}
    th_abs = float(th.get("abs_max", 0.0) or 0.0)
    th_rmse = float(th.get("rmse", 0.0) or 0.0)
    module = str(summary.get("module", ""))
    token = str(summary.get("token", ""))
    passed = int(summary.get("passed_cases", 0) or 0)
    total = int(summary.get("total_cases", 0) or 0)
    rate = float(summary.get("pass_rate", (passed / max(total, 1))) or 0.0)
    overall = bool(summary.get("overall_pass", False))

    status = "PASS" if overall else "FAIL"
    err_name, err = _pick_worst_err(worst_cases)
    if not err_name or not err:
        return f"[{status}] {module} token={token} pass={passed}/{total} ({rate*100.0:.1f}%)"

    abs_max = float(err["abs_max"])
    rmse = float(err["rmse"])
    abs_x = (abs_max / th_abs) if th_abs > 0 else float("inf")
    rmse_x = (rmse / th_rmse) if th_rmse > 0 else float("inf")
    reason = f"worst {err_name}: abs_max={abs_max:.3e} ({abs_x:.1f}×th) rmse={rmse:.3e} ({rmse_x:.1f}×th)"
    return f"[{status}] {module} token={token} pass={passed}/{total} ({rate*100.0:.1f}%) | {reason}"

