#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
统一解算器验证 Runner。

两类检查均基于 ``ValidationSpec``：

1. ``run_position_roundtrip(spec, solver, samples, seed)``
   - Solver：``q → joint_to_motor → motor_to_joint → q'``（接口自洽）；
   - MuJoCo：`write`/``sync_mujoco_after_write``（膝/肘含被动闭链）后 ``read_*_from_mj`` 与 ``q``/``p`` 比对；
   - 三组误差均套用同一阈值；失败计数与 jacobian runner 一致（不 fail-fast）。

2. ``run_jacobian_check(spec, solver, samples, seed)``
   - 采样 q、dq，调用 ``spec.jacobian_probe`` 得 ``dp_solver`` 与 ``dp_mj``；
   - 若 spec 还提供 (J_constraint, J_actuator) 对，则额外对矩阵逐元素比较。
   - 失败策略：按阈值统计 pass/fail，worst_cases 打出来便于排查。
"""

from __future__ import annotations

import os
import sys

_VROOT = os.path.dirname(os.path.abspath(__file__))
_SROOT = os.path.dirname(_VROOT)
_lib = os.path.join(_SROOT, "lib")
if _lib not in sys.path:
    sys.path.insert(0, _lib)
if _SROOT not in sys.path:
    sys.path.insert(0, _SROOT)
if _VROOT not in sys.path:
    sys.path.insert(0, _VROOT)
from dataclasses import asdict
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

try:
    import mujoco
except Exception as e:  # pragma: no cover
    raise RuntimeError("solver_validation_runner 需要 mujoco Python 包") from e

from solver_validation_spec import JacobianProbe, ValidationSpec
from validation_common import StrictThreshold, pass_threshold, vector_err


def _make_model_data(spec: ValidationSpec):
    model = mujoco.MjModel.from_xml_path(spec.mjcf_path)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


def _vector_err_maybe_indices(
    a: np.ndarray, b: np.ndarray, indices: Optional[Tuple[int, ...]]
) -> Dict[str, float]:
    aa = np.asarray(a, dtype=float).reshape(-1)
    bb = np.asarray(b, dtype=float).reshape(-1)
    if indices:
        ix = np.asarray(indices, dtype=int).reshape(-1)
        aa = aa[ix]
        bb = bb[ix]
    return vector_err(aa, bb)


def run_position_roundtrip(
    *, spec: ValidationSpec, solver: Any, samples: int, seed: int = 42, threshold: StrictThreshold = StrictThreshold()
) -> Dict[str, Any]:
    """
    位置一致性：

    - **solver_rt**：``q → joint_to_motor → motor_to_joint → q'``，比较 ``q`` 与 ``q'``；
    - **MJ 读回**：``write/sync`` 后 ``read_q_from_mj`` / ``read_p_from_mj``，与上文 ``q`` / ``p`` 比较。
      ``spec.mj_roundtrip_{q,p}_indices`` 可指定只比较的 leg 向量下标（例如膝单机 MJ 仅对齐 L/R 膝两行）。
    """
    rng = np.random.default_rng(int(seed))
    cases: List[Dict[str, Any]] = []
    any_fail = False
    model, data = _make_model_data(spec)

    sync = getattr(spec, "sync_mujoco_after_write", None)
    ix_q_mj = getattr(spec, "mj_roundtrip_q_indices", None)
    ix_p_mj = getattr(spec, "mj_roundtrip_p_indices", None)

    def _amax_case(c: Dict[str, Any]) -> float:
        return max(
            float(c["err_solver_rt"]["abs_max"]),
            float(c["err_q_mujoco"]["abs_max"]),
            float(c["err_p_mujoco"]["abs_max"]),
        )

    for _ in range(int(samples)):
        q = np.asarray(spec.sample_q(rng), dtype=float).reshape((spec.dim_joint,))
        p = np.asarray(spec.joint_to_motor_position(solver, q), dtype=float).reshape((spec.dim_motor,))
        mh = getattr(solver, "motor_to_joint_position_hint", None)
        if spec.module == "waist" and callable(mh):
            q_back = np.asarray(mh(p, q), dtype=float).reshape((spec.dim_joint,))
        else:
            q_back = np.asarray(spec.motor_to_joint_position(solver, p), dtype=float).reshape((spec.dim_joint,))

        err_solver_rt = vector_err(q, q_back)

        if callable(sync):
            sync(model, data, q, p)
        else:
            spec.write_state_to_mj(model, data, q, p)
            mujoco.mj_forward(model, data)

        q_mj = np.asarray(spec.read_q_from_mj(model, data), dtype=float).reshape((spec.dim_joint,))
        p_mj = np.asarray(spec.read_p_from_mj(model, data), dtype=float).reshape((spec.dim_motor,))

        err_q_mujoco = _vector_err_maybe_indices(q, q_mj, ix_q_mj)
        err_p_mujoco = _vector_err_maybe_indices(p, p_mj, ix_p_mj)

        ok_solver = pass_threshold(err_solver_rt, threshold)
        ok_q = pass_threshold(err_q_mujoco, threshold)
        ok_p = pass_threshold(err_p_mujoco, threshold)
        ok = bool(ok_solver and ok_q and ok_p)
        any_fail = any_fail or (not ok)
        cases.append({
            "q": q.tolist(), "p": p.tolist(), "q_back": q_back.tolist(),
            "q_mujoco": q_mj.tolist(), "p_mujoco": p_mj.tolist(),
            "err_solver_rt": err_solver_rt,
            "err_q_mujoco": err_q_mujoco,
            "err_p_mujoco": err_p_mujoco,
            "pass": ok,
        })

    summary = {
        "module": f"{spec.module}_position_roundtrip",
        "token": spec.token,
        "model_path_used": spec.mjcf_path,
        "threshold": {"abs_max": threshold.abs_max, "rmse": threshold.rmse, "rel_l2": threshold.rel_l2},
        "total_cases": len(cases),
        "passed_cases": int(sum(1 for c in cases if c["pass"])),
        "failed_cases": int(sum(1 for c in cases if not c["pass"])),
        "pass_rate": float(sum(1 for c in cases if c["pass"]) / max(len(cases), 1)),
        "overall_pass": not any_fail,
        "has_mujoco_readback_check": True,
        "mj_roundtrip_q_indices": list(ix_q_mj) if ix_q_mj else None,
        "mj_roundtrip_p_indices": list(ix_p_mj) if ix_p_mj else None,
    }
    worst = sorted(cases, key=_amax_case, reverse=True)[:5]
    return {"summary": summary, "cases": cases, "worst_cases": worst}


def _probe_errors(probe: JacobianProbe) -> Dict[str, Dict[str, float]]:
    """把 probe 里的比较项都换成 err 字典；缺省矩阵的字段不出现。"""
    out: Dict[str, Dict[str, float]] = {}
    out["err_dp"] = vector_err(np.asarray(probe.dp_solver), np.asarray(probe.dp_mj))
    if probe.J_constraint_sv is not None and probe.J_constraint_mj is not None:
        out["err_j_constraint"] = vector_err(
            np.asarray(probe.J_constraint_sv, dtype=float).reshape((-1,)),
            np.asarray(probe.J_constraint_mj, dtype=float).reshape((-1,)),
        )
    if probe.J_actuator_sv is not None and probe.J_actuator_mj is not None:
        out["err_j_actuator"] = vector_err(
            np.asarray(probe.J_actuator_sv, dtype=float).reshape((-1,)),
            np.asarray(probe.J_actuator_mj, dtype=float).reshape((-1,)),
        )
    return out


def run_jacobian_check(
    *, spec: ValidationSpec, solver: Any, samples: int, seed: int = 42, threshold: StrictThreshold = StrictThreshold()
) -> Dict[str, Any]:
    """
    Jacobian 一致性验证：
    - 对每次采样比较 dp_solver 与 dp_mj；
    - 若 spec 能给出 (Jc, Ja) 对，则同时比较这两个矩阵（flatten 后用同阈值）。
    """
    model, data = _make_model_data(spec)
    rng = np.random.default_rng(int(seed))

    cases: List[Dict[str, Any]] = []
    any_fail = False
    has_matrix = False
    for _ in range(int(samples)):
        q = np.asarray(spec.sample_q(rng), dtype=float).reshape((spec.dim_joint,))
        # 先让 solver 推出 p，保证 MJ 侧写入的是"运动学一致"的位形
        p = spec.joint_to_motor_position(solver, q)
        dq = np.asarray(spec.sample_dq(rng, q), dtype=float).reshape((spec.dim_joint,))
        probe = spec.jacobian_probe(solver, model, data, q, p, dq)
        errs = _probe_errors(probe)

        ok_dp = pass_threshold(errs["err_dp"], threshold)
        oks = [ok_dp]
        if "err_j_constraint" in errs:
            has_matrix = True
            oks.append(pass_threshold(errs["err_j_constraint"], threshold))
        if "err_j_actuator" in errs:
            oks.append(pass_threshold(errs["err_j_actuator"], threshold))
        ok = all(oks)
        any_fail = any_fail or (not ok)
        cases.append({
            "q": q.tolist(), "p": p.tolist(), "dq": dq.tolist(),
            "dp_solver": probe.dp_solver.tolist(),
            "dp_mj": probe.dp_mj.tolist(),
            **errs,
            "pass": bool(ok),
        })

    summary = {
        "module": f"{spec.module}_jacobian",
        "token": spec.token,
        "model_path_used": spec.mjcf_path,
        "threshold": {"abs_max": threshold.abs_max, "rmse": threshold.rmse, "rel_l2": threshold.rel_l2},
        "total_cases": len(cases),
        "passed_cases": int(sum(1 for c in cases if c["pass"])),
        "failed_cases": int(sum(1 for c in cases if not c["pass"])),
        "pass_rate": float(sum(1 for c in cases if c["pass"]) / max(len(cases), 1)),
        "overall_pass": not any_fail,
        "has_matrix_check": bool(has_matrix),
    }

    def _score(c: Dict[str, Any]) -> float:
        s = float((c.get("err_dp") or {}).get("abs_max", 0.0))
        s = max(s, float((c.get("err_j_constraint") or {}).get("abs_max", 0.0)))
        s = max(s, float((c.get("err_j_actuator") or {}).get("abs_max", 0.0)))
        return s

    worst = sorted(cases, key=_score, reverse=True)[:5]
    return {"summary": summary, "cases": cases, "worst_cases": worst}
