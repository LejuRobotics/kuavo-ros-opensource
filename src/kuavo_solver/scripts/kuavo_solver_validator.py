#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Kuavo Solver Validator -- 交互验证 & CLI 直通.

用法:
    # 交互菜单（默认）
    python3 kuavo_solver_validator.py

    # CLI 直通（脚本化 / CI）
    python3 kuavo_solver_validator.py -t s2gen_2 viewer   --module ankle
    python3 kuavo_solver_validator.py -t 7gen    jacobian --module arm_wrist --num-steps 20
    python3 kuavo_solver_validator.py -t 7gen    position-verify --module waist --num-steps 50
    python3 kuavo_solver_validator.py -t 7gen    convert-position --module knee --side L --direction j2m --values 1.2
"""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import traceback
from typing import Any, Dict, List, Optional, Sequence, Tuple, Union

import numpy as np

import yaml

_SCRIPTS_ROOT = os.path.dirname(os.path.abspath(__file__))
for _sub in ("lib", "validation", "tools"):
    _p = os.path.join(_SCRIPTS_ROOT, _sub)
    if _p not in sys.path:
        sys.path.insert(0, _p)
if _SCRIPTS_ROOT not in sys.path:
    sys.path.insert(0, _SCRIPTS_ROOT)

from solver_selection import load_default_index
from solver_validation_spec import ValidationSpec, build_spec
from solver_validation_runner import run_jacobian_check, run_position_roundtrip
from validation_common import StrictThreshold, format_conclusion

KUAVO_SOLVER_ROOT = os.path.abspath(os.path.join(_SCRIPTS_ROOT, ".."))
REPO_ROOT = os.path.abspath(os.path.join(KUAVO_SOLVER_ROOT, "..", ".."))


# ── solver 模块加载 ──────────────────────────────────────────────────────────

def _setup_solver_path() -> None:
    try:
        import kuavo_solver_py as _  # noqa: F401
        return
    except Exception:
        pass

    def _add(p: str) -> None:
        if p and os.path.isdir(p) and p not in sys.path:
            sys.path.insert(0, p)

    _add(os.path.join(REPO_ROOT, "build", "kuavo_solver", "python"))
    _add(KUAVO_SOLVER_ROOT)
    _add(os.path.join(KUAVO_SOLVER_ROOT, "python"))
    for base in ("devel", "install"):
        root = os.path.join(REPO_ROOT, base)
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
    for start in (os.path.join(REPO_ROOT, "build"), REPO_ROOT):
        if not os.path.isdir(start):
            continue
        for cur, dirs, _files in os.walk(start):
            depth = cur[len(start):].count(os.sep)
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


def _import_solver():
    _setup_solver_path()
    try:
        import kuavo_solver_py as solver_py  # noqa: F401
        return solver_py
    except ImportError as e:
        raise RuntimeError(
            f"无法 import kuavo_solver_py，请先 source 工作空间环境; python={sys.executable}"
        ) from e


# ── 常量 ────────────────────────────────────────────────────────────────────

VALID_MODULES = ("ankle", "knee", "waist", "arm_elbow", "arm_wrist")
SIDE_MODULES = ("ankle", "knee", "waist", "arm_elbow", "arm_wrist")

_MODULE_LABELS = {
    "ankle": "脚踝 (ankle)", "knee": "膝盖 (knee)", "waist": "腰部 (waist)",
    "arm_elbow": "肘部 (arm_elbow)", "arm_wrist": "腕部 (arm_wrist)",
}

_MODULE_TO_INDEX_KEY = {
    "ankle": "ankle", "knee": "ankle",
    "waist": "waist",
    "arm_elbow": "arm", "arm_wrist": "arm",
}

IndexPart = Union[slice, int, Tuple[int, ...]]

_SIDE_QP_INDICES: Dict[str, Dict[str, Tuple[IndexPart, IndexPart]]] = {
    "ankle": {"left": (slice(0, 2), slice(0, 2)), "right": (slice(2, 4), slice(2, 4))},
    "knee": {"left": (3, 3), "right": (9, 9)},
    "arm_elbow": {"left": (3, 3), "right": (10, 10)},
    "arm_wrist": {"left": ((5, 6), (5, 6)), "right": ((12, 13), (12, 13))},
    "waist": {"pair": ((1, 2), (1, 2))},
}


# ── 工具 ────────────────────────────────────────────────────────────────────

def _scatter_fill(dim: int, idx: IndexPart, values: Sequence[float]) -> np.ndarray:
    out = np.zeros(dim, dtype=float)
    vals = [float(v) for v in values]
    if isinstance(idx, slice):
        if len(vals) != int(idx.stop - idx.start):
            raise ValueError(f"需要 {idx.stop - idx.start} 个数，收到 {len(vals)}")
        out[idx] = vals
    elif isinstance(idx, tuple):
        if len(vals) != len(idx):
            raise ValueError(f"需要 {len(idx)} 个数，收到 {len(vals)}")
        for j, jp in enumerate(idx):
            out[int(jp)] = vals[j]
    else:
        if len(vals) != 1:
            raise ValueError(f"需要 1 个数，收到 {len(vals)}")
        out[int(idx)] = vals[0]
    return out


def _parse_floats_line(s: str) -> List[float]:
    raw = (s or "").strip()
    if not raw:
        return []
    for sep in (",", "，", ";", "；"):
        raw = raw.replace(sep, " ")
    return [float(x) for x in raw.split() if x]


def _normalize_side(side: str) -> str:
    s = (side or "").strip().lower()
    if s in ("l", "left", "1"):   return "left"
    if s in ("r", "right", "2"):  return "right"
    raise ValueError("需要 side=L/R")

def _convert_slot(module: str, side: Optional[str]) -> str:
    if module == "waist":
        s = (side or "").strip().lower()
        if s in ("", "pair", "p", "pr", "2"):
            return "pair"
        raise ValueError("腰为 pitch+roll↔bars; 省略 --side 或传 pair")
    if not (side or "").strip():
        raise ValueError("需要 side=L/R")
    return _normalize_side(side)

def _normalize_dir(direction: str) -> str:
    d = (direction or "").strip().lower()
    if d in ("", "j2m", "joint", "joint_to_motor"):  return "j2m"
    if d in ("m2j", "motor", "motor_to_joint"):      return "m2j"
    raise ValueError("需要 direction=j2m/m2j")

def _slot_display(slot: str) -> str:
    if slot == "left": return "L"
    if slot == "right": return "R"
    if slot == "pair": return "pitch+roll"
    return slot

def _part_positions(part: IndexPart) -> List[int]:
    if isinstance(part, slice): return list(range(int(part.start), int(part.stop)))
    if isinstance(part, tuple): return [int(x) for x in part]
    return [int(part)]

def _gather(v: np.ndarray, part: IndexPart) -> np.ndarray:
    vv = np.asarray(v, dtype=float).reshape(-1)
    if isinstance(part, slice):   return np.asarray(vv[part].copy())
    if isinstance(part, tuple):   return np.asarray([float(vv[int(i)]) for i in part])
    return np.asarray([float(vv[int(part)])])

def _field_names(spec: ValidationSpec, slot: str, kind: str, indices: Sequence[int]) -> List[str]:
    out: List[str] = []
    for i in indices:
        name = None
        for ch in spec.channels:
            if slot not in ("pair", "shared") and ch.side != slot:
                continue
            if kind == "q" and ch.q_index == i:
                name = ch.q_name or ch.label; break
            if kind == "p" and ch.p_index == i:
                name = ch.p_name or ch.label; break
        out.append(name if name else f"{kind}[{i}]")
    return out

def _print_result(title: str, result: dict, as_json: bool) -> None:
    if as_json:
        print(json.dumps({"summary": result["summary"], "worst_cases": result["worst_cases"]},
                         indent=2, ensure_ascii=False))
    else:
        print(format_conclusion(summary=result["summary"], worst_cases=result["worst_cases"]))


# ── solver 构建 ─────────────────────────────────────────────────────────────

def _build(token: str, module: str) -> tuple[ValidationSpec, object, object]:
    if module not in VALID_MODULES:
        raise ValueError(f"未知 module={module!r}，可选: {VALID_MODULES}")
    solver_py = _import_solver()
    spec = build_spec(module=module, version=token, solver_module=solver_py)
    solver = spec.construct_solver(solver_py)
    return spec, solver, solver_py


# ── 枚举数据: module → [token, ...] ──────────────────────────────────────────

def _build_enum_data() -> Dict[str, List[str]]:
    idx = load_default_index()
    data: Dict[str, set] = {m: set() for m in VALID_MODULES}
    for _ver, entry in idx.entries.items():
        if "modules" not in entry:
            mod_v1 = str(entry.get("module", "")).strip()
            token = str(entry.get("token", "")).strip()
            if not mod_v1 or not token: continue
            for vm in VALID_MODULES:
                if _MODULE_TO_INDEX_KEY.get(vm) == mod_v1:
                    data[vm].add(token)
        else:
            for mod_key, me in (entry.get("modules") or {}).items():
                if not isinstance(me, dict): continue
                token = str(me.get("token", "")).strip()
                if not token: continue
                for vm in VALID_MODULES:
                    if _MODULE_TO_INDEX_KEY.get(vm) == mod_key:
                        data[vm].add(token)
    return {m: sorted(tokens) for m, tokens in data.items() if tokens}


# ── 位置转换 ────────────────────────────────────────────────────────────────

def convert_side_position(
    *, token: str, module: str,
    side: Optional[str], direction: str, values: Sequence[float],
) -> Dict[str, object]:
    smap = _SIDE_QP_INDICES.get(module)
    if smap is None:
        raise ValueError(f"convert-position 不支持 module={module}")
    slot = _convert_slot(module, side)
    qi, pi = smap[slot]
    d = _normalize_dir(direction)
    spec, solver, _ = _build(token, module)
    dim_q, dim_p = int(spec.dim_joint), int(spec.dim_motor)
    qi_list, pi_list = _part_positions(qi), _part_positions(pi)

    if d == "m2j":
        pf = _scatter_fill(dim_p, pi, values)
        q_out = np.asarray(spec.motor_to_joint_position(solver, pf)).reshape((dim_q,))
        return {
            "module": module, "side": slot, "direction": d,
            "input_kind": "p", "output_kind": "q",
            "input_full": pf.tolist(), "output_full": q_out.tolist(),
            "input_side": _gather(pf, pi).tolist(), "output_side": _gather(q_out, qi).tolist(),
            "input_names": _field_names(spec, slot, "p", pi_list),
            "output_names": _field_names(spec, slot, "q", qi_list),
            "dim_joint": dim_q, "dim_motor": dim_p,
        }
    else:
        qf = _scatter_fill(dim_q, qi, values)
        p_out = np.asarray(spec.joint_to_motor_position(solver, qf)).reshape((dim_p,))
        return {
            "module": module, "side": slot, "direction": d,
            "input_kind": "q", "output_kind": "p",
            "input_full": qf.tolist(), "output_full": p_out.tolist(),
            "input_side": _gather(qf, qi).tolist(), "output_side": _gather(p_out, pi).tolist(),
            "input_names": _field_names(spec, slot, "q", qi_list),
            "output_names": _field_names(spec, slot, "p", pi_list),
            "dim_joint": dim_q, "dim_motor": dim_p,
        }


# ── CLI 直通 ────────────────────────────────────────────────────────────────

def cmd_viewer(args: argparse.Namespace) -> int:
    import mujoco, mujoco.viewer
    spec, solver, _ = _build(args.token, args.module)
    xml_path = spec.mjcf_path
    if getattr(args, "kinematics_only", False):
        from mjcf_asset_audit import write_kinematics_only_mjcf
        xml_path = write_kinematics_only_mjcf(spec.mjcf_path)
        print(f"[viewer] kinematics-only: {xml_path}")
    model, data = mujoco.MjModel.from_xml_path(xml_path), mujoco.MjData(mujoco.MjModel.from_xml_path(xml_path))
    mujoco.mj_forward(model, data)
    from unified_solver_textual import run_unified_textual
    with mujoco.viewer.launch_passive(model, data) as v:
        run_unified_textual(spec=spec, solver=solver, mujoco_module=mujoco, model=model, data=data, viewer=v,
                            max_points=int(args.max_points), window_seconds=float(args.window_seconds),
                            sample_every_n_steps=int(args.sample_every_n_steps), render_hz=float(args.render_hz))
    return 0

def cmd_audit_assets(args: argparse.Namespace) -> int:
    from mjcf_asset_audit import audit_mjcf, format_report_text
    spec, _, _ = _build(args.token, args.module)
    r = audit_mjcf(spec.mjcf_path, try_load=not args.no_load, write_kinematics_only=bool(args.kinematics_only))
    print(format_report_text(r) if not args.json else json.dumps(r.to_dict(), indent=2, ensure_ascii=False))
    return 2 if (r.mesh_missing > 0 or (not r.mujoco_load_ok and not args.no_load)) else 0

def cmd_jacobian(args: argparse.Namespace) -> int:
    spec, solver, _ = _build(args.token, args.module)
    th = StrictThreshold(abs_max=float(args.abs_th), rmse=float(args.rmse_th), rel_l2=float(args.rel_l2_th))
    r = run_jacobian_check(spec=spec, solver=solver, samples=int(args.num_steps), seed=int(args.seed), threshold=th)
    _print_result("jacobian", r, args.json)
    return 0 if r["summary"]["overall_pass"] else 2

def cmd_position_verify(args: argparse.Namespace) -> int:
    spec, solver, _ = _build(args.token, args.module)
    th = StrictThreshold(abs_max=float(args.abs_th), rmse=float(args.rmse_th), rel_l2=float(args.rel_l2_th))
    r = run_position_roundtrip(spec=spec, solver=solver, samples=int(args.num_steps), seed=int(args.seed), threshold=th)
    _print_result("position-verify", r, args.json)
    return 0 if r["summary"]["overall_pass"] else 2

def cmd_convert_position(args: argparse.Namespace) -> int:
    try:
        r = convert_side_position(token=args.token, module=args.module,
                                  side=args.side, direction=args.direction,
                                  values=_parse_floats_line(args.values))
    except Exception as exc:
        print(f"[ERROR] {exc}"); return 2
    if args.json:
        print(json.dumps(r, ensure_ascii=False)); return 0
    ik, ok = str(r["input_kind"]), str(r["output_kind"])
    print(f"[{args.module} · {_slot_display(str(r['side']))} · {ik}→{ok}]")
    print(f"输入  {ik}: {', '.join(f'{n}={float(v):.6f}' for n,v in zip(r['input_names'], r['input_side']))}")
    print(f"输出  {ok}: {', '.join(f'{n}={float(v):.6f}' for n,v in zip(r['output_names'], r['output_side']))}")
    return 0


# ── 踝关节 YAML 参数误差检查 ────────────────────────────────────────────────────

def _load_ankle_yaml_config() -> dict:
    """加载 axisoffsetanklesolver.yaml + fixaxisanklesolver.yaml，返回 variant→cfg 映射。"""
    configs = {}
    for fname in ["config/ankle/axisoffsetanklesolver.yaml", "config/ankle/fixaxisanklesolver.yaml"]:
        path = os.path.join(KUAVO_SOLVER_ROOT, fname)
        if not os.path.isfile(path):
            continue
        cfg = yaml.safe_load(open(path, "r", encoding="utf-8"))
        for name, v in cfg.get("variants", {}).items():
            configs[name] = v
    return configs


def _ankle_solver_tendon_len(cfg: dict, side: str,
                              pitch: float = 0.0, roll: float = 0.0, bar: float = 0.0) -> float:
    """复现 AxisOffsetAnkleSolver compute_tendon_vector (bar_rpy=0)。"""
    keys = {
        "ll": ("x_lleq","y_lleq","z_lleq","x_llbar","z_llbar","x_lltd","y_lltd","z_lltd"),
        "lr": ("x_lreq","y_lreq","z_lreq","x_lrbar","z_lrbar","x_lrtd","y_lrtd","z_lrtd"),
        "rl": ("x_rleq","y_rleq","z_rleq","x_rlbar","z_rlbar","x_rltd","y_rltd","z_rltd"),
        "rr": ("x_rreq","y_rreq","z_rreq","x_rrbar","z_rrbar","x_rrtd","y_rrtd","z_rrtd"),
    }[side]
    x_eq, y_eq, z_eq = (cfg[keys[0]], cfg[keys[1]], cfg[keys[2]])
    x_bar, z_bar = cfg[keys[3]], cfg[keys[4]]
    x_td, y_td, z_td = cfg[keys[5]], cfg[keys[6]], cfg[keys[7]]

    zpr = cfg["z_pitch"] - cfg["z_roll"]
    c1, s1 = math.cos(pitch), math.sin(pitch)
    c2, s2 = math.cos(roll), math.sin(roll)
    ca, sa = math.cos(bar), math.sin(bar)

    p_eq_x = cfg["x_pitch"] + cfg["z_roll"]*s1 + c1*x_eq + s1*s2*y_eq + s1*c2*z_eq
    p_eq_y = c2*y_eq - s2*z_eq
    p_eq_z = zpr + cfg["z_roll"]*c1 - s1*x_eq + c1*s2*y_eq + c1*c2*z_eq

    p_td_x = x_bar + x_td
    p_td_y = ca*y_td - sa*z_td
    p_td_z = z_bar + sa*y_td + ca*z_td

    return math.sqrt((p_td_x-p_eq_x)**2 + (p_td_y-p_eq_y)**2 + (p_td_z-p_eq_z)**2)


def _ankle_check_mjcf(cfg: dict, token: str) -> bool:
    """MJCF 运动学链一致性检查。"""
    import xml.etree.ElementTree as ET
    mjcf_path = os.path.join(KUAVO_SOLVER_ROOT, "robot-descriptions/ankles",
                              f"biped_ankles_{token}", "mjcf/biped_ankles.xml")
    if not os.path.isfile(mjcf_path):
        print(f"    MJCF 不存在: {mjcf_path}")
        return False
    tree = ET.parse(mjcf_path)
    wb = tree.getroot().find("worldbody")

    def find_chain(bar_n, tdn_n):
        for b in wb.iter("body"):
            if b.get("name") == bar_n:
                bp = np.array([float(v) for v in b.get("pos","0 0 0").split()])
                for sub in b:
                    if sub.tag == "body" and sub.get("name") == tdn_n:
                        body = np.array([float(v) for v in sub.get("pos","0 0 0").split()])
                        jt = sub.find("joint")
                        ball = np.array([float(v) for v in jt.get("pos","0 0 0").split()]) if jt is not None else np.zeros(3)
                        return bp, body, ball
        return None

    ok = True
    for name, bn, tn in [("ll","l_l_bar","l_l_tendon"),("lr","l_r_bar","l_r_tendon"),
                          ("rl","r_l_bar","r_l_tendon"),("rr","r_r_bar","r_r_tendon")]:
        r = find_chain(bn, tn)
        if r is None:
            print(f"    {name}: MJCF parse failed")
            ok = False; continue
        bp, body, ball = r
        mj_total = bp + body + ball
        yb = np.array([cfg[f"x_{name}bar"], 0.0, cfg[f"z_{name}bar"]])
        yt = np.array([cfg[f"x_{name}td"], cfg[f"y_{name}td"], cfg[f"z_{name}td"]])
        y_pin = yb + yt
        err = float(np.linalg.norm(mj_total - y_pin) * 1000)
        st = "✓" if err < 0.01 else "✗"
        if err >= 0.01: ok = False
        print(f"    {name}: MJCF chain vs YAML pin  err={err:.3f}mm {st}")
    return ok


def cmd_ankle_check(args: argparse.Namespace) -> int:
    """CLI: python3 kuavo_solver_validator.py -t <variant> ankle-check [--mjcf] [--all] [--table]"""
    configs = _load_ankle_yaml_config()

    if args.all:
        targets = sorted(configs.keys())
    elif args.token:
        targets = [args.token]
    else:
        targets = sorted(configs.keys())

    if args.table:
        # 表格模式：扫所有 AxisOffset variant
        print(f"\n{'variant':>14s} | {'零位 err(mm)':>14s} | {'扫参 max(mm)':>14s} | "
              f"{'扫参 p95(mm)':>14s} | {'扫参 p99(mm)':>14s} | {'状态':>6s}")
        print("-" * 90)
        for name in targets:
            cfg = configs.get(name)
            if cfg is None or "z_roll" not in cfg:
                print(f"{name:>14s} | {'N/A':>14s} | {'N/A':>14s} | {'N/A':>14s} | {'N/A':>14s} | {'skip':>6s}")
                continue
            p_lim = cfg["ankle_pitch_limits"]; r_lim = cfg["ankle_roll_limits"]
            g = 7
            ps = np.linspace(p_lim[0], p_lim[1], g); rs = np.linspace(r_lim[0], r_lim[1], g)
            zero_errs, sweep_errs = [], []
            for s in ["ll","lr","rl","rr"]:
                l0 = cfg[f"l0_{s}_eqtd"]
                zero_errs.append(abs(_ankle_solver_tendon_len(cfg, s) - l0) * 1000)
                for pt in range(g):
                    for rl in range(g):
                        sweep_errs.append(abs(_ankle_solver_tendon_len(cfg, s, float(ps[pt]), float(rs[rl])) - l0) * 1000)
            z = np.array(zero_errs); sw = np.array(sweep_errs)
            zero_ok = z.max() < 0.001
            print(f"{name:>14s} | {z.max():>13.4f} | {sw.max():>13.1f} | "
                  f"{np.percentile(sw,95):>13.1f} | {np.percentile(sw,99):>13.1f} | "
                  f"{'✓' if zero_ok else '✗':>6s}")
        print("\n  注: 扫参列显示 bar=0 时腱长距离 l0 的几何范围，非精度误差")
        return 0

    # 单 / 多 variant 详细模式
    all_ok = True
    for name in targets:
        cfg = configs.get(name)
        if cfg is None:
            print(f"[ERROR] variant '{name}' 未找到"); all_ok = False; continue
        if "z_roll" not in cfg:
            print(f"\n{'='*60}\n  {name}  (FixedAxis, 跳过腱长检查)\n{'='*60}")
            continue

        print(f"\n{'='*60}\n  {name}\n{'='*60}")

        # 1 零位自洽
        print("\n[1] 零位自洽 (pitch=roll=bar=0)")
        for s in ["ll","lr","rl","rr"]:
            L = _ankle_solver_tendon_len(cfg, s)
            l0 = cfg[f"l0_{s}_eqtd"]
            err = abs(L - l0) * 1000
            st = "✓" if err < 0.001 else "✗"
            if err >= 0.001: all_ok = False
            print(f"    {s}: len={L:.12f}  l0={l0:.12f}  err={err:.4f}mm {st}")

        # 2 工作空间扫参（bar=0 几何范围，仅供参考）
        print("\n[2] 扫参几何范围 (bar=0, {}×{} grid, 非精度误差)".format(
            int(args.grid_size), int(args.grid_size)))
        p_lim = cfg["ankle_pitch_limits"]; r_lim = cfg["ankle_roll_limits"]
        g = int(args.grid_size)
        ps = np.linspace(p_lim[0], p_lim[1], g)
        rs = np.linspace(r_lim[0], r_lim[1], g)
        for s in ["ll","lr","rl","rr"]:
            errs = []
            l0 = cfg[f"l0_{s}_eqtd"]
            for pt in range(g):
                for rl in range(g):
                    L = _ankle_solver_tendon_len(cfg, s, float(ps[pt]), float(rs[rl]))
                    errs.append(abs(L - l0) * 1000)
            e = np.array(errs)
            print(f"    {s}: max={e.max():.1f}mm  mean={e.mean():.1f}mm  "
                  f"p95={np.percentile(e,95):.1f}mm  p99={np.percentile(e,99):.1f}mm")

        # 3 C++ 往返精度（调用 position-verify）
        if args.roundtrip:
            print("\n[3] C++ solver 往返精度 (MuJoCo vs solver)")
            try:
                import subprocess
                validator = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                        "kuavo_solver_validator.py")
                result = subprocess.run(
                    [sys.executable, validator, "-t", name, "position-verify",
                     "--module", "ankle", "--num-steps", str(int(args.grid_size))],
                    capture_output=True, text=True, timeout=120)
                for line in result.stdout.strip().split("\n"):
                    if "PASS" in line or "FAIL" in line:
                        print(f"    {line.strip()}")
                if result.returncode != 0:
                    all_ok = False
            except Exception as exc:
                print(f"    [SKIP] 无法运行 C++ roundtrip: {exc}")

        # 4 MJCF 链路 (可选)
        if args.mjcf:
            print("\n[4] MJCF 运动学链")
            if not _ankle_check_mjcf(cfg, name):
                all_ok = False

        print(f"\n  → {'全部通过 ✓' if all_ok else '存在问题 ✗'}")

    print(f"\n{'='*60}")
    print(f"  总计: {len(targets)} variant, {'全部通过 ✓' if all_ok else '存在问题 ✗'}")
    print(f"{'='*60}")
    return 0 if all_ok else 1


# ── 交互菜单 ─────────────────────────────────────────────────────────────────

MENU_BANNER = r"""
╔══════════════════════════════════════════════════════╗
║         Kuavo Solver Validator                       ║
╚══════════════════════════════════════════════════════╝"""


def _pick_module() -> Optional[str]:
    print()
    for i, m in enumerate(VALID_MODULES):
        print(f"  [{i + 1}] {_MODULE_LABELS[m]}")
    print(f"  [0] 返回")
    c = input("\n  选择部位: ").strip()
    if c in ("0", ""): return None
    try:
        idx = int(c) - 1
        if 0 <= idx < len(VALID_MODULES): return VALID_MODULES[idx]
    except ValueError: pass
    return None


def _pick_token(module: str, enum_data: dict) -> Optional[str]:
    tokens = enum_data.get(module, [])
    if not tokens:
        print(f"\n  ⚠ 部位 {module} 无可用解算器"); return None
    print(f"\n  {_MODULE_LABELS.get(module, module)} 可选解算器 (xgen):")
    for i, tok in enumerate(tokens):
        print(f"    [{i + 1}] {tok}")
    print(f"    [0] 返回")
    c = input("\n  选择编号: ").strip()
    if c in ("0", ""): return None
    try:
        idx = int(c) - 1
        if 0 <= idx < len(tokens): return tokens[idx]
    except ValueError: pass
    if c in tokens: return c
    return None


def _do_verify(token: str, module: str, mode: str) -> None:
    samples = input("  采样数 [50]: ").strip() or "50"
    print()
    try:
        spec, solver, _ = _build(token, module)
        th = StrictThreshold()
        if mode == "jacobian":
            r = run_jacobian_check(spec=spec, solver=solver, samples=int(samples), seed=0, threshold=th)
        else:
            r = run_position_roundtrip(spec=spec, solver=solver, samples=int(samples), seed=0, threshold=th)
        _print_result(mode, r, False)
    except Exception as e:
        print(f"  ✗ 错误: {e}\n{traceback.format_exc()}")


def menu_enum(enum_data: dict) -> None:
    print("\n" + "=" * 60)
    print("  解算器枚举 (xgen)")
    print("=" * 60)
    for module in VALID_MODULES:
        tokens = enum_data.get(module, [])
        if not tokens: continue
        print(f"\n  ── {_MODULE_LABELS.get(module, module)} ──")
        print(f"    {', '.join(tokens)}")
    print()

def menu_position_verify(enum_data: dict) -> None:
    m = _pick_module()
    if not m: return
    t = _pick_token(m, enum_data)
    if not t: return
    _do_verify(t, m, "position-verify")

def menu_jacobian(enum_data: dict) -> None:
    m = _pick_module()
    if not m: return
    t = _pick_token(m, enum_data)
    if not t: return
    _do_verify(t, m, "jacobian")

def menu_viewer(enum_data: dict) -> None:
    m = _pick_module()
    if not m: return
    t = _pick_token(m, enum_data)
    if not t: return
    print(f"\n  启动 MuJoCo viewer: token={t} module={m} ...")
    cli = os.path.join(_SCRIPTS_ROOT, "kuavo_solver_validator.py")
    subprocess.Popen([sys.executable, cli, "-t", t, "viewer", "--module", m], cwd=_SCRIPTS_ROOT)

def menu_convert(enum_data: dict) -> None:
    m = _pick_module()
    if not m: return
    t = _pick_token(m, enum_data)
    if not t: return
    if m == "waist":
        side_val = "pair"; print("  腰部默认 pair (pitch+roll↔bars)")
    else:
        side_val = input("  侧 (L/R) [L]: ").strip() or "L"
    direction = input("  方向 (j2m/m2j) [j2m]: ").strip() or "j2m"
    vals_raw = input("  数值 (逗号分隔): ").strip()
    if not vals_raw:
        print("  ⚠ 需要输入数值"); return
    print()
    try:
        r = convert_side_position(token=t, module=m, side=side_val, direction=direction,
                                  values=_parse_floats_line(vals_raw))
        ik, ok = str(r["input_kind"]), str(r["output_kind"])
        print(f"  [{m} · {_slot_display(str(r['side']))} · {ik}→{ok}]")
        print(f"  输入  {ik}: {', '.join(f'{n}={float(v):.6f}' for n,v in zip(r['input_names'], r['input_side']))}")
        print(f"  输出  {ok}: {', '.join(f'{n}={float(v):.6f}' for n,v in zip(r['output_names'], r['output_side']))}")
    except Exception as e:
        print(f"  ✗ 错误: {e}")

def menu_batch() -> None:
    from solver_test_runner import (SolverTestRunner, write_json_report, write_html_report,
                                    write_csv_report, write_markdown_summary, write_plots)
    print("\n  批量回归测试")
    level = input("  级别 [1=Smoke(5)/2=Full(50)]: ").strip() or "1"
    token_f = input("  过滤 token (空=全部): ").strip() or None
    mod_f = input("  过滤 module (空=全部): ").strip() or None
    odir = input("  输出目录 [/tmp/solver_test_results]: ").strip() or "/tmp/solver_test_results"
    print(f"\n  运行中...")
    try:
        runner = SolverTestRunner()
        kw = dict(filter_token=token_f, filter_module=mod_f, filter_version=None)
        if level == "1":
            results, label = runner.run_smoke(**kw), "L1 Smoke"
        else:
            results, label = runner.run_all(samples=50, seed=0, **kw), "L2 Full"
        os.makedirs(odir, exist_ok=True)
        write_json_report(results, os.path.join(odir, "solver_test_report.json"))
        write_html_report(results, os.path.join(odir, "solver_test_report.html"))
        write_csv_report(results, os.path.join(odir, "solver_test_report.csv"))
        write_markdown_summary(results, os.path.join(odir, "solver_test_summary.md"))
        write_plots(results, odir)
        total, passed = len(results), sum(1 for r in results if r.passed)
        print(f"\n  [{label}] {passed}/{total} passed · 报告: {odir}")
        for r in results:
            print(f"    {'✓' if r.passed else '✗'} {r.test_type:25s} {r.case.canonical_key}")
    except Exception as e:
        print(f"  ✗ 错误: {e}")

def menu_assets(enum_data: dict) -> None:
    from mjcf_asset_audit import audit_mjcf, format_report_text
    m = _pick_module()
    if not m: return
    t = _pick_token(m, enum_data)
    if not t: return
    print()
    try:
        spec, _, _ = _build(t, m)
        rpt = audit_mjcf(spec.mjcf_path, try_load=True, write_kinematics_only=False)
        print(format_report_text(rpt))
    except Exception as e:
        print(f"  ✗ 错误: {e}")


def interactive_menu() -> int:
    print(MENU_BANNER)
    try:
        enum_data = _build_enum_data()
    except Exception as e:
        print(f"  ⚠ 无法加载版本索引: {e}"); return 1

    items = [
        ("1", "枚举解算器        列出每个部位可选解算器 (xgen)"),
        ("2", "位置验证 (position) q→p→q 往返 & MuJoCo 读回"),
        ("3", "Jacobian 验证      solver vs MuJoCo 速度映射"),
        ("4", "Viewer             启动 MuJoCo 3D 查看器"),
        ("5", "位置转换           单侧 joint↔motor 转换"),
        ("6", "批量回归测试       Smoke / Full regression"),
        ("7", "资产审计           检查 MJCF mesh 可用性"),
        ("0", "退出"),
    ]
    while True:
        print("\n" + "─" * 60)
        for num, desc in items:
            print(f"  [{num}] {desc}")
        c = input("\n  选择: ").strip()
        if c == "0":        print("  退出."); return 0
        elif c == "1":      menu_enum(enum_data)
        elif c == "2":      menu_position_verify(enum_data)
        elif c == "3":      menu_jacobian(enum_data)
        elif c == "4":      menu_viewer(enum_data)
        elif c == "5":      menu_convert(enum_data)
        elif c == "6":      menu_batch()
        elif c == "7":      menu_assets(enum_data)
        elif c == "":       continue
        else:               print(f"  未知选项: {c}")


# ── argparse ────────────────────────────────────────────────────────────────

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Kuavo Solver Validator")
    p.add_argument("-t", "--token", help="解算器 xgen 标识 (如 s2gen_2, 7gen, 5gen)")

    sub = p.add_subparsers(dest="command")

    pv = sub.add_parser("viewer", help="MuJoCo viewer + 实时对比")
    pv.add_argument("--module", required=True, choices=VALID_MODULES)
    pv.add_argument("--kinematics-only", action="store_true")
    pv.add_argument("--max-points", type=int, default=800)
    pv.add_argument("--window-seconds", type=float, default=10.0)
    pv.add_argument("--sample-every-n-steps", type=int, default=5)
    pv.add_argument("--render-hz", type=float, default=20.0)
    pv.set_defaults(func=cmd_viewer)

    pa = sub.add_parser("audit-assets", help="MJCF mesh 审计")
    pa.add_argument("--module", required=True, choices=VALID_MODULES)
    pa.add_argument("--json", action="store_true"); pa.add_argument("--no-load", action="store_true")
    pa.add_argument("--kinematics-only", action="store_true")
    pa.set_defaults(func=cmd_audit_assets)

    pj = sub.add_parser("jacobian", help="Jacobian 验证")
    pj.add_argument("--module", required=True, choices=VALID_MODULES)
    pj.add_argument("--num-steps", type=int, default=50); pj.add_argument("--seed", type=int, default=0)
    pj.add_argument("--abs-th", type=float, default=1e-4); pj.add_argument("--rmse-th", type=float, default=5e-5)
    pj.add_argument("--rel-l2-th", type=float, default=2e-3); pj.add_argument("--json", action="store_true")
    pj.set_defaults(func=cmd_jacobian)

    pp = sub.add_parser("position-verify", help="位置验证")
    pp.add_argument("--module", required=True, choices=VALID_MODULES)
    pp.add_argument("--num-steps", type=int, default=50); pp.add_argument("--seed", type=int, default=0)
    pp.add_argument("--abs-th", type=float, default=1e-4); pp.add_argument("--rmse-th", type=float, default=5e-5)
    pp.add_argument("--rel-l2-th", type=float, default=2e-3); pp.add_argument("--json", action="store_true")
    pp.set_defaults(func=cmd_position_verify)

    pc = sub.add_parser("convert-position", help="单侧 joint↔motor 转换")
    pc.add_argument("--module", required=True, choices=SIDE_MODULES)
    pc.add_argument("--side", default=None); pc.add_argument("--direction", required=True, choices=("j2m", "m2j"))
    pc.add_argument("--values", required=True); pc.add_argument("--json", action="store_true")
    pc.set_defaults(func=cmd_convert_position)

    # ankle-check: 踝关节 YAML 参数腱长误差
    pa = sub.add_parser("ankle-check", help="踝关节 YAML 腱长误差扫参")
    pa.add_argument("--mjcf", action="store_true", help="同时检查 MJCF 运动学链一致性")
    pa.add_argument("--roundtrip", action="store_true", help="运行 C++ solver MuJoCo 往返精度验证 (需要编译)")
    pa.add_argument("--all", action="store_true", help="检查所有 ankle variant")
    pa.add_argument("--table", action="store_true", help="表格模式（快速对比多版本）")
    pa.add_argument("--grid-size", type=int, default=9, help="扫参网格点数 (default: 9)")
    pa.set_defaults(func=cmd_ankle_check)

    return p


def main(argv: Optional[List[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    if args.command is not None:
        if not args.token:
            print("[ERROR] 需要 --token/-t (xgen 标识, 如 s2gen_2, 7gen)"); return 2
        return int(args.func(args))
    return interactive_menu()


if __name__ == "__main__":
    raise SystemExit(main())
