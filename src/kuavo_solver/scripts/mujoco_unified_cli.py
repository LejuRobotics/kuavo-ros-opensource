#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MuJoCo 并联机构统一 CLI。

两种用法：

1. 平铺子命令（非交互，便于脚本化 / smoke 测试）::

      python3 mujoco_unified_cli.py --version s70 viewer   --module ankle
      python3 mujoco_unified_cli.py --version s70 jacobian --module arm_wrist --num-steps 20
      python3 mujoco_unified_cli.py --version s70 position-verify --module waist --num-steps 50
      python3 mujoco_unified_cli.py --version s70 convert-position --module knee --side L --direction j2m --values 1.2 --json

2. 交互菜单（保留原有 UX）；子菜单可选 4「单侧 L/R，joint→motor 或 motor→joint」（踝/膝/肘/腕）::

      python3 mujoco_unified_cli.py --interactive

框架
----
- 所有 module 共用同一 Spec（见 `solver_validation_spec.build_spec`）；
- viewer 走 `unified_solver_gui.run_unified_gui`；
- jacobian / position-verify 走 `solver_validation_runner`；
- 严格 fail-fast：solver 接口异常直接冒泡，不静默 fallback。
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from typing import Dict, List, Optional, Sequence, Tuple, Union

import numpy as np

# 保证 `lib` / `validation` / `tools` 下模块可按平铺名导入（与历史用法一致）
_SCRIPTS_ROOT = os.path.dirname(os.path.abspath(__file__))
for _sub in ("lib", "validation", "tools"):
    _p = os.path.join(_SCRIPTS_ROOT, _sub)
    if _p not in sys.path:
        sys.path.insert(0, _p)
if _SCRIPTS_ROOT not in sys.path:
    sys.path.insert(0, _SCRIPTS_ROOT)

from solver_selection import normalize_version
from solver_validation_spec import ValidationSpec, build_spec
from solver_validation_runner import run_jacobian_check, run_position_roundtrip
from unified_solver_gui import run_unified_gui
from validation_common import StrictThreshold, format_conclusion


KUAVO_SOLVER_ROOT = os.path.abspath(os.path.join(_SCRIPTS_ROOT, ".."))
REPO_ROOT = os.path.abspath(os.path.join(KUAVO_SOLVER_ROOT, "..", ".."))


# ---------------------------------------------------------------------------
# solver 模块加载
# ---------------------------------------------------------------------------

def _setup_solver_path() -> None:
    try:
        import kuavo_solver_py as _  # type: ignore  # noqa: F401
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
        import kuavo_solver_py as solver_py  # type: ignore
        return solver_py
    except ImportError as e:
        raise RuntimeError(
            f"无法 import kuavo_solver_py，请先 source 工作空间环境；当前 python={sys.executable}"
        ) from e


# ---------------------------------------------------------------------------
# 三个子命令的统一实现（spec-driven）
# ---------------------------------------------------------------------------

VALID_MODULES = ("ankle", "knee", "waist", "arm_elbow", "arm_wrist")
SIDE_MODULES = ("ankle", "knee", "arm_elbow", "arm_wrist")

IndexPart = Union[slice, int, Tuple[int, ...]]

# interactive 单侧 j↔m：单侧 q、p 下标（其余分量填 0）
_SIDE_QP_INDICES: Dict[str, Dict[str, Tuple[IndexPart, IndexPart]]] = {
    "ankle": {
        "left": (slice(0, 2), slice(0, 2)),
        "right": (slice(2, 4), slice(2, 4)),
    },
    "knee": {"left": (3, 3), "right": (9, 9)},
    "arm_elbow": {"left": (3, 3), "right": (10, 10)},
    "arm_wrist": {
        "left": ((5, 6), (5, 6)),
        "right": ((12, 13), (12, 13)),
    },
}


def _dof_count(part: IndexPart) -> int:
    if isinstance(part, slice):
        return int(part.stop - part.start)
    if isinstance(part, tuple):
        return len(part)
    return 1


def _scatter_fill(dim: int, idx: IndexPart, values: Sequence[float]) -> np.ndarray:
    out = np.zeros(dim, dtype=float)
    vals = [float(v) for v in values]
    if isinstance(idx, slice):
        n = idx.stop - idx.start  # type: ignore[operator]
        if len(vals) != int(n):
            raise ValueError(f"需要 {n} 个数，收到 {len(vals)}")
        out[idx] = vals
        return out
    if isinstance(idx, tuple):
        if len(vals) != len(idx):
            raise ValueError(f"需要 {len(idx)} 个数，收到 {len(vals)}")
        for j, jp in enumerate(idx):
            out[int(jp)] = vals[j]
        return out
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
    parts = [p for p in raw.split() if p]
    return [float(x) for x in parts]


def _normalize_side_key(side: str) -> str:
    s = (side or "").strip().lower()
    if s in ("l", "左", "left", "1"):
        return "left"
    if s in ("r", "右", "right", "2"):
        return "right"
    raise ValueError("需要 side=L/R（或 left/right）")


def _normalize_direction(direction: str) -> str:
    d = (direction or "").strip().lower()
    if d in ("", "j2m", "joint", "joint_to_motor", "joint→motor", "joint->motor"):
        return "j2m"
    if d in ("m", "m2j", "motor", "motor_to_joint", "motor→joint", "motor->joint"):
        return "m2j"
    raise ValueError("需要 direction=j2m/m2j")


def _part_positions(part: IndexPart) -> List[int]:
    if isinstance(part, slice):
        return list(range(int(part.start), int(part.stop)))
    if isinstance(part, tuple):
        return [int(x) for x in part]
    return [int(part)]


def _side_field_names(spec: ValidationSpec, side: str, kind: str, indices: Sequence[int]) -> List[str]:
    if kind not in ("q", "p"):
        raise ValueError("kind 必须是 q 或 p")
    out: List[str] = []
    for idx in indices:
        name: Optional[str] = None
        for ch in spec.channels:
            if ch.side != side:
                continue
            if kind == "q" and ch.q_index == idx:
                name = ch.q_name or ch.label
                break
            if kind == "p" and ch.p_index == idx:
                name = ch.p_name or ch.label
                break
        out.append(name if name else f"{kind}[{idx}]")
    return out


def _build(version: str, module: str) -> tuple[ValidationSpec, object, object]:
    if module not in VALID_MODULES:
        raise ValueError(f"未知 module={module!r}，可选: {VALID_MODULES}")
    solver_py = _import_solver()
    spec = build_spec(module=module, version=version, solver_module=solver_py)
    solver = spec.construct_solver(solver_py)
    return spec, solver, solver_py


def convert_side_position(
    *,
    version: str,
    module: str,
    side: str,
    direction: str,
    values: Sequence[float],
) -> Dict[str, object]:
    smap = _SIDE_QP_INDICES.get(module)
    if smap is None:
        raise ValueError(f"convert-position/单侧 q↔p 不支持 module={module}")

    side_k = _normalize_side_key(side)
    qi, pi = smap[side_k]
    dnorm = _normalize_direction(direction)

    spec, solver, _ = _build(version, module)
    dim_q = int(spec.dim_joint)
    dim_p = int(spec.dim_motor)
    q_positions = _part_positions(qi)
    p_positions = _part_positions(pi)

    if dnorm == "m2j":
        pf = _scatter_fill(dim_p, pi, values)
        q_out = np.asarray(spec.motor_to_joint_position(solver, pf), dtype=float).reshape((dim_q,))
        side_in = _gather_slots(pf, pi).tolist()
        side_out = _gather_slots(q_out, qi).tolist()
        in_names = _side_field_names(spec, side_k, "p", p_positions)
        out_names = _side_field_names(spec, side_k, "q", q_positions)
        input_full = pf.tolist()
        output_full = q_out.tolist()
        input_kind = "p"
        output_kind = "q"
    else:
        qf = _scatter_fill(dim_q, qi, values)
        p_out = np.asarray(spec.joint_to_motor_position(solver, qf), dtype=float).reshape((dim_p,))
        side_in = _gather_slots(qf, qi).tolist()
        side_out = _gather_slots(p_out, pi).tolist()
        in_names = _side_field_names(spec, side_k, "q", q_positions)
        out_names = _side_field_names(spec, side_k, "p", p_positions)
        input_full = qf.tolist()
        output_full = p_out.tolist()
        input_kind = "q"
        output_kind = "p"

    return {
        "module": module,
        "side": side_k,
        "direction": dnorm,
        "input_kind": input_kind,
        "output_kind": output_kind,
        "input_full": input_full,
        "output_full": output_full,
        "input_side": side_in,
        "output_side": side_out,
        "input_names": in_names,
        "output_names": out_names,
        "dim_joint": dim_q,
        "dim_motor": dim_p,
    }


def cmd_viewer(args: argparse.Namespace) -> int:
    import mujoco
    import mujoco.viewer

    spec, solver, _ = _build(args.version, args.module)
    model = mujoco.MjModel.from_xml_path(spec.mjcf_path)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        run_unified_gui(
            spec=spec, solver=solver,
            mujoco_module=mujoco, model=model, data=data, viewer=viewer,
            max_points=int(args.max_points),
            window_seconds=float(args.window_seconds),
            sample_every_n_steps=int(args.sample_every_n_steps),
            render_hz=float(args.render_hz),
        )
    return 0


def _print_result(title: str, result: dict, as_json: bool) -> None:
    summary = result["summary"]
    if as_json:
        print(json.dumps({"summary": summary, "worst_cases": result["worst_cases"]}, indent=2, ensure_ascii=False))
    else:
        print(format_conclusion(summary=summary, worst_cases=result["worst_cases"]))


def cmd_jacobian(args: argparse.Namespace) -> int:
    spec, solver, _ = _build(args.version, args.module)
    th = StrictThreshold(abs_max=float(args.abs_th), rmse=float(args.rmse_th), rel_l2=float(args.rel_l2_th))
    result = run_jacobian_check(spec=spec, solver=solver, samples=int(args.num_steps), seed=int(args.seed), threshold=th)
    _print_result("jacobian", result, args.json)
    return 0 if result["summary"]["overall_pass"] else 2


def cmd_position_verify(args: argparse.Namespace) -> int:
    spec, solver, _ = _build(args.version, args.module)
    th = StrictThreshold(abs_max=float(args.abs_th), rmse=float(args.rmse_th), rel_l2=float(args.rel_l2_th))
    result = run_position_roundtrip(spec=spec, solver=solver, samples=int(args.num_steps), seed=int(args.seed), threshold=th)
    _print_result("position-verify", result, args.json)
    return 0 if result["summary"]["overall_pass"] else 2


def _format_named_pairs(names: Sequence[str], values: Sequence[float]) -> str:
    return ", ".join(f"{n}={float(v):.6f}" for n, v in zip(names, values))


def cmd_convert_position(args: argparse.Namespace) -> int:
    try:
        result = convert_side_position(
            version=args.version,
            module=args.module,
            side=args.side,
            direction=args.direction,
            values=_parse_floats_line(args.values),
        )
    except Exception as exc:
        print(f"[ERROR] {exc}")
        return 2

    if args.json:
        print(json.dumps(result, ensure_ascii=False))
        return 0

    in_kind = str(result["input_kind"])
    out_kind = str(result["output_kind"])
    side = "L" if str(result["side"]) == "left" else "R"
    print(f"[{args.module} · {side} · {in_kind}→{out_kind}]")
    print(f"输入  {in_kind}[{side}]: {_format_named_pairs(result['input_names'], result['input_side'])}")
    print(f"输出  {out_kind}[{side}]: {_format_named_pairs(result['output_names'], result['output_side'])}")
    print(f"q(full)={np.asarray(result['input_full'] if in_kind == 'q' else result['output_full'], dtype=float)}")
    print(f"p(full)={np.asarray(result['input_full'] if in_kind == 'p' else result['output_full'], dtype=float)}")
    return 0


# ---------------------------------------------------------------------------
# 交互菜单
# ---------------------------------------------------------------------------

def _prompt(msg: str) -> str:
    try:
        return input(msg).strip()
    except EOFError:
        return ""


def _gather_slots(v: np.ndarray, part: IndexPart) -> np.ndarray:
    vv = np.asarray(v, dtype=float).reshape(-1)
    if isinstance(part, slice):
        return np.asarray(vv[part].copy())
    if isinstance(part, tuple):
        return np.asarray([float(vv[int(i)]) for i in part], dtype=float)
    return np.asarray([float(vv[int(part)])], dtype=float)


def interactive_side_joint_motor(version: str, module: str) -> None:
    smap = _SIDE_QP_INDICES.get(module)
    if smap is None:
        print("[INFO] 当前部位无双侧单列项。")
        return
    skey = _prompt("侧 [左=L · 右=R]: ").strip().lower()
    try:
        side_k = _normalize_side_key(skey)
    except Exception as exc:
        print(f"[ERROR] {exc}")
        return
    qi, pi = smap[side_k]
    dirhint = _prompt("方向 [回车=joint→motor · m=motor→joint]: ").strip()
    try:
        dnorm = _normalize_direction(dirhint)
    except Exception as exc:
        print(f"[ERROR] {exc}")
        return

    try:
        spec, _, _ = _build(version, module)
        if dnorm == "m2j":
            names = _side_field_names(spec, side_k, "p", _part_positions(pi))
            n_need = _dof_count(pi)
            side_s = "L" if side_k == "left" else "R"
            print(f"[{module} · {side_s} · motor→joint] 需要 {n_need} 个数，顺序: {', '.join(names)}")
            line = _prompt(f"p 单侧分量（{n_need} 个数）: ")
        else:
            names = _side_field_names(spec, side_k, "q", _part_positions(qi))
            n_need = _dof_count(qi)
            side_s = "L" if side_k == "left" else "R"
            print(f"[{module} · {side_s} · joint→motor] 需要 {n_need} 个数，顺序: {', '.join(names)}")
            line = _prompt(f"q 单侧分量（{n_need} 个数）: ")

        result = convert_side_position(
            version=version,
            module=module,
            side=side_k,
            direction=dnorm,
            values=_parse_floats_line(line),
        )
        side_s = "L" if str(result["side"]) == "left" else "R"
        in_kind = str(result["input_kind"])
        out_kind = str(result["output_kind"])
        print(f"输入  {in_kind}[{side_s}]: {_format_named_pairs(result['input_names'], result['input_side'])}")
        print(f"输出  {out_kind}[{side_s}]: {_format_named_pairs(result['output_names'], result['output_side'])}")
        print(f"q(full)={np.asarray(result['input_full'] if in_kind == 'q' else result['output_full'], dtype=float)}")
        print(f"p(full)={np.asarray(result['input_full'] if in_kind == 'p' else result['output_full'], dtype=float)}")
    except Exception as exc:
        print(f"[ERROR] {exc}")


def _prompt_leg_module() -> str:
    hint = _prompt("子机构 [回车=踝 · k=膝]: ").lower()
    return "knee" if hint == "k" else "ankle"


def _prompt_arm_module() -> str:
    hint = _prompt("子机构 [回车=腕 · e=肘]: ").lower()
    return "arm_elbow" if hint == "e" else "arm_wrist"


def _default_ns(version: str, module: str) -> argparse.Namespace:
    return argparse.Namespace(
        version=version, module=module,
        num_steps=20, seed=0,
        abs_th=5e-3, rmse_th=1e-3, rel_l2_th=5e-3,
        json=False,
        max_points=800, window_seconds=10.0, sample_every_n_steps=5, render_hz=20.0,
    )


def _group_actions(version: str, module: str, label: str) -> None:
    while True:
        print(f"\n  [{label}]  1 viewer  2 jacobian  3 position-verify  4 单侧 joint↔motor  0 返回")
        c = _prompt("选: ")
        if c == "0":
            return
        ns = _default_ns(version, module)
        if c == "1":
            cmd_viewer(ns)
            continue
        if c == "2":
            ns.num_steps = 20
            cmd_jacobian(ns)
            continue
        if c == "3":
            ns.num_steps = 50
            cmd_position_verify(ns)
            continue
        if c == "4":
            interactive_side_joint_motor(version, module)
            continue
        print("[ERROR] 无效选项")


def run_interactive(initial_version: Optional[str]) -> int:
    ver = normalize_version(initial_version) if initial_version else ""

    print("\n" + "=" * 56)
    print("  MuJoCo 并联求解器 · 统一入口（spec-driven）")
    print("=" * 56)

    while True:
        if not ver:
            s = _prompt("版本号（如 s70，与 kuavo_vNN 对应）: ")
            ver = normalize_version(s)
            if not ver:
                print("[ERROR] 需要版本号")
                continue

        print(f"\n当前版本: {ver}")
        print("  1 腿（踝/膝）   2 手（腕/肘）   3 腰")
        print("  v 重输版本     q 退出")
        p = _prompt("选部位: ").lower()
        if p in ("q", "quit", "exit"):
            return 0
        if p == "v":
            ver = ""
            continue
        if p in ("1", "l", "leg"):
            mod = _prompt_leg_module()
            _group_actions(ver, mod, f"腿 · {mod}")
            continue
        if p in ("2", "a", "arm"):
            mod = _prompt_arm_module()
            _group_actions(ver, mod, f"手 · {mod}")
            continue
        if p in ("3", "w", "waist"):
            _group_actions(ver, "waist", "腰")
            continue
        print("[ERROR] 请输入 1/2/3 或 v/q")


# ---------------------------------------------------------------------------
# argparse 入口
# ---------------------------------------------------------------------------

def _attach_common(p: argparse.ArgumentParser) -> None:
    p.add_argument("--module", required=True, choices=VALID_MODULES, help="要验证的机构")


def _attach_verify(p: argparse.ArgumentParser) -> None:
    p.add_argument("--num-steps", type=int, default=50)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--abs-th", type=float, default=5.0e-3)
    p.add_argument("--rmse-th", type=float, default=1.0e-3)
    p.add_argument("--rel-l2-th", type=float, default=5.0e-3)
    p.add_argument("--json", action="store_true")


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="MuJoCo 并联机构统一 CLI（spec-driven）")
    p.add_argument("-v", "--version", help="solver 版本（如 s70；交互模式下不填会提示）")
    p.add_argument("--interactive", action="store_true", help="进入交互菜单（兼容旧 UX）")

    sub = p.add_subparsers(dest="command")

    pv = sub.add_parser("viewer", help="MuJoCo viewer + PyQtGraph 曲线（solver 反解作为虚线）")
    _attach_common(pv)
    pv.add_argument("--max-points", type=int, default=800)
    pv.add_argument("--window-seconds", type=float, default=10.0)
    pv.add_argument("--sample-every-n-steps", type=int, default=5)
    pv.add_argument("--render-hz", type=float, default=20.0)
    pv.set_defaults(func=cmd_viewer)

    pj = sub.add_parser("jacobian", help="Jacobian 一致性验证（dp_solver vs dp_mj；能提供则对 (Jc,Ja) 也比较）")
    _attach_common(pj)
    _attach_verify(pj)
    pj.set_defaults(func=cmd_jacobian)

    pp = sub.add_parser("position-verify", help="位置验证：solver q↔p 往返 + MuJoCo 读回(q,p) 一致（膝/肘含闭链 sync）")
    _attach_common(pp)
    _attach_verify(pp)
    pp.set_defaults(func=cmd_position_verify)

    pc = sub.add_parser("convert-position", help="单侧位置转换：joint→motor 或 motor→joint（踝/膝/肘/腕）")
    pc.add_argument("--module", required=True, choices=SIDE_MODULES, help="支持 ankle/knee/arm_elbow/arm_wrist（不含 waist）")
    pc.add_argument("--side", required=True, choices=("L", "R", "left", "right"), help="左/右侧")
    pc.add_argument("--direction", required=True, choices=("j2m", "m2j"), help="j2m=joint→motor，m2j=motor→joint")
    pc.add_argument("--values", required=True, help="单侧输入分量，支持逗号/空格分隔")
    pc.add_argument("--json", action="store_true", help="输出 JSON（便于 skill 调用）")
    pc.set_defaults(func=cmd_convert_position)

    return p


def main(argv: Optional[List[str]] = None) -> int:
    args = build_parser().parse_args(argv)

    if args.interactive or args.command is None:
        return run_interactive(args.version)

    if not args.version:
        print("[ERROR] 非交互模式必须提供 --version/-v")
        return 2

    args.version = normalize_version(args.version)
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
