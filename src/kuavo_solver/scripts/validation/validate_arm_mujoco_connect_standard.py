#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
手臂仿真验证（position-only）

思路：
1) 用 kuavo_solver_py.ParallerLinearArmSolver 给定 joint14 -> motor14；
2) 在 MuJoCo 中把对应 joint/motor(线性滑块) qpos 设置进去；
3) 直接读取 MuJoCo equality(connect) 中使用到的 connect anchor 点（eq_data）
   计算两点世界坐标差的范数，作为“闭环几何误差”指标；
4) 验证 elbow 子机构用 biped_elbows_mujoco.xml，wrist 子机构用 biped_wrists_mujoco.xml。

为什么不使用 site：wrist 的 connect anchor 点不一定等于你以为的 site 点（site 只是一种几何采样）。
直接读取 eq_data 能严格对齐 MJCF 里真正约束的连接点。
"""

from __future__ import annotations

import argparse
import os
import sys
from typing import Any, Dict, List, Optional, Tuple

import mujoco
import numpy as np


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
WORKSPACE_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "../../.."))


def _setup_solver_py() -> None:
    # 兼容你当前工作区的两种常见构建输出目录：
    # 1) /root/workspace/kuavo-ros-control/build/kuavo_solver/python
    # 2) /root/workspace/build_kuavo_solver/python（你上次 arm CLI 似乎就是用它）
    candidates = [
        os.path.join(WORKSPACE_ROOT, "build/kuavo_solver/python"),
        os.path.join(WORKSPACE_ROOT, "build_kuavo_solver/python"),
        os.path.join(os.path.dirname(WORKSPACE_ROOT), "build_kuavo_solver/python"),
    ]
    for solver_py_dir in candidates:
        if os.path.isdir(solver_py_dir) and solver_py_dir not in sys.path:
            sys.path.insert(0, solver_py_dir)


def _setup_sys_path_for_scripts() -> None:
    # 让 `scripts/` 根与 `lib` 可被 import（solver_selection 等）
    scripts_root = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
    lib_dir = os.path.join(scripts_root, "lib")
    for _p in (scripts_root, lib_dir):
        if _p not in sys.path:
            sys.path.insert(0, _p)


def _joint_range(model: "mujoco.MjModel", joint_name: str) -> Tuple[float, float]:
    try:
        j = model.joint(joint_name)
    except Exception:
        # 该关节在最小化子机构 MJCF 里可能不存在；返回一个保底范围（但采样仍主要来自 wrist 模型）。
        return -1.0, 1.0
    r = np.array(j.range, dtype=float).reshape((-1,))
    if r.size >= 2:
        return float(r[0]), float(r[1])
    return -1.0, 1.0


def _set_joint_qpos(model: "mujoco.MjModel", data: "mujoco.MjData", joint_name: str, value: float) -> None:
    try:
        j = model.joint(joint_name)
    except Exception:
        # 最小化子机构可能不存在某些 joint/slider，直接跳过即可
        return
    if getattr(j, "qposadr", None) is None:
        return
    if j.qposadr.size > 0:
        data.qpos[j.qposadr[0]] = float(value)


def _body_world_point(data: "mujoco.MjData", body_id: int, local_pt: np.ndarray) -> np.ndarray:
    # d.xmat: 9 元素 row-major，reshape(3,3) 后做局部->世界旋转
    x = data.xpos[body_id]
    R = data.xmat[body_id].reshape(3, 3)
    return x + R.dot(local_pt)


def _connect_anchor_errors(model: "mujoco.MjModel", data: "mujoco.MjData") -> List[float]:
    # 只统计 mjEQ_CONNECT 类型
    errs: List[float] = []
    for i in range(model.neq):
        if model.eq_type[i] != mujoco.mjtEq.mjEQ_CONNECT.value:
            continue
        b1 = int(model.eq_obj1id[i])
        b2 = int(model.eq_obj2id[i])
        pt1 = np.array(model.eq_data[i, 0:3], dtype=float)
        pt2 = np.array(model.eq_data[i, 3:6], dtype=float)
        w1 = _body_world_point(data, b1, pt1)
        w2 = _body_world_point(data, b2, pt2)
        errs.append(float(np.linalg.norm(w1 - w2)))
    return errs


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Validate arm elbow/wrist closed-loop geometry")
    p.add_argument("--version", required=True, help="外部版本字符串，例如 s70")
    p.add_argument("--num-samples", "-n", type=int, default=20)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--abs-th", type=float, default=5.0e-3, help="connect anchor 最大误差阈值（m）")
    p.add_argument(
        "--use-split-xml",
        action="store_true",
        help="启用闭环子模型约束收敛步进，并将默认 elbow/wrist 路径切到 mjcf/biped_*.xml（与旧 *_mujoco.xml 二选一）",
    )
    p.add_argument(
        "--skip-elbow",
        action="store_true",
        help="仅跳过肘部 connect 验证（腕部仍验证）",
    )
    p.add_argument(
        "--skip-wrist",
        action="store_true",
        help="仅跳过腕部 connect 验证（肘部仍验证）",
    )
    p.add_argument(
        "--elbow-xml",
        default=os.path.join(
            WORKSPACE_ROOT,
            "src/kuavo_solver/robot-descriptions/arms/biped_arms_7gen/assets/xml/biped_elbows_mujoco.xml",
        ),
    )
    p.add_argument(
        "--wrist-xml",
        default=os.path.join(
            WORKSPACE_ROOT,
            "src/kuavo_solver/robot-descriptions/arms/biped_arms_7gen/assets/xml/biped_wrists_mujoco.xml",
        ),
    )
    return p


def main() -> int:
    _setup_sys_path_for_scripts()
    _setup_solver_py()
    args = build_arg_parser().parse_args()
    return run(args)


def _resolve_verify_flags(args: Any) -> Tuple[bool, bool]:
    """肘/腕是否参与验证：arm CLI 用 no_verify_wrist + verify_elbow；否则可用 verify_elbow+verify_wrist 或 skip_*。"""
    if hasattr(args, "no_verify_wrist"):
        return bool(getattr(args, "verify_elbow", False)), not bool(getattr(args, "no_verify_wrist", False))
    ve = getattr(args, "verify_elbow", None)
    vw = getattr(args, "verify_wrist", None)
    if ve is not None and vw is not None:
        return bool(ve), bool(vw)
    skip_elbow = bool(getattr(args, "skip_elbow", False))
    skip_wrist = bool(getattr(args, "skip_wrist", False))
    return (not skip_elbow), (not skip_wrist)


def run(args: Any) -> int:
    """
    供统一 CLI 直接调用（避免 main() 里二次 argparse 解析）。
    """
    report = compute_report(args)
    _print_report(report)
    return 0 if report["pass"] else 1


def compute_report(args: Any) -> Dict[str, Any]:
    """
    计算并返回 report（不做 JSON 输出）。
    供 arm CLI 复用。
    """
    _setup_sys_path_for_scripts()
    _setup_solver_py()
    from solver_selection import load_default_index

    idx = load_default_index()
    sel = idx.resolve(args.version, module="arm")

    verify_elbow, verify_wrist = _resolve_verify_flags(args)
    if not verify_elbow and not verify_wrist:
        return {"pass": False, "error": "verify_elbow 与 verify_wrist 不能同时为 False"}

    # 初始化 C++ solver（真实实现）
    import kuavo_solver_py  # type: ignore
    solver = kuavo_solver_py.ParallelLinearArmSolver(sel.token, sel.config_dir)

    # 加载 MJCF（按需；采样 q14 仍需要两侧关节范围时可从已加载模型读取）
    elbow_xml = args.elbow_xml
    wrist_xml = args.wrist_xml
    if getattr(args, "use_split_xml", False):
        elbow_xml = os.path.join(
            WORKSPACE_ROOT,
            "src/kuavo_solver/robot-descriptions/arms/biped_arms_7gen/mjcf/biped_elbows.xml",
        )
        wrist_xml = os.path.join(
            WORKSPACE_ROOT,
            "src/kuavo_solver/robot-descriptions/arms/biped_arms_7gen/mjcf/biped_wrists.xml",
        )
    m_elbow = None
    d_elbow = None
    m_wrist = None
    d_wrist = None
    if verify_elbow:
        m_elbow = mujoco.MjModel.from_xml_path(elbow_xml)
        d_elbow = mujoco.MjData(m_elbow)
    if verify_wrist:
        m_wrist = mujoco.MjModel.from_xml_path(wrist_xml)
        d_wrist = mujoco.MjData(m_wrist)

    def _range_for(joint_name: str) -> Tuple[float, float]:
        # 腕关节优先 wrist 子模型；前臂优先 elbow 子模型；否则任一侧已加载模型
        order: List[Optional["mujoco.MjModel"]]
        if "hand" in joint_name:
            order = [m_wrist, m_elbow]
        elif "foream" in joint_name:
            order = [m_elbow, m_wrist]
        else:
            order = [m_wrist, m_elbow]
        for m in order:
            if m is None:
                continue
            lo, hi = _joint_range(m, joint_name)
            return lo, hi
        return -1.0, 1.0

    def _should_settle(path: str) -> bool:
        return os.path.basename(path) in ("biped_elbows.xml", "biped_wrists.xml")

    rng = np.random.default_rng(args.seed)

    # joint14 索引约定（和 solver/arm_solver_stub 保持一致）
    # 每边 7D：elbow idx=3，wrist_roll/pitch idx=5/6
    # 双臂拼接：左 [0:7)，右 [7:14)；右 elbow idx=10，wrist_roll/pitch=12/13
    Q_IDX_ELBOW_L = 3
    Q_IDX_WRIST_RP_L = 5
    Q_IDX_WRIST_PP_L = 6
    Q_IDX_ELBOW_R = 10
    Q_IDX_WRIST_RP_R = 12
    Q_IDX_WRIST_PP_R = 13

    # 注意：joint_map/motor_joint_map 这类“提前访问 joint”的静态映射
    # 在最小化子机构 MJCF 下可能访问不到关节，因此这里不再构造。

    # 采样范围（从已加载子模型的 MJCF joint range 读取）
    ranges = {
        "L_foream_joint": _range_for("L_foream_joint"),
        "R_foream_joint": _range_for("R_foream_joint"),
        "L_hand_roll_joint": _range_for("L_hand_roll_joint"),
        "R_hand_roll_joint": _range_for("R_hand_roll_joint"),
        "L_hand_pitch_joint": _range_for("L_hand_pitch_joint"),
        "R_hand_pitch_joint": _range_for("R_hand_pitch_joint"),
    }

    samples: List[Dict[str, Any]] = []
    elbow_pass = not verify_elbow
    wrist_pass = not verify_wrist

    for si in range(args.num_samples):
        q14 = np.zeros(14, dtype=float)
        # elbow
        q14[Q_IDX_ELBOW_L] = rng.uniform(*ranges["L_foream_joint"])
        q14[Q_IDX_ELBOW_R] = rng.uniform(*ranges["R_foream_joint"])
        # wrist roll/pitch
        q14[Q_IDX_WRIST_RP_L] = rng.uniform(*ranges["L_hand_roll_joint"])
        q14[Q_IDX_WRIST_PP_L] = rng.uniform(*ranges["L_hand_pitch_joint"])
        q14[Q_IDX_WRIST_RP_R] = rng.uniform(*ranges["R_hand_roll_joint"])
        q14[Q_IDX_WRIST_PP_R] = rng.uniform(*ranges["R_hand_pitch_joint"])

        p14 = np.array(solver.joint_to_motor_position(q14), dtype=float).reshape((14,))

        elbow_max = 0.0
        elbow_errs: List[float] = []
        if verify_elbow and m_elbow is not None and d_elbow is not None:
            d_elbow.qpos[:] = 0.0
            for jname, idx in [
                ("L_foream_joint", Q_IDX_ELBOW_L),
                ("L_hand_roll_joint", Q_IDX_WRIST_RP_L),
                ("L_hand_pitch_joint", Q_IDX_WRIST_PP_L),
                ("R_foream_joint", Q_IDX_ELBOW_R),
                ("R_hand_roll_joint", Q_IDX_WRIST_RP_R),
                ("R_hand_pitch_joint", Q_IDX_WRIST_PP_R),
            ]:
                _set_joint_qpos(m_elbow, d_elbow, jname, float(q14[idx]))
            for jname, midx in [
                ("L_foream_Prismatic_joint", 3),
                ("L_Hand_Prismatic_joint_A", 5),
                ("L_Hand_Prismatic_joint_B", 6),
                ("R_foream_Prismatic_joint", 10),
                ("R_Hand_Prismatic_joint_A", 12),
                ("R_Hand_Prismatic_joint_B", 13),
            ]:
                _set_joint_qpos(m_elbow, d_elbow, jname, float(p14[midx]))

            mujoco.mj_forward(m_elbow, d_elbow)
            if getattr(args, "use_split_xml", False) or _should_settle(elbow_xml):
                d_elbow.ctrl[:] = 0.0
                for _ in range(15):
                    mujoco.mj_step(m_elbow, d_elbow)
            elbow_errs = _connect_anchor_errors(m_elbow, d_elbow)
            elbow_max = max(elbow_errs) if elbow_errs else 0.0
            if elbow_max > args.abs_th:
                elbow_pass = False

        wrist_max = 0.0
        wrist_errs: List[float] = []
        if verify_wrist and m_wrist is not None and d_wrist is not None:
            d_wrist.qpos[:] = 0.0
            for jname, idx in [
                ("L_foream_joint", Q_IDX_ELBOW_L),
                ("L_hand_roll_joint", Q_IDX_WRIST_RP_L),
                ("L_hand_pitch_joint", Q_IDX_WRIST_PP_L),
                ("R_foream_joint", Q_IDX_ELBOW_R),
                ("R_hand_roll_joint", Q_IDX_WRIST_RP_R),
                ("R_hand_pitch_joint", Q_IDX_WRIST_PP_R),
            ]:
                _set_joint_qpos(m_wrist, d_wrist, jname, float(q14[idx]))
            for jname, midx in [
                ("L_foream_Prismatic_joint", 3),
                ("L_Hand_Prismatic_joint_A", 5),
                ("L_Hand_Prismatic_joint_B", 6),
                ("R_foream_Prismatic_joint", 10),
                ("R_Hand_Prismatic_joint_A", 12),
                ("R_Hand_Prismatic_joint_B", 13),
            ]:
                _set_joint_qpos(m_wrist, d_wrist, jname, float(p14[midx]))

            mujoco.mj_forward(m_wrist, d_wrist)
            if getattr(args, "use_split_xml", False) or _should_settle(wrist_xml):
                d_wrist.ctrl[:] = 0.0
                for _ in range(15):
                    mujoco.mj_step(m_wrist, d_wrist)
            wrist_errs = _connect_anchor_errors(m_wrist, d_wrist)
            wrist_max = max(wrist_errs) if wrist_errs else 0.0
            if wrist_max > args.abs_th:
                wrist_pass = False

        row: Dict[str, Any] = {
            "sample": si,
            "q14": [float(x) for x in q14],
            "p14": [float(x) for x in p14],
        }
        if verify_elbow:
            row["elbow"] = {"max_dist": float(elbow_max), "all": [float(x) for x in elbow_errs]}
        if verify_wrist:
            row["wrist"] = {"max_dist": float(wrist_max), "all": [float(x) for x in wrist_errs]}
        samples.append(row)

    report = {
        "version": args.version,
        "arm_token": sel.token,
        "arm_config_dir": sel.config_dir,
        "abs_th": args.abs_th,
        "verify_elbow": verify_elbow,
        "verify_wrist": verify_wrist,
        "elbow_pass": elbow_pass,
        "wrist_pass": wrist_pass,
        "pass": elbow_pass and wrist_pass,
        "num_samples": args.num_samples,
        "seed": args.seed,
        "samples": samples,
        "models": {"elbow_xml": elbow_xml, "wrist_xml": wrist_xml},
    }
    return report


def _print_report(report: Dict[str, Any]) -> None:
    if not report.get("pass", False) and report.get("error"):
        print(f"[ERROR] {report['error']}")
        return

    ver = report.get("version", "")
    token = report.get("arm_token", "")
    th_abs = float(report.get("abs_th", 0.0))
    num = int(report.get("num_samples", 0))
    ve = bool(report.get("verify_elbow", False))
    vw = bool(report.get("verify_wrist", False))

    parts: list[str] = []
    if vw:
        parts.append("wrist")
    if ve:
        parts.append("elbow")
    parts_s = "+".join(parts) if parts else "none"

    # 每个 sample 对每个启用部件给一个 max_dist；我们按“部件”为 item 统计通过率
    total_items = num * max(len(parts), 1)
    ok_items = 0
    worst = 0.0
    for row in report.get("samples", []):
        if vw and "wrist" in row:
            e = float(row["wrist"]["max_dist"])
            worst = max(worst, e)
            ok_items += 1 if e <= th_abs else 0
        if ve and "elbow" in row:
            e = float(row["elbow"]["max_dist"])
            worst = max(worst, e)
            ok_items += 1 if e <= th_abs else 0

    # ankle 风格输出
    print(f"\n=== Arm connect 验证: {token} ({parts_s}) ===")
    print(f"  version : {ver}")
    print(f"  samples : {num} (parts={len(parts)}), total={total_items}")
    print(f"  th_abs  : {th_abs:.3e}")
    if total_items > 0:
        print(f"  pass    : {ok_items}/{total_items} ({(ok_items/total_items*100.0):.1f}%)")
    print(f"  worst   : {worst:.3e}")


if __name__ == "__main__":
    raise SystemExit(main())

