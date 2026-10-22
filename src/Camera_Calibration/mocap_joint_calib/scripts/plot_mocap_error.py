#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
纯动捕标定 优化前后误差绘图（参考棋盘标定 output 布局）。

布局（每张图对应棋盘标定的 board_pose_bars_vs_urdf.png）：
  - 大图：2 行 × 3 列
      上排: p_x, p_y, p_z   (位置分量, mm)
      下排: roll, pitch, yaw (姿态分量, deg)
  - 每张子图 = 所有有效采样姿态的分量柱状图
  - 左右手分别输出 pre、post 和 summary，共 6 张图：
      mocap_pose_pre_<body>.png      优化前 FK预测 vs 动捕实测
      mocap_pose_post_<body>.png     优化后 FK预测 vs 动捕实测
      mocap_pose_summary_<body>.png  位置/姿态范数的 pre/post 双柱

用法：
    python3 plot_mocap_error.py --capture capture_xxx.json --layout auto
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

_SCRIPT_DIR = Path(__file__).resolve().parent
_MODULE_DIR = _SCRIPT_DIR.parent  # mocap_joint_calib/
_CAMERA_CAL_DIR = _MODULE_DIR.parent
_MOCAP_DIR = _CAMERA_CAL_DIR / "mocap_checkerboard_pose"
for _p in (str(_CAMERA_CAL_DIR), str(_MOCAP_DIR)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from plot_board_error_from_csv import load_urdf_joints, fk_root_to_tip_transform  # noqa: E402
from mocap_pose_utils import load_mocap_frame_align_matrix, quat_xyzw_to_R  # noqa: E402

FREE_JOINTS = [
    "zarm_l1_joint", "zarm_l2_joint", "zarm_l3_joint", "zarm_l4_joint",
    "zarm_l5_joint", "zarm_l6_joint", "zarm_l7_joint",
    "zarm_r1_joint", "zarm_r2_joint", "zarm_r3_joint", "zarm_r4_joint",
    "zarm_r5_joint", "zarm_r6_joint", "zarm_r7_joint",
]
COMPONENTS = ["x", "y", "z", "roll", "pitch", "yaw"]
LAYOUT_DEFAULTS = {
    "biped45": {
        "urdf": "biped_v3_arm_s45.urdf",
        "fk_root": "base_link",
        "left_arm": [12, 19],
        "right_arm": [19, 26],
    },
    "biped52": {
        "urdf": "biped_v3_arm.urdf",
        "fk_root": "waist_yaw_link",
        "left_arm": [13, 20],
        "right_arm": [20, 27],
    },
    "biped56": {
        "urdf": "biped_v3_arm_s56.urdf",
        "fk_root": "waist_yaw_link",
        "left_arm": [13, 20],
        "right_arm": [20, 27],
    },
    "wheel62": {
        "urdf": "biped_v3_arm_s62.urdf",
        "fk_root": "waist_yaw_link",
        "left_arm": [4, 11],
        "right_arm": [11, 18],
    },
}
VERSION_LAYOUT = {
    "45": "biped45",
    "52": "biped52",
    "56": "biped56",
    "62": "wheel62",
    "63": "wheel62",
}
URDF_LAYOUT = {
    cfg["urdf"]: layout for layout, cfg in LAYOUT_DEFAULTS.items()
}


def _joint_q_indices(joint_indices: dict, layout: str) -> dict:
    """读取 capture 索引；旧数据缺少索引时按已确认的机型布局回退。"""
    ji = joint_indices if isinstance(joint_indices, dict) else {}
    defaults = LAYOUT_DEFAULTS[layout]
    out = {}
    if "waist" in ji:
        out["waist_yaw_joint"] = int(ji["waist"])
    lo, _ = ji.get("left_arm", defaults["left_arm"])
    for k, name in enumerate(FREE_JOINTS[:7]):
        out[name] = int(lo) + k
    lo, _ = ji.get("right_arm", defaults["right_arm"])
    for k, name in enumerate(FREE_JOINTS[7:]):
        out[name] = int(lo) + k
    return out


def _q_reported(q_full, idx_map: dict) -> dict:
    n = len(q_full)
    out = {}
    for name, idx in idx_map.items():
        actual = idx if idx >= 0 else n + idx
        if not 0 <= actual < n:
            raise ValueError(
                f"joint_q 索引越界: {name}={idx}, joint_q 长度={n}"
            )
        out[name] = float(q_full[actual])
    return out


def _infer_capture_layout(meta: dict) -> str:
    """从 capture 的多项元数据交叉推断布局；矛盾时拒绝继续。"""
    candidates = []
    layout = str(meta.get("robot_layout") or "")
    if layout:
        if layout not in LAYOUT_DEFAULTS:
            raise ValueError(f"capture robot_layout 不支持: {layout}")
        candidates.append(("robot_layout", layout))

    version = str(meta.get("robot_version") or "")
    if version:
        if version not in VERSION_LAYOUT:
            raise ValueError(f"capture robot_version 不支持: {version}")
        candidates.append(("robot_version", VERSION_LAYOUT[version]))

    urdf_name = Path(str(meta.get("urdf") or "")).name
    if urdf_name in URDF_LAYOUT:
        candidates.append(("urdf", URDF_LAYOUT[urdf_name]))

    layouts = {value for _, value in candidates}
    if len(layouts) > 1:
        details = ", ".join(f"{source}={value}" for source, value in candidates)
        raise ValueError(f"capture 机型元数据相互矛盾: {details}")
    return next(iter(layouts), "")


def _resolve_model(capture: dict, requested_layout: str):
    meta = capture.get("meta")
    if not isinstance(meta, dict):
        raise ValueError("capture 缺少对象类型的 meta")

    inferred_layout = _infer_capture_layout(meta)
    if requested_layout == "auto":
        if not inferred_layout:
            raise ValueError(
                "无法从 capture 推断机型，请显式传 "
                "--layout biped45|biped52|biped56|wheel62"
            )
        layout = inferred_layout
    else:
        layout = requested_layout
        if inferred_layout and inferred_layout != layout:
            raise ValueError(
                f"--layout={layout} 与 capture 推断机型 {inferred_layout} 不一致"
            )

    defaults = LAYOUT_DEFAULTS[layout]
    fk_root = str(meta.get("fk_root") or defaults["fk_root"])
    if fk_root != defaults["fk_root"]:
        raise ValueError(
            f"capture fk_root={fk_root} 与 {layout} 期望的 "
            f"{defaults['fk_root']} 不一致"
        )

    meta_urdf_name = Path(str(meta.get("urdf") or "")).name
    urdf_name = meta_urdf_name or defaults["urdf"]
    urdf_path = _CAMERA_CAL_DIR / urdf_name
    if not urdf_path.is_file():
        urdf_path = _CAMERA_CAL_DIR / defaults["urdf"]
    if not urdf_path.is_file():
        raise FileNotFoundError(f"找不到 {layout} 的 URDF: {urdf_path}")
    return layout, urdf_path, fk_root


def _load_biases(path: Path) -> dict:
    import yaml

    if not path.is_file():
        raise FileNotFoundError(f"calibration.yaml 不存在: {path}")
    with open(path, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError(f"calibration.yaml 顶层不是字典: {path}")

    missing = [joint for joint in FREE_JOINTS if joint not in data]
    if missing:
        raise ValueError(f"calibration.yaml 缺少手臂关节: {missing}")
    biases = {}
    for joint in FREE_JOINTS:
        value = float(data[joint])
        if not math.isfinite(value):
            raise ValueError(f"calibration.yaml 中 {joint} 不是有限数值")
        biases[joint] = value
    return biases


def _mat_to_rpy_deg(R):
    """旋转矩阵 → roll/pitch/yaw (deg)。"""
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    if sy > 1e-8:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0.0
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def compute_components(
    capture: dict,
    urdf_joints,
    fk_root: str,
    layout: str,
    bias: dict,
):
    """对每样本每刚体，算 FK 预测 vs 动捕实测 的 x/y/z/roll/pitch/yaw 分量差。

    返回: [{sample, body, x, y, z, roll, pitch, yaw}]（位置 mm，姿态 deg）
    """
    idx_map = _joint_q_indices(
        capture["meta"].get("joint_q_indices", {}), layout
    )
    align_cfg = capture["meta"].get("mocap_frame_align", {"enabled": False})
    R_align = load_mocap_frame_align_matrix(align_cfg)

    results = []
    for s in capture.get("samples", []):
        q_full = s.get("joint_q_full")
        if not q_full:
            continue
        qrep = _q_reported(q_full, idx_map)
        for body in capture.get("bodies", []):
            name = body["name"]
            if name == "torso":
                continue
            key = f"{name}_in_torso"
            obs = s.get("bodies", {}).get(key)
            if obs is None:
                continue
            tip = body.get("tip_link")
            q_used = dict(qrep)
            for j, b in bias.items():
                if j in q_used:
                    q_used[j] = qrep[j] + b
            T_fk = fk_root_to_tip_transform(urdf_joints, fk_root, tip, q_used)

            p_obs = R_align @ (np.array(obs["xyz_mm"]) / 1000.0)
            R_obs = R_align @ quat_xyzw_to_R(np.array(obs["quaternion_xyzw"]))

            # 位置差 (mm)
            dpos = (T_fk[:3, 3] - p_obs) * 1000.0
            # 姿态差：R_err = R_obs⁻¹ · R_fk → rpy
            R_err = R_obs.T @ T_fk[:3, :3]
            dr, dp, dy = _mat_to_rpy_deg(R_err)

            results.append({
                "sample": s.get("index", 0),
                "body": name,
                "x": dpos[0], "y": dpos[1], "z": dpos[2],
                "roll": dr, "pitch": dp, "yaw": dy,
            })
    return results


def _draw_pose_fig(results, body, title, out_png):
    """画单刚体的 2x3 大图：上排 x/y/z，下排 roll/pitch/yaw。"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    sids = sorted(set(r["sample"] for r in results if r["body"] == body))
    x = np.arange(len(sids))
    color = "#C73E1D" if body == "l_hand" else "#1B998B"

    fig, axes = plt.subplots(2, 3, figsize=(11.5, 7.2), constrained_layout=True)
    fig.suptitle(title, fontsize=13, fontweight="600")

    for col, comp in enumerate(COMPONENTS):
        ax = axes[col // 3, col % 3]
        ordered = []
        for sid in sids:
            hits = [r[comp] for r in results if r["body"] == body and r["sample"] == sid]
            ordered.append(hits[0] if hits else float("nan"))
        ax.bar(x, ordered, width=0.6, color=color, edgecolor="0.15", linewidth=0.5)
        ax.set_xticks(x)
        ax.set_xticklabels([f"S{s}" for s in sids], fontsize=9)
        ax.axhline(0, color="0.65", linewidth=0.6)
        ax.grid(axis="y", alpha=0.35)
        unit = "(mm)" if comp in ("x", "y", "z") else "(deg)"
        ax.set_title(f"{comp} {unit}", fontsize=11)
    fig.savefig(out_png, dpi=160)
    plt.close(fig)
    return out_png


def _draw_summary_fig(pre, post, out_png):
    """汇总：位置norm(mm) + 姿态norm(deg) 的 pre/post 双柱。"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    bodies = sorted(set(r["body"] for r in pre))
    sids = sorted(set(r["sample"] for r in pre))
    x = np.arange(len(sids))

    def _norm(results, sid, body, comps):
        hits = [r for r in results if r["sample"] == sid and r["body"] == body]
        if not hits:
            return float("nan")
        r = hits[0]
        return math.sqrt(sum(r[c] ** 2 for c in comps))

    # 每只手一张 summary 图：左=位置norm(mm), 右=姿态norm(deg)，pre/post 双柱
    out_paths = []
    for body in bodies:
        color_pre = "#C73E1D"
        color_post = "#1B998B"
        body_png = out_png.with_name(f"{out_png.stem}_{body}{out_png.suffix}")
        fig, axes = plt.subplots(1, 2, figsize=(10.5, 4.8), constrained_layout=True)
        fig.suptitle(f"Mocap calibration error summary: {body} (pre vs post)",
                     fontsize=12, fontweight="600")

        for col, (comps, ylabel, title) in enumerate([
            (["x", "y", "z"], "position norm (mm)", "position error"),
            (["roll", "pitch", "yaw"], "rotation norm (deg)", "rotation error"),
        ]):
            ax = axes[col]
            pre_vals = np.array([_norm(pre, sid, body, comps) for sid in sids])
            post_vals = np.array([_norm(post, sid, body, comps) for sid in sids])
            ax.bar(x - 0.2, pre_vals, width=0.4, color=color_pre,
                   edgecolor="0.15", label="pre")
            ax.bar(x + 0.2, post_vals, width=0.4, color=color_post,
                   edgecolor="0.15", label="post")
            ax.set_xticks(x)
            ax.set_xticklabels([f"S{s}" for s in sids], fontsize=9)
            ax.set_ylabel(ylabel)
            ax.set_title(title)
            ax.legend(fontsize=8)
            ax.grid(axis="y", alpha=0.35)

        fig.savefig(body_png, dpi=160)
        plt.close(fig)
        out_paths.append(body_png)
    return out_paths


def main() -> int:
    ap = argparse.ArgumentParser(description="动捕标定误差绘图（参考棋盘标定 output 布局）")
    ap.add_argument("--capture", required=True, type=Path, help="capture_*.json")
    ap.add_argument(
        "--layout",
        choices=["auto", *LAYOUT_DEFAULTS],
        default="auto",
        help="机器人布局；默认从 capture 元数据自动识别",
    )
    ap.add_argument("--calibration_yaml", default=None, type=Path, help="calibration.yaml")
    ap.add_argument("--output", default=None, type=Path, help="输出目录")
    args = ap.parse_args()

    if not args.capture.is_file():
        raise FileNotFoundError(f"capture 不存在: {args.capture}")
    capture = json.loads(args.capture.read_text(encoding="utf-8"))
    layout, urdf_path, fk_root = _resolve_model(capture, args.layout)
    urdf_joints = load_urdf_joints(urdf_path)

    if args.calibration_yaml:
        yaml_path = args.calibration_yaml
    else:
        yaml_path = _MODULE_DIR / "output" / "calibration.yaml"
    bias = _load_biases(yaml_path)

    out_dir = args.output or (_MODULE_DIR / "output")
    out_dir.mkdir(parents=True, exist_ok=True)

    pre = compute_components(capture, urdf_joints, fk_root, layout, {})
    post = compute_components(capture, urdf_joints, fk_root, layout, bias)
    if not pre:
        raise ValueError("capture 中没有可绘制的 hand_in_torso 有效样本")
    bodies = sorted(set(r["body"] for r in pre))

    # 每只手 × pre/post 各一张 2x3 大图（共 4 张），左右手分开
    out_pngs = []
    for body in bodies:
        out_pngs.append(_draw_pose_fig(
            pre, body,
            f"Mocap: FK vs Mocap, PRE (bias=0) - {body}",
            out_dir / f"mocap_pose_pre_{body}.png"))
        out_pngs.append(_draw_pose_fig(
            post, body,
            f"Mocap: FK vs Mocap, POST (bias=solved) - {body}",
            out_dir / f"mocap_pose_post_{body}.png"))

    # 汇总：每只手一张（位置 + 姿态两子图）
    out_pngs.extend(_draw_summary_fig(
        pre, post, out_dir / "mocap_pose_summary.png"
    ))

    # 终端汇总
    print(f"[plot] layout={layout}, urdf={urdf_path}, fk_root={fk_root}")
    print(f"== 动捕标定误差 (FK vs Mocap, 相对 {fk_root}) ==")
    print("== 位置(mm) 与 姿态(deg) 的 pre/post 平均绝对误差 ==")
    print(f"{'body':<10}{'metric':<8}{'pre':>10}{'post':>10}")
    for body in bodies:
        for comp, _unit in [("x", "mm"), ("y", "mm"), ("z", "mm"),
                            ("roll", "deg"), ("pitch", "deg"), ("yaw", "deg")]:
            p_vals = [r[comp] for r in pre if r["body"] == body]
            po_vals = [r[comp] for r in post if r["body"] == body]
            print(
                f"{body:<10}{comp:<8}"
                f"{np.mean(np.abs(p_vals)):>10.3f}"
                f"{np.mean(np.abs(po_vals)):>10.3f}"
            )
    for p in out_pngs:
        print(f"[plot] 已输出: {p}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
