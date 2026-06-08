#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
关节轴线一致性检查（相邻轴夹角/偏差）：

- 输入：模块一 run_data_collection.py 生成的 raw_data_motion/joint_<k>/raw_poses.csv
  其中每帧包含 joint_1..7/joint_end/belly 的 px,py,pz (mm) 与 qx,qy,qz,qw。
- 对 joint_1..7：逐帧解算 joint_{k+1} 在 belly(base) 坐标系下的平移轨迹点，在该系下做 3D 圆拟合，
  得到 (center, radius, axis)；保存拟合图与结果 CSV。
- 对相邻轴：计算夹角（使用 |dot| 保证方向无关，得到 [0, 90] 度的锐角），并与理想 90 度比较。

重要：
- 这里复用 run_batch_process.py 的解算接口 solve_joints_in_frame，并强制 enable_motion_control=True（不乘关节偏移）。
- belly 刚体在配置中 joint 名为 base，因此这里使用 reference_joint="base" 来表示 belly(base) 坐标系。

输出：
<output_dir>/
  joint_1/ circle_fit.png, axis_fit_result.csv
  ...
  joint_7/ ...
  adjacent_axis_angles.csv
  axes_overview.png
"""

from __future__ import print_function

import argparse
import csv
import math
import os
import sys
from dataclasses import dataclass
from typing import List, Optional

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

_script_dir = os.path.dirname(os.path.abspath(__file__))
_parent = os.path.dirname(_script_dir)
if _parent not in sys.path:
    sys.path.insert(0, _parent)

from arm_accuracy_calibration.functions import load_config, solve_joints_in_frame
from arm_accuracy_calibration.functions.fit_rotation_axis_ls import (
    configure_matplotlib_plot_fonts,
    fit_circle_3d_ls,
    visualize_3d_circle,
)


WORKSPACE_BASE = "calibration_output"
RAW_BASE = "raw_data_motion"
DEFAULT_OUTPUT_SUBDIR = "mocap_axis_consistency"
RAW_POSES_FILENAME = "raw_poses.csv"

# 与 run_data_collection/run_batch_process 的 raw_poses.csv 列一致
_BODIES_NAMES = [
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
    "joint_7",
    "joint_end",
    "belly",
]


def _parse_float(s: str) -> float:
    s = (s or "").strip().lower()
    if s == "" or s == "nan":
        return float("nan")
    try:
        return float(s)
    except ValueError:
        return float("nan")


def _safe_mkdir(path: str) -> None:
    if path and not os.path.exists(path):
        os.makedirs(path, exist_ok=True)


def _unit(v: np.ndarray) -> np.ndarray:
    v = np.asarray(v, dtype=float).reshape(3)
    n = float(np.linalg.norm(v))
    if n < 1e-12:
        return v * 0.0
    return v / n


def _angle_deg_directionless(a: np.ndarray, b: np.ndarray) -> float:
    """返回方向无关的夹角（度）：acos(|a·b|)，范围 [0, 90]。"""
    a = _unit(a)
    b = _unit(b)
    c = float(abs(np.dot(a, b)))
    c = max(-1.0, min(1.0, c))
    return float(np.degrees(math.acos(c)))


def _row_to_rigid_body_poses_from_dict(row: dict) -> dict:
    """从 DictReader 的 row 构建 rigid_body_poses：{name: {"pos": [x,y,z], "quat": [x,y,z,w]}}，单位 mm。"""
    out = {}
    for name in _BODIES_NAMES:
        px = _parse_float(row.get(f"{name}_px", ""))
        py = _parse_float(row.get(f"{name}_py", ""))
        pz = _parse_float(row.get(f"{name}_pz", ""))
        qx = _parse_float(row.get(f"{name}_qx", ""))
        qy = _parse_float(row.get(f"{name}_qy", ""))
        qz = _parse_float(row.get(f"{name}_qz", ""))
        qw = _parse_float(row.get(f"{name}_qw", ""))
        if not (math.isfinite(px) and math.isfinite(py) and math.isfinite(pz)):
            continue
        if abs(px) < 1e-12 and abs(py) < 1e-12 and abs(pz) < 1e-12:
            continue
        out[name] = {
            "pos": np.array([px, py, pz], dtype=float),
            "quat": np.array([qx, qy, qz, qw], dtype=float),
        }
    return out


def _load_joint_kp1_in_belly_points_mm(
    csv_path: str,
    joint_k: int,
    *,
    verbose: bool,
) -> np.ndarray:
    """
    逐帧解算 joint_{k+1} in belly(base)，取其平移作为点云（mm）。
    复用 run_batch_process.py 的 solve_joints_in_frame 调用方式，并强制 enable_motion_control=True（不乘关节偏移）。
    """
    k = int(joint_k)
    if k < 1 or k > 7:
        raise ValueError(f"joint_k 超范围: {joint_k}")
    target = f"joint_{k + 1}" if k < 7 else "joint_end"

    all_config = load_config()
    ref_joint = "base"  # 配置中 belly 的 joint 名为 base

    pts: List[List[float]] = []
    with open(csv_path, "r", newline="") as f:
        reader = csv.DictReader(f)
        if not reader.fieldnames:
            raise ValueError(f"CSV 空表头: {csv_path}")
        for row in reader:
            poses = _row_to_rigid_body_poses_from_dict(row)
            # 注意：reference_joint 这里是 joint 名（"base"），不是 raw_poses.csv 的刚体键。
            # raw_poses.csv 中 belly 刚体的键为 "belly"；若 belly 或 target 缺失则该帧无法解算。
            if "belly" not in poses or target not in poses:
                continue
            result, _ = solve_joints_in_frame(
                all_config,
                poses,
                reference_joint=ref_joint,
                target_joints=[target],
                enable_motion_control=True,  # 关键：不要乘关节偏移
            )
            T = result.get(target)
            if T is None:
                continue
            t = np.asarray(T, dtype=float)[:3, 3]
            if not np.isfinite(t).all():
                continue
            pts.append([float(t[0]), float(t[1]), float(t[2])])

    if len(pts) < 3:
        if verbose:
            print(f"[run_mocap_axis_consistency][WARN] joint_{k}: 解算 {target} in belly(base) 有效点不足: {len(pts)}")
        raise ValueError(f"{csv_path} 有效点不足：{len(pts)}")
    return np.asarray(pts, dtype=float)


@dataclass
class AxisFitResult:
    joint_k: int
    center_mm: np.ndarray  # (3,)
    radius_mm: float
    axis: np.ndarray  # unit (3,)
    n_points: int
    raw_csv: str
    circle_image: str
    result_csv: str
    reference_body: str


def _fit_axis_from_joint_dir(
    joint_k: int,
    joint_dir: str,
    out_dir: str,
    max_iterations: int,
    tolerance: float,
    verbose: bool,
) -> Optional[AxisFitResult]:
    csv_path = os.path.join(joint_dir, RAW_POSES_FILENAME)
    if not os.path.isfile(csv_path):
        if verbose:
            print(f"[run_mocap_axis_consistency] 跳过 joint_{joint_k}: 未找到 {csv_path}")
        return None

    # 直接得到 joint_{k+1} in belly(base) 的平移点云（mm）
    points_mm = _load_joint_kp1_in_belly_points_mm(csv_path, int(joint_k), verbose=verbose)
    effective_ref = "belly(base)"
    target_name = f"joint_{int(joint_k) + 1}" if int(joint_k) < 7 else "joint_end"
    points_m = points_mm / 1000.0
    center_m, radius_m, axis = fit_circle_3d_ls(points_m, max_iterations=max_iterations, tolerance=tolerance)
    axis = _unit(axis)
    center_mm = np.asarray(center_m, dtype=float).reshape(3) * 1000.0
    radius_mm = float(radius_m) * 1000.0

    _safe_mkdir(out_dir)
    circle_png = os.path.join(out_dir, "circle_fit.png")
    result_csv = os.path.join(out_dir, "axis_fit_result.csv")

    # 可视化用 m（让 visualize_3d_circle 的轴标单位不撒谎）
    try:
        visualize_3d_circle(
            points=points_m,
            center=np.asarray(center_m, dtype=float).reshape(3),
            radius=float(radius_m),
            axis=axis,
            reference_axis=np.array([0.0, 0.0, 1.0], dtype=float),
            output_path=circle_png,
            reference_axis_name="z",
        )
    except Exception as e:
        if verbose:
            print(f"[run_mocap_axis_consistency] 警告: joint_{joint_k} 拟合图生成失败: {e}")

    header = [
        "joint_k",
        "body_name",
        "reference_body",
        "n_points",
        "center_x_mm",
        "center_y_mm",
        "center_z_mm",
        "radius_mm",
        "axis_x",
        "axis_y",
        "axis_z",
        "raw_csv",
        "circle_fit_png",
    ]
    row = [
        int(joint_k),
        target_name,
        effective_ref,
        int(points_mm.shape[0]),
        float(center_mm[0]),
        float(center_mm[1]),
        float(center_mm[2]),
        float(radius_mm),
        float(axis[0]),
        float(axis[1]),
        float(axis[2]),
        os.path.abspath(csv_path),
        os.path.abspath(circle_png),
    ]
    with open(result_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerow(["%.10g" % x if isinstance(x, float) else x for x in row])

    if verbose:
        print(
            f"[run_mocap_axis_consistency] joint_{joint_k}: axis=({axis[0]:.6f},{axis[1]:.6f},{axis[2]:.6f}), "
            f"center_mm=({center_mm[0]:.2f},{center_mm[1]:.2f},{center_mm[2]:.2f}), r_mm={radius_mm:.2f}, n={points_mm.shape[0]}"
        )

    return AxisFitResult(
        joint_k=joint_k,
        center_mm=center_mm,
        radius_mm=radius_mm,
        axis=axis,
        n_points=int(points_mm.shape[0]),
        raw_csv=csv_path,
        circle_image=circle_png,
        result_csv=result_csv,
        reference_body=effective_ref,
    )


def _plot_axes_overview(results: List[AxisFitResult], out_path: str) -> bool:
    if not results:
        return False
    configure_matplotlib_plot_fonts()

    fig = plt.figure(figsize=(12.5, 10.0))
    ax = fig.add_subplot(111, projection="3d")
    ref = (results[0].reference_body or "").strip() or "mocap_world"
    ax.set_title(f"Fitted joint axes in '{ref}' frame (directionless)", fontsize=14)

    centers = np.array([r.center_mm for r in results], dtype=float)
    max_range = float(np.max(np.ptp(centers, axis=0))) if centers.shape[0] >= 2 else 200.0
    if not np.isfinite(max_range) or max_range < 1e-6:
        max_range = 200.0
    axis_len = 0.35 * max_range

    for r in sorted(results, key=lambda x: x.joint_k):
        c = r.center_mm
        a = _unit(r.axis)
        # 方向无关：画双向箭头
        ax.scatter([c[0]], [c[1]], [c[2]], s=45, alpha=0.95)
        ax.quiver(c[0], c[1], c[2], a[0], a[1], a[2], length=axis_len, normalize=True, linewidth=2.2, color="g")
        ax.quiver(c[0], c[1], c[2], -a[0], -a[1], -a[2], length=axis_len, normalize=True, linewidth=2.2, color="g")
        ax.text(c[0], c[1], c[2], f"  j{r.joint_k}", fontsize=10)

    mid = np.mean(centers, axis=0)
    half = 0.6 * max_range
    ax.set_xlim(mid[0] - half, mid[0] + half)
    ax.set_ylim(mid[1] - half, mid[1] + half)
    ax.set_zlim(mid[2] - half, mid[2] + half)
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_zlabel("Z (mm)")
    ax.grid(True, alpha=0.25)
    if hasattr(ax, "set_box_aspect"):
        ax.set_box_aspect((1.0, 1.0, 1.0))

    _safe_mkdir(os.path.dirname(out_path))
    plt.tight_layout()
    plt.savefig(out_path, dpi=220, bbox_inches="tight")
    plt.close(fig)
    return True


def main() -> int:
    parser = argparse.ArgumentParser(
        description="对 joint_1..7 逐帧解算 joint_{k+1} in belly(base) 并做圆拟合，输出相邻轴夹角/偏差"
    )
    parser.add_argument("--workspace", "-w", type=str, default=WORKSPACE_BASE, help=f"工作区（默认: {WORKSPACE_BASE}）")
    parser.add_argument(
        "--raw_base",
        type=str,
        default=None,
        help="raw_data_motion 根目录（默认: <script_dir>/<workspace>/raw_data_motion）",
    )
    parser.add_argument(
        "--output_dir",
        "-o",
        type=str,
        default=None,
        help=f"输出目录（默认: <raw_base>/../{DEFAULT_OUTPUT_SUBDIR}）",
    )
    parser.add_argument("--max_iterations", type=int, default=8, help="圆拟合最大迭代次数（默认: 8）")
    parser.add_argument("--tolerance", type=float, default=1e-8, help="圆拟合收敛阈值（默认: 1e-8）")
    parser.add_argument("--quiet", "-q", action="store_true", help="少打印")
    args = parser.parse_args()

    workspace = os.path.join(_script_dir, args.workspace)
    raw_base = args.raw_base or os.path.join(workspace, RAW_BASE)
    output_dir = args.output_dir or os.path.join(workspace, DEFAULT_OUTPUT_SUBDIR)
    verbose = not args.quiet

    if verbose:
        print(f"[run_mocap_axis_consistency] raw_base: {raw_base}")
        print(f"[run_mocap_axis_consistency] output_dir: {output_dir}")
        print("[run_mocap_axis_consistency] mode: joint_{k+1} in belly(base), enable_motion_control=True (no joint offsets)")

    _safe_mkdir(output_dir)

    results: List[AxisFitResult] = []
    for k in range(1, 8):
        joint_dir = os.path.join(raw_base, f"joint_{k}")
        out_dir = os.path.join(output_dir, f"joint_{k}")
        if not os.path.isdir(joint_dir):
            if verbose:
                print(f"[run_mocap_axis_consistency] 跳过 joint_{k}: 目录不存在 {joint_dir}")
            continue
        r = _fit_axis_from_joint_dir(
            joint_k=k,
            joint_dir=joint_dir,
            out_dir=out_dir,
            max_iterations=int(args.max_iterations),
            tolerance=float(args.tolerance),
            verbose=verbose,
        )
        if r is not None:
            results.append(r)

    results = sorted(results, key=lambda x: x.joint_k)
    if len(results) < 2:
        print("[run_mocap_axis_consistency] 有效关节轴结果不足（<2），无法计算相邻夹角")
        return 2

    # 相邻轴夹角与偏差（理想：90°，以“方向无关锐角”计）
    adjacent_csv = os.path.join(output_dir, "adjacent_axis_angles.csv")
    with open(adjacent_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(
            [
                "reference_body",
                "joint_i",
                "joint_j",
                "angle_deg_directionless",
                "ideal_deg",
                "deviation_deg",
                "axis_i_x",
                "axis_i_y",
                "axis_i_z",
                "axis_j_x",
                "axis_j_y",
                "axis_j_z",
            ]
        )
        for i in range(len(results) - 1):
            a = results[i].axis
            b = results[i + 1].axis
            ang = _angle_deg_directionless(a, b)
            ideal = 90.0
            dev = ang - ideal
            w.writerow(
                [
                    (results[i].reference_body or ""),
                    results[i].joint_k,
                    results[i + 1].joint_k,
                    "%.6f" % ang,
                    "%.1f" % ideal,
                    "%.6f" % dev,
                    "%.8f" % float(a[0]),
                    "%.8f" % float(a[1]),
                    "%.8f" % float(a[2]),
                    "%.8f" % float(b[0]),
                    "%.8f" % float(b[1]),
                    "%.8f" % float(b[2]),
                ]
            )

    if verbose:
        print(f"[run_mocap_axis_consistency] 已保存相邻夹角汇总: {adjacent_csv}")

    overview_png = os.path.join(output_dir, "axes_overview.png")
    ok = _plot_axes_overview(results, overview_png)
    if ok and verbose:
        print(f"[run_mocap_axis_consistency] 已保存总览图: {overview_png}")

    if verbose:
        print("[run_mocap_axis_consistency] 完成")
    return 0


if __name__ == "__main__":
    sys_exit = 0
    try:
        sys_exit = main()
    except Exception as e:
        print(f"[run_mocap_axis_consistency] 失败: {e}")
        sys_exit = 1
    raise SystemExit(sys_exit)

