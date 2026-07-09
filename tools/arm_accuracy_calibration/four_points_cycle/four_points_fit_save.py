#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
读取第二个脚本（four_points_to_joint1.py）的输出：output/joint1/cycle_*/point_*_in_joint1.csv。
对每个 (cycle, point) 的多行数据：
- 用现有最小二乘拟合把每个 joint(2..7) 的 T 拟合成一个矩阵并保存；
- 从 output/cycles/cycle_*/data.csv 按点取传感器角度，对每列求均值作为拟合值并保存。
结果输出到独立目录（默认 output/joint1_fitted/）。

可选：
- 通过 --plot-joint 指定某个 joint（2..7），对所有 cycle 上该 joint→joint_1
  的拟合平移量进行汇总绘图：每个点输出一张 1×4 子图的 PNG，
  第 1 张为 XYZ 三维位置散点图，第 2～4 张分别为 X/Y/Z 的柱状图。
"""
from __future__ import print_function

# 无头环境（如 SSH）下避免依赖库尝试连接 X11 显示服务器
import os
os.environ.setdefault("MPLBACKEND", "Agg")
os.environ.setdefault("GTK_BACKEND", "minimal")

import argparse
import csv
import math
import os
import re
import sys

_script_dir = os.path.dirname(os.path.abspath(__file__))
_parent = os.path.dirname(_script_dir)
_pkg_parent = os.path.dirname(_parent)
if _pkg_parent not in sys.path:
    sys.path.insert(0, _pkg_parent)

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

import rospy
from kuavo_msgs.srv import fkSrvWithReferFrame, fkSrvWithReferFrameRequest

from arm_accuracy_calibration.functions.fit_matrix_ls import (
    fit_transform_matrix_ls,
    filter_matrices_by_x_position,
)

JOINTS = (2, 3, 4, 5, 6, 7)
POINT_IDXS = (0, 1, 2, 3)
SENSOR_COLS = ["left_arm_1", "left_arm_2", "left_arm_3", "left_arm_4", "left_arm_5", "left_arm_6", "left_arm_7"]

# FK 相关：默认基坐标系、各关节末端坐标系名称与 URDF 路径
FK_BASE_FRAME = "zarm_l1_ref_link"
FK_END_FRAMES = {
    2: "zarm_l2_link",
    3: "zarm_l3_link",
    4: "zarm_l4_link",
    5: "zarm_l5_link",
    6: "zarm_l6_link",
    7: "zarm_l7_link",
}
# 使用用户指定的 URDF：tools/arm_accuracy_calibration/biped_v3_arm.urdf
FK_URDF_PATH = os.path.join(_parent, "biped_v3_arm.urdf")


def _parse_float(s):
    s = (s or "").strip().lower()
    if s == "nan" or s == "":
        return math.nan
    try:
        return float(s)
    except ValueError:
        return math.nan


def _load_point_joint1_csv(path):
    """
    读取 point_P_in_joint1.csv，返回 (header, list of rows)。
    每行：cycle_idx, frame_idx, timestamp, 然后 6 组 16 列 (m00..m33)。
    """
    if not os.path.isfile(path):
        return None, []
    with open(path, "r", newline="") as f:
        r = csv.reader(f)
        header = next(r, None)
        if not header:
            return header, []
        rows = [row for row in r if len(row) >= 3 + 6 * 16]
    return header, rows


def _extract_matrices_for_joint(rows, joint_idx, base=3):
    """从多行中提取某一 joint 的 4x4 矩阵列表。joint_idx 为 2..7。"""
    j_offset = (joint_idx - 2) * 16
    matrices = []
    for row in rows:
        flat = []
        for r in range(4):
            for c in range(4):
                v = _parse_float(row[base + j_offset + r * 4 + c])
                flat.append(v)
        if any(math.isnan(x) for x in flat):
            continue
        matrices.append(np.array(flat).reshape(4, 4))
    return np.array(matrices) if matrices else None


def _fit_and_save_T_for_point(joint1_cycle_dir, point_idx, out_cycle_dir, x_threshold, verbose):
    """对单个 point 的 point_P_in_joint1.csv 中 6 个 joint 的 T 分别拟合，合并写入一个 CSV。"""
    csv_name = "point_%d_in_joint1.csv" % point_idx
    csv_path = os.path.join(joint1_cycle_dir, csv_name)
    _, rows = _load_point_joint1_csv(csv_path)
    if not rows:
        if verbose:
            print("  跳过 point_%d：无数据 %s" % (point_idx, csv_path))
        return
    T_list = []
    for j in JOINTS:
        matrices = _extract_matrices_for_joint(rows, j)
        if matrices is None or len(matrices) == 0:
            if verbose:
                print("  point_%d joint_%d：无有效矩阵" % (point_idx, j))
            T_list.append(None)
            continue
        matrices_filtered, _ = filter_matrices_by_x_position(matrices, x_threshold=x_threshold)
        if len(matrices_filtered) == 0:
            if verbose:
                print("  point_%d joint_%d：筛选后无剩余矩阵" % (point_idx, j))
            T_list.append(None)
            continue
        T_fit = fit_transform_matrix_ls(matrices_filtered)
        T_list.append(T_fit)
    # 若至少有一个有效 T，则写入一个 CSV：每个 T 占一行，共 6 行（joint 2..7）
    valid = [T for T in T_list if T is not None]
    if not valid:
        return
    header = ["joint"] + [f"m{r}{c}" for r in range(4) for c in range(4)]
    out_name = "point_%d_T_all_joints_in_joint1_fitted.csv" % point_idx
    out_path = os.path.join(out_cycle_dir, out_name)
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    with open(out_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        for j in JOINTS:
            T = T_list[j - 2]
            if T is not None:
                row = [j] + ["%.8g" % T[r, c] for r in range(4) for c in range(4)]
            else:
                row = [j] + [""] * 16
            w.writerow(row)
    if verbose:
        print("  已保存 %s" % out_path)


def _fit_sensor_angles_for_cycle_point(cycles_cycle_dir, point_idx, out_cycle_dir, verbose):
    """
    从 cycles_dir/cycle_N/data.csv 中取出 point_idx 对应的所有行的 left_arm_1..7，
    对每列求均值（忽略 nan），写入 point_P_sensor_angles_fitted.csv。
    """
    data_csv = os.path.join(cycles_cycle_dir, "data.csv")
    if not os.path.isfile(data_csv):
        if verbose:
            print("  跳过 point_%d 传感器拟合：无 data.csv %s" % (point_idx, data_csv))
        return
    cols = [[] for _ in range(7)]
    with open(data_csv, "r", newline="") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        if not header:
            return
        try:
            point_col_idx = header.index("point_idx")
        except ValueError:
            point_col_idx = 1
        try:
            left_arm_indices = [header.index(c) for c in SENSOR_COLS]
        except ValueError:
            # 退化为固定列：前 4 列为 meta，8*7=56 为动捕，接着 7 列为 left_arm
            left_arm_indices = list(range(4 + 8 * 7, 4 + 8 * 7 + 7))
        for row in reader:
            if len(row) <= max(left_arm_indices):
                continue
            try:
                pid = int(row[point_col_idx])
            except (ValueError, IndexError):
                continue
            if pid != point_idx:
                continue
            for i in range(7):
                v = _parse_float(row[left_arm_indices[i]])
                if not math.isnan(v):
                    cols[i].append(v)
    if not all(len(c) > 0 for c in cols):
        if verbose:
            print("  point_%d 传感器：无有效数据" % point_idx)
        return
    angles = [float(np.mean(c)) for c in cols]
    out_path = os.path.join(out_cycle_dir, "point_%d_sensor_angles_fitted.csv" % point_idx)
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    with open(out_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(SENSOR_COLS)
        w.writerow(["%.8g" % x for x in angles])
    if verbose:
        print("  已保存 %s" % out_path)


def _list_cycle_dirs(joint1_root):
    """返回 joint1_root 下所有 cycle_N 目录名，按 N 排序。"""
    if not os.path.isdir(joint1_root):
        return []
    out = []
    for name in os.listdir(joint1_root):
        m = re.match(r"cycle_(\d+)$", name)
        if m and os.path.isdir(os.path.join(joint1_root, name)):
            out.append((int(m.group(1)), name))
    return [name for _, name in sorted(out)]


def _load_fitted_angles_deg_for_one_cycle(output_root, cname, point_idx):
    """
    从 joint1_fitted/<cname>/point_P_sensor_angles_fitted.csv 读取该 cycle 拟合后的左臂 7 关节角（deg）。
    成功返回 [a1..a7]，失败返回 None。
    """
    csv_path = os.path.join(
        output_root,
        cname,
        "point_%d_sensor_angles_fitted.csv" % point_idx,
    )
    if not os.path.isfile(csv_path):
        return None
    with open(csv_path, "r", newline="") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        row = next(reader, None)
    if not header or not row:
        return None
    angles_deg = []
    for col in SENSOR_COLS:
        try:
            idx = header.index(col)
        except ValueError:
            return None
        if idx >= len(row):
            return None
        v = _parse_float(row[idx])
        if math.isnan(v):
            return None
        angles_deg.append(v)
    return angles_deg if len(angles_deg) == 7 else None


def _collect_fk_positions_for_cycles(output_root, cycles, point_idx, joint_idx):
    """
    对每个 cycle 读取该 cycle 的 point_P_sensor_angles_fitted.csv，调用 FK 得到 joint 在 joint_1 下的位置 (mm)。
    返回 (fk_xs, fk_ys, fk_zs)，与 cycles 一一对应；某 cycle 无数据或 FK 失败时该位置为 nan。
    """
    fk_xs, fk_ys, fk_zs = [], [], []
    for cname in cycles:
        angles = _load_fitted_angles_deg_for_one_cycle(output_root, cname, point_idx)
        if angles is None:
            fk_xs.append(math.nan)
            fk_ys.append(math.nan)
            fk_zs.append(math.nan)
            continue
        pos = _call_fk_for_joint(angles, joint_idx)
        if pos is None:
            fk_xs.append(math.nan)
            fk_ys.append(math.nan)
            fk_zs.append(math.nan)
        else:
            fx, fy, fz = pos
            fk_xs.append(fx)
            fk_ys.append(fy)
            fk_zs.append(fz)
    return fk_xs, fk_ys, fk_zs


def _ensure_ros_node():
    if not rospy.core.is_initialized():
        rospy.init_node("four_points_fit_save_fk", anonymous=True)


def _call_fk_for_joint(q_left_deg, joint_idx):
    """
    调用 FK 服务，给定左臂 7 关节角（deg），返回某个 joint 在 joint_1 基坐标系下的位置 (mm)。
    使用 test_fk.py 中相同的服务接口 /ik/fk_srv_with_refer_frame。
    """
    end_frame = FK_END_FRAMES.get(joint_idx)
    if end_frame is None:
        return None

    _ensure_ros_node()
    try:
        rospy.wait_for_service("/ik/fk_srv_with_refer_frame", timeout=5.0)
    except rospy.ROSException:
        return None

    client = rospy.ServiceProxy("/ik/fk_srv_with_refer_frame", fkSrvWithReferFrame)

    # 左臂 7 关节角（deg -> rad），右臂补 0，组成 14 维 q
    q_left_rad = [math.radians(float(v)) for v in q_left_deg]
    q_full = q_left_rad + [0.0] * 7

    req = fkSrvWithReferFrameRequest()
    req.q = [float(v) for v in q_full]
    req.base_frame = FK_BASE_FRAME
    req.end_effector_frame = end_frame
    req.hand_side = 0  # 0 = 左臂
    # 优先使用用户指定的 URDF（若 srv 支持该字段）
    if hasattr(req, "urdf_path"):
        req.urdf_path = FK_URDF_PATH

    try:
        res = client(req)
    except rospy.ServiceException:
        return None
    if not res.success:
        return None

    pose = res.hand_poses.left_pose
    x, y, z = pose.pos_xyz
    # FK 返回单位为 m，这里统一换成 mm 与拟合 T 对齐
    return float(x * 1000.0), float(y * 1000.0), float(z * 1000.0)


def _collect_translations_for_joint(output_root, cycle_names, point_idx, joint_idx):
    """
    从拟合后的 CSV（joint1_fitted/cycle_*/point_P_T_all_joints_in_joint1_fitted.csv）中，
    收集指定 joint 在某个 point 上各个 cycle 的平移量 tx/ty/tz（来自 4x4 T 的 m03/m13/m23）。
    """
    xs = []
    ys = []
    zs = []
    cycles = []
    for cname in cycle_names:
        csv_path = os.path.join(
            output_root,
            cname,
            "point_%d_T_all_joints_in_joint1_fitted.csv" % point_idx,
        )
        if not os.path.isfile(csv_path):
            continue
        with open(csv_path, "r", newline="") as f:
            reader = csv.reader(f)
            header = next(reader, None)
            if not header:
                continue
            try:
                joint_col_idx = header.index("joint")
            except ValueError:
                joint_col_idx = 0
            rows = list(reader)
        for row in rows:
            if not row:
                continue
            try:
                j = int(row[joint_col_idx])
            except (ValueError, IndexError):
                continue
            if j != joint_idx:
                continue
            # 后面 16 列按照 m00..m33 展平
            if len(row) < joint_col_idx + 1 + 16:
                continue
            flat = []
            for k in range(16):
                v = _parse_float(row[joint_col_idx + 1 + k])
                flat.append(v)
            if any(math.isnan(v) for v in flat):
                continue
            m = np.array(flat, dtype=float).reshape(4, 4)
            tx = float(m[0, 3])
            ty = float(m[1, 3])
            tz = float(m[2, 3])
            xs.append(tx)
            ys.append(ty)
            zs.append(tz)
            cycles.append(cname)
            break
    return cycles, xs, ys, zs


def _plot_joint_translations(output_root, cycle_names, point_idx, joint_idx):
    """
    一张图两行四列，动捕与 FK 画图逻辑一致（每个 cycle 一个点）。
    - 第一行：动捕多次循环 — 3D 散点、X/Y/Z 柱状图（横轴 cycle）；
    - 第二行：FK 多次循环 — 每个 cycle 用该 cycle 的拟合关节角算 FK，同样 3D 散点 + X/Y/Z 柱状图。
    """
    cycles, xs, ys, zs = _collect_translations_for_joint(
        output_root, cycle_names, point_idx, joint_idx
    )
    if not cycles:
        return

    # 每个 cycle 各算一次 FK，得到与 cycles 一一对应的 fk_xs, fk_ys, fk_zs
    fk_xs, fk_ys, fk_zs = _collect_fk_positions_for_cycles(
        output_root, cycles, point_idx, joint_idx
    )

    idx = np.arange(len(cycles))
    fig = plt.figure(figsize=(18, 9))

    # 统计量（忽略 nan）：平均值、标准差、最小值、最大值
    def _stats(arr):
        a = np.array(arr, dtype=float)
        a = a[~np.isnan(a)]
        if a.size == 0:
            return None
        return float(a.mean()), float(a.std(ddof=0)), float(a.min()), float(a.max())

    mocap_stats = {
        "X": _stats(xs),
        "Y": _stats(ys),
        "Z": _stats(zs),
    }
    fk_stats = {
        "X": _stats(fk_xs),
        "Y": _stats(fk_ys),
        "Z": _stats(fk_zs),
    }

    def _enclosing_sphere(x_list, y_list, z_list):
        """
        以 (mean(x), mean(y), mean(z)) 为球心，
        max_i sqrt((x-xc)^2+(y-yc)^2+(z-zc)^2) 为半径的包络球。
        返回 (xc, yc, zc, r)。若无有效点返回 None。
        """
        pts = [(float(x), float(y), float(z)) for x, y, z in zip(x_list, y_list, z_list)
               if not (math.isnan(x) or math.isnan(y) or math.isnan(z))]
        if not pts:
            return None
        xs0 = np.array([p[0] for p in pts], dtype=float)
        ys0 = np.array([p[1] for p in pts], dtype=float)
        zs0 = np.array([p[2] for p in pts], dtype=float)
        xc = float(xs0.mean())
        yc = float(ys0.mean())
        zc = float(zs0.mean())
        d = np.sqrt((xs0 - xc) ** 2 + (ys0 - yc) ** 2 + (zs0 - zc) ** 2)
        r = float(d.max()) if d.size else 0.0
        return xc, yc, zc, r

    def _draw_sphere_on_3d(ax, xc, yc, zc, r, color, label):
        # 半透明球面 + 三条正交大圆（比 plot_wireframe 更兼容）
        u = np.linspace(0.0, 2.0 * math.pi, 40)
        v = np.linspace(0.0, math.pi, 20)
        sx = xc + r * np.outer(np.cos(u), np.sin(v))
        sy = yc + r * np.outer(np.sin(u), np.sin(v))
        sz = zc + r * np.outer(np.ones_like(u), np.cos(v))
        ax.plot_surface(sx, sy, sz, color=color, alpha=0.12, linewidth=0.0, shade=False)
        # 三条正交大圆：XY(常 z)、XZ(常 y)、YZ(常 x)
        t = np.linspace(0.0, 2.0 * math.pi, 240)
        ax.plot(xc + r * np.cos(t), yc + r * np.sin(t), np.full_like(t, zc), color=color, alpha=0.55, linewidth=1.2)
        ax.plot(xc + r * np.cos(t), np.full_like(t, yc), zc + r * np.sin(t), color=color, alpha=0.55, linewidth=1.2)
        ax.plot(np.full_like(t, xc), yc + r * np.cos(t), zc + r * np.sin(t), color=color, alpha=0.55, linewidth=1.2)
        ax.scatter([xc], [yc], [zc], c=color, marker="x", s=90, label=label)

    # ---------- 第一行：动捕多次循环（一行四列）----------
    ax1 = fig.add_subplot(2, 4, 1, projection="3d")
    ax1.scatter(xs, ys, zs, c="b", marker="o")
    ax1.set_xlabel("X (mm)", fontsize=14)
    ax1.set_ylabel("Y (mm)", fontsize=14)
    ax1.set_zlabel("Z (mm)", fontsize=14)
    ax1.tick_params(axis="both", labelsize=12)
    mocap_sphere = _enclosing_sphere(xs, ys, zs)
    if mocap_sphere is not None:
        xc, yc, zc, r = mocap_sphere
        _draw_sphere_on_3d(ax1, xc, yc, zc, r, color="k", label="Enclosing sphere center")
        ax1.set_title("Mocap cycles: 3D position (R=%.3f mm)" % r)
        print("[plot] point_%d joint_%d mocap sphere: center=(%.3f, %.3f, %.3f) R=%.3f mm"
              % (point_idx, joint_idx, xc, yc, zc, r))
        ax1.legend(loc="best", fontsize=10)
    else:
        ax1.set_title("Mocap cycles: 3D position")

    ax2 = fig.add_subplot(2, 4, 2)
    ax2.bar(idx, xs, color="r", alpha=0.7)
    ax2.set_title("Mocap cycles: X (mm)", fontsize=14)
    ax2.set_xticks(idx)
    ax2.set_xticklabels(cycles, rotation=45, ha="right", fontsize=11)
    ax2.tick_params(axis="y", labelsize=12)

    ax3 = fig.add_subplot(2, 4, 3)
    ax3.bar(idx, ys, color="g", alpha=0.7)
    ax3.set_title("Mocap cycles: Y (mm)", fontsize=14)
    ax3.set_xticks(idx)
    ax3.set_xticklabels(cycles, rotation=45, ha="right", fontsize=11)
    ax3.tick_params(axis="y", labelsize=12)

    ax4 = fig.add_subplot(2, 4, 4)
    ax4.bar(idx, zs, color="b", alpha=0.7)
    ax4.set_title("Mocap cycles: Z (mm)", fontsize=14)
    ax4.set_xticks(idx)
    ax4.set_xticklabels(cycles, rotation=45, ha="right", fontsize=11)
    ax4.tick_params(axis="y", labelsize=12)

    # ---------- 第二行：FK 多次循环（与动捕相同画法，每个 cycle 一个点）----------
    # 3D：只画有效点（x,y,z 均非 nan）
    valid_fk = [(x, y, z) for x, y, z in zip(fk_xs, fk_ys, fk_zs)
                if not (math.isnan(x) or math.isnan(y) or math.isnan(z))]
    ax5 = fig.add_subplot(2, 4, 5, projection="3d")
    ax5.set_xlabel("X (mm)", fontsize=14)
    ax5.set_ylabel("Y (mm)", fontsize=14)
    ax5.set_zlabel("Z (mm)", fontsize=14)
    ax5.tick_params(axis="both", labelsize=12)
    fk_sphere = _enclosing_sphere(fk_xs, fk_ys, fk_zs)
    if fk_sphere is not None:
        xc, yc, zc, r = fk_sphere
        _draw_sphere_on_3d(ax5, xc, yc, zc, r, color="k", label="Enclosing sphere center")
        ax5.set_title("FK cycles: 3D position (R=%.3f mm)" % r)
        print("[plot] point_%d joint_%d FK   sphere: center=(%.3f, %.3f, %.3f) R=%.3f mm"
              % (point_idx, joint_idx, xc, yc, zc, r))
        ax5.legend(loc="best", fontsize=10)
    else:
        ax5.set_title("FK cycles: 3D position")
    if valid_fk:
        fk_x_ok, fk_y_ok, fk_z_ok = [p[0] for p in valid_fk], [p[1] for p in valid_fk], [p[2] for p in valid_fk]
        ax5.scatter(fk_x_ok, fk_y_ok, fk_z_ok, c="r", marker="^")

    ax6 = fig.add_subplot(2, 4, 6)
    ax6.bar(idx, fk_xs, color="r", alpha=0.7)
    ax6.set_title("FK cycles: X (mm)", fontsize=14)
    ax6.set_xticks(idx)
    ax6.set_xticklabels(cycles, rotation=45, ha="right", fontsize=11)
    ax6.tick_params(axis="y", labelsize=12)

    ax7 = fig.add_subplot(2, 4, 7)
    ax7.bar(idx, fk_ys, color="g", alpha=0.7)
    ax7.set_title("FK cycles: Y (mm)", fontsize=14)
    ax7.set_xticks(idx)
    ax7.set_xticklabels(cycles, rotation=45, ha="right", fontsize=11)
    ax7.tick_params(axis="y", labelsize=12)

    ax8 = fig.add_subplot(2, 4, 8)
    ax8.bar(idx, fk_zs, color="b", alpha=0.7)
    ax8.set_title("FK cycles: Z (mm)", fontsize=14)
    ax8.set_xticks(idx)
    ax8.set_xticklabels(cycles, rotation=45, ha="right", fontsize=11)
    ax8.tick_params(axis="y", labelsize=12)

    # 在各自的柱状图内部写“均值 & 标准差”，字体更大更清晰
    def _annotate_stats(ax, stats, label, extra_lines=None):
        if stats is None:
            return
        mean_v, std_v, _, _ = stats
        lines = ["%s" % label, "mean=%.3f" % mean_v, "std=%.3f" % std_v]
        if extra_lines:
            lines.extend(extra_lines)
        txt = "\n".join(lines)
        ax.text(
            0.02,
            0.97,
            txt,
            transform=ax.transAxes,
            va="top",
            ha="left",
            fontsize=14,
            bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.7),
        )

    mocap_r_line = None
    if mocap_sphere is not None:
        mocap_r_line = "R=%.3f mm" % mocap_sphere[3]
    _annotate_stats(ax2, mocap_stats.get("X"), "Mocap X", extra_lines=[mocap_r_line] if mocap_r_line else None)
    _annotate_stats(ax3, mocap_stats.get("Y"), "Mocap Y")
    _annotate_stats(ax4, mocap_stats.get("Z"), "Mocap Z")

    fk_r_line = None
    if fk_sphere is not None:
        fk_r_line = "R=%.3f mm" % fk_sphere[3]
    _annotate_stats(ax6, fk_stats.get("X"), "FK X", extra_lines=[fk_r_line] if fk_r_line else None)
    _annotate_stats(ax7, fk_stats.get("Y"), "FK Y")
    _annotate_stats(ax8, fk_stats.get("Z"), "FK Z")

    fig.suptitle(
        "joint_%d → joint_1 (point %d): row1=mocap cycles, row2=FK baseline" % (joint_idx, point_idx),
        fontsize=16,
    )
    fig.tight_layout(rect=[0, 0.02, 1, 0.96])

    plots_dir = os.path.join(output_root, "plots", "joint%d_in_joint_1" % joint_idx)
    os.makedirs(plots_dir, exist_ok=True)
    out_path = os.path.join(
        plots_dir,
        "point_%d_joint_%d_in_joint1_summary.png" % (point_idx, joint_idx),
    )
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(
        description="读取 joint1 下每轮每点的 CSV，拟合成单 T 与传感器角度，输出到 joint1_fitted"
    )
    parser.add_argument(
        "--joint1_dir",
        type=str,
        default=os.path.join(_script_dir, "output", "joint1"),
        help="第二个脚本输出根目录（其下为 cycle_0, cycle_1, ...）",
    )
    parser.add_argument(
        "--cycles_dir",
        type=str,
        default=os.path.join(_script_dir, "output", "cycles"),
        help="四点循环 data.csv 所在根目录（其下为 cycle_0/data.csv, ...）",
    )
    parser.add_argument(
        "--output_dir",
        type=str,
        default=os.path.join(_script_dir, "output", "joint1_fitted"),
        help="拟合结果输出根目录",
    )
    parser.add_argument(
        "--x_threshold",
        type=float,
        default=9999.0,
        help="T 矩阵 x 位置筛选阈值（mm），大于此值的样本剔除",
    )
    parser.add_argument(
        "--plot-joint",
        type=int,
        default=None,
        help="可选：指定 joint(2..7) 编号，对该关节在各个 cycle、各个点上的拟合平移做统计绘图",
    )
    parser.add_argument("-v", "--verbose", action="store_true", help="打印详情")
    args = parser.parse_args()

    # 先根据 joint1_dir 列出 cycle_*（用于拟合）；如果没有而且只想画图，则尝试从 output_dir 推断
    cycle_names = _list_cycle_dirs(args.joint1_dir)
    if not cycle_names:
        if args.plot_joint is None:
            print("未找到任何 cycle_* 目录: %s" % args.joint1_dir)
            return
        # 仅绘图模式：允许 joint1 目录不存在，直接从 joint1_fitted 结果中推断 cycle_*
        cycle_names = _list_cycle_dirs(args.output_dir)
        if not cycle_names:
            print(
                "未找到任何 cycle_* 目录: %s 或 %s，既无法拟合也无法绘图"
                % (args.joint1_dir, args.output_dir)
            )
            return

    # 如果 joint1_dir 下存在 cycle_*，先做拟合与传感器角度统计
    if os.path.isdir(args.joint1_dir):
        for cycle_name in cycle_names:
            joint1_cycle = os.path.join(args.joint1_dir, cycle_name)
            cycles_cycle = os.path.join(args.cycles_dir, cycle_name)
            out_cycle = os.path.join(args.output_dir, cycle_name)
            os.makedirs(out_cycle, exist_ok=True)
            if args.verbose:
                print("[%s]" % cycle_name)
            for p in POINT_IDXS:
                _fit_and_save_T_for_point(
                    joint1_cycle, p, out_cycle, args.x_threshold, args.verbose
                )
                _fit_sensor_angles_for_cycle_point(
                    cycles_cycle, p, out_cycle, args.verbose
                )
        print("拟合结果已写入: %s" % args.output_dir)

    # 如指定了绘图关节，则对每个点使用“模块三拟合结果”输出汇总图
    if args.plot_joint is not None:
        if args.plot_joint not in JOINTS:
            print("警告: --plot-joint 只能在 %s 中选择，当前为 %s，跳过绘图" % (JOINTS, args.plot_joint))
        else:
            for p in POINT_IDXS:
                _plot_joint_translations(args.output_dir, cycle_names, p, args.plot_joint)



if __name__ == "__main__":
    main()
