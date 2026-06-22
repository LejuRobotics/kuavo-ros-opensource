#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
四点循环运动与动捕记录：
  - 四组左臂关节角度对应空间中四个点，在四点之间循环运动。
  - 先插值到第一个点（不参与循环计数），再在四个点之间插值循环。
  - 到达每个目标点后保持 10s；在「到达后 3s」到「结束前 3s」这段区间内持续记录动捕（8 个刚体），保证已静止。
  - 每轮循环的四个点数据单独保存为一个 CSV，放在 output/cycles/cycle_0/、output/cycles/cycle_1/、... 下；每轮结束后绘制位置图（每轮 4 张图，对应 4 个点）保存到对应文件夹。
"""
from __future__ import print_function

import argparse
import csv
import math
import os
import sys
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import ScalarFormatter

# 本脚本所在目录（four_points_cycle/）；其父为 arm_accuracy_calibration，再父为 tools
_script_dir = os.path.dirname(os.path.abspath(__file__))
_parent = os.path.dirname(_script_dir)  # arm_accuracy_calibration
_pkg_parent = os.path.dirname(_parent)  # tools
if _pkg_parent not in sys.path:
    sys.path.insert(0, _pkg_parent)

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

try:
    from kuavo_msgs.msg import sensorsData
except Exception:
    sensorsData = None

from arm_accuracy_calibration.functions import (
    joint_1_callback,
    joint_2_callback,
    joint_3_callback,
    joint_4_callback,
    joint_5_callback,
    joint_6_callback,
    joint_7_callback,
    joint_end_callback,
    get_all_rigid_body_poses,
)
from arm_accuracy_calibration.functions.cmd_arm_joint import (
    ARM_NAMES_JOINT1_14,
    build_positions_all,
    get_current_joints_deg,
    get_current_joints_deg_from_sensors,
    try_switch_arm_ctrl_mode,
    enable_wbc_arm_trajectory,
)


# 四组左臂关节角度 (deg)，对应空间中四个点。可后续替换为实际值。
POSE_POINT_1 = [-21.42, 28.68, -54.64, -80.54, 49.35, 18.80, -21.48]
POSE_POINT_2 = [-12.35, -8.70, -27.39, -66.98, 2.35, 18.73, -9.14]
POSE_POINT_3 = [-14.03, -10.27, -27.20, -97.85, -15.00, 14.86, 27.69]
POSE_POINT_4 = [-14.08, 21.07, -23.97, -109.42, 13.86, 18.53, 25.45]

# 记录用的 8 个刚体：joint_1 ~ joint_7 + joint_end
RECORD_BODIES = [
    "joint_1", "joint_2", "joint_3", "joint_4",
    "joint_5", "joint_6", "joint_7", "joint_end",
]
# 左臂 7 关节在 sensors_data_raw 的 joint_q 中的下标
LEFT_ARM_JOINT_SLICE = slice(13, 20)

_latest_sensor_data = None


def _sensor_data_callback(msg):
    global _latest_sensor_data
    _latest_sensor_data = msg


def _get_left_arm_joints():
    """从已订阅的 sensors_data_raw 中取左臂 7 关节角(deg)，无数据时返回 None。若为弧度则转为度。"""
    sd = _latest_sensor_data
    if sd is None or not hasattr(sd, "joint_data") or not hasattr(sd.joint_data, "joint_q"):
        return None
    q = sd.joint_data.joint_q
    if len(q) < 20:
        return None
    vals = [float(q[i]) for i in range(LEFT_ARM_JOINT_SLICE.start, LEFT_ARM_JOINT_SLICE.stop)]
    if vals and max(abs(v) for v in vals) <= 2 * 3.1416 + 0.5:
        vals = [math.degrees(v) for v in vals]
    return vals


def _interpolate(a, b, t):
    return a + (b - a) * t


def _csv_header():
    cols = ["cycle_idx", "point_idx", "frame_idx", "timestamp"]
    for name in RECORD_BODIES:
        cols.extend(["%s_px" % name, "%s_py" % name, "%s_pz" % name])
        cols.extend(["%s_qx" % name, "%s_qy" % name, "%s_qz" % name, "%s_qw" % name])
    cols.extend(["left_arm_%d" % (i + 1) for i in range(7)])
    return cols


def _mocap_row(rigid_body_poses, record_stamp, cycle_idx, point_idx, frame_idx, left_arm_joints=None):
    """一行动捕数据 + 左臂传感器关节角；没有该刚体/传感器数据或 nan 时填 0。"""
    row = [cycle_idx, point_idx, frame_idx, "%.6f" % record_stamp]
    for name in RECORD_BODIES:
        if name in rigid_body_poses:
            d = rigid_body_poses[name]
            for x in list(d["pos"]) + list(d["quat"]):
                v = float(x) if x is not None else 0.0
                row.append("0" if (v != v) else "%.8g" % v)  # nan != nan
        else:
            row.extend(["0"] * 7)
    if left_arm_joints is not None and len(left_arm_joints) >= 7:
        for x in left_arm_joints[:7]:
            v = float(x) if x is not None else 0.0
            row.append("0" if (v != v) else "%.8g" % v)
    else:
        row.extend(["0"] * 7)
    return row


def _write_separator(writer, cycle_idx, point_idx):
    """写一行分隔行，便于区分第几次循环、第几个点。"""
    row = ["=== Cycle %d Point %d ===" % (cycle_idx, point_idx)] + [""] * (len(_csv_header()) - 1)
    writer.writerow(row)


def _parse_float(s):
    s = (s or "").strip().lower()
    if s == "nan" or s == "":
        return math.nan
    try:
        return float(s)
    except ValueError:
        return math.nan


def _plot_cycle_folder(csv_path):
    """读取本轮循环的 CSV，按 point_idx 分组；每个点对每个刚体单独画一张位置图（X/Y/Z vs time）。"""
    if not os.path.isfile(csv_path):
        rospy.logwarn("[four_points_cycle] 未找到 CSV，跳过绘图: %s", csv_path)
        return
    out_dir = os.path.dirname(csv_path)
    cols_per_body = 7
    time_col = 3
    by_point = {0: [], 1: [], 2: [], 3: []}
    with open(csv_path, "r", newline="") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        if not header:
            return
        for row in reader:
            if len(row) <= 4:
                continue
            if str(row[0]).strip().startswith("==="):
                continue
            try:
                point_idx = int(row[1])
            except (ValueError, IndexError):
                continue
            if point_idx not in by_point:
                continue
            by_point[point_idx].append(row)
    for point_idx in (0, 1, 2, 3):
        rows = by_point.get(point_idx, [])
        if not rows:
            continue
        # 预先解析该点的时间戳与各刚体 xyz
        t_vals = []
        x_by_body = [[] for _ in RECORD_BODIES]
        y_by_body = [[] for _ in RECORD_BODIES]
        z_by_body = [[] for _ in RECORD_BODIES]
        for row in rows:
            if len(row) <= 4 + 2:
                continue
            # 注意：时间戳不保证连续，直接作为 x 轴
            t_vals.append(_parse_float(row[time_col]))
            for i in range(len(RECORD_BODIES)):
                start = 4 + i * cols_per_body
                if len(row) <= start + 2:
                    x_by_body[i].append(math.nan)
                    y_by_body[i].append(math.nan)
                    z_by_body[i].append(math.nan)
                else:
                    x_by_body[i].append(_parse_float(row[start]))
                    y_by_body[i].append(_parse_float(row[start + 1]))
                    z_by_body[i].append(_parse_float(row[start + 2]))

        for i, body_name in enumerate(RECORD_BODIES):
            fig, axes = plt.subplots(3, 1, sharex=True, figsize=(8, 6))
            fig.suptitle("Cycle point %d - %s position vs time" % (point_idx, body_name), fontsize=12)
            for ax in axes:
                ax.yaxis.set_major_formatter(ScalarFormatter(useOffset=False))
                ax.ticklabel_format(style="plain", axis="y")
                ax.grid(True, alpha=0.3)
            axes[0].plot(t_vals, x_by_body[i], "r-", label="X")
            axes[0].set_ylabel("X (mm)")
            axes[0].legend(loc="upper right")
            axes[1].plot(t_vals, y_by_body[i], "g-", label="Y")
            axes[1].set_ylabel("Y (mm)")
            axes[1].legend(loc="upper right")
            axes[2].plot(t_vals, z_by_body[i], "b-", label="Z")
            axes[2].set_ylabel("Z (mm)")
            axes[2].set_xlabel("Time (s)")
            axes[2].legend(loc="upper right")
            plt.tight_layout()
            png_path = os.path.join(out_dir, "point_%d_pose_%s.png" % (point_idx, body_name))
            fig.savefig(png_path, dpi=150)
            plt.close(fig)
            rospy.loginfo("[four_points_cycle] 已保存 %s", png_path)


def run_interpolation(pub, js, rate, hz, from_pose, to_pose, duration_s, arm="left"):
    """从 from_pose 插值到 to_pose，持续 duration_s 秒。"""
    dur = max(0.01, duration_s)
    n_steps = max(1, int(dur * hz))
    for i in range(n_steps + 1):
        if rospy.is_shutdown():
            return False
        t = i / n_steps
        interp = [_interpolate(from_pose[j], to_pose[j], t) for j in range(7)]
        js.header.stamp = rospy.Time.now()
        js.position = build_positions_all(arm, interp)
        pub.publish(js)
        rate.sleep()
    return True


def main():
    parser = argparse.ArgumentParser(description="四点循环运动：插值到第一点后，在四点点间循环，每点停留10s并记录动捕")
    parser.add_argument("--arm", choices=["left", "right", "both"], default="left")
    parser.add_argument("--duration_to_first", type=float, default=5.0, help="插值到第一个点耗时(s)")
    parser.add_argument("--duration_segment", type=float, default=5.0, help="点与点之间插值耗时(s)")
    parser.add_argument("--hold_duration", type=float, default=10.0, help="到达每点后保持时间(s)")
    parser.add_argument("--record_interval", type=float, nargs=2, default=[3.0, 7.0],
                        help="保持期内持续记录的区间(s)，默认 [3,7] 即到达后3s到结束前3s")
    parser.add_argument("--record_hz", type=float, default=100.0, help="记录区间内的采样频率(Hz)")
    parser.add_argument("--cycles", type=int, default=30, help="循环次数，0 表示无限直到 Ctrl+C")
    parser.add_argument("--hz", type=float, default=100.0)
    parser.add_argument("--output-dir", type=str, default=None,
                        help="输出根目录，每轮循环在该目录下生成 cycle_0/、cycle_1/、... 默认本目录 output/")
    args = parser.parse_args()

    poses = [POSE_POINT_1, POSE_POINT_2, POSE_POINT_3, POSE_POINT_4]
    for i, p in enumerate(poses):
        if len(p) != 7:
            raise ValueError("POSE_POINT_%d 须为 7 个关节角(deg)" % (i + 1))

    record_t1, record_t2 = sorted(args.record_interval)
    if not (0 < record_t1 < record_t2 < args.hold_duration):
        raise ValueError("--record_interval 两时刻须在 (0, hold_duration) 内且 t1 < t2，例如 3 7")

    rospy.init_node("four_points_cycle", anonymous=True)

    try_switch_arm_ctrl_mode(2)
    enable_wbc_arm_trajectory()
    pub = rospy.Publisher("/kuavo_arm_traj", JointState, queue_size=1, tcp_nodelay=True)
    t0 = time.time()
    while not rospy.is_shutdown() and pub.get_num_connections() == 0 and (time.time() - t0) < 3.0:
        rospy.sleep(0.05)

    js = JointState()
    js.name = ARM_NAMES_JOINT1_14
    js.velocity = [0.0] * 14
    js.effort = [0.0] * 14
    hz = max(1.0, args.hz)
    rate = rospy.Rate(hz)

    # 订阅动捕（joint_1 -> joint_1_callback, joint_end -> joint_end_callback）
    name_to_cb = {
        "joint_1": joint_1_callback, "joint_2": joint_2_callback, "joint_3": joint_3_callback,
        "joint_4": joint_4_callback, "joint_5": joint_5_callback, "joint_6": joint_6_callback,
        "joint_7": joint_7_callback, "joint_end": joint_end_callback,
    }
    for name in RECORD_BODIES:
        rospy.Subscriber(name, PoseStamped, name_to_cb[name], queue_size=10)
    if sensorsData is not None:
        rospy.Subscriber("/sensors_data_raw", sensorsData, _sensor_data_callback, queue_size=10)
    rospy.loginfo("[four_points_cycle] 已订阅动捕 %s 与 /sensors_data_raw（左臂关节角）...", RECORD_BODIES)
    rospy.sleep(3.0)

    # 输出根目录；每轮循环在 output/cycles/cycle_0/、cycle_1/、... 下各一个 CSV + 4 张图
    if args.output_dir is None:
        args.output_dir = os.path.join(_script_dir, "output")
    cycles_base = os.path.join(args.output_dir, "cycles")
    os.makedirs(cycles_base, exist_ok=True)

    # ---------- 阶段 0：从当前插值到第一个点（不参与循环） ----------
    rospy.loginfo("[four_points_cycle] 读取当前关节角并插值到第一点...")
    current = get_current_joints_deg_from_sensors(args.arm, timeout_s=5.0)
    if current is None:
        current = get_current_joints_deg(args.arm)
    if current is None or len(current) != 7:
        current = list(poses[0])
    if not run_interpolation(pub, js, rate, hz, current, poses[0], args.duration_to_first, args.arm):
        return 1
    rospy.loginfo("[four_points_cycle] 已到达第一点，开始循环")

    cycle_idx = 0
    current_pose = list(poses[0])
    n_cycles = args.cycles  # 0 = 无限

    while True:
        if n_cycles > 0 and cycle_idx >= n_cycles:
            break
        if rospy.is_shutdown():
            break

        cycle_dir = os.path.join(cycles_base, "cycle_%d" % cycle_idx)
        os.makedirs(cycle_dir, exist_ok=True)
        cycle_csv = os.path.join(cycle_dir, "data.csv")

        with open(cycle_csv, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(_csv_header())

            # 本循环内依次去 点4 -> 点3 -> 点2 -> 点1
            for point_idx in [3, 2, 1, 0]:
                if rospy.is_shutdown():
                    break
                from_pose = current_pose
                to_pose = poses[point_idx]
                rospy.loginfo("[four_points_cycle] Cycle %d -> Point %d", cycle_idx, point_idx)

                if not run_interpolation(pub, js, rate, hz, from_pose, to_pose, args.duration_segment, args.arm):
                    break
                current_pose = list(to_pose)

                # 记录阶段：只在内存中攒行，避免在控制循环里写文件导致卡顿
                point_rows = []
                hold_start = rospy.Time.now().to_sec()
                record_hz = max(1.0, args.record_hz)
                record_period = 1.0 / record_hz
                last_record_time = [hold_start - record_period]
                frame_idx = [0]
                while not rospy.is_shutdown():
                    now = rospy.Time.now().to_sec()
                    elapsed = now - hold_start
                    if elapsed >= args.hold_duration:
                        break
                    js.header.stamp = rospy.Time.now()
                    js.position = build_positions_all(args.arm, to_pose)
                    pub.publish(js)
                    if record_t1 <= elapsed <= record_t2:
                        if now - last_record_time[0] >= record_period:
                            last_record_time[0] = now
                            stamp = rospy.Time.now().to_sec()
                            rb = get_all_rigid_body_poses()
                            left_arm = _get_left_arm_joints()
                            point_rows.append(_mocap_row(rb, stamp, cycle_idx, point_idx, frame_idx[0], left_arm))
                            frame_idx[0] += 1
                    rate.sleep()
                # 该点记录结束后再统一写 CSV，不占用控制循环时间
                _write_separator(writer, cycle_idx, point_idx)
                writer.writerows(point_rows)
                if frame_idx[0] > 0:
                    rospy.loginfo("[four_points_cycle] Cycle %d Point %d 记录 %d 帧 (%.1fs~%.1fs)",
                                  cycle_idx, point_idx, frame_idx[0], record_t1, record_t2)

        rospy.loginfo("[four_points_cycle] 已保存 %s，正在绘制位置图...", cycle_csv)
        _plot_cycle_folder(cycle_csv)
        cycle_idx += 1

    rospy.loginfo("[four_points_cycle] 完成，共 %d 轮循环，数据与图表在 %s 下", cycle_idx, cycles_base)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except rospy.ROSInterruptException:
        pass
