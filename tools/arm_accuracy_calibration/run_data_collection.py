#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
模块一：数据采集入口。
本文件独立完成：关节运动控制、话题订阅、数据保存、CSV 生成。
"""
from __future__ import print_function

import argparse
import csv
import math
import os
import subprocess
import sys
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import ScalarFormatter

_script_dir = os.path.dirname(os.path.abspath(__file__))
_parent = os.path.dirname(_script_dir)
if _parent not in sys.path:
    sys.path.insert(0, _parent)

import rospy
from geometry_msgs.msg import PoseStamped
from kuavo_msgs.msg import sensorsData

from arm_accuracy_calibration.functions import (
    sensor_data_callback,
    joint_1_callback,
    joint_2_callback,
    joint_3_callback,
    joint_4_callback,
    joint_5_callback,
    joint_6_callback,
    joint_7_callback,
    belly_pose_callback,
    get_all_rigid_body_poses,
    joint_end_callback,
)

# CSV 列顺序（与 get_all_rigid_body_poses 的 key 一致）：七个关节 + joint_end + belly
BODIES_NAMES = [
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
LEFT_ARM_JOINT_SLICE = slice(13, 20)  # joint_q[13:20]
RECORD_TIMEOUT_SEC = 10.0

# 单关节 1~7 的往复角度范围 (start, end)
JOINT_DEFAULT_ANGLE_DEG = {
    1: (0.0, 20.0),
    2: (0.0, 20.0),
    3: (10.0, -10.0),
    4: (0.0, -20.0),
    5: (10.0, -10.0),
    6: (0.0, 20.0),
    7: (10.0, -10.0),

}

# 单关节往复（--joint 1~7）时，非运动关节保持的七关节位姿（度），共一组即可。
# 运动关节对应分量会被 JOINT_DEFAULT_ANGLE_DEG 的 start/end（或 --start/--end）覆盖。
SWEEP_BASE_POSE_DEG = [10.0, 10.0, 10.0, -10.0, 10.0, 10.0, 10.0]
SWEEP_BASE_POSE_DEG = [20.0, 20.0, 20.0, -20.0, 20.0, 20.0, 20.0]
SWEEP_BASE_POSE_DEG = [30.0, 30.0, 30.0, -30.0, 30.0, 0.0, 30.0]
SWEEP_BASE_POSE_DEG = [40.0, 40.0, 40.0, -40.0, -10.0, 0.0, -10.0]

# 关节 -1：全关节一次性位姿。指定 --joint -1 时，起始和终止均为此位姿（保持不动）

#
# ALL_JOINTS_POSE_DEG = [-60.0, 30.0, -70.0, -70.0, 0.0, 0.0, 0.0]#test2
# ALL_JOINTS_POSE_DEG = [-45.0, 45.0, -50.0, -90.0, 0.0, 0.0, 0.0]#zero
# ALL_JOINTS_POSE_DEG = [-30.0, 20.0, -60.0, -90.0, 0.0, 0.0, 0.0]#txst1
# ALL_JOINTS_POSE_DEG = [-35.0, 25.0, -55.0, -90.0, 0.0, 0.0, 0.0]#txst3
# ALL_JOINTS_POSE_DEG = [-50.0, 40.0, -65.0, -90.0, 0.0, 0.0, 0.0]#txst4
ALL_JOINTS_POSE_DEG = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


_latest_sensor_data = None

def _sensor_data_callback(msg):
    global _latest_sensor_data
    _latest_sensor_data = msg
    sensor_data_callback(msg)


def _get_left_arm_joints():
    """从已订阅的 sensor_data 中取左臂 7 关节角，无数据时返回 None。"""
    sd = _latest_sensor_data
    if sd is None or not hasattr(sd, "joint_data") or not hasattr(sd.joint_data, "joint_q"):
        return None
    q = sd.joint_data.joint_q
    if len(q) < 20:
        return None
    return [float(q[i]) for i in range(LEFT_ARM_JOINT_SLICE.start, LEFT_ARM_JOINT_SLICE.stop)]


def _mocap_csv_header():
    """动捕 CSV 表头：仅刚体位姿。"""
    cols = ["timestamp"]
    for name in BODIES_NAMES:
        cols.extend(["%s_px" % name, "%s_py" % name, "%s_pz" % name])
        cols.extend(["%s_qx" % name, "%s_qy" % name, "%s_qz" % name, "%s_qw" % name])
    return cols


def _sensor_csv_header():
    """传感器 CSV 表头：时间戳 + 左臂 7 关节角。"""
    return ["timestamp"] + ["left_arm_%d" % (i + 1) for i in range(7)]


def _mocap_row(rigid_body_poses, record_stamp):
    """动捕一行：未收到的话题对应列用 0 填充（便于后续数据处理，不写 nan）。"""
    row = [str(record_stamp)]
    for name in BODIES_NAMES:
        if name in rigid_body_poses:
            d = rigid_body_poses[name]
            row.extend("%.8g" % x for x in d["pos"])
            row.extend("%.8g" % x for x in d["quat"])
        else:
            row.extend(["0"] * 7)
    return row


def _sensor_row(record_stamp, left_arm_joints):
    """传感器一行：时间戳 + 左臂 7 关节角，无数据用 nan。"""
    row = [str(record_stamp)]
    if left_arm_joints is not None:
        row.extend("%.8g" % x for x in left_arm_joints)
    else:
        row.extend(["nan"] * 7)
    return row


def _parse_float(s):
    """将 CSV 中的字符串转为 float，'nan' 转为 math.nan。"""
    s = (s or "").strip().lower()
    if s == "nan" or s == "":
        return math.nan
    try:
        return float(s)
    except ValueError:
        return math.nan


def _plot_rigid_body_poses(csv_path):
    """读取刚写入的 CSV，按时间为横轴为每个刚体画 X/Y/Z，一刚体一图，保存到 CSV 同目录下。"""
    if not os.path.isfile(csv_path):
        rospy.logwarn("[run_data_collection] 未找到 CSV，跳过绘图: %s", csv_path)
        return
    out_dir = os.path.dirname(csv_path)
    cols_per_body = 7
    time_col = 0
    rows = []
    with open(csv_path, "r", newline="") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        if not header:
            return
        for row in reader:
            if len(row) <= 1:
                continue
            rows.append(row)
    if not rows:
        rospy.logwarn("[run_data_collection] CSV 无数据行，跳过绘图")
        return
    for i, name in enumerate(BODIES_NAMES):
        start_col = 1 + i * cols_per_body
        t_vals = []
        x_vals = []
        y_vals = []
        z_vals = []
        for row in rows:
            if len(row) <= start_col + 2:
                continue
            t_vals.append(_parse_float(row[time_col]))
            x_vals.append(_parse_float(row[start_col]))
            y_vals.append(_parse_float(row[start_col + 1]))
            z_vals.append(_parse_float(row[start_col + 2]))
        fig, axes = plt.subplots(3, 1, sharex=True, figsize=(8, 6))
        fig.suptitle("Rigid body %s - Position vs time" % name, fontsize=12)
        for ax in axes:
            ax.yaxis.set_major_formatter(ScalarFormatter(useOffset=False))
            ax.ticklabel_format(style="plain", axis="y")
        axes[0].plot(t_vals, x_vals, "r-", label="X")
        axes[0].set_ylabel("X (mm)")
        axes[0].legend(loc="upper right")
        axes[0].grid(True, alpha=0.3)
        axes[1].plot(t_vals, y_vals, "g-", label="Y")
        axes[1].set_ylabel("Y (mm)")
        axes[1].legend(loc="upper right")
        axes[1].grid(True, alpha=0.3)
        axes[2].plot(t_vals, z_vals, "b-", label="Z")
        axes[2].set_ylabel("Z (mm)")
        axes[2].set_xlabel("Time (s)")
        axes[2].legend(loc="upper right")
        axes[2].grid(True, alpha=0.3)
        plt.tight_layout()
        png_path = os.path.join(out_dir, "pose_%s.png" % name)
        fig.savefig(png_path, dpi=150)
        plt.close(fig)
        rospy.loginfo("[run_data_collection] 已保存 %s", png_path)


def main():
    parser = argparse.ArgumentParser(description="模块一：关节运动控制 + 原始位姿记录（本脚本内订阅话题并写 CSV）")
    parser.add_argument("--arm", choices=["left", "right", "both"], default="left", help="手臂")
    parser.add_argument("--joint", type=int, default=0, choices=range(-1, 8),
                        help="关节 -1~7：-1=全关节一次性位姿(表 ALL_JOINTS_POSE_DEG)；0=全关节 0°；1-7=对应单关节")
    parser.add_argument("--start", type=float, default=None,
                        help="起始角度 deg（默认按 joint 查表；关节 0 时忽略，固定 0°）")
    parser.add_argument("--end", type=float, default=None,
                        help="终止角度 deg（默认按 joint 查表；关节 0 时忽略，固定 0°）")
    parser.add_argument("--duration_segment", type=float, default=30.0, help="start<->end 每段耗时 s")
    parser.add_argument("--duration_to_start", type=float, default=2.0, help="到起始点耗时 s")
    parser.add_argument("--workspace", type=str, default="calibration_output",
                        help="工作区根目录，其下为 raw_data_motion、matrix_data_motion、fitted_results（默认: calibration_output）")
    parser.add_argument("--output_dir", type=str, default=None, help="覆盖：指定输出目录，默认 <workspace>/raw_data_motion/joint_<N>")
    parser.add_argument("--record_delay", type=float, default=2.0, help="启动运动后延迟几秒再记录")
    parser.add_argument("--max_frames", type=int, default=10000, help="记录最大帧数")
    parser.add_argument("--record_hz", type=float, default=100.0, help="记录频率 Hz")
    args = parser.parse_args()

    motion_control_arm = args.arm if isinstance(args.arm, str) else "left"
    motion_control_joint = str(args.joint)

    cmd_arm_joint_script = os.path.join(_script_dir, "functions", "cmd_arm_joint.py")

    if args.joint == -1:
        # 关节 -1：全关节一次性位姿，起始和终止均为 ALL_JOINTS_POSE_DEG
        target_str = ",".join(str(a) for a in ALL_JOINTS_POSE_DEG)
        cmd_args = [
            sys.executable, cmd_arm_joint_script,
            "--arm", motion_control_arm,
            "--target=" + target_str,  # 使用 = 形式避免参数解析歧义
            "--hold",
            "--duration_to_start", str(args.duration_to_start),
        ]
    elif args.joint == 0:
        # 关节 0：全关节 0°
        cmd_args = [
            sys.executable, cmd_arm_joint_script,
            "--arm", motion_control_arm,
            "--joint", "1",
            "--start", "0",
            "--end", "0",
            "--duration_segment", str(args.duration_segment),
        ]
    else:
        # 关节 1-7：单关节往复
        if args.start is None or args.end is None:
            default_range = JOINT_DEFAULT_ANGLE_DEG.get(args.joint)
            if default_range is None:
                rospy.logerr("[run_data_collection] 未为关节 %s 配置默认角度范围", args.joint)
                return 1
            start_deg, end_deg = default_range
        else:
            start_deg, end_deg = args.start, args.end
        cmd_args = [
            sys.executable, cmd_arm_joint_script,
            "--arm", motion_control_arm,
            "--joint", motion_control_joint,
            "--start", str(start_deg),
            "--end", str(end_deg),
            "--duration_segment", str(args.duration_segment),
            "--base_pose", ",".join(str(x) for x in SWEEP_BASE_POSE_DEG),
        ]

    output_dir = args.output_dir if args.output_dir is not None else os.path.join(_script_dir, args.workspace, "raw_data_motion", "joint_%s" % motion_control_joint)

    mocap_file = "raw_poses.csv"
    sensor_file = "raw_sensor.csv"
    csv_path = os.path.join(output_dir, mocap_file)
    sensor_csv_path = os.path.join(output_dir, sensor_file)

    # 关节控制子进程：不再屏蔽 stderr，便于排查“关节不动”等问题
    arm_joint_proc = subprocess.Popen(
        cmd_args,
        stdout=subprocess.DEVNULL,
        stderr=None,  # 直接继承终端，可见 cmd_arm_joint 报错
    )
    rospy.loginfo(
        "[run_data_collection] 已启动关节控制: 关节=%s, 输出=%s（需机器人控制栈运行，手臂才会运动）",
        motion_control_joint, output_dir,
    )
    time.sleep(args.record_delay)

    try:
        rospy.init_node("run_data_collection", anonymous=True)
        # 订阅刚体位姿与传感器话题，使用 functions 中 callback
        rospy.Subscriber("/sensors_data_raw", sensorsData, _sensor_data_callback, queue_size=10)
        rospy.Subscriber("joint_1", PoseStamped, joint_1_callback, queue_size=10)
        rospy.Subscriber("joint_2", PoseStamped, joint_2_callback, queue_size=10)
        rospy.Subscriber("joint_3", PoseStamped, joint_3_callback, queue_size=10)
        rospy.Subscriber("joint_4", PoseStamped, joint_4_callback, queue_size=10)
        rospy.Subscriber("joint_5", PoseStamped, joint_5_callback, queue_size=10)
        rospy.Subscriber("joint_6", PoseStamped, joint_6_callback, queue_size=10)
        rospy.Subscriber("joint_7", PoseStamped, joint_7_callback, queue_size=10)
        rospy.Subscriber("joint_end", PoseStamped, joint_end_callback, queue_size=10)
        rospy.Subscriber("belly_pose", PoseStamped, belly_pose_callback, queue_size=10)
        rospy.loginfo("[run_data_collection] 已订阅 joint_1~7、joint_end、belly_pose 及 /sensors_data_raw（左臂 7 关节），等待数据...")
        rospy.sleep(8.0)

        os.makedirs(output_dir, exist_ok=True)
        rate = rospy.Rate(args.record_hz)
        start_time = rospy.Time.now()
        frame_count = 0

        rospy.loginfo("[run_data_collection] 开始记录 -> %s, %s", csv_path, sensor_csv_path)
        try:
            with open(csv_path, "w", newline="") as f_mocap, open(sensor_csv_path, "w", newline="") as f_sensor:
                writer_mocap = csv.writer(f_mocap)
                writer_sensor = csv.writer(f_sensor)
                writer_mocap.writerow(_mocap_csv_header())
                writer_sensor.writerow(_sensor_csv_header())
                while not rospy.is_shutdown():
                    elapsed = (rospy.Time.now() - start_time).to_sec()
                    rigid_body_poses = get_all_rigid_body_poses()
                    # 超时退出已屏蔽：无数据时继续等待，不退出
                    # if elapsed >= RECORD_TIMEOUT_SEC and not rigid_body_poses:
                    #     rospy.logerr("[run_data_collection] 超时：%s s 内未收到话题数据", RECORD_TIMEOUT_SEC)
                    #     return 1
                    if frame_count >= args.max_frames:
                        rospy.loginfo("[run_data_collection] 已记录 %s 帧，结束", args.max_frames)
                        break
                    left_arm_joints = _get_left_arm_joints()
                    record_stamp = rospy.Time.now().to_sec()
                    frame_count += 1
                    writer_mocap.writerow(_mocap_row(rigid_body_poses, record_stamp))
                    writer_sensor.writerow(_sensor_row(record_stamp, left_arm_joints))
                    if frame_count % 500 == 0:
                        rospy.loginfo("[run_data_collection] 已记录 %s 帧...", frame_count)
                    rate.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo("[run_data_collection] 用户中断 (Ctrl+C)，记录已停止")
    finally:
        # 退出时终止单关节控制子进程，避免手臂继续运动
        if arm_joint_proc.poll() is None:
            arm_joint_proc.terminate()
            try:
                arm_joint_proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                arm_joint_proc.kill()
            rospy.loginfo("[run_data_collection] 已终止关节控制子进程")

    rospy.loginfo("[run_data_collection] 已保存 %s, %s", csv_path, sensor_csv_path)
    _plot_rigid_body_poses(csv_path)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except rospy.ROSInterruptException:
        pass
