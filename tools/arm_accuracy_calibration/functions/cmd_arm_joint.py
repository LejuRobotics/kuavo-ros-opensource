#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
手臂关节运动控制：
  1) 单关节往复：指定关节在 [start, end] 间插值循环。
  2) 全关节目标：一次性指定 7 个关节角度，插值到目标位姿。

可导入函数：
  build_positions_all(arm, deg_list)       - 构建 14 维位姿向量
  get_current_joints_deg(arm)              - 从 /joint_states 获取 7 关节角
  get_current_joints_deg_from_sensors(arm) - 从 /sensors_data_raw 获取 7 关节角（标定用）

用法：
  # 单关节往复（可选：七关节基准位姿，运动关节仍由 --start/--end 决定）
  python cmd_arm_joint.py --arm left --joint 4 --start -90 --end 90
  python cmd_arm_joint.py --arm left --joint 3 --start 50 --end -10 --base_pose "0,0,0,-90,0,0,0"
  （省略 --base_pose 时非运动关节为 0°。）

  # 全关节目标（到达后持续保持）
  python cmd_arm_joint.py --arm left --target "0,45,-30,0,0,0,0" --hold
"""
from __future__ import annotations

import argparse
import math
import time
from typing import List, Optional

import rospy
from sensor_msgs.msg import JointState

try:
    from kuavo_msgs.srv import changeArmCtrlMode
except Exception:
    changeArmCtrlMode = None
try:
    from kuavo_msgs.msg import sensorsData
except Exception:
    sensorsData = None


ARM_NAMES_JOINT1_14 = [f"joint{i}" for i in range(1, 15)]
ARM_NAMES_LR = [
    "l_arm_pitch", "l_arm_roll", "l_arm_yaw", "l_forearm_pitch",
    "l_hand_yaw", "l_hand_pitch", "l_hand_roll",
    "r_arm_pitch", "r_arm_roll", "r_arm_yaw", "r_forearm_pitch",
    "r_hand_yaw", "r_hand_pitch", "r_hand_roll",
]
ARM_NAMES_ZARM = (
    [f"zarm_l{i}_joint" for i in range(1, 8)] +
    [f"zarm_r{i}_joint" for i in range(1, 8)]
)

ARM_BASE_ZEROS = [0.0] * 14


def _call_service(name: str, mode: int, timeout_s: float = 0.5) -> bool:
    if changeArmCtrlMode is None:
        return False
    try:
        rospy.wait_for_service(name, timeout=timeout_s)
        proxy = rospy.ServiceProxy(name, changeArmCtrlMode)
        proxy(control_mode=mode)
        return True
    except Exception:
        return False


def try_switch_arm_ctrl_mode(mode: int, timeout_s: float = 0.5) -> bool:
    """humanoid_change_arm_ctrl_mode: 2=external_control"""
    return _call_service("humanoid_change_arm_ctrl_mode", mode, timeout_s)


def enable_wbc_arm_trajectory(timeout_s: float = 0.5) -> bool:
    """enable_wbc_arm_trajectory_control: 1=启用，使 WBC 使用 /kuavo_arm_traj 的 14 维关节目标"""
    return _call_service("/enable_wbc_arm_trajectory_control", 1, timeout_s)


def _maybe_rad_to_deg(values: List[float]) -> List[float]:
    if not values:
        return values
    if max(abs(v) for v in values) <= 2.0 * math.pi + 0.5:
        return [math.degrees(v) for v in values]
    return values


def get_current_joint_deg(arm: str, joint_idx: int, timeout_s: float = 1.0) -> Optional[float]:
    """从 /joint_states 读取指定关节当前角度(deg)。"""
    last_msg = {"msg": None}
    def cb(m):
        last_msg["msg"] = m
    sub = rospy.Subscriber("/joint_states", JointState, cb, queue_size=1)
    t0 = time.time()
    rate = rospy.Rate(50)
    while not rospy.is_shutdown() and (time.time() - t0) < timeout_s:
        if last_msg["msg"]:
            m = last_msg["msg"]
            if m.name and m.position and len(m.name) == len(m.position):
                name_to_pos = dict(zip(m.name, m.position))
                for names in (ARM_NAMES_LR, ARM_NAMES_ZARM):
                    if all(n in name_to_pos for n in names):
                        vals = _maybe_rad_to_deg([float(name_to_pos[n]) for n in names])
                        idx = (joint_idx - 1) if arm in ("left", "both") else (7 + joint_idx - 1)
                        sub.unregister()
                        return vals[idx]
        rate.sleep()
    sub.unregister()
    return None


def get_current_joints_deg_from_sensors(arm: str, timeout_s: float = 1.0) -> Optional[List[float]]:
    """从 /sensors_data_raw 读取指定手臂的 7 个关节当前角度(deg)。
    joint_q[13:20]=左臂, joint_q[20:27]=右臂。若为弧度则自动转角度。
    """
    if sensorsData is None:
        return None
    last_msg = {"msg": None}
    def cb(m):
        last_msg["msg"] = m
    sub = rospy.Subscriber("/sensors_data_raw", sensorsData, cb, queue_size=1)
    t0 = time.time()
    rate = rospy.Rate(50)
    while not rospy.is_shutdown() and (time.time() - t0) < timeout_s:
        if last_msg["msg"]:
            m = last_msg["msg"]
            if hasattr(m, "joint_data") and hasattr(m.joint_data, "joint_q"):
                q = list(m.joint_data.joint_q)
                if len(q) >= 27:
                    if arm == "left":
                        vals = [float(q[i]) for i in range(13, 20)]
                    elif arm == "right":
                        vals = [float(q[i]) for i in range(20, 27)]
                    else:
                        left = [float(q[i]) for i in range(13, 20)]
                        right = [float(q[i]) for i in range(20, 27)]
                        vals = [(left[i] + right[i]) / 2 for i in range(7)]
                    sub.unregister()
                    return _maybe_rad_to_deg(vals)
        rate.sleep()
    sub.unregister()
    return None


def build_positions(arm: str, joint_idx: int, deg: float) -> List[float]:
    q = list(ARM_BASE_ZEROS)
    idx = joint_idx - 1
    if arm in ("left", "both"):
        q[idx] = float(deg)
    if arm in ("right", "both"):
        q[7 + idx] = float(deg)
    return q


def build_positions_all(arm: str, deg_list: List[float]) -> List[float]:
    """一次性指定手臂所有关节角度，构建 14 维位姿向量。
    deg_list: 7 个关节角度(deg)，顺序对应 joint1~joint7。
    """
    if len(deg_list) != 7:
        raise ValueError(f"deg_list 需包含 7 个关节角，当前 {len(deg_list)} 个")
    q = list(ARM_BASE_ZEROS)
    degs = [float(d) for d in deg_list]
    if arm in ("left", "both"):
        q[0:7] = degs
    if arm in ("right", "both"):
        q[7:14] = degs
    return q


def get_current_joints_deg(arm: str, timeout_s: float = 1.0) -> Optional[List[float]]:
    """从 /joint_states 读取指定手臂的 7 个关节当前角度(deg)。"""
    last_msg = {"msg": None}
    def cb(m):
        last_msg["msg"] = m
    sub = rospy.Subscriber("/joint_states", JointState, cb, queue_size=1)
    t0 = time.time()
    rate = rospy.Rate(50)
    while not rospy.is_shutdown() and (time.time() - t0) < timeout_s:
        if last_msg["msg"]:
            m = last_msg["msg"]
            if m.name and m.position and len(m.name) == len(m.position):
                name_to_pos = dict(zip(m.name, m.position))
                for names in (ARM_NAMES_LR, ARM_NAMES_ZARM):
                    if all(n in name_to_pos for n in names):
                        vals = _maybe_rad_to_deg([float(name_to_pos[n]) for n in names])
                        if arm == "left":
                            result = vals[0:7]
                        elif arm == "right":
                            result = vals[7:14]
                        else:
                            result = [(vals[i] + vals[i + 7]) / 2 for i in range(7)]
                        sub.unregister()
                        return result
        rate.sleep()
    sub.unregister()
    return None


def _interpolate(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def _parse_target(s: str) -> List[float]:
    """解析 --target 字符串为 7 个浮点数。"""
    parts = [x.strip() for x in s.split(",")]
    if len(parts) != 7:
        raise ValueError(f"--target 需 7 个关节角(逗号分隔)，当前 {len(parts)} 个")
    return [float(x) for x in parts]


def main():
    parser = argparse.ArgumentParser(
        description="手臂关节运动：单关节往复插值 或 一次性指定所有关节目标位姿")
    parser.add_argument("--arm", choices=["left", "right", "both"], default="left")
    parser.add_argument("--joint", type=int, default=4, help="关节 1-7（仅单关节模式）")
    parser.add_argument("--start", type=float, help="起始角度(deg)（单关节模式）")
    parser.add_argument("--end", type=float, help="终止角度(deg)（单关节模式）")
    parser.add_argument(
        "--target", type=str,
        help="一次性指定 7 个关节角度(deg)，逗号分隔，如 '0,45,-30,0,0,0,0'。与 --start/--end 互斥")
    parser.add_argument("--hz", type=float, default=100.0)
    parser.add_argument("--duration_to_start", type=float, default=2.0, help="到起始点耗时(s)")
    parser.add_argument("--duration_segment", type=float, default=2.0, help="start<->end 每段耗时(s)")
    parser.add_argument("--hold", action="store_true", help="目标位姿模式下，到达后持续保持（否则只执行一次）")
    parser.add_argument(
        "--base_pose",
        type=str,
        default=None,
        help="单关节往复时七关节基准位姿(deg)，逗号分隔 7 个数；运动关节对应分量会被 --start/--end 覆盖。"
             "未指定时非运动关节为 0°。",
    )
    args = parser.parse_args()

    use_target_mode = args.target is not None
    if use_target_mode:
        target_deg_list = _parse_target(args.target)
    else:
        if args.start is None or args.end is None:
            parser.error("单关节模式需要 --start 和 --end，或使用 --target 指定所有关节")
        start_deg = args.start
        end_deg = args.end

    rospy.init_node("cmd_arm_joint_sweep", anonymous=True)
    try_switch_arm_ctrl_mode(2)   # MPC 手臂外部控制
    enable_wbc_arm_trajectory()   # 启用 WBC 使用 /kuavo_arm_traj

    pub = rospy.Publisher("/kuavo_arm_traj", JointState, queue_size=1, tcp_nodelay=True)
    t0 = time.time()
    while not rospy.is_shutdown() and pub.get_num_connections() == 0 and (time.time() - t0) < 2.0:
        rospy.sleep(0.05)

    js = JointState()
    js.name = ARM_NAMES_JOINT1_14
    js.velocity = [0.0] * 14
    js.effort = [0.0] * 14

    hz = max(1.0, args.hz)
    rate = rospy.Rate(hz)

    if use_target_mode:
        # === 全关节目标位姿模式：优先从 /sensors_data_raw 读传感器角度作为插值起点 ===
        current_list = get_current_joints_deg_from_sensors(args.arm)
        if current_list is None:
            current_list = get_current_joints_deg(args.arm)
        if current_list is None:
            current_list = [0.0] * 7
        rospy.loginfo("[cmd_arm_joint] 全关节: 当前 %s -> 目标 %s",
                      [f"{x:.1f}" for x in current_list], [f"{x:.1f}" for x in target_deg_list])

        dur = max(0.1, args.duration_to_start)
        n_steps = max(1, int(dur * hz))
        for i in range(n_steps + 1):
            if rospy.is_shutdown():
                return
            t = i / n_steps
            interp = [ _interpolate(current_list[j], target_deg_list[j], t) for j in range(7) ]
            js.header.stamp = rospy.Time.now()
            js.position = build_positions_all(args.arm, interp)
            pub.publish(js)
            rate.sleep()

        if args.hold:
            while not rospy.is_shutdown():
                js.header.stamp = rospy.Time.now()
                js.position = build_positions_all(args.arm, target_deg_list)
                pub.publish(js)
                rate.sleep()
    else:
        # === 单关节往复模式：从 sensors_data_raw 读当前 7 关节，再插值到运动起点 ===
        current_list = get_current_joints_deg_from_sensors(args.arm, timeout_s=2.0)
        if current_list is None:
            current_list = get_current_joints_deg(args.arm)
        if current_list is None or len(current_list) != 7:
            current_list = [0.0] * 7

        # 运动起点：先取七关节基准位姿，再将运动关节设为 start_deg（往复终点仅改该关节为 end_deg）
        if args.base_pose is not None:
            base_deg_list = _parse_target(args.base_pose)
            start_pose = list(base_deg_list)
            start_pose[args.joint - 1] = start_deg
        else:
            start_pose = [0.0] * 7
            start_pose[args.joint - 1] = start_deg

        rospy.loginfo("[cmd_arm_joint] 关节 %d: 规划起点 [%s]，往复 %.2f <-> %.2f",
                      args.joint,
                      ", ".join("%.1f" % x for x in start_pose),
                      start_deg, end_deg)

        # Phase 1 插值起点：在真正开始运动前再次从 sensors_data_raw 读取当前角，确保“从当前插值到起点”
        rospy.loginfo("[cmd_arm_joint] 正在读取当前关节角(sensors_data_raw)作为插值起点...")
        interp_from = get_current_joints_deg_from_sensors(args.arm, timeout_s=5.0)
        if interp_from is None:
            interp_from = get_current_joints_deg(args.arm)
        if interp_from is None or len(interp_from) != 7:
            interp_from = list(current_list)
            rospy.logwarn("[cmd_arm_joint] 未读到 sensors 当前角，使用之前读数作为插值起点")
        else:
            rospy.loginfo("[cmd_arm_joint] 当前角(插值起点) [%s] -> 规划起点 [%s]",
                          ", ".join("%.1f" % x for x in interp_from),
                          ", ".join("%.1f" % x for x in start_pose))

        def publish_pose(deg_list: List[float]):
            """发布 7 关节目标位姿（14 维）。"""
            js.header.stamp = rospy.Time.now()
            js.position = build_positions_all(args.arm, deg_list)
            pub.publish(js)

        # Phase 1: 从当前关节角(interp_from)插值到运动起点(start_pose)，全部发完后再做后续
        dur = max(0.1, args.duration_to_start)
        n_steps = max(1, int(dur * hz))
        for i in range(n_steps + 1):
            if rospy.is_shutdown():
                return
            t = i / n_steps
            interp = [_interpolate(interp_from[j], start_pose[j], t) for j in range(7)]
            publish_pose(interp)
            rate.sleep()

        # 插值完成后在起点保持一小段时间，再开始循环往复
        hold_steps = max(1, int(1.0 * hz))  # 约 1 秒
        for _ in range(hold_steps):
            if rospy.is_shutdown():
                return
            publish_pose(start_pose)
            rate.sleep()
        rospy.loginfo("[cmd_arm_joint] 插值完成，开始循环往复运动 (%.2f <-> %.2f)", start_deg, end_deg)

        # Phase 2 & 3: 往复时也始终对七个关节一起插值
        # 去程：从 start_pose 插值到 (start_pose 仅运动关节改为 end_deg)
        # 回程：从 (start_pose 仅运动关节为 end_deg) 插值回 start_pose（七关节都插值回起始位置）
        dur_seg = max(0.1, args.duration_segment)
        n_per_seg = max(1, int(dur_seg * hz))
        pose_at_end = list(start_pose)
        pose_at_end[args.joint - 1] = end_deg
        forward = True
        while not rospy.is_shutdown():
            if forward:
                # 去程：七关节从 start_pose 插值到 pose_at_end（仅运动关节从 start_deg -> end_deg）
                from_pose = start_pose
                to_pose = pose_at_end
            else:
                # 回程：七关节从 pose_at_end 插值回 start_pose（七关节都插值回起始位置）
                from_pose = pose_at_end
                to_pose = start_pose
            for i in range(n_per_seg + 1):
                if rospy.is_shutdown():
                    return
                t = i / n_per_seg
                interp = [_interpolate(from_pose[j], to_pose[j], t) for j in range(7)]
                publish_pose(interp)
                rate.sleep()
            forward = not forward


if __name__ == "__main__":
    main()
