#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
将手臂“关节4”打到 -90° 的最小控制脚本（通过 /kuavo_arm_traj）。

背景/约定（来自 docs/运动控制API.md 与 humanoidController.cpp）：
- 发布话题: /kuavo_arm_traj
- 消息类型: sensor_msgs/JointState
- 单位: degree（控制器内部会转为 rad）
- 维度: 14（前 7 个为左臂，后 7 个为右臂）
- 重要: 控制器会检查 msg.name.size() == 14（name 内容本身不重要，但长度必须对）

默认行为：
- 尝试从 /joint_states 读取当前 14 个上肢关节（常见命名 l_arm_* / r_arm_*），读取成功则“只改关节4，其余保持当前”；
- 若读取失败，则使用一个安全的默认初始姿态（与校准脚本一致），再将关节4改到 -90°；
- 默认控制左右两臂的关节4（--arm both）。你也可以只控制单臂。
"""

from __future__ import annotations

import argparse
import math
import time
from typing import List, Optional

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

try:
    from kuavo_msgs.srv import changeArmCtrlMode
except Exception:
    changeArmCtrlMode = None


# 常见的上肢关节命名（在 calibration 脚本中使用过）
ARM_NAMES_LR = [
    "l_arm_pitch",
    "l_arm_roll",
    "l_arm_yaw",
    "l_forearm_pitch",  # ← 关节4（左臂）
    "l_hand_yaw",
    "l_hand_pitch",
    "l_hand_roll",
    "r_arm_pitch",
    "r_arm_roll",
    "r_arm_yaw",
    "r_forearm_pitch",  # ← 关节4（右臂）
    "r_hand_yaw",
    "r_hand_pitch",
    "r_hand_roll",
]

# 某些 demo/工具里会用 joint1..joint14 作为 name（控制器只看长度）
ARM_NAMES_JOINT1_14 = [f"joint{i}" for i in range(1, 15)]

# 某些模型/仿真环境可能用 zarm_* 命名（示例见 src/kuavo-isaac-sim/.../arm_control_demo.py）
ARM_NAMES_ZARM = [
    "zarm_l1_joint",
    "zarm_l2_joint",
    "zarm_l3_joint",
    "zarm_l4_joint",
    "zarm_l5_joint",
    "zarm_l6_joint",
    "zarm_l7_joint",
    "zarm_r1_joint",
    "zarm_r2_joint",
    "zarm_r3_joint",
    "zarm_r4_joint",
    "zarm_r5_joint",
    "zarm_r6_joint",
    "zarm_r7_joint",
]

# 校准脚本里使用的默认初始关节（degree）
DEFAULT_INIT_ARM_POS_DEG = [20,  0,  0,  -30,   0,   0,  0,
                            20,  0,  0,  -30,   0,   0,  0,
]

# 默认期望关节角（degree），方便用户在脚本顶部集中修改
# - 这里给出完整的 14 维关节角：[左臂7关节，右臂7关节]
# - 如需一次性设定左右臂所有关节，只需修改下面这一行
DEFAULT_TARGET_DEG: List[float] = [
    0.0,   0.0,   0.0,  -90.0,  0.0,  0.0,  0.0,   # left arm  7 joints
    0.0,   0.0,   0.0,  -90.0,  0.0,  0.0,  0.0,   # right arm 7 joints
]


def try_switch_arm_ctrl_mode(mode: int, timeout_s: float = 0.5) -> bool:
    """尝试调用 humanoid_change_arm_ctrl_mode（存在则切换；不存在则忽略）。"""
    if changeArmCtrlMode is None:
        return False
    service_name = "humanoid_change_arm_ctrl_mode"
    try:
        rospy.wait_for_service(service_name, timeout=timeout_s)
        proxy = rospy.ServiceProxy(service_name, changeArmCtrlMode)
        proxy(control_mode=mode)
        rospy.loginfo("[joint4] switched %s to mode=%d", service_name, mode)
        return True
    except Exception as e:
        rospy.logwarn("[joint4] skip switching arm ctrl mode (%s): %s", service_name, e)
        return False


def _maybe_rad_to_deg(values: List[float]) -> List[float]:
    """如果看起来像 rad（最大绝对值 <= 2*pi+0.5），则转为 deg；否则认为已是 deg。"""
    if not values:
        return values
    max_abs = max(abs(v) for v in values)
    if max_abs <= (2.0 * math.pi + 0.5):
        return [math.degrees(v) for v in values]
    return values


def _extract_arm_positions_deg_from_joint_states(
    msg: JointState,
    ordered_names: List[str],
) -> Optional[List[float]]:
    """从 /joint_states 提取 ordered_names 对应的关节角（deg）。找不到则返回 None。"""
    if not msg.name or not msg.position or len(msg.name) != len(msg.position):
        return None
    name_to_pos = dict(zip(msg.name, msg.position))
    if not all(n in name_to_pos for n in ordered_names):
        return None
    vals = [float(name_to_pos[n]) for n in ordered_names]
    return _maybe_rad_to_deg(vals)


def get_current_arm_deg(timeout_s: float = 1.0) -> Optional[List[float]]:
    """尝试在 timeout 内从 /joint_states 读取一帧上肢 14 关节（deg）。"""
    last_msg = {"msg": None}

    def cb(m: JointState):
        last_msg["msg"] = m

    sub = rospy.Subscriber("/joint_states", JointState, cb, queue_size=1)
    t0 = time.time()
    rate = rospy.Rate(50)
    while not rospy.is_shutdown() and (time.time() - t0) < timeout_s:
        if last_msg["msg"] is not None:
            m = last_msg["msg"]
            for names in (ARM_NAMES_LR, ARM_NAMES_ZARM):
                q = _extract_arm_positions_deg_from_joint_states(m, names)
                if q is not None and len(q) == 14:
                    sub.unregister()
                    return q
        rate.sleep()
    sub.unregister()
    return None


def build_target_positions(
    arm: str,
    joint_index_1_based: int,
    target_deg: float,
    base_positions_deg: List[float],
) -> List[float]:
    if len(base_positions_deg) != 14:
        raise ValueError(f"base_positions_deg must be length 14, got {len(base_positions_deg)}")
    if not (1 <= joint_index_1_based <= 7):
        raise ValueError(f"joint_index must be in [1,7], got {joint_index_1_based}")
    idx0 = joint_index_1_based - 1
    q = list(base_positions_deg)

    if arm in ("left", "both"):
        q[idx0] = float(target_deg)
    if arm in ("right", "both"):
        q[7 + idx0] = float(target_deg)
    return q


def main():
    parser = argparse.ArgumentParser(description="Move arm joint4 to -90 deg via /kuavo_arm_traj")
    parser.add_argument("--arm", choices=["left", "right", "both"], default="both", help="which arm(s) to control")
    parser.add_argument("--joint", type=int, default=4, help="joint index within each arm (1..7). default=4")
    parser.add_argument("--hz", type=float, default=20.0, help="publish rate (Hz)")
    parser.add_argument("--timeout_joint_states", type=float, default=1.0, help="seconds to wait /joint_states")
    parser.add_argument("--duration", type=float, default=0.0, help="publish duration (s); 0 means auto")
    args = parser.parse_args()

    rospy.init_node("cmd_arm_joint4_to_minus90", anonymous=True)

    # 可选：切到手臂控制模式（2 为校准脚本常用）
    try_switch_arm_ctrl_mode(2)

    # 直接使用顶部 DEFAULT_TARGET_DEG 作为 14 维目标关节角（不再读取 /joint_states）
    if len(DEFAULT_TARGET_DEG) != 14:
        raise ValueError(f"DEFAULT_TARGET_DEG must have length 14, got {len(DEFAULT_TARGET_DEG)}")
    target_deg = list(DEFAULT_TARGET_DEG)
    rospy.loginfo("[joint4] use DEFAULT_TARGET_DEG (14 joints) as target: %s", target_deg)

    pub = rospy.Publisher("/kuavo_arm_traj", JointState, queue_size=1, tcp_nodelay=True)
    reach_time = {"t": 0.0}

    def reach_cb(msg: Float32):
        reach_time["t"] = float(msg.data)

    rospy.Subscriber("/lb_arm_joint_reach_time", Float32, reach_cb, queue_size=1, tcp_nodelay=True)

    # 等订阅者（避免只发一帧被吞掉）
    t0 = time.time()
    while not rospy.is_shutdown() and pub.get_num_connections() == 0 and (time.time() - t0) < 2.0:
        rospy.sleep(0.05)

    js = JointState()
    js.name = ARM_NAMES_JOINT1_14  # 关键：长度必须为 14（控制器只检查 size）
    js.velocity = [0.0] * 14
    js.effort = [0.0] * 14

    hz = max(1.0, float(args.hz))
    rate = rospy.Rate(hz)

    # 自动 duration：先连续发布 0.5s，随后如果收到 reach_time>0 则补足到 reach_time+0.5
    auto_publish_s = 0.5
    t_pub_start = time.time()
    while not rospy.is_shutdown() and (time.time() - t_pub_start) < auto_publish_s:
        js.header.stamp = rospy.Time.now()
        js.position = target_deg
        pub.publish(js)
        rate.sleep()

    if float(args.duration) > 0.0:
        remaining = float(args.duration) - (time.time() - t_pub_start)
        remaining = max(0.0, remaining)
    else:
        # 等待 reach_time 回来一点点（最多 1s）
        t_wait = time.time()
        while not rospy.is_shutdown() and reach_time["t"] <= 0.0 and (time.time() - t_wait) < 1.0:
            rospy.sleep(0.02)
        remaining = max(0.0, reach_time["t"] + 0.5 - (time.time() - t_pub_start))

    t_end = time.time() + remaining
    while not rospy.is_shutdown() and time.time() < t_end:
        js.header.stamp = rospy.Time.now()
        js.position = target_deg
        pub.publish(js)
        rate.sleep()

    rospy.loginfo(
        "[joint4] done. arm=%s joint=%d target=%.2f deg (published %.2fs)",
        args.arm,
        args.joint,
        args.deg,
        time.time() - t_pub_start,
    )


if __name__ == "__main__":
    main()


