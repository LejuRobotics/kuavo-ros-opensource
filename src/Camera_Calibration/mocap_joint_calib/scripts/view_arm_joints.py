#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
实时查看手臂关节角度（S45/S52/S56/S62/S63）——每秒终端打印一次。

订阅 /sensors_data_raw，按 joint_q 索引布局提取左右臂各 7 关节（弧度→度）。
按 ROBOT_VERSION 对应的 kuavo.json 自动推导关节索引；无腰机型不显示 waist。

用法：
    ROBOT_VERSION=62 python3 view_arm_joints.py  # L=[4,11), R=[11,18)，无腰
    python3 view_arm_joints.py --robot-version 56
    python3 view_arm_joints.py --waist 12 --left 13 --right 20  # 手动指定
    python3 view_arm_joints.py --interval 1.0   # 打印间隔秒数（默认 1.0）
    python3 view_arm_joints.py --joints zarm_l3 zarm_l4   # 只打印指定关节
    python3 view_arm_joints.py --joints l3 r4             # 简写同样可用
"""

import argparse
import json
import math
import os
from pathlib import Path
from typing import List, Optional

import rospy
from kuavo_msgs.msg import sensorsData

_deg = lambda r: float(r) * 180.0 / math.pi  # noqa: E731

LEFT_NAMES = ["zarm_l1_joint", "zarm_l2_joint", "zarm_l3_joint", "zarm_l4_joint",
              "zarm_l5_joint", "zarm_l6_joint", "zarm_l7_joint"]
RIGHT_NAMES = ["zarm_r1_joint", "zarm_r2_joint", "zarm_r3_joint", "zarm_r4_joint",
               "zarm_r5_joint", "zarm_r6_joint", "zarm_r7_joint"]


def _normalize_joint_names(names: List[str]) -> List[tuple]:
    """把 'zarm_l3' / 'l3' / 'zarm_l3_joint' 统一成 (完整关节名, 组内索引, 臂) 三元组。"""
    out: List[tuple] = []
    for raw in names:
        n = str(raw).strip().lower().replace("_joint", "").replace("zarm_", "")
        if len(n) != 2 or n[0] not in "lr" or not n[1].isdigit():
            rospy.logwarn("跳过无法识别的关节名: %s（格式如 zarm_l3 / l3 / zarm_l3_joint）", raw)
            continue
        arm = n[0]
        k = int(n[1])
        if not (1 <= k <= 7):
            rospy.logwarn("跳过越界关节: %s（臂关节 1..7）", raw)
            continue
        names_list = LEFT_NAMES if arm == "l" else RIGHT_NAMES
        out.append((names_list[k - 1], k - 1, arm))
    return out


def _indices_for_version(version: str) -> tuple:
    config = (Path(__file__).resolve().parents[3] / "kuavo_assets" / "config"
              / f"kuavo_v{version}" / "kuavo.json")
    if not config.is_file():
        raise ValueError(f"找不到机型配置: {config}")
    data = json.loads(config.read_text(encoding="utf-8"))
    total = int(data["NUM_JOINT"])
    arm_count = int(data["NUM_ARM_JOINT"])
    head_count = int(data.get("NUM_HEAD_JOINT", 0))
    waist_count = int(data.get("NUM_WAIST_JOINT", 0))
    if arm_count != 14 or waist_count not in (0, 1):
        raise ValueError(f"ROBOT_VERSION={version} 不是脚本支持的 14 轴双臂布局")
    left = total - arm_count - head_count
    right = left + arm_count // 2
    waist = left - waist_count if waist_count else None
    return waist, left, right


class ArmJointViewer:
    def __init__(self, topic: str, waist: Optional[int], left_lo: int, right_lo: int,
                 joints: Optional[List[str]] = None) -> None:
        self._waist = int(waist) if waist is not None else None
        self._left_lo = int(left_lo)
        self._right_lo = int(right_lo)
        self._min_len = max(self._right_lo + 7, self._left_lo + 7,
                            self._waist + 1 if self._waist is not None else 0)
        self._last = None
        self._selected = _normalize_joint_names(joints) if joints else None
        self._sub = rospy.Subscriber(topic, sensorsData, self._cb, queue_size=10)

    def _cb(self, msg: sensorsData) -> None:
        self._last = msg

    def run(self, interval: float) -> None:
        scope = ", ".join(f"{name}@{self._left_lo + i if arm == 'l' else self._right_lo + i}"
                          for name, i, arm in (self._selected or []))
        rospy.loginfo("view_arm_joints: topic=%s interval=%.2fs selected=[%s]",
                      self._sub.resolved_name, interval, scope or "全部14关节")
        rate = rospy.Rate(1.0 / interval if interval > 0 else 1.0)
        while not rospy.is_shutdown():
            if self._last is not None:
                q = list(self._last.joint_data.joint_q)
                if len(q) >= self._min_len:
                    if self._selected:
                        parts = [f"{name}={_deg(q[self._left_lo + i if arm == 'l' else self._right_lo + i]):+8.2f}°"
                                 for name, i, arm in self._selected]
                    else:
                        parts = ([f"waist={_deg(q[self._waist]):+8.2f}°"]
                                 if self._waist is not None else [])
                        parts += [f"L{i+1}={_deg(q[self._left_lo + i]):+8.2f}°" for i in range(7)]
                        parts += [f"R{i+1}={_deg(q[self._right_lo + i]):+8.2f}°" for i in range(7)]
                    print("  ".join(parts), flush=True)
                else:
                    print(f"[warn] joint_q 长度 {len(q)} < {self._min_len}", flush=True)
            else:
                print("[warn] 尚未收到 /sensors_data_raw", flush=True)
            rate.sleep()


def main() -> int:
    parser = argparse.ArgumentParser(description="按机型每秒打印手臂关节角度")
    parser.add_argument("--topic", default="/sensors_data_raw")
    parser.add_argument("--robot-version", default=os.environ.get("ROBOT_VERSION", ""),
                        help="机型；默认取 ROBOT_VERSION，自动读取 kuavo.json")
    parser.add_argument("--waist", type=int, help="显式覆盖腰关节索引；无腰机型勿传")
    parser.add_argument("--left", type=int, help="显式覆盖左臂起始索引")
    parser.add_argument("--right", type=int, help="显式覆盖右臂起始索引")
    parser.add_argument("--interval", type=float, default=1.0)
    parser.add_argument(
        "--joints", nargs="+", default=None,
        help="只打印指定关节（如 zarm_l3 zarm_l4 或 l3 r4；缺省打印全部14关节）",
    )
    args = parser.parse_args()

    try:
        waist, left, right = _indices_for_version(args.robot_version) if args.robot_version else (None, None, None)
    except (OSError, ValueError, KeyError) as exc:
        parser.error(str(exc))
    if args.waist is not None:
        waist = args.waist
    if args.left is not None:
        left = args.left
    if args.right is not None:
        right = args.right
    if left is None or right is None:
        parser.error("请设置 ROBOT_VERSION、传 --robot-version，或同时指定 --left 和 --right")
    if min(left, right) < 0 or right < left + 7 or (waist is not None and waist < 0):
        parser.error("关节索引无效或左右臂索引重叠")

    rospy.init_node("view_arm_joints", anonymous=True)
    rospy.loginfo("关节索引: ROBOT_VERSION=%s waist=%s left=%d right=%d",
                  args.robot_version or "manual", waist, left, right)
    ArmJointViewer(args.topic, waist, left, right, args.joints).run(args.interval)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
