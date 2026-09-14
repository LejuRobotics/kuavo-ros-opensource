#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Motive 动捕多刚体位姿采集：订阅 ROS 话题，同步写入 CSV。

时序：等待全部刚体有效 → 预热 warmup（不写盘）→ 正式采集 duration → 自动退出。
"""

from __future__ import print_function

import argparse
import csv
import os
import sys
import time
from typing import Any, Dict, List, Optional

import numpy as np

try:
    import rospy
    from geometry_msgs.msg import Pose, PoseStamped
except ImportError as e:
    rospy = None
    Pose = PoseStamped = None
    _ros_err = e

_script_dir = os.path.dirname(os.path.abspath(__file__))
if _script_dir not in sys.path:
    sys.path.insert(0, _script_dir)

from mocap_pose_utils import (  # noqa: E402
    ensure_quat_continuity,
    is_valid_pose,
    load_bodies_config,
)


def _default_config_path() -> str:
    return os.path.join(_script_dir, "config", "bodies.yaml")


def _csv_header(body_names: List[str]) -> List[str]:
    cols = ["timestamp"]
    for name in body_names:
        cols.extend(
            [
                f"{name}_px",
                f"{name}_py",
                f"{name}_pz",
                f"{name}_qx",
                f"{name}_qy",
                f"{name}_qz",
                f"{name}_qw",
            ]
        )
    return cols


def _pose_row(
    timestamp: float,
    body_names: List[str],
    cache: Dict[str, Dict[str, np.ndarray]],
) -> List[str]:
    row = [f"{timestamp:.6f}"]
    for name in body_names:
        d = cache[name]
        row.extend(f"{x:.8g}" for x in d["pos"])
        row.extend(f"{x:.8g}" for x in d["quat"])
    return row


class MocapPoseRecorder:
    """多刚体同步采集器。"""

    def __init__(
        self,
        config_path: str,
        output_csv: str,
        duration: float,
        warmup: float,
        wait_timeout: float,
    ):
        if rospy is None:
            raise RuntimeError(f"ROS 依赖不可用: {_ros_err}")

        cfg = load_bodies_config(config_path)
        self.bodies = cfg["bodies"]
        self.body_names = [b["name"] for b in self.bodies]
        self.output_csv = output_csv
        self.duration = duration
        self.warmup = warmup
        self.wait_timeout = wait_timeout

        # 缓存：{name: {"pos": ndarray, "quat": ndarray}}
        self._cache: Dict[str, Optional[Dict[str, np.ndarray]]] = {
            b["name"]: None for b in self.bodies
        }
        self._prev_quats: Dict[str, np.ndarray] = {}
        self._subs = []

        os.makedirs(os.path.dirname(os.path.abspath(output_csv)), exist_ok=True)
        with open(output_csv, "w", newline="") as f:
            csv.writer(f).writerow(_csv_header(self.body_names))

        for body in self.bodies:
            topic = body["topic"]
            msg_type = body.get("msg_type", "PoseStamped")
            if msg_type == "Pose":
                sub = rospy.Subscriber(
                    topic,
                    Pose,
                    lambda msg, n=body["name"]: self._on_pose(msg, n),
                    queue_size=10,
                )
            else:
                sub = rospy.Subscriber(
                    topic,
                    PoseStamped,
                    lambda msg, n=body["name"]: self._on_pose_stamped(msg, n),
                    queue_size=10,
                )
            self._subs.append(sub)
            print(f"[record] 订阅 {topic} ({msg_type}) -> {body['name']}")

    def _extract_pose(self, msg, name: str) -> None:
        if hasattr(msg, "pose"):
            p = msg.pose.position
            o = msg.pose.orientation
        else:
            p = msg.position
            o = msg.orientation
        pos = np.array([p.x, p.y, p.z], dtype=float)
        quat = np.array([o.x, o.y, o.z, o.w], dtype=float)
        if not is_valid_pose(pos, quat):
            return
        prev = self._prev_quats.get(name)
        quat = ensure_quat_continuity(quat, prev)
        self._prev_quats[name] = quat.copy()
        self._cache[name] = {"pos": pos, "quat": quat}

    def _on_pose_stamped(self, msg: "PoseStamped", name: str) -> None:
        self._extract_pose(msg, name)

    def _on_pose(self, msg: "Pose", name: str) -> None:
        self._extract_pose(msg, name)

    def _all_bodies_ready(self) -> bool:
        return all(self._cache[n] is not None for n in self.body_names)

    def _wait_until_ready(self) -> bool:
        print(f"[record] 等待全部刚体有效（超时 {self.wait_timeout:.0f}s）...")
        t0 = time.time()
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self._all_bodies_ready():
                print("[record] 全部刚体已就绪")
                return True
            if time.time() - t0 > self.wait_timeout:
                missing = [n for n in self.body_names if self._cache[n] is None]
                print(f"[record] 超时，缺失刚体: {missing}")
                return False
            rate.sleep()
        return False

    def run(self) -> int:
        if not self._wait_until_ready():
            return 1

        # 预热：不写 CSV
        if self.warmup > 0:
            print(f"[record] 预热 {self.warmup:.1f}s（不写 CSV）...")
            t_warm_end = time.time() + self.warmup
            rate = rospy.Rate(100)
            while time.time() < t_warm_end and not rospy.is_shutdown():
                rate.sleep()

        print(f"[record] 开始采集 {self.duration:.1f}s -> {self.output_csv}")
        t_record_end = time.time() + self.duration
        frame_count = 0
        valid_count = 0

        with open(self.output_csv, "a", newline="") as f:
            writer = csv.writer(f)
            rate = rospy.Rate(100)
            while time.time() < t_record_end and not rospy.is_shutdown():
                frame_count += 1
                if self._all_bodies_ready():
                    ts = rospy.Time.now().to_sec()
                    row = _pose_row(ts, self.body_names, self._cache)  # type: ignore
                    writer.writerow(row)
                    valid_count += 1
                    if valid_count % 100 == 0:
                        print(f"[record] 已写入 {valid_count} 帧...")
                rate.sleep()

        print(
            f"[record] 完成：总循环 {frame_count} 次，有效帧 {valid_count}，"
            f"文件 {self.output_csv}"
        )
        return 0 if valid_count > 0 else 1


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Motive 动捕多刚体位姿采集（同步写 CSV）"
    )
    parser.add_argument(
        "--config",
        default=_default_config_path(),
        help="bodies.yaml 路径",
    )
    parser.add_argument(
        "--output",
        required=True,
        help="输出 CSV 路径",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=10.0,
        help="正式采集时长（秒），到点自动退出",
    )
    parser.add_argument(
        "--warmup",
        type=float,
        default=2.0,
        help="预热时长（秒），不写 CSV",
    )
    parser.add_argument(
        "--wait_timeout",
        type=float,
        default=30.0,
        help="等待全部刚体有效的最长时间（秒）",
    )
    args = parser.parse_args()

    if rospy is None:
        print(f"ROS 不可用: {_ros_err}", file=sys.stderr)
        return 2

    rospy.init_node("record_mocap_poses", anonymous=True)
    recorder = MocapPoseRecorder(
        config_path=args.config,
        output_csv=args.output,
        duration=args.duration,
        warmup=args.warmup,
        wait_timeout=args.wait_timeout,
    )
    return recorder.run()


if __name__ == "__main__":
    raise SystemExit(main())
