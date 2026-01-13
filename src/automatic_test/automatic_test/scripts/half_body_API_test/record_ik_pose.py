#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import json
import time
import argparse
from threading import Lock

import rospy
from kuavo_msgs.msg import sensorsData
import tf
import rospkg

DEFAULT_OUT = rospkg.RosPack().get_path('automatic_test') + "/scripts/half_body_API_test/actions/ik_pose_samples.jsonl"


class ArmPoseRecorder:
    def __init__(self, out_path: str, samples: int = 10, rate_hz: int = 10):
        self._out_path = out_path
        self._samples = samples
        self._rate_hz = rate_hz
        self._lock = Lock()
        self._last_q_arm = None  # 14 关节，单位：弧度（从 /sensors_data_raw 直接拿）

        # 订阅传感器原始数据
        self._sub = rospy.Subscriber("/sensors_data_raw", sensorsData, self._sensors_cb, queue_size=1)

        # TF 监听器
        self._tf_listener = tf.TransformListener()
        self._collected = 0

    def _sensors_cb(self, msg: sensorsData):
        try:
            # joint_q 为全身，索引 12~25 为手臂，共 14 个，单位：弧度
            q = msg.joint_data.joint_q
            if len(q) >= 26:
                q_arm = q[12:26]
                if len(q_arm) == 14:
                    with self._lock:
                        self._last_q_arm = list(q_arm)
        except Exception as e:
            rospy.logwarn(f"sensors_data_raw callback error: {e}")

    def _get_latest_q_arm(self):
        with self._lock:
            return None if self._last_q_arm is None else list(self._last_q_arm)

    def _ensure_out_dir(self):
        d = os.path.dirname(self._out_path)
        if d and not os.path.exists(d):
            os.makedirs(d, exist_ok=True)
    
    def _get_existing_count(self) -> int:
        if not os.path.exists(self._out_path):
            return 0
        try:
            with open(self._out_path, 'r', encoding='utf-8') as f:
                return sum(1 for _ in f if _.strip())
        except Exception:
            return 0

    def _lookup_pose(self, target_frame: str, source_frame: str, timeout_s: float = 0.8):
        start = time.time()
        while (time.time() - start) < timeout_s and not rospy.is_shutdown():
            try:
                self._tf_listener.waitForTransform(target_frame, source_frame, rospy.Time(0), rospy.Duration(0.1))
                (trans, rot) = self._tf_listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
                return (list(trans), list(rot))
            except Exception:
                rospy.sleep(0.02)
        return None

    def run_capture(self, base_frame: str = "base_link", right_ee_frame: str = "zarm_r7_end_effector", left_ee_frame: str = "zarm_l7_end_effector"):
        self._ensure_out_dir()
        rate = rospy.Rate(self._rate_hz)

        # 等待第一帧
        rospy.loginfo("等待 /sensors_data_raw 第一帧...")
        start_wait = time.time()
        while not rospy.is_shutdown():
            if self._get_latest_q_arm() is not None:
                break
            if time.time() - start_wait > 5.0:
                rospy.logwarn("5s 内未收到 /sensors_data_raw，继续等待...")
                start_wait = time.time()
            rate.sleep()

        rospy.loginfo(f"开始采集 {self._samples} 条样本，频率 {self._rate_hz} Hz，输出文件: {self._out_path}")

        base_index = self._get_existing_count()
        with open(self._out_path, 'a', encoding='utf-8') as f:
            try:
                while not rospy.is_shutdown() and self._collected < self._samples:
                    q_arm = self._get_latest_q_arm()
                    if q_arm is None:
                        rate.sleep()
                        continue

                    idx_now = base_index + self._collected
                    try:
                        # 通过 TF 读取末端位姿
                        tf_r = self._lookup_pose(base_frame, right_ee_frame, timeout_s=0.8)
                        tf_l = self._lookup_pose(base_frame, left_ee_frame, timeout_s=0.8)
                        if tf_r is None or tf_l is None:
                            rospy.logwarn("TF 暂不可用，跳过本次样本")
                            rate.sleep()
                            continue
                        (r_trans, r_rot) = tf_r
                        (l_trans, l_rot) = tf_l

                        record = {
                            "index": int(idx_now),
                            "q_arm": q_arm,  # 弧度
                            "left": {
                                "pos_xyz": list(l_trans),
                                "quat_xyzw": list(l_rot),
                            },
                            "right": {
                                "pos_xyz": list(r_trans),
                                "quat_xyzw": list(r_rot),
                            },
                        }
                        f.write(json.dumps(record, ensure_ascii=False) + "\n")
                        f.flush()
                        self._collected += 1
                        rospy.loginfo(f"采集进度: {self._collected}/{self._samples}")
                    except Exception as e:
                        rospy.logerr(f"采集或写入异常: {e}")

                    rate.sleep()
            except KeyboardInterrupt:
                rospy.loginfo("收到中断信号，准备退出并保存已采集数据...")
            finally:
                # 文件使用 with 自动关闭；逐条 flush 已确保落盘
                pass

        rospy.loginfo(f"采集结束，已写入 {self._collected} 条到 {self._out_path}")


def parse_args():
    parser = argparse.ArgumentParser(description="记录手臂 TF 位姿作为 IK 输入样本（JSON Lines）")
    parser.add_argument("--out", type=str, default=DEFAULT_OUT, help="输出文件路径（.jsonl）")
    parser.add_argument("--samples", type=int, default=10, help="采样数量，默认10")
    parser.add_argument("--rate", type=int, default=10, help="采样频率Hz，默认10")
    parser.add_argument("--base_frame", type=str, default="base_link", help="基坐标系名")
    parser.add_argument("--right_ee_frame", type=str, default="zarm_r7_end_effector", help="右手末端坐标系名")
    parser.add_argument("--left_ee_frame", type=str, default="zarm_l7_end_effector", help="左手末端坐标系名")
    return parser.parse_args(rospy.myargv(argv=sys.argv)[1:])


if __name__ == "__main__":
    rospy.init_node("record_ik_pose_node", anonymous=True)
    args = parse_args()
    recorder = ArmPoseRecorder(out_path=args.out, samples=args.samples, rate_hz=args.rate)
    try:
        recorder.run_capture(base_frame=args.base_frame, right_ee_frame=args.right_ee_frame, left_ee_frame=args.left_ee_frame)
    except rospy.ROSInterruptException:
        pass

