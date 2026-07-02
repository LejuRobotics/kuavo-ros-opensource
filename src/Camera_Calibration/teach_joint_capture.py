#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
示教式关节采集：在 0-torque / 拖动示教过程中，用回车记录一帧传感器关节角；输入 s 回车保存并退出。

依赖：已 source 工作空间；机器人发布 /sensors_data_raw（kuavo_msgs/sensorsData）。

关节切片约定（单位：弧度，与仓库内 sensorsData 常用用法一致）：
  - 腰部 yaw：joint_q[12]（~waist_index）
  - 左臂 7 关节：joint_q[13:20]
  - 右臂 7 关节：joint_q[20:27]
  - 头部：默认取 joint_q 末尾两维 yaw/pitch（与 head_joint_state_bridge 一致）；
    若机型为 ROBAN 等使用 joint_q[21:23] 作为头且与臂索引不冲突，可设 ~use_head_tail_two:=false 并调 ~head_indices

默认要求 joint_q 长度 >= 29（保证臂之后仍有头部两维）。结果目录：本脚本同级的 teach_capture_output/。

  # 工作空间 source 后：
  python3 src/Camera_Calibration/teach_joint_capture.py
  # 或指定保存路径：
  python3 src/Camera_Calibration/teach_joint_capture.py _output_dir:=/path/to/dir
"""

from __future__ import print_function

import json
import os
import sys
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

import rospy
from kuavo_msgs.msg import sensorsData


# 默认与 cmd_arm_joint、head_joint_state_bridge 一致（臂后仍有 head 两维时需 len>=29）
DEFAULT_IDX_WAIST = 12
DEFAULT_SLICE_LEFT = (13, 20)
DEFAULT_SLICE_RIGHT = (20, 27)
DEFAULT_SLICE_HEAD = (21, 23)  # 仅 use_head_tail_two:=false 时使用


def _pick_mode_interactive() -> str:
    print("")
    print("========== teach_joint_capture 模式选择 ==========")
    print("  1) 头部示教（只记录 head yaw/pitch）")
    print("  2) 左手示教（只记录 left_arm_joints 7 关节）")
    print("  3) 右手示教（只记录 right_arm_joints 7 关节）")
    print("===============================================")
    while True:
        sys.stdout.write("请选择 1/2/3 并回车: ")
        sys.stdout.flush()
        line = sys.stdin.readline()
        if not line:
            return "head"
        s = line.strip()
        if s == "1":
            return "head"
        if s == "2":
            return "left"
        if s == "3":
            return "right"
        print("输入无效，请输入 1/2/3。")


def _pick_purpose_interactive() -> str:
    print("")
    print("========== 采集用途选择 ==========")
    print("  1) 用于优化（生成 teach_*_joint.json）")
    print("  2) 用于测试（生成 teach_*_joint_test.json）")
    print("=================================")
    while True:
        sys.stdout.write("请选择 1/2 并回车: ")
        sys.stdout.flush()
        line = sys.stdin.readline()
        if not line:
            return "optimize"
        s = line.strip()
        if s == "1":
            return "optimize"
        if s == "2":
            return "test"
        print("输入无效，请输入 1/2。")


class TeachJointCapture(object):
    def __init__(self, mode: str, purpose: str):
        self._lock = threading.Lock()
        self._last_q: Optional[List[float]] = None
        self._last_rospy_time = None
        self.samples: List[Dict[str, Any]] = []
        self._mode = str(mode).strip().lower()
        if self._mode not in ("head", "left", "right"):
            raise ValueError("Unsupported mode: %s" % self._mode)

        self._purpose = str(purpose).strip().lower()
        if self._purpose not in ("optimize", "test"):
            raise ValueError("Unsupported purpose: %s" % self._purpose)

        self._topic = rospy.get_param("~sensor_topic", "/sensors_data_raw")
        self._min_len = int(rospy.get_param("~min_joint_q_length", 29))
        self._idx_waist = int(rospy.get_param("~waist_index", DEFAULT_IDX_WAIST))
        self._sl_l = tuple(rospy.get_param("~left_arm_indices", list(DEFAULT_SLICE_LEFT)))
        self._sl_r = tuple(rospy.get_param("~right_arm_indices", list(DEFAULT_SLICE_RIGHT)))
        self._use_head_tail = bool(rospy.get_param("~use_head_tail_two", True))
        self._head_idx = tuple(rospy.get_param("~head_indices", list(DEFAULT_SLICE_HEAD)))

        script_dir = os.path.dirname(os.path.abspath(__file__))
        self._out_dir = rospy.get_param("~output_dir", os.path.join(script_dir, "teach_capture_output"))

        self._sub = rospy.Subscriber(self._topic, sensorsData, self._cb, queue_size=10)

    def _cb(self, msg: sensorsData):
        if not hasattr(msg, "joint_data") or not hasattr(msg.joint_data, "joint_q"):
            return
        q = [float(x) for x in msg.joint_data.joint_q]
        with self._lock:
            self._last_q = q
            self._last_rospy_time = rospy.Time.now()

    def _snapshot(self) -> Optional[Dict[str, Any]]:
        with self._lock:
            if self._last_q is None:
                return None
            q = list(self._last_q)
        if len(q) < self._min_len:
            rospy.logwarn(
                "joint_q 长度 %d < %d，无法按默认切片采集，请检查机型或调小 ~min_joint_q_length / 索引参数",
                len(q),
                self._min_len,
            )
            return None

        snap: Dict[str, Any] = {"unit": "rad", "joint_q_length": len(q)}

        if self._mode == "left":
            lo_l, hi_l = int(self._sl_l[0]), int(self._sl_l[1])
            left = [float(q[i]) for i in range(lo_l, hi_l)]
            snap["left_arm_joints"] = left
            return snap

        if self._mode == "right":
            lo_r, hi_r = int(self._sl_r[0]), int(self._sl_r[1])
            right = [float(q[i]) for i in range(lo_r, hi_r)]
            snap["right_arm_joints"] = right
            return snap

        # head
        if self._use_head_tail:
            head_yaw = float(q[-2])
            head_pitch = float(q[-1])
            head_note = "from joint_q[-2], joint_q[-1]"
        else:
            h0, h1 = int(self._head_idx[0]), int(self._head_idx[1])
            head_yaw = float(q[h0])
            head_pitch = float(q[h1])
            head_note = "from joint_q[%d], joint_q[%d]" % (h0, h1)
        snap["head_yaw"] = head_yaw
        snap["head_pitch"] = head_pitch
        snap["_head_source"] = head_note
        return snap

    def record_one(self) -> None:
        snap = self._snapshot()
        if snap is None:
            print("[teach_joint_capture] 未记录：尚无有效 joint 数据或长度不足。")
            return
        idx = len(self.samples)
        snap["index"] = idx
        self.samples.append(snap)
        if self._mode == "head":
            print(
                "[teach_joint_capture] 已记录第 %d 帧：头(yaw,pitch)=(%.4f, %.4f) rad"
                % (idx + 1, snap["head_yaw"], snap["head_pitch"])
            )
        elif self._mode == "left":
            print(
                "[teach_joint_capture] 已记录第 %d 帧：左臂[0]=%.4f rad"
                % (idx + 1, snap["left_arm_joints"][0] if snap.get("left_arm_joints") else float("nan"))
            )
        else:
            print(
                "[teach_joint_capture] 已记录第 %d 帧：右臂[0]=%.4f rad"
                % (idx + 1, snap["right_arm_joints"][0] if snap.get("right_arm_joints") else float("nan"))
            )

    def save_and_exit(self) -> None:
        os.makedirs(self._out_dir, exist_ok=True)
        suffix = "_test" if self._purpose == "test" else ""
        if self._mode == "head":
            out_name = f"teach_head_joint{suffix}.json"
        elif self._mode == "left":
            out_name = f"teach_left_joint{suffix}.json"
        else:
            out_name = f"teach_right_joint{suffix}.json"
        out_path = os.path.join(self._out_dir, out_name)
        payload = {
            "description": "Teach capture (%s, %s) from /sensors_data_raw joint_q" % (self._mode, self._purpose),
            "purpose": self._purpose,
            "sensor_topic": self._topic,
            "indices_note": {
                "waist_index": self._idx_waist,
                "left_arm_range": list(self._sl_l),
                "right_arm_range": list(self._sl_r),
                "head_mode": "tail_two" if self._use_head_tail else list(self._head_idx),
            },
            "sample_count": len(self.samples),
            "samples": self.samples,
        }
        with open(out_path, "w", encoding="utf-8") as f:
            json.dump(payload, f, ensure_ascii=False, indent=2, sort_keys=False)

        print("[teach_joint_capture] 已保存 %d 帧 -> %s" % (len(self.samples), out_path))
        rospy.signal_shutdown("saved")


def _stdin_loop(capture: TeachJointCapture) -> None:
    print("")
    print("========== teach_joint_capture ==========")
    if capture._mode == "head":
        print("  回车：记录当前一帧（头部 yaw/pitch）")
    elif capture._mode == "left":
        print("  回车：记录当前一帧（左臂 7 关节）")
    else:
        print("  回车：记录当前一帧（右臂 7 关节）")
    print("  输入 s 回车：保存 JSON 并退出")
    print("  （请确保已启动机器人并发布 %s）" % capture._topic)
    print("==========================================")
    print("")
    while not rospy.is_shutdown():
        try:
            line = sys.stdin.readline()
        except EOFError:
            break
        if line is None:
            break
        cmd = line.strip().lower()
        if cmd == "s":
            capture.save_and_exit()
            break
        if cmd == "":
            capture.record_one()
        else:
            print("[teach_joint_capture] 仅支持：直接回车记录，或 s + 回车保存退出。")


def main() -> None:
    rospy.init_node("teach_joint_capture", disable_signals=False)
    mode = rospy.get_param("~mode", "").strip().lower()
    if mode not in ("head", "left", "right"):
        mode = _pick_mode_interactive()
    purpose = rospy.get_param("~purpose", "").strip().lower()
    if purpose not in ("optimize", "test"):
        purpose = _pick_purpose_interactive()
    cap = TeachJointCapture(mode=mode, purpose=purpose)
    # 等首帧
    timeout = float(rospy.get_param("~wait_sensor_timeout", 30.0))
    t0 = time.time()
    rate = rospy.Rate(50)
    while not rospy.is_shutdown() and (time.time() - t0) < timeout:
        with cap._lock:
            ok = cap._last_q is not None and len(cap._last_q) >= cap._min_len
        if ok:
            break
        rate.sleep()
    if rospy.is_shutdown():
        return
    with cap._lock:
        ok = cap._last_q is not None and len(cap._last_q) >= cap._min_len
    if not ok:
        rospy.logerr(
            "超时未收到有效 sensorsData（topic=%s, 需要 joint_q 长度 >= %d）",
            cap._topic,
            cap._min_len,
        )
        return

    th = threading.Thread(target=_stdin_loop, args=(cap,))
    th.daemon = True
    th.start()
    rospy.spin()
    th.join(timeout=2.0)


if __name__ == "__main__":
    main()
