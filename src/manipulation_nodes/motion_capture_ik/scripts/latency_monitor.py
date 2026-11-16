#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
[非正式入口] 现场订阅延迟话题。正式诊断请用 bag 脚本:
  python3 tools/bag_tools/extract_joint_topics_and_plot.py -b <bag> --preset q3-inc-humanoid
"""

import argparse
import os
import signal
import sys
import time
from collections import deque, OrderedDict

import rospy
import std_msgs.msg

PRINT_INTERVAL = 5.0
MAX_HISTORY = 10000

STAGE_VR = "1.VR数据处理"
STAGE_HOP = "层间.VR→IK"
STAGE_IK = "2.IK解算"
STAGE_CTRL = "3.控制器"
STAGE_ACT = "4.电机与反馈"

# (topic, name, stage, group)  group: absolute / incremental / common
QUEST3_COARSE = [
    ("/quest3/node_processing_latency_ms", "Quest3 VR数据处理", STAGE_VR, "common"),
    ("/vr_incremental/comm_latency_ms", "VR发布→增量IK收到", STAGE_HOP, "incremental"),
    ("/vr_incremental/arm_traj_latency_ms", "增量IK(骨骼收到→求解完成)", STAGE_IK, "incremental"),
    ("/vr_absolute/comm_latency_ms", "VR发布→绝对IK收到", STAGE_HOP, "absolute"),
    ("/vr_absolute/arm_traj_latency_ms", "绝对IK处理", STAGE_IK, "absolute"),
    ("/vr_incremental/wbc_processing_latency_ms",
     "控制器处理(收轨迹→发/joint_cmd)", STAGE_CTRL, "common"),
]

QUEST3_FINE = [
    ("/vr_absolute/transform_processing_latency_ms", "绝对式骨骼转换处理", STAGE_IK, "absolute"),
    ("/vr_absolute/transform_pipeline_latency_ms", "绝对式骨骼转换流水线", STAGE_IK, "absolute"),
    ("/vr_absolute/finger_processing_latency_ms", "绝对式异步手指计算", STAGE_IK, "absolute"),
    ("/vr_absolute/end_to_end_latency_ms", "绝对式发布→IK同帧端到端", STAGE_IK, "absolute"),
    ("/vr_absolute/ik_wait_latency_ms", "绝对IK求解前等待", STAGE_IK, "absolute"),
    ("/vr_absolute/ik_solve_latency_ms", "绝对IK求解器计算", STAGE_IK, "absolute"),
    ("/vr_absolute/ik_postprocess_latency_ms", "绝对IK求解后处理", STAGE_IK, "absolute"),
    ("/vr_absolute/ik_target_commit_latency_ms", "绝对IK目标提交", STAGE_IK, "absolute"),
    ("/vr_absolute/ik_thread_wakeup_latency_ms", "绝对IK线程唤醒", STAGE_IK, "absolute"),
    ("/vr_absolute/ik_target_snapshot_latency_ms", "绝对IK目标快照", STAGE_IK, "absolute"),
    ("/vr_absolute/ik_fk_latency_ms", "绝对IK求解前FK", STAGE_IK, "absolute"),
    ("/vr_absolute/ik_input_prepare_latency_ms", "绝对IK输入准备", STAGE_IK, "absolute"),
    ("/vr_absolute/ik_solution_ready_latency_ms", "绝对IK事件驱动处理", STAGE_IK, "absolute"),
    ("/vr_absolute/ik_solution_to_publish_latency_ms", "绝对IK解等待首次发布", STAGE_IK, "absolute"),
    ("/vr_absolute/ik_publish_execution_latency_ms", "绝对IK单次轨迹发布", STAGE_IK, "absolute"),
    ("/vr_absolute/ik_publish_period_ms", "绝对IK轨迹发布周期", STAGE_IK, "absolute"),
    ("/vr_absolute/published_arm_traj_latency_ms", "绝对IK回调→首次发布", STAGE_IK, "absolute"),
    ("/vr_absolute/published_end_to_end_latency_ms", "绝对式VR→轨迹首次发布", STAGE_IK, "absolute"),
]

OTHER_TOPICS = [
    ("/pico/node_processing_latency_ms", "Pico VR数据处理(UDP→骨骼发布)", STAGE_VR, "pico"),
    ("/pico/comm_latency_ms", "Pico 骨骼话题通信", STAGE_HOP, "pico"),
    ("/pico/node_latency_ms", "Pico 骨骼→末端指令处理", STAGE_VR, "pico"),
    ("/ocs2_ik/comm_latency_ms", "VR/Pico→OCS2 IK通信", STAGE_HOP, "ocs2"),
    ("/ocs2_ik/processing_latency_ms", "OCS2 IK处理", STAGE_IK, "ocs2"),
]


def _stats(vals):
    if not vals:
        return 0, 0.0, 0.0, 0.0, 0.0
    n = len(vals)
    avg = sum(vals) / n
    var = sum((v - avg) ** 2 for v in vals) / n
    return n, avg, min(vals), max(vals), var ** 0.5


def build_topic_table(fine, include_all):
    rows = list(QUEST3_COARSE)
    if fine:
        rows.extend(QUEST3_FINE)
    if include_all:
        rows.extend(OTHER_TOPICS)
    table = OrderedDict()
    for topic, name, stage, group in rows:
        table[topic] = {"name": name, "stage": stage, "group": group}
    return table


class LatencyMonitor:
    def __init__(self, topic_table, show_empty):
        self.topic_table = topic_table
        self.show_empty = show_empty
        self.data = {t: deque(maxlen=MAX_HISTORY) for t in topic_table}
        self.start_time = None
        self.last_print = 0.0
        self._shutdown = False

    def make_callback(self, topic):
        def cb(msg):
            if self.start_time is None:
                self.start_time = time.time()
            self.data[topic].append(msg.data)
        return cb

    def _active_groups(self):
        groups = set()
        for topic, meta in self.topic_table.items():
            if self.data[topic]:
                groups.add(meta["group"])
        return groups

    def _link_hint(self):
        groups = self._active_groups()
        parts = []
        if "absolute" in groups:
            parts.append("绝对IK")
        if "incremental" in groups:
            parts.append("增量IK")
        if "common" in groups and "absolute" not in groups and "incremental" not in groups:
            parts.append("控制器/VR")
        if "pico" in groups:
            parts.append("Pico")
        if "ocs2" in groups:
            parts.append("OCS2")
        return "+".join(parts) if parts else "等待数据"

    def print_summary(self, force=False):
        now = time.time()
        if not force and (now - self.last_print) < PRINT_INTERVAL:
            return
        self.last_print = now

        rospy.loginfo("")
        rospy.loginfo("=" * 86)
        rospy.loginfo(
            "Quest3 四段延迟  运行 %.1f s  活跃: %s",
            now - (self.start_time or now),
            self._link_hint(),
        )
        rospy.loginfo("-" * 86)
        rospy.loginfo(
            "%-12s %-34s %6s %8s %8s %8s %8s",
            "阶段", "指标", "样本", "平均ms", "最小ms", "最大ms", "标准差",
        )
        rospy.loginfo("-" * 86)

        printed = 0
        last_stage = None
        for topic, meta in self.topic_table.items():
            vals = list(self.data[topic])
            if not vals and not self.show_empty:
                continue
            if meta["stage"] != last_stage:
                last_stage = meta["stage"]
            n, avg, mn, mx, std = _stats(vals)
            if n == 0:
                rospy.loginfo(
                    "%-12s %-34s %6s %8s %8s %8s %8s",
                    meta["stage"], meta["name"], 0, "-", "-", "-", "-",
                )
            else:
                rospy.loginfo(
                    "%-12s %-34s %6d %8.2f %8.2f %8.2f %8.2f",
                    meta["stage"], meta["name"], n, avg, mn, mx, std,
                )
            printed += 1

        if printed == 0:
            rospy.loginfo("(尚无样本。绝对式 VR/IK 需 enable_vr_latency_diagnostics:=true；"
                          "增量 comm/arm_traj 由 C++ IK 默认发布)")

        rospy.loginfo("-" * 86)
        rospy.loginfo(
            "%-8s %s",
            STAGE_ACT,
            "在线无打点 → bag: extract_joint_topics_and_plot.py 的 cmd→sens",
        )
        rospy.loginfo("=" * 86)

    def save_report(self):
        script_dir = os.path.dirname(os.path.abspath(__file__))
        log_dir = os.path.join(script_dir, "log")
        os.makedirs(log_dir, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        filepath = os.path.join(log_dir, f"latency_report_{ts}.csv")

        with open(filepath, "w") as f:
            f.write("stage,group,topic,name,samples,avg_ms,min_ms,max_ms,std_ms\n")
            for topic, meta in self.topic_table.items():
                n, avg, mn, mx, std = _stats(list(self.data[topic]))
                f.write(
                    "{stage},{group},{topic},{name},{n},{avg:.4f},{mn:.4f},{mx:.4f},{std:.4f}\n".format(
                        stage=meta["stage"],
                        group=meta["group"],
                        topic=topic,
                        name=meta["name"],
                        n=n,
                        avg=avg,
                        mn=mn,
                        mx=mx,
                        std=std,
                    )
                )
        rospy.loginfo("报告已保存: %s", filepath)

        raw_path = os.path.join(log_dir, f"latency_raw_{ts}.csv")
        topics = list(self.topic_table.keys())
        with open(raw_path, "w") as f:
            f.write("index," + ",".join(topics) + "\n")
            max_len = max(len(self.data[t]) for t in topics) if topics else 0
            for i in range(max_len):
                row = [str(i)]
                for topic in topics:
                    vals = self.data[topic]
                    row.append("%.4f" % vals[i] if i < len(vals) else "")
                f.write(",".join(row) + "\n")
        rospy.loginfo("原始数据已保存: %s", raw_path)

    def run(self):
        rospy.init_node("latency_monitor", anonymous=True)
        for topic, meta in self.topic_table.items():
            rospy.Subscriber(topic, std_msgs.msg.Float64, self.make_callback(topic))
            rospy.loginfo("订阅: %s [%s] %s", topic, meta["stage"], meta["name"])

        rospy.loginfo(
            "现场订阅已启动 (非正式诊断)。正式报告请用 extract_joint_topics_and_plot.py。每 %.0f 秒打印一次。",
            PRINT_INTERVAL,
        )

        rate = rospy.Rate(1)
        while not rospy.is_shutdown() and not self._shutdown:
            self.print_summary()
            rate.sleep()

        self.print_summary(force=True)
        self.save_report()

    def shutdown(self):
        self._shutdown = True


def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description="Quest3 遥操四段延迟监控（VR / IK / 控制器；电机+反馈见 bag 脚本）",
    )
    p.add_argument(
        "--fine",
        action="store_true",
        help="额外订阅绝对式 IK 细分话题（默认只看四段粗指标）",
    )
    p.add_argument(
        "--all",
        action="store_true",
        help="同时订阅 Pico/OCS2 话题（兼容旧 latency_monitor 行为）",
    )
    p.add_argument(
        "--show-empty",
        action="store_true",
        help="打印尚无样本的话题（默认隐藏，避免绝对/增量串台）",
    )
    return p.parse_args(argv)


def main():
    args = parse_args(rospy.myargv(sys.argv)[1:])
    topic_table = build_topic_table(fine=args.fine, include_all=args.all)
    monitor = LatencyMonitor(topic_table, show_empty=args.show_empty)
    signal.signal(signal.SIGINT, lambda *_: monitor.shutdown())
    try:
        monitor.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
