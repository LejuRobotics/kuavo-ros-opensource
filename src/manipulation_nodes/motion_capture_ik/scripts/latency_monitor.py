#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
延迟监控报告脚本
订阅延迟话题，定期打印统计摘要，退出时保存完整报告到 log/ 目录。

话题说明:
  /quest3/node_processing_latency_ms        Quest3 VR数据处理延迟
  /pico/node_processing_latency_ms           Pico VR数据处理延迟
  /pico/comm_latency_ms                     Pico VR数据→ROS节点通信延迟
  /pico/node_latency_ms                     Pico ROS节点处理延迟
  /vr_incremental/comm_latency_ms          VR数据处理节点→增量IK节点通信延迟
  /vr_incremental/arm_traj_latency_ms       增量IK处理延迟
  /vr_absolute/comm_latency_ms             VR→绝对IK通信延迟
  /vr_absolute/transform_processing_latency_ms 骨骼坐标转换处理耗时
  /vr_absolute/transform_pipeline_latency_ms 回调入口→坐标转换完成延迟
  /vr_absolute/finger_processing_latency_ms 异步手指计算耗时
  /vr_absolute/arm_traj_latency_ms          绝对IK处理延迟
  /vr_absolute/end_to_end_latency_ms        VR发布→同一帧IK完成延迟
  /vr_absolute/ik_wait_latency_ms           转换完成→IK求解开始等待/准备耗时
  /vr_absolute/ik_solve_latency_ms          IK求解器计算耗时
  /vr_absolute/ik_postprocess_latency_ms    IK求解后轨迹后处理耗时
  /vr_absolute/ik_target_commit_latency_ms  转换完成→目标提交耗时
  /vr_absolute/ik_thread_wakeup_latency_ms  目标提交→IK线程取得目标耗时
  /vr_absolute/ik_target_snapshot_latency_ms IK目标快照复制耗时
  /vr_absolute/ik_fk_latency_ms             IK求解前双臂FK耗时
  /vr_absolute/ik_input_prepare_latency_ms  FK完成→IK求解开始准备耗时
  /vr_absolute/ik_solution_ready_latency_ms 目标提交→最新有效解就绪耗时
  /vr_absolute/ik_solution_to_publish_latency_ms 解就绪→首次100Hz发布耗时
  /vr_absolute/ik_publish_execution_latency_ms 100Hz发布线程单次执行耗时
  /vr_absolute/ik_publish_period_ms         实际轨迹发布周期间隔
  /vr_absolute/published_arm_traj_latency_ms IK回调→首次轨迹发布耗时
  /vr_absolute/published_end_to_end_latency_ms VR发布→首次轨迹发布耗时
  /vr_incremental/wbc_processing_latency_ms WBC控制器处理延迟（SHM读取→/joint_cmd发布）
  /ocs2_ik/comm_latency_ms                 VR/Pico→OCS2 IK通信延迟
  /ocs2_ik/processing_latency_ms           OCS2 IK处理延迟（指令接收→/mm_kuavo_arm_traj发布）
"""

import os
import sys
import time
import signal
import rospy
import std_msgs.msg
from collections import deque

# ── 配置 ────────────────────────────────────────────────
PRINT_INTERVAL = 5.0       # 终端打印间隔（秒）
MAX_HISTORY    = 10000     # 每个话题最多保留的历史样本数
# ────────────────────────────────────────────────────────

TOPICS = {
    "/quest3/node_processing_latency_ms":        "Quest3 VR数据处理延迟",
    "/pico/node_processing_latency_ms":           "Pico VR数据处理延迟",
    "/pico/comm_latency_ms":                     "Pico VR→ROS节点通信延迟",
    "/pico/node_latency_ms":                     "Pico ROS节点处理延迟",
    "/vr_incremental/comm_latency_ms":          "VR→增量IK通信延迟",
    "/vr_incremental/arm_traj_latency_ms":       "增量IK处理延迟",
    "/vr_absolute/comm_latency_ms":             "VR→绝对IK通信延迟",
    "/vr_absolute/transform_processing_latency_ms": "绝对式骨骼转换处理耗时",
    "/vr_absolute/transform_pipeline_latency_ms": "绝对式骨骼转换流水线延迟",
    "/vr_absolute/finger_processing_latency_ms": "绝对式异步手指计算耗时",
    "/vr_absolute/arm_traj_latency_ms":          "绝对IK处理延迟",
    "/vr_absolute/end_to_end_latency_ms":        "绝对式发布→IK同帧端到端延迟",
    "/vr_absolute/ik_wait_latency_ms":           "绝对IK求解前等待与准备耗时",
    "/vr_absolute/ik_solve_latency_ms":          "绝对IK求解器计算耗时",
    "/vr_absolute/ik_postprocess_latency_ms":    "绝对IK求解后处理耗时",
    "/vr_absolute/ik_target_commit_latency_ms":  "绝对IK目标提交耗时",
    "/vr_absolute/ik_thread_wakeup_latency_ms":  "绝对IK线程唤醒耗时",
    "/vr_absolute/ik_target_snapshot_latency_ms": "绝对IK目标快照耗时",
    "/vr_absolute/ik_fk_latency_ms":             "绝对IK求解前双臂FK耗时",
    "/vr_absolute/ik_input_prepare_latency_ms":  "绝对IK输入准备耗时",
    "/vr_absolute/ik_solution_ready_latency_ms": "绝对IK事件驱动处理耗时",
    "/vr_absolute/ik_solution_to_publish_latency_ms": "绝对IK解等待首次发布耗时",
    "/vr_absolute/ik_publish_execution_latency_ms": "绝对IK单次轨迹发布耗时",
    "/vr_absolute/ik_publish_period_ms":         "绝对IK轨迹发布周期间隔",
    "/vr_absolute/published_arm_traj_latency_ms": "绝对IK回调→首次发布延迟",
    "/vr_absolute/published_end_to_end_latency_ms": "绝对式VR发布→轨迹首次发布延迟",
    "/vr_incremental/wbc_processing_latency_ms": "WBC控制器处理延迟",
    "/ocs2_ik/comm_latency_ms":                 "VR/Pico→OCS2 IK通信延迟",
    "/ocs2_ik/processing_latency_ms":            "OCS2 IK处理延迟",
}


class LatencyMonitor:
    def __init__(self):
        self.data = {t: deque(maxlen=MAX_HISTORY) for t in TOPICS}
        self.start_time = None
        self.last_print = 0.0
        self._shutdown = False

    def make_callback(self, topic):
        def cb(msg):
            if self.start_time is None:
                self.start_time = time.time()
            self.data[topic].append(msg.data)
        return cb

    def print_summary(self, force=False):
        now = time.time()
        if not force and (now - self.last_print) < PRINT_INTERVAL:
            return
        self.last_print = now

        rospy.loginfo("")
        rospy.loginfo("=" * 72)
        rospy.loginfo("延迟报告  (运行时长 %.1f s)", now - (self.start_time or now))
        rospy.loginfo("-" * 72)
        rospy.loginfo(
            "%-28s %6s %8s %8s %8s %8s",
            "话题", "样本数", "平均(ms)", "最小(ms)", "最大(ms)", "标准差"
        )
        rospy.loginfo("-" * 72)
        for topic, name in TOPICS.items():
            vals = list(self.data[topic])
            if not vals:
                rospy.loginfo("%-28s %6s %8s %8s %8s %8s",
                              name, 0, "-", "-", "-", "-")
                continue
            n = len(vals)
            avg = sum(vals) / n
            mn = min(vals)
            mx = max(vals)
            var = sum((v - avg) ** 2 for v in vals) / n
            std = var ** 0.5
            rospy.loginfo("%-28s %6d %8.2f %8.2f %8.2f %8.2f",
                          name, n, avg, mn, mx, std)
        rospy.loginfo("=" * 72)

    def save_report(self):
        script_dir = os.path.dirname(os.path.abspath(__file__))
        log_dir = os.path.join(script_dir, "log")
        os.makedirs(log_dir, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        filepath = os.path.join(log_dir, f"latency_report_{ts}.csv")

        with open(filepath, "w") as f:
            f.write("topic,name,samples,avg_ms,min_ms,max_ms,std_ms\n")
            for topic, name in TOPICS.items():
                vals = list(self.data[topic])
                if not vals:
                    f.write(f"{topic},{name},0,0,0,0,0\n")
                    continue
                n = len(vals)
                avg = sum(vals) / n
                mn = min(vals)
                mx = max(vals)
                var = sum((v - avg) ** 2 for v in vals) / n
                std = var ** 0.5
                f.write(f"{topic},{name},{n},{avg:.4f},{mn:.4f},{mx:.4f},{std:.4f}\n")

        rospy.loginfo("报告已保存: %s", filepath)

        # 同时保存原始数据
        raw_path = os.path.join(log_dir, f"latency_raw_{ts}.csv")
        with open(raw_path, "w") as f:
            f.write("index," + ",".join(TOPICS.keys()) + "\n")
            max_len = max(len(self.data[t]) for t in TOPICS)
            for i in range(max_len):
                row = [str(i)]
                for topic in TOPICS:
                    vals = self.data[topic]
                    row.append(f"{vals[i]:.4f}" if i < len(vals) else "")
                f.write(",".join(row) + "\n")
        rospy.loginfo("原始数据已保存: %s", raw_path)

    def run(self):
        rospy.init_node("latency_monitor", anonymous=True)
        for topic in TOPICS:
            rospy.Subscriber(topic, std_msgs.msg.Float64, self.make_callback(topic))
            rospy.loginfo("订阅: %s (%s)", topic, TOPICS[topic])

        rospy.loginfo("延迟监控已启动，每 %.0f 秒打印一次摘要，Ctrl+C 退出并保存报告", PRINT_INTERVAL)

        rate = rospy.Rate(1)
        while not rospy.is_shutdown() and not self._shutdown:
            self.print_summary()
            rate.sleep()

        self.print_summary(force=True)
        self.save_report()

    def shutdown(self):
        self._shutdown = True


def main():
    monitor = LatencyMonitor()
    signal.signal(signal.SIGINT, lambda *_: monitor.shutdown())
    try:
        monitor.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
