#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Quest3 bone pose rate benchmark.

订阅 `/leju_quest_bone_poses`，在指定时长内统计：
- 收到的消息数量
- 平均频率 (Hz)
- 周期最小/最大/平均/标准差
可选：把每条消息的接收时间和 timestamp_ms 写入 CSV 文件，方便离线分析。

用法示例（各跑 3 分钟）：
  # Python 版本监控节点
  roslaunch noitom_hi5_hand_udp_python launch_quest3_ik.launch use_cpp_monitor:=false
  rosrun noitom_hi5_hand_udp_python quest3_bone_rate_benchmark.py --duration 180 --label python --csv /tmp/quest3_python.csv

  # C++ 版本监控节点
  roslaunch noitom_hi5_hand_udp_python launch_quest3_ik.launch use_cpp_monitor:=true
  rosrun noitom_hi5_hand_udp_python quest3_bone_rate_benchmark.py --duration 180 --label cpp --csv /tmp/quest3_cpp.csv
"""

import math
import csv
import sys
import rospy

from noitom_hi5_hand_udp_python.msg import PoseInfoList


class RateStats(object):
    def __init__(self):
        self.recv_times = []      # float seconds (ROS time)
        self.msg_timestamp_ms = []  # from PoseInfoList.timestamp_ms

    def add_sample(self, recv_time_sec, timestamp_ms):
        self.recv_times.append(float(recv_time_sec))
        self.msg_timestamp_ms.append(int(timestamp_ms))

    def has_enough_data(self):
        return len(self.recv_times) >= 2

    def compute(self):
        """返回一个 dict，包括频率和周期统计."""
        n = len(self.recv_times)
        if n < 2:
            return None

        periods = [
            self.recv_times[i] - self.recv_times[i - 1]
            for i in range(1, n)
        ]

        total_time = self.recv_times[-1] - self.recv_times[0]
        mean_period = sum(periods) / len(periods)

        # 避免除零
        if mean_period <= 0:
            rate = float('inf')
        else:
            rate = 1.0 / mean_period

        min_period = min(periods)
        max_period = max(periods)

        # population standard deviation
        var = sum((p - mean_period) ** 2 for p in periods) / len(periods)
        std_period = math.sqrt(var)

        return {
            "count": n,
            "duration": total_time,
            "rate_hz": rate,
            "min_period": min_period,
            "max_period": max_period,
            "mean_period": mean_period,
            "std_period": std_period,
        }


class BoneRateBenchmark(object):
    def __init__(self, topic, duration, label="", csv_path=None):
        self.topic = topic
        self.duration = duration
        self.label = label or ""
        self.csv_path = csv_path

        self.stats = RateStats()
        self.start_time = None

        # 标记基准测试是否已经结束，结束后忽略新的回调，防止在关闭 CSV 后继续写入
        self.finished = False

        self._csv_file = None
        self._csv_writer = None

        if self.csv_path:
            try:
                self._csv_file = open(self.csv_path, "w", newline="")
                self._csv_writer = csv.writer(self._csv_file)
                self._csv_writer.writerow(
                    ["label", "index", "recv_time_sec", "timestamp_ms"]
                )
            except Exception as e:
                rospy.logerr("Failed to open CSV file %s: %s", self.csv_path, e)
                self._csv_file = None
                self._csv_writer = None

        self.sub = rospy.Subscriber(self.topic, PoseInfoList,
                                    self._callback,
                                    queue_size=1000)

    def _callback(self, msg):
        # 如果已经结束统计，直接忽略后续消息，避免对已关闭文件进行写操作
        if self.finished:
            return

        now = rospy.get_time()
        if self.start_time is None:
            self.start_time = now

        self.stats.add_sample(now, msg.timestamp_ms)

        if self._csv_writer is not None:
            idx = len(self.stats.recv_times) - 1
            try:
                self._csv_writer.writerow(
                    [self.label, idx, now, int(msg.timestamp_ms)]
                )
            except Exception as e:
                rospy.logwarn("Failed to write CSV row: %s", e)

    def run(self):
        rospy.loginfo("Benchmarking topic '%s' for %.1f seconds (label='%s')",
                      self.topic, self.duration, self.label)
        end_time = rospy.get_time() + self.duration

        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            now = rospy.get_time()
            if now >= end_time:
                break
            rate.sleep()

        self._finish()

    def _finish(self):
        # 标记结束，防止回调继续尝试写入 CSV
        self.finished = True

        if self._csv_file is not None:
            try:
                self._csv_file.flush()
                self._csv_file.close()
            except Exception:
                pass
            finally:
                self._csv_file = None
                self._csv_writer = None

        result = self.stats.compute()
        if result is None:
            rospy.logwarn("Not enough data received on %s to compute stats.", self.topic)
            return

        label_prefix = f"[{self.label}] " if self.label else ""

        rospy.loginfo("%sBenchmark results on topic '%s':", label_prefix, self.topic)
        rospy.loginfo("%s  Messages received : %d", label_prefix, result["count"])
        rospy.loginfo("%s  Time span        : %.3f s", label_prefix, result["duration"])
        rospy.loginfo("%s  Avg rate         : %.3f Hz", label_prefix, result["rate_hz"])
        rospy.loginfo("%s  Min period       : %.4f s", label_prefix, result["min_period"])
        rospy.loginfo("%s  Max period       : %.4f s", label_prefix, result["max_period"])
        rospy.loginfo("%s  Mean period      : %.4f s", label_prefix, result["mean_period"])
        rospy.loginfo("%s  Stddev period    : %.4f s", label_prefix, result["std_period"])

        print("")
        print("==== Quest3 bone rate benchmark ====")
        print(f"Label           : {self.label}")
        print(f"Topic           : {self.topic}")
        print(f"Messages        : {result['count']}")
        print(f"Time span (s)   : {result['duration']:.3f}")
        print(f"Avg rate (Hz)   : {result['rate_hz']:.3f}")
        print(f"Min period (s)  : {result['min_period']:.4f}")
        print(f"Max period (s)  : {result['max_period']:.4f}")
        print(f"Mean period (s) : {result['mean_period']:.4f}")
        print(f"Std period (s)  : {result['std_period']:.4f}")
        print("====================================")


def parse_args():
    import argparse
    # 使用 rospy.myargv() 过滤掉 ROS 自己的参数
    argv = rospy.myargv(sys.argv)
    parser = argparse.ArgumentParser(
        description="Benchmark update rate of /leju_quest_bone_poses."
    )
    parser.add_argument(
        "--topic",
        default="/leju_quest_bone_poses",
        help="Topic name to benchmark (default: /leju_quest_bone_poses)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=180.0,
        help="Benchmark duration in seconds (default: 180 = 3 minutes)",
    )
    parser.add_argument(
        "--label",
        type=str,
        default="",
        help="Label to include in logs/CSV (e.g. 'python' or 'cpp')",
    )
    parser.add_argument(
        "--csv",
        type=str,
        default="",
        help="Optional CSV output path for per-message timestamps.",
    )

    return parser.parse_args(argv[1:])


def main():
    rospy.init_node("quest3_bone_rate_benchmark", anonymous=True)
    args = parse_args()

    benchmark = BoneRateBenchmark(
        topic=args.topic,
        duration=args.duration,
        label=args.label,
        csv_path=args.csv or None,
    )
    benchmark.run()


if __name__ == "__main__":
    main()