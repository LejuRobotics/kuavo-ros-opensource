#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Quest3 遥操延迟诊断 (离线, 单一入口)。

不再依赖在线 latency_monitor: 从 rosbag 读取各遥操方式的延迟话题,
再对关节波形做互相关, 汇总成一份四段 + 端到端报告。

Quest3 四条链路请用 --preset:
  q3-abs-humanoid  Quest3+绝对+人形
  q3-inc-humanoid  Quest3+增量+人形
  q3-abs-wheel     Quest3+绝对+轮臂
  q3-inc-wheel     Quest3+增量+轮臂
Pico 绝对式:
  pico-abs-humanoid  Pico+绝对+人形  (轨迹 /mm_kuavo_arm_traj)

用法:
  python3 extract_joint_topics_and_plot.py -b xxx.bag --preset q3-inc-humanoid
  python3 extract_joint_topics_and_plot.py -b xxx.bag --preset q3-abs-wheel --robot-version 62 --fine
  python3 extract_joint_topics_and_plot.py -b xxx.bag --preset pico-abs-humanoid --robot-version 55

报告构成:
  前半段 (读 Float64 延迟话题):
    1. VR 数据处理
    层间 VR→IK 通信
    2. IK 解算
  后半段 (关节角互相关, delay>0 表示下游滞后上游):
    3. 控制器  /joint_cmd 滞后 IK轨迹
    4. 电机+反馈  传感器滞后 /joint_cmd
  端到端:
    四段相加; 以及传感器滞后 IK轨迹; 绝对式若有 published_end_to_end 一并列出。
  --fine: 额外打印绝对 IK 细分话题 (ik_solve 等)。

依赖: rosbag, numpy, matplotlib, scipy  (需先 source devel/setup.bash)
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

try:
    import rosbag
except ImportError as e:
    sys.exit(f"[FATAL] 无法导入 rosbag: {e}\n请先 source ROS 环境 (source devel/setup.bash)")

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    # 配置中文字体, 避免中文乱码
    import matplotlib.font_manager as fm
    _cjk_fonts = [
        "WenQuanYi Micro Hei", "WenQuanYi Zen Hei", "Noto Sans CJK SC",
        "Noto Sans CJK JP", "Source Han Sans SC", "Source Han Sans CN",
        "SimHei", "Microsoft YaHei", "PingFang SC", "Heiti SC", "AR PL UMing CN",
    ]
    _available = {f.name for f in fm.fontManager.ttflist}
    _chosen = next((f for f in _cjk_fonts if f in _available), None)
    if _chosen is not None:
        plt.rcParams["font.sans-serif"] = [_chosen] + plt.rcParams.get("font.sans-serif", [])
        plt.rcParams["axes.unicode_minus"] = False
    else:
        # 没有中文字体时, 警告但不中断
        import warnings
        warnings.warn("未找到中文字体, 图中中文可能显示为方框。建议安装: sudo apt install fonts-wqy-microhei")
except ImportError:
    sys.exit("[FATAL] 需要 matplotlib: pip install matplotlib")

try:
    from scipy.signal import correlate
    from scipy.optimize import least_squares
except ImportError:
    sys.exit("[FATAL] 需要 scipy: pip install scipy")


# ==============================================================================
# 常量
# ==============================================================================
RAD2DEG = 180.0 / math.pi
DEG2RAD = math.pi / 180.0

# 话题定义
TOPIC_ARM_TRAJ_INCREMENTAL = "/vr_incremental/kuavo_arm_traj_shm"  # 增量式遥操, sensor_msgs/JointState, position=deg
TOPIC_ARM_TRAJ_ABSOLUTE = "/kuavo_arm_traj"                        # 绝对式遥操, sensor_msgs/JointState, position=deg
TOPIC_ARM_TRAJ_FILTERED = "/vr_incremental/kuavo_arm_traj_filtered"  # WBC滤波后, sensor_msgs/JointState, position=deg
TOPIC_ARM_TRAJ_PICO = "/mm_kuavo_arm_traj"                          # Pico遥操, sensor_msgs/JointState, position=deg
TOPIC_JOINT_CMD = "/joint_cmd"              # kuavo_msgs/jointCmd, joint_q=rad
TOPIC_SENSORS = "/sensors_data_raw"        # kuavo_msgs/sensorsData, joint_data.joint_q=rad

# 遥操模式 → 手臂轨迹话题映射
ARM_TRAJ_TOPICS = {
    "incremental": TOPIC_ARM_TRAJ_INCREMENTAL,
    "absolute": TOPIC_ARM_TRAJ_ABSOLUTE,
    "pico": TOPIC_ARM_TRAJ_PICO,
}

# Quest3 四条链路预设 (人形/轮臂轨迹话题相同, 版本靠 --robot-version / ROBOT_VERSION)
QUEST3_PRESETS = {
    "q3-abs-humanoid": {
        "mode": "absolute",
        "label": "Quest3+绝对+人形",
    },
    "q3-inc-humanoid": {
        "mode": "incremental",
        "label": "Quest3+增量+人形",
    },
    "q3-abs-wheel": {
        "mode": "absolute",
        "label": "Quest3+绝对+轮臂",
    },
    "q3-inc-wheel": {
        "mode": "incremental",
        "label": "Quest3+增量+轮臂",
    },
    "pico-abs-humanoid": {
        "mode": "pico",
        "label": "Pico+绝对+人形",
    },
}

# 前半段: 按遥操方式从 bag 读 Float64 延迟话题 (ms)
# key: vr / hop / ik
FRONT_STAGE_TOPICS = {
    "incremental": [
        ("vr", "/quest3/node_processing_latency_ms", "1.VR数据处理"),
        ("hop", "/vr_incremental/comm_latency_ms", "层间 VR→IK通信"),
        ("ik", "/vr_incremental/arm_traj_latency_ms", "2.IK解算(骨骼收到→求解完成)"),
    ],
    "absolute": [
        ("vr", "/quest3/node_processing_latency_ms", "1.VR数据处理"),
        ("hop", "/vr_absolute/comm_latency_ms", "层间 VR→IK通信"),
        ("ik", "/vr_absolute/arm_traj_latency_ms", "2.IK解算"),
    ],
    "pico": [
        ("vr", "/pico/node_processing_latency_ms", "1.VR数据处理"),
        ("bone_hop", "/pico/comm_latency_ms", "层间 骨骼话题通信"),
        ("eef", "/pico/node_latency_ms", "Pico末端指令处理"),
        ("hop", "/ocs2_ik/comm_latency_ms", "层间 Pico→OCS2通信"),
        ("ik", "/ocs2_ik/processing_latency_ms", "2.IK解算"),
    ],
}

CTRL_STOPWATCH_TOPIC = "/vr_incremental/wbc_processing_latency_ms"

# 互相关顶到 ±max_lag 附近视为搜索失败; 与最小二乘相差过大时改用最小二乘.
LAG_BOUND_FRAC = 0.95
LAG_DISAGREE_SEC = 0.15

# 端到端 / IK 细分 (有则进报告)
E2E_STOPWATCH_TOPICS = [
    ("/vr_absolute/published_end_to_end_latency_ms", "绝对式 VR发布→轨迹首次发布"),
    ("/vr_absolute/end_to_end_latency_ms", "绝对式 VR发布→IK同帧完成"),
]

IK_FINE_TOPICS = [
    ("/vr_absolute/ik_solve_latency_ms", "IK求解器纯计算"),
    ("/vr_absolute/ik_wait_latency_ms", "IK求解前等待"),
    ("/vr_absolute/ik_postprocess_latency_ms", "IK求解后处理"),
    ("/vr_absolute/ik_fk_latency_ms", "IK求解前FK"),
    ("/vr_absolute/transform_processing_latency_ms", "骨骼坐标转换"),
    ("/vr_absolute/published_arm_traj_latency_ms", "IK回调→首次发布"),
]


# ==============================================================================
# 机器人布局加载 (参考 scripts/increment_test/plot_arm_traj_sensor_cmd.py)
# ==============================================================================
def load_layout(robot_version: Optional[int] = None, assets_root: Optional[str] = None) -> dict:
    """返回机器人关节布局信息。

    Returns:
        dict with keys:
            n_arm (int): 单条手臂关节数 (NUM_ARM_JOINT, 含左右合计, 如 14)
            n_head (int): 头部关节数
            n_waist (int): 腰部关节数
            n_tot (int): 总关节数 (NUM_JOINT)
            arm_offset (int): 手臂关节在 /joint_cmd 与 /sensors_data_raw 中的起始索引
            single_arm (int): 单条手臂关节数 = n_arm // 2
            config (str): 使用的配置文件路径
    """
    if robot_version is None:
        # 尝试从环境变量读取
        robot_version = int(os.environ.get("ROBOT_VERSION", 45))

    if assets_root is None:
        # 优先 ROS 包路径
        try:
            import rospkg
            assets_root = os.path.join(rospkg.RosPack().get_path("kuavo_assets"), "config")
        except Exception:
            # 相对脚本定位 <ws>/src/kuavo_assets/config
            ws = os.environ.get("KUAVO_WS") or os.path.abspath(
                os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
            assets_root = os.path.join(ws, "src/kuavo_assets/config")

    path = os.path.join(assets_root, f"kuavo_v{robot_version}", "kuavo.json")
    if not os.path.isfile(path):
        # 回退: 假设人形 v45 布局
        print(f"[WARN] 找不到配置 {path}, 使用默认人形布局 (n_arm=14, n_head=2, n_tot=28)")
        n_arm = 14
        n_head = 2
        n_waist = 0
        n_tot = 28
        path = "(default)"
    else:
        with open(path) as f:
            cfg = json.load(f)
        n_arm = int(cfg["NUM_ARM_JOINT"])
        n_head = int(cfg.get("NUM_HEAD_JOINT", 2))
        n_waist = int(cfg.get("NUM_WAIST_JOINT", 0))
        n_tot = int(cfg["NUM_JOINT"])

    arm_offset = n_tot - n_head - n_arm
    if arm_offset < 0:
        raise RuntimeError(
            f"非法布局: total={n_tot} head={n_head} arm={n_arm} -> offset={arm_offset}")

    return {
        "n_arm": n_arm,
        "n_head": n_head,
        "n_waist": n_waist,
        "n_tot": n_tot,
        "arm_offset": arm_offset,
        "single_arm": n_arm // 2,
        "config": path,
    }


# ==============================================================================
# 数据容器
# ==============================================================================
@dataclass
class TopicData:
    """单个话题提取后的数据。"""
    name: str
    times: np.ndarray = field(default_factory=lambda: np.array([]))      # 秒
    values: np.ndarray = field(default_factory=lambda: np.array([]))    # deg
    unit_label: str = "deg"

    def __len__(self) -> int:
        return len(self.times)


# ==============================================================================
# 从 bag 提取数据
# ==============================================================================
def _stamp_to_sec(t) -> float:
    """ros Time / rosbag time -> float 秒。"""
    try:
        return t.to_sec()
    except AttributeError:
        return float(t)


def extract_arm_traj(bag_path: str, joint_index: int, single_arm: int, topic: str) -> TopicData:
    """从手臂轨迹话题 (sensor_msgs/JointState) 提取指定关节。

    topic: 话题名, 增量式为 /vr_incremental/kuavo_arm_traj_shm, 绝对式为 /kuavo_arm_traj
    joint_index: 1-based, 范围 [1, 2*single_arm]
                 1..single_arm  -> 左臂关节 1..N
                 single_arm+1..2*single_arm -> 右臂关节 1..N
    返回数据为 deg (原话题已是 deg)。
    """
    if not (1 <= joint_index <= 2 * single_arm):
        raise ValueError(
            f"{topic} 关节索引 {joint_index} 超出范围 [1, {2 * single_arm}]")

    times: List[float] = []
    values: List[float] = []
    idx = joint_index - 1  # 0-based

    with rosbag.Bag(bag_path, "r") as bag:
        for t_topic, msg, t in bag.read_messages(topics=[topic]):
            pos = msg.position
            if pos is None or len(pos) <= idx:
                continue
            times.append(_stamp_to_sec(t))
            values.append(float(pos[idx]))

    return TopicData(
        name=topic,
        times=np.array(times, dtype=float),
        values=np.array(values, dtype=float),
        unit_label="deg",
    )


def extract_joint_cmd(bag_path: str, joint_index: int, layout: dict) -> TopicData:
    """从 /joint_cmd (kuavo_msgs/jointCmd) 提取指定手臂关节。

    joint_index: 1-based, 范围 [1, 2*single_arm] (手臂关节序号, 与 /vr_incremental/kuavo_arm_traj_shm 对齐)
    返回数据为 deg (从 rad 转换)。
    """
    single_arm = layout["single_arm"]
    if not (1 <= joint_index <= 2 * single_arm):
        raise ValueError(
            f"/joint_cmd 关节索引 {joint_index} 超出范围 [1, {2 * single_arm}]")

    arm_offset = layout["arm_offset"]
    idx = arm_offset + (joint_index - 1)  # 0-based 在 joint_q 中的位置

    times: List[float] = []
    values: List[float] = []

    with rosbag.Bag(bag_path, "r") as bag:
        for topic, msg, t in bag.read_messages(topics=[TOPIC_JOINT_CMD]):
            jq = msg.joint_q
            if jq is None or len(jq) <= idx:
                continue
            times.append(_stamp_to_sec(t))
            values.append(float(jq[idx]) * RAD2DEG)

    return TopicData(
        name=TOPIC_JOINT_CMD,
        times=np.array(times, dtype=float),
        values=np.array(values, dtype=float),
        unit_label="deg",
    )


def extract_sensors(bag_path: str, joint_index: int, layout: dict) -> TopicData:
    """从 /sensors_data_raw (kuavo_msgs/sensorsData) 提取指定手臂关节。

    joint_index: 1-based, 范围 [1, 2*single_arm] (手臂关节序号, 与 /vr_incremental/kuavo_arm_traj_shm 对齐)
    返回数据为 deg (从 rad 转换)。
    """
    single_arm = layout["single_arm"]
    if not (1 <= joint_index <= 2 * single_arm):
        raise ValueError(
            f"/sensors_data_raw 关节索引 {joint_index} 超出范围 [1, {2 * single_arm}]")

    arm_offset = layout["arm_offset"]
    idx = arm_offset + (joint_index - 1)

    times: List[float] = []
    values: List[float] = []

    with rosbag.Bag(bag_path, "r") as bag:
        for topic, msg, t in bag.read_messages(topics=[TOPIC_SENSORS]):
            jq = msg.joint_data.joint_q
            if jq is None or len(jq) <= idx:
                continue
            times.append(_stamp_to_sec(t))
            values.append(float(jq[idx]) * RAD2DEG)

    return TopicData(
        name=TOPIC_SENSORS,
        times=np.array(times, dtype=float),
        values=np.array(values, dtype=float),
        unit_label="deg",
    )


# ==============================================================================
# 批量提取: 一次遍历 bag 提取所有手臂关节 (避免重复打开 bag)
# ==============================================================================
def extract_arm_traj_all(bag_path: str, single_arm: int, topic: str) -> List[TopicData]:
    """一次遍历 rosbag 提取手臂轨迹话题所有关节 (1..2*single_arm)。"""
    n_joints = 2 * single_arm
    times_list: List[List[float]] = [[] for _ in range(n_joints)]
    values_list: List[List[float]] = [[] for _ in range(n_joints)]

    with rosbag.Bag(bag_path, "r") as bag:
        for _t_topic, msg, t in bag.read_messages(topics=[topic]):
            pos = msg.position
            if pos is None or len(pos) < n_joints:
                continue
            ts = _stamp_to_sec(t)
            for i in range(n_joints):
                times_list[i].append(ts)
                values_list[i].append(float(pos[i]))

    return [
        TopicData(
            name=topic,
            times=np.array(times_list[i], dtype=float),
            values=np.array(values_list[i], dtype=float),
            unit_label="deg",
        )
        for i in range(n_joints)
    ]


def extract_joint_cmd_all(bag_path: str, layout: dict) -> List[TopicData]:
    """一次遍历 rosbag 提取 /joint_cmd 所有手臂关节。"""
    single_arm = layout["single_arm"]
    n_joints = 2 * single_arm
    arm_offset = layout["arm_offset"]
    times_list: List[List[float]] = [[] for _ in range(n_joints)]
    values_list: List[List[float]] = [[] for _ in range(n_joints)]

    with rosbag.Bag(bag_path, "r") as bag:
        for _topic, msg, t in bag.read_messages(topics=[TOPIC_JOINT_CMD]):
            jq = msg.joint_q
            if jq is None or len(jq) < arm_offset + n_joints:
                continue
            ts = _stamp_to_sec(t)
            for i in range(n_joints):
                times_list[i].append(ts)
                values_list[i].append(float(jq[arm_offset + i]) * RAD2DEG)

    return [
        TopicData(
            name=TOPIC_JOINT_CMD,
            times=np.array(times_list[i], dtype=float),
            values=np.array(values_list[i], dtype=float),
            unit_label="deg",
        )
        for i in range(n_joints)
    ]


def extract_sensors_all(bag_path: str, layout: dict) -> List[TopicData]:
    """一次遍历 rosbag 提取 /sensors_data_raw 所有手臂关节。"""
    single_arm = layout["single_arm"]
    n_joints = 2 * single_arm
    arm_offset = layout["arm_offset"]
    times_list: List[List[float]] = [[] for _ in range(n_joints)]
    values_list: List[List[float]] = [[] for _ in range(n_joints)]

    with rosbag.Bag(bag_path, "r") as bag:
        for _topic, msg, t in bag.read_messages(topics=[TOPIC_SENSORS]):
            jq = msg.joint_data.joint_q
            if jq is None or len(jq) < arm_offset + n_joints:
                continue
            ts = _stamp_to_sec(t)
            for i in range(n_joints):
                times_list[i].append(ts)
                values_list[i].append(float(jq[arm_offset + i]) * RAD2DEG)

    return [
        TopicData(
            name=TOPIC_SENSORS,
            times=np.array(times_list[i], dtype=float),
            values=np.array(values_list[i], dtype=float),
            unit_label="deg",
        )
        for i in range(n_joints)
    ]


def extract_float64_topic_stats(bag_path: str, topics: List[str]) -> dict:
    """一次遍历 bag, 统计 Float64 延迟话题 (ms)。

    Returns:
        {topic: {"n": int, "avg": float, "min": float, "max": float, "std": float}}
        无样本的 topic 不会出现在返回值中。
    """
    buckets = {t: [] for t in topics}
    with rosbag.Bag(bag_path, "r") as bag:
        info = bag.get_type_and_topic_info().topics
        present = [t for t in topics if t in info]
        if not present:
            return {}
        for topic, msg, _t in bag.read_messages(topics=present):
            try:
                buckets[topic].append(float(msg.data))
            except (AttributeError, TypeError, ValueError):
                continue

    stats = {}
    for topic, vals in buckets.items():
        if not vals:
            continue
        arr = np.asarray(vals, dtype=float)
        stats[topic] = {
            "n": int(arr.size),
            "avg": float(np.mean(arr)),
            "min": float(np.min(arr)),
            "max": float(np.max(arr)),
            "std": float(np.std(arr)),
        }
    return stats


def collect_latency_topics(mode: str, fine: bool) -> List[str]:
    topics = [row[1] for row in FRONT_STAGE_TOPICS.get(mode, [])]
    topics.append(CTRL_STOPWATCH_TOPIC)
    topics.extend(t for t, _ in E2E_STOPWATCH_TOPICS)
    if fine or mode == "absolute":
        topics.extend(t for t, _ in IK_FINE_TOPICS)
    # 去重保序
    seen = set()
    ordered = []
    for t in topics:
        if t not in seen:
            seen.add(t)
            ordered.append(t)
    return ordered


def _avg_or_none(stats: dict, topic: str) -> Optional[float]:
    st = stats.get(topic)
    return None if st is None else float(st["avg"])


def _print_stopwatch_line(label: str, topic: str, stats: dict) -> Optional[float]:
    st = stats.get(topic)
    if st is None:
        print(f"  {label:28s}  缺失  ({topic})")
        return None
    print(
        f"  {label:28s}  {st['avg']:8.2f} ± {st['std']:6.2f} ms"
        f"  n={st['n']:<6d}  [{st['min']:.2f}, {st['max']:.2f}]"
        f"  {topic}"
    )
    return float(st["avg"])


def build_and_print_report(
    mode: str,
    link_label: Optional[str],
    topic_arm_traj: str,
    stats: dict,
    all_delays: List[dict],
    fine: bool,
) -> dict:
    """打印并返回统一诊断报告 dict。"""
    front_spec = FRONT_STAGE_TOPICS.get(mode, FRONT_STAGE_TOPICS["incremental"])
    front_avgs = {}

    print("\n" + "=" * 88)
    print("                    Quest3 延迟诊断报告 (离线)")
    print("=" * 88)
    if link_label:
        print(f"  链路: {link_label}")
    print(f"  模式: {mode}    IK轨迹: {topic_arm_traj}")
    print("  前半段=bag 延迟话题均值; 后半段=下游滞后上游 (全臂关节互相关均值)")
    print("-" * 88)
    print("  【前半段】")
    for key, topic, label in front_spec:
        front_avgs[key] = _print_stopwatch_line(label, topic, stats)

    print("-" * 88)
    print("  【后半段】")
    ctrl_xcorr = None
    motor_xcorr = None
    traj_sens = None
    filt_ms = None
    n_j = len(all_delays)

    def _stage_mean(key):
        vals = [d[key] for d in all_delays if np.isfinite(d.get(key, float("nan")))]
        if not vals:
            return None, 0
        return float(np.mean(vals)), len(vals)

    if n_j:
        ctrl_xcorr, n_ctrl = _stage_mean("traj_cmd_delay")
        motor_xcorr, n_motor = _stage_mean("cmd_sens_delay")
        traj_sens, n_ts = _stage_mean("traj_sens_delay")
        filt_ms, n_filt = _stage_mean("traj_filtered_delay")
        if ctrl_xcorr is not None:
            print(
                f"  {'3.控制器 /joint_cmd滞后IK':28s}  {ctrl_xcorr:+8.2f} ms"
                f"  (互相关, {n_ctrl}/{n_j}关节)"
            )
        else:
            print("  3.控制器 /joint_cmd滞后IK       缺失  (无可靠关节)")
        wbc = _avg_or_none(stats, CTRL_STOPWATCH_TOPIC)
        if wbc is not None:
            print(
                f"  {'   对照 控制器 stopwatch':28s}  {wbc:8.2f} ms"
                f"  ({CTRL_STOPWATCH_TOPIC})"
            )
        if motor_xcorr is not None:
            print(
                f"  {'4.电机+反馈 传感器滞后cmd':28s}  {motor_xcorr:+8.2f} ms"
                f"  (互相关, {n_motor}/{n_j}关节)"
            )
        else:
            print("  4.电机+反馈                     缺失  (无可靠关节)")
        if mode == "incremental" and filt_ms is not None:
            print(
                f"  {'   对照 控制器滤波相位':28s}  {filt_ms:+8.2f} ms"
                f"  (filtered 滞后 traj, {n_filt}/{n_j}关节)"
            )
    else:
        print("  3.控制器 / 4.电机+反馈           缺失  (关节话题不足, 无法互相关)")

    print("-" * 88)
    print("  【端到端】")
    front_vals = [front_avgs.get(key) for key, _topic, _label in front_spec]
    parts = list(front_vals) + [ctrl_xcorr, motor_xcorr]
    if all(v is not None for v in parts):
        e2e_sum = float(sum(parts))
        print(
            f"  {'链路相加 (前半段+控制+电机)':28s}  {e2e_sum:8.2f} ms"
        )
    else:
        e2e_sum = None
        missing = []
        names = [label for _k, _t, label in front_spec] + ["控制器", "电机+反馈"]
        for n, v in zip(names, parts):
            if v is None:
                missing.append(n)
        print(f"  链路相加                         无法计算, 缺: {', '.join(missing)}")

    if all(v is not None for v in front_vals) and front_vals:
        front_sum = float(sum(front_vals))
        print(f"  {'软件前半段':28s}  {front_sum:8.2f} ms")
        if traj_sens is not None:
            print(
                f"  {'前半段 + IK输出→传感器':28s}  {front_sum + traj_sens:8.2f} ms"
            )
    else:
        front_sum = None

    if traj_sens is not None:
        print(
            f"  {'IK输出→传感器 (不含前半段)':28s}  {traj_sens:+8.2f} ms"
            f"  (互相关, 传感器滞后 IK轨迹)"
        )

    for topic, label in E2E_STOPWATCH_TOPICS:
        if topic in stats:
            _print_stopwatch_line(label, topic, stats)

    show_fine = fine or mode == "absolute"
    if show_fine:
        fine_present = [row for row in IK_FINE_TOPICS if row[0] in stats]
        if fine_present:
            print("-" * 88)
            print("  【IK 细分】")
            for topic, label in fine_present:
                _print_stopwatch_line(label, topic, stats)
        elif fine:
            print("-" * 88)
            print("  【IK 细分】 bag 中无绝对式细分话题")

    print("=" * 88)
    print()

    return {
        "mode": mode,
        "link": link_label or mode,
        "traj_topic": topic_arm_traj,
        "vr_ms": front_avgs.get("vr"),
        "bone_hop_ms": front_avgs.get("bone_hop"),
        "eef_ms": front_avgs.get("eef"),
        "hop_ms": front_avgs.get("hop"),
        "ik_ms": front_avgs.get("ik"),
        "controller_xcorr_ms": ctrl_xcorr,
        "controller_stopwatch_ms": _avg_or_none(stats, CTRL_STOPWATCH_TOPIC),
        "motor_xcorr_ms": motor_xcorr,
        "traj_sens_xcorr_ms": traj_sens,
        "filter_xcorr_ms": filt_ms if mode == "incremental" else None,
        "e2e_four_stage_ms": e2e_sum,
        "e2e_front_ms": front_sum,
        "n_joints": n_j,
    }


def save_latency_report_csv(report: dict, stats: dict, all_delays: List[dict],
                            out_path: str) -> None:
    import csv
    with open(out_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["section", "metric", "value_ms", "note"])
        w.writerow(["meta", "link", "", report.get("link") or ""])
        w.writerow(["meta", "mode", "", report.get("mode") or ""])
        w.writerow(["meta", "traj_topic", "", report.get("traj_topic") or ""])
        w.writerow(["meta", "n_joints", report.get("n_joints"), "互相关关节数"])
        rows = [
            ("front", "1.VR数据处理", report.get("vr_ms"), "Float64"),
            ("front", "层间骨骼话题通信", report.get("bone_hop_ms"), "Pico /leju_pico_bone_poses"),
            ("front", "Pico末端指令处理", report.get("eef_ms"), "骨骼回调→/mm/two_arm_hand_pose_cmd"),
            ("front", "层间VR→IK通信", report.get("hop_ms"), "Float64"),
            ("front", "2.IK解算", report.get("ik_ms"), "Float64"),
            ("back", "3.控制器 cmd滞后traj", report.get("controller_xcorr_ms"), "互相关 下游滞后"),
            ("back", "控制器 stopwatch", report.get("controller_stopwatch_ms"), "Float64"),
            ("back", "4.电机+反馈 sens滞后cmd", report.get("motor_xcorr_ms"), "互相关 下游滞后"),
            ("e2e", "链路相加", report.get("e2e_four_stage_ms"), "前半段+控制+电机"),
            ("e2e", "软件前半段", report.get("e2e_front_ms"), "VR+中间节点+IK"),
            ("e2e", "IK输出→传感器", report.get("traj_sens_xcorr_ms"), "互相关"),
        ]
        for section, metric, value, note in rows:
            val = "" if value is None else f"{value:.4f}"
            w.writerow([section, metric, val, note])
        w.writerow([])
        w.writerow(["# stopwatch topics in bag"])
        w.writerow(["topic", "n", "avg_ms", "std_ms", "min_ms", "max_ms"])
        for topic, st in stats.items():
            w.writerow([topic, st["n"], f"{st['avg']:.4f}", f"{st['std']:.4f}",
                        f"{st['min']:.4f}", f"{st['max']:.4f}"])
        if all_delays:
            w.writerow([])
            w.writerow(["# per-joint cross-correlation (ms)"])
            w.writerow(["joint", "side", "traj_cmd", "cmd_sens", "traj_sens",
                        "traj_filtered"])
            for d in all_delays:
                w.writerow([
                    d["joint"], d["side"],
                    f"{d['traj_cmd_delay']:.4f}",
                    f"{d['cmd_sens_delay']:.4f}",
                    f"{d['traj_sens_delay']:.4f}",
                    f"{d['traj_filtered_delay']:.4f}",
                ])
    print(f"[INFO] 延迟诊断报告已保存: {out_path}")


def plot_stage_report(report: dict, out_path: str) -> None:
    labels = [
        "1.VR",
        "VR→IK通信",
        "2.IK",
        "3.控制器",
        "4.电机+反馈",
        "四段相加",
    ]
    keys = ["vr_ms", "hop_ms", "ik_ms", "controller_xcorr_ms", "motor_xcorr_ms",
            "e2e_four_stage_ms"]
    vals = [report.get(k) for k in keys]
    if all(v is None for v in vals):
        return
    plot_vals = [0.0 if v is None else float(v) for v in vals]
    colors = ["#4c78a8", "#f58518", "#54a24b", "#e45756", "#72b7b2", "#9d755d"]
    fig, ax = plt.subplots(figsize=(10, 5))
    x = np.arange(len(labels))
    bars = ax.bar(x, plot_vals, color=colors, alpha=0.9)
    for i, (bar, raw) in enumerate(zip(bars, vals)):
        if raw is None:
            ax.text(bar.get_x() + bar.get_width() / 2, 0.0, "缺失",
                    ha="center", va="bottom", fontsize=8)
        else:
            ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height(),
                    f"{raw:.1f}", ha="center", va="bottom", fontsize=8)
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_ylabel("延迟 (ms)")
    title = report.get("link") or report.get("mode") or ""
    ax.set_title(f"延迟诊断  {title}")
    ax.grid(True, axis="y", alpha=0.3)
    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"[INFO] 四段汇总图已保存: {out_path}")


# ==============================================================================
# 延迟估计 (互相关法)
# ==============================================================================
def _resample_uniform_abs(times: np.ndarray, values: np.ndarray, dt: float,
                          t_start: float, t_end: float
                          ) -> Tuple[np.ndarray, np.ndarray]:
    """在绝对时间区间 [t_start, t_end] 上以等间隔 dt 重采样 (线性插值)。

    Returns:
        t_uniform (绝对时间), v_uniform
    """
    n = int(math.floor((t_end - t_start) / dt)) + 1
    if n < 2:
        return np.array([]), np.array([])
    t_uniform = t_start + np.arange(n) * dt
    v_uniform = np.interp(t_uniform, times, values)
    return t_uniform, v_uniform


def _overlap_interval(a: TopicData, b: TopicData) -> Tuple[float, float]:
    """返回两个话题绝对时间的重叠区间 [t_lo, t_hi]。"""
    if len(a) == 0 or len(b) == 0:
        return 0.0, 0.0
    t_lo = max(float(a.times[0]), float(b.times[0]))
    t_hi = min(float(a.times[-1]), float(b.times[-1]))
    if t_hi <= t_lo:
        return 0.0, 0.0
    return t_lo, t_hi


def estimate_delay_cross_corr(a: TopicData, b: TopicData, max_lag: float = 2.0
                               ) -> Tuple[float, float]:
    """用归一化互相关估计 a 相对 b 的延迟 (基于绝对时间重叠区间)。

    返回 (delay_sec, peak_corr)
    delay > 0 表示 a 滞后于 b (a 是后发生的); delay < 0 表示 a 超前于 b。

    原理: 若 a(t) ≈ b(t - delay), 即 a 是 b 延迟 delay 秒后的版本,
    则互相关 correlate(a, b) 在 lag = +delay_samples 处取峰值。
    """
    if len(a) < 2 or len(b) < 2:
        return 0.0, 0.0

    # 只在绝对时间重叠区间内做互相关, 避免非重叠段引入偏差
    t_lo, t_hi = _overlap_interval(a, b)
    if t_hi <= t_lo:
        return 0.0, 0.0

    # 选择统一采样间隔 (取两者中较细的)
    dt_a = np.median(np.diff(a.times)) if len(a.times) > 1 else 0.01
    dt_b = np.median(np.diff(b.times)) if len(b.times) > 1 else 0.01
    dt = float(min(dt_a, dt_b))
    if dt <= 0:
        dt = 0.01

    ta, va = _resample_uniform_abs(a.times, a.values, dt, t_lo, t_hi)
    tb, vb = _resample_uniform_abs(b.times, b.values, dt, t_lo, t_hi)
    if len(ta) < 4 or len(tb) < 4:
        return 0.0, 0.0

    # 对齐到相同长度 (理论上已相同, 但保险起见)
    n = min(len(ta), len(tb))
    va = va[:n]
    vb = vb[:n]

    # 去均值 (去除直流分量, 突出波形相似性)
    va = va - np.mean(va)
    vb = vb - np.mean(vb)

    # 限制最大 lag 样本数
    max_lag_samples = int(max_lag / dt)
    max_lag_samples = min(max_lag_samples, n - 1)
    if max_lag_samples < 1:
        return 0.0, 0.0

    # scipy.signal.correlate 返回长度 2n-1, 中心在 n-1
    corr = correlate(va, vb, mode="full", method="auto")
    # 归一化
    norm = np.sqrt(np.sum(va * va) * np.sum(vb * vb))
    if norm < 1e-12:
        return 0.0, 0.0
    corr = corr / norm

    center = n - 1
    # 只在 [-max_lag_samples, +max_lag_samples] 范围内找峰值
    lo = center - max_lag_samples
    hi = center + max_lag_samples + 1
    lo = max(0, lo)
    hi = min(len(corr), hi)
    search = corr[lo:hi]
    if len(search) == 0:
        return 0.0, 0.0
    peak = int(np.argmax(np.abs(search)))
    peak_idx = lo + peak
    lag_samples = peak_idx - center
    delay = lag_samples * dt
    peak_corr = float(corr[peak_idx])
    # 互相关峰值符号: corr(a, b) 在 lag>0 处取峰 => a 滞后 b
    return float(delay), peak_corr


def estimate_delay_ls(a: TopicData, b: TopicData, max_lag: float = 2.0
                      ) -> Tuple[float, float]:
    """用最小二乘拟合估计 a 相对 b 的延迟 (基于绝对时间重叠区间, 作为互相关法的交叉验证)。

    模型: a(t) ≈ b(t - delay)  =>  delay>0 表示 a 滞后 b
    通过在 [-max_lag, max_lag] 范围内扫描, 找最小化 ||a(t) - b(t-delay)|| 的 delay。
    """
    if len(a) < 2 or len(b) < 2:
        return 0.0, 0.0

    t_lo, t_hi = _overlap_interval(a, b)
    if t_hi <= t_lo:
        return 0.0, 0.0

    dt_a = np.median(np.diff(a.times)) if len(a.times) > 1 else 0.01
    dt_b = np.median(np.diff(b.times)) if len(b.times) > 1 else 0.01
    dt = float(min(dt_a, dt_b))
    if dt <= 0:
        dt = 0.01

    ta, va = _resample_uniform_abs(a.times, a.values, dt, t_lo, t_hi)
    tb, vb = _resample_uniform_abs(b.times, b.values, dt, t_lo, t_hi)
    if len(ta) < 4 or len(tb) < 4:
        return 0.0, 0.0

    n = min(len(ta), len(tb))
    va = va[:n]
    vb = vb[:n]
    t = tb[:n]

    def residual(d):
        shifted = np.interp(t - d, t, vb, left=vb[0], right=vb[-1])
        return va - shifted

    # 粗扫描
    best_d = 0.0
    best_err = np.inf
    for d in np.arange(-max_lag, max_lag + dt, dt):
        r = residual(d)
        err = float(np.sum(r * r))
        if err < best_err:
            best_err = err
            best_d = d

    # 精细化 (least_squares)
    try:
        res = least_squares(residual, best_d, bounds=(-max_lag, max_lag), method="trf")
        best_d = float(res.x[0])
        r = residual(best_d)
        best_err = float(np.sum(r * r))
    except Exception:
        pass

    # 相关系数
    ss_tot = float(np.sum((va - np.mean(va)) ** 2))
    r2 = 1.0 - (best_err / ss_tot) if ss_tot > 1e-12 else 0.0
    return best_d, float(math.sqrt(max(0.0, r2)))


def estimate_downstream_lag(
    upstream: TopicData, downstream: TopicData, max_lag: float = 2.0
) -> Tuple[Tuple[float, float], Tuple[float, float]]:
    """下游相对上游的滞后。delay>0 表示 downstream 比 upstream 晚发生。

    互相关与最小二乘均按 (downstream, upstream) 估计, 与四段报告「下游滞后」一致。
    返回 ((xcorr_sec, r), (ls_sec, r2)).
    """
    xcorr = estimate_delay_cross_corr(downstream, upstream, max_lag)
    ls = estimate_delay_ls(downstream, upstream, max_lag)
    return xcorr, ls


def _at_search_bound(delay_sec: float, max_lag: float) -> bool:
    return abs(float(delay_sec)) >= float(max_lag) * LAG_BOUND_FRAC


def select_reliable_lag(
    xcorr: Tuple[float, float],
    ls: Tuple[float, float],
    max_lag: float,
) -> Tuple[Tuple[float, float], bool, str]:
    """从互相关 / 最小二乘中选出用于总报告的延迟.

    互相关顶到搜索边界、或与最小二乘相差过大时改用最小二乘, 避免单关节污染总平均.
    两者都顶到边界则不参与总平均.
    返回 ((delay_sec, r), ok, note).
    """
    xd, xr = xcorr
    ld, lr = ls
    x_bound = _at_search_bound(xd, max_lag)
    l_bound = _at_search_bound(ld, max_lag)
    if x_bound and l_bound:
        return (ld, lr), False, "互相关与最小二乘均顶到搜索边界, 不参与总平均"
    if x_bound and not l_bound:
        return (ld, lr), True, "互相关顶到搜索边界, 改用最小二乘"
    if (not l_bound) and abs(float(xd) - float(ld)) > LAG_DISAGREE_SEC:
        return (ld, lr), True, "互相关与最小二乘相差过大, 改用最小二乘"
    return (xd, xr), True, ""


# ==============================================================================
# 绘图
# ==============================================================================
def plot_all(traj: TopicData, cmd: TopicData, sens: TopicData,
             joint_index: int, out_path: str,
             delay_traj_cmd: Tuple[float, float],
             delay_cmd_sens: Tuple[float, float],
             delay_traj_sens: Tuple[float, float] = (0.0, 0.0),
             single_arm: int = 7) -> None:
    """绘制三条曲线在一张图上, 并标注延迟。

    使用绝对时间 (相对三话题最早起点), 这样曲线在时间轴上自然对齐。
    delay>0 表示下游滞后上游: 控制器=/joint_cmd 滞后 IK轨迹, 电机=传感器滞后 /joint_cmd。
    """
    fig, ax = plt.subplots(figsize=(14, 7))

    # 统一时间基准: 三话题中最早的起点
    starts = []
    for d in (traj, cmd, sens):
        if len(d):
            starts.append(float(d.times[0]))
    t0 = min(starts) if starts else 0.0

    # 绝对时间 (减去公共基准), 保持时间对齐
    t_traj = (traj.times - t0) if len(traj) else np.array([])
    t_cmd = (cmd.times - t0) if len(cmd) else np.array([])
    t_sens = (sens.times - t0) if len(sens) else np.array([])

    ax.plot(t_traj, traj.values, color="#1f77b4", linewidth=1.2,
            label=f"{traj.name} (deg, IK 输出)", alpha=0.9)
    ax.plot(t_cmd, cmd.values, color="#ff7f0e", linewidth=1.2,
            label=f"{TOPIC_JOINT_CMD} (deg, 控制器输出)", alpha=0.9)
    ax.plot(t_sens, sens.values, color="#2ca02c", linewidth=1.2,
            label=f"{TOPIC_SENSORS} (deg, 传感器反馈)", alpha=0.9)

    ax.set_xlabel("Time (s, 绝对时间对齐)")
    ax.set_ylabel("Joint Angle (deg)")
    side = "左臂" if joint_index <= single_arm else "右臂"
    ax.set_title(
        f"关节延迟对比  |  关节 #{joint_index} ({side})  |  "
        f"控制器 cmd滞后traj: {delay_traj_cmd[0]*1000:+.1f}ms (r={delay_traj_cmd[1]:.2f})  |  "
        f"电机+反馈 sens滞后cmd: {delay_cmd_sens[0]*1000:+.1f}ms (r={delay_cmd_sens[1]:.2f})")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=9)

    textstr = (
        "delay>0 表示下游滞后上游\n"
        "3.控制器: /joint_cmd 滞后 IK轨迹\n"
        "4.电机+反馈: 传感器滞后 /joint_cmd\n"
        "时间轴为绝对时间对齐"
    )
    props = dict(boxstyle="round", facecolor="wheat", alpha=0.5)
    ax.text(0.015, 0.985, textstr, transform=ax.transAxes, fontsize=8,
            verticalalignment="top", bbox=props)

    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"[INFO] 曲线图已保存: {out_path}")


def plot_summary(all_delays: List[dict], out_path: str, single_arm: int = 7) -> None:
    """绘制所有关节延迟汇总柱状图 + 平均值标注。

    all_delays: list of dict, 每个元素为:
        {
            "joint": int (1-based),
            "side": str,
            "traj_cmd_delay": float (ms),
            "traj_cmd_r": float,
            "cmd_sens_delay": float (ms),
            "cmd_sens_r": float,
            "traj_sens_delay": float (ms),
            "traj_sens_r": float,
        }
    """
    n = len(all_delays)
    if n == 0:
        print("[WARN] 无有效延迟数据, 跳过汇总图")
        return

    joints = [d["joint"] for d in all_delays]
    labels = [f"#{j}\n({d['side']})" for j, d in zip(joints, all_delays)]
    x = np.arange(n)
    width = 0.25

    traj_cmd_vals = [d["traj_cmd_delay"] for d in all_delays]
    cmd_sens_vals = [d["cmd_sens_delay"] for d in all_delays]
    traj_sens_vals = [d["traj_sens_delay"] for d in all_delays]

    # 计算平均值 (仅可靠关节)
    def _safe_mean(vals):
        arr = np.asarray(vals, dtype=float)
        arr = arr[np.isfinite(arr)]
        return float(np.mean(arr)) if arr.size else float("nan")
    def _safe_std(vals):
        arr = np.asarray(vals, dtype=float)
        arr = arr[np.isfinite(arr)]
        return float(np.std(arr)) if arr.size else float("nan")

    mean_tc = _safe_mean(traj_cmd_vals)
    std_tc = _safe_std(traj_cmd_vals)
    mean_cs = _safe_mean(cmd_sens_vals)
    std_cs = _safe_std(cmd_sens_vals)
    mean_ts = _safe_mean(traj_sens_vals)
    std_ts = _safe_std(traj_sens_vals)

    fig, ax = plt.subplots(figsize=(max(12, n * 0.8), 6))

    bars1 = ax.bar(x - width, traj_cmd_vals, width, color="#1f77b4", alpha=0.85,
                   label="3.控制器 cmd滞后traj")
    bars2 = ax.bar(x, cmd_sens_vals, width, color="#ff7f0e", alpha=0.85,
                   label="4.电机+反馈 sens滞后cmd")
    bars3 = ax.bar(x + width, traj_sens_vals, width, color="#2ca02c", alpha=0.85,
                   label="sens滞后traj (总表观)")

    # 平均值虚线
    ax.axhline(y=mean_tc, color="#1f77b4", linestyle="--", linewidth=1.5, alpha=0.7)
    ax.axhline(y=mean_cs, color="#ff7f0e", linestyle="--", linewidth=1.5, alpha=0.7)
    ax.axhline(y=mean_ts, color="#2ca02c", linestyle="--", linewidth=1.5, alpha=0.7)

    ax.set_xlabel("关节")
    ax.set_ylabel("延迟 (ms)")
    ax.set_title(
        f"Quest3 控制器/电机延迟汇总 (N={n})  |  "
        f"控制器: {mean_tc:+.1f}±{std_tc:.1f}ms  |  "
        f"电机+反馈: {mean_cs:+.1f}±{std_cs:.1f}ms"
    )
    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=8)
    ax.grid(True, alpha=0.3, axis="y")
    ax.legend(loc="best", fontsize=9)

    # 文本框: 平均延迟
    textstr = (
        f"3.控制器 cmd滞后traj:  {mean_tc:+.1f} ± {std_tc:.1f} ms\n"
        f"4.电机+反馈 sens滞后cmd:  {mean_cs:+.1f} ± {std_cs:.1f} ms\n"
        f"sens滞后traj (总表观): {mean_ts:+.1f} ± {std_ts:.1f} ms"
    )
    props = dict(boxstyle="round", facecolor="wheat", alpha=0.5)
    ax.text(0.015, 0.985, textstr, transform=ax.transAxes, fontsize=9,
            verticalalignment="top", bbox=props)

    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"[INFO] 汇总图已保存: {out_path}")


# ==============================================================================
# CSV 保存
# ==============================================================================
def save_csv(traj: TopicData, cmd: TopicData, sens: TopicData, out_path: str) -> None:
    """将三组数据保存为 CSV (各自时间戳, 不对齐)。"""
    import csv
    with open(out_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["# 关节角度数据 (deg), 时间为相对 bag 起点的秒数"])
        w.writerow(["topic", "time_s", "angle_deg"])
        for name, data in [(traj.name, traj), ("joint_cmd", cmd), ("sensors_data_raw", sens)]:
            if len(data) == 0:
                continue
            t0 = data.times[0]
            for tt, vv in zip(data.times, data.values):
                w.writerow([name, f"{tt - t0:.6f}", f"{vv:.6f}"])
    print(f"[INFO] CSV 已保存: {out_path}")


# ==============================================================================
# 主流程
# ==============================================================================
def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="从 rosbag 做 Quest3 延迟诊断: 前半段读延迟话题, 后半段关节互相关, "
                    "输出四段+端到端报告。",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument("-b", "--bag", required=True, help="rosbag 文件路径")
    p.add_argument("-j", "--joint", type=int, default=None,
                   help="手臂关节序号 (1-based, 1..N=左臂, N+1..2N=右臂, 默认=所有关节)")
    p.add_argument("--mode", choices=list(ARM_TRAJ_TOPICS.keys()), default="incremental",
                   help="遥操模式: incremental=/vr_incremental/kuavo_arm_traj_shm, "
                        "absolute=/kuavo_arm_traj, pico=/mm_kuavo_arm_traj。"
                        "若同时给 --preset, 以 preset 的 mode 为准")
    p.add_argument(
        "--preset",
        choices=list(QUEST3_PRESETS.keys()),
        default=None,
        help="遥操链路预设: "
             "q3-abs-humanoid / q3-inc-humanoid / q3-abs-wheel / q3-inc-wheel / "
             "pico-abs-humanoid",
    )
    p.add_argument("--robot-version", type=int, default=None,
                   help="机器人版本 (影响 arm_offset, 默认从 ROBOT_VERSION 环境变量或 45)")
    p.add_argument("--assets-root", default=None,
                   help="kuavo_assets/config 路径 (默认自动定位)")
    p.add_argument("-o", "--output-dir",
                   default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "png", "joint_delay"),
                   help="输出目录 (默认: tools/bag_tools/png/joint_delay)")
    p.add_argument("-n", "--name", default=None,
                   help="输出文件名前缀 (不含扩展名, 默认与 bag 同名)")
    p.add_argument("--csv", action="store_true", help="同时保存各关节角度 CSV (延迟报告 CSV 总会写)")
    p.add_argument("--fine", action="store_true",
                   help="额外汇总绝对式 IK 细分延迟话题 (ik_solve 等)")
    p.add_argument("--max-lag", type=float, default=2.0,
                   help="延迟估计最大搜索范围 (秒, 默认 2.0)")
    p.add_argument("--no-plot", action="store_true", help="不绘图")
    return p.parse_args()


def main() -> int:
    args = parse_args()

    if not os.path.isfile(args.bag):
        print(f"[FATAL] bag 文件不存在: {args.bag}")
        return 1

    link_label = None
    if args.preset:
        preset = QUEST3_PRESETS[args.preset]
        args.mode = preset["mode"]
        link_label = preset["label"]
        print(f"[INFO] 延迟诊断预设: {args.preset} ({link_label}) → mode={args.mode}")
        if args.robot_version is None:
            env_ver = os.environ.get("ROBOT_VERSION")
            if "wheel" in args.preset:
                print(
                    "[WARN] 轮臂预设未给 --robot-version; "
                    "当前使用 ROBOT_VERSION=%s (未设置则默认 45)。轮臂请设 61/62/63。"
                    % (env_ver if env_ver else "未设置")
                )
            elif "humanoid" in args.preset:
                print(
                    "[WARN] 人形预设未给 --robot-version; "
                    "当前使用 ROBOT_VERSION=%s (未设置则默认 45, arm_offset=12)。"
                    "带腰的人形 (如 v55) 下肢+腰共 13 轴, 请设 --robot-version 55。"
                    % (env_ver if env_ver else "未设置")
                )

    os.makedirs(args.output_dir, exist_ok=True)

    # 默认文件名与 bag 同名
    name = args.name or os.path.splitext(os.path.basename(args.bag))[0]

    # 加载布局
    layout = load_layout(args.robot_version, args.assets_root)
    single_arm = layout["single_arm"]
    n_total_arm = 2 * single_arm
    topic_arm_traj = ARM_TRAJ_TOPICS[args.mode]
    print(f"[INFO] 机器人布局: {layout}")
    print(f"[INFO] 单臂关节数: {single_arm}, 手臂关节总数: {n_total_arm}")
    print(f"[INFO] 遥操模式: {args.mode}, 手臂轨迹话题: {topic_arm_traj}")
    if link_label:
        print(f"[INFO] 链路: {link_label}")

    # 确定要处理的关节列表
    if args.joint is not None:
        joint_list = [args.joint]
        if not (1 <= args.joint <= n_total_arm):
            print(f"[FATAL] 关节索引 {args.joint} 超出范围 [1, {n_total_arm}]")
            return 1
    else:
        joint_list = list(range(1, n_total_arm + 1))

    print(f"[INFO] 待处理关节: {joint_list}")
    print(f"[INFO] 正在从 bag 提取数据: {args.bag}")

    latency_topics = collect_latency_topics(args.mode, fine=args.fine)
    latency_stats = extract_float64_topic_stats(args.bag, latency_topics)

    # ---- 批量提取 (一次遍历) ----
    if args.joint is not None:
        # 单关节模式: 用原有逐关节提取, 保持兼容
        trajs = [extract_arm_traj(args.bag, args.joint, single_arm, topic_arm_traj)]
        cmds = [extract_joint_cmd(args.bag, args.joint, layout)]
        senses = [extract_sensors(args.bag, args.joint, layout)]
        filtereds = [extract_arm_traj(args.bag, args.joint, single_arm, TOPIC_ARM_TRAJ_FILTERED)]
    else:
        # 全关节模式: 批量一次遍历提取
        trajs = extract_arm_traj_all(args.bag, single_arm, topic_arm_traj)
        cmds = extract_joint_cmd_all(args.bag, layout)
        senses = extract_sensors_all(args.bag, layout)
        filtereds = extract_arm_traj_all(args.bag, single_arm, TOPIC_ARM_TRAJ_FILTERED)

    # 打印数据量概览
    for i, (traj, cmd, sens, filtered) in enumerate(zip(trajs, cmds, senses, filtereds)):
        jidx = joint_list[i]
        print(f"[INFO] 关节 #{jidx}: traj={len(traj)}f, cmd={len(cmd)}f, "
              f"sens={len(sens)}f, filtered={len(filtered)}f")

    # ---- 逐关节计算延迟 & 绘图 ----
    all_delays: List[dict] = []  # 汇总用

    for i, jidx in enumerate(joint_list):
        traj = trajs[i]
        cmd = cmds[i]
        sens = senses[i]
        filtered = filtereds[i]

        side = "左臂" if jidx <= single_arm else "右臂"
        arm_local = jidx if jidx <= single_arm else jidx - single_arm
        print(f"\n--- 关节 #{jidx} ({side} 臂内 #{arm_local}) ---")

        if len(traj) == 0 or len(cmd) == 0 or len(sens) == 0:
            print(f"[WARN] 关节 #{jidx} 存在空话题, 跳过延迟估计")
            continue

        # 延迟估计
        delay_traj_cmd = (0.0, 0.0)
        delay_cmd_sens = (0.0, 0.0)
        delay_traj_sens = (0.0, 0.0)
        delay_traj_filtered = (0.0, 0.0)
        traj_cmd_ok = cmd_sens_ok = traj_sens_ok = filt_ok = True

        if len(traj) >= 2 and len(cmd) >= 2:
            xcorr, ls = estimate_downstream_lag(traj, cmd, args.max_lag)
            delay_traj_cmd, traj_cmd_ok, traj_cmd_note = select_reliable_lag(
                xcorr, ls, args.max_lag)
            print(f"  控制器 cmd滞后traj 互相关: delay={xcorr[0]*1000:+.1f}ms, r={xcorr[1]:.3f} | "
                  f"最小二乘: delay={ls[0]*1000:+.1f}ms, r={ls[1]:.3f}")
            if traj_cmd_note:
                print(f"    选用 {delay_traj_cmd[0]*1000:+.1f}ms  ({traj_cmd_note})")

        if len(cmd) >= 2 and len(sens) >= 2:
            xcorr, ls = estimate_downstream_lag(cmd, sens, args.max_lag)
            delay_cmd_sens, cmd_sens_ok, cmd_sens_note = select_reliable_lag(
                xcorr, ls, args.max_lag)
            print(f"  电机   sens滞后cmd 互相关: delay={xcorr[0]*1000:+.1f}ms, r={xcorr[1]:.3f} | "
                  f"最小二乘: delay={ls[0]*1000:+.1f}ms, r={ls[1]:.3f}")
            if cmd_sens_note:
                print(f"    选用 {delay_cmd_sens[0]*1000:+.1f}ms  ({cmd_sens_note})")

        if len(traj) >= 2 and len(sens) >= 2:
            xcorr, ls = estimate_downstream_lag(traj, sens, args.max_lag)
            delay_traj_sens, traj_sens_ok, traj_sens_note = select_reliable_lag(
                xcorr, ls, args.max_lag)
            print(f"  总表观 sens滞后traj 互相关: delay={xcorr[0]*1000:+.1f}ms, r={xcorr[1]:.3f}")
            if traj_sens_note:
                print(f"    选用 {delay_traj_sens[0]*1000:+.1f}ms  ({traj_sens_note})")

        delay_traj_filtered = (0.0, 0.0)
        filt_ok = True
        if args.mode == "incremental" and len(traj) >= 2 and len(filtered) >= 2:
            xcorr, ls = estimate_downstream_lag(traj, filtered, args.max_lag)
            delay_traj_filtered, filt_ok, filt_note = select_reliable_lag(
                xcorr, ls, args.max_lag)
            print(f"  滤波   filtered滞后traj 互相关: delay={xcorr[0]*1000:+.1f}ms, r={xcorr[1]:.3f} | "
                  f"最小二乘: delay={ls[0]*1000:+.1f}ms, r={ls[1]:.3f}")
            if filt_note:
                print(f"    选用 {delay_traj_filtered[0]*1000:+.1f}ms  ({filt_note})")

        # 记录汇总数据 (单位 ms); 不可靠样本记 nan, 不进总平均
        def _ms_or_nan(ok, pair):
            return (pair[0] * 1000.0) if ok else float("nan")

        all_delays.append({
            "joint": jidx,
            "side": side,
            "arm_local": arm_local,
            "traj_cmd_delay": _ms_or_nan(traj_cmd_ok, delay_traj_cmd),
            "traj_cmd_r": delay_traj_cmd[1],
            "traj_cmd_ok": traj_cmd_ok,
            "cmd_sens_delay": _ms_or_nan(cmd_sens_ok, delay_cmd_sens),
            "cmd_sens_r": delay_cmd_sens[1],
            "cmd_sens_ok": cmd_sens_ok,
            "traj_sens_delay": _ms_or_nan(traj_sens_ok, delay_traj_sens),
            "traj_sens_r": delay_traj_sens[1],
            "traj_sens_ok": traj_sens_ok,
            "traj_filtered_delay": _ms_or_nan(filt_ok, delay_traj_filtered),
            "traj_filtered_r": delay_traj_filtered[1],
            "traj_filtered_ok": filt_ok,
        })

        # 单个关节绘图
        if not args.no_plot:
            joint_name = f"{name}_joint{jidx:02d}"
            plot_path = os.path.join(args.output_dir, f"{joint_name}.png")
            plot_all(traj, cmd, sens, jidx, plot_path,
                     delay_traj_cmd, delay_cmd_sens, delay_traj_sens,
                     single_arm=single_arm)

        # CSV (单关节)
        if args.csv:
            csv_path = os.path.join(args.output_dir, f"{name}_joint{jidx:02d}.csv")
            save_csv(traj, cmd, sens, csv_path)

    # ---- 汇总 ----
    if len(all_delays) == 0:
        print("\n[WARN] 无有效关节互相关数据 (控制器/电机层)")
    else:
        print("\n" + "=" * 70)
        print("                      全关节延迟汇总")
        print("=" * 70)
        def _fmt_ms(val):
            return "     跳过" if not np.isfinite(val) else f"{val:+9.1f}ms"

        print(f"{'关节':>6}  {'侧':>4}  {'cmd滞后':>10}  {'r':>6}  {'sens滞后cmd':>12}  {'r':>6}  {'sens滞后traj':>12}  {'r':>6}  {'filt滞后':>10}")
        print("-" * 70)
        for d in all_delays:
            print(f"#{d['joint']:>4}  {d['side']:>4}  {_fmt_ms(d['traj_cmd_delay'])}  {d['traj_cmd_r']:.3f}  "
                  f"{_fmt_ms(d['cmd_sens_delay'])}  {d['cmd_sens_r']:.3f}  "
                  f"{_fmt_ms(d['traj_sens_delay'])}  {d['traj_sens_r']:.3f}  "
                  f"{_fmt_ms(d['traj_filtered_delay'])}")
        print("-" * 70)

        def _mean_ok(key):
            vals = [d[key] for d in all_delays if np.isfinite(d[key])]
            if not vals:
                return float("nan"), float("nan"), 0
            arr = np.asarray(vals, dtype=float)
            return float(np.mean(arr)), float(np.std(arr)), len(vals)

        mean_tc, std_tc, n_tc = _mean_ok("traj_cmd_delay")
        mean_cs, std_cs, n_cs = _mean_ok("cmd_sens_delay")
        mean_ts, std_ts, n_ts = _mean_ok("traj_sens_delay")
        mean_tf, std_tf, n_tf = _mean_ok("traj_filtered_delay")

        def _fmt_avg(mean, std, n):
            if n == 0 or not np.isfinite(mean):
                return "      无有效样本"
            return f"{mean:+9.1f}±{std:.1f} (n={n})"

        print(f"{'平均':>6}        {_fmt_avg(mean_tc, std_tc, n_tc):<22}  "
              f"{_fmt_avg(mean_cs, std_cs, n_cs):<22}  "
              f"{_fmt_avg(mean_ts, std_ts, n_ts):<22}  "
              f"{_fmt_avg(mean_tf, std_tf, n_tf)}")
        print("=" * 70)
        n_skip = sum(1 for d in all_delays if not d.get("traj_cmd_ok", True)
                     or not d.get("cmd_sens_ok", True)
                     or not d.get("traj_sens_ok", True))
        if n_skip:
            print("[INFO] 部分关节互相关顶到搜索边界或与最小二乘严重不一致, 已排除出总平均")

        left_delays = [d for d in all_delays if d["joint"] <= single_arm]
        right_delays = [d for d in all_delays if d["joint"] > single_arm]
        for side_name, side_data in [("左臂", left_delays), ("右臂", right_delays)]:
            def _side_mean(key):
                vals = [d[key] for d in side_data if np.isfinite(d[key])]
                if len(vals) < 1:
                    return None
                arr = np.asarray(vals, dtype=float)
                return float(np.mean(arr)), float(np.std(arr)), len(vals)

            tc = _side_mean("traj_cmd_delay")
            cs = _side_mean("cmd_sens_delay")
            ts = _side_mean("traj_sens_delay")
            if tc is None and cs is None:
                continue
            parts = []
            if tc:
                parts.append(f"cmd滞后traj: {tc[0]:+.1f}±{tc[1]:.1f}ms (n={tc[2]})")
            if cs:
                parts.append(f"sens滞后cmd: {cs[0]:+.1f}±{cs[1]:.1f}ms (n={cs[2]})")
            if ts:
                parts.append(f"sens滞后traj: {ts[0]:+.1f}±{ts[1]:.1f}ms (n={ts[2]})")
            print(f"{side_name}   " + "  ".join(parts))
        print("=" * 70)
        print()

    # 汇总柱状图
    if not args.no_plot and len(all_delays) > 0:
        summary_path = os.path.join(args.output_dir, f"{name}_summary.png")
        plot_summary(all_delays, summary_path, single_arm=single_arm)

    report = build_and_print_report(
        mode=args.mode,
        link_label=link_label,
        topic_arm_traj=topic_arm_traj,
        stats=latency_stats,
        all_delays=all_delays,
        fine=args.fine,
    )
    report_csv = os.path.join(args.output_dir, f"{name}_latency_report.csv")
    save_latency_report_csv(report, latency_stats, all_delays, report_csv)
    if not args.no_plot:
        plot_stage_report(
            report, os.path.join(args.output_dir, f"{name}_stages.png")
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
