#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
从指定 rosbag 中提取三个话题的指定关节角度数据，统一转换为角度(deg)后绘制曲线，
并基于互相关(cross-correlation)估计话题之间的延迟。

支持话题 (根据 --mode 选择手臂轨迹源):
  1a. --mode incremental (默认): /vr_incremental/kuavo_arm_traj_shm  sensor_msgs/JointState  .position  单位: deg
  1b. --mode absolute:           /kuavo_arm_traj                     sensor_msgs/JointState  .position  单位: deg
  1c. --mode pico:               /mm_kuavo_arm_traj                 sensor_msgs/JointState  .position  单位: deg
  2. /joint_cmd             kuavo_msgs/jointCmd      .joint_q         单位: rad  (控制器输出, 布局: leg+waist+arm+head)
  3. /sensors_data_raw      kuavo_msgs/sensorsData   .joint_data.joint_q 单位: rad (传感器反馈, 布局同 /joint_cmd)

关节布局说明 (人形 v45 等):
  - 手臂轨迹话题 (增量式或绝对式): 只含手臂关节, 顺序 [左臂 N 个, 右臂 N 个], N=NUM_ARM_JOINT/2 (通常 7)
  - /joint_cmd 与 /sensors_data_raw: [leg(12) + waist(0~3) + 左臂(N) + 右臂(N) + head(0~2)]
    手臂起始索引 arm_offset = NUM_JOINT - NUM_HEAD_JOINT - NUM_ARM_JOINT

用法示例:
  # 默认处理全部 14 个手臂关节, 生成每个关节的曲线图 + 汇总柱状图
  python3 extract_joint_topics_and_plot.py -b /path/to/xxx.bag

  # 增量式遥操 (默认): 使用 /vr_incremental/kuavo_arm_traj_shm
  python3 extract_joint_topics_and_plot.py -b /path/to/xxx.bag --joint 1

  # 绝对式遥操: 使用 /kuavo_arm_traj
  python3 extract_joint_topics_and_plot.py -b /path/to/xxx.bag --joint 1 --mode absolute

  # Pico遥操: 使用 /mm_kuavo_arm_traj
  python3 extract_joint_topics_and_plot.py -b /path/to/xxx.bag --joint 1 --mode pico

  # 提取右臂第 3 个关节 (对应手臂轨迹话题的 joint_10, /joint_cmd 与 sensors 的 arm_offset+7+2)
  python3 extract_joint_topics_and_plot.py -b /path/to/xxx.bag --joint 10

  # 指定机器人版本 (影响 arm_offset 计算)
  python3 extract_joint_topics_and_plot.py -b /path/to/xxx.bag --joint 1 --robot-version 45

  # 指定输出目录与图片名
  python3 extract_joint_topics_and_plot.py -b /path/to/xxx.bag --joint 1 -o /tmp/out --name arm_delay

  # 不保存 csv, 只画图
  python3 extract_joint_topics_and_plot.py -b /path/to/xxx.bag --joint 1 --no-csv

依赖: rosbag, numpy, matplotlib, scipy  (需先 source devel/setup.bash 以获得 kuavo_msgs)
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
        # a(t) vs b(t - d): 对 b 在 t-d 处插值
        shifted = np.interp(t, t - d, vb, left=vb[0], right=vb[-1])
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

    使用绝对时间 (相对三话题最早起点), 这样曲线在时间轴上自然对齐,
    可直观看出手臂轨迹话题滞后 /joint_cmd, /joint_cmd 滞后 /sensors_data_raw。
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
        f"traj→cmd: {delay_traj_cmd[0]*1000:+.1f}ms (r={delay_traj_cmd[1]:.2f})  |  "
        f"cmd→sens: {delay_cmd_sens[0]*1000:+.1f}ms (r={delay_cmd_sens[1]:.2f})  |  "
        f"traj→sens: {delay_traj_sens[0]*1000:+.1f}ms (r={delay_traj_sens[1]:.2f})")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=9)

    # 在图上文本框标注延迟含义
    textstr = (
        "延迟定义: A→B 的 delay>0 表示 A 滞后于 B (A 晚发生)\n"
        f"traj→cmd: {traj.name} 相对 /joint_cmd\n"
        "cmd→sens: /joint_cmd 相对 /sensors_data_raw\n"
        f"traj→sens: {traj.name} 相对 /sensors_data_raw\n"
        "时间轴为绝对时间对齐 (非各自归零)"
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

    # 计算平均值 (全部关节)
    def _safe_mean(vals):
        return float(np.mean(vals))
    def _safe_std(vals):
        return float(np.std(vals))

    mean_tc = _safe_mean(traj_cmd_vals)
    std_tc = _safe_std(traj_cmd_vals)
    mean_cs = _safe_mean(cmd_sens_vals)
    std_cs = _safe_mean(cmd_sens_vals)
    mean_ts = _safe_mean(traj_sens_vals)
    std_ts = _safe_std(traj_sens_vals)

    fig, ax = plt.subplots(figsize=(max(12, n * 0.8), 6))

    bars1 = ax.bar(x - width, traj_cmd_vals, width, color="#1f77b4", alpha=0.85, label="traj→cmd")
    bars2 = ax.bar(x, cmd_sens_vals, width, color="#ff7f0e", alpha=0.85, label="cmd→sens")
    bars3 = ax.bar(x + width, traj_sens_vals, width, color="#2ca02c", alpha=0.85, label="traj→sens")

    # 平均值虚线
    ax.axhline(y=mean_tc, color="#1f77b4", linestyle="--", linewidth=1.5, alpha=0.7)
    ax.axhline(y=mean_cs, color="#ff7f0e", linestyle="--", linewidth=1.5, alpha=0.7)
    ax.axhline(y=mean_ts, color="#2ca02c", linestyle="--", linewidth=1.5, alpha=0.7)

    ax.set_xlabel("关节")
    ax.set_ylabel("延迟 (ms)")
    ax.set_title(
        f"全臂关节延迟汇总 (N={n})  |  "
        f"traj→cmd 均值: {mean_tc:+.1f}±{std_tc:.1f}ms  |  "
        f"cmd→sens 均值: {mean_cs:+.1f}±{std_cs:.1f}ms  |  "
        f"traj→sens 均值: {mean_ts:+.1f}±{std_ts:.1f}ms"
    )
    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=8)
    ax.grid(True, alpha=0.3, axis="y")
    ax.legend(loc="best", fontsize=9)

    # 文本框: 平均延迟
    textstr = (
        f"traj→cmd:  {mean_tc:+.1f} ± {std_tc:.1f} ms\n"
        f"cmd→sens:  {mean_cs:+.1f} ± {std_cs:.1f} ms\n"
        f"traj→sens: {mean_ts:+.1f} ± {std_ts:.1f} ms"
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
        description="从 rosbag 提取手臂轨迹话题, /joint_cmd, /sensors_data_raw 的关节角度, "
                    "统一转 deg 后绘图并估计延迟。默认处理全部手臂关节。",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument("-b", "--bag", required=True, help="rosbag 文件路径")
    p.add_argument("-j", "--joint", type=int, default=None,
                   help="手臂关节序号 (1-based, 1..N=左臂, N+1..2N=右臂, 默认=所有关节)")
    p.add_argument("--mode", choices=list(ARM_TRAJ_TOPICS.keys()), default="incremental",
                   help="遥操模式: incremental=增量式 (/vr_incremental/kuavo_arm_traj_shm), "
                        "absolute=绝对式 (/kuavo_arm_traj), "
                        "pico=Pico遥操 (/mm_kuavo_arm_traj), 默认 incremental")
    p.add_argument("--robot-version", type=int, default=None,
                   help="机器人版本 (影响 arm_offset, 默认从 ROBOT_VERSION 环境变量或 45)")
    p.add_argument("--assets-root", default=None,
                   help="kuavo_assets/config 路径 (默认自动定位)")
    p.add_argument("-o", "--output-dir",
                   default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "png", "joint_delay"),
                   help="输出目录 (默认: tools/bag_tools/png/joint_delay)")
    p.add_argument("-n", "--name", default=None,
                   help="输出文件名前缀 (不含扩展名, 默认与 bag 同名)")
    p.add_argument("--csv", action="store_true", help="同时保存 CSV (默认不保存)")
    p.add_argument("--max-lag", type=float, default=2.0,
                   help="延迟估计最大搜索范围 (秒, 默认 2.0)")
    p.add_argument("--no-plot", action="store_true", help="不绘图")
    return p.parse_args()


def main() -> int:
    args = parse_args()

    if not os.path.isfile(args.bag):
        print(f"[FATAL] bag 文件不存在: {args.bag}")
        return 1

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

        if len(traj) >= 2 and len(cmd) >= 2:
            d1, r1 = estimate_delay_cross_corr(traj, cmd, args.max_lag)
            d2, r2 = estimate_delay_ls(traj, cmd, args.max_lag)
            delay_traj_cmd = (d1, r1)
            print(f"  traj→cmd 互相关: delay={d1*1000:+.1f}ms, r={r1:.3f} | "
                  f"最小二乘: delay={d2*1000:+.1f}ms, r={r2:.3f}")

        if len(cmd) >= 2 and len(sens) >= 2:
            d1, r1 = estimate_delay_cross_corr(cmd, sens, args.max_lag)
            d2, r2 = estimate_delay_ls(cmd, sens, args.max_lag)
            delay_cmd_sens = (d1, r1)
            print(f"  cmd→sens 互相关: delay={d1*1000:+.1f}ms, r={r1:.3f} | "
                  f"最小二乘: delay={d2*1000:+.1f}ms, r={r2:.3f}")

        if len(traj) >= 2 and len(sens) >= 2:
            d1, r1 = estimate_delay_cross_corr(traj, sens, args.max_lag)
            delay_traj_sens = (d1, r1)
            print(f"  traj→sens 互相关: delay={d1*1000:+.1f}ms, r={r1:.3f}")

        # 相位延迟: 滤波前(traj) → 滤波后(filtered), 纯滤波器相位延迟 (仅 incremental 模式)
        delay_traj_filtered = (0.0, 0.0)
        if args.mode == "incremental" and len(traj) >= 2 and len(filtered) >= 2:
            d1, r1 = estimate_delay_cross_corr(traj, filtered, args.max_lag)
            d2, r2 = estimate_delay_ls(traj, filtered, args.max_lag)
            delay_traj_filtered = (d1, r1)
            print(f"  traj→filtered 互相关: delay={d1*1000:+.1f}ms, r={r1:.3f} | "
                  f"最小二乘: delay={d2*1000:+.1f}ms, r={r2:.3f}")

        # 记录汇总数据 (单位 ms)
        all_delays.append({
            "joint": jidx,
            "side": side,
            "arm_local": arm_local,
            "traj_cmd_delay": delay_traj_cmd[0] * 1000,
            "traj_cmd_r": delay_traj_cmd[1],
            "cmd_sens_delay": delay_cmd_sens[0] * 1000,
            "cmd_sens_r": delay_cmd_sens[1],
            "traj_sens_delay": delay_traj_sens[0] * 1000,
            "traj_sens_r": delay_traj_sens[1],
            "traj_filtered_delay": delay_traj_filtered[0] * 1000,
            "traj_filtered_r": delay_traj_filtered[1],
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
        print("\n[WARN] 无有效延迟数据")
        return 0

    print("\n" + "=" * 70)
    print("                      全关节延迟汇总")
    print("=" * 70)
    print(f"{'关节':>6}  {'侧':>4}  {'traj→cmd':>10}  {'r':>6}  {'cmd→sens':>10}  {'r':>6}  {'traj→sens':>10}  {'r':>6}  {'traj→filtered':>12}")
    print("-" * 70)
    for d in all_delays:
        print(f"#{d['joint']:>4}  {d['side']:>4}  {d['traj_cmd_delay']:+9.1f}ms  {d['traj_cmd_r']:.3f}  "
              f"{d['cmd_sens_delay']:+9.1f}ms  {d['cmd_sens_r']:.3f}  "
              f"{d['traj_sens_delay']:+9.1f}ms  {d['traj_sens_r']:.3f}  "
              f"{d['traj_filtered_delay']:+9.1f}ms")
    print("-" * 70)

    # 计算平均值
    def _mean_std(vals):
        arr = np.array(vals)
        return float(np.mean(arr)), float(np.std(arr))

    tc_delays = [d["traj_cmd_delay"] for d in all_delays]
    cs_delays = [d["cmd_sens_delay"] for d in all_delays]
    ts_delays = [d["traj_sens_delay"] for d in all_delays]
    tf_delays = [d["traj_filtered_delay"] for d in all_delays]

    mean_tc, std_tc = _mean_std(tc_delays)
    mean_cs, std_cs = _mean_std(cs_delays)
    mean_ts, std_ts = _mean_std(ts_delays)
    mean_tf, std_tf = _mean_std(tf_delays)

    print(f"{'平均':>6}        {mean_tc:+9.1f}±{std_tc:.1f}  {'':>6}  "
          f"{mean_cs:+9.1f}±{std_cs:.1f}  {'':>6}  "
          f"{mean_ts:+9.1f}±{std_ts:.1f}  {'':>6}  "
          f"{mean_tf:+9.1f}±{std_tf:.1f}")
    print("=" * 70)

    # 按左右臂分别统计
    left_delays = [d for d in all_delays if d["joint"] <= single_arm]
    right_delays = [d for d in all_delays if d["joint"] > single_arm]
    for side_name, side_data in [("左臂", left_delays), ("右臂", right_delays)]:
        if len(side_data) < 2:
            continue
        tc = [d["traj_cmd_delay"] for d in side_data]
        cs = [d["cmd_sens_delay"] for d in side_data]
        ts = [d["traj_sens_delay"] for d in side_data]
        mtc, stc = _mean_std(tc)
        mcs, scs = _mean_std(cs)
        mts, sts = _mean_std(ts)
        print(f"{side_name} (N={len(side_data)})   traj→cmd: {mtc:+.1f}±{stc:.1f}ms  "
              f"cmd→sens: {mcs:+.1f}±{scs:.1f}ms  "
              f"traj→sens: {mts:+.1f}±{sts:.1f}ms")
    print("=" * 70)
    print()

    # 汇总柱状图
    if not args.no_plot and len(all_delays) > 0:
        summary_path = os.path.join(args.output_dir, f"{name}_summary.png")
        plot_summary(all_delays, summary_path, single_arm=single_arm)

    # 汇总
    print("\n========== 延迟汇总 ==========")
    if all_delays:
        mean_tc = sum(d["traj_cmd_delay"] for d in all_delays) / len(all_delays)
        mean_cs = sum(d["cmd_sens_delay"] for d in all_delays) / len(all_delays)
        mean_ts = sum(d["traj_sens_delay"] for d in all_delays) / len(all_delays)
        print(f"  {topic_arm_traj} → /joint_cmd:              {mean_tc:+8.1f} ms (平均, {len(all_delays)}关节)")
        print(f"  /joint_cmd → /sensors_data_raw:        {mean_cs:+8.1f} ms (平均, {len(all_delays)}关节)")
        print(f"  {topic_arm_traj} → /sensors_data_raw:     {mean_ts:+8.1f} ms (平均, {len(all_delays)}关节)")
        if args.mode == "incremental":
            mean_tf = sum(d["traj_filtered_delay"] for d in all_delays) / len(all_delays)
            print(f"  {topic_arm_traj} → {TOPIC_ARM_TRAJ_FILTERED}:  {mean_tf:+8.1f} ms (平均, {len(all_delays)}关节) [纯相位延迟]")
    print("==============================\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
