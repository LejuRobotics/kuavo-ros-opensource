#!/usr/bin/env python3
"""
对比画图：ArmTraj SHM vs /sensors_data_raw vs /joint_cmd（双臂）
+ 增量 IK → SHM/kuavo_arm_traj 中间链路（方便排查卡在哪一级）

主路径（#3198 / #3095）：IK 写 ArmTrajShm → WBC ArmTrajReceiver 读 SHM；
本脚本用同布局 ctypes 只读监控，不再订阅 /kuavo_arm_traj（避免 TCPROS 长连接劣化）。

轮臂 / 人形通用：从 kuavo_v${ROBOT_VERSION}/kuavo.json 读
  arm_offset = NUM_JOINT - NUM_HEAD_JOINT - NUM_ARM_JOINT

用法:
  python3 scripts/increment_test/plot_arm_traj_sensor_cmd.py --duration 20 -o /tmp/arm_cmp
  python3 scripts/increment_test/plot_arm_traj_sensor_cmd.py --npz .../arm_cmp.npz -o /tmp/arm_cmp

输出含 report.html：段选择 / 时间区间 / 时序统计 / 全链路样本；优先全量嵌入。
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import threading
import time
from collections import deque
from datetime import datetime

import numpy as np

try:
    import rospy
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64
    from kuavo_msgs.msg import jointCmd, sensorsData
except ImportError as e:
    sys.exit(f"[FATAL] ROS 未就绪: {e}\nsource devel/setup.bash")

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError:
    sys.exit("[FATAL] 需要 matplotlib: pip install matplotlib")


def load_layout(robot_version=None, assets_root=None):
    """返回 (n_arm, arm_offset, n_total, meta)。"""
    if robot_version is None:
        try:
            robot_version = int(rospy.get_param("/robot_version"))
        except Exception:
            robot_version = int(os.environ.get("ROBOT_VERSION", 45))
    if assets_root is None:
        # 优先 ROS 包路径；否则相对本脚本定位 <ws>/src/kuavo_assets/config
        try:
            import rospkg
            assets_root = os.path.join(rospkg.RosPack().get_path("kuavo_assets"), "config")
        except Exception:
            ws = os.environ.get("KUAVO_WS") or os.path.abspath(
                os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
            assets_root = os.path.join(ws, "src/kuavo_assets/config")

    path = os.path.join(assets_root, f"kuavo_v{robot_version}", "kuavo.json")
    if os.path.isfile(path):
        cfg = json.load(open(path))
        n_arm = int(cfg["NUM_ARM_JOINT"])
        n_head = int(cfg.get("NUM_HEAD_JOINT", 2))
        n_waist = int(cfg.get("NUM_WAIST_JOINT", 0))
        n_tot = int(cfg["NUM_JOINT"])
    else:
        # 无 json 时：用 /armRealDof + 一帧 sensor 长度推 offset
        print(f"[WARN] 无 {path}，改用 rosparam + 首帧 sensor 推断")
        n_arm = int(rospy.get_param("/armRealDof", 14))
        n_head = 2
        n_waist = int(rospy.get_param("/waistRealDof", 0))
        n_tot = None
        path = "(inferred)"

    # 与 ArmControlBaseROS::loadJointDimensionsWithFallback 一致
    if n_tot is None:
        # 等一帧 sensor 拿 total
        holder = {"n": None}

        def _once(msg):
            holder["n"] = len(msg.joint_data.joint_q)

        sub = rospy.Subscriber("/sensors_data_raw", sensorsData, _once, queue_size=1)
        t0 = time.time()
        while holder["n"] is None and time.time() - t0 < 3.0:
            rospy.sleep(0.05)
        sub.unregister()
        if holder["n"] is None:
            raise RuntimeError("无法从 /sensors_data_raw 推断 NUM_JOINT")
        n_tot = int(holder["n"])

    arm_offset = n_tot - n_head - n_arm
    if arm_offset < 0:
        raise RuntimeError(
            f"非法布局: total={n_tot} head={n_head} arm={n_arm} → offset={arm_offset}")

    meta = {
        "robot_version": robot_version,
        "NUM_ARM_JOINT": n_arm,
        "NUM_HEAD_JOINT": n_head,
        "NUM_WAIST_JOINT": n_waist,
        "NUM_JOINT": n_tot,
        "arm_offset": arm_offset,
        "config": path,
        "wheel_like": (arm_offset <= 6),  # v60/v62 offset≈4；人形 offset≈12~13
    }
    return n_arm, arm_offset, n_tot, meta


def traj_to_rad(arr):
    """JointState 话题若为度则转为弧度；已是弧度则原样返回。
    判定：|q|_max > 10 → 不可能是合理关节弧度，当作度。
    """
    a = np.asarray(arr, dtype=float)
    if a.size == 0:
        return a
    if np.nanmax(np.abs(a)) > 10.0:
        return a * math.pi / 180.0
    return a


def ensure_joint_rad(data):
    """快照/旧 npz 统一：traj、traj_sens 在话题上是 deg，内部一律 rad。"""
    out = dict(data)
    for key in ("traj_q", "traj_sens_q"):
        if key in out:
            out[key] = traj_to_rad(out[key])
    # sens_q / cmd_q 来自 sensors/joint_cmd，本身就是 rad，不转
    return out


RAD2DEG = 180.0 / math.pi


# 增量 → SHM / kuavo_arm_traj 排查用中间话题（轮臂/人形共用）
PIPELINE_DOC = """
# Incremental IK → ArmTraj SHM pipeline (#3198)

```
[1] /leju_quest_bone_poses          PoseInfoList   VR bone (≥24, Chest@23)
        │  Quest3ArmInfoTransformer (vrQuat2RobotQuat + chest frame)
        ▼
[2] /ik_debug/left|right_hand_pose_from_transformer   PoseStamped
        │  IncrementalControlModule (grip anchor + delta * scale + fhan)
        │  Drake / velocity IK
        ▼
[3] /ik_fk_result/left|right_ee_pose                  PoseStamped   IK 解 FK
[3b]/ik_fk_result/left|right_ee_pose_filter           PoseStamped   滤波后
        │  关节 fhan (fhan_r_joint / max_joint_velocity)
        ▼
[4] /kuavo_arm_traj_sensor_data     JointState(deg)  IK 从 sensors 切出的臂关节
[5a] ArmTraj SHM (key=343434, rad)  ★WBC 增量链路主路径（本报告 traj=SHM）
[5b] /kuavo_arm_traj_cpp remap→ /kuavo_arm_traj  JointState(deg)  仍发布给 MPC 同步
        │  SetIncrementalArmTrajLink: NONE=0 / SHM=1 / KUAVO_ARM_TRAJ=2
        │  /ik_debug/arm_traj_receive/{using_shm,transport}
        ▼
[6] /humanoid_mpc_target_arm        ocs2 target
        │  MPC / WBC (EXTERN_CONTROL=2；增量期 WBC 读 SHM)
        ▼
[7] /joint_cmd                      jointCmd(rad)
[8] /sensors_data_raw               sensorsData（arm_offset 切片, rad）

频率诊断:
  shm_dt_recv_ms    = 本进程 SHM readIfUpdated 间隔（无 TCPROS）
  shm_dt_stamp_ms   = SHM stamp_nsec 间隔（写端时钟）
  ee_dt_recv_ms     = /ik_fk_result/*_ee_pose 间隔
  loop_period_ms    = /ik_debug/solve_loop_ms/loop_period（轮臂）
  若 ROS 订阅 /kuavo_arm_traj 后半段 dt 抬高而 SHM 平稳 → #3095 TCP 阻塞已绕开

单位: 关节曲线 deg（内存 rad）；末端 m；间隔 ms
```

排查口诀:
  [2] 无 → bone/transformer
  [3] 无 → 未进增量 / 无 grip
  [5a] 无 → IK 未写 SHM / transport≠SHM / Writer 未初始化
  using_shm≠1 → 未 set_incremental_arm_traj_link(SHM) 或 stale 回退
  [5a]有 [7]无 → mode 未 [2,2]
  [7]≠[8] → 跟踪/标定（看 err）
"""

RECV_STATUS_TOPICS = {
    "using_shm": "/ik_debug/arm_traj_receive/using_shm",
    "transport": "/ik_debug/arm_traj_receive/transport",
}

PIPELINE_EE_TOPICS = {
    "tf_left": "/ik_debug/left_hand_pose_from_transformer",
    "tf_right": "/ik_debug/right_hand_pose_from_transformer",
    "ee_left": "/ik_fk_result/left_ee_pose",
    "ee_right": "/ik_fk_result/right_ee_pose",
    "ee_left_f": "/ik_fk_result/left_ee_pose_filter",
    "ee_right_f": "/ik_fk_result/right_ee_pose_filter",
}

# 轮臂 IK 自带；人形可能没有
SOLVE_LOOP_TOPICS = {
    "loop_period": "/ik_debug/solve_loop_ms/loop_period",
    "fsm_block_total": "/ik_debug/solve_loop_ms/fsm_block_total",
    "fsm_process": "/ik_debug/solve_loop_ms/fsm_process",
}

# 判定「段落式阻塞」：单间隔超过该阈值 (ms)
GAP_DT_MS = 25.0


class ArmCmpRecorder:
    def __init__(self, n_arm, arm_offset, max_bytes=500 * 1024 * 1024):
        self.n_arm = n_arm
        self.arm_offset = arm_offset
        self.max_bytes = int(max_bytes) if max_bytes else 0
        self.lock = threading.Lock()
        self.t0 = None
        # traj_* = ArmTraj SHM（rad）；不再走 /kuavo_arm_traj ROS 订阅
        self.traj_t, self.traj_q = deque(), deque()
        self.sens_t, self.sens_q = deque(), deque()
        self.cmd_t, self.cmd_q = deque(), deque()
        self.traj_sens_t, self.traj_sens_q = deque(), deque()
        self.ee = {k: {"t": deque(), "xyz": deque()} for k in PIPELINE_EE_TOPICS}
        self.segments = []
        self._seg_open = None
        # SHM stamp_nsec→秒；seq 便于对账
        self.traj_hdr = deque()
        self.traj_seq = deque()
        self.solve_ms = {k: {"t": deque(), "v": deque()} for k in SOLVE_LOOP_TOPICS}
        self.recv_status = {k: {"t": deque(), "v": deque()} for k in RECV_STATUS_TOPICS}
        self._t0_wall = None
        self._bytes_est = 0
        self._trim_count = 0
        self._append_since_check = 0
        self._shm_poller = None
        self.shm_stats = {}

    def _stamp(self):
        t = rospy.Time.now().to_sec()
        if self.t0 is None:
            self.t0 = t
            self._t0_wall = t
        return t - self.t0

    def _note_bytes(self, n_float):
        """粗估：每个 float64 ≈ 8B；附带时间戳再 +8。"""
        self._bytes_est += int(n_float) * 8 + 8
        self._append_since_check += 1
        if self.max_bytes > 0 and self._append_since_check >= 200:
            self._append_since_check = 0
            self._trim_if_needed()

    def _trim_if_needed(self):
        """超 max_bytes 时从最旧端丢弃，直到约 80% 预算。"""
        if self.max_bytes <= 0 or self._bytes_est <= self.max_bytes:
            return
        target = int(self.max_bytes * 0.8)
        dropped = 0
        while self._bytes_est > target and (
                self.sens_t or self.cmd_t or self.traj_t or self.traj_sens_t):
            # 优先丢高频 sens/cmd，再丢 traj/ee
            if len(self.sens_t) > len(self.traj_t) and self.sens_t:
                self.sens_t.popleft(); self.sens_q.popleft()
                self._bytes_est -= (self.n_arm + 1) * 8
                dropped += 1
            elif len(self.cmd_t) > len(self.traj_t) and self.cmd_t:
                self.cmd_t.popleft(); self.cmd_q.popleft()
                self._bytes_est -= (self.n_arm + 1) * 8
                dropped += 1
            elif self.traj_t:
                self.traj_t.popleft(); self.traj_q.popleft()
                if self.traj_hdr:
                    self.traj_hdr.popleft()
                self._bytes_est -= (self.n_arm + 2) * 8
                dropped += 1
            elif self.traj_sens_t:
                self.traj_sens_t.popleft(); self.traj_sens_q.popleft()
                self._bytes_est -= (self.n_arm + 1) * 8
                dropped += 1
            else:
                break
            for key, buf in self.ee.items():
                if buf["t"] and (not self.traj_t or buf["t"][0] < (self.traj_t[0] if self.traj_t else 1e9)):
                    buf["t"].popleft(); buf["xyz"].popleft()
                    self._bytes_est -= 4 * 8
            for key, buf in self.solve_ms.items():
                if buf["t"] and (not self.traj_t or buf["t"][0] < (self.traj_t[0] if self.traj_t else 1e9)):
                    buf["t"].popleft(); buf["v"].popleft()
                    self._bytes_est -= 2 * 8
        if self._bytes_est < 0:
            self._bytes_est = 0
        if dropped:
            self._trim_count += dropped
            if self._trim_count == dropped or self._trim_count % 5000 < dropped:
                print(f"[PLOT] npz 缓冲截断: dropped≈{dropped} est≈{self._bytes_est/1024/1024:.1f}MB "
                      f"(cap={self.max_bytes/1024/1024:.0f}MB)")

    def mark_segment(self, name):
        """开始新段；若有未闭合段则先以当前时刻收尾。"""
        with self.lock:
            t = self._stamp()
            if self._seg_open is not None:
                self.segments.append({
                    "name": self._seg_open["name"],
                    "t0": float(self._seg_open["t0"]),
                    "t1": float(t),
                })
            self._seg_open = {"name": str(name), "t0": float(t)}

    def end_segment(self):
        with self.lock:
            if self._seg_open is None:
                return
            t = self._stamp()
            self.segments.append({
                "name": self._seg_open["name"],
                "t0": float(self._seg_open["t0"]),
                "t1": float(t),
            })
            self._seg_open = None

    def get_segments(self):
        with self.lock:
            segs = [dict(s) for s in self.segments]
            if self._seg_open is not None:
                segs.append({
                    "name": self._seg_open["name"],
                    "t0": float(self._seg_open["t0"]),
                    "t1": float(self._stamp()),
                })
            return segs

    def on_shm_sample(self, sample):
        """ArmTraj SHM 更新：position 已是 rad。

        stamp_nsec 来自 IK Writer clock（monotonic epoch ns），
        转为相对首帧的秒数，避免跨进程时钟偏移导致 interval 失真。
        """
        pos = sample.get("position") or []
        if len(pos) < self.n_arm:
            return
        q = np.asarray(pos[:self.n_arm], dtype=float)
        stamp_nsec = sample.get("stamp_nsec", 0) or 0
        with self.lock:
            t = self._stamp()
            self.traj_t.append(t)
            self.traj_q.append(q)
            self.traj_seq.append(int(sample.get("seq", 0)))
            # stamp: 存递增量，首帧为 0，后续 = 相对秒数
            if stamp_nsec > 0:
                ns = float(stamp_nsec) * 1e-9
                if not hasattr(self, "_traj_hdr_base"):
                    self._traj_hdr_base = ns
                self.traj_hdr.append(ns - self._traj_hdr_base)
            else:
                self.traj_hdr.append(float("nan"))
            self._note_bytes(self.n_arm + 2)

    def on_traj_sensor(self, msg):
        """/kuavo_arm_traj_sensor_data：IK 发布为度。"""
        if len(msg.position) < self.n_arm:
            return
        q = traj_to_rad(msg.position[:self.n_arm])
        with self.lock:
            self.traj_sens_t.append(self._stamp())
            self.traj_sens_q.append(q)
            self._note_bytes(self.n_arm)

    def on_recv_status(self, key):
        def cb(msg):
            with self.lock:
                self.recv_status[key]["t"].append(self._stamp())
                self.recv_status[key]["v"].append(float(msg.data))
                self._note_bytes(1)
        return cb

    def on_sens(self, msg):
        q = msg.joint_data.joint_q
        end = self.arm_offset + self.n_arm
        if len(q) < end:
            return
        with self.lock:
            self.sens_t.append(self._stamp())
            self.sens_q.append(np.asarray(q[self.arm_offset:end], dtype=float))
            self._note_bytes(self.n_arm)

    def on_cmd(self, msg):
        q = msg.joint_q
        end = self.arm_offset + self.n_arm
        if len(q) < end:
            return
        with self.lock:
            self.cmd_t.append(self._stamp())
            self.cmd_q.append(np.asarray(q[self.arm_offset:end], dtype=float))
            self._note_bytes(self.n_arm)

    def on_ee(self, key):
        def cb(msg):
            p = msg.pose.position
            with self.lock:
                self.ee[key]["t"].append(self._stamp())
                self.ee[key]["xyz"].append([p.x, p.y, p.z])
                self._note_bytes(3)
        return cb

    def on_solve_ms(self, key):
        def cb(msg):
            with self.lock:
                self.solve_ms[key]["t"].append(self._stamp())
                self.solve_ms[key]["v"].append(float(msg.data))
                self._note_bytes(1)
        return cb

    def subscribe_all(self):
        """ROS 中间话题 + SHM 轮询（不订阅 /kuavo_arm_traj）。"""
        from arm_traj_shm_reader import ArmTrajShmPoller

        rospy.Subscriber("/kuavo_arm_traj_sensor_data", JointState, self.on_traj_sensor, queue_size=200)
        rospy.Subscriber("/sensors_data_raw", sensorsData, self.on_sens, queue_size=20)
        rospy.Subscriber("/joint_cmd", jointCmd, self.on_cmd, queue_size=20)
        for key, topic in PIPELINE_EE_TOPICS.items():
            rospy.Subscriber(topic, PoseStamped, self.on_ee(key), queue_size=200)
        for key, topic in SOLVE_LOOP_TOPICS.items():
            rospy.Subscriber(topic, Float64, self.on_solve_ms(key), queue_size=200)
        for key, topic in RECV_STATUS_TOPICS.items():
            rospy.Subscriber(topic, Float64, self.on_recv_status(key), queue_size=20)

        self._shm_poller = ArmTrajShmPoller(self.on_shm_sample, poll_sleep_s=0.0002)
        ok = self._shm_poller.start()
        print(f"[PLOT] ArmTraj SHM monitor {'OK' if ok else 'FAIL'} "
              f"(no /kuavo_arm_traj ROS sub)")

    def stop(self):
        if self._shm_poller is not None:
            self.shm_stats = dict(self._shm_poller.stats)
            self._shm_poller.stop()
            self._shm_poller = None
            print(f"[PLOT] SHM poller stopped: {self.shm_stats}")

    def snapshot(self):
        with self.lock:
            out = {
                "traj_t": np.asarray(self.traj_t, dtype=float),
                "traj_q": np.asarray(self.traj_q, dtype=float) if self.traj_q else np.zeros((0, self.n_arm)),
                "traj_hdr": np.asarray(self.traj_hdr, dtype=float),
                "traj_seq": np.asarray(self.traj_seq, dtype=float),
                "sens_t": np.asarray(self.sens_t, dtype=float),
                "sens_q": np.asarray(self.sens_q, dtype=float) if self.sens_q else np.zeros((0, self.n_arm)),
                "cmd_t": np.asarray(self.cmd_t, dtype=float),
                "cmd_q": np.asarray(self.cmd_q, dtype=float) if self.cmd_q else np.zeros((0, self.n_arm)),
                "traj_sens_t": np.asarray(self.traj_sens_t, dtype=float),
                "traj_sens_q": np.asarray(self.traj_sens_q, dtype=float) if self.traj_sens_q else np.zeros((0, self.n_arm)),
            }
            for key, buf in self.ee.items():
                out[f"{key}_t"] = np.asarray(buf["t"], dtype=float)
                out[f"{key}_xyz"] = np.asarray(buf["xyz"], dtype=float) if buf["xyz"] else np.zeros((0, 3))
            for key, buf in self.solve_ms.items():
                out[f"solve_{key}_t"] = np.asarray(buf["t"], dtype=float)
                out[f"solve_{key}_v"] = np.asarray(buf["v"], dtype=float)
            for key, buf in self.recv_status.items():
                out[f"recv_{key}_t"] = np.asarray(buf["t"], dtype=float)
                out[f"recv_{key}_v"] = np.asarray(buf["v"], dtype=float)
            return out

    def pipeline_counts(self):
        with self.lock:
            c = {
                "shm_traj": len(self.traj_t),
                "traj_sensor": len(self.traj_sens_t),
                "sens": len(self.sens_t),
                "cmd": len(self.cmd_t),
                **{k: len(v["t"]) for k, v in self.ee.items()},
                **{f"recv_{k}": len(v["t"]) for k, v in self.recv_status.items()},
            }
            # 兼容旧字段名
            c["traj"] = c["shm_traj"]
            return c


def save_npz(path, data, meta):
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    np.savez_compressed(path, **data, meta_json=json.dumps(meta))
    print(f"[SAVE] {path}")


def load_npz(path):
    z = np.load(path, allow_pickle=True)
    meta = json.loads(str(z["meta_json"])) if "meta_json" in z.files else {}
    data = {k: z[k] for k in z.files if k != "meta_json"}
    return data, meta


def write_pipeline_doc(out_dir, counts=None):
    os.makedirs(out_dir, exist_ok=True)
    fp = os.path.join(out_dir, "pipeline.md")
    with open(fp, "w") as f:
        f.write(PIPELINE_DOC)
        if counts:
            f.write("\n## 本次录制样本数\n\n")
            f.write("| stage | count |\n|---|---|\n")
            for k, v in counts.items():
                f.write(f"| {k} | {v} |\n")
            # 快速诊断
            f.write("\n## 自动诊断\n\n")
            tips = []
            if counts.get("tf_left", 0) == 0 and counts.get("tf_right", 0) == 0:
                tips.append("- [2] transformer 无输出 → 检查 bone poses≥24 / IK 是否在跑")
            if counts.get("ee_left", 0) == 0 and counts.get("ee_right", 0) == 0:
                tips.append("- [3] ee_pose 无输出 → 未进增量 / 无 grip / 未 forceActivate")
            shm_n = counts.get("shm_traj", counts.get("traj", 0))
            if shm_n == 0:
                tips.append("- [5a] ArmTraj SHM 无更新 → IK Writer 未写 / transport≠SHM / shmget 失败")
            elif counts.get("cmd", 0) == 0:
                tips.append("- [7] joint_cmd 无臂跟踪 → mode 是否 [2,2] / WBC 是否 TRANSPORT_SHM")
            if counts.get("recv_using_shm", 0) and False:
                pass
            if not tips:
                tips.append("- 各阶段均有样本，继续看曲线幅值/相位与 SHM timing")
            f.write("\n".join(tips) + "\n")
    print(f"[DOC] {fp}")


def plot_pipeline(data, out_dir):
    """画增量链路：transformer / ee / ee_filter 末端 xyz + traj vs traj_sensor 关节。"""
    data = ensure_joint_rad(data)
    os.makedirs(out_dir, exist_ok=True)

    # 末端 xyz：左/右各一张
    for side, keys in (
        ("left", [("tf_left", "transformer"), ("ee_left", "ee_raw"), ("ee_left_f", "ee_filter")]),
        ("right", [("tf_right", "transformer"), ("ee_right", "ee_raw"), ("ee_right_f", "ee_filter")]),
    ):
        fig, axes = plt.subplots(3, 1, figsize=(12, 7), sharex=True)
        for ax, axis_i, axis_name in zip(axes, range(3), "xyz"):
            for key, label in keys:
                t = data.get(f"{key}_t", np.array([]))
                xyz = data.get(f"{key}_xyz", np.zeros((0, 3)))
                if t.size and xyz.ndim == 2 and xyz.shape[0] == t.size:
                    ax.plot(t, xyz[:, axis_i], label=label, lw=1.0)
            ax.set_ylabel(f"{axis_name} (m)")
            ax.grid(True, alpha=0.3)
            if axis_i == 0:
                ax.legend(loc="upper right", fontsize=8)
                ax.set_title(f"Pipeline EE {side}: transformer → IK FK → filter")
        axes[-1].set_xlabel("t (s)")
        fig.tight_layout()
        fp = os.path.join(out_dir, f"pipeline_ee_{side}.png")
        fig.savefig(fp, dpi=140)
        plt.close(fig)
        print(f"[PLOT] {fp}")

    # 关节：kuavo_arm_traj vs kuavo_arm_traj_sensor_data（IK 内外对照）
    tq, qq = data.get("traj_t", np.array([])), data.get("traj_q", np.zeros((0, 14)))
    ts, qs = data.get("traj_sens_t", np.array([])), data.get("traj_sens_q", np.zeros((0, 14)))
    if (tq.size or ts.size) and (qq.ndim == 2 or qs.ndim == 2):
        n = int(qq.shape[1] if qq.ndim == 2 and qq.size else (qs.shape[1] if qs.ndim == 2 else 14))
        for side, idxs, title in (
            ("left", range(0, n // 2), "Left"),
            ("right", range(n // 2, n), "Right"),
        ):
            idxs = list(idxs)
            fig, axes = plt.subplots(len(idxs), 1, figsize=(12, 2.0 * len(idxs)), sharex=True)
            if len(idxs) == 1:
                axes = [axes]
            for ax, j in zip(axes, idxs):
                if tq.size and qq.ndim == 2 and qq.shape[0] == tq.size:
                    ax.plot(tq, qq[:, j] * RAD2DEG, label="ArmTraj SHM", lw=1.2, color="C0")
                if ts.size and qs.ndim == 2 and qs.shape[0] == ts.size:
                    ax.plot(ts, qs[:, j] * RAD2DEG, label="/kuavo_arm_traj_sensor_data", lw=1.0, color="C3", alpha=0.85)
                ax.set_ylabel(f"j{j+1}\n(deg)")
                ax.grid(True, alpha=0.3)
                if j == idxs[0]:
                    ax.legend(fontsize=8)
                    ax.set_title(f"Pipeline joints {title}: IK cmd vs IK-seen sensor")
            axes[-1].set_xlabel("t (s)")
            fig.tight_layout()
            fp = os.path.join(out_dir, f"pipeline_joint_{side}.png")
            fig.savefig(fp, dpi=140)
            plt.close(fig)
            print(f"[PLOT] {fp}")


def _dt_ms(t):
    t = np.asarray(t, dtype=float)
    if t.size < 2:
        return np.array([]), np.array([])
    dt = np.diff(t) * 1000.0
    return t[1:], dt


def timing_stats(data, gap_ms=GAP_DT_MS):
    """计算 traj/ee/solve 间隔统计，供报告与自动诊断。"""
    out = {}

    def pack(name, t_mid, dt):
        if dt.size == 0:
            out[name] = {"n": 0}
            return
        gaps = dt > gap_ms
        # 连续 gap 段数
        gap_runs = 0
        if gaps.any():
            gap_runs = int(np.sum(np.diff(gaps.astype(np.int8)) == 1) + (1 if gaps[0] else 0))
        out[name] = {
            "n": int(dt.size),
            "median_ms": float(np.median(dt)),
            "mean_ms": float(np.mean(dt)),
            "p95_ms": float(np.percentile(dt, 95)),
            "max_ms": float(np.max(dt)),
            "gap_n": int(np.sum(gaps)),
            "gap_pct": float(np.mean(gaps) * 100.0),
            "gap_runs": gap_runs,
            "hz_med": float(1000.0 / np.median(dt)) if np.median(dt) > 1e-9 else 0.0,
        }

    t_recv, dt_recv = _dt_ms(data.get("traj_t", []))
    pack("traj_dt_recv", t_recv, dt_recv)

    hdr = np.asarray(data.get("traj_hdr", []), dtype=float)
    if hdr.size >= 2 and np.isfinite(hdr).sum() >= 2:
        valid = np.isfinite(hdr)
        # 用有效 stamp 的 diff；对齐到 recv 时刻索引
        idx = np.where(valid)[0]
        if idx.size >= 2:
            dt_h = np.diff(hdr[idx]) * 1000.0
            t_h = np.asarray(data.get("traj_t", []), dtype=float)
            t_mid = t_h[idx[1:]] if t_h.size == hdr.size else np.arange(dt_h.size)
            pack("traj_dt_hdr", t_mid, dt_h)
        else:
            pack("traj_dt_hdr", np.array([]), np.array([]))
    else:
        pack("traj_dt_hdr", np.array([]), np.array([]))

    t_ee, dt_ee = _dt_ms(data.get("ee_left_t", []))
    pack("ee_dt_recv", t_ee, dt_ee)

    for key in SOLVE_LOOP_TOPICS:
        t = data.get(f"solve_{key}_t", np.array([]))
        v = data.get(f"solve_{key}_v", np.array([]))
        if getattr(v, "size", 0) and getattr(t, "size", 0):
            out[f"solve_{key}"] = {
                "n": int(v.size),
                "median_ms": float(np.median(v)),
                "mean_ms": float(np.mean(v)),
                "p95_ms": float(np.percentile(v, 95)),
                "max_ms": float(np.max(v)),
            }
        else:
            out[f"solve_{key}"] = {"n": 0}
    return out


def plot_timing(data, out_dir, gap_ms=GAP_DT_MS):
    """画发送/解算间隔时序 + 直方图，对齐 #3095 段落式阻塞。

    traj_dt_recv: Python poller wall clock 间隔（反映接收端真实间隔）
    traj_dt_hdr:  IK Writer stamp_nsec 差分（反映写端发布间隔，相对首帧）
    两者应在同一量级（~10ms @100Hz），hdr 更干净（不受 Python GIL/调度影响）。
    """
    os.makedirs(out_dir, exist_ok=True)
    stats = timing_stats(data, gap_ms)

    # 推断期望频率
    tr_n = int(np.asarray(data.get("traj_t", [])).size)
    if tr_n >= 3:
        tr_t = np.asarray(data["traj_t"])
        exp_hz = tr_n / max(tr_t[-1] - tr_t[0], 0.001)
        exp_ms = 1000.0 / max(exp_hz, 1.0)
    else:
        exp_hz = 100.0
        exp_ms = 10.0

    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    series = []

    t_r, dt_r = _dt_ms(data.get("traj_t", []))
    if dt_r.size:
        axes[0].plot(t_r, dt_r, lw=0.5, color="C0", alpha=0.6, label="recv dt (raw)")
        axes[0].axhline(exp_ms, color="C0", ls=":", lw=1.0, alpha=0.7,
                        label=f"est ~{exp_hz:.0f}Hz ({exp_ms:.1f}ms)")
        # 滚动中值（去毛刺）
        w = min(31, max(3, len(dt_r) // 20))
        if len(dt_r) > w:
            rm = np.convolve(dt_r, np.ones(w) / w, mode="valid")
            axes[0].plot(t_r[w // 2: w // 2 + len(rm)], rm,
                        lw=1.2, color="C0", label=f"recv dt (rolling median {w})")
        axes[0].axhline(gap_ms, color="r", ls="--", lw=0.8, label=f"gap>{gap_ms:.0f}ms")
        series.append(("traj_recv", t_r, dt_r))

    hdr = np.asarray(data.get("traj_hdr", []), dtype=float)
    traj_t = np.asarray(data.get("traj_t", []), dtype=float)
    valid_hdr = np.isfinite(hdr)
    if valid_hdr.sum() >= 2 and traj_t.size == hdr.size:
        idx = np.where(valid_hdr)[0]
        dt_h = np.diff(hdr[idx]) * 1000.0  # hdr 已是相对秒
        # 过滤明显异常值（>200ms 多半是时钟跳变或首帧漂移）
        ok = dt_h < 200.0
        if ok.sum() >= 2:
            axes[0].plot(traj_t[idx[1:]][ok], dt_h[ok],
                        lw=1.0, color="C3", alpha=0.85, label="stamp dt (IK writer)")
            series.append(("traj_hdr", traj_t[idx[1:]][ok], dt_h[ok]))
        else:
            axes[0].text(0.5, 0.5, "hdr dt all >200ms (bad clock?)",
                        ha="center", va="center", transform=axes[0].transAxes, color="#999")
    else:
        axes[0].text(0.5, 0.9, "no valid stamp — IK writer may not set stamp_nsec",
                    ha="center", va="top", transform=axes[0].transAxes,
                    fontsize=8, color="#999")

    axes[0].set_ylabel("SHM dt (ms)")
    axes[0].set_title(f"ArmTraj SHM update interval  (est ~{exp_hz:.0f}Hz)  (#3198)")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(fontsize=7, loc="upper right")
    # Y 轴不要被毛刺拉得太高
    ymax = max(np.percentile(dt_r, 99) if dt_r.size else 50, gap_ms * 3)
    axes[0].set_ylim(0, min(ymax, exp_ms * 10))

    t_e, dt_e = _dt_ms(data.get("ee_left_t", []))
    if dt_e.size:
        axes[1].plot(t_e, dt_e, lw=0.8, color="C1", label="ee_left recv dt")
        axes[1].axhline(gap_ms, color="r", ls="--", lw=0.8)
        series.append(("ee_left", t_e, dt_e))
    axes[1].set_ylabel("ee dt (ms)")
    axes[1].set_title("IK FK publish interval (/ik_fk_result/left_ee_pose)")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(fontsize=8, loc="upper right")

    plotted = False
    for key, color in (("loop_period", "C2"), ("fsm_block_total", "C4"), ("fsm_process", "C5")):
        t = data.get(f"solve_{key}_t", np.array([]))
        v = data.get(f"solve_{key}_v", np.array([]))
        if getattr(t, "size", 0) and getattr(v, "size", 0) and t.size == v.size:
            axes[2].plot(t, v, lw=0.8, color=color, label=key, alpha=0.9)
            plotted = True
    if not plotted:
        axes[2].text(0.5, 0.5, "no /ik_debug/solve_loop_ms/* (biped may omit)",
                     ha="center", va="center", transform=axes[2].transAxes, color="#666666")
    axes[2].set_ylabel("solve ms")
    axes[2].set_xlabel("t (s)")
    axes[2].set_title("IK solve_loop_ms (wheel)")
    axes[2].grid(True, alpha=0.3)
    axes[2].legend(fontsize=8, loc="upper right")

    fig.tight_layout()
    fp = os.path.join(out_dir, "timing_intervals.png")
    fig.savefig(fp, dpi=140)
    plt.close(fig)
    print(f"[PLOT] {fp}")

    # 直方图
    fig, axes = plt.subplots(1, 2, figsize=(10, 3.5))
    for ax, (name, _t, dt), title in zip(
        axes,
        [
            ("traj",) + ((t_r, dt_r) if dt_r.size else (np.array([]), np.array([]))),
            ("ee",) + ((t_e, dt_e) if dt_e.size else (np.array([]), np.array([]))),
        ],
        ["/kuavo_arm_traj dt → SHM dt", "ee_left dt"],
    ):
        if dt.size:
            ax.hist(dt, bins=60, color="C0", alpha=0.85)
            ax.axvline(gap_ms, color="r", ls="--", lw=1)
        ax.set_title(title)
        ax.set_xlabel("ms")
        ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fp = os.path.join(out_dir, "timing_hist.png")
    fig.savefig(fp, dpi=140)
    plt.close(fig)
    print(f"[PLOT] {fp}")

    # timing.md
    fp = os.path.join(out_dir, "timing.md")
    with open(fp, "w") as f:
        f.write("# Publish / IK timing (#3095)\n\n")
        f.write(f"gap threshold = {gap_ms:.0f} ms\n\n")
        f.write("| metric | n | med_ms | p95 | max | gap% | gap_runs | Hz_med |\n|---|---|---|---|---|---|---|---|\n")
        for k, s in stats.items():
            if s.get("n", 0) == 0:
                f.write(f"| {k} | 0 | | | | | | |\n")
                continue
            f.write(
                f"| {k} | {s['n']} | {s.get('median_ms', s.get('mean_ms', 0)):.2f} | "
                f"{s.get('p95_ms', 0):.2f} | {s.get('max_ms', 0):.2f} | "
                f"{s.get('gap_pct', 0):.1f} | {s.get('gap_runs', '-')} | "
                f"{s.get('hz_med', 0):.1f} |\n"
            )
        tips = []
        tr = stats.get("traj_dt_recv", {})
        if tr.get("gap_runs", 0) >= 2 or tr.get("gap_pct", 0) > 5:
            tips.append("- traj 出现多段/较多 gap → 对齐 #3095「段落式阻塞发送」")
        if tr.get("n") and tr.get("median_ms", 0) > 15:
            tips.append(f"- traj 中位间隔 {tr['median_ms']:.1f}ms，低于 ~100Hz 预期")
        ee = stats.get("ee_dt_recv", {})
        if ee.get("n") and tr.get("n") and abs(ee.get("median_ms", 0) - tr.get("median_ms", 0)) > 5:
            tips.append("- ee 与 traj 中位间隔差大 → 发布路径不同步或丢包")
        lp = stats.get("solve_loop_period", {})
        if not lp.get("n"):
            tips.append("- 无 /ik_debug/solve_loop_ms/loop_period（人形常见）；以 traj/ee dt 为准")
        if not tips:
            tips.append("- 间隔未见明显段落阻塞；若实机 bag 有，对比本仿真是否复现")
        f.write("\n## 诊断\n\n" + "\n".join(tips) + "\n")
    print(f"[DOC] {fp}")
    with open(os.path.join(out_dir, "timing.json"), "w") as f:
        json.dump(stats, f, indent=2)
    return stats


def plot_compare(data, meta, out_dir, joints=None, deg=True):
    data = ensure_joint_rad(data)
    os.makedirs(out_dir, exist_ok=True)
    n_arm = int(meta.get("NUM_ARM_JOINT", data["traj_q"].shape[1] if data["traj_q"].ndim == 2 and data["traj_q"].size else 14))
    if joints is None:
        joints = list(range(n_arm))
    # 关节图统一 deg（忽略 --rad，避免混单位）
    if not deg:
        print("[WARN] 关节图已统一为 deg，忽略 --rad")
    scale = RAD2DEG
    unit = "deg"

    # 总览：左臂 0-6 / 右臂 7-13 两张大图（每关节一行）
    for side, idxs, title in (
        ("left", [j for j in joints if j < n_arm // 2], "Left arm"),
        ("right", [j for j in joints if j >= n_arm // 2], "Right arm"),
    ):
        if not idxs:
            continue
        nrows = len(idxs)
        fig, axes = plt.subplots(nrows, 1, figsize=(12, 2.0 * nrows), sharex=True)
        if nrows == 1:
            axes = [axes]
        for ax, j in zip(axes, idxs):
            if data["traj_t"].size:
                ax.plot(data["traj_t"], data["traj_q"][:, j] * scale,
                        label="ArmTraj SHM", lw=1.2, color="C0")
            if data["sens_t"].size:
                ax.plot(data["sens_t"], data["sens_q"][:, j] * scale,
                        label="/sensors_data_raw", lw=1.0, color="C1", alpha=0.9)
            if data["cmd_t"].size:
                ax.plot(data["cmd_t"], data["cmd_q"][:, j] * scale,
                        label="/joint_cmd", lw=1.0, color="C2", alpha=0.85, ls="--")
            ax.set_ylabel(f"j{j+1}\n({unit})")
            ax.grid(True, alpha=0.3)
            if j == idxs[0]:
                ax.legend(loc="upper right", fontsize=8)
                ax.set_title(
                    f"{title}  v{meta.get('robot_version','?')}  "
                    f"offset={meta.get('arm_offset','?')}  "
                    f"{'wheel-like' if meta.get('wheel_like') else 'biped'}")
        axes[-1].set_xlabel("t (s)")
        fig.tight_layout()
        fp = os.path.join(out_dir, f"arm_cmp_{side}.png")
        fig.savefig(fp, dpi=140)
        plt.close(fig)
        print(f"[PLOT] {fp}")

    # 误差图：cmd-sens / traj-sens（有数据才画）
    fig, axes = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
    _plot_err(axes[0], data, "traj", "sens", scale, "traj − sensor")
    _plot_err(axes[1], data, "cmd", "sens", scale, "cmd − sensor")
    axes[1].set_xlabel("t (s)")
    fig.suptitle(f"Arm tracking error ({unit})")
    fig.tight_layout()
    fp = os.path.join(out_dir, "arm_cmp_err.png")
    fig.savefig(fp, dpi=140)
    plt.close(fig)
    print(f"[PLOT] {fp}")

    # 摘要
    summary = {
        "n_traj": int(data["traj_t"].size),
        "n_sens": int(data["sens_t"].size),
        "n_cmd": int(data["cmd_t"].size),
        "n_traj_sensor": int(data.get("traj_sens_t", np.array([])).size),
        "meta": meta,
    }
    for key in PIPELINE_EE_TOPICS:
        summary[f"n_{key}"] = int(data.get(f"{key}_t", np.array([])).size)
    with open(os.path.join(out_dir, "summary.json"), "w") as f:
        json.dump(summary, f, indent=2)
    print(f"[PLOT] samples traj={summary['n_traj']} sens={summary['n_sens']} cmd={summary['n_cmd']} "
          f"traj_sensor={summary['n_traj_sensor']}")

    # 增量链路图 + 文档
    plot_pipeline(data, out_dir)
    write_pipeline_doc(out_dir, {k.replace("n_", ""): v for k, v in summary.items() if k.startswith("n_")})
    timing = plot_timing(data, out_dir)
    summary["timing"] = timing
    with open(os.path.join(out_dir, "summary.json"), "w") as f:
        json.dump(summary, f, indent=2)


# HTML 嵌入：尽量全量；仅超大序列才抽稀
HTML_KEEP_FULL_N = 80000  # ≤ 此点数：全量写入 HTML
HTML_CAP_N = 25000        # > KEEP_FULL：抽到此上限


def _auto_tips(counts):
    tips = []
    if counts.get("tf_left", 0) == 0 and counts.get("tf_right", 0) == 0:
        tips.append("[2] transformer 无输出 → bone/IK")
    if counts.get("ee_left", 0) == 0 and counts.get("ee_right", 0) == 0:
        tips.append("[3] ee_pose 无 → 未进增量/无 grip")
    shm_n = counts.get("shm_traj", counts.get("traj", 0))
    if shm_n == 0:
        tips.append("[5a] ArmTraj SHM 无更新 → Writer/transport")
    elif counts.get("cmd", 0) == 0:
        tips.append("[7] joint_cmd 无 → mode/[2,2]/WBC SHM")
    if not tips:
        tips.append("各阶段有样本，看曲线与 timing_stats")
    return tips


def _decimate(t, arr, max_n=None):
    """均匀抽稀。max_n=None 时自适应：小序列全量，大序列抽到 HTML_CAP_N。"""
    t = np.asarray(t, dtype=float)
    arr = np.asarray(arr, dtype=float)
    if t.size == 0:
        empty = [] if arr.ndim != 2 else np.zeros((0, arr.shape[1])).tolist()
        return [], empty
    if max_n is None:
        max_n = t.size if t.size <= HTML_KEEP_FULL_N else HTML_CAP_N
    if t.size <= max_n:
        return t.tolist(), arr.tolist()
    idx = np.unique(np.linspace(0, t.size - 1, max_n).astype(int))
    return t[idx].tolist(), arr[idx].tolist()


def _build_report_payload(data, meta, max_n=None):
    """塞进 HTML 的 JSON。关节 deg、末端 m。默认自适应全量/抽稀。"""
    data = ensure_joint_rad(data)
    segments = list(meta.get("segments") or [])
    n_src_total = 0
    n_out_total = 0

    def jq(prefix):
        nonlocal n_src_total, n_out_total
        t = data.get(f"{prefix}_t", np.array([]))
        q = data.get(f"{prefix}_q", np.zeros((0, 0)))
        n_src_total += int(getattr(t, "size", 0))
        td, qd = _decimate(t, q, max_n)
        n_out_total += len(td)
        if qd and isinstance(qd[0], (list, tuple)):
            qd = [[float(v) * RAD2DEG for v in row] for row in qd]
        return {"t": td, "q": qd, "n_src": int(getattr(t, "size", 0)), "n_out": len(td)}

    def ee(key):
        nonlocal n_src_total, n_out_total
        t = data.get(f"{key}_t", np.array([]))
        xyz = data.get(f"{key}_xyz", np.zeros((0, 3)))
        n_src_total += int(getattr(t, "size", 0))
        td, xd = _decimate(t, xyz, max_n)
        n_out_total += len(td)
        return {"t": td, "xyz": xd, "n_src": int(getattr(t, "size", 0)), "n_out": len(td)}

    t_candidates = []
    for k in ("traj_t", "sens_t", "cmd_t", "traj_sens_t"):
        a = data.get(k, np.array([]))
        if getattr(a, "size", 0):
            t_candidates.append(float(np.nanmax(a)))
    for key in PIPELINE_EE_TOPICS:
        a = data.get(f"{key}_t", np.array([]))
        if getattr(a, "size", 0):
            t_candidates.append(float(np.nanmax(a)))
    t_max = max(t_candidates) if t_candidates else 1.0

    n_arm = int(meta.get("NUM_ARM_JOINT", 14))
    timing = _timing_payload(data, max_n=max_n)
    stats = timing_stats(data)

    def status_series(key):
        nonlocal n_src_total, n_out_total
        t = data.get(f"recv_{key}_t", np.array([]))
        v = data.get(f"recv_{key}_v", np.array([]))
        n_src_total += int(getattr(t, "size", 0))
        if getattr(t, "size", 0) and getattr(v, "size", 0) and t.size == v.size:
            td, vd = _decimate(t, v.reshape(-1, 1), max_n)
            if vd and isinstance(vd[0], list):
                vd = [row[0] for row in vd]
            n_out_total += len(td)
            return {"t": td, "v": vd, "n_src": int(t.size), "n_out": len(td)}
        return {"t": [], "v": [], "n_src": 0, "n_out": 0}

    seq = data.get("traj_seq", np.array([]))
    seq_t = data.get("traj_t", np.array([]))
    if getattr(seq, "size", 0) and getattr(seq_t, "size", 0) and seq.size == seq_t.size:
        st, sv = _decimate(seq_t, np.asarray(seq, dtype=float).reshape(-1, 1), max_n)
        if sv and isinstance(sv[0], list):
            sv = [row[0] for row in sv]
        seq_payload = {"t": st, "seq": sv}
    else:
        seq_payload = {"t": [], "seq": []}

    return {
        "meta": {k: meta[k] for k in meta if k != "segments"},
        "segments": segments,
        "t_max": t_max,
        "n_arm": n_arm,
        "traj": jq("traj"),
        "sens": jq("sens"),
        "cmd": jq("cmd"),
        "traj_sens": jq("traj_sens"),
        "ee": {k: ee(k) for k in PIPELINE_EE_TOPICS},
        "timing": timing,
        "timing_stats": stats,
        "recv_status": {k: status_series(k) for k in RECV_STATUS_TOPICS},
        "traj_seq": seq_payload,
        "gap_ms": GAP_DT_MS,
        "display": {
            "keep_full_n": HTML_KEEP_FULL_N,
            "cap_n": HTML_CAP_N,
            "n_src": n_src_total,
            "n_out": n_out_total,
            "decimated": n_out_total < n_src_total,
            "traj_source": "ArmTraj SHM",
        },
    }


def _timing_payload(data, max_n=None):
    """间隔曲线：默认自适应全量/抽稀。hdr 已是相对秒，filter >200ms outliers。"""
    out = {}
    t_r, dt_r = _dt_ms(data.get("traj_t", []))
    if len(t_r):
        td, dd = _decimate(t_r, dt_r.reshape(-1, 1), max_n)
        if dd and isinstance(dd[0], list):
            dd = [row[0] for row in dd]
        out["traj_recv"] = {"t": td, "dt_ms": dd}

        # 滚动中值（去毛刺，抽稀后计算）
        dt_rr = np.asarray(dd, dtype=float) if dd else np.array([])
        w = min(31, max(3, len(dt_rr) // 20))
        if dt_rr.size > w:
            rm = np.convolve(dt_rr, np.ones(w) / w, mode="valid")
            t_rm = td[w // 2: w // 2 + len(rm)] if len(td) == len(dt_rr) else [0]
            out["traj_recv_rm"] = {"t": t_rm, "dt_ms": [float(x) for x in rm]}
        else:
            out["traj_recv_rm"] = {"t": [], "dt_ms": []}
    else:
        out["traj_recv"] = {"t": [], "dt_ms": []}
        out["traj_recv_rm"] = {"t": [], "dt_ms": []}

    hdr = np.asarray(data.get("traj_hdr", []), dtype=float)
    traj_t = np.asarray(data.get("traj_t", []), dtype=float)
    if hdr.size >= 2 and traj_t.size == hdr.size:
        idx = np.where(np.isfinite(hdr))[0]
        if idx.size >= 2:
            dt_h = np.diff(hdr[idx]) * 1000.0  # hdr 已是相对秒
            ok = dt_h < 200.0  # 过滤 >200ms 异常值（时钟跳变）
            if ok.sum() >= 2:
                t_h = traj_t[idx[1:]][ok]
                dt_h = dt_h[ok]
                td, dd = _decimate(t_h, dt_h.reshape(-1, 1), max_n)
                if dd and isinstance(dd[0], list):
                    dd = [row[0] for row in dd]
                out["traj_hdr"] = {"t": td, "dt_ms": dd}
            else:
                out["traj_hdr"] = {"t": [], "dt_ms": []}
        else:
            out["traj_hdr"] = {"t": [], "dt_ms": []}
    else:
        out["traj_hdr"] = {"t": [], "dt_ms": []}

    t_e, dt_e = _dt_ms(data.get("ee_left_t", []))
    if len(t_e):
        td, dd = _decimate(t_e, dt_e.reshape(-1, 1), max_n)
        if dd and isinstance(dd[0], list):
            dd = [row[0] for row in dd]
        out["ee_left"] = {"t": td, "dt_ms": dd}
    else:
        out["ee_left"] = {"t": [], "dt_ms": []}

    for key in SOLVE_LOOP_TOPICS:
        t = np.asarray(data.get(f"solve_{key}_t", []), dtype=float)
        v = np.asarray(data.get(f"solve_{key}_v", []), dtype=float)
        if t.size and v.size and t.size == v.size:
            td, vd = _decimate(t, v.reshape(-1, 1), max_n)
            if vd and isinstance(vd[0], list):
                vd = [row[0] for row in vd]
            out[key] = {"t": td, "ms": vd}
        else:
            out[key] = {"t": [], "ms": []}
    return out


def write_html_report(data, meta, out_dir, arm_cmp_rel="arm_cmp", counts=None, shm_stats=None):
    """生成可交互 HTML：SHM traj + 全链路 + timing_stats。返回 report 路径。"""
    os.makedirs(out_dir, exist_ok=True)
    payload = _build_report_payload(data, meta)
    if counts:
        payload["counts"] = counts
    else:
        payload["counts"] = {
            "shm_traj": int(np.asarray(data.get("traj_t", [])).size),
            "traj": int(np.asarray(data.get("traj_t", [])).size),
            "sens": int(np.asarray(data.get("sens_t", [])).size),
            "cmd": int(np.asarray(data.get("cmd_t", [])).size),
            "traj_sensor": int(np.asarray(data.get("traj_sens_t", [])).size),
            **{k: int(np.asarray(data.get(f"{k}_t", [])).size) for k in PIPELINE_EE_TOPICS},
        }
    payload["tips"] = _auto_tips(payload["counts"])
    payload["shm_stats"] = shm_stats or meta.get("shm_stats") or {}
    payload["shm_link_verified"] = bool(meta.get("shm_link_verified", False))
    payload["shm_link_active"] = bool(meta.get("shm_link_active", False))
    payload["shm_svc_name"] = meta.get("shm_svc_name", "/humanoid_controller/set_incremental_arm_traj_link")

    # SHM 链路健康：检查 using_shm 是否在轨迹段中出现过
    rs = data.get("recv_using_shm_v", np.array([]))
    shm_ever_on = bool(getattr(rs, "size", 0) and float(np.max(np.asarray(rs, dtype=float))) > 0.5)
    payload["shm_link_ever_on"] = shm_ever_on
    if shm_ever_on:
        idx = np.where(np.asarray(rs, dtype=float) > 0.5)[0]
        rs_t = data.get("recv_using_shm_t", np.array([]))
        if getattr(rs_t, "size", 0) and idx.size:
            payload["shm_on_times"] = [float(rs_t[i]) for i in idx[:2]]  # first 2 transitions
    else:
        payload["shm_on_times"] = []

    imgs = []
    cmp_dir = os.path.join(out_dir, arm_cmp_rel) if arm_cmp_rel not in (".", "") else out_dir
    for name in (
        "arm_cmp_left.png", "arm_cmp_right.png", "arm_cmp_err.png",
        "pipeline_ee_left.png", "pipeline_ee_right.png",
        "pipeline_joint_left.png", "pipeline_joint_right.png",
        "timing_intervals.png", "timing_hist.png",
    ):
        if os.path.isfile(os.path.join(cmp_dir, name)):
            rel = name if arm_cmp_rel in (".", "") else f"{arm_cmp_rel}/{name}"
            imgs.append(rel)

    # summary.json 旁路全文
    summary_path = os.path.join(cmp_dir, "summary.json")
    summary_obj = {}
    if os.path.isfile(summary_path):
        try:
            with open(summary_path, "r", encoding="utf-8") as sf:
                summary_obj = json.load(sf)
        except Exception:
            summary_obj = {}
    payload["summary"] = summary_obj

    payload_json = json.dumps(payload, ensure_ascii=False)
    imgs_json = json.dumps(imgs)

    html = f"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1"/>
<title>Incremental IK SHM Report</title>
<script src="https://cdn.jsdelivr.net/npm/plotly.js@2.27.0/dist/plotly.min.js"></script>
<style>
  :root {{ font-family: ui-sans-serif, system-ui, sans-serif; color: #111827; }}
  body {{ margin: 0; background: #eef1f6; }}
  header {{ background: #0f172a; color: #f8fafc; padding: 16px 20px; }}
  header h1 {{ margin: 0 0 6px; font-size: 1.2rem; }}
  header .meta {{ opacity: .85; font-size: .82rem; line-height: 1.45; }}
  .wrap {{ max-width: 1400px; margin: 0 auto; padding: 14px; }}
  .card {{ background: #fff; border-radius: 10px; padding: 12px 14px; margin-bottom: 12px;
           box-shadow: 0 1px 3px rgba(0,0,0,.08); }}
  .row {{ display: flex; flex-wrap: wrap; gap: 8px; align-items: center; margin: 6px 0; }}
  button, select {{ cursor: pointer; border: 1px solid #d1d5db; background: #fff;
                    border-radius: 6px; padding: 6px 10px; font-size: .88rem; }}
  button.active {{ background: #0f172a; color: #fff; border-color: #0f172a; }}
  button.seg {{ background: #eef2ff; border-color: #c7d2fe; }}
  button.seg.active {{ background: #4338ca; color: #fff; border-color: #4338ca; }}
  input[type=number] {{ width: 88px; padding: 5px 8px; border: 1px solid #d1d5db; border-radius: 6px; }}
  label {{ font-size: .85rem; color: #4b5563; }}
  #plot {{ width: 100%; min-height: 640px; }}
  .counts {{ display: grid; grid-template-columns: repeat(auto-fill, minmax(120px, 1fr)); gap: 8px; }}
  .counts div {{ background: #f8fafc; border-radius: 6px; padding: 8px; font-size: .82rem; }}
  .counts b {{ display: block; font-size: 1.05rem; }}
  table.stats {{ width: 100%; border-collapse: collapse; font-size: .82rem; }}
  table.stats th, table.stats td {{ border: 1px solid #e5e7eb; padding: 6px 8px; text-align: left; }}
  table.stats th {{ background: #f1f5f9; }}
  table.stats tr.warn td {{ background: #fef2f2; }}
  pre.pipe, pre.json {{ white-space: pre-wrap; font-size: .75rem; background: #0b1220; color: #e5e7eb;
              padding: 12px; border-radius: 8px; overflow: auto; max-height: 480px; }}
  .tips li {{ margin: 4px 0; }}
  .imgs img {{ max-width: 100%; border: 1px solid #e5e7eb; border-radius: 6px; margin: 8px 0; }}
  details summary {{ cursor: pointer; font-weight: 600; }}
  .badge {{ display: inline-block; background: #dbeafe; color: #1e40af; border-radius: 999px;
            padding: 2px 8px; font-size: .75rem; margin-right: 6px; }}
  .health-grid {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(180px, 1fr)); gap: 10px; }}
  .health-item {{ background: #f8fafc; border-radius: 8px; padding: 10px 14px; border: 1px solid #e5e7eb; }}
  .health-item .h-label {{ font-size: .72rem; color: #64748b; text-transform: uppercase; letter-spacing: .04em; }}
  .health-item .h-value {{ font-size: 1.1rem; font-weight: 600; margin: 4px 0; }}
  .health-item.ok .h-value {{ color: #16a34a; }}
  .health-item.warn .h-value {{ color: #ea580c; }}
  .health-item.bad .h-value {{ color: #dc2626; }}
</style>
</head>
<body>
<header>
  <h1>增量 IK → ArmTraj SHM 报告</h1>
  <div class="meta" id="metaLine"></div>
  <div class="meta">traj 源 = ArmTraj SHM (rad→deg 显示) · 不订阅 /kuavo_arm_traj · 末端 xyz=m · 间隔=ms</div>
</header>
<div class="wrap">
  <div class="card" id="shmLinkCard">
    <h3 style="margin:0 0 8px">SHM 链路健康</h3>
    <div class="health-grid" id="shmHealth"></div>
  </div>

  <div class="card">
    <div class="row">
      <span class="badge" id="srcBadge">SHM</span>
      <label>视图</label>
      <select id="viewSel">
        <option value="joint_left">左臂关节 SHM/sensor/cmd</option>
        <option value="joint_right">右臂关节 SHM/sensor/cmd</option>
        <option value="ee_left">左末端 transformer→FK→filter</option>
        <option value="ee_right">右末端 transformer→FK→filter</option>
        <option value="traj_vs_sensor">ArmTraj SHM vs traj_sensor_data</option>
        <option value="timing">SHM/解算间隔</option>
        <option value="transport">using_shm / transport / seq</option>
      </select>
      <label><input type="checkbox" id="showMarkers"/> 显示散点</label>
    </div>
    <div class="row" id="segRow">
      <label>轨迹段</label>
      <button type="button" class="seg active" data-all="1" id="btnAll">全部</button>
    </div>
    <div class="row">
      <label>时间区间 (s)</label>
      <input type="number" id="t0" step="0.1" min="0"/>
      <span>—</span>
      <input type="number" id="t1" step="0.1" min="0"/>
      <button type="button" id="btnApply">应用</button>
      <button type="button" id="btnReset">重置</button>
      <span id="rangeHint" style="color:#6b7280;font-size:.85rem"></span>
    </div>
  </div>

  <div class="card"><div id="plot"></div></div>

  <div class="card">
    <h3 style="margin:0 0 8px">样本数</h3>
    <div class="counts" id="counts"></div>
  </div>

  <div class="card">
    <h3 style="margin:0 0 8px">自动诊断</h3>
    <ul class="tips" id="tips"></ul>
  </div>

  <div class="card">
    <h3 style="margin:0 0 8px">Timing 统计 (gap&gt;{GAP_DT_MS:.0f}ms)</h3>
    <div style="overflow:auto"><table class="stats" id="timingStats"></table></div>
  </div>

  <div class="card">
    <h3 style="margin:0 0 8px">SHM poller / 段列表</h3>
    <div class="row" id="shmStats"></div>
    <div style="overflow:auto;margin-top:8px"><table class="stats" id="segTable"></table></div>
  </div>

  <div class="card">
    <details open>
      <summary>增量链路说明</summary>
      <pre class="pipe" id="pipeDoc"></pre>
    </details>
  </div>

  <div class="card">
    <details>
      <summary>Meta / summary.json 全文</summary>
      <pre class="json" id="metaJson"></pre>
    </details>
  </div>

  <div class="card">
    <details>
      <summary>静态图（若有 PNG）</summary>
      <div class="imgs" id="imgs"></div>
    </details>
  </div>
</div>

<script>
const DATA = {payload_json};
const IMGS = {imgs_json};
const PIPE_DOC = {json.dumps(PIPELINE_DOC)};

const tMax = DATA.t_max || 1;
let range = [0, tMax];
let activeSeg = null;

function $(id) {{ return document.getElementById(id); }}

function fmt(v, n=3) {{
  if (v === undefined || v === null || Number.isNaN(v)) return "-";
  return Number(v).toFixed(n);
}}

function initShmHealth() {{
  const svc = DATA.shm_svc_name || "?";
  const verified = DATA.shm_link_verified;
  const everOn = DATA.shm_link_ever_on;
  const shmN = (DATA.counts && DATA.counts.shm_traj) || 0;
  const ss = DATA.shm_stats || {{}};
  const gapMs = DATA.gap_ms || 25;
  const ts = DATA.timing_stats || {{}};
  const tr = ts.traj_dt_recv || {{}};

  const items = [];

  // Service
  items.push({{
    label: "SetIncrementalArmTrajLink",
    value: verified ? "✓ 已注册" : "✗ 未注册",
    cls: verified ? "ok" : "warn",
    detail: svc
  }});

  // SHM ever active
  items.push({{
    label: "SHM 激活",
    value: everOn ? "✓ 检测到 using_shm=1" : "— 未激活",
    cls: everOn ? "ok" : "warn",
    detail: DATA.shm_on_times && DATA.shm_on_times.length
      ? `首次: ${{DATA.shm_on_times[0].toFixed(2)}}s` : ""
  }});

  // SHM samples
  items.push({{
    label: "SHM 样本数",
    value: shmN > 0 ? `${{shmN}} 帧` : "0",
    cls: shmN > 0 ? "ok" : "warn",
    detail: shmN > 0 ? `seq ${{ss.first_seq || "?"}} – ${{ss.last_seq || "?"}}` : ""
  }});

  // Interval
  items.push({{
    label: "SHM 中位间隔",
    value: tr.n ? `${{tr.median_ms.toFixed(2)}} ms` : "—",
    cls: (tr.n && tr.median_ms < gapMs) ? "ok" : ((tr.n && tr.median_ms < gapMs * 2) ? "warn" : "bad"),
    detail: tr.n ? `p95=${{tr.p95_ms.toFixed(2)}}ms  Hz~${{tr.hz_med.toFixed(0)}}` : ""
  }});

  // Gap
  items.push({{
    label: "SHM 阻塞",
    value: tr.gap_pct !== undefined ? `${{tr.gap_pct.toFixed(1)}}% (${{tr.gap_runs || 0}} 段)` : "—",
    cls: (tr.gap_pct || 0) < 1 ? "ok" : ((tr.gap_pct || 0) < 5 ? "warn" : "bad"),
    detail: `max dt=${{fmt(tr.max_ms, 2)}}ms`
  }});

  // Poller stats
  items.push({{
    label: "SHM poller",
    value: ss.init_ok ? `✓ ${{ss.updates || 0}} 次更新` : "✗",
    cls: ss.init_ok ? "ok" : "bad",
    detail: ss.max_inter_update_ms
      ? `max间隙=${{ss.max_inter_update_ms.toFixed(1)}}ms` : ""
  }});

  $("shmHealth").innerHTML = items.map(it =>
    `<div class="health-item ${{it.cls}}">
      <div class="h-label">${{it.label}}</div>
      <div class="h-value">${{it.value}}</div>
      <div style="font-size:.72rem;color:#94a3b8">${{it.detail || ""}}</div>
    </div>`
  ).join("");
}}
  const m = DATA.meta || {{}};
  const disp = DATA.display || {{}};
  $("metaLine").textContent =
    `v${{m.robot_version ?? "?"}}  arm=${{DATA.n_arm}}  offset=${{m.arm_offset ?? "?"}}  ` +
    `${{m.wheel_like ? "wheel-like" : "biped"}}  t_max=${{tMax.toFixed(2)}}s  ` +
    `laps=${{m.circle_laps ?? "?"}}  sec/lap=${{m.sec_per_lap ?? "?"}}`;
  if (disp.decimated) {{
    $("metaLine").textContent +=
      `  | HTML抽稀 ${{disp.n_src}}→${{disp.n_out}} (cap=${{disp.cap_n}})`;
  }} else {{
    $("metaLine").textContent += `  | HTML全量 n=${{disp.n_out ?? "?"}}`;
  }}
  $("srcBadge").textContent = disp.traj_source || "ArmTraj SHM";
  $("t0").value = 0;
  $("t1").value = tMax.toFixed(2);
  $("t1").max = tMax;
  $("rangeHint").textContent = `全长 ${{tMax.toFixed(2)}}s`;
  $("pipeDoc").textContent = PIPE_DOC;
  $("metaJson").textContent = JSON.stringify({{meta: m, summary: DATA.summary, shm_stats: DATA.shm_stats, timing_stats: DATA.timing_stats}}, null, 2);

  const c = DATA.counts || {{}};
  $("counts").innerHTML = Object.keys(c).map(k =>
    `<div><b>${{c[k]}}</b>${{k}}</div>`).join("");

  $("tips").innerHTML = (DATA.tips || []).map(t => `<li>${{t}}</li>`).join("");

  // timing stats table
  const ts = DATA.timing_stats || {{}};
  const rows = Object.keys(ts).map(k => {{
    const s = ts[k] || {{}};
    const warn = (s.gap_pct || 0) > 1.0 || (s.max_ms || 0) > (DATA.gap_ms || 25) * 2;
    return `<tr class="${{warn ? "warn" : ""}}"><td>${{k}}</td><td>${{s.n ?? 0}}</td>` +
      `<td>${{fmt(s.median_ms,2)}}</td><td>${{fmt(s.mean_ms,2)}}</td><td>${{fmt(s.p95_ms,2)}}</td>` +
      `<td>${{fmt(s.max_ms,2)}}</td><td>${{s.gap_n ?? "-"}}</td><td>${{fmt(s.gap_pct,2)}}</td>` +
      `<td>${{fmt(s.hz_med,1)}}</td></tr>`;
  }}).join("");
  $("timingStats").innerHTML =
    `<thead><tr><th>series</th><th>n</th><th>med ms</th><th>mean</th><th>p95</th><th>max</th><th>gap_n</th><th>gap%</th><th>Hz</th></tr></thead><tbody>${{rows}}</tbody>`;

  const ss = DATA.shm_stats || {{}};
  $("shmStats").innerHTML = Object.keys(ss).length
    ? Object.keys(ss).map(k => `<div class="badge">${{k}}=${{ss[k]}}</div>`).join("")
    : `<span style="color:#6b7280">无 shm_stats</span>`;

  const segs = DATA.segments || [];
  $("segTable").innerHTML = `<thead><tr><th>#</th><th>name</th><th>t0</th><th>t1</th><th>Δs</th></tr></thead><tbody>` +
    segs.map((s,i) => `<tr><td>${{i}}</td><td>${{s.name}}</td><td>${{fmt(s.t0,2)}}</td><td>${{fmt(s.t1,2)}}</td><td>${{fmt(s.t1-s.t0,2)}}</td></tr>`).join("") +
    `</tbody>`;

  const segRow = $("segRow");
  segs.forEach((s, i) => {{
    const b = document.createElement("button");
    b.type = "button";
    b.className = "seg";
    b.textContent = s.name;
    b.title = `${{s.t0.toFixed(2)}} – ${{s.t1.toFixed(2)}} s`;
    b.onclick = () => {{
      document.querySelectorAll("button.seg").forEach(x => x.classList.remove("active"));
      b.classList.add("active");
      activeSeg = i;
      setRange(s.t0, s.t1, false);
      render();
    }};
    segRow.appendChild(b);
  }});

  $("btnAll").onclick = () => {{
    document.querySelectorAll("button.seg").forEach(x => x.classList.remove("active"));
    $("btnAll").classList.add("active");
    activeSeg = null;
    setRange(0, tMax, false);
    render();
  }};
  $("btnApply").onclick = () => {{
    document.querySelectorAll("button.seg").forEach(x => x.classList.remove("active"));
    activeSeg = null;
    setRange(parseFloat($("t0").value), parseFloat($("t1").value), false);
    render();
  }};
  $("btnReset").onclick = () => $("btnAll").onclick();
  $("viewSel").onchange = render;
  $("showMarkers").onchange = render;

  const box = $("imgs");
  if (!IMGS.length) box.innerHTML = "<span style='color:#6b7280'>无 PNG（信息已在上方交互图/表）</span>";
  IMGS.forEach(src => {{
    const img = document.createElement("img");
    img.src = src; img.alt = src;
    box.appendChild(img);
  }});
}}

function setRange(a, b, syncInputs=true) {{
  let t0 = Math.max(0, Math.min(a, b));
  let t1 = Math.min(tMax, Math.max(a, b));
  if (!(t1 > t0)) {{ t0 = 0; t1 = tMax; }}
  range = [t0, t1];
  $("t0").value = t0.toFixed(2);
  $("t1").value = t1.toFixed(2);
  $("rangeHint").textContent = `显示 ${{t0.toFixed(2)}} – ${{t1.toFixed(2)}} s  (Δ=${{(t1-t0).toFixed(2)}}s)`;
}}

function sliceSeries(t, rows) {{
  const outT = [], outR = [];
  if (!t || !t.length) return {{t: outT, rows: outR}};
  for (let i = 0; i < t.length; i++) {{
    if (t[i] < range[0] || t[i] > range[1]) continue;
    outT.push(t[i]);
    if (rows && rows[i] !== undefined) outR.push(rows[i]);
  }}
  return {{t: outT, rows: outR}};
}}

function modeStyle() {{
  return $("showMarkers").checked ? "lines+markers" : "lines";
}}

function mkTrace(opts) {{
  const n = (opts.x && opts.x.length) || 0;
  return Object.assign({{
    type: n > 4000 ? "scattergl" : "scatter",
    mode: modeStyle(),
    marker: {{size: 3}},
  }}, opts);
}}

function jointTraces(side) {{
  const n = DATA.n_arm;
  const lo = side === "left" ? 0 : Math.floor(n / 2);
  const hi = side === "left" ? Math.floor(n / 2) : n;
  const sources = [
    ["traj", "ArmTraj SHM", "#2563eb"],
    ["sens", "/sensors_data_raw", "#ea580c"],
    ["cmd", "/joint_cmd", "#16a34a"],
  ];
  const figs = [];
  for (let j = lo; j < hi; j++) {{
    const tr = [];
    sources.forEach(([key, name, color]) => {{
      const s = sliceSeries(DATA[key].t, DATA[key].q);
      tr.push(mkTrace({{
        x: s.t, y: s.rows.map(r => r[j]), name,
        line: {{width: 1.2, color}}, legendgroup: name, showlegend: j === lo,
      }}));
    }});
    figs.push(tr);
  }}
  return figs;
}}

function eeTraces(side) {{
  const keys = side === "left"
    ? [["tf_left","transformer","#6366f1"],["ee_left","ee_raw","#2563eb"],["ee_left_f","ee_filter","#16a34a"]]
    : [["tf_right","transformer","#6366f1"],["ee_right","ee_raw","#2563eb"],["ee_right_f","ee_filter","#16a34a"]];
  const axes = ["x","y","z"];
  return axes.map((ax, ai) => keys.filter(([k]) => (DATA.ee[k]?.t?.length || 0) > 0).map(([k, name, color]) => {{
    const s = sliceSeries(DATA.ee[k].t, DATA.ee[k].xyz);
    return mkTrace({{
      x: s.t, y: s.rows.map(r => r[ai]), name,
      line: {{width: 1.2, color}}, legendgroup: name, showlegend: ai === 0,
    }});
  }}));
}}

function trajSensorFigs() {{
  const n = DATA.n_arm;
  const figs = [];
  for (let j = 0; j < n; j++) {{
    const tr = [];
    [["traj","ArmTraj SHM","#2563eb"],["traj_sens","/kuavo_arm_traj_sensor_data","#dc2626"]].forEach(([key, name, color]) => {{
      const s = sliceSeries(DATA[key].t, DATA[key].q);
      if (!s.t.length) return;
      tr.push(mkTrace({{
        x: s.t, y: s.rows.map(r => r[j]), name,
        line: {{width: 1.2, color}}, legendgroup: name, showlegend: j === 0,
      }}));
    }});
    figs.push(tr);
  }}
  return figs;
}}

function timingFigs() {{
  const gap = DATA.gap_ms || 25;
  const figs = [];
  const titles = [];
  const tm = DATA.timing || {{}};
  // SHM recv dt (raw + rolling median)
  {{
    const tr = [];
    const a = tm.traj_recv || {{t:[], dt_ms:[]}};
    const sa = sliceSeries(a.t, a.dt_ms.map(v => [v]));
    if (sa.t.length) tr.push(mkTrace({{x: sa.t, y: sa.rows.map(r => r[0]),
      name: "recv dt (raw)", line:{{width:0.6, color:"#2563eb"}}, opacity:0.5}}));
    const rm = tm.traj_recv_rm || {{t:[], dt_ms:[]}};
    const srm = sliceSeries(rm.t, (rm.dt_ms||[]).map(v => [v]));
    if (srm.t.length) tr.push(mkTrace({{x: srm.t, y: srm.rows.map(r => r[0]),
      name: "recv dt (rolling med)", line:{{width:1.5, color:"#2563eb"}}}}));
    // stamp dt (IK writer side)
    const b = tm.traj_hdr || {{t:[], dt_ms:[]}};
    const sb = sliceSeries(b.t, (b.dt_ms||[]).map(v => [v]));
    if (sb.t.length) tr.push(mkTrace({{x: sb.t, y: sb.rows.map(r => r[0]),
      name: "stamp dt (IK writer)", line:{{width:1.0, color:"#dc2626"}}}}));
    tr.push(mkTrace({{x: [range[0], range[1]], y: [gap, gap],
      name: `gap>${{gap}}ms`, line:{{width:1, color:"#ef4444", dash:"dash"}}}}));
    figs.push(tr); titles.push("SHM dt (ms)");
  }}
  {{
    const a = tm.ee_left || {{t:[], dt_ms:[]}};
    const sa = sliceSeries(a.t, a.dt_ms.map(v => [v]));
    const tr = [];
    if (sa.t.length) tr.push(mkTrace({{x: sa.t, y: sa.rows.map(r => r[0]), name: "ee_left dt", line:{{width:1.1, color:"#ea580c"}}}}));
    tr.push(mkTrace({{x: [range[0], range[1]], y: [gap, gap], name: `gap>${{gap}}ms`, line:{{width:1, color:"#ef4444", dash:"dash"}}, showlegend:false}}));
    figs.push(tr); titles.push("ee dt (ms)");
  }}
  {{
    const tr = [];
    [["loop_period","#16a34a"],["fsm_block_total","#7c3aed"],["fsm_process","#0891b2"]].forEach(([k,c]) => {{
      const a = tm[k] || {{t:[], ms:[]}};
      const sa = sliceSeries(a.t, (a.ms||[]).map(v => [v]));
      if (sa.t.length) tr.push(mkTrace({{x: sa.t, y: sa.rows.map(r => r[0]), name: k, line:{{width:1.1, color:c}}}}));
    }});
    if (!tr.length) tr.push(mkTrace({{x:[], y:[], name:"(no solve_loop_ms)"}}));
    figs.push(tr); titles.push("solve_loop_ms");
  }}
  return {{figs, titles, ylab: "ms"}};
}}

function transportFigs() {{
  const figs = [], titles = [];
  const rs = DATA.recv_status || {{}};
  {{
    const tr = [];
    [["using_shm","#2563eb"],["transport","#ea580c"]].forEach(([k,c]) => {{
      const a = rs[k] || {{t:[], v:[]}};
      const sa = sliceSeries(a.t, (a.v||[]).map(v => [v]));
      if (sa.t.length) tr.push(mkTrace({{x: sa.t, y: sa.rows.map(r => r[0]), name: k,
        line:{{width:1.4, color:c, shape:"hv"}}}}));
    }});
    if (!tr.length) tr.push(mkTrace({{x:[], y:[], name:"(no recv status)"}}));
    figs.push(tr); titles.push("transport");
  }}
  {{
    const a = DATA.traj_seq || {{t:[], seq:[]}};
    const sa = sliceSeries(a.t, (a.seq||[]).map(v => [v]));
    const tr = [];
    if (sa.t.length) tr.push(mkTrace({{x: sa.t, y: sa.rows.map(r => r[0]), name: "SHM seq", line:{{width:1.1, color:"#16a34a"}}}}));
    else tr.push(mkTrace({{x:[], y:[], name:"(no seq)"}}));
    figs.push(tr); titles.push("seq");
  }}
  return {{figs, titles, ylab: ""}};
}}

function render() {{
  const view = $("viewSel").value;
  let figs, titles, ylab;
  if (view === "joint_left") {{
    figs = jointTraces("left"); titles = figs.map((_,i) => `L j${{i+1}}`); ylab = "deg";
  }} else if (view === "joint_right") {{
    figs = jointTraces("right"); titles = figs.map((_,i) => `R j${{i+1}}`); ylab = "deg";
  }} else if (view === "ee_left") {{
    figs = eeTraces("left"); titles = ["x","y","z"]; ylab = "m";
  }} else if (view === "ee_right") {{
    figs = eeTraces("right"); titles = ["x","y","z"]; ylab = "m";
  }} else if (view === "timing") {{
    const r = timingFigs(); figs = r.figs; titles = r.titles; ylab = r.ylab;
  }} else if (view === "transport") {{
    const r = transportFigs(); figs = r.figs; titles = r.titles; ylab = r.ylab;
  }} else {{
    figs = trajSensorFigs(); titles = figs.map((_,i) => `j${{i+1}}`); ylab = "deg";
  }}

  const n = figs.length;
  const layout = {{
    grid: {{rows: n, columns: 1, pattern: "independent"}},
    height: Math.max(480, 88 * n),
    margin: {{t: 40, r: 20, b: 40, l: 55}},
    legend: {{orientation: "h", y: 1.08}},
    xaxis: {{title: "t (s)", range: range}},
  }};
  const traces = [];
  figs.forEach((trList, i) => {{
    const xa = i === 0 ? "x" : `x${{i+1}}`;
    const ya = i === 0 ? "y" : `y${{i+1}}`;
    trList.forEach(tr => {{
      traces.push(Object.assign({{}}, tr, {{xaxis: xa, yaxis: ya}}));
    }});
    layout[xa === "x" ? "xaxis" : `xaxis${{i+1}}`] = {{
      title: i === n - 1 ? "t (s)" : "",
      range: range,
      matches: i === 0 ? undefined : "x",
      showticklabels: i === n - 1,
    }};
    layout[ya === "y" ? "yaxis" : `yaxis${{i+1}}`] = {{
      title: titles[i] + (ylab ? ` (${{ylab}})` : ""),
      zeroline: false,
      autorange: true,
    }};
  }});
  const gap = 0.02;
  const h = (1 - gap * (n - 1)) / n;
  for (let i = 0; i < n; i++) {{
    const y1 = 1 - i * (h + gap);
    const y0 = y1 - h;
    const key = i === 0 ? "yaxis" : `yaxis${{i+1}}`;
    layout[key].domain = [y0, y1];
    const xkey = i === 0 ? "xaxis" : `xaxis${{i+1}}`;
    layout[xkey].anchor = i === 0 ? "y" : `y${{i+1}}`;
    layout[key].anchor = i === 0 ? "x" : `x${{i+1}}`;
  }}
  Plotly.react("plot", traces, layout, {{responsive: true, displayModeBar: true, scrollZoom: true}});
}}

initMeta();
  initShmHealth();
render();
</script>
</body>
</html>
"""
    fp = os.path.join(out_dir, "report.html")
    with open(fp, "w", encoding="utf-8") as f:
        f.write(html)
    print(f"[HTML] {fp}")
    return fp


def _plot_err(ax, data, a_key, b_key, scale, title):
    ta, qa = data[f"{a_key}_t"], data[f"{a_key}_q"]
    tb, qb = data[f"{b_key}_t"], data[f"{b_key}_q"]
    if ta.size < 2 or tb.size < 2 or qa.ndim != 2 or qb.ndim != 2:
        ax.set_title(f"{title} (insufficient data)")
        ax.grid(True, alpha=0.3)
        return
    # 把 a 插值到 b 的时间轴
    n = min(qa.shape[1], qb.shape[1])
    err = np.zeros((tb.size, n))
    for j in range(n):
        err[:, j] = np.interp(tb, ta, qa[:, j]) - qb[:, j]
    rms = np.sqrt(np.mean(err**2, axis=0)) * scale
    for j in range(n):
        ax.plot(tb, err[:, j] * scale, lw=0.8, alpha=0.7, label=f"j{j+1}")
    ax.set_title(f"{title}  RMS(deg)=" + ",".join(f"{v:.2f}" for v in rms))
    ax.set_ylabel("err")
    ax.grid(True, alpha=0.3)
    ax.legend(ncol=7, fontsize=7, loc="upper right")


def record_live(duration, n_arm, arm_offset):
    rec = ArmCmpRecorder(n_arm, arm_offset)
    rec.subscribe_all()
    print(f"[REC] 录制 {duration}s … (arm_n={n_arm}, offset={arm_offset}, source=SHM)")
    t0 = time.time()
    try:
        while time.time() - t0 < duration and not rospy.is_shutdown():
            elapsed = time.time() - t0
            c = rec.pipeline_counts()
            sys.stdout.write(
                f"\r  {elapsed:5.1f}s  shm={c['shm_traj']} eeL={c['ee_left']} sens={c['sens']} cmd={c['cmd']}  ")
            sys.stdout.flush()
            rospy.sleep(0.2)
    finally:
        rec.stop()
    print("")
    return rec.snapshot(), rec.pipeline_counts()


def main():
    p = argparse.ArgumentParser(description="Plot kuavo_arm_traj vs sensors vs joint_cmd")
    p.add_argument("--duration", type=float, default=20.0, help="录制秒数")
    p.add_argument("--npz", default=None, help="已有 npz，跳过录制")
    p.add_argument("-o", "--output", default="/tmp/arm_cmp")
    p.add_argument("--version", type=int, default=None, help="ROBOT_VERSION，默认读环境/rosparam")
    p.add_argument("--joints", default=None, help="逗号分隔关节索引，默认全部手臂")
    p.add_argument("--rad", action="store_true", help="纵轴用弧度（默认度）")
    a = p.parse_args()

    rospy.init_node("plot_arm_traj_sensor_cmd", anonymous=True, disable_signals=True)
    n_arm, arm_offset, n_tot, meta = load_layout(a.version)
    print(f"[LAYOUT] v{meta['robot_version']} total={n_tot} arm={n_arm} "
          f"offset={arm_offset} ({'wheel-like' if meta['wheel_like'] else 'biped'})")
    print(f"         config={meta['config']}")

    label = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = os.path.join(a.output, label)
    os.makedirs(out_dir, exist_ok=True)

    if a.npz:
        data, meta2 = load_npz(a.npz)
        meta.update(meta2)
    else:
        data, _counts = record_live(a.duration, n_arm, arm_offset)
        save_npz(os.path.join(out_dir, "arm_cmp.npz"), data, meta)

    joints = None
    if a.joints:
        joints = [int(x) for x in a.joints.split(",") if x.strip() != ""]

    plot_compare(data, meta, out_dir, joints=joints, deg=not a.rad)
    report = write_html_report(data, meta, out_dir, arm_cmp_rel=".", counts=None)
    print(f"[DONE] {out_dir}")
    print(f"[HTML] {report}")


if __name__ == "__main__":
    main()
