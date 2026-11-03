#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""头部标定示教点独立运动脚本：不依赖 launch，逐点驱动头部并回报反馈角。

点位来源：teach_capture_output/teach_head_joint.json（teach_joint_capture.py 拖动示教采集，
取自 /sensors_data_raw joint_q[-2], joint_q[-1] 的 9 个 yaw/pitch，单位弧度）。

执行流程：
  等待 /sensors_data_raw  ->  打印 9 个目标点（角度制）与当前反馈  ->  人工确认
  ->  逐点：move_sec 插值移动，hold_sec 保持（保持后半段统计反馈均值/误差）
  ->  回到 0°  ->  停止录制  ->  打印每点误差汇总。

录制（默认开启）：
  - rosbag：/joint_cmd、/sensors_data_raw（可用 _bag_topics:="..." 覆盖）
  - CSV：逐帧只记头部字段（指令/反馈/joint_cmd 的 yaw/pitch、kp/kd/tau）
  - 输出到 output/head_teach_points_motion/session_YYYYmmdd_HHMMSS/
  关闭录制：_record_bag:=false / _record_csv:=false

关节约定（与 head_joint_state_bridge / teach_joint_capture 一致）：
  头部 yaw/pitch 取 joint_q 末尾两维，不假定总长度。S62 轮臂 joint_q 长度为 20，
  双足机型为 29；如需严格要求可用 _min_joint_q_length:=N 指定。

安全约定：
  - 头部限位 yaw ±30°、pitch ±25°，脚本内做 clamp（与 head_table_publisher 一致）。
  - 只发 /robot_head_motion_data（角度制），不会改任何标定/零点文件。
  - Ctrl+C 停发后头部停在最后一条指令；异常时先停控制程序，再按急停，最后断主电。
    不要用手扶头。

用法（source 工作空间、运控已启动）：
  python3 src/Camera_Calibration/demos/kuavo_head_demo/head_teach_points_motion.py
  # 只打印点位，不连 ROS、不发指令
  python3 .../head_teach_points_motion.py --dry-run
  # 调整时序 / 跳过确认
  python3 .../head_teach_points_motion.py _move_sec:=2.0 _hold_sec:=3.0 _auto_start:=true
"""

from __future__ import print_function

import csv
import math
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Dict, List, Optional, TextIO, Tuple

# --dry-run 只打印点位，不应要求 source 过工作空间；ROS 依赖延迟到真正运行时检查。
try:
    import rospy
    from kuavo_msgs.msg import jointCmd, robotHeadMotionData, sensorsData

    _ROS_AVAILABLE = True
    _ROS_IMPORT_ERROR = None
except ImportError as e:  # pragma: no cover - 仅在未 source 工作空间时触发
    rospy = None
    jointCmd = None
    robotHeadMotionData = None
    sensorsData = None
    _ROS_AVAILABLE = False
    _ROS_IMPORT_ERROR = e

# 与 /robot_head_motion_data 约定一致（度）
YAW_LIMIT_DEG = 30.0
PITCH_LIMIT_DEG = 25.0

# teach_head_joint.json：samples[*].head_yaw / head_pitch，单位 rad
TEACH_POINTS_RAD = [
    {"name": "T0", "yaw": 0.005338613297078513, "pitch": -0.004576447128184653},
    {"name": "T1", "yaw": 0.17776908866449462, "pitch": -0.21362670721101068},
    {"name": "T2", "yaw": -0.23422603721639326, "pitch": -0.21400788390658412},
    {"name": "T3", "yaw": -0.1899736526533745, "pitch": -0.30365467658220147},
    {"name": "T4", "yaw": 0.1766244864618193, "pitch": -0.3036543976792258},
    {"name": "T5", "yaw": 0.19188321443553977, "pitch": -0.032425912414324805},
    {"name": "T6", "yaw": -0.22697833123332492, "pitch": -0.03280695866169266},
    {"name": "T7", "yaw": -0.1766223760321624, "pitch": -0.14190790847700563},
    {"name": "T8", "yaw": 0.2803814553257301, "pitch": -0.1419073730995035},
]


def _deg(rad: float) -> float:
    return float(rad) * 180.0 / math.pi


def _clamp(yaw_deg: float, pitch_deg: float) -> Dict[str, float]:
    return {
        "yaw_deg": max(-YAW_LIMIT_DEG, min(YAW_LIMIT_DEG, float(yaw_deg))),
        "pitch_deg": max(-PITCH_LIMIT_DEG, min(PITCH_LIMIT_DEG, float(pitch_deg))),
    }


def load_points() -> List[Dict[str, float]]:
    """示教点（rad）-> 指令点（deg，已 clamp）。"""
    points: List[Dict[str, float]] = []
    for idx, sample in enumerate(TEACH_POINTS_RAD):
        q = _clamp(_deg(sample["yaw"]), _deg(sample["pitch"]))
        q["name"] = sample["name"]
        q["index"] = idx
        points.append(q)
    return points


def print_points(points: List[Dict[str, float]]) -> None:
    print("")
    print("头部标定示教点（来自 teach_head_joint.json，已转换为度并 clamp）")
    print("  #   名称      yaw(deg)   pitch(deg)")
    for p in points:
        print("  %d   %-4s  %+9.3f  %+10.3f" % (p["index"], p["name"], p["yaw_deg"], p["pitch_deg"]))
    print("  限位: yaw ±%.0f°, pitch ±%.0f°" % (YAW_LIMIT_DEG, PITCH_LIMIT_DEG))
    print("")


class HeadRecorder:
    """订阅 /sensors_data_raw 与 /joint_cmd，只保留头部末尾两维并逐帧记行。

    - 反馈（sensors）：joint_q/joint_v/joint_torque 的末尾两维，弧度 -> 度
    - 指令（joint_cmd）：joint_q/control_modes/joint_kp/joint_kd/tau 的末尾两维
    - target_* 为脚本当前下发的目标点（度），便于区分移动段与保持段
    """

    CSV_FIELDS = [
        "t",
        "target_name", "target_yaw_deg", "target_pitch_deg",
        "fb_yaw_deg", "fb_pitch_deg",
        "fb_yaw_vel", "fb_pitch_vel",
        "fb_yaw_torque", "fb_pitch_torque",
        "cmd_yaw_deg", "cmd_pitch_deg",
        "cmd_modes_yaw", "cmd_modes_pitch",
        "cmd_kp_yaw", "cmd_kp_pitch",
        "cmd_kd_yaw", "cmd_kd_pitch",
        "cmd_tau_yaw", "cmd_tau_pitch",
    ]

    def __init__(self, sensor_topic: str, cmd_topic: str) -> None:
        self._sensor_topic = sensor_topic
        self._lock = threading.Lock()
        self._q: Optional[List[float]] = None
        self._q_len = 0
        self._t0: Optional[float] = None
        self._cmd = None
        self._target: Dict[str, float] = {"name": "-", "yaw_deg": 0.0, "pitch_deg": 0.0}
        self.rows: List[Dict] = []
        rospy.Subscriber(sensor_topic, sensorsData, self._on_sensors, queue_size=500)
        rospy.Subscriber(cmd_topic, jointCmd, self._on_joint_cmd, queue_size=500)

    @staticmethod
    def _tail(seq, n: int = 2) -> Tuple[Optional[float], Optional[float]]:
        values = list(seq)
        if len(values) < n:
            return None, None
        return float(values[-2]), float(values[-1])

    def _on_sensors(self, msg: sensorsData) -> None:
        data = msg.joint_data
        q = list(data.joint_q)
        if len(q) < 2:
            return
        yaw_r, pitch_r = q[-2], q[-1]
        vyaw, vpitch = self._tail(data.joint_v)
        tyaw, tpitch = self._tail(data.joint_torque)
        now = msg.header.stamp.to_sec() if msg.header.stamp.to_sec() > 0 else rospy.Time.now().to_sec()

        with self._lock:
            self._q = q
            self._q_len = len(q)
            if self._t0 is None:
                self._t0 = now
            cmd = self._cmd
            target = dict(self._target)
            row = {
                "t": round(now - self._t0, 6),
                "target_name": target["name"],
                "target_yaw_deg": round(target["yaw_deg"], 4),
                "target_pitch_deg": round(target["pitch_deg"], 4),
                "fb_yaw_deg": round(_deg(yaw_r), 4),
                "fb_pitch_deg": round(_deg(pitch_r), 4),
                "fb_yaw_vel": None if vyaw is None else round(vyaw, 5),
                "fb_pitch_vel": None if vpitch is None else round(vpitch, 5),
                "fb_yaw_torque": None if tyaw is None else round(tyaw, 5),
                "fb_pitch_torque": None if tpitch is None else round(tpitch, 5),
            }
            for key in ("cmd_yaw_deg", "cmd_pitch_deg", "cmd_modes_yaw", "cmd_modes_pitch",
                        "cmd_kp_yaw", "cmd_kp_pitch", "cmd_kd_yaw", "cmd_kd_pitch",
                        "cmd_tau_yaw", "cmd_tau_pitch"):
                row[key] = None if cmd is None else cmd[key]
            self.rows.append(row)

    def _on_joint_cmd(self, msg: jointCmd) -> None:
        q = list(msg.joint_q)
        if len(q) < 2:
            return
        kp_yaw, kp_pitch = self._tail(msg.joint_kp)
        kd_yaw, kd_pitch = self._tail(msg.joint_kd)
        tau_yaw, tau_pitch = self._tail(msg.tau)
        mode_yaw, mode_pitch = self._tail(getattr(msg, "control_modes", []))
        with self._lock:
            self._cmd = {
                "cmd_yaw_deg": round(_deg(q[-2]), 4),
                "cmd_pitch_deg": round(_deg(q[-1]), 4),
                "cmd_modes_yaw": None if mode_yaw is None else int(mode_yaw),
                "cmd_modes_pitch": None if mode_pitch is None else int(mode_pitch),
                "cmd_kp_yaw": None if kp_yaw is None else round(kp_yaw, 4),
                "cmd_kp_pitch": None if kp_pitch is None else round(kp_pitch, 4),
                "cmd_kd_yaw": None if kd_yaw is None else round(kd_yaw, 4),
                "cmd_kd_pitch": None if kd_pitch is None else round(kd_pitch, 4),
                "cmd_tau_yaw": None if tau_yaw is None else round(tau_yaw, 5),
                "cmd_tau_pitch": None if tau_pitch is None else round(tau_pitch, 5),
            }

    def set_target(self, point: Dict[str, float]) -> None:
        with self._lock:
            self._target = {
                "name": str(point.get("name", "-")),
                "yaw_deg": float(point.get("yaw_deg", 0.0)),
                "pitch_deg": float(point.get("pitch_deg", 0.0)),
            }

    def wait_valid(self, timeout: float, min_len: int = 2) -> Dict[str, float]:
        deadline = time.time() + timeout
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            with self._lock:
                ok = self._q is not None and len(self._q) >= min_len
            if ok:
                return self.current_deg()
            if time.time() > deadline:
                raise RuntimeError(
                    "等待 %s 超时：需要 joint_q 长度 >= %d 且至少一帧有效数据"
                    % (self._sensor_topic, min_len)
                )
            rate.sleep()
        raise RuntimeError("rospy 已关闭")

    def joint_q_length(self) -> int:
        return self._q_len

    def current_deg(self) -> Dict[str, float]:
        with self._lock:
            q = self._q
        if q is None or len(q) < 2:
            raise RuntimeError("尚未收到有效 sensorsData")
        return {"yaw_deg": _deg(q[-2]), "pitch_deg": _deg(q[-1])}

    def row_count(self) -> int:
        with self._lock:
            return len(self.rows)

    def write_csv(self, path: Path) -> None:
        with self._lock:
            rows = list(self.rows)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=self.CSV_FIELDS)
            writer.writeheader()
            for row in rows:
                writer.writerow(row)


def _start_bag(bag_path: Path, topics: List[str]) -> Tuple[Optional[subprocess.Popen], Optional[TextIO]]:
    """后台启动 rosbag record，等到 bag 开始写盘再返回。"""
    log_path = bag_path.with_suffix(".rosbag.log")
    log_file = log_path.open("w", encoding="utf-8")
    try:
        proc = subprocess.Popen(
            ["rosbag", "record", "--lz4", "-O", str(bag_path), *topics],
            stdout=log_file,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
    except OSError as e:
        log_file.close()
        print("[WARN] 无法启动 rosbag record（%s），跳过录包。" % e, file=sys.stderr)
        return None, None
    active = Path(str(bag_path) + ".active")
    deadline = time.time() + 8.0
    while time.time() < deadline:
        if proc.poll() is not None:
            log_file.close()
            detail = log_path.read_text(encoding="utf-8", errors="replace").strip()
            print("[WARN] rosbag record 已退出 code=%s\n%s" % (proc.returncode, detail), file=sys.stderr)
            return None, None
        if active.is_file() or bag_path.is_file():
            return proc, log_file
        time.sleep(0.05)
    _stop_bag(proc, log_file)
    print("[WARN] rosbag record 8 s 内没有开始写盘，跳过录包。", file=sys.stderr)
    return None, None


def _stop_bag(proc: Optional[subprocess.Popen], log_file: Optional[TextIO]) -> None:
    if proc is not None and proc.poll() is None:
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=20)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=5)
    if log_file is not None and not log_file.closed:
        log_file.close()


def publish(pub: "rospy.Publisher", yaw_deg: float, pitch_deg: float) -> Dict[str, float]:
    q = _clamp(yaw_deg, pitch_deg)
    msg = robotHeadMotionData()
    msg.joint_data = [q["yaw_deg"], q["pitch_deg"]]
    pub.publish(msg)
    return q


def move(pub: "rospy.Publisher", start: Dict[str, float], end: Dict[str, float],
         duration: float, dt: float) -> None:
    duration = max(0.0, float(duration))
    if duration <= 1e-9:
        publish(pub, end["yaw_deg"], end["pitch_deg"])
        return
    t0 = rospy.Time.now().to_sec()
    rate = rospy.Rate(1.0 / dt)
    while not rospy.is_shutdown():
        elapsed = rospy.Time.now().to_sec() - t0
        if elapsed >= duration:
            break
        ratio = max(0.0, min(1.0, elapsed / duration))
        publish(
            pub,
            start["yaw_deg"] + (end["yaw_deg"] - start["yaw_deg"]) * ratio,
            start["pitch_deg"] + (end["pitch_deg"] - start["pitch_deg"]) * ratio,
        )
        rate.sleep()
    publish(pub, end["yaw_deg"], end["pitch_deg"])


def hold(pub: "rospy.Publisher", target: Dict[str, float], hold_sec: float, dt: float,
         recorder: HeadRecorder) -> Dict[str, float]:
    """保持目标并统计后半段反馈均值，返回 cmd/feedback 误差。"""
    hold_sec = max(0.0, float(hold_sec))
    t0 = rospy.Time.now().to_sec()
    rate = rospy.Rate(1.0 / dt)
    samples_yaw: List[float] = []
    samples_pitch: List[float] = []
    tail_start = hold_sec * 0.5
    while not rospy.is_shutdown():
        elapsed = rospy.Time.now().to_sec() - t0
        if elapsed >= hold_sec:
            break
        publish(pub, target["yaw_deg"], target["pitch_deg"])
        if elapsed >= tail_start:
            try:
                fb = recorder.current_deg()
            except RuntimeError:
                fb = None
            if fb is not None:
                samples_yaw.append(fb["yaw_deg"])
                samples_pitch.append(fb["pitch_deg"])
        rate.sleep()
    publish(pub, target["yaw_deg"], target["pitch_deg"])

    result = {"cmd_yaw": target["yaw_deg"], "cmd_pitch": target["pitch_deg"],
              "fb_yaw": None, "fb_pitch": None, "dyaw": None, "dpitch": None}
    if samples_yaw:
        result["fb_yaw"] = sum(samples_yaw) / len(samples_yaw)
        result["fb_pitch"] = sum(samples_pitch) / len(samples_pitch)
        result["dyaw"] = result["fb_yaw"] - target["yaw_deg"]
        result["dpitch"] = result["fb_pitch"] - target["pitch_deg"]
    return result


def confirm_start(auto_start: bool) -> bool:
    if auto_start:
        return True
    print("")
    print("即将驱动头部按上述点位运动；请确认人员远离、急停可用。")
    try:
        sys.stdout.write("继续请回车，输入 n 取消: ")
        sys.stdout.flush()
        line = sys.stdin.readline()
    except KeyboardInterrupt:
        return False
    if not line:
        return False
    return line.strip().lower() != "n"


def main() -> int:
    # 先解析 --dry-run：此时可能还没 source 工作空间，rospy 为 None。
    points = load_points()
    print_points(points)
    if "--dry-run" in sys.argv:
        print("[dry-run] 只打印点位，不连接 ROS、不发指令。")
        return 0
    if not _ROS_AVAILABLE:
        print("[ERROR] 未加载 ROS 依赖（%s）。请先 source /opt/ros/noetic/setup.bash "
              "和工作空间 devel/setup.bash；只查看点位可用 --dry-run。"
              % _ROS_IMPORT_ERROR, file=sys.stderr)
        return 1

    rospy.init_node("head_teach_points_motion", argv=sys.argv, anonymous=False)
    move_sec = float(rospy.get_param("~move_sec", 2.0))
    hold_sec = float(rospy.get_param("~hold_sec", 3.0))
    dt = float(rospy.get_param("~dt", 0.04))
    auto_start = bool(rospy.get_param("~auto_start", False))
    sensor_topic = str(rospy.get_param("~sensor_topic", "/sensors_data_raw"))
    cmd_topic = str(rospy.get_param("~cmd_topic", "/joint_cmd"))
    head_topic = str(rospy.get_param("~head_topic", "/robot_head_motion_data"))
    wait_timeout = float(rospy.get_param("~wait_sensors_timeout", 30.0))
    min_joint_q_length = int(rospy.get_param("~min_joint_q_length", 2))
    record_bag = bool(rospy.get_param("~record_bag", True))
    record_csv = bool(rospy.get_param("~record_csv", True))
    bag_topics = str(rospy.get_param("~bag_topics", "%s %s" % (cmd_topic, sensor_topic))).split()
    script_dir = Path(__file__).resolve().parents[2]  # .../src/Camera_Calibration
    output_root = Path(rospy.get_param(
        "~output_dir", str(script_dir / "output" / "head_teach_points_motion")))

    recorder = HeadRecorder(sensor_topic, cmd_topic)
    print("[INFO] 等待 %s ..." % sensor_topic)
    try:
        current = recorder.wait_valid(wait_timeout, min_joint_q_length)
    except RuntimeError as e:
        rospy.logerr("%s", e)
        return 1
    print("[INFO] joint_q 长度=%d（头部取末尾两维）" % recorder.joint_q_length())
    print("[INFO] 当前反馈 yaw=%+.2f° pitch=%+.2f°" % (current["yaw_deg"], current["pitch_deg"]))
    print("[INFO] 时序: move=%.2fs, hold=%.2fs, dt=%.3fs, 点数=%d"
          % (move_sec, hold_sec, dt, len(points)))

    if not confirm_start(auto_start):
        print("[INFO] 已取消，不发送任何头部指令。")
        return 0

    # 会话目录：录包与 CSV 都放这里，避免覆盖历史数据。
    session_dir = output_root / ("session_%s" % time.strftime("%Y%m%d_%H%M%S"))
    session_dir.mkdir(parents=True, exist_ok=True)
    bag_path = session_dir / "head_points.bag"
    csv_path = session_dir / "head_recording.csv"
    bag_proc = None
    bag_log = None

    pub = rospy.Publisher(head_topic, robotHeadMotionData, queue_size=10)
    rospy.sleep(0.3)

    summaries: List[Dict] = []
    try:
        if record_bag:
            bag_proc, bag_log = _start_bag(bag_path, bag_topics)
            if bag_proc is not None:
                print("[INFO] 录包中: %s  话题: %s" % (bag_path, " ".join(bag_topics)))
            rospy.sleep(0.5)
        for p in points:
            if rospy.is_shutdown():
                break
            recorder.set_target(p)
            move(pub, current, p, move_sec, dt)
            current = {"yaw_deg": p["yaw_deg"], "pitch_deg": p["pitch_deg"]}
            s = hold(pub, p, hold_sec, dt, recorder)
            s["name"] = p["name"]
            s["index"] = p["index"]
            summaries.append(s)
            if s["dpitch"] is None:
                rospy.loginfo("点 %s  指令 yaw=%+.2f° pitch=%+.2f°  （保持段无反馈）",
                              p["name"], p["yaw_deg"], p["pitch_deg"])
            else:
                rospy.loginfo("点 %s  指令 yaw=%+.2f° pitch=%+.2f°  反馈 yaw=%+.2f° pitch=%+.2f°  "
                              "误差 dyaw=%+.2f° dpitch=%+.2f°",
                              p["name"], p["yaw_deg"], p["pitch_deg"],
                              s["fb_yaw"], s["fb_pitch"], s["dyaw"], s["dpitch"])
        if not rospy.is_shutdown():
            print("[INFO] 回到 0°")
            zero = {"name": "ZERO", "yaw_deg": 0.0, "pitch_deg": 0.0}
            recorder.set_target(zero)
            move(pub, current, zero, move_sec, dt)
    except KeyboardInterrupt:
        print("[WARN] 已中断，头部停在最后一条指令。")
    finally:
        _stop_bag(bag_proc, bag_log)
        if record_csv:
            try:
                recorder.write_csv(csv_path)
                print("[INFO] 头部记录 CSV: %s（%d 帧）" % (csv_path, recorder.row_count()))
            except OSError as e:
                print("[WARN] 写 CSV 失败: %s" % e, file=sys.stderr)
        if record_bag and bag_path.exists():
            print("[INFO] 录包: %s" % bag_path)

    print("")
    print("点    yaw指令  pitch指令   yaw反馈  pitch反馈   dyaw    dpitch")
    for s in summaries:
        if s["dpitch"] is None:
            print("%-4s %+8.2f %+10.2f        -          -      -        -"
                  % (s["name"], s["cmd_yaw"], s["cmd_pitch"]))
        else:
            print("%-4s %+8.2f %+10.2f %+9.2f %+10.2f %+7.2f %+8.2f"
                  % (s["name"], s["cmd_yaw"], s["cmd_pitch"],
                     s["fb_yaw"], s["fb_pitch"], s["dyaw"], s["dpitch"]))
    print("")
    return 0


if __name__ == "__main__":
    sys.exit(main())
