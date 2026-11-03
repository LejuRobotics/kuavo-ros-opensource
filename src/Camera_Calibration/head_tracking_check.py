#!/usr/bin/env python3
"""只发头部指令，对照反馈是否跟上。不开相机。

默认轨迹为 pitch 扫描后再做 yaw 扫描：
  pitch: yaw=0, pitch ±22°（限位 ±25°）
  yaw  : pitch=0, yaw ±22°（限位 ±30°）
每个点：2 s 插值移动，3 s 保持。同时写 CSV，并 rosbag record：
  /robot_head_motion_data   joint_data = [yaw_deg, pitch_deg]
  /joint_cmd                末两维为头部，弧度；含 kp/kd/tau/control_modes
  /sensors_data_raw         joint_q/v/torque 末两维为头部
  /cmd_head_vel /cmd_head_delta   与绝对指令写同一头部目标，需一起留下

保持最后 0.5 s 的反馈减指令。若某个轴在保持段浮点完全不变，记为停更。

实机、ROBOT_VERSION=63。头部限位 yaw ±30°、pitch ±25°。
异常时停控制程序，再按急停，最后断主电。不要用手扶头。Ctrl+C 停发，头部停在最后一条指令。
"""

import csv
import json
import math
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Dict, List, Optional, TextIO, Tuple

import rospy
from kuavo_msgs.msg import jointCmd, robotHeadMotionData, sensorsData

YAW_LIMIT_DEG = 30.0
PITCH_LIMIT_DEG = 25.0
BAG_TOPICS = (
    "/robot_head_motion_data",
    "/joint_cmd",
    "/sensors_data_raw",
    "/cmd_head_vel",
    "/cmd_head_delta",
)


def _deg(rad: float) -> float:
    return float(rad) * 180.0 / math.pi


def _clamp(yaw_deg: float, pitch_deg: float) -> Dict[str, float]:
    return {
        "yaw_deg": max(-YAW_LIMIT_DEG, min(YAW_LIMIT_DEG, float(yaw_deg))),
        "pitch_deg": max(-PITCH_LIMIT_DEG, min(PITCH_LIMIT_DEG, float(pitch_deg))),
    }


def pitch_sweep_points(amplitude_deg: float) -> List[Dict[str, float]]:
    """yaw 保持 0，pitch 在 ±amplitude 之间来回。幅度收到限位内 1°。"""
    amp = max(1.0, min(PITCH_LIMIT_DEG - 1.0, float(amplitude_deg)))
    points = []
    for pitch in [amp, -amp, amp, -amp, amp, -amp]:
        q = _clamp(0.0, pitch)
        q["index"] = len(points)
        q["axis"] = "pitch"
        points.append(q)
    return points


def yaw_sweep_points(amplitude_deg: float) -> List[Dict[str, float]]:
    """pitch 保持 0，yaw 在 ±amplitude 之间来回。幅度收到限位内 1°。"""
    amp = max(1.0, min(YAW_LIMIT_DEG - 1.0, float(amplitude_deg)))
    points = []
    for yaw in [amp, -amp, amp, -amp, amp, -amp]:
        q = _clamp(yaw, 0.0)
        q["index"] = len(points)
        q["axis"] = "yaw"
        points.append(q)
    return points


def combined_sweep_points(pitch_amplitude_deg: float, yaw_amplitude_deg: float) -> List[Dict[str, float]]:
    """先 pitch 扫描，再 yaw 扫描；每个点都标明测试轴。"""
    points = pitch_sweep_points(pitch_amplitude_deg) + yaw_sweep_points(yaw_amplitude_deg)
    for index, point in enumerate(points):
        point["index"] = index
    return points


def load_points(json_path: Path) -> List[Dict[str, float]]:
    payload = json.loads(json_path.read_text(encoding="utf-8"))
    points = []
    for sample in payload.get("samples", []):
        if "head_yaw" not in sample or "head_pitch" not in sample:
            continue
        q = _clamp(_deg(sample["head_yaw"]), _deg(sample["head_pitch"]))
        q["index"] = int(sample.get("index", len(points)))
        q["axis"] = "teach"
        points.append(q)
    if not points:
        raise RuntimeError("teach json 里没有 head_yaw/head_pitch: %s" % json_path)
    return points


class HeadLog:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.fb: Optional[Dict[str, float]] = None
        self.joint_cmd: Optional[Dict[str, float]] = None
        self.rows: List[dict] = []
        self._t0 = None

    def on_sensors(self, msg: sensorsData) -> None:
        q = list(msg.joint_data.joint_q)
        if len(q) < 2:
            return
        now = rospy.Time.now().to_sec()
        with self._lock:
            if self._t0 is None:
                self._t0 = now
            self.fb = {"yaw_deg": _deg(q[-2]), "pitch_deg": _deg(q[-1]), "t": now}
            cmd = self.joint_cmd
            self.rows.append({
                "t": now - self._t0,
                "wall_time_s": time.time(),
                "fb_yaw_deg": self.fb["yaw_deg"],
                "fb_pitch_deg": self.fb["pitch_deg"],
                "cmd_yaw_deg": None if cmd is None else cmd["yaw_deg"],
                "cmd_pitch_deg": None if cmd is None else cmd["pitch_deg"],
                "tau_yaw": None if cmd is None else cmd["tau_yaw"],
                "kp_yaw": None if cmd is None else cmd["kp_yaw"],
                "kd_yaw": None if cmd is None else cmd["kd_yaw"],
                "tau_pitch": None if cmd is None else cmd["tau_pitch"],
                "kp_pitch": None if cmd is None else cmd["kp_pitch"],
                "kd_pitch": None if cmd is None else cmd["kd_pitch"],
            })

    def on_joint_cmd(self, msg: jointCmd) -> None:
        if len(msg.joint_q) < 2:
            return
        tau = list(msg.tau)
        kp = list(msg.joint_kp)
        kd = list(msg.joint_kd)
        with self._lock:
            self.joint_cmd = {
                "yaw_deg": _deg(msg.joint_q[-2]),
                "pitch_deg": _deg(msg.joint_q[-1]),
                "tau_yaw": tau[-2] if len(tau) >= 2 else None,
                "kp_yaw": kp[-2] if len(kp) >= 2 else None,
                "kd_yaw": kd[-2] if len(kd) >= 2 else None,
                "tau_pitch": tau[-1] if tau else None,
                "kp_pitch": kp[-1] if kp else None,
                "kd_pitch": kd[-1] if kd else None,
                "t": rospy.Time.now().to_sec(),
            }

    def snapshot(self) -> tuple:
        with self._lock:
            fb = None if self.fb is None else dict(self.fb)
            cmd = None if self.joint_cmd is None else dict(self.joint_cmd)
            n = len(self.rows)
        return fb, cmd, n

    def rows_since(self, n0: int) -> List[dict]:
        with self._lock:
            return list(self.rows[n0:])


def _publish(pub: rospy.Publisher, yaw_deg: float, pitch_deg: float) -> Dict[str, float]:
    q = _clamp(yaw_deg, pitch_deg)
    msg = robotHeadMotionData()
    msg.joint_data = [q["yaw_deg"], q["pitch_deg"]]
    pub.publish(msg)
    return q


def _move(pub, start, end, duration, dt) -> None:
    t0 = rospy.Time.now().to_sec()
    rate = rospy.Rate(1.0 / dt)
    while not rospy.is_shutdown():
        elapsed = rospy.Time.now().to_sec() - t0
        if elapsed >= duration:
            break
        ratio = max(0.0, min(1.0, elapsed / duration))
        _publish(
            pub,
            start["yaw_deg"] + (end["yaw_deg"] - start["yaw_deg"]) * ratio,
            start["pitch_deg"] + (end["pitch_deg"] - start["pitch_deg"]) * ratio,
        )
        rate.sleep()
    _publish(pub, end["yaw_deg"], end["pitch_deg"])


def _hold(pub, target, hold_sec, dt) -> None:
    t0 = rospy.Time.now().to_sec()
    rate = rospy.Rate(1.0 / dt)
    while not rospy.is_shutdown() and (rospy.Time.now().to_sec() - t0) < hold_sec:
        _publish(pub, target["yaw_deg"], target["pitch_deg"])
        rate.sleep()


def _axis_frozen(values: List[float]) -> bool:
    if len(values) < 5:
        return False
    return max(values) - min(values) < 1e-4


def _pitch_frozen(rows: List[dict]) -> bool:
    return _axis_frozen([r["fb_pitch_deg"] for r in rows])


def _yaw_frozen(rows: List[dict]) -> bool:
    return _axis_frozen([r["fb_yaw_deg"] for r in rows])


def _mean_optional(rows: List[dict], key: str) -> Optional[float]:
    values = [r[key] for r in rows if r.get(key) is not None]
    return sum(values) / len(values) if values else None


def _last_optional(rows: List[dict], key: str) -> Optional[float]:
    values = [r[key] for r in rows if r.get(key) is not None]
    return values[-1] if values else None


def _summarize(index: int, target: Dict[str, float], rows: List[dict], tail_sec: float) -> Dict[str, float]:
    if not rows:
        raise RuntimeError("点 %d 保持期间没有 sensors_data_raw" % index)
    t_end = rows[-1]["t"]
    tail = [r for r in rows if r["t"] >= t_end - tail_sec] or rows[-5:]
    fb_yaw = sum(r["fb_yaw_deg"] for r in tail) / len(tail)
    fb_pitch = sum(r["fb_pitch_deg"] for r in tail) / len(tail)
    return {
        "index": index,
        "axis": target.get("axis", ""),
        "cmd_yaw": target["yaw_deg"],
        "cmd_pitch": target["pitch_deg"],
        "fb_yaw": fb_yaw,
        "fb_pitch": fb_pitch,
        "dyaw": fb_yaw - target["yaw_deg"],
        "dpitch": fb_pitch - target["pitch_deg"],
        "joint_cmd_yaw": _mean_optional(tail, "cmd_yaw_deg"),
        "joint_cmd_pitch": _mean_optional(tail, "cmd_pitch_deg"),
        "tau_yaw": _last_optional(tail, "tau_yaw"),
        "kp_yaw": _last_optional(tail, "kp_yaw"),
        "kd_yaw": _last_optional(tail, "kd_yaw"),
        "tau_pitch": _last_optional(tail, "tau_pitch"),
        "kp_pitch": _last_optional(tail, "kp_pitch"),
        "kd_pitch": _last_optional(tail, "kd_pitch"),
        "yaw_frozen": _yaw_frozen(rows),
        "pitch_frozen": _pitch_frozen(rows),
        "n": len(rows),
    }


def _start_bag(bag_path: Path, topics: Tuple[str, ...]) -> Tuple[subprocess.Popen, TextIO]:
    log_path = bag_path.with_suffix(".rosbag.log")
    log_file = log_path.open("w", encoding="utf-8")
    proc = subprocess.Popen(
        ["rosbag", "record", "--lz4", "-O", str(bag_path), *topics],
        stdout=log_file,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    active = Path(str(bag_path) + ".active")
    deadline = time.time() + 8.0
    while time.time() < deadline:
        if proc.poll() is not None:
            log_file.close()
            detail = log_path.read_text(encoding="utf-8", errors="replace").strip()
            raise RuntimeError("rosbag record 已退出 code=%s\n%s" % (proc.returncode, detail))
        if active.is_file() or bag_path.is_file():
            return proc, log_file
        time.sleep(0.05)
    if proc.poll() is None:
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
    log_file.close()
    raise RuntimeError("rosbag record 8 s 内没有开始写 %s" % bag_path)


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


def main() -> None:
    rospy.init_node("head_tracking_check", argv=sys.argv, anonymous=False)

    script_dir = Path(__file__).resolve().parent
    teach_default = script_dir / "teach_capture_output" / "teach_head_joint.json"
    teach_path = Path(rospy.get_param("~teach_json", str(teach_default)))
    move_sec = float(rospy.get_param("~move_sec", 2.0))
    hold_sec = float(rospy.get_param("~hold_sec", 3.0))
    dt = float(rospy.get_param("~dt", 0.04))
    tail_sec = float(rospy.get_param("~tail_sec", 0.5))
    out_root = Path(rospy.get_param(
        "~output_dir",
        str(script_dir / "output" / "head_tracking_check"),
    ))

    trajectory = str(rospy.get_param("~trajectory", "combined_sweep")).strip()
    if trajectory == "teach":
        points = load_points(teach_path)
        rospy.loginfo("teach 轨迹，共 %d 点", len(points))
    elif trajectory == "pitch_sweep":
        points = pitch_sweep_points(float(rospy.get_param("~pitch_amplitude", 22.0)))
        rospy.loginfo(
            "pitch 上下扫 yaw=0，幅度 %.1f°：%s",
            points[0]["pitch_deg"],
            " ".join("%+.0f" % p["pitch_deg"] for p in points),
        )
    elif trajectory == "yaw_sweep":
        points = yaw_sweep_points(float(rospy.get_param("~yaw_amplitude", 22.0)))
        rospy.loginfo(
            "yaw 左右扫 pitch=0，幅度 %.1f°：%s",
            points[0]["yaw_deg"],
            " ".join("%+.0f" % p["yaw_deg"] for p in points),
        )
    elif trajectory == "combined_sweep":
        points = combined_sweep_points(
            float(rospy.get_param("~pitch_amplitude", 22.0)),
            float(rospy.get_param("~yaw_amplitude", 22.0)),
        )
        rospy.loginfo(
            "组合扫描：pitch ±%.1f°（yaw=0），然后 yaw ±%.1f°（pitch=0），共 %d 点",
            float(rospy.get_param("~pitch_amplitude", 22.0)),
            float(rospy.get_param("~yaw_amplitude", 22.0)),
            len(points),
        )
    else:
        raise RuntimeError(
            "未知 trajectory=%s，可选 combined_sweep|pitch_sweep|yaw_sweep|teach" % trajectory
        )

    log = HeadLog()
    rospy.Subscriber("/sensors_data_raw", sensorsData, log.on_sensors, queue_size=200)
    rospy.Subscriber("/joint_cmd", jointCmd, log.on_joint_cmd, queue_size=200)
    pub = rospy.Publisher("/robot_head_motion_data", robotHeadMotionData, queue_size=10)

    rospy.loginfo("等待 /sensors_data_raw …")
    deadline = time.time() + float(rospy.get_param("~wait_sensors_sec", 30.0))
    while not rospy.is_shutdown():
        fb, _, _ = log.snapshot()
        if fb is not None:
            break
        if time.time() > deadline:
            raise RuntimeError("30 s 内没有收到 /sensors_data_raw")
        rospy.sleep(0.05)

    stamp = time.strftime("%Y%m%d_%H%M%S")
    out_dir = out_root / ("session_%s" % stamp)
    out_dir.mkdir(parents=True, exist_ok=True)
    record_bag = bool(rospy.get_param("~record_bag", True))
    bag_path = out_dir / "head_tracking.bag" if record_bag else None
    bag_proc = None
    bag_log = None
    try:
        if record_bag:
            bag_proc, bag_log = _start_bag(bag_path, BAG_TOPICS)
            rospy.loginfo("录包 %s  话题: %s", bag_path, " ".join(BAG_TOPICS))
        rospy.sleep(0.8 if record_bag else 0.3)

        fb, _, _ = log.snapshot()
        current = {"yaw_deg": fb["yaw_deg"], "pitch_deg": fb["pitch_deg"]}
        rospy.loginfo(
            "起点反馈 yaw=%.2f° pitch=%.2f°，点数=%d，移动 %.1fs，保持 %.1fs",
            current["yaw_deg"], current["pitch_deg"], len(points), move_sec, hold_sec,
        )

        summaries = []
        for point in points:
            if rospy.is_shutdown():
                break
            _move(pub, current, point, move_sec, dt)
            _, _, n_hold = log.snapshot()
            _hold(pub, point, hold_sec, dt)
            current = {"yaw_deg": point["yaw_deg"], "pitch_deg": point["pitch_deg"]}
            summary = _summarize(point["index"], point, log.rows_since(n_hold), tail_sec)
            summaries.append(summary)
            flags = []
            if summary["pitch_frozen"]:
                flags.append("pitch停更")
            if summary["yaw_frozen"]:
                flags.append("yaw停更")
            rospy.loginfo(
                "点 %d(%s)  指令 yaw %+.2f° pitch %+.2f°  "
                "反馈 yaw %+.2f° pitch %+.2f°  误差 yaw %+.2f° pitch %+.2f°%s",
                summary["index"],
                summary["axis"],
                summary["cmd_yaw"],
                summary["cmd_pitch"],
                summary["fb_yaw"],
                summary["fb_pitch"],
                summary["dyaw"],
                summary["dpitch"],
                (" " + ",".join(flags)) if flags else "",
            )

        if not rospy.is_shutdown():
            rospy.loginfo("回到 0°")
            _move(pub, current, {"yaw_deg": 0.0, "pitch_deg": 0.0}, move_sec, dt)
    finally:
        _stop_bag(bag_proc, bag_log)

    raw_path = out_dir / "tracking.csv"
    with raw_path.open("w", newline="", encoding="utf-8") as f:
        fields = [
            "t", "wall_time_s", "fb_yaw_deg", "fb_pitch_deg",
            "cmd_yaw_deg", "cmd_pitch_deg",
            "tau_yaw", "kp_yaw", "kd_yaw",
            "tau_pitch", "kp_pitch", "kd_pitch",
        ]
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in log.rows_since(0):
            writer.writerow(row)

    summary_path = out_dir / "hold_summary.csv"
    with summary_path.open("w", newline="", encoding="utf-8") as f:
        fields = [
            "index", "axis", "cmd_yaw", "cmd_pitch", "fb_yaw", "fb_pitch",
            "dyaw", "dpitch", "joint_cmd_yaw", "joint_cmd_pitch",
            "tau_yaw", "kp_yaw", "kd_yaw",
            "tau_pitch", "kp_pitch", "kd_pitch",
            "yaw_frozen", "pitch_frozen", "n",
        ]
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(summaries)

    print("")
    print("点 轴     指令yaw 指令pitch 反馈yaw 反馈pitch 误差yaw 误差pitch 停更")
    for s in summaries:
        flags = []
        if s["yaw_frozen"]:
            flags.append("yaw")
        if s["pitch_frozen"]:
            flags.append("pitch")
        print(
            "%2d %-5s %+8.2f %+9.2f %+7.2f %+8.2f %+7.2f %+9.2f %s"
            % (
                s["index"],
                s["axis"],
                s["cmd_yaw"],
                s["cmd_pitch"],
                s["fb_yaw"],
                s["fb_pitch"],
                s["dyaw"],
                s["dpitch"],
                ",".join(flags) if flags else "-",
            )
        )
    print("原始记录: %s" % raw_path)
    print("保持段汇总: %s" % summary_path)
    if bag_path is not None:
        print("bag: %s" % bag_path)

    yaw_frozen = [s["index"] for s in summaries if s["yaw_frozen"]]
    pitch_frozen = [s["index"] for s in summaries if s["pitch_frozen"]]
    if yaw_frozen:
        print("yaw 在保持段浮点不变的点: %s" % yaw_frozen)
    else:
        print("各点 yaw 在保持段都有更新。")
    if pitch_frozen:
        print("pitch 在保持段浮点不变的点: %s" % pitch_frozen)
    else:
        print("各点 pitch 在保持段都有更新。")


if __name__ == "__main__":
    main()
