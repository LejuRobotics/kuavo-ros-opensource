#!/usr/bin/env python3

import math
import json
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rospy
from kuavo_msgs.msg import robotHeadMotionData, sensorsData
from std_msgs.msg import Bool, Float64
from std_srvs.srv import SetBool, SetBoolRequest


def _deg(rad: float) -> float:
    return float(rad) * 180.0 / math.pi


def _find_latest_teach_json(teach_dir: Path) -> Optional[Path]:
    patterns = ["teach_head_joint*.json", "teach_joints_*.json"]
    all_files = []
    for pat in patterns:
        all_files.extend(list(teach_dir.glob(pat)))
    files = sorted(set(all_files), key=lambda p: p.stat().st_mtime, reverse=True)
    return files[0] if files else None


def load_head_points_from_teach_json(json_path: Path) -> List[Dict]:
    """
    读取 teach_joint_capture 输出 JSON：
      payload.samples[*].head_yaw/head_pitch (rad)
    转换为 head_table_publisher 使用的 points 列表（deg）。
    """
    payload = json.loads(json_path.read_text(encoding="utf-8", errors="ignore"))
    samples = payload.get("samples", [])
    if not isinstance(samples, list) or not samples:
        raise ValueError(f"teach json samples 为空: {json_path}")

    points: List[Dict] = []
    for s in samples:
        if not isinstance(s, dict):
            continue
        if "head_yaw" not in s or "head_pitch" not in s:
            continue
        idx = s.get("index", len(points))
        yaw_deg = _deg(float(s["head_yaw"]))
        pitch_deg = _deg(float(s["head_pitch"]))
        points.append({"name": f"T{idx}", "yaw_deg": yaw_deg, "pitch_deg": pitch_deg})

    if not points:
        raise ValueError(f"teach json 未找到 head_yaw/head_pitch: {json_path}")
    return points


class HeadJointStateMonitor:
    """监听 /sensors_data_raw：joint_q 末尾两维为头部 yaw/pitch（弧度）。

    get_current_yaw_pitch_deg() 与 /robot_head_motion_data 的 joint_data 均为度（与 headCmdCallback 中 *M_PI/180 一致）。
    """

    def __init__(self, topic: str = "/sensors_data_raw") -> None:
        self._topic = topic
        self._last_msg = None
        self._sub = rospy.Subscriber(topic, sensorsData, self._cb, queue_size=10)

    def _cb(self, msg: sensorsData) -> None:
        self._last_msg = msg

    def wait_for_msg(self, timeout: float) -> None:
        start = time.time()
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self._last_msg is not None:
                return
            if timeout > 0 and (time.time() - start) > timeout:
                raise TimeoutError(f"等待 JointState 超时（topic={self._topic}）")
            rate.sleep()

    def wait_until_valid_head(self, timeout: float) -> None:
        """等到 joint_q 至少含头两轴，避免首帧未填全就以 0° 起步。"""
        start = time.time()
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self._last_msg is not None:
                q = list(self._last_msg.joint_data.joint_q)
                if len(q) >= 2:
                    return
            if timeout > 0 and (time.time() - start) > timeout:
                raise TimeoutError(f"等待有效 sensorsData（joint_q 长度>=2）超时（topic={self._topic}）")
            rate.sleep()

    def get_current_yaw_pitch_deg(self) -> Dict[str, float]:
        if self._last_msg is None:
            raise RuntimeError("尚未收到任何 sensorsData 消息")
        q = list(self._last_msg.joint_data.joint_q)
        if len(q) < 2:
            raise RuntimeError("sensorsData.joint_q 长度不足，无法获取头部关节")
        return {
            "yaw_deg": q[-2] * 180.0 / math.pi,
            "pitch_deg": q[-1] * 180.0 / math.pi,
        }


def clamp_head_angles(yaw_deg: float, pitch_deg: float) -> Dict[str, float]:
    yaw = max(-30.0, min(30.0, float(yaw_deg)))
    pitch = max(-25.0, min(25.0, float(pitch_deg)))
    return {"yaw_deg": yaw, "pitch_deg": pitch}


def build_head_msg(yaw_deg: float, pitch_deg: float) -> robotHeadMotionData:
    q = clamp_head_angles(yaw_deg, pitch_deg)
    msg = robotHeadMotionData()
    msg.joint_data = [q["yaw_deg"], q["pitch_deg"]]
    return msg


def interp(a: float, b: float, ratio: float) -> float:
    r = max(0.0, min(1.0, ratio))
    return a + (b - a) * r


def publish_segment(
    head_pub: rospy.Publisher,
    start: Dict[str, float],
    end: Dict[str, float],
    duration: float,
    dt: float,
) -> None:
    duration = max(0.0, float(duration))
    if duration <= 1e-9:
        head_pub.publish(build_head_msg(end["yaw_deg"], end["pitch_deg"]))
        return
    t0 = rospy.Time.now().to_sec()
    rate = rospy.Rate(1.0 / dt if dt > 0 else 25.0)
    while not rospy.is_shutdown():
        now = rospy.Time.now().to_sec()
        elapsed = now - t0
        if elapsed >= duration:
            break
        ratio = elapsed / duration
        yaw = interp(start["yaw_deg"], end["yaw_deg"], ratio)
        pitch = interp(start["pitch_deg"], end["pitch_deg"], ratio)
        head_pub.publish(build_head_msg(yaw, pitch))
        rate.sleep()
    head_pub.publish(build_head_msg(end["yaw_deg"], end["pitch_deg"]))


def hold_and_capture(
    head_pub: rospy.Publisher,
    flag_pub: rospy.Publisher,
    point_name: str,
    keyframe_idx: int,
    target: Dict[str, float],
    hold_sec: float,
    capture_at_sec: float,
    dt: float,
) -> None:
    hold_sec = max(0.0, float(hold_sec))
    capture_at_sec = max(0.0, min(float(capture_at_sec), hold_sec))
    t0 = rospy.Time.now().to_sec()
    captured = False
    rate = rospy.Rate(1.0 / dt if dt > 0 else 25.0)
    while not rospy.is_shutdown():
        now = rospy.Time.now().to_sec()
        elapsed = now - t0
        if elapsed >= hold_sec:
            break
        head_pub.publish(build_head_msg(target["yaw_deg"], target["pitch_deg"]))
        if (not captured) and elapsed >= capture_at_sec:
            m = Float64()
            # 关键帧编号（1-based），用于下游按关键帧对齐写入 CSV / 绘图
            m.data = float(keyframe_idx)
            flag_pub.publish(m)
            rospy.loginfo(
                "触发采样: point=%s, kf=%d, t=%.3fs, yaw=%.3f, pitch=%.3f",
                point_name,
                keyframe_idx,
                elapsed,
                target["yaw_deg"],
                target["pitch_deg"],
            )
            captured = True
        rate.sleep()
    head_pub.publish(build_head_msg(target["yaw_deg"], target["pitch_deg"]))
    if not captured:
        m = Float64()
        m.data = float(keyframe_idx)
        flag_pub.publish(m)
        rospy.logwarn("采样触发补发: point=%s, kf=%d, hold_sec=%.3f", point_name, keyframe_idx, hold_sec)


def hold_pose(
    head_pub: rospy.Publisher,
    target: Dict[str, float],
    hold_sec: float,
    dt: float,
) -> None:
    """轨迹启动前保持当前头部姿态，减少模式切换后首段跳变。"""
    hold_sec = max(0.0, float(hold_sec))
    if hold_sec <= 1e-9:
        return
    t0 = rospy.Time.now().to_sec()
    rate = rospy.Rate(1.0 / dt if dt > 0 else 25.0)
    while not rospy.is_shutdown():
        now = rospy.Time.now().to_sec()
        if (now - t0) >= hold_sec:
            break
        head_pub.publish(build_head_msg(target["yaw_deg"], target["pitch_deg"]))
        rate.sleep()


def _try_enable_arm_traj_interpolator(enabled: bool = True) -> bool:
    service_name = "/enable_arm_traj_interpolator"
    try:
        rospy.wait_for_service(service_name, timeout=2.0)
        proxy = rospy.ServiceProxy(service_name, SetBool)
        req = SetBoolRequest()
        req.data = bool(enabled)
        resp = proxy(req)
        if resp.success:
            rospy.loginfo("Service %s ok: enabled=%s msg=%s", service_name, enabled, resp.message)
            return True
        rospy.logwarn("Service %s failed: enabled=%s msg=%s", service_name, enabled, resp.message)
        return False
    except rospy.ROSException:
        rospy.logwarn("Service not available: %s", service_name)
        return False
    except rospy.ServiceException as e:
        rospy.logwarn("Service call error %s: %s", service_name, e)
        return False


def validate_points(points: List[Dict]) -> None:
    if not points:
        raise ValueError("points 不能为空")
    for i, p in enumerate(points):
        for k in ("name", "yaw_deg", "pitch_deg"):
            if k not in p:
                raise ValueError(f"points[{i}] 缺少字段 {k}: {p}")
        _ = float(p["yaw_deg"])
        _ = float(p["pitch_deg"])


def main() -> None:
    rospy.init_node("head_table_publisher", argv=sys.argv, anonymous=False)

    # 话题与时序参数（支持 rosparam 覆盖）
    joint_state_topic = rospy.get_param("~joint_state_topic", "/sensors_data_raw")
    head_traj_topic = rospy.get_param("~head_traj_topic", "/robot_head_motion_data")
    keyframe_flag_topic = rospy.get_param("~keyframe_flag_topic", "/head_keyframe_flag")
    done_flag_topic = rospy.get_param("~done_flag_topic", "/head_keyframe_done")

    dt = float(rospy.get_param("~dt", 0.04))
    play_loop_count = int(rospy.get_param("~play_loop_count", 1))
    move_duration = float(rospy.get_param("~move_duration", 2.0))
    hold_sec = float(rospy.get_param("~hold_sec", 5.0))
    capture_at_sec = float(rospy.get_param("~capture_at_sec", 4.0))
    startup_align_sec = float(rospy.get_param("~startup_align_sec", 2.0))
    return_to_zero_sec = float(rospy.get_param("~return_to_zero_sec", startup_align_sec))
    wait_sensors_timeout = float(rospy.get_param("~wait_sensors_timeout", 30.0))
    require_valid_sensors = bool(rospy.get_param("~require_valid_sensors", True))
    robot_layout = str(rospy.get_param("~robot_layout", "biped52")).strip()
    is_wheel62 = robot_layout == "wheel62"
    use_arm_traj_interpolator = bool(rospy.get_param("~enable_arm_traj_interpolator", is_wheel62))

    # 从 teach_capture_output 的 JSON 读取头部角度表（拖动示教采集的结果）
    # - 优先使用 ~teach_json_path（测试阶段由脚本传入 *_test.json）
    # - 否则固定使用 teach_head_joint.json（不再“自动找最新”）
    script_dir = Path(__file__).resolve().parents[2]  # .../src/Camera_Calibration
    teach_dir_default = script_dir / "teach_capture_output"
    teach_output_dir = Path(rospy.get_param("~teach_output_dir", str(teach_dir_default)))
    teach_json_path_param = rospy.get_param("~teach_json_path", "").strip()
    if teach_json_path_param:
        teach_json_path = Path(teach_json_path_param)
    else:
        teach_json_path = teach_output_dir / "teach_head_joint.json"

    if not teach_json_path.is_file():
        raise RuntimeError(
            f"找不到 teach json: {teach_json_path}。"
            f" 请先运行 teach_joint_capture.py 生成，或设置 ~teach_json_path。"
            f" (teach_output_dir={teach_output_dir})"
        )

    points = load_head_points_from_teach_json(teach_json_path)
    validate_points(points)
    rospy.loginfo("Loaded head points from teach json: %s (n=%d)", str(teach_json_path), len(points))

    # 先等到 /sensors_data_raw 有效再发布头部轨迹，避免下游在未对齐前将参考拉到 0。
    monitor = HeadJointStateMonitor(topic=joint_state_topic)
    try:
        monitor.wait_until_valid_head(timeout=wait_sensors_timeout)
        current = monitor.get_current_yaw_pitch_deg()
    except Exception as e:
        if require_valid_sensors:
            rospy.logfatal("未取得有效头部传感器数据，拒绝以 0° 起步: %s", e)
            sys.exit(1)
        rospy.logwarn("读取当前关节失败，使用 0 度作为起点: %s", e)
        current = {"yaw_deg": 0.0, "pitch_deg": 0.0}

    head_pub = rospy.Publisher(head_traj_topic, robotHeadMotionData, queue_size=10)
    kf_flag_pub = rospy.Publisher(keyframe_flag_topic, Float64, queue_size=10)
    done_pub = rospy.Publisher(done_flag_topic, Bool, queue_size=1)
    rospy.sleep(float(rospy.get_param("~post_mode_pub_connect_sec", 0.2)))

    if is_wheel62 and use_arm_traj_interpolator:
        if not _try_enable_arm_traj_interpolator(True):
            rospy.logwarn("未能开启 /enable_arm_traj_interpolator，手臂可能仍出现阶梯指令抖动")

    # 启动预热保持，减少起步瞬间回默认位/零位的风险
    warmup_hold_sec = float(rospy.get_param("~warmup_hold_sec", 0.8))
    rospy.loginfo(
        "起步预热保持: %.2fs, current_head=[yaw=%.2f,pitch=%.2f], first_target=[yaw=%.2f,pitch=%.2f]",
        warmup_hold_sec,
        current["yaw_deg"],
        current["pitch_deg"],
        points[0]["yaw_deg"],
        points[0]["pitch_deg"],
    )
    hold_pose(head_pub, current, warmup_hold_sec, dt)

    rospy.loginfo(
        "启动 head_table_publisher: loops=%d, move=%.2fs, hold=%.2fs, capture_at=%.2fs, points=%d",
        max(1, play_loop_count),
        move_duration,
        hold_sec,
        capture_at_sec,
        len(points),
    )

    loops = max(1, play_loop_count)
    for loop_idx in range(loops):
        if rospy.is_shutdown():
            break
        rospy.loginfo("开始第 %d/%d 轮", loop_idx + 1, loops)

        # 每轮开始先平滑对齐到第一个点，避免大跳变。
        first = points[0]
        publish_segment(head_pub, current, first, startup_align_sec, dt)
        current = {"yaw_deg": first["yaw_deg"], "pitch_deg": first["pitch_deg"]}
        hold_and_capture(head_pub, kf_flag_pub, first["name"], 1, first, hold_sec, capture_at_sec, dt)

        for i in range(len(points) - 1):
            a = points[i]
            b = points[i + 1]
            publish_segment(head_pub, a, b, move_duration, dt)
            current = {"yaw_deg": b["yaw_deg"], "pitch_deg": b["pitch_deg"]}
            hold_and_capture(head_pub, kf_flag_pub, b["name"], i + 2, b, hold_sec, capture_at_sec, dt)

    # 采样完成后，头部从当前位姿平滑回零位。
    zero_head = {"yaw_deg": 0.0, "pitch_deg": 0.0}
    publish_segment(head_pub, current, zero_head, return_to_zero_sec, dt)
    current = dict(zero_head)
    rospy.loginfo("头部已回零位")

    done_msg = Bool()
    done_msg.data = True
    done_pub.publish(done_msg)
    rospy.loginfo("头部轨迹下发完成，已发送 done 标志")


if __name__ == "__main__":
    main()

