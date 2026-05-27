#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional

import rospy
from kuavo_msgs.msg import sensorsData
from kuavo_msgs.srv import changeArmCtrlMode, changeArmCtrlModeRequest
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64


RIGHT_ARM_JOINT_NAMES = [
    "r_arm_pitch",
    "r_arm_roll",
    "r_arm_yaw",
    "r_forearm_pitch",
    "r_hand_yaw",
    "r_hand_pitch",
    "r_hand_roll",
]

LEFT_ARM_JOINT_NAMES = [
    "l_arm_pitch",
    "l_arm_roll",
    "l_arm_yaw",
    "l_forearm_pitch",
    "l_hand_yaw",
    "l_hand_pitch",
    "l_hand_roll",
]


def _deg(rad: float) -> float:
    return float(rad) * 180.0 / math.pi


def _find_latest_teach_json(teach_dir: Path) -> Optional[Path]:
    patterns = ["teach_right_joint*.json", "teach_joints_*.json"]
    all_files: List[Path] = []
    for pat in patterns:
        all_files.extend(list(teach_dir.glob(pat)))
    files = sorted(set(all_files), key=lambda p: p.stat().st_mtime, reverse=True)
    return files[0] if files else None


def load_right_arm_points_from_teach_json(json_path: Path) -> List[Dict]:
    payload = json.loads(json_path.read_text(encoding="utf-8", errors="ignore"))
    samples = payload.get("samples", [])
    if not isinstance(samples, list) or not samples:
        raise ValueError(f"teach json samples 为空: {json_path}")

    points: List[Dict] = []
    for s in samples:
        if not isinstance(s, dict):
            continue
        if "right_arm_joints" not in s:
            continue
        arr = s["right_arm_joints"]
        if not isinstance(arr, list) or len(arr) == 0:
            continue
        idx = s.get("index", len(points))
        q_deg = [_deg(float(x)) for x in arr]
        points.append({"name": f"T{idx}", "right_deg": q_deg})

    if not points:
        raise ValueError(f"teach json 未找到 right_arm_joints: {json_path}")
    return points


class SensorsArmMonitor:
    """监听 /sensors_data_raw（joint_q 为弧度）。

    返回与 publish 的关节均为度，与 /kuavo_arm_traj 约定一致（控制器内部再转弧度）。
    """

    def __init__(self, topic: str = "/sensors_data_raw"):
        self._last_msg = None
        self._sub = rospy.Subscriber(topic, sensorsData, self._cb, queue_size=10)

    def _cb(self, msg: sensorsData):
        self._last_msg = msg

    def wait_for_msg(self, timeout: float) -> None:
        start = time.time()
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self._last_msg is not None:
                return
            if timeout > 0 and (time.time() - start) > timeout:
                raise TimeoutError("等待 sensorsData 超时")
            rate.sleep()

    def wait_until_valid_arms(self, timeout: float) -> None:
        """等到 joint_q 含左右臂 7+7，避免首帧未填全就以 0° 起步。"""
        start = time.time()
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self._last_msg is not None:
                q = list(self._last_msg.joint_data.joint_q)
                if len(q) >= 27:
                    return
            if timeout > 0 and (time.time() - start) > timeout:
                raise TimeoutError("等待有效 sensorsData（joint_q 长度>=27）超时")
            rate.sleep()

    def get_current_left_right_deg(self) -> Dict[str, List[float]]:
        if self._last_msg is None:
            raise RuntimeError("尚未收到 sensorsData 消息")
        q = list(self._last_msg.joint_data.joint_q)
        if len(q) < 27:
            raise RuntimeError("joint_q 长度不足，无法获取左右臂 7 关节")
        left = [_deg(float(x)) for x in q[13:20]]
        right = [_deg(float(x)) for x in q[20:27]]
        return {"left_deg": left, "right_deg": right}


def interp(a: float, b: float, ratio: float) -> float:
    r = max(0.0, min(1.0, ratio))
    return a + (b - a) * r


def publish_segment(
    pub: rospy.Publisher,
    left_hold_deg: List[float],
    start_right_deg: List[float],
    end_right_deg: List[float],
    duration: float,
    dt: float,
) -> None:
    duration = max(0.0, float(duration))
    if duration <= 1e-9:
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.name = LEFT_ARM_JOINT_NAMES + RIGHT_ARM_JOINT_NAMES
        js.position = [float(x) for x in left_hold_deg] + [float(x) for x in end_right_deg]
        js.velocity = [0.0] * 14
        js.effort = [0.0] * 14
        pub.publish(js)
        return

    t0 = rospy.Time.now().to_sec()
    rate = rospy.Rate(1.0 / dt if dt > 0 else 25.0)
    while not rospy.is_shutdown():
        now = rospy.Time.now().to_sec()
        elapsed = now - t0
        if elapsed >= duration:
            break
        ratio = elapsed / duration
        q = [interp(start_right_deg[i], end_right_deg[i], ratio) for i in range(7)]
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.name = LEFT_ARM_JOINT_NAMES + RIGHT_ARM_JOINT_NAMES
        js.position = [float(x) for x in left_hold_deg] + [float(x) for x in q]
        js.velocity = [0.0] * 14
        js.effort = [0.0] * 14
        pub.publish(js)
        rate.sleep()

    js = JointState()
    js.header.stamp = rospy.Time.now()
    js.name = LEFT_ARM_JOINT_NAMES + RIGHT_ARM_JOINT_NAMES
    js.position = [float(x) for x in left_hold_deg] + [float(x) for x in end_right_deg]
    js.velocity = [0.0] * 14
    js.effort = [0.0] * 14
    pub.publish(js)


def hold_and_capture(
    pub: rospy.Publisher,
    flag_pub: rospy.Publisher,
    point_name: str,
    keyframe_idx: int,
    left_hold_deg: List[float],
    right_deg: List[float],
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
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.name = LEFT_ARM_JOINT_NAMES + RIGHT_ARM_JOINT_NAMES
        js.position = [float(x) for x in left_hold_deg] + [float(x) for x in right_deg]
        js.velocity = [0.0] * 14
        js.effort = [0.0] * 14
        pub.publish(js)

        if (not captured) and elapsed >= capture_at_sec:
            m = Float64()
            m.data = float(keyframe_idx)
            flag_pub.publish(m)
            rospy.loginfo("触发采样: point=%s, kf=%d, t=%.3fs", point_name, keyframe_idx, elapsed)
            captured = True
        rate.sleep()
    if not captured:
        m = Float64()
        m.data = float(keyframe_idx)
        flag_pub.publish(m)
        rospy.logwarn("采样触发补发: point=%s, kf=%d, hold_sec=%.3f", point_name, keyframe_idx, hold_sec)


def hold_pose(
    pub: rospy.Publisher,
    left_deg: List[float],
    right_deg: List[float],
    hold_sec: float,
    dt: float,
) -> None:
    """切控制模式后先保持当前姿态，避免控制器空窗导致跳变。"""
    hold_sec = max(0.0, float(hold_sec))
    if hold_sec <= 1e-9:
        return
    t0 = rospy.Time.now().to_sec()
    rate = rospy.Rate(1.0 / dt if dt > 0 else 25.0)
    while not rospy.is_shutdown():
        now = rospy.Time.now().to_sec()
        if (now - t0) >= hold_sec:
            break
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.name = LEFT_ARM_JOINT_NAMES + RIGHT_ARM_JOINT_NAMES
        js.position = [float(x) for x in left_deg] + [float(x) for x in right_deg]
        js.velocity = [0.0] * 14
        js.effort = [0.0] * 14
        pub.publish(js)
        rate.sleep()


def main() -> None:
    rospy.init_node("right_wrist_table_publisher", argv=sys.argv, anonymous=False)

    sensor_topic = rospy.get_param("~sensor_topic", "/sensors_data_raw")
    arm_traj_topic = rospy.get_param("~arm_traj_topic", "/kuavo_arm_traj")
    keyframe_flag_topic = rospy.get_param("~keyframe_flag_topic", "/right_wrist_keyframe_flag")
    done_flag_topic = rospy.get_param("~done_flag_topic", "/right_wrist_keyframe_done")

    dt = float(rospy.get_param("~dt", 0.04))
    play_loop_count = int(rospy.get_param("~play_loop_count", 1))
    move_duration = float(rospy.get_param("~move_duration", 2.0))
    hold_sec = float(rospy.get_param("~hold_sec", 5.0))
    capture_at_sec = float(rospy.get_param("~capture_at_sec", 4.0))
    startup_align_sec = float(rospy.get_param("~startup_align_sec", 2.0))
    return_to_zero_sec = float(rospy.get_param("~return_to_zero_sec", startup_align_sec))
    wait_sensors_timeout = float(rospy.get_param("~wait_sensors_timeout", 30.0))
    require_valid_sensors = bool(rospy.get_param("~require_valid_sensors", True))
    set_external_control = bool(rospy.get_param("~set_external_control_mode", True))
    enable_wbc = bool(rospy.get_param("~enable_wbc_arm_trajectory_control", True))

    def _try_call(service_name: str, mode: int) -> bool:
        try:
            rospy.wait_for_service(service_name, timeout=2.0)
            proxy = rospy.ServiceProxy(service_name, changeArmCtrlMode)
            req = changeArmCtrlModeRequest()
            req.control_mode = int(mode)
            resp = proxy(req)
            if hasattr(resp, "result") and resp.result:
                rospy.loginfo("Service %s ok: mode=%s msg=%s", service_name, resp.mode, resp.message)
                return True
            rospy.logwarn("Service %s failed: mode=%s msg=%s", service_name, getattr(resp, "mode", None), getattr(resp, "message", ""))
            return False
        except rospy.ROSException:
            rospy.logwarn("Service not available: %s", service_name)
            return False
        except rospy.ServiceException as e:
            rospy.logwarn("Service call error %s: %s", service_name, e)
            return False

    # teach json
    script_dir = Path(__file__).resolve().parents[2]  # .../src/Camera_Calibration
    teach_dir_default = script_dir / "teach_capture_output"
    teach_output_dir = Path(rospy.get_param("~teach_output_dir", str(teach_dir_default)))
    teach_json_path_param = rospy.get_param("~teach_json_path", "").strip()

    if teach_json_path_param:
        teach_json_path = Path(teach_json_path_param)
    else:
        teach_json_path = teach_output_dir / "teach_right_joint.json"
    if not teach_json_path.is_file():
        raise RuntimeError(f"找不到 teach json: {teach_json_path}（或请设置 ~teach_json_path）")

    points = load_right_arm_points_from_teach_json(teach_json_path)
    rospy.loginfo("Loaded right arm points from teach json: %s (n=%d)", str(teach_json_path), len(points))

    # 必须先等到 /sensors_data_raw 有效再切控制模式：否则 WBC/外控切换时参考易阶跃到 0。
    monitor = SensorsArmMonitor(topic=sensor_topic)
    try:
        monitor.wait_until_valid_arms(timeout=wait_sensors_timeout)
        cur = monitor.get_current_left_right_deg()
        left_hold = cur["left_deg"]
        current_right = cur["right_deg"]
    except Exception as e:
        if require_valid_sensors:
            rospy.logfatal("未取得有效手臂传感器数据，拒绝以 0° 起步: %s", e)
            sys.exit(1)
        rospy.logwarn("读取当前关节失败，使用 0 度作为起点: %s", e)
        left_hold = [0.0] * 7
        current_right = [0.0] * 7

    if set_external_control:
        ok = False
        for srv in ("/arm_traj_change_mode", "/humanoid_change_arm_ctrl_mode", "/change_arm_ctrl_mode"):
            ok = _try_call(srv, 2) or ok
        if not ok:
            rospy.logwarn("未能切换到 external_control(2)，继续执行但可能无法响应 /kuavo_arm_traj")

    if enable_wbc:
        if not _try_call("/enable_wbc_arm_trajectory_control", 1):
            rospy.logwarn("未能使能 /enable_wbc_arm_trajectory_control，继续执行但可能无法响应 /kuavo_arm_traj")

    arm_pub = rospy.Publisher(arm_traj_topic, JointState, queue_size=10, tcp_nodelay=True)
    kf_flag_pub = rospy.Publisher(keyframe_flag_topic, Float64, queue_size=10)
    done_pub = rospy.Publisher(done_flag_topic, Bool, queue_size=1)
    rospy.sleep(float(rospy.get_param("~post_mode_pub_connect_sec", 0.2)))

    warmup_hold_sec = float(rospy.get_param("~warmup_hold_sec", 0.8))
    rospy.loginfo(
        "起步预热保持: %.2fs, current_right=%s, first_target=%s",
        warmup_hold_sec,
        [round(x, 2) for x in current_right],
        [round(x, 2) for x in points[0]["right_deg"]],
    )
    hold_pose(arm_pub, left_hold, current_right, warmup_hold_sec, dt)

    loops = max(1, play_loop_count)
    rospy.loginfo(
        "启动 right_wrist_table_publisher: loops=%d, move=%.2fs, hold=%.2fs, capture_at=%.2fs, points=%d",
        loops,
        move_duration,
        hold_sec,
        capture_at_sec,
        len(points),
    )

    for loop_idx in range(loops):
        if rospy.is_shutdown():
            break
        rospy.loginfo("开始第 %d/%d 轮", loop_idx + 1, loops)

        first = points[0]
        target_right = first["right_deg"]
        publish_segment(arm_pub, left_hold, current_right, target_right, startup_align_sec, dt)
        current_right = list(target_right)
        # 左右手首尾关键帧通常难以稳定识别棋盘：跳过第 1 帧与最后一帧的采样触发
        # 关键帧触发改为仅在后续循环中对 k=2..N-1 触发（并压缩编号为 1..N-2）

        for i in range(len(points) - 1):
            a = points[i]["right_deg"]
            b = points[i + 1]["right_deg"]
            publish_segment(arm_pub, left_hold, a, b, move_duration, dt)
            current_right = list(b)
            k = i + 2
            n_total = len(points)
            if n_total > 2 and (k != 1) and (k != n_total):
                hold_and_capture(arm_pub, kf_flag_pub, points[i + 1]["name"], k - 1, left_hold, current_right, hold_sec, capture_at_sec, dt)

    # 采样完成后，右臂从当前位姿平滑回零位。
    zero_right = [0.0] * 7
    publish_segment(arm_pub, left_hold, current_right, zero_right, return_to_zero_sec, dt)
    current_right = list(zero_right)
    rospy.loginfo("右臂已回零位")

    done_msg = Bool()
    done_msg.data = True
    done_pub.publish(done_msg)
    rospy.loginfo("右手轨迹下发完成，已发送 done 标志")


if __name__ == "__main__":
    main()

