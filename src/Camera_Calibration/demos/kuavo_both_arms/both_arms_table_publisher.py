#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import sys
import time
from pathlib import Path
from typing import Dict, List

import rospy
from kuavo_msgs.msg import sensorsData
from kuavo_msgs.srv import changeArmCtrlMode, changeArmCtrlModeRequest, changeLbQuickModeSrv, changeLbQuickModeSrvRequest
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64
from std_srvs.srv import SetBool, SetBoolRequest


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


def load_arm_points_from_teach_json(json_path: Path, key: str) -> List[Dict]:
    payload = json.loads(json_path.read_text(encoding="utf-8", errors="ignore"))
    samples = payload.get("samples", [])
    if not isinstance(samples, list) or not samples:
        raise ValueError(f"teach json samples 为空: {json_path}")

    points: List[Dict] = []
    for s in samples:
        if not isinstance(s, dict) or key not in s:
            continue
        arr = s[key]
        if not isinstance(arr, list) or len(arr) < 7:
            continue
        idx = s.get("index", len(points))
        q_deg = [_deg(float(x)) for x in arr[:7]]
        points.append({"name": f"T{idx}", "deg": q_deg})

    if not points:
        raise ValueError(f"teach json 未找到 {key}: {json_path}")
    return points


class SensorsArmMonitor:
    """sensorsData.joint_q 为弧度；本类输出/下发 JointState.position 为度（与 kuavo_arm_traj 约定一致）。"""

    def __init__(
        self,
        topic: str = "/sensors_data_raw",
        left_start_index: int = 13,
        right_start_index: int = 20,
    ):
        self._left_start_index = int(left_start_index)
        self._right_start_index = int(right_start_index)
        self._required_len = max(self._left_start_index + 7, self._right_start_index + 7)
        self._last_msg = None
        self._sub = rospy.Subscriber(topic, sensorsData, self._cb, queue_size=10)

    def _cb(self, msg: sensorsData):
        self._last_msg = msg

    def wait_until_valid_arms(self, timeout: float) -> None:
        start = time.time()
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self._last_msg is not None:
                q = list(self._last_msg.joint_data.joint_q)
                if len(q) >= self._required_len:
                    return
            if timeout > 0 and (time.time() - start) > timeout:
                raise TimeoutError(
                    f"等待有效 sensorsData（joint_q 长度>={self._required_len}）超时"
                )
            rate.sleep()

    def get_current_left_right_deg(self) -> Dict[str, List[float]]:
        if self._last_msg is None:
            raise RuntimeError("尚未收到 sensorsData 消息")
        q = list(self._last_msg.joint_data.joint_q)
        if len(q) < self._required_len:
            raise RuntimeError("joint_q 长度不足，无法获取左右臂 7 关节")
        li = self._left_start_index
        ri = self._right_start_index
        left = [_deg(float(x)) for x in q[li : li + 7]]
        right = [_deg(float(x)) for x in q[ri : ri + 7]]
        return {"left_deg": left, "right_deg": right}


def interp(a: float, b: float, ratio: float) -> float:
    r = max(0.0, min(1.0, ratio))
    return a + (b - a) * r


def _publish(pub: rospy.Publisher, left_deg: List[float], right_deg: List[float]) -> None:
    js = JointState()
    js.header.stamp = rospy.Time.now()
    js.name = LEFT_ARM_JOINT_NAMES + RIGHT_ARM_JOINT_NAMES
    js.position = [float(x) for x in left_deg] + [float(x) for x in right_deg]
    js.velocity = [0.0] * 14
    js.effort = [0.0] * 14
    pub.publish(js)


def publish_segment(
    pub: rospy.Publisher,
    start_left: List[float],
    start_right: List[float],
    end_left: List[float],
    end_right: List[float],
    duration: float,
    dt: float,
) -> None:
    duration = max(0.0, float(duration))
    if duration <= 1e-9:
        _publish(pub, end_left, end_right)
        return

    t0 = rospy.Time.now().to_sec()
    rate = rospy.Rate(1.0 / dt if dt > 0 else 25.0)
    while not rospy.is_shutdown():
        now = rospy.Time.now().to_sec()
        elapsed = now - t0
        if elapsed >= duration:
            break
        ratio = elapsed / duration
        left = [interp(start_left[i], end_left[i], ratio) for i in range(7)]
        right = [interp(start_right[i], end_right[i], ratio) for i in range(7)]
        _publish(pub, left, right)
        rate.sleep()
    _publish(pub, end_left, end_right)


def hold_pose(pub: rospy.Publisher, left: List[float], right: List[float], hold_sec: float, dt: float) -> None:
    hold_sec = max(0.0, float(hold_sec))
    if hold_sec <= 1e-9:
        return
    t0 = rospy.Time.now().to_sec()
    rate = rospy.Rate(1.0 / dt if dt > 0 else 25.0)
    while not rospy.is_shutdown():
        if (rospy.Time.now().to_sec() - t0) >= hold_sec:
            break
        _publish(pub, left, right)
        rate.sleep()


def hold_and_capture_both(
    pub: rospy.Publisher,
    left_flag_pub: rospy.Publisher,
    right_flag_pub: rospy.Publisher,
    point_name: str,
    keyframe_idx: int,
    left: List[float],
    right: List[float],
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
        elapsed = rospy.Time.now().to_sec() - t0
        if elapsed >= hold_sec:
            break
        _publish(pub, left, right)
        if (not captured) and elapsed >= capture_at_sec:
            m = Float64()
            m.data = float(keyframe_idx)
            left_flag_pub.publish(m)
            right_flag_pub.publish(m)
            rospy.loginfo("触发采样(左右臂): point=%s, kf=%d, t=%.3fs", point_name, keyframe_idx, elapsed)
            captured = True
        rate.sleep()
    if not captured:
        m = Float64()
        m.data = float(keyframe_idx)
        left_flag_pub.publish(m)
        right_flag_pub.publish(m)
        rospy.logwarn("采样触发补发(左右臂): point=%s, kf=%d, hold_sec=%.3f", point_name, keyframe_idx, hold_sec)


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


def _try_lb_quick_mode(service_name: str, quick_mode: int) -> bool:
    try:
        rospy.wait_for_service(service_name, timeout=2.0)
        proxy = rospy.ServiceProxy(service_name, changeLbQuickModeSrv)
        req = changeLbQuickModeSrvRequest()
        req.quickMode = int(quick_mode)
        resp = proxy(req)
        if hasattr(resp, "success") and resp.success:
            rospy.loginfo("Service %s ok: quickMode=%s msg=%s", service_name, quick_mode, resp.message)
            return True
        rospy.logwarn(
            "Service %s failed: quickMode=%s msg=%s",
            service_name,
            quick_mode,
            getattr(resp, "message", ""),
        )
        return False
    except rospy.ROSException:
        rospy.logwarn("Service not available: %s", service_name)
        return False
    except rospy.ServiceException as e:
        rospy.logwarn("Service call error %s: %s", service_name, e)
        return False


def _try_enable_arm_traj_interpolator(enabled: bool = True) -> bool:
    """轮臂 WBC：开启 /enable_arm_traj_interpolator，平滑 /kuavo_arm_traj。"""
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


def main() -> None:
    rospy.init_node("both_arms_table_publisher", argv=sys.argv, anonymous=False)

    sensor_topic = rospy.get_param("~sensor_topic", "/sensors_data_raw")
    arm_traj_topic = rospy.get_param("~arm_traj_topic", "/kuavo_arm_traj")
    left_keyframe_flag_topic = rospy.get_param("~left_keyframe_flag_topic", "/left_wrist_keyframe_flag")
    right_keyframe_flag_topic = rospy.get_param("~right_keyframe_flag_topic", "/right_wrist_keyframe_flag")
    left_done_flag_topic = rospy.get_param("~left_done_flag_topic", "/left_wrist_keyframe_done")
    right_done_flag_topic = rospy.get_param("~right_done_flag_topic", "/right_wrist_keyframe_done")

    # 默认 100Hz（0.01s）：轮臂 WBC ~500Hz，25Hz 阶梯指令易抖；可用 ~dt 覆盖
    dt = float(rospy.get_param("~dt", 0.01))
    play_loop_count = int(rospy.get_param("~play_loop_count", 1))
    move_duration = float(rospy.get_param("~move_duration", 2.0))
    # 与单臂 demo（left_wrist_table_publisher/right_wrist_table_publisher）保持一致：默认每个关键帧 hold 5s。
    # 若仅希望“连续运动不驻留”，可通过私参 ~hold_sec 覆盖为 0。
    hold_sec = float(rospy.get_param("~hold_sec", 5.0))
    startup_align_sec = float(rospy.get_param("~startup_align_sec", 2.0))
    return_to_zero_sec = float(rospy.get_param("~return_to_zero_sec", startup_align_sec))
    capture_at_sec = float(rospy.get_param("~capture_at_sec", 4.0))
    wait_sensors_timeout = float(rospy.get_param("~wait_sensors_timeout", 30.0))
    require_valid_sensors = bool(rospy.get_param("~require_valid_sensors", True))
    set_external_control = bool(rospy.get_param("~set_external_control_mode", True))
    enable_arm_quick_mode = bool(rospy.get_param("~enable_arm_quick_mode", True))
    if rospy.has_param("~enable_wbc_arm_trajectory_control"):
        enable_arm_quick_mode = bool(rospy.get_param("~enable_wbc_arm_trajectory_control"))
    robot_layout = str(rospy.get_param("~robot_layout", "biped52")).strip()
    is_wheel62 = robot_layout == "wheel62"
    # 轮臂标定默认开启 WBC 手臂轨迹插补，减轻 /kuavo_arm_traj 阶梯指令抖动
    use_arm_traj_interpolator = bool(rospy.get_param("~enable_arm_traj_interpolator", is_wheel62))
    left_start_index = int(rospy.get_param("~left_start_index", 4 if is_wheel62 else 13))
    right_start_index = int(rospy.get_param("~right_start_index", 11 if is_wheel62 else 20))
    # 模式切换期间保持发布当前关节，避免下游在切换瞬间回默认零位导致抖动
    pre_mode_hold_sec = float(rospy.get_param("~pre_mode_hold_sec", 0.6))
    post_mode_hold_sec = float(rospy.get_param("~post_mode_hold_sec", 0.6))

    script_dir = Path(__file__).resolve().parents[2]  # .../src/Camera_Calibration
    teach_dir_default = script_dir / "teach_capture_output"
    teach_output_dir = Path(rospy.get_param("~teach_output_dir", str(teach_dir_default)))
    teach_left = Path(rospy.get_param("~teach_left_json", str(teach_output_dir / "teach_left_joint.json")))
    teach_right = Path(rospy.get_param("~teach_right_json", str(teach_output_dir / "teach_right_joint.json")))

    if not teach_left.is_file():
        raise RuntimeError(f"找不到 teach left json: {teach_left}")
    if not teach_right.is_file():
        raise RuntimeError(f"找不到 teach right json: {teach_right}")

    left_points = load_arm_points_from_teach_json(teach_left, "left_arm_joints")
    right_points = load_arm_points_from_teach_json(teach_right, "right_arm_joints")
    rospy.loginfo("Loaded teach: left=%s (n=%d) right=%s (n=%d)", str(teach_left), len(left_points), str(teach_right), len(right_points))

    # 必须先等到 /sensors_data_raw 有效再切控制模式：否则 WBC/外控切换时参考易阶跃到 0。
    monitor = SensorsArmMonitor(
        topic=sensor_topic,
        left_start_index=left_start_index,
        right_start_index=right_start_index,
    )
    try:
        monitor.wait_until_valid_arms(timeout=wait_sensors_timeout)
        cur = monitor.get_current_left_right_deg()
        current_left = cur["left_deg"]
        current_right = cur["right_deg"]
    except Exception as e:
        if require_valid_sensors:
            rospy.logfatal("未取得有效手臂传感器数据，拒绝以 0° 起步: %s", e)
            sys.exit(1)
        rospy.logwarn("读取当前关节失败，使用 0 度作为起点: %s", e)
        current_left = [0.0] * 7
        current_right = [0.0] * 7

    # 先建 publisher 并发布当前姿态，减少“切模式瞬间参考跳到 0”的风险
    arm_pub = rospy.Publisher(arm_traj_topic, JointState, queue_size=10, tcp_nodelay=True)
    left_kf_flag_pub = rospy.Publisher(left_keyframe_flag_topic, Float64, queue_size=10)
    right_kf_flag_pub = rospy.Publisher(right_keyframe_flag_topic, Float64, queue_size=10)
    left_done_pub = rospy.Publisher(left_done_flag_topic, Bool, queue_size=1)
    right_done_pub = rospy.Publisher(right_done_flag_topic, Bool, queue_size=1)

    # 切模式前先保持一段当前姿态（此时未必已切到外控，但发布不会有副作用）
    if pre_mode_hold_sec > 0:
        hold_pose(arm_pub, current_left, current_right, pre_mode_hold_sec, dt)

    if set_external_control:
        ok = False
        if is_wheel62:
            service_candidates = (
                "/wheel_arm_change_arm_ctrl_mode",
                "/change_arm_ctrl_mode",
                "/arm_traj_change_mode",
                "/humanoid_change_arm_ctrl_mode",
            )
        else:
            service_candidates = (
                "/arm_traj_change_mode",
                "/humanoid_change_arm_ctrl_mode",
                "/change_arm_ctrl_mode",
            )
        for srv in service_candidates:
            ok = _try_call(srv, 2) or ok
        if not ok:
            rospy.logwarn("未能切换到 external_control(2)，继续执行但可能无法响应 /kuavo_arm_traj")

    if enable_arm_quick_mode:
        if is_wheel62:
            if not _try_lb_quick_mode("/enable_lb_arm_quick_mode", 2):
                rospy.logwarn("未能使能 /enable_lb_arm_quick_mode(quickMode=2)，继续执行但可能无法响应 /kuavo_arm_traj")
        else:
            if not _try_call("/enable_wbc_arm_trajectory_control", 1):
                rospy.logwarn("未能使能 /enable_wbc_arm_trajectory_control，继续执行但可能无法响应 /kuavo_arm_traj")

    if is_wheel62 and use_arm_traj_interpolator:
        if not _try_enable_arm_traj_interpolator(True):
            rospy.logwarn("未能开启 /enable_arm_traj_interpolator，手臂可能仍出现阶梯指令抖动")

    rospy.sleep(float(rospy.get_param("~post_mode_pub_connect_sec", 0.2)))

    # 切模式后再保持当前姿态一段，确保控制链路吃到“当前姿态作为参考”
    if post_mode_hold_sec > 0:
        hold_pose(arm_pub, current_left, current_right, post_mode_hold_sec, dt)

    warmup_hold_sec = float(rospy.get_param("~warmup_hold_sec", 0.8))
    hold_pose(arm_pub, current_left, current_right, warmup_hold_sec, dt)

    loops = max(1, play_loop_count)
    for loop_idx in range(loops):
        if rospy.is_shutdown():
            break
        rospy.loginfo("开始第 %d/%d 轮（左右臂合并下发）", loop_idx + 1, loops)

        # 对齐到各自第一帧
        first_left = left_points[0]["deg"]
        first_right = right_points[0]["deg"]
        publish_segment(arm_pub, current_left, current_right, first_left, first_right, startup_align_sec, dt)
        current_left, current_right = list(first_left), list(first_right)
        # 左右手首尾关键帧通常难以稳定识别棋盘：跳过第 1 帧与最后一帧的采样触发

        # 逐帧推进到 max_len，短序列使用最后一帧保持
        max_len = max(len(left_points), len(right_points))
        for i in range(max_len - 1):
            a_left = left_points[i]["deg"] if i < len(left_points) else left_points[-1]["deg"]
            b_left = left_points[i + 1]["deg"] if (i + 1) < len(left_points) else left_points[-1]["deg"]
            a_right = right_points[i]["deg"] if i < len(right_points) else right_points[-1]["deg"]
            b_right = right_points[i + 1]["deg"] if (i + 1) < len(right_points) else right_points[-1]["deg"]
            publish_segment(arm_pub, a_left, a_right, b_left, b_right, move_duration, dt)
            current_left, current_right = list(b_left), list(b_right)
            if hold_sec > 0:
                point_name = left_points[i + 1]["name"] if (i + 1) < len(left_points) else right_points[i + 1]["name"]
                k = i + 2  # 原始关键帧序号（1-based）
                n_total = max_len
                if n_total > 2 and (k != 1) and (k != n_total):
                    hold_and_capture_both(
                        arm_pub,
                        left_kf_flag_pub,
                        right_kf_flag_pub,
                        point_name,
                        k - 1,  # 压缩：2..N-1 -> 1..N-2
                        current_left,
                        current_right,
                        hold_sec,
                        capture_at_sec,
                        dt,
                    )

    zero_left = [0.0] * 7
    zero_right = [0.0] * 7
    publish_segment(arm_pub, current_left, current_right, zero_left, zero_right, return_to_zero_sec, dt)

    done_msg = Bool()
    done_msg.data = True
    left_done_pub.publish(done_msg)
    right_done_pub.publish(done_msg)
    rospy.loginfo("左右臂合并轨迹下发完成，已发送 done 标志")


if __name__ == "__main__":
    main()

