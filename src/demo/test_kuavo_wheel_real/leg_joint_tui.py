#!/usr/bin/env python3
"""下肢 4 关节角度控制 TUI

通过 Textual 界面调整 joint1~joint4 角度并发布到 /lb_leg_traj
发布时用五次多项式 rest-to-rest 轨迹（起止速度/加速度为 0），
按速度与加速度上限规划时间，从当前实际关节角过渡到目标角度。
"""

from __future__ import annotations

import math
import threading
import time

import rospy
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from textual.app import App, ComposeResult
from textual.containers import Horizontal
from textual.timer import Timer
from textual.widgets import Button, Footer, Header, Input, Label

from kuavo_msgs.msg import sensorsData

JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4"]
"""关节名称"""
JOINT_MIN = [0.0, -60.0, 0.0, 0.0]
"""关节最小角度"""
JOINT_MAX = [60.0, 0.0, 0.0, 0.0]
"""关节最大角度"""

INTERP_RATE = 50.0
"""插值频率"""
INTERP_MAX_VEL_DEG_S = 12.0
"""关节最大速度 (deg/s)，大角度行程靠拉长时长而不是提高起步速度"""
INTERP_MAX_ACC_DEG_S2 = 15.0
"""关节最大加速度 (deg/s^2)"""
# 五次多项式 s=10τ³-15τ⁴+6τ⁵ 的 |ds/dτ|、|d²s/dτ²| 峰值
_QUINTIC_VEL_COEFF = 15.0 / 8.0
_QUINTIC_ACC_COEFF = 10.0 * math.sqrt(3.0) / 3.0
CURRENT_REFRESH_RATE = 5.0
"""当前关节角刷新频率"""


def _quintic_s(tau: float) -> float:
    """Rest-to-rest 五次多项式位置比例, tau in [0, 1]."""
    tau = max(0.0, min(1.0, tau))
    tau2 = tau * tau
    tau3 = tau2 * tau
    return tau3 * (10.0 + tau * (-15.0 + 6.0 * tau))


def _plan_interp_duration(start: list[float], target: list[float]) -> float:
    """按 vmax/amax 规划四关节同步到达时长。"""
    duration = 1.0 / INTERP_RATE
    for q0, qf in zip(start, target):
        delta = abs(qf - q0)
        if delta < 1e-9:
            continue
        t_vel = _QUINTIC_VEL_COEFF * delta / INTERP_MAX_VEL_DEG_S
        t_acc = math.sqrt(_QUINTIC_ACC_COEFF * delta / INTERP_MAX_ACC_DEG_S2)
        duration = max(duration, t_vel, t_acc)
    return duration


class LegJointTui(App):
    """下肢关节角度控制应用"""

    _pub: rospy.Publisher | None = None
    """话题发布器"""
    _lock_srv: rospy.ServiceProxy | None = None
    """knee/leg 锁服务"""
    _current_angles: list[float] | None = None
    """当前关节角缓存 (度)"""
    _interp_timer: Timer | None = None
    """插值 Timer"""
    _interp_start: list[float] | None = None
    """插值起点角度 (度)"""
    _interp_target: list[float] | None = None
    """插值目标角度 (度)"""
    _interp_current: list[float] | None = None
    """插值当前角度 (度)"""
    _interp_t0: float = 0.0
    """插值开始单调时钟"""
    _interp_duration: float = 0.0
    """插值总时长 (s)"""

    TITLE = "下肢关节控制"
    CSS = """
    .joint-row { height: 3; }
    .joint-name { width: 10; content-align: right middle; padding-right: 1; }
    .joint-input { width: 12; }
    .joint-range { width: 16; color: $text-muted; padding-left: 1; }
    .joint-current { width: 16; color: $text-success; padding-left: 1; }
    #status { color: $text-muted; }
    #buttons { height: 3; }
    """

    def compose(self) -> ComposeResult:
        yield Header()
        for name, lo, hi in zip(JOINT_NAMES, JOINT_MIN, JOINT_MAX):
            with Horizontal(classes="joint-row"):
                yield Label(name, classes="joint-name")
                yield Input(value="0.0", id=f"in_{name}", classes="joint-input")
                yield Label(f"{lo:g} ~ {hi:g}", classes="joint-range")
                yield Label("当前: --", id=f"cur_{name}", classes="joint-current")
        yield Label("就绪", id="status")
        with Horizontal(id="buttons"):
            yield Button("发布", id="publish", variant="primary")
            yield Button("归零", id="zero")
        yield Footer()

    def on_mount(self) -> None:
        """初始化 ROS 节点, 发布器, 传感器订阅与 knee/leg 锁服务"""
        try:
            rospy.init_node("leg_joint_tui", anonymous=True)
            self._pub = rospy.Publisher(
                "/lb_leg_traj", JointState, queue_size=1, latch=True
            )
            self._lock_srv = rospy.ServiceProxy("/quest3/set_lock_knee_leg", SetBool)
            self._sensor_sub = rospy.Subscriber(
                "/sensors_data_raw", sensorsData, self._sensor_callback, queue_size=1
            )
            threading.Thread(target=rospy.spin, daemon=True).start()
            self.set_interval(1.0 / CURRENT_REFRESH_RATE, self._refresh_current_display)
            self._set_status("ROS 就绪, 等待发布")
        except rospy.exceptions.ROSException as exc:
            self._set_status(f"ROS 初始化失败: {exc}")

    def _set_status(self, text: str) -> None:
        self.query_one("#status", Label).update(text)

    def _sensor_callback(self, msg: sensorsData) -> None:
        """缓存传感器当前关节角 (弧度转角度), 供显示与插值起点使用

        Args:
            msg: 传感器数据消息
        """
        if len(msg.joint_data.joint_q) < 4:
            return
        angles = [math.degrees(q) for q in msg.joint_data.joint_q[:4]]
        self._current_angles = angles

    def _refresh_current_display(self) -> None:
        """刷新界面上的当前关节角显示"""
        angles = self._current_angles
        if angles is None:
            return
        for name, angle in zip(JOINT_NAMES, angles):
            self.query_one(f"#cur_{name}", Label).update(f"当前: {angle:.1f}°")

    def _read_value(self, name: str):
        """解析关节输入值, 非法返回 None

        Args:
            name: 关节名
        """
        raw = self.query_one(f"#in_{name}", Input).value.strip()
        try:
            return float(raw)
        except ValueError:
            return None

    def _collect_values(self):
        """读取并 clamp 4 个关节值, 非法返回 None"""
        values = []
        for name, lo, hi in zip(JOINT_NAMES, JOINT_MIN, JOINT_MAX):
            value = self._read_value(name)
            if value is None:
                return None
            clamped = max(lo, min(hi, value))
            if clamped != value:
                self.query_one(f"#in_{name}", Input).value = f"{clamped:g}"
            values.append(clamped)
        return values

    def _set_lock_knee_leg(self, lock: bool) -> None:
        """解锁或锁定 knee/leg 关节

        Args:
            lock: True 锁定, False 解锁
        """
        if self._lock_srv is None:
            return
        try:
            self._lock_srv(lock)
        except (rospy.ServiceException, rospy.exceptions.ROSException) as exc:
            rospy.logwarn(f"set_lock_knee_leg({lock}) 调用失败: {exc}")

    def _publish(self) -> None:
        """发布目标关节角度到 /lb_leg_traj, 用限速五次多项式平滑过渡

        发布前解锁 knee/leg, 插值完成后重新锁定。中途再次发布时从当前插值点重规划。
        """
        if self._pub is None:
            self._set_status("ROS 未就绪, 无法发布")
            return

        start = self._interp_current if self._interp_timer is not None else self._current_angles
        if start is None:
            self._set_status("当前关节角未知, 等待传感器数据")
            return

        values = self._collect_values()
        if values is None:
            self._set_status("输入非法, 请检查各关节数值")
            return

        self._stop_interp()
        self._set_lock_knee_leg(False)
        self._interp_start = list(start)
        self._interp_target = values
        self._interp_current = list(start)
        self._interp_t0 = time.monotonic()
        self._interp_duration = _plan_interp_duration(self._interp_start, values)
        self._interp_timer = self.set_interval(1.0 / INTERP_RATE, self._interp_step)
        self._set_status(f"插值中... 预计 {self._interp_duration:.1f}s")

    def _interp_step(self) -> None:
        """按规划时长走五次多项式一步并发布, 到达后停止并锁定 knee/leg"""
        start = self._interp_start
        target = self._interp_target
        if start is None or target is None:
            self._stop_interp()
            return

        elapsed = time.monotonic() - self._interp_t0
        duration = max(self._interp_duration, 1.0 / INTERP_RATE)
        tau = elapsed / duration
        if tau >= 1.0:
            self._interp_current = list(target)
            self._publish_values(target)
            self._stop_interp()
            self._set_lock_knee_leg(True)
            self._set_status(f"已发布: {[f'{v:g}' for v in target]}")
            return

        scale = _quintic_s(tau)
        current = [q0 + scale * (qf - q0) for q0, qf in zip(start, target)]
        self._interp_current = current
        self._publish_values(current)
        remaining = duration - elapsed
        self._set_status(f"插值中... 剩余 {remaining:.1f}s")

    def _publish_values(self, values) -> None:
        """发布指定关节角度到 /lb_leg_traj

        Args:
            values: 4 个关节角度 (度)
        """
        if self._pub is None:
            return
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = list(JOINT_NAMES)
        msg.position = values
        msg.velocity = [0.0] * len(JOINT_NAMES)
        self._pub.publish(msg)

    def _stop_interp(self) -> None:
        """停止插值 Timer"""
        if self._interp_timer is not None:
            self._interp_timer.stop()
            self._interp_timer = None

    def on_button_pressed(self, event: Button.Pressed) -> None:
        if event.button.id == "publish":
            self._publish()
        elif event.button.id == "zero":
            for name in JOINT_NAMES:
                self.query_one(f"#in_{name}", Input).value = "0.0"
            self._publish()


def main() -> None:
    LegJointTui().run()


if __name__ == "__main__":
    main()
