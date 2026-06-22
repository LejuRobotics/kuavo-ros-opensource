"""5W 平台单手摇杆控制躯干（折叠臂升降 + 前后倾 + 腰部旋转）"""
from __future__ import annotations
import math
import time
from typing import Callable, Optional


# ===== 阈值 =====
GRIP_PRESS_TH = 0.7
GRIP_RELEASE_TH = 0.3
STICK_DEAD_ZONE = 0.1

# ===== 缩放系数（增量速率） =====
SCALE_HEIGHT = 0.10   # m/s
SCALE_PITCH = 0.30    # rad/s

# ===== 限幅 =====
Z_MIN, Z_MAX = -0.05, 0.40           # m，相对初始位姿
PITCH_MIN, PITCH_MAX = -0.524, 0.524 # rad（±30°）
YAW_TARGET = math.pi                 # rad（180°）

# ===== 时序 =====
MIN_DT = 0.001
MAX_DT = 0.1


class TorsoController:
    """单手摇杆控躯干的状态机 + 命令发布器。

    依赖注入：publisher、clock 由调用方传入，便于单元测试。
    """

    def __init__(
        self,
        initial_pose_xyz,
        publisher: Optional[Callable] = None,
        clock: Optional[Callable[[], float]] = None,
    ):
        self.initial_xyz = tuple(initial_pose_xyz)
        # 索引 [0:3] = x/y/z（取自 initial_pose_xyz）
        # 索引 [3:6] = roll/pitch/yaw（机械零位为基准）
        self.current_pose = list(initial_pose_xyz) + [0.0, 0.0, 0.0]
        self._publish_fn = publisher
        self._clock = clock if clock is not None else time.monotonic
        self.main_hand: Optional[str] = None  # None / 'left' / 'right'
        self._l_was_pressed = False
        self._r_was_pressed = False
        self._last_t: Optional[float] = None
        # A/B 按键边沿检测的"上一帧按下"状态
        self._first_btn_was_pressed = False
        self._second_btn_was_pressed = False

    def handle_joystick(self, msg) -> None:
        self._arbitrate_main_hand(msg)
        if self.main_hand is None:
            return
        dt = self._compute_dt()
        self._handle_yaw_buttons(msg)
        self._apply_stick_increment(msg, dt)
        self._publish_torso_pose()

    def _compute_dt(self) -> float:
        now = self._clock()
        if self._last_t is None:
            self._last_t = now
            return MIN_DT
        dt = now - self._last_t
        self._last_t = now
        return max(MIN_DT, min(dt, MAX_DT))

    def _select_stick(self, msg):
        if self.main_hand == "left":
            return msg.left_x, msg.left_y
        return msg.right_x, msg.right_y

    def _apply_stick_increment(self, msg, dt: float) -> None:
        sx, sy = self._select_stick(msg)
        sx = sx if abs(sx) > STICK_DEAD_ZONE else 0.0
        sy = sy if abs(sy) > STICK_DEAD_ZONE else 0.0

        # 升高：摇杆 Y > 0 → z 增
        self.current_pose[2] += sy * SCALE_HEIGHT * dt
        # 前倾：摇杆 X > 0 → pitch 减（spec 6.1）
        self.current_pose[4] -= sx * SCALE_PITCH * dt

        # 限幅
        z_lo = self.initial_xyz[2] + Z_MIN
        z_hi = self.initial_xyz[2] + Z_MAX
        self.current_pose[2] = max(z_lo, min(self.current_pose[2], z_hi))
        self.current_pose[4] = max(PITCH_MIN, min(self.current_pose[4], PITCH_MAX))

    def _select_buttons(self, msg):
        if self.main_hand == "left":
            return msg.left_first_button_pressed, msg.left_second_button_pressed
        return msg.right_first_button_pressed, msg.right_second_button_pressed

    def _handle_yaw_buttons(self, msg) -> None:
        first, second = self._select_buttons(msg)
        # 上升沿：基于当前 yaw 增量 ±YAW_TARGET（互逆，可复位）
        if first and not self._first_btn_was_pressed:
            self.current_pose[5] -= YAW_TARGET
        elif second and not self._second_btn_was_pressed:
            self.current_pose[5] += YAW_TARGET
        self._first_btn_was_pressed = first
        self._second_btn_was_pressed = second
        # wrap yaw 到 [-π, π]，避免连续按按键累加超出 controller 限位
        self.current_pose[5] = math.atan2(
            math.sin(self.current_pose[5]), math.cos(self.current_pose[5])
        )

    def _publish_torso_pose(self) -> None:
        if self._publish_fn is None:
            return
        # 延迟导入，避免在测试环境强制依赖 ROS
        from geometry_msgs.msg import Twist
        twist = Twist()
        twist.linear.x = self.current_pose[0]
        twist.linear.y = self.current_pose[1]
        twist.linear.z = self.current_pose[2]
        twist.angular.x = self.current_pose[3]
        twist.angular.y = self.current_pose[4]
        twist.angular.z = self.current_pose[5]
        self._publish_fn(twist)

    def _arbitrate_main_hand(self, msg) -> None:
        l_pressed = msg.left_grip > GRIP_PRESS_TH
        r_pressed = msg.right_grip > GRIP_PRESS_TH
        l_released = msg.left_grip < GRIP_RELEASE_TH
        r_released = msg.right_grip < GRIP_RELEASE_TH

        if self.main_hand is None:
            # 边沿检测：上一帧没按下、这一帧按下 → 锁定
            if l_pressed and not self._l_was_pressed:
                self.main_hand = "left"
            elif r_pressed and not self._r_was_pressed:
                self.main_hand = "right"
        else:
            # 已锁定，仅主指手 grip 松开（< release_th）才释放
            if self.main_hand == "left" and l_released:
                self.main_hand = None
                self._last_t = None  # 重置时钟基准，下次重新激活首帧返回 MIN_DT
            elif self.main_hand == "right" and r_released:
                self.main_hand = None
                self._last_t = None

        # 维护"上一帧按下状态"用于边沿检测
        self._l_was_pressed = l_pressed
        self._r_was_pressed = r_pressed


def create_torso_controller(init_timeout_sec: float = 5.0) -> "TorsoController":
    """在真实 ROS 环境里装配 TorsoController。失败时调用 rospy.signal_shutdown。"""
    import rospy
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Float32

    try:
        # 优先用包内副本（tools/lb_ctrl_api.py），fallback 到全局 PYTHONPATH
        try:
            from . import lb_ctrl_api as ct
        except (ImportError, ValueError):
            import lb_ctrl_api as ct  # 5W 平台专用
    except ImportError as e:
        rospy.logerr(f"TorsoController 初始化失败：无法导入 lb_ctrl_api ({e})")
        rospy.signal_shutdown("lb_ctrl_api unavailable")
        raise

    # 获取躯干初始位姿，带超时
    deadline = rospy.Time.now() + rospy.Duration(init_timeout_sec)
    success, initial_pose = False, None
    while rospy.Time.now() < deadline and not rospy.is_shutdown():
        try:
            success, initial_pose = ct.get_torso_initial_pose(True)
            if success:
                break
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"get_torso_initial_pose 异常: {e}")
        rospy.sleep(0.1)

    if not success or initial_pose is None:
        rospy.logerr("TorsoController 初始化失败：超时未获取躯干初始位姿")
        rospy.signal_shutdown("get_torso_initial_pose timeout")
        raise RuntimeError("get_torso_initial_pose timeout")

    initial_xyz = initial_pose["position"]

    pub = rospy.Publisher("/cmd_lb_torso_pose", Twist, queue_size=10)
    # 订阅 reach_time 仅用于日志/观测，TorsoController 自身不阻塞依赖
    rospy.Subscriber(
        "/lb_torso_pose_reach_time",
        Float32,
        lambda msg: rospy.logdebug(f"torso reach_time={msg.data:.3f}s"),
    )

    def _publish(twist):
        try:
            pub.publish(twist)
        except Exception as e:
            rospy.logerr_throttle(5.0, f"/cmd_lb_torso_pose publish 异常: {e}")

    return TorsoController(
        initial_pose_xyz=initial_xyz,
        publisher=_publish,
        clock=lambda: rospy.Time.now().to_sec(),
    )
