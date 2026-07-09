"""5W 平台单手摇杆控制躯干（折叠臂升降 + 前后倾 + 腰部旋转）"""
from __future__ import annotations
import json
import math
import os
import time
from enum import Enum
from typing import Callable, Optional, Tuple


# ===== 阈值 =====
GRIP_PRESS_TH = 0.7
GRIP_RELEASE_TH = 0.3
STICK_DEAD_ZONE = 0.1
POSE_PUBLISH_EPS = 1e-5

# ===== 缩放系数（增量速率） =====
SCALE_HEIGHT = 0.10   # m/s
SCALE_PITCH = 0.30    # rad/s

# ===== 限幅（与 MobileManipulatorJoyCommandNode 一致，相对初始位姿） =====
Z_MIN, Z_MAX = 0.0, 0.32            # m，升降相对初始位姿
X_MIN = 0.0                           # m，前后相对初始位姿
PITCH_MIN, PITCH_MAX = -0.5235, 0.0   # rad；VR 约定：负值前倾，0 为直立
YAW_MIN, YAW_MAX = -0.5235, 0.5235    # rad，仅用于摇杆积分（按钮转身仍走 ±π）
YAW_TARGET = math.pi                 # rad（180°）

# ===== 时序 =====
MIN_DT = 0.001
MAX_DT = 0.1

# ===== grip 双击复位防误触参数 =====
GRIP_DOUBLE_CLICK_WINDOW = 0.40  # s，两次短按之间的最大间隔
GRIP_SHORT_PRESS_MAX = 0.30      # s，超过该时长视为正常长按控制
RESET_COOLDOWN = 1.00            # s，复位后的冷却时间
RESET_STICK_DEAD_ZONE = 0.10     # 复位触发时要求摇杆接近零位


class GripClickState(Enum):
    IDLE = 0
    CLICK1_DOWN = 1
    WAIT_CLICK2 = 2
    CLICK2_DOWN = 3


def get_torso_max_x(z_increment: float) -> float:
    """根据 z 抬升量计算 x 方向最大允许偏移（与轮臂手柄节点经验曲线一致）。"""
    if z_increment <= 0.0:
        return 0.05
    if z_increment <= 0.3:
        return 0.05 + (z_increment / 0.3) * 0.1
    if z_increment <= 0.5:
        return 0.15 + ((z_increment - 0.3) / 0.2) * 0.1
    return 0.25


def load_torso_limits_from_config() -> Tuple[float, float, float, float, float, float]:
    """从 kuavo.json 的 kuavo_wheel_torso_limit 加载限位；失败时返回模块默认值。"""
    robot_version = os.getenv("ROBOT_VERSION")
    if not robot_version:
        return Z_MIN, Z_MAX, X_MIN, PITCH_MIN, PITCH_MAX, YAW_MAX

    try:
        import rospkg
        kuavo_assets_path = rospkg.RosPack().get_path("kuavo_assets")
        config_path = os.path.join(
            kuavo_assets_path, "config", f"kuavo_v{robot_version}", "kuavo.json"
        )
        if not os.path.exists(config_path):
            return Z_MIN, Z_MAX, X_MIN, PITCH_MIN, PITCH_MAX, YAW_MAX

        with open(config_path, "r", encoding="utf-8") as f:
            kuavo_config = json.load(f)
        torso_cfg = kuavo_config.get("kuavo_wheel_torso_limit")
        if not torso_cfg:
            return Z_MIN, Z_MAX, X_MIN, PITCH_MIN, PITCH_MAX, YAW_MAX

        z_range = torso_cfg.get("z_range")
        x_range = torso_cfg.get("x_range")
        pitch_range = torso_cfg.get("pitch_range")
        base_position = torso_cfg.get("base_position", [0.0, 0.0, 0.0])

        z_min, z_max = Z_MIN, Z_MAX
        if z_range and len(z_range) == 2:
            z_span = float(z_range[1]) - float(z_range[0])
            if z_span > 0.0:
                z_min, z_max = 0.0, min(Z_MAX, z_span)

        x_min = X_MIN
        if x_range and len(x_range) == 2:
            x_span = float(x_range[1]) - float(x_range[0])
            if x_span > 0.0:
                x_min = 0.0

        pitch_min, pitch_max = PITCH_MIN, PITCH_MAX
        if pitch_range and len(pitch_range) == 2:
            pitch_lo_deg = float(pitch_range[0])
            pitch_hi_deg = float(pitch_range[1])
            # VR 约定前倾为负 pitch：配置 [0, 20°] → [-20°, 0]
            pitch_min = -math.radians(max(pitch_hi_deg, 0.0))
            pitch_max = -math.radians(min(pitch_lo_deg, 0.0))

        yaw_max = YAW_MAX
        return z_min, z_max, x_min, pitch_min, pitch_max, yaw_max
    except Exception:
        return Z_MIN, Z_MAX, X_MIN, PITCH_MIN, PITCH_MAX, YAW_MAX


class TorsoController:
    """单手摇杆控躯干的状态机 + 命令发布器。

    依赖注入：publisher、clock 由调用方传入，便于单元测试。
    """

    def __init__(
        self,
        initial_pose_xyz,
        publisher: Optional[Callable] = None,
        clock: Optional[Callable[[], float]] = None,
        resetter: Optional[Callable[[], object]] = None,
        limits: Optional[Tuple[float, float, float, float, float, float]] = None,
    ):
        self.initial_xyz = tuple(initial_pose_xyz)
        # 索引 [0:3] = x/y/z（取自 initial_pose_xyz）
        # 索引 [3:6] = roll/pitch/yaw（机械零位为基准）
        self.current_pose = list(initial_pose_xyz) + [0.0, 0.0, 0.0]
        (
            self._z_min,
            self._z_max,
            self._x_min,
            self._pitch_min,
            self._pitch_max,
            self._yaw_max,
        ) = limits if limits is not None else load_torso_limits_from_config()
        self._yaw_min = -self._yaw_max
        self._publish_fn = publisher
        self._clock = clock if clock is not None else time.monotonic
        self._resetter = resetter
        self.main_hand: Optional[str] = None  # None / 'left' / 'right'
        self._last_published_pose: Optional[list] = None
        self._force_publish = False
        self._l_was_pressed = False
        self._r_was_pressed = False
        self._last_t: Optional[float] = None
        # A/B 按键边沿检测的"上一帧按下"状态
        self._first_btn_was_pressed = False
        self._second_btn_was_pressed = False
        # grip 短按双击复位状态
        self._grip_click_state = GripClickState.IDLE
        self._grip_click_hand: Optional[str] = None
        self._grip_press_t: Optional[float] = None
        self._first_click_release_t: Optional[float] = None
        self._reset_cooldown_until = 0.0
        self._reset_busy_until = 0.0

    def handle_joystick(self, msg) -> None:
        l_pressed, r_pressed, l_released, r_released = self._get_grip_states(msg)
        now = self._clock()

        # 复位轨迹未完成前不接受新的摇杆/按键控制，避免覆盖复位目标。
        if now < self._reset_busy_until:
            self._l_was_pressed = l_pressed
            self._r_was_pressed = r_pressed
            return

        # 先基于 press edge 锁定主控手，release 清理放在双击检测之后，
        # 这样第一下短按释放时仍可被双击状态机看到。
        self._arbitrate_main_hand_press(l_pressed, r_pressed)

        reset_triggered = False
        if self.main_hand is not None:
            reset_triggered = self._detect_grip_reset(msg, l_pressed, r_pressed, l_released, r_released)

        self._arbitrate_main_hand_release(l_released, r_released)

        if self.main_hand is None and not reset_triggered:
            self._l_was_pressed = l_pressed
            self._r_was_pressed = r_pressed
            return

        if reset_triggered:
            self._publish_torso_pose()
            self._l_was_pressed = l_pressed
            self._r_was_pressed = r_pressed
            return

        dt = self._compute_dt()
        self._handle_yaw_buttons(msg)
        self._apply_stick_increment(msg, dt)
        if self._should_publish_pose():
            self._publish_torso_pose()
        self._l_was_pressed = l_pressed
        self._r_was_pressed = r_pressed

    def is_reset_busy(self) -> bool:
        return self._clock() < self._reset_busy_until

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

    def _is_main_stick_zero(self, msg) -> bool:
        sx, sy = self._select_stick(msg)
        return abs(sx) < RESET_STICK_DEAD_ZONE and abs(sy) < RESET_STICK_DEAD_ZONE

    def _reset_grip_click_state(self) -> None:
        self._grip_click_state = GripClickState.IDLE
        self._grip_click_hand = None
        self._grip_press_t = None
        self._first_click_release_t = None

    def _reset_runtime_state(self) -> None:
        """复位成功后回到干净 IDLE，要求用户重新松开/按下 grip 才能控制。"""
        self.main_hand = None
        self._last_t = None
        self._first_btn_was_pressed = False
        self._second_btn_was_pressed = False
        self._reset_grip_click_state()

    def _execute_reset(self) -> Optional[float]:
        if self._resetter is None:
            return None
        try:
            result = self._resetter()
        except Exception:
            return None

        # lb_ctrl_api.reset_torso_to_initial() 成功时返回估计时间；0.0 表示失败。
        # 也接受 True 这类 truthy 返回值，便于测试和未来封装。
        if not result:
            return None

        self.current_pose = list(self.initial_xyz) + [0.0, 0.0, 0.0]
        self._last_published_pose = None
        self._reset_runtime_state()
        if isinstance(result, bool):
            return 0.0
        try:
            return float(result)
        except (TypeError, ValueError):
            return 0.0

    def _detect_grip_reset(self, msg, l_pressed: bool, r_pressed: bool, l_released: bool, r_released: bool) -> bool:
        now = self._clock()
        if now < self._reset_cooldown_until:
            return False

        if self.main_hand == "left":
            hand = "left"
            pressed = l_pressed
            released = l_released
            press_edge = l_pressed and not self._l_was_pressed
        elif self.main_hand == "right":
            hand = "right"
            pressed = r_pressed
            released = r_released
            press_edge = r_pressed and not self._r_was_pressed
        else:
            self._reset_grip_click_state()
            return False

        if not self._is_main_stick_zero(msg):
            self._reset_grip_click_state()
            return False

        if self._grip_click_state == GripClickState.IDLE:
            if press_edge:
                self._grip_click_state = GripClickState.CLICK1_DOWN
                self._grip_click_hand = hand
                self._grip_press_t = now
            return False

        if hand != self._grip_click_hand:
            self._reset_grip_click_state()
            return False

        if self._grip_click_state == GripClickState.CLICK1_DOWN:
            if pressed and self._grip_press_t is not None and (now - self._grip_press_t) > GRIP_SHORT_PRESS_MAX:
                self._reset_grip_click_state()
                return False
            if released:
                if self._grip_press_t is not None and (now - self._grip_press_t) <= GRIP_SHORT_PRESS_MAX:
                    self._grip_click_state = GripClickState.WAIT_CLICK2
                    self._first_click_release_t = now
                else:
                    self._reset_grip_click_state()
            return False

        if self._grip_click_state == GripClickState.WAIT_CLICK2:
            if self._first_click_release_t is None or (now - self._first_click_release_t) > GRIP_DOUBLE_CLICK_WINDOW:
                self._reset_grip_click_state()
                return False
            if press_edge:
                self._grip_click_state = GripClickState.CLICK2_DOWN
                self._grip_press_t = now
            return False

        if self._grip_click_state == GripClickState.CLICK2_DOWN:
            if pressed and self._grip_press_t is not None and (now - self._grip_press_t) > GRIP_SHORT_PRESS_MAX:
                self._reset_grip_click_state()
                return False
            if released:
                if self._grip_press_t is None or (now - self._grip_press_t) > GRIP_SHORT_PRESS_MAX:
                    self._reset_grip_click_state()
                    return False
                reset_duration = self._execute_reset()
                success = reset_duration is not None
                if success:
                    reset_done_t = self._clock()
                    self._reset_cooldown_until = reset_done_t + RESET_COOLDOWN
                    self._reset_busy_until = reset_done_t + max(0.0, reset_duration)
                else:
                    self._reset_grip_click_state()
                return success

        return False

    def _z_limits(self) -> Tuple[float, float]:
        return self.initial_xyz[2] + self._z_min, self.initial_xyz[2] + self._z_max

    def _x_limits(self) -> Tuple[float, float]:
        z_offset = self.current_pose[2] - self.initial_xyz[2]
        x_hi = self.initial_xyz[0] + get_torso_max_x(z_offset)
        return self.initial_xyz[0] + self._x_min, x_hi

    def _apply_stick_increment(self, msg, dt: float) -> None:
        sx, sy = self._select_stick(msg)
        sx = sx if abs(sx) > STICK_DEAD_ZONE else 0.0
        sy = sy if abs(sy) > STICK_DEAD_ZONE else 0.0

        z_lo, z_hi = self._z_limits()
        # 触限时不再积分，避免持续向 MPC 发送不可达目标导致下肢顶限振动
        if sy > 0.0 and self.current_pose[2] >= z_hi - POSE_PUBLISH_EPS:
            sy = 0.0
        elif sy < 0.0 and self.current_pose[2] <= z_lo + POSE_PUBLISH_EPS:
            sy = 0.0
        if sx > 0.0 and self.current_pose[4] <= self._pitch_min + POSE_PUBLISH_EPS:
            sx = 0.0
        elif sx < 0.0 and self.current_pose[4] >= self._pitch_max - POSE_PUBLISH_EPS:
            sx = 0.0

        # 升高：摇杆 Y > 0 → z 增
        self.current_pose[2] += sy * SCALE_HEIGHT * dt
        # 前倾：摇杆 X > 0 → pitch 减（spec 6.1）
        self.current_pose[4] -= sx * SCALE_PITCH * dt

        self._clamp_pose()

    def _clamp_pose(self) -> None:
        z_lo, z_hi = self._z_limits()
        self.current_pose[2] = max(z_lo, min(self.current_pose[2], z_hi))

        x_lo, x_hi = self._x_limits()
        self.current_pose[0] = max(x_lo, min(self.current_pose[0], x_hi))

        self.current_pose[4] = max(
            self._pitch_min, min(self.current_pose[4], self._pitch_max)
        )

    def _pose_changed(self) -> bool:
        if self._last_published_pose is None:
            return True
        return any(
            abs(a - b) > POSE_PUBLISH_EPS
            for a, b in zip(self.current_pose, self._last_published_pose)
        )

    def _should_publish_pose(self) -> bool:
        if self._force_publish:
            self._force_publish = False
            return True
        return self._pose_changed()

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
        self._clamp_pose()

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
        # VR 内部前倾为负 pitch，发布给 MPC 时取反（MPC 约定：0 直立，正值前倾）
        twist.angular.y = -self.current_pose[4]
        twist.angular.z = self.current_pose[5]
        self._publish_fn(twist)
        self._last_published_pose = list(self.current_pose)

    def _get_grip_states(self, msg):
        l_pressed = msg.left_grip > GRIP_PRESS_TH
        r_pressed = msg.right_grip > GRIP_PRESS_TH
        l_released = msg.left_grip < GRIP_RELEASE_TH
        r_released = msg.right_grip < GRIP_RELEASE_TH
        return l_pressed, r_pressed, l_released, r_released

    def _arbitrate_main_hand_press(self, l_pressed: bool, r_pressed: bool) -> None:
        if self.main_hand is not None:
            return
        # 边沿检测：上一帧没按下、这一帧按下 → 锁定
        if l_pressed and not self._l_was_pressed:
            self.main_hand = "left"
            self._force_publish = True
        elif r_pressed and not self._r_was_pressed:
            self.main_hand = "right"
            self._force_publish = True

    def _arbitrate_main_hand_release(self, l_released: bool, r_released: bool) -> None:
        # 已锁定，仅主指手 grip 松开（< release_th）才释放
        if self.main_hand == "left" and l_released:
            self.main_hand = None
            self._last_t = None  # 重置时钟基准，下次重新激活首帧返回 MIN_DT
        elif self.main_hand == "right" and r_released:
            self.main_hand = None
            self._last_t = None


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
        resetter=lambda: ct.reset_torso_to_initial(wait_timeout_sec=1.0),
    )
