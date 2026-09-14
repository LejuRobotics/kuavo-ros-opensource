"""5W 平台单手摇杆控制躯干（折叠臂升降 + 前后倾 + 腰部旋转）

速度/增量语义（不再内部积分开环位姿）：
  - 摇杆 → /cmd_torso_vel（瞬时速度）
  - yaw 按键 → /cmd_torso_delta（oneshot 相对位移）
  - 复位 → /cmd_lb_torso_pose + reset service

Each input source self-clamps against /torso_open_loop_state (open-loop pose from ReferenceManager).
"""
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

# ===== 速度缩放（摇杆满行程对应的速度） =====
SCALE_HEIGHT = 0.10   # m/s
SCALE_PITCH = 0.30    # rad/s

# ===== 限幅（与 MobileManipulatorJoyCommandNode 一致，相对初始位姿） =====
Z_MIN, Z_MAX = 0.0, 0.32            # m，升降相对初始位姿
X_MIN = 0.0                           # m，前后相对初始位姿
PITCH_MIN, PITCH_MAX = -0.5235, 0.0   # rad；VR 内部：负值前倾，0 为直立
YAW_MAX = 0.5235                      # rad（±30°，仅摇杆积分）
# 按钮转身限幅：URDF waist_yaw_joint 限位 ±π，贴边界时四元数双覆盖 → IK 振荡。
# 留 0.05 rad 约 3° margin，单次按键直达 ±YAW_BUTTON_LIMIT，反方向按键可原路返回。
YAW_BUTTON_LIMIT = math.pi - 0.05   # rad（≈±177°）
YAW_TARGET = YAW_BUTTON_LIMIT       # rad，按钮单次转身量（与限幅对齐，保证对称可逆）

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
    """从 kuavo.json 的 kuavo_wheel_torso_limit 加载限位；失败时返回模块默认值。

    Return values use open-loop frame (positive pitch = lean forward):
      (z_min, z_max, x_min, pitch_min, pitch_max, yaw_max)
    """
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
            # 配置约定 [0, 30°] 表示仅前倾；VR 内部负值前倾
            # Return value uses VR-internal convention; caller converts to open-loop frame
            pitch_min = -math.radians(max(pitch_hi_deg, 0.0))
            pitch_max = -math.radians(min(pitch_lo_deg, 0.0))

        yaw_max = YAW_MAX
        return z_min, z_max, x_min, pitch_min, pitch_max, yaw_max
    except Exception:
        return Z_MIN, Z_MAX, X_MIN, PITCH_MIN, PITCH_MAX, YAW_MAX


class TorsoController:
    """单手摇杆控躯干的状态机 + 命令发布器。

    依赖注入：publisher / vel_publisher / delta_publisher / clock / state_getter 由调用方传入。
    - publisher: 绝对位姿（复位路径）
    - vel_publisher: 连续速度 /cmd_torso_vel
    - delta_publisher: oneshot 相对位移 /cmd_torso_delta
    - state_getter: current open-loop 4D pose (x, z, yaw, pitch) or None
    - limits: (z_min, z_max, x_min, pitch_min, pitch_max, yaw_max), VR 内部约定
    """

    def __init__(
        self,
        initial_pose_xyz,
        publisher: Optional[Callable] = None,
        vel_publisher: Optional[Callable] = None,
        delta_publisher: Optional[Callable] = None,
        clock: Optional[Callable[[], float]] = None,
        resetter: Optional[Callable[[], object]] = None,
        state_getter: Optional[Callable[[], Optional[Tuple[float, float, float, float]]]] = None,
        limits: Optional[Tuple[float, float, float, float, float, float]] = None,
    ):
        self.initial_xyz = tuple(initial_pose_xyz)
        self._publish_fn = publisher
        self._vel_publish_fn = vel_publisher
        self._delta_publish_fn = delta_publisher
        self._clock = clock if clock is not None else time.monotonic
        self._resetter = resetter
        self._state_getter = state_getter
        # 限位（VR 内部约定 pitch 负值 = 前倾）
        (
            self._z_min,
            self._z_max,
            self._x_min,
            self._pitch_min,
            self._pitch_max,
            self._yaw_max,
        ) = limits if limits is not None else (Z_MIN, Z_MAX, X_MIN, PITCH_MIN, PITCH_MAX, YAW_MAX)
        self._yaw_min = -self._yaw_max
        # open-loop pitch limits (positive = lean forward; VR internal is negated)
        self._pitch_min_open_loop = -self._pitch_max
        self._pitch_max_open_loop = -self._pitch_min
        self.main_hand: Optional[str] = None  # None / 'left' / 'right'
        self._l_was_pressed = False
        self._r_was_pressed = False
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
        self._pending_post_reset_pose = False
        # True after a non-zero /cmd_torso_vel publish; used to emit a single zero to clear ReferenceManager velocity latch.
        self._latched_vel_active = False

    def handle_joystick(self, msg) -> None:
        l_pressed, r_pressed, l_released, r_released = self._get_grip_states(msg)
        now = self._clock()
        busy_eps = 1e-6

        # 复位窗口结束后再发绝对零位（SDK/观测）；紧接在服务后发布会抢先清零 RM 开环起点 → 复位过快
        if self._pending_post_reset_pose and now + busy_eps >= self._reset_busy_until:
            self._publish_torso_pose()
            self._pending_post_reset_pose = False

        # 复位轨迹未完成前不接受新的摇杆/按键控制，避免覆盖复位目标。
        if now + busy_eps < self._reset_busy_until:
            self._l_was_pressed = l_pressed
            self._r_was_pressed = r_pressed
            return

        # 先基于 press edge 锁定主控手，release 清理放在双击检测之后，
        # 这样第一下短按释放时仍可被双击状态机看到。
        self._arbitrate_main_hand_press(l_pressed, r_pressed)

        reset_triggered = False
        if self.main_hand is not None:
            reset_triggered = self._detect_grip_reset(msg, l_pressed, r_pressed, l_released, r_released)

        prev_main_hand = self.main_hand
        self._arbitrate_main_hand_release(l_released, r_released)

        # Grip release while velocity was latched → one zero to clear ReferenceManager velocity latch.
        if prev_main_hand is not None and self.main_hand is None and self._latched_vel_active:
            self._publish_torso_vel(vx=0.0, vz=0.0, vpitch=0.0, vyaw=0.0)
            self._latched_vel_active = False

        if self.main_hand is None and not reset_triggered:
            self._l_was_pressed = l_pressed
            self._r_was_pressed = r_pressed
            return

        if reset_triggered:
            self._pending_post_reset_pose = True
            self._l_was_pressed = l_pressed
            self._r_was_pressed = r_pressed
            return

        self._handle_yaw_buttons(msg)
        self._apply_stick_velocity(msg)
        self._l_was_pressed = l_pressed
        self._r_was_pressed = r_pressed

    def is_reset_busy(self) -> bool:
        return self._clock() < self._reset_busy_until

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
        self._first_btn_was_pressed = False
        self._second_btn_was_pressed = False
        self._latched_vel_active = False
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

    def _apply_stick_velocity(self, msg) -> None:
        """Stick → velocity; clamp against /torso_open_loop_state workspace limits."""
        sx, sy = self._select_stick(msg)
        sx = sx if abs(sx) > STICK_DEAD_ZONE else 0.0
        sy = sy if abs(sy) > STICK_DEAD_ZONE else 0.0

        # 升高：摇杆 Y > 0 → +vz
        vz = sy * SCALE_HEIGHT
        # Lean forward: stick X > 0 → decrease VR pitch (spec 6.1) → negative vpitch (open-loop frame)
        vpitch = -sx * SCALE_PITCH

        # Read open-loop state; zero over-limit velocity components
        open_loop_state = self._get_open_loop_state()
        if open_loop_state is not None:
            _ol_x, ol_z, _ol_yaw, ol_pitch = open_loop_state
            z_abs_lo, z_abs_hi = self._z_limits()
            # Z 限位：超上限不发正向速度，超下限不发负向速度
            if vz > 0.0 and ol_z >= z_abs_hi - POSE_PUBLISH_EPS:
                vz = 0.0
            elif vz < 0.0 and ol_z <= z_abs_lo + POSE_PUBLISH_EPS:
                vz = 0.0
            # Pitch limits (open-loop frame: positive = lean forward)
            if vpitch > 0.0 and ol_pitch >= self._pitch_max_open_loop - POSE_PUBLISH_EPS:
                vpitch = 0.0
            elif vpitch < 0.0 and ol_pitch <= self._pitch_min_open_loop + POSE_PUBLISH_EPS:
                vpitch = 0.0

        if abs(vz) < 1e-9 and abs(vpitch) < 1e-9:
            if self._latched_vel_active:
                self._publish_torso_vel(vx=0.0, vz=0.0, vpitch=0.0, vyaw=0.0)
                self._latched_vel_active = False
            return
        self._latched_vel_active = True
        self._publish_torso_vel(vx=0.0, vz=vz, vpitch=vpitch, vyaw=0.0)

    def _select_buttons(self, msg):
        if self.main_hand == "left":
            return msg.left_first_button_pressed, msg.left_second_button_pressed
        return msg.right_first_button_pressed, msg.right_second_button_pressed

    def _handle_yaw_buttons(self, msg) -> None:
        """Yaw button edge → oneshot delta; clamp target against open-loop yaw limit.

        从 /torso_open_loop_state 读取当前开环目标 yaw，计算 desired = ol_yaw ± YAW_TARGET，
        clamp 到 ±YAW_BUTTON_LIMIT，再折算为 delta 发出。到限后再按同方向 → delta=0（无响应）。
        反方向始终可用（可在越界状态下手动拉回）。
        """
        first, second = self._select_buttons(msg)
        first_edge = first and not self._first_btn_was_pressed
        second_edge = second and not self._second_btn_was_pressed
        self._first_btn_was_pressed = first
        self._second_btn_was_pressed = second

        if not first_edge and not second_edge:
            return

        direction = -YAW_TARGET if first_edge else +YAW_TARGET
        open_loop_state = self._get_open_loop_state()
        ol_yaw = open_loop_state[2] if open_loop_state is not None else 0.0

        desired = max(-YAW_BUTTON_LIMIT,
                      min(ol_yaw + direction, YAW_BUTTON_LIMIT))
        effective_delta = desired - ol_yaw

        if abs(effective_delta) < POSE_PUBLISH_EPS:
            return  # 已在限位，无响应（issue 3595 预期："连续按同一方向按键时躯干无响应"）

        self._publish_torso_delta(yaw=effective_delta)

    def _get_open_loop_state(self) -> Optional[Tuple[float, float, float, float]]:
        """返回 (x, z, yaw, pitch) 或 None。"""
        if self._state_getter is None:
            return None
        try:
            return self._state_getter()
        except Exception:
            return None

    def _make_twist(self, lx=0.0, ly=0.0, lz=0.0, ax=0.0, ay=0.0, az=0.0):
        from geometry_msgs.msg import Twist
        twist = Twist()
        twist.linear.x = lx
        twist.linear.y = ly
        twist.linear.z = lz
        twist.angular.x = ax
        twist.angular.y = ay
        twist.angular.z = az
        return twist

    def _publish_via(self, publish_fn, lx=0.0, ly=0.0, lz=0.0, ax=0.0, ay=0.0, az=0.0) -> None:
        if publish_fn is None:
            return
        publish_fn(self._make_twist(lx=lx, ly=ly, lz=lz, ax=ax, ay=ay, az=az))

    def _publish_torso_pose(self) -> None:
        """复位路径：发布 initial_xyz 处的绝对零姿态。"""
        self._publish_via(
            self._publish_fn,
            lx=self.initial_xyz[0],
            ly=self.initial_xyz[1],
            lz=self.initial_xyz[2],
        )

    def _publish_torso_vel(self, vx=0.0, vz=0.0, vpitch=0.0, vyaw=0.0) -> None:
        # 4D: linear.x/z + angular.y/z (pitch/yaw)
        self._publish_via(self._vel_publish_fn, lx=vx, lz=vz, ay=vpitch, az=vyaw)

    def _publish_torso_delta(self, x=0.0, z=0.0, pitch=0.0, yaw=0.0) -> None:
        # same 4D layout as vel
        self._publish_via(self._delta_publish_fn, lx=x, lz=z, ay=pitch, az=yaw)

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
        elif r_pressed and not self._r_was_pressed:
            self.main_hand = "right"

    def _arbitrate_main_hand_release(self, l_released: bool, r_released: bool) -> None:
        # 已锁定，仅主指手 grip 松开（< release_th）才释放
        if self.main_hand == "left" and l_released:
            self.main_hand = None
        elif self.main_hand == "right" and r_released:
            self.main_hand = None


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

    # 加载 workspace 限位（beta 逻辑）
    limits = load_torso_limits_from_config()
    rospy.loginfo(
        f"TorsoController limits from config: "
        f"z=[{limits[0]:.3f}, {limits[1]:.3f}], "
        f"pitch=[{limits[3]:.3f}, {limits[4]:.3f}], "
        f"yaw=±{limits[5]:.3f}"
    )

    pub_pose = rospy.Publisher("/cmd_lb_torso_pose", Twist, queue_size=10)
    pub_vel = rospy.Publisher("/cmd_torso_vel", Twist, queue_size=10)
    pub_delta = rospy.Publisher("/cmd_torso_delta", Twist, queue_size=10)

    # Subscribe open-loop torso state from ReferenceManager
    open_loop_state_lock = [None]  # mutable closure

    def _open_loop_state_callback(msg):
        open_loop_state_lock[0] = (msg.linear.x, msg.linear.z, msg.angular.z, msg.angular.y)

    rospy.Subscriber("/torso_open_loop_state", Twist, _open_loop_state_callback, queue_size=1)

    # 订阅 reach_time 仅用于日志/观测，TorsoController 自身不阻塞依赖
    rospy.Subscriber(
        "/lb_torso_pose_reach_time",
        Float32,
        lambda msg: rospy.logdebug(f"torso reach_time={msg.data:.3f}s"),
    )

    def _safe_publish(pub, topic: str):
        def _publish(twist):
            try:
                pub.publish(twist)
            except Exception as e:
                rospy.logerr_throttle(5.0, f"{topic} publish 异常: {e}")
        return _publish

    return TorsoController(
        initial_pose_xyz=initial_xyz,
        publisher=_safe_publish(pub_pose, "/cmd_lb_torso_pose"),
        vel_publisher=_safe_publish(pub_vel, "/cmd_torso_vel"),
        delta_publisher=_safe_publish(pub_delta, "/cmd_torso_delta"),
        clock=lambda: rospy.Time.now().to_sec(),
        resetter=lambda: ct.reset_torso_to_initial(wait_timeout_sec=1.0),
        state_getter=lambda: open_loop_state_lock[0],
        limits=limits,
    )
