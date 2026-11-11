"""单元测试：TorsoController（不依赖 rospy）。

语义：
  - 摇杆升降 → /cmd_torso_vel；摇杆俯仰 → /cmd_torso_rotation_vel
  - yaw 边沿 → /cmd_torso_delta oneshot（旋转组内由 RM 累计到有界关节目标，不退组）
  - 切组不向旧话题补零；松 grip 仍经 _clear_latched_velocities 清零
  - 复位 → resetter + 一次绝对 /cmd_lb_torso_pose
"""
import os
import sys
from types import ModuleType, SimpleNamespace
from unittest.mock import MagicMock

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

if "geometry_msgs.msg" not in sys.modules:
    _stub = ModuleType("geometry_msgs.msg")

    class _Vec3:
        def __init__(self):
            self.x = self.y = self.z = 0.0

    class _Twist:
        def __init__(self):
            self.linear = _Vec3()
            self.angular = _Vec3()

    _stub.Twist = _Twist
    sys.modules.setdefault("geometry_msgs", ModuleType("geometry_msgs"))
    sys.modules["geometry_msgs.msg"] = _stub

from tools.torso_joystick_controller import (  # noqa: E402
    SCALE_HEIGHT,
    SCALE_PITCH,
    STICK_PITCH_AXIS_THRESHOLD,
    TorsoController,
    YAW_BUTTON_LIMIT,
    YAW_TARGET,
)


def make_msg(**kwargs):
    defaults = dict(
        left_x=0.0, left_y=0.0, left_trigger=0.0, left_grip=0.0,
        left_first_button_pressed=False, left_second_button_pressed=False,
        left_first_button_touched=False, left_second_button_touched=False,
        right_x=0.0, right_y=0.0, right_trigger=0.0, right_grip=0.0,
        right_first_button_pressed=False, right_second_button_pressed=False,
        right_first_button_touched=False, right_second_button_touched=False,
    )
    defaults.update(kwargs)
    return SimpleNamespace(**defaults)


def make_controller(t0=0.0, resetter=None):
    clock = MagicMock(return_value=t0)
    pub = MagicMock()
    vel = MagicMock()
    rot = MagicMock()
    delta = MagicMock()
    ctrl = TorsoController(
        initial_pose_xyz=(0.0, 0.0, 0.0),
        publisher=pub,
        vel_publisher=vel,
        rotation_vel_publisher=rot,
        delta_publisher=delta,
        clock=clock,
        resetter=resetter,
    )
    return ctrl, pub, vel, rot, delta, clock


def make_controller_with_state(t0=0.0, resetter=None, state_getter=None):
    """same as make_controller but with a state_getter for /torso_open_loop_state."""
    clock = MagicMock(return_value=t0)
    pub = MagicMock()
    vel = MagicMock()
    rot = MagicMock()
    delta = MagicMock()
    ctrl = TorsoController(
        initial_pose_xyz=(0.0, 0.0, 0.0),
        publisher=pub,
        vel_publisher=vel,
        rotation_vel_publisher=rot,
        delta_publisher=delta,
        clock=clock,
        resetter=resetter,
        state_getter=state_getter,
    )
    return ctrl, pub, vel, rot, delta, clock


def _double_click_reset(ctrl, clock, grip_kw="left_grip", t0=0.0):
    """短按-放-短按-放。"""
    g = {grip_kw: 0.8}
    z = {grip_kw: 0.0}
    clock.return_value = t0
    ctrl.handle_joystick(make_msg(**g))
    clock.return_value = t0 + 0.10
    ctrl.handle_joystick(make_msg(**z))
    clock.return_value = t0 + 0.25
    ctrl.handle_joystick(make_msg(**g))
    clock.return_value = t0 + 0.32
    ctrl.handle_joystick(make_msg(**z))


# ---- idle / grip 仲裁 ----

def test_idle_no_publish_even_with_resetter():
    resetter = MagicMock(return_value=0.5)
    ctrl, pub, vel, rot, delta, _ = make_controller(resetter=resetter)
    ctrl.handle_joystick(make_msg())
    pub.assert_not_called()
    vel.assert_not_called()
    rot.assert_not_called()
    delta.assert_not_called()
    resetter.assert_not_called()
    assert ctrl.main_hand is None


def test_grip_lock_hysteresis_and_hand_switch():
    ctrl, _, _, _, _, _ = make_controller()
    ctrl.handle_joystick(make_msg(left_grip=0.8))
    assert ctrl.main_hand == "left"
    # 第二手不抢占；滞回区内不释放
    ctrl.handle_joystick(make_msg(left_grip=0.8, right_grip=0.8))
    ctrl.handle_joystick(make_msg(left_grip=0.4, right_grip=0.0))
    assert ctrl.main_hand == "left"
    # 松开后可换手
    ctrl.handle_joystick(make_msg(left_grip=0.0))
    assert ctrl.main_hand is None
    ctrl.handle_joystick(make_msg(right_grip=0.8))
    assert ctrl.main_hand == "right"


# ---- 速度语义（本 issue 核心）----

def test_stick_publishes_instant_velocity():
    """满杆 → 瞬时速度；与 dt 无关；错手摇杆忽略。同帧俯仰优先走旋转组。"""
    ctrl, pub, vel, rot, _, clock = make_controller(t0=0.0)
    ctrl.handle_joystick(make_msg(left_grip=0.8))
    vel.reset_mock()
    rot.reset_mock()
    clock.return_value = 1.0  # 大 dt 也不应改变速度幅值
    # 同帧升降+俯仰：俯仰优先，只发旋转组
    ctrl.handle_joystick(make_msg(left_grip=0.8, left_y=1.0, left_x=1.0))
    vel.assert_not_called()
    rot.assert_called_once()
    tw = rot.call_args[0][0]
    assert abs(tw.angular.y - (-SCALE_PITCH)) < 1e-9
    assert abs(tw.linear.z) < 1e-9
    pub.assert_not_called()  # 连续控制不走绝对位姿

    # 纯升降走平移组；切组不向旧旋转话题补零
    vel.reset_mock()
    rot.reset_mock()
    ctrl.handle_joystick(make_msg(left_grip=0.8, left_y=1.0, left_x=0.0))
    rot.assert_not_called()
    vel.assert_called_once()
    tw_v = vel.call_args[0][0]
    assert abs(tw_v.linear.z - SCALE_HEIGHT) < 1e-9
    assert abs(tw_v.angular.y) < 1e-9

    # 错手：主手杆已回零会发一次 0 清 RM 粘性；不得出现右杆驱动的非零速度
    vel.reset_mock()
    rot.reset_mock()
    ctrl.handle_joystick(make_msg(left_grip=0.8, right_y=1.0))
    if vel.called:
        tw0 = vel.call_args[0][0]
        assert abs(tw0.linear.z) < 1e-9 and abs(tw0.angular.y) < 1e-9
        assert abs(tw0.linear.z - SCALE_HEIGHT) > 0.01  # not right-stick height


def test_stick_deadzone_and_zero_clear_latch():
    """死区/零位：未发过非零则不发；发过非零后回零发一次 0 清 RM 粘性速度。"""
    ctrl, _, vel, rot, _, clock = make_controller(t0=0.0)
    ctrl.handle_joystick(make_msg(left_grip=0.8))
    vel.reset_mock()
    rot.reset_mock()
    clock.return_value = 0.1
    ctrl.handle_joystick(make_msg(left_grip=0.8, left_x=0.05, left_y=0.05))
    vel.assert_not_called()
    rot.assert_not_called()

    clock.return_value = 0.2
    ctrl.handle_joystick(make_msg(left_grip=0.8, left_y=1.0))
    assert vel.call_count == 1
    rot.assert_not_called()

    vel.reset_mock()
    clock.return_value = 0.3
    ctrl.handle_joystick(make_msg(left_grip=0.8, left_y=0.0))
    vel.assert_called_once()
    tw = vel.call_args[0][0]
    assert abs(tw.linear.z) < 1e-9 and abs(tw.angular.y) < 1e-9

    # 俯仰走旋转组，回零清旋转粘性
    rot.reset_mock()
    clock.return_value = 0.4
    ctrl.handle_joystick(make_msg(left_grip=0.8, left_x=1.0))
    assert rot.call_count == 1
    assert abs(rot.call_args[0][0].angular.y - (-SCALE_PITCH)) < 1e-9

    rot.reset_mock()
    clock.return_value = 0.5
    ctrl.handle_joystick(make_msg(left_grip=0.8, left_x=0.0))
    rot.assert_called_once()
    assert abs(rot.call_args[0][0].angular.y) < 1e-9


def test_pitch_requires_larger_x_threshold():
    """侧扳机升降时，左右偏未达俯仰阈值仍走升降，不抢旋转组。"""
    ctrl, _, vel, rot, _, clock = make_controller(t0=0.0)
    ctrl.handle_joystick(make_msg(left_grip=0.8))
    vel.reset_mock()
    rot.reset_mock()

    # |x| 介于普通死区与俯仰阈值之间：只升降
    mid_x = 0.5 * (0.1 + STICK_PITCH_AXIS_THRESHOLD)
    assert 0.1 < mid_x < STICK_PITCH_AXIS_THRESHOLD
    clock.return_value = 0.1
    ctrl.handle_joystick(make_msg(left_grip=0.8, left_y=1.0, left_x=mid_x))
    vel.assert_called_once()
    assert abs(vel.call_args[0][0].linear.z - SCALE_HEIGHT) < 1e-9
    rot.assert_not_called()

    # 超过俯仰阈值：切入旋转组（俯仰优先）；切组不向旧平移话题补零
    vel.reset_mock()
    rot.reset_mock()
    clock.return_value = 0.2
    ctrl.handle_joystick(make_msg(left_grip=0.8, left_y=1.0, left_x=1.0))
    vel.assert_not_called()
    rot.assert_called_once()
    assert abs(rot.call_args[0][0].angular.y - (-SCALE_PITCH)) < 1e-9


def test_yaw_buttons_oneshot_delta_edge():
    """按钮边沿 oneshot，delta 受 YAW_BUTTON_LIMIT 钳制；按住不连发；错手忽略。

    无 state_getter 时 ol_yaw 回退为 0，首按 delta = ±YAW_BUTTON_LIMIT（≈±177°）。
    YAW_TARGET 已与 YAW_BUTTON_LIMIT 对齐，保证往返对称可逆（0→+177°→0→−177°→0）。
    """
    ctrl, _, _, _, delta, _ = make_controller()
    ctrl.handle_joystick(make_msg(left_grip=0.8))
    delta.reset_mock()

    ctrl.handle_joystick(make_msg(left_grip=0.8, left_first_button_pressed=True))
    ctrl.handle_joystick(make_msg(left_grip=0.8, left_first_button_pressed=True))  # hold
    assert delta.call_count == 1
    assert abs(delta.call_args[0][0].angular.z - (-YAW_BUTTON_LIMIT)) < 1e-9

    ctrl.handle_joystick(make_msg(left_grip=0.8))  # release
    ctrl.handle_joystick(make_msg(left_grip=0.8, left_second_button_pressed=True))
    assert delta.call_count == 2
    assert abs(delta.call_args[0][0].angular.z - YAW_BUTTON_LIMIT) < 1e-9

    delta.reset_mock()
    ctrl.handle_joystick(make_msg(left_grip=0.8, right_first_button_pressed=True))
    delta.assert_not_called()


def test_yaw_button_clamp_at_limit():
    """到限后连按同方向 → 无响应；反方向可拉回。"""
    ol_yaw = YAW_BUTTON_LIMIT  # 已在上限

    def state_getter():
        return (0.0, 0.0, ol_yaw, 0.0)  # (x, z, yaw, pitch)

    ctrl, _, _, _, delta, _ = make_controller_with_state(state_getter=state_getter)
    ctrl.handle_joystick(make_msg(left_grip=0.8))
    delta.reset_mock()

    # 同方向（second button → +π）：已在限位 → delta=0 → 无发布
    ctrl.handle_joystick(make_msg(left_grip=0.8, left_second_button_pressed=True))
    delta.assert_not_called()

    # 反方向（first button → -π）：可拉回
    ctrl.handle_joystick(make_msg(left_grip=0.8))
    ctrl.handle_joystick(make_msg(left_grip=0.8, left_first_button_pressed=True))
    assert delta.call_count == 1
    assert delta.call_args[0][0].angular.z < 0.0  # direction toward zero

    # 下限同理
    state_getter_dn = lambda: (0.0, 0.0, -YAW_BUTTON_LIMIT, 0.0)
    ctrl2, _, _, _, delta2, _ = make_controller_with_state(state_getter=state_getter_dn)
    ctrl2.handle_joystick(make_msg(left_grip=0.8))
    delta2.reset_mock()

    ctrl2.handle_joystick(make_msg(left_grip=0.8, left_first_button_pressed=True))
    delta2.assert_not_called()  # 同方向 → 无响应

    ctrl2.handle_joystick(make_msg(left_grip=0.8))
    ctrl2.handle_joystick(make_msg(left_grip=0.8, left_second_button_pressed=True))
    assert delta2.call_count == 1
    assert delta2.call_args[0][0].angular.z > 0.0  # reverse direction


# ---- 复位 ----

def test_execute_reset_success_and_failure():
    ok = MagicMock(return_value=0.5)
    ctrl, _, _, _, _, _ = make_controller(resetter=ok)
    ctrl.main_hand = "left"
    ctrl._first_btn_was_pressed = True
    assert ctrl._execute_reset() == 0.5
    assert ctrl.main_hand is None
    assert ctrl._first_btn_was_pressed is False

    bad = MagicMock(return_value=0.0)
    ctrl, _, _, _, _, _ = make_controller(resetter=bad)
    ctrl.main_hand = "left"
    assert ctrl._execute_reset() is None
    assert ctrl.main_hand == "left"

    boom = MagicMock(side_effect=RuntimeError("down"))
    ctrl, _, _, _, _, _ = make_controller(resetter=boom)
    ctrl.main_hand = "right"
    assert ctrl._execute_reset() is None
    assert ctrl.main_hand == "right"

    # True → 成功但 busy 时长 0（勿 float(True)==1）
    truthy = MagicMock(return_value=True)
    ctrl, _, _, _, _, _ = make_controller(resetter=truthy)
    ctrl.main_hand = "left"
    assert ctrl._execute_reset() == 0.0


def test_double_click_reset_success_and_guards():
    """双击复位成功；摇杆非零 / 超时 / 长按 均不触发。"""
    resetter = MagicMock(return_value=0.5)
    ctrl, pub, _, _, _, clock = make_controller(t0=0.0, resetter=resetter)
    _double_click_reset(ctrl, clock)
    resetter.assert_called_once()
    assert ctrl.main_hand is None
    pub.assert_not_called()  # 绝对零位延后到复位窗口结束
    assert ctrl._pending_post_reset_pose is True

    clock.return_value = 0.82  # t0+0.32 + reset_duration 0.5
    ctrl.handle_joystick(make_msg())
    pub.assert_called_once()
    tw = pub.call_args[0][0]
    assert abs(tw.linear.x) < 1e-9 and abs(tw.linear.z) < 1e-9
    assert ctrl._pending_post_reset_pose is False

    # 摇杆非零拒绝
    resetter.reset_mock()
    ctrl, _, _, _, _, clock = make_controller(t0=0.0, resetter=resetter)
    clock.return_value = 0.0
    ctrl.handle_joystick(make_msg(left_grip=0.8, left_y=0.2))
    clock.return_value = 0.10
    ctrl.handle_joystick(make_msg(left_grip=0.0, left_y=0.2))
    clock.return_value = 0.25
    ctrl.handle_joystick(make_msg(left_grip=0.8, left_y=0.2))
    clock.return_value = 0.32
    ctrl.handle_joystick(make_msg(left_grip=0.0, left_y=0.2))
    resetter.assert_not_called()

    # 双击窗口超时
    resetter.reset_mock()
    ctrl, _, _, _, _, clock = make_controller(t0=0.0, resetter=resetter)
    clock.return_value = 0.0
    ctrl.handle_joystick(make_msg(left_grip=0.8))
    clock.return_value = 0.10
    ctrl.handle_joystick(make_msg(left_grip=0.0))
    clock.return_value = 0.80  # > window
    ctrl.handle_joystick(make_msg(left_grip=0.8))
    clock.return_value = 0.87
    ctrl.handle_joystick(make_msg(left_grip=0.0))
    resetter.assert_not_called()

    # 长按不是双击
    resetter.reset_mock()
    ctrl, _, _, _, _, clock = make_controller(t0=0.0, resetter=resetter)
    clock.return_value = 0.0
    ctrl.handle_joystick(make_msg(left_grip=0.8))
    clock.return_value = 0.40
    ctrl.handle_joystick(make_msg(left_grip=0.8))
    clock.return_value = 0.45
    ctrl.handle_joystick(make_msg(left_grip=0.0))
    resetter.assert_not_called()


def test_reset_busy_blocks_control_and_uses_service_return_time():
    resetter = MagicMock(return_value=0.5)
    ctrl, _, vel, _, _, clock = make_controller(t0=0.0, resetter=resetter)
    _double_click_reset(ctrl, clock)
    # 完成时刻 0.32 + 估计 0.5
    assert abs(ctrl._reset_busy_until - 0.82) < 1e-9
    clock.return_value = 0.40
    assert ctrl.is_reset_busy() is True

    vel.reset_mock()
    ctrl.handle_joystick(make_msg(left_grip=0.8, left_y=1.0))
    assert ctrl.main_hand is None
    vel.assert_not_called()

    clock.return_value = 0.83
    assert ctrl.is_reset_busy() is False
    ctrl.handle_joystick(make_msg(left_grip=0.0))
    clock.return_value = 0.84
    ctrl.handle_joystick(make_msg(left_grip=0.8))
    assert ctrl.main_hand == "left"

    # 服务耗时：busy 从返回时刻起算
    ctrl, _, _, _, _, clock = make_controller(t0=0.0)

    def _slow():
        clock.return_value = 1.30
        return 0.5

    ctrl._resetter = MagicMock(side_effect=_slow)
    _double_click_reset(ctrl, clock)
    assert abs(ctrl._reset_busy_until - 1.80) < 1e-9
    assert abs(ctrl._reset_cooldown_until - 2.30) < 1e-9

def test_group_switch_does_not_zero_old_topic():
    """height↔pitch 切组只发新组非零；旧话题不补零；松 grip 仍清当前组。"""
    ctrl, _, vel, rot, delta, clock = make_controller(t0=0.0)
    ctrl.handle_joystick(make_msg(left_grip=0.8))

    # 先升降
    clock.return_value = 0.1
    vel.reset_mock()
    rot.reset_mock()
    ctrl.handle_joystick(make_msg(left_grip=0.8, left_y=1.0))
    vel.assert_called_once()
    rot.assert_not_called()
    assert ctrl._latched_trans_vel_active is True

    # 切俯仰：不得向 /cmd_torso_vel 发零
    clock.return_value = 0.2
    vel.reset_mock()
    rot.reset_mock()
    ctrl.handle_joystick(make_msg(left_grip=0.8, left_x=1.0))
    vel.assert_not_called()
    rot.assert_called_once()
    assert abs(rot.call_args[0][0].angular.y - (-SCALE_PITCH)) < 1e-9
    assert ctrl._latched_trans_vel_active is False
    assert ctrl._latched_rot_vel_active is True

    # 再切回升降：不得向 /cmd_torso_rotation_vel 发零
    clock.return_value = 0.3
    vel.reset_mock()
    rot.reset_mock()
    ctrl.handle_joystick(make_msg(left_grip=0.8, left_y=1.0))
    rot.assert_not_called()
    vel.assert_called_once()
    assert abs(vel.call_args[0][0].linear.z - SCALE_HEIGHT) < 1e-9
    assert ctrl._latched_rot_vel_active is False
    assert ctrl._latched_trans_vel_active is True

    # 松 grip：_clear_latched_velocities 向当前（平移）组发零
    clock.return_value = 0.4
    vel.reset_mock()
    rot.reset_mock()
    delta.reset_mock()
    ctrl.handle_joystick(make_msg(left_grip=0.0))
    vel.assert_called_once()
    tw0 = vel.call_args[0][0]
    assert abs(tw0.linear.z) < 1e-9 and abs(tw0.angular.y) < 1e-9
    rot.assert_not_called()
    delta.assert_not_called()
    assert ctrl._latched_trans_vel_active is False
    assert ctrl.main_hand is None


def test_yaw_while_pitch_publishes_oneshot_delta():
    """持续俯仰时按 yaw：同帧仍发 rotation_vel（仅 pitch），并 oneshot /cmd_torso_delta。

    完成量交给 RM 在旋转组内累计；本节点不再用 dt 扣减剩余量，也不把 yaw 并入 rotation_vel。
    """
    ctrl, _, vel, rot, delta, clock = make_controller(t0=0.0)
    ctrl.handle_joystick(make_msg(left_grip=0.8))

    # 持续俯仰
    clock.return_value = 0.1
    ctrl.handle_joystick(make_msg(left_grip=0.8, left_x=1.0))
    assert ctrl._latched_rot_vel_active is True

    vel.reset_mock()
    rot.reset_mock()
    delta.reset_mock()

    # 同帧保持俯仰 + yaw 边沿：rotation_vel 只带 pitch；delta 一次发满有效 yaw
    clock.return_value = 0.12
    ctrl.handle_joystick(make_msg(
        left_grip=0.8, left_x=1.0, left_second_button_pressed=True))
    vel.assert_not_called()
    rot.assert_called_once()
    tw = rot.call_args[0][0]
    assert abs(tw.angular.y - (-SCALE_PITCH)) < 1e-9
    assert abs(tw.angular.z) < 1e-9  # yaw 不再走速度话题
    delta.assert_called_once()
    assert abs(delta.call_args[0][0].angular.z - YAW_BUTTON_LIMIT) < 1e-9

    # 按住不连发；不规则间隔继续俯仰时仍只发 rotation_vel，不重复 delta
    for t in (0.14, 0.19, 0.31, 0.33):
        vel.reset_mock()
        rot.reset_mock()
        delta.reset_mock()
        clock.return_value = t
        ctrl.handle_joystick(make_msg(
            left_grip=0.8, left_x=1.0, left_second_button_pressed=True))
        delta.assert_not_called()
        vel.assert_not_called()
        rot.assert_called_once()
        tw = rot.call_args[0][0]
        assert abs(tw.angular.y - (-SCALE_PITCH)) < 1e-9
        assert abs(tw.angular.z) < 1e-9

    # 松 yaw 键后继续俯仰：仍无额外 delta，旋转组保持
    clock.return_value = 0.40
    vel.reset_mock()
    rot.reset_mock()
    delta.reset_mock()
    ctrl.handle_joystick(make_msg(left_grip=0.8, left_x=1.0))
    delta.assert_not_called()
    vel.assert_not_called()
    rot.assert_called_once()
    assert abs(rot.call_args[0][0].angular.y - (-SCALE_PITCH)) < 1e-9


def test_yaw_oneshot_near_button_limit_while_pitching():
    """接近 ±YAW_BUTTON_LIMIT 时俯仰中按键：只发剩余小增量 oneshot，不越界、不残留。"""
    ol_yaw = YAW_BUTTON_LIMIT - 0.001
    state_getter = lambda: (0.0, 0.0, ol_yaw, 0.1)
    ctrl, _, vel, rot, delta, clock = make_controller_with_state(
        t0=0.0, state_getter=state_getter)
    ctrl.handle_joystick(make_msg(left_grip=0.8))
    clock.return_value = 0.1
    ctrl.handle_joystick(make_msg(left_grip=0.8, left_x=1.0))

    vel.reset_mock()
    rot.reset_mock()
    delta.reset_mock()
    clock.return_value = 0.12
    ctrl.handle_joystick(make_msg(
        left_grip=0.8, left_x=1.0, left_second_button_pressed=True))
    delta.assert_called_once()
    d = delta.call_args[0][0].angular.z
    assert abs(d - 0.001) < 1e-9
    assert ol_yaw + d <= YAW_BUTTON_LIMIT + 1e-12
    rot.assert_called_once()
    assert abs(rot.call_args[0][0].angular.z) < 1e-9

    # 已贴上限后再按同方向：无 delta
    state_getter2 = lambda: (0.0, 0.0, YAW_BUTTON_LIMIT, 0.1)
    ctrl2, _, _, rot2, delta2, clock2 = make_controller_with_state(
        t0=0.0, state_getter=state_getter2)
    ctrl2.handle_joystick(make_msg(left_grip=0.8))
    clock2.return_value = 0.1
    ctrl2.handle_joystick(make_msg(left_grip=0.8, left_x=1.0))
    delta2.reset_mock()
    rot2.reset_mock()
    clock2.return_value = 0.12
    ctrl2.handle_joystick(make_msg(
        left_grip=0.8, left_x=1.0, left_second_button_pressed=True))
    delta2.assert_not_called()
    rot2.assert_called_once()

    # 负向上限对称
    ol_yaw_n = -YAW_BUTTON_LIMIT + 0.001
    state_getter_n = lambda: (0.0, 0.0, ol_yaw_n, 0.1)
    ctrl3, _, _, _, delta3, clock3 = make_controller_with_state(
        t0=0.0, state_getter=state_getter_n)
    ctrl3.handle_joystick(make_msg(left_grip=0.8))
    clock3.return_value = 0.1
    ctrl3.handle_joystick(make_msg(left_grip=0.8, left_x=1.0))
    delta3.reset_mock()
    clock3.return_value = 0.12
    ctrl3.handle_joystick(make_msg(
        left_grip=0.8, left_x=1.0, left_first_button_pressed=True))
    delta3.assert_called_once()
    assert abs(delta3.call_args[0][0].angular.z - (-0.001)) < 1e-9

