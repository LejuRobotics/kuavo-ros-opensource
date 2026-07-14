"""单元测试：TorsoController（不依赖 rospy）。

语义：
  - 摇杆 → /cmd_torso_vel（瞬时速度，无上层开环）
  - yaw 边沿 → /cmd_torso_delta（oneshot）
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
    TorsoController,
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
    delta = MagicMock()
    ctrl = TorsoController(
        initial_pose_xyz=(0.0, 0.0, 0.0),
        publisher=pub,
        vel_publisher=vel,
        delta_publisher=delta,
        clock=clock,
        resetter=resetter,
    )
    return ctrl, pub, vel, delta, clock


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
    ctrl, pub, vel, delta, _ = make_controller(resetter=resetter)
    ctrl.handle_joystick(make_msg())
    pub.assert_not_called()
    vel.assert_not_called()
    delta.assert_not_called()
    resetter.assert_not_called()
    assert ctrl.main_hand is None


def test_grip_lock_hysteresis_and_hand_switch():
    ctrl, _, _, _, _ = make_controller()
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
    """满杆 → 瞬时速度；与 dt 无关；错手摇杆忽略。"""
    ctrl, pub, vel, _, clock = make_controller(t0=0.0)
    ctrl.handle_joystick(make_msg(left_grip=0.8))
    vel.reset_mock()
    clock.return_value = 1.0  # 大 dt 也不应改变速度幅值
    ctrl.handle_joystick(make_msg(left_grip=0.8, left_y=1.0, left_x=1.0))
    vel.assert_called_once()
    tw = vel.call_args[0][0]
    assert abs(tw.linear.z - SCALE_HEIGHT) < 1e-9
    assert abs(tw.angular.y - (-SCALE_PITCH)) < 1e-9
    assert abs(tw.linear.x) < 1e-9
    pub.assert_not_called()  # 连续控制不走绝对位姿

    # 错手：主手杆已回零会发一次 0 清 RM 粘性；不得出现右杆驱动的非零速度
    vel.reset_mock()
    ctrl.handle_joystick(make_msg(left_grip=0.8, right_y=1.0))
    if vel.called:
        tw0 = vel.call_args[0][0]
        assert abs(tw0.linear.z) < 1e-9 and abs(tw0.angular.y) < 1e-9
        assert abs(tw0.linear.z - SCALE_HEIGHT) > 0.01  # not right-stick height


def test_stick_deadzone_and_zero_clear_latch():
    """死区/零位：未发过非零则不发；发过非零后回零发一次 0 清 RM 粘性速度。"""
    ctrl, _, vel, _, clock = make_controller(t0=0.0)
    ctrl.handle_joystick(make_msg(left_grip=0.8))
    vel.reset_mock()
    clock.return_value = 0.1
    ctrl.handle_joystick(make_msg(left_grip=0.8, left_x=0.05, left_y=0.05))
    vel.assert_not_called()

    clock.return_value = 0.2
    ctrl.handle_joystick(make_msg(left_grip=0.8, left_y=1.0))
    assert vel.call_count == 1

    vel.reset_mock()
    clock.return_value = 0.3
    ctrl.handle_joystick(make_msg(left_grip=0.8, left_y=0.0))
    vel.assert_called_once()
    tw = vel.call_args[0][0]
    assert abs(tw.linear.z) < 1e-9 and abs(tw.angular.y) < 1e-9


def test_yaw_buttons_oneshot_delta_edge():
    """A/B 边沿 oneshot ±π；按住不连发；错手忽略。"""
    ctrl, _, _, delta, _ = make_controller()
    ctrl.handle_joystick(make_msg(left_grip=0.8))
    delta.reset_mock()

    ctrl.handle_joystick(make_msg(left_grip=0.8, left_first_button_pressed=True))
    ctrl.handle_joystick(make_msg(left_grip=0.8, left_first_button_pressed=True))  # hold
    assert delta.call_count == 1
    assert abs(delta.call_args[0][0].angular.z - (-YAW_TARGET)) < 1e-9

    ctrl.handle_joystick(make_msg(left_grip=0.8))  # release
    ctrl.handle_joystick(make_msg(left_grip=0.8, left_second_button_pressed=True))
    assert delta.call_count == 2
    assert abs(delta.call_args[0][0].angular.z - YAW_TARGET) < 1e-9

    delta.reset_mock()
    ctrl.handle_joystick(make_msg(left_grip=0.8, right_first_button_pressed=True))
    delta.assert_not_called()


# ---- 复位 ----

def test_execute_reset_success_and_failure():
    ok = MagicMock(return_value=0.5)
    ctrl, _, _, _, _ = make_controller(resetter=ok)
    ctrl.main_hand = "left"
    ctrl._first_btn_was_pressed = True
    assert ctrl._execute_reset() == 0.5
    assert ctrl.main_hand is None
    assert ctrl._first_btn_was_pressed is False

    bad = MagicMock(return_value=0.0)
    ctrl, _, _, _, _ = make_controller(resetter=bad)
    ctrl.main_hand = "left"
    assert ctrl._execute_reset() is None
    assert ctrl.main_hand == "left"

    boom = MagicMock(side_effect=RuntimeError("down"))
    ctrl, _, _, _, _ = make_controller(resetter=boom)
    ctrl.main_hand = "right"
    assert ctrl._execute_reset() is None
    assert ctrl.main_hand == "right"

    # True → 成功但 busy 时长 0（勿 float(True)==1）
    truthy = MagicMock(return_value=True)
    ctrl, _, _, _, _ = make_controller(resetter=truthy)
    ctrl.main_hand = "left"
    assert ctrl._execute_reset() == 0.0


def test_double_click_reset_success_and_guards():
    """双击复位成功；摇杆非零 / 超时 / 长按 均不触发。"""
    resetter = MagicMock(return_value=0.5)
    ctrl, pub, _, _, clock = make_controller(t0=0.0, resetter=resetter)
    _double_click_reset(ctrl, clock)
    resetter.assert_called_once()
    assert ctrl.main_hand is None
    pub.assert_called()  # 一次绝对零位
    tw = pub.call_args[0][0]
    assert abs(tw.linear.x) < 1e-9 and abs(tw.linear.z) < 1e-9

    # 摇杆非零拒绝
    resetter.reset_mock()
    ctrl, _, _, _, clock = make_controller(t0=0.0, resetter=resetter)
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
    ctrl, _, _, _, clock = make_controller(t0=0.0, resetter=resetter)
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
    ctrl, _, _, _, clock = make_controller(t0=0.0, resetter=resetter)
    clock.return_value = 0.0
    ctrl.handle_joystick(make_msg(left_grip=0.8))
    clock.return_value = 0.40
    ctrl.handle_joystick(make_msg(left_grip=0.8))
    clock.return_value = 0.45
    ctrl.handle_joystick(make_msg(left_grip=0.0))
    resetter.assert_not_called()


def test_reset_busy_blocks_control_and_uses_service_return_time():
    resetter = MagicMock(return_value=0.5)
    ctrl, _, vel, _, clock = make_controller(t0=0.0, resetter=resetter)
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
    ctrl, _, _, _, clock = make_controller(t0=0.0)

    def _slow():
        clock.return_value = 1.30
        return 0.5

    ctrl._resetter = MagicMock(side_effect=_slow)
    _double_click_reset(ctrl, clock)
    assert abs(ctrl._reset_busy_until - 1.80) < 1e-9
    assert abs(ctrl._reset_cooldown_until - 2.30) < 1e-9
