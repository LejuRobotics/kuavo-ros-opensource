"""单元测试：TorsoController（不依赖 rospy）。"""
import sys
import os
from types import SimpleNamespace
from unittest.mock import MagicMock

sys.path.insert(
    0,
    os.path.join(os.path.dirname(__file__), "..", "scripts"),
)

# 注入 geometry_msgs.msg 测试桩，避免 ROS 环境依赖
import sys as _sys
if "geometry_msgs.msg" not in _sys.modules:
    from types import ModuleType
    _stub = ModuleType("geometry_msgs.msg")
    class _Vec3:
        def __init__(self):
            self.x = self.y = self.z = 0.0
    class _Twist:
        def __init__(self):
            self.linear = _Vec3()
            self.angular = _Vec3()
    _stub.Twist = _Twist
    _sys.modules.setdefault("geometry_msgs", ModuleType("geometry_msgs"))
    _sys.modules["geometry_msgs.msg"] = _stub

from tools.torso_joystick_controller import (
    TorsoController,
    GRIP_PRESS_TH,
    GRIP_RELEASE_TH,
    STICK_DEAD_ZONE,
    SCALE_HEIGHT,
    SCALE_PITCH,
    Z_MIN,
    Z_MAX,
    PITCH_MIN,
    PITCH_MAX,
    YAW_TARGET,
    MAX_DT,
    MIN_DT,
)


def make_msg(**kwargs):
    """构造 JoySticks 风格 msg；未指定字段默认 0/False。"""
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


def make_controller(t0=0.0):
    """构造一个 TorsoController，附带可控时钟和 mock publisher。"""
    clock = MagicMock(return_value=t0)
    publisher = MagicMock()
    ctrl = TorsoController(
        initial_pose_xyz=(0.0, 0.0, 0.0),
        publisher=publisher,
        clock=clock,
    )
    return ctrl, publisher, clock


def test_idle_no_publish():
    """IDLE 状态下（无 grip 按下）不发布任何命令。"""
    ctrl, pub, _ = make_controller()
    ctrl.handle_joystick(make_msg())
    pub.assert_not_called()
    assert ctrl.main_hand is None


def test_grip_press_locks_main_hand():
    """左 grip 上升沿（0→0.8）→ main_hand 锁定为 left。"""
    ctrl, _, _ = make_controller()
    # 上一帧没人按 → 这一帧左 grip > 0.7
    ctrl.handle_joystick(make_msg(left_grip=0.8))
    assert ctrl.main_hand == "left"


def test_grip_second_hand_blocked_when_locked():
    """已锁定 left 后，right grip 按下也不抢占。"""
    ctrl, _, _ = make_controller()
    ctrl.handle_joystick(make_msg(left_grip=0.8))
    # 之后右手 grip 也按下
    ctrl.handle_joystick(make_msg(left_grip=0.8, right_grip=0.8))
    assert ctrl.main_hand == "left"


def test_grip_hysteresis_no_chatter():
    """grip 在 0.4（位于 release_th=0.3 与 press_th=0.7 之间）反复变化不抖动。"""
    ctrl, _, _ = make_controller()
    # 已经锁定
    ctrl.handle_joystick(make_msg(left_grip=0.8))
    assert ctrl.main_hand == "left"
    # 降到 0.4（>release_th，仍按住）
    ctrl.handle_joystick(make_msg(left_grip=0.4))
    assert ctrl.main_hand == "left"
    # 回到 0.6
    ctrl.handle_joystick(make_msg(left_grip=0.6))
    assert ctrl.main_hand == "left"


def test_grip_release_enters_holding():
    """主指手 grip 松开 → main_hand 置 None；不再发布。"""
    ctrl, pub, _ = make_controller()
    ctrl.handle_joystick(make_msg(left_grip=0.8))
    assert ctrl.main_hand == "left"
    pub.reset_mock()
    # 松开
    ctrl.handle_joystick(make_msg(left_grip=0.0))
    assert ctrl.main_hand is None
    pub.assert_not_called()


def test_non_main_hand_release_does_not_release():
    """非主指手的 grip 状态变化不影响主指手锁定。"""
    ctrl, _, _ = make_controller()
    # 左 grip 按下锁定
    ctrl.handle_joystick(make_msg(left_grip=0.8))
    # 同时右 grip 从 0 升到 0.8 再降到 0 → 左仍被锁定
    ctrl.handle_joystick(make_msg(left_grip=0.8, right_grip=0.8))
    ctrl.handle_joystick(make_msg(left_grip=0.8, right_grip=0.0))
    assert ctrl.main_hand == "left"


def test_holding_then_reactivate_can_switch_hand():
    """HOLDING 后任意手按 grip 都能重新进入 ACTIVE（可换手）。"""
    ctrl, _, _ = make_controller()
    ctrl.handle_joystick(make_msg(left_grip=0.8))
    ctrl.handle_joystick(make_msg(left_grip=0.0))
    assert ctrl.main_hand is None
    # 现在右手 grip 按下 → 锁定 right
    ctrl.handle_joystick(make_msg(right_grip=0.8))
    assert ctrl.main_hand == "right"


def test_stick_dead_zone():
    """|stick| < dead_zone 时 pose 不变。"""
    ctrl, _, clock = make_controller(t0=0.0)
    ctrl.handle_joystick(make_msg(left_grip=0.8))  # 锁定 + 设 _last_t
    z0 = ctrl.current_pose[2]
    p0 = ctrl.current_pose[4]
    # 0.05 < DEAD_ZONE=0.1
    clock.return_value = 0.1
    ctrl.handle_joystick(make_msg(left_grip=0.8, left_x=0.05, left_y=0.05))
    assert ctrl.current_pose[2] == z0
    assert ctrl.current_pose[4] == p0


def test_stick_increment_height():
    """stick.y=1.0, dt=0.1 → z 增加 SCALE_HEIGHT * 0.1。"""
    ctrl, _, clock = make_controller(t0=0.0)
    ctrl.handle_joystick(make_msg(left_grip=0.8))
    z0 = ctrl.current_pose[2]
    clock.return_value = 0.1
    ctrl.handle_joystick(make_msg(left_grip=0.8, left_y=1.0))
    expected = z0 + SCALE_HEIGHT * 0.1
    assert abs(ctrl.current_pose[2] - expected) < 1e-9


def test_stick_increment_pitch_negative_sign():
    """stick.x=1.0（推杆右）→ body_pitch 减小（前倾）。"""
    ctrl, _, clock = make_controller(t0=0.0)
    ctrl.handle_joystick(make_msg(left_grip=0.8))
    p0 = ctrl.current_pose[4]
    clock.return_value = 0.1
    ctrl.handle_joystick(make_msg(left_grip=0.8, left_x=1.0))
    expected = p0 - SCALE_PITCH * 0.1
    assert abs(ctrl.current_pose[4] - expected) < 1e-9


def test_stick_clamp_z_max():
    """持续推到 Z_MAX 后不再增加。"""
    ctrl, _, clock = make_controller(t0=0.0)
    ctrl.handle_joystick(make_msg(left_grip=0.8))
    # 大跨度推动直接超上限
    clock.return_value = 100.0
    ctrl.handle_joystick(make_msg(left_grip=0.8, left_y=1.0))
    # dt 被钳到 MAX_DT，单帧最多走 SCALE_HEIGHT * MAX_DT = 0.01 m
    # 累计推 200 次到 dt 钳上限仍会超 Z_MAX，被限幅
    for i in range(200):
        clock.return_value = 100.0 + (i + 1) * 0.1
        ctrl.handle_joystick(make_msg(left_grip=0.8, left_y=1.0))
    assert ctrl.current_pose[2] == ctrl.initial_xyz[2] + Z_MAX


def test_dt_clamp():
    """dt=1.0（卡顿）被钳到 MAX_DT=0.1，单帧 z 增量上限为 SCALE_HEIGHT * MAX_DT。"""
    ctrl, _, clock = make_controller(t0=0.0)
    ctrl.handle_joystick(make_msg(left_grip=0.8))
    z0 = ctrl.current_pose[2]
    clock.return_value = 1.0  # dt 名义 1.0 s
    ctrl.handle_joystick(make_msg(left_grip=0.8, left_y=1.0))
    expected = z0 + SCALE_HEIGHT * MAX_DT
    assert abs(ctrl.current_pose[2] - expected) < 1e-9


def test_last_t_reset_on_release():
    """主指手 grip 松开后再次激活，首帧 dt 应回到 MIN_DT（避免 stale _last_t）。"""
    ctrl, _, clock = make_controller(t0=0.0)
    # 第一次激活，推杆走一帧
    ctrl.handle_joystick(make_msg(left_grip=0.8))
    clock.return_value = 0.05
    ctrl.handle_joystick(make_msg(left_grip=0.8, left_y=1.0))
    # 松开
    clock.return_value = 0.06
    ctrl.handle_joystick(make_msg(left_grip=0.0))
    # 时钟跳到 100s 后重新激活
    clock.return_value = 100.0
    z_before = ctrl.current_pose[2]
    ctrl.handle_joystick(make_msg(left_grip=0.8))
    # 这一帧 _arbitrate 触发，main_hand 重锁，但首帧 dt 应是 MIN_DT
    # 紧接下一帧 dt = MAX_DT (不是 99.94)
    clock.return_value = 100.0  # 首帧无变化
    ctrl.handle_joystick(make_msg(left_grip=0.8, left_y=1.0))
    # 首帧增量 = 1.0 * SCALE_HEIGHT * MIN_DT = 0.10 * 0.001 = 0.0001
    expected = z_before + SCALE_HEIGHT * 0.001
    assert abs(ctrl.current_pose[2] - expected) < 1e-9


def test_button_a_decrements_yaw_by_pi():
    """主指手 first_button 按下边沿 → body_yaw 减 π（基于当前值的增量）。"""
    ctrl, _, _ = make_controller()
    ctrl.handle_joystick(make_msg(left_grip=0.8))
    ctrl.current_pose[5] = 0.3  # 预置一个非零基准
    ctrl.handle_joystick(make_msg(left_grip=0.8, left_first_button_pressed=True))
    assert abs(ctrl.current_pose[5] - (0.3 - YAW_TARGET)) < 1e-9


def test_button_b_increments_yaw_by_pi():
    """主指手 second_button 按下边沿 → body_yaw 加 π（基于当前值的增量）。"""
    ctrl, _, _ = make_controller()
    ctrl.handle_joystick(make_msg(left_grip=0.8))
    ctrl.current_pose[5] = 0.3
    ctrl.handle_joystick(make_msg(left_grip=0.8, left_second_button_pressed=True))
    assert abs(ctrl.current_pose[5] - (0.3 + YAW_TARGET)) < 1e-9


def test_button_ab_can_reset():
    """A 减 π 后再按 B 加 π → 回到原值（可复位）。"""
    ctrl, _, _ = make_controller()
    ctrl.handle_joystick(make_msg(left_grip=0.8))
    ctrl.current_pose[5] = 0.5
    # A 减 π
    ctrl.handle_joystick(make_msg(left_grip=0.8, left_first_button_pressed=True))
    # 松开 A（重置边沿）
    ctrl.handle_joystick(make_msg(left_grip=0.8))
    # B 加 π
    ctrl.handle_joystick(make_msg(left_grip=0.8, left_second_button_pressed=True))
    assert abs(ctrl.current_pose[5] - 0.5) < 1e-9


def test_button_edge_no_repeat():
    """按住 A 多帧只触发一次增量（边沿后续状态不再叠加）。"""
    ctrl, _, _ = make_controller()
    ctrl.handle_joystick(make_msg(left_grip=0.8))
    # 首次边沿：yaw 从 0 减 π
    ctrl.handle_joystick(make_msg(left_grip=0.8, left_first_button_pressed=True))
    yaw_after_edge = ctrl.current_pose[5]
    # 同一按键持续按下，不应再减 π
    ctrl.handle_joystick(make_msg(left_grip=0.8, left_first_button_pressed=True))
    assert ctrl.current_pose[5] == yaw_after_edge


def test_wrong_hand_stick_ignored():
    """main_hand='left' 时右摇杆动作被忽略。"""
    ctrl, _, clock = make_controller(t0=0.0)
    ctrl.handle_joystick(make_msg(left_grip=0.8))  # 锁 left
    z0 = ctrl.current_pose[2]
    clock.return_value = 0.1
    ctrl.handle_joystick(make_msg(left_grip=0.8, right_y=1.0))  # 右摇杆推满
    assert ctrl.current_pose[2] == z0  # 左摇杆是 0，z 不变


def test_wrong_hand_buttons_ignored():
    """main_hand='left' 时右手 A/B 不触发 yaw 跳转。"""
    ctrl, _, _ = make_controller()
    ctrl.handle_joystick(make_msg(left_grip=0.8))
    y0 = ctrl.current_pose[5]
    ctrl.handle_joystick(make_msg(left_grip=0.8, right_first_button_pressed=True))
    assert ctrl.current_pose[5] == y0


def test_publish_after_activation():
    """grip 按下后 publisher 被调用，参数包含当前位姿。"""
    ctrl, pub, clock = make_controller(t0=0.0)
    ctrl.handle_joystick(make_msg(left_grip=0.8))
    pub.assert_called()
    twist = pub.call_args[0][0]
    assert twist.linear.x == 0.0
    assert twist.linear.y == 0.0
    assert twist.linear.z == 0.0
    assert twist.angular.x == 0.0
    assert twist.angular.y == 0.0
    assert twist.angular.z == 0.0


def test_publish_reflects_stick_increment():
    """摇杆推动后发布的 Twist 反映了 height/pitch 变化。"""
    ctrl, pub, clock = make_controller(t0=0.0)
    ctrl.handle_joystick(make_msg(left_grip=0.8))
    pub.reset_mock()
    clock.return_value = 0.1
    ctrl.handle_joystick(make_msg(left_grip=0.8, left_y=1.0, left_x=1.0))
    twist = pub.call_args[0][0]
    assert abs(twist.linear.z - SCALE_HEIGHT * 0.1) < 1e-9
    assert abs(twist.angular.y - (-SCALE_PITCH * 0.1)) < 1e-9
