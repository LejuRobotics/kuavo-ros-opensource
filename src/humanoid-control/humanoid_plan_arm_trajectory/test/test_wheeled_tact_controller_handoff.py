#!/usr/bin/env python3
"""#5096: wheel-arm tact playback must not stall on a missing controller list.

On Kuavo 5W (ROBOT_VERSION=62, default KUAVO_CONTROL_SCHEME=multi),
/humanoid_controller/get_controller_list is not advertised, so the current
controller stays None. The early execute-action gate already skips this for
is_wheeled and continues the OCS2 path; ensure_tact_playback_controller must
do the same, otherwise playback is cancelled after ~3s with
「动作控制器交接未就绪」.

These tests stub ROS modules so they can run without a sourced workspace.
They also replay the bag-derived G12 sequence from kuavodevlab#5096 offline
(the bag itself is not required). A full live /execute_arm_action bag play
is not attempted here.
"""
import importlib.util
import sys
import threading
import time
import types
import unittest
from pathlib import Path
from unittest.mock import MagicMock


BAG_BAOQUAN_RESOLVED_TO_FAIL_SEC = 3.041  # 13:54:12.682 state=1 -> 13:54:15.723 WARN
BAG_DIANZAN_RESOLVED_TO_FAIL_SEC = 3.011  # 13:54:22.864 state=1 -> 13:54:25.875 WARN


def _attr_module(name):
    class AttrModule(types.ModuleType):
        def __getattr__(self, item):
            if item.startswith('__'):
                raise AttributeError(item)
            value = MagicMock(name=item)
            setattr(self, item, value)
            return value

    return AttrModule(name)


def _install_ros_stubs():
    names = [
        'rospy', 'rospkg',
        'geometry_msgs', 'geometry_msgs.msg',
        'sensor_msgs', 'sensor_msgs.msg',
        'std_msgs', 'std_msgs.msg',
        'std_srvs', 'std_srvs.srv',
        'trajectory_msgs', 'trajectory_msgs.msg',
        'ocs2_msgs', 'ocs2_msgs.msg',
        'kuavo_msgs', 'kuavo_msgs.msg', 'kuavo_msgs.srv',
        'humanoid_plan_arm_trajectory',
        'humanoid_plan_arm_trajectory.msg',
        'humanoid_plan_arm_trajectory.srv',
    ]
    for name in names:
        if name not in sys.modules:
            sys.modules[name] = _attr_module(name)

    class DummyRosPack:
        def get_path(self, _name):
            raise ImportError('no ros pack in unit test')

    rospkg = sys.modules['rospkg']
    rospkg.RosPack = DummyRosPack
    rospkg.ResourceNotFound = type('ResourceNotFound', (Exception,), {})

    rospy = sys.modules['rospy']
    rospy.is_shutdown = lambda: False
    rospy.get_param = lambda *a, **k: k.get('default', a[1] if len(a) > 1 else None)
    rospy.loginfo = rospy.logwarn = rospy.logerr = rospy.logdebug = lambda *a, **k: None
    rospy.Time = MagicMock()
    rospy.Subscriber = MagicMock()
    rospy.Publisher = MagicMock()
    rospy.Service = MagicMock()
    rospy.ServiceProxy = MagicMock()
    rospy.wait_for_service = MagicMock()


def _load_bezier():
    _install_ros_stubs()
    script = Path(__file__).resolve().parents[1] / 'script' / 'arm_trajectory_bezier_process.py'
    spec = importlib.util.spec_from_file_location('wheeled_tact_handoff_bezier', script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


bezier = _load_bezier()


def _unfixed_ensure_tact_playback_controller(demo, timeout=3.0):
    """Replica of official beta logic before the wheeled fallback (a904fb975)."""
    if demo.kuavo_control_scheme != "multi":
        return True

    target_controller = "amp_controller"
    deadline = time.monotonic() + timeout
    switch_requested = False

    while time.monotonic() < deadline and not bezier.rospy.is_shutdown():
        if (demo.interrupt_flag or not demo.enable_control_state_
                or demo._controller_switch_abort_event.is_set()):
            return False
        current_controller = demo.get_current_controller_name()
        if current_controller is None:
            time.sleep(0.05)
            continue
        action = bezier.get_tact_playback_controller_action(
            demo.kuavo_control_scheme,
            demo.robot_class,
            demo.robot_version.major(),
            current_controller,
            demo._amp_family_active_member,
        )
        if current_controller == "amp_assemble_controller":
            if demo._amp_action_ready():
                return True
        elif action == "allowed":
            return True

        if not switch_requested:
            demo.request_stance()
            if not demo.call_switch_controller_service(target_controller):
                return False
            switch_requested = True

        time.sleep(0.05)

    return False


def _make_demo(scheme="multi", is_wheeled=True, controller=None, major=6):
    demo = bezier.ArmTrajectoryBezierDemo.__new__(bezier.ArmTrajectoryBezierDemo)
    demo.kuavo_control_scheme = scheme
    demo.is_wheeled = is_wheeled
    demo.interrupt_flag = False
    demo.enable_control_state_ = True
    demo._controller_switch_abort_event = threading.Event()
    demo._amp_family_active_member = ""
    demo.robot_class = bezier.KUAVO
    demo.robot_version = MagicMock()
    demo.robot_version.major.return_value = major
    demo.get_current_controller_name = lambda: controller
    demo.request_stance = MagicMock()
    demo.call_switch_controller_service = MagicMock(return_value=True)
    demo._amp_action_ready = MagicMock(return_value=False)
    return demo


class WheeledTactHandoffTest(unittest.TestCase):
    def test_unfixed_loop_rejects_wheeled_multi_when_controller_is_none(self):
        demo = _make_demo(scheme="multi", is_wheeled=True, controller=None)
        started = time.monotonic()
        allowed = _unfixed_ensure_tact_playback_controller(demo, timeout=0.3)
        elapsed = time.monotonic() - started
        self.assertFalse(allowed)
        self.assertGreaterEqual(elapsed, 0.3)
        demo.request_stance.assert_not_called()
        demo.call_switch_controller_service.assert_not_called()

    def test_fixed_method_allows_wheeled_multi_when_controller_is_none(self):
        demo = _make_demo(scheme="multi", is_wheeled=True, controller=None)
        started = time.monotonic()
        allowed = demo.ensure_tact_playback_controller(timeout=3.0)
        elapsed = time.monotonic() - started
        self.assertTrue(allowed)
        self.assertLess(elapsed, 0.2)
        demo.request_stance.assert_not_called()
        demo.call_switch_controller_service.assert_not_called()

    def test_biped_multi_still_waits_and_rejects_when_controller_is_none(self):
        demo = _make_demo(scheme="multi", is_wheeled=False, controller=None, major=5)
        started = time.monotonic()
        allowed = demo.ensure_tact_playback_controller(timeout=0.3)
        elapsed = time.monotonic() - started
        self.assertFalse(allowed)
        self.assertGreaterEqual(elapsed, 0.3)

    def test_biped_multi_amp_controller_still_allowed(self):
        demo = _make_demo(scheme="multi", is_wheeled=False, controller="amp_controller", major=5)
        self.assertTrue(demo.ensure_tact_playback_controller(timeout=0.3))

    def test_non_multi_scheme_still_short_circuits(self):
        demo = _make_demo(scheme="ocs2", is_wheeled=False, controller=None, major=5)
        self.assertTrue(demo.ensure_tact_playback_controller(timeout=0.3))

    def test_bag_derived_g12_sequence_is_allowed_after_fix(self):
        """Replay kuavodevlab#5096 G12 bag sequence: 5W + multi + controller None.

        Bag 2026-11-17-13-53-14_1.bag (Asia/Shanghai):
          13:54:12.657 joy_node Trigger customize_action_RR_A / 抱拳_45
          13:54:12.672 resolved to 抱拳 (robot 6.2.0), state=1 at 13:54:12.682
          13:54:15.723 WARN 当前控制器为 None，动作控制器交接未就绪 (~3.04s)
          13:54:22.810 Trigger customize_action_RR_C / 点赞_45
          13:54:22.864 state=1, 13:54:25.875 same WARN (~3.01s)

        The recorder whitelist missed /execute_arm_action request topics, so this
        is an offline decision-path replay, not a live ROS service playback.
        """
        self.assertAlmostEqual(BAG_BAOQUAN_RESOLVED_TO_FAIL_SEC, 3.0, delta=0.1)
        self.assertAlmostEqual(BAG_DIANZAN_RESOLVED_TO_FAIL_SEC, 3.0, delta=0.1)

        for action_name in ("抱拳_45", "点赞_45"):
            with self.subTest(action=action_name):
                demo = _make_demo(scheme="multi", is_wheeled=True, controller=None, major=6)
                self.assertFalse(
                    _unfixed_ensure_tact_playback_controller(demo, timeout=0.3),
                    "unfixed code should still reproduce the bag failure",
                )
                demo = _make_demo(scheme="multi", is_wheeled=True, controller=None, major=6)
                self.assertTrue(
                    demo.ensure_tact_playback_controller(timeout=3.0),
                    "fixed code must not reject wheeled playback for controller None",
                )

    def test_410062_is_treated_as_wheeled_by_constructor_flags(self):
        # Constructor flag coverage lives in test_heiman_v4_trajectory.py.
        # Here we only assert the handoff uses the is_wheeled flag, including
        # encoded wheel-arm versions once that flag is set.
        demo = _make_demo(scheme="multi", is_wheeled=True, controller=None, major=4)
        self.assertTrue(demo.ensure_tact_playback_controller(timeout=3.0))


if __name__ == '__main__':
    unittest.main()
