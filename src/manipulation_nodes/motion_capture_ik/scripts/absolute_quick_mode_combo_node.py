#!/usr/bin/env python3
"""Quest3 absolute-mode quick-control combo bridge.

Left front trigger + X uses a rising-edge toggle:
  normal (quickMode=0) -> upper-arm quick (quickMode=2)
  upper-arm quick (quickMode=2) -> normal (quickMode=0)

The node is only launched for Python absolute IK.  It never changes IK, PD,
MPC parameters, arm targets, interpolation, or low-pass parameters.
"""

import threading

import rospy
from kuavo_msgs.msg import JoySticks
from kuavo_msgs.srv import changeLbQuickModeSrv, changeLbQuickModeSrvRequest
from std_msgs.msg import Int8


MARKER = "ABSOLUTE_QUICK_COMBO_BETA_FDB60BBC_V1"


class AbsoluteQuickComboBridge:
    def __init__(self):
        self._trigger_threshold = float(
            rospy.get_param("~left_trigger_threshold", 0.8)
        )
        self._debounce_sec = max(0.05, float(rospy.get_param("~debounce_sec", 0.35)))
        self._service_name = rospy.get_param(
            "~service_name", "/enable_lb_arm_quick_mode"
        )

        self._lock = threading.Lock()
        self._combo_last = False
        self._quick_enabled = False
        self._last_edge_time = rospy.Time(0)
        self._shutting_down = False

        self._client = rospy.ServiceProxy(
            self._service_name, changeLbQuickModeSrv, persistent=False
        )
        self._state_pub = rospy.Publisher(
            "/wheel_arm_latency/absolute_quick_combo_state",
            Int8,
            queue_size=1,
            latch=True,
        )
        self._state_pub.publish(Int8(data=0))

        self._sub = rospy.Subscriber(
            "/quest_joystick_data",
            JoySticks,
            self._joystick_callback,
            queue_size=1,
            tcp_nodelay=True,
        )
        rospy.on_shutdown(self._on_shutdown)
        rospy.loginfo(
            "[%s] ready: left_trigger>%.2f + X toggles quickMode 0/2",
            MARKER,
            self._trigger_threshold,
        )

    def _request_mode(self, target_mode, quiet=False):
        try:
            self._client.wait_for_service(timeout=0.30)
        except rospy.ROSException:
            if not quiet:
                rospy.logwarn_throttle(
                    1.0, "[%s] service unavailable: %s", MARKER, self._service_name
                )
            return False

        request = changeLbQuickModeSrvRequest()
        request.quickMode = int(target_mode)
        try:
            response = self._client(request)
        except (rospy.ServiceException, rospy.ROSException) as exc:
            if not quiet:
                rospy.logerr("[%s] quick-mode service call failed: %s", MARKER, exc)
            return False

        if not response.success:
            if not quiet:
                rospy.logwarn(
                    "[%s] controller rejected quickMode=%d: %s",
                    MARKER,
                    target_mode,
                    response.message,
                )
            return False
        return True

    def _joystick_callback(self, msg):
        combo = (
            msg.left_trigger > self._trigger_threshold
            and bool(msg.left_first_button_pressed)  # Quest left controller X
        )

        with self._lock:
            rising_edge = combo and not self._combo_last
            self._combo_last = combo
            if not rising_edge or self._shutting_down:
                return

            now = rospy.Time.now()
            if (
                not self._last_edge_time.is_zero()
                and (now - self._last_edge_time).to_sec() < self._debounce_sec
            ):
                return
            self._last_edge_time = now
            target_mode = 0 if self._quick_enabled else 2

        # Service calls are made outside the state lock.
        if not self._request_mode(target_mode):
            return

        with self._lock:
            self._quick_enabled = target_mode == 2
            state = 2 if self._quick_enabled else 0

        self._state_pub.publish(Int8(data=state))
        rospy.logwarn(
            "[%s] left trigger + X: quickMode=%d (%s)",
            MARKER,
            state,
            "ABSOLUTE QUICK" if state == 2 else "NORMAL",
        )

    def _on_shutdown(self):
        with self._lock:
            if self._shutting_down:
                return
            self._shutting_down = True
            was_enabled = self._quick_enabled
            self._quick_enabled = False

        if was_enabled:
            self._request_mode(0, quiet=True)
        try:
            self._state_pub.publish(Int8(data=0))
        except Exception:
            pass


def main():
    rospy.init_node("absolute_quick_mode_combo")
    AbsoluteQuickComboBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
