#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""轮臂末端力控通用接口封装。"""

import rospy
from geometry_msgs.msg import Wrench, WrenchStamped
from std_msgs.msg import Bool, Header

try:
    from kuavo_msgs.srv import setContactForceInterpParams, setContactForceInterpParamsRequest  # type: ignore
except ImportError:
    setContactForceInterpParams = None
    setContactForceInterpParamsRequest = None


GRAVITY = 9.8
VALID_HANDS = ("left", "right", "both")


def _normalize_hand(hand):
    if hand not in VALID_HANDS:
        raise ValueError("hand must be one of: left, right, both")
    return hand


def _normalize_vector(vec):
    if vec is None:
        return (0.0, 0.0, 0.0)
    if len(vec) != 3:
        raise ValueError("vector must contain exactly 3 values")
    return tuple(float(v) for v in vec)


def _build_wrench(force=None, torque=None):
    force = _normalize_vector(force)
    torque = _normalize_vector(torque)
    msg = Wrench()
    msg.force.x, msg.force.y, msg.force.z = force
    msg.torque.x, msg.torque.y, msg.torque.z = torque
    return msg


def _build_wrench_stamped(force=None, torque=None, frame_id="base_link"):
    msg = WrenchStamped()
    msg.header = Header(stamp=rospy.Time.now(), frame_id=frame_id)
    msg.wrench = _build_wrench(force=force, torque=torque)
    return msg


class LBForceController(object):
    """统一封装左右手期望力、仿真外力和挥空检测开关。"""

    def __init__(
        self,
        wait_for_connection=False,
        connection_delay=0.1,
        connection_timeout=2.0,
        publish_repeat=3,
        publish_interval=0.05,
    ):
        self.pub_desired_left = rospy.Publisher("/desired_ee_force/left", WrenchStamped, queue_size=10)
        self.pub_desired_right = rospy.Publisher("/desired_ee_force/right", WrenchStamped, queue_size=10)
        self.pub_external_left = rospy.Publisher("/external_wrench/left_hand", Wrench, queue_size=10)
        self.pub_external_right = rospy.Publisher("/external_wrench/right_hand", Wrench, queue_size=10)
        self.pub_force_empty_detact = rospy.Publisher("/enable_force_empty_detact", Bool, queue_size=10, latch=True)
        self.connection_timeout = max(0.0, float(connection_timeout))
        self.publish_repeat = max(1, int(publish_repeat))
        self.publish_interval = max(0.0, float(publish_interval))

        if wait_for_connection and connection_delay > 0.0:
            rospy.sleep(connection_delay)
            self.wait_for_subscribers(timeout=self.connection_timeout)

    def _wait_for_publisher_connections(self, publisher, topic_name, timeout=None):
        timeout = self.connection_timeout if timeout is None else max(0.0, float(timeout))
        deadline = rospy.Time.now() + rospy.Duration.from_sec(timeout)
        warned = False

        while not rospy.is_shutdown() and publisher.get_num_connections() == 0:
            if timeout > 0.0 and rospy.Time.now() > deadline:
                rospy.logwarn("No subscribers connected to %s within %.2fs", topic_name, timeout)
                return False
            if not warned:
                rospy.loginfo("Waiting for subscribers on %s ...", topic_name)
                warned = True
            rospy.sleep(0.05)
        return publisher.get_num_connections() > 0

    def wait_for_subscribers(self, timeout=None):
        desired_left = self._wait_for_publisher_connections(self.pub_desired_left, "/desired_ee_force/left", timeout)
        desired_right = self._wait_for_publisher_connections(self.pub_desired_right, "/desired_ee_force/right", timeout)
        return desired_left and desired_right

    def _publish_with_retry(self, publisher, msg):
        for index in range(self.publish_repeat):
            publisher.publish(msg)
            if index < self.publish_repeat - 1 and self.publish_interval > 0.0:
                rospy.sleep(self.publish_interval)

    def _publish_desired_single(self, hand, force=None, torque=None, frame_id="base_link"):
        msg = _build_wrench_stamped(force=force, torque=torque, frame_id=frame_id)
        publisher = self.pub_desired_left if hand == "left" else self.pub_desired_right
        topic_name = "/desired_ee_force/left" if hand == "left" else "/desired_ee_force/right"
        self._wait_for_publisher_connections(publisher, topic_name)
        self._publish_with_retry(publisher, msg)
        rospy.loginfo(
            "Published desired EE force to %s: force=%s N torque=%s Nm frame_id=%s",
            hand,
            _normalize_vector(force),
            _normalize_vector(torque),
            frame_id,
        )

    def set_desired_ee_force(self, hand, force=None, torque=None, frame_id="base_link"):
        hand = _normalize_hand(hand)
        if hand == "both":
            self.set_desired_ee_force_both(
                left_force=force,
                right_force=force,
                left_torque=torque,
                right_torque=torque,
                frame_id=frame_id,
            )
            return
        self._publish_desired_single(hand, force=force, torque=torque, frame_id=frame_id)

    def set_desired_ee_force_both(
        self,
        left_force=None,
        right_force=None,
        left_torque=None,
        right_torque=None,
        frame_id="base_link",
    ):
        self._publish_desired_single("left", force=left_force, torque=left_torque, frame_id=frame_id)
        self._publish_desired_single("right", force=right_force, torque=right_torque, frame_id=frame_id)

    def set_desired_ee_force_kg(self, hand, force_kg=None, torque=None, frame_id="base_link"):
        force_n = tuple(value * GRAVITY for value in _normalize_vector(force_kg))
        self.set_desired_ee_force(hand, force=force_n, torque=torque, frame_id=frame_id)

    def set_desired_ee_force_both_kg(
        self,
        left_force_kg=None,
        right_force_kg=None,
        left_torque=None,
        right_torque=None,
        frame_id="base_link",
    ):
        left_force_n = tuple(value * GRAVITY for value in _normalize_vector(left_force_kg))
        right_force_n = tuple(value * GRAVITY for value in _normalize_vector(right_force_kg))
        self.set_desired_ee_force_both(
            left_force=left_force_n,
            right_force=right_force_n,
            left_torque=left_torque,
            right_torque=right_torque,
            frame_id=frame_id,
        )

    def clear_desired_ee_force(self, hand=None, frame_id="base_link"):
        hand = "both" if hand is None else _normalize_hand(hand)
        self.set_desired_ee_force(hand, force=(0.0, 0.0, 0.0), torque=(0.0, 0.0, 0.0), frame_id=frame_id)

    def _publish_external_single(self, hand, force=None, torque=None):
        msg = _build_wrench(force=force, torque=torque)
        publisher = self.pub_external_left if hand == "left" else self.pub_external_right
        topic_name = "/external_wrench/left_hand" if hand == "left" else "/external_wrench/right_hand"
        self._wait_for_publisher_connections(publisher, topic_name)
        self._publish_with_retry(publisher, msg)
        rospy.loginfo(
            "Published external wrench to %s: force=%s N torque=%s Nm",
            hand,
            _normalize_vector(force),
            _normalize_vector(torque),
        )

    def set_external_wrench(self, hand, force=None, torque=None):
        hand = _normalize_hand(hand)
        if hand == "both":
            self.set_external_wrench_both(
                left_force=force,
                right_force=force,
                left_torque=torque,
                right_torque=torque,
            )
            return
        self._publish_external_single(hand, force=force, torque=torque)

    def set_external_wrench_both(self, left_force=None, right_force=None, left_torque=None, right_torque=None):
        self._publish_external_single("left", force=left_force, torque=left_torque)
        self._publish_external_single("right", force=right_force, torque=right_torque)

    def clear_external_wrench(self, hand=None):
        hand = "both" if hand is None else _normalize_hand(hand)
        self.set_external_wrench(hand, force=(0.0, 0.0, 0.0), torque=(0.0, 0.0, 0.0))

    def enable_force_empty_detact(self, enabled):
        enabled = bool(enabled)
        self._publish_with_retry(self.pub_force_empty_detact, Bool(data=enabled))
        rospy.loginfo("Force empty detact set to %s", enabled)

    def set_contact_force_params(self, transition_time, interpolation_speed, timeout=2.0):
        if setContactForceInterpParams is None or setContactForceInterpParamsRequest is None:
            rospy.logwarn("set_contact_force_params service type is unavailable in current environment")
            return False

        try:
            rospy.wait_for_service("/set_contact_force_params", timeout=timeout)
            request = setContactForceInterpParamsRequest()
            request.transition_time = float(transition_time)
            request.interpolation_speed = float(interpolation_speed)
            rospy.ServiceProxy("/set_contact_force_params", setContactForceInterpParams)(request)
            rospy.loginfo(
                "Updated contact force params: transition_time=%.3f interpolation_speed=%.3f",
                request.transition_time,
                request.interpolation_speed,
            )
            return True
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logwarn("Failed to set contact force params: %s", exc)
            return False
