#!/usr/bin/env python3
"""Adapt Quest input to LeTools DAgger candidates; never publish actuator commands."""
from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass
class GripHysteresis:
    pressed: bool = False

    def update(self, value):
        if not math.isfinite(value):
            self.pressed = False
        elif self.pressed:
            if value <= 0.30:
                self.pressed = False
        elif value >= 0.70:
            self.pressed = True
        return self.pressed


class DaggerQuestInputAdapter:
    def __init__(self):
        import rospy
        from kuavo_msgs.msg import JoySticks
        from noitom_hi5_hand_udp_python.msg import PoseInfoList
        from geometry_msgs.msg import PoseStamped
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool

        self.left = GripHysteresis()
        self.right = GripHysteresis()
        self._Bool, self._JointState, self._PoseStamped = Bool, JointState, PoseStamped
        self.left_pub = rospy.Publisher('/dagger/vr/left_intervention', Bool, queue_size=1)
        self.right_pub = rospy.Publisher('/dagger/vr/right_intervention', Bool, queue_size=1)
        self.eef_pub = rospy.Publisher('/dagger/vr/eef_action', JointState, queue_size=1)
        self.body_pub = rospy.Publisher('/dagger/vr/body_pose', PoseStamped, queue_size=1)
        self.stop_pub = rospy.Publisher('/stop_robot', Bool, queue_size=1)
        self.stop_enabled = rospy.get_param('/enable_vr_stop_robot', True)
        self._stop_down = False
        self.joy_sub = rospy.Subscriber('/quest_joystick_data', JoySticks, self.dagger_joystick_callback,
                                        queue_size=1, tcp_nodelay=True)
        self.pose_sub = rospy.Subscriber('/leju_quest_bone_poses', PoseInfoList, self.dagger_pose_callback,
                                         queue_size=1, tcp_nodelay=True)

    def dagger_joystick_callback(self, msg):
        import rospy
        stop = bool(msg.left_first_button_pressed and msg.left_second_button_pressed)
        if self.stop_enabled and stop and not self._stop_down:
            self.stop_pub.publish(self._Bool(data=True))
        self._stop_down = stop
        left = self.left.update(float(msg.left_grip))
        right = self.right.update(float(msg.right_grip))
        self.left_pub.publish(self._Bool(data=left))
        self.right_pub.publish(self._Bool(data=right))
        values = [float(msg.left_trigger), float(msg.right_trigger)]
        if not all(math.isfinite(value) for value in values):
            return
        eef = self._JointState()
        eef.header.stamp = rospy.Time.now()
        eef.name = ['left_eef', 'right_eef']
        eef.position = [min(1.0, max(0.0, value)) for value in values]
        self.eef_pub.publish(eef)

    def dagger_pose_callback(self, msg):
        import rospy
        if len(msg.poses) <= 23:
            return
        chest = msg.poses[23]
        q = chest.orientation
        values = [q.x, q.y, q.z, q.w]
        if not all(math.isfinite(x) for x in values + [chest.position.x, chest.position.y, chest.position.z]):
            return
        norm = math.sqrt(sum(x*x for x in values))
        if norm < 1e-8:
            return
        pose = self._PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'quest_world'
        pose.pose.position = chest.position
        pose.pose.orientation.x = q.x / norm
        pose.pose.orientation.y = q.y / norm
        pose.pose.orientation.z = q.z / norm
        pose.pose.orientation.w = q.w / norm
        self.body_pub.publish(pose)


def main():
    import rospy
    rospy.init_node('dagger_quest_input_adapter')
    DaggerQuestInputAdapter()
    rospy.spin()


if __name__ == '__main__':
    main()
