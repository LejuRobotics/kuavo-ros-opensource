#!/usr/bin/env python3
"""Adapt the two beta monitor streams to a single chest-relative control sample."""
import copy
import threading
import time

import numpy as np
import rospy
from kuavo_msgs.msg import JoySticks
from noitom_hi5_hand_udp_python.msg import PoseInfoList
from wheel_arm_abs_vel.msg import DualQuestSample
from quest_input_buffer import SourceClockGate
from tools.lightweight_transforms import quaternion_to_matrix, matrix_to_quaternion


class DualQuest:
    ARM = ((0, 1, 4, *range(6, 14)), (2, 3, 5, *range(14, 22)))
    JOY = ('x', 'y', 'trigger', 'grip', 'first_button_pressed', 'second_button_pressed',
           'first_button_touched', 'second_button_touched')

    def __init__(self, owner, joy_timeout=.5):
        if owner not in ('left', 'right'):
            raise ValueError('torso_control_side must be left or right')
        self.owner = int(owner == 'right')
        self.joy_timeout = joy_timeout
        self.joys = [None, None]
        self.bones = [None, None]
        self.stamps = [None, None]
        self.clocks = [SourceClockGate(), SourceClockGate()]
        self.last_output_ms = 0

    def joystick(self, side, msg, now):
        if msg is not None:
            self.joys[side] = (now, msg)
        changed = msg is not None
        result = JoySticks()
        for i, snapshot in enumerate(self.joys):
            if snapshot is None:
                continue
            received, value = snapshot
            if now - received > self.joy_timeout:
                self.joys[i] = None
                changed = True
                continue
            for f in self.JOY:
                name = ('left_' if i == 0 else 'right_') + f
                setattr(result, name, getattr(value, name))
        return result if changed else None

    @staticmethod
    def transform(pose):
        q = pose.orientation
        T = np.eye(4)
        T[:3,:3] = quaternion_to_matrix([q.x, q.y, q.z, q.w])
        T[:3,3] = [pose.position.x, pose.position.y, pose.position.z]
        if not np.isfinite(T).all():
            raise ValueError('non-finite Quest pose')
        return T

    def skeleton(self, side, msg, now):
        if msg.timestamp_ms <= 0 or len(msg.poses) < 24:
            self.bones[side] = None
            return
        previous = self.stamps[side]
        if previous and msg.timestamp_ms <= previous[0]:
            if now - previous[1] <= .5 or msg.timestamp_ms == previous[0]:
                return
            self.clocks[side].reset()
        if not self.clocks[side].accept(msg.timestamp_ms, now):
            return
        self.stamps[side] = (msg.timestamp_ms, now)
        self.bones[side] = (now, msg)
        if not all(self.bones):
            return
        # Beta updates each side independently and retains the other side's
        # latest pose. A skeleton gap is not a clutch operation: do not pair
        # frames or manufacture a per-arm HOLD/re-engagement edge from age.
        source = self.bones[1-self.owner][1]
        owner = self.bones[self.owner][1]
        if owner.is_hand_tracking != source.is_hand_tracking:
            return
        result = copy.deepcopy(owner)
        # The two rooms can have unrelated origins and headings. Place the
        # non-owner arm in the owner's chest frame before the common skeleton CV.
        transform = self.transform(owner.poses[23]) @ np.linalg.inv(self.transform(source.poses[23]))
        for i in self.ARM[1-self.owner]:
            pose = result.poses[i]
            mapped = transform @ self.transform(source.poses[i])
            pose.position.x, pose.position.y, pose.position.z = mapped[:3,3]
            q = pose.orientation
            q.x, q.y, q.z, q.w = matrix_to_quaternion(mapped[:3,:3])
        result.is_high_confidence = owner.is_high_confidence and source.is_high_confidence
        # Never compare timestamps across devices. The common processing clock
        # is continuous when the torso owner changes availability.
        stamp = int(1000*now)
        if stamp <= self.last_output_ms:
            return
        self.last_output_ms = result.timestamp_ms = stamp
        packet = DualQuestSample()
        packet.skeleton = result
        packet.source_time_ms = [m.timestamp_ms for _,m in self.bones]
        packet.side_available = [True, True]
        packet.owner_chest_fresh = now - self.bones[self.owner][0] + self.clocks[self.owner].relative_age <= .5
        packet.assembled_at = rospy.Time.now()
        return packet


def main():
    rospy.init_node('incremental_dual_quest')
    dual = DualQuest(rospy.get_param('~torso_control_side', 'left'),
                     rospy.get_param('~joystick_timeout', .5))
    joys = rospy.Publisher('/quest_joystick_data', JoySticks, queue_size=1)
    bones = rospy.Publisher('/whole_body_velocity_ik/dual_quest_sample', DualQuestSample, queue_size=1)
    lock = threading.Lock()

    def emit(fn, pub, side, msg):
        with lock:
            try:
                result = fn(side, msg, time.monotonic())
            except (ValueError, np.linalg.LinAlgError) as exc:
                rospy.logwarn_throttle(1., 'Invalid dual Quest sample: %s', exc)
                return
            if result is not None:
                pub.publish(result)

    for i, side in enumerate(('left', 'right')):
        ns = '/vr_' + side
        rospy.Subscriber(ns + '/quest_joystick_data', JoySticks,
                         lambda m, i=i: emit(dual.joystick, joys, i, m), queue_size=1, tcp_nodelay=True)
        rospy.Subscriber(ns + '/leju_quest_bone_poses', PoseInfoList,
                         lambda m, i=i: emit(dual.skeleton, bones, i, m), queue_size=1, tcp_nodelay=True)
    rospy.Timer(rospy.Duration(.05), lambda _: emit(dual.joystick, joys, 0, None))
    rospy.spin()


if __name__ == '__main__':
    main()
