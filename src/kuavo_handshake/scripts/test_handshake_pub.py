#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""测试发布 — base_link 系坐标

启动:
  source /home/lab/kuavo-ros-control/devel/setup.bash
  rosrun kuavo_handshake test_handshake_pub.py
"""

import rospy
from std_msgs.msg import Header
from kuavo_msgs.msg import HandshakeTarget

TEST_HAND = "right"
TEST_WRIST_X = 0.485
TEST_WRIST_Y = -0.072
TEST_WRIST_Z = 0.262  # base_link 系胸高, 自然握手位置


def main():
    rospy.init_node("test_handshake_pub")
    pub = rospy.Publisher("/handshake/target", HandshakeTarget, queue_size=5)
    rate = rospy.Rate(10)
    rospy.loginfo("/handshake/target hand=%s (%.2f %.2f %.2f)",
                  TEST_HAND, TEST_WRIST_X, TEST_WRIST_Y, TEST_WRIST_Z)

    for i in range(2):
        msg = HandshakeTarget()
        msg.header = Header(stamp=rospy.Time.now(), frame_id="base_link")
        msg.hand = TEST_HAND
        msg.wrist_x = TEST_WRIST_X
        msg.wrist_y = TEST_WRIST_Y
        msg.wrist_z = TEST_WRIST_Z
        pub.publish(msg)
        rospy.loginfo("Published msg %d", i + 1)
        rate.sleep()


if __name__ == "__main__":
    main()
