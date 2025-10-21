#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String

def gait_change_publisher():
    # 初始化 ROS 节点
    rospy.init_node('gait_change_publisher', anonymous=False)
    pub = rospy.Publisher('/humanoid_mpc_gait_change', String, queue_size=10)
    rospy.sleep(0.5)
    rate = rospy.Rate(0.2)  # 0.2 Hz, 即每 5 秒发布一次

    # 要发布的步态列表，自行增删
    gait_list = ['walk', 'stance']
    idx = 0

    rospy.loginfo("Starting to publish gait changes on /humanoid_mpc_gait_change ...")
    while not rospy.is_shutdown():
        gait = gait_list[idx % len(gait_list)]
        msg = String(data=gait)
        pub.publish(msg)
        rospy.loginfo(f"[gait_change_publisher] Published: {gait}")
        idx += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        gait_change_publisher()
    except rospy.ROSInterruptException:
        pass
