#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from kuavo_msgs.msg import armTargetPoses


def publish_arm_target_poses(times, values):
    """
    发布手臂目标姿态到指定话题。
    :param pub: Publisher对象
    :param times: 时间列表
    :param values: 关节角度列表
    """
    # 创建Publisher对象
    pub = rospy.Publisher('/arm_collision/kuavo_arm_target_poses', armTargetPoses, queue_size=10)
    rospy.sleep(0.5)  # 确保Publisher注册
    # 创建消息对象并设置传入的times和values
    msg = armTargetPoses()
    msg.times = times
    msg.values = values

    rospy.loginfo("正在将手臂目标姿态发布到话题 '/arm_collision/kuavo_arm_target_poses'")

    # 等待订阅者连接
    rate = rospy.Rate(10)  # 10Hz
    while pub.get_num_connections() == 0 and not rospy.is_shutdown():
        rospy.loginfo("等待订阅者连接...")
        rate.sleep()

    # 发布消息
    pub.publish(msg)
    rospy.loginfo("消息已发布。")

def main():
    # 初始化ROS节点
    rospy.init_node('arm_target_poses_publisher', anonymous=True)

    # 调用函数并传入times和values
    publish_arm_target_poses([3], [-30, 0, -50, -70, 0, 0, 0, -30, 0, 50, -70, 0, 0, 0])

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass