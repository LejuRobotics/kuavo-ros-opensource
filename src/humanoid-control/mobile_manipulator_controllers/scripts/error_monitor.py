#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64

com_height = 0.0
terrain_height = 0.0

def getComHeight():
    global com_height
    while not rospy.has_param("/com_height"):
      rospy.sleep(0.1)
      print("Waiting for com_height parameter...")
    com_height = rospy.get_param("/com_height")

def terrainHeightCalllBack(msg):
    global terrain_height
    terrain_height = msg.data

def publish_transform_coordinates():
    rospy.init_node('frame_transform_publisher', anonymous=True)
    mm_pub = rospy.Publisher('mm_eef_poses', Float64MultiArray, queue_size=10)
    
    huamnoid_pub = rospy.Publisher('humanoid_eef_poses', Float64MultiArray, queue_size=10)
    
    # 创建一个发布者，发布Float64MultiArray类型的话题
    mm_eef_pub = rospy.Publisher('/mm_eef_poses_error', Float64MultiArray, queue_size=10)
    mm_base_error_pub = rospy.Publisher('/mm_base_error', Float64MultiArray, queue_size=10)

    terrain_height_sub = rospy.Subscriber('/humanoid/mpc/terrainHeight', Float64, terrainHeightCalllBack)
    
    # 创建tf监听器
    listener = tf.TransformListener()
    # getComHeight()
    print("com_height: ", com_height)
    
    rate = rospy.Rate(500)  # 10 Hz
    while not rospy.is_shutdown():
        try:
            # 获取frameA相对于frameB的变换
            (trans_mm, rot_mm) = listener.lookupTransform('mm/world', 'mm/zarm_l7_link', rospy.Time(0))
            # (trans_mm, rot_mm) = listener.lookupTransform('world', 'mm/zarm_l4_link', rospy.Time(0))
            # 将坐标转换为Float64MultiArray
            float_array = Float64MultiArray()
            float_array.data = [trans_mm[0], trans_mm[1], trans_mm[2]]
            mm_pub.publish(float_array)
            # 发布坐标
            (trans_humanoid, rot_humanoid) = listener.lookupTransform('mm/world', 'mm/zarm_l7_link', rospy.Time(0))
            # (trans_humanoid, rot_humanoid) = listener.lookupTransform('world', 'zarm_l4_link', rospy.Time(0))
            trans_humanoid[2] -= (com_height + terrain_height)
            float_array.data = [trans_humanoid[0], trans_humanoid[1], trans_humanoid[2]]
            huamnoid_pub.publish(float_array)
            # error
            float_array.data = [abs(trans_mm[0]-trans_humanoid[0]), abs(trans_mm[1]-trans_humanoid[1]), abs(trans_mm[2]-trans_humanoid[2])]
            mm_eef_pub.publish(float_array)

            (base_trans_mm, rot_mm) = listener.lookupTransform('mm/world', 'mm/base_link', rospy.Time(0))
            (base_trans_humanoid, rot_humanoid) = listener.lookupTransform('mm/world', 'base_link', rospy.Time(0))
            base_trans_humanoid[2] -= (com_height + terrain_height)
            float_array.data = [abs(base_trans_mm[0]-base_trans_humanoid[0]), abs(base_trans_mm[1]-base_trans_humanoid[1]), abs(base_trans_mm[2]-base_trans_humanoid[2])]
            mm_base_error_pub.publish(float_array)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            # rospy.logerr("无法获取变换信息")
            pass

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_transform_coordinates()
    except rospy.ROSInterruptException:
        pass
