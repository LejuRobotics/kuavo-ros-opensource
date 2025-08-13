#! /sur/bin/env python
import numpy as np
import rospy
from ocs2_msgs.msg import mpc_observation
from std_msgs.msg import Float64MultiArray
from ocs2_msgs.msg import mpc_target_trajectories, mpc_state, mpc_input  #发布的消息类型
import time

"""
    发布手柄控制腰部话题
"""
class MPCObservationSubscriber:
    def __init__(self):
        rospy.init_node('waist_target_publisher', anonymous=True)
        
        self.waist_publish()
    def process_data(self, value):
        # 目标角度设置
        self.target_angle_rad = value
        
        # 创建目标轨迹消息
        self.msg = Float64MultiArray()

        # 设置时间轨迹（基于观测时间）
        self.msg.data = [self.target_angle_rad]

        self.pub.publish(self.msg)
        time.sleep(2)   #等待运动结束
    def is_number(self,value):
        try:
            float(value)  # 尝试转换为浮点数
            return True
        except ValueError:
            return False
    def waist_publish(self):
        self.pub = rospy.Publisher('/robot_waist_motion_data', Float64MultiArray, queue_size=10)
        user_input = 0
        print("按q退出") 
        while not rospy.is_shutdown():
            try:
                user_input_last = user_input
                user_input = input("请输入角度(deg): ")

            except EOFError:
                break
            if self.is_number(user_input):
                dJointValue = float(user_input) - float(user_input_last)  # 角度增量
                if abs(dJointValue) > 120:  # 角度增量小于120度才发布
                    print("角度增量大于120度，请重新输入")
                    continue
                else:
                    jointValue = float(user_input)
            elif user_input == "q":
                break
            else:
                print("无效输入，请重新输入")
                continue
            self.process_data(jointValue)

if __name__ == '__main__':
    
    subscriber = MPCObservationSubscriber()