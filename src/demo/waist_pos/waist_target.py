#! /sur/bin/env python
import numpy as np
import rospy
from ocs2_msgs.msg import mpc_observation
from ocs2_msgs.msg import mpc_target_trajectories, mpc_state, mpc_input  #发布的消息类型
import time

"""
    使用python实现消息发布：
    1.导包
    2.初始化ROS节点
    3.创建发布者对象
    4.编写发布逻辑并发布数据
"""
class MPCObservationSubscriber:
    def __init__(self):
        rospy.init_node('waist_target_publisher', anonymous=True)
        self.Time_init = 0
        self.time_captured = False
        
        self.latest_observation = mpc_observation()  # 存储最新数据
        self.subscriber = rospy.Subscriber(
            '/humanoid_mpc_observation', 
            mpc_observation, 
            self.observation_callback
        )
        time.sleep(1)
        
        self.waist_publish(self.Time_init)
        
    def waist_publish(self, Time_init):
        self.pub = rospy.Publisher('humanoid_mpc_target_waist', mpc_target_trajectories, queue_size=10)
        time.sleep(1)
        # 目标角度设置
        self.target_angle_rad = np.deg2rad(-45)
        self.end_target_angle_rad = np.deg2rad(45)
        
        # 创建目标轨迹消息
        self.msg = mpc_target_trajectories()
        print("self.Time_init:%.3f",Time_init)

        # 设置时间轨迹（基于观测时间）
        self.msg.timeTrajectory = [Time_init + 2]
        
        # 设置状态轨迹
        state_msg0 = mpc_state([0])  # 初始状态
        state_msg1 = mpc_state([self.target_angle_rad])  # 目标状态
        state_msg2 = mpc_state([self.end_target_angle_rad])  # 结束状态
        self.msg.stateTrajectory = [state_msg0]

        # 设置输入轨迹（设为0）
        input_msg = mpc_input([0])
        self.msg.inputTrajectory = [input_msg]
        
        # 设置发布频率控制
        self.rate = rospy.Rate(500)  # 500Hz = 每秒500次
        self.counter = 0
        self.print_interval = 500  # 每500次打印一次（每秒1次）

        self.pub.publish(self.msg)
        # while not rospy.is_shutdown():
        #     # 发布消息
        #     self.pub.publish(self.msg)
            
        #     # 打印运行状态（控制频率避免刷屏）
        #     self.counter += 1
        #     if self.counter % self.print_interval == 0:
        #         rospy.loginfo("Waist target topic is running")
        #     # 按照500Hz频率休眠
        #     self.rate.sleep()

    def observation_callback(self, msg):
        """存储最新观测数据"""
        if not self.time_captured:
            self.latest_observation = msg
            self.Time_init = msg.time
            self.time_captured = True
            rospy.loginfo("self.latest_observation: %.3f", self.latest_observation.time)
            # rospy.loginfo("Received new MPC observation")
            # rospy.loginfo("Time: %.3f", msg.time)
            # rospy.loginfo("Mode: %d", msg.mode)
    
    def get_latest_observation(self):
        """获取最新观测数据"""
        return self.latest_observation


if __name__ == '__main__':
    
    subscriber = MPCObservationSubscriber()