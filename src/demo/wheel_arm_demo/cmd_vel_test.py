#!/usr/bin/env python3

import rospy
import math
import threading
from geometry_msgs.msg import Twist
from ocs2_msgs.msg import mpc_observation
import ocs2_msgs.msg

cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

class RobotController:
    def __init__(self):
        rospy.init_node('improved_robot_controller', anonymous=True)
        
        # 状态变量
        self.latest_observation = None
        self.observation_mutex = threading.Lock()
        self.rotation_completed = False
        self.target_yaw = math.pi / 2.0  # 90度
        self.linear_speed = 0.4
        self.angular_speed = 0.5
        self.yaw_tolerance = 0.005  # 角度容差
        
        # 订阅observation话题
        observation_topic = "mobile_manipulator_mpc_observation"
        
        self.observation_sub = rospy.Subscriber(
            observation_topic, 
            mpc_observation, 
            self.observation_callback,
            queue_size=1
        )
        
        print(f"订阅observation话题: {observation_topic}")
        
    def observation_callback(self, msg):
        """处理observation回调"""
        with self.observation_mutex:
            # 这里需要根据实际的OCS2 ROS消息转换来解析数据
            # 假设我们可以直接从msg中获取时间、状态等信息
            self.latest_observation = {
                'time': msg.time,
                'state': msg.state.value,  # 状态向量
                'input': msg.input.value,  # 输入向量
                'mode': msg.mode
            }
    
    def get_current_yaw(self):
        """从observation中获取当前yaw角"""
        with self.observation_mutex:
            if self.latest_observation is None:
                return None
            
            # 根据您提供的信息，state(2)是实际的yaw角
            if len(self.latest_observation['state']) > 2:
                return self.latest_observation['state'][2]
            else:
                return None
    
    def control_loop(self):
        """主控制循环"""
        rate = rospy.Rate(20)  # 20Hz控制频率
        
        print("等待接收observation数据...")
        
        # 等待直到收到observation数据
        while not rospy.is_shutdown() and self.get_current_yaw() is None:
            rate.sleep()
        
        # 获取初始yaw角并计算目标yaw角
        self.initial_yaw = self.get_current_yaw()
        self.target_yaw = self.initial_yaw + math.pi / 2.0  # 在当前角度基础上转90度
        
        print("开始执行控制序列...")
        
        while not rospy.is_shutdown():
            current_yaw = self.get_current_yaw()
            if current_yaw is None:
                rospy.logwarn("无法获取当前yaw角")
                rate.sleep()
                continue
            
            twist_msg = Twist()
            
            yaw_error = self.target_yaw - current_yaw
            
            # 角度归一化到[-pi, pi]
            while yaw_error > math.pi:
                yaw_error -= 2 * math.pi
            while yaw_error < -math.pi:
                yaw_error += 2 * math.pi
            
            if abs(yaw_error) > self.yaw_tolerance:
                # 执行旋转
                twist_msg.angular.z = self.angular_speed if yaw_error > 0 else -self.angular_speed
                print(f"旋转中... 当前角度: {math.degrees(current_yaw):.1f}°, 目标: {math.degrees(self.target_yaw):.1f}°, 误差: {math.degrees(yaw_error):.1f}°")
            else:
                # 旋转完成
                twist_msg.angular.z = 0.0
            
            # TODO: 组合一个世界系y方向运动的msg
            # # 在世界坐标系中，向Y正方向运动需要根据当前机器人的朝向进行速度分解
            twist_msg.linear.x = self.linear_speed * math.sin(current_yaw - self.initial_yaw)  # X方向分量
            twist_msg.linear.y = self.linear_speed * math.cos(current_yaw - self.initial_yaw)   # Y方向分量
            
            # 发布控制命令
            cmd_vel_pub.publish(twist_msg)
            rate.sleep()

def main():
    try:
        controller = RobotController()
        
        # 等待ROS初始化
        rospy.sleep(1.0)
        
        # 启动控制循环
        controller.control_loop()
        
    except rospy.ROSInterruptException:
        print("程序被用户中断")
    except Exception as e:
        print(f"程序发生错误: {e}")

if __name__ == '__main__':
    main()