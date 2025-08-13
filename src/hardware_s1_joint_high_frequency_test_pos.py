#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from kuavo_msgs.msg import jointCmd
import time
import signal
import sys

class SingleJointController:
    def __init__(self):
        rospy.init_node('single_joint_controller', anonymous=True)
        self.joint_pub = rospy.Publisher('/joint_cmd', jointCmd, queue_size=10)
        self.joint_states_pub = rospy.Publisher('/joint_states', Float64MultiArray, queue_size=10)
        
        # 控制参数
        self.rate = rospy.Rate(500)  # 控制频率
        self.speed_multiplier = 1.0
        self.frequency = 10          # 测试频率（赫兹）
        self.test_duration = 6.0    # 测试时长（秒）
        
        # 关节信息
        self.joint_names = [
            'leg_l1_joint', 'leg_l2_joint', 'leg_l3_joint', 'leg_l4_joint', 'leg_l5_joint', 'leg_l6_joint',
            'leg_r1_joint', 'leg_r2_joint', 'leg_r3_joint', 'leg_r4_joint', 'leg_r5_joint', 'leg_r6_joint',
            'waist_joint',
            'zarm_l1_joint', 'zarm_l2_joint', 'zarm_l3_joint', 'zarm_l4_joint',
            'zarm_r1_joint', 'zarm_r2_joint', 'zarm_r3_joint', 'zarm_r4_joint'
        ]
        self.joint_limits = [
            (-0.50, 0.50), (-1.10, 1.10), (-0.20, 0.20), (0.0, 0.8), (0.0, 0.0), (0.0, 0.0),
            (0.50, -0.50), (0.10, -0.10), (-0.20, 0.20), (0.0, 0.8), (0.0, 0.0), (0.0, 0.0),
            (0.0, 0.0),
            (-0.8, 0.8), (0.1, 0.2), (-0.1, 0.1), (0.0, -0.9),
            (-0.8, 0.8), (-0.2, -0.1), (-0.1, 0.1), (0.0, -0.9)
        ]
        
        # 用户选择的关节
        self.current_joint_index = None
        self.current_joint_name = None
        self.current_joint_limit = None
        
        # 数据记录
        self.expected_positions = []
        self.actual_positions = []
        self.timestamps = []
        
        # 订阅实际关节位置
        rospy.Subscriber('/sensor_data_motor/motor_pos', Float64MultiArray, self.motor_pos_callback)
        
        # 信号处理
        signal.signal(signal.SIGINT, self.signal_handler)
        self.running = True

        # 用户选择关节
        self.select_joint()

    def motor_pos_callback(self, msg):
        """回调函数，处理接收到的实际关节位置"""
        if self.running and self.current_joint_index is not None:
            try:
                actual_position = msg.data[self.current_joint_index]  # 获取当前关节的实际位置
                self.actual_positions.append(actual_position)
            except IndexError:
                rospy.logerr(f"Index {self.current_joint_index} is out of range for msg.data of length {len(msg.data)}")

    def signal_handler(self, sig, frame):
        """处理 Ctrl+C 信号，保存数据并平滑过渡到零点"""
        print("\nStopping joint movement and shutting down...")
        self.running = False
        self.smooth_transition_to_zero()
        sys.exit(0)

    def smooth_transition_to_zero(self):
        """平滑过渡到零点"""
        transition_time = 2.0  # 过渡时间（秒）
        transition_steps = int(transition_time * self.rate.sleep_dur.to_sec())
        current_positions = self.actual_positions[-1] if self.actual_positions else [0.0] * len(self.joint_names)
        
        for step in range(transition_steps):
            positions = [pos * (1 - step / transition_steps) for pos in current_positions]
            self.publish_joint_state(positions)
            self.rate.sleep()

    def calculate_joint_position(self, t):
        """计算期望关节位置"""
        min_val, max_val = self.current_joint_limit
        mid_val = (max_val + min_val) / 2
        amplitude = (max_val - min_val) / 2
        return mid_val + amplitude * np.sin(self.frequency * t)

    def publish_joint_state(self, positions):
        """发布关节命令和状态"""
        joint_cmd = jointCmd()
        joint_cmd.header.stamp = rospy.Time.now()
        joint_cmd.joint_q = positions
        joint_cmd.joint_v = [0.0] * len(self.joint_names)
        joint_cmd.tau = [0.0] * len(self.joint_names)
        joint_cmd.tau_max = [2.0] * len(self.joint_names)
        joint_cmd.tau_ratio = [0.0] * len(self.joint_names)
        joint_cmd.joint_kp = [0.0] * len(self.joint_names)
        joint_cmd.joint_kd = [0.0] * len(self.joint_names)
        joint_cmd.control_modes = [2] * len(self.joint_names)
        self.joint_pub.publish(joint_cmd)

        # 发布关节状态
        joint_state = Float64MultiArray()
        joint_state.data = joint_cmd.joint_q
        self.joint_states_pub.publish(joint_state)

    def select_joint(self):
        """让用户选择要控制的关节"""
        print("\nAvailable Joints:")
        for idx, joint_name in enumerate(self.joint_names):
            print(f"{idx + 1}. {joint_name}")
        
        while True:
            try:
                choice = int(input("Enter the number of the joint you want to control: "))
                if 1 <= choice <= len(self.joint_names):
                    self.current_joint_index = choice - 1
                    self.current_joint_name = self.joint_names[self.current_joint_index]
                    self.current_joint_limit = self.joint_limits[self.current_joint_index]
                    print(f"Selected joint: {self.current_joint_name}")
                    break
                else:
                    print("Invalid choice. Please enter a valid number.")
            except ValueError:
                print("Invalid input. Please enter a number.")

    def run(self):
        """主循环"""
        print(f"\nStarting single joint movement for {self.current_joint_name} with frequency {self.frequency}Hz. Press Ctrl+C to stop.")
        self.start_time = time.time()
        while not rospy.is_shutdown() and self.running:
            t = time.time() - self.start_time
            if t > self.test_duration:
                break

            expected_position = self.calculate_joint_position(t)
            self.publish_joint_state([expected_position if i == self.current_joint_index else 0.0 for i in range(len(self.joint_names))])
            self.expected_positions.append(expected_position)
            self.timestamps.append(t)
            self.rate.sleep()

        self.smooth_transition_to_zero()  # 平滑过渡到零点
        print("Test completed.")

if __name__ == '__main__':
    try:
        controller = SingleJointController()
        controller.run()
    except rospy.ROSInterruptException:
        pass