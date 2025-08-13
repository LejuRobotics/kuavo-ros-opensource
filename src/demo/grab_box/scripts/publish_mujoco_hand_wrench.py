#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
import signal
import sys
import argparse

def signal_handler(sig, frame):
    print('\n正在退出...')
    sys.exit(0)

def main():
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='发布手部接触力指令')
    parser.add_argument('--force', type=float, default=80.0,
                      help='设置接触力大小（默认：80.0）')
    args = parser.parse_args()

    # 初始化ROS节点
    rospy.init_node('hand_wrench_publisher', anonymous=False)
    
    # 创建发布者
    pub_cmd = rospy.Publisher('/hand_wrench_cmd', Float64MultiArray, queue_size=1)
    pub_cmd_mujoco = rospy.Publisher('/hand_wrench_cmd_mujoco', Float64MultiArray, queue_size=1)
    
    # 设置消息
    msg = Float64MultiArray()
    msg.layout.dim = []
    msg.layout.data_offset = 0
    force = args.force
    msg.data = [0, 0, -force, 0, 0, 0, 0, 0, -force, 0, 0, 0]
    
    # 注册Ctrl+C处理函数
    signal.signal(signal.SIGINT, signal_handler)
    rate = rospy.Rate(1000)  # 1000Hz
    
    # 首先发布一次hand_wrench_cmd
    rospy.sleep(1.5)  # 等待发布者初始化
    pub_cmd.publish(msg)
    rospy.loginfo(f'已发布单次hand_wrench_cmd，力大小: {force}')
    
    # 持续发布hand_wrench_cmd_mujoco
    rospy.loginfo(f'开始持续发布hand_wrench_cmd_mujoco，力大小: {force}')
    while not rospy.is_shutdown():
        pub_cmd_mujoco.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    main() 
