#!/usr/bin/env python3
import rospy
from kuavo_msgs.msg import dexhandCommand
import time
import signal
import sys

def signal_handler(sig, frame):
    print("\nCtrl+C detected, exiting...")
    sys.exit(0)

def main():
    signal.signal(signal.SIGINT, signal_handler)
    
    rospy.init_node('dexhand_test')
    
    # Create publishers for each topic
    dual_pub = rospy.Publisher('/dexhand/command', dexhandCommand, queue_size=10)
    left_pub = rospy.Publisher('/dexhand/left/command', dexhandCommand, queue_size=10)
    right_pub = rospy.Publisher('/dexhand/right/command', dexhandCommand, queue_size=10)

    # Wait for publishers to be ready
    rospy.sleep(1)

    # Create command message
    cmd = dexhandCommand()
    cmd.control_mode = dexhandCommand.POSITION_CONTROL

    # Wave motion positions (6 finger positions per step)
    positions_list = [
        [10, 10, 10, 10, 10, 10],
        [20, 10, 100, 10, 10, 10],
        [30, 10, 100, 90, 10, 10],
        [40, 10, 100, 90, 90, 10],
        [50, 10, 100, 100, 100, 100],
    ]
    num_steps = len(positions_list)
    delay_sec = 1.0  # 1000ms = 1s

    print("\033[36m[dexhand_test] INFO: 开始位置控制测试 - 波浪运动模式\033[0m")

    for j in range(10):  # 10 waves
        print(f"\033[36m[dexhand_test] INFO: 第 {j+1} 轮波浪运动\033[0m")

        # Forward wave
        for i in range(num_steps):
            # Set positions for both hands (12 values total)
            cmd.data = positions_list[i] * 2  # Duplicate for both hands
            
            print(f"发布波浪步进 {i+1}/{num_steps} 到双手")
            dual_pub.publish(cmd)
            rospy.sleep(delay_sec)

        # Reverse wave
        for i in range(num_steps-1, -1, -1):
            cmd.data = positions_list[i] * 2  # Duplicate for both hands
            
            print(f"发布反向波浪步进 {num_steps-i}/{num_steps} 到双手")
            dual_pub.publish(cmd)
            rospy.sleep(delay_sec)

    print("\033[36m[dexhand_test] INFO: 波浪运动测试完成\033[0m")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass