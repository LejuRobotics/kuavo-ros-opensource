#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from utils.load_keyframes import load_keyframes
from utils.interpolation_utils import interpolate_keyframes
from utils.publish_keyframes import set_mpc_control_mode, set_arm_quick_mode, publish_single_frame


def main():
    # 循环次数
    loop_count = 5
    # 循环之间的间隔时间（秒）
    loop_interval = 2

    # 初始化ROS节点
    rospy.init_node('motion_capture_calibration_wheel', anonymous=True)
    
    # 读取配置文件中的关键帧
    rospy.loginfo("读取关键帧配置...")
    keyframes = load_keyframes()
    
    # 提取所有带mark的关键帧时间集合
    marked_keyframe_times = set()
    for kf in keyframes:
        if kf.get('mark', False):
            marked_keyframe_times.add(kf['time'])
    rospy.loginfo(f"检测到 {len(marked_keyframe_times)} 个带mark的关键帧: {sorted(marked_keyframe_times)}")
    
    # 对关键帧进行直线插值，100Hz频率
    rospy.loginfo("对关键帧进行直线插值...")
    interpolated_sequence = interpolate_keyframes(keyframes, rate_hz=100)
    rospy.loginfo(f"插值完成，共 {len(interpolated_sequence)} 个点")
    
    # 创建发布器
    torso_pub = rospy.Publisher('/cmd_lb_torso_pose', Twist, queue_size=10)
    arm_pub = rospy.Publisher('/kuavo_arm_traj', JointState, queue_size=10)
    flag_pub = rospy.Publisher('/keyframe_flag', Float32, queue_size=10)
    
    # 等待连接建立
    rospy.sleep(1)
    
    # 切换到ArmOnly
    set_mpc_control_mode(1)
    rospy.sleep(0.5)  # 等待模式切换生效
    # 启用手臂快速模式
    set_arm_quick_mode(True)
    flag_msg = Float32()
    try:
        # 循环播放轨迹
        rospy.loginfo(f"开始播放轨迹，循环次数: {loop_count}")
        
        for loop_idx in range(loop_count):
            if rospy.is_shutdown():
                break
            
            if loop_count > 1:
                rospy.loginfo(f"第 {loop_idx + 1}/{loop_count} 次循环")
            
            # 按100Hz频率发布
            rate = rospy.Rate(100)
            
            for idx, frame in enumerate(interpolated_sequence):
                if rospy.is_shutdown():
                    break
                
                # 发布当前帧
                publish_single_frame(torso_pub, arm_pub, frame)
                
                # 检查当前帧时间是否匹配带mark的关键帧时间（考虑浮点数精度）
                current_time = frame['time']
                flag_value = 0.0
                for marked_time in marked_keyframe_times:
                    if abs(current_time - marked_time) < 1e-3:  # 1ms精度
                        flag_value = marked_time
                        break
                
                # 发布关键帧标志
                flag_msg.data = flag_value
                flag_pub.publish(flag_msg)
                
                # 控制发布频率
                rate.sleep()
            
            # 如果不是最后一次循环，等待间隔时间
            if loop_idx < loop_count - 1 and not rospy.is_shutdown():
                rospy.loginfo(f"等待 {loop_interval} 秒后开始下一次循环...")
                rospy.sleep(loop_interval)
                
    finally:
        # 禁用手臂快速模式
        set_arm_quick_mode(False)
        rospy.loginfo("轨迹播放完成")

if __name__ == '__main__':
    main()

