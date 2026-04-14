#!/usr/bin/env python3
import sys
import os

# 方法1：动态添加路径
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import rospy
import lb_ctrl_api as ct
import argparse

def execute_torso_tests():
    """依次发布躯干位姿测试数据，并等待每次运动结束"""
    global reach_time, initialTorsoPose_

    # 初始化节点
    rospy.init_node('torso_pose_publisher', anonymous=True)

    # 首先获取躯干初始位姿
    rospy.loginfo("正在获取躯干初始位姿...")

    start_time = rospy.get_time()  # 记录开始时间
    success, initial_pose = ct.get_torso_initial_pose(True)
    end_time = rospy.get_time()    # 记录结束时间

    execution_time = end_time - start_time

    rospy.loginfo(f"✅ 获取躯干初始位姿完成，耗时: {execution_time:.3f} 秒")
    
    if not success:
        rospy.logerr("❌ 无法获取躯干初始位姿，退出程序")
        return
    
    # 提取初始位置
    initialTorsoPose_ = initial_pose['position']

    parser = argparse.ArgumentParser(description='躯干测试程序')
    parser.add_argument('--no-reset', action='store_true', help='跳过躯干重置')
    args = parser.parse_args()
    reset_torso = not args.no_reset
    resetTime = 0
    if reset_torso:
        resetTime = ct.reset_torso_to_initial()

    # 等待连接建立
    rospy.sleep(resetTime + 0.5)

    # 测试用例列表： (名称, 期望时间, [lx, lz, yaw, pitch])
    # 注意：第一个参数是期望时间（秒），lx, lz 是相对于初始位置的增量
    test_cases = [
        ("初始抬高", 4.0, [0.0, 0.3,   0.0,  0.0]),   
        ("前移",     4.0, [0.2, 0.3,  0.0,  0.0]),    
        ("偏航+30°", 4.0, [0.2, 0.3,  0.52356,  0.0]),   
        ("偏航-30°", 4.0, [0.2, 0.3, -0.52356,  0.0]),
        ("俯仰-10°", 4.0, [0.2, 0.3,  0.0, -0.1745]), 
        ("俯仰+30°", 4.0, [0.2, 0.3,  0.0,  0.524]),  
        ("复位",     4.0, [0.0, 0.0,  0.0,  0.0]),    
    ]

    rospy.loginfo("开始发布躯干位姿测试数据...")
    rospy.loginfo(f"初始躯干位置: {initialTorsoPose_}")

    for idx, (name, desire_time, pos_data) in enumerate(test_cases, 1):
        # 解包位置数据
        lx, lz, az, ay = pos_data
        
        # 计算绝对坐标：初始位置 + 增量
        abs_x = initialTorsoPose_[0] + lx
        abs_z = initialTorsoPose_[2] + lz

        rospy.loginfo(f"\n=== 第{idx}组测试: {name} ===")
        rospy.loginfo(f"  期望时间: {desire_time:.1f} 秒")
        rospy.loginfo(f"  相对目标: [{lx}, {lz}, {az}, {ay}]")
        rospy.loginfo(f"  绝对坐标: [{abs_x:.3f}, {abs_z:.3f}]")

        pos = [abs_x, abs_z, az, ay]  # 注意顺序：x, z, yaw, pitch
        success, actual_time = ct.send_timed_single_command(
            planner_index=2,
            desire_time=desire_time,
            cmd_vec=pos
        )

        if success:
            rospy.loginfo(f"  命令发送成功，实际执行时间: {actual_time:.2f} 秒")
        else:
            rospy.logwarn(f"  命令发送失败")

        # 等待运动完成再发下一组
        rospy.sleep(actual_time + 0.5)
        rospy.loginfo(f"  {name} 完成!")

    rospy.loginfo("\n所有躯干位姿测试数据发布完成！")

# -------------- 主入口 --------------
def main():
    try:
        execute_torso_tests()
    except rospy.ROSInterruptException:
        rospy.logwarn("ROS 中断异常")
    except Exception as e:
        rospy.logerr(f"程序执行出错: {e}")

if __name__ == '__main__':
    main()