#!/usr/bin/env python3
import sys
import os

# 方法1：动态添加路径
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import rospy
import lb_ctrl_api as ct

def execute_base_tests():
    """依次发布底盘世界系位姿数据，并等待每次运动结束"""
    
    # 初始化节点
    rospy.init_node('base_pose_publisher', anonymous=True)

    # 等待连接建立
    rospy.sleep(0.01)

    # 测试用例列表： (名称, 时间, [x, y, yaw])
    # 注意：x, y单位为米，yaw单位为弧度
    test_cases = [
        ("测试数据1", 5.0, [-0.3, 0.0, 0.0]), 
        ("测试数据2", 5.0, [0.3, 0.0, 0.0]),  
        ("测试数据1", 5.0, [-0.3, 0.0, 0.0]), 
        ("测试数据2", 5.0, [0.3, 0.0, 0.0]), 
    ]

    rospy.loginfo("开始发布底盘世界系位姿测试数据...")

    for idx, (name, desire_time, pose_data) in enumerate(test_cases, 1):
        rospy.loginfo(f"\n=== 第{idx}组测试: {name} ===")
        rospy.loginfo(f"  期望时间: {desire_time:.1f} 秒")
        rospy.loginfo(f"  位姿: [x={pose_data[0]:.2f}m, y={pose_data[1]:.2f}m, yaw={pose_data[2]:.2f}rad]")

        # 发送命令（使用底盘世界系位姿）
        success, actual_time = ct.send_timed_single_command(
            planner_index=1,  # 假设3是底盘控制的索引，请根据实际情况调整
            desire_time=desire_time,
            cmd_vec=pose_data  # 直接使用 [x, y, yaw]
        )

        if success:
            rospy.loginfo(f"  命令发送成功，实际执行时间: {actual_time:.2f} 秒")
        else:
            rospy.logwarn(f"  命令发送失败")

        # 等待运动完成再发下一组
        rospy.sleep(actual_time + 0.5)
        rospy.loginfo(f"  {name} 完成!")

    rospy.loginfo("\n所有底盘世界系位姿测试数据发布完成！")

# -------------- 主入口 --------------
def main():
    try:
        execute_base_tests()
    except rospy.ROSInterruptException:
        rospy.logwarn("ROS 中断异常")
    except Exception as e:
        rospy.logerr(f"程序执行出错: {e}")

if __name__ == '__main__':
    main()