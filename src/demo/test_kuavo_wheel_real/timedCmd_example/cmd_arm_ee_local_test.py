#!/usr/bin/env python3
import sys
import os

# 方法1：动态添加路径
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import rospy
import lb_ctrl_api as ct
import math
import argparse

def degrees_to_radians(pose_deg):
    """将位姿数据中的角度（度）转换为弧度"""
    pose_rad = list(pose_deg)  # 创建副本
    # 姿态角度（索引3,4,5）从度转换为弧度
    for i in [3, 4, 5]:
        pose_rad[i] = math.radians(pose_deg[i])
    return pose_rad

def execute_dual_arm_pose_tests():
    """依次发布双臂末端位姿数据，并等待每次运动结束"""
    
    # 初始化节点
    rospy.init_node('dual_arm_pose_publisher', anonymous=True)

    # 等待连接建立
    ct.set_arm_control_mode(1)  # 重置手臂, 避免奇异点问题
    rospy.sleep(1.5)
    ct.set_arm_control_mode(2)

    # 创建解析器
    parser = argparse.ArgumentParser(description='设置笛卡尔跟踪焦点')
    
    # 添加参数
    parser.add_argument('--focus', '-f', type=str, default='ee',
                       choices=['torso', 'ee'],
                       help='跟踪焦点: torso(躯干) 或 ee(末端)')
    
    # 解析参数
    args = parser.parse_args()

    # 使用参数
    focus_ee = (args.focus == 'ee')  # 简洁写法
    ct.set_focus_ee(focus_ee)

    # 测试用例列表： (名称, 时间, [左臂x,y,z,yaw,pitch,roll, 右臂x,y,z,yaw,pitch,roll])
    # 注意：位置单位为米，姿态单位为弧度
    test_cases = [
        ("左右展开", 2.0, [0.1, 0.4, 0.7, 0.0, 0.0, 0.0], [0.1, -0.4, 0.7, 0.0, 0.0, 0.0]),
        ("前摆臂",   2.0, [0.3, 0.4, 0.7, 0.0, -90, 0.0], [0.3, -0.4, 0.7, 0.0, -90, 0.0]),
        ("前摆臂",   2.0, [0.3, 0.2, 0.7, 0.0, -90, 0.0], [0.3, -0.2, 0.7, 0.0, -90, 0.0]),
        ("前摆臂",   2.0, [0.5, 0.2, 0.7, 0.0, -90, 0.0], [0.5, -0.2, 0.7, 0.0, -90, 0.0]),
        ("前摆臂",   2.0, [0.5, 0.2, 0.85, 0.0, -90, 0.0], [0.5, -0.2, 0.85, 0.0, -90, 0.0]),
        ("前摆臂",   4.0, [1.2, 0.2, 0.85, 0.0, -90, 0.0], [1.2, -0.2, 0.85, 0.0, -90, 0.0]),
        ("前摆臂",   4.0, [0.5, 0.2, 0.85, 0.0, -90, 0.0], [0.5, -0.2, 0.85, 0.0, -90, 0.0]),
    ]

    rospy.loginfo("开始发布双臂末端位姿测试数据...")

    for idx, (name, desire_time, left_arm_pose, right_arm_pose) in enumerate(test_cases, 1):
        rospy.loginfo(f"\n=== 第{idx}组测试: {name} ===")
        rospy.loginfo(f"  期望时间: {desire_time:.1f} 秒")
        
        # 角度转弧度
        left_arm_rad = degrees_to_radians(left_arm_pose)
        right_arm_rad = degrees_to_radians(right_arm_pose)
        
        # 显示原始数据（度）
        rospy.loginfo(f"  左臂位姿(度): [x={left_arm_pose[0]:.2f}m, y={left_arm_pose[1]:.2f}m, z={left_arm_pose[2]:.2f}m, yaw={left_arm_pose[3]:.1f}°, pitch={left_arm_pose[4]:.1f}°, roll={left_arm_pose[5]:.1f}°]")
        rospy.loginfo(f"  右臂位姿(度): [x={right_arm_pose[0]:.2f}m, y={right_arm_pose[1]:.2f}m, z={right_arm_pose[2]:.2f}m, yaw={right_arm_pose[3]:.1f}°, pitch={right_arm_pose[4]:.1f}°, roll={right_arm_pose[5]:.1f}°]")

        # 发送命令（使用双臂末端位姿）
        left_success, left_actual_time = ct.send_timed_single_command(
            planner_index=6,
            desire_time=desire_time,
            cmd_vec=left_arm_rad  # 直接使用 [左臂x,y,z,yaw,pitch,roll, 右臂x,y,z,yaw,pitch,roll]
        )
        right_success, right_actual_time = ct.send_timed_single_command(
            planner_index=7,
            desire_time=desire_time,
            cmd_vec=right_arm_rad  # 直接使用 [左臂x,y,z,yaw,pitch,roll, 右臂x,y,z,yaw,pitch,roll]
        )
        if left_success and right_success:
            actual_time = max(left_actual_time, right_actual_time)
            rospy.loginfo(f"  双臂命令发送成功，实际执行时间: {actual_time:.2f} 秒")
        else:
            rospy.logwarn(f"  命令发送失败: 左臂{'成功' if left_success else '失败'}, 右臂{'成功' if right_success else '失败'}")

        # 等待运动完成再发下一组
        rospy.sleep(actual_time + 0.5 if 'actual_time' in locals() else desire_time + 0.5)
        rospy.loginfo(f"  {name} 完成!")

    rospy.loginfo("\n所有双臂末端位姿测试数据发布完成！")

# -------------- 主入口 --------------
def main():
    try:
        execute_dual_arm_pose_tests()
    except rospy.ROSInterruptException:
        rospy.logwarn("ROS 中断异常")
    except Exception as e:
        rospy.logerr(f"程序执行出错: {e}")

if __name__ == '__main__':
    main()