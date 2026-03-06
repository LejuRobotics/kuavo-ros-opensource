#!/usr/bin/env python3
import sys
import os

# 方法1：动态添加路径
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))

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

def execute_combined_tests():
    """组合发布底盘和双臂末端位姿数据，使用多指令同时执行"""
    
    # 初始化节点
    rospy.init_node('combined_pose_publisher', anonymous=True)

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
    focus_ee = (args.focus == 'ee')
    ct.set_focus_ee(focus_ee)

    # 测试用例列表： (名称, 时间, 底盘位姿, 左臂位姿, 右臂位姿)
    # 注意：底盘位姿 [x, y, yaw] (米, 米, 弧度)
    # 手臂位姿 [x,y,z,yaw,pitch,roll] (位置米, 姿态度 - 会被转换)
    test_cases = [
        ("测试1-底盘不动+手臂展开", 3.0, 
         [0.0, 0.0, 0.0],  # 底盘位姿
         [0.1, 0.4, 0.7, 0.0, 0.0, 0.0],  # 左臂
         [0.1, -0.4, 0.7, 0.0, 0.0, 0.0]), # 右臂
         
        ("测试2-底盘后退+手臂前摆", 4.0, 
         [-0.3, 0.0, 0.0],  # 底盘位姿
         [0.3, 0.2, 0.85, 0.0, -90, 0.0],  # 左臂
         [0.3, -0.2, 0.85, 0.0, -90, 0.0]), # 右臂
         
        ("测试3-底盘前进+手臂收回", 4.0, 
         [0.0, 0.0, 0.0],  # 底盘位姿
         [0.5, 0.2, 0.7, 0.0, -90, 0.0],  # 左臂
         [0.5, -0.2, 0.7, 0.0, -90, 0.0]), # 右臂
         
        ("测试4-底盘旋转+手臂前伸", 5.0, 
         [-0.3, 0.0, 1.57],  # 底盘旋转90度
         [1.2, 0.2, 0.85, 0.0, -90, 0.0],  # 左臂
         [1.2, -0.2, 0.85, 0.0, -90, 0.0]), # 右臂
         
        ("测试5-零位复位", 3.0, 
         [0.0, 0.0, 0.0],  # 底盘零位
         [0.5, 0.2, 0.85, 0.0, -90, 0.0],  # 左臂
         [0.5, -0.2, 0.85, 0.0, -90, 0.0]), # 右臂
    ]

    rospy.loginfo("开始发布组合位姿测试数据（底盘+双臂末端）...")

    for idx, (name, desire_time, base_pose, left_arm_pose, right_arm_pose) in enumerate(test_cases, 1):
        rospy.loginfo(f"\n=== 第{idx}组测试: {name} ===")
        rospy.loginfo(f"  期望时间: {desire_time:.1f} 秒")
        
        # 手臂角度转弧度
        left_arm_rad = degrees_to_radians(left_arm_pose)
        right_arm_rad = degrees_to_radians(right_arm_pose)
        
        # 显示原始数据
        rospy.loginfo(f"  底盘位姿: [x={base_pose[0]:.2f}m, y={base_pose[1]:.2f}m, yaw={base_pose[2]:.2f}rad]")
        rospy.loginfo(f"  左臂位姿(度): [x={left_arm_pose[0]:.2f}m, y={left_arm_pose[1]:.2f}m, z={left_arm_pose[2]:.2f}m, yaw={left_arm_pose[3]:.1f}°, pitch={left_arm_pose[4]:.1f}°, roll={left_arm_pose[5]:.1f}°]")
        rospy.loginfo(f"  右臂位姿(度): [x={right_arm_pose[0]:.2f}m, y={right_arm_pose[1]:.2f}m, z={right_arm_pose[2]:.2f}m, yaw={right_arm_pose[3]:.1f}°, pitch={right_arm_pose[4]:.1f}°, roll={right_arm_pose[5]:.1f}°]")

        # 构建多指令列表
        timed_cmd_vec = [
            {
                'planner_index': 0,  # 底盘规划器
                'desire_time': desire_time,
                'cmd_vec': base_pose  # [x, y, yaw]
            },
            {
                'planner_index': 6,  # 左臂末端规划器
                'desire_time': desire_time,
                'cmd_vec': left_arm_rad
            },
            {
                'planner_index': 7,  # 右臂末端规划器
                'desire_time': desire_time,
                'cmd_vec': right_arm_rad
            }
        ]
        
        # 发送多指令
        success, actual_time, message = ct.send_timed_multi_commands(
            timed_cmd_vec=timed_cmd_vec,
            is_sync=True  # 底盘和手臂同步运动
        )

        if success:
            rospy.loginfo(f"  ✅ 命令发送成功，实际执行时间: {actual_time:.2f} 秒")
        else:
            rospy.logwarn(f"  ❌ 命令发送失败: {message}")
            actual_time = 0.0

        # 等待运动完成再发下一组
        rospy.sleep(actual_time + 0.5)
        rospy.loginfo(f"  {name} 完成!")

    rospy.loginfo("\n所有组合位姿测试数据发布完成！")

# -------------- 主入口 --------------
def main():
    try:
        # 运行组合测试
        execute_combined_tests()
        
        rospy.sleep(2.0)
        
    except rospy.ROSInterruptException:
        rospy.logwarn("ROS 中断异常")
    except Exception as e:
        rospy.logerr(f"程序执行出错: {e}")

if __name__ == '__main__':
    main()