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

def execute_combined_base_torso_arm_tests():
    """组合发布底盘、躯干和双臂末端位姿数据，使用多指令同时执行"""
    global initialTorsoPose_
    
    # 初始化节点
    rospy.init_node('base_torso_arm_combined_publisher', anonymous=True)

    # 首先获取躯干初始位姿
    rospy.loginfo("正在获取躯干初始位姿...")

    start_time = rospy.get_time()
    success, initial_pose = ct.get_torso_initial_pose(True)
    end_time = rospy.get_time()
    execution_time = end_time - start_time

    rospy.loginfo(f"✅ 获取躯干初始位姿完成，耗时: {execution_time:.3f} 秒")
    
    if not success:
        rospy.logerr("❌ 无法获取躯干初始位姿，退出程序")
        return
    
    # 提取初始位置
    initialTorsoPose_ = initial_pose['position']
    rospy.loginfo(f"初始躯干位置: {initialTorsoPose_}")

    # 设置手臂模式
    ct.set_arm_control_mode(1)  # 重置手臂, 避免奇异点问题
    rospy.sleep(1.5)
    ct.set_arm_control_mode(2)

    # 创建解析器
    parser = argparse.ArgumentParser(description='底盘、躯干和手臂组合测试程序')
    parser.add_argument('--focus', '-f', type=str, default='ee',
                       choices=['torso', 'ee'],
                       help='跟踪焦点: torso(躯干) 或 ee(末端)')
    parser.add_argument('--no-reset', action='store_true', help='跳过躯干重置')
    args = parser.parse_args()

    # 设置焦点
    focus_ee = (args.focus == 'ee')
    ct.set_focus_ee(focus_ee)

    # 躯干重置
    reset_torso = not args.no_reset
    resetTime = 0
    if reset_torso:
        resetTime = ct.reset_torso_to_initial()
    rospy.sleep(resetTime + 0.5)

    # 测试用例列表： (名称, 时间, 底盘位姿, 躯干相对位姿, 左臂位姿, 右臂位姿)
    # 底盘位姿: [x, y, yaw] (米, 米, 弧度)
    # 躯干位姿: [lx, lz, yaw, pitch] - 相对于初始位置的增量
    # 手臂位姿: [x,y,z,yaw,pitch,roll] (位置米, 姿态度)
    test_cases = [
        ("测试1-初始抬高+手臂展开", 5.0, 
         [0.0, 0.0, 0.0],  # 底盘不动
         [0.0, 0.3, 0.0, 0.0],  # 躯干抬高
         [0.1, 0.4, 1.0, 0.0, 0.0, 0.0],  # 左臂
         [0.1, -0.4, 1.0, 0.0, 0.0, 0.0]), # 右臂
         
        ("测试2-底盘后退+手臂前摆", 5.0, 
         [-0.3, 0.0, 0.0],  # 底盘后退
         [0.2, 0.3, 0.0, 0.0],  # 躯干前移
         [0.5, 0.2, 1.15, 0.0, -90, 0.0],  # 左臂
         [0.5, -0.2, 1.15, 0.0, -90, 0.0]), # 右臂
         
        ("测试3-底盘前进+偏航30°", 5.0, 
         [0.0, 0.0, 0.0],  # 底盘前进并旋转
         [0.2, 0.3, 0.52356, 0.0],  # 躯干偏航+30°
         [0.3, 0.2, 1.0, 0.0, -90, 0.0],  # 左臂
         [0.3, -0.2, 1.0, 0.0, -90, 0.0]), # 右臂
         
        ("测试4-底盘旋转+俯仰", 6.0, 
         [-0.3, 0.0, 1.57],  # 底盘旋转90度
         [0.2, 0.3, 0.0, 0.524],  # 躯干俯仰+30°
         [1.2, 0.2, 1.15, 0.0, -90, 0.0],  # 左臂
         [1.2, -0.2, 1.15, 0.0, -90, 0.0]), # 右臂
         
        ("测试5-复位", 5.0, 
         [0.0, 0.0, 0.0],  # 底盘复位
         [0.0, 0.0, 0.0, 0.0],  # 躯干复位
         [0.1, 0.4, 0.7, 0.0, 0.0, 0.0],  # 左臂复位
         [0.1, -0.4, 0.7, 0.0, 0.0, 0.0]), # 右臂复位
    ]

    rospy.loginfo("开始发布底盘+躯干+手臂组合测试数据...")

    for idx, (name, desire_time, base_pose, torso_rel_pose, left_arm_pose, right_arm_pose) in enumerate(test_cases, 1):
        # 解包躯干位置数据
        lx, lz, az, ay = torso_rel_pose
        
        # 计算躯干绝对坐标：初始位置 + 增量
        abs_x = initialTorsoPose_[0] + lx
        abs_z = initialTorsoPose_[2] + lz
        torso_pose = [abs_x, abs_z, az, ay]  # [x, z, yaw, pitch]
        
        # 手臂角度转弧度
        left_arm_rad = degrees_to_radians(left_arm_pose)
        right_arm_rad = degrees_to_radians(right_arm_pose)

        rospy.loginfo(f"\n=== 第{idx}组测试: {name} ===")
        rospy.loginfo(f"  期望时间: {desire_time:.1f} 秒")
        rospy.loginfo(f"  底盘位姿: [x={base_pose[0]:.2f}m, y={base_pose[1]:.2f}m, yaw={base_pose[2]:.2f}rad]")
        rospy.loginfo(f"  躯干相对: [{lx}, {lz}, {az:.2f}, {ay:.2f}]")
        rospy.loginfo(f"  躯干绝对: [x={abs_x:.3f}m, z={abs_z:.3f}m, yaw={az:.2f}rad, pitch={ay:.2f}rad]")
        rospy.loginfo(f"  左臂位姿(度): [x={left_arm_pose[0]:.2f}m, y={left_arm_pose[1]:.2f}m, z={left_arm_pose[2]:.2f}m, yaw={left_arm_pose[3]:.1f}°, pitch={left_arm_pose[4]:.1f}°, roll={left_arm_pose[5]:.1f}°]")
        rospy.loginfo(f"  右臂位姿(度): [x={right_arm_pose[0]:.2f}m, y={right_arm_pose[1]:.2f}m, z={right_arm_pose[2]:.2f}m, yaw={right_arm_pose[3]:.1f}°, pitch={right_arm_pose[4]:.1f}°, roll={right_arm_pose[5]:.1f}°]")

        # 构建多指令列表 - 同时发送三个规划器的指令
        timed_cmd_vec = [
            {
                'planner_index': 0,  # 底盘规划器
                'desire_time': desire_time,
                'cmd_vec': base_pose  # [x, y, yaw]
            },
            {
                'planner_index': 2,  # 躯干规划器
                'desire_time': desire_time,
                'cmd_vec': torso_pose  # [x, z, yaw, pitch]
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
            is_sync=True  # 底盘、躯干和手臂同步运动
        )

        if success:
            rospy.loginfo(f"  ✅ 命令发送成功，实际执行时间: {actual_time:.2f} 秒")
            rospy.loginfo(f"  三个规划器(0,2,5)同步执行")
        else:
            rospy.logwarn(f"  ❌ 命令发送失败: {message}")
            actual_time = 0.0

        # 等待运动完成再发下一组
        rospy.sleep(actual_time + 0.5)
        rospy.loginfo(f"  {name} 完成!")

    rospy.loginfo("\n所有底盘+躯干+手臂组合测试数据发布完成！")

# -------------- 主入口 --------------
def main():
    try:
        # 运行组合测试（底盘+躯干+手臂）
        execute_combined_base_torso_arm_tests()
        
    except rospy.ROSInterruptException:
        rospy.logwarn("ROS 中断异常")
    except Exception as e:
        rospy.logerr(f"程序执行出错: {e}")

if __name__ == '__main__':
    main()