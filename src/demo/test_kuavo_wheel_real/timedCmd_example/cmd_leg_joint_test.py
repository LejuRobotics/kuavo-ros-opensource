#!/usr/bin/env python3
import sys
import os
import math

# 方法1：动态添加路径
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import rospy
import lb_ctrl_api as ct

def execute_leg_tests():
    """依次发布下肢关节数据，并等待每次运动结束，支持循环多轮执行"""
    
    # 初始化节点
    rospy.init_node('leg_joint_publisher', anonymous=True)

    # 等待连接建立
    rospy.sleep(0.01)

    # 测试用例列表： (名称, 时间, 关节角度)
    # 注意：角度单位为度，后续会转换为弧度
    test_cases = [
        ("action_1", 2.0, [15.0, -30.0, 18.0, 0.0]),
        ("action_2", 2.0, [30.0, -70.0, 40.0, 90.0]),
        ("action_3", 4.0, [15.0, -30.0, 18.0, -90.0]),
        ("action_4", 2.0, [15.0, -30.0, 18.0, 0.0]),
        ("action_5", 2.0, [0.0] * 4),
    ]

    # 计算一轮动作总耗时
    time_per_round = sum(tc[1] for tc in test_cases)
    rospy.loginfo(f"一轮动作大约需要 {time_per_round:.1f} 秒")
    
    # 用户输入执行轮数
    num_rounds = int(input("请输入要执行的磨线轮数: "))
    rospy.loginfo(f"将执行 {num_rounds} 轮下肢关节测试动作")

    rospy.loginfo("开始发布下肢关节测试数据...")

    # 循环执行多轮动作
    for round_num in range(1, num_rounds + 1):
        rospy.loginfo(f"\n========== 第 {round_num}/{num_rounds} 轮 ==========")
        
        for idx, (name, desire_time, joint_angles_deg) in enumerate(test_cases, 1):
            rospy.loginfo(f"\n=== 第{idx}组测试: {name} ===")
            # rospy.loginfo(f"  期望时间: {desire_time:.1f} 秒")
            # rospy.loginfo(f"  关节角度(度): {joint_angles_deg}")
            
            # 角度转换为弧度
            joint_angles_rad = [math.radians(angle) for angle in joint_angles_deg]
            # rospy.loginfo(f"  关节角度(弧度): {[f'{angle:.3f}' for angle in joint_angles_rad]}")

            # 发送命令
            success, actual_time = ct.send_timed_single_command(
                planner_index=3,  # 下肢控制索引
                desire_time=desire_time,
                cmd_vec=joint_angles_rad
            )

            if success:
                # rospy.loginfo(f"  命令发送成功，实际执行时间: {actual_time:.2f} 秒")
                pass
            else:
                rospy.logwarn(f"  命令发送失败")

            # 等待运动完成再发下一组
            rospy.sleep(actual_time + 0.5)
            rospy.loginfo(f"  {name} 完成!")

    rospy.loginfo("\n所有下肢关节测试数据发布完成！")

# -------------- 主入口 --------------
def main():
    try:
        execute_leg_tests()
    except rospy.ROSInterruptException:
        rospy.logwarn("ROS 中断异常")
    except Exception as e:
        rospy.logerr(f"程序执行出错: {e}")

if __name__ == '__main__':
    main()