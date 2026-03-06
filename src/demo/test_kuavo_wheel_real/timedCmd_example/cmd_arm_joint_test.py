#!/usr/bin/env python3
import sys
import os

# 方法1：动态添加路径
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import rospy
import lb_ctrl_api as ct

def execute_arm_tests():
    """依次发布双臂关节数据，并等待每次运动结束"""
    
    # 初始化节点
    rospy.init_node('arm_joint_publisher', anonymous=True)

    # 等待连接建立
    rospy.sleep(0.01)

    # 测试用例列表： (名称, 期望时间, [14个关节角度])
    # 注意：角度单位为度，后续会转换为弧度
    test_cases = [
        ("展开双臂", 2.0, [-30, 20, 15, -45, 25, 10, -35,
                           -30, -20, -15, -45, -25, -10, -35]),
        ("弯曲收回", 2.0, [-20, 30, -25, -20, 40, -15, 25,
                           -20, -30, 25, -20, -40, 15, 25]),
        ("回到零位", 2.0, [0.0] * 14),
    ]

    rospy.loginfo("开始发布双臂关节测试数据...")

    for idx, (name, desire_time, joint_angles_deg) in enumerate(test_cases, 1):
        rospy.loginfo(f"\n=== 第{idx}组测试: {name} ===")
        rospy.loginfo(f"  期望时间: {desire_time:.1f} 秒")
        rospy.loginfo(f"  关节角度(度): {joint_angles_deg}")
        
        # 角度转换为弧度
        import math
        joint_angles_rad = [math.radians(angle) for angle in joint_angles_deg]
        rospy.loginfo(f"  关节角度(弧度): {[f'{angle:.3f}' for angle in joint_angles_rad]}")

        # 发送命令
        success, actual_time = ct.send_timed_single_command(
            planner_index=6,  # 假设4是双臂控制的索引，请根据实际情况调整
            desire_time=desire_time,
            cmd_vec=joint_angles_rad
        )

        if success:
            rospy.loginfo(f"  命令发送成功，实际执行时间: {actual_time:.2f} 秒")
        else:
            rospy.logwarn(f"  命令发送失败")

        # 等待运动完成再发下一组
        rospy.sleep(actual_time + 0.5)
        rospy.loginfo(f"  {name} 完成!")

    rospy.loginfo("\n所有双臂关节测试数据发布完成！")

# -------------- 主入口 --------------
def main():
    try:
        execute_arm_tests()
    except rospy.ROSInterruptException:
        rospy.logwarn("ROS 中断异常")
    except Exception as e:
        rospy.logerr(f"程序执行出错: {e}")

if __name__ == '__main__':
    main()