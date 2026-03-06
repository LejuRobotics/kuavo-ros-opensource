#!/usr/bin/env python3
import sys
import os

# 方法1：动态添加路径
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))

import rospy
import lb_ctrl_api as ct
import math

def execute_leg_tests():
    """依次发布下肢关节数据，并等待每次运动结束"""
    
    # 初始化节点
    rospy.init_node('leg_joint_publisher', anonymous=True)

    # 等待连接建立
    rospy.sleep(0.01)

    # 测试用例列表： (名称, 时间, 下肢角度, 手臂角度)
    # 注意：角度单位为度，后续会转换为弧度
    # 规划器索引说明：
    #   3: 下肢
    #   6: 双臂
    test_cases = [
        ("测试1-正常组合", 4.0, 
         [14.90, -32.01, 18.03, 0.0],  # 下肢角度
         [-30, 20, 15, -45, 25, 10, -35, -30, -20, -15, -45, -25, -10, -35]),  # 手臂角度
         
        ("测试2-正常组合", 4.0, 
         [14.90, -32.01, 18.03, 30.0],  # 下肢角度
         [-20, 30, -25, -20, 40, -15, 25, -20, -30, 25, -20, -40, 15, 25]),  # 手臂角度
         
        ("测试3-重复索引", 4.0, 
         [14.90, -32.01, 18.03, 0.0],  # 下肢角度
         [14.90, -32.01, 18.03, 0.0]),  # 故意用下肢角度作为手臂（错误，应该失败）
         
        ("测试4-零位组合", 4.0, 
         [0.0] * 4,  # 下肢零位
         [0.0] * 14),  # 手臂零位
    ]

    rospy.loginfo("开始发布组合关节测试数据...")

    for idx, (name, desire_time, leg_angles_deg, arm_angles_deg) in enumerate(test_cases, 1):
        rospy.loginfo(f"\n=== 第{idx}组测试: {name} ===")
        rospy.loginfo(f"  期望时间: {desire_time:.1f} 秒")
        rospy.loginfo(f"  下肢角度(度): {leg_angles_deg}")
        rospy.loginfo(f"  手臂角度(度): {arm_angles_deg[:7]} ...")  # 只显示前几个
        
        # 角度转换为弧度
        leg_angles_rad = [math.radians(angle) for angle in leg_angles_deg]
        arm_angles_rad = [math.radians(angle) for angle in arm_angles_deg]
        
        rospy.loginfo(f"  下肢角度(弧度): {[f'{angle:.3f}' for angle in leg_angles_rad]}")
        rospy.loginfo(f"  手臂角度(弧度): {[f'{angle:.3f}' for angle in arm_angles_rad[:3]]} ...")

        # # 发送命令
        # success, actual_time = ct.send_timed_single_command(
        #     planner_index=3,  # 假设4是双臂控制的索引，请根据实际情况调整
        #     desire_time=desire_time,
        #     cmd_vec=joint_angles_rad
        # )
        # 发送命令 - 修改这里使用新的多指令接口，但只发单个指令
        timed_cmd_vec = [
            {
                'planner_index': 3,  # 下肢规划器索引
                'desire_time': desire_time,
                'cmd_vec': leg_angles_rad
            },
            {
                'planner_index': 6,  # 双臂规划器索引
                'desire_time': desire_time,
                'cmd_vec': arm_angles_rad
            }
        ]
        
        success, actual_time, message = ct.send_timed_multi_commands(
            timed_cmd_vec=timed_cmd_vec,
            is_sync=True  # 单指令时同步标志不影响
        )

        if success:
            rospy.loginfo(f"  命令发送成功，实际执行时间: {actual_time:.2f} 秒")
        else:
            rospy.logwarn(f"  命令发送失败")
            actual_time = 0.0

        # 等待运动完成再发下一组
        rospy.sleep(actual_time + 0.5)
        rospy.loginfo(f"  {name} 完成!")

    rospy.loginfo("\n所有双臂关节测试数据发布完成！")

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