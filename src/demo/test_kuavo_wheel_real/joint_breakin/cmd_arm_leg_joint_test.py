#!/usr/bin/env python3
import sys
import os

sys.path.append(os.path.dirname(__file__))

import rospy
import lb_ctrl_api as ct
import math

def interpolate_poses(pose1, pose2, num_steps):
    """在两个姿态之间生成插值点"""
    interpolated = []
    for i in range(1, num_steps + 1):
        t = i / num_steps
        interpolated.append([p1 + (p2 - p1) * t for p1, p2 in zip(pose1, pose2)])
    return interpolated

def send_interpolated_motion(leg_start, arm_start, leg_end, arm_end, total_time, num_steps=50):
    """发送插值过渡动作"""
    step_time = total_time / num_steps
    
    leg_interp = interpolate_poses(leg_start, leg_end, num_steps)
    arm_interp = interpolate_poses(arm_start, arm_end, num_steps)
    
    for i in range(num_steps):
        leg_angles_rad = [math.radians(angle) for angle in leg_interp[i]]
        arm_angles_rad = [math.radians(angle) for angle in arm_interp[i]]
        
        timed_cmd_vec = [
            {
                'planner_index': 3,
                'desire_time': step_time,
                'cmd_vec': leg_angles_rad
            },
            {
                'planner_index': 8,
                'desire_time': step_time,
                'cmd_vec': arm_angles_rad[:7]
            },
            {
                'planner_index': 9,
                'desire_time': step_time,
                'cmd_vec': arm_angles_rad[7:14]
            }
        ]
        
        ct.send_timed_multi_commands(timed_cmd_vec=timed_cmd_vec, is_sync=False, verbose=False)
        rospy.sleep(step_time)

def execute_leg_tests():
    """依次发布下肢关节数据，并等待每次运动结束"""
    
    # 测试用例列表： (名称, 时间, 下肢角度, 手臂角度)
    # 注意：角度单位为度，后续会转换为弧度
    test_cases = [
        # 帧0（起始位置）
        ("帧0-起始位置", 0.5, 
         [0.0, 0.0, 0.0, 0.0],  # 下肢角度
         [0.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0]),
        
        # 帧1
        ("帧1", 1.0, 
         [20.0, -30.0, -5.0, 0.0],  # 下肢角度
         [10.0, 15.0, 15.0, -60.0, 6.0, 6.0, 6.0, 10.0, -15.0, -15.0, -60.0, -6.0, -6.0, 6.0]),
        
        # 帧2
        ("帧2", 1.0, 
         [40.0, -80.0, -10.0, 0.0],  # 下肢角度
         [45.0, 45.0, 30.0, -100.0, 55.0, 15.0, 15.0, 45.0, -45.0, -30.0, -100.0, -55.0, -15.0, 15.0]),
        
        # 帧3
        ("帧3", 3.0, 
         [50.0, -100.0, 50.0, 175.0],  # 下肢角度
         [85.0, 60.0, 45.0, -120.0, 80.0, 25.0, 25.0, 85.0, -60.0, -45.0, -120.0, -80.0, -25.0, 25.0]),
        
        # 帧4
        ("帧4", 3.0, 
         [60.0, -140.0, 85.0, 0.0],  # 下肢角度
         [20.0, 120.0, 70.0, -100.0, 0.0, 25.0, 25.0, 20.0, -120.0, -70.0, -100.0, 0.0, -25.0, 25.0]),
        
        # 帧5
        ("帧5", 3.0, 
         [60.0, -140.0, 85.0, -175.0],  # 下肢角度
         [-50.0, 25.0, -35.0, -90.0, -80.0, 25.0, 25.0, -50.0, -25.0, 35.0, -90.0, 80.0, -25.0, 25.0]),
        
        # 帧6
        ("帧6", 3.0, 
         [50.0, -100.0, 50.0, 0.0],  # 下肢角度
         [-50.0, 25.0, -35.0, -90.0, -80.0, 25.0, 25.0, -50.0, -25.0, 35.0, -90.0, 80.0, -25.0, 25.0]),
        
        # 帧7
        ("帧7", 3.0, 
         [40.0, -80.0, 160.0, 0.0],  # 下肢角度
         [-145.0, 15.0, -15.0, -50.0, -40.0, 15.0, 15.0, -145.0, -15.0, 15.0, -50.0, 40.0, -15.0, 15.0]),
        
        # 帧8
        ("帧8", 2.0, 
         [40.0, -80.0, 50.0, 0.0],  # 下肢角度
         [-50.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0, -50.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0]),
        
        # 帧9
        ("帧9", 1.5, 
         [20.0, -40.0, 20.0, 0.0],  # 下肢角度
         [0.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0]),
        
        # 帧10（结束位置）
        ("帧10-结束位置", 1.0, 
         [0.0, 0.0, 0.0, 0.0],  # 下肢角度
         [0.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0]),
    ]
    # 计算一轮所需时间
    time_per_round = sum(tc[1] for tc in test_cases)
    print(f"一轮动作大约需要 {time_per_round:.1f} 秒")
    
    num_rounds = int(input("请输入要执行的轮数: "))
    rospy.loginfo(f"将执行 {num_rounds} 轮动作序列")

    # 初始化节点
    rospy.init_node('leg_joint_publisher', anonymous=True)

    # 等待连接建立
    rospy.sleep(0.01)

    rospy.loginfo("开始发布组合关节测试数据...")

    prev_leg = test_cases[0][2]
    prev_arm = test_cases[0][3]

    for round_num in range(1, num_rounds + 1):
        rospy.loginfo(f"\n========== 第 {round_num}/{num_rounds} 轮 ==========")
        
        for idx, (name, desire_time, leg_angles_deg, arm_angles_deg) in enumerate(test_cases, 1):
            send_interpolated_motion(
                prev_leg, prev_arm,
                leg_angles_deg, arm_angles_deg,
                desire_time, num_steps=50
            )
            
            rospy.loginfo(f"  {name} 到达!")
            prev_leg = leg_angles_deg
            prev_arm = arm_angles_deg

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