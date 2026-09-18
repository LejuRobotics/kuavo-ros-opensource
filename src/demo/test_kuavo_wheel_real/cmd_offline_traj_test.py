#!/usr/bin/env python3
import rospy
import time
import lb_ctrl_api as ct

# -------------- 全局变量 --------------
execution_status = False

# -------------- 业务函数 --------------
def execute_left_arm_test():
    """左臂运动测试"""
    rospy.loginfo("=" * 50)
    rospy.loginfo("左臂离线轨迹测试")
    rospy.loginfo("=" * 50)
    
    # 左臂轨迹 - 直接使用给定数据
    traj = [{
        'planner_index': 0,      # 左臂
        'frame': 0,              # 世界系
        'timed_traj': [
            {'desire_time': 0.0, 'cmd_vec': [0.3, 0.2, 0.3, 0.0, 0.0, 0.0]},
            {'desire_time': 1.0, 'cmd_vec': [0.4, 0.2, 0.35, 0.1, 0.0, 0.0]},
            {'desire_time': 2.0, 'cmd_vec': [0.5, 0.2, 0.4, 0.2, 0.0, 0.0]},
            {'desire_time': 3.0, 'cmd_vec': [0.6, 0.2, 0.45, 0.3, 0.0, 0.0]}
        ]
    }]
    
    rospy.loginfo(f"轨迹点数: {len(traj[0]['timed_traj'])}")
    rospy.loginfo(f"总时长: {traj[0]['timed_traj'][-1]['desire_time']:.2f}s")
    rospy.loginfo(f"起始点: {traj[0]['timed_traj'][0]['cmd_vec']}")
    rospy.loginfo(f"终点点: {traj[0]['timed_traj'][-1]['cmd_vec']}")
    
    # 发送轨迹
    success, msg = ct.set_lb_multi_timed_offline_traj(traj)
    if success:
        rospy.loginfo("✅ 左臂轨迹设置成功!")
        rospy.loginfo("开始执行...")
        rospy.sleep(traj[0]['timed_traj'][-1]['desire_time'] + 1.0)
        rospy.loginfo("✅ 左臂运动完成!")
    else:
        rospy.logerr(f"❌ 左臂轨迹设置失败: {msg}")


def execute_right_arm_test():
    """右臂运动测试"""
    rospy.loginfo("\n" + "=" * 50)
    rospy.loginfo("右臂离线轨迹测试")
    rospy.loginfo("=" * 50)
    
    # 右臂轨迹
    traj = [{
        'planner_index': 1,      # 右臂
        'frame': 0,              # 世界系
        'timed_traj': [
            {'desire_time': 0.0, 'cmd_vec': [0.3, -0.2, 0.3, 0.0, 0.0, 0.0]},
            {'desire_time': 1.0, 'cmd_vec': [0.4, -0.25, 0.35, 0.0, 0.1, 0.0]},
            {'desire_time': 2.0, 'cmd_vec': [0.5, -0.3, 0.4, 0.0, 0.2, 0.0]},
            {'desire_time': 3.0, 'cmd_vec': [0.6, -0.35, 0.45, 0.0, 0.3, 0.0]}
        ]
    }]
    
    rospy.loginfo(f"轨迹点数: {len(traj[0]['timed_traj'])}")
    rospy.loginfo(f"总时长: {traj[0]['timed_traj'][-1]['desire_time']:.2f}s")
    rospy.loginfo(f"起始点: {traj[0]['timed_traj'][0]['cmd_vec']}")
    rospy.loginfo(f"终点点: {traj[0]['timed_traj'][-1]['cmd_vec']}")
    
    # 发送轨迹
    success, msg = ct.set_lb_multi_timed_offline_traj(traj)
    if success:
        rospy.loginfo("✅ 右臂轨迹设置成功!")
        rospy.loginfo("开始执行...")
        rospy.sleep(traj[0]['timed_traj'][-1]['desire_time'] + 1.0)
        rospy.loginfo("✅ 右臂运动完成!")
    else:
        rospy.logerr(f"❌ 右臂轨迹设置失败: {msg}")


def execute_dual_arm_test():
    """双臂协同运动测试"""
    rospy.loginfo("\n" + "=" * 50)
    rospy.loginfo("双臂协同运动测试")
    rospy.loginfo("=" * 50)
    
    # 双臂轨迹
    traj = [
        {
            'planner_index': 0,      # 左臂
            'frame': 0,
            'timed_traj': [
                {'desire_time': 0.0, 'cmd_vec': [0.3, 0.2, 0.3, 0.0, 0.0, 0.0]},
                {'desire_time': 1.0, 'cmd_vec': [0.3, 0.2, 0.3, 0.0, 0.0, 0.0]},
                {'desire_time': 2.0, 'cmd_vec': [0.3, 0.2, 0.3, 0.0, 0.0, 0.0]},
                {'desire_time': 3.0, 'cmd_vec': [0.3, 0.2, 0.3, 0.0, 0.0, 0.0]},
                {'desire_time': 4.0, 'cmd_vec': [0.3, 0.2, 0.3, 0.0, 0.0, 0.0]},
            ]
        },
        {
            'planner_index': 1,      # 右臂
            'frame': 1,
            'timed_traj': [
                {'desire_time': 0.0, 'cmd_vec': [0.3, -0.2, 0.3, 0.0, 0.0, 0.0]},
                {'desire_time': 1.0, 'cmd_vec': [0.3, -0.2, 0.3, 0.0, 0.0, 0.0]},
                {'desire_time': 2.0, 'cmd_vec': [0.3, -0.2, 0.3, 0.0, 0.0, 0.0]},
                {'desire_time': 3.0, 'cmd_vec': [0.3, -0.2, 0.3, 0.0, 0.0, 0.0]},
                {'desire_time': 4.0, 'cmd_vec': [0.3, -0.2, 0.3, 0.0, 0.0, 0.0]},
            ]
        },
        {
            'planner_index': 2,      # 躯干
            'frame': 0,
            'timed_traj': [
                {'desire_time': 0.0, 'cmd_vec': [0.3, 0.2, 0.0, 0.0]},
                {'desire_time': 1.0, 'cmd_vec': [0.3, 0.2, 0.0, 0.0]},
                {'desire_time': 2.0, 'cmd_vec': [0.3, 0.2, 0.0, 0.0]},
                {'desire_time': 3.0, 'cmd_vec': [0.3, 0.2, 0.0, 0.0]},
                {'desire_time': 4.0, 'cmd_vec': [0.3, 0.2, 0.0, 0.0]},
            ]
        }
    ]
    
    rospy.loginfo(f"左臂轨迹点数: {len(traj[0]['timed_traj'])}")
    rospy.loginfo(f"右臂轨迹点数: {len(traj[1]['timed_traj'])}")
    rospy.loginfo(f"总时长: {traj[0]['timed_traj'][-1]['desire_time']:.2f}s")
    
    # 发送轨迹
    success, msg = ct.set_lb_multi_timed_offline_traj(traj)
    if success:
        rospy.loginfo("✅ 双臂轨迹设置成功!")
    else:
        rospy.logerr(f"❌ 双臂轨迹设置失败: {msg}")
    

    # 启动轨迹执行
    ct.set_offline_trajectory_enable(True)


# -------------- 主入口 --------------
def main():
    try:
        rospy.init_node('arm_offline_test', anonymous=True)
        # 选择要执行的测试
        # execute_left_arm_test()    # 左臂测试
        # execute_right_arm_test()   # 右臂测试
        execute_dual_arm_test()      # 双臂测试
        
        rospy.loginfo("\n" + "=" * 50)
        rospy.loginfo("所有测试完成！")
        rospy.loginfo("=" * 50)
        
    except rospy.ROSInterruptException:
        rospy.logwarn("ROS 中断异常")
    except Exception as e:
        rospy.logerr(f"程序执行出错: {e}")

if __name__ == '__main__':
    main()