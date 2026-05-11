#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
使用服务来修改ruckig规划器参数的测试脚本

"""

import rospy
import lb_ctrl_api as ct

def execute_ruckig_param_setting_tests():
     # 初始化ROS节点
    rospy.init_node('simple_ruckig_demo', anonymous=True)
    rospy.loginfo("🚀 Ruckig规划器参数设置Demo启动...")
    
    # ========== Demo 1: 设置Base Pose Planner (索引0) ==========
    rospy.loginfo("\n📋 Demo 1: 设置Base Pose Planner")
    
    # 构建形参所需参数
    planner_index_1 = 0          # Base Pose Planner
    is_sync_1 = True             # 同步模式或非同步模式
    velocity_max_1 = [0.2, 0.2, 0.2]      # 3个自由度的最大速度
    acceleration_max_1 = [2.0, 2.0, 1.5]  # 最大加速度
    jerk_max_1 = [20.0, 15.0, 12.0]     # 最大急动度

    # 调用函数
    success_1 = ct.set_ruckig_planner_params(
        planner_index=planner_index_1,
        is_sync=is_sync_1,
        velocity_max=velocity_max_1,
        acceleration_max=acceleration_max_1,
        jerk_max=jerk_max_1
    )

    if success_1:
        rospy.loginfo("✅ Base Pose Planner 设置完成!")
    else:
        rospy.loginfo("❌ Base Pose Planner 设置失败!")
    
    # 等待2秒
    rospy.sleep(2.0)

# -------------- 主入口 --------------
def main():
    try:
        execute_ruckig_param_setting_tests()
    except rospy.ROSInterruptException:
        rospy.logwarn("ROS 中断异常")
    except Exception as e:
        rospy.logerr(f"程序执行出错: {e}")

if __name__ == '__main__':
    main()