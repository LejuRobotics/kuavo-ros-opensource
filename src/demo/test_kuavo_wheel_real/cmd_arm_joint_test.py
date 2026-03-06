#!/usr/bin/env python3
import rospy
import time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
import lb_ctrl_api as ct

# -------------- 全局变量 --------------
reach_time = 0.0
joint_names = [f'joint{i}' for i in range(1, 15)]   # 14 个关节

# -------------- 回调函数 --------------
def time_callback(msg):
    global reach_time
    reach_time = msg.data
    rospy.loginfo(f"reach_time is {reach_time:.3f} s")

# -------------- 业务函数 --------------
def build_joint_state(positions):
    """快速构造 JointState 消息"""
    js = JointState()
    js.header.stamp = rospy.Time.now()
    js.name  = joint_names
    js.position = positions
    return js

def execute_arm_tests():
    """依次发布若干组 14 关节角度，并等待每次运动结束"""
    global reach_time

    # 初始化节点
    rospy.init_node('test_arm_joint_publisher', anonymous=True)

    # 发布 / 订阅
    pub = rospy.Publisher('/kuavo_arm_traj', JointState, queue_size=10)
    rospy.Subscriber('/lb_arm_joint_reach_time/left', Float32, time_callback)

    # 等待连接
    rospy.sleep(1.0)

    # 测试用例列表： (名称, 14 个关节角度)
    test_cases = [
        ("展开双臂",   [-30, 20, 15, -45, 25, 10, -35,
                         -30,-20,-15, -45,-25,-10, -35]),
        ("弯曲收回",   [-20, 30, -25, -20, 40, -15, 25,
                         -20,-30, 25, -20,-40, 15,  25]),
        ("回到零位",   [0.0]*14),
    ]

    rospy.loginfo("开始发布上肢关节测试数据...")

    for idx, (name, pos) in enumerate(test_cases, 1):
        rospy.loginfo(f"\n=== 第{idx}组测试: {name} ===")
        rospy.loginfo(f"  目标角度: {pos}")

        reach_time = 0.0
        pub.publish(build_joint_state(pos))

        # 等待底册返回 reach_time
        while reach_time == 0.0 and not rospy.is_shutdown():
            rospy.sleep(0.1)

        # 等待运动完成再发下一组
        rospy.sleep(reach_time + 0.5)
        rospy.loginfo(f"  {name} 完成!")

    rospy.loginfo("\n所有上肢关节测试数据发布完成！")

# -------------- 主入口 --------------
def main():
    try:
        # 根据需要开关快速模式
        # set_arm_quick_mode(True)
        ct.set_arm_quick_mode(0)
        execute_arm_tests()
        ct.set_arm_quick_mode(0)
    except rospy.ROSInterruptException:
        rospy.logwarn("ROS 中断异常")
    except Exception as e:
        rospy.logerr(f"程序执行出错: {e}")

if __name__ == '__main__':
    main()