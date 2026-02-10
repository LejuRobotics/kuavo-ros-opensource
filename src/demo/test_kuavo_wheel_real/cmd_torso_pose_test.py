#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import lb_ctrl_api as ct
import argparse

# -------------- 全局变量 --------------
reach_time = 0.0
initialTorsoPose_ = [0.0, 0.0, 0.0]

# -------------- 回调函数 --------------
def time_callback(msg):
    """接收躯干到达目标所需时间"""
    global reach_time
    reach_time = msg.data
    rospy.loginfo(f"reach_time is {reach_time:.3f} s")

# -------------- 业务函数 --------------
def build_twist(lx=0.0, ly=0.0, lz=0.0, ax=0.0, ay=0.0, az=0.0):
    """快速构造 Twist 消息"""
    tw = Twist()
    tw.linear.x, tw.linear.y, tw.linear.z = lx, ly, lz
    tw.angular.x, tw.angular.y, tw.angular.z = ax, ay, az
    return tw

def execute_torso_tests():
    """依次发布躯干位姿测试数据，并等待每次运动结束"""
    global reach_time, initialTorsoPose_

    # 初始化节点
    rospy.init_node('torso_pose_publisher', anonymous=True)
    
    # 发布 / 订阅
    pub = rospy.Publisher('/cmd_lb_torso_pose', Twist, queue_size=10)
    rospy.Subscriber('/lb_torso_pose_reach_time', Float32, time_callback)

    # 首先获取躯干初始位姿
    rospy.loginfo("正在获取躯干初始位姿...")

    start_time = rospy.get_time()  # 记录开始时间
    success, initial_pose = ct.get_torso_initial_pose(True)
    end_time = rospy.get_time()    # 记录结束时间

    execution_time = end_time - start_time

    rospy.loginfo(f"✅ 获取躯干初始位姿完成，耗时: {execution_time:.3f} 秒")
    
    if not success:
        rospy.logerr("❌ 无法获取躯干初始位姿，退出程序")
        return
    
    # 提取初始位置
    initialTorsoPose_ = initial_pose['position']

    parser = argparse.ArgumentParser(description='躯干测试程序')
    parser.add_argument('--no-reset', action='store_true', help='跳过躯干重置')
    args = parser.parse_args()
    reset_torso = not args.no_reset
    resetTime = 0
    if reset_torso:
        resetTime = ct.reset_torso_to_initial()
    # 等待连接建立
    rospy.sleep(resetTime + 0.5)

    # 测试用例列表： (名称, lx, ly, lz, ax, ay, az)
    # 注意：这里的lx, ly, lz是相对于初始位置的增量
    test_cases = [
        ("初始抬高", 0.0, 0.0, 0.3, 0.0,  0.0,  0.0),
        ("前移",     0.2, 0.0, 0.3, 0.0,  0.0,  0.0),
        ("偏航+30°", 0.2, 0.0, 0.3, 0.0,  0.0,  0.52356),
        ("偏航-30°", 0.2, 0.0, 0.3, 0.0,  0.0, -0.52356),
        ("俯仰-10°", 0.2, 0.0, 0.3, 0.0, -0.1745, 0.0),
        ("俯仰+30°", 0.2, 0.0, 0.3, 0.0,  0.524, 0.0),
        ("复位",     0.0, 0.0, 0.0, 0.0,  0.0,  0.0),
    ]

    rospy.loginfo("开始发布躯干位姿测试数据...")
    rospy.loginfo(f"初始躯干位置: {initialTorsoPose_}")

    for idx, (name, lx, ly, lz, ax, ay, az) in enumerate(test_cases, 1):
        # 计算绝对坐标：初始位置 + 增量
        abs_x = initialTorsoPose_[0] + lx
        abs_y = initialTorsoPose_[1] + ly
        abs_z = initialTorsoPose_[2] + lz
        
        rospy.loginfo(f"\n=== 第{idx}组测试: {name} ===")
        rospy.loginfo(f"  相对目标: [{lx}, {ly}, {lz}, {ax}, {ay}, {az}]")
        rospy.loginfo(f"  绝对坐标: [{abs_x:.3f}, {abs_y:.3f}, {abs_z:.3f}]")

        reach_time = 0.0
        pub.publish(build_twist(abs_x, abs_y, abs_z, ax, ay, az))

        # 等待底册返回 reach_time
        while reach_time == 0.0 and not rospy.is_shutdown():
            rospy.sleep(0.1)

        # 等待运动完成再发下一组
        rospy.sleep(reach_time + 0.5)
        rospy.loginfo(f"  {name} 完成!")

    rospy.loginfo("\n所有躯干位姿测试数据发布完成！")

# -------------- 主入口 --------------
def main():
    try:
        execute_torso_tests()
    except rospy.ROSInterruptException:
        rospy.logwarn("ROS 中断异常")
    except Exception as e:
        rospy.logerr(f"程序执行出错: {e}")

if __name__ == '__main__':
    main()