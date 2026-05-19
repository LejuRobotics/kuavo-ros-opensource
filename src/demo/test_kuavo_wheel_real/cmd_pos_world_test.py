#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import time
import lb_ctrl_api as ct
    
# 全局变量
reachTime = 0.0

def time_callback(msg):
    """时间回调函数"""
    global reachTime
    reachTime = msg.data
    print(f"reach_time is {reachTime}")

def execute_twist_tests():
    """执行Twist测试的主函数"""
    global reachTime

    # 初始化ROS节点
    rospy.init_node('test_twist_publisher', anonymous=True)
    
    # 创建发布器
    pub = rospy.Publisher('/cmd_pose_world', Twist, queue_size=10)

    # 创建订阅器
    time_sub = rospy.Subscriber('/lb_cmd_pose_reach_time', Float32, time_callback)

    # 等待发布器建立连接
    time.sleep(1)
    
    # 测试数据列表
    test_cases = [
        # (名称, linear_x, linear_y, linear_z, angular_x, angular_y, angular_z)
        # ("测试数据1", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        ("测试数据2", 0.3, 0.0, 0.0, 0.0, 0.0, 0.0),
        ("测试数据3", 0.3, 0.3, 0.0, 0.0, 0.0, 1.57),
        ("测试数据4", 0.0, 0.3, 0.0, 0.0, 0.0, 3.14),
        ("测试数据5", 0.0, 0.0, 0.0, 0.0, 0.0, -1.57),
    ]
    
    print("开始发布测试数据...")
    
    for i, (name, lx, ly, lz, ax, ay, az) in enumerate(test_cases, 1):
        print(f"\n=== 第{i}组测试: {name} ===")
        
        # 创建Twist消息
        twist_msg = Twist()
        twist_msg.linear.x = lx
        twist_msg.linear.y = ly
        twist_msg.linear.z = lz
        twist_msg.angular.x = ax
        twist_msg.angular.y = ay
        twist_msg.angular.z = az
        
        print(f"  位置: ({lx}, {ly})")
        print(f"  偏航角: {az}")
        
        reachTime = 0.0  # 重置时间
        pub.publish(twist_msg)
        
        # 等待直到收到reachTime
        while reachTime == 0.0:
            rospy.sleep(0.1)
        
        # 等待运动完成
        rospy.sleep(reachTime + 0.5)
        print(f"  {name} 完成!")
    
    print("\n所有测试数据发布完成！请检查C++程序的输出。")

def main():
    """主函数"""
    try:
        execute_twist_tests()
    except rospy.ROSInterruptException:
        print("ROS中断异常")
    except Exception as e:
        print(f"程序执行出错: {e}")

if __name__ == '__main__':
    main()