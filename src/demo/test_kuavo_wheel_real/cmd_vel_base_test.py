#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import time
import lb_ctrl_api as ct
    
# 全局变量
reachTime = 3.0
PUBLISH_RATE = 100  # Hz

def execute_twist_tests():
    """执行Twist测试的主函数"""
    global reachTime

    # 初始化ROS节点
    rospy.init_node('test_twist_publisher', anonymous=True)
    
    # 创建发布器
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # 等待发布器建立连接
    time.sleep(1)
    
    # 测试数据列表
    test_cases = [
        # (名称, linear_x, linear_y, linear_z, angular_x, angular_y, angular_z)
        ("原地旋转", 0.0, 0.0, 0.0, 0.0, 0.0, 0.1),
        ("本体系x前进", 0.1, 0.0, 0.0, 0.0, 0.0, 0.0),
        ("本体系y前进", 0.0, 0.1, 0.0, 0.0, 0.0, 0.0),
    ]
    
    print("开始发布测试数据...")
    
    for i, (name, lx, ly, lz, ax, ay, az) in enumerate(test_cases, 1):
        print(f"\n=== 第{i}组测试: {name} ===")
        print(f"  位置: ({lx}, {ly})")
        print(f"  偏航角: {az}")
        
        # 创建Twist消息
        twist_msg = Twist()
        twist_msg.linear.x = lx
        twist_msg.linear.y = ly
        twist_msg.linear.z = lz
        twist_msg.angular.x = ax
        twist_msg.angular.y = ay
        twist_msg.angular.z = az
        
        # 计算需要发布的次数
        publish_count = int(reachTime * PUBLISH_RATE)
        
        # 创建rate对象
        rate = rospy.Rate(PUBLISH_RATE)
        
        # 持续发布reachTime秒
        for j in range(publish_count):
            pub.publish(twist_msg)
            rate.sleep()
        
        print(f"  {name} 完成! 持续发布了 {publish_count} 次命令")
    
    # 发布停止命令
    stop_msg = Twist()  # 所有字段默认为0
    for i in range(10):  # 多发几次确保停止
        pub.publish(stop_msg)
        rospy.sleep(0.1)
    
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