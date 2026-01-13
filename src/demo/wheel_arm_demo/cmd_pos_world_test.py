#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import time

reachTime = 0.0
def time_callback(msg):
   global reachTime
   reachTime = msg.data
   print("reach_time is ", reachTime)

def test_twist_callback():
    global reachTime

    # 初始化ROS节点
    rospy.init_node('test_twist_publisher', anonymous=True)
    
    # 创建发布器
    pub = rospy.Publisher('/cmd_pose_world', Twist, queue_size=10)

    # 创建标志位订阅
    time_sub = rospy.Subscriber('/lb_cmd_pose_reach_time', Float32, time_callback)

    # 等待发布器建立连接
    time.sleep(1)
    
    pubTime = 1.0  # 发布时间间隔
    # 测试数据1：基本数据
    twist_msg1 = Twist()
    twist_msg1.linear.x = 1.0
    twist_msg1.linear.y = 2.0
    twist_msg1.linear.z = 0.0
    twist_msg1.angular.x = 0.0
    twist_msg1.angular.y = 0.0
    twist_msg1.angular.z = 1.57
    
    print("发布测试数据1:")
    print(f"  位置: ({twist_msg1.linear.x}, {twist_msg1.linear.y})")
    print(f"  偏航角: {twist_msg1.angular.z}")
    
    reachTime = 0.0 # 重置时间
    pub.publish(twist_msg1)

    while reachTime == 0.0:
        rospy.sleep(0.1)

    # 短暂等待确保消息发送
    rospy.sleep(reachTime + 0.5)
    
    # 测试数据2：不同数据
    twist_msg2 = Twist()
    twist_msg2.linear.x = -0.5
    twist_msg2.linear.y = 1.5
    twist_msg2.linear.z = 0.0
    twist_msg2.angular.x = 0.0
    twist_msg2.angular.y = 0.0
    twist_msg2.angular.z = 3.14
    
    print("\n发布测试数据2:")
    print(f"  位置: ({twist_msg2.linear.x}, {twist_msg2.linear.y})")
    print(f"  偏航角: {twist_msg2.angular.z}")
    
    reachTime = 0.0 # 重置时间
    pub.publish(twist_msg2)

    while reachTime == 0.0:
        rospy.sleep(0.1)

    # 短暂等待确保消息发送
    rospy.sleep(reachTime + 0.5)
    
    # 测试数据3：零值
    twist_msg3 = Twist()
    twist_msg3.linear.x = 0.0
    twist_msg3.linear.y = 0.0
    twist_msg3.linear.z = 0.0
    twist_msg3.angular.x = 0.0
    twist_msg3.angular.y = 0.0
    twist_msg3.angular.z = 6.28
    
    print("\n发布测试数据3（零值）:")
    print(f"  位置: ({twist_msg3.linear.x}, {twist_msg3.linear.y})")
    print(f"  偏航角: {twist_msg3.angular.z}")
    
    reachTime = 0.0 # 重置时间
    pub.publish(twist_msg3)

    while reachTime == 0.0:
        rospy.sleep(0.1)

    # 短暂等待确保消息发送
    rospy.sleep(reachTime + 0.5)
    
    print("\n测试数据发布完成！请检查C++程序的输出。")

if __name__ == '__main__':
    try:
        test_twist_callback()
    except rospy.ROSInterruptException:
        pass