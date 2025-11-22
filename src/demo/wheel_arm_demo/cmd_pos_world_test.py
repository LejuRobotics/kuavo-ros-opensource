#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import time

current_flag = False

def flag_callback(msg):
   global current_flag
   current_flag = msg.data
   print("current_flag is ", current_flag)

def wait_until_false(timeout=None):
    """循环等待直到 current_flag = false"""
    global current_flag

    rospy.loginfo("开始等待标志位变为 False...")
    
    rate = rospy.Rate(10)  # 10Hz检查频率
    start_time = rospy.Time.now()
    
    while not rospy.is_shutdown():
        # 检查超时
        if timeout is not None:
            if (rospy.Time.now() - start_time).to_sec() > timeout:
                rospy.logwarn(f"等待超时 ({timeout}秒)")
                return False
        
        # 检查条件
        if not current_flag:
            rospy.loginfo("标志位已变为 False, 跳出循环")
            return True
            
        rate.sleep()
    
    return False

def wait_until_true(timeout=None):
    """循环等待直到 current_flag = true"""
    global current_flag

    rospy.loginfo("开始等待标志位变为 True...")
    
    rate = rospy.Rate(10)  # 10Hz检查频率
    start_time = rospy.Time.now()
    
    while not rospy.is_shutdown():

        # 检查超时
        if timeout is not None:
            if (rospy.Time.now() - start_time).to_sec() > timeout:
                rospy.logwarn(f"等待超时 ({timeout}秒)")
                return False
        
        # 检查条件
        if current_flag:
            rospy.loginfo("标志位已变为 True, 跳出循环")
            return True
            
        rate.sleep()
    
    return False

def test_twist_callback():
    # 初始化ROS节点
    rospy.init_node('test_twist_publisher', anonymous=True)
    
    # 创建发布器
    pub = rospy.Publisher('/cmd_pose_world', Twist, queue_size=10)

    # 创建标志位订阅
    flag_sub = rospy.Subscriber('/cmd_pose_world_flag', Bool, flag_callback)

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
    
    pub.publish(twist_msg1)
    
    wait_until_false()
    wait_until_true()

    time.sleep(pubTime)
    
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
    
    pub.publish(twist_msg2)
    
    wait_until_false()
    wait_until_true()

    time.sleep(pubTime)
    
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
    
    pub.publish(twist_msg3)
    
    wait_until_false()
    wait_until_true()

    time.sleep(pubTime)
    
    print("\n测试数据发布完成！请检查C++程序的输出。")

if __name__ == '__main__':
    try:
        test_twist_callback()
    except rospy.ROSInterruptException:
        pass