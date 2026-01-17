#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
import time

reachTime = 0.0
def time_callback(msg):
   global reachTime
   reachTime = msg.data
   print("reach_time is ", reachTime)

def test_leg_joint_control():
    global reachTime

    # 初始化ROS节点
    rospy.init_node('test_leg_joint_publisher', anonymous=True)
    
    # 创建发布器
    pub = rospy.Publisher('/lb_leg_traj', JointState, queue_size=10)
    
    # 创建标志位订阅
    time_sub = rospy.Subscriber('/lb_leg_joint_reach_time', Float32, time_callback)

    # 等待发布器建立连接
    time.sleep(1)
    
    pubTime = 1.0  # 发布时间间隔
    joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
    
    # 测试数据1：指定关节角度
    joint_msg1 = JointState()
    joint_msg1.header.stamp = rospy.Time.now()
    joint_msg1.name = joint_names
    joint_msg1.position = [14.90, -32.01, 18.03, 0.0]
    
    print("发布测试数据1:")
    print(f"  关节角度: {joint_msg1.position}")
    
    reachTime = 0.0 # 重置时间
    pub.publish(joint_msg1)
    
    while reachTime == 0.0:
        rospy.sleep(0.1)

    # 短暂等待确保消息发送
    rospy.sleep(reachTime + 0.5)
    
    # 测试数据2：零位置
    joint_msg2 = JointState()
    joint_msg2.header.stamp = rospy.Time.now()
    joint_msg2.name = joint_names
    joint_msg2.position = [14.90, -32.01, 18.03, 90.0]
    
    print("\n发布测试数据2:")
    print(f"  关节角度: {joint_msg2.position}")
    
    reachTime = 0.0 # 重置时间
    pub.publish(joint_msg2)
    
    while reachTime == 0.0:
        rospy.sleep(0.1)

    # 短暂等待确保消息发送
    rospy.sleep(reachTime + 0.5)
    
    # 测试数据3：再次指定关节角度
    joint_msg3 = JointState()
    joint_msg3.header.stamp = rospy.Time.now()
    joint_msg3.name = joint_names
    joint_msg3.position = [0.0, 0.0, 0.0, 0.0]
    
    print("\n发布测试数据3:")
    print(f"  关节角度: {joint_msg3.position}")
    
    reachTime = 0.0 # 重置时间
    pub.publish(joint_msg3)
    
    while reachTime == 0.0:
        rospy.sleep(0.1)

    # 短暂等待确保消息发送
    rospy.sleep(reachTime + 0.5)
    
    print("\n测试数据发布完成！")

if __name__ == '__main__':
    try:
        test_leg_joint_control()
    except rospy.ROSInterruptException:
        pass