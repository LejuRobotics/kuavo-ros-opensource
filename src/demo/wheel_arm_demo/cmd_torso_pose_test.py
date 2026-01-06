#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

initialTorsoPose_ = [0.196123, 0.0005, 0.789919]

reachTime = 0.0
def time_callback(msg):
   global reachTime
   reachTime = msg.data
   print("reach_time is ", reachTime)

def publish_torso_pose():
    global reachTime, initialTorsoPose_

    rospy.init_node('torso_pose_publisher', anonymous=True)
    pub = rospy.Publisher('/cmd_lb_torso_pose', Twist, queue_size=10)
    time_sub = rospy.Subscriber('/lb_torso_pose_reach_time', Float32, time_callback)
    
    # 等待连接
    rospy.sleep(1)
    
    # 创建消息
    msg = Twist()
    msg.linear.x = 0.0 + initialTorsoPose_[0]
    msg.linear.y = 0.0
    msg.linear.z = 0.4 + initialTorsoPose_[2]
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = 0.0
    
    # 发布消息
    reachTime = 0.0 # 重置时间
    pub.publish(msg)
    print("Published: [0, 0, 0.4, 0, 0, 0]")

    while reachTime == 0.0:
        rospy.sleep(0.1)

    # 短暂等待确保消息发送
    rospy.sleep(reachTime + 0.5)

    # 创建消息
    msg = Twist()
    msg.linear.x = 0.2 + initialTorsoPose_[0]
    msg.linear.y = 0.0
    msg.linear.z = 0.4 + initialTorsoPose_[2]
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = 0.0
    
    # 发布消息
    reachTime = 0.0 # 重置时间
    pub.publish(msg)
    print("Published: [0.2, 0, 0.4, 0, 0, 0]")

    while reachTime == 0.0:
        rospy.sleep(0.1)

    # 短暂等待确保消息发送
    rospy.sleep(reachTime + 0.5)

    # 创建消息
    msg = Twist()
    msg.linear.x = 0.2 + initialTorsoPose_[0]
    msg.linear.y = 0.0
    msg.linear.z = 0.4 + initialTorsoPose_[2]
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = 1.047
    
    # 发布消息
    reachTime = 0.0 # 重置时间
    pub.publish(msg)
    print("Published: [0.2, 0, 0.4, 1.047, 0, 0]")

    while reachTime == 0.0:
        rospy.sleep(0.1)

    # 短暂等待确保消息发送
    rospy.sleep(reachTime + 0.5)

    # 创建消息
    msg = Twist()
    msg.linear.x = 0.2 + initialTorsoPose_[0]
    msg.linear.y = 0.0
    msg.linear.z = 0.4 + initialTorsoPose_[2]
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = -1.047
    
    # 发布消息
    reachTime = 0.0 # 重置时间
    pub.publish(msg)
    print("Published: [0.2, 0, 0.4, -1.047, 0, 0]")

    while reachTime == 0.0:
        rospy.sleep(0.1)

    # 短暂等待确保消息发送
    rospy.sleep(reachTime + 0.5)

    # 创建消息
    msg = Twist()
    msg.linear.x = 0.2 + initialTorsoPose_[0]
    msg.linear.y = 0.0
    msg.linear.z = 0.4 + initialTorsoPose_[2]
    msg.angular.x = 0.0
    msg.angular.y = -0.524
    msg.angular.z = 0.0
    
    # 发布消息
    pub.publish(msg)
    print("Published: [0.2, 0, 0.4, 0, -0.524, 0]")

    while reachTime == 0.0:
        rospy.sleep(0.1)

    # 短暂等待确保消息发送
    rospy.sleep(reachTime + 0.5)

    # 创建消息
    msg = Twist()
    msg.linear.x = 0.2 + initialTorsoPose_[0]
    msg.linear.y = 0.0
    msg.linear.z = 0.4 + initialTorsoPose_[2]
    msg.angular.x = 0.0
    msg.angular.y = 0.524
    msg.angular.z = 0.0
    
    # 发布消息
    pub.publish(msg)
    print("Published: [0.2, 0, 0.4, 0, 0.524, 0]")

    while reachTime == 0.0:
        rospy.sleep(0.1)

    # 短暂等待确保消息发送
    rospy.sleep(reachTime + 0.5)

    # 创建消息
    msg = Twist()
    msg.linear.x = 0.0 + initialTorsoPose_[0]
    msg.linear.y = 0.0
    msg.linear.z = 0.0 + initialTorsoPose_[2]
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = 0.0
    
    # 发布消息
    pub.publish(msg)
    print("Published: [0.0, 0, 0.0, 0.0, 0, 0]")
    

if __name__ == '__main__':
    try:
        publish_torso_pose()
    except rospy.ROSInterruptException:
        pass