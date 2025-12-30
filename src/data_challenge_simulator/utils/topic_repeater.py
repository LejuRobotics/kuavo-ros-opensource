#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, TwistStamped

class TopicRelay:
    def __init__(self):
        rospy.init_node('topic_relay_node')
        
        # 获取参数
        self.publish_rate = rospy.get_param('~publish_rate', 50)  # 默认50Hz
        
        # cmd_pose_world (Twist) 相关
        self.last_cmd_twist = None
        self.cmd_twist_sub = rospy.Subscriber(
            '/cmd_pose_world', 
            Twist,
            self.cmd_twist_callback
        )
        self.cmd_twist_pub = rospy.Publisher(
            '/cmd_pose_world_synced',
            TwistStamped,  # 带时间戳的Twist
            queue_size=10
        )
        
        # kuavo_arm_traj (JointState) 相关
        self.last_joint_state = None
        self.joint_state_sub = rospy.Subscriber(
            '/kuavo_arm_traj',
            JointState,
            self.joint_state_callback
        )
        self.joint_state_pub = rospy.Publisher(
            '/kuavo_arm_traj_synced',
            JointState,
            queue_size=10
        )
        
        # 定时器以固定频率发布
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate), 
            self.timer_callback
        )
        
        rospy.loginfo("Topic relay node started at {} Hz".format(self.publish_rate))
        rospy.loginfo("Subscribing to: /cmd_pose_world (Twist) and /kuavo_arm_traj (JointState)")
        rospy.loginfo("Publishing to: /cmd_pose_world_synced (TwistStamped) and /kuavo_arm_traj_synced (JointState)")
    
    def cmd_twist_callback(self, msg):
        """接收原始 cmd_pose_world (Twist) 消息"""
        self.last_cmd_twist = msg
        rospy.logdebug("Received cmd_pose_world: linear.x={:.3f}, angular.z={:.3f}".format(
            msg.linear.x, msg.angular.z))
    
    def joint_state_callback(self, msg):
        """接收原始 kuavo_arm_traj (JointState) 消息"""
        self.last_joint_state = msg
        rospy.logdebug("Received kuavo_arm_traj with {} joints".format(len(msg.name)))
    
    def timer_callback(self, event):
        """定时发布消息,保持数据连续性"""
        current_time = rospy.Time.now()
        
        # 发布 cmd_pose_world_synced (添加时间戳)
        if self.last_cmd_twist is not None:
            stamped_msg = TwistStamped()
            stamped_msg.header.stamp = current_time
            stamped_msg.header.frame_id = "world"
            stamped_msg.twist = self.last_cmd_twist
            self.cmd_twist_pub.publish(stamped_msg)
        
        # 发布 kuavo_arm_traj_synced (更新时间戳)
        if self.last_joint_state is not None:
            # 创建副本并更新时间戳
            joint_msg = JointState()
            joint_msg.header = self.last_joint_state.header
            joint_msg.header.stamp = current_time
            joint_msg.name = self.last_joint_state.name
            joint_msg.position = self.last_joint_state.position
            joint_msg.velocity = self.last_joint_state.velocity
            joint_msg.effort = self.last_joint_state.effort
            self.joint_state_pub.publish(joint_msg)
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        relay = TopicRelay()
        relay.run()
    except rospy.ROSInterruptException:
        pass