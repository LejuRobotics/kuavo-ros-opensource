#! /usr/bin/env python
import time
import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf2_geometry_msgs
from kuavo_msgs.msg import robotHeadMotionData  # 头部电机控制
from joint_state_publisher import JointStatePublisher
import copy
from geometry_msgs.msg import TransformStamped, PoseStamped

class ARControlNode(object):
    def __init__(self):
        rospy.init_node('ar_control_node')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self._head_traj_pub = rospy.Publisher("/robot_head_motion_data", robotHeadMotionData, queue_size=10)
        self.publisher = rospy.Publisher('/robot_tag_info', AprilTagDetectionArray, queue_size=10)
        self.publisher_odom = rospy.Publisher('/robot_tag_info_odom', AprilTagDetectionArray, queue_size=10)
        self.subscription = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_callback)
        
        # 检查机器人类型：robot_type=1 表示轮臂机器人
        self.robot_type = rospy.get_param('/robot_type', 0)
        self.is_wheel_robot = (self.robot_type == 1)
        
        # 根据机器人类型设置对应的坐标系名称
        if self.is_wheel_robot:
            # 轮臂机器人：world 等价于 odom，waist_yaw_link 等价于 base_link
            self.odom_frame = 'odom'
            self.base_frame = 'waist_yaw_link'
            rospy.loginfo(f"ARControlNode: 轮臂机器人 - odom_frame='{self.odom_frame}', base_frame='{self.base_frame}'")
        else:
            # 人形机器人：标准命名
            self.odom_frame = 'odom'
            self.base_frame = 'base_link'
            rospy.loginfo(f"ARControlNode: 人形机器人 - odom_frame='{self.odom_frame}', base_frame='{self.base_frame}'")
        
        # 等待 TF 树建立，然后打印所有 frame
        # rospy.sleep(0.5)  # 等待 TF 树初始化
        # self.print_tf_frames()
    
    def print_tf_frames(self):
        """打印 TF 树中所有可用的 frame"""
        try:
            # 获取所有 frame 的名称
            all_frames = self.tf_buffer.all_frames_as_string()
            rospy.loginfo("=" * 60)
            rospy.loginfo("TF 树中的所有 frame:")
            rospy.loginfo("=" * 60)
            rospy.loginfo("\n" + all_frames)
            rospy.loginfo("=" * 60)
        except Exception as e:
            rospy.logwarn(f"无法获取 TF 树信息: {e}")
    
    def pub_kuavo_head_traj(self, from_joint_state, target_joint_state):
        # 头部包含两个电机，数据格式为长度为 2 的数组
        # 第一个值代表水平方向，范围 -30 ~ 30
        # 第二个值代表竖直方向，范围 -25 ~ 25
        # 按照每次 3 度进行插值处理
        head_traj_msg = robotHeadMotionData()

        step = 3 if target_joint_state[0] > from_joint_state[0] else -3
        for i in range(from_joint_state[0], target_joint_state[0], step):
            head_traj_msg.joint_data = [i, from_joint_state[1]]
            self._head_traj_pub.publish(head_traj_msg)
            time.sleep(0.1)
        step = 3 if target_joint_state[1] > from_joint_state[1] else -3
        for i in range(from_joint_state[1], target_joint_state[1], step):
            head_traj_msg.joint_data = [target_joint_state[0], i]
            self._head_traj_pub.publish(head_traj_msg)
            time.sleep(0.1)

    def tag_callback(self, msg):
        # 发布到 base_link/waist_yaw_link 坐标系的消息
        new_msg = AprilTagDetectionArray()
        new_msg.header.stamp = rospy.Time.now()
        new_msg.header.frame_id = self.base_frame  # 使用动态frame名称

        # 发布到 odom/world 坐标系的消息
        new_msg_odom = AprilTagDetectionArray()
        new_msg_odom.header.stamp = rospy.Time.now()
        new_msg_odom.header.frame_id = self.odom_frame  # 使用动态frame名称
    
        for detection in msg.detections:
            # Make sure the detection's pose has a valid frame_id
            if detection.pose.header.frame_id == '':
                detection.pose.header.frame_id = 'camera_color_optical_frame'
    
            # 1. 转换到 base_frame (base_link 或 waist_yaw_link)
            try:
                # 查找从 camera 到 base_frame 的变换
                transform_to_base = self.tf_buffer.lookup_transform(
                    self.base_frame,                  # 目标坐标系（动态）
                    detection.pose.header.frame_id,   # 源坐标系（通常是 camera）
                    rospy.Time(0),
                    rospy.Duration(1.0)
                )
            
                # 将 tag 位姿转换到 base_frame
                transformed_pose_base = tf2_geometry_msgs.do_transform_pose(detection.pose.pose, transform_to_base)
        
                # 更新 detection 为 base_frame 下的数据
                detection.pose.pose.pose.position = transformed_pose_base.pose.position
                detection.pose.pose.pose.orientation = transformed_pose_base.pose.orientation
                detection.pose.header.frame_id = self.base_frame
    
                # 添加到 base_frame 消息
                new_msg.detections.append(detection)
    
                # Broadcast transform to tf
                self.broadcast_transform(transformed_pose_base, detection.id)

                # 2. 转换到 odom_frame (odom 或 world)
                try:
                    # 查找从 base_frame 到 odom_frame 的变换
                    transform_to_odom = self.tf_buffer.lookup_transform(
                        self.odom_frame,    # 目标坐标系（odom 或 world）
                        self.base_frame,    # 源坐标系（base_link 或 waist_yaw_link）
                        rospy.Time(0),
                        rospy.Duration(1.0)
                    )
                    
                    # 创建 PoseStamped 消息用于转换
                    pose_stamped = PoseStamped()
                    pose_stamped.header.frame_id = self.base_frame
                    pose_stamped.header.stamp = rospy.Time.now()
                    pose_stamped.pose = transformed_pose_base.pose
                    
                    # 执行转换：base_frame -> odom_frame
                    transformed_pose_odom = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform_to_odom)
                    
                    # 创建新的检测消息
                    detection_odom = copy.deepcopy(detection)
                    detection_odom.pose.pose.pose = transformed_pose_odom.pose
                    detection_odom.pose.header.frame_id = self.odom_frame
                    
                    # 添加到 odom 消息中
                    new_msg_odom.detections.append(detection_odom)
                    
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    rospy.logwarn(f"TF Error: {e} - Cannot transform from {self.base_frame} to {self.odom_frame}")
                    continue
            
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logwarn(f"TF Error: {e} - Cannot transform from {detection.pose.header.frame_id} to {self.base_frame}")
                continue
    
        # 发布消息
        self.publisher.publish(new_msg)
        self.publisher_odom.publish(new_msg_odom)

    def broadcast_transform(self, pose, tag_id):
        """广播 tag 在 base_frame 下的 TF 变换"""
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = rospy.Time.now()
        transform_stamped.header.frame_id = self.base_frame  # 使用动态 frame 名称
        transform_stamped.child_frame_id = 'tag_origin_' + str(tag_id)
        transform_stamped.transform.translation.x = pose.pose.position.x
        transform_stamped.transform.translation.y = pose.pose.position.y
        transform_stamped.transform.translation.z = pose.pose.position.z
        transform_stamped.transform.rotation = pose.pose.orientation

        self.tf_broadcaster.sendTransform(transform_stamped)

def main():
    try:
        ar_control_node = ARControlNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
