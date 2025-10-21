#!/usr/bin/env python3
# coding=utf-8

import rospy
import numpy as np
from apriltag_ros.msg import AprilTagDetectionArray, AprilTagDetection
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_matrix, quaternion_multiply, quaternion_inverse
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point, Quaternion


class AprilTagPublisher:
    def __init__(self):
        rospy.init_node('apriltag_publisher', anonymous=True)
        self.pub = rospy.Publisher('/robot_tag_info', AprilTagDetectionArray, queue_size=10)
        self.marker_pub = rospy.Publisher('/visualization_box_marker', Marker, queue_size=10)
        self.rate = rospy.Rate(10)  # Publish frequency: 10Hz

        self.robot_pose = None
        self.tag_poses_world = [
            {
                "id": 1,
                "pose": Pose(
                    position=Point(1.0, 0.3, 1.3),
                    # orientation=Quaternion(0.0, -0.707, 0.0, 0.707)
                    # orientation=Quaternion(0.488, -0.4545, -0.5209, 0.5324)
                    orientation=Quaternion(0.6666145423605649, 0.023693991691330407, 0.008133758341800246, 0.744981535775473)
                ),
                "size": 0.06,
            },
            {
                "id": 2,
                "pose": Pose(
                    position=Point(0.5, -1.5, 1.55),
                    orientation=Quaternion(-0.079, 0.57899, 0.809566, -0.055816)
                ),
                "size": 0.06,
            },
                        {
                "id": 0,
                "pose": Pose(
                    position=Point(1.0, 0.5, 1.7),
                    orientation=Quaternion(0.488, -0.4545, -0.5209, 0.5324)
                    # w=0.5,x=−0.5,y=−0.5,z=0.5
                ),
                "size": 0.06,
            },
            # {
            #     "id": 10,
            #     "pose": Pose(
            #         position=Point(-1.0, 0.0, 0.9),
            #         # orientation=Quaternion(0.0, -0.707, 0.0, 0.707)
            #         orientation=Quaternion( 0.69782711, 0.005843221508388842,  -0.04662733200630576, 0.7147230696446081)
                    
            #     ),
            #     "size": 0.06,
            # },
        ]

        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

    def odom_callback(self, msg):
        self.robot_pose = msg.pose.pose

    def publish_marker(self, tag_pose_world, tag_id):
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = rospy.Time.now()
        marker.ns = 'apriltag'
        marker.id = tag_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose = tag_pose_world

        marker.scale.x = 0.3
        marker.scale.y = 0.5
        marker.scale.z = 0.8

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.marker_pub.publish(marker)

    def get_transformed_tag_pose(self, tag_pose_world, robot_pose):
        robot_position = np.array([robot_pose.position.x, robot_pose.position.y, robot_pose.position.z])
        robot_orientation = np.array([
            robot_pose.orientation.x, robot_pose.orientation.y,
            robot_pose.orientation.z, robot_pose.orientation.w
        ])
        tag_position = np.array([tag_pose_world.position.x, tag_pose_world.position.y, tag_pose_world.position.z])
        tag_orientation = np.array([
            tag_pose_world.orientation.x, tag_pose_world.orientation.y,
            tag_pose_world.orientation.z, tag_pose_world.orientation.w
        ])

        relative_position = tag_position - robot_position
        rotation_matrix = quaternion_matrix(quaternion_inverse(robot_orientation))[:3, :3]
        transformed_position = rotation_matrix.dot(relative_position)

        tag_pose_relative = Pose()
        tag_pose_relative.position.x = transformed_position[0]
        tag_pose_relative.position.y = transformed_position[1]
        tag_pose_relative.position.z = transformed_position[2]

        relative_orientation = quaternion_multiply(quaternion_inverse(robot_orientation), tag_orientation)
        tag_pose_relative.orientation.x = relative_orientation[0]
        tag_pose_relative.orientation.y = relative_orientation[1]
        tag_pose_relative.orientation.z = relative_orientation[2]
        tag_pose_relative.orientation.w = relative_orientation[3]

        return tag_pose_relative

    def run(self):
        while not rospy.is_shutdown():
            if self.robot_pose is None:
                rospy.logwarn("Waiting for robot pose data...")
                rospy.sleep(0.1)
                continue

            tag_detection_array = AprilTagDetectionArray()

            for tag_info in self.tag_poses_world:
                tag_id = tag_info["id"]
                tag_pose_world = tag_info["pose"]
                tag_size = tag_info["size"]

                tag_pose_relative = self.get_transformed_tag_pose(tag_pose_world, self.robot_pose)

                detection = AprilTagDetection()
                detection.id = [tag_id]
                detection.size = [tag_size]

                pose = PoseWithCovarianceStamped()
                pose.header.stamp = rospy.Time.now()
                pose.header.frame_id = 'odom'
                pose.pose.pose = tag_pose_relative
                pose.pose.covariance = [0] * 36

                detection.pose = pose
                tag_detection_array.detections.append(detection)

                self.publish_marker(tag_pose_world, tag_id)

            self.pub.publish(tag_detection_array)
            self.rate.sleep()


if __name__ == '__main__':
    try:
        publisher = AprilTagPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        pass
