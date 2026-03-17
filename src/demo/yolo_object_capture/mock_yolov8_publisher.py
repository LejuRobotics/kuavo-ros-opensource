#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Mock YOLOv8 detections + TF broadcaster
Topic : /robot_yolov8_info
TF    : base_link -> yolov8_<name>_<id>
"""

import rospy
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from geometry_msgs.msg import (
    PoseWithCovariance,
    Point,
    Quaternion,
    Vector3,
    TransformStamped
)
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
import copy

# =======================
# COCO IDs
# =======================
BOTTLE_ID = 39
CUP_ID    = 41
ORANGE_ID = 49

# =======================
# 物体定义（推荐集中管理）
# =======================
OBJECTS = {
    # BOTTLE_ID: {
    #     "name": "bottle",
    #     "position": Point(0.45, -0.20, 0.15),
    #     "color": (1.0, 0.0, 0.0),
    # },
    # CUP_ID: {
    #     "name": "cup",
    #     "position": Point(0.35,  0.10, 0.12),
    #     "color": (0.0, 1.0, 0.0),
    # },
    ORANGE_ID: {
        "name": "orange",
        "position": Point(0.40, -0.15, -0.10),
        "color": (1.0, 0.5, 0.0),
    },
}

orientation = Quaternion(*quaternion_from_euler(0, 0, 0))

# =======================

def create_detection(obj_id, obj):
    detection = Detection2D()
    detection.header.stamp = rospy.Time.now()
    detection.header.frame_id = "base_link"

    result = ObjectHypothesisWithPose()
    result.id = obj_id
    result.score = 0.9

    pose = PoseWithCovariance()
    pose.pose.position = obj["position"]
    pose.pose.orientation = orientation
    # covariance 可选，保持全 0 即可

    result.pose = pose
    detection.results.append(result)

    return detection


def create_detection_array():
    msg = Detection2DArray()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "base_link"

    for obj_id, obj in OBJECTS.items():
        msg.detections.append(create_detection(obj_id, obj))

    return msg

def publish_marker(marker_pub, obj_id, obj):
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "yolov8_mock"
    marker.id = obj_id
    marker.type = Marker.CYLINDER
    marker.action = Marker.ADD

    marker.pose.position = copy.deepcopy(obj["position"])
    marker.pose.position.z -= 0.05
    marker.pose.orientation = orientation

    marker.scale = Vector3(0.05, 0.05, 0.10)

    marker.color.a = 1.0
    marker.color.r, marker.color.g, marker.color.b = obj["color"]

    marker_pub.publish(marker)

def broadcast_tf(br, obj_id, obj):
    """
    base_link -> yolov8_<name>_<id>
    """
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "base_link"
    t.child_frame_id = f"yolov8_{obj['name']}_{obj_id}"

    t.transform.translation.x = obj["position"].x
    t.transform.translation.y = obj["position"].y
    t.transform.translation.z = obj["position"].z
    t.transform.rotation = orientation

    br.sendTransform(t)

def mock_yolov8_publisher():
    rospy.init_node("mock_yolov8_publisher", anonymous=True)

    pub = rospy.Publisher(
        "/robot_yolov8_info",
        Detection2DArray,
        queue_size=10
    )

    marker_pub = rospy.Publisher(
        "/visualization_marker",
        Marker,
        queue_size=10
    )

    tf_broadcaster = TransformBroadcaster()

    rate = rospy.Rate(10)
    rospy.loginfo("Mock YOLOv8 publisher with TF started.")

    while not rospy.is_shutdown():
        pub.publish(create_detection_array())

        for obj_id, obj in OBJECTS.items():
            publish_marker(marker_pub, obj_id, obj)
            broadcast_tf(tf_broadcaster, obj_id, obj)

        rate.sleep()

if __name__ == "__main__":
    try:
        mock_yolov8_publisher()
    except rospy.ROSInterruptException:
        pass
