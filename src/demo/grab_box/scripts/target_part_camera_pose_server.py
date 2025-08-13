#!/usr/bin/env python3
# coding=utf-8

import rospy
import tf2_ros
import tf2_geometry_msgs # For PoseStamped
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from visualization_msgs.msg import Marker
from kuavo_msgs.srv import GetTargetPartPoseInCamera, GetTargetPartPoseInCameraResponse

class TargetPartCameraPoseServer:
    def __init__(self):
        rospy.init_node('target_part_camera_pose_server')

        # --- Configuration ---
        self.world_frame_id = "odom"
        self.camera_frame_id = "camera_base" # IMPORTANT: Change to your actual camera frame
        self.served_part_id = 0 # The ID of the part whose pose will be served
        self.marker_publish_rate = 1.0 # Hz
        self.tf_lookup_timeout = rospy.Duration(1.0)

        # Define the pose of the target part(s) in the world frame ("odom")
        # For simplicity, we'll serve the pose of the part with ID `self.served_part_id`
        self.target_parts_world = [
            {
                "id": 0,
                "pose": Pose(
                    position=Point(1.0, 0.5, 0.9), # As per your box_pose_pub_test.py for ID 0
                    orientation=Quaternion(0, -0.67566370964, 0, 0.73720997571945)
                ),
                "size": [0.1, 0.1, 0.1] # Default size for visualization
            },
            # You can add more parts here if needed for visualization
            {
                "id": 1,
                "pose": Pose(
                    position=Point(0.1, 0.3, 0.9),
                    orientation=Quaternion(0, -0.67566370964, 0, 0.73720997571945)
                ),
                 "size": [0.08, 0.08, 0.08]
            },
        ]
        
        self.served_part_world_pose = None
        for part in self.target_parts_world:
            if part["id"] == self.served_part_id:
                self.served_part_world_pose = part["pose"]
                break
        
        if self.served_part_world_pose is None:
            rospy.logerr(f"Part with ID {self.served_part_id} not found in target_parts_world. Shutting down.")
            rospy.signal_shutdown(f"Served part ID {self.served_part_id} not defined.")
            return

        # TF Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # ROS Service Server
        self.service = rospy.Service(
            'get_target_part_pose_in_camera', 
            GetTargetPartPoseInCamera, 
            self.handle_get_part_in_camera
        )
        rospy.loginfo("Service 'get_target_part_pose_in_camera' is ready.")

        # ROS Publisher for RVIZ Markers
        self.marker_pub = rospy.Publisher('/target_parts_visualization', Marker, queue_size=10)
        rospy.loginfo("Publishing markers to /target_parts_visualization")
        
        # Timer to publish markers
        self.marker_timer = rospy.Timer(rospy.Duration(1.0/self.marker_publish_rate), self.publish_markers_callback)

    def handle_get_part_in_camera(self, req):
        rospy.loginfo(f"Received request for part pose in '{self.camera_frame_id}' frame.")
        response = GetTargetPartPoseInCameraResponse()

        if self.served_part_world_pose is None:
            response.success = False
            response.message = f"Configuration error: Served part ID {self.served_part_id} not found."
            rospy.logerr(response.message)
            return response

        # Create a PoseStamped message for the part in the world frame
        pose_stamped_world = PoseStamped()
        pose_stamped_world.header.frame_id = self.world_frame_id
        pose_stamped_world.header.stamp = rospy.Time(0) # Use latest available transform
        pose_stamped_world.pose = self.served_part_world_pose

        try:
            # Transform the pose to the camera frame
            transformed_pose_stamped = self.tf_buffer.transform(
                pose_stamped_world,
                self.camera_frame_id,
                timeout=self.tf_lookup_timeout
            )
            
            response.pose_in_camera = transformed_pose_stamped.pose
            response.success = True
            response.message = f"Successfully transformed pose of part ID {self.served_part_id} to {self.camera_frame_id}"
            rospy.loginfo(response.message)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            response.success = False
            response.message = f"TF transform error from '{self.world_frame_id}' to '{self.camera_frame_id}': {e}"
            rospy.logwarn(response.message)
        
        return response

    def publish_markers_callback(self, event=None):
        for part_info in self.target_parts_world:
            marker = Marker()
            marker.header.frame_id = self.world_frame_id
            marker.header.stamp = rospy.Time.now()
            marker.ns = "target_parts"
            marker.id = part_info["id"]
            marker.type = Marker.CUBE 
            marker.action = Marker.ADD

            marker.pose = part_info["pose"]
            
            marker.scale.x = part_info["size"][0]
            marker.scale.y = part_info["size"][1]
            marker.scale.z = part_info["size"][2]

            if part_info["id"] == self.served_part_id:
                marker.color.r = 0.0  # Green for the served part
                marker.color.g = 1.0
                marker.color.b = 0.0
            else:
                marker.color.r = 1.0  # Red for other parts
                marker.color.g = 0.0
                marker.color.b = 0.0
            marker.color.a = 0.8 # Alpha

            self.marker_pub.publish(marker)

    def run(self):
        rospy.loginfo("Target Part Camera Pose Server is running...")
        rospy.spin()

if __name__ == '__main__':
    try:
        server = TargetPartCameraPoseServer()
        if server.served_part_world_pose is not None: # Check if init was successful
             server.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Server shutting down.")
    except Exception as e:
        rospy.logerr(f"Unhandled exception: {e}") 