#!/usr/bin/env python3

import rospy
import sys
from kuavo_msgs.msg import twoArmHandPoseCmd, twoArmHandPose, armHandPose
from std_srvs.srv import SetBool
from kuavo_msgs.srv import changeTorsoCtrlMode, changeArmCtrlMode
from visualization_msgs.msg import Marker, MarkerArray

def enable_control_mode():
    """Enable the necessary control modes for arm control"""
    try:
        # Enable mobile manipulator MPC control
        rospy.wait_for_service('/mobile_manipulator_mpc_control', timeout=5.0)
        mpc_control = rospy.ServiceProxy('/mobile_manipulator_mpc_control', changeTorsoCtrlMode)
        mpc_control(control_mode=1)
        
        # Enable humanoid arm control mode
        rospy.wait_for_service('/humanoid_change_arm_ctrl_mode', timeout=5.0)
        arm_ctrl = rospy.ServiceProxy('/humanoid_change_arm_ctrl_mode', changeArmCtrlMode)
        arm_ctrl(control_mode=2)
        
        # Enable WBC arm trajectory control
        rospy.wait_for_service('/enable_mm_wbc_arm_trajectory_control', timeout=5.0)
        wbc_ctrl = rospy.ServiceProxy('/enable_mm_wbc_arm_trajectory_control', changeArmCtrlMode)
        wbc_ctrl(control_mode=1)
        
        rospy.loginfo("Successfully enabled all control modes")
        return True
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to enable control modes: {e}")
        return False
    except rospy.ROSException as e:
        rospy.logerr(f"Service not available: {e}")
        return False

def publish_visualization_markers(marker_pub, left_pos, right_pos, quat_left, quat_right):
    """Publish visualization markers for RViz"""
    marker_array = MarkerArray()
    
    # Create marker for left hand
    left_marker = Marker()
    left_marker.header.frame_id = "odom"  # Adjust frame_id as needed
    left_marker.header.stamp = rospy.Time.now()
    left_marker.ns = "hand_position_markers"
    left_marker.id = 0
    left_marker.type = Marker.SPHERE
    left_marker.action = Marker.ADD
    
    # Set position and orientation
    left_marker.pose.position.x = left_pos[0]
    left_marker.pose.position.y = left_pos[1]
    left_marker.pose.position.z = left_pos[2]
    left_marker.pose.orientation.x = quat_left[0]
    left_marker.pose.orientation.y = quat_left[1]
    left_marker.pose.orientation.z = quat_left[2]
    left_marker.pose.orientation.w = quat_left[3]
    
    # Set scale
    left_marker.scale.x = 0.05
    left_marker.scale.y = 0.05
    left_marker.scale.z = 0.05
    
    # Set color (blue for left hand)
    left_marker.color.r = 0.0
    left_marker.color.g = 0.0
    left_marker.color.b = 1.0
    left_marker.color.a = 0.8
    
    # Create marker for right hand
    right_marker = Marker()
    right_marker.header.frame_id = "world"  # Adjust frame_id as needed
    right_marker.header.stamp = rospy.Time.now()
    right_marker.ns = "hand_position_markers"
    right_marker.id = 1
    right_marker.type = Marker.SPHERE
    right_marker.action = Marker.ADD
    
    # Set position and orientation
    right_marker.pose.position.x = right_pos[0]
    right_marker.pose.position.y = right_pos[1]
    right_marker.pose.position.z = right_pos[2]
    right_marker.pose.orientation.x = quat_right[0]
    right_marker.pose.orientation.y = quat_right[1]
    right_marker.pose.orientation.z = quat_right[2]
    right_marker.pose.orientation.w = quat_right[3]
    
    # Set scale
    right_marker.scale.x = 0.05
    right_marker.scale.y = 0.05
    right_marker.scale.z = 0.05
    
    # Set color (red for right hand)
    right_marker.color.r = 1.0
    right_marker.color.g = 0.0
    right_marker.color.b = 0.0
    right_marker.color.a = 0.8
    
    # Add both markers to the array
    marker_array.markers.append(left_marker)
    marker_array.markers.append(right_marker)
    
    # Publish the marker array
    marker_pub.publish(marker_array)

def publish_hand_pose(pub, marker_pub, x, y, z, arm='left'):
    """Publish hand pose command"""
    msg = twoArmHandPoseCmd()
    two_arm_pose = twoArmHandPose()
    msg.frame = 0
    
    # Default orientation (neutral)
    quat = [0, -0.67566370964, 0, 0.73720997571945]  # x, y, z, w
    
    # Default positions
    elbow_pos_left = [0.08, 0.4, 0.18]
    elbow_pos_right = [0.08, -0.4, 0.18]
    joint_angles = [0, 0, 0, 0, 0, 0, 0]
    
    # Default positions for visualization
    left_pos = [0.3692052960395813, 0.43259960412979126, 0.8304591178894043]
    right_pos = [0.3692052960395813, -0.43259960412979126, 0.8304591178894043]
    
    # Set left hand pose
    left_pose = armHandPose()
    if arm == 'left':
        left_pose.pos_xyz = [x, y, z]
        left_pos = [x, y, z]
    else:
        left_pose.pos_xyz = left_pos
    left_pose.quat_xyzw = quat
    left_pose.elbow_pos_xyz = elbow_pos_left
    left_pose.joint_angles = joint_angles
    
    # Set right hand pose
    right_pose = armHandPose()
    if arm == 'right':
        right_pose.pos_xyz = [x, y, z]
        right_pos = [x, y, z]
    else:
        right_pose.pos_xyz = right_pos
    right_pose.quat_xyzw = quat
    right_pose.elbow_pos_xyz = elbow_pos_right
    right_pose.joint_angles = joint_angles
    
    # Assign poses to message
    two_arm_pose.left_pose = left_pose
    two_arm_pose.right_pose = right_pose
    msg.hand_poses = two_arm_pose
    
    # Publish message
    pub.publish(msg)
    rospy.loginfo(f"Published new position for {arm} arm: x={x:.3f}, y={y:.3f}, z={z:.3f}")
    
    # Publish visualization markers
    publish_visualization_markers(marker_pub, left_pos, right_pos, quat, quat)

def main():
    # Initialize ROS node
    rospy.init_node('hand_position_control', anonymous=True)
    
    # Enable control modes
    if not enable_control_mode():
        rospy.logerr("Failed to enable control modes. Exiting...")
        return
    
    # Create publishers
    pub = rospy.Publisher('/mm/two_arm_hand_pose_cmd', twoArmHandPoseCmd, queue_size=10)
    marker_pub = rospy.Publisher('/hand_position_markers', MarkerArray, queue_size=10)
    rospy.sleep(1.0)  # Wait for publishers to initialize
    
    print("\n=== Hand Position Control ===")
    print("Enter 'q' to quit")
    print("Format: <arm> <x> <y> <z>")
    print("Example: left 0.3 0.4 0.8")
    print("===========================\n")
    print("Hand positions visualization available in RViz (topic: /hand_position_markers)")
    
    while not rospy.is_shutdown():
        try:
            # Get user input
            user_input = input("Enter command: ").strip()
            
            if user_input.lower() == 'q':
                print("Exiting...")
                break
            
            # Parse input
            parts = user_input.split()
            if len(parts) != 4:
                print("Invalid input format. Please use: <arm> <x> <y> <z>")
                continue
            
            arm = parts[0].lower()
            if arm not in ['left', 'right']:
                print("Invalid arm selection. Use 'left' or 'right'")
                continue
            
            try:
                x = float(parts[1])
                y = float(parts[2])
                z = float(parts[3])
            except ValueError:
                print("Invalid position values. Please enter numbers.")
                continue
            
            # Publish the new position
            publish_hand_pose(pub, marker_pub, x, y, z, arm)
            
        except KeyboardInterrupt:
            print("\nExiting...")
            break
        except Exception as e:
            print(f"Error: {e}")

if __name__ == '__main__':
    main() 