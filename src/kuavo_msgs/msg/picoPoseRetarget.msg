# Header with timestamp
std_msgs/Header header

# Base (root) pose in world frame
# Note: geometry_msgs/Pose.orientation is quaternion in (x, y, z, w) format
geometry_msgs/Pose base_link_pose

# Base velocity: [linear_x, linear_y, linear_z, angular_x, angular_y, angular_z]
float64[6] base_velocity

# Joint positions for 27 DOF (excluding 2 head joints)
# Sequence:
#   Motors 0-5:   Left leg (hip_yaw, hip_roll, hip_pitch, knee, ankle_pitch, ankle_roll)
#   Motors 6-11:  Right leg (hip_yaw, hip_roll, hip_pitch, knee, ankle_pitch, ankle_roll)
#   Motor  12:    Waist (waist_yaw)
#   Motors 13-19: Left arm (shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_yaw, wrist_pitch, wrist_roll)
#   Motors 20-26: Right arm (shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_yaw, wrist_pitch, wrist_roll)
float64[27] joint_position

# Joint velocities for 27 DOF (excluding 2 head joints)
float64[27] joint_velocity

# End effector poses in world frame: [left_foot, right_foot, left_hand, right_hand]
geometry_msgs/Point[4] end_effector_poses