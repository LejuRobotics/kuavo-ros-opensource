# Navigation target point goal message

Header header

# Goal ID
string goal_id

# Target pose in map frame
geometry_msgs/PoseStamped target_pose

# Navigation parameters
float32 max_linear_vel     # Maximum linear velocity (m/s)
float32 max_angular_vel    # Maximum angular velocity (rad/s)
float32 position_tolerance # Position tolerance (m)
float32 orientation_tolerance # Orientation tolerance (rad)

# Navigation mode
uint8 MODE_NORMAL = 0
uint8 MODE_CAREFUL = 1      # More conservative navigation
uint8 MODE_FAST = 2         # Faster, less conservative
uint8 navigation_mode

# Whether to ignore final orientation
bool ignore_orientation

# Timeout (seconds, 0 for no timeout)
float32 timeout
