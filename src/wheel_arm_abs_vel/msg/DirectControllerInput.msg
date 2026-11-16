# Explicit controller-only input contract; no body skeleton or inferred palm.
uint8 VERIFIED_CONTROLLER_GRIP=1
uint8 SYNTHETIC_FULL_URDF=2
uint8 HISTORICAL_UNVERIFIED_CONTROLLER_LABEL=3
# Acquisition stamp must be in the shared ROS clock domain, not a device clock.
uint8 CONTROL_INACTIVE=0
uint8 CONTROL_ARMED=1
uint8 CONTROL_ACTIVE=2
std_msgs/Header header
uint8 provenance
uint64 session_id
uint64 generation
uint8 control_state
bool tracking_valid
geometry_msgs/Pose[2] controller_pose
# Point velocity at controller origin, expressed in header.frame_id.
geometry_msgs/Twist[2] controller_twist
