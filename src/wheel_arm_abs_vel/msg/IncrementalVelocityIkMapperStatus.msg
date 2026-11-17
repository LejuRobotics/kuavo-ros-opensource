# Minimal same-cycle feedback consumed by the Quest/Pico mapper.
# The complete VelocityIkShadow remains on diagnostics/reference for recording.
# These fields preserve the existing preparation/acknowledgement predicates.
std_msgs/Header header
uint64 session_id
uint64 generation
uint8 control_state
bool valid
bool takeover_active
bool preparation_complete

# Coordinate/session binding is available while ARMED, before execution ACTIVE.
bool session_bound

# Full-model measured TCP in the same axes/origin as the mapper target.
geometry_msgs/Pose[2] measured_mapper_tcp

# Measured full-model geometry for beta incremental targets, from the same q.
bool incremental_geometry_valid
geometry_msgs/Pose[2] base_from_mapper
geometry_msgs/Pose base_from_chest
geometry_msgs/Point[2] mapper_shoulder
geometry_msgs/Vector3[2] tcp_to_link6
geometry_msgs/Quaternion[2] mapper_link4
float64[2] arm_reach
bool[2] arm_at_motion_boundary
float64[2] upper_arm_length
geometry_msgs/Point[2] mapper_elbow

# Command-reference FK (never measured FK) for a clutch hold handoff.
# Same snapshot and coordinate system as the geometry above.
bool command_geometry_valid
geometry_msgs/Pose[2] command_mapper_tcp
geometry_msgs/Point[2] command_mapper_elbow
uint64[2] command_clutch_seq
uint64 command_operation_seq
bool[2] command_held
geometry_msgs/Pose command_base_chest
