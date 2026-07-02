# High-precision velocity combination control goal message
# For pure motion control with high precision walking

# Motion command type
uint8 CMD_STOP = 0
uint8 CMD_FORWARD = 1
uint8 CMD_BACKWARD = 2
uint8 CMD_ROTATE_LEFT = 3
uint8 CMD_ROTATE_RIGHT = 4
uint8 CMD_CUSTOM = 5
uint8 command_type

# Custom velocity command (only for CMD_CUSTOM)
geometry_msgs/Twist velocity

# Movement distance (m) for linear motion
# Rotation angle (rad) for rotation motion
float32 target_distance

# Maximum velocity during motion
float32 max_velocity

# Maximum acceleration
float32 max_acceleration

# Position/angle tolerance
float32 tolerance

# Whether to maintain final position after reaching target
bool hold_position

# Timeout (seconds, 0 for no timeout)
float32 timeout
