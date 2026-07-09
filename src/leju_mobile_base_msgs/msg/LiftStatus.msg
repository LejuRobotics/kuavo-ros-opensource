# Current lift system status information
# Publish on /lift_status topic to describe lift height, motion state and errors

std_msgs/Header header

# Current lift platform height in meters relative to zero reference
float32 current_height

# Desired target height in meters, if a motion command is active
float32 target_height

# Minimum and maximum allowed heights in meters (hardware limits)
float32 min_height
float32 max_height

# Current lift velocity in meters per second (positive = upward)
float32 velocity

# Whether the lift hardware is powered and ready for motion
bool is_enabled

# Whether height calibration/reference alignment is complete
bool is_calibrated

# Motion state enumeration
uint8 STATE_UNKNOWN = 0
uint8 STATE_IDLE = 1
uint8 STATE_MOVING_UP = 2
uint8 STATE_MOVING_DOWN = 3
uint8 STATE_HOLDING = 4
uint8 STATE_ERROR = 5
uint8 state

# Error code (0 when no error is present)
uint32 error_code

# Human-readable error description (empty when no error)
string error_msg
