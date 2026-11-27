# Latched DAgger waist configuration sent from LeTools to KRC.

std_msgs/Header header
uint32 schema_version
string session
string which_arm

# [knee, leg, waist_pitch, waist_yaw]. Positions are radians.
float64[4] lower
float64[4] upper
bool[4] lock_mask
float64[4] lock_position

# radians/second and seconds.
float64 max_speed
float64 action_timeout
float64 signal_timeout
