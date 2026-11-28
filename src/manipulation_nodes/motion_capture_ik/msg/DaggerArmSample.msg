# Arm order is left J1..J7, right J1..J7. SI units throughout.
# For the local IK sample, active means requested and solver-ready.
# For the execution mirror it identifies the sides using this IK sample.
std_msgs/Header header
string session
uint64 sequence
uint64[2] generation
bool[2] active
float64[14] position
float64[14] velocity
float64[14] acceleration

# Optional lower-body sample, in knee, leg, waist pitch, waist yaw order.
string waist_config_session
uint64 waist_generation
bool[4] waist_active
float64[4] waist_position
float64[4] waist_velocity
float64[4] waist_lower
float64[4] waist_upper
