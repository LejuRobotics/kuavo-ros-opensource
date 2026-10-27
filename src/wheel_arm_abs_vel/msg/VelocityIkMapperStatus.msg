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
