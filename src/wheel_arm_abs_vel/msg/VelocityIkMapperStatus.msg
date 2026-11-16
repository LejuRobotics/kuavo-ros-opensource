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

# Distinguish execution ownership from a completed first follow transition.
bool unlock_transition_complete
bool input_confirmation_required
