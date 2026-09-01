# Base velocity command effectiveness status
# Reports whether base_cmd_vel commands are currently being executed by the chassis.

Header header

# Whether base_cmd_vel commands are currently effective
# true  - velocity commands are executed normally
# false - velocity commands are ignored, chassis can be pushed manually
bool cmd_vel_effective

# Reason why base_cmd_vel is ineffective
# Only meaningful when cmd_vel_effective is false (use REASON_NONE when effective)
uint8 REASON_NONE = 0
uint8 REASON_MANUAL_MODE = 1          # Chassis in manual mode
uint8 REASON_DISABLED = 2             # Motors de-energized / disabled
uint8 REASON_SOFT_ESTOP = 3          # Soft emergency stop (dispatch_mode=0)
uint8 REASON_OTHER = 255              # Other reason
uint8 reason

# Human-readable description of the current state (empty when no special state)
string description
