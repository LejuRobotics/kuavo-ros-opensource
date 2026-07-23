# High-precision lift control goal message

# Target lift position (mm)
float32 target_position

# Maximum velocity (mm/s)
float32 max_velocity

# Maximum acceleration (mm/s^2)
float32 max_acceleration

# Position tolerance (mm)
float32 position_tolerance

# Control mode
uint8 MODE_POSITION = 0
uint8 MODE_VELOCITY = 1
uint8 MODE_FORCE = 2
uint8 control_mode

# Force control target (N, only for MODE_FORCE)
float32 target_force

# Timeout (seconds, 0 for no timeout)
float32 timeout
