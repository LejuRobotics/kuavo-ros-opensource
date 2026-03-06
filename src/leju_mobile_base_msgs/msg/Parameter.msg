# Lower-level controller parameter information

Header header

# Robot dimensions
float32 wheel_base      # Distance between front and rear wheels (m)
float32 track_width     # Distance between left and right wheels (m)
float32 wheel_radius    # Wheel radius (m)

# Maximum velocities
float32 max_linear_vel  # Maximum linear velocity (m/s)
float32 max_angular_vel # Maximum angular velocity (rad/s)

# Maximum accelerations
float32 max_linear_acc  # Maximum linear acceleration (m/s^2)
float32 max_angular_acc # Maximum angular acceleration (rad/s^2)

# Motor parameters
uint8 motor_count       # Number of motors
float32[] motor_ratios  # Gear ratios for each motor

# Control parameters
float32 pid_linear_p    # PID P gain for linear velocity
float32 pid_linear_i    # PID I gain for linear velocity
float32 pid_linear_d    # PID D gain for linear velocity
float32 pid_angular_p   # PID P gain for angular velocity
float32 pid_angular_i   # PID I gain for angular velocity
float32 pid_angular_d   # PID D gain for angular velocity

# Safety parameters
float32 safety_stop_distance  # Emergency stop distance (m)
float32 slow_down_distance    # Slow down distance (m)
