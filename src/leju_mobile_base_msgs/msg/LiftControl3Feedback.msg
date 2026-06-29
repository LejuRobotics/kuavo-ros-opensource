# Lift control process feedback message

# Current lift position (mm)
float32 current_position

# Target lift position (mm)
float32 target_position

# Current lift velocity (mm/s)
float32 velocity

# Lift status
uint8 STATUS_IDLE = 0
uint8 STATUS_MOVING_UP = 1
uint8 STATUS_MOVING_DOWN = 2
uint8 STATUS_HOLDING = 3
uint8 STATUS_ERROR = 4
uint8 status

# Progress percentage (0-100)
uint8 progress

# Error code (0 if no error)
uint32 error_code
