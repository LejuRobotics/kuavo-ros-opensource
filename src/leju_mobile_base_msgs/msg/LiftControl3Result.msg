# Lift control execution result message

# Final lift position (mm)
float32 final_position

# Execution result
uint8 RESULT_SUCCESS = 0
uint8 RESULT_FAILED = 1
uint8 RESULT_TIMEOUT = 2
uint8 RESULT_CANCELLED = 3
uint8 RESULT_OUT_OF_RANGE = 4
uint8 result

# Error code (0 if successful)
uint32 error_code

# Error message
string error_msg

# Execution time (seconds)
float32 execution_time
