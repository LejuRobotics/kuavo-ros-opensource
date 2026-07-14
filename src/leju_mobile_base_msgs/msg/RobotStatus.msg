# Robot task status information
# This message is published only when there is a change in status

Header header

# Robot operating mode
uint8 MODE_IDLE = 0
uint8 MODE_MANUAL = 1
uint8 MODE_AUTO = 2
uint8 MODE_ERROR = 3
uint8 mode

# Task execution status
uint8 TASK_NONE = 0
uint8 TASK_RUNNING = 1
uint8 TASK_PAUSED = 2
uint8 TASK_COMPLETED = 3
uint8 TASK_FAILED = 4
uint8 task_status

# Current task ID
string task_id

# Task progress (0-100)
uint8 progress

# Error code (0 if no error)
uint32 error_code

# Error description
string error_msg
