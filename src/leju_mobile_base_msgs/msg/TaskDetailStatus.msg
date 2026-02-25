# Task detail status information with waypoint-level progress
# This message is published only when task status changes
# Does not publish if no task is currently executing

Header header

# Current task ID
string task_id

# Waypoint progress information
uint32 current_waypoint_index      # Index of waypoint currently navigating to
uint32[] passed_waypoints           # List of waypoint indices already passed
uint32 total_waypoints              # Total number of waypoints in the task

# Time and distance estimates
float32 estimated_time_remaining    # Estimated remaining time in seconds
float32 distance_to_next_waypoint   # Distance to next waypoint in meters

# Additional status information
uint8 TASK_NONE = 0
uint8 TASK_RUNNING = 1
uint8 TASK_PAUSED = 2
uint8 TASK_COMPLETED = 3
uint8 TASK_FAILED = 4
uint8 task_status                   # Current task execution status

# Error information (when task_status == TASK_FAILED)
uint32 error_code                   # Error code (0 if no error)
string error_msg                    # Error description
