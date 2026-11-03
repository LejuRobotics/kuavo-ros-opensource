uint8 REJECTED_INVALID=0
uint8 ACCEPTED_INITIAL=1
uint8 ACCEPTED_UNIQUE=2
uint8 ACCEPTED_DUPLICATE=3
uint8 REJECTED_SOURCE=4

# This low-rate input diagnostic is recorded only by the Task07 focused bag.
std_msgs/Header header
time source_stamp
uint8 source
uint8 decision
bool accepted
bool duplicate

# Packet cadence and source header cadence. They are not pose sample cadence.
float64 packet_dt
float64 source_stamp_dt

# Non-zero only when a changed pose is accepted for feedforward differencing.
float64 unique_sample_dt
float64 feedforward_sample_age
float64 max_position_delta
float64 max_orientation_delta

uint64 received_count
uint64 accepted_count
uint64 unique_count
uint64 duplicate_count
uint64 rejected_source_count
uint64 rejected_invalid_count
