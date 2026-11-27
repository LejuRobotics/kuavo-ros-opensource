# DAgger four-joint lower-body candidate produced by KRC.

# Oldest upstream input used to solve this candidate. Re-publishing a candidate
# must not refresh this timestamp.
time source_stamp

string config_session
bool requested
bool ready
uint64 generation

# [knee, leg, waist_pitch, waist_yaw], radians.
float64[4] position
