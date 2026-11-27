# DAgger arm candidate and takeover readiness produced by KRC.

std_msgs/Header header
uint32 schema_version
string session
uint64 sequence

# [left arm 7, right arm 7], radians and radians/second.
float64[14] position
float64[14] velocity

bool left_requested
bool right_requested
bool left_ready
bool right_ready
uint64 left_generation
uint64 right_generation

# Nested messages always exist on the ROS wire; this flag states whether KRC
# populated the waist candidate for this packet.
bool waist_valid
kuavo_msgs/DaggerWaistCandidate waist
