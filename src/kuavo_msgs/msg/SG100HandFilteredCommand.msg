# SG100 Hand Filtered Command Message
# Published by sg100_hand_ros_node at the control loop rate (e.g. 50Hz).
# Contains the low-pass filtered position values actually sent to the SDK this
# cycle (after applying cmd_filter_alpha to the received /sg100_hand_command).
# Kept in a dedicated topic so /sg100_hand_command and /sg100_hand_state stay
# free of filtered data, and so recording a single topic captures both hands.
# Arrays are empty until the filter has been initialized from valid hardware
# feedback (consumers can detect "no valid filtered command yet").

std_msgs/Header header

# Left hand filtered positions (11 joints; unit: radians, rad)
# Same joint order as SG100HandCommand.left_hand_positions:
# [TH_CMC_ABD, TH_MCP_FLEX, TH_IP_FLEX,
#  IF_MCP_ABD, IF_MCP_FLEX, IF_PIP_FLEX,
#  MF_MCP_FLEX, MF_PIP_FLEX,
#  LF_MCP_ABD, LF_MCP_FLEX, LF_PIP_FLEX]
float32[] left_hand_filtered_positions

# Right hand filtered positions (11 joints; unit: radians, rad)
# Same joint order as left hand
float32[] right_hand_filtered_positions
