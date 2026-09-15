# SG100 Hand State Message
# Published by sg100_hand_ros_node at fixed frequency (e.g., 50Hz)
# Feedback layout: 11 joints per hand (same order as SG100HandCommand)

std_msgs/Header header

# Control mode enums (mirror SDK CtrlMode_e in sg100_types.h)
int8 MODE_NULL=0
int8 MODE_POSITION=1
int8 MODE_SPEED=2
int8 MODE_TORQUE=3
int8 MODE_SPEED_TORQUE=4
int8 MODE_POS_VEL_TRQ=5
int8 MODE_OPENLOOP=6
int8 MODE_JOINT_POSITION=7
int8 MODE_IMPEDANCE=8
int8 MODE_JOINT_IMPEDANCE=9

# Connection status
bool left_hand_connected
bool right_hand_connected

# Error codes (0 = OK)
uint8 left_error_code
uint8 right_error_code

# Left hand joint states (11 joints)
# Order: [TH_CMC_ABD, TH_MCP_FLEX, TH_IP_FLEX,
#         IF_MCP_ABD, IF_MCP_FLEX, IF_PIP_FLEX,
#         MF_MCP_FLEX, MF_PIP_FLEX,
#         LF_MCP_ABD, LF_MCP_FLEX, LF_PIP_FLEX]
float32[] left_hand_positions      # Current joint positions (rad)
float32[] left_hand_velocities     # Current joint velocities (rad/s)
float32[] left_hand_currents       # Motor Q-axis currents (A)
float32[] left_hand_currents_d     # Motor D-axis currents (A)
uint8[] left_hand_modes            # Control modes per joint (SDK CtrlMode_e codes), 默认填充为 MODE_JOINT_POSITION

# Right hand joint states (11 joints)
# Same order as left hand
float32[] right_hand_positions
float32[] right_hand_velocities
float32[] right_hand_currents
float32[] right_hand_currents_d
uint8[] right_hand_modes

# Communication statistics
uint32 left_frame_count    # Number of frames received from left hand
uint32 right_frame_count   # Number of frames received from right hand
