# SG100 Hand Command Message
# Used to control 11-DOF SG100 command layout per hand (joint positions in radians, rad)
# Published by ik_ros_uni.py, subscribed by sg100_hand_ros_node

std_msgs/Header header

# Control mode (global fallback when per-joint arrays are empty)
# Enums mirror the SDK CtrlMode_e in sg100_types.h — same numbering end-to-end,
# so the C++ node passes modes through to the SDK without translation.
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

int8 control_mode  # Default: MODE_JOINT_POSITION (global fallback)

# Left hand joint positions (11 joints; unit: radians, rad)
# Order: [TH_CMC_ABD, TH_MCP_FLEX, TH_IP_FLEX,
#         IF_MCP_ABD, IF_MCP_FLEX, IF_PIP_FLEX,
#         MF_MCP_FLEX, MF_PIP_FLEX,
#         LF_MCP_ABD, LF_MCP_FLEX, LF_PIP_FLEX]
float32[] left_hand_positions

# Right hand joint positions (11 joints; unit: radians, rad)
# Same order as left hand
float32[] right_hand_positions

# Optional: joint velocities (11 joints each, in rad/s)
# Only used when control_mode is MODE_SPEED or for feedforward
float32[] left_hand_velocities
float32[] right_hand_velocities

# Enable flags (bitmask for which joints to control)
# 0x07FF = control all 11 joints (bits 0-10)
uint16 left_enable_mask
uint16 right_enable_mask

# === Per-joint parameters (11 elements each, optional) ===
# When these arrays are non-empty and have JOINT_COUNT (11) elements,
# they override the global control_mode / ros params on a per-joint basis.
# Empty arrays → use global fallback (backward compatible).

# Per-joint control mode (MODE_JOINT_POSITION=7, MODE_JOINT_IMPEDANCE=9, etc.)
# SDK Hand_Control_t supports: 7 (position fields), 9 (impedance fields), 0 (NULL).
# Other modes have no frame fields and are rejected (joint falls back to MODE_NULL).
int8[] left_hand_control_mode
int8[] right_hand_control_mode

# Per-joint PID / impedance gains (for MODE_JOINT_POSITION or MODE_JOINT_IMPEDANCE)
float32[] left_hand_kp
float32[] right_hand_kp
float32[] left_hand_kd
float32[] right_hand_kd

# Per-joint torque feedforward (for MODE_JOINT_IMPEDANCE)
float32[] left_hand_torque_ff
float32[] right_hand_torque_ff

# Per-joint output limit (0.0 ~ 1.0, encoded by SDK)
float32[] left_hand_output_limit
float32[] right_hand_output_limit
