# SG100 hand command. Joint order is identical for both hands:
# thumb 3, index 3, middle 2, little 3. Positions are radians.
std_msgs/Header header

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

int8 control_mode
float32[] left_hand_positions
float32[] right_hand_positions
float32[] left_hand_velocities
float32[] right_hand_velocities
uint16 left_enable_mask
uint16 right_enable_mask

int8[] left_hand_control_mode
int8[] right_hand_control_mode
float32[] left_hand_kp
float32[] right_hand_kp
float32[] left_hand_kd
float32[] right_hand_kd
float32[] left_hand_torque_ff
float32[] right_hand_torque_ff
float32[] left_hand_output_limit
float32[] right_hand_output_limit
