# SG100 hand feedback. Positions are radians and velocities are radians/second.
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

bool left_hand_connected
bool right_hand_connected
uint8 left_error_code
uint8 right_error_code

float32[] left_hand_positions
float32[] left_hand_velocities
float32[] left_hand_currents
float32[] left_hand_currents_d
uint8[] left_hand_modes

float32[] right_hand_positions
float32[] right_hand_velocities
float32[] right_hand_currents
float32[] right_hand_currents_d
uint8[] right_hand_modes

uint32 left_frame_count
uint32 right_frame_count
