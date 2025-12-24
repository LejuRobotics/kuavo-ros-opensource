twoArmHandPose  hand_poses
# params for the IK solver
bool use_custom_ik_param
bool joint_angles_as_q0
int32 desire_mode  # 0: end-effector command in World Frame | 1: end-effector command in Local Frame | 2: joint command
ikSolveParam ik_param
int32 frame # 0 keep current frame  1 world frame (based on odom)  2  local frame   3  manipulation world frame 