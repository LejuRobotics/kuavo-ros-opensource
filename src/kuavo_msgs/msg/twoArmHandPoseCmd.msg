twoArmHandPose  hand_poses
# params for the IK solver
bool use_custom_ik_param
bool joint_angles_as_q0
ikSolveParam ik_param
int32 frame # 0 keep current frame  1 world frame (based on odom)  2  local frame  3  VRFrame  4  manipulation world frame   5  joint space
# 发布时刻的时间戳（Unix毫秒），用于测量节点间通信延迟
int64 timestamp_ms