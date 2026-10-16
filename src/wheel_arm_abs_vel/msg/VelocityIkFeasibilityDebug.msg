uint8 MODE_OFF=0
uint8 MODE_SHADOW=1
uint8 MODE_ACTIVE=2

uint8 STATE_NORMAL=0
uint8 STATE_APPROACH=1
uint8 STATE_ESCAPE=2
uint8 GROUP_BASE=0
uint8 GROUP_LOWER_BODY=1
uint8 GROUP_LEFT_ARM=2
uint8 GROUP_RIGHT_ARM=3

# Workspace/singularity observability for absolute non-MPC velocity IK.
# In MODE_SHADOW every suggestion is diagnostic. In MODE_ACTIVE the workspace
# feasible pose remains diagnostic because the whole-body primary task uses the
# raw target; joint-limit barriers remain active. singular_escape_direction is
# diagnostic until a later validated design.
std_msgs/Header header
# Every variable-length per-DoF array below uses this exact name/group order.
uint32 dof_layout_version
string[] dof_names
uint8[] dof_groups
time source_stamp
uint64 target_seq
uint64 session_id
uint64 generation
uint64 reference_seq
uint8 control_state
uint8 mode
bool valid
bool tracking_active
bool workspace_projection_applied
bool joint_limit_barrier_applied
bool operational_joint_limit_applied
float64 sample_age

# All poses and points below use header.frame_id.
bool[2] arm_controlled
geometry_msgs/Pose[2] raw_tcp_pose
geometry_msgs/Pose[2] feasible_tcp_pose
geometry_msgs/Pose[2] reference_tcp_pose
geometry_msgs/Point[2] shoulder_position

# torso_pose is expressed in header.frame_id. The coupled workspace is
# expressed relative to torso_constraint_reference_frame, so base translation
# and yaw do not consume lower-body workspace. Constraint order is [torso_x,
# torso_z, knee+leg+waist_pitch, waist_yaw]. The last two values are angular
# coordinates in radians; the first two are positions in metres.
string torso_frame
string torso_constraint_reference_frame
geometry_msgs/Pose torso_pose
float64[4] torso_constraint_value
float64[4] torso_constraint_lower
float64[4] torso_constraint_upper
float64[4] torso_lower_margin
float64[4] torso_upper_margin
bool torso_constraints_valid

# Workspace projection. projection_distance_by_reserve is arm-major:
# [left/reserve0..2, right/reserve0..2].
float64[2] nominal_arm_reach
float64 selected_reach_reserve
float64[3] reach_reserve_candidates
float64[2] shoulder_to_raw_target
float64[2] shoulder_to_feasible_target
float64[2] shoulder_to_reference_tcp
float64[2] raw_reach_reserve
float64[2] selected_projection_distance
float64[6] projection_distance_by_reserve
bool[2] projection_active

# sigma_translation is for Jp. sigma_scaled_6d is for [Jp; Lc*Jw], so position
# and orientation rows have compatible units. sigma_unscaled_6d preserves the
# old [Jp; Jw] metric for comparison only.
float64 characteristic_length
float64[2] sigma_translation
float64[2] sigma_scaled_6d
float64[2] sigma_unscaled_6d
uint8[2] singularity_state

# Margins use the effective limits seen by velocity IK, including the optional
# shoulder-pitch upper limit. Ratios are normalized by each joint range.
float64[] joint_lower_margin
float64[] joint_upper_margin
float64[] joint_margin_ratio
float64[2] minimum_joint_margin_ratio
uint8[2] joint_limit_state

# The URDF limits above remain the absolute physical boundary.  The following
# limits define the preferred operating envelope around a fixed natural whole-
# body posture. Crossing this envelope never clamps q or blocks inward motion:
# only velocity farther toward the physical boundary is smoothly attenuated.
float64[] joint_safe_center
float64[] joint_safe_lower
float64[] joint_safe_upper
float64[] joint_safe_range_ratio
float64[] joint_safe_margin_ratio
float64[] joint_outward_velocity_scale
float64[2] minimum_safe_margin_ratio
uint8[2] operational_limit_state

# Unitless diagnostics. singular_escape_direction is selected by a
# q +/- epsilon*v_min look-ahead but is never applied. limit_barrier_direction
# points inward and is applied as a lower-priority QP objective in MODE_ACTIVE.
float64[] singular_escape_direction
float64[] limit_barrier_direction
float64[2] escape_score_plus
float64[2] escape_score_minus
