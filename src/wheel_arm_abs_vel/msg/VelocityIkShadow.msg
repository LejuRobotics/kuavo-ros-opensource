uint8 WAITING_FOR_INPUT=0
uint8 TRACKING=1
uint8 STALE_BRAKING=2
uint8 HOLD=3
uint8 MODE_BLOCKED=4
uint8 QP_ERROR=5
uint8 SAFETY_BRAKING=6
uint8 RECOVERING=7
uint8 PREPARING=8

uint8 CANDIDATE_UNAVAILABLE=0
uint8 CANDIDATE_ACCEPTED=1
uint8 CANDIDATE_TOO_OLD=2
uint8 CANDIDATE_CONTRACT_MISMATCH=3
uint8 CANDIDATE_REFERENCE_LAG=4
uint8 CANDIDATE_FUTURE_TARGET=5
uint8 APPLIED_NONE=0
uint8 APPLIED_NEW_SOLUTION=1
uint8 APPLIED_ZERO_ORDER_HOLD=2
uint8 APPLIED_BRAKING=3
uint8 APPLIED_PREPARATION=4
uint8 GROUP_BASE=0
uint8 GROUP_LOWER_BODY=1
uint8 GROUP_LEFT_ARM=2
uint8 GROUP_RIGHT_ARM=3

std_msgs/Header header
# Every variable-length per-DoF array below uses this exact name/group order.
uint32 dof_layout_version
string[] dof_names
uint8[] dof_groups
time source_stamp
uint64 target_seq
uint64 session_id
uint64 generation
uint8 control_state
float64 target_age
float64 cycle_dt
float64 solve_time_us
uint8 state
bool valid
bool solver_valid
# Return code of the primary 21-D QP.  Zero is qpOASES success; negative
# values are node-local contract/bound failures declared by the solver.
int32 primary_qp_return_code
bool fallback_active
bool emergency_acceleration_used
bool takeover_active
uint32 consecutive_solver_failures
uint64 solver_base_seq
float64 candidate_age
uint8 candidate_decision
uint8 applied_command_mode
uint64 candidate_session_id
uint64 candidate_generation
uint64 candidate_target_seq
uint32 candidate_reference_lag
uint64 candidate_target_lag

# Whole-body coordinate contract. The target arrives in a frozen session frame and
# is resolved exactly once through session_anchor_pose into world_frame.
string world_frame
string source_target_frame
string session_frame
bool session_anchor_valid
uint64 anchor_session_id
uint64 anchor_generation
uint64 anchor_base_coordinate_epoch
bool base_feedback_valid
float64 base_state_age
bool base_coordinate_jump_detected
uint64 base_coordinate_epoch
# True when base motion is explicitly disabled, regardless of recruitment.
# The 21-D wire layout is retained; the three base velocities are QP equalities.
bool base_recruitment_locked
# In the development-only shadow process, base q remains odometry state while
# base dq is a non-commanded IK suggestion. Formal takeover changes this
# ownership contract and removes the shadow process.
bool base_reference_position_from_feedback
geometry_msgs/Pose measured_base_pose
geometry_msgs/Pose reference_base_pose
geometry_msgs/Pose session_anchor_pose

# Human-body intent is sampled atomically with both hand targets.  BODY_MIRROR
# moves the hand targets and torso/base task together; REACH_ASSIST moves only
# robot body DoFs while leaving the world hand target unchanged.
uint8 BASE_REASON_NONE=0
uint8 BASE_REASON_ROOT_INTENT=1
uint8 BASE_REASON_REACH_ASSIST=2
uint8 BASE_REASON_BOTH=3
bool body_intent_valid
string body_tracking_frame
uint64 body_coordinate_epoch
geometry_msgs/Pose raw_root_pose
geometry_msgs/Pose raw_chest_pose
float64[2] root_translation_intent
float64 root_translation_confidence
bool root_translation_intent_active
float64 relative_hand_motion
float64 reach_assist_demand
float64[4] desired_torso_feature
float64 torso_task_error
float64 torso_task_scale
float64 base_intent_error
float64 base_intent_task_scale
uint8 base_recruitment_reason
# Explicit hand/body decomposition. The normal target stays relative to the
# current robot torso; only geometric reach assistance blends toward the
# separately recorded world target.
geometry_msgs/Pose[2] torso_relative_tcp_pose
geometry_msgs/Pose[2] current_torso_world_tcp_pose
geometry_msgs/Pose[2] reach_assist_world_tcp_pose
float64 reach_assist_blend
# Legacy aliases: raw_world is the current-torso projection, resolved_world is
# the actual geometric-assist blend consumed by IK. The historical governor
# name remains only for bag/API compatibility; there is no temporal governor.
geometry_msgs/Pose[2] raw_world_tcp_pose
geometry_msgs/Pose[2] resolved_world_tcp_pose
bool cartesian_governor_initialized
float64[2] cartesian_governor_position_error
float64[2] cartesian_governor_orientation_error
float64[2] cartesian_governor_linear_speed
float64[2] cartesian_governor_angular_speed
float64 base_translation_tracking_error
float64 base_yaw_tracking_error
# Margins use [x, y, yaw] relative to the frozen session anchor.
float64[3] base_session_lower_margin
float64[3] base_session_upper_margin

float64[] q
# Same-cycle measured joint/base state used by the dynamic QP and watchdog.
# This is diagnostic only and follows the identical 21-D layout as q.
float64[] measured_q
# Contract v2: fixed-body-relative full pose, one 18-D task QP.
uint8 task_contract_version
uint8 active_dof_count
float64[] qp_reference_q
geometry_msgs/Pose[2] measured_torso_relative_tcp_pose
geometry_msgs/Pose[2] reference_torso_relative_tcp_pose
float64[6] measured_task_error
bool[2] hand_task_active
float64[2] hand_orientation_weight
float64[4] measured_torso_feature
float64[4] reference_torso_feature
float64[] dq
float64[] ddq
# Strict hierarchy diagnostics. task_dq is already inside the common next-step
# q/dq/ddq domain; the publisher performs no normal-path motion shaping.
# secondary_dq is constrained to the global 6/12-D controlled-TCP nullspace
# and must not alter either TCP's higher-priority task velocity.
float64[] task_dq
float64[] primary_dq
float64[] secondary_dq
uint32 primary_task_rank
uint32 nullspace_dimension
float64 nullspace_residual
float64 primary_task_leakage
int32 secondary_qp_return_code
# Norms use [base, lower_body, left_arm, right_arm].
float64[4] group_velocity_norm

# Continuous recruitment. Each arm pressure is max(reach, singularity, joint),
# and whole_body_pressure is the maximum over controlled arms. Target values
# are the memoryless smooth-step result. Active values describe the stateful
# geometric admission policy; solver values are those used by the synchronous
# 500 Hz solve-and-commit cycle that produced primary_dq/task_dq.
bool continuous_recruitment_enabled
bool workspace_projection_applied_to_primary
float64[2] reach_pressure
float64[2] singularity_pressure
float64[2] joint_pressure
float64[2] arm_pressure
float64 whole_body_pressure
float64 lower_body_recruitment_target
float64 base_recruitment_target
float64 lower_body_recruitment
float64 base_recruitment
float64 solver_lower_body_recruitment
float64 solver_base_recruitment
# Ordered capability diagnostics. Residuals are RMS values in the common
# characteristic-length-scaled Cartesian task units.  The base may be
# recruited only after arms+lower-body capability has been evaluated.
float64 arm_only_task_residual
float64 arm_lower_task_residual
float64 lower_body_saturation
float64 primary_task_scale
# Per-controlled-arm translation progress selected inside the single
# dynamically feasible 21-D QP.  A value below one means the current physical
# q/dq/ddq domain cannot realize the complete instantaneous request; it never
# creates or advances a separate Cartesian reference state.
float64[2] translation_task_scale
# Per-joint lower-body cascade uses decision-layout lower-body order
# [knee_joint, leg_joint, waist_pitch_joint, waist_yaw_joint].  Behavioural
# admission order is the reverse: waist yaw -> waist pitch -> leg -> knee.
float64[4] lower_joint_recruitment_target
float64[4] lower_joint_recruitment
float64[4] solver_lower_joint_recruitment
float64[4] lower_joint_directional_effectiveness
float64 geometric_reach_demand
float64 base_admission_dwell
bool base_admission_active
# Stateful session cascade.  Stage 0 is arm-only; stages 1..4 count admitted
# lower joints and stage 5 denotes planar-base admission.
uint8 cascade_stage
bool[4] lower_joint_stage_active

# X+A ready-pose lifecycle.  During PREPARING the same 21-D ReferenceManager
# remains the sole owner; base is held and no legacy position-IK source runs.
bool preparation_active
bool preparation_complete
float64 preparation_progress
# Reserved legacy field; false for task_contract_version=2. Natural-hang
# startup acquires the current human target through the same constrained QP.
bool startup_bootstrap_active
# Compatibility diagnostics for the physical velocity domain.  Normal
# tracking is solved directly in that domain and has no post-QP follower.
float64 supervisor_lower_body_recruitment
float64 supervisor_base_recruitment
# The dynamic fault is latched until a new session/generation.  A false
# supervisor-valid sample never reuses the unsafe previous velocity.
bool dynamic_supervisor_valid
bool dynamic_fault_latched
float64 arm_reference_tracking_error
float64[2] arm_reference_tracking_error_by_arm
float64 lower_reference_tracking_error
float64 reference_tracking_scale
bool reference_tracking_soft_limited
bool reference_tracking_fault_latched
float64 reference_tracking_recovery_elapsed
# Per-arm zero-centred regularization used by the TCP-primary QP.  This is a
# stability/conditioning term, not a behavioural secondary objective.
float64[2] primary_regularization
# Contract v2: orientation_task_scale is achieved/requested angular progress,
# not a weight or a command multiplier. A zero request reports zero progress.
# position_task_weight_scale remains 1; hand_orientation_weight
# is the explicit active pose weight, independent of joint soft-margin gating.
float64[2] orientation_task_scale
float64[2] position_task_weight_scale
# Historical dynamic-supervisor diagnostics.  Normal tracking enforces q/dq/
# ddq in its single QP and therefore publishes zero jerk slack.
float64[] jerk_slack
float64[] boundary_brake
# Marks per-joint outward attenuation in the configured soft operating region.
# This envelope is not a hard wall; URDF position limits remain the hard
# protection.
float64[] operational_boundary_brake
# task_scale and per-task scales are compatibility progress diagnostics only;
# they do not modify the input target or multiply a committed QP velocity.
bool task_scaling_active
# Historical field name retained for bag/API compatibility. For contract v2
# it reports acceptance of the single full-pose/body residual QP.
bool task_scaling_qp_valid
float64 task_scale
bool torso_viability_relaxed
float64 torso_viability_relaxation
float64[] unscaled_task_twist
# Raw Cartesian feedback/feedforward request before the constrained full-pose QP.
float64[] requested_task_twist
float64[] task_scaling_baseline_twist
float64[] scaled_task_twist
float64[] achieved_task_twist
# Compatibility residual; zero on the direct normal tracking path.
float64 task_scaling_residual
# Translation-only singularity safety used by the active QP. sigma_min below
# remains the characteristic-length-scaled 6-D TCP conditioning diagnostic.
float64[2] translation_sigma
bool[2] singularity_escape_constraint_active
# True when an activated singularity row was numerically infeasible and the
# solver retried this cycle with physical/velocity and torso constraints only.
bool singularity_escape_constraint_fallback
float64[2] sigma_min
float64[6] task_error
