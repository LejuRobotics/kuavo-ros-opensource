uint8 AUTHORITY_INACTIVE=0
uint8 AUTHORITY_ARMED=1
uint8 AUTHORITY_ACTIVE=2
uint8 AUTHORITY_BRAKING=3
uint8 AUTHORITY_HOLD=4

uint8 GROUP_BASE=0
uint8 GROUP_LOWER_BODY=1
uint8 GROUP_LEFT_ARM=2
uint8 GROUP_RIGHT_ARM=3

std_msgs/Header header
uint32 dof_layout_version
string[] dof_names
uint8[] dof_groups

# A session/generation change is an atomic reference discontinuity. Consumers
# must reject mixed samples rather than independently updating DoF groups.
uint64 session_id
uint64 generation
uint64 reference_seq
uint8 authority_state
bool valid
bool takeover_requested
bool takeover_active

string world_frame
string session_frame
bool session_anchor_valid
bool base_feedback_valid

# Exactly one atomic 21-D sample in dof_names order.
float64[] q
float64[] dq
float64[] ddq

# Final supervisor diagnostics carried with the command sample.
float64 task_scale
bool safety_braking
bool emergency_dynamics_used

# Execution-owner recovery protocol. Zero epoch is the initial producer lease.
# Consumer selects a new opaque epoch on timeout; producer may rejoin only
# after echoing its stopped snapshot and receiving AUTHORITY_ACTIVE feedback.
uint64 execution_epoch
uint64 recovery_anchor_seq
# Same joint order; software stopping contract, NOT hardware certification.
float64[] stopping_acceleration
float64[] position_lower
float64[] position_upper

# Shared Linux boot/monotonic domain; generated time is immutable on resend.
string execution_clock_domain
string producer_instance
string execution_owner_instance
int64 generated_monotonic_ns
# Independent echo heartbeat sequence; reference_seq still names a q/v/a state.
uint64 owner_feedback_seq
# Owner grant deadline in the verified shared domain. Consumer computes the
# input lease from generated_monotonic_ns + configured timeout, not this field.
int64 producer_lease_deadline_ns
