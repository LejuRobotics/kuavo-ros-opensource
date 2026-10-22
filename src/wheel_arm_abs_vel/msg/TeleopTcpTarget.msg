uint8 SOURCE_UNKNOWN=0
uint8 SOURCE_QUEST_CONTROLLER=1
uint8 SOURCE_QUEST_HAND_TRACKING=2

# Absolute non-MPC command lifecycle.  ARMED mirrors measured joints while the
# arm-mode handshake is in flight; ACTIVE may drive velocity IK; HOLD asks the
# trajectory supervisor to brake and retain the current session.
uint8 CONTROL_INACTIVE=0
uint8 CONTROL_ARMED=1
uint8 CONTROL_ACTIVE=2
uint8 CONTROL_HOLD=3
# The velocity-IK owner keeps mode 2 and drives a bounded joint-space return
# of the arms to ready pose. Locked folding joints hold their current pose.
# This is deliberately not legacy mode 1.
uint8 CONTROL_PREPARE=4

# header.frame_id describes hand TCP poses and elbow hints. The fixed-base
# contract uses quest_chest_robot_axes: Chest-relative, fixed robot-anatomical
# axes, with configured shoulder offsets. Body poses have their own frame below.
std_msgs/Header header

# These names identify the robot frames controlled by the 6-D velocity IK.
string left_controlled_frame
string right_controlled_frame

# Each pose is one rigid TCP target: position and orientation have identical
# frame semantics.  They must never contain the legacy l6-position/l7-attitude
# hybrid target.
geometry_msgs/Pose left_tcp_pose
geometry_msgs/Pose right_tcp_pose

# Optional secondary redundancy hints.  TCP tracking remains the primary task.
geometry_msgs/Point left_elbow_hint
geometry_msgs/Point right_elbow_hint
bool left_elbow_hint_valid
bool right_elbow_hint_valid

bool left_valid
bool right_valid
uint8 source

# Atomic human-body intent from the very same Quest skeleton packet as the
# hands above.  Poses use the monitor's right-handed Quest tracking frame;
# consumers must use only session-relative deltas and must never interpret
# Root as robot odometry. An explicit source-clock/session reset changes the
# body epoch; human motion magnitude is never used to infer a coordinate reset.
string body_tracking_frame
# Host-side timing is separate from the source/device header stamp.
time mapper_receive_stamp
time mapper_publish_stamp
float64 mapper_processing_time
float64 source_relative_age
geometry_msgs/Pose root_pose
geometry_msgs/Pose chest_pose
geometry_msgs/Pose head_pose
geometry_msgs/Pose left_shoulder_pose
geometry_msgs/Pose right_shoulder_pose
bool root_valid
bool chest_valid
bool head_valid
bool left_shoulder_valid
bool right_shoulder_valid
bool body_high_confidence
uint64 body_coordinate_epoch

# Transaction identity shared by Python target mapping, velocity IK and WBC
# command diagnostics.  A generation change invalidates all in-flight work.
uint64 session_id
uint64 generation
uint8 absolute_state
uint8 control_state
# PREPARE command authorization is distinct from hand/body pose validity.
# Consumers must admit the fixed transaction with valid pose once before
# continuing it on fresh control_valid packets with left/right_valid=false.
bool control_valid

# Explicit folding/torso permission, atomic with the input session and pose.
# False requires lower-joint braking/hold; hand tracking cannot recruit them.
bool torso_control_enabled

# Arm-only pause within an ACTIVE torso session; does not release the owner.
# Both arm joint groups brake/hold, while torso permission remains independent.
# Sequence orders pause/resume commands even when they share a skeleton stamp.
bool arm_hold
uint64 arm_operation_seq

# Version 1: causal skeleton CV estimates, expressed in each pose's frame.
# False retains compatibility with synthetic/legacy publishers. When true,
# these twists replace (never supplement) finite-difference feedforward.
bool estimated_twists_valid
geometry_msgs/Twist left_tcp_twist
geometry_msgs/Twist right_tcp_twist
geometry_msgs/Vector3 left_elbow_velocity
geometry_msgs/Vector3 right_elbow_velocity
geometry_msgs/Twist chest_twist
geometry_msgs/Vector3 root_velocity
