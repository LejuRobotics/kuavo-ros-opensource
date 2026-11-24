# header.stamp is the host CAN query-start time, not publish time.
std_msgs/Header header
# Explicit sample identity; ROS1 roscpp rewrites Header.seq on publication.
# Cached republishing retains this field and cannot count as another sample.
uint32 feedback_sequence

int8 kError = -1                
int8 kUnknown = 0              
int8 kMoving = 1              
int8 kReached = 2            
int8 kGrabbed = 3         
int8 kDropped = 4           # DH 夹爪：物体掉落

# Unknown also indicates missing or unreadable feedback for that side.
int8[] state # 0: left; 1: right
# position: measured percent, 0 open / 100 closed; retain last valid value on
# position read failure and report kUnknown. velocity/effort are command echoes,
# not measured speed or physical force.
kuavo_msgs/endEffectorData data
