std_msgs/Header header

int8 kError = -1                
int8 kUnknown = 0              
int8 kMoving = 1              
int8 kReached = 2            
int8 kGrabbed = 3         

int8[] state # 0:; left; 1: right
kuavo_msgs/endEffectorData data