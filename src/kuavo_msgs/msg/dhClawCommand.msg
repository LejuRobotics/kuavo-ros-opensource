# kuavo_msgs/endEffectorData:
# string[] name  
# float64[] position
# float64[] velocity  
# float64[] effort
# 
# ** For the Topic Notes **
# 
# name     : 'left_claw' , 'right_claw'
# position : 0 ~ 100, the percentage of the claw's opening angle
#            0: closed, 100: open   
# velocity : 0 ~ 100, if size is 0, will use default `50.0`.
# effort   : DH force percent 20~100, if size is 0, will use default `50.0`.
# 
# ** Example **
# data:
#   - name: ['left_claw', 'right_claw']
#     position: [20.0, 20.0]
#     velocity: [50.0, 50.0]
#     effort: [50.0, 50.0]

std_msgs/Header header
kuavo_msgs/endEffectorData data
