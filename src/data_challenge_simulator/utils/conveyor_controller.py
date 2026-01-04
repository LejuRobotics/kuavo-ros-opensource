import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3Stamped
import time

class ConveyorController:
    def __init__(self):
        self.belt_speed_pub = rospy.Publisher('/belt/speed_command', Vector3Stamped, queue_size=10, latch=True)
        self.belt_speed_pub2 = rospy.Publisher('/belt/speed_command2', Vector3Stamped, queue_size=10, latch=True)
        time.sleep(0.5)
        
    def control_speed(self, speed, belt_id=1):
        msg = Vector3Stamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = f"belt{belt_id}"
        msg.vector.x = max(-0.1, min(0.1, speed))
        msg.vector.y = 0.0
        msg.vector.z = 0.0 
        
        if belt_id == 1:
            self.belt_speed_pub.publish(msg)
            print(f"Belt 1 speed command: {msg.vector.x} m/s")
        elif belt_id == 2:
            self.belt_speed_pub2.publish(msg)
            print(f"Belt 2 speed command: {msg.vector.x} m/s")
        else:
            raise ValueError(f"Invalid belt_id: {belt_id}. Must be 1 or 2.")
        
    def control_speed_both(self, speed1, speed2):
        self.control_speed(speed1, belt_id=1)
        self.control_speed(speed2, belt_id=2)
        
    def stop(self, belt_id=None):
        if belt_id is None:
            self.control_speed(0.0, belt_id=1)
            self.control_speed(0.0, belt_id=2)
            print("All belts stopped")
        else:
            self.control_speed(0.0, belt_id=belt_id)
            print(f"Belt {belt_id} stopped")