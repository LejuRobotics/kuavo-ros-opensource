import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3Stamped
import time

class ConveyorController:
    def __init__(self):
        self.belt_speed_pub = rospy.Publisher('/belt/speed_command', Vector3Stamped, queue_size=10)
        time.sleep(0.5)
        
    def control_speed(self, speed):
        """
        控制传送带速度
        Args:
            speed (float): 传送带速度 (-0.1 到 0.1 m/s, 正值向前，负值向后)
        """
        msg = Vector3Stamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "belt"
        msg.vector.x = max(-0.1, min(0.1, speed))
        msg.vector.y = 0.0
        msg.vector.z = 0.0 
        
        self.belt_speed_pub.publish(msg)
        print(f"Belt speed command: {msg.vector.x} m/s")
        
    def stop(self):
        """停止传送带"""
        self.control_speed(0.0)
        print("Belt stopped")