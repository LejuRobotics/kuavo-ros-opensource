import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3Stamped
import time

class ConveyorController:
    def __init__(self):
        """初始化传送带控制器，支持两个传送带"""
        self.belt_speed_pub = rospy.Publisher('/belt/speed_command', Vector3Stamped, queue_size=10, latch=True)
        self.belt_speed_pub2 = rospy.Publisher('/belt/speed_command2', Vector3Stamped, queue_size=10, latch=True)
        time.sleep(0.5)
        
    def control_speed(self, speed, belt_id=1):
        """
        控制传送带速度
        Args:
            speed (float): 传送带速度 (-0.1 到 0.1 m/s, 正值向前，负值向后)
            belt_id (int): 传送带ID，1或2，默认为1（保持向后兼容）
        """
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
        """
        同时控制两个传送带的速度
        Args:
            speed1 (float): 传送带1速度 (-0.1 到 0.1 m/s)
            speed2 (float): 传送带2速度 (-0.1 到 0.1 m/s)
        """
        self.control_speed(speed1, belt_id=1)
        self.control_speed(speed2, belt_id=2)
        
    def stop(self, belt_id=None):
        """
        停止传送带
        Args:
            belt_id (int, optional): 传送带ID，1或2。如果为None，则停止所有传送带
        """
        if belt_id is None:
            self.control_speed(0.0, belt_id=1)
            self.control_speed(0.0, belt_id=2)
            print("All belts stopped")
        else:
            self.control_speed(0.0, belt_id=belt_id)
            print(f"Belt {belt_id} stopped")