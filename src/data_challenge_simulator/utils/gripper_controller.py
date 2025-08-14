import rospy
from kuavo_msgs.msg import Rq2f85ClawCmd
from geometry_msgs.msg import Vector3Stamped
import time
import threading

class GripperController:
    def __init__(self, publish_frequency=200.0):
        """
        初始化夹爪控制器
        Args:
            publish_frequency (float): 命令发布频率 (Hz)，默认100Hz
        """
        self.gripper_cmd_pub = rospy.Publisher('/gripper/command', Rq2f85ClawCmd, queue_size=10)
        self.current_left_cmd = 0.0
        self.current_right_cmd = 0.0
        
        # 发布频率设置
        self.publish_frequency = publish_frequency
        self.publish_rate = rospy.Rate(publish_frequency)
        
        # 线程控制
        self.is_running = True
        self.command_lock = threading.Lock()
        
        # 启动定时发布线程
        self.publish_thread = threading.Thread(target=self._publish_loop)
        self.publish_thread.daemon = True
        self.publish_thread.start()
        
        rospy.loginfo(f"Gripper Controller initialized with {publish_frequency}Hz publishing frequency")
        
    def _publish_loop(self):
        """
        定时发布当前夹爪命令的线程函数
        持续以指定频率发布当前状态的夹爪命令
        """
        while not rospy.is_shutdown() and self.is_running:
            try:
                with self.command_lock:
                    msg = Rq2f85ClawCmd()
                    msg.header.stamp = rospy.Time.now()
                    msg.header.frame_id = "gripper"
                    msg.left_cmd = self.current_left_cmd
                    msg.right_cmd = self.current_right_cmd
                
                self.gripper_cmd_pub.publish(msg)
                self.publish_rate.sleep()
                
            except rospy.ROSInterruptException:
                break
        
        rospy.loginfo("Gripper command publishing loop stopped")
    
    def set_gripper_position(self, left_cmd, right_cmd):
        """
        设置夹爪命令
        Args:
            left_cmd (float): 左夹爪命令 (0-255, 0=完全张开, 255=完全闭合)
            right_cmd (float): 右夹爪命令 (0-255, 0=完全张开, 255=完全闭合)
        """
        with self.command_lock:
            # 限制命令范围并更新当前状态
            self.current_left_cmd = max(0.0, min(255.0, left_cmd))
            self.current_right_cmd = max(0.0, min(255.0, right_cmd))
        
        rospy.loginfo(f"Gripper target updated - Left: {self.current_left_cmd}, Right: {self.current_right_cmd}")
    
    def control_right_gripper(self, right_cmd):
        """
        控制右夹爪
        Args:
            right_cmd (float): 右夹爪命令 (0-255, 0=完全张开, 255=完全闭合)
        """
        with self.command_lock:
            # 保持左夹爪当前状态，只改变右夹爪
            self.current_right_cmd = max(0.0, min(255.0, right_cmd))
        
        rospy.loginfo(f"Right gripper target updated - Left: {self.current_left_cmd}, Right: {self.current_right_cmd}")

    def control_left_gripper(self, left_cmd):
        """
        控制左夹爪
        Args:
            left_cmd (float): 左夹爪命令 (0-255, 0=完全张开, 255=完全闭合)
        """
        with self.command_lock:
            # 保持右夹爪当前状态，只改变左夹爪
            self.current_left_cmd = max(0.0, min(255.0, left_cmd))
        
        rospy.loginfo(f"Left gripper target updated - Left: {self.current_left_cmd}, Right: {self.current_right_cmd}")

    def open_grippers(self):
        """张开两个夹爪"""
        self.set_gripper_position(0.0, 0.0)
        rospy.loginfo("Opening both grippers")
        
    def close_grippers(self):
        """闭合两个夹爪"""
        self.set_gripper_position(255.0, 255.0)
        rospy.loginfo("Closing both grippers")
    
    def get_current_commands(self):
        """
        获取当前夹爪命令
        Returns:
            tuple: (left_cmd, right_cmd)
        """
        with self.command_lock:
            return self.current_left_cmd, self.current_right_cmd
    
    def stop(self):
        """
        停止定时发布线程
        """
        self.is_running = False
        if self.publish_thread.is_alive():
            self.publish_thread.join()
        rospy.loginfo("Gripper Controller stopped")