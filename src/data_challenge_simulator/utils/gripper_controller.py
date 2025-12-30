import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3Stamped
import time
import threading

class GripperController:
    def __init__(self, publish_frequency=100.0):
        """
        初始化夹爪控制器
        Args:
            publish_frequency (float): 命令发布频率 (Hz)，默认200Hz
        """
        self.gripper_cmd_pub = rospy.Publisher('/gripper/command', JointState, queue_size=10)
        self.gripper_state_sub = rospy.Subscriber('/gripper/state', JointState, self._state_callback)
        
        # 当前命令值 (0-255范围)
        self.current_left_cmd = 0.0
        self.current_right_cmd = 0.0
        
        # 当前状态值 (0-0.8范围)
        self.current_left_position = 0.0
        self.current_right_position = 0.0
        
        # 发布频率设置
        self.publish_frequency = publish_frequency
        self.publish_rate = rospy.Rate(publish_frequency)
        
        # 线程控制
        self.is_running = True
        self.command_lock = threading.Lock()
        self.state_lock = threading.Lock()
        
        # 启动定时发布线程
        self.publish_thread = threading.Thread(target=self._publish_loop)
        self.publish_thread.daemon = True
        self.publish_thread.start()
        
        rospy.loginfo(f"Gripper Controller initialized with {publish_frequency}Hz publishing frequency")
        
    def _state_callback(self, msg):
        """
        夹爪状态回调函数
        Args:
            msg (JointState): 夹爪状态消息
        """
        with self.state_lock:
            # 解析关节状态
            for i, name in enumerate(msg.name):
                if name == "left_gripper_joint" and i < len(msg.position):
                    self.current_left_position = msg.position[i]
                elif name == "right_gripper_joint" and i < len(msg.position):
                    self.current_right_position = msg.position[i]
        
    def _publish_loop(self):
        """
        定时发布当前夹爪命令的线程函数
        持续以指定频率发布当前状态的夹爪命令
        """
        while not rospy.is_shutdown() and self.is_running:
            try:
                with self.command_lock:
                    msg = JointState()
                    msg.header.stamp = rospy.Time.now()
                    msg.header.frame_id = "gripper"
                    
                    # 设置关节名称和位置命令
                    msg.name = ["left_gripper_joint", "right_gripper_joint"]
                    msg.position = [self.current_left_cmd, self.current_right_cmd]
                    
                    msg.velocity = []
                    msg.effort = []
                
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
            tuple: (left_cmd, right_cmd) - 命令值 (0-255)
        """
        with self.command_lock:
            return self.current_left_cmd, self.current_right_cmd
    
    def get_current_positions(self):
        """
        获取当前夹爪实际位置
        Returns:
            tuple: (left_position, right_position) - 实际位置 (0-0.8)
        """
        with self.state_lock:
            return self.current_left_position, self.current_right_position
    
    def wait_for_position(self, target_left=None, target_right=None, tolerance=0.02, timeout=5.0):
        """
        等待夹爪到达目标位置
        Args:
            target_left (float): 目标左夹爪位置，None表示不检查
            target_right (float): 目标右夹爪位置，None表示不检查
            tolerance (float): 位置容差
            timeout (float): 超时时间(秒)
        Returns:
            bool: 是否在超时前到达目标位置
        """
        start_time = time.time()
        
        while not rospy.is_shutdown() and (time.time() - start_time) < timeout:
            left_pos, right_pos = self.get_current_positions()
            
            left_ok = target_left is None or abs(left_pos - target_left) < tolerance
            right_ok = target_right is None or abs(right_pos - target_right) < tolerance
            
            if left_ok and right_ok:
                return True
                
            time.sleep(0.01)  # 10ms检查间隔
        
        rospy.logwarn("Gripper position wait timeout")
        return False
    
    def stop(self):
        """
        停止定时发布线程
        """
        self.is_running = False
        if self.publish_thread.is_alive():
            self.publish_thread.join()
        rospy.loginfo("Gripper Controller stopped")
