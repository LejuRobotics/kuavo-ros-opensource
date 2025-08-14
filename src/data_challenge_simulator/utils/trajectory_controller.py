import rospy
import threading
import time
from typing import List, Optional
import numpy as np

class TrajectoryController:
    """
    高频轨迹控制器 - 持续以指定频率发布当前目标关节角度
    """
    
    def __init__(self, robot, publish_frequency=200.0):
        """
        初始化轨迹控制器
        Args:
            robot: KuavoRobot实例
            publish_frequency (float): 发布频率 (Hz)，默认100Hz
        """
        self.robot = robot
        self.publish_frequency = publish_frequency
        self.publish_rate = rospy.Rate(publish_frequency)
        
        # 当前目标关节角度
        self.current_target_positions = [0.0] * 14
        
        # 线程控制
        self.is_running = True
        self.position_lock = threading.Lock()
        
        # 启动定时发布线程
        self.publish_thread = threading.Thread(target=self._publish_loop)
        self.publish_thread.daemon = True
        self.publish_thread.start()
        
        rospy.loginfo(f"Trajectory Controller initialized with {publish_frequency}Hz publishing frequency")
    
    def _publish_loop(self):
        """
        定时发布当前目标关节角度的线程函数
        """
        while not rospy.is_shutdown() and self.is_running:
            try:
                with self.position_lock:
                    # 获取当前目标位置的副本
                    current_positions = self.current_target_positions.copy()
                
                # 发布关节角度命令
                self.robot.control_arm_joint_positions(current_positions)
                self.publish_rate.sleep()
                
            except rospy.ROSInterruptException:
                break
            except Exception as e:
                rospy.logwarn(f"Trajectory publishing error: {e}")
                time.sleep(0.01)  # 短暂延时后重试
        
        rospy.loginfo("Trajectory publishing loop stopped")
    
    def set_target_positions(self, positions: List[float]):
        """
        设置目标关节角度 - 线程安全版本
        Args:
            positions (List[float]): 14个关节的目标角度 [左臂7个, 右臂7个]
        """
        if len(positions) != 14:
            rospy.logwarn(f"Expected 14 joint positions, got {len(positions)}")
            return False
            
        with self.position_lock:
            self.current_target_positions = positions.copy()
        
        rospy.logdebug(f"Target positions updated: {positions}")
        return True
    
    def get_current_target_positions(self) -> List[float]:
        """
        获取当前目标关节角度 - 线程安全版本
        Returns:
            List[float]: 当前目标关节角度
        """
        with self.position_lock:
            return self.current_target_positions.copy()
    
    def execute_trajectory(self, q_list: List[List[float]], sleep_time: float = 0.02):
        """
        执行轨迹序列
        Args:
            q_list (List[List[float]]): 轨迹点列表，每个点包含14个关节角度
            sleep_time (float): 每个轨迹点之间的延时
        """
        # rospy.loginfo(f"Executing trajectory with {len(q_list)} points")
        
        for i, q in enumerate(q_list):
            if rospy.is_shutdown():
                break
                
            self.set_target_positions(q)
            time.sleep(sleep_time)
            
            # 可选：显示进度
            if (i + 1) % 10 == 0 or i == len(q_list) - 1:
                rospy.logdebug(f"Trajectory progress: {i+1}/{len(q_list)}")
        
        rospy.loginfo("Trajectory execution completed")
    
    def smooth_transition_to(self, target_positions: List[float], 
                           duration: float = 1.0, num_steps: int = None):
        """
        平滑过渡到目标位置
        Args:
            target_positions (List[float]): 目标关节角度
            duration (float): 过渡时间(秒)
            num_steps (int): 插值步数，如果为None则根据发布频率自动计算
        """
        if len(target_positions) != 14:
            rospy.logwarn(f"Expected 14 joint positions, got {len(target_positions)}")
            return False
        
        # 获取当前位置
        current_positions = self.get_current_target_positions()
        
        # 计算插值步数
        if num_steps is None:
            num_steps = int(duration * self.publish_frequency / 10)  # 除以10是为了减少计算量
        
        # 生成插值轨迹
        trajectory = []
        for i in range(num_steps + 1):
            alpha = i / num_steps
            interpolated = []
            for j in range(14):
                interpolated.append(current_positions[j] + alpha * (target_positions[j] - current_positions[j]))
            trajectory.append(interpolated)
        
        # 执行平滑过渡
        sleep_time = duration / num_steps
        self.execute_trajectory(trajectory, sleep_time)
        
        rospy.loginfo(f"Smooth transition completed in {duration}s")
        return True
    
    def hold_position(self, duration: float = None):
        """
        保持当前位置
        Args:
            duration (float): 保持时间(秒)，如果为None则持续保持
        """
        rospy.loginfo(f"Holding current position for {duration}s" if duration else "Holding current position indefinitely")
        
        if duration:
            time.sleep(duration)
        else:
            # 持续保持直到被打断
            try:
                rospy.spin()
            except rospy.ROSInterruptException:
                pass
    
    def stop(self):
        """
        停止轨迹控制器
        """
        rospy.loginfo("Stopping trajectory controller...")
        self.is_running = False
        if self.publish_thread.is_alive():
            self.publish_thread.join(timeout=2.0)
        rospy.loginfo("Trajectory Controller stopped")
