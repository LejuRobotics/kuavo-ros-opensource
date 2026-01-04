import rospy
import threading
import time
from typing import List, Optional
import numpy as np

class TrajectoryController:
    
    def __init__(self, robot, publish_frequency=100.0):

        self.robot = robot
        self.publish_frequency = publish_frequency
        self.publish_rate = rospy.Rate(publish_frequency)

        self.current_target_positions = [0.0] * 14

        self.is_running = True
        self.position_lock = threading.Lock()

        self.publish_thread = threading.Thread(target=self._publish_loop)
        self.publish_thread.daemon = True
        self.publish_thread.start()
        
        rospy.loginfo(f"Trajectory Controller initialized with {publish_frequency}Hz publishing frequency")
    
    def _publish_loop(self):

        while not rospy.is_shutdown() and self.is_running:
            try:
                with self.position_lock:

                    current_positions = self.current_target_positions.copy()

                self.robot.control_arm_joint_positions(current_positions)
                self.publish_rate.sleep()
                
            except rospy.ROSInterruptException:
                break
            except Exception as e:
                rospy.logwarn(f"Trajectory publishing error: {e}")
                time.sleep(0.01)
        
        rospy.loginfo("Trajectory publishing loop stopped")
    
    def set_target_positions(self, positions: List[float]):

        if len(positions) != 14:
            rospy.logwarn(f"Expected 14 joint positions, got {len(positions)}")
            return False
            
        with self.position_lock:
            self.current_target_positions = positions.copy()
        
        rospy.logdebug(f"Target positions updated: {positions}")
        return True
    
    def get_current_target_positions(self) -> List[float]:

        with self.position_lock:
            return self.current_target_positions.copy()
    
    def execute_trajectory(self, q_list: List[List[float]], sleep_time = 0.02):

        if isinstance(sleep_time, (list, tuple)):

            if len(sleep_time) != len(q_list):
                rospy.logwarn(f"sleep_time 列表长度 ({len(sleep_time)}) 与轨迹点数量 ({len(q_list)}) 不匹配，使用第一个值")
                sleep_time = sleep_time[0] if sleep_time else 0.02
                use_list = False
            else:
                use_list = True
        else:

            use_list = False
        
        for i, q in enumerate(q_list):
            if rospy.is_shutdown():
                break
                
            self.set_target_positions(q)

            if use_list:
                time.sleep(sleep_time[i])
            else:
                time.sleep(sleep_time)
            
            if (i + 1) % 10 == 0 or i == len(q_list) - 1:
                rospy.logdebug(f"Trajectory progress: {i+1}/{len(q_list)}")
        
        rospy.loginfo("Trajectory execution completed")
    
    def smooth_transition_to(self, target_positions: List[float], 
                           duration: float = 1.0, num_steps: int = None):

        if len(target_positions) != 14:
            rospy.logwarn(f"Expected 14 joint positions, got {len(target_positions)}")
            return False

        current_positions = self.get_current_target_positions()

        if num_steps is None:
            num_steps = int(duration * self.publish_frequency / 10) 

        trajectory = []
        for i in range(num_steps + 1):
            alpha = i / num_steps
            interpolated = []
            for j in range(14):
                interpolated.append(current_positions[j] + alpha * (target_positions[j] - current_positions[j]))
            trajectory.append(interpolated)

        sleep_time = duration / num_steps
        self.execute_trajectory(trajectory, sleep_time)
        
        rospy.loginfo(f"Smooth transition completed in {duration}s")
        return True
    
    def hold_position(self, duration: float = None):

        rospy.loginfo(f"Holding current position for {duration}s" if duration else "Holding current position indefinitely")
        
        if duration:
            time.sleep(duration)
        else:

            try:
                rospy.spin()
            except rospy.ROSInterruptException:
                pass
    
    def stop(self):

        rospy.loginfo("Stopping trajectory controller...")
        self.is_running = False
        if self.publish_thread.is_alive():
            self.publish_thread.join(timeout=2.0)
        rospy.loginfo("Trajectory Controller stopped")
