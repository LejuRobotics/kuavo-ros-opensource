#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import rospy
import subprocess
from enum import Enum
from std_srvs.srv import Trigger, TriggerRequest
from geometry_msgs.msg import Twist
import signal
from std_msgs.msg import Bool
from automatic_test.conftest import get_node_list, get_node_pid
HUMANOID_ROBOT_SESSION_NAME = "humanoid_robot"
LAUNCH_HUMANOID_ROBOT_SIM_CMD = "roslaunch humanoid_controllers load_kuavo_gazebo_sim.launch trace_path_automatic_test:=true output_system_info:=false rviz:=false"
LAUNCH_HUMANOID_ROBOT_REAL_CMD = "roslaunch humanoid_controllers load_kuavo_real.launch"

class HardwareStatus(Enum):
    HARDWARE_STATUS_NOT_READY = -1
    HARDWARE_STATUS_PREPARED = 0
    HARDWARE_STATUS_READY = 1

class RobotManager:
    def __init__(self):
        self.sim = rospy.get_param('~sim', True)
        self.robot_hardware_status = rospy.get_param('/hardware/is_ready', HardwareStatus.HARDWARE_STATUS_NOT_READY)
        self.node_name = "RobotManager"
        self.robot_ready = False
        rospy.set_param('/robot_ready', self.robot_ready)
        self.cmd_pose_pub = rospy.Publisher('/cmd_pose', Twist, queue_size=10)
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _signal_handler(self, sig, frame):
        """内部信号处理函数"""
        print("Received signal to terminate the program")
        try:
            self.stop_humanoid_robot()
        except:
            pass
        rospy.signal_shutdown("Received signal to terminate the program")

    def check_humanoid_robot_node_status(self):
        nodes = get_node_list()
        node_exist = False
        for node in nodes:
            if "nodelet_manager" in node:
                node_exist = True
                break
        if node_exist:
            if get_node_pid("nodelet_manager") is None:
                node_exist = False
        return node_exist

    def start_humanoid_robot(self):
        subprocess.run(["tmux", "kill-session", "-t", HUMANOID_ROBOT_SESSION_NAME], 
                    stderr=subprocess.DEVNULL) 
        if self.sim:
            launch_cmd = LAUNCH_HUMANOID_ROBOT_SIM_CMD
        else:
            launch_cmd = LAUNCH_HUMANOID_ROBOT_REAL_CMD
       
        rospy.loginfo(f"[{self.node_name}] launch_cmd: {launch_cmd}")
        rospy.loginfo(f"[{self.node_name}] If you want to check the session, please run 'tmux attach -t humanoid_robot'")
        tmux_cmd = [
            "tmux", "new-session",
            "-s", HUMANOID_ROBOT_SESSION_NAME, 
            "-d",  
            f"  {launch_cmd}; exec bash"
        ]

        process = subprocess.Popen(
            tmux_cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            env=os.environ.copy()
        )

        rospy.sleep(5.0)
        result = subprocess.run(["tmux", "has-session", "-t", HUMANOID_ROBOT_SESSION_NAME], 
                                capture_output=True)
        if result.returncode == 0:
            rospy.loginfo(f"[{self.node_name}] Started humanoid_robot in tmux session: {HUMANOID_ROBOT_SESSION_NAME}")
        else:
            rospy.logerr(f"[{self.node_name}] Failed to start humanoid_robot")
            raise Exception("Failed to start humanoid_robot")

        if self.sim == False:
            try:
                self._wait_for_hardware_status(
                    target_status=HardwareStatus.HARDWARE_STATUS_PREPARED,
                    timeout=120,
                    message="Waiting for hardware to be prepared"
                )
                
                self.call_real_initialize()
                
                self._wait_for_hardware_status(
                    target_status=HardwareStatus.HARDWARE_STATUS_READY,
                    timeout=120,
                    message="Waiting for hardware to be ready"
                )
            except TimeoutError as e:
                rospy.logerr(f"[{self.node_name}] {str(e)}")
                raise Exception(f"Hardware initialization failed: {str(e)}")
            except Exception as e:
                rospy.logerr(f"[{self.node_name}] Unexpected error during hardware initialization: {str(e)}")
                raise
        
        self.robot_ready = True
        rospy.set_param('/robot_ready', self.robot_ready)
        
    def call_real_initialize(self):
        client = rospy.ServiceProxy('/humanoid_controller/real_initial_start', Trigger)
        req = TriggerRequest()

        try:
            # Call the service
            if client.call(req):
                rospy.loginfo(f"[{self.node_name}] RealInitializeSrv service call successful")
            else:
                rospy.logerr(f"[{self.node_name}] Failed to callRealInitializeSrv service")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def stop_humanoid_robot(self):
        rospy.loginfo(f"[{self.node_name}] Stopping humanoid_robot")
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = -0.3
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        
        rate = rospy.Rate(10)  # 10Hz
        for i in range(20):
            self.cmd_pose_pub.publish(twist)
            rate.sleep()
        
        rospy.sleep(1.0)
        
        subprocess.run(["tmux", "kill-session", "-t", HUMANOID_ROBOT_SESSION_NAME], 
                    stderr=subprocess.DEVNULL) 
        self.robot_ready = False
        rospy.set_param('/robot_ready', self.robot_ready)

    def _wait_for_hardware_status(self, target_status, timeout=120, message="Waiting for hardware status"):
        """
        Wait for hardware to reach the specified status
        
        Args:
            target_status: Target hardware status
            timeout: Timeout (seconds)
            message: Message during waiting
            
        Raises:
            TimeoutError: If the target status is not reached within the timeout
            Exception: If the humanoid robot node is not found
        """
        start_time = rospy.get_time()
        rate = rospy.Rate(1)  # 1Hz check frequency
        
        while not rospy.is_shutdown():
            if not self.check_humanoid_robot_node_status():
                raise Exception("Humanoid robot node not found")
            
            current_status = rospy.get_param('/hardware/is_ready', HardwareStatus.HARDWARE_STATUS_NOT_READY)
            
            if current_status == target_status.value:
                return
            
            if (rospy.get_time() - start_time) > timeout:
                raise TimeoutError(f"Timeout waiting for hardware status. Expected {target_status}, current status: {current_status}")
            
            rospy.loginfo(f"[{self.node_name}] {message} (Status: {current_status})")
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node('robot_manager_node')
    robot_manager = RobotManager()
    
    try:
        robot_manager.start_humanoid_robot()
        while not rospy.is_shutdown(): 
            rospy.sleep(1)
             
    except rospy.ROSInterruptException:
        pass
    finally:
        pass
