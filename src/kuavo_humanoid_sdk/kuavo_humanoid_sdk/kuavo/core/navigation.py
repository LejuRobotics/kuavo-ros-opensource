import time
import math
import rospy
import threading
import numpy as np
from typing import Tuple
from transitions import Machine, State
from geometry_msgs.msg import Pose
from kuavo_humanoid_sdk.common.logger import SDKLogger
from kuavo_humanoid_sdk.kuavo.core.ros.navigation import Navigation, NavigationStatus

class KuavoRobotNavigationCore:
    def __init__(self):
        self.robot_navigation = Navigation()

    def stop_navigation(self) -> bool:
        """
        stop navigation
        """
        return self.robot_navigation.pub_stop_navigation()

    def get_current_status(self) -> NavigationStatus:
        """
        get current status
        """
        return self.robot_navigation.get_current_status()

    def load_map(self, map_name: str) -> bool:
        """
        load map
        """
        return self.robot_navigation.srv_load_map(map_name) 

    def get_current_map(self) -> str:
        """
        get current map
        """
        return self.robot_navigation.srv_get_current_map()

    def get_all_maps(self) -> list:
        """
        get all maps
        """
        return self.robot_navigation.srv_get_all_maps()
    
    def navigate_to_task_point(self, task_point_name: str) -> bool:
        """
        navigate to task point
        """
        return self.robot_navigation.srv_navigate_to_task_point(task_point_name)
    
    def init_localization_by_pose(self, pose: Pose) -> bool:
        """
        initialize localization by pose
        """
        return self.robot_navigation.pub_init_localization_by_pose(pose)
    
    def init_localization_by_task_point(self, task_point_name: str) -> bool:
        """
        initialize localization by task point
        """
        return self.robot_navigation.srv_init_localization_by_task_point(task_point_name)
    
    def navigate_to_goal(self, goal: Pose) -> bool:
        """
        navigate to goal
        """
        return self.robot_navigation.pub_navigate_to_goal(goal)
    
    