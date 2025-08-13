#!/usr/bin/env python3
# coding: utf-8
from kuavo_humanoid_sdk.kuavo.core.navigation import KuavoRobotNavigationCore, NavigationStatus
import tf
from geometry_msgs.msg import Pose, Point, Quaternion
import rospy
import time

class RobotNavigation:
    """Interface class for robot navigation."""

    def __init__(self):
        """Initialize RobotNavigation object."""
        self.robot_navigation = KuavoRobotNavigationCore()

    def navigate_to_goal(
        self, x: float, y: float, z: float, roll: float, pitch: float, yaw: float
    ) -> bool:
        """Navigate to the specified goal.

        Args:
            x (float): x coordinate of the goal.
            y (float): y coordinate of the goal.
            z (float): z coordinate of the goal.
            roll (float): roll of the goal.
            pitch (float): pitch of the goal.
            yaw (float): yaw of the goal.

        Returns:
            bool: Whether navigation succeeded.
        """
        orientation = tf.transformations.quaternion_from_euler(yaw, pitch, roll)
        goal = Pose(position=Point(x=x, y=y, z=z), orientation=Quaternion(x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3]))
        self.robot_navigation.navigate_to_goal(goal)
        while self.get_current_status() is not NavigationStatus.ACTIVE:
            time.sleep(0.01)
        while not rospy.is_shutdown():
            if self.get_current_status() == NavigationStatus.SUCCEEDED:
                break
            time.sleep(0.01)
        return True

    def navigate_to_task_point(self, task_point_name: str) -> bool:
        """Navigate to the specified task point.

        Args:
            task_point_name (str): Name of the task point.

        Returns:
            bool: Whether navigation succeeded.
        """
        self.robot_navigation.navigate_to_task_point(task_point_name)
        while self.get_current_status() is not NavigationStatus.ACTIVE:
            time.sleep(0.01)
        while not rospy.is_shutdown():
            if self.get_current_status() == NavigationStatus.SUCCEEDED:
                break
            time.sleep(0.01)
        return True

    def stop_navigation(self) -> bool:
        """Stop navigation.

        Returns:
            bool: Whether stopping navigation succeeded.
        """
        return self.robot_navigation.stop_navigation()

    def get_current_status(self) -> str:
        """Get the current navigation status.

        Returns:
            str: Current navigation status.
        """
        return self.robot_navigation.get_current_status()

    def init_localization_by_pose(
        self, x: float, y: float, z: float, roll: float, pitch: float, yaw: float
    ) -> bool:
        """Initialize localization by pose.

        Args:
            x (float): x coordinate of the pose.
            y (float): y coordinate of the pose.
            z (float): z coordinate of the pose.
            roll (float): roll of the pose.
            pitch (float): pitch of the pose.
            yaw (float): yaw of the pose.

        Returns:
            bool: Whether localization initialization succeeded.
        """
        orientation = tf.transformations.quaternion_from_euler(yaw, pitch, roll)
        pose = Pose(position=Point(x=x, y=y, z=z), orientation=Quaternion(x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3]))
        return self.robot_navigation.init_localization_by_pose(pose)
    
    def init_localization_by_task_point(
        self, task_point_name: str
    ) -> bool:
        """Initialize localization by task point.

        Args:
            task_point_name (str): Name of the task point.

        Returns:
            bool: Whether localization initialization succeeded.
        """
        return self.robot_navigation.init_localization_by_task_point(task_point_name)

    def load_map(self, map_name: str) -> bool:
        """Load a map.

        Args:
            map_name (str): Name of the map.

        Returns:
            bool: Whether loading the map succeeded.
        """
        return self.robot_navigation.load_map(map_name)

    def get_all_maps(self) -> list:
        """Get all map names.

        Returns:
            list: List of map names.
        """
        return self.robot_navigation.get_all_maps()

    def get_current_map(self) -> str:
        """Get the current map name.

        Returns:
            str: Current map name.
        """
        return self.robot_navigation.get_current_map()
