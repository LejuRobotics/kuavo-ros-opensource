#!/usr/bin/env python3
# coding: utf-8
from kuavo_humanoid_sdk.kuavo.core.navigation import KuavoRobotNavigationCore, NavigationStatus
import tf
from geometry_msgs.msg import Pose, Point, Quaternion
import rospy
import time

class RobotNavigation:
    """机器人导航接口类。"""

    def __init__(self):
        """初始化 RobotNavigation 对象。"""
        self.robot_navigation = KuavoRobotNavigationCore()

    def navigate_to_goal(
        self, x: float, y: float, z: float, roll: float, pitch: float, yaw: float
    ) -> bool:
        """导航到指定目标位置。

        Args:
            x (float): 目标点的x坐标。
            y (float): 目标点的y坐标。
            z (float): 目标点的z坐标。
            roll (float): 目标点的横滚角。
            pitch (float): 目标点的俯仰角。
            yaw (float): 目标点的偏航角。

        Returns:
            bool: 导航是否成功。
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
        """导航到指定的任务点。

        Args:
            task_point_name (str): 任务点的名称。

        Returns:
            bool: 导航是否成功。
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
        """停止导航。

        Returns:
            bool: 停止导航是否成功。
        """
        return self.robot_navigation.stop_navigation()

    def get_current_status(self) -> str:
        """获取当前导航状态。

        Returns:
            str: 当前导航状态。
        """
        return self.robot_navigation.get_current_status()

    def init_localization_by_pose(
        self, x: float, y: float, z: float, roll: float, pitch: float, yaw: float
    ) -> bool:
        """通过位姿初始化定位。

        Args:
            x (float): 位姿的x坐标。
            y (float): 位姿的y坐标。
            z (float): 位姿的z坐标。
            roll (float): 位姿的横滚角。
            pitch (float): 位姿的俯仰角。
            yaw (float): 位姿的偏航角。

        Returns:
            bool: 定位初始化是否成功。
        """
        orientation = tf.transformations.quaternion_from_euler(yaw, pitch, roll)
        pose = Pose(position=Point(x=x, y=y, z=z), orientation=Quaternion(x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3]))
        return self.robot_navigation.init_localization_by_pose(pose)
    
    def init_localization_by_task_point(
        self, task_point_name: str
    ) -> bool:
        """通过任务点初始化定位。

        Args:
            task_point_name (str): 任务点的名称。

        Returns:
            bool: 定位初始化是否成功。
        """
        return self.robot_navigation.init_localization_by_task_point(task_point_name)

    def load_map(self, map_name: str) -> bool:
        """加载地图。

        Args:
            map_name (str): 地图名称。

        Returns:
            bool: 加载地图是否成功。
        """
        return self.robot_navigation.load_map(map_name)

    def get_all_maps(self) -> list:
        """获取所有地图名称。

        Returns:
            list: 地图名称列表。
        """
        return self.robot_navigation.get_all_maps()

    def get_current_map(self) -> str:
        """获取当前地图名称。

        Returns:
            str: 当前地图名称。
        """
        return self.robot_navigation.get_current_map()
