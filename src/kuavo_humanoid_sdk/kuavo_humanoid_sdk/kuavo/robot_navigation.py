#!/usr/bin/env python3
# coding: utf-8
from kuavo_humanoid_sdk.kuavo.core.navigation import KuavoRobotNavigationCore, NavigationStatus
import tf
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import OccupancyGrid
import rospy
import time
import math

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

    def navigate_to_point_with_heading(self, x: float, y: float, heading: float) -> bool:
        """前往指定坐标点并设置朝向。

        Args:
            x (float): 目标点的x坐标（PNG像素坐标）。
            y (float): 目标点的y坐标（PNG像素坐标）。
            heading (float): 目标朝向角度（度），0度为正东方向，90度为正北方向。

        Returns:
            bool: 导航是否成功。
        """
        # PNG坐标转换为Map坐标系（导航使用map坐标系）
        map_x, map_y = self._png_to_map(x, y)
        if map_x is None or map_y is None:
            print("PNG坐标转换失败，无法执行导航")
            return False

        print(f"PNG坐标 ({x}, {y}) 转换为Map坐标 ({map_x}, {map_y})")

        # 将角度转换为弧度
        yaw = math.radians(heading)

        # 使用转换后的Map坐标进行导航
        return self.navigate_to_goal(x=map_x, y=map_y, z=0.0, roll=0.0, pitch=0.0, yaw=yaw)

    def _png_to_map(self, png_x: float, png_y: float, map_info=None):
        """
        PNG坐标系转Map坐标系（不使用TF，直接转换）

        导航使用Map坐标系，所以PNG坐标需要转换为Map坐标

        Args:
            png_x: PNG坐标系X坐标（像素）
            png_y: PNG坐标系Y坐标（像素）
            map_info: 地图信息字典，包含resolution, origin, width, height（可选）

        Returns:
            tuple: (map_x, map_y) Map坐标系坐标（米）
        """
        try:
            # 从/map话题获取地图消息
            msg = rospy.wait_for_message('/map', OccupancyGrid, timeout=5.0)

            # 从地图消息中提取信息
            map_info = {
                "width": msg.info.width,
                "height": msg.info.height,
                "resolution": msg.info.resolution,
                "origin": {
                    "x": msg.info.origin.position.x,
                    "y": msg.info.origin.position.y,
                    "z": msg.info.origin.position.z
                }
            }

            # 获取地图参数
            resolution = map_info.get("resolution", 0.05)  # 米/像素
            origin_x = map_info.get("origin", {}).get("x", 0.0)  # 地图原点X（米）
            origin_y = map_info.get("origin", {}).get("y", 0.0)  # 地图原点Y（米）
            width = map_info.get("width", 0)  # 图片宽度（像素）
            height = map_info.get("height", 0)  # 图片高度（像素）

            if resolution <= 0 or width <= 0 or height <= 0:
                print("错误: 地图参数无效")
                return None, None

            # PNG坐标系 -> Map坐标系（不使用TF）
            # Map_X = Origin_X + PNG_X * Resolution
            # Map_Y = Origin_Y + (Height - PNG_Y) * Resolution
            map_x = origin_x + png_x * resolution
            map_y = origin_y + (height - png_y) * resolution

            return map_x, map_y

        except Exception as e:
            print(f"PNG转Map坐标失败: {e}")
            return None, None
