#!/usr/bin/env python3
# coding: utf-8
from kuavo_humanoid_sdk.kuavo.core.navigation import KuavoRobotNavigationCore, NavigationStatus
import tf
import threading
import time
import math
import roslibpy
from kuavo_humanoid_sdk.common.websocket_kuavo_sdk import WebSocketKuavoSDK


class RobotNavigation:
    """Robot navigation interface class."""

    def __init__(self):
        """Initialize RobotNavigation object."""
        self.robot_navigation = KuavoRobotNavigationCore()

    def navigate_to_goal(
        self, x: float, y: float, z: float, roll: float, pitch: float, yaw: float
    ) -> bool:
        """Navigate to the specified goal position.

        Args:
            x (float): Target x coordinate.
            y (float): Target y coordinate.
            z (float): Target z coordinate.
            roll (float): Target roll angle.
            pitch (float): Target pitch angle.
            yaw (float): Target yaw angle.

        Returns:
            bool: Whether navigation was successful.
        """
        orientation = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        goal = {
            "position": {"x": x, "y": y, "z": z},
            "orientation": {"x": orientation[0], "y": orientation[1], "z": orientation[2], "w": orientation[3]}
        }
        self.robot_navigation.navigate_to_goal(goal)

        # Wait for navigation to become ACTIVE, with timeout and failure-state handling.
        _FAILURE_STATES = (
            NavigationStatus.ABORTED,
            NavigationStatus.REJECTED,
            NavigationStatus.PREEMPTED,
        )
        elapsed = 0.0
        activation_timeout = 10.0
        while elapsed < activation_timeout:
            status = self.get_current_status()
            if status == NavigationStatus.ACTIVE:
                break
            if status in _FAILURE_STATES:
                print(f"Navigation goal activation failed with status: {status}")
                return False
            time.sleep(0.01)
            elapsed += 0.01
        else:
            print(f"Timeout waiting for navigation goal activation, current status: {self.get_current_status()}")
            return False

        # Wait for navigation to complete, with timeout and failure-state handling.
        elapsed = 0.0
        navigation_timeout = 120.0
        while elapsed < navigation_timeout:
            status = self.get_current_status()
            if status == NavigationStatus.SUCCEEDED:
                return True
            if status in _FAILURE_STATES:
                print(f"Navigation failed with status: {status}")
                return False
            time.sleep(0.01)
            elapsed += 0.01

        print("Navigation timeout")
        return False

    def navigate_to_task_point(self, task_point_name: str) -> bool:
        """Navigate to the specified task point.

        Args:
            task_point_name (str): Name of the task point.

        Returns:
            bool: Whether navigation was successful.
        """
        if not self.robot_navigation.navigate_to_task_point(task_point_name):
            print(f"Navigate to task point service call failed for: {task_point_name}")
            return False

        # Wait for navigation to become ACTIVE, with timeout and failure-state handling.
        _FAILURE_STATES = (
            NavigationStatus.ABORTED,
            NavigationStatus.REJECTED,
            NavigationStatus.PREEMPTED,
        )
        elapsed = 0.0
        activation_timeout = 10.0
        while elapsed < activation_timeout:
            status = self.get_current_status()
            if status == NavigationStatus.ACTIVE:
                break
            if status in _FAILURE_STATES:
                print(f"Navigation task point activation failed with status: {status}")
                return False
            time.sleep(0.01)
            elapsed += 0.01
        else:
            print(f"Timeout waiting for navigation task point activation, current status: {self.get_current_status()}")
            return False

        # Wait for navigation to complete, with timeout and failure-state handling.
        elapsed = 0.0
        navigation_timeout = 120.0
        while elapsed < navigation_timeout:
            status = self.get_current_status()
            if status == NavigationStatus.SUCCEEDED:
                return True
            if status in _FAILURE_STATES:
                print(f"Navigation failed with status: {status}")
                return False
            time.sleep(0.01)
            elapsed += 0.01

        print("Navigation timeout")
        return False

    def stop_navigation(self) -> bool:
        """Stop navigation.

        Returns:
            bool: Whether stopping navigation was successful.
        """
        return self.robot_navigation.stop_navigation()

    def get_current_status(self) -> str:
        """Get current navigation status.

        Returns:
            str: Current navigation status.
        """
        return self.robot_navigation.get_current_status()

    def init_localization_by_pose(
        self, x: float, y: float, z: float, roll: float, pitch: float, yaw: float
    ) -> bool:
        """Initialize localization by pose.

        Args:
            x (float): Pose x coordinate.
            y (float): Pose y coordinate.
            z (float): Pose z coordinate.
            roll (float): Pose roll angle.
            pitch (float): Pose pitch angle.
            yaw (float): Pose yaw angle.

        Returns:
            bool: Whether localization initialization was successful.
        """
        orientation = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        pose = {
            "position": {"x": x, "y": y, "z": z},
            "orientation": {"x": orientation[0], "y": orientation[1], "z": orientation[2], "w": orientation[3]}
        }
        return self.robot_navigation.init_localization_by_pose(pose)

    def init_localization_by_task_point(
        self, task_point_name: str
    ) -> bool:
        """Initialize localization by task point.

        Args:
            task_point_name (str): Name of the task point.

        Returns:
            bool: Whether localization initialization was successful.
        """
        return self.robot_navigation.init_localization_by_task_point(task_point_name)

    def load_map(self, map_name: str) -> bool:
        """Load a map.

        Args:
            map_name (str): Name of the map.

        Returns:
            bool: Whether loading the map was successful.
        """
        return self.robot_navigation.load_map(map_name)

    def get_all_maps(self) -> list:
        """Get all map names.

        Returns:
            list: List of map names.
        """
        return self.robot_navigation.get_all_maps()

    def get_current_map(self) -> str:
        """Get current map name.

        Returns:
            str: Current map name.
        """
        return self.robot_navigation.get_current_map()

    def navigate_to_point_with_heading(self, x: float, y: float, heading: float) -> bool:
        """Navigate to the specified point with heading.

        Args:
            x (float): Target x coordinate (PNG pixel coordinate).
            y (float): Target y coordinate (PNG pixel coordinate).
            heading (float): Target heading angle (degrees), 0 = east, 90 = north.

        Returns:
            bool: Whether navigation was successful.
        """
        map_x, map_y = self._png_to_map(x, y)
        if map_x is None or map_y is None:
            print("PNG coordinate conversion failed, cannot execute navigation")
            return False

        print(f"PNG coordinates ({x}, {y}) converted to Map coordinates ({map_x}, {map_y})")

        yaw = math.radians(heading)

        return self.navigate_to_goal(x=map_x, y=map_y, z=0.0, roll=0.0, pitch=0.0, yaw=yaw)

    def _png_to_map(self, png_x: float, png_y: float, map_info=None):
        """PNG coordinates to Map coordinates conversion (no TF, direct conversion).

        Args:
            png_x: PNG coordinate X (pixels)
            png_y: PNG coordinate Y (pixels)

        Returns:
            tuple: (map_x, map_y) Map coordinates (meters)
        """
        try:
            # Subscribe to /map topic via WebSocket
            ws = WebSocketKuavoSDK()
            map_topic = roslibpy.Topic(ws.client, '/map', 'nav_msgs/OccupancyGrid')
            result = {}
            event = threading.Event()

            def _on_map(msg):
                result['msg'] = msg
                event.set()

            map_topic.subscribe(_on_map)
            if event.wait(timeout=5.0):
                msg = result['msg']
                map_topic.unsubscribe()
            else:
                map_topic.unsubscribe()
                print("Timeout waiting for /map topic")
                return None, None

            info = msg.get('info', {})
            map_info = {
                "width": info.get('width', 0),
                "height": info.get('height', 0),
                "resolution": info.get('resolution', 0.05),
                "origin": {
                    "x": info.get('origin', {}).get('position', {}).get('x', 0.0),
                    "y": info.get('origin', {}).get('position', {}).get('y', 0.0),
                    "z": info.get('origin', {}).get('position', {}).get('z', 0.0)
                }
            }

            resolution = map_info.get("resolution", 0.05)
            origin_x = map_info.get("origin", {}).get("x", 0.0)
            origin_y = map_info.get("origin", {}).get("y", 0.0)
            width = map_info.get("width", 0)
            height = map_info.get("height", 0)

            if resolution <= 0 or width <= 0 or height <= 0:
                print("Error: Invalid map parameters")
                return None, None

            map_x = origin_x + png_x * resolution
            map_y = origin_y + (height - png_y) * resolution

            return map_x, map_y

        except Exception as e:
            print(f"PNG to Map coordinate conversion failed: {e}")
            return None, None
