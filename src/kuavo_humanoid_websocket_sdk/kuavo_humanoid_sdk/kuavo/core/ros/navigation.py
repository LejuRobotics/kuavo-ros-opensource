#!/usr/bin/env python3
# coding: utf-8
import time
from kuavo_humanoid_sdk.common.logger import SDKLogger
from kuavo_humanoid_sdk.common.websocket_kuavo_sdk import WebSocketKuavoSDK
import roslibpy
from enum import Enum


class NavigationStatus(Enum):
    """Navigation status."""
    PENDING = 0
    ACTIVE = 1
    PREEMPTED = 2
    SUCCEEDED = 3
    ABORTED = 4
    REJECTED = 5


class Navigation:
    """WebSocket-based navigation system interface for controlling navigation functionality."""

    def __init__(self):
        """Initialize the navigation system via WebSocket."""
        websocket = WebSocketKuavoSDK()

        self._init_with_pose_pub = roslibpy.Topic(
            websocket.client, '/initialpose', 'geometry_msgs/PoseWithCovarianceStamped')
        self._init_with_task_point_pub = roslibpy.Topic(
            websocket.client, 'initialpose_with_taskpoint', 'std_msgs/String')
        self._goal_pub = roslibpy.Topic(
            websocket.client, '/move_base_simple/goal', 'geometry_msgs/PoseStamped')
        self._move_base_cancel_pub = roslibpy.Topic(
            websocket.client, '/move_base/cancel', 'actionlib_msgs/GoalID')

        self._status_sub = roslibpy.Topic(
            websocket.client, '/move_base/status', 'actionlib_msgs/GoalStatusArray')
        self._status_sub.subscribe(self._status_callback)

        self.status = NavigationStatus.PENDING
        self._goal_id = None

    def _status_callback(self, msg: dict):
        """Callback for the status topic."""
        status_list = msg.get('status_list', [])
        if len(status_list) > 0:
            status_value = status_list[0].get('status', 0)
            self.status = NavigationStatus(status_value)
            if 'goal_id' in status_list[0]:
                self._goal_id = status_list[0]['goal_id'].get('id', None)

    def srv_load_map(self, map_name: str):
        """Load the map."""
        try:
            websocket = WebSocketKuavoSDK()
            service = roslibpy.Service(websocket.client, '/load_map', 'kuavo_msgs/LoadMap')
            req = {"map_name": map_name}
            res = service.call(req)
            return res.get('success', False)
        except Exception as e:
            SDKLogger.error(f"Service `load_map` call failed: {e}")
            return False

    def srv_get_current_map(self):
        """Get the current map."""
        try:
            websocket = WebSocketKuavoSDK()
            service = roslibpy.Service(websocket.client, 'get_current_map', 'kuavo_msgs/GetCurrentMap')
            req = {}
            res = service.call(req)
            return res.get('current_map', None)
        except Exception as e:
            SDKLogger.error(f"Service `get_current_map` call failed: {e}")
            return None

    def srv_get_all_maps(self):
        """Get all maps."""
        try:
            websocket = WebSocketKuavoSDK()
            service = roslibpy.Service(websocket.client, 'get_all_maps', 'kuavo_msgs/GetAllMaps')
            req = {}
            res = service.call(req)
            return res.get('maps', None)
        except Exception as e:
            SDKLogger.error(f"Service `get_all_maps` call failed: {e}")
            return None

    def srv_navigate_to_task_point(self, task_point_name: str):
        """Navigate to the task point."""
        try:
            websocket = WebSocketKuavoSDK()
            service = roslibpy.Service(websocket.client, 'navigate_to_task_point', 'kuavo_msgs/NavigateToTaskPoint')
            req = {"task_name": task_point_name}
            res = service.call(req)
            return res.get('success', False)
        except Exception as e:
            SDKLogger.error(f"Service `navigate_to_task_point` call failed: {e}")
            return False

    def srv_init_localization_by_task_point(self, task_point_name: str):
        """Initialize the localization by task point."""
        try:
            websocket = WebSocketKuavoSDK()
            service = roslibpy.Service(websocket.client, '/initialpose_with_taskpoint', 'kuavo_msgs/InitialPoseWithTaskPoint')
            req = {"task_point_name": task_point_name}
            res = service.call(req)
            return res.get('success', False)
        except Exception as e:
            SDKLogger.error(f"Service `/initialpose_with_taskpoint` call failed: {e}")
            return False

    def pub_init_localization_by_pose(self, pose: dict):
        """Initialize the localization by pose.

        Args:
            pose: dict with 'position' and 'orientation' keys
        """
        try:
            t = time.time()
            initialpose_msg = {
                "header": {
                    "frame_id": "map",
                    "stamp": {"secs": int(t), "nsecs": int((t % 1) * 1e9)}
                },
                "pose": {
                    "pose": pose,
                    "covariance": [0.0] * 36
                }
            }
            self._init_with_pose_pub.publish(roslibpy.Message(initialpose_msg))
        except Exception as e:
            SDKLogger.error(f"Failed to initialize localization by pose: {e}")
            return False
        return True

    def pub_init_localization_by_task_point(self, task_point_name: str):
        """Initialize the localization by task point."""
        try:
            msg = {"data": task_point_name}
            self._init_with_task_point_pub.publish(roslibpy.Message(msg))
            time.sleep(3)
        except Exception as e:
            SDKLogger.error(f"Failed to initialize localization by task point: {e}")
            return False
        return True

    def pub_navigate_to_goal(self, goal: dict):
        """Navigate to the goal.

        Args:
            goal: dict with 'position' and 'orientation' keys
        """
        try:
            t = time.time()
            goal_msg = {
                "header": {
                    "frame_id": "map",
                    "stamp": {"secs": int(t), "nsecs": int((t % 1) * 1e9)}
                },
                "pose": goal
            }
            self._goal_pub.publish(roslibpy.Message(goal_msg))
        except Exception as e:
            SDKLogger.error(f"Failed to navigate to goal: {e}")
            return False
        return True

    def get_current_status(self):
        """Get the current navigation status."""
        return self.status

    def pub_stop_navigation(self):
        """Stop the navigation. Cancel all goals."""
        try:
            cancel_msg = {
                "goal_id": {"id": self._goal_id or "", "stamp": {"secs": 0, "nsecs": 0}}
            }
            self._move_base_cancel_pub.publish(roslibpy.Message(cancel_msg))
        except Exception as e:
            SDKLogger.error(f"Failed to stop navigation: {e}")
            return False
        return True
