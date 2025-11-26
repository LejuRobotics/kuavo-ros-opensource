#!/usr/bin/env python3
# coding: utf-8
import math
from kuavo_humanoid_sdk.kuavo.core.core import KuavoRobotCore
from kuavo_humanoid_sdk.common.logger import SDKLogger


class KuavoRobotWaist:
    """
    Control the waist of the robot.

    The waist has 1 DOF: yaw (rotation around Z-axis).
    The value is published to the topic `/robot_waist_motion_data`
    as a Float64MultiArray: [yaw_degrees]
    """

    def __init__(self):
        self._kuavo_core = KuavoRobotCore()

        # Waist angle limit in degrees (recommended: -180° to +180°)
        self.WAIST_LIMIT_DEG = 180

    def control_waist(self, yaw: float) -> bool:
        """
        Control the waist of the robot.

        Args:
            yaw (float): The yaw angle of the waist in **degrees**.
                         Valid range: [-180, 180] degrees.
                         Values outside this range will be automatically limited.

        Returns:
            bool: True if the waist is controlled successfully, False otherwise.
        """

        # Normalize input: must be float
        if not isinstance(yaw, (int, float)):
            raise TypeError("yaw must be a float value representing degrees.")

        # --- Check yaw limit (degrees) ---
        if yaw < -self.WAIST_LIMIT_DEG or yaw > self.WAIST_LIMIT_DEG:
            SDKLogger.warn(
                f"[Robot] waist yaw {yaw}° exceeds limit "
                f"[{-self.WAIST_LIMIT_DEG}, {self.WAIST_LIMIT_DEG}]°, will be limited"
            )

        # Apply limits
        limited_yaw = min(self.WAIST_LIMIT_DEG, max(-self.WAIST_LIMIT_DEG, yaw))

        # Convert to list format expected by lower-level interface
        yaw_list = [limited_yaw]

        return self._kuavo_core.control_robot_waist(yaw_list)
