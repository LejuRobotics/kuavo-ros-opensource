#!/usr/bin/env python3
# coding: utf-8
from kuavo_humanoid_sdk.interfaces.data_types import (
    KuavoJointCommand, KuavoTwist)
from kuavo_humanoid_sdk.common.logger import SDKLogger
import rospy
from geometry_msgs.msg import Twist
from kuavo_humanoid_sdk.msg.kuavo_msgs.msg import jointCmd

class KuavoRobotObservationCore:
    _instance = None

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self):
        if not hasattr(self, '_initialized'):
            rospy.Subscriber("/joint_cmd", jointCmd, self._joint_cmd_callback)
            rospy.Subscriber("/cmd_vel", Twist, self._cmd_vel_callback)
            rospy.Subscriber("/cmd_pose", Twist, self._cmd_pose_callback)
            
            """ data """
            self._joint_cmd = KuavoJointCommand(
                joint_q = [0.0] * 28,
                joint_v = [0.0] * 28,
                tau = [0.0] * 28,
                tau_max = [0.0] * 28,
                tau_ratio = [0.0] * 28,
                joint_kp = [0.0] * 28,
                joint_kd = [0.0] * 28,
                control_modes = [0] * 28
            )

            self._cmd_vel = KuavoTwist(
                linear = (0.0, 0.0, 0.0),
                angular = (0.0, 0.0, 0.0)
            )

            self._cmd_pose = KuavoTwist(
                linear = (0.0, 0.0, 0.0),
                angular = (0.0, 0.0, 0.0)
            )

            self._initialized = True
            
    def _joint_cmd_callback(self, msg):
        self._joint_cmd.joint_q = list(msg.joint_q)
        self._joint_cmd.joint_v = list(msg.joint_v)
        self._joint_cmd.tau = list(msg.tau)
        self._joint_cmd.tau_max = list(msg.tau_max)
        self._joint_cmd.tau_ratio = list(msg.tau_ratio)
        self._joint_cmd.joint_kp = list(msg.joint_kp)
        self._joint_cmd.joint_kd = list(msg.joint_kd)
        self._joint_cmd.control_modes = list(msg.control_modes)
        
    def _cmd_vel_callback(self, msg):
        self._cmd_vel.linear = (msg.linear.x, msg.linear.y, msg.linear.z)
        self._cmd_vel.angular = (msg.angular.x, msg.angular.y, msg.angular.z)
        
    def _cmd_pose_callback(self, msg):
        self._cmd_pose.linear = (msg.linear.x, msg.linear.y, msg.linear.z)
        self._cmd_pose.angular = (msg.angular.x, msg.angular.y, msg.angular.z)
        
    @property
    def joint_command(self) -> KuavoJointCommand:
        return self._joint_cmd
        
    @property
    def cmd_vel(self) -> KuavoTwist:
        return self._cmd_vel
        
    @property
    def cmd_pose(self) -> KuavoTwist:
        return self._cmd_pose
    
    @property    
    def arm_position_command(self) -> list:
        """Return the position commands for the arm joints (indices 12-25).
        
        Returns:
            list: Position commands for arm joints
        """
        return self._joint_cmd.joint_q[12:26]
    
    @property
    def head_position_command(self) -> list:
        """Return the position commands for the head joints (indices 26-27).
        
        Returns:
            list: Position commands for head joints
        """
        return self._joint_cmd.joint_q[-2:]
