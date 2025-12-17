#!/bin/bash
rosservice call /mobile_manipulator_mpc_control "control_mode: 1"
rosservice call /humanoid_change_arm_ctrl_mode "control_mode: 2"
rosservice call /enable_mm_wbc_arm_trajectory_control "control_mode: 1"