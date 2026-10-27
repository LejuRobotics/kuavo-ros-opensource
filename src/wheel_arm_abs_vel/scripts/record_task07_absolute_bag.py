#!/usr/bin/env python3
import legacy_dependencies

"""Disposable Task07 recorder for the absolute, non-MPC arm-control work."""

import argparse
import os
import sys

import rospy


TOPICS = [
    "/mm/teleop_tcp_target",
    "/leju_quest_bone_poses",
    "/quest_joystick_data",
    "/kuavo_arm_traj",
    "/ik_debug/absolute_enable/status",
    "/ik_debug/absolute_enable/captured_q",
    "/ik_debug/quest_tcp_mapping",
    "/ik_debug/arm_traj_receive/metadata",
    "/ik_debug/arm_traj_receive/receive_latency_ms",
    "/ik_debug/arm_traj_receive/receive_period_ms",
    "/ik_debug/arm_traj_receive/stamp_period_ms",
    "/ik_debug/arm_traj_receive/transport",
    "/ik_debug/arm_traj_receive/using_shm",
    "/humanoid/mpc/arm_control_mode",
    "/humanoid_wheel/arm_control_mode_status",
    "/humanoid_wheel/target_arm_joints_new",
    "/humanoid_wheel/arm_target_qpos_raw",
    "/humanoid_wheel/arm_target_qvel_raw",
    "/humanoid_wheel/arm_target_qpos_interp",
    "/humanoid_wheel/arm_target_qvel_interp",
    "/humanoid_wheel/interpolatedState",
    "/humanoid_wheel/optimizedState_mrt",
    "/humanoid_wheel/optimizedInput_mrt",
    "/humanoid_wheel/optimizedState_wbc_in",
    "/humanoid_wheel/optimizedInput_wbc_in",
    "/joint_cmd",
    "/sensors_data_raw",
    "/odom",
    "/tf",
    "/tf_static",
    "/humanoid_wheel/zarm_l7_end_effector_error_6d",
    "/humanoid_wheel/zarm_r7_end_effector_error_6d",
    "/velocity_ik_shadow/q",
    "/velocity_ik_shadow/reference",
    "/velocity_ik_shadow/dq",
    "/velocity_ik_shadow/ddq",
    "/velocity_ik_shadow/task_dq",
    "/velocity_ik_shadow/status",
    "/velocity_ik_shadow/task_error",
    "/velocity_ik_shadow/target_twist_ff",
    "/velocity_ik_shadow/input_debug",
    "/velocity_ik_shadow/feasibility_debug",
    "/whole_body_velocity_ik_shadow/q",
    "/whole_body_velocity_ik_shadow/reference",
    "/whole_body_velocity_ik_shadow/dq",
    "/whole_body_velocity_ik_shadow/ddq",
    "/whole_body_velocity_ik_shadow/task_dq",
    "/whole_body_velocity_ik_shadow/status",
    "/whole_body_velocity_ik_shadow/task_error",
    "/whole_body_velocity_ik_shadow/target_twist_ff",
    "/whole_body_velocity_ik_shadow/input_debug",
    "/whole_body_velocity_ik_shadow/feasibility_debug",
    "/whole_body_velocity_ik/reference",
    "/whole_body_velocity_ik/diagnostics/q",
    "/whole_body_velocity_ik/diagnostics/reference",
    "/whole_body_velocity_ik/diagnostics/dq",
    "/whole_body_velocity_ik/diagnostics/ddq",
    "/whole_body_velocity_ik/diagnostics/task_dq",
    "/whole_body_velocity_ik/diagnostics/status",
    "/whole_body_velocity_ik/diagnostics/task_error",
    "/whole_body_velocity_ik/diagnostics/target_twist_ff",
    "/whole_body_velocity_ik/diagnostics/execution_governor_v1",
    "/whole_body_velocity_ik/diagnostics/operating_diagnostics_v1",
    "/whole_body_velocity_ik/diagnostics/cycle_timing",
    "/whole_body_velocity_ik/diagnostics/input_debug",
    "/quest3/input_timing",
    "/whole_body_velocity_ik/diagnostics/feasibility_debug",
    "/monitor/time_cost/controller_loop_time",
    "/monitor/time_cost/wbc",
    "/rosout",
]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", required=True)
    # roslaunch 会附加 __name:=... 和 __log:=...；先过滤 ROS remap 参数。
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    output_dir = os.path.abspath(os.path.expanduser(args.output_dir))
    os.makedirs(output_dir, exist_ok=True)
    prefix = os.path.join(output_dir, "absolute_non_mpc_debug")

    command = [
        "rosbag",
        "record",
        "--buffsize=256",
        "--chunksize=768",
        "--output-prefix=" + prefix,
    ] + TOPICS
    os.execvp(command[0], command)


if __name__ == "__main__":
    main()
