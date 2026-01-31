#!/usr/bin/env python3
# coding: utf-8
import os
import json
import rospy
import rospkg
import numpy as np
from typing import List, Tuple, Optional, Union
from scipy.interpolate import PchipInterpolator, UnivariateSpline
from kuavo_msgs.msg import footPose, footPoseTargetTrajectories, footPoses
from kuavo_msgs.msg import footPose6D, footPose6DTargetTrajectories, footPoses6D
from std_srvs.srv import SetBool, SetBoolRequest
from kuavo_msgs.srv import stairAlignmentSrv, stairAlignmentSrvRequest


# Constants for magic numbers
class StairClimbingConstants:
    DEFAULT_DT = 1.0  # 上下楼梯的步态周期 (aligned with continuousStairClimber-roban.py)
    DEFAULT_SS_TIME = 0.6  # 上下楼梯的支撑迈步时间 (aligned with continuousStairClimber-roban.py)
    DEFAULT_FOOT_WIDTH = 0.108535  # 脚宽度 (aligned with continuousStairClimber-roban.py)
    DEFAULT_STEP_HEIGHT = 0.08  # 台阶高度 (aligned with continuousStairClimber-roban.py)
    DEFAULT_STEP_LENGTH = 0.25  # 上楼梯的台阶长度 (aligned with continuousStairClimber-roban.py)
    DEFAULT_MAX_STEP_X = 0.28  # Default max step in X direction
    DEFAULT_MAX_STEP_Y = 0.15  # Default max step in Y direction
    DEFAULT_MAX_STEP_YAW = 30.0  # Default max yaw step (degrees)
    DEFAULT_SWING_HEIGHT = 0.10  # Default swing phase height
    DEFAULT_SWING_POINTS = 7  # Default swing trajectory points
    TORSO_HEIGHT_OFFSET = -0.02  # 躯干高度偏移 (aligned with continuousStairClimber-roban.py)
    WALK_DT = 0.6  # 前进/转弯的步态周期 (aligned with continuousStairClimber-roban.py)
    WALK_SS_TIME = 0.4  # 前进/转弯的支撑迈步时间 (aligned with continuousStairClimber-roban.py)
    DOWN_STAIRS_SS_TIME = 0.35  # Down stairs single support time


def set_pitch_limit(enable: bool) -> bool:
    """
    Set base pitch angle limit
    Args:
        enable: bool, True to enable limit, False to disable limit
    Returns:
        bool: Whether the operation was successful
    """
    print(f"call set_pitch_limit:{enable}")
    rospy.wait_for_service("/humanoid/mpc/enable_base_pitch_limit")
    try:
        set_pitch_limit_service = rospy.ServiceProxy(
            "/humanoid/mpc/enable_base_pitch_limit", SetBool
        )
        req = SetBoolRequest()
        req.data = enable
        resp = set_pitch_limit_service(req)
        if resp.success:
            rospy.loginfo(
                f"Successfully {'enabled' if enable else 'disabled'} pitch limit"
            )
        else:
            rospy.logwarn(f"Failed to {'enable' if enable else 'disable'} pitch limit")
        return resp.success
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False


def publish_foot_pose_traj(
    time_traj: List[float],
    foot_idx_traj: List[int],
    foot_traj: List[List[float]],
    torso_traj: List[List[float]],
    swing_trajectories: Optional[List] = None,
    verbose: bool = True,
) -> None:
    """
    Publish foot pose trajectory
    Args:
        time_traj: Time trajectory
        foot_idx_traj: Foot index trajectory
        foot_traj: Foot trajectory
        torso_traj: Torso trajectory
        swing_trajectories: Swing phase trajectories (optional)
        verbose: Whether to enable detailed logging
    """
    num_points = len(time_traj)

    if verbose:
        rospy.loginfo(f"[ClimbStair] Publishing trajectory with {num_points} points")
        rospy.logdebug(f"[ClimbStair] Time trajectory: {time_traj}")
        rospy.logdebug(f"[ClimbStair] Foot index trajectory: {foot_idx_traj}")

        # Log first few trajectory points for debugging
        log_count = min(3, num_points)  # Reduced from 5 to 3
        for i in range(log_count):
            rospy.logdebug(
                f"[ClimbStair] Point {i}: time={time_traj[i]:.3f}, foot_idx={foot_idx_traj[i]}, "
                f"foot=[{foot_traj[i][0]:.3f}, {foot_traj[i][1]:.3f}, {foot_traj[i][2]:.3f}, {foot_traj[i][3]:.3f}], "
                f"torso=[{torso_traj[i][0]:.3f}, {torso_traj[i][1]:.3f}, {torso_traj[i][2]:.3f}, {torso_traj[i][3]:.3f}]"
            )

        if num_points > log_count:
            rospy.logdebug(
                f"[ClimbStair] ... (showing first {log_count} of {num_points} points)"
            )

    # Create publisher with appropriate queue size
    pub = rospy.Publisher(
        "/humanoid_mpc_foot_pose_target_trajectories",
        footPoseTargetTrajectories,
        queue_size=1,
        latch=True,
    )
    rospy.sleep(0.5)  # Reduced sleep time

    # Build message
    msg = footPoseTargetTrajectories()
    msg.timeTrajectory = time_traj
    msg.footIndexTrajectory = foot_idx_traj
    msg.footPoseTrajectory = []
    msg.additionalFootPoseTrajectory = []

    # Pre-allocate lists for better performance
    msg.footPoseTrajectory = [footPose() for _ in range(num_points)]
    msg.additionalFootPoseTrajectory = [footPoses() for _ in range(num_points)]

    for i in range(num_points):
        msg.footPoseTrajectory[i].footPose = foot_traj[i]
        msg.footPoseTrajectory[i].torsoPose = torso_traj[i]

        # Handle swing trajectories efficiently
        if (
            swing_trajectories is not None
            and i < len(swing_trajectories)
            and swing_trajectories[i] is not None
        ):
            msg.additionalFootPoseTrajectory[i] = swing_trajectories[i]
            if verbose:
                swing_points = (
                    len(swing_trajectories[i].data)
                    if hasattr(swing_trajectories[i], "data")
                    else 0
                )
                rospy.logdebug(
                    f"[ClimbStair] Point {i}: Adding swing trajectory with {swing_points} points"
                )

    if verbose:
                    rospy.loginfo(
                "[ClimbStair] Publishing to /humanoid_mpc_foot_pose_target_trajectories"
            )

    pub.publish(msg)

    if verbose:
        rospy.loginfo("[ClimbStair] Trajectory published successfully")

    rospy.sleep(1.0)  # Reduced sleep time


class KuavoRobotClimbStair:
    """Kuavo robot stair climbing implementation with SDK interface

    Merged with EnhancedSlopeController for 6D trajectory support.
    """

    def __init__(
        self,
        stand_height: float = 0.0,
        verbose_logging: bool = True,
    ):
        """
        Initialize the stair climbing system.

        Args:
            stand_height: Standing height offset
            verbose_logging: Whether to enable verbose logging
        """

        # Use constants for parameters (aligned with continuousStairClimber-roban.py)
        self.dt = StairClimbingConstants.DEFAULT_DT
        self.ss_time = StairClimbingConstants.DEFAULT_SS_TIME
        self.foot_width = StairClimbingConstants.DEFAULT_FOOT_WIDTH
        self.step_height = StairClimbingConstants.DEFAULT_STEP_HEIGHT
        self.step_length = StairClimbingConstants.DEFAULT_STEP_LENGTH
        self.down_step_length = 0.25  # 下楼梯的迈步距离（独立参数）
        self.up_stairs_double_step_offset = 0.00
        self.down_stairs_double_step_offset = -0.00
        self.temp_x_offset = 0.002  # 临时x方向偏置，每步叠加
        self.walk_dt = StairClimbingConstants.WALK_DT  # 前进/转弯的步态周期
        self.walk_ss_time = StairClimbingConstants.WALK_SS_TIME  # 前进/转弯的支撑迈步时间
        self.total_step = 0
        self.is_left_foot = False  # 当前是否为左脚 (aligned with continuousStairClimber-roban.py)

        # Global variables from original script
        self.PLOT = False
        self.STAND_HEIGHT = stand_height
        self.verbose_logging = verbose_logging

        # Trajectory accumulation for continuous planning
        self._clear_trajectory_data()

        # Pre-compute commonly used values
        self._rotation_matrices_cache = {}

        # === EnhancedSlopeController attributes (merged) ===
        # 全局高度管理
        self.swing_height = 0.12
        self.torso_height = 0.0

        # 楼梯参数 (用于6D轨迹)
        self.stair_height = 0.0
        self.stair_length = 0.0


        rospy.loginfo(
            "[ClimbStair] Initialized with stand_height=%.3f, verbose=%s",
            stand_height,
            verbose_logging,
        )

    def set_stair_parameters(
        self,
        step_height: float = None,
        step_length: float = None,
        foot_width: float = None,
        stand_height: float = None,
        dt: float = None,
        ss_time: float = None,
    ) -> bool:
        """
        Set stair climbing parameters.

        Args:
            step_height: Step height (m), must be > 0
            step_length: Step length (m), must be > 0
            foot_width: Foot width (m), must be > 0
            stand_height: Standing height offset (m)
            dt: Gait cycle time (s), must be > 0
            ss_time: Single support time ratio, must be between 0 and 1

        Returns:
            bool: Whether parameter setting was successful
        """
        # Use current values as defaults if None provided
        step_height = step_height if step_height is not None else self.step_height
        step_length = step_length if step_length is not None else self.step_length
        foot_width = foot_width if foot_width is not None else self.foot_width
        stand_height = stand_height if stand_height is not None else self.STAND_HEIGHT
        dt = dt if dt is not None else self.dt
        ss_time = ss_time if ss_time is not None else self.ss_time

        # Input validation
        if step_height <= 0 or step_length <= 0 or foot_width <= 0:
            rospy.logerr(
                "[ClimbStair] Invalid parameters: step_height, step_length, foot_width must be positive"
            )
            return False

        if dt <= 0:
            rospy.logerr("[ClimbStair] Invalid dt: must be positive")
            return False

        if not (0 < ss_time < 1):
            rospy.logerr("[ClimbStair] Invalid ss_time: must be between 0 and 1")
            return False

        if step_height > 0.5:  # Reasonable safety limit
            rospy.logwarn(
                "[ClimbStair] Step height %.3f seems very high, consider checking",
                step_height,
            )

        if step_length > 1.0:  # Reasonable safety limit
            rospy.logwarn(
                "[ClimbStair] Step length %.3f seems very long, consider checking",
                step_length,
            )

        # Clear rotation matrix cache as foot_width affects calculations
        self._rotation_matrices_cache.clear()

        self.step_height = step_height
        self.step_length = step_length
        self.foot_width = foot_width
        self.STAND_HEIGHT = stand_height
        self.dt = dt
        self.ss_time = ss_time

        if self.verbose_logging:
            rospy.loginfo(
                f"[ClimbStair] Parameters updated - step_height: {step_height:.3f}, "
                f"step_length: {step_length:.3f}, foot_width: {foot_width:.3f}, "
                f"stand_height: {stand_height:.3f}, dt: {dt:.3f}, ss_time: {ss_time:.3f}"
            )
        return True

    def set_gait_parameters(self, dt: float = None, ss_time: float = None) -> bool:
        """
        Set gait timing parameters.

        Args:
            dt: Gait cycle time (s), must be > 0. Default is 0.6s for stair climbing
            ss_time: Single support time ratio, must be between 0 and 1. Default is 0.5

        Returns:
            bool: Whether parameter setting was successful
        """
        # Use current values as defaults if None provided
        dt = dt if dt is not None else self.dt
        ss_time = ss_time if ss_time is not None else self.ss_time

        # Input validation
        if dt <= 0:
            rospy.logerr("[ClimbStair] Invalid dt: must be positive")
            return False

        if not (0 < ss_time < 1):
            rospy.logerr("[ClimbStair] Invalid ss_time: must be between 0 and 1")
            return False

        # Safety warnings for extreme values
        if dt < 0.2:
            rospy.logwarn(
                "[ClimbStair] Very fast gait cycle (dt=%.3f), may cause instability", dt
            )
        elif dt > 2.0:
            rospy.logwarn(
                "[ClimbStair] Very slow gait cycle (dt=%.3f), consider checking", dt
            )

        if ss_time < 0.3:
            rospy.logwarn(
                "[ClimbStair] Very short single support time (%.3f), may cause instability",
                ss_time,
            )
        elif ss_time > 0.8:
            rospy.logwarn(
                "[ClimbStair] Very long single support time (%.3f), may cause instability",
                ss_time,
            )

        self.dt = dt
        self.ss_time = ss_time

        if self.verbose_logging:
            rospy.loginfo(
                f"[ClimbStair] Gait parameters updated - dt: {dt:.3f}s, ss_time: {ss_time:.3f}"
            )
        return True

    def get_parameters(self) -> dict:
        """
        Get current stair climbing and gait parameters.

        Returns:
            dict: Dictionary containing all current parameters
        """
        return {
            "step_height": self.step_height,
            "step_length": self.step_length,
            "foot_width": self.foot_width,
            "stand_height": self.STAND_HEIGHT,
            "dt": self.dt,
            "ss_time": self.ss_time,
            "down_step_length": self.down_step_length,
            "up_stairs_double_step_offset": self.up_stairs_double_step_offset,
            "down_stairs_double_step_offset": self.down_stairs_double_step_offset,
            "temp_x_offset": self.temp_x_offset,
            "walk_dt": self.walk_dt,
            "walk_ss_time": self.walk_ss_time,
        }

    def _clear_trajectory_data(self) -> None:
        """Internal method to clear trajectory data."""
        self.time_traj = []
        self.foot_idx_traj = []
        self.foot_traj = []
        self.torso_traj = []
        self.swing_trajectories = []

    # === EnhancedSlopeController methods (merged) ===
    def plan_swing_phase_6d(self, prev_foot_pose, next_foot_pose, num_points):
        """
        使用形状保持的三次样条插值规划腾空相的6D轨迹
        Args:
            prev_foot_pose: 上一个落点位置 [x, y, z, yaw]
            next_foot_pose: 下一个落点位置 [x, y, z, yaw]
            num_points: 轨迹点数
        Returns:
            additionalFootPoseTrajectory: 包含腾空相轨迹的footPoses6D消息
        """
        additionalFootPoseTrajectory = footPoses6D()

        # 创建时间序列
        t = np.linspace(0, 1, num_points)

        # 计算x和y方向的移动距离
        x_distance = next_foot_pose[0] - prev_foot_pose[0]
        y_distance = next_foot_pose[1] - prev_foot_pose[1]
        z_distance = next_foot_pose[2] - prev_foot_pose[2]

        # 创建控制点
        control_points = {
            't': [0, 0.25, 0.5, 0.75, 1.0],
            'x': [
                prev_foot_pose[0] + x_distance * 0.0,
                prev_foot_pose[0] + x_distance * 0.25,
                prev_foot_pose[0] + x_distance * 0.5,
                prev_foot_pose[0] + x_distance * 0.75,
                prev_foot_pose[0] + x_distance * 1.0,
            ],
            'z': [
                prev_foot_pose[2] + z_distance * 0.0,
                next_foot_pose[2] + self.swing_height * 0.5,
                next_foot_pose[2] + self.swing_height,
                next_foot_pose[2] + self.swing_height * 0.5,
                next_foot_pose[2],
            ]
        }

        # 为x和z创建形状保持的三次样条插值
        x_spline = PchipInterpolator(control_points['t'], control_points['x'])
        z_spline = PchipInterpolator(control_points['t'], control_points['z'])

        # 生成轨迹点
        for i in range(num_points):
            step_fp = footPose6D()
            x = float(x_spline(t[i]))
            z = float(z_spline(t[i]))

            step_fp.footPose6D = [x, next_foot_pose[1], z, next_foot_pose[3], 0.0, 0.0]
            additionalFootPoseTrajectory.data.append(step_fp)

        if len(additionalFootPoseTrajectory.data) >= 2:
            additionalFootPoseTrajectory.data.pop(0)  # 删除第一个点
            additionalFootPoseTrajectory.data.pop(-1)  # 删除最后一个点

        return additionalFootPoseTrajectory

    def plan_swing_phase_by_stair_6d(self, prev_foot_pose, next_foot_pose, num_points):
        """
        使用形状保持的三次样条插值规划楼梯腾空相的6D轨迹
        Args:
            prev_foot_pose: 上一个落点位置 [x, y, z, yaw]
            next_foot_pose: 下一个落点位置 [x, y, z, yaw]
            num_points: 轨迹点数
        Returns:
            additionalFootPoseTrajectory_stair: 包含腾空相轨迹的footPoses6D消息
        """
        additionalFootPoseTrajectory_stair = footPoses6D()

        # 创建时间序列
        t = np.linspace(0, 1, num_points)

        # 根据落足点高度差计算台阶数量
        stairNum = int(np.round((next_foot_pose[2] - prev_foot_pose[2]) / self.stair_height)) if self.stair_height > 0 else 1
        print("stairNum: ", stairNum)

        # 一阶段最高点的scale
        phase1_height_scale = 1.6
        # 计算第二阶段起点和终点的z值
        phase2_start_z = prev_foot_pose[2] + self.stair_height * phase1_height_scale
        phase2_end_z = prev_foot_pose[2] + self.stair_height * stairNum * 1.0 + self.swing_height * 1.0
        phase2_num = 3

        control_points = {
            't': [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0],
            'x': [
                prev_foot_pose[0] + self.stair_length * stairNum * 0.0,
                prev_foot_pose[0] + self.stair_length * stairNum * 0.0,
                prev_foot_pose[0] + self.stair_length * stairNum * 0.0,
                prev_foot_pose[0] + self.stair_length * stairNum * 0.0,
                prev_foot_pose[0] + self.stair_length * stairNum * 0.0,
                prev_foot_pose[0] + self.stair_length * stairNum * 0.17,
                prev_foot_pose[0] + self.stair_length * stairNum * 0.33,
                prev_foot_pose[0] + self.stair_length * stairNum * 0.5,
                prev_foot_pose[0] + self.stair_length * stairNum * 0.67,
                prev_foot_pose[0] + self.stair_length * stairNum * 0.83,
                prev_foot_pose[0] + self.stair_length * stairNum * 1.0,
            ],
            'z': [
                prev_foot_pose[2] + self.stair_height * 0.0,
                prev_foot_pose[2] + self.stair_height * phase1_height_scale / 6,
                prev_foot_pose[2] + self.stair_height * phase1_height_scale / 2,
                prev_foot_pose[2] + self.stair_height * phase1_height_scale / 4 * 3,
                prev_foot_pose[2] + self.stair_height * phase1_height_scale,
                phase2_end_z * 1/phase2_num + phase2_start_z * (1-1/phase2_num),
                phase2_end_z * 2/phase2_num + phase2_start_z * (1-2/phase2_num),
                phase2_end_z * 3/phase2_num,
                prev_foot_pose[2] + self.stair_height * stairNum * 1.0 + self.swing_height * 0.5,
                prev_foot_pose[2] + self.stair_height * stairNum * 1.0 + self.swing_height * 0.2,
                prev_foot_pose[2] + self.stair_height * stairNum * 1.0,
            ]
        }

        x_spline = UnivariateSpline(control_points['t'], control_points['x'], s=0.1, k=3)
        z_spline = UnivariateSpline(control_points['t'], control_points['z'], s=0.1, k=3)

        # 生成轨迹点
        for i in range(num_points):
            step_fp = footPose6D()
            x = float(x_spline(t[i]))
            z = float(z_spline(t[i]))

            step_fp.footPose6D = [x, next_foot_pose[1], z, next_foot_pose[3], 0.0, 0.0]
            additionalFootPoseTrajectory_stair.data.append(step_fp)

        if len(additionalFootPoseTrajectory_stair.data) >= 2:
            additionalFootPoseTrajectory_stair.data.pop(0)  # 删除第一个点
            additionalFootPoseTrajectory_stair.data.pop(-1)  # 删除最后一个点

        return additionalFootPoseTrajectory_stair

    def simple_up_stairs_6d(self, stair_height, stair_length, stair_num, distance_initial):
        """完整的楼梯6D轨迹生成函数

        Args:
            stair_height: 单级楼梯高度
            stair_length: 单级楼梯长度（深度）
            stair_num: 楼梯级数（需要迈步的次数）
            distance_initial: 脚掌距离斜坡的初始距离
        """
        # 保存楼梯参数供swing规划使用
        self.stair_height = stair_height
        self.stair_length = stair_length

        # 两脚之间的宽度
        foot_width = 0.1175175

        # 脚掌距离斜坡的初始距离
        foot_distance_initial = distance_initial

        # 脚末端到脚掌中心的 x 方向距离
        foot_offset = 0.0185
        foot_toe = 0.121

        # 数据结构初始化
        time_traj = []
        foot_idx_traj = []
        foot_poses_6d = []
        torso_poses_6d = []
        swing_trajectories_array = []

        # 迈步时间间隔
        swing_time = 0.8
        contact_time = 1.2

        # 除第一步外的步进距离
        step_x_increment = stair_length
        step_z_increment = stair_height

        # 躯干位置偏置
        torso_offset_x = -0.03
        torso_offset_z = 0.08

        # 生成每一步的轨迹
        step_x_init = foot_distance_initial + foot_toe + stair_length/2 - foot_offset
        step_z_init = stair_height
        torso_x_init = step_x_init + torso_offset_x
        torso_z_init = stair_height - torso_offset_z
        step_swing_time = swing_time

        # 存储上一摆动相的足端高度
        last_step_x = step_x_init
        last_step_z = step_z_init
        last_step_x_left = last_step_x
        last_step_z_left = last_step_z
        last_step_x_right = 0
        last_step_z_right = 0

        # 存储当前躯干位置
        current_torso_x = 0
        current_torso_z = 0

        # 添加最后的并拢步
        total_phases = stair_num * 2 + 1

        for step in range(total_phases):
            if step < stair_num * 2:
                if step % 2 == 0:  # 摆动相
                    step_index = step // 2
                    foot_index = step_index % 2

                    if step_index == 0:
                        time_traj.append(step_swing_time)
                        foot_idx_traj.append(foot_index)
                        foot_poses_6d.append([
                            step_x_init, foot_width, step_z_init, 0.0, 0.0, 0.0
                        ])
                        torso_poses_6d.append([
                            current_torso_x, 0.0, current_torso_z, 0.0, 0.0, 0.0
                        ])
                        y_pos = foot_width if foot_index == 0 else -foot_width
                        prev_foot = [0.0, y_pos, 0.0, 0.0]
                        new_foot = [step_x_init, y_pos, step_z_init, 0.0]
                        FootPoseTrajectory = self.plan_swing_phase_by_stair_6d(prev_foot, new_foot, 8)
                        swing_trajectories_array.append(FootPoseTrajectory)
                        current_torso_x += torso_x_init
                        current_torso_z += torso_z_init
                    else:
                        time_traj.append(step_swing_time)
                        foot_idx_traj.append(foot_index)

                        step_x_new = step_x_init + step_x_increment * step_index
                        step_z_new = step_z_init + step_z_increment * step_index

                        y_pos = foot_width if foot_index == 0 else -foot_width

                        foot_poses_6d.append([
                            step_x_new, y_pos, step_z_new, 0.0, 0.0, 0.0
                        ])
                        torso_poses_6d.append([
                            current_torso_x, 0.0, current_torso_z, 0.0, 0.0, 0.0
                        ])

                        if foot_index == 0:
                            last_step_x = last_step_x_left
                            last_step_z = last_step_z_left
                        else:
                            last_step_x = last_step_x_right
                            last_step_z = last_step_z_right

                        prev_foot = [last_step_x, y_pos, last_step_z, 0.0]
                        new_foot = [step_x_new, y_pos, step_z_new, 0.0]
                        FootPoseTrajectory = self.plan_swing_phase_by_stair_6d(prev_foot, new_foot, 8)
                        swing_trajectories_array.append(FootPoseTrajectory)
                        current_torso_x = torso_x_init + step_x_increment * step_index
                        current_torso_z = torso_z_init + step_z_increment * step_index

                        last_step_z = step_z_new
                        last_step_x = step_x_new

                        if foot_index == 0:
                            last_step_x_left = last_step_x
                            last_step_z_left = last_step_z
                        else:
                            last_step_x_right = last_step_x
                            last_step_z_right = last_step_z

                    step_swing_time += contact_time

                else:  # 接触相
                    step_index = (step - 1) // 2
                    time_traj.append(step_swing_time)
                    foot_idx_traj.append(2)

                    if step_index == 0:
                        foot_poses_6d.append([
                            step_x_init, 0.0, step_z_init, 0.0, 0.0, 0.0
                        ])
                        torso_poses_6d.append([
                            current_torso_x, 0.0, current_torso_z, 0.0, 0.0, 0.0
                        ])
                        swing_trajectories_array.append(footPoses6D())
                    else:
                        step_x_current = last_step_x
                        step_z_current = last_step_z

                        foot_poses_6d.append([
                            step_x_current, 0.0, step_z_current, 0.0, 0.0, 0.0
                        ])
                        torso_poses_6d.append([
                            current_torso_x, 0.0, current_torso_z, 0.0, 0.0, 0.0
                        ])
                        swing_trajectories_array.append(footPoses6D())
                    step_swing_time += swing_time
            else:
                step_index = step // 2
                foot_index = step_index % 2
                y_pos = foot_width if foot_index == 0 else -foot_width

                step_x_last = step_x_init + step_x_increment * (stair_num-1)
                step_z_last = step_z_init + step_z_increment * (stair_num-1)
                torso_x_last = torso_x_init + step_x_increment * (stair_num-1)
                torso_z_last = torso_z_init + step_z_increment * (stair_num-1) + torso_offset_z

                time_traj.append(step_swing_time)
                foot_idx_traj.append(foot_index)
                foot_poses_6d.append([
                    step_x_last, y_pos, step_z_last, 0.0, 0.0, 0.0
                ])
                torso_poses_6d.append([
                    torso_x_last, 0.0, torso_z_last, 0.0, 0.0, 0.0
                ])

                if foot_index == 0:
                    last_step_x = last_step_x_left
                    last_step_z = last_step_z_left
                else:
                    last_step_x = last_step_x_right
                    last_step_z = last_step_z_right
                prev_foot = [last_step_x, y_pos, last_step_z, 0.0]
                new_foot = [step_x_last, y_pos, step_z_last, 0.0]
                FootPoseTrajectory = self.plan_swing_phase_by_stair_6d(prev_foot, new_foot, 8)
                swing_trajectories_array.append(FootPoseTrajectory)

        return self.get_foot_pose_6d_traj_msg(time_traj, foot_idx_traj, foot_poses_6d, torso_poses_6d, swing_trajectories_array)

    def get_foot_pose_6d_traj_msg(self, time_traj, foot_idx_traj, foot_traj_6d, torso_traj_6d, swing_trajectories=None):
        """
        创建6D足部姿态目标轨迹消息

        Args:
            time_traj: 时间轨迹列表
            foot_idx_traj: 足部索引轨迹列表
            foot_traj_6d: 6D足部姿态轨迹列表，每个元素为[x, y, z, yaw, pitch, roll]
            torso_traj_6d: 6D躯干姿态轨迹列表，每个元素为[x, y, z, yaw, pitch, roll]
            swing_trajectories: 摆动轨迹列表（可选）

        Returns:
            footPose6DTargetTrajectories消息
        """
        num = len(time_traj)

        msg = footPose6DTargetTrajectories()
        msg.timeTrajectory = time_traj
        msg.footIndexTrajectory = foot_idx_traj
        msg.footPoseTrajectory = []

        for i in range(num):
            foot_pose_msg = footPose6D()
            foot_pose_msg.footPose6D = foot_traj_6d[i]
            foot_pose_msg.torsoPose6D = torso_traj_6d[i]

            msg.footPoseTrajectory.append(foot_pose_msg)

            if swing_trajectories is not None:
                if swing_trajectories[i] is not None:
                    msg.additionalFootPoseTrajectory.append(swing_trajectories[i])
                else:
                    msg.additionalFootPoseTrajectory.append(footPoses6D())

        msg.swingHeightTrajectory = [self.swing_height] * num

        return msg

    def publish_trajectory_6d(self, msg) -> bool:
        """发布6D轨迹消息"""
        if msg is not None:
            # 6D轨迹发布器 (用于斜坡控制)
            traj_pub_6d = rospy.Publisher('/humanoid_mpc_foot_pose_6d_target_trajectories',
                                           footPose6DTargetTrajectories, queue_size=1)
            rospy.sleep(0.5)  # 等待发布器准备就绪
            traj_pub_6d.publish(msg)
            rospy.loginfo("[ClimbStair] 6D轨迹消息发布成功")
            return True
        else:
            rospy.logerr("[ClimbStair] 错误: 6D轨迹消息为空")
            return False

    def set_stair_params_6d(self, stair_height: float, stair_length: float) -> None:
        """设置6D轨迹使用的楼梯参数

        Args:
            stair_height: 单级楼梯高度
            stair_length: 单级楼梯长度（深度）
        """
        self.stair_height = stair_height
        self.stair_length = stair_length
        rospy.loginfo(f"[ClimbStair] 6D stair params set: height={stair_height}, length={stair_length}")

    def set_swing_height_6d(self, swing_height: float) -> None:
        """设置6D轨迹的抬脚高度

        Args:
            swing_height: 抬脚高度
        """
        self.swing_height = swing_height
        rospy.loginfo(f"[ClimbStair] 6D swing height set: {swing_height}")
    
    def simple_up_stairs(self, stair_height = 0.25,stair_length = 0.08,stair_num = 4):
        """
        生成简单的上楼梯轨迹
        Args:
            stair_height: 楼梯高度
            stair_length: 楼梯长度
            stair_num: 楼梯级数
        Returns:
            bool: True if successful, False otherwise.
        """
        try:
            msg = self.simple_up_stairs_6d(stair_height, stair_length, stair_num,0.0)
            time_delay = msg.timeTrajectory[-1] + 1   # 等待时间应为最后足迹点的时间加上1秒的缓冲时间
            if self.publish_trajectory_6d(msg):
                rospy.sleep(time_delay)
                rospy.loginfo(f"[ClimbStair] Simple up stair published successfully")
                return True
            else:
                rospy.logerr(f"[ClimbStair] Simple up stair publish failed")
                return False
        except Exception as e:
            print(f"Simple up stair generate failed: {str(e)}")
            return False    

    def simple_align_stair(self):
        """对齐楼梯"""
        rospy.wait_for_service('stair_alignment')
        try:
            offset_x, offset_y, offset_yaw, tag_id = self.get_align_stair_offset()
            align_stair_service = rospy.ServiceProxy('stair_alignment', stairAlignmentSrv)
            req = stairAlignmentSrvRequest()
            req.tag_id = tag_id         # 固定提供一个 id 的 tag，用于对齐
            req.offset_x = offset_x
            req.offset_y = offset_y
            # 将度转换为弧度 (服务需要弧度)
            req.offset_yaw = offset_yaw

            resp = align_stair_service(req)
            if resp.result:
                print(f"楼梯对齐成功: {resp.message}")
                return True
            else:
                print(f"楼梯对齐失败: {resp.message}")
                return False
        except rospy.ServiceException as e:
            print(f"楼梯对齐服务调用失败: {e}")
            return False

    def get_align_stair_offset(self):
        """从 JSON 配置文件获取楼梯对齐偏置

        从 package_path/upload_files/stair_alignment_config.json 读取配置

        Returns:
            tuple: (offset_x,offset_y,offset_yaw, tag_id) 如果成功
            tuple: (None, None, None, None) 如果失败
        """
        try:
            # 包路径和对齐配置文件路径
            package_name = 'planarmwebsocketservice'
            package_path = rospkg.RosPack().get_path(package_name)
            print(f"package_path: {package_path}")
            # 对齐配置文件路径 (在 upload_files 目录下)
            align_config_file = os.path.join(package_path, "upload_files", "stair_alignment_config.json")
            if os.path.exists(align_config_file):
                with open(align_config_file, 'r') as f:
                    config = json.load(f)
                    offset_x = config.get('offset_x', None)
                    offset_y = config.get('offset_y', None)
                    offset_yaw = config.get('offset_yaw', None)
                    tag_id = config.get('tag_id', None)
                    rospy.loginfo(f"[ClimbStair] Loaded align config: offset_x={offset_x}, offset_y={offset_y}, offset_yaw={offset_yaw}, tag_id={tag_id}")
                    return offset_x, offset_y, offset_yaw, tag_id
            else:
                rospy.logwarn(f"[ClimbStair] Align config file not found: {align_config_file}")
                return None, None, None, None
        except Exception as e:
            rospy.logerr(f"[ClimbStair] 获取楼梯对齐偏置失败: {str(e)}")
            return None, None, None, None


    def _get_rotation_matrix(self, yaw: float) -> np.ndarray:
        """Get cached rotation matrix for yaw angle."""
        # Cache rotation matrices to avoid repeated computation
        yaw_key = round(yaw, 6)  # Round to avoid floating point precision issues
        if yaw_key not in self._rotation_matrices_cache:
            cos_yaw, sin_yaw = np.cos(yaw), np.sin(yaw)
            self._rotation_matrices_cache[yaw_key] = np.array(
                [[cos_yaw, -sin_yaw, 0], [sin_yaw, cos_yaw, 0], [0, 0, 1]]
            )
        return self._rotation_matrices_cache[yaw_key]

    def _convert_arrays_to_lists(self, torso_traj: List) -> None:
        """Convert numpy arrays to lists for ROS message compatibility."""
        for i in range(len(torso_traj)):
            if isinstance(torso_traj[i], np.ndarray):
                torso_traj[i] = torso_traj[i].tolist()

    def execute_trajectory(self) -> bool:
        """Execute the complete accumulated trajectory."""
        if len(self.time_traj) == 0:
            if self.verbose_logging:
                rospy.logwarn("[ClimbStair] No trajectory to publish")
            return False

        # Convert numpy arrays to lists for ROS message compatibility
        self._convert_arrays_to_lists(self.torso_traj)

        if self.verbose_logging:
            rospy.loginfo(
                f"[ClimbStair] Publishing complete trajectory with {len(self.time_traj)} points"
            )

        publish_foot_pose_traj(
            self.time_traj,
            self.foot_idx_traj,
            self.foot_traj,
            self.torso_traj,
            self.swing_trajectories,
            self.verbose_logging,
        )
        return True

    def generate_steps(
        self,
        torso_pos: Union[np.ndarray, List[float]],
        torso_yaw: float,
        foot_height: float = 0,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Calculate foot placement based on torso position

        Args:
            torso_pos: Torso position [x, y, z]
            torso_yaw: Torso yaw angle
            foot_height: Foot height offset

        Returns:
            Tuple of left and right foot positions
        """
        torso_pos = np.asarray(torso_pos)

        # Use cached rotation matrix for better performance
        R_z = self._get_rotation_matrix(torso_yaw)

        # Pre-compute foot biases
        foot_height_offset = -torso_pos[2] + foot_height
        l_foot_bias = np.array([0, self.foot_width, foot_height_offset])
        r_foot_bias = np.array([0, -self.foot_width, foot_height_offset])

        # Compute foot positions
        l_foot = torso_pos + R_z.dot(l_foot_bias)
        r_foot = torso_pos + R_z.dot(r_foot_bias)

        return l_foot, r_foot

    def plan_move_to(
        self,
        dx=0.2,
        dy=0.0,
        dyaw=0.0,
        time_traj=None,
        foot_idx_traj=None,
        foot_traj=None,
        torso_traj=None,
        swing_trajectories=None,
        max_step_x=0.28,
        max_step_y=0.15,
        max_step_yaw=30.0,
    ):
        """
        Plan trajectory to move to target position
        """
        if time_traj is None:
            time_traj = []
        if foot_idx_traj is None:
            foot_idx_traj = []
        if foot_traj is None:
            foot_traj = []
        if torso_traj is None:
            torso_traj = []
        if swing_trajectories is None:
            swing_trajectories = []
        current_height = self.STAND_HEIGHT
        # Get the last trajectory point as starting position
        if len(torso_traj) > 0:
            current_torso_pos = np.array(torso_traj[-1])
            current_foot_pos = np.array(foot_traj[-1][0:3])
            current_yaw = current_torso_pos[3]
            current_height = current_foot_pos[2]
            R_z = np.array(
                [
                    [np.cos(current_yaw), -np.sin(current_yaw), 0],
                    [np.sin(current_yaw), np.cos(current_yaw), 0],
                    [0, 0, 1],
                ]
            )
            dx, dy, _ = R_z.dot(np.array([dx, dy, 0]))
            print("new dx, dy, dyaw", dx, dy, dyaw)

        else:
            current_torso_pos = np.array([0.0, 0.0, 0.0, 0.0])
            current_foot_pos = np.array([0.0, 0.0, self.STAND_HEIGHT])
            current_yaw = 0.0

        # Calculate required number of steps
        num_steps_x = max(1, int(np.ceil(abs(dx) / max_step_x)))
        num_steps_y = max(1, int(np.ceil(abs(dy) / max_step_y)))
        num_steps_yaw = max(1, int(np.ceil(abs(dyaw) / max_step_yaw)))
        num_steps = max(num_steps_x, num_steps_y, num_steps_yaw)

        # Calculate actual step size
        actual_step_x = dx / num_steps
        actual_step_y = dy / num_steps
        
        # Record initial yaw angle for target calculation (aligned with roban script)
        initial_yaw = current_torso_pos[3]
        target_yaw = initial_yaw + np.radians(dyaw)
        
        # is_left_foot = ((self.total_step - 1) % 2 == 0 or dyaw > 0)
        if dyaw > 0:
            self.is_left_foot = True
        # Record starting trajectory length (for debugging)
        # start_traj_len = len(foot_traj)  # Currently unused
        num_steps += 1  # First and last steps are half steps
        # 使用类变量中的时间参数 (aligned with continuousStairClimber-roban.py)
        walk_dt = self.walk_dt if hasattr(self, 'walk_dt') else StairClimbingConstants.WALK_DT
        walk_ss_time = self.walk_ss_time if hasattr(self, 'walk_ss_time') else StairClimbingConstants.WALK_SS_TIME

        for i in range(num_steps):
            self.total_step += 1
            time_traj.append((time_traj[-1] if len(time_traj) > 0 else 0) + walk_dt)

            # Alternate left and right feet
            self.is_left_foot = not self.is_left_foot
            foot_idx_traj.append(0 if self.is_left_foot else 1)
            
            # Calculate current step target yaw angle (linear interpolation, aligned with roban script)
            if abs(dyaw) > 0.1:  # Only update yaw angle when significant turning is needed
                progress = (i + 1) / num_steps
                current_torso_yaw = initial_yaw + progress * np.radians(dyaw)
            else:
                current_torso_yaw = initial_yaw  # Keep yaw angle unchanged for straight walking
            
            # Update torso position
            if i == 0:
                current_torso_pos[0] += actual_step_x / 2
                current_torso_pos[1] += actual_step_y / 2
                current_torso_pos[3] = current_torso_yaw
                # Calculate foot placement offset based on current yaw angle
                desire_torso_pos = [
                    current_torso_pos[0] + actual_step_x / 2,
                    current_torso_pos[1] + actual_step_y / 2,
                    current_torso_pos[2],
                ]
                lf_foot, rf_foot = self.generate_steps(
                    desire_torso_pos, current_torso_yaw, current_height
                )
                current_foot_pos = lf_foot if self.is_left_foot else rf_foot
            # elif i == num_steps - 1 or (abs(dyaw)>0 and i == num_steps - 2):
            elif i == num_steps - 1:
                current_torso_pos[0] += actual_step_x / 2
                current_torso_pos[1] += actual_step_y / 2
                current_torso_pos[3] = target_yaw  # Last step ensures reaching target yaw angle
                # Calculate foot placement offset based on current yaw angle
                lf_foot, rf_foot = self.generate_steps(
                    current_torso_pos[:3], current_torso_pos[3], current_height
                )
                current_foot_pos = lf_foot if self.is_left_foot else rf_foot
            else:
                current_torso_pos[0] += actual_step_x
                current_torso_pos[1] += actual_step_y
                current_torso_pos[3] = current_torso_yaw
                # Calculate foot placement offset based on current yaw angle
                desire_torso_pos = [
                    current_torso_pos[0] + actual_step_x / 2,
                    current_torso_pos[1] + actual_step_y / 2,
                    current_torso_pos[2],
                ]
                lf_foot, rf_foot = self.generate_steps(
                    desire_torso_pos, current_torso_yaw, current_height
                )
                current_foot_pos = lf_foot if self.is_left_foot else rf_foot

            # 叠加临时x方向偏置（每步都叠加，参考continuousStairClimber-roban.py）
            current_torso_pos[0] += self.temp_x_offset * (i + 1)

            # Add trajectory point
            foot_traj.append(
                [
                    current_foot_pos[0],
                    current_foot_pos[1],
                    current_foot_pos[2],
                    current_torso_pos[3],
                ]
            )
            torso_traj.append(current_torso_pos.copy())
            swing_trajectories.append(footPoses())

            time_traj.append(time_traj[-1] + walk_ss_time)
            foot_idx_traj.append(2)
            foot_traj.append(foot_traj[-1].copy())
            torso_traj.append(torso_traj[-1].copy())
            swing_trajectories.append(footPoses())

        return time_traj, foot_idx_traj, foot_traj, torso_traj, swing_trajectories

    def plan_up_stairs(
        self,
        num_steps=5,
        time_traj=None,
        foot_idx_traj=None,
        foot_traj=None,
        torso_traj=None,
        swing_trajectories=None,
        stair_offset=0.0,
    ):
        """Plan up stairs trajectory implementation"""
        if time_traj is None:
            time_traj = []
        if foot_idx_traj is None:
            foot_idx_traj = []
        if foot_traj is None:
            foot_traj = []
        if torso_traj is None:
            torso_traj = []
        if swing_trajectories is None:
            swing_trajectories = []
        torso_yaw = 0.0

        # Get the last trajectory point as starting position
        start_foot_pos_x = 0.0
        start_foot_pos_z = self.STAND_HEIGHT
        if len(torso_traj) > 0:
            current_torso_pos = np.array(torso_traj[-1][0:3])
            current_foot_pos = np.array(foot_traj[-1][0:3])
            start_foot_pos_x = current_foot_pos[0]
            torso_yaw = torso_traj[-1][3]
            start_foot_pos_z = current_foot_pos[2]
        else:
            current_torso_pos = np.array([0.0, 0.0, 0.0])
            current_foot_pos = np.array([0.0, 0.0, self.STAND_HEIGHT])

        # Initial position
        torso_height_offset = -0.02  # 躯干高度偏移 (aligned with continuousStairClimber-roban.py)
        current_torso_pos[2] += torso_height_offset
        # 基础offset数组，后续会加上stair_offset (aligned with continuousStairClimber-roban.py)
        base_offset_x = [0.00, self.up_stairs_double_step_offset, self.up_stairs_double_step_offset, self.up_stairs_double_step_offset, 0.0]
        # 所有offset都加上离楼梯的偏置距离
        offset_x = [offset + stair_offset for offset in base_offset_x]

        # Record previous left and right foot positions
        prev_left_foot = [start_foot_pos_x, self.foot_width, start_foot_pos_z, torso_yaw]
        prev_right_foot = [start_foot_pos_x, -self.foot_width, start_foot_pos_z, torso_yaw]
        initial_index = len(foot_traj)
        # Generate footsteps for each step
        for step in range(num_steps):
            # Update time
            self.total_step += 1
            time_traj.append((time_traj[-1] if len(time_traj) > 0 else 0) + self.dt)

            # Alternate left and right feet
            self.is_left_foot = not self.is_left_foot
            foot_idx_traj.append(0 if self.is_left_foot else 1)

            # Calculate torso position (aligned with continuousStairClimber-roban.py)
            if step == 0:
                current_foot_pos[0] = current_torso_pos[0] + self.step_length  # 脚掌相对躯干前移
                current_foot_pos[1] = current_torso_pos[1] + self.foot_width if self.is_left_foot else -self.foot_width  # 左右偏移
                current_foot_pos[2] = start_foot_pos_z + self.step_height + self.STAND_HEIGHT  # 脚掌高度
                current_torso_pos[0] += self.step_length/2
                current_torso_pos[2] += self.step_height/2
                
            elif step == num_steps - 1: # 最后一步
                current_torso_pos[0] = current_foot_pos[0] # 最后一步躯干x在双脚上方
                current_foot_pos[1] = current_torso_pos[1] + self.foot_width if self.is_left_foot else -self.foot_width  # 左右偏移
                current_torso_pos[2] += self.step_height/2 
            else:
                current_torso_pos[0] += self.step_length  # 向前移动
                current_torso_pos[2] += self.step_height  # 向上移动
            
                # 计算落脚点位置
                current_foot_pos[0] = current_torso_pos[0] + self.step_length/2  # 脚掌相对躯干前移
                current_foot_pos[1] = current_torso_pos[1] + self.foot_width if self.is_left_foot else -self.foot_width  # 左右偏移
                current_foot_pos[2] += self.step_height

            # 叠加临时x方向偏置（每步都叠加，参考continuousStairClimber-roban.py）
            current_torso_pos[0] += self.temp_x_offset * (step + 1)
                
            if step < len(offset_x) and not step == num_steps - 1:  # 脚掌偏移
                current_foot_pos[0] += offset_x[step]

            # Record current foot position
            current_foot = [*current_foot_pos, torso_yaw]

            # Generate swing phase trajectory
            if (
                prev_left_foot is not None and prev_right_foot is not None
            ):  # Generate swing phase from second step onwards
                prev_foot = prev_left_foot if self.is_left_foot else prev_right_foot
                swing_traj = self.plan_swing_phase(
                    prev_foot,
                    current_foot,
                    swing_height=0.12,
                    is_first_step=(step == 0 or step == num_steps - 1),
                )
                swing_trajectories.append(swing_traj)
            else:
                swing_trajectories.append(None)

            # Update previous foot position
            if self.is_left_foot:
                prev_left_foot = current_foot
            else:
                prev_right_foot = current_foot

            # Add trajectory point
            foot_traj.append(current_foot)
            torso_traj.append([*current_torso_pos, torso_yaw])

            last_torso_pose = torso_traj[-1].copy()
            last_foot_pose = foot_traj[-1].copy()
            # 添加支撑相 (aligned with continuousStairClimber-roban.py)
            if step != num_steps - 1:
                pass
                time_traj.append(time_traj[-1] + self.ss_time)
                foot_idx_traj.append(2)
                foot_traj.append(foot_traj[-1].copy())
                last_torso_pose[0] = last_foot_pose[0] - self.step_length*0.0
                torso_traj.append(last_torso_pose)
                swing_trajectories.append(footPoses())
            else: # 最后一步站立恢复站直
                time_traj.append(time_traj[-1] + self.ss_time)
                foot_idx_traj.append(2)
                foot_traj.append(foot_traj[-1].copy())
                last_torso_pose[0] = last_foot_pose[0]
                last_torso_pose[2] = last_foot_pose[2] - self.STAND_HEIGHT
                torso_traj.append(last_torso_pose)
                swing_trajectories.append(footPoses())

        # Handle rotation offset
        if initial_index > 0:
            init_torso_pos = torso_traj[initial_index - 1]
            # init_foot_pos = foot_traj[initial_index-1]  # Currently unused
            for i in range(initial_index, len(foot_traj)):
                diff_yaw = torso_traj[i][3]
                R_z = np.array(
                    [
                        [np.cos(diff_yaw), -np.sin(diff_yaw), 0],
                        [np.sin(diff_yaw), np.cos(diff_yaw), 0],
                        [0, 0, 1],
                    ]
                )
                d_torso_pos = torso_traj[i][0:3] - init_torso_pos[0:3]
                torso_traj[i][0:2] = (R_z.dot(d_torso_pos) + init_torso_pos[0:3])[:2]

                d_foot_pos = (
                    foot_traj[i][0:3] - init_torso_pos[0:3]
                )  # 计算相对于躯干位置的偏移量
                foot_traj[i][0:2] = (R_z.dot(d_foot_pos) + init_torso_pos[0:3])[:2]
                if swing_trajectories[i] is not None:  # 旋转腾空相规划
                    for j in range(len(swing_trajectories[i].data)):
                        d_foot_pos = (
                            swing_trajectories[i].data[j].footPose[0:3]
                            - init_torso_pos[0:3]
                        )
                        swing_trajectories[i].data[j].footPose[0:2] = (
                            R_z.dot(d_foot_pos) + init_torso_pos[0:3]
                        )[:2]

        return time_traj, foot_idx_traj, foot_traj, torso_traj, swing_trajectories

    def plan_down_stairs(
        self,
        num_steps=5,
        time_traj=None,
        foot_idx_traj=None,
        foot_traj=None,
        torso_traj=None,
        swing_trajectories=None,
    ):
        """Plan down stairs trajectory implementation"""
        if time_traj is None:
            time_traj = []
        if foot_idx_traj is None:
            foot_idx_traj = []
        if foot_traj is None:
            foot_traj = []
        if torso_traj is None:
            torso_traj = []
        if swing_trajectories is None:
            swing_trajectories = []
        self.dt = 0.6
        self.step_length = 0.28
        torso_yaw = 0.0
        start_foot_pos_x = 0.0
        start_foot_pos_z = self.STAND_HEIGHT

        # Get the last trajectory point as starting position
        if len(torso_traj) > 0:
            current_torso_pos = np.array(torso_traj[-1][0:3])
            current_foot_pos = np.array(foot_traj[-1][0:3])
            start_foot_pos_x = current_foot_pos[0]
            torso_yaw = torso_traj[-1][3]
            start_foot_pos_z = current_foot_pos[2]

        else:
            current_torso_pos = np.array([0.0, 0.0, 0.0])
            current_foot_pos = np.array([0.0, 0.0, self.STAND_HEIGHT])
            start_foot_pos_x = 0.0
        R_z = np.array(
            [
                [np.cos(torso_yaw), -np.sin(torso_yaw), 0],
                [np.sin(torso_yaw), np.cos(torso_yaw), 0],
                [0, 0, 1],
            ]
        )
        # Initial position
        torso_height_offset = -0.0  # 躯干高度偏移
        current_torso_pos[2] += torso_height_offset
        offset_x = [0.0, -0.0, -0.0, -0.0, -0.0]
        # first_step_offset = self.step_length + 0.05

        # Record previous left and right foot positions
        prev_left_foot = [start_foot_pos_x, 0.1, start_foot_pos_z, torso_yaw]
        prev_right_foot = [start_foot_pos_x, -0.1, start_foot_pos_z, torso_yaw]
        if len(foot_traj) > 0:
            if foot_idx_traj[-2] == 0:  # 最后一步是左脚
                prev_left_foot = foot_traj[-2]
                prev_right_foot = foot_traj[-4] if len(foot_traj) > 3 else None
            else:  # 最后一步是右脚
                prev_right_foot = foot_traj[-2]
                prev_left_foot = foot_traj[-4] if len(foot_traj) > 3 else None
        initial_index = len(foot_traj)
        print("prev_left_foot: ", prev_left_foot)
        print("prev_right_foot: ", prev_right_foot)
        # 添加下蹲
        if len(time_traj) > 0:
            time_traj.append(time_traj[-1] + 1)
            foot_idx_traj.append(2)
            foot_traj.append(foot_traj[-1].copy())
            torso_traj.append(torso_traj[-1].copy())
            torso_traj[-1][2] = current_torso_pos[2]
            swing_trajectories.append(None)
        else:
            time_traj.append(1)
            foot_idx_traj.append(2)
            foot_traj.append([0, 0, 0, 0])
            torso_traj.append([0, 0, current_torso_pos[2], 0])
            swing_trajectories.append(None)

        first_step_offset = -0.01
        # Generate footsteps for each step
        for step in range(num_steps):
            # Update time
            self.total_step += 1
            time_traj.append((time_traj[-1] if len(time_traj) > 0 else 0) + self.dt)

            # Alternate left and right feet
            self.is_left_foot = not self.is_left_foot
            foot_idx_traj.append(0 if self.is_left_foot else 1)

            # Calculate torso position
            if step == 0:
                # current_torso_pos[0] += self.step_length/2 + first_step_offset
                current_foot_pos[0] = (
                    current_torso_pos[0] + self.step_length + first_step_offset
                )  # 脚掌相对躯干前移
                current_torso_pos[0] += self.step_length / 2 + first_step_offset
                # current_torso_pos[0] = current_foot_pos[0] - 0.03 # 躯干落在前脚掌
                current_foot_pos[1] = (
                    current_torso_pos[1] + self.foot_width
                    if self.is_left_foot
                    else -self.foot_width
                )  # Left/right offset
                current_foot_pos[2] -= self.step_height  # 脚掌高度
                current_torso_pos[2] -= self.step_height - 0.0  # 脚掌高度
            elif step == num_steps - 1:  # Last step
                current_torso_pos[0] = current_foot_pos[
                    0
                ]  # Last step: torso x above both feet
                # current_foot_pos[0] = current_torso_pos[0]  #
                current_foot_pos[1] = (
                    current_torso_pos[1] + self.foot_width
                    if self.is_left_foot
                    else -self.foot_width
                )  # Left/right offset
                # current_torso_pos[2] += self.step_height  # 脚掌高度
            else:
                current_torso_pos[0] += self.step_length  # Move forward
                current_torso_pos[2] -= self.step_height  # 向下移动

                # Calculate foot placement position
                current_foot_pos[0] = (
                    current_torso_pos[0] + self.step_length / 2
                )  # 脚掌相对躯干前移
                current_foot_pos[1] = (
                    current_torso_pos[1] + self.foot_width
                    if self.is_left_foot
                    else -self.foot_width
                )  # Left/right offset
                current_foot_pos[2] -= self.step_height

            if step < len(offset_x) and not step == num_steps - 1:  # Foot offset
                current_foot_pos[0] += offset_x[step]

            # Record current foot position
            current_foot = [*current_foot_pos, torso_yaw]

            # Generate swing phase trajectory
            if (
                prev_left_foot is not None and prev_right_foot is not None
            ):  # Generate swing phase from second step onwards
                prev_foot = prev_left_foot if self.is_left_foot else prev_right_foot
                swing_traj = self.plan_swing_phase(
                    prev_foot,
                    current_foot,
                    swing_height=0.05,
                    down_stairs=True,
                    is_first_step=(step == 0 or step == num_steps - 1),
                )
                swing_trajectories.append(swing_traj)
            else:
                swing_trajectories.append(None)

            # Update previous foot position
            if self.is_left_foot:
                prev_left_foot = current_foot
            else:
                prev_right_foot = current_foot

            # Add trajectory point
            # print("step: ", step, "foot: ", foot_idx_traj[-1])
            # print("current_foot: ", current_foot)
            # print("current_torso_pos", current_torso_pos)
            foot_traj.append(current_foot)
            torso_traj.append([*current_torso_pos, torso_yaw])

            last_torso_pose = torso_traj[-1].copy()
            last_foot_pose = foot_traj[-1].copy()
            # add SS
            self.ss_time = 0.4
            if step != num_steps - 1:
                time_traj.append(time_traj[-1] + self.ss_time)
                foot_idx_traj.append(2)
                foot_traj.append(foot_traj[-1].copy())
                last_torso_pose[0] = last_foot_pose[0]
                torso_traj.append(last_torso_pose)
                swing_trajectories.append(footPoses())

            else:  # Last step: standing recovery to straight position
                time_traj.append(time_traj[-1] + self.ss_time)
                foot_idx_traj.append(2)
                foot_traj.append(foot_traj[-1].copy())
                last_torso_pose[0] = last_foot_pose[0]
                last_torso_pose[2] = last_foot_pose[2] - self.STAND_HEIGHT
                torso_traj.append(last_torso_pose)
                swing_trajectories.append(footPoses())
            # break

        # Handle rotation offset
        if initial_index > 0:
            init_torso_pos = torso_traj[initial_index - 1]
            # init_foot_pos = foot_traj[initial_index-1]  # Currently unused
            for i in range(initial_index, len(foot_traj)):
                diff_yaw = torso_traj[i][3]
                R_z = np.array(
                    [
                        [np.cos(diff_yaw), -np.sin(diff_yaw), 0],
                        [np.sin(diff_yaw), np.cos(diff_yaw), 0],
                        [0, 0, 1],
                    ]
                )
                d_torso_pos = torso_traj[i][0:3] - init_torso_pos[0:3]
                torso_traj[i][0:2] = (R_z.dot(d_torso_pos) + init_torso_pos[0:3])[:2]

                d_foot_pos = (
                    foot_traj[i][0:3] - init_torso_pos[0:3]
                )  # 计算相对于躯干位置的偏移量
                foot_traj[i][0:2] = (R_z.dot(d_foot_pos) + init_torso_pos[0:3])[:2]

                if swing_trajectories[i] is not None:  # 旋转腾空相规划
                    for j in range(len(swing_trajectories[i].data)):
                        d_foot_pos = (
                            swing_trajectories[i].data[j].footPose[0:3]
                            - init_torso_pos[0:3]
                        )
                        swing_trajectories[i].data[j].footPose[0:2] = (
                            R_z.dot(d_foot_pos) + init_torso_pos[0:3]
                        )[:2]
        return time_traj, foot_idx_traj, foot_traj, torso_traj, swing_trajectories

    def plan_swing_phase(
        self,
        prev_foot_pose,
        next_foot_pose,
        swing_height=0.10,
        down_stairs=False,
        is_first_step=False,
    ):
        """
        使用三角函数+五次多项式插值规划腾空相的轨迹
        """
        return self._trigonometric_quintic_interpolation(
            prev_foot_pose=prev_foot_pose,
            next_foot_pose=next_foot_pose,
            swing_height=swing_height,
            num_points=7,
            is_first_step=is_first_step,
            down_stairs=down_stairs,
        )
    

    
    def _trigonometric_quintic_interpolation(self, prev_foot_pose, next_foot_pose, swing_height, 
                                           num_points, is_first_step, down_stairs):
        """三角函数+五次多项式插值方法（Z方向使用三角函数，XY方向使用摆线）"""
        additionalFootPoseTrajectory = footPoses()
        
        # 计算移动距离
        x_distance = next_foot_pose[0] - prev_foot_pose[0]
        y_distance = next_foot_pose[1] - prev_foot_pose[1]
        z_distance = next_foot_pose[2] - prev_foot_pose[2]
        
        # 下楼梯时使用反向规划（先多项式再三角函数）
        if down_stairs:
            return self._trigonometric_quintic_interpolation_downstairs(prev_foot_pose, next_foot_pose, swing_height, 
                                                                      num_points, is_first_step)
        
        # 三角函数参数设置（上楼梯）
        if is_first_step:
            # 第一步：更保守的参数
            trig_ratio = 0.6  # 三角函数部分占比
            max_height_ratio = 1.0  # 最高点相对于总高度的比例
        else:
            # 后续步骤：优化参数
            trig_ratio = 0.6  # 三角函数部分占比
            max_height_ratio = 0.9  # 最高点相对于总高度的比例
        
        # 计算基准高度（取两个落点中较高的点）
        base_height = max(prev_foot_pose[2], next_foot_pose[2])
        min_height = min(prev_foot_pose[2], next_foot_pose[2])
        
        # 三角函数最高点高度参考三次样条：base_height + swing_height
        max_height = base_height + swing_height
        
        # 1. 生成三角函数轨迹的4个控制点（Z方向三角函数，XY方向摆线）
        trig_control_points = []
        
        # 使用三角函数生成控制点（确保在最高点零加速度）
        trig_progress = [0.0, 0.33, 0.67, 1.0]  # 三角函数内部进度
        
        for i, progress in enumerate(trig_progress):
            # 计算平滑进度（使用三次多项式确保在t=1时导数为0）
            t = progress
            smooth_progress = 3 * t**2 - 2 * t**3  # 三次多项式，在t=1时导数为0
            
            # 三角函数方程（Z方向）- 使用正弦函数从起点到最高点
            # z = start_z + (max_height - start_z) * sin(π/2 * smooth_progress)
            start_z = prev_foot_pose[2]
            z = start_z + (max_height - start_z) * np.sin(np.pi/2 * smooth_progress)
            
            # XY方向使用摆线插值
            # 摆线参数：t从0到1
            t_cycloid = progress * trig_ratio  # 归一化到三角函数部分的时间
            
            # 摆线方程：x = t - sin(t), y = 1 - cos(t)
            # 映射到实际坐标
            cycloid_x = t_cycloid - np.sin(2 * np.pi * t_cycloid) / (2 * np.pi)
            cycloid_y = (1 - np.cos(2 * np.pi * t_cycloid)) / 2
            
            # 映射到实际XY坐标
            x = prev_foot_pose[0] + x_distance * cycloid_x
            y = prev_foot_pose[1] + y_distance * cycloid_y
            
            trig_control_points.append([x, y, z])
        
        # 2. 生成多项式轨迹的控制点
        polynomial_control_points = []
        
        # 控制点1：多项式起点（后移，避免与三角函数末端重合）
        t_poly_start = trig_ratio + (1 - trig_ratio) * 0.32  # 三角函数占比后32%位置
        
        # XY方向使用摆线规划
        cycloid_x_poly_start = t_poly_start - np.sin(2 * np.pi * t_poly_start) / (2 * np.pi)
        cycloid_y_poly_start = (1 - np.cos(2 * np.pi * t_poly_start)) / 2
        
        x_poly_start = prev_foot_pose[0] + x_distance * cycloid_x_poly_start
        y_poly_start = prev_foot_pose[1] + y_distance * cycloid_y_poly_start
        
        # Z方向平滑下降（从三角函数终点高度开始）
        z_trig_end = trig_control_points[-1][2]  # 三角函数终点高度
        z_poly_start = z_trig_end + (next_foot_pose[2] - z_trig_end) * 0.15  # 下降15%
        polynomial_control_points.append([x_poly_start, y_poly_start, z_poly_start])
        
        # 控制点2：中间点（使用摆线插值）
        t_mid = trig_ratio + (1 - trig_ratio) * 0.64  # 多项式部分64%位置
        
        # 摆线插值
        cycloid_x_mid = t_mid - np.sin(2 * np.pi * t_mid) / (2 * np.pi)
        cycloid_y_mid = (1 - np.cos(2 * np.pi * t_mid)) / 2
        
        x_mid = prev_foot_pose[0] + x_distance * cycloid_x_mid
        y_mid = prev_foot_pose[1] + y_distance * cycloid_y_mid
        z_mid = next_foot_pose[2] + (z_poly_start - next_foot_pose[2]) * 0.5  # 平滑下降
        polynomial_control_points.append([x_mid, y_mid, z_mid])
        
        # 控制点3：目标位置
        x_end = next_foot_pose[0]
        y_end = next_foot_pose[1]
        z_end = next_foot_pose[2]
        polynomial_control_points.append([x_end, y_end, z_end])
        
        # 3. 生成完整轨迹（7个控制点：4个三角函数点 + 3个多项式点）
        full_trajectory = trig_control_points + polynomial_control_points
        
        # 删除第一个点（三角函数起始点）和最后一个点（多项式终点）
        full_trajectory = full_trajectory[1:-1]
        
        # 4. 生成时间序列（调整时间分布，让后半段更均匀）
        # 时间分配：三角函数部分占trig_ratio，多项式部分占(1-trig_ratio)
        # 延长三角函数部分时间，让抬腿更慢
        extended_trig_ratio = trig_ratio * 1.3  # 延长30%
        trig_times = [extended_trig_ratio * 0.17, extended_trig_ratio * 0.5, extended_trig_ratio]  # 去掉起始点0.0
        
        # 调整多项式部分时间分布，让后半段更均匀
        polynomial_times = [extended_trig_ratio + (1-extended_trig_ratio) * 0.32, 
                           extended_trig_ratio + (1-extended_trig_ratio) * 0.64]  # 删除最后一个时间点1.0
        full_times = trig_times + polynomial_times
        
        # 5. 生成轨迹消息（确保平滑性）
        for i, point in enumerate(full_trajectory):
            step_fp = footPose()
            x, y, z = point[0], point[1], point[2]
            
            # Yaw角度使用平滑插值
            progress = full_times[i]
            yaw = prev_foot_pose[3] + (next_foot_pose[3] - prev_foot_pose[3]) * progress
            
            step_fp.footPose = [x, y, z, yaw]
            additionalFootPoseTrajectory.data.append(step_fp)
        
        return additionalFootPoseTrajectory
    
    def _trigonometric_quintic_interpolation_downstairs(self, prev_foot_pose, next_foot_pose, swing_height, 
                                                      num_points, is_first_step):
        """下楼梯专用：与上楼梯完全镜像对称，使用相同的控制点结构和三角函数范围"""
        additionalFootPoseTrajectory = footPoses()
        
        # 计算移动距离
        x_distance = next_foot_pose[0] - prev_foot_pose[0]
        y_distance = next_foot_pose[1] - prev_foot_pose[1]
        z_distance = next_foot_pose[2] - prev_foot_pose[2]
        
        # 下楼梯参数设置（与上楼梯完全一致）
        if is_first_step:
            # 第一步：更保守的参数
            trig_ratio = 0.6  # 三角函数部分占比
            max_height_ratio = 1.0  # 最高点相对于总高度的比例
        else:
            # 后续步骤：优化参数
            trig_ratio = 0.6  # 三角函数部分占比
            max_height_ratio = 0.9  # 最高点相对于总高度的比例
        
        # 计算基准高度（取两个落点中较高的点）
        base_height = max(prev_foot_pose[2], next_foot_pose[2])
        min_height = min(prev_foot_pose[2], next_foot_pose[2])
        
        # 下楼梯最高点高度：从当前台阶高度+swing_height，然后减去一级step_height
        max_height = prev_foot_pose[2] + swing_height - 0.08  # 当前台阶高度 + swing_height - step_height
        
        # 1. 生成三角函数轨迹的4个控制点（与上楼梯完全相同的结构）
        trig_control_points = []
        
        # 使用三角函数生成控制点（确保在最高点零加速度）
        trig_progress = [0.0, 0.33, 0.67, 1.0]  # 三角函数内部进度（与上楼梯相同）
        
        for i, progress in enumerate(trig_progress):
            # 计算平滑进度（使用三次多项式确保在t=1时导数为0）
            t = progress
            smooth_progress = 3 * t**2 - 2 * t**3  # 三次多项式，在t=1时导数为0
            
            # 三角函数方程（Z方向）- 下楼梯：从起点上升到最高点（与上楼梯相同）
            # z = start_z + (max_height - start_z) * sin(π/2 * smooth_progress)
            start_z = prev_foot_pose[2]
            z = start_z + (max_height - start_z) * np.sin(np.pi/2 * smooth_progress)
            
            # XY方向使用摆线插值（与上楼梯相同）
            # 摆线参数：t从0到1
            t_cycloid = progress * trig_ratio  # 归一化到三角函数部分的时间
            
            # 摆线方程：x = t - sin(t), y = 1 - cos(t)
            # 映射到实际坐标
            cycloid_x = t_cycloid - np.sin(2 * np.pi * t_cycloid) / (2 * np.pi)
            cycloid_y = (1 - np.cos(2 * np.pi * t_cycloid)) / 2
            
            # 映射到实际XY坐标
            x = prev_foot_pose[0] + x_distance * cycloid_x
            y = prev_foot_pose[1] + y_distance * cycloid_y
            
            trig_control_points.append([x, y, z])
        
        # 2. 生成多项式轨迹的控制点（与上楼梯完全相同的结构）
        polynomial_control_points = []
        
        # 控制点1：多项式起点（后移，避免与三角函数末端重合）
        t_poly_start = trig_ratio + (1 - trig_ratio) * 0.32  # 三角函数占比后32%位置
        
        # XY方向使用摆线规划
        cycloid_x_poly_start = t_poly_start - np.sin(2 * np.pi * t_poly_start) / (2 * np.pi)
        cycloid_y_poly_start = (1 - np.cos(2 * np.pi * t_poly_start)) / 2
        
        x_poly_start = prev_foot_pose[0] + x_distance * cycloid_x_poly_start
        y_poly_start = prev_foot_pose[1] + y_distance * cycloid_y_poly_start
        
        # Z方向平滑下降（从三角函数终点高度开始）
        z_trig_end = trig_control_points[-1][2]  # 三角函数终点高度
        z_poly_start = z_trig_end + (next_foot_pose[2] - z_trig_end) * 0.15  # 下降15%
        polynomial_control_points.append([x_poly_start, y_poly_start, z_poly_start])
        
        # 控制点2：中间点（使用摆线插值）
        t_mid = trig_ratio + (1 - trig_ratio) * 0.64  # 多项式部分64%位置
        
        # 摆线插值
        cycloid_x_mid = t_mid - np.sin(2 * np.pi * t_mid) / (2 * np.pi)
        cycloid_y_mid = (1 - np.cos(2 * np.pi * t_mid)) / 2
        
        x_mid = prev_foot_pose[0] + x_distance * cycloid_x_mid
        y_mid = prev_foot_pose[1] + y_distance * cycloid_y_mid
        z_mid = next_foot_pose[2] + (z_poly_start - next_foot_pose[2]) * 0.5  # 平滑下降
        polynomial_control_points.append([x_mid, y_mid, z_mid])
        
        # 控制点3：目标位置
        x_end = next_foot_pose[0]
        y_end = next_foot_pose[1]
        z_end = next_foot_pose[2]
        polynomial_control_points.append([x_end, y_end, z_end])
        
        # 3. 生成完整轨迹（7个控制点：4个三角函数点 + 3个多项式点，与上楼梯相同）
        full_trajectory = trig_control_points + polynomial_control_points
        
        # 删除第一个点（三角函数起始点）和最后一个点（多项式终点）
        full_trajectory = full_trajectory[1:-1]
        
        # 4. 生成时间序列（与上楼梯完全相同的时间分布）
        # 时间分配：三角函数部分占trig_ratio，多项式部分占(1-trig_ratio)
        # 延长三角函数部分时间，让抬腿更慢
        extended_trig_ratio = trig_ratio * 1.3  # 延长30%
        trig_times = [extended_trig_ratio * 0.17, extended_trig_ratio * 0.5, extended_trig_ratio]  # 去掉起始点0.0
        
        # 调整多项式部分时间分布，让后半段更均匀
        polynomial_times = [extended_trig_ratio + (1-extended_trig_ratio) * 0.32, 
                           extended_trig_ratio + (1-extended_trig_ratio) * 0.64]  # 删除最后一个时间点1.0
        full_times = trig_times + polynomial_times
        
        # 5. 生成轨迹消息（与上楼梯完全相同的执行逻辑）
        for i, point in enumerate(full_trajectory):
            step_fp = footPose()
            x, y, z = point[0], point[1], point[2]
            
            # Yaw角度使用平滑插值
            progress = full_times[i]
            yaw = prev_foot_pose[3] + (next_foot_pose[3] - prev_foot_pose[3]) * progress
            
            step_fp.footPose = [x, y, z, yaw]
            additionalFootPoseTrajectory.data.append(step_fp)
        
        return additionalFootPoseTrajectory

    # SDK-style interface methods
    def climb_up_stairs(self, num_steps: int = 5, stair_offset: float = 0.0) -> bool:
        """
        Plan up stairs trajectory and add to accumulated trajectory.

        Args:
            num_steps: Number of steps to climb stairs, must be > 0 and <= 20
            stair_offset: Offset distance from stairs (m), default 0.0

        Returns:
            bool: Whether planning was successful
        """
        # Input validation
        if not isinstance(num_steps, int) or num_steps <= 0:
            rospy.logerr("[ClimbStair] num_steps must be a positive integer")
            return False

        if num_steps > 20:  # Reasonable safety limit
            rospy.logwarn(
                "[ClimbStair] Planning %d steps seems excessive, consider breaking into smaller segments",
                num_steps,
            )

        try:
            if self.verbose_logging:
                rospy.loginfo(
                    f"[ClimbStair] Planning up stairs trajectory with {num_steps} steps"
                )

            # Plan trajectory using existing accumulated trajectory as starting point
            time_traj, foot_idx_traj, foot_traj, torso_traj, swing_trajectories = (
                self.plan_up_stairs(
                    num_steps,
                    self.time_traj.copy(),
                    self.foot_idx_traj.copy(),
                    self.foot_traj.copy(),
                    self.torso_traj.copy(),
                    self.swing_trajectories.copy(),
                    stair_offset,
                )
            )

            # Replace accumulated trajectory with new complete trajectory
            self.time_traj = time_traj
            self.foot_idx_traj = foot_idx_traj
            self.foot_traj = foot_traj
            self.torso_traj = torso_traj
            self.swing_trajectories = swing_trajectories

            if self.verbose_logging:
                rospy.loginfo(
                    f"[ClimbStair] Up stairs planning completed: {len(time_traj)} total trajectory points"
                )
            return True
        except Exception as e:
            rospy.logerr(f"[ClimbStair] Failed to plan up stairs: {e}")
            return False

    def climb_down_stairs(self, num_steps: int = 5) -> bool:
        """
        Plan down stairs trajectory and add to accumulated trajectory.

        Args:
            num_steps: Number of steps to climb down stairs, must be > 0 and <= 20

        Returns:
            bool: Whether planning was successful
        """
        # TEMPORARILY DISABLED: Down stairs functionality is under development
        rospy.logwarn(
            "[ClimbStair] Down stairs functionality is currently disabled (under development)"
        )
        rospy.loginfo(
            "[ClimbStair] Please use climb_up_stairs() and move_to_position() instead"
        )
        return False

        # Input validation
        if not isinstance(num_steps, int) or num_steps <= 0:
            rospy.logerr("[ClimbStair] num_steps must be a positive integer")
            return False

        if num_steps > 20:  # Reasonable safety limit
            rospy.logwarn(
                "[ClimbStair] Planning %d steps seems excessive, consider breaking into smaller segments",
                num_steps,
            )

        try:
            if self.verbose_logging:
                rospy.loginfo(
                    f"[ClimbStair] Planning down stairs trajectory with {num_steps} steps"
                )

            # Plan trajectory using existing accumulated trajectory as starting point
            time_traj, foot_idx_traj, foot_traj, torso_traj, swing_trajectories = (
                self.plan_down_stairs(
                    num_steps,
                    self.time_traj.copy(),
                    self.foot_idx_traj.copy(),
                    self.foot_traj.copy(),
                    self.torso_traj.copy(),
                    self.swing_trajectories.copy(),
                )
            )

            # Replace accumulated trajectory with new complete trajectory
            self.time_traj = time_traj
            self.foot_idx_traj = foot_idx_traj
            self.foot_traj = foot_traj
            self.torso_traj = torso_traj
            self.swing_trajectories = swing_trajectories

            if self.verbose_logging:
                rospy.loginfo(
                    f"[ClimbStair] Down stairs planning completed: {len(time_traj)} total trajectory points"
                )
            return True
        except Exception as e:
            rospy.logerr(f"[ClimbStair] Failed to plan down stairs: {e}")
            return False

    def move_to_position(
        self,
        dx: float = 0.2,
        dy: float = 0.0,
        dyaw: float = 0.0,
        max_step_x: float = None,
        max_step_y: float = None,
        max_step_yaw: float = None,
    ) -> bool:
        """
        Plan move to position trajectory and add to accumulated trajectory.

        Args:
            dx: X direction displacement (m)
            dy: Y direction displacement (m)
            dyaw: Yaw angle displacement (degrees)
            max_step_x: Maximum step size in X direction (m)
            max_step_y: Maximum step size in Y direction (m)
            max_step_yaw: Maximum yaw step size (degrees)

        Returns:
            bool: Whether planning was successful
        """
        # Use defaults if not provided
        max_step_x = (
            max_step_x
            if max_step_x is not None
            else StairClimbingConstants.DEFAULT_MAX_STEP_X
        )
        max_step_y = (
            max_step_y
            if max_step_y is not None
            else StairClimbingConstants.DEFAULT_MAX_STEP_Y
        )
        max_step_yaw = (
            max_step_yaw
            if max_step_yaw is not None
            else StairClimbingConstants.DEFAULT_MAX_STEP_YAW
        )

        # Input validation
        if abs(dx) > 5.0 or abs(dy) > 5.0:  # Reasonable safety limits
            rospy.logerr(
                "[ClimbStair] Movement distance too large: dx=%.3f, dy=%.3f", dx, dy
            )
            return False

        if abs(dyaw) > 180.0:  # Reasonable safety limit
            rospy.logerr(
                "[ClimbStair] Rotation angle too large: dyaw=%.3f degrees", dyaw
            )
            return False

        if max_step_x <= 0 or max_step_y <= 0 or max_step_yaw <= 0:
            rospy.logerr("[ClimbStair] All max_step parameters must be positive")
            return False

        try:
            if self.verbose_logging:
                rospy.loginfo(
                    f"[ClimbStair] Planning move trajectory: dx={dx:.3f}, dy={dy:.3f}, dyaw={dyaw:.3f}"
                )

            # Plan trajectory using existing accumulated trajectory as starting point
            time_traj, foot_idx_traj, foot_traj, torso_traj, swing_trajectories = (
                self.plan_move_to(
                    dx,
                    dy,
                    dyaw,
                    self.time_traj.copy(),
                    self.foot_idx_traj.copy(),
                    self.foot_traj.copy(),
                    self.torso_traj.copy(),
                    self.swing_trajectories.copy(),
                    max_step_x,
                    max_step_y,
                    max_step_yaw,
                )
            )

            # Replace accumulated trajectory with new complete trajectory
            self.time_traj = time_traj
            self.foot_idx_traj = foot_idx_traj
            self.foot_traj = foot_traj
            self.torso_traj = torso_traj
            self.swing_trajectories = swing_trajectories

            if self.verbose_logging:
                rospy.loginfo(
                    f"[ClimbStair] Move planning completed: {len(time_traj)} total trajectory points"
                )
            return True
        except Exception as e:
            rospy.logerr(f"[ClimbStair] Failed to plan move to position: {e}")
            return False

    def get_step_count(self) -> int:
        """Get the current total step count."""
        return self.total_step

    def reset_step_counter(self) -> None:
        """Reset the total step counter."""
        self.total_step = 0

    def get_trajectory_statistics(self) -> dict:
        """
        Get statistics about the current accumulated trajectory.

        Returns:
            dict: Dictionary containing trajectory statistics
        """
        if not self.time_traj:
            return {
                "total_points": 0,
                "duration": 0.0,
                "total_steps": self.total_step,
                "has_swing_trajectories": False,
            }

        swing_count = sum(1 for swing in self.swing_trajectories if swing is not None)

        return {
            "total_points": len(self.time_traj),
            "duration": self.time_traj[-1] - self.time_traj[0]
            if len(self.time_traj) > 1
            else 0.0,
            "total_steps": self.total_step,
            "swing_trajectories_count": swing_count,
            "has_swing_trajectories": swing_count > 0,
            "time_range": (self.time_traj[0], self.time_traj[-1])
            if self.time_traj
            else (0, 0),
        }


def parse_args():
    """Parse command line arguments (aligned with continuousStairClimber-roban.py)"""
    import argparse
    parser = argparse.ArgumentParser(description='Kuavo Robot Stair Climbing SDK')
    parser.add_argument('--plot', action='store_true', help='Enable trajectory plotting (not implemented in SDK)')
    parser.add_argument('--initH', type=float, default=0.0, help='Stand height offset (default: 0.0)')
    parser.add_argument('--steps', type=int, default=5, help='Number of stairs to climb (default: 5)')
    parser.add_argument('--move_x', type=float, default=0.15, help='Forward movement after stairs (default: 0.15m)')
    parser.add_argument('--stair_offset', type=float, default=0.03, help='Offset distance from stairs (default: 0.03m)')
    parser.add_argument('--verbose', action='store_true', help='Enable verbose logging')
    return parser.parse_args()


if __name__ == '__main__':
    try:
        rospy.init_node("climb_stair_node")
        args = parse_args()
        
        # Set parameters based on command line arguments
        stand_height = args.initH
        verbose_logging = args.verbose
        num_stairs = args.steps
        move_distance = args.move_x
        stair_offset = args.stair_offset
        
        if args.plot:
            rospy.logwarn("[ClimbStair] Plot functionality is not implemented in SDK version")
        
        # Disable pitch limit (aligned with roban script)
        rospy.loginfo("[ClimbStair] Disabling pitch limit...")
        set_pitch_limit(False)
        
        # Initialize the SDK robot
        robot = KuavoRobotClimbStair(
            stand_height=stand_height,
            verbose_logging=verbose_logging
        )
        
        rospy.loginfo(f"[ClimbStair] Initialized robot with stand_height={stand_height:.3f}")
        
        # Execute stair climbing sequence (aligned with continuousStairClimber-roban.py)
        rospy.loginfo(f"[ClimbStair] Planning up stairs with {num_stairs} steps, stair_offset={stair_offset:.3f}m...")
        success = robot.climb_up_stairs(num_stairs, stair_offset=stair_offset)
        if success:
            rospy.loginfo("[ClimbStair] Up stairs planning completed successfully")
        else:
            rospy.logerr("[ClimbStair] Up stairs planning failed")
            exit(1)
        
        # Print trajectory details
        stats = robot.get_trajectory_statistics()
        rospy.loginfo(f"[ClimbStair] Trajectory statistics: {stats}")
        
        # Add forward movement after stairs (aligned with roban script)
        rospy.loginfo(f"[ClimbStair] Planning forward movement: {move_distance:.3f}m...")
        success = robot.move_to_position(dx=move_distance, dy=0.0, dyaw=0.0)
        if success:
            rospy.loginfo("[ClimbStair] Move planning completed successfully")
        else:
            rospy.logerr("[ClimbStair] Move planning failed")
            exit(1)
        
        # Print final trajectory details
        final_stats = robot.get_trajectory_statistics()
        rospy.loginfo(f"[ClimbStair] Final trajectory statistics: {final_stats}")
        
        # Print detailed trajectory (similar to roban script)
        if verbose_logging and robot.time_traj:
            rospy.loginfo("[ClimbStair] Detailed trajectory:")
            for i, t in enumerate(robot.time_traj):
                rospy.loginfo(
                    f"  {i:2}: t={t:5.2f} foot_idx={robot.foot_idx_traj[i]} "
                    f"foot=[{robot.foot_traj[i][0]:6.3f}, {robot.foot_traj[i][1]:6.3f}, "
                    f"{robot.foot_traj[i][2]:6.3f}, {robot.foot_traj[i][3]:6.3f}] "
                    f"torso=[{robot.torso_traj[i][0]:6.3f}, {robot.torso_traj[i][1]:6.3f}, "
                    f"{robot.torso_traj[i][2]:6.3f}, {robot.torso_traj[i][3]:6.3f}]"
                )
        
        # Execute the complete trajectory
        rospy.loginfo("[ClimbStair] Executing complete trajectory...")
        success = robot.execute_trajectory()
        if success:
            rospy.loginfo("[ClimbStair] Trajectory execution completed successfully")
        else:
            rospy.logerr("[ClimbStair] Trajectory execution failed")
            exit(1)
            
        rospy.loginfo("[ClimbStair] All operations completed successfully!")
        
    except rospy.ROSInterruptException:
        rospy.loginfo("[ClimbStair] Interrupted by user")
        # Ensure pitch limit is re-enabled on interruption (aligned with roban script)
        set_pitch_limit(True)
    except Exception as e:
        rospy.logerr(f"[ClimbStair] Unexpected error: {e}")
        # Ensure pitch limit is re-enabled on error
        set_pitch_limit(True)
        exit(1)
