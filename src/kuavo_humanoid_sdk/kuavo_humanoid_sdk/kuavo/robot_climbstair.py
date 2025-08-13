#!/usr/bin/env python3
# coding: utf-8
import rospy
import numpy as np
from typing import List, Tuple, Optional, Union
from scipy.interpolate import PchipInterpolator
from kuavo_msgs.msg import footPose, footPoseTargetTrajectories
from std_srvs.srv import SetBool, SetBoolRequest


# Constants for magic numbers
class StairClimbingConstants:
    DEFAULT_DT = 0.6  # Default gait cycle time
    DEFAULT_SS_TIME = 0.5  # Default single support time
    DEFAULT_FOOT_WIDTH = 0.10  # Default foot width
    DEFAULT_STEP_HEIGHT = 0.13  # Default step height
    DEFAULT_STEP_LENGTH = 0.28  # Default step length
    DEFAULT_MAX_STEP_X = 0.28  # Default max step in X direction
    DEFAULT_MAX_STEP_Y = 0.15  # Default max step in Y direction
    DEFAULT_MAX_STEP_YAW = 30.0  # Default max yaw step (degrees)
    DEFAULT_SWING_HEIGHT = 0.10  # Default swing phase height
    DEFAULT_SWING_POINTS = 7  # Default swing trajectory points
    TORSO_HEIGHT_OFFSET = -0.02  # Torso height offset
    WALK_DT = 0.4  # Walking gait cycle time
    WALK_SS_TIME = 0.5  # Walking single support time
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
            f"[ClimbStair] Publishing to /humanoid_mpc_foot_pose_target_trajectories"
        )

    pub.publish(msg)

    if verbose:
        rospy.loginfo(f"[ClimbStair] Trajectory published successfully")

    rospy.sleep(1.0)  # Reduced sleep time


class KuavoRobotClimbStair:
    """Kuavo robot stair climbing implementation with SDK interface"""

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

        # Use constants for parameters
        self.dt = StairClimbingConstants.DEFAULT_DT
        self.ss_time = StairClimbingConstants.DEFAULT_SS_TIME
        self.foot_width = StairClimbingConstants.DEFAULT_FOOT_WIDTH
        self.step_height = StairClimbingConstants.DEFAULT_STEP_HEIGHT
        self.step_length = StairClimbingConstants.DEFAULT_STEP_LENGTH
        self.total_step = 0
        self.is_left_foot = True

        # Global variables from original script
        self.PLOT = False
        self.STAND_HEIGHT = stand_height
        self.verbose_logging = verbose_logging

        # Trajectory accumulation for continuous planning
        self._clear_trajectory_data()

        # Pre-compute commonly used values
        self._rotation_matrices_cache = {}

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
        }

    def _clear_trajectory_data(self) -> None:
        """Internal method to clear trajectory data."""
        self.time_traj = []
        self.foot_idx_traj = []
        self.foot_traj = []
        self.torso_traj = []
        self.swing_trajectories = []

    def clear_trajectory(self) -> None:
        """Clear all accumulated trajectories."""
        self._clear_trajectory_data()
        if self.verbose_logging:
            rospy.loginfo("[ClimbStair] Trajectory cleared")

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
        actual_step_yaw = dyaw / num_steps
        # is_left_foot = ((self.total_step - 1) % 2 == 0 or dyaw > 0)
        if dyaw > 0:
            self.is_left_foot = True
        # Record starting trajectory length (for debugging)
        # start_traj_len = len(foot_traj)  # Currently unused
        num_steps += 1  # First and last steps are half steps
        walk_dt = 0.4
        walk_ss_time = 0.5

        for i in range(num_steps):
            self.total_step += 1
            time_traj.append((time_traj[-1] if len(time_traj) > 0 else 0) + walk_dt)

            # Alternate left and right feet
            self.is_left_foot = not self.is_left_foot
            foot_idx_traj.append(0 if self.is_left_foot else 1)
            # Update torso position
            if i == 0:
                current_torso_pos[0] += actual_step_x / 2
                current_torso_pos[1] += actual_step_y / 2
                current_torso_pos[3] += np.radians(actual_step_yaw)
                # Calculate foot placement offset based on current yaw angle
                current_yaw = current_torso_pos[3]
                desire_torso_pos = [
                    current_torso_pos[0] + actual_step_x / 2,
                    current_torso_pos[1] + actual_step_y / 2,
                    current_torso_pos[2],
                ]
                lf_foot, rf_foot = self.generate_steps(
                    desire_torso_pos, current_yaw, current_height
                )
                current_foot_pos = lf_foot if self.is_left_foot else rf_foot
            # elif i == num_steps - 1 or (abs(dyaw)>0 and i == num_steps - 2):
            elif i == num_steps - 1:
                current_torso_pos[0] += actual_step_x / 2
                current_torso_pos[1] += actual_step_y / 2
                # current_torso_pos[3] += np.radians(actual_step_yaw) if not (abs(dyaw)>0 and i == num_steps - 1) else 0
                current_torso_pos[3] += 0
                # Calculate foot placement offset based on current yaw angle
                current_yaw = current_torso_pos[3]
                lf_foot, rf_foot = self.generate_steps(
                    current_torso_pos[:3], current_yaw, current_height
                )
                current_foot_pos = lf_foot if self.is_left_foot else rf_foot
            else:
                current_torso_pos[0] += actual_step_x
                current_torso_pos[1] += actual_step_y
                current_torso_pos[3] += np.radians(actual_step_yaw)
                # Calculate foot placement offset based on current yaw angle
                current_yaw = current_torso_pos[3]
                desire_torso_pos = [
                    current_torso_pos[0] + actual_step_x / 2,
                    current_torso_pos[1] + actual_step_y / 2,
                    current_torso_pos[2],
                ]
                lf_foot, rf_foot = self.generate_steps(
                    desire_torso_pos, current_yaw, current_height
                )
                current_foot_pos = lf_foot if self.is_left_foot else rf_foot

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
        torso_height_offset = -0.02  # Torso height offset
        current_torso_pos[2] += torso_height_offset
        # current_foot_pos = np.array([0.0, 0.0, 0.0])
        offset_x = [0.0, 0.0, 0.0, 0.0, 0.0]
        # first_step_offset = 0.35  # Currently unused

        # Record previous left and right foot positions
        prev_left_foot = [start_foot_pos_x, 0.1, start_foot_pos_z, torso_yaw]
        prev_right_foot = [start_foot_pos_x, -0.1, start_foot_pos_z, torso_yaw]
        initial_index = len(foot_traj)
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
                current_foot_pos[0] = (
                    current_torso_pos[0] + self.step_length
                )  # Foot moves forward relative to torso
                current_foot_pos[1] = (
                    current_torso_pos[1] + self.foot_width
                    if self.is_left_foot
                    else -self.foot_width
                )  # Left/right offset
                current_foot_pos[2] = (
                    self.step_height + self.STAND_HEIGHT
                )  # Foot height
                current_torso_pos[0] += self.step_length / 3

            elif step == num_steps - 1:  # Last step
                # current_torso_pos[0] += self.step_length/2  # 向前移动
                # current_torso_pos[2] += self.step_height/2  # 向上移动
                # current_foot_pos[0] = current_torso_pos[0] # 最后一步x不动
                current_torso_pos[0] = current_foot_pos[
                    0
                ]  # Last step: torso x above both feet
                current_foot_pos[1] = (
                    current_torso_pos[1] + self.foot_width
                    if self.is_left_foot
                    else -self.foot_width
                )  # Left/right offset
                current_torso_pos[2] += self.step_height
            else:
                current_torso_pos[0] += self.step_length  # Move forward
                current_torso_pos[2] += self.step_height  # Move upward

                # Calculate foot placement position
                current_foot_pos[0] = (
                    current_torso_pos[0] + self.step_length / 2
                )  # 脚掌相对躯干前移
                current_foot_pos[1] = (
                    current_torso_pos[1] + self.foot_width
                    if self.is_left_foot
                    else -self.foot_width
                )  # Left/right offset
                current_foot_pos[2] += self.step_height

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
            # add SS
            if step != num_steps - 1:
                pass
                time_traj.append(time_traj[-1] + self.ss_time)
                foot_idx_traj.append(2)
                foot_traj.append(foot_traj[-1].copy())
                last_torso_pose[0] = last_foot_pose[0] - self.step_length * 0.0
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
        Plan swing phase trajectory using shape-preserving cubic spline interpolation
        """
        additionalFootPoseTrajectory = footPoses()
        num_points = 7  # Number of trajectory points

        # Create time series
        t = np.linspace(0, 1, num_points)

        # Calculate movement distance in x and y directions
        x_distance = next_foot_pose[0] - prev_foot_pose[0]
        y_distance = next_foot_pose[1] - prev_foot_pose[1]

        # Calculate reference height (take the higher of the two landing points)
        base_height = max(prev_foot_pose[2], next_foot_pose[2])
        min_height = min(prev_foot_pose[2], next_foot_pose[2])

        # Create control points
        control_points = None
        if not down_stairs:
            control_points = {
                "t": [0, 0.2, 0.6, 1.0],
                "x": [
                    prev_foot_pose[0],  # Start point
                    prev_foot_pose[0] + x_distance * 0.05,  # 前10%
                    prev_foot_pose[0] + x_distance * 0.6,  # 前50%
                    next_foot_pose[0],  # End point
                ],
                "y": [
                    prev_foot_pose[1],  # 起点
                    prev_foot_pose[1] + y_distance * 0.05,  # 前10%
                    prev_foot_pose[1] + y_distance * 0.6,  # 前50%
                    next_foot_pose[1],  # 终点
                ],
                "z": [
                    prev_foot_pose[2],  # 起点
                    (base_height + swing_height * 0.6)
                    if is_first_step
                    else (
                        prev_foot_pose[2] + (base_height - min_height) * 0.5
                    ),  # 最高点（基于较高的落点）
                    base_height + swing_height,  # 保持最高点
                    next_foot_pose[2],  # 终点
                ],
            }
        else:  # Down stairs
            if not is_first_step:  # 非第一步或者最后一步
                control_points = {
                    "t": [0, 0.3, 0.5, 0.6, 1.0],
                    "x": [
                        prev_foot_pose[0],  # Start point
                        prev_foot_pose[0] + x_distance * 0.4,  # 前50%
                        prev_foot_pose[0] + x_distance * 0.7,  # 前50%
                        prev_foot_pose[0] + x_distance * 0.9,  # 前10%
                        next_foot_pose[0],  # End point
                    ],
                    "y": [
                        prev_foot_pose[1],  # 起点
                        prev_foot_pose[1] + y_distance * 0.4,  # 前50%
                        prev_foot_pose[1] + y_distance * 0.7,  # 前50%
                        prev_foot_pose[1] + y_distance * 0.9,  # 前10%
                        next_foot_pose[1],  # 终点
                    ],
                    "z": [
                        prev_foot_pose[2],  # 起点
                        base_height + swing_height,  # 保持最高点
                        (next_foot_pose[2] + (base_height - min_height) * 0.9),
                        (
                            next_foot_pose[2] + (base_height - min_height) * 0.7
                        ),  # 最高点（基于较高的落点）
                        # prev_foot_pose[2],           # 最高点（基于较高的落点）
                        next_foot_pose[2],  # 终点
                    ],
                }
            else:
                control_points = {
                    "t": [0, 0.5, 0.6, 1.0],
                    "x": [
                        prev_foot_pose[0],  # Start point
                        prev_foot_pose[0] + x_distance * 0.60,  # 前50%
                        prev_foot_pose[0] + x_distance * 0.95,  # 前10%
                        next_foot_pose[0],  # End point
                    ],
                    "y": [
                        prev_foot_pose[1],  # 起点
                        prev_foot_pose[1] + y_distance * 0.6,  # 前50%
                        prev_foot_pose[1] + y_distance * 0.95,  # 前10%
                        next_foot_pose[1],  # 终点
                    ],
                    "z": [
                        prev_foot_pose[2],  # 起点
                        base_height + swing_height,  # 保持最高点
                        base_height + swing_height * 0.2,  # 最高点（基于较高的落点）
                        # prev_foot_pose[2],           # 最高点（基于较高的落点）
                        next_foot_pose[2],  # 终点
                    ],
                }

        # Create shape-preserving cubic spline interpolation for x, y, and z
        x_spline = PchipInterpolator(control_points["t"], control_points["x"])
        y_spline = PchipInterpolator(control_points["t"], control_points["y"])
        z_spline = PchipInterpolator(control_points["t"], control_points["z"])

        # Use shape-preserving cubic spline for yaw angle
        yaw_spline = PchipInterpolator([0, 1], [prev_foot_pose[3], next_foot_pose[3]])

        # Generate trajectory points
        trajectory_points = []
        for i in range(num_points):
            step_fp = footPose()
            x = float(x_spline(t[i]))
            y = float(y_spline(t[i]))
            z = float(z_spline(t[i]))
            yaw = float(yaw_spline(t[i]))

            step_fp.footPose = [x, y, z, yaw]
            additionalFootPoseTrajectory.data.append(step_fp)
            trajectory_points.append([x, y, z])

        return additionalFootPoseTrajectory

    # SDK-style interface methods
    def climb_up_stairs(self, num_steps: int = 5) -> bool:
        """
        Plan up stairs trajectory and add to accumulated trajectory.

        Args:
            num_steps: Number of steps to climb stairs, must be > 0 and <= 20

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
