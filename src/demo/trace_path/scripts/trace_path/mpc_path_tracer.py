#!/usr/bin/env python3

import rospy
import numpy as np
import math
import time
import tf
import tf2_ros
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, PoseStamped, Point, Quaternion, Pose, TransformStamped
from pydrake.all import MathematicalProgram
from pydrake.all import Solve, SnoptSolver
from std_msgs.msg import Bool
from enum import Enum
from std_msgs.msg import Int8

class FollowState(Enum):
    NOT_FOLLOWING = 0
    FOLLOWING = 1
    FINISHED = 2

class Utils:
    @staticmethod
    def get_yaw_from_orientation(orientation):
        """Extract yaw from a quaternion orientation"""
        q = [orientation.x, orientation.y, orientation.z, orientation.w]
        euler = tf.transformations.euler_from_quaternion(q)
        return euler[2]  # yaw
    
    @staticmethod
    def calculate_distance(point1, point2):
        """Calculate Euclidean distance between two points"""
        return math.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2)
    
    @staticmethod
    def calculate_angle(goal, current):
        """Calculate angle between two points"""
        return math.atan2(goal.y - current.y, goal.x - current.x)
    
    @staticmethod
    def normalize_angle(angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

class PathTracerBase:
    def __init__(self, node_name='path_tracer_node'):
        # ROS setup
        self.node_name = node_name
        
        
        # State variables
        self.flag_reset_start_point = True
        self.start_point_pose = None
        self.current_pose = Pose()
        self.path = Path()
   
        
        # Get frame IDs from ROS parameters
        self.robot_frame = rospy.get_param('~robot_frame', 'base_link')
        self.world_frame = rospy.get_param('~world_frame', 'odom')
        self.v_max_linear = rospy.get_param('~v_max_linear', 0.4)
        self.v_max_angular = rospy.get_param('~v_max_angular', 0.4)
        self.dt = rospy.get_param('~dt', 0.1)
        
        rospy.loginfo(f"Using frames: robot='{self.robot_frame}', world='{self.world_frame}'")
        rospy.loginfo(f"Using max linear velocity: {self.v_max_linear} m/s")
        rospy.loginfo(f"Using max angular velocity: {self.v_max_angular} rad/s")
        
        
        # TF Listener setup
        self.tf_listener = tf.TransformListener()
        self.tf_rate = rospy.Rate(100.0) # TF query rate
        self.follow_state = FollowState.NOT_FOLLOWING
        
        # ROS publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.cmd_pose_pub = rospy.Publisher('/cmd_pose', Twist, queue_size=10)
        self.cmd_pose_world_pub = rospy.Publisher('/cmd_pose_world', Twist, queue_size=10)
        self.path_pub = rospy.Publisher('trace_path/path', Path, queue_size=10)
        self.realtime_path_pub = rospy.Publisher('trace_path/realtime_path', Path, queue_size=10)
        self.follow_state_pub = rospy.Publisher('trace_path/follow_state', Int8, queue_size=10)

        
        # Initialize path
        self.path.header.frame_id = self.world_frame
        self.path.header.stamp = rospy.Time.now()

        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish_follow_state)
        
        # Start TF polling thread
        import threading
        self.tf_thread = threading.Thread(target=self.tf_polling)
        self.tf_thread.daemon = True
        self.tf_thread.start()

    def publish_follow_state(self, event):
        self.follow_state_pub.publish(self.follow_state.value)

    def tf_polling(self):
        """Thread function to continuously poll for robot's pose from TF"""
        last_publish_time = rospy.Time.now()
        
        while not rospy.is_shutdown():
            try:
                self.tf_listener.waitForTransform(self.world_frame, self.robot_frame, rospy.Time(0), rospy.Duration(1.0))
                (trans, rot) = self.tf_listener.lookupTransform(self.world_frame, self.robot_frame, rospy.Time(0))
                
                # Update current pose
                self.current_pose.position.x = trans[0]
                self.current_pose.position.y = trans[1]
                self.current_pose.position.z = trans[2]
                self.current_pose.orientation.x = rot[0]
                self.current_pose.orientation.y = rot[1]
                self.current_pose.orientation.z = rot[2]
                self.current_pose.orientation.w = rot[3]
                
                # Initialize start point if needed
                if self.flag_reset_start_point:
                    self.start_point_pose = Pose()
                    self.start_point_pose.position.x = trans[0]
                    self.start_point_pose.position.y = trans[1]
                    self.start_point_pose.position.z = trans[2]
                    self.start_point_pose.orientation.x = rot[0]
                    self.start_point_pose.orientation.y = rot[1]
                    self.start_point_pose.orientation.z = rot[2]
                    self.start_point_pose.orientation.w = rot[3]
                    self.flag_reset_start_point = False
                    rospy.loginfo(f"Start point initialized at: [{trans[0]:.2f}, {trans[1]:.2f}, {trans[2]:.2f}]")
                
                # Publish realtime motion trajectory
                current_time = rospy.Time.now()
                if (current_time - last_publish_time).to_sec() >= 0.1:  # publish path every 1 second
                    path_pose = Pose()
                    path_pose.position.x = trans[0]
                    path_pose.position.y = trans[1]
                    path_pose.position.z = 0.0
                    path_pose.orientation.x = rot[0]
                    path_pose.orientation.y = rot[1]
                    path_pose.orientation.z = rot[2]
                    path_pose.orientation.w = rot[3]
                    
                    pose_stamped = PoseStamped()
                    pose_stamped.header.frame_id = self.path.header.frame_id
                    pose_stamped.header.stamp = current_time
                    pose_stamped.pose = path_pose
                    self.path.poses.append(pose_stamped)
                    self.realtime_path_pub.publish(self.path)
                    last_publish_time = current_time
                
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf2_ros.TransformException) as e:
                # rospy.logwarn(f"TF Error: {e}")
                pass
            
            self.tf_rate.sleep()

    def follow(self, global_path):
        """Base implementation of path following - to be overridden by subclasses"""
        pass
    
    def set_max_linear_velocity(self, v):
        """Set maximum linear velocity"""
        self.v_max_linear = v
    
    def set_max_angular_velocity(self, v):
        """Set maximum angular velocity"""
        self.v_max_angular = v
    
    def publish_path(self, path):
        """Publish path"""
        self.path_pub.publish(path)
    
    def publish_cmd_vel(self, linear_velocity, angular_velocity):
        """Publish velocity command"""
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_velocity
        cmd_vel.angular.z = angular_velocity
        self.cmd_vel_pub.publish(cmd_vel)

    def publish_cmd_pose(self, x, y, yaw):
        """Publish pose command"""
        cmd_pose = Twist()
        cmd_pose.linear.x = x
        cmd_pose.linear.y = y
        cmd_pose.angular.z = yaw
        self.cmd_pose_pub.publish(cmd_pose)
    
    def publish_cmd_pose_world(self, x, y, yaw):
        """Publish pose command in world frame"""
        cmd_pose_world = Twist()
        cmd_pose_world.linear.x = x
        cmd_pose_world.linear.y = y
        cmd_pose_world.angular.z = yaw
        self.cmd_pose_world_pub.publish(cmd_pose_world)
    

def transform_to_base_link(current_pose, global_point):
    """
    Transform a point from global frame (world) to robot base_link frame
    
    Both current_pose and global_point are in world coordinate frame.
    This function transforms global_point to the robot's local coordinate frame.
    """
    # Get rotation matrix from current pose quaternion
    q = [current_pose.orientation.x, current_pose.orientation.y, 
         current_pose.orientation.z, current_pose.orientation.w]
    rot_matrix = tf.transformations.quaternion_matrix(q)[:3, :3]
    
    # Create vectors for transformation
    global_vec = np.array([
        global_point.x - current_pose.position.x,
        global_point.y - current_pose.position.y,
        0.0  # Assuming planar motion
    ])
    
    # Apply inverse rotation to get coordinates in robot's local frame
    local_vec = np.linalg.inv(rot_matrix).dot(global_vec)
    
    # Set output point
    local_point = Point()
    local_point.x = local_vec[0]
    local_point.y = local_vec[1]
    local_point.z = 0.0
    
    return local_point

class TrajectoryAnalyzer:
    def __init__(self):
        # initialize basic performance metrics
        self.total_distance_error = 0.0
        self.max_distance_error = 0.0
        self.min_distance_error = float('inf')
        self.duration = 0.0
        self.avg_speed = 0.0
        self.total_distance = 0.0
        
        # initialize advanced performance metrics
        self.distance_errors = []  # store all distance errors for calculating standard deviation
        
        # resample parameter
        self.resample_distance = 0.1  # resample interval (meter)
        
    def resample_path(self, path):
        """将路径重新参数化为等距点
        
        Args:
            path (nav_msgs.msg.Path): 输入路径
            
        Returns:
            nav_msgs.msg.Path: 重新参数化后的路径
        """
        if len(path.poses) < 2:
            return path
            
        # 创建新的路径消息
        resampled_path = Path()
        resampled_path.header = path.header
        
        # 计算路径总长度
        total_length = 0.0
        segment_lengths = []
        for i in range(1, len(path.poses)):
            length = Utils.calculate_distance(
                path.poses[i].pose.position,
                path.poses[i-1].pose.position
            )
            total_length += length
            segment_lengths.append(length)
            
        # 计算需要采样的点数
        num_points = int(total_length / self.resample_distance) + 1
        if num_points < 2:
            num_points = 2
            
        # 计算每个采样点的距离
        sample_distances = np.linspace(0, total_length, num_points)
        
        # 重新采样路径
        current_segment = 0
        accumulated_length = 0.0
        
        for sample_dist in sample_distances:
            # 找到当前采样点所在的段
            while (current_segment < len(segment_lengths) and 
                   accumulated_length + segment_lengths[current_segment] < sample_dist):
                accumulated_length += segment_lengths[current_segment]
                current_segment += 1
                
            if current_segment >= len(segment_lengths):
                # 如果超出范围，使用最后一个点
                resampled_path.poses.append(path.poses[-1])
                continue
                
            # 计算在当前段中的位置
            segment_start = path.poses[current_segment]
            segment_end = path.poses[current_segment + 1]
            segment_progress = (sample_dist - accumulated_length) / segment_lengths[current_segment]
            
            # 线性插值位置
            new_pose = PoseStamped()
            new_pose.header = path.header
            new_pose.pose.position.x = (segment_end.pose.position.x - segment_start.pose.position.x) * segment_progress + segment_start.pose.position.x
            new_pose.pose.position.y = (segment_end.pose.position.y - segment_start.pose.position.y) * segment_progress + segment_start.pose.position.y
            new_pose.pose.position.z = (segment_end.pose.position.z - segment_start.pose.position.z) * segment_progress + segment_start.pose.position.z
            
            # 保持原始朝向
            new_pose.pose.orientation = segment_start.pose.orientation
            
            resampled_path.poses.append(new_pose)
            
        return resampled_path
        
    def analyze(self, actual_path, target_path):
        """分析实际轨迹与目标路径的差异
        
        分析过程说明：
        1. 首先对实际路径和目标路径进行重新参数化，确保点分布均匀，消除采样不均匀带来的偏差
        2. 计算轨迹跟踪的时间相关指标（总时间、总距离、平均速度）
        3. 对每个实际轨迹点，找到最近的目标点，计算位置误差
        4. 统计所有误差数据，计算各种统计指标
        
        分析目的：
        1. 评估轨迹跟踪的准确性：通过位置误差反映跟踪精度
        2. 评估轨迹跟踪的效率：通过时间相关指标反映跟踪速度
        3. 评估轨迹跟踪的稳定性：通过误差的统计分布反映跟踪的稳定性
        
        Args:
            actual_path (nav_msgs.msg.Path): 实际轨迹，机器人实际行走的路径
            target_path (nav_msgs.msg.Path): 目标路径，期望机器人行走的路径
        """
        if len(actual_path.poses) == 0 or len(target_path.poses) == 0:
            rospy.logwarn("无法分析轨迹性能：实际轨迹或目标路径为空")
            return
        
        # reset performance metrics, ensure each analysis is independent
        self._reset_metrics()
        
        # resample path to ensure uniform point distribution, eliminate the bias caused by uneven sampling
        resampled_actual = self.resample_path(actual_path)
        resampled_target = self.resample_path(target_path)
        
        # calculate completion time
        # purpose: evaluate the efficiency of trajectory tracking
        # meaning: the total time used from the start of tracking to the end
        start_time = actual_path.poses[0].header.stamp
        end_time = actual_path.poses[-1].header.stamp
        self.duration = (end_time - start_time).to_sec()
        
        # calculate the total distance of the actual path
        # purpose: evaluate the completeness of trajectory tracking
        # meaning: the total distance that the robot actually walks
        for i in range(1, len(resampled_actual.poses)):
            self.total_distance += Utils.calculate_distance(
                resampled_actual.poses[i].pose.position,
                resampled_actual.poses[i-1].pose.position
            )
        # calculate the average speed
        # purpose: evaluate the efficiency of trajectory tracking
        # meaning: the distance the robot walks per unit time
        self.avg_speed = self.total_distance / self.duration if self.duration > 0 else 0.0
        
        # analyze the error of each actual trajectory point and the nearest target point
        # purpose: evaluate the accuracy of trajectory tracking
        for actual_pose in resampled_actual.poses:
            # find the nearest target point
            # purpose: establish the correspondence between the actual point and the target point
            min_dist = float('inf')
            closest_goal_pose = None
            
            for goal_pose in resampled_target.poses:
                dist = Utils.calculate_distance(
                    actual_pose.pose.position,
                    goal_pose.pose.position
                )
                if dist < min_dist:
                    min_dist = dist
                    closest_goal_pose = goal_pose
            
            if closest_goal_pose is not None:
                # calculate the distance error
                # purpose: evaluate the accuracy of position tracking
                # meaning: the Euclidean distance between the actual position and the nearest target position
                distance_error = min_dist
                self.total_distance_error += distance_error
                self.max_distance_error = max(self.max_distance_error, distance_error)
                self.min_distance_error = min(self.min_distance_error, distance_error)
                self.distance_errors.append(distance_error)  # 存储距离误差用于计算统计指标
        
        # calculate the average error
        # purpose: evaluate the overall tracking accuracy
        # meaning: reflect the average deviation of trajectory tracking
        avg_distance_error = self.total_distance_error / len(resampled_actual.poses)
        
        # calculate the advanced statistical metrics
        # purpose: evaluate the stability and consistency of tracking
        # RMSE: reflect the overall size of the error, more sensitive to larger errors
        # standard deviation: reflect the dispersion of the error, the smaller the value, the more stable the tracking
        rmse_distance = self._calculate_rmse(self.distance_errors)
        std_distance = self._calculate_std(self.distance_errors)
        
        # output the performance analysis results
        # purpose: provide a complete performance evaluation report
        # meaning: help understand the overall performance of trajectory tracking
        self._print_analysis_results(avg_distance_error, rmse_distance, std_distance)
        
    def _reset_metrics(self):
        """重置所有性能指标"""
        self.total_distance_error = 0.0
        self.max_distance_error = 0.0
        self.min_distance_error = float('inf')
        self.duration = 0.0
        self.avg_speed = 0.0
        self.total_distance = 0.0
        self.distance_errors = []
        
    def _calculate_rmse(self, errors):
        """计算均方根误差 (RMSE)"""
        if not errors:
            return 0.0
        squared_errors = [e * e for e in errors]
        mean_squared_error = sum(squared_errors) / len(errors)
        return math.sqrt(mean_squared_error)
        
    def _calculate_std(self, errors):
        """计算标准差"""
        if not errors:
            return 0.0
        mean = sum(errors) / len(errors)
        squared_diffs = [(e - mean) ** 2 for e in errors]
        variance = sum(squared_diffs) / len(errors)
        return math.sqrt(variance)
        
    def _print_analysis_results(self, avg_distance_error, rmse_distance, std_distance):
        """输出性能分析结果"""
        rospy.loginfo("=============================================")
        rospy.loginfo("轨迹跟踪性能分析:")
        rospy.loginfo(f"总跟踪时间: {self.duration:.2f} 秒")
        rospy.loginfo(f"总行驶距离: {self.total_distance:.2f} 米")
        rospy.loginfo(f"平均速度: {self.avg_speed:.2f} m/s")
        rospy.loginfo("距离误差:")
        rospy.loginfo(f"  平均误差: {avg_distance_error:.3f} m")
        rospy.loginfo(f"  最大误差: {self.max_distance_error:.3f} m")
        rospy.loginfo(f"  最小误差: {self.min_distance_error:.3f} m")
        rospy.loginfo(f"  RMSE: {rmse_distance:.3f} m")
        rospy.loginfo(f"  标准差: {std_distance:.3f} m")
        rospy.loginfo("=============================================")

class MpcPathTracer(PathTracerBase):
    def __init__(self, node_name='mpc_path_tracer_node'):
        super().__init__(node_name)
        # MPC parameters
        self.NUM_KNOTS = 31  # Number of knot points
        
        # Path and control state
        self.global_path = None
        self.stop_requested = False
        
        # MPC path publisher
        self.mpc_path_pub = rospy.Publisher('trace_path/mpc/path', Path, queue_size=10)
        
        # Topic subscribers
        self.path_sub = rospy.Subscriber('~path', Path, self.path_callback)
        self.start_sub = rospy.Subscriber('~start', Bool, self.start_callback)
        self.stop_sub = rospy.Subscriber('~stop', Bool, self.stop_callback)
        
        rospy.loginfo("MPC Path Tracer initialized. Waiting for path on ~path topic.")
        
        self.trajectory_analyzer = TrajectoryAnalyzer()
    
    def path_callback(self, path_msg):
        """Callback for receiving a path"""
        if len(path_msg.poses) <= 0:
            rospy.logwarn("Received empty path. Cannot use it.")
            return
        
        # Ensure the path is in the world frame
        if path_msg.header.frame_id != self.world_frame:
            rospy.logwarn(f"Received path with frame {path_msg.header.frame_id}, but expected '{self.world_frame}'. Converting...")
            path_msg.header.frame_id = self.world_frame
        
        self.global_path = path_msg
        rospy.loginfo(f"Received path with {len(path_msg.poses)} points.")
        
        # Publish the received path for visualization
        self.publish_path(self.global_path)
    
    def start_callback(self, msg):
        """Callback to start following the path"""
        if msg.data and (self.follow_state == FollowState.NOT_FOLLOWING or self.follow_state == FollowState.FINISHED):
            if self.global_path is None or len(self.global_path.poses) <= 0:
                rospy.logwarn("No path available. Load a path before starting.")
                return
            
            # update the max linear velocity from the ROS parameter server
            try:
                v_max_linear = rospy.get_param('~v_max_linear', 0.2)
                # verify the velocity value is in a reasonable range
                if v_max_linear <= 0 or v_max_linear > 1.0:
                    rospy.logwarn(f"Invalid max linear velocity: {v_max_linear} m/s, using default value 0.2 m/s")
                    v_max_linear = 0.2
                self.set_max_linear_velocity(v_max_linear)
                rospy.loginfo(f"Updated max linear velocity: {v_max_linear} m/s")
            except Exception as e:
                rospy.logerr(f"Failed to update max linear velocity: {e}, using default value 0.2 m/s")
                self.set_max_linear_velocity(0.2)
            
            # Start following in a separate thread to avoid blocking
            self.stop_requested = False
            self.follow_state = FollowState.FOLLOWING
            self.path.poses = []
            import threading
            self.motion_interface = rospy.get_param('~motion_interface', '/cmd_vel')
            rospy.loginfo(f"Motion interface: {self.motion_interface}")
            self.follow_thread = threading.Thread(target=self.follow, args=(self.global_path,))
            self.follow_thread.start()
            rospy.loginfo("Path following started.")
        elif msg.data and self.follow_state == FollowState.FOLLOWING:
            rospy.logwarn("Already following a path. Stop first.")
    
    def stop_callback(self, msg):
        """Callback to stop following the path"""
        if msg.data:
            self.stop_requested = True
            rospy.loginfo("Stop requested. Stopping path following...")
    
    def no_mpc_follow(self, global_path):
        """Follow path without MPC by directly publishing each point as cmd_pose_world.
        
        Args:
            global_path (nav_msgs.msg.Path): The path to follow
        """
        rate = rospy.Rate(5)  # 5Hz = 0.2s interval
        
        # Wait for start point to be set
        self.flag_reset_start_point = True
        while not rospy.is_shutdown() and self.flag_reset_start_point:
            rate.sleep()
            rospy.loginfo("Waiting for start point...")

        # Validate path
        if len(global_path.poses) <= 0:
            self.follow_state = FollowState.NOT_FOLLOWING
            return

        # First publish the path for visualization
        self.publish_path(global_path)

        # Log path info
        rospy.loginfo("-------------------------------------------------------")
        rospy.loginfo(f"Path Start Pose:\n{global_path.poses[0].pose}")
        rospy.loginfo("-------------------------------------------------------")
        rospy.loginfo(f"Path End Pose:\n{global_path.poses[-1].pose}")
        rospy.loginfo("-------------------------------------------------------")
        rospy.loginfo(f"Current yaw: {Utils.get_yaw_from_orientation(self.current_pose.orientation)}")
        rospy.loginfo(f"Path Point Counts: {len(global_path.poses)}")

        # Follow each point in the path
        for i, pose_stamped in enumerate(global_path.poses):
            if rospy.is_shutdown() or self.stop_requested:
                break

            # Get position and orientation
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            yaw = Utils.get_yaw_from_orientation(pose_stamped.pose.orientation)

            # Publish pose command
            self.publish_cmd_pose_world(x, y, yaw)
            
            # Publish current pose to realtime path
            self.path.poses.append(pose_stamped)
            self.realtime_path_pub.publish(self.path)
            
            rate.sleep()

        # Handle stop request
        if self.stop_requested:
            self.follow_state = FollowState.NOT_FOLLOWING
            self.stop_requested = False
        else:
            self.follow_state = FollowState.FINISHED
            
        rospy.sleep(1.5)
        rospy.loginfo("Path Follower Finished!")
        self.publish_cmd_vel(0, 0)  # Ensure robot stops
        rospy.loginfo("Stop moving...")
        rospy.loginfo(f"Current pose:\n{self.current_pose}")

        # Analyze trajectory performance
        if self.follow_state == FollowState.FINISHED or self.stop_requested:
            self.trajectory_analyzer.analyze(self.path, global_path)

    def follow(self, global_path):
        """Follow a given path using MPC control or direct pose control.
        
        Args:
            global_path (nav_msgs.msg.Path): The path to follow
        """
        if self.motion_interface == '/cmd_pose_world':
            self.no_mpc_follow(global_path)
            return

        # Original MPC following code continues here...
        rate = rospy.Rate(1)  # Control frequency

        # Wait for start point to be set
        self.flag_reset_start_point = True
        while not rospy.is_shutdown() and self.flag_reset_start_point:
            rate.sleep()
            rospy.loginfo("Waiting for start point...")

        # Validate path
        if len(global_path.poses) <= 0:
            self.follow_state = FollowState.NOT_FOLLOWING
            return

        # First publish the path for visualization
        self.publish_path(global_path)

        # Log path info
        rospy.loginfo("-------------------------------------------------------")
        rospy.loginfo(f"Path Start Pose:\n{global_path.poses[0].pose}")
        rospy.loginfo("-------------------------------------------------------")
        rospy.loginfo(f"Path End Pose:\n{global_path.poses[-1].pose}")
        rospy.loginfo("-------------------------------------------------------")
        rospy.loginfo(f"Current yaw: {Utils.get_yaw_from_orientation(self.current_pose.orientation)}")
        rospy.loginfo(f"Path Point Counts: {len(global_path.poses)}")

        # Path end point
        goal_pose = global_path.poses[-1].pose.position

        # MPC control loop
        point_counts = len(global_path.poses)
        index = 0
        unchanged_count = 0
        kNextPointUnchangedThreshold = 10  # Threshold to prevent robot from getting stuck

        while not rospy.is_shutdown() and index < point_counts and not self.stop_requested:
            # Find closest path point to current robot position considering yaw
            min_index = index
            min_cost = float('inf')
            yaw_weight = 0.5  # Weight for yaw difference in cost function

            for i in range(min_index, min(index + self.NUM_KNOTS, point_counts)):
                distance = Utils.calculate_distance(self.current_pose.position, global_path.poses[i].pose.position)
                ref_yaw = Utils.get_yaw_from_orientation(global_path.poses[i].pose.orientation)
                current_yaw = Utils.get_yaw_from_orientation(self.current_pose.orientation)
                yaw_diff = abs(Utils.normalize_angle(current_yaw - ref_yaw))

                # Calculate composite cost
                cost = distance * distance + yaw_weight * yaw_diff * yaw_diff
                if cost < min_cost:
                    min_cost = cost
                    min_index = i

            # Update index and check if stuck
            if min_index == index:
                unchanged_count += 1
            else:
                unchanged_count = 0

            index = min_index

            # Prevent robot from getting stuck
            if unchanged_count >= kNextPointUnchangedThreshold:
                unchanged_count = 0
                if index < point_counts:
                    index += 1
                else:
                    rospy.loginfo("Unable to find a path to the next point. Aborting...")
                    self.follow_state = FollowState.NOT_FOLLOWING
                    break

            # Check if reached goal
            current_yaw = Utils.get_yaw_from_orientation(self.current_pose.orientation)
            goal_yaw = Utils.get_yaw_from_orientation(global_path.poses[-1].pose.orientation)
            yaw_error = abs(Utils.normalize_angle(current_yaw - goal_yaw))
            
            if (0.1 > Utils.calculate_distance(goal_pose, self.current_pose.position) and 
                index >= point_counts - 2 and
                yaw_error < 0.03):
                rospy.loginfo("Stop moving...")
                self.publish_cmd_vel(0, 0)
                self.follow_state = FollowState.FINISHED
                break

            # Add deceleration near goal
            distance_to_goal = Utils.calculate_distance(goal_pose, self.current_pose.position)
            deceleration_distance = 1.0
            deceleration_angular_distance = 0.785
            linear_velocity_scale = 1.0
            angular_velocity_scale = 1.0

            if distance_to_goal < deceleration_distance:
                linear_velocity_scale = max(0.3, distance_to_goal / deceleration_distance)
            
            if yaw_error < deceleration_angular_distance:
                angular_velocity_scale = max(0.3, yaw_error / deceleration_angular_distance)

            # Transform reference trajectory to base_link frame
            ref_x = np.zeros(self.NUM_KNOTS)
            ref_y = np.zeros(self.NUM_KNOTS)
            ref_yaw = np.zeros(self.NUM_KNOTS)

            for i in range(self.NUM_KNOTS):
                global_index = min(index + i, point_counts - 1)
                local_point = transform_to_base_link(self.current_pose, global_path.poses[global_index].pose.position)
                ref_x[i] = local_point.x
                ref_y[i] = local_point.y
                ref_yaw[i] = Utils.normalize_angle(Utils.get_yaw_from_orientation(global_path.poses[global_index].pose.orientation) - current_yaw)

            # Setup MPC optimization problem
            prog = MathematicalProgram()

            # Decision variables
            x = prog.NewContinuousVariables(self.NUM_KNOTS, "x")
            y = prog.NewContinuousVariables(self.NUM_KNOTS, "y")
            yaw = prog.NewContinuousVariables(self.NUM_KNOTS, "yaw")
            v = prog.NewContinuousVariables(self.NUM_KNOTS, "v")
            yaw_dot = prog.NewContinuousVariables(self.NUM_KNOTS, "yaw_dot")

            # Initial constraints
            prog.AddBoundingBoxConstraint(0.0, 0.0, x[0])
            prog.AddBoundingBoxConstraint(0.0, 0.0, y[0])
            prog.AddBoundingBoxConstraint(0.0, 0.0, yaw[0])

            # Velocity constraints
            for i in range(self.NUM_KNOTS):
                if self.motion_interface == '/cmd_vel':
                    prog.AddBoundingBoxConstraint(-self.v_max_linear, self.v_max_linear, v[i])
                    prog.AddBoundingBoxConstraint(-self.v_max_angular, self.v_max_angular, yaw_dot[i])
                elif self.motion_interface == '/cmd_pose' or self.motion_interface == '/cmd_pose_world':
                    # told by wangzhengtao@lejurobot.com
                    prog.AddBoundingBoxConstraint(-0.4, 0.4, v[i])
                    prog.AddBoundingBoxConstraint(-0.4, 0.4, yaw_dot[i])

            # Dynamic constraints
            dt = 0.1  # Time step
            for i in range(self.NUM_KNOTS - 1):
                prog.AddConstraint(x[i + 1] == x[i] + v[i] * np.cos(yaw[i]) * dt)
                prog.AddConstraint(y[i + 1] == y[i] + v[i] * np.sin(yaw[i]) * dt)
                prog.AddConstraint(yaw[i + 1] == yaw[i] + yaw_dot[i] * dt)

            # Cost function
            for i in range(self.NUM_KNOTS):
                prog.AddQuadraticCost((x[i] - ref_x[i])**2 + (y[i] - ref_y[i])**2)
                prog.AddQuadraticCost(0.6 * (yaw[i] - ref_yaw[i])**2)
                prog.AddQuadraticCost(0.1 * v[i]**2)
                prog.AddQuadraticCost(0.2 * yaw_dot[i]**2)

            # Solve optimization problem
            solver = SnoptSolver()
            result = Solve(prog)

            if result.is_success():
                optimal_x = result.GetSolution(x)
                optimal_y = result.GetSolution(y)
                optimal_yaw = result.GetSolution(yaw)
                optimal_v = result.GetSolution(v)
                optimal_yaw_dot = result.GetSolution(yaw_dot)

                # Apply velocity scaling
                scaled_v = optimal_v[0] * linear_velocity_scale
                scaled_yaw_dot = optimal_yaw_dot[0] * angular_velocity_scale

                # Publish control commands
                if self.motion_interface == '/cmd_vel':
                    self.publish_cmd_vel(scaled_v, scaled_yaw_dot)
                elif self.motion_interface == '/cmd_pose':
                    self.publish_cmd_pose(optimal_x[3], optimal_y[3], optimal_yaw[3])
                elif self.motion_interface == '/cmd_pose_world':
                    self.publish_cmd_pose_world(
                        self.current_pose.position.x + optimal_x[3], 
                        self.current_pose.position.y + optimal_y[3],
                        current_yaw + optimal_yaw[3])

                # Publish MPC path for visualization
                self.publish_mpc_path(optimal_x, optimal_y, optimal_yaw)
            else:
                rospy.logerr(f"Optimization failed to find a solution: {result.get_solution_result()}")
                self.follow_state = FollowState.NOT_FOLLOWING
                break

            # Publish fixed path
            self.publish_path(global_path)
            rospy.sleep(dt)

        # Handle stop request
        if self.stop_requested:
            self.follow_state = FollowState.NOT_FOLLOWING
            self.stop_requested = False
            
        rospy.sleep(1.5)
        rospy.loginfo("Path Follower Finished!")
        self.publish_cmd_vel(0, 0)  # Ensure robot stops
        rospy.loginfo("Stop moving...")
        rospy.loginfo(f"Current pose:\n{self.current_pose}")
        
        # If we didn't explicitly set the state to FINISHED or NOT_FOLLOWING due to stop request,
        # set it based on whether we reached the end of the path
        if self.follow_state == FollowState.FOLLOWING:
            if index >= point_counts - 1:
                self.follow_state = FollowState.FINISHED
            else:
                self.follow_state = FollowState.NOT_FOLLOWING

        # Analyze trajectory performance
        if self.follow_state == FollowState.FINISHED or self.stop_requested:
            self.trajectory_analyzer.analyze(self.path, global_path)

    def publish_mpc_path(self, x, y, yaw):
        # Create a path message
        mpc_path = Path()
        mpc_path.header.stamp = rospy.Time.now()
        mpc_path.header.frame_id = self.world_frame
        
        # Get current robot pose
        q = [self.current_pose.orientation.x, self.current_pose.orientation.y,
             self.current_pose.orientation.z, self.current_pose.orientation.w]
        rot_matrix = tf.transformations.quaternion_matrix(q)[:3, :3]
        
        # Get current robot yaw
        current_yaw = Utils.get_yaw_from_orientation(self.current_pose.orientation)
        
        # Iterate through all optimization results
        for i in range(self.NUM_KNOTS):
            # Transform points from local to global coordinate system
            local_vec = np.array([x[i], y[i], 0.0])
            global_vec = rot_matrix.dot(local_vec) + np.array([
                self.current_pose.position.x,
                self.current_pose.position.y,
                0.0
            ])
            
            # Create a new path point
            pose_stamped = PoseStamped()
            pose_stamped.header = mpc_path.header
            pose_stamped.pose.position.x = global_vec[0]
            pose_stamped.pose.position.y = global_vec[1]
            pose_stamped.pose.position.z = 0.3
            
            # Convert local yaw to global yaw
            global_yaw = yaw[i] + current_yaw
            
            # Set orientation
            q = tf.transformations.quaternion_from_euler(0, 0, global_yaw)
            pose_stamped.pose.orientation.x = q[0]
            pose_stamped.pose.orientation.y = q[1]
            pose_stamped.pose.orientation.z = q[2]
            pose_stamped.pose.orientation.w = q[3]
            
            # Add to path message
            mpc_path.poses.append(pose_stamped)
        
        # Publish path
        self.mpc_path_pub.publish(mpc_path)

if __name__ == "__main__":
    try:
        rospy.init_node('mpc_path_tracer_node')
        tracer = MpcPathTracer()
        rospy.loginfo("MPC Path Tracer node started. Waiting for path on ~path topic...")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
