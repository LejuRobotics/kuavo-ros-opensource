#!/usr/bin/env python3

import rospy
import math
import tf
import tf2_ros
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Bool

home_pose = None

class BaseParams:
    def __init__(self, v_max_linear=0.3, dt=0.1):
        self.v_max_linear = v_max_linear  # m/s
        self.dt = dt       # s

class CircleParams(BaseParams):
    def __init__(self, radius=2.0, v_max_linear=0.3, dt=0.1):
        super().__init__(v_max_linear, dt)
        self.radius = radius

class SquareParams(BaseParams):
    def __init__(self, side_length=4.0, v_max_linear=0.3, dt=0.1, yaw_offset=0.0):
        super().__init__(v_max_linear, dt)
        self.side_length = side_length
        self.yaw_offset = yaw_offset  

class SCurveParams(BaseParams):
    def __init__(self, length=4.0, amplitude=2.0, v_max_linear=0.3, dt=0.1):
        super().__init__(v_max_linear, dt)
        self.length = length      # S形曲线的长度
        self.amplitude = amplitude  # S形曲线的振幅

class TriangleParams(BaseParams):
    def __init__(self, side_length=2.0, v_max_linear=0.3, dt=0.1, yaw_offset=0.0):
        super().__init__(v_max_linear, dt)
        self.side_length = side_length
        self.yaw_offset = yaw_offset  

class LineParams(BaseParams):
    def __init__(self, length=2.0, v_max_linear=0.3, dt=0.1, yaw_offset=0.0):
        super().__init__(v_max_linear, dt)
        self.length = length        # 直线长度
        self.yaw_offset = yaw_offset  

class PathGenerator:
    def __init__(self, world_frame='odom', robot_frame='base_link', home_frame='home'):
        self.world_frame = world_frame
        self.robot_frame = robot_frame
        self.home_frame = home_frame
        self.tf_listener = tf.TransformListener()
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        
    def get_robot_pose(self):
        """Get current robot pose from TF"""
        try:
            self.tf_listener.waitForTransform(self.world_frame, self.robot_frame, rospy.Time(0), rospy.Duration(1.0))
            (trans, rot) = self.tf_listener.lookupTransform(self.world_frame, self.robot_frame, rospy.Time(0))
            return trans, rot
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf2_ros.TransformException) as e:
            rospy.logwarn(f"Failed to get robot pose: {e}")
            raise Exception(f"无法获取机器人位姿: {e}")
    
    def set_home_pose(self):
        """Get home pose from TF or use current robot pose as home pose"""
        try:
            # Try to get home pose from TF first
            self.tf_listener.waitForTransform(self.world_frame, self.home_frame, rospy.Time(0), rospy.Duration(1.0))
            (trans, rot) = self.tf_listener.lookupTransform(self.world_frame, self.home_frame, rospy.Time(0))
            rospy.loginfo("Home pose found in TF tree")
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf2_ros.TransformException) as e:
            # If home pose not found, use current robot pose as home pose
            rospy.loginfo("Home pose not found, using current robot pose as home pose")
            trans, rot = self.get_robot_pose()
            
            # Publish static transform for home pose
            static_transform = TransformStamped()
            static_transform.header.stamp = rospy.Time.now()
            static_transform.header.frame_id = self.world_frame
            static_transform.child_frame_id = self.home_frame
            static_transform.transform.translation.x = trans[0]
            static_transform.transform.translation.y = trans[1]
            static_transform.transform.translation.z = trans[2]
            static_transform.transform.rotation.x = rot[0]
            static_transform.transform.rotation.y = rot[1]
            static_transform.transform.rotation.z = rot[2]
            static_transform.transform.rotation.w = rot[3]
            self.static_broadcaster.sendTransform(static_transform)
            rospy.loginfo(f"Published static transform from {self.world_frame} to {self.home_frame}")
        
        global home_pose
        home_pose = PoseStamped()
        home_pose.header.frame_id = self.world_frame
        home_pose.header.stamp = rospy.Time.now()
        home_pose.pose.position.x = trans[0]
        home_pose.pose.position.y = trans[1]
        home_pose.pose.position.z = trans[2]
        home_pose.pose.orientation.x = rot[0]
        home_pose.pose.orientation.y = rot[1]
        home_pose.pose.orientation.z = rot[2]
        home_pose.pose.orientation.w = rot[3]
        return home_pose

    def create_test_path(self, params=None):
        """Create a circular test path starting from robot's current position"""
        if params is None:
            params = CircleParams()
            
        path = Path()
        path.header.frame_id = self.world_frame
        path.header.stamp = rospy.Time.now()
        
        # Get robot's current pose
        trans, rot = self.get_robot_pose()
        if trans is None:
            rospy.logerr("Could not get robot pose")
            return None
            
        start_x, start_y = trans[0], trans[1]
        start_yaw = tf.transformations.euler_from_quaternion(rot)[2]
        
        # Calculate number of points based on path length and point interval
        point_interval = params.v_max_linear * params.dt  # meters between points
        circle_length = 2 * math.pi * params.radius
        num_points = int(circle_length / point_interval)
        
        # 添加起始点（机器人当前位置）
        pose_stamped = PoseStamped()
        pose_stamped.header = path.header
        pose_stamped.pose.position.x = start_x
        pose_stamped.pose.position.y = start_y
        pose_stamped.pose.position.z = 0.0
        pose_stamped.pose.orientation.x = rot[0]
        pose_stamped.pose.orientation.y = rot[1]
        pose_stamped.pose.orientation.z = rot[2]
        pose_stamped.pose.orientation.w = rot[3]
        path.poses.append(pose_stamped)
        
        # 生成圆形路径，从机器人当前位置开始
        for i in range(num_points):
            angle = 2.0 * math.pi * i / num_points
            x = start_x + params.radius * (1 - math.cos(angle))
            y = start_y + params.radius * math.sin(angle)
            yaw = math.atan2(math.cos(angle), math.sin(angle))  # 切线方向
            
            pose_stamped = PoseStamped()
            pose_stamped.header = path.header
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            pose_stamped.pose.position.z = 0.0
            
            q = tf.transformations.quaternion_from_euler(0, 0, yaw)
            pose_stamped.pose.orientation.x = q[0]
            pose_stamped.pose.orientation.y = q[1]
            pose_stamped.pose.orientation.z = q[2]
            pose_stamped.pose.orientation.w = q[3]
            
            path.poses.append(pose_stamped)
        
        return path

    def create_square_path(self, params=None):
        """Create a square test path starting from robot's current position"""
        if params is None:
            params = SquareParams()
            
        path = Path()
        path.header.frame_id = self.world_frame
        path.header.stamp = rospy.Time.now()
        
        # Get robot's current pose
        trans, rot = self.get_robot_pose()
        if trans is None:
            rospy.logerr("Could not get robot pose")
            return None
            
        current_x, current_y = trans[0], trans[1]
        current_yaw = tf.transformations.euler_from_quaternion(rot)[2]  # 当前朝向角
        total_yaw = current_yaw + params.yaw_offset  # 总偏转角度 = 当前yaw + 额外偏转
        
        # 计算每一步的距离
        step_distance = params.v_max_linear * params.dt
        
        # 定义正方形的顶点（相对于起始点）
        # 先定义未旋转的正方形顶点
        square_points_local = [
            (0, 0),                                        # 起点
            (params.side_length, 0),                      # 右
            (params.side_length, params.side_length),     # 右上
            (0, params.side_length),                      # 左上
            (0, 0)                                        # 回到起点
        ]
        
        # 将局部坐标点旋转并平移到世界坐标系
        square_points = []
        for point in square_points_local:
            # 应用旋转变换
            x_rot = point[0] * math.cos(total_yaw) - point[1] * math.sin(total_yaw)
            y_rot = point[0] * math.sin(total_yaw) + point[1] * math.cos(total_yaw)
            # 平移到世界坐标系
            x_world = x_rot + current_x
            y_world = y_rot + current_y
            square_points.append((x_world, y_world))
        
        # 生成路径点
        for i in range(1, len(square_points)):
            prev_point = square_points[i - 1]
            next_point = square_points[i]
            
            # 计算当前边的向量
            dx = next_point[0] - prev_point[0]
            dy = next_point[1] - prev_point[1]
            distance = math.sqrt(dx * dx + dy * dy)
            yaw = math.atan2(dy, dx)  # 计算当前边的朝向
            
            # 计算需要插入的点数
            num_steps = int(distance / step_distance)
            if num_steps <= 0:
                num_steps = 1
                
            # 在两个顶点之间插入点
            for j in range(num_steps + 1):
                t = j / float(num_steps)
                x = prev_point[0] + t * dx
                y = prev_point[1] + t * dy
                
                # 创建路径点
                pose_stamped = PoseStamped()
                pose_stamped.header = path.header
                pose_stamped.pose.position.x = x
                pose_stamped.pose.position.y = y
                pose_stamped.pose.position.z = 0.0
                
                # 设置朝向（考虑当前边的朝向）
                q = tf.transformations.quaternion_from_euler(0, 0, yaw)
                pose_stamped.pose.orientation.x = q[0]
                pose_stamped.pose.orientation.y = q[1]
                pose_stamped.pose.orientation.z = q[2]
                pose_stamped.pose.orientation.w = q[3]
                
                path.poses.append(pose_stamped)
        
        return path

    def create_scurve_path(self, params=None):
        """Create an S-curve path starting from robot's current position"""
        if params is None:
            params = SCurveParams()
            
        path = Path()
        path.header.frame_id = self.world_frame
        path.header.stamp = rospy.Time.now()
        
        # Get robot's current pose
        trans, rot = self.get_robot_pose()
        if trans is None:
            rospy.logerr("Could not get robot pose")
            return None
            
        # 当前位置
        current_x = trans[0]
        current_y = trans[1]
        current_theta = tf.transformations.euler_from_quaternion(rot)[2]  # 当前朝向角

        # 计算每一步的距离
        length = params.length
        amplitude = params.amplitude
        v_max_linear = params.v_max_linear
        dt = params.dt
        step_distance = v_max_linear * dt

        # 总长度
        total_length = length * 2  # S形曲线的实际长度为两倍的长度

        # 插入路径中的点
        x = current_x
        y = current_y
        t = 0.0

        while t < total_length:
            # 计算"S"形曲线的Y坐标偏移量
            y_offset = amplitude * math.sin(math.pi * t / length) * math.sin(math.pi * t / length)
            dy_dt = (2 * amplitude * math.pi / length) * math.cos(math.pi * t / length) * math.sin(math.pi * t / length)

            # 创建路径点
            pose_stamped = PoseStamped()
            pose_stamped.header = path.header
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y + y_offset
            pose_stamped.pose.position.z = 0.0

            # 计算该点的朝向
            yaw = math.atan2(dy_dt, 1.0)
            yaw = yaw + current_theta  # 考虑机器人初始朝向
            q = tf.transformations.quaternion_from_euler(0, 0, yaw)
            pose_stamped.pose.orientation.x = q[0]
            pose_stamped.pose.orientation.y = q[1]
            pose_stamped.pose.orientation.z = q[2]
            pose_stamped.pose.orientation.w = q[3]

            path.poses.append(pose_stamped)

            # 更新位置
            x += step_distance
            t += step_distance

        # 根据机器人的朝向旋转路径中的所有点
        for pose in path.poses:
            x = pose.pose.position.x - current_x
            y = pose.pose.position.y - current_y
            # 应用旋转变换
            x_rot = x * math.cos(current_theta) - y * math.sin(current_theta)
            y_rot = x * math.sin(current_theta) + y * math.cos(current_theta)
            pose.pose.position.x = x_rot + current_x
            pose.pose.position.y = y_rot + current_y

        return path

    def create_triangle_path(self, params=None):
        """Create an equilateral triangle path starting from robot's current position"""
        if params is None:
            params = TriangleParams()
            
        path = Path()
        path.header.frame_id = self.world_frame
        path.header.stamp = rospy.Time.now()
        
        # Get robot's current pose
        trans, rot = self.get_robot_pose()
        if trans is None:
            rospy.logerr("Could not get robot pose")
            return None
            
        current_x, current_y = trans[0], trans[1]
        current_yaw = tf.transformations.euler_from_quaternion(rot)[2]  # 当前朝向角
        total_yaw = current_yaw + params.yaw_offset  # 总偏转角度 = 当前yaw + 额外偏转
        
        # 计算每一步的距离
        step_distance = params.v_max_linear * params.dt
        
        # 计算等边三角形的顶点（相对于起始点）
        # 等边三角形的高 = 边长 * sqrt(3)/2
        height = params.side_length * math.sqrt(3) / 2
        
        # 定义三角形的顶点（相对于起始点）
        triangle_points_local = [
            (0, 0),                                    # 起点
            (params.side_length, 0),                   # 右
            (params.side_length/2, height),            # 顶点
            (0, 0)                                     # 回到起点
        ]
        
        # 将局部坐标点旋转并平移到世界坐标系
        triangle_points = []
        for point in triangle_points_local:
            # 应用旋转变换
            x_rot = point[0] * math.cos(total_yaw) - point[1] * math.sin(total_yaw)
            y_rot = point[0] * math.sin(total_yaw) + point[1] * math.cos(total_yaw)
            # 平移到世界坐标系
            x_world = x_rot + current_x
            y_world = y_rot + current_y
            triangle_points.append((x_world, y_world))
        
        # 生成路径点
        for i in range(1, len(triangle_points)):
            prev_point = triangle_points[i - 1]
            next_point = triangle_points[i]
            
            # 计算当前边的向量
            dx = next_point[0] - prev_point[0]
            dy = next_point[1] - prev_point[1]
            distance = math.sqrt(dx * dx + dy * dy)
            yaw = math.atan2(dy, dx)  # 计算当前边的朝向
            
            # 计算需要插入的点数
            num_steps = max(int(distance / step_distance), 1)
            
            # 在两个顶点之间插入点
            for j in range(num_steps + 1):
                t = j / float(num_steps)
                x = prev_point[0] + t * dx
                y = prev_point[1] + t * dy
                
                # 创建路径点
                pose_stamped = PoseStamped()
                pose_stamped.header = path.header
                pose_stamped.pose.position.x = x
                pose_stamped.pose.position.y = y
                pose_stamped.pose.position.z = 0.0
                
                # 设置朝向（考虑当前边的朝向）
                q = tf.transformations.quaternion_from_euler(0, 0, yaw)
                pose_stamped.pose.orientation.x = q[0]
                pose_stamped.pose.orientation.y = q[1]
                pose_stamped.pose.orientation.z = q[2]
                pose_stamped.pose.orientation.w = q[3]
                
                path.poses.append(pose_stamped)
        
        return path

    def create_line_path(self, params=None):
        """Create a straight line path starting from robot's current position"""
        if params is None:
            params = LineParams()
        
        path = Path()
        path.header.frame_id = self.world_frame
        path.header.stamp = rospy.Time.now()
        
        # Get robot's current pose
        trans, rot = self.get_robot_pose()
        if trans is None:
            rospy.logerr("Could not get robot pose")
            return None
        
        current_x, current_y = trans[0], trans[1]
        current_yaw = tf.transformations.euler_from_quaternion(rot)[2]  # 当前朝向角
        total_yaw = current_yaw + params.yaw_offset  # 总偏转角度 = 当前yaw + 额外偏转
        
        # 计算每一步的距离
        step_distance = params.v_max_linear * params.dt
        
        # 计算直线终点（在机器人当前朝向的基础上）
        end_x = current_x + params.length * math.cos(total_yaw)
        end_y = current_y + params.length * math.sin(total_yaw)
        
        # 计算需要插入的点数
        distance = params.length
        num_steps = max(int(distance / step_distance), 1)
        
        # 生成路径点
        for i in range(num_steps + 1):
            t = i / float(num_steps)
            # 线性插值计算位置
            x = current_x + t * (end_x - current_x)
            y = current_y + t * (end_y - current_y)
            
            # 创建路径点
            pose_stamped = PoseStamped()
            pose_stamped.header = path.header
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            pose_stamped.pose.position.z = 0.0
            
            # 设置朝向（保持恒定）
            q = tf.transformations.quaternion_from_euler(0, 0, total_yaw)
            pose_stamped.pose.orientation.x = q[0]
            pose_stamped.pose.orientation.y = q[1]
            pose_stamped.pose.orientation.z = q[2]
            pose_stamped.pose.orientation.w = q[3]
            
            path.poses.append(pose_stamped)
        
        return path

class PathGeneratorFactory:
    """路径生成器工厂类"""
    
    @staticmethod
    def get_params_config():
        """获取各类型路径的参数配置"""
        return {
            'line': {
                'params_class': LineParams,
                'create_method': 'create_line_path',
                'params': {
                    'length': ('length', 2.0),
                    'yaw_offset': ('yaw_offset', 0.0, math.radians)
                }
            },
            'circle': {
                'params_class': CircleParams,
                'create_method': 'create_test_path',
                'params': {
                    'radius': ('radius', 2.0)
                }
            },
            'square': {
                'params_class': SquareParams,
                'create_method': 'create_square_path',
                'params': {
                    'side_length': ('side_length', 4.0),
                    'yaw_offset': ('yaw_offset', 0.0, math.radians)
                }
            },
            'triangle': {
                'params_class': TriangleParams,
                'create_method': 'create_triangle_path',
                'params': {
                    'side_length': ('side_length', 2.0),
                    'yaw_offset': ('yaw_offset', 0.0, math.radians)
                }
            },
            'scurve': {
                'params_class': SCurveParams,
                'create_method': 'create_scurve_path',
                'params': {
                    'length': ('length', 4.0),
                    'amplitude': ('amplitude', 2.0)
                }
            }
        }

    @staticmethod
    def create_path(path_type, path_generator, dt):
        """
        创建指定类型的路径
        
        Args:
            path_type (str): 路径类型
            path_generator (PathGenerator): 路径生成器实例
            dt (float): 时间步长
            
        Returns:
            Path: 生成的路径对象
        """
        configs = PathGeneratorFactory.get_params_config()
        
        if path_type not in configs:
            rospy.logerr(f"Unknown path type: {path_type}")
            return None
            
        # get the max linear velocity of the path type from the ROS parameter server
        v_max_linear = rospy.get_param(f'/mpc_path_tracer_node/path_velocities/{path_type}',0.2) 
        rospy.set_param('/mpc_path_tracer_node/v_max_linear', v_max_linear)
        config = configs[path_type]
        param_values = {}
        
        # get the parameters from the ROS parameter server
        for param_name, param_config in config['params'].items():
            ros_param_name = f'/mpc_path_tracer_node/path_generators/{path_type}/{param_config[0]}'
            value = rospy.get_param(ros_param_name, param_config[1])
            
            # if there is a conversion function (like math.radians), apply it
            if len(param_config) > 2:
                value = param_config[2](value)
                
            param_values[param_name] = value
        
        # create the parameter object
        params = config['params_class'](
            v_max_linear=v_max_linear,
            dt=dt,
            **param_values
        )
        
        # generate the path
        create_method = getattr(path_generator, config['create_method'])
        path = create_method(params)
        
        return path

def start_mpc_tracer(path_type='circle'):
    """Start the MPC path tracer with a given path type"""
    
    # get the frame ID and parameters from the ROS parameter server
    world_frame = rospy.get_param('/mpc_path_tracer_node/world_frame', 'odom')
    robot_frame = rospy.get_param('/mpc_path_tracer_node/robot_frame', 'base_link')
    home_frame = rospy.get_param('/mpc_path_tracer_node/home_frame', 'home')
    dt = rospy.get_param('/mpc_path_tracer_node/dt', 0.1)
    
    rospy.loginfo(f"Using world frame: '{world_frame}', robot frame: '{robot_frame}', "
                  f"dt: {dt} s")
    
    # create the path generator
    path_generator = PathGenerator(world_frame, robot_frame, home_frame)
    global home_pose
    if home_pose is None:
        home_pose = path_generator.set_home_pose()
    
    # use the factory class to generate the path
    path = PathGeneratorFactory.create_path(path_type, path_generator, dt)
    
    if path is None:
        rospy.logerr("Failed to generate path")
        return
    
    # Create publishers for path and control commands
    path_pub = rospy.Publisher('/mpc_path_tracer_node/path', Path, queue_size=10)
    start_pub = rospy.Publisher('/mpc_path_tracer_node/start', Bool, queue_size=10)
    test_path_pub = rospy.Publisher('test_path', Path, queue_size=10)
    
    # Wait for publishers to connect
    rospy.sleep(1.0)
    
    # Publish the path
    rospy.loginfo(f"Sending {path_type} path to MPC path tracer...")
    path_pub.publish(path)
    test_path_pub.publish(path)
    
    # Give time for the path to be received
    rospy.sleep(0.5)
    
    # Start the path follower
    rospy.loginfo("Starting MPC path follower...")
    start_msg = Bool()
    start_msg.data = True
    start_pub.publish(start_msg)
    
    rospy.loginfo("Path sent and follower started.")

def stop_mpc_tracer():
    """Stop the MPC path tracer"""
    if not rospy.core.is_initialized():
        rospy.init_node('mpc_client_example', anonymous=True)
    
    # Create publisher for stop command
    stop_pub = rospy.Publisher('/mpc_path_tracer_node/stop', Bool, queue_size=10)
    
    # Wait for publisher to connect
    rospy.sleep(1.0)
    
    # Send stop command
    rospy.loginfo("Sending stop command to MPC path tracer...")
    stop_msg = Bool()
    stop_msg.data = True
    stop_pub.publish(stop_msg)
    
    rospy.loginfo("Stop command sent.")

def back_to_home():
    """Generate a straight line path from current position to home pose"""
    if not rospy.core.is_initialized():
        rospy.init_node('mpc_client_example', anonymous=True)
    
    # Get frame ID and parameters from ROS parameter server
    world_frame = rospy.get_param('/mpc_path_tracer_node/world_frame', 'odom')
    robot_frame = rospy.get_param('/mpc_path_tracer_node/robot_frame', 'base_link')
    home_frame = rospy.get_param('/mpc_path_tracer_node/home_frame', 'home')
    v_max_linear = rospy.get_param('/mpc_path_tracer_node/v_max_linear', 0.4)  # default v_max_linear 0.4 m/s
    dt = rospy.get_param('/mpc_path_tracer_node/dt', 0.1)       # default dt 0.1 s
    
    # Set motion interface to cmd_vel for back to home
    rospy.set_param('/mpc_path_tracer_node/motion_interface', '/cmd_vel')
    
    rospy.loginfo(f"Back to home: Using world frame: '{world_frame}', robot frame: '{robot_frame}'")
    
    # Create path generator and get current robot pose
    path_generator = PathGenerator(world_frame, robot_frame, home_frame)
    trans, rot = path_generator.get_robot_pose()
    
    if trans is None:
        rospy.logerr("Could not get robot pose")
        return

    global home_pose

    
    # Create a path message
    path = Path()
    path.header.frame_id = world_frame
    path.header.stamp = rospy.Time.now()
    
    # Current position
    current_x, current_y = trans[0], trans[1]
    
    # Home position from home_pose
    home_x = home_pose.pose.position.x
    home_y = home_pose.pose.position.y
    home_yaw = tf.transformations.euler_from_quaternion([
        home_pose.pose.orientation.x,
        home_pose.pose.orientation.y,
        home_pose.pose.orientation.z,
        home_pose.pose.orientation.w
    ])[2]  # Get yaw from quaternion
    
    # Create a straight line from current position to home pose
    dx = home_x - current_x
    dy = home_y - current_y
    distance = math.sqrt(dx**2 + dy**2)
    step_distance = v_max_linear * dt
    num_steps = max(int(distance / step_distance), 2)
    
    for i in range(num_steps + 1):
        t = i / float(num_steps)
        # Linear interpolation from current position to home position
        x = current_x + dx * t
        y = current_y + dy * t
        
        # Calculate orientation (pointing towards the next point)
        if i < num_steps:  # For all points except the last one
            next_x = current_x + dx * ((i+1) / float(num_steps))
            next_y = current_y + dy * ((i+1) / float(num_steps))
            path_dx = next_x - x
            path_dy = next_y - y
            yaw = math.atan2(path_dy, path_dx)
        else:  # At home position, use home orientation
            yaw = home_yaw
        
        # Create pose
        pose_stamped = PoseStamped()
        pose_stamped.header = path.header
        pose_stamped.pose.position.x = x
        pose_stamped.pose.position.y = y
        pose_stamped.pose.position.z = 0.0
        
        # Set orientation
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        pose_stamped.pose.orientation.x = q[0]
        pose_stamped.pose.orientation.y = q[1]
        pose_stamped.pose.orientation.z = q[2]
        pose_stamped.pose.orientation.w = q[3]
        
        path.poses.append(pose_stamped)
    
    # Add some points at the end to ensure the robot reaches and stays at the final pose
    for i in range(15):
        path.poses.append(path.poses[-1])

    # Publish the path
    path_pub = rospy.Publisher('/mpc_path_tracer_node/path', Path, queue_size=10)
    start_pub = rospy.Publisher('/mpc_path_tracer_node/start', Bool, queue_size=10)
    test_path_pub = rospy.Publisher('test_path', Path, queue_size=10)
    
    # Wait for publishers to connect
    rospy.sleep(1.0)
    
    # Publish the path
    rospy.loginfo(f"Sending back-to-home path to MPC path tracer (target: x={home_x:.2f}, y={home_y:.2f}, yaw={math.degrees(home_yaw):.1f}°)...")
    path_pub.publish(path)
    test_path_pub.publish(path)
    
    # Give time for the path to be received
    rospy.sleep(0.5)
    
    # Start the path follower
    rospy.loginfo("Starting MPC path follower...")
    start_msg = Bool()
    start_msg.data = True
    start_pub.publish(start_msg)
    
    rospy.loginfo("Back-to-home path sent and follower started.")

# def back_to_home():
#     pass

if __name__ == '__main__':
    rospy.init_node('mpc_client_example', anonymous=True)
    import sys
    
    try:
        # Default to circle path if no argument provided
        path_type = 'circle'
        command = 'start'
        
        # Parse command line arguments
        if len(sys.argv) > 1:
            command = sys.argv[1].lower()
        
        if len(sys.argv) > 2:
            path_type = sys.argv[2].lower()
        
        if command == 'start':
            start_mpc_tracer(path_type)
        elif command == 'stop':
            stop_mpc_tracer()
        else:
            print("Usage: mpc_client_example.py [start|stop] [line|circle|square|triangle|scurve]")
    
    except rospy.ROSInterruptException:
        pass 