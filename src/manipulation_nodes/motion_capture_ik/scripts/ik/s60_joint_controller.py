#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
S60关节控制器 - 简化版
"""

import rospy
import numpy as np
import time
import argparse
import sys
import signal
import os
import traceback
from geometry_msgs.msg import Pose, Point, Quaternion
from kuavo_msgs.msg import jointCmd, sensorsData
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import pydrake
from pydrake.multibody.inverse_kinematics import InverseKinematics
from pydrake.math import RotationMatrix
from pydrake.common.eigen_geometry import Quaternion
from pydrake.solvers import Solve, SnoptSolver

# 导入重力补偿库
try:
    sys.path.append('/opt/drake/lib/python3.8/site-packages')
    from pydrake.all import *
    DRAKE_AVAILABLE = True
    print("\033[34mDrake导入成功\033[0m")
except ImportError:
    DRAKE_AVAILABLE = False
    print("\031[33mDrake导入失败\031[0m")

try:
    import pinocchio
    PINOCCHIO_AVAILABLE = True
    print("\033[34mPinocchio导入成功\033[0m")
except ImportError:
    PINOCCHIO_AVAILABLE = False

# 导入底盘控制接口
try:
    # 添加当前目录到Python路径
    import sys
    import os
    current_dir = os.path.dirname(os.path.abspath(__file__))
    if current_dir not in sys.path:
        sys.path.insert(0, current_dir)
    
    from chassis_move import ChassisController
    CHASSIS_CONTROLLER_AVAILABLE = True
except ImportError:
    CHASSIS_CONTROLLER_AVAILABLE = False
    rospy.logwarn("chassis_move模块不可用，底盘控制功能将不可用")

# 全局控制变量
running = True

def signal_handler(signum, frame):
    global running
    print(f"\n接收到信号 {signum}，正在退出...")
    running = False
    rospy.signal_shutdown("接收到退出信号")
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

# 控制参数列表 - 可以通过修改这些列表来调整对应关节的PID参数
# 使用方法：直接修改下面的KP_LIST和KD_LIST中的值，然后重新运行脚本

# 位置控制增益列表 (kp)
KP_LIST = [
    # 腿部关节 (4个)
    300.0, 300.0, 300.0, 300.0,  # knee_joint, leg_joint, waist_pitch_joint, waist_yaw_joint
    # 左臂关节 (7个)
    300.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0,  # zarm_l1_joint to zarm_l7_joint
    # 右臂关节 (7个)
    300.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0,  # zarm_r1_joint to zarm_r7_joint
    # 头部关节 (2个)
    50.0, 50.0  # zhead_1_joint, zhead_2_joint
]

# 速度控制增益列表 (kd)
KD_LIST = [
    # 腿部关节 (4个)
    150.0, 150.0, 150.0, 100.0,  # knee_joint, leg_joint, waist_pitch_joint, waist_yaw_joint
    # 左臂关节 (7个)
    100.0, 30.0, 30.0, 30.0, 30.0, 30.0, 40.0,  # zarm_l1_joint to zarm_l7_joint
    # 右臂关节 (7个)
    100.0, 30.0, 30.0, 30.0, 30.0, 30.0, 40.0,  # zarm_r1_joint to zarm_r7_joint
    # 头部关节 (2个)
    30.0, 30.0  # zhead_1_joint, zhead_2_joint
]

class PIDController:
    def __init__(self, kp, ki, kd, output_min=-60.0, output_max=60.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
    
    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        output = max(self.output_min, min(self.output_max, output))
        
        self.prev_error = error
        return output
    
    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()

class S60JointController:
    def __init__(self, init_node=True, gravity_calculator='auto'):
        if init_node:
            rospy.init_node('s60_joint_controller', anonymous=True)
        
        # 关节配置
        self.joint_names = [
            'knee_joint', 'leg_joint', 'waist_pitch_joint', 'waist_yaw_joint',
            'zarm_l1_joint', 'zarm_l2_joint', 'zarm_l3_joint', 'zarm_l4_joint', 
            'zarm_l5_joint', 'zarm_l6_joint', 'zarm_l7_joint',
            'zarm_r1_joint', 'zarm_r2_joint', 'zarm_r3_joint', 'zarm_r4_joint', 
            'zarm_r5_joint', 'zarm_r6_joint', 'zarm_r7_joint',
            'zhead_1_joint', 'zhead_2_joint'
        ]
        
        # 初始化PID控制器
        self.pid_controllers = {}
        for i, name in enumerate(self.joint_names):
            self.pid_controllers[name] = PIDController(KP_LIST[i], 0.0, KD_LIST[i])
        
        # 状态变量
        self.current_positions = [0.0] * len(self.joint_names)
        self.target_positions = [0.0] * len(self.joint_names)
        self.enable_gravity_compensation = True
        self.gravity_calculator = gravity_calculator
        self.need_new_trajectory = False  # 新增：是否需要规划新轨迹的标志
        self.control_loop_running = False  # 新增：控制循环是否正在运行的标志
        print("重力补偿模式:" , self.gravity_calculator)
        
        # 轨迹生成相关变量
        self.trajectory_active = False
        self.trajectory_start_time = None
        self.trajectory_duration = 3.0  # 轨迹持续时间（秒）
        self.trajectory_start_positions = None
        self.trajectory_target_positions = None
        self.trajectory_complete = False
        self.trajectory_type = 's_curve'  # 'linear', 's_curve', 'trapezoidal'
        
        # 轨迹参数
        self.max_velocity = 0.5  # 最大角速度 (rad/s)
        self.max_acceleration = 2.0  # 最大角加速度 (rad/s²)
        self.trapezoidal_accel_time = 0.5  # 梯形轨迹加速时间 (s)
        
        # 稳定性检测
        self.stable_start_time = None
        self.stable_duration = 3.0  # 稳定持续时间要求（秒）
        self.stable_threshold = 0.02  # 稳定阈值（弧度）
        self._stability_delay_start = None  # 稳定校验延时开始时间
        
        # 重力补偿相关
        self.drake_plant = None
        self.drake_context = None
        self.has_floating_base = False
        self.pinocchio_model = None
        self.pinocchio_data = None
        
        # IK求解相关成员变量
        self.leg_joint_indices = None
        self.base_frame = None
        self.world_frame = None
        self.chassis_frame = None
        self.solver = None
        self.W_smooth = np.eye(4) * 100.0  # 关节平滑性权重矩阵
        self.W_chassis = np.diag([100, 100, 100, 50, 50, 50, 50])  # chassis位置权重矩阵[w, x, y, z, x_pos, y_pos, z_pos]
        self.max_additional_move = 0.1  # 底盘允许的额外移动量
        self.z_tolerance = 0.005  # Z方向约束容差（增加到5cm）
        self.initial_chassis_z = None  # 初始底盘Z方向位置
        
        # 初始化重力计算器
        self._init_gravity_calculator()
        
        # 初始化底盘控制器
        if CHASSIS_CONTROLLER_AVAILABLE:
            self.chassis_controller = ChassisController(init_node=False)
            rospy.loginfo("底盘控制器初始化成功")
        else:
            self.chassis_controller = None
            rospy.logwarn("底盘控制器不可用")
        
        # 设置发布者和订阅者
        self._setup_ros_interface()
        
        # 等待连接
        self.wait_for_connections()
    
    def _init_gravity_calculator(self):
        print(f"重力计算器设置: '{self.gravity_calculator}' (长度: {len(self.gravity_calculator)})")
        print(f"DRAKE_AVAILABLE: {DRAKE_AVAILABLE}")
        print(f"PINOCCHIO_AVAILABLE: {PINOCCHIO_AVAILABLE}")
        
        if self.gravity_calculator == 'pinocchio':
            print("尝试初始化Pinocchio...")
            if PINOCCHIO_AVAILABLE:
                self._init_pinocchio()
                if self.pinocchio_model is not None:
                    print("\033[34mPinocchio初始化成功\033[0m")
                    return
            print("\033[33mPinocchio初始化失败\033[0m")
        
        if self.gravity_calculator != 'pinocchio' and (self.gravity_calculator == 'drake' or self.gravity_calculator == 'auto'):
            print("尝试初始化Drake...")
            if DRAKE_AVAILABLE:
                self._init_drake()
                if self.drake_plant is not None:
                    print("\033[34mDrake初始化成功\033[0m")
                    return
            print("\033[33mDrake初始化失败\033[0m")
        
        if self.gravity_calculator != 'auto':
            rospy.logerr(f"指定的重力计算器 {self.gravity_calculator} 不可用")
            sys.exit(1)
        else:
            rospy.logwarn("重力补偿库不可用，将使用预设值")
    
    def _init_pinocchio(self):
        try:
            urdf_path = os.path.join(os.path.dirname(__file__),'..', '..', '..', '..', 'kuavo_assets', 'models', 'biped_s60', 'urdf', 'drake', 'biped_v3_full.urdf')
            print(f"尝试加载Pinocchio URDF文件: {urdf_path}")
            
            if os.path.exists(urdf_path):
                print("URDF文件存在，开始加载...")
                self.pinocchio_model = pinocchio.RobotWrapper.BuildFromURDF(urdf_path)
                self.pinocchio_data = pinocchio.Data(self.pinocchio_model)
                rospy.loginfo("Pinocchio模型加载成功")

                # 打印模型信息
                print("\n=== Pinocchio 模型信息 ===")
                print(f"位置变量数 (nq): {self.pinocchio_model.nq}")
                print(f"速度变量数 (nv): {self.pinocchio_model.nv}")
                print(f"关节数: {len(self.pinocchio_model.joints)}")
                print(f"关节名称: {[j.name for j in self.pinocchio_model.joints]}")
                print(f"状态向量结构: {self.pinocchio_model.names}")

                # 检查浮动基座
                self.has_floating_base = (self.pinocchio_model.nq > len(self.joint_names))
                print(f"浮动基座: {'是' if self.has_floating_base else '否'}")

                # 打印前几个状态的含义
                if self.has_floating_base:
                    print("状态向量结构:")
                    print("索引0-2: 基座位置 (x, y, z)")
                    print("索引3-6: 基座姿态 (四元数 w, x, y, z)")
                    print(f"索引7-{self.pinocchio_model.nq-1}: 关节位置")
                else:
                    print("状态向量结构: 直接是关节位置")
            else:
                print(f"URDF文件不存在: {urdf_path}")
                rospy.logwarn(f"Pinocchio URDF文件不存在: {urdf_path}")
        except Exception as e:
            rospy.logwarn(f"Pinocchio初始化失败: {e}")
            print(f"详细错误信息: {type(e).__name__}: {str(e)}")
            traceback.print_exc()
    
    def _init_drake(self):
        try:
            urdf_path = os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', 'kuavo_assets', 'models', 'biped_s60', 'urdf', 'drake', 'biped_v3_full.urdf')
            if os.path.exists(urdf_path):
                builder = DiagramBuilder()
                self.drake_plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.001)
                self.drake_plant.mutable_gravity_field().set_gravity_vector([0.0, 0.0, -9.81])
                
                parser = Parser(self.drake_plant)
                package_path = os.path.join(os.path.dirname(__file__), '..', '..', '..', '..')
                parser.package_map().Add("kuavo_assets", package_path)
                parser.AddModelFromFile(urdf_path)
                
                self.drake_plant.Finalize()
                self.drake_context = self.drake_plant.CreateDefaultContext()
                
                # 设置初始底盘位置（前4个是四元数，后3个是位置）
                initial_positions = self.drake_plant.GetPositions(self.drake_context)
                initial_positions[0] = 1.0  # 四元数w（单位姿态）
                initial_positions[1] = 0.0  # 四元数x
                initial_positions[2] = 0.0  # 四元数y
                initial_positions[3] = 0.0  # 四元数z
                initial_positions[4] = 0.0  # x
                initial_positions[5] = 0.0  # y
                initial_positions[6] = 0.195  # z

                # for i in range(self.drake_plant.num_joints()):
                #     joint = self.drake_plant.get_joint(drake.multibody.JointIndex(i))
                #     print(f"关节{i}：名称={joint.name()}, 位置起始索引={joint.position_start()}")

                self.drake_plant.SetPositions(self.drake_context, initial_positions)
                
                rospy.loginfo("Drake重力计算器初始化成功")

                            # 打印模型信息
                print("\n=== Drake 模型信息 ===")
                print(f"位置变量数: {self.drake_plant.num_positions()}")
                print(f"速度变量数: {self.drake_plant.num_velocities()}")
                print(f"关节数: {self.drake_plant.num_joints()}")

                # 打印关节信息
                print("关节列表:")
                
                # 尝试获取关节信息（简化版本，避免API兼容性问题）
                try:
                    print("  尝试获取关节名称:")
                    # 使用更安全的方法获取关节信息
                    num_joints = int(self.drake_plant.num_joints())  # 确保是整数
                    for i in range(min(num_joints, 10)):  # 限制数量避免错误
                        try:
                            joint = self.drake_plant.get_joint(drake.multibody.JointIndex(i))
                            print(f"    关节{i}: {joint.name()}")
                        except:
                            print(f"    关节{i}: 无法获取名称")
                except Exception as e:
                    print(f"  获取关节信息失败: {e}")
                
                # 检查浮动基座
                self.has_floating_base = (self.drake_plant.num_positions() > len(self.joint_names))
                print(f"浮动基座: {'是' if self.has_floating_base else '否'}")
                
                # 分析状态向量结构
                if self.has_floating_base:
                    print("状态向量结构分析:")
                    print("  前7个变量: 浮动基座 (位置3个 + 姿态4个)")
                    print(f"  后{self.drake_plant.num_positions()-7}个变量: 关节位置")
                    print(f"  你的代码定义了{len(self.joint_names)}个关节")
                else:
                    print("状态向量结构: 直接是关节位置")
                    print(f"  你的代码定义了{len(self.joint_names)}个关节")
        except Exception as e:
            rospy.logwarn(f"Drake初始化失败: {e}")
    
    def _setup_ros_interface(self):
        # 发布者
        self.joint_cmd_pub = rospy.Publisher('/lb_joint_cmd', jointCmd, queue_size=10)
        self.joint_ref_cmd_pub = rospy.Publisher('/ref_joint_cmd', jointCmd, queue_size=10)
        self.chassis_pose_pub = rospy.Publisher('/lb_chassis_pose', Pose, queue_size=10)
        
        # 订阅者 - 根据 use_shm_communication 参数选择话题
        use_shm = rospy.get_param('/use_shm_communication', False)
        if use_shm:
            sensor_topic = '/sensors_data_raw_shm'
            rospy.loginfo(f"[S60JointController] Using shared memory mode, subscribing to: {sensor_topic}")
        else:
            sensor_topic = '/sensors_data_raw'
            rospy.loginfo(f"[S60JointController] Using standard mode, subscribing to: {sensor_topic}")
        
        self.sensors_sub = rospy.Subscriber(sensor_topic, sensorsData, self.sensors_callback)
    
    def wait_for_connections(self):
        rate = rospy.Rate(10)
        start_time = time.time()
        timeout = 30.0  # 5秒超时
        
        rospy.loginfo("等待ROS连接...")
        while not rospy.is_shutdown() and running:
            # joint_cmd_pub是发布者，检查是否有订阅者
            joint_subscribers = self.joint_cmd_pub.get_num_connections()
            # sensors_sub是订阅者，检查是否有发布者
            sensor_publishers = self.sensors_sub.get_num_connections()
            
            # rospy.loginfo(f"连接状态: joint_cmd订阅者={joint_subscribers}, sensors发布者={sensor_publishers}")
            
            # 只要有sensors数据的发布者就可以开始控制
            if sensor_publishers > 0:
                rospy.loginfo("传感器数据连接已建立，开始控制")
                break
            
            if time.time() - start_time > timeout:
                rospy.logwarn("连接超时，继续运行...")
                break
                
            rate.sleep()
    
    def sensors_callback(self, msg):
        """传感器数据回调函数"""
        # 更新当前关节位置
        self.current_positions = list(msg.joint_data.joint_q)
        
        # 更新Drake位置（如果可用）
        if self.drake_plant is not None:
            try:
                nq = self.drake_plant.num_positions()
                q = np.zeros(nq)
                
                if self.has_floating_base:
                    q[0:4] = [1.0, 0.0, 0.0, 0.0]  # 基座姿态（四元数）
                    q[4:7] = [0.0, 0.0, 0.195]  # 基座位置
                    start_idx = 7
                else:
                    start_idx = 0
                
                # 设置关节位置
                for i in range(len(self.joint_names)):
                    if start_idx + i < nq:
                        if i < 4:
                            q[start_idx + i] = self.current_positions[i]
                        else:
                            q[start_idx + i] = 0.0

                        # q[start_idx + i] = self.current_positions[i]
                
                self.drake_plant.SetPositions(self.drake_context, q)
                
            except Exception as e:
                rospy.logwarn(f"更新Drake位置失败: {e}")
    
    def get_gravity_compensation(self, joint_name, joint_positions=None):
        if not self.enable_gravity_compensation:
            return 0.0
        
        target_joints = ['knee_joint', 'leg_joint', 'waist_pitch_joint']
        if joint_name not in target_joints:
            return 0.0
        
        try:
            joint_idx = self.joint_names.index(joint_name)
        except ValueError:
            rospy.logwarn(f"未知关节名: {joint_name}")
            return 0.0
        
        # 使用Pinocchio计算
        if self.pinocchio_model is not None:
            try:
                nq = self.pinocchio_model.nq
                q = np.zeros(nq)
                
                # 根据模型类型设置起始索引
                if self.has_floating_base:
                    # 浮动基座：前7个是基座状态
                    q[0:3] = [0.0, 0.0, 0.195]  # 基座位置
                    q[3:7] = [1.0, 0.0, 0.0, 0.0]  # 基座姿态
                    start_idx = 7
                    # 广义力中关节扭矩从索引6开始（前6个是基座力和力矩）
                    force_offset = 6
                else:
                    # 固定基座：直接从0开始
                    start_idx = 0
                    force_offset = 0
                
                # 设置关节位置
                if joint_positions and len(joint_positions) == len(self.joint_names):
                    for i in range(len(self.joint_names)):
                        if start_idx + i < nq:
                            q[start_idx + i] = float(joint_positions[i])
                
                pinocchio.computeGeneralizedGravity(self.pinocchio_model, self.pinocchio_data, q)
                gravity_forces = self.pinocchio_data.g
                
                # 计算正确的力索引
                force_idx = force_offset + joint_idx
                if force_idx < len(gravity_forces):
                    rospy.logdebug(f"Pinocchio重力补偿: {joint_name} = {gravity_forces[force_idx]:.2f} Nm")
                    return -gravity_forces[force_idx]
            except Exception as e:
                rospy.logwarn(f"Pinocchio重力计算失败: {e}")
        
        # 使用Drake计算
        if self.drake_plant is not None:
            try:
                # Drake位置已经在sensors_callback中更新，直接计算重力
                gravity_forces = self.drake_plant.CalcGravityGeneralizedForces(self.drake_context)
                
                # 计算力索引
                if self.has_floating_base:
                    force_offset = 6  # Drake中浮动基座有6个速度变量（3平移+3旋转）
                else:
                    force_offset = 0
                
                force_idx = force_offset + joint_idx
                if force_idx < gravity_forces.size:
                    rospy.logdebug(f"Drake重力补偿: {joint_name} = {gravity_forces[force_idx]:.2f} Nm")
                    return -gravity_forces[force_idx]
            except Exception as e:
                rospy.logwarn(f"Drake重力计算失败: {e}")
        
        return 0.0
    
    def create_joint_cmd_msg(self, target_positions=None):
        if target_positions is None:
            target_positions = self.get_current_target_positions()
        
        msg = jointCmd()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        
        # 初始化所有数组
        num_joints = len(self.joint_names)
        msg.joint_q = [0.0] * num_joints
        msg.joint_v = [0.0] * num_joints
        msg.tau = [0.0] * num_joints
        msg.tau_max = [100.0] * num_joints  # 增加最大扭矩限制
        msg.tau_ratio = [1.0] * num_joints
        msg.joint_kp = [0.0] * num_joints
        msg.joint_kd = [0.0] * num_joints
        msg.control_modes = [0] * num_joints  # 0: 力矩控制模式
        
        # 计算PID控制输出和重力补偿
        current_time = time.time()
        target_joints = ['knee_joint', 'leg_joint', 'waist_pitch_joint']
        
        for i, name in enumerate(self.joint_names):
            if name in self.pid_controllers:
                pid = self.pid_controllers[name]
                error = target_positions[i] - self.current_positions[i]
                dt = current_time - pid.last_time
                pid.last_time = current_time
                
                # PID控制输出
                pid_output = pid.compute(error, dt)
                
                # 重力补偿
                gravity_comp = self.get_gravity_compensation(name, self.current_positions)
                
                # 总输出
                total_output = pid_output + gravity_comp
                msg.tau[i] = total_output
                
                # 设置关节角用于观察响应曲线
                msg.joint_q[i] = target_positions[i]
                
                # # 打印目标关节的输出信息（仅在轨迹跟踪过程中）
                if name in target_joints and (self.trajectory_active or self.trajectory_complete):
                    rospy.logdebug(f"{name}: PID={pid_output:.2f}, 重力补偿={gravity_comp:.2f}, 总输出={total_output:.2f}, 误差={error:.4f}, target={target_positions[i]:.4f}, current={self.current_positions[i]:.4f}")
        
        return msg
    
    def send_joint_cmd(self, target_positions=None):
        msg = self.create_joint_cmd_msg(target_positions)
        self.joint_cmd_pub.publish(msg)
        self.joint_ref_cmd_pub.publish(msg)
    
    def send_chassis_pose(self, x, y, z, roll=0, pitch=0, yaw=0):
        pose = Pose()
        pose.position = Point(x, y, z)
        
        # 欧拉角转四元数
        cy, sy = np.cos(yaw * 0.5), np.sin(yaw * 0.5)
        cp, sp = np.cos(pitch * 0.5), np.sin(pitch * 0.5)
        cr, sr = np.cos(roll * 0.5), np.sin(roll * 0.5)
        
        pose.orientation.w = cy * cp * cr + sy * sp * sr
        pose.orientation.x = cy * cp * sr - sy * sp * cr
        pose.orientation.y = sy * cp * sr + cy * sp * cr
        pose.orientation.z = sy * cp * cr - cy * sp * sr
        
        self.chassis_pose_pub.publish(pose)
    
    def run_control_loop(self, target_joints, is_check_stability=True):
        """
        运行控制循环
        Args:
            target_joints: 目标关节角度列表，只需要前4个腿部关节 [knee, leg, waist_pitch, waist_yaw]
                如果为None，直接返回
            is_check_stability: 是否进行稳定性检查
        """
        # 检查输入参数
        if target_joints is None:
            rospy.logwarn("目标关节角度为None，控制循环退出")
            return
        
        # 检查列表长度，只需要4个关节
        if len(target_joints) < 4:
            rospy.logerr(f"目标关节角度数量不足，需要4个，实际{len(target_joints)}")
            return
        
        # 设置目标位置，只更新前4个关节
        self.target_positions[:4] = target_joints[:4]
        
        rospy.loginfo("控制循环开始，轨迹跟踪已启动")
        rospy.loginfo(f"目标关节位置: {self.target_positions}")
        
        print("是否做稳定性检查: ", is_check_stability)
        rate = rospy.Rate(200) #100Hz
        print("控制频率: ", 200)
        # 设置控制循环运行标志
        self.control_loop_running = True
        self.need_new_trajectory = True
        while not rospy.is_shutdown() and running:
            # 检查是否需要规划新轨迹
            if self.need_new_trajectory:
                self.start_trajectory(self.target_positions)
                self.need_new_trajectory = False
                rospy.loginfo(f"开始新的轨迹规划，目标位置: {self.target_positions}")
            
            self.send_joint_cmd()

            # # 检查是否稳定
            # if  self.check_stability(is_check_stability):
            #     rospy.loginfo("关节已稳定在目标位置")
            #     break
            
            rate.sleep()

        # 重置控制循环运行标志
        self.control_loop_running = False
        self.need_new_trajectory = False
        rospy.loginfo("轮臂腿部控制循环结束")
    
    def update_target_joints(self, target_joints, duration=None):
        """
        更新目标关节角度（用于持续控制循环中）
        Args:
            target_joints: 目标关节角度列表，只需要前4个腿部关节 [knee, leg, waist_pitch, waist_yaw]
        Returns:
            bool: 更新是否成功
        """
        if target_joints is None:
            rospy.logwarn("目标关节角度为None，忽略更新")
            return False
        
        # 检查列表长度，只需要4个关节
        if len(target_joints) < 4:
            rospy.logerr(f"目标关节角度数量不足，需要4个，实际{len(target_joints)}")
            return False
        
        # 只更新前4个关节位置，其他保持当前值
        self.target_positions[:4] = target_joints[:4]

        # 设置轨迹持续时间，默认为3秒
        if duration is not None and duration > 0:
            self.trajectory_duration = duration
            rospy.loginfo(f"更新轨迹持续时间: {duration}秒")
        else:
            self.trajectory_duration = 3.0
        
        # 设置需要规划新轨迹的标志
        self.need_new_trajectory = True
        
        rospy.loginfo(f"更新目标关节位置: {self.target_positions[:4]}")
        return True
    
    def print_status(self):
        rospy.loginfo("=== S60关节控制器状态 ===")
        rospy.loginfo(f"重力补偿: {'启用' if self.enable_gravity_compensation else '禁用'}")
        rospy.loginfo(f"重力计算器: {self.gravity_calculator}")
        rospy.loginfo(f"Pinocchio: {'可用' if self.pinocchio_model else '不可用'}")
        rospy.loginfo(f"Drake: {'可用' if self.drake_plant else '不可用'}")

    def start_trajectory(self, target_positions, trajectory_type=None):
        """
        开始轨迹生成
        Args:
            target_positions: 目标关节位置列表（只需要前4个腿部关节）
            trajectory_type: 轨迹类型 ('linear', 's_curve', 'trapezoidal')
        """

        if trajectory_type is not None:
            self.trajectory_type = trajectory_type
        
        # 只处理前4个关节的轨迹规划
        self.trajectory_start_positions = self.current_positions[:4].copy()
        self.trajectory_target_positions = target_positions[:4].copy()
        self.trajectory_start_time = time.time()
        self.trajectory_active = True
        self.trajectory_complete = False
        self.stable_start_time = None
        self._stability_delay_start = None  # 重置稳定校验延时标志
        
        rospy.loginfo("=== 开始轨迹生成 ===")
        rospy.loginfo(f"轨迹类型: {self.trajectory_type}")
        rospy.loginfo(f"轨迹持续时间: {self.trajectory_duration}秒")
        rospy.loginfo(f"起始位置: {[f'{p:.4f}' for p in self.trajectory_start_positions]}")
        rospy.loginfo(f"目标位置: {[f'{p:.4f}' for p in self.trajectory_target_positions]}")
    
    def compute_trajectory_positions(self):
        """
        计算当前时刻的轨迹位置
        Returns:
            list: 轨迹上的关节位置
        """
        if not self.trajectory_active or self.trajectory_start_time is None:
            return self.target_positions
        
        current_time = time.time()
        elapsed_time = current_time - self.trajectory_start_time
        
        if elapsed_time >= self.trajectory_duration:
            # 轨迹完成
            rospy.loginfo("轨迹完成，切换到目标位置控制")
            rospy.loginfo(f"轨迹目标位置长度: {len(self.trajectory_target_positions) if self.trajectory_target_positions else 'None'}")
            rospy.loginfo(f"轨迹目标位置内容: {self.trajectory_target_positions}")
            
            self.trajectory_active = False
            self.trajectory_complete = True
            
            # 返回完整的20关节位置，前4个是轨迹目标位置，其他保持当前值
            full_trajectory_positions = self.current_positions.copy()
            full_trajectory_positions[:4] = self.trajectory_target_positions
            return full_trajectory_positions
        
        # 计算归一化时间 (0.0 到 1.0)
        t = elapsed_time / self.trajectory_duration
        
        # 根据轨迹类型计算插值比例
        if self.trajectory_type == 'linear':
            s = t
        elif self.trajectory_type == 's_curve':
            # S曲线插值 (三次贝塞尔曲线)
            s = 3 * t * t - 2 * t * t * t
        elif self.trajectory_type == 'trapezoidal':
            # 梯形速度曲线
            s = self._trapezoidal_interpolation(t)
        else:
            # 默认使用S曲线
            s = 3 * t * t - 2 * t * t * t
        
        # 计算前4个关节的轨迹位置
        trajectory_positions = []
        rospy.logdebug(f"计算轨迹位置: start_positions长度={len(self.trajectory_start_positions)}, target_positions长度={len(self.trajectory_target_positions)}")
        
        for i in range(len(self.trajectory_start_positions)):
            start_pos = self.trajectory_start_positions[i]
            target_pos = self.trajectory_target_positions[i]
            trajectory_pos = start_pos + s * (target_pos - start_pos)
            trajectory_positions.append(trajectory_pos)
        
        # 返回完整的20关节位置，前4个是轨迹位置，其他保持当前值
        full_trajectory_positions = self.current_positions.copy()
        full_trajectory_positions[:4] = trajectory_positions
        
        return full_trajectory_positions
    
    def _trapezoidal_interpolation(self, t):
        """
        梯形速度曲线插值
        Args:
            t: 归一化时间 (0.0 到 1.0)
        Returns:
            float: 插值比例 (0.0 到 1.0)
        """
        # 计算梯形轨迹的时间分配
        accel_time = self.trapezoidal_accel_time / self.trajectory_duration
        decel_time = accel_time
        const_time = 1.0 - accel_time - decel_time
        
        if const_time < 0:
            # 如果加速时间太长，调整为三角形轨迹
            accel_time = 0.5
            decel_time = 0.5
            const_time = 0.0
        
        if t <= accel_time:
            # 加速段
            s = 0.5 * (t / accel_time) ** 2
        elif t <= accel_time + const_time:
            # 匀速段
            s = 0.5 + (t - accel_time) / const_time * 0.5
        else:
            # 减速段
            t_decel = t - accel_time - const_time
            s = 1.0 - 0.5 * (1.0 - t_decel / decel_time) ** 2
        
        return s
    
    def check_stability(self, is_check_stability=True):
        """
        检查关节是否稳定在目标位置
        Returns:
            bool: 是否稳定
        """
        if not self.trajectory_complete:
            rospy.logdebug("check_stability: 轨迹未完成，返回False")
            return False

        if not is_check_stability:
            # 不做稳定校验时，延时3秒后直接返回True
            if self._stability_delay_start is None:
                self._stability_delay_start = time.time()
                rospy.loginfo("不做稳定校验，延时3秒后退出...")
            
            if time.time() - self._stability_delay_start >= 3.0:
                rospy.loginfo("延时3秒完成，直接退出")
                return True
            
            return False
        
        # 检查前4个关节的误差
        max_error = 0.0
        rospy.logdebug(f"check_stability: 检查稳定性，current_positions长度: {len(self.current_positions)}")
        rospy.logdebug(f"check_stability: trajectory_target_positions: {self.trajectory_target_positions}")
        
        # 强制转换为整数
        loop_count = int(min(4, len(self.current_positions)))
        # print(f"loop_count: {loop_count}, 类型: {type(loop_count)}")
        
        for i in range(loop_count):
            error = abs(self.current_positions[i] - self.trajectory_target_positions[i])
            max_error = max(max_error, error)

        # 如果误差小于等于阈值，开始计时
        if max_error <= self.stable_threshold:
            if self.stable_start_time is None:
                self.stable_start_time = time.time()
                rospy.loginfo(f"关节开始稳定，最大误差: {max_error:.4f}")
            
            # 检查是否稳定足够长时间
            stable_time = time.time() - self.stable_start_time
            if stable_time >= self.stable_duration:
                # 计算所有关节误差
                joint_errors = []
                for i in range(loop_count):
                    error = abs(self.current_positions[i] - self.trajectory_target_positions[i])
                    joint_errors.append(f"{error:.4f}")
                rospy.loginfo(f"关节已稳定 {self.stable_duration} 秒，当前关节误差: [{', '.join(joint_errors)}], 最大误差: {max_error:.4f}")
                return True
        else:
            # 误差超过阈值，重置稳定计时
            if self.stable_start_time is not None:
                # 计算所有关节误差
                joint_errors = []
                for i in range(loop_count):
                    error = abs(self.current_positions[i] - self.trajectory_target_positions[i])
                    joint_errors.append(f"{error:.4f}")
                rospy.loginfo(f"关节不稳定，重置稳定计时，当前关节误差: [{', '.join(joint_errors)}], 最大误差: {max_error:.4f}")
            self.stable_start_time = None
        
        return False
    
    def get_current_target_positions(self):
        """
        获取当前应该使用的目标位置（轨迹上的位置）
        Returns:
            list: 当前目标位置
        """
        rospy.logdebug(f"获取当前目标位置: trajectory_active={self.trajectory_active}, trajectory_complete={self.trajectory_complete}")
        
        if self.trajectory_active:
            rospy.logdebug("轨迹激活中，计算轨迹位置")
            return self.compute_trajectory_positions()
        elif self.trajectory_complete:
            rospy.logdebug("轨迹已完成，返回轨迹目标位置")
            rospy.logdebug(f"trajectory_target_positions: {self.trajectory_target_positions}")
            # 返回完整的20关节位置，前4个是轨迹目标位置，其他保持当前值
            full_target_positions = self.current_positions.copy()
            full_target_positions[:4] = self.trajectory_target_positions
            return full_target_positions
        else:
            rospy.logdebug("使用默认目标位置")
            return self.target_positions

    def compute_base_pose_from_joints(self, q_vector):
        """
        基于前11个状态向量计算base_link位姿
        Args:
            q_vector: 前11个状态向量 [qw, qx, qy, qz, x, y, z, q1, q2, q3, q4]
                     - 前4个: 底盘姿态四元数 [qw, qx, qy, qz]
                     - 中间3个: 底盘位置 [x, y, z]  
                     - 后4个: 腿部关节角度 [q1, q2, q3, q4]
        Returns:
            tuple: (base_position, base_orientation, chassis_position, chassis_orientation)
        """
        if not DRAKE_AVAILABLE or self.drake_plant is None:
            rospy.logwarn("Drake不可用，无法计算正运动学")
            return None, None, None, None
        
        try:
            # 检查输入向量长度
            if len(q_vector) < 11:
                rospy.logerr(f"输入向量长度不足，需要11个元素，实际只有{len(q_vector)}个")
                return None, None, None, None
            
            # 创建临时Context，不影响当前状态
            temp_context = self.drake_plant.CreateDefaultContext()
            
            # 设置状态向量
            q = np.zeros(self.drake_plant.num_positions())
            
            # 直接复制前11个元素
            q[:11] = q_vector[:11]
            
            chassis_yaw = self.quaternion_to_euler(q[0], q[1], q[2], q[3])[2]
            rospy.loginfo(f"设置底盘姿yaw: {chassis_yaw}")
            rospy.loginfo(f"设置底盘位置: {q[4:7]}")
            rospy.loginfo(f"设置腿部关节角度: {q[7:11]}")
            
            # 更新临时Context的状态
            self.drake_plant.SetPositions(temp_context, q)
            
            # 获取base_link位姿
            base_frame = self.drake_plant.GetFrameByName("base_link")
            world_frame = self.drake_plant.world_frame()
            base_pose = self.drake_plant.CalcRelativeTransform(
                        temp_context, 
                        world_frame,  # 参考坐标系：世界坐标系
                        base_frame  # 目标坐标系：base_link
            )
            
            base_position = base_pose.translation()
            base_orientation = base_pose.rotation().ToQuaternion()
            print(f"获取base位置: {base_position}")
            print(f"获取base姿态: {base_orientation}")
            # 获取chassis_link位姿
            chassis_frame = self.drake_plant.GetFrameByName("chassis_link")
            chassis_pose = self.drake_plant.CalcRelativeTransform(
                temp_context,
                world_frame,    # 参考坐标系：世界坐标系
                chassis_frame   # 目标坐标系：chassis_link
            )
            chassis_position = chassis_pose.translation()
            chassis_orientation = chassis_pose.rotation().ToQuaternion()
            print(f"获取底盘位置: {chassis_position}")
            print(f"获取底盘姿态: {chassis_orientation}")
            return base_position, base_orientation, chassis_position, chassis_orientation
            
        except Exception as e:
            rospy.logerr(f"正运动学计算失败: {e}")
            traceback.print_exc()
            return None, None, None, None

    def compute_leg_ik(self, target_base_position, target_base_orientation, reference_positions=None, solver_type="chassis_priority", refer_iteration=False):
        """
        基于目标base_link位姿计算腿部关节角度和chassis_link位姿
        Args:
            target_base_position: 目标base_link位置 [x, y, z]
            target_base_orientation: 目标base_link姿态 (四元数)
            current_chassis_position: 当前chassis_link位置 [x, y, z]，如果为None则从Drake状态获取
            current_chassis_yaw: 当前chassis_link偏航角，如果为None则从Drake状态获取
        Returns:
            tuple: (success, chassis_position, chassis_yaw, joint_angles)
        """
        if not DRAKE_AVAILABLE or self.drake_plant is None:
            rospy.logwarn("Drake不可用，无法计算逆运动学")
            return False, None, None, None
        
        try:
            # 1. 初始化设置 - 创建临时context避免影响当前状态
            plant = self.drake_plant
            temp_context = self.drake_plant.CreateDefaultContext()  # 创建临时context
            
            # 初始化IK相关成员变量
            self._init_ik_members(temp_context)
            
            # 2. 获取当前状态
            current_positions = plant.GetPositions(self.drake_context)  # 从主context获取当前状态
            if reference_positions is not None:
                print(f"使用外部设置的参考位置: {reference_positions[:11]}")
                current_positions[:11] = reference_positions[:11]
            print(f"参考位置: {current_positions[:11]}")

            # 3. 执行三步求解策略
            start_time = time.time()
            success, solution, method = self._solve_ik_three_steps(
                temp_context, target_base_orientation,
                target_base_position, current_positions,
                solver_type, 
                refer_iteration
                )
            
            # 4. 提取结果
            if success:
                result = self._extract_ik_results(temp_context, solution, target_base_position)
                total_time = (time.time() - start_time) * 1000
                rospy.loginfo(f"逆运动学求解成功 ({method})，耗时: {total_time:.2f} ms")
                return result
            else:
                total_time = (time.time() - start_time) * 1000
                rospy.logerr(f"逆运动学求解失败，总耗时: {total_time:.2f} ms")
                return False, None, None, None
                
        except Exception as e:
            rospy.logerr(f"逆运动学计算失败: {e}")
            traceback.print_exc()
            return False, None, None, None

    def _get_leg_joint_indices(self):
        """获取腿部关节索引"""
        leg_joint_names = ["knee_joint", "leg_joint", "waist_pitch_joint", "waist_yaw_joint"]
        leg_joint_indices = []
        
        # 添加日志查看所有关节的索引
        try:
            # 获取默认模型实例的关节索引
            default_model = self.drake_plant.GetModelInstanceByName("biped_v3_full")
            all_joints = self.drake_plant.GetJointIndices(default_model)
            # print(f"关节数量: {all_joints}")
            
            # 直接遍历前10个关节
            for i in range(10):
                try:
                    joint = self.drake_plant.get_joint(all_joints[i])
                    joint_name = joint.name()
                    start_idx = joint.position_start()
                    # print(f"索引 {i}: 关节名={joint_name}, 位置起始索引={start_idx}")
                except IndexError:
                    break  # 如果索引超出范围就停止
                except Exception as e:
                    print(f"获取关节 {i} 信息失败: {e}")
                    break
        except Exception as e:
            print(f"获取关节列表失败: {e}")
        
        # print("=== 腿部关节索引 ===")
        for joint_name in leg_joint_names:
            try:
                joint = self.drake_plant.GetJointByName(joint_name)
                leg_joint_indices.append(joint.position_start())
                # print(f"腿部关节 {joint_name} 索引: {joint.position_start()}")
            except Exception as e:
                rospy.logerr(f"无法获取关节 {joint_name} 的索引: {e}")
                return None
        
        if len(leg_joint_indices) != 4:
            rospy.logerr("无法获取所有4个腿部关节索引")
            return None
        
        print(f"腿部关节索引: {leg_joint_indices}")
        return leg_joint_indices

    def _get_current_chassis_state(self, context, current_chassis_position, current_chassis_yaw):
        """获取当前chassis状态"""

        if current_chassis_position is None or current_chassis_yaw is None:
            # 确保IK成员变量已初始化
            self._init_ik_members(context)
            
            chassis_pose = self.drake_plant.CalcRelativeTransform(
                context, 
                self.world_frame, 
                self.chassis_frame
            )
            current_chassis_position = chassis_pose.translation()
            chassis_orientation = chassis_pose.rotation()
            chassis_rotation_matrix = chassis_orientation.matrix()
            current_chassis_yaw = np.arctan2(chassis_rotation_matrix[1, 0], chassis_rotation_matrix[0, 0])
            print(f"从Drake状态获取 - chassis位置: {current_chassis_position}, yaw: {current_chassis_yaw:.4f}")
        
        return current_chassis_position, current_chassis_yaw

    def _set_initial_state(self, current_positions, current_chassis_position, current_chassis_yaw):
        """设置初始状态"""
        if len(current_positions) >= 7 and self.has_floating_base:
            # 确保使用正确的底盘位置
            if current_chassis_position is None:
                current_chassis_position = [0.0, 0.0, 0.195]
            
            rospy.loginfo(f"设置初始状态:")
            rospy.loginfo(f"  输入chassis位置: {current_chassis_position}")
            rospy.loginfo(f"  输入chassis yaw: {current_chassis_yaw:.4f}")
            
            # 设置底盘姿态和位置（前4个是四元数，后3个是位置）
            # 将yaw角转换为四元数（绕z轴旋转）
            yaw_quat = RotationMatrix.MakeZRotation(current_chassis_yaw).ToQuaternion()
            current_positions[0] = yaw_quat.w()  # qw
            current_positions[1] = yaw_quat.x()  # qx
            current_positions[2] = yaw_quat.y()  # qy
            current_positions[3] = yaw_quat.z()  # qz
            # 设置底盘位置
            current_positions[4] = current_chassis_position[0]  # x
            current_positions[5] = current_chassis_position[1]  # y
            current_positions[6] = current_chassis_position[2]  # z
            
            rospy.loginfo(f"  设置后的前7个位置值: {current_positions[:7]}")
            rospy.loginfo(f"    - 四元数: [{current_positions[0]:.6f}, {current_positions[1]:.6f}, {current_positions[2]:.6f}, {current_positions[3]:.6f}]")
            rospy.loginfo(f"    - 位置: [{current_positions[4]:.6f}, {current_positions[5]:.6f}, {current_positions[6]:.6f}]")
            
            # 更新Drake context的状态
            if self.drake_plant is not None:
                self.drake_plant.SetPositions(self.drake_context, current_positions)
        """设置初始状态"""
        if len(current_positions) >= 7 and self.has_floating_base:
            # 确保使用正确的底盘位置
            if current_chassis_position is None:
                current_chassis_position = [0.0, 0.0, 0.195]
            
            rospy.loginfo(f"设置初始状态:")
            rospy.loginfo(f"  输入chassis位置: {current_chassis_position}")
            rospy.loginfo(f"  输入chassis yaw: {current_chassis_yaw:.4f}")
            
            # 设置底盘姿态和位置（前4个是四元数，后3个是位置）
            # 将yaw角转换为四元数（绕z轴旋转）
            yaw_quat = RotationMatrix.MakeZRotation(current_chassis_yaw).ToQuaternion()
            current_positions[0] = yaw_quat.w()  # qw
            current_positions[1] = yaw_quat.x()  # qx
            current_positions[2] = yaw_quat.y()  # qy
            current_positions[3] = yaw_quat.z()  # qz
            # 设置底盘位置
            current_positions[4] = current_chassis_position[0]  # x
            current_positions[5] = current_chassis_position[1]  # y
            current_positions[6] = current_chassis_position[2]  # z
            
            rospy.loginfo(f"  设置后的前7个位置值: {current_positions[:7]}")
            rospy.loginfo(f"    - 四元数: [{current_positions[0]:.6f}, {current_positions[1]:.6f}, {current_positions[2]:.6f}, {current_positions[3]:.6f}]")
            rospy.loginfo(f"    - 位置: [{current_positions[4]:.6f}, {current_positions[5]:.6f}, {current_positions[6]:.6f}]")
            
            # 更新Drake context的状态
            if self.drake_plant is not None:
                self.drake_plant.SetPositions(self.drake_context, current_positions)

    def quaternion_to_euler(self, qw, qx, qy, qz):
        """
        将四元数转换为欧拉角（ZYX顺序）
        Args:
            qw, qx, qy, qz: 四元数的四个分量
        Returns:
            tuple: (roll, pitch, yaw) 弧度制
        """
        # 检查四元数是否为零
        if abs(qw) < 1e-10 and abs(qx) < 1e-10 and abs(qy) < 1e-10 and abs(qz) < 1e-10:
            rospy.logwarn("四元数所有元素都为零，返回零欧拉角")
            return 0.0, 0.0, 0.0
        
        # 检查四元数是否归一化
        norm = np.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
        if abs(norm - 1.0) > 1e-6:
            rospy.logwarn(f"四元数未归一化，norm={norm}，进行归一化")
            qw, qx, qy, qz = qw/norm, qx/norm, qy/norm, qz/norm
        
        # 计算旋转矩阵的元素
        # 旋转矩阵 R = [r11 r12 r13]
        #                [r21 r22 r23]
        #                [r31 r32 r33]
        r11 = 1.0 - 2.0 * (qy*qy + qz*qz)
        r12 = 2.0 * (qx*qy - qw*qz)
        r13 = 2.0 * (qx*qz + qw*qy)
        
        r21 = 2.0 * (qx*qy + qw*qz)
        r22 = 1.0 - 2.0 * (qx*qx + qz*qz)
        r23 = 2.0 * (qy*qz - qw*qx)
        
        r31 = 2.0 * (qx*qz - qw*qy)
        r32 = 2.0 * (qy*qz + qw*qx)
        r33 = 1.0 - 2.0 * (qx*qx + qy*qy)
        
        # 从旋转矩阵提取欧拉角（ZYX顺序）
        # Yaw (绕Z轴)
        yaw = np.arctan2(r21, r11)
        
        # Pitch (绕Y轴)
        pitch = np.arcsin(-r31)
        
        # Roll (绕X轴)
        roll = np.arctan2(r32, r33)
        
        return roll, pitch, yaw
    
    def quaternion_to_euler_degrees(self, qw, qx, qy, qz):
        """
        将四元数转换为欧拉角（ZYX顺序），返回度数
        Args:
            qw, qx, qy, qz: 四元数的四个分量
        Returns:
            tuple: (roll, pitch, yaw) 度数制
        """
        roll, pitch, yaw = self.quaternion_to_euler(qw, qx, qy, qz)
        return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)

    def _convert_orientation_to_matrix(self, target_base_orientation):
        """转换目标姿态为Drake RotationMatrix对象"""
        # 检查四元数是否为零
        w, x, y, z = target_base_orientation.w(), target_base_orientation.x(), target_base_orientation.y(), target_base_orientation.z()
        rospy.loginfo(f"四元数分量: w={w}, x={x}, y={y}, z={z}")
        
        if abs(w) < 1e-10 and abs(x) < 1e-10 and abs(y) < 1e-10 and abs(z) < 1e-10:
            rospy.logerr("四元数所有元素都为零，使用单位四元数")
            target_base_orientation = Quaternion(1.0, 0.0, 0.0, 0.0)
        
        # 检查四元数是否归一化
        norm = np.sqrt(w*w + x*x + y*y + z*z)
        if abs(norm - 1.0) > 1e-6:
            rospy.logwarn(f"四元数未归一化，norm={norm}，进行归一化")
            w, x, y, z = w/norm, x/norm, y/norm, z/norm
            print(f"归一化后的四元数: {w}, {x}, {y}, {z}")
            target_base_orientation = Quaternion(w, x, y, z)
        
        # 直接使用Drake的RotationMatrix构造函数
        rotation_matrix = RotationMatrix(target_base_orientation)
        return rotation_matrix

    def _init_ik_members(self, context):
        """初始化IK求解相关的成员变量 - 只在第一次调用时初始化"""
        # 使用类级别的标志来避免重复初始化
        if hasattr(self, '_ik_members_initialized') and self._ik_members_initialized:
            return
        
        if self.leg_joint_indices is None:
            self.leg_joint_indices = self._get_leg_joint_indices()
        
        if self.base_frame is None:
            self.base_frame = self.drake_plant.GetFrameByName("base_link")
        
        if self.world_frame is None:
            self.world_frame = self.drake_plant.world_frame()
        
        if self.chassis_frame is None:
            self.chassis_frame = self.drake_plant.GetFrameByName("chassis_link")

        if self.initial_chassis_z is None:
            self.initial_chassis_z = 0.195
            rospy.loginfo(f"初始化底盘Z方向位置: {self.initial_chassis_z:.4f}")
        
        if self.solver is None:
            self.solver = SnoptSolver()
        
        # 标记为已初始化
        self._ik_members_initialized = True
        rospy.loginfo("IK成员变量初始化完成")

    def _solve_ik_three_steps(self, context, target_base_orientation,
                             target_base_position, current_positions, solver_type="chassis_priority", refer_iteration=False):
        """三步求解策略"""
        
        # 准备成本函数
        q_ref = np.array([current_positions[idx] for idx in self.leg_joint_indices])
        print(f"Drake context中的关节角度: {q_ref}")
        
        # Step 3: 二分法自适应容差
        result = self._binary_search_ik(context, target_base_orientation,
                                    target_base_position, q_ref, current_positions, solver_type, refer_iteration)
        if result[0]:
            return True, result[1], "二分法自适应容差"
        else:
            return False, None, "二分法自适应容差失败"

    def _create_ik_solver_chassis_priority(self, rotation_matrix, target_base_position, pos_tol, orient_tol, 
                         q_ref, initial_guess=None):
        """创建IK求解器，根据目标base_link位置动态调整底盘约束范围，优先考虑底盘位置"""
        # 确保IK成员变量已初始化
        self._init_ik_members(self.drake_context)
        
        ik = InverseKinematics(self.drake_plant, self.drake_context, with_joint_limits=True)
        
        # 调整求解器参数以避免奇异矩阵问题
        ik.get_mutable_prog().SetSolverOption(self.solver.solver_id(), "Major Optimality Tolerance", 3e-2)
        ik.get_mutable_prog().SetSolverOption(self.solver.solver_id(), "Major Iterations Limit", 2000)
        ik.get_mutable_prog().SetSolverOption(self.solver.solver_id(), "Minor Iterations Limit", 500)
        ik.get_mutable_prog().SetSolverOption(self.solver.solver_id(), "Scale Option", 2)  # 自动缩放

        # 1. 计算底盘和base_link之间的大致距离
        if initial_guess is None:
            rospy.loginfo(f"initial_guess is None")
            return None
        else:
            joint_positions = initial_guess
            print(f"initial_guess: {initial_guess[7:11]}")

        current_chassis_position = initial_guess[4:7]
        current_chassis_yaw = self.quaternion_to_euler(initial_guess[0], initial_guess[1], initial_guess[2], initial_guess[3])[2]
            
        tmp_fk_result = self.compute_base_pose_from_joints(joint_positions[:11])
        if tmp_fk_result is not None:
            current_base_pos, _, _, _ = tmp_fk_result
            # 计算当前底盘和base_link之间的距离
            base_chassis_dist = np.linalg.norm(np.array(current_chassis_position[:2]) - np.array(current_base_pos[:2]))
            rospy.loginfo(f"底盘与base_link的在XY平面的当前距离: {base_chassis_dist:.4f} m")
        else:
            # 如果无法计算，使用默认值
            base_chassis_dist = 0.5
            rospy.loginfo(f"使用默认底盘与base_link距离: {base_chassis_dist:.4f} m")

        # 2. 基于目标base_link位置和机械结构限制设置底盘位置约束
        xy_center = target_base_position[:2]  # 以目标base_link的XY为中心

        # 计算Z方向的约束范围 - 地面运行的底盘Z方向移动受限
        target_z = target_base_position[2]
        current_z = self.initial_chassis_z
        
        # Z方向约束 - 地面底盘Z坐标基本固定，主要依赖腿部关节调整base_link高度
        # Z方向容差也随着pos_tol变化，从较大容差开始尝试
        z_tolerance_dynamic = max(self.z_tolerance, pos_tol * 0.3)  # 随pos_tol变化，但保持较小比例
        
        rospy.loginfo(f"\033[34mZ方向约束: chassis Z={current_z:.4f}, 容差={z_tolerance_dynamic:.4f}, pos_tol={pos_tol:.4f}, orient_tol={orient_tol:.4f}\033[0m")

        # 计算约束范围 - 地面底盘XY方向移动能力强，Z方向受限
        # XY方向：保持相对宽松的约束，允许底盘在平面上移动
        xy_tolerance = base_chassis_dist + self.max_additional_move
        
        # Z方向：严格限制，主要依赖腿部关节调整高度
        z_tolerance = z_tolerance_dynamic
        
        chassis_pos_lower = np.array([
            xy_center[0] - xy_tolerance,
            xy_center[1] - xy_tolerance,
            current_z - z_tolerance
        ])
        chassis_pos_upper = np.array([
            xy_center[0] + xy_tolerance,
            xy_center[1] + xy_tolerance,
            current_z + z_tolerance
        ])

        # 3. 添加位置约束
        rospy.loginfo(f"设置chassis位置约束:")
        rospy.loginfo(f"  当前chassis位置: {current_chassis_position}")
        rospy.loginfo(f"  当前chassis yaw: {current_chassis_yaw:.4f}")
        rospy.loginfo(f"  约束下界: {chassis_pos_lower}")
        rospy.loginfo(f"  约束上界: {chassis_pos_upper}")
        rospy.loginfo(f"  约束范围: X[{chassis_pos_lower[0]:.4f}, {chassis_pos_upper[0]:.4f}], Y[{chassis_pos_lower[1]:.4f}, {chassis_pos_upper[1]:.4f}], Z[{chassis_pos_lower[2]:.4f}, {chassis_pos_upper[2]:.4f}]")
        
        ik.AddPositionConstraint(
            self.chassis_frame, 
            np.zeros(3),  # 底盘坐标系中的点（原点）
            self.world_frame, 
            chassis_pos_lower, 
            chassis_pos_upper
        )

        # 添加base_link位置约束
        ik.AddPositionConstraint(
            self.base_frame, 
            np.zeros(3),  # base_link坐标系中的点（原点）
            self.world_frame,
            target_base_position - pos_tol, 
            target_base_position + pos_tol
        )

        # 添加姿态约束：base_frame的目标姿态相对于world_frame
        ik.AddOrientationConstraint(
            self.world_frame, 
            rotation_matrix, 
            self.base_frame, 
            RotationMatrix(), 
            orient_tol
        )

        # 添加成本函数
        # 1. 关节平滑性成本
        ik.get_mutable_prog().AddQuadraticErrorCost(
            self.W_smooth, 
            q_ref, 
            [ik.q()[idx] for idx in self.leg_joint_indices]
        )
        
        # 2. 关节角度偏好成本 - 适度鼓励求解器选择接近当前关节角度的解
        joint_preference_weight = np.diag([200.0, 200.0, 500.0, 500.0])  # 降低关节偏好权重，允许更多变化
        current_joint_angles = q_ref  # 使用当前关节角度作为参考
        ik.get_mutable_prog().AddQuadraticErrorCost(
            joint_preference_weight,
            current_joint_angles,  # 当前关节角度作为参考
            [ik.q()[idx] for idx in self.leg_joint_indices]
        )

        # 3. 底盘姿态约束 - 使用旋转矩阵约束roll和pitch
        # 创建目标底盘姿态（只有yaw旋转，roll和pitch为0）
        target_chassis_yaw = current_chassis_yaw if current_chassis_yaw is not None else 0.0
        target_chassis_rotation = RotationMatrix.MakeZRotation(target_chassis_yaw)
        
        # 添加底盘姿态约束，只允许yaw方向旋转，限制roll和pitch
        ik.AddOrientationConstraint(
            self.world_frame,  # 参考坐标系：世界坐标系
            target_chassis_rotation,  # 目标底盘姿态（只有yaw旋转）
            self.chassis_frame,  # 目标坐标系：底盘坐标系   
            RotationMatrix(),  # 底盘坐标系中的参考姿态
            0.02  # 姿态容差（约5.7度） 
        )

        # # 3. base_link位置成本 - 通过腿部关节影响base_link位置
        # # 注意：base_link的位置是通过腿部关节运动间接控制的，不是直接由浮动基座控制
        # # 这里我们添加一个软约束，鼓励求解器找到更好的关节配置
        # base_pos_weight = np.diag([500.0, 500.0, 500.0])  # X, Y, Z方向权重
        # ik.get_mutable_prog().AddQuadraticErrorCost(
        #     base_pos_weight,
        #     target_base_position,  # 目标位置
        #     ik.q()[4:7]  # 浮动基座位置（间接影响base_link位置）
        # )

        # 4. chassis位置成本（可选）
        # chassis_ref = np.array([
        #     current_chassis_position[0], 
        #     current_chassis_position[1], 
        #     current_chassis_position[2], 
        #     1.0, 0.0, 0.0, 0.0  # 单位四元数
        # ])
        # ik.get_mutable_prog().AddQuadraticErrorCost(self.W_chassis, chassis_ref, ik.q()[:7])

        # 5. 底盘姿态成本 - 惩罚roll和pitch偏离0，允许yaw自由变化
        # 使用更大的权重来确保底盘保持水平
        # chassis_orientation_weight = np.diag([100.0, 5000.0, 5000.0, 100.0])  # 大幅增加x和y分量的权重
        # target_chassis_orientation = np.array([1.0, 0.0, 0.0, 0.0])  # 目标：水平姿态 (w=1, x=0, y=0, z=0)
        
        # ik.get_mutable_prog().AddQuadraticErrorCost(
        #     chassis_orientation_weight,
        #     target_chassis_orientation,  # 目标水平姿态
        #     ik.q()[:4]  # 底盘四元数 (w,x,y,z)
        # )

        return ik

    def _create_ik_solver_waist_priority(self, rotation_matrix, target_base_position, pos_tol, orient_tol, 
                         q_ref, initial_guess=None):
        """创建IK求解器，根据目标base_link位置调节轮臂腰部关节"""
        # 确保IK成员变量已初始化
        self._init_ik_members(self.drake_context)
        
        ik = InverseKinematics(self.drake_plant, self.drake_context, with_joint_limits=True)
        
        # 调整求解器参数以避免奇异矩阵问题
        ik.get_mutable_prog().SetSolverOption(self.solver.solver_id(), "Major Optimality Tolerance", 1e-2)
        ik.get_mutable_prog().SetSolverOption(self.solver.solver_id(), "Major Iterations Limit", 2000)
        ik.get_mutable_prog().SetSolverOption(self.solver.solver_id(), "Minor Iterations Limit", 500)
        ik.get_mutable_prog().SetSolverOption(self.solver.solver_id(), "Scale Option", 2)  # 自动缩放

        # 1. 保存底盘的初始位置和姿态，用于严格约束
        if initial_guess is None:
            print(f"\033[31minitial_guess is None\033[0m")
            return None
        else:
            joint_positions = initial_guess
            print(f"initial_guess: {initial_guess[7:11]}")

        # 保存底盘的初始状态，确保约束中心不变
        initial_chassis_position = initial_guess[4:7].copy()
        initial_chassis_quat = initial_guess[0:4].copy()
        initial_chassis_yaw = self.quaternion_to_euler(initial_guess[0], initial_guess[1], initial_guess[2], initial_guess[3])[2]
        
        rospy.loginfo(f"底盘初始位置: {initial_chassis_position}")
        rospy.loginfo(f"底盘初始四元数: {initial_chassis_quat}")
        rospy.loginfo(f"底盘初始yaw: {initial_chassis_yaw:.4f}")

        # 2. 设置底盘位置约束 - 严格固定，只允许极小移动
        # 设置极小的容差，基本固定底盘位置
        xy_tolerance = 0.01  # 只允许1cm移动
        z_tolerance = 0.01   # Z方向也严格限制
        
        rospy.loginfo(f"\033[34m底盘位置约束: 容差XY={xy_tolerance:.4f}m, Z={z_tolerance:.4f}m\033[0m")
        
        # 约束中心改为当前底盘位置，而不是目标base_link位置
        chassis_pos_lower = np.array([
            initial_chassis_position[0] - xy_tolerance,  # 以当前底盘位置为中心
            initial_chassis_position[1] - xy_tolerance,
            initial_chassis_position[2] - z_tolerance
        ])
        chassis_pos_upper = np.array([
            initial_chassis_position[0] + xy_tolerance,
            initial_chassis_position[1] + xy_tolerance,
            initial_chassis_position[2] + z_tolerance
        ])

        # 3. 添加位置约束
        rospy.loginfo(f"设置chassis位置约束:")
        rospy.loginfo(f"  初始chassis位置: {initial_chassis_position}")
        rospy.loginfo(f"  初始chassis yaw: {initial_chassis_yaw:.4f}")
        rospy.loginfo(f"  约束下界: {chassis_pos_lower}")
        rospy.loginfo(f"  约束上界: {chassis_pos_upper}")
        rospy.loginfo(f"  约束范围: X[{chassis_pos_lower[0]:.4f}, {chassis_pos_upper[0]:.4f}], Y[{chassis_pos_lower[1]:.4f}, {chassis_pos_upper[1]:.4f}], Z[{chassis_pos_lower[2]:.4f}, {chassis_pos_upper[2]:.4f}]")
        
        ik.AddPositionConstraint(
            self.chassis_frame, 
            np.zeros(3),  # 底盘坐标系中的点（原点）
            self.world_frame, 
            chassis_pos_lower, 
            chassis_pos_upper
        )

        # 添加base_link位置约束
        ik.AddPositionConstraint(
            self.base_frame, 
            np.zeros(3),  # base_link坐标系中的点（原点）
            self.world_frame,
            target_base_position - pos_tol, 
            target_base_position + pos_tol
        )

        # 添加姿态约束：base_frame的目标姿态相对于world_frame
        ik.AddOrientationConstraint(
            self.world_frame, 
            rotation_matrix, 
            self.base_frame, 
            RotationMatrix(), 
            orient_tol
        )

        # 添加成本函数
        # 1. 关节平滑性成本
        ik.get_mutable_prog().AddQuadraticErrorCost(
            self.W_smooth, 
            q_ref, 
            [ik.q()[idx] for idx in self.leg_joint_indices]
        )
        
        # 2. 关节角度偏好成本 - 降低权重，允许关节更多变化来达到目标
        joint_preference_weight = np.diag([50.0, 50.0, 100.0, 100.0])  # 大幅降低关节偏好权重
        current_joint_angles = q_ref  # 使用当前关节角度作为参考
        ik.get_mutable_prog().AddQuadraticErrorCost(
            joint_preference_weight,
            current_joint_angles,  # 当前关节角度作为参考
            [ik.q()[idx] for idx in self.leg_joint_indices]
        )

        # 3. 底盘姿态约束 - 严格保持当前姿态，不允许任何旋转
        # 创建目标底盘姿态（完全保持当前姿态）
        initial_chassis_rotation = RotationMatrix(Quaternion(initial_chassis_quat))
        
        # 添加底盘姿态约束，不允许任何旋转变化
        ik.AddOrientationConstraint(
            self.world_frame,  # 参考坐标系：世界坐标系
            initial_chassis_rotation,  # 目标底盘姿态（保持当前姿态）
            self.chassis_frame,  # 目标坐标系：底盘坐标系   
            RotationMatrix(),  # 底盘坐标系中的参考姿态
            0.01  # 极小的姿态容差（约0.6度） 
        )

        # 3. 底盘位置成本 - 极大权重惩罚底盘移动，确保原地不动
        chassis_pos_weight = np.diag([10000.0, 10000.0, 10000.0])  # 极大权重
        ik.get_mutable_prog().AddQuadraticErrorCost(
            chassis_pos_weight,
            initial_chassis_position,  # 目标：保持当前位置
            ik.q()[4:7]  # 底盘位置
        )

        # 4. 底盘姿态成本 - 极大权重惩罚姿态变化，确保姿态不变
        chassis_orientation_weight = np.diag([10000.0, 10000.0, 10000.0, 10000.0])  # 极大权重
        ik.get_mutable_prog().AddQuadraticErrorCost(
            chassis_orientation_weight,
            initial_chassis_quat,  # 目标：保持当前姿态
            ik.q()[:4]  # 底盘四元数 (w,x,y,z)
        )

        return ik

    def _binary_search_ik(self, context, target_base_orientation,
                         target_base_position, q_ref, current_positions, solver_type="chassis_priority", refer_iteration=False):
        """二分法搜索最佳容差"""
        
        low_tol = 0.5    # 从较大的初始容差开始（20厘米）
        high_tol = 0.1   # 更严格的失败容差（1厘米）
        min_diff = 0.05  # 最小容差区间（3毫米）
        
        # 转换目标姿态为旋转矩阵
        rotation_matrix = self._convert_orientation_to_matrix(target_base_orientation)
        
        iteration = 0
        best_solution = None  # 保存最佳解
        initial_guess = current_positions
        ref_guess = current_positions
        
        while abs(high_tol - low_tol) > min_diff:
            mid_tol = (low_tol + high_tol) / 2.0
            if refer_iteration:
                initial_guess = ref_guess
            else:
                initial_guess = current_positions

            ik_mid = None
            if solver_type == "chassis_priority":
                ik_mid = self._create_ik_solver_chassis_priority(rotation_matrix, target_base_position, mid_tol, mid_tol, 
                                           q_ref, initial_guess)
            elif solver_type == "waist_priority":
                ik_mid = self._create_ik_solver_waist_priority(rotation_matrix, target_base_position, mid_tol, mid_tol, 
                                           q_ref, initial_guess)
            if ik_mid is None:
                print(f"\033[31mik solver create failed\033[0m")
                best_solution = None
                break
            
            result = Solve(ik_mid.prog(), initial_guess)
            iteration += 1
            
            if result.is_success():
                low_tol = mid_tol  # 成功，缩小下界
                best_solution = result.GetSolution(ik_mid.q())  # 保存成功的结果
                ref_guess = best_solution  # 使用成功的结果作为下次初始值
                print(f"best solution, chassis: {best_solution[:7]}")
                print(f"best solution, q1~q4: {best_solution[7:11]}")
                rospy.loginfo(f"二分搜索 [{iteration}]: 容差={mid_tol:.4f}, 成功=True")
            else:
                high_tol = mid_tol  # 失败，缩小上界
                rospy.loginfo(f"二分搜索 [{iteration}]: 容差={mid_tol:.4f}, 成功=False")

            if best_solution is None:
                break
        
        if best_solution is not None:
            rospy.loginfo(f"二分搜索完成，返回之前找到的最佳解，最终容差: {low_tol:.4f}, 迭代次数: {iteration}")
            print(f"result best_solution: {best_solution[7:11]}")
            return True, best_solution, f"二分搜索(容差:{low_tol:.4f},迭代:{iteration})"
        else:
            rospy.logwarn("二分搜索失败，未找到任何有效解")
            return False, None, "二分搜索失败"

    def _extract_ik_results(self, context, solution, target_base_position):
        """提取IK结果"""
        # 确保IK成员变量已初始化
        self._init_ik_members(context)
        
        # 更新plant状态
        self.drake_plant.SetPositions(context, solution)
        
        # 提取关节角度
        joint_angles = np.array([solution[idx] for idx in self.leg_joint_indices])
        
        # 计算chassis位置和姿态
        chassis_pose = self.drake_plant.CalcRelativeTransform(
            context,
            self.world_frame,    # 参考坐标系：世界坐标系
            self.chassis_frame   # 目标坐标系：chassis_link
        )
        chassis_position = chassis_pose.translation()
        chassis_orientation = chassis_pose.rotation()
        chassis_rotation_matrix = chassis_orientation.matrix()
        
        # 计算chassis的欧拉角 (ZYX顺序)
        chassis_yaw = np.arctan2(chassis_rotation_matrix[1, 0], chassis_rotation_matrix[0, 0])
        chassis_pitch = np.arcsin(-chassis_rotation_matrix[2, 0])
        chassis_roll = np.arctan2(chassis_rotation_matrix[2, 1], chassis_rotation_matrix[2, 2])
        
        # 转换为度数
        chassis_yaw_deg = np.degrees(chassis_yaw)
        chassis_pitch_deg = np.degrees(chassis_pitch)
        chassis_roll_deg = np.degrees(chassis_roll)
        
        rospy.loginfo(f"目标base位置: {target_base_position}")
        rospy.loginfo(f"计算得到的关节角度: {joint_angles}")
        rospy.loginfo(f"计算得到的chassis位置: {chassis_position}")
        rospy.loginfo(f"\033[36m计算得到的chassis姿态 (弧度): roll={chassis_roll:.4f}, pitch={chassis_pitch:.4f}, yaw={chassis_yaw:.4f}\033[0m")
        rospy.loginfo(f"\033[36m计算得到的chassis姿态 (度数): roll={chassis_roll_deg:.2f}°, pitch={chassis_pitch_deg:.2f}°, yaw={chassis_yaw_deg:.2f}°\033[0m")
        
        # 添加调试信息：检查solution中的前7个值（浮动基座状态）
        rospy.loginfo(f"Solution中前7个值（浮动基座）: {solution[:7]}")
        rospy.loginfo(f"  - 四元数: [{solution[0]:.6f}, {solution[1]:.6f}, {solution[2]:.6f}, {solution[3]:.6f}]")
        rospy.loginfo(f"  - 位置: [{solution[4]:.6f}, {solution[5]:.6f}, {solution[6]:.6f}]")
        
        return True, chassis_position, chassis_yaw, joint_angles

    def move_chassis_to_target(self, chassis_position, chassis_yaw, duration=3.0):
        """
        移动底盘到目标位置和姿态
        Args:
            chassis_position: 目标底盘位置 [x, y, z]
            chassis_yaw: 目标底盘偏航角（弧度）
            duration: 移动持续时间（秒）
        Returns:
            bool: 是否成功完成移动
        """
        if self.chassis_controller is not None:
            try:
                # 将计算得到的chassis位置和yaw转换为度数
                chassis_yaw_degrees = np.degrees(chassis_yaw)
                chassis_x, chassis_y, chassis_z = chassis_position[0], chassis_position[1], chassis_position[2]
                
                rospy.loginfo(f"调用底盘控制接口: 位置({chassis_x:.3f}, {chassis_y:.3f}, {chassis_z:.3f}), 偏航角({chassis_yaw_degrees:.2f}°)")
                
                # 移动底盘到计算得到的位置
                success = self.chassis_controller.move_chassis(chassis_x, chassis_y, chassis_yaw_degrees, duration)
                
                if success:
                    rospy.loginfo("底盘移动完成")
                else:
                    rospy.logwarn("底盘移动失败")
                
                return success
            except Exception as e:
                rospy.logerr(f"底盘控制接口调用失败: {e}")
                return False
        else:
            rospy.logwarn("底盘控制器不可用，跳过底盘控制")
            return False

def main():
    try:
        # 设置ROS日志级别为DEBUG，这样rospy.logdebug就会显示
        import logging
        logging.getLogger('rosout').setLevel(logging.DEBUG)
        
        controller = S60JointController(gravity_calculator='auto')
        controller.print_status()

        # 示例：设置自定义关节角度并使用轨迹跟踪
        custom_joints = [0.314, -0.377, 0.0314, -1.51]  # [knee, leg, waist_pitch, waist_yaw]

        # custom_joints = [0.251, -0.377, 0.0942, -1.57]  # [knee, leg, waist_pitch, waist_yaw]
        
        # 创建一个包含前11个元素的向量：底盘姿态(4) + 底盘位置(3) + 关节角度(4)
        # test_q_vector = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.195, 0.314, -0.377, 0.0314, -1.51]
        # fk_result = controller.compute_base_pose_from_joints(test_q_vector)
        # target_base_pos, target_base_ori, chassis_pos, chassis_ori = fk_result
        # rospy.loginfo(f"设置目标base_link位置: {target_base_pos}")
        # rospy.loginfo(f"设置目标base_link姿态: {target_base_ori}")
        test_q_vector = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.195, 0.226, -0.942, 0.66, 0.0]
        fk_result = controller.compute_base_pose_from_joints(test_q_vector)
        target_base_pos, target_base_ori, chassis_pos, chassis_ori = fk_result
        rospy.loginfo(f"设置目标base_link位置: {target_base_pos}")
        rospy.loginfo(f"设置目标base_link姿态: {target_base_ori}")

        # 设置轨迹参数
        controller.trajectory_type = 's_curve'  # 'linear', 's_curve', 'trapezoidal'
        controller.trajectory_duration = 3.0  # 轨迹持续时间
        controller.run_control_loop(custom_joints, False)
        
        # 测试逆运动学
        rospy.loginfo("=== 测试逆运动学 ===")


        # 等待传感器数据
        rospy.sleep(1.0)
        
        if len(controller.current_positions) >= 4:
            # 使用当前关节角度计算正运动学，得到目标base_link位姿
            current_joints = controller.current_positions[:4]
            print(f"current_joints: {current_joints}")
            # 创建一个包含前11个元素的向量：底盘姿态(4) + 底盘位置(3) + 关节角度(4)
            current_q_vector = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.195] + current_joints
            fk_result = controller.compute_base_pose_from_joints(current_q_vector)
            
            if fk_result[0] is not None:
                base_pos, base_ori, chassis_pos, chassis_ori = fk_result
                rospy.loginfo(f"当前base_link位置: {base_pos}")
                rospy.loginfo(f"当前base_link姿态: {base_ori}")
                
                # 设置一个稍微偏移的目标位置
                # target_base_position = np.array([0.9, 0.8, 0.99266941])
                # target_base_position = np.array([0.01459352, -0.01692463, 0.87558809])
                target_base_position = target_base_pos + np.array([0.0, 0.0, 0.0])
                target_base_orientation = target_base_ori
                
                rospy.loginfo(f"\033[35m目标base_link位置: {target_base_position}\033[0m")
                rospy.loginfo(f"\033[35m目标base_link姿态: {target_base_orientation}\033[0m")
                
                # 计算逆运动学
                # reference_positions = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.195, 0.26, -0.66, 0.44, -0.942]
                reference_positions = None
                ik_success, chassis_position, chassis_yaw, joint_angles = controller.compute_leg_ik(
                    target_base_position, target_base_orientation, reference_positions
                )
                
                if ik_success:
                    rospy.loginfo("========== 逆运动学结果 ==========")
                    rospy.loginfo(f"\033[32mchassis位置: {chassis_position}\033[0m")
                    rospy.loginfo(f"\033[32mchassis yaw: {chassis_yaw:.4f}\033[0m")
                    rospy.loginfo(f"\033[32m关节角度: {joint_angles}\033[0m")

                    # 验证结果：使用计算出的关节角度重新计算正运动学
                    # 创建验证用的状态向量
                    chassis_yaw_quat = RotationMatrix.MakeZRotation(chassis_yaw).ToQuaternion()
                    verify_q_vector = [
                        chassis_yaw_quat.w(), chassis_yaw_quat.x(), chassis_yaw_quat.y(), chassis_yaw_quat.z(),  # 底盘姿态四元数
                        chassis_position[0], chassis_position[1], chassis_position[2],  # 底盘位置
                        joint_angles[0], joint_angles[1], joint_angles[2], joint_angles[3]  # 关节角度
                    ]
                    fk_verify = controller.compute_base_pose_from_joints(verify_q_vector)
                    if fk_verify[0] is not None:
                        verify_base_pos, verify_base_ori, _, _ = fk_verify
                        position_error = np.linalg.norm(target_base_position - verify_base_pos)
                        # 计算每个轴的旋转误差
                        # 将四元数转换为欧拉角（ZYX顺序）
                        target_rot = RotationMatrix(target_base_orientation)
                        verify_rot = RotationMatrix(verify_base_ori)
                        
                        # 计算目标姿态的欧拉角
                        target_yaw = np.arctan2(target_rot.matrix()[1, 0], target_rot.matrix()[0, 0])
                        target_pitch = np.arcsin(-target_rot.matrix()[2, 0])
                        target_roll = np.arctan2(target_rot.matrix()[2, 1], target_rot.matrix()[2, 2])
                        
                        # 计算验证姿态的欧拉角
                        verify_yaw = np.arctan2(verify_rot.matrix()[1, 0], verify_rot.matrix()[0, 0])
                        verify_pitch = np.arcsin(-verify_rot.matrix()[2, 0])
                        verify_roll = np.arctan2(verify_rot.matrix()[2, 1], verify_rot.matrix()[2, 2])
                        
                        # 计算各轴的误差（弧度）
                        roll_error = abs(target_roll - verify_roll)
                        pitch_error = abs(target_pitch - verify_pitch)
                        yaw_error = abs(target_yaw - verify_yaw)
                        
                        # 转换为度数
                        roll_error_deg = np.degrees(roll_error)
                        pitch_error_deg = np.degrees(pitch_error)
                        yaw_error_deg = np.degrees(yaw_error)
                        
                        # 转换为度数
                        target_roll_deg = np.degrees(target_roll)
                        target_pitch_deg = np.degrees(target_pitch)
                        target_yaw_deg = np.degrees(target_yaw)
                        verify_roll_deg = np.degrees(verify_roll)
                        verify_pitch_deg = np.degrees(verify_pitch)
                        verify_yaw_deg = np.degrees(verify_yaw)
                        
                        rospy.loginfo(f"\033[34m位置误差: {position_error:.6f} m\033[0m")
                        rospy.loginfo(f"\033[34m期望Roll: {target_roll:.6f} rad ({target_roll_deg:.2f}°) | 逆解Roll: {verify_roll:.6f} rad ({verify_roll_deg:.2f}°) | Roll误差: {roll_error:.6f} rad ({roll_error_deg:.2f}°)\033[0m")
                        rospy.loginfo(f"\033[34m期望Pitch: {target_pitch:.6f} rad ({target_pitch_deg:.2f}°) | 逆解Pitch: {verify_pitch:.6f} rad ({verify_pitch_deg:.2f}°) | Pitch误差: {pitch_error:.6f} rad ({pitch_error_deg:.2f}°)\033[0m")
                        rospy.loginfo(f"\033[34m期望Yaw: {target_yaw:.6f} rad ({target_yaw_deg:.2f}°) | 逆解Yaw: {verify_yaw:.6f} rad ({verify_yaw_deg:.2f}°) | Yaw误差: {yaw_error:.6f} rad ({yaw_error_deg:.2f}°)\033[0m")
                        rospy.loginfo(f"\033[35m验证base位置: {verify_base_pos}\033[0m")
                        rospy.loginfo(f"\033[35m验证base姿态: {verify_base_ori}\033[0m")

                    # 调用底盘控制接口
                    controller.move_chassis_to_target(chassis_position, chassis_yaw)
                    rospy.sleep(3.0)
                    
                    #  # 将numpy数组转换为字典格式，并使用轨迹跟踪
                    # joint_angles_dict = {}
                    # for i, joint_name in enumerate(controller.joint_names[:4]):  # 只取前4个腿部关节
                    #     joint_angles_dict[joint_name] = float(joint_angles[i])
                    
                    # rospy.loginfo(f"转换后的关节角度字典: {joint_angles_dict}")

                    # # 使用轨迹跟踪执行逆运动学结果
                    controller.trajectory_type = 's_curve'  # 使用梯形轨迹
                    controller.trajectory_duration = 4.0  # 4秒轨迹
                    controller.run_control_loop(joint_angles, True)
                    

                else:
                    rospy.logwarn("逆运动学求解失败")
            else:
                rospy.logwarn("无法获取当前base_link位姿")
        else:
            rospy.logwarn("传感器数据不足，无法测试逆运动学")
        

            
    except KeyboardInterrupt:
        print("\n用户中断程序")
    except Exception as e:
        rospy.logerr(f"程序异常: {e}")
        import traceback
        traceback.print_exc()
    finally:
        running = False
        rospy.loginfo("程序退出")

if __name__ == '__main__':
    main() 