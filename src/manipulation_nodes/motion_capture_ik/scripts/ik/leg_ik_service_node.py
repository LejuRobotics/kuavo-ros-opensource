#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
腿部IK服务节点
"""

import rospy
import numpy as np
import time
import sys
import os
import traceback
import threading        

from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from pydrake.common.eigen_geometry import Quaternion
from pydrake.math import RotationMatrix

# 导入服务定义
from kuavo_msgs.srv import lbBaseLinkPoseCmdSrv, lbBaseLinkPoseCmdSrvResponse
from kuavo_msgs.srv import lbLegControlSrv, lbLegControlSrvResponse
from kuavo_msgs.srv import chassisPoseCmdSrv, chassisPoseCmdSrvResponse

# 导入现有的关节控制器
try:
    # 添加当前目录到Python路径
    current_dir = os.path.dirname(os.path.abspath(__file__))
    if current_dir not in sys.path:
        sys.path.insert(0, current_dir)
    
    from s60_joint_controller import S60JointController
    from chassis_move import ChassisController
    CONTROLLER_AVAILABLE = True
except ImportError:
    CONTROLLER_AVAILABLE = False
    rospy.logwarn("s60_joint_controller模块不可用")

class LegIKServiceNode:
    def __init__(self):
        rospy.init_node('lb_leg_ik_service_node', anonymous=True)
        
        # 初始化关节控制器
        if CONTROLLER_AVAILABLE:
            self.joint_controller = S60JointController(init_node=False)
            rospy.loginfo("关节控制器初始化成功")
        else:
            self.joint_controller = None
            rospy.logwarn("关节控制器不可用")
        
        # 设置发布者
        self.joint_traj_pub = rospy.Publisher('/lb_leg_ik/joint_trajectory', JointState, queue_size=10)
        
        # 设置服务
        self.ik_service = rospy.Service('/lb_leg_ik_srv', 
                                       lbBaseLinkPoseCmdSrv, self.handle_ik_service_request)
        
        # 设置腿部控制服务
        self.leg_control_service = rospy.Service('/lb_leg_control_srv', 
                                                lbLegControlSrv, self.handle_leg_control_request)
        
        # 设置底盘位置控制服务
        self.chassis_pose_service = rospy.Service('/chassisPoseCmdSrv', 
                                                 chassisPoseCmdSrv, self.handle_chassis_pose_request)
        
        # 状态变量
        self.current_positions = [0.0] * 20
        
        # 底盘控制相关
        self.chassis_controller = ChassisController(init_node=False)
        self.chassis_moving = False
        self.chassis_target_pose = None
        self.chassis_duration = 2.0
        self.chassis_thread = None
        self.chassis_thread_event = threading.Event()
        
        # 启动底盘控制线程
        self.start_chassis_control_thread()
        
        rospy.loginfo("腿部IK服务节点初始化完成")
    
    def sensors_callback(self, msg):
        """传感器数据回调函数"""
        self.current_positions = list(msg.joint_data.joint_q)
    
    def handle_ik_service_request(self, req):
        """处理IK服务请求"""
        start_time = time.time()
        
        # 初始化失败响应
        response = lbBaseLinkPoseCmdSrvResponse()
        response.success = False
        response.lb_leg = [0.0] * 4
        response.time_cost = (time.time() - start_time) * 1000
        
        try:
            # 检查关节控制器是否可用
            if self.joint_controller is None:
                rospy.logerr("关节控制器不可用")
                return response
            
            # 解析请求参数
            with_chassis = req.with_chassis
            chassis_info = req.chassis_info  # [x, y, yaw]
            q_lb = req.q_lb  # [knee, leg, waist_pitch, waist_yaw]
            base_link = req.base_link  # [x, y, z, qw, qx, qy, qz]
            control_type = req.control_type  # 0-底盘优先，1-腰部优先
            
            # 解析目标base_link姿态
            target_base_position = np.array(base_link[:3])   # [x, y, z]
            
            # 创建Drake四元数对象
            target_base_orientation = Quaternion(
                base_link[3],  # qw
                base_link[4],  # qx
                base_link[5],  # qy
                base_link[6]   # qz
            )
            
            control_type_str = "底盘优先" if control_type == 0 else "腰部优先"
            rospy.loginfo(f"收到IK请求: with_chassis={with_chassis}, control_type={control_type}({control_type_str}), target base_link位置={target_base_position}, target base_link姿态={target_base_orientation}")
            
            # 准备参考位置
            reference_positions = None
            if with_chassis and len(chassis_info) >= 3:
                chassis_x, chassis_y, chassis_yaw = chassis_info
                chassis_quat = RotationMatrix.MakeZRotation(chassis_yaw).ToQuaternion()
                reference_positions = [
                    chassis_quat.w(), chassis_quat.x(), chassis_quat.y(), chassis_quat.z(),  # 底盘姿态四元数
                    chassis_x, chassis_y, 0.195,  # 底盘位置 (z固定为0.223)
                    q_lb[0], q_lb[1], q_lb[2], q_lb[3]  # 腿部关节角度
                ]
                rospy.loginfo(f"使用底盘信息构建参考位置: {reference_positions}")
            
            # 根据control_type设置solver_type
            solver_type = "chassis_priority" if control_type == 0 else "waist_priority"
            
            # 调用IK求解函数
            ik_success, chassis_position, chassis_yaw, joint_angles = self.joint_controller.compute_leg_ik(
                target_base_position, target_base_orientation, reference_positions, solver_type=solver_type, refer_iteration=False
            )
            
            # 更新响应
            response.success = ik_success
            response.time_cost = (time.time() - start_time) * 1000
            
            if ik_success:
                response.lb_leg = joint_angles.tolist()
                rospy.loginfo(f"\033[34mIK求解成功，耗时: {response.time_cost:.2f} ms\033[0m")
                rospy.loginfo(f"计算得到的关节角度: {response.lb_leg}")
            else:
                rospy.logwarn(f"\033[33mIK求解失败\033[0m")
            
            return response
                
        except Exception as e:
            rospy.logerr(f"IK服务处理失败: {e}")
            traceback.print_exc()
            return response
    
    def handle_leg_control_request(self, req):
        """处理腿部控制服务请求"""
        response = lbLegControlSrvResponse()
        response.success = False
        
        try:
            if self.joint_controller is None:
                rospy.logerr("关节控制器不可用")
                return response
            
            target_joints = req.target_joints
            duration = req.duration
            if len(target_joints) < 4:
                print("目标关节角度数量不足，需要4个，实际", len(target_joints))
                return response
            
            # 检查控制循环状态
            if not self.joint_controller.control_loop_running:
                # 启动新控制循环，直接传入4个关节数据
                self.control_thread = threading.Thread(
                    target=self.joint_controller.run_control_loop, 
                    args=(target_joints, False),
                    daemon=True
                )
                self.control_thread.start()
            else:
                # 更新目标位置
                if not self.joint_controller.update_target_joints(target_joints, duration):
                    return response
            
            response.success = True
            
        except Exception as e:
            rospy.logerr(f"腿部控制失败: {e}")
        
        return response
    
    def start_chassis_control_thread(self):
        """启动底盘控制线程"""
        self.chassis_thread = threading.Thread(target=self.chassis_control_loop, daemon=True)
        self.chassis_thread.start()
        rospy.loginfo("底盘控制线程已启动")
    
    def chassis_control_loop(self):
        """底盘控制循环"""
        while not rospy.is_shutdown():
            # 等待新的移动任务
            self.chassis_thread_event.wait()
            self.chassis_thread_event.clear()
            
            if self.chassis_target_pose is not None and self.chassis_moving:
                try:
                    rospy.loginfo("开始底盘移动任务")
                    
                    # 从四元数计算偏航角
                    qw = self.chassis_target_pose.orientation.w
                    qx = self.chassis_target_pose.orientation.x
                    qy = self.chassis_target_pose.orientation.y
                    qz = self.chassis_target_pose.orientation.z
                    
                    # 计算偏航角 (yaw)
                    yaw = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
                    yaw_degrees = np.degrees(yaw)
                    
                    # 调用底盘移动函数
                    success = self.chassis_controller.move_chassis_velocity(
                        self.chassis_target_pose.position.x,
                        self.chassis_target_pose.position.y,
                        yaw_degrees,
                        self.chassis_duration
                    )
                    
                    if success:
                        rospy.loginfo("底盘移动任务完成")
                    else:
                        rospy.logwarn("底盘移动任务失败")
                        
                except Exception as e:
                    rospy.logerr(f"底盘移动任务异常: {e}")
                
                finally:
                    # 重置状态
                    self.chassis_moving = False
                    self.chassis_target_pose = None
    
    def handle_chassis_pose_request(self, req):
        """处理底盘位置控制服务请求"""
        response = chassisPoseCmdSrvResponse()
        response.success = False
        
        try:
            # 检查是否正在移动
            if self.chassis_moving:
                print("底盘正在移动中，请等待当前任务完成")
                return response
            
            # 解析请求参数
            target_pose = req.target_pose
            duration_array = req.duration
            
            # 默认2秒
            self.chassis_duration = 2.0
            # 设置持续时间
            if len(duration_array) > 0:
                self.chassis_duration = float(duration_array[0])   
            
            # 设置目标位置和标志位
            self.chassis_target_pose = target_pose
            self.chassis_moving = True
            
            # 触发线程执行
            self.chassis_thread_event.set()
            
            response.success = True
            rospy.loginfo(f"收到底盘位置控制请求: 位置=({target_pose.position.x:.3f}, {target_pose.position.y:.3f}), 持续时间={self.chassis_duration:.1f}秒")
            
        except Exception as e:
            rospy.logerr(f"底盘位置控制服务处理失败: {e}")
        
        return response
    
    def run(self):
        """运行服务节点"""
        rospy.loginfo("腿部IK服务节点启动")
        rospy.spin()

def main():
    try:
        node = LegIKServiceNode()
        node.run()
    except KeyboardInterrupt:
        print("\n用户中断程序")
    except Exception as e:
        rospy.logerr(f"程序异常: {e}")

if __name__ == '__main__':
    main() 