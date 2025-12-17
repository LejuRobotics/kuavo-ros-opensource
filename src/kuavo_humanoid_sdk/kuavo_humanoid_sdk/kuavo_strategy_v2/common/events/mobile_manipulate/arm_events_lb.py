import time
import rospy
from typing import Any, Tuple, List
import time as time_module

from kuavo_humanoid_sdk.interfaces.data_types import KuavoManipulationMpcControlFlow
from kuavo_msgs.srv import changeArmCtrlModeRequest, changeArmCtrlModeResponse, changeArmCtrlMode

from kuavo_humanoid_sdk.kuavo_strategy_v2.common.events.base_event import BaseEvent, EventStatus
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.data_type import Pose, Tag, Frame
from kuavo_humanoid_sdk.interfaces.data_types import (
    KuavoPose,
    KuavoManipulationMpcCtrlMode,
    KuavoManipulationMpcFrame)

import numpy as np
from scipy.interpolate import CubicSpline
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.data_type import Transform3D
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.robot_sdk import RobotSDK

class EventArmMoveKeyPoint(BaseEvent):
    def __init__(self,
                 robot_sdk: RobotSDK,
                 timeout,
                 arm_control_mode: str,
                 pos_threshold: float,
                 angle_threshold: float,
                 use_relative_control: bool = False
                 ):
        """
        初始化手臂控制事件。

        参数：
            robot_sdk (RobotSDK): 机器人SDK实例。
            timeout (float): 事件超时时间。
            arm_control_mode (str): 手臂控制模式。
            pos_threshold (float): 位置误差阈值，单位米。
            angle_threshold (float): 角度误差阈值，单位弧度。
            use_relative_control (bool): 是否使用相对底座控制模式，默认False（世界坐标系控制）
        """
        super().__init__("EventArmMoveKeyPoint")

        self.robot_sdk = robot_sdk
        self.use_relative_control = use_relative_control  # 保存配置

        # members
        self.target: List
        self.target_wrench: List = None
        self.current_pose_id = 0
        self.pre_pose_id = -1
        self.current_left_pose: Pose = None  # 目标位置
        self.current_right_pose: Pose = None  # 目标位置
        
        # 完整轨迹执行相关
        self.complete_trajectory = None  # 完整规划的轨迹
        self.trajectory_planned = False  # 轨迹是否已规划
        self.trajectory_executing = False  # 轨迹是否正在执行
        self.trajectory_start_time = None  # 轨迹开始时间
        
        # 预规划轨迹相关
        self.use_precomputed_trajectory = False  # 是否使用预规划的轨迹
        self.precomputed_trajectory_segments = None  # 预规划的多段轨迹列表
        self.segment_sleep_time = 0.0  # 每段轨迹执行完后的额外休眠时间（秒）
        self.segment_finished_time = None  # 当前段执行完成的时间
        self.segment_waiting = False  # 当前段是否正在等待

        # params
        self.timeout = timeout  # 事件超时时间
        self.sub_goal_wait_time = 0.1  # 等待这么多时间再判断成功与否
        self.arm_control_mode = arm_control_mode  # 手臂控制模式
        self.pos_threshold = pos_threshold  # 位置误差阈值，单位米
        self.angle_threshold = angle_threshold  # 角度误差阈值，单位弧度（10度）

        # 安全范围参数（世界系）
        self.min_height = 0.3
        self.max_height = 1.8
        self.max_reach = 0.85
        self.min_reach = 0.15

        # 计时器相关变量
        self.threshold_met_start_time = None  # 开始计时的时间
        self.threshold_met = False  # 是否已经满足阈值条件
        self.delay = None

    def get_base_to_odom_transform(self):
        """
        根据配置获取Base到ODOM的坐标转换
        
        Returns:
            Transform3D: Base到ODOM的坐标转换
        """
        if self.use_relative_control:
            # 相对底座控制模式：固定底盘位姿为原点
            self.logger.info("🔵 使用相对底座控制模式（底盘位姿固定为原点）")
            transform_base_to_world = self.get_current_transform(source_frame=Frame.BASE, target_frame=Frame.ODOM)
            print("当前真实odom到躯干: ", transform_base_to_world.trans_pose.get_euler)
            return Transform3D(
                trans_pose=Pose(
                    pos=[0.0, 0.0, 0.0],
                    quat=[0.0, 0.0, 0.0, 1.0],  # 无旋转
                    frame=Frame.ODOM
                ),
                source_frame=Frame.BASE,
                target_frame=Frame.ODOM
            )
        else:
            # 世界坐标系控制模式：从TF获取实时位姿
            print("使用通过TF获取的实时位姿")
            return self.get_current_transform(source_frame=Frame.BASE, target_frame=Frame.ODOM)

    def calculate_elbow_y(self, target_y: float, is_left_arm: bool) -> float:
        """
        计算手肘Y坐标约束
        
        参数：
            target_y: 目标位置的Y坐标
            is_left_arm: 是否为左臂
            
        返回：
            计算后的手肘Y坐标
            
        规则：
            - 如果 |target_y| < 0.3，设置为 ±0.3（左手+0.3，右手-0.3）
            - 如果 |target_y| >= 0.3，在原有基础上向外扩展0.05（左手+0.05，右手-0.05）
        """
        if abs(target_y) < 0.4:
            return 0.4 if is_left_arm else -0.4
        else:
            return target_y + 0.05 if is_left_arm else target_y - 0.05

    def reset(self):
        """
        重置事件状态。
        """
        self.current_pose_id = 0
        self.pre_pose_id = -1
        self.current_left_pose: Pose = None  # 目标位置
        self.current_right_pose: Pose = None  # 目标位置
        self.target = None
        
        # 重置计时器相关变量
        self.threshold_met_start_time = None
        self.threshold_met = False
        
        # 重置轨迹执行相关变量
        self.complete_trajectory = None
        self.trajectory_planned = False
        self.trajectory_executing = False
        self.trajectory_start_time = None
        
        # 重置预规划轨迹相关变量
        self.use_precomputed_trajectory = False
        self.precomputed_trajectory_segments = None
        self.segment_sleep_time = 3.0
        self.segment_finished_time = None
        self.segment_waiting = False
        self.delay = None

        # 禁用WBC手臂轨迹控制
        # rospy.wait_for_service('/enable_wbc_arm_trajectory_control')
        # try:
        #     change_mode = rospy.ServiceProxy('/enable_wbc_arm_trajectory_control', changeArmCtrlMode)
        #     req = changeArmCtrlModeRequest()
        #     req.control_mode = 0
        #     res = change_mode(req)
        #     if res.result:
        #         rospy.loginfo("wbc轨迹控制模式已更改为 %d", req.control_mode)
        #     else:
        #         rospy.logerr("无法将wbc轨迹控制模式更改为 %d", req.control_mode)
        # except rospy.ServiceException as e:
        #     rospy.logerr("服务调用失败: %s", e)

    def get_current_transform(self, source_frame: Frame, target_frame: Frame) -> Transform3D:
        """
        将tf的变换转换为Transform3D对象。

        参数：
            source_frame (Frame): 源坐标系。
            target_frame (Frame): 目标坐标系。

        返回：
            Transform3D: 转换后的Transform3D对象。
        """
        tf_pose = self.robot_sdk.tools.get_tf_transform(target_frame, source_frame)

        source_to_target_pose = Pose(
            pos=tf_pose.position,
            quat=tf_pose.orientation,
            frame=target_frame
        )

        transform_source_to_target = Transform3D(
            trans_pose=source_to_target_pose,
            source_frame=source_frame,  # 源坐标系为Tag坐标系
            target_frame=target_frame  # 目标坐标系为里程计坐标系
        )

        return transform_source_to_target

    def open(self):
        """
        开始走到指定位置事件。
        """
        super().open()
        # 原有代码保持不变

        # self.robot_sdk.control.arm_reset()
        if self.arm_control_mode == "manipulation_mpc":
            self.robot_sdk.control.set_manipulation_mpc_mode(KuavoManipulationMpcCtrlMode.BaseArm)
        elif self.arm_control_mode == "fixed_base":
            self.robot_sdk.control.set_manipulation_mpc_mode(KuavoManipulationMpcCtrlMode.ArmOnly)
        else:
            self.robot_sdk.control.set_fixed_arm_mode()



        # WBC控制时使用
        # try:
        #     rospy.wait_for_service('/enable_wbc_arm_trajectory_control', timeout=2.0)
        #     change_mode = rospy.ServiceProxy('/enable_wbc_arm_trajectory_control', changeArmCtrlMode)
        #     req = changeArmCtrlModeRequest()
        #     req.control_mode = 2
        #     res = change_mode(req)
        #     if res.result:
        #         rospy.loginfo("wbc轨迹控制模式已更改为 %d", req.control_mode)
        #     else:
        #         rospy.logerr("无法将wbc轨迹控制模式更改为 %d", req.control_mode)
        # except rospy.ROSException as e:
        #     rospy.logerr("服务等待超时或不可用: %s", e)
        # except rospy.ServiceException as e:
        #     rospy.logerr("服务调用失败: %s", e)

    def set_arm_control_mode(self, arm_control_mode: str):
        """
        设置手臂控制模式。
        """

        if arm_control_mode == "manipulation_mpc":
            self.robot_sdk.control.set_manipulation_mpc_mode(KuavoManipulationMpcCtrlMode.BaseArm)
            print("设置手臂控制模式为manipulation_mpc")
        elif arm_control_mode == "fixed_base":
            self.robot_sdk.control.set_manipulation_mpc_mode(KuavoManipulationMpcCtrlMode.ArmOnly)
            print("设置手臂控制模式为fixed_base")
        else:
            self.robot_sdk.control.set_fixed_arm_mode()
            print("设置手臂控制模式为fixed_arm")

    def close(self):
        """
        关闭事件并重置状态。
        """
        super().close()
        # self.robot_sdk.control.set_manipulation_mpc_mode(KuavoManipulationMpcCtrlMode.NoControl)
        # self.robot_sdk.control.set_manipulation_mpc_control_flow(KuavoManipulationMpcControlFlow.ThroughFullBodyMpc)
        self.reset()


    def interpolate_joint_positions_bezier(self, start_joints, end_joints, num_points=20):
        """
        在两个关节位置之间进行贝塞尔曲线插值。

        参数：
            start_joints (list): 起始关节角度列表（14维）
            end_joints (list): 目标关节角度列表（14维）
            num_points (int): 插值点数量

        返回：
            List[list]: 插值后的关节角度列表
        """
        start_joints = np.array(start_joints)
        end_joints = np.array(end_joints)

        # 生成参数t
        t = np.linspace(0, 1, num_points)

        # 计算控制点（在起始点和终点之间创建平滑的贝塞尔曲线）
        # 使用二次贝塞尔曲线：P(t) = (1-t)²P₀ + 2(1-t)tP₁ + t²P₂
        # 其中P₁是控制点，位于起始点和终点的中间，稍微偏移以创建平滑曲线
        
        interp_joints = []
        for i in range(num_points):
            # 计算控制点（在起始点和终点之间，稍微偏移）
            mid_point = (start_joints + end_joints) / 2
            # 添加一些偏移以创建更平滑的曲线
            offset = (end_joints - start_joints) * 0.1  # 10%的偏移
            control_point = mid_point + offset

            # 二次贝塞尔曲线插值
            joint_pos = (1 - t[i])**2 * start_joints + \
                         2 * (1 - t[i]) * t[i] * control_point + \
                         t[i]**2 * end_joints

            interp_joints.append(joint_pos.tolist())

        return interp_joints

    def general_traj(self, num_points_per_segment=20, segment_sleep_time=3.0):
        """
        将多个关键点规划成多段贝塞尔轨迹（从 self.target 获取关键点）
        
        参数：
            num_points_per_segment (int): 每段轨迹的插值点数，默认20
            segment_sleep_time (float): 每段轨迹执行完后的等待时间（秒），默认3.0秒
            
        返回：
            List[List[list]]: 多段轨迹列表，trajectory_segments[i] 是第 i 段轨迹（14维关节角度列表）
                             例如：[[seg0_point0, seg0_point1, ...], [seg1_point0, seg1_point1, ...], ...]
        """
        print("开始生成多段轨迹")
        
        start_time = time_module.time()
        
        # 检查是否已设置目标
        if self.target is None:
            self.logger.error("❌ 未设置目标关键点，请先调用 set_target()")
            return None
        
        # 从 self.target 获取关键点
        left_target_list, right_target_list = self.target
        
        # 设置每段执行完后的等待时间
        self.segment_sleep_time = segment_sleep_time
        
        self.logger.info(f"🔵 开始规划多段轨迹，共 {len(left_target_list)} 个关键点，每段等待 {segment_sleep_time}秒...")
        
        # 获取起始关节位置
        start_joint_positions = self.robot_sdk.state.arm_joint_state().position
        current_joints = start_joint_positions.copy()
        
        # 获取坐标转换
        transform_base_to_odom = self.get_current_transform(
            source_frame=Frame.ODOM, target_frame=Frame.BASE
        )
        
        # 存储多段轨迹
        trajectory_segments = []
        
        # 遍历所有关键点，规划每段轨迹
        for idx, (left_target_pose, right_target_pose) in enumerate(zip(left_target_list, right_target_list)):
            self.logger.info(f"  规划第 {idx + 1}/{len(left_target_list)} 段轨迹...")
            
            # 转换到BASE坐标系
            if left_target_pose.frame == Frame.BASE:
                left_target_pose_in_base = left_target_pose
                right_target_pose_in_base = right_target_pose
            else:
                left_target_pose_in_base = transform_base_to_odom.apply_to_pose(
                    Pose(pos=left_target_pose.pos, quat=left_target_pose.quat, frame=Frame.ODOM)
                )
                right_target_pose_in_base = transform_base_to_odom.apply_to_pose(
                    Pose(pos=right_target_pose.pos, quat=right_target_pose.quat, frame=Frame.ODOM)
                )
            
            # 转换为KuavoPose
            left_target_kuavo_pose = KuavoPose(
                position=list(left_target_pose_in_base.pos),
                orientation=list(left_target_pose_in_base.quat)
            )
            right_target_kuavo_pose = KuavoPose(
                position=list(right_target_pose_in_base.pos),
                orientation=list(right_target_pose_in_base.quat)
            )
            
            # 计算手肘约束
            left_elbow_y = self.calculate_elbow_y(left_target_kuavo_pose.position[1], is_left_arm=True)
            right_elbow_y = self.calculate_elbow_y(right_target_kuavo_pose.position[1], is_left_arm=False)
            
            # 获取肘关节位置
            # 获取当前左右手臂link4的位置作为肘关节参考
            try:
                left_elbow_current = self.robot_sdk.tools.get_link_position("zarm_l4_link")
                right_elbow_current = self.robot_sdk.tools.get_link_position("zarm_r4_link")
                z_offset = 0.0
                y_offset = 0.3
                x_offset = 0.05
                if left_elbow_current is not None and right_elbow_current is not None:
                    # 使用当前肘关节的x和z坐标，但y坐标使用计算后的约束值
                    left_elbow_pos_xyz = [
                        x_offset, #left_elbow_current[0][0],  # 当前x坐标
                        y_offset,  # 计算后的y坐标约束
                        z_offset   # 当前z坐标
                    ]
                    right_elbow_pos_xyz = [
                        x_offset, # left_elbow_current[0][0],  # 当前x坐标
                        -y_offset,  # 计算后的y坐标约束
                        z_offset   # 当前z坐标
                    ]
                else:
                    # 如果获取失败，使用默认值，但y坐标仍使用计算后的约束值
                    left_elbow_pos_xyz = [
                        0.0,  # 默认x坐标
                        left_elbow_y,  # 计算后的y坐标约束
                        0.0   # 默认z坐标
                    ]
                    right_elbow_pos_xyz = [
                        0.0,  # 默认x坐标
                        right_elbow_y,  # 计算后的y坐标约束
                        0.0   # 默认z坐标
                    ]
                    self.logger.warn("无法获取当前肘关节位置，使用默认值")
                    
            except Exception as e:
                # 异常处理，使用默认值，但y坐标仍使用计算后的约束值
                left_elbow_pos_xyz = [
                    0.0,  # 默认x坐标
                    left_elbow_y,  # 计算后的y坐标约束
                    0.0   # 默认z坐标
                ]
                right_elbow_pos_xyz = [
                    0.0,  # 默认x坐标
                    right_elbow_y,  # 计算后的y坐标约束
                    0.0   # 默认z坐标
                ]
                self.logger.warn(f"获取肘关节位置时出错: {e}，使用默认值")

            print(f"左手肘位置约束: {left_elbow_pos_xyz}")
            print(f"右手肘位置约束: {right_elbow_pos_xyz}")
            
            # IK求解（带重试机制）
            target_joint_positions = None
            for retry in range(5):
                if retry > 0:
                    left_elbow_pos_xyz[1] -= 0.02
                    right_elbow_pos_xyz[1] += 0.02
                
                target_joint_positions = self.robot_sdk.arm.arm_ik(
                    left_pose=left_target_kuavo_pose,
                    right_pose=right_target_kuavo_pose,
                    left_elbow_pos_xyz=left_elbow_pos_xyz,
                    right_elbow_pos_xyz=right_elbow_pos_xyz,
                    arm_q0=current_joints
                )
                
                if target_joint_positions is not None:
                    break

            # 检查IK求解是否成功
            if target_joint_positions is None:
                self.logger.error(f"❌ 逆解失败：5次重试后仍无法找到可行解")
                return None

            #  return res.hand_poses.left_pose.joint_angles + res.hand_poses.right_pose.joint_angles
            left_joint_pose = target_joint_positions[:7]
            right_joint_pose = target_joint_positions[7:]
            print("left_joint: ",left_joint_pose)
            print("right_joint: ",right_joint_pose)
            if(left_joint_pose[1] > 0):
                print("镜像左->右")
                right_joint_pose = (left_joint_pose[0], -left_joint_pose[1],
                                    -left_joint_pose[2], left_joint_pose[3],
                                    -left_joint_pose[4], -left_joint_pose[5], left_joint_pose[6])
            else:
                print("镜像右->左")
                left_joint_pose = (right_joint_pose[0], -right_joint_pose[1],
                                    -right_joint_pose[2], right_joint_pose[3],
                                    -right_joint_pose[4], -right_joint_pose[5], right_joint_pose[6])

            target_joint_positions = left_joint_pose + right_joint_pose
            print("逆解关节：", target_joint_positions)
            
            # 使用贝塞尔插值生成这一段轨迹
            segment_trajectory = self.interpolate_joint_positions_bezier(
                current_joints,
                target_joint_positions,
                num_points=num_points_per_segment
            )
            
            # 添加到多段轨迹列表
            trajectory_segments.append(segment_trajectory)
            
            # 更新当前关节位置为这一段的终点（作为下一段的起点）
            current_joints = target_joint_positions
            
            self.logger.info(f"    第 {idx + 1} 段轨迹规划完成，包含 {len(segment_trajectory)} 个插值点")
        
        # 计算耗时
        end_time = time_module.time()
        elapsed_time = end_time - start_time
        
        self.logger.info(f"✅ 多段轨迹规划完成，共 {len(trajectory_segments)} 段，耗时: {elapsed_time:.3f}秒")
        
        # 保存预规划的轨迹并设置标志位
        self.precomputed_trajectory_segments = trajectory_segments
        self.use_precomputed_trajectory = True
        self.logger.info(f"🔵 已启用预规划轨迹模式，step 将直接使用规划好的轨迹")
        
        return trajectory_segments

    def interpolate_poses(self, start_pose, end_pose, num_points=20):
        """
        在两个笛卡尔空间姿态之间进行三次样条插值。

        参数：
            start_pose: 起始KuavoPose或Pose。
            end_pose: 终点KuavoPose或Pose。
            num_points (int): 插值点数量。

        返回：
            List[KuavoPose]: 插值后的KuavoPose列表。
        """
        # 提取位置
        start_pos = np.array(start_pose.position)
        end_pos = np.array(end_pose.position)

        # 提取四元数
        start_quat = np.array(start_pose.orientation)
        end_quat = np.array(end_pose.orientation)

        # 确保四元数方向一致（避免绕远路）
        if np.dot(start_quat, end_quat) < 0:
            end_quat = -end_quat

        # 生成参数t
        t = np.linspace(0, 1, num_points)

        # 位置插值 - 使用三次样条
        # 为了进行三次样条插值，我们需要在t, x, y, z上分别拟合样条

        # 四元数插值 - 球面线性插值 (SLERP)
        interp_poses = []
        for i in range(num_points):
            # 位置插值
            pos = start_pos * (1 - t[i]) + end_pos * t[i]
            pos = (pos[0], pos[1], pos[2])

            # 四元数球面插值
            # 计算四元数之间的夹角
            cos_half_theta = np.dot(start_quat, end_quat)
            cos_half_theta = np.clip(cos_half_theta, -1.0, 1.0)  # 确保在有效范围内

            if abs(cos_half_theta) >= 1.0:
                # 如果四元数几乎相同，直接使用起始四元数
                quat = start_quat
            else:
                half_theta = np.arccos(cos_half_theta)
                sin_half_theta = np.sqrt(1.0 - cos_half_theta * cos_half_theta)

                # 如果夹角足够大，使用SLERP插值
                if abs(sin_half_theta) < 0.001:
                    # 夹角太小，使用线性插值
                    quat = start_quat * (1 - t[i]) + end_quat * t[i]
                    quat = quat / np.linalg.norm(quat)  # 归一化
                else:
                    # SLERP公式
                    ratio_a = np.sin((1 - t[i]) * half_theta) / sin_half_theta
                    ratio_b = np.sin(t[i] * half_theta) / sin_half_theta
                    quat = start_quat * ratio_a + end_quat * ratio_b
                    quat = quat / np.linalg.norm(quat)  # 归一化

            # 创建新的KuavoPose
            interp_poses.append(KuavoPose(
                position=pos,
                orientation=quat.tolist()
            ))

        return interp_poses

    def util_set_arm_dof_to_work(self):
        """
        设置手臂的dof位置为双手抬起手肘弯曲姿态，用于检测零点。
        """
        joint_position_default = [
                    -0.45, 0.05, -0.27, -1.88, -0.00, 0.22, 0.53,
                    -0.45, -0.05, 0.27, -1.88, 0.00, -0.22, 0.53
                ]
        ratio = 0
        while True:
            ratio += 0.1
            ratio = np.min([ratio, 1.0])
            ratio = np.max([ratio, 0.0])
            joint_position = np.asarray(joint_position_default) * ratio
            joint_position = list(joint_position)
            self.robot_sdk.control.control_arm_joint_positions(
                joint_positions=joint_position # 手臂站立位置的关节角度，单位弧度
            )
            time.sleep(0.1)

    def util_set_arm_dof_to_zero(self):
        """
        手臂从工作姿态回到零点姿态。
        """
        joint_position_default = [
                    -0.45, 0.05, -0.27, -1.88, -0.00, 0.22, 0.53,
                    -0.45, -0.05, 0.27, -1.88, 0.00, -0.22, 0.53
                ]
        ratio = 1
        while True:
            ratio -= 0.1
            ratio = np.min([ratio, 1.0])
            ratio = np.max([ratio, 0.0])
            joint_position = np.asarray(joint_position_default) * ratio
            joint_position = list(joint_position)
            self.robot_sdk.control.control_arm_joint_positions(
                joint_positions=joint_position # 手臂站立位置的关节角度，单位弧度
            )
            time.sleep(0.1)

    def get_arm_pose_world(self, mode="tf"):
        """
        获取手臂在世界坐标系中的姿态。

        参数：
            mode (str): 获取模式，默认为"tf"。

        返回：
            Pose: 手臂的世界坐标系姿态。
        """
        assert mode in ["fk", "tf"], self.logger.error("mode must be 'fk' or 'tf'")

        if mode == "fk":
            # 用前向运动学计算手臂末端位置
            arm_state = self.robot_sdk.state.arm_joint_state()
            left_pose, right_pose = self.robot_sdk.arm.arm_fk(arm_state.position)


            # transform_base_to_world = self.get_current_transform(source_frame=Frame.BASE, target_frame=Frame.ODOM)

            # 改为固定值
            transform_base_to_world = Transform3D(
                trans_pose=Pose(
                    pos=[0.0, 0.0, 0.0],
                    quat=[0.0, 0.0, 0.0, 1.0],  # 无旋转
                    frame=Frame.BASE
                ),
                source_frame=Frame.BASE,
                target_frame=Frame.ODOM
)

            self.current_left_pose = transform_base_to_world.apply_to_pose(
                Pose(
                pos=left_pose.position,
                quat=left_pose.orientation,
                frame=Frame.BASE
            ))
            self.current_right_pose = transform_base_to_world.apply_to_pose(
                Pose(
                pos=right_pose.position,
                quat=right_pose.orientation,
                frame=Frame.BASE
            ))

        elif mode == "tf":
            # ============ 另外一种获取末端位置方式 (gazebo) =================
            # 通过TF获取手臂末端位置
            left_pose = self.robot_sdk.tools.get_link_pose(
                link_name="zarm_l7_end_effector",
                reference_frame=Frame.ODOM
            )
            right_pose: KuavoPose = self.robot_sdk.tools.get_link_pose(
                link_name="zarm_r7_end_effector",
                reference_frame=Frame.ODOM
            )

            self.current_left_pose = Pose(
                pos=left_pose.position,
                quat=left_pose.orientation,
                frame=Frame.ODOM
            )
            self.current_right_pose = Pose(
                pos=right_pose.position,
                quat=right_pose.orientation,
                frame=Frame.ODOM
            )

    def set_delay_time(self, user_set_time):
        """
        设置手臂执行到位延时时间
        """
        self.delay = user_set_time
        print("设置手臂延时时间：", self.delay)

    def step(self):
        """
        执行事件的每一步操作。
        """
        if self.target is None:
            self.logger.error("arm_event.target is None, cannot step!")
            return EventStatus.FAILED

        status = self.get_status()
        if status != EventStatus.RUNNING:
            return status

        left_target_list, right_target_list = self.target
        
        # ========== 使用预规划轨迹模式 ==========
        if self.use_precomputed_trajectory and self.precomputed_trajectory_segments is not None:
            self.logger.info(f"🔍 当前状态: pose_id={self.current_pose_id}, pre_id={self.pre_pose_id}, waiting={self.segment_waiting}")
            
            # 检查当前关键点是否已完成所有
            if self.current_pose_id >= len(self.precomputed_trajectory_segments):
                self.logger.info("✅ 所有预规划轨迹段已执行完成")
                return status
            
            # 检查是否是新的关键点
            if self.current_pose_id != self.pre_pose_id and not self.segment_waiting:
                self.logger.info(f"🔵 执行预规划轨迹第 {self.current_pose_id + 1}/{len(self.precomputed_trajectory_segments)} 段")
                self.pre_pose_id = self.current_pose_id
                
                # 施加末端力（如果有）
                if self.target_wrench is not None:
                    left_wrench_list, right_wrench_list = self.target_wrench
                    current_left_wrench = left_wrench_list[self.current_pose_id]
                    current_right_wrench = right_wrench_list[self.current_pose_id]
                    self.logger.info(f"🔵 施加末端力: 左{current_left_wrench}, 右{current_right_wrench}")
                    self.robot_sdk.arm.control_hand_wrench(
                        current_left_wrench,
                        current_right_wrench,
                    )
                    time.sleep(0.1)
                
                # 获取当前段的轨迹
                current_segment_trajectory = self.precomputed_trajectory_segments[self.current_pose_id]
                
                # 执行这段轨迹
                total_time = 1.5  # 每段0.8秒
                time_per_point = total_time / len(current_segment_trajectory) if len(current_segment_trajectory) > 1 else total_time
                
                self.logger.info(f"  执行轨迹段，包含 {len(current_segment_trajectory)} 个插值点")
                for joint_pos in current_segment_trajectory:
                    self.robot_sdk.control.control_arm_joint_positions(joint_positions=joint_pos)
                    if len(current_segment_trajectory) > 1:
                        time.sleep(time_per_point)
                    else:
                        time.sleep(0.1)
                
                self.logger.info(f"✅ 预规划轨迹第 {self.current_pose_id + 1} 段执行完成")
                
                # 开始等待计时
                self.segment_finished_time = time.time()
                self.segment_waiting = True
                self.logger.info(f"🔵 段执行完成，开始等待计时，等待时长: {self.segment_sleep_time}秒")
                return status  # 立即返回，下次 step() 再检查等待
            
            # 检查等待时间是否已满足
            if self.segment_waiting:
                self.logger.info(f"🔵 检查等待状态: segment_waiting={self.segment_waiting}")
                elapsed_time = time.time() - self.segment_finished_time
                segment_sleep_time = self.segment_sleep_time
                if self.delay is not None:
                    segment_sleep_time = self.delay[self.current_pose_id]

                if elapsed_time <segment_sleep_time:  # 还没等够，继续等待
                    self.logger.info(f"⏱️ 等待中... ({elapsed_time:.1f}s/{segment_sleep_time:.1f}s)")
                    time.sleep(0.1)  # 避免空转，每次等待0.1秒再检查
                    return status  # 继续等待，不移动到下一个关键点
                else:
                    # 等待时间已满足（elapsed_time >= segment_sleep_time），移到下一个关键点
                    self.logger.info(f"✅ 等待时间满足，移到下一个关键点")
                    self.segment_waiting = False
                    self.segment_finished_time = None
                    self.current_pose_id += 1
            
            return status
        
        # ========== 原有的逐段规划执行模式 ==========
        current_left_target_pose = left_target_list[self.current_pose_id]
        current_right_target_pose = right_target_list[self.current_pose_id]
        # self.get_arm_pose_world()
        current_meet = self.check_current_point_meet(current_left_target_pose, current_right_target_pose)
        if not current_meet and self.current_pose_id != self.pre_pose_id:
            if self.target_wrench is not None:
                left_wrench_list, right_wrench_list = self.target_wrench
                current_left_wrench = left_wrench_list[self.current_pose_id]
                current_right_wrench = right_wrench_list[self.current_pose_id]
                self.logger.info(f"🔵 开始施加末端力 {current_left_wrench}, {current_right_wrench}")
                self.robot_sdk.arm.control_hand_wrench(
                    current_left_wrench,
                    current_right_wrench,
                )
                # time.sleep(0.1)

            self.logger.info(f"🔵 开始执行关键点{self.current_pose_id + 1}")
            self.pre_pose_id = self.current_pose_id

           # 获取起始关节位置
            start_joint_positions = self.robot_sdk.state.arm_joint_state().position

                        # 获取从odom到base_link的变换
            transform_base_to_odom = self.get_current_transform(
                source_frame=Frame.ODOM,
                target_frame=Frame.BASE
            )
            print(f"🔵 当前torso_to_odom姿态: {transform_base_to_odom.trans_pose}")

            transform_odom_to_base = self.get_current_transform(
                source_frame=Frame.BASE,
                target_frame=Frame.ODOM
            )
            print(f"🔵 当前odom_to_torso姿态: {transform_odom_to_base.trans_pose}")
            
            # 根据目标点的frame做不同处理
            if current_left_target_pose.frame == Frame.BASE:
                # BASE坐标系：直接使用，不需要转换
                print(f"🔵 目标点在BASE坐标系，直接使用")
                left_target_pose_in_base = Pose(
                    pos=current_left_target_pose.pos,
                    quat=current_left_target_pose.quat,
                    frame=Frame.BASE
                )
                right_target_pose_in_base = Pose(
                    pos=current_right_target_pose.pos,
                    quat=current_right_target_pose.quat,
                    frame=Frame.BASE
                )
            else:
                # ODOM坐标系：转换到base_link坐标系
                print(f"🔵 目标点在ODOM坐标系，转换到BASE")
                left_target_pose_in_base = transform_base_to_odom.apply_to_pose(
                    Pose(
                        pos=current_left_target_pose.pos,
                        quat=current_left_target_pose.quat,
                        frame=Frame.ODOM
                    )
                )
                right_target_pose_in_base = transform_base_to_odom.apply_to_pose(
                    Pose(
                        pos=current_right_target_pose.pos,
                        quat=current_right_target_pose.quat,
                        frame=Frame.ODOM
                    )
                )

            # 将目标位姿转换为KuavoPose
            left_target_kuavo_pose = KuavoPose(
                position=list(left_target_pose_in_base.pos),
                orientation=list(left_target_pose_in_base.quat)
            )
            right_target_kuavo_pose = KuavoPose(
                position=list(right_target_pose_in_base.pos),
                orientation=list(right_target_pose_in_base.quat)
            )

            print(f"🔵 左臂目标位姿: {left_target_kuavo_pose}")
            print(f"🔵 右臂目标位姿: {right_target_kuavo_pose}")

            # 计算手肘Y坐标
            left_elbow_y = self.calculate_elbow_y(left_target_kuavo_pose.position[1], is_left_arm=True)
            right_elbow_y = self.calculate_elbow_y(right_target_kuavo_pose.position[1], is_left_arm=False)
            
            # 获取当前左右手臂link4的位置作为肘关节参考
            try:
                left_elbow_current = self.robot_sdk.tools.get_link_position("zarm_l4_link")
                right_elbow_current = self.robot_sdk.tools.get_link_position("zarm_r4_link")
                z_offset = 0.0
                y_offset = 0.3
                x_offset = 0.05
                if left_elbow_current is not None and right_elbow_current is not None:
                    # 使用当前肘关节的x和z坐标，但y坐标使用计算后的约束值
                    left_elbow_pos_xyz = [
                        x_offset, #left_elbow_current[0][0],  # 当前x坐标
                        y_offset,  # 计算后的y坐标约束
                        z_offset   # 当前z坐标
                    ]
                    right_elbow_pos_xyz = [
                        x_offset, # left_elbow_current[0][0],  # 当前x坐标
                        -y_offset,  # 计算后的y坐标约束
                        z_offset   # 当前z坐标
                    ]
                else:
                    # 如果获取失败，使用默认值，但y坐标仍使用计算后的约束值
                    left_elbow_pos_xyz = [
                        0.0,  # 默认x坐标
                        left_elbow_y,  # 计算后的y坐标约束
                        0.0   # 默认z坐标
                    ]
                    right_elbow_pos_xyz = [
                        0.0,  # 默认x坐标
                        right_elbow_y,  # 计算后的y坐标约束
                        0.0   # 默认z坐标
                    ]
                    self.logger.warn("无法获取当前肘关节位置，使用默认值")
                    
            except Exception as e:
                # 异常处理，使用默认值，但y坐标仍使用计算后的约束值
                left_elbow_pos_xyz = [
                    0.0,  # 默认x坐标
                    left_elbow_y,  # 计算后的y坐标约束
                    0.0   # 默认z坐标
                ]
                right_elbow_pos_xyz = [
                    0.0,  # 默认x坐标
                    right_elbow_y,  # 计算后的y坐标约束
                    0.0   # 默认z坐标
                ]
                self.logger.warn(f"获取肘关节位置时出错: {e}，使用默认值")

            print(f"左手肘位置约束: {left_elbow_pos_xyz}")
            print(f"右手肘位置约束: {right_elbow_pos_xyz}")

            # 获取目标位姿对应的关节角度
            target_joint_positions = None
            for retry in range(5):  # 最多重试5次
                if retry > 0:
                    # 调整肘关节位置：左手肘y坐标减少0.02，右手肘y坐标增加0.02
                    left_elbow_pos_xyz[1] -= 0.02
                    right_elbow_pos_xyz[1] += 0.02
                    self.logger.info(f"重试{retry}次，肘关节位置: 左{left_elbow_pos_xyz[1]:.3f}, 右{right_elbow_pos_xyz[1]:.3f}")
                
                target_joint_positions = self.robot_sdk.arm.arm_ik(
                    left_pose=left_target_kuavo_pose,
                    right_pose=right_target_kuavo_pose,
                    left_elbow_pos_xyz=left_elbow_pos_xyz,
                    right_elbow_pos_xyz=right_elbow_pos_xyz,
                    # arm_q0=start_joint_positions
                )
                
                if target_joint_positions is not None:
                    break  # 成功则跳出循环

            if target_joint_positions is None:
                self.logger.error("目标位姿逆运动学求解失败！")
                return EventStatus.FAILED

            #  return res.hand_poses.left_pose.joint_angles + res.hand_poses.right_pose.joint_angles
            left_joint_pose = target_joint_positions[:7]
            right_joint_pose = target_joint_positions[7:]
            print("left_joint: ",left_joint_pose)
            print("right_joint: ",right_joint_pose)
            if(left_joint_pose[1] > 0):
                print("镜像左->右")
                right_joint_pose = (left_joint_pose[0], -left_joint_pose[1],
                                    -left_joint_pose[2], left_joint_pose[3],
                                    -left_joint_pose[4], -left_joint_pose[5], left_joint_pose[6])
            else:
                print("镜像右->左")
                left_joint_pose = (right_joint_pose[0], -right_joint_pose[1],
                                    -right_joint_pose[2], right_joint_pose[3],
                                    -right_joint_pose[4], -right_joint_pose[5], right_joint_pose[6])

            target_joint_positions = left_joint_pose + right_joint_pose
            print("逆解关节：", target_joint_positions)

            # 在关节空间进行贝塞尔插值
            joint_trajectory = self.interpolate_joint_positions_bezier(
                start_joint_positions, 
                target_joint_positions, 
                num_points=20
            )

            # 执行关节轨迹
            total_time = 0.8
            time_per_point = total_time / len(joint_trajectory) if len(joint_trajectory) > 1 else total_time

            print(f"贝塞尔插值：joint_trajectory size: {len(joint_trajectory)}")
            for joint_pos in joint_trajectory:
                self.robot_sdk.control.control_arm_joint_positions(
                    joint_positions=joint_pos
                )
                if len(joint_trajectory) > 1:
                    time.sleep(time_per_point)
                    print(f"执行关节轨迹，时间: {time_per_point:.2f}s")
                else:
                    time.sleep(0.01)

        return status

    def arm_reset(self):
        """
        重置手臂到初始状态。
        """
        # self.robot_sdk.control.arm_reset()
        init_joint_pos = [0.0]*14
        self.robot_sdk.control.control_arm_joint_positions(joint_positions=init_joint_pos)
        time.sleep(0.2)
        self.logger.info("🔵 手臂已复位")

    def set_target(self, target: Any, *args, **kwargs):
        """
        设置事件的目标。

        参数：
            target (Any): 目标。
            `*args`: 额外的参数。
            `**kwargs`: 额外的关键字参数。

        返回：
            bool: 如果目标设置成功返回True，否则返回False。
        """

        # 直接检查整个 [left_target_list, right_target_list] 结构
        target_wrench = kwargs.get('arm_wrench', None)

        left_target_world = []
        right_target_world = []
        for left_key_pose, right_key_pose in zip(target[0], target[1]):
            if Frame.BASE == left_key_pose.frame:
                # BASE坐标系：保持原样，不转换，在step时相对底盘执行
                transform_base_to_world = self.get_current_transform(source_frame=Frame.BASE, target_frame=Frame.ODOM)
                self.logger.info(f'🔵 使用BASE坐标系，保持相对底盘的位置')
                if not self.use_relative_control:
                    left_key_pose_world = transform_base_to_world.apply_to_pose(left_key_pose)
                    right_key_pose_world = transform_base_to_world.apply_to_pose(right_key_pose)
                else:
                    left_key_pose_world = left_key_pose
                    right_key_pose_world = right_key_pose

            elif Frame.ODOM == left_key_pose.frame:
                left_key_pose_world = left_key_pose
                right_key_pose_world = right_key_pose

            elif Frame.TAG == left_key_pose.frame:
                tag = kwargs.get('tag', None)
                assert tag is not None, "Tag must be provided when target frame is TAG"
                transform_source_to_target = Transform3D(
                    trans_pose=tag.pose,
                    source_frame=Frame.TAG,  # 源坐标系为Tag坐标系
                    target_frame=Frame.ODOM  # 目标坐标系为里程计坐标系
                )

                left_key_pose_world = transform_source_to_target.apply_to_pose(left_key_pose)
                right_key_pose_world = transform_source_to_target.apply_to_pose(right_key_pose)

            # 如果左右手高度差大于0.05米，则取平均值
            if abs(left_key_pose_world.pos[2] - right_key_pose_world.pos[2]) > 0.05:
                average_height = (right_key_pose_world.pos[2] + left_key_pose_world.pos[2])/ 2.0
                left_key_pose_world.pos[2] = average_height
                right_key_pose_world.pos[2] = average_height

            #统一添加到list中    
            left_target_world.append(left_key_pose_world)
            right_target_world.append(right_key_pose_world) 

        res = super().set_target(target=[left_target_world, right_target_world], *args, **kwargs)

        if res:
            # 为了应对相对位置控制的情况，记录设置目标时机器人的位姿
            self.target_wrench = target_wrench
            self.get_arm_pose_world()

        return True

    def check_current_point_meet(self, current_left_target_pose: Pose, current_right_target_pose: Pose):
        """
        检查当前点是否满足目标条件。

        参数：
            current_left_target_pose (Pose): 当前左手目标姿态。
            current_right_target_pose (Pose): 当前右手目标姿态。

        返回：
            bool: 如果满足条件返回True，否则返回False。
        """

        self.get_arm_pose_world()  # 获取当前手臂位置（ODOM坐标系）

        current_id = self.current_pose_id

        # 统一坐标系：如果目标是BASE坐标系，需要转换到ODOM坐标系再比较
        if current_left_target_pose.frame == Frame.BASE:
            print(f"🔵 目标在BASE坐标系，转换到ODOM进行误差计算")
            transform_base_to_odom = self.get_current_transform(
                source_frame=Frame.BASE,
                target_frame=Frame.ODOM
            )
            left_target_in_odom = transform_base_to_odom.apply_to_pose(current_left_target_pose)
            right_target_in_odom = transform_base_to_odom.apply_to_pose(current_right_target_pose)
            print(f"   左臂目标 BASE: {current_left_target_pose.pos} -> ODOM: {left_target_in_odom.pos}")
        else:
            # 目标已经在ODOM坐标系
            print(f"🔵 目标在ODOM坐标系，直接计算误差")
            left_target_in_odom = current_left_target_pose
            right_target_in_odom = current_right_target_pose

        # print('🔵 当前左手点:', self.current_left_pose)
        # print('🔵 当前右手点:', self.current_right_pose)

        # 统一计算误差（都在ODOM坐标系）
        pos_err_left = left_target_in_odom.position_l2_norm(self.current_left_pose)
        angle_err_left = left_target_in_odom.angle(self.current_left_pose)
        pos_err_right = right_target_in_odom.position_l2_norm(self.current_right_pose)
        angle_err_right = right_target_in_odom.angle(self.current_right_pose)

        print(f"手臂误差：{pos_err_left:.4f} | {np.rad2deg(angle_err_left):.4f} | {pos_err_right:.4f} | {np.rad2deg(angle_err_right):.4f}")
        # print(f"当前左手臂位置误差: {pos_err_left:.4f} 米，角度误差: {np.rad2deg(angle_err_left):.4f} 度")
        # print(f"当前右手臂位置误差: {pos_err_right:.4f} 米，角度误差: {np.rad2deg(angle_err_right):.4f} 度")

        # 检查是否满足阈值条件
        threshold_met_now = (
            pos_err_left <= self.pos_threshold and
            angle_err_left <= self.angle_threshold and
            pos_err_right <= self.pos_threshold and
            angle_err_right <= self.angle_threshold
        )

        # 如果刚满足阈值条件，开始计时
        if threshold_met_now and not self.threshold_met:
            self.threshold_met_start_time = time.time()
            self.threshold_met = True
            print(f"🔵 关键点{self.current_pose_id + 1}满足阈值条件，开始计时...")

        # 如果还没有满足阈值条件，重置计时器
        if not threshold_met_now:
            self.threshold_met = False
            self.threshold_met_start_time = None
            return False

        # 检查是否已经等待足够的时间
        if self.threshold_met_start_time is not None:
            elapsed_time = time.time() - self.threshold_met_start_time
            if elapsed_time < self.sub_goal_wait_time:
                print(f"⏱️ 关键点{self.current_pose_id + 1}等待中... ({elapsed_time:.1f}s/{self.sub_goal_wait_time}s)")
                return False

        # 等待时间已满足，执行后续逻辑
        print(f"✅ 关键点{self.current_pose_id + 1}等待时间满足，开始最终检查...")

        # 再计算一次，等待了之后可能会更准
        self.get_arm_pose_world()
        pos_err_left = current_left_target_pose.position_l2_norm(self.current_left_pose)
        angle_err_left = current_left_target_pose.angle(self.current_left_pose)
        pos_err_right = current_right_target_pose.position_l2_norm(self.current_right_pose)
        angle_err_right = current_right_target_pose.angle(self.current_right_pose)

        # 当前位置执行到位
        print(
            f"✅ 关键点{self.current_pose_id + 1}执行到位 | 左手误差: {pos_err_left:.4f}m, {np.rad2deg(angle_err_left):.2f}°")
        print(
            f"✅ 关键点{self.current_pose_id + 1}执行到位 | 右手误差: {pos_err_right:.4f}m, {np.rad2deg(angle_err_right):.2f}°")

        transform_base_to_world = self.get_base_to_odom_transform()
        print(f'此刻base在odom下的位置：{transform_base_to_world.trans_pose}')
        current_id += 1
        self.current_pose_id = current_id

        # 重置计时器状态，为下一个关键点做准备
        self.threshold_met = False
        self.threshold_met_start_time = None

        return True

    def _check_target_valid(self, target, *args, **kwargs) -> bool:
        """
        检查目标是否有效。

        参数：
            target: 目标。
            `*args`: 额外的参数。
            `**kwargs`: 额外的关键字参数。

        返回：
            bool: 如果目标有效返回True，否则返回False。
        """
        # 如果是 [list, list]，递归检查
        target_wrench = kwargs.get('arm_wrench', None)

        if isinstance(target, (list, tuple)) and len(target) == 2:
            left_list, right_list = target

            if not (isinstance(left_list, list) and isinstance(right_list, list)):
                print("❌目标不是两个列表")
                return False

            if target_wrench is not None:
                if not isinstance(target_wrench, (list, tuple)):
                    # print(target_wrench)
                    print("❌目标力矩必须是列表的列表")
                    return False
                left_wrench, right_wrench = target_wrench
                if len(left_wrench) != len(left_list):
                    print("❌左手力矩列表长度与关键点列表长度不匹配")
                    return False
                if len(right_wrench) != len(right_list):
                    print("❌右手力矩列表长度与关键点列表长度不匹配")
                    return False

            # 检查每个关键点的有效性和可达性
            for i, (left_pose, right_pose) in enumerate(zip(left_list, right_list)):
                if not isinstance(left_pose, Pose) or not isinstance(right_pose, Pose):
                    print(f"关键点{i + 1}不是Pose对象: {type(left_pose)}, {type(right_pose)}")
                    return False

                # 检查坐标系有效性
                if left_pose.frame not in [Frame.ODOM, Frame.BASE, Frame.TAG]:
                    print(f"关键点{i + 1}左手臂坐标系无效: {left_pose.frame}")
                    return False
                if right_pose.frame not in [Frame.ODOM, Frame.BASE, Frame.TAG]:
                    print(f"关键点{i + 1}右手臂坐标系无效: {right_pose.frame}")
                    return False

                # 检查位置有效性和可达性
                if not self._check_arm_pose_validity(left_pose, "左手臂", i + 1, tag=kwargs.get('tag', None)):
                    return False
                if not self._check_arm_pose_validity(right_pose, "右手臂", i + 1, tag=kwargs.get('tag', None)):
                    return False

            return True
        # 单个 Pose 检查
        if not isinstance(target, Pose):
            print("❌目标位置必须是Pose对象")
            return False
        if target.frame not in [Frame.ODOM, Frame.BASE]:
            print("❌目标位姿的坐标系必须是'base_link'或'odom'")
            return False
        return True

    def _check_arm_pose_validity(self, pose: Pose, arm_name: str, point_id: int, *args, **kwargs) -> bool:
        """
        检查手臂姿态的有效性。

        参数：
            pose (Pose): 位姿。
            arm_name (str): 手臂名称。
            point_id (int): 点ID。
            `*args`: 额外的参数。
            `**kwargs`: 额外的关键字参数。

        返回：
            bool: 如果姿态有效返回True，否则返回False。
        """
        # try:
        #     # 提前计算所有需要的转换

        #     # 1. 根据输入坐标系，计算odom坐标系下的位置
        #     if pose.frame == Frame.BASE:
        #         # 转换到odom坐标系
        #         transform_base_to_odom = self.get_current_transform(
        #             source_frame=Frame.BASE,
        #             target_frame=Frame.ODOM
        #         )
        #         pose_in_odom = transform_base_to_odom.apply_to_pose(pose)
        #         pose_in_base = pose  # 已经在base_link坐标系下
        #     elif pose.frame == Frame.ODOM:
        #         # 已经在odom坐标系下
        #         pose_in_odom = pose
        #         # 转换到base_link坐标系
        #         transform_odom_to_base = self.get_current_transform(
        #             source_frame=Frame.ODOM,
        #             target_frame=Frame.BASE
        #         )
        #         pose_in_base = transform_odom_to_base.apply_to_pose(pose)
        #     elif pose.frame == Frame.TAG:
        #         # 对于TAG坐标系，使用self.tag转换到odom坐标系
        #         tag: Tag = kwargs.get('tag', None)
        #         assert tag is not None, "Tag must be provided when target frame is TAG"
        #         transform_tag_to_odom = Transform3D(
        #             trans_pose=tag.pose,
        #             source_frame=Frame.TAG,
        #             target_frame=Frame.ODOM
        #         )
        #         pose_in_odom = transform_tag_to_odom.apply_to_pose(pose)

        #         # 再转换到base_link坐标系
        #         transform_odom_to_base = self.get_current_transform(
        #             source_frame=Frame.ODOM,
        #             target_frame=Frame.BASE
        #         )
        #         pose_in_base = transform_odom_to_base.apply_to_pose(pose_in_odom)
        #     else:
        #         print(f"❌ 关键点{point_id} {arm_name}坐标系不支持: {pose.frame}")
        #         return False

        #     # 2. 检查高度范围（在odom坐标系下）
        #     height = pose_in_odom.pos[2]
        #     if height < self.min_height or height > self.max_height:
        #         print(
        #             f"❌ 关键点{point_id} {arm_name}高度 {height:.2f}m 超出工作范围({self.min_height}m-{self.max_height}m)")
        #         return False

        #     # 3. 检查末端pose相对于肩关节的位置
        #     if arm_name == "左手臂":
        #         shoulder_joint_name = "zarm_l1_link"  # 左臂肩关节
        #     else:
        #         shoulder_joint_name = "zarm_r1_link"  # 右臂肩关节

        #     try:

        #         shoulder_pos_data = self.robot_sdk.tools.get_link_position(shoulder_joint_name)
        #         if shoulder_pos_data is not None:
        #             # 解包位置和方向，只使用位置部分
        #             shoulder_pos, _ = shoulder_pos_data

        #         if shoulder_pos is not None:
        #             # 计算末端pose相对于肩关节的位置向量（只在x,y平面上）
        #             shoulder_to_end_xy = np.array([pose_in_base.pos[0] - shoulder_pos[0],
        #                                            pose_in_base.pos[1] - shoulder_pos[1]])

        #             # 计算相对于肩关节的水平距离（x,y平面）
        #             shoulder_distance_xy = np.linalg.norm(shoulder_to_end_xy)

        #             # 检查最小距离
        #             if shoulder_distance_xy < self.min_reach:
        #                 self.logger.error(
        #                     f"❌ 关键点{point_id} {arm_name}相对于肩关节水平距离 {shoulder_distance_xy:.2f}m 小于最小可达距离({self.min_reach}m)")
        #                 return False

        #             # 检查最大距离
        #             if shoulder_distance_xy > self.max_reach:
        #                 self.logger.error(
        #                     f"❌ 关键点{point_id} {arm_name}相对于肩关节水平距离 {shoulder_distance_xy:.2f}m 超出最大可达距离({self.max_reach}m)")
        #                 return False

        #             self.logger.info(f"✅ 关键点{point_id} {arm_name}相对于肩关节水平距离 {shoulder_distance_xy:.2f}m 在允许范围内")
        #         else:
        #             self.logger.warn(f"⚠️ 无法获取{arm_name}肩关节位置，跳过肩关节距离检查")
        #     except Exception as e:
        #         self.logger.warn(f"⚠️ 获取{arm_name}肩关节位置出错，跳过肩关节距离检查: {e}")

        #     self.logger.info(f"✅ 关键点{point_id} {arm_name}位置检查通过 - 高度(odom):{height:.2f}m")
        #     return True

        # except Exception as e:
        #     self.logger.error(f"❌ 关键点{point_id} {arm_name}位置检查出错: {e}")
        #     return False

        return True

    def _check_failed(self):
        """
        检查事件是否失败。

        返回：
            bool: 如果事件失败返回True，否则返回False。
        """
        return False

    def _check_success(self):
        """
        检查事件是否成功。

        返回：
            bool: 如果事件成功返回True，否则返回False。
        """
        if self.current_pose_id >= len(self.target[0]):
            self.status = EventStatus.SUCCESS
            print("✅运动执行到位")
            return True

        return False

    def interpolate_poses_endff(self, start_pose, end_pose, max_speed=0.2, time_step=0.01):
        """
        基于速度控制的姿态插值
        位置使用三次样条插值，姿态使用SLERP，总时间由距离和最大速度决定

        参数：
            start_pose: 起始姿态（KuavoPose或Pose）
            end_pose: 目标姿态（KuavoPose或Pose）
            max_speed: 最大移动速度 (m/s)，默认0.2m/s
            time_step: 时间步长 (s)，默认0.01s

        返回：
            List[KuavoPose]: 插值后的姿态列表（含时间戳）
            List[float]: 每个姿态对应的时间戳（相对于起始时刻）
        """
        # 提取位置和姿态
        start_pos = np.array(start_pose.position, dtype=np.float64)
        end_pos = np.array(end_pose.position, dtype=np.float64)
        start_quat = np.array(start_pose.orientation, dtype=np.float64)
        end_quat = np.array(end_pose.orientation, dtype=np.float64)

        # 1. 计算位置距离和总时间
        pos_diff = end_pos - start_pos
        distance = np.linalg.norm(pos_diff)  # 直线距离

        if distance < 1e-6:  # 距离过近，无需运动
            return [KuavoPose(position=tuple(start_pos), orientation=start_quat.tolist())], [0.0]

        # 总时间由距离和最大速度决定（确保不超过速度限制）
        total_time = distance / max_speed
        
        # 限制最大插值点数为20
        max_points = 20
        # 根据总时间和最大点数计算实际时间步长
        actual_time_step = total_time / (max_points - 1) if max_points > 1 else total_time
        
        # 生成时间戳（最多20个点）
        timestamps = np.linspace(0.0, total_time, max_points)
        num_points = len(timestamps)

        # 2. 位置三次样条插值（确保速度连续）
        # 构造控制点（加入起始和终止速度约束，这里设为0，确保平滑启停）
        t_control = [0.0, total_time/3, 2*total_time/3, total_time]
        # 中间点使用线性插值，确保轨迹通过起点和终点
        pos_control = np.array([
            start_pos,
            start_pos + pos_diff/3,
            start_pos + 2*pos_diff/3,
            end_pos
        ])
        # 对x, y, z分别做三次样条
        cs_x = CubicSpline(t_control, pos_control[:, 0], bc_type=((1, 0.0), (1, 0.0)))  # 始末速度为0
        cs_y = CubicSpline(t_control, pos_control[:, 1], bc_type=((1, 0.0), (1, 0.0)))
        cs_z = CubicSpline(t_control, pos_control[:, 2], bc_type=((1, 0.0), (1, 0.0)))

        # 3. 四元数SLERP插值（保持姿态平滑）
        # 修正四元数方向（取最短路径）
        if np.dot(start_quat, end_quat) < 0:
            end_quat = -end_quat

        # 4. 生成插值轨迹
        interp_poses = []
        for t in timestamps:
            # 位置插值（三次样条）
            pos = np.array([cs_x(t), cs_y(t), cs_z(t)])
            pos = tuple(pos)

            # 姿态插值（SLERP）
            t_norm = t / total_time  # 归一化到[0,1]
            cos_half_theta = np.dot(start_quat, end_quat)
            cos_half_theta = np.clip(cos_half_theta, -1.0, 1.0)

            if abs(cos_half_theta) >= 1.0:
                quat = start_quat.copy()
            else:
                half_theta = np.arccos(cos_half_theta)
                sin_half_theta = np.sqrt(1.0 - cos_half_theta**2)

                if abs(sin_half_theta) < 1e-6:
                    # 角度过小时用线性插值
                    quat = start_quat * (1 - t_norm) + end_quat * t_norm
                else:
                    # SLERP公式
                    ratio_a = np.sin((1 - t_norm) * half_theta) / sin_half_theta
                    ratio_b = np.sin(t_norm * half_theta) / sin_half_theta
                    quat = start_quat * ratio_a + end_quat * ratio_b

                quat /= np.linalg.norm(quat)  # 归一化

            interp_poses.append(KuavoPose(
                position=pos,
                orientation=quat.tolist()
            ))

        return interp_poses, timestamps

    def plan_and_execute_smooth_trajectory_endff(self, max_speed=0.2, time_step=0.01):
        """
        一次性规划整条轨迹并平滑执行（用于endff模式，使用末端执行器位姿控制）

        参数：
            max_speed (float): 最大移动速度 (m/s)
            time_step (float): 时间步长 (s)

        返回：
            EventStatus: 执行状态
        """
        if self.target is None:
            self.logger.error("arm_event.target is None, cannot plan trajectory!")
            return EventStatus.FAILED

        status = self.get_status()
        if status != EventStatus.RUNNING:
            return status

        self.get_arm_pose_world()
        left_target_list, right_target_list = self.target

        # 获取起始位姿
        left_start_kuavo_pose = KuavoPose(
            position=self.current_left_pose.pos,
            orientation=self.current_left_pose.quat.tolist()
        )
        right_start_kuavo_pose = KuavoPose(
            position=self.current_right_pose.pos,
            orientation=self.current_right_pose.quat.tolist()
        )

        # 为所有关键点生成完整轨迹
        all_left_trajectories = []
        all_right_trajectories = []
        all_left_timestamps = []
        all_right_timestamps = []

        self.logger.info(f"🔵 开始规划整条轨迹，共{len(left_target_list)}个关键点")

        for pose_id in range(len(left_target_list)):
            current_left_target_pose = left_target_list[pose_id]
            current_right_target_pose = right_target_list[pose_id]

            print(f"🔵 关键点{pose_id+1} - 左臂目标位姿: {current_left_target_pose}")
            print(f"🔵 关键点{pose_id+1} - 右臂目标位姿: {current_right_target_pose}")

            # 将目标位姿转换为KuavoPose
            left_target_kuavo_pose = KuavoPose(
                position=list(current_left_target_pose.pos),
                orientation=list(current_left_target_pose.quat)
            )
            right_target_kuavo_pose = KuavoPose(
                position=list(current_right_target_pose.pos),
                orientation=list(current_right_target_pose.quat)
            )

            # 确定起始位姿
            if pose_id == 0:
                # 第一个关键点，从当前位置开始
                left_start = left_start_kuavo_pose
                right_start = right_start_kuavo_pose
            else:
                # 后续关键点，从上一个关键点的结束位置开始
                left_start = all_left_trajectories[-1][-1]
                right_start = all_right_trajectories[-1][-1]

            # 生成当前关键点的轨迹
            left_arm_traj, left_timestamps = self.interpolate_poses_endff(
                left_start, left_target_kuavo_pose, max_speed=max_speed, time_step=time_step
            )
            right_arm_traj, right_timestamps = self.interpolate_poses_endff(
                right_start, right_target_kuavo_pose, max_speed=max_speed, time_step=time_step
            )

            # 调整时间戳（累加到总时间）
            if pose_id > 0:
                last_left_time = all_left_timestamps[-1][-1] if all_left_timestamps else 0
                last_right_time = all_right_timestamps[-1][-1] if all_right_timestamps else 0
                base_time = max(last_left_time, last_right_time)

                left_timestamps = [t + base_time for t in left_timestamps]
                right_timestamps = [t + base_time for t in right_timestamps]

            all_left_trajectories.append(left_arm_traj)
            all_right_trajectories.append(right_arm_traj)
            all_left_timestamps.append(left_timestamps)
            all_right_timestamps.append(right_timestamps)

            self.logger.info(f"关键点{pose_id+1}轨迹规划完成，左臂{len(left_arm_traj)}个点，右臂{len(right_arm_traj)}个点")

        # 合并所有轨迹
        total_left_trajectory = []
        total_right_trajectory = []
        total_left_timestamps = []
        total_right_timestamps = []

        for i in range(len(all_left_trajectories)):
            total_left_trajectory.extend(all_left_trajectories[i])
            total_right_trajectory.extend(all_right_trajectories[i])
            total_left_timestamps.extend(all_left_timestamps[i])
            total_right_timestamps.extend(all_right_timestamps[i])

        # 执行整条轨迹
        self.logger.info(f"🔵 开始执行整条轨迹，左臂{len(total_left_trajectory)}个点，右臂{len(total_right_trajectory)}个点")

        # 左右臂轨迹点数相同（由interpolate_poses_endff保证每段都是20个点）
        num_points = len(total_left_trajectory)

        for i in range(num_points):
            self.robot_sdk.control.control_robot_end_effector_pose(
                left_pose=total_left_trajectory[i],
                right_pose=total_right_trajectory[i],
                frame=KuavoManipulationMpcFrame.WorldFrame,
            )

            if i < num_points - 1:  # 最后一个点不需要延时
                left_sleep = total_left_timestamps[i+1] - total_left_timestamps[i]
                right_sleep = total_right_timestamps[i+1] - total_right_timestamps[i]
                sleep_time = max(left_sleep, right_sleep)
                time.sleep(sleep_time)

        # 更新当前关键点ID
        self.current_pose_id = len(left_target_list)

        self.logger.info("✅ 整条轨迹执行完成")
        return EventStatus.SUCCESS

    def step_endff(self):
        """
        执行事件的每一步操作（使用末端执行器笛卡尔控制）。
        """
        if self.target is None:
            self.logger.error("arm_event.target is None, cannot step!")
            return EventStatus.FAILED

        status = self.get_status()
        if status != EventStatus.RUNNING:
            return status

        return self.plan_and_execute_smooth_trajectory_endff()

        self.get_arm_pose_world()
        left_target_list, right_target_list = self.target
        current_left_target_pose = left_target_list[self.current_pose_id]
        current_right_target_pose = right_target_list[self.current_pose_id]
        
        if self.current_pose_id != self.pre_pose_id:
            if self.target_wrench is not None:
                left_wrench_list, right_wrench_list = self.target_wrench
                current_left_wrench = left_wrench_list[self.current_pose_id]
                current_right_wrench = right_wrench_list[self.current_pose_id]
                self.logger.info(f"🔵 开始施加末端力 {current_left_wrench}, {current_right_wrench}")
                self.robot_sdk.arm.control_hand_wrench(
                    current_left_wrench,
                    current_right_wrench,
                )

            self.logger.info(f"🔵 开始执行关键点{self.current_pose_id + 1}")
            self.pre_pose_id = self.current_pose_id

            # 将 Pose 转换为 KuavoPose 类型
            left_start_kuavo_pose = KuavoPose(
                position=self.current_left_pose.pos,
                orientation=self.current_left_pose.quat.tolist()
            )
            left_target_kuavo_pose = KuavoPose(
                position=current_left_target_pose.pos,
                orientation=current_left_target_pose.quat.tolist()
            )
            right_start_kuavo_pose = KuavoPose(
                position=self.current_right_pose.pos,
                orientation=self.current_right_pose.quat.tolist()
            )
            right_target_kuavo_pose = KuavoPose(
                position=current_right_target_pose.pos,
                orientation=current_right_target_pose.quat.tolist()
            )

            left_arm_traj, left_timestamps = self.interpolate_poses_endff(left_start_kuavo_pose, left_target_kuavo_pose, max_speed=0.2)
            right_arm_traj, right_timestamps = self.interpolate_poses_endff(right_start_kuavo_pose, right_target_kuavo_pose, max_speed=0.2)

            # 左右臂轨迹点数都是20（由interpolate_poses_endff保证）
            num_points = len(left_arm_traj)
            self.logger.info(f"🔵 轨迹点数 - 左臂:{len(left_arm_traj)}, 右臂:{len(right_arm_traj)}")
            self.logger.info(f"🔵 开始执行left_start_kuavo_pose{left_start_kuavo_pose} left_target_kuavo_pose{left_target_kuavo_pose}")
            self.logger.info(f"🔵 开始执行right_start_kuavo_pose{right_start_kuavo_pose} right_target_kuavo_pose{right_target_kuavo_pose}")

            for i in range(num_points):
                self.robot_sdk.control.control_robot_end_effector_pose(
                    left_pose=left_arm_traj[i],
                    right_pose=right_arm_traj[i],
                    frame=KuavoManipulationMpcFrame.WorldFrame,
                )
                if i < num_points - 1:  # 最后一个点不需要延时
                    left_sleep = left_timestamps[i+1] - left_timestamps[i]
                    right_sleep = right_timestamps[i+1] - right_timestamps[i]
                    sleep_time = max(left_sleep, right_sleep)
                    time.sleep(sleep_time)
            time.sleep(1)
            self.current_pose_id += 1

        return status

