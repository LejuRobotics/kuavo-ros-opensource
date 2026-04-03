from kuavo_humanoid_sdk.kuavo_strategy_pytree.common.robot_sdk import RobotSDK
from kuavo_humanoid_sdk.kuavo_strategy_pytree.utils.utils import normalize_angle
from kuavo_humanoid_sdk.interfaces.data_types import (
    KuavoPose,
    KuavoManipulationMpcCtrlMode,
    KuavoArmCtrlMode,
    KuavoManipulationMpcFrame)
from kuavo_humanoid_sdk.kuavo_strategy_pytree.common.data_type import Pose, Tag, Frame, Transform3D, WheelArmFrame
from kuavo_humanoid_sdk.interfaces.data_types import KuavoManipulationMpcControlFlow

import threading
from concurrent.futures import ThreadPoolExecutor, Future
from typing import List, Tuple
import time
import numpy as np
import copy
import rospy
from kuavo_msgs.srv import lbTimedPosCmd, lbTimedPosCmdRequest
from kuavo_msgs.srv import setRuckigPlannerParams, setRuckigPlannerParamsRequest


def resample_and_execute_traj(traj, total_time, publish_fn, publish_rate=100.0):
    """
    对轨迹进行重采样并以固定频率下发
    
    参数：
        traj: 轨迹数据，每个元素是一个位置列表
        total_time: 总执行时间（秒）
        publish_fn: 下发函数，接受一个位置列表作为参数
        publish_rate: 下发频率（Hz），默认100Hz
    """
    num_points_orig = len(traj)
    if num_points_orig < 1:
        return
    
    if num_points_orig == 1:
        publish_fn(traj[0])
        return
    
    # 根据总时间和固定频率计算需要的点数
    num_points_target = int(total_time * publish_rate) + 1
    dt = 1.0 / publish_rate
    
    # 对轨迹进行线性插值重采样
    traj_np = np.array(traj)
    t_orig = np.linspace(0, 1, num_points_orig)
    t_target = np.linspace(0, 1, num_points_target)
    
    resampled_traj = []
    for t in t_target:
        idx = np.searchsorted(t_orig, t)
        if idx == 0:
            resampled_traj.append(traj_np[0].tolist())
        elif idx >= num_points_orig:
            resampled_traj.append(traj_np[-1].tolist())
        else:
            t0, t1 = t_orig[idx - 1], t_orig[idx]
            alpha = (t - t0) / (t1 - t0)
            interpolated = (1 - alpha) * traj_np[idx - 1] + alpha * traj_np[idx]
            resampled_traj.append(interpolated.tolist())

    # 以固定频率下发
    for i, pos in enumerate(resampled_traj):
        publish_fn(pos)
        if i < len(resampled_traj) - 1:
            time.sleep(dt)


def pose_to_list(pose):
    """
    将KuavoPose对象转换为数值列表 [x, y, z, qx, qy, qz, qw]
    如果已经是列表格式则直接返回
    """
    if hasattr(pose, 'position') and hasattr(pose, 'orientation'):
        # KuavoPose 对象
        return list(pose.position) + list(pose.orientation)
    else:
        # 已经是列表格式
        return pose


def transform_pose_from_tag_to_world(tag: Tag, pose: Pose) -> Pose:
    """
    将tag坐标系下的位姿转换到世界坐标系下。

    参数：
        tag (Tag): Tag对象，包含位姿信息。
        pose (Pose): 需要转换的位姿。

    返回：
        Pose: 转换后的Pose对象。
    """
    # 转换stand_pose_in_tag到世界坐标系。注意、需要搞清楚tag的坐标定义和机器人的坐标定义
    transform_tag_to_world = Transform3D(
        trans_pose=tag.pose,
        source_frame=Frame.TAG,  # 源坐标系为Tag坐标系
        target_frame=Frame.ODOM  # 目标坐标系为里程计坐标系
    )
    stand_pose_in_world = transform_tag_to_world.apply_to_pose(
        pose  # 将站立位置转换到世界坐标系
    )
    return stand_pose_in_world


class HeadAPI:
    """
    头部控制API
    """

    def __init__(self, robot_sdk: RobotSDK):
        self.robot_sdk = robot_sdk
        self._pool = ThreadPoolExecutor(max_workers=2)

    def _move_head_traj(self,
                        head_traj: List[Tuple[float, float]] = [],  # 头部目标点列表，格式为[(yaw, pitch), ...
                        ):
        for pair in head_traj:
            yaw, pitch = pair
            self.robot_sdk.control.control_head(yaw, pitch)
            time.sleep(0.7)

    def move_head_traj(self,
                       head_traj: List[Tuple[float, float]] = [],  # 头部目标点列表，格式为[(yaw, pitch), ...
                       asynchronous: bool = False,  # 布尔值，指定运动命令是否为异步。默认值为 false，表示函数会阻塞
                       ) -> Future:
        if asynchronous:
            fut = self._pool.submit(self._move_head_traj, head_traj)
            return fut  # 外部拿到 Future

        else:
            self._move_head_traj(head_traj)
            return None


class ArmAPI:
    """
    根据手臂和躯干控制接口，封装手臂控制的API
    """

    def __init__(self, robot_sdk: RobotSDK):
        self.robot_sdk = robot_sdk
        self._pool = ThreadPoolExecutor(max_workers=2)

    def _move_eef_traj_kmpc(self,
                            left_traj: List[List[float]],  # 末端6d位姿的轨迹，带时间戳
                            right_traj: List[List[float]],  # 末端6d位姿的轨迹，带时间戳
                            total_time: float,  # 轨迹总时间，单位秒
                            control_base: bool,  # 是否连带base一起控制
                            direct_to_wbc: bool,  # 指令是否经过全身MPC的优化再到WBC
                            frame: str,
                            arm_pos_threshold: float = 0.1,  # 位置误差阈值（米）
                            arm_angle_threshold: float = 0.174,  # 角度误差阈值（弧度）
                            arm_error_detect: bool = True  # 是否开启手臂误差检测
                            ):

        # 切成外部控制模式
        self.robot_sdk.control.set_external_control_arm_mode()
        if control_base:
            self.robot_sdk.control.set_manipulation_mpc_mode(KuavoManipulationMpcCtrlMode.BaseArm)
        else:
            self.robot_sdk.control.set_manipulation_mpc_mode(KuavoManipulationMpcCtrlMode.ArmOnly)
        if direct_to_wbc:
            self.robot_sdk.control.set_manipulation_mpc_control_flow(KuavoManipulationMpcControlFlow.DirectToWbc)
        time.sleep(0.15)
        num_points = min(len(left_traj), len(right_traj))
        time_per_point = total_time / (num_points - 1) if num_points > 1 else total_time
        for i in range(num_points):
            self.robot_sdk.control.control_robot_end_effector_pose(
                left_pose=left_traj[i],
                right_pose=right_traj[i],
                frame=frame,
            )
            if i < num_points - 1:  # 最后一个点不需要延时
                time.sleep(time_per_point)

        # 获取轨迹的最后一个点作为期望位置
        left_target_point = left_traj[-1]  # KuavoPose 对象，有 position 和 orientation 属性
        right_target_point = right_traj[-1]
        # 误差检测逻辑（可选）
        success = True
        if arm_error_detect:
            timeout = 2.0  # 超时时间（秒）
            start_time = time.time()
            success = False
            while (time.time() - start_time) < timeout:
                # 获取当前末端位置（世界坐标系）
                left_current_pose, right_current_pose = self.get_eef_pose_world()
                # 构造目标位姿
                # 注意：如果 frame 是 LocalFrame，需要将目标转换到世界坐标系进行比较
                if frame == KuavoManipulationMpcFrame.LocalFrame:
                    # 获取当前 base 到 odom 的变换
                    transform_base_to_world = self.get_current_transform(
                        source_frame=Frame.BASE,
                        target_frame=Frame.ODOM
                    )
                    # 将目标从 LocalFrame 转换到世界坐标系
                    left_target_pose_local = Pose(pos=left_target_point.position, quat=left_target_point.orientation, frame=Frame.BASE)
                    right_target_pose_local = Pose(pos=right_target_point.position, quat=right_target_point.orientation, frame=Frame.BASE)
                    # LocalFrame: x,y 使用 BASE 坐标系，z 使用绝对高度
                    left_target_pose_world = transform_base_to_world.apply_to_pose(left_target_pose_local)
                    left_target_pose_world.pos = np.array([
                        left_target_pose_world.pos[0], 
                        left_target_pose_world.pos[1], 
                        left_target_point.position[2]  # 使用原始的 z 值（绝对高度）
                    ])
                    left_target_pose_world.frame = Frame.ODOM
                    right_target_pose_world = transform_base_to_world.apply_to_pose(right_target_pose_local)
                    right_target_pose_world.pos = np.array([
                        right_target_pose_world.pos[0], 
                        right_target_pose_world.pos[1], 
                        right_target_point.position[2]
                    ])
                    right_target_pose_world.frame = Frame.ODOM
                else:
                    # WorldFrame：直接使用目标点
                    left_target_pose_world = Pose(pos=left_target_point.position, quat=left_target_point.orientation, frame=Frame.ODOM)
                    right_target_pose_world = Pose(pos=right_target_point.position, quat=right_target_point.orientation, frame=Frame.ODOM)
                
                # 计算位置误差
                left_pos_error = left_current_pose.position_l2_norm(left_target_pose_world)
                right_pos_error = right_current_pose.position_l2_norm(right_target_pose_world)
                
                # 计算角度误差
                left_angle_error = left_current_pose.angle(left_target_pose_world)
                right_angle_error = right_current_pose.angle(right_target_pose_world)

                print(f"手臂动作执行中，位置误差={left_pos_error:.4f}m (阈值={arm_pos_threshold}m), 角度误差={left_angle_error:.4f}rad (阈值={arm_angle_threshold}rad)")
                # 检查是否满足阈值
                if (left_pos_error <= arm_pos_threshold and right_pos_error <= arm_pos_threshold and
                    left_angle_error <= arm_angle_threshold and right_angle_error <= arm_angle_threshold):
                    success = True
                    break
                time.sleep(0.1)  # 误差检测间隔0.1s一次
            
            # 如果超时，打印错误信息
            if not success:
                print(f"[NodeArm] 手臂轨迹执行超时（{timeout}秒）！")
                print(f"  左臂: 位置误差={left_pos_error:.4f}m (阈值={arm_pos_threshold}m), "
                      f"角度误差={left_angle_error:.4f}rad (阈值={arm_angle_threshold}rad)")
                print(f"  右臂: 位置误差={right_pos_error:.4f}m (阈值={arm_pos_threshold}m), "
                      f"角度误差={right_angle_error:.4f}rad (阈值={arm_angle_threshold}rad)")

        # 运动结束后，切回默认模式
        self.robot_sdk.control.set_manipulation_mpc_mode(KuavoManipulationMpcCtrlMode.NoControl)
        self.robot_sdk.control.set_manipulation_mpc_control_flow(KuavoManipulationMpcControlFlow.ThroughFullBodyMpc)

    def _move_eef_traj_wheel_mpc(self,
                            left_traj: List[List[float]],  # 末端6d位姿的轨迹，不带时间戳
                            right_traj: List[List[float]],  # 末端6d位姿的轨迹，不带时间戳
                            total_time: float,  # 轨迹总时间，单位秒
                            direct_to_wbc: bool,  # 指令是否经过全身MPC的优化再到WBC
                            back_default: bool,  # 是否返回默认模式
                            frame: str,  # 坐标系
                            mpc_ctrl_mode: KuavoManipulationMpcCtrlMode = KuavoManipulationMpcCtrlMode.ArmOnly  # MPC控制模式
                            ):

        # 切成外部控制模式
        self.robot_sdk.control.set_external_control_arm_mode()
        self.robot_sdk.control.set_manipulation_mpc_mode(mpc_ctrl_mode)
        
        if direct_to_wbc:
            self.robot_sdk.control.set_manipulation_mpc_control_flow(KuavoManipulationMpcControlFlow.DirectToWbc)

        # 将轨迹转换为数值列表格式（兼容KuavoPose对象和List[float]）
        left_traj_list = [pose_to_list(p) for p in left_traj]
        right_traj_list = [pose_to_list(p) for p in right_traj]

        # 将左右轨迹合并为一个轨迹（每个点包含左右位姿的数值列表）
        num_points = min(len(left_traj_list), len(right_traj_list))
        combined_traj = [[left_traj_list[i], right_traj_list[i]] for i in range(num_points)]
        
        # 定义下发函数：将数值列表转回KuavoPose对象
        def publish_eef(pos):
            left_list, right_list = pos
            left_pose = KuavoPose(
                position=tuple(left_list[:3]),
                orientation=tuple(left_list[3:7])
            )
            right_pose = KuavoPose(
                position=tuple(right_list[:3]),
                orientation=tuple(right_list[3:7])
            )
            self.robot_sdk.control.control_robot_end_effector_pose(
                left_pose=left_pose,
                right_pose=right_pose,
                frame=frame,
            )
        
        # 使用通用函数进行重采样和下发
        resample_and_execute_traj(combined_traj, total_time, publish_eef)

        # 运动结束后，切回默认模式
        if back_default:
            self.robot_sdk.control.set_manipulation_mpc_mode(KuavoManipulationMpcCtrlMode.NoControl)
            self.robot_sdk.control.set_manipulation_mpc_control_flow(KuavoManipulationMpcControlFlow.ThroughFullBodyMpc)

    def _move_joint_traj(self,
                        joint_traj: List[List[float]],  # 关节角度轨迹，每个元素是14维关节角度列表
                        total_time: float,  # 轨迹总时间，单位秒
                        mpc_ctrl_mode: KuavoManipulationMpcCtrlMode = KuavoManipulationMpcCtrlMode.ArmOnly  # MPC控制模式
                        ):
        """
        执行关节轨迹（私有方法，内部调用）
        下发频率固定为 100Hz，根据 total_time 对轨迹进行重采样

        参数：
            joint_traj (List[List[float]]): 关节角度轨迹，每个元素是14维关节角度列表
            total_time (float): 轨迹总时间，单位秒
            mpc_ctrl_mode (KuavoManipulationMpcCtrlMode): MPC控制模式，默认为ArmOnly
        """
        
        # 切换到外部控制模式
        self.robot_sdk.control.set_external_control_arm_mode()
        self.robot_sdk.control.set_manipulation_mpc_mode(mpc_ctrl_mode)

        # 定义下发函数
        def publish_joint(pos):
            self.robot_sdk.control.control_arm_joint_positions(joint_positions=pos)
        
        # 使用通用函数进行重采样和下发
        resample_and_execute_traj(joint_traj, total_time, publish_joint)

    def move_joint_traj(self,
                       joint_traj: List[List[float]],  # 关节角度轨迹，每个元素是14维关节角度列表
                       asynchronous: bool = False,  # 布尔值，指定运动命令是否为异步。默认值为 false，表示函数会阻塞
                       total_time: float = 5.0,  # 轨迹总时间，单位秒
                       mpc_ctrl_mode: KuavoManipulationMpcCtrlMode = KuavoManipulationMpcCtrlMode.ArmOnly  # MPC控制模式
                       ):
        """
        执行关节轨迹控制接口

        参数：
            joint_traj (List[List[float]]): 关节角度轨迹，每个元素是14维关节角度列表
            asynchronous (bool): 布尔值，指定运动命令是否为异步。默认值为 false，表示函数会阻塞
            total_time (float): 轨迹总时间，单位秒
            mpc_ctrl_mode (KuavoManipulationMpcCtrlMode): MPC控制模式，默认为ArmOnly
                - ArmOnly: 仅控制手臂
                - BaseOnly: 仅控制底盘
                - BaseArm: 同时控制底盘和手臂
                - NoControl: 无控制

        返回：
            Future: 如果 asynchronous=True，返回 Future 对象；否则返回 None
        """
        if asynchronous:
            # 多线程
            fut = self._pool.submit(self._move_joint_traj, joint_traj, total_time, mpc_ctrl_mode)
            return fut  # 外部拿到 Future

        else:
            # 本函数阻塞
            self._move_joint_traj(joint_traj, total_time, mpc_ctrl_mode)
            return None

    def move_eef_traj_kmpc(
            self,
            left_traj: List[List[float]],  # 末端6d位姿的轨迹，不带时间戳
            right_traj: List[List[float]],  # 末端6d位姿的轨迹，不带时间戳
            asynchronous: bool = False,  # 布尔值，指定运动命令是否为异步。默认值为 false，表示函数会阻塞
            control_base: bool = False,  # 是否连带base一起控制
            direct_to_wbc: bool = True,  # 指令是否经过全身MPC的优化再到WBC
            total_time: float = 2.0,  # 轨迹总时间，单位秒
            frame: str = KuavoManipulationMpcFrame.WorldFrame,
            # 指令位置所在的坐标系： 'base_link'： 在机器人base_link坐标系下； 'foot_print': 'base_link' 在地面的投影; 'world': 世界系
            arm_pos_threshold: float = 0.15,  # 位置误差阈值（米）
            arm_angle_threshold: float = np.deg2rad(15),  # 角度误差阈值（弧度）
            arm_error_detect: bool = True  # 是否开启手臂误差检测
    ):
        if asynchronous:
            # 多线程
            fut = self._pool.submit(self._move_eef_traj_kmpc,
                                    left_traj, right_traj, total_time, control_base, direct_to_wbc, frame, arm_pos_threshold, arm_angle_threshold, arm_error_detect)
            return fut  # 外部拿到 Future

        else:
            # 本函数阻塞
            self._move_eef_traj_kmpc(
                left_traj, right_traj, total_time, control_base, direct_to_wbc, frame, arm_pos_threshold, arm_angle_threshold, arm_error_detect
            )

            return None

    def move_eef_traj_wheel_mpc(
            self,
            left_traj: List[List[float]],  # 末端6d位姿的轨迹，不带时间戳
            right_traj: List[List[float]],  # 末端6d位姿的轨迹，不带时间戳
            asynchronous: bool = False,  # 布尔值，指定运动命令是否为异步。默认值为 false，表示函数会阻塞
            direct_to_wbc: bool = True,  # 指令是否经过全身MPC的优化再到WBC
            total_time: float = 5.0,  # 轨迹总时间，单位秒
            back_default: bool = True,  # 是否返回默认模式
            frame: str = KuavoManipulationMpcFrame.WorldFrame,
            # 指令位置所在的坐标系： 'base_link'： 在机器人base_link坐标系下； 'foot_print': 'base_link' 在地面的投影; 'world': 世界系
            mpc_ctrl_mode: KuavoManipulationMpcCtrlMode = KuavoManipulationMpcCtrlMode.ArmOnly  # MPC控制模式
    ):
        if asynchronous:
            # 多线程
            fut = self._pool.submit(self._move_eef_traj_wheel_mpc,
                                    left_traj, right_traj, total_time, direct_to_wbc, back_default, frame, mpc_ctrl_mode)
            return fut  # 外部拿到 Future

        else:
            # 本函数阻塞
            self._move_eef_traj_wheel_mpc(
                left_traj, right_traj, total_time, direct_to_wbc, back_default, frame, mpc_ctrl_mode
            )

            return None

    def get_eef_pose_world(self):
        target_frame = Frame.ODOM

        left_pose = self.robot_sdk.tools.get_link_pose(
            link_name="zarm_l7_end_effector",
            reference_frame=target_frame
        )
        right_pose: KuavoPose = self.robot_sdk.tools.get_link_pose(
            link_name="zarm_r7_end_effector",
            reference_frame=target_frame
        )

        current_left_pose = Pose(
            pos=left_pose.position,
            quat=left_pose.orientation,
            frame=target_frame
        )
        current_right_pose = Pose(
            pos=right_pose.position,
            quat=right_pose.orientation,
            frame=target_frame
        )

        return current_left_pose, current_right_pose

    def get_eef_pose_world(self):
        target_frame = Frame.ODOM

        left_pose = self.robot_sdk.tools.get_link_pose(
            link_name="zarm_l7_end_effector",
            reference_frame=target_frame
        )
        right_pose: KuavoPose = self.robot_sdk.tools.get_link_pose(
            link_name="zarm_r7_end_effector",
            reference_frame=target_frame
        )

        current_left_pose = Pose(
            pos=left_pose.position,
            quat=left_pose.orientation,
            frame=target_frame
        )
        current_right_pose = Pose(
            pos=right_pose.position,
            quat=right_pose.orientation,
            frame=target_frame
        )

        return current_left_pose, current_right_pose

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

    def send_timed_arm_cmd(self, left_pose: Pose, right_pose: Pose, desire_time: float) -> Tuple[bool, float]:
        """
        通过服务发送单次手臂位姿命令（左右单臂各发一次）
        
        根据 Pose 的 frame 自动选择规划器：
            - ODOM/世界系 → 左臂 planner 4、右臂 planner 5
            - BASE/局部系 → 左臂 planner 6、右臂 planner 7
        
        参数：
            left_pose (Pose): 左臂目标位姿
            right_pose (Pose): 右臂目标位姿
            desire_time (float): 期望执行时间（秒）
            
        返回：
            Tuple[bool, float]: (是否成功, 实际执行时间)
        """
        frame = left_pose.frame
        if frame in [WheelArmFrame.ODOM, Frame.ODOM, "odom"]:
            left_planner, right_planner = 4, 5  # 左/右臂笛卡尔世界系
        else:
            left_planner, right_planner = 6, 7  # 左/右臂笛卡尔局部系
        
        # Pose 转 [x, y, z, yaw, pitch, roll]，get_euler 返回 [roll, pitch, yaw]
        left_euler = left_pose.get_euler(degrees=False)
        right_euler = right_pose.get_euler(degrees=False)
        left_vec = [*left_pose.pos, left_euler[2], left_euler[1], left_euler[0]]
        right_vec = [*right_pose.pos, right_euler[2], right_euler[1], right_euler[0]]
        
        if desire_time <= 0:
            rospy.logwarn(f"期望执行时间应大于0，当前值: {desire_time:.3f}")
        
        try:
            rospy.wait_for_service('/mobile_manipulator_timed_single_cmd', timeout=5.0)
            client = rospy.ServiceProxy('/mobile_manipulator_timed_single_cmd', lbTimedPosCmd)
            req_left = lbTimedPosCmdRequest()
            req_left.planner_index = left_planner
            req_left.desireTime = desire_time
            req_left.cmdVec = left_vec
            req_right = lbTimedPosCmdRequest()
            req_right.planner_index = right_planner
            req_right.desireTime = desire_time
            req_right.cmdVec = right_vec
            resp_left = client(req_left)
            # 调用服务
            resp_right = client(req_right)
            ok_left = resp_left.isSuccess
            ok_right = resp_right.isSuccess
            actual_time = max(resp_left.actualTime, resp_right.actualTime)
            if ok_left and ok_right:
                rospy.loginfo(f"手臂指令执行成功，实际时间: {actual_time}s")
            else:
                rospy.logerr(f"手臂指令执行失败（左: {ok_left}, 右: {ok_right}）")
            return ok_left and ok_right, actual_time
        except rospy.ROSException as e:
            rospy.logerr(f"服务等待超时: {e}")
            return False, 0.0
        except rospy.ServiceException as e:
            rospy.logerr(f"服务调用失败: {e}")
            return False, 0.0
        except Exception as e:
            rospy.logerr(f"未知错误: {e}")
            return False, 0.0


class TimedCmdAPI:
    """
    通用轮臂运动控制API
    
    通过 /mobile_manipulator_timed_single_cmd 服务控制轮臂各部分运动，
    支持底盘、下肢、上肢等多种命令类型。
    """
    
    # 命令类型配置：planner_index 和维度映射（服务侧已改为左右单臂：4/5 世界系、6/7 局部系、8/9 关节）
    CMD_CONFIG = {
        'chassis_world': {'planner_index': 0, 'dim': 3, 'name': '底盘世界系'},
        'chassis_local': {'planner_index': 1, 'dim': 3, 'name': '底盘局部系'},
        'torso': {'planner_index': 2, 'dim': 4, 'name': '躯干'},
        'leg': {'planner_index': 3, 'dim': 4, 'name': '下肢'},
        # 双臂逻辑类型（12/14 维）：在 send_timed_cmd 内拆成左右两次调用
        'arm_ee_world': {'dim': 12, 'name': '双臂末端世界系', 'split_left_planner': 4, 'split_right_planner': 5},
        'arm_ee_local': {'dim': 12, 'name': '双臂末端局部系', 'split_left_planner': 6, 'split_right_planner': 7},
        'arm': {'dim': 14, 'name': '上肢', 'split_left_planner': 8, 'split_right_planner': 9},
        # 单臂类型（供单臂节点或 Ruckig 用例）
        'left_arm_ee_world': {'planner_index': 4, 'dim': 6, 'name': '左臂末端世界系'},
        'right_arm_ee_world': {'planner_index': 5, 'dim': 6, 'name': '右臂末端世界系'},
        'left_arm_ee_local': {'planner_index': 6, 'dim': 6, 'name': '左臂末端局部系'},
        'right_arm_ee_local': {'planner_index': 7, 'dim': 6, 'name': '右臂末端局部系'},
        'left_arm': {'planner_index': 8, 'dim': 7, 'name': '左臂关节'},
        'right_arm': {'planner_index': 9, 'dim': 7, 'name': '右臂关节'},
    }
    
    # 向后兼容：旧的配置名
    JOINT_CONFIG = CMD_CONFIG

    def _call_timed_single_cmd(self, planner_index: int, cmd_vec: List[float],
                               desire_time: float) -> Tuple[bool, float]:
        """内部方法：单次调用 /mobile_manipulator_timed_single_cmd。返回 (是否成功, 实际执行时间)。"""
        try:
            rospy.wait_for_service('/mobile_manipulator_timed_single_cmd', timeout=5.0)
            client = rospy.ServiceProxy('/mobile_manipulator_timed_single_cmd', lbTimedPosCmd)
            req = lbTimedPosCmdRequest()
            req.planner_index = planner_index
            req.desireTime = desire_time
            req.cmdVec = list(cmd_vec)
            resp = client(req)
            if resp.isSuccess:
                return True, resp.actualTime
            rospy.logerr(f"单次指令失败 (planner={planner_index}): {resp.message}")
            return False, resp.actualTime
        except rospy.ROSException as e:
            rospy.logerr(f"服务等待超时: {e}")
            return False, 0.0
        except rospy.ServiceException as e:
            rospy.logerr(f"服务调用失败: {e}")
            return False, 0.0
        except Exception as e:
            rospy.logerr(f"未知错误: {e}")
            return False, 0.0

    def send_timed_cmd(self, cmd_type: str, cmd_vec: List[float], 
                       desire_time: float) -> Tuple[bool, float]:
        """
        通过服务发送定时命令（通用方法）
        
        参数：
            cmd_type: 命令类型
                - 'chassis_world' : 底盘世界系,[x,y,yaw],索引0
                - 'chassis_local' : 底盘局部系,[x,y,yaw],索引1
                - 'torso' : 躯干,[x,z,yaw,pitch],索引2
                - 'leg' : 下肢,[joint1,joint2,joint3,joint4],索引3
                - 'arm_ee_world_left' : 左臂末端世界系,[x,y,z,yaw,pitch,roll],索引4
                - 'arm_ee_world_right' : 右臂末端世界系,[x,y,z,yaw,pitch,roll],索引5
                - 'arm_ee_local_left' : 左臂末端局部系,[x,y,z,yaw,pitch,roll],索引6
                - 'arm_ee_local_right' : 右臂末端局部系,[x,y,z,yaw,pitch,roll],索引7
                - 'arm_joint_left' : 左臂关节，[joint1,joint2,joint3,joint4,joint5,joint6,joint7],索引8
                - 'arm_joint_right' : 右臂关节，[joint1,joint2,joint3,joint4,joint5,joint6,joint7],索引9

            cmd_vec: 命令向量
            desire_time: 期望执行时间（秒）
            
        返回：
            Tuple[bool, float]: (是否成功, 实际执行时间)
        """
        # 获取配置
        if cmd_type not in self.CMD_CONFIG:
            supported = ', '.join(self.CMD_CONFIG.keys())
            rospy.logerr(f"无效的命令类型: {cmd_type}，支持: {supported}")
            return False, 0.0
        
        config = self.CMD_CONFIG[cmd_type]
        expected_dim = config['dim']
        cmd_name = config['name']
        
        # 验证命令向量维度
        if len(cmd_vec) != expected_dim:
            rospy.logerr(f"{cmd_name}命令维度错误，需要{expected_dim}个，实际{len(cmd_vec)}个")
            return False, 0.0
        
        # 验证期望时间
        if desire_time <= 0:
            rospy.logwarn(f"期望执行时间应大于0，当前值: {desire_time:.3f}")
        
        # 双臂拆分：左+右各发一次
        if 'split_left_planner' in config:
            left_planner = config['split_left_planner']
            right_planner = config['split_right_planner']
            half = expected_dim // 2
            left_vec = cmd_vec[:half]
            right_vec = cmd_vec[half:]
            ok_left, actual_left = self._call_timed_single_cmd(left_planner, left_vec, desire_time)
            ok_right, actual_right = self._call_timed_single_cmd(right_planner, right_vec, desire_time)
            if ok_left:
                rospy.loginfo(f"{cmd_name}左臂指令执行成功，实际时间: {actual_left}s")
            if ok_right:
                rospy.loginfo(f"{cmd_name}右臂指令执行成功，实际时间: {actual_right}s")
            success = ok_left and ok_right
            actual_time = max(actual_left, actual_right)
            if success:
                rospy.loginfo(f"{cmd_name}指令执行成功，实际时间: {actual_time}s")
            else:
                rospy.logerr(f"{cmd_name}指令执行失败（左: {ok_left}, 右: {ok_right}）")
            return success, actual_time
        
        # 单规划器
        planner_index = config['planner_index']
        success, actual_time = self._call_timed_single_cmd(planner_index, cmd_vec, desire_time)
        if success:
            rospy.loginfo(f"{cmd_name}指令执行成功，实际时间: {actual_time}s")
        else:
            rospy.logerr(f"{cmd_name}指令执行失败")
        return success, actual_time

    def send_timed_joint_cmd(self, joint_type: str, joint_angles: List[float], 
                              desire_time: float) -> Tuple[bool, float]:
        """关节命令"""
        return self.send_timed_cmd(joint_type, joint_angles, desire_time)
    
    def send_timed_leg_cmd(self, joint_angles: List[float], desire_time: float) -> Tuple[bool, float]:
        """下肢关节命令"""
        return self.send_timed_cmd('leg', joint_angles, desire_time)
    
    def send_timed_arm_joint_cmd(self, joint_angles: List[float], desire_time: float) -> Tuple[bool, float]:
        """上肢关节命令"""
        return self.send_timed_cmd('arm', joint_angles, desire_time)
    
    # ========== 底盘命令便捷方法 ==========
    def send_timed_chassis_world_cmd(self, pose: List[float], desire_time: float) -> Tuple[bool, float]:
        """底盘世界系位姿命令 (x, y, yaw)"""
        return self.send_timed_cmd('chassis_world', pose, desire_time)
    
    def send_timed_chassis_local_cmd(self, pose: List[float], desire_time: float) -> Tuple[bool, float]:
        """底盘局部系位姿命令 (x, y, yaw)"""
        return self.send_timed_cmd('chassis_local', pose, desire_time)
    
    # ========== 规划器参数设置 ==========
    def set_ruckig_params(self, 
                          planner_index: int,
                          is_sync: bool,
                          velocity_max: List[float],
                          acceleration_max: List[float],
                          jerk_max: List[float],
                          velocity_min: List[float] = None,
                          acceleration_min: List[float] = None) -> bool:
        """
        设置Ruckig规划器参数
        
        参数：
            planner_index: 规划器索引
                - 0: 底盘世界系  
                - 1: 底盘局部系
                - 2: 躯干  
                - 3: 下肢
                - 4: 左臂末端世界系
                - 5: 右臂末端世界系
                - 6: 左臂末端局部系
                - 7: 右臂末端局部系
                - 8: 左臂关节
                - 9: 右臂关节
            is_sync: 是否同步模式
            velocity_max: 最大速度列表
            acceleration_max: 最大加速度列表
            jerk_max: 最大急动度列表
            velocity_min: 最小速度列表（可选）
            acceleration_min: 最小加速度列表（可选）
            
        返回：
            bool: 是否成功
        """
        try:
            rospy.wait_for_service('/mobile_manipulator_set_ruckig_planner_params', timeout=5.0)
            client = rospy.ServiceProxy('/mobile_manipulator_set_ruckig_planner_params', setRuckigPlannerParams)
            req = setRuckigPlannerParamsRequest()
            req.planner_index = planner_index
            req.is_sync = is_sync
            req.velocity_max = velocity_max
            req.acceleration_max = acceleration_max
            req.jerk_max = jerk_max
            if velocity_min is not None:
                req.velocity_min = velocity_min
            if acceleration_min is not None:
                req.acceleration_min = acceleration_min

            resp = client(req)
            if resp.result:
                rospy.loginfo(f"✅ 规划器参数设置成功 (planner_index={planner_index}): {resp.message}")
                return True
            else:
                rospy.logerr(f"❌ 规划器参数设置失败 (planner_index={planner_index}): {resp.message}")
                return False
        except rospy.ROSException as e:
            rospy.logerr(f"❌ 服务等待超时: {e}")
            return False
        except rospy.ServiceException as e:
            rospy.logerr(f"❌ 服务调用失败: {e}")
            return False
        except Exception as e:
            rospy.logerr(f"❌ 未知错误: {e}")
            return False


class TorsoAPI:
    """
    根据手臂和躯干控制接口，封装躯干控制的API
    """

    def __init__(self, robot_sdk: RobotSDK):
        self.robot_sdk = robot_sdk
        self._pool = ThreadPoolExecutor(max_workers=2)

        # self._goal_lock = threading.Lock()
        # self._goal_cv   = threading.Condition(self._goal_lock)
        # self._current_goal = None  # 当前目标位姿

        # 共享状态
        self._target_lock = threading.Lock()
        self._current_target: Pose = None

    def _move_wheel_lower_joint(self,
                            joint_traj: list,
                            total_time: float):
        """
        执行轮臂下关节轨迹控制
        
        参数：
            joint_traj: 关节轨迹，可以是单个目标位置 [pos1, pos2, pos3, pos4] 
                        或多点轨迹 [[pos1, pos2, pos3, pos4], ...]
            total_time: 执行时间（秒）
        """
        self.robot_sdk.control.set_manipulation_mpc_mode(KuavoManipulationMpcCtrlMode.ArmOnly)

        # 获取当前轮臂下关节位置（前4个关节），joint_state.position 是弧度，需要转为角度
        try:
            joint_state = self.robot_sdk.state.joint_state
            if joint_state is not None and joint_state.position is not None and len(joint_state.position) >= 4:
                # joint_state.position 是弧度，转换为角度（degrees）
                current_pos = [np.degrees(pos) for pos in joint_state.position[:4]]
            else:
                raise RuntimeError("获取当前轮臂下关节位置失败，joint_state 无效")
        except Exception as e:
            raise RuntimeError(f"获取当前轮臂下关节位置异常: {e}")

        # 判断是单个目标还是轨迹
        if len(joint_traj) > 0 and not isinstance(joint_traj[0], (list, tuple)):
            # 单个目标位置，生成从当前位置到目标位置的轨迹
            target_pos = joint_traj
            traj = [current_pos, target_pos]
        else:
            # 已经是多点轨迹，将当前位置作为起点
            traj = [current_pos] + joint_traj

        print(f"[TorsoAPI] 当前轮臂下关节位置: {current_pos}")
        print(f"[TorsoAPI] 目标轮臂下关节位置: {traj[-1]}")

        # 定义下发函数
        def publish_wheel_joint(pos):
            self.robot_sdk.control.control_wheel_lower_joint(pos)
        
        # 使用通用函数进行重采样和下发
        resample_and_execute_traj(traj, total_time, publish_wheel_joint)
        

    def move_wheel_lower_joint(self,
                            joint_traj: list,
                            asynchronous: bool = False,
                            total_time: float = 5.0):
        if asynchronous:
            return self._pool.submit(self._move_wheel_lower_joint, joint_traj, total_time)
        else:
            self._move_wheel_lower_joint(joint_traj, total_time)
            return None

    def _move_torso_pose(self,
                         desir_torso_pose: Pose,
                         total_time: float = 5.0):
        """
        执行躯干位姿控制，从当前位姿插值到目标位姿
        
        参数：
            desir_torso_pose: 目标躯干位姿
            total_time: 执行时间（秒）
        """
        if desir_torso_pose is None:
            raise ValueError("desir_torso_pose must not be None")

        self.robot_sdk.control.set_manipulation_mpc_mode(KuavoManipulationMpcCtrlMode.ArmOnly)

        # 获取目标位姿 [x, y, z, roll, pitch, yaw]
        target_x, target_y, target_z = desir_torso_pose.pos.tolist()
        target_roll, target_pitch, target_yaw = desir_torso_pose.get_euler(degrees=False).tolist()
        target_pose = [target_x, target_y, target_z, target_roll, target_pitch, target_yaw]

        # 获取当前躯干位姿（通过 tf 树获取 base_link 的位置，会被映射为 waist_yaw_link）
        try:
            current_tf_pose = self.robot_sdk.tools.get_link_pose(
                link_name="base_link",
                reference_frame="odom"
            )
            if current_tf_pose is not None:
                # 打印原始 tf 位姿
                print(f"[TorsoAPI] base_link(->waist_yaw_link) tf position: {current_tf_pose.position}")
                print(f"[TorsoAPI] base_link(->waist_yaw_link) tf orientation (quat): {current_tf_pose.orientation}")
                
                # 将四元数转换为欧拉角
                current_pose_obj = Pose(
                    pos=current_tf_pose.position,
                    quat=current_tf_pose.orientation,
                    frame=Frame.ODOM
                )
                current_euler = current_pose_obj.get_euler(degrees=False).tolist()
                print(f"[TorsoAPI] base_link euler (rad): {current_euler}")
                
                current_pose = [
                    current_tf_pose.position[0],
                    current_tf_pose.position[1],
                    current_tf_pose.position[2],
                    current_euler[0],
                    current_euler[1],
                    current_euler[2]
                ]
            else:
                # 如果获取失败，使用默认零位姿
                print("[Warning] 获取当前躯干位姿失败，使用默认零位姿")
                current_pose = [0.0, 0.0, 0.8, 0.0, 0.0, 0.0]
        except Exception as e:
            print(f"[Warning] 获取当前躯干位姿异常: {e}，使用默认零位姿")
            current_pose = [0.0, 0.0, 0.8, 0.0, 0.0, 0.0]

        print(f"[TorsoAPI] 当前位姿: {current_pose}")
        print(f"[TorsoAPI] 目标位姿: {target_pose}")

        # 生成从当前位姿到目标位姿的轨迹
        traj = [current_pose, target_pose]

        # 定义下发函数
        def publish_torso_pose(pose):
            x, y, z, roll, pitch, yaw = pose
            self.robot_sdk.control.control_torso_pose(x, y, z, roll, pitch, yaw)
        
        # 使用通用函数进行重采样和下发
        resample_and_execute_traj(traj, total_time, publish_torso_pose)

    def move_torso_pose(self,
                        desir_torso_pose: Pose,
                        asynchronous: bool = False,
                        total_time: float = 5.0):
        """
        执行躯干位姿控制，以 100Hz 频率持续下发目标位姿
        
        参数：
            desir_torso_pose: 目标躯干位姿
            asynchronous: 是否异步执行
            total_time: 执行时间（秒）
        """
        if asynchronous:
            return self._pool.submit(
                self._move_torso_pose,
                desir_torso_pose,
                total_time,
            )

        self._move_torso_pose(desir_torso_pose, total_time)
        return None

    def _check_success_walk(self,
                            target_in_odom,
                            yaw_threshold,
                            pos_threshold
                            ):
        target_yaw = target_in_odom.get_euler(degrees=False)[2]  # 获取目标偏航角

        # === check_yaw ===
        robot_pose = Pose(
            pos=self.robot_sdk.state.robot_position(),
            quat=self.robot_sdk.state.robot_orientation()
        )
        robot_yaw = robot_pose.get_euler(degrees=False)[2]  # 获取机器人的偏航角
        yaw_diff = normalize_angle(target_yaw - robot_yaw)
        yaw_reached = abs(yaw_diff) <= yaw_threshold

        robot_pos = self.robot_sdk.state.robot_position()
        pos_diff = np.linalg.norm(np.array(robot_pos[:2]) - np.array(target_in_odom.pos[:2]))
        pos_reached = pos_diff <= pos_threshold

        # print(
        # f'目标未到达: {target_in_odom.pos}, 偏航角未到达: {target_yaw:.2f} rad, diff: {yaw_diff:.2f} rad | {pos_diff}')

        if yaw_reached and pos_reached:
            print(
                f'目标位置已到达: {target_in_odom.pos}, 偏航角已到达: {target_yaw:.2f} rad, diff: {yaw_diff:.2f} rad | {pos_diff}')
            return True

        return False

    def _walk_to_pose_by_pose_world(self,
                                    pos_threshold=0.05,
                                    timeout=60):
        """
        躯干行走到某个点，通过世界位置控制
        """
        # 获取目标位姿
        target = None
        with self._target_lock:
            if self._current_target is not None:
                target = copy.deepcopy(self._current_target)
                self._current_target = None  # 取走目标
        
        if target is None:
            print("_walk_to_pose_by_pose_world: 没有目标位姿")
            return False
            
        # 发送位置命令（只发送一次）
        target_x = target.pos[0]
        target_y = target.pos[1]
        target_z = 0.0
        target_yaw = target.get_euler(degrees=False)[2]
        
        self.robot_sdk.control.control_command_pose_world(target_x, target_y, target_z, target_yaw)
        print(f"📤 cmd_pos_world 发送一次: 目标=[{target_x:.3f}, {target_y:.3f}, {target.get_euler(degrees=True)[2]:.1f}°]")
        
        # 等待到达目标（使用轮询方式）
        tic = time.time()
        while time.time() - tic < timeout:
            success = self._check_success_walk(
                target, 
                yaw_threshold=np.deg2rad(5), 
                pos_threshold=pos_threshold
            )
            if success:
                return True
            time.sleep(0.1)
        
        print(f"❌ cmd_pos_world 超时 {timeout}s，未到达目标")
        return False

    def _walk_to_pose_by_pose(self,
                              pos_threshold=0.05,
                              timeout=60):
        """
        躯干行走到某个点，通过相对位置控制
        """
        # 获取目标位姿
        target = None
        with self._target_lock:
            if self._current_target is not None:
                target = copy.deepcopy(self._current_target)
                self._current_target = None  # 取走目标
        
        if target is None:
            print("_walk_to_pose_by_pose: 没有目标位姿")
            return False
            
        robot_pose_when_start = Pose(
            pos=self.robot_sdk.state.robot_position(),
            quat=self.robot_sdk.state.robot_orientation(),
            frame=Frame.ODOM
        )
        
        # 如果目标是base_link坐标系，需要转换到世界坐标系
        if target.frame == Frame.BASE:
            transform_base_to_world = Transform3D(
                trans_pose=robot_pose_when_start,
                source_frame=Frame.BASE,
                target_frame=Frame.ODOM
            )
            target_in_world = transform_base_to_world.apply_to_pose(target)
        else:
            target_in_world = target
        
        # 发送相对位置命令
        self.robot_sdk.control.control_command_pose(
            target.pos[0], target.pos[1], target.pos[2],
            target.get_euler(degrees=False)[2]
        )
        print(f"📤 cmd_pos 发送相对位移: Δx={target.pos[0]:.3f}, Δy={target.pos[1]:.3f}, yaw={target.get_euler(degrees=True)[2]:.1f}°")
        
        # 等待到达目标
        tic = time.time()
        while time.time() - tic < timeout:
            success = self._check_success_walk(
                target_in_world, 
                yaw_threshold=np.deg2rad(5), 
                pos_threshold=pos_threshold
            )
            if success:
                return True
            time.sleep(0.1)
        
        print(f"❌ cmd_pos 超时 {timeout}s，未到达目标")
        return False


    def _walk_to_pose_by_vel(self,
                             # target: Pose,
                             pos_threshold=0.05,
                             kp_pos=0.5,
                             kp_yaw=0.5,
                             max_vel_x=0.4,
                             max_vel_yaw=0.6,
                             timeout=60,
                             backward_mode=False
                             ):
        """
        躯干行走到某个点，通过速度控制
        """
        robot_pose_when_start = Pose(
            pos=self.robot_sdk.state.robot_position(),
            quat=self.robot_sdk.state.robot_orientation(),
            frame=Frame.ODOM
        )

        # 2) 取目标（如果外部刚更新，这里能立刻看到）
        is_target_new = True

        tic = time.time()

        while time.time() - tic < timeout:
            # 0. 获取并处理target

            with self._target_lock:
                if self._current_target is not None:
                    target = copy.deepcopy(self._current_target)
                    is_target_new = True  # 目标更新了

            with self._target_lock:
                self._current_target = None  # 取走目标

            # assert target is not None, "目标位姿不能为空"

            if is_target_new:
                if target.frame not in [Frame.ODOM, Frame.BASE]:
                    print("使用'cmd_vel'速度控制模式时，目标位姿的坐标系必须是'odom' 或'base_link'")
                    return False

                if Frame.BASE == target.frame:
                    transform_init_to_world = Transform3D(
                        trans_pose=robot_pose_when_start,
                        source_frame=Frame.BASE,  # 源坐标系为base_link
                        target_frame=Frame.ODOM  # 目标坐标系为odom
                    )
                    target_in_odom = transform_init_to_world.apply_to_pose(target)
                else:
                    target_in_odom = target
                is_target_new = False  # 只处理一次

            # 1. 获取当前世界系下位姿
            robot_pose = Pose(
                pos=self.robot_sdk.state.robot_position(),
                quat=self.robot_sdk.state.robot_orientation(),
                frame=Frame.ODOM
            )
            # 2. 目标位姿，默认只能是世界系
            assert Frame.ODOM == target_in_odom.frame, "目标位姿必须是世界坐标系（odom）"
            # 算目标朝向
            # angle_diff = robot_pose.angle_yaw(self.target)
            # 目标朝向是机器人与目标位置连线的朝向
            # compute target in frame of base

            euler = robot_pose.get_euler(degrees=False)
            euler[0] = 0.0
            euler[1] = 0.0
            robot_pose_2d = Pose.from_euler(
                pos=robot_pose.pos,  # 只取x, y坐标
                euler=euler,  # 只取yaw朝向
                frame=Frame.ODOM,  
                degrees=False
            )

            euler = target_in_odom.get_euler(degrees=False)
            euler[0] = 0.0
            euler[1] = 0.0

            target_in_odom_2d = Pose.from_euler(
                pos=target_in_odom.pos,  # 只取x, y坐标
                euler=euler,  # 只取yaw朝向
                frame=Frame.ODOM, 
                degrees=False
            )

            transform_basa_to_odom = Transform3D(
                trans_pose=robot_pose_2d,
                source_frame=Frame.BASE,  # 源坐标系为base_link
                target_frame=Frame.ODOM  # 目标坐标系为odom
            )
            target_in_base = transform_basa_to_odom.apply_to_pose_inverse(target_in_odom_2d)
            # print(f"目标在base_link坐标系下的位置：{target_in_base}")
            # print(f"base in odom pose: {robot_pose_2d}")
            # print(f"target in odom pose: {target_in_odom_2d}")

            angle_diff_line = np.arctan2(
                target_in_base.pos[1],
                target_in_base.pos[0]
            )
            angle_diff_line = normalize_angle(angle_diff_line)  # 归一化角度
            angle_diff_frame = target_in_base.get_euler(degrees=False)[2]  ## 两个坐标系间的角度差
            dis_diff = np.linalg.norm(target_in_base.pos[:2])

            # print(
            #     f"当前坐标系间角度差：{np.rad2deg(angle_diff_frame):.2f}°，距离差：{dis_diff:.2f}米， 连线角度差：{np.rad2deg(angle_diff_line):.2f}°")
            # 如果朝向大于某个值，先转不走
            max_yaw_to_walk = np.deg2rad(10)  # 超过这个值就不走只转
            max_dis_to_rotate = pos_threshold  # 小于这个距离就转到angle_diff_frame

            # 倒退模式：跳过转向，直接根据base坐标系下的位置给速度
            if backward_mode:
                x_diff = target_in_base.pos[0]
                y_diff = target_in_base.pos[1]
                vel_x = kp_pos * x_diff
                vel_x = np.clip(vel_x, -max_vel_x, max_vel_x)
                vel_y = kp_pos * y_diff
                vel_y = np.clip(vel_y, -max_vel_x, max_vel_x)
                self.robot_sdk.control.walk(
                    linear_x=vel_x,
                    linear_y=vel_y,
                    angular_z=0.0
                )
                print("vel_x:",vel_x)
                print("vel_y:",vel_y)
                # 检查是否到达
                if dis_diff < pos_threshold:
                    self.stop_walk()
                    break

            # 1. if dis too small， then use holonomic fine tune
            else:
                if dis_diff < max_dis_to_rotate:
                    vel_yaw = kp_yaw * angle_diff_frame
                    vel_yaw = np.clip(vel_yaw, -max_vel_yaw, max_vel_yaw)  # 限制转速
                    # print(f"转向target frame朝向，转动速度：{vel_yaw:.2f} rad/s")
                    self.robot_sdk.control.walk(
                        linear_x=0.0,  # 不前进
                        linear_y=0.0,  # 不侧移
                        angular_z=vel_yaw  # 只转动
                    )

                elif dis_diff < (max_dis_to_rotate + 0.1) or (
                        abs(target_in_base.pos[1]) < 0.1 and abs(angle_diff_frame) < np.deg2rad(10)):
                    # 如果距离小于阈值，使用holonomic控制
                    x_diff = target_in_base.pos[0]
                    y_diff = target_in_base.pos[1]
                    vel_x = kp_pos * x_diff
                    vel_x = np.clip(vel_x, -max_vel_x, max_vel_x)
                    vel_y = kp_pos * y_diff
                    vel_y = np.clip(vel_y, -max_vel_x, max_vel_x)
                    # print(f'holonomic控制，前进速度：{vel_x:.2f} m/s, 侧移速度：{vel_y:.2f} m/s')
                    self.robot_sdk.control.walk(
                        linear_x=vel_x,  # 前进
                        linear_y=vel_y,  # 侧移
                        angular_z=0.0  # 不转动
                    )

                elif abs(angle_diff_line) > max_yaw_to_walk:
                    vel_yaw = kp_yaw * angle_diff_line
                    vel_yaw = np.clip(vel_yaw, -max_vel_yaw, max_vel_yaw)  # 限制转速
                    # print(f"dis_diff {dis_diff}; 转向连线方向，转动速度：{vel_yaw:.2f} rad/s")
                    self.robot_sdk.control.walk(
                        linear_x=0.0,  # 不前进
                        linear_y=0.0,  # 不侧移
                        angular_z=vel_yaw  # 只转动
                    )
                elif dis_diff >= max_dis_to_rotate:
                    # 如果连线朝向小于某个值，开始前进
                    # dis_sign = (abs(angle_diff_line) > np.pi)

                    vel_x = kp_pos * dis_diff
                    vel_x = np.clip(vel_x, -max_vel_x, max_vel_x)  # 限制前进速度

                    vel_yaw = kp_yaw * angle_diff_line
                    vel_yaw = np.clip(vel_yaw, -max_vel_yaw, max_vel_yaw)  # 限制转速
                    # print(f"dis_diff {dis_diff}, 前进速度：{vel_x:.2f} m/s, 转动速度：{vel_yaw:.2f} rad/s")
                    self.robot_sdk.control.walk(
                        linear_x=vel_x,  # 前进
                        linear_y=0.0,  # 不侧移
                        angular_z=vel_yaw  # 不转动
                    )

            time.sleep(0.01)    # 控制频率,不能太低，太低的話行走會出現行走站立來回切換的現象

            success = self._check_success_walk(
                target_in_odom, yaw_threshold=np.deg2rad(10), pos_threshold=pos_threshold)
            if success:
                print("success:stop_walk")
                self.stop_walk()
                break
            else:
                print("not_success")
        return None

    def walk_to_pose_by_vel(self,
                            # target: Pose,
                            pos_threshold=0.1,
                            kp_pos=0.5,
                            kp_yaw=0.5,
                            max_vel_x=0.5,
                            max_vel_yaw=0.4,
                            backward_mode=False,
                            asynchronous: bool = True):
        if asynchronous:
            # 多线程，异步

            fut = self._pool.submit(self._walk_to_pose_by_vel, pos_threshold, kp_pos, kp_yaw, max_vel_x, max_vel_yaw, backward_mode=backward_mode)

            return fut

        else:
            self._walk_to_pose_by_vel(pos_threshold, kp_pos, kp_yaw, max_vel_x, max_vel_yaw, backward_mode=backward_mode)

            return None

    def update_walk_goal(self, new_goal: Pose, backward_mode=False):
        """线程安全：更新当前目标并唤醒控制线程；返回新的版本号。"""
        print(f'接收到新的行走目标：{new_goal}')
        with self._target_lock:
            self._current_target = new_goal
            self._backward_mode = backward_mode

    def walk_to_pose(self,
                            # target: Pose,
                            pos_threshold=0.1,
                            kp_pos=0.5,
                            kp_yaw=0.5,
                            max_vel_x=0.5,
                            max_vel_yaw=0.4,
                            timeout=60,
                            walk_mode='cmd_vel',
                            asynchronous: bool = True):
        if asynchronous:
            # 多线程，异步
            if walk_mode == 'cmd_vel':
                fut = self._pool.submit(self._walk_to_pose_by_vel, pos_threshold, kp_pos, kp_yaw, max_vel_x, max_vel_yaw, timeout)
            elif walk_mode == 'cmd_pos_world':
                fut = self._pool.submit(self._walk_to_pose_by_pose_world, pos_threshold, timeout)
            elif walk_mode == 'cmd_pos':
                fut = self._pool.submit(self._walk_to_pose_by_pose, pos_threshold, timeout)
            else:
                raise ValueError(f"无效的行走模式: {walk_mode}")

            return fut

        else:
            if walk_mode == 'cmd_vel':
                self._walk_to_pose_by_vel(pos_threshold, kp_pos, kp_yaw, max_vel_x, max_vel_yaw, timeout)
            elif walk_mode == 'cmd_pos_world':
                self._walk_to_pose_by_pose_world(pos_threshold, timeout)
            elif walk_mode == 'cmd_pos':
                self._walk_to_pose_by_pose(pos_threshold, timeout)
            else:
                raise ValueError(f"无效的行走模式: {walk_mode}")

            return None

    def _walk_to_pose_by_world(self,
                             pos_threshold=0.05,
                             timeout=60
                             ):
        """
        躯干行走到某个点，通过world坐标控制
        """
        # 取目标（如果外部刚更新，这里能立刻看到）
        is_target_new = True

        tic = time.time()

        while time.time() - tic < timeout:
            # 0. 获取并处理target
            with self._target_lock:
                if self._current_target is not None:
                    target = copy.deepcopy(self._current_target)
                    is_target_new = True  # 目标更新了

            with self._target_lock:
                self._current_target = None  # 取走目标

            if is_target_new:
                if target.frame not in [Frame.ODOM, Frame.BASE]:
                    print("使用'cmd_pose_world'位置控制模式时，目标位姿的坐标系必须是'odom' 或'base_link'")
                    return False

                if Frame.BASE == target.frame:
                    # 获取当前机器人位置和姿态
                    robot_pos = self.robot_sdk.state.robot_position()
                    robot_quat = self.robot_sdk.state.robot_orientation()
                    robot_pose = Pose(pos=robot_pos, quat=robot_quat, frame=Frame.ODOM)

                    # 创建BASE到ODOM的变换
                    transform_base_to_odom = Transform3D(
                        trans_pose=robot_pose,
                        source_frame=Frame.BASE,
                        target_frame=Frame.ODOM
                    )

                    # 将BASE坐标系下的目标转换到ODOM坐标系
                    target_in_odom = transform_base_to_odom.apply_to_pose(target)
                else:
                    target_in_odom = target
                is_target_new = False  # 只处理一次

            # 发送位置控制指令
            self.robot_sdk.control.control_command_pose_world(
                target_pose_x=target_in_odom.pos[0],
                target_pose_y=target_in_odom.pos[1],
                target_pose_z=0.0,
                target_pose_yaw=target_in_odom.get_euler(degrees=False)[2]
            )

            # 检查是否到达目标位置
            robot_pos = self.robot_sdk.state.robot_position()
            robot_quat = self.robot_sdk.state.robot_orientation()
            robot_pose = Pose(pos=robot_pos, quat=robot_quat, frame=Frame.ODOM)

            # 计算位置和角度差异
            pos_diff = np.linalg.norm(np.array(robot_pos[:2]) - np.array(target_in_odom.pos[:2]))
            target_yaw = target_in_odom.get_euler(degrees=False)[2]
            robot_yaw = robot_pose.get_euler(degrees=False)[2]
            yaw_diff = abs(normalize_angle(target_yaw - robot_yaw))

            # 到达条件：位置误差小于设定阈值，角度误差小于0.1弧度
            if pos_diff < pos_threshold and yaw_diff < 0.1:
                print(f'目标位置已到达: {target_in_odom.pos}, 偏航角已到达: {target_yaw:.2f} rad, diff: {yaw_diff:.2f} rad | {pos_diff}')
                break

            time.sleep(0.05)  # 控制频率

        return None

    def walk_to_pose_by_world(self,
                            pos_threshold=0.1,
                            timeout=60,
                            asynchronous: bool = True):
        """
        躯干行走到某个点，通过world坐标控制
        """
        print("cmd_pose_world")
        if asynchronous:
            # 多线程，异步
            fut = self._pool.submit(self._walk_to_pose_by_world, pos_threshold, timeout)
            return fut
        else:
            self._walk_to_pose_by_world(pos_threshold, timeout)
            return None

    def stop_walk(self):
        for _ in range(10):
            self.robot_sdk.control.walk(0.0, 0.0, 0.0)
            print("stop_walk:停止行走")
            time.sleep(0.02)
        self.robot_sdk.control.stance()
