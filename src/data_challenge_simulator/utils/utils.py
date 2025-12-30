import math
import os
from typing import List, Tuple, Optional, Literal
import xml.etree.ElementTree as ET
import random, time
from kuavo_humanoid_sdk import KuavoPose, KuavoRobotState, KuavoRobot
from scipy.spatial.transform import Rotation as R
import numpy as np
from utils.object_pos import ObjectPose
from kuavo_humanoid_sdk.interfaces.data_types import KuavoIKParams
class Utils:
    def __init__(self):
        pass

    @staticmethod
    def is_in_target_region(position, region_bounds):
        """
        判断给定 position 是否在 region_bounds 所定义的包围盒内。

        参数:
            position: (x, y, z)
            region_bounds: [(x_min, x_max), (y_min, y_max), (z_min, z_max)]
        返回:
            True / False
        """
        if position is None:
            return False

        for i in range(3):
            if not (region_bounds[i][0] <= position[i] <= region_bounds[i][1]):
                return False
        return True

    @staticmethod
    def interpolate_joint_trajectory(q_target: List[float], q_start: List[float] = None, num: int = 50) -> List[List[float]]:
        """
        从起始关节位置 q_start 插值到目标关节位置 q_target，共生成 num 段。
        所有角度输入以“度”为单位，输出为弧度制的每帧关节角度列表。

        参数：
            q_target (List[float]): 目标关节角度（单位：度）
            q_start (List[float], optional): 起始关节角度（单位：度），默认是全 0
            num (int): 插值段数

        返回：
            List[List[float]]: 每一帧的关节角度（单位：弧度）
        """
        if q_start is None:
            q_start = [0.0] * len(q_target)

        q_list = []
        for i in range(num):
            q_tmp = [
                math.radians(q_start[j] + i / float(num) * (q_target[j] - q_start[j]))
                for j in range(len(q_target))
            ]
            q_list.append(q_tmp)
        return q_list

    @staticmethod
    def interpolate_joint_trajectory_polynomial(
            q_target: List[float], 
            q_start: List[float] = None, 
            num: int = 50,
            order: int = 3,
            start_velocity: Optional[List[float]] = None,
            end_velocity: Optional[List[float]] = None
        ) -> List[List[float]]:
        """
        使用多项式插值从起始关节位置 q_start 插值到目标关节位置 q_target，共生成 num 段。
        所有角度输入以“度”为单位，输出为弧度制的每帧关节角度列表。
        
        使用三次多项式（默认）可以实现平滑的速度和加速度曲线，比线性插值更自然。

        参数：
            q_target (List[float]): 目标关节角度（单位：度）
            q_start (List[float], optional): 起始关节角度（单位：度），默认是全 0
            num (int): 插值段数
            order (int): 多项式阶数，默认3（三次多项式）
                         - 1: 线性插值（等同于 interpolate_joint_trajectory）
                         - 3: 三次多项式（平滑的速度和加速度）
                         - 5: 五次多项式（更平滑，但计算更复杂）
            start_velocity (List[float], optional): 起始速度（单位：度/段），默认0
            end_velocity (List[float], optional): 终点速度（单位：度/段），默认0

        返回：
            List[List[float]]: 每一帧的关节角度（单位：弧度）

        示例：
            >>> # 三次多项式插值（默认）
            >>> q_list = Utils.interpolate_joint_trajectory_polynomial(
            ...     q_target=[90, 0, 0, -120, 90, 0, 0],
            ...     q_start=[0, 0, 0, 0, 0, 0, 0],
            ...     num=50
            ... )
            
            >>> # 指定起点和终点速度
            >>> q_list = Utils.interpolate_joint_trajectory_polynomial(
            ...     q_target=[90, 0, 0, -120, 90, 0, 0],
            ...     q_start=[0, 0, 0, 0, 0, 0, 0],
            ...     num=50,
            ...     start_velocity=[0, 0, 0, 0, 0, 0, 0],
            ...     end_velocity=[0, 0, 0, 0, 0, 0, 0]
            ... )
        """
        if q_start is None:
            q_start = [0.0] * len(q_target)
        
        if start_velocity is None:
            start_velocity = [0.0] * len(q_target)
        if end_velocity is None:
            end_velocity = [0.0] * len(q_target)
        
        # 确保所有列表长度一致
        if len(q_start) != len(q_target) or len(start_velocity) != len(q_target) or len(end_velocity) != len(q_target):
            raise ValueError("q_start, q_target, start_velocity, end_velocity 的长度必须相同")
        
        q_list = []
        
        # 归一化时间参数 t: [0, 1]
        for i in range(num):
            t = i / float(num - 1) if num > 1 else 0.0  # t 从 0 到 1
            
            q_tmp = []
            for j in range(len(q_target)):
                q0 = q_start[j]
                q1 = q_target[j]
                v0 = start_velocity[j]
                v1 = end_velocity[j]
                delta_q = q1 - q0
                
                if order == 1:
                    # 线性插值
                    q_interp = q0 + t * delta_q
                elif order == 3:
                    # 三次多项式插值: q(t) = a + b*t + c*t^2 + d*t^3
                    # 边界条件: q(0)=q0, q(1)=q1, q'(0)=v0, q'(1)=v1
                    # 解得: a=q0, b=v0, c=3*delta_q-2*v0-v1, d=v0+v1-2*delta_q
                    a = q0
                    b = v0
                    c = 3.0 * delta_q - 2.0 * v0 - v1
                    d = v0 + v1 - 2.0 * delta_q
                    q_interp = a + b * t + c * t * t + d * t * t * t
                elif order == 5:
                    # 五次多项式插值: q(t) = a + b*t + c*t^2 + d*t^3 + e*t^4 + f*t^5
                    # 边界条件: q(0)=q0, q(1)=q1, q'(0)=v0, q'(1)=v1, q''(0)=0, q''(1)=0
                    # 假设起点和终点加速度为0（平滑启动和停止）
                    a = q0
                    b = v0
                    c = 0.0  # 起点加速度为0
                    # 通过边界条件求解 d, e, f
                    # q(1) = q1: a + b + c + d + e + f = q1
                    # q'(1) = v1: b + 2*c + 3*d + 4*e + 5*f = v1
                    # q''(1) = 0: 2*c + 6*d + 12*e + 20*f = 0
                    # 解得:
                    d = 10.0 * delta_q - 6.0 * v0 - 4.0 * v1
                    e = -15.0 * delta_q + 8.0 * v0 + 7.0 * v1
                    f = 6.0 * delta_q - 3.0 * v0 - 3.0 * v1
                    q_interp = a + b * t + c * t * t + d * t * t * t + e * t * t * t * t + f * t * t * t * t * t
                else:
                    raise ValueError(f"不支持的多项式阶数: {order}，支持的值: 1, 3, 5")
                
                q_tmp.append(math.radians(q_interp))
            
            q_list.append(q_tmp)
        
        return q_list

    @staticmethod
    def compute_pose(
        robot: KuavoRobot,
        robot_state: KuavoRobotState,
        mode: Literal["left", "right", "both"],
        pos_left: Optional[List[float]] = None,
        quat_left: Optional[List[float]] = None,
        pos_right: Optional[List[float]] = None,
        quat_right: Optional[List[float]] = None,
    ) -> Tuple[List[float], List[float]]:
        """
        输入世界坐标系下的位置 + 四元数方向，进行 position 和 orientation（world → robot base）坐标转换，
        最后执行 IK 计算出左右手关节角度。
        """
        # 获取 base 在世界坐标系下的位置和朝向
        base_pos = np.array(robot_state.robot_position())
        base_quat = np.array(robot_state.robot_orientation())
        base_rot_inv = R.from_quat(base_quat).inv()

        def convert_pose(pos_world: List[float], quat_world: List[float]) -> KuavoPose:
            # 位置转换
            pos_rel = np.array(pos_world) - base_pos
            pos_robot = base_rot_inv.apply(pos_rel)
            # 朝向转换
            rot_world = R.from_quat(quat_world)
            rot_robot = base_rot_inv * rot_world
            quat_robot = rot_robot.as_quat().tolist()
            return KuavoPose(position=pos_robot.tolist(), orientation=quat_robot)

        left_pose = None
        right_pose = None

        if mode in ["left", "both"]:
            if pos_left is None or quat_left is None:
                raise ValueError("左手 pos / quat 缺失")
            left_pose = convert_pose(pos_left, quat_left)

        if mode in ["right", "both"]:
            if pos_right is None or quat_right is None:
                raise ValueError("右手 pos / quat 缺失")
            right_pose = convert_pose(pos_right, quat_right)

        # 如果只控制一只手，保持另一只手当前位置
        q_curr = robot_state.arm_joint_state().position
        if mode == "left":
            # 获取当前右手位置，保持右手不变
            fk_result = robot.arm_fk(q_curr)
            if fk_result is None or fk_result[0] is None or fk_result[1] is None:
                raise RuntimeError("FK 求解失败，无法获取当前手臂位置")
            right_pose = fk_result[1]
        elif mode == "right":
            # 获取当前左手位置，保持左手不变
            fk_result = robot.arm_fk(q_curr)
            if fk_result is None or fk_result[0] is None or fk_result[1] is None:
                raise RuntimeError("FK 求解失败，无法获取当前手臂位置")
            left_pose = fk_result[0]

        # 确保两个 pose 都已正确设置
        if left_pose is None or right_pose is None:
            raise ValueError(f"IK 求解失败：left_pose 或 right_pose 为 None (mode={mode})")
        params = KuavoIKParams(major_optimality_tol=1e-3, major_feasibility_tol=1e-3, minor_feasibility_tol=1e-3, major_iterations_limit=100, oritation_constraint_tol=1e-3, pos_constraint_tol=1e-3, pos_cost_weight=0.0)
        q_result = robot.arm_ik(left_pose, right_pose, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], q_curr,params)

        if q_result is None:
            raise RuntimeError("IK 求解失败，请检查目标位姿是否合理")

        left_q = list(q_result[:7]) if mode in ["left", "both"] else []
        right_q = list(q_result[7:]) if mode in ["right", "both"] else []

        return left_q, right_q
    
    @staticmethod
    def robot_to_world(pos_robot, robot_state):
        """
        机器人坐标系下的点，转换为世界坐标系下的位置

        参数:
            pos_robot: [x, y, z] 机器人坐标系下的点
            robot_state: KuavoRobotState 实例，需要有 robot_position() 和 robot_orientation()

        返回:
            pos_world: [x, y, z] 世界坐标系下的点
        """
        base_pos = np.array(robot_state.robot_position())          # base在世界坐标
        base_quat = np.array(robot_state.robot_orientation())      # 四元数(x, y, z, w)
        base_rot = R.from_quat(base_quat)
        pos_world = base_pos + base_rot.apply(pos_robot)
        return pos_world.tolist()
    
    @staticmethod
    def wait_for_grasp_pose(
            robot, robot_state,
            y_grasp_left=None,    # 左手目标抓取 y（世界系），可选（y轴模式）
            quat_left=None,       # 左手抓取姿态四元数，可选
            y_grasp_right=None,   # 右手目标抓取 y（世界系），可选（y轴模式）
            quat_right=None,      # 右手抓取姿态四元数，可选
            v=0.1,                # 传送带速度（y轴或x轴方向，取决于axis参数）
            mode="left",          # "left" / "right" / "both"
            obj_name_left="box_grab",  # 左手物体名
            obj_name_right="box_grab", # 右手物体名
            x_grasp_left=None,    # 左手目标x，可选
            z_grasp_left=None,    # 左手目标z，可选
            x_grasp_right=None,   # 右手目标x，可选
            z_grasp_right=None,   # 右手目标z，可选
            move_lead_time=0.3,   # 提前量（秒）
            check_interval=0.01,  # 轮询周期
            axis="y",             # 移动方向："y" 或 "x"（默认"y"保持向后兼容）
            timeout: Optional[float] = None  # 最大等待时间（秒），超时后返回基于当前位姿的估计或 (None, None)
        ):
        """
        等待物体移动到抓取位置并计算抓取姿态
        
        支持y轴和x轴方向的移动物体抓取：
        - axis="y": 物体沿y轴移动（默认，向后兼容）
        - axis="x": 物体沿x轴移动
        
        参数:
            axis: "y" 或 "x"，指定物体移动方向
            - "y": 使用 y_grasp_left/right 作为目标位置，物体沿y轴移动
            - "x": 使用 x_grasp_left/right 作为目标位置，物体沿x轴移动
        """

        from utils.utils import Utils

        obj_pos_reader = ObjectPose()
        
        # 验证axis参数
        if axis not in ["x", "y"]:
            raise ValueError(f"axis must be 'x' or 'y', got '{axis}'")

        start_time = time.time()

        def elapsed():
            return time.time() - start_time

        while True:
            # ------- 左手 -------
            if mode in ["left", "both"]:
                # 等待物体位姿可用，若超时则尝试返回当前可用数据
                while obj_pos_reader.get_position(obj_name_left) is None:
                    if timeout is not None and elapsed() > timeout:
                        print(f"[wait_for_grasp_pose] 超时等待物体 {obj_name_left} 的位姿，elapsed={elapsed():.2f}s")
                        # 超时：若有部分数据可用，继续到超时处理点
                        obj_pos_left = None
                        break
                    print(f"[wait_for_grasp_pose] 等待物体 {obj_name_left} 的位姿数据...")
                    time.sleep(0.01)
                obj_pos_left = obj_pos_reader.get_position(obj_name_left)
                
                # 根据axis选择移动方向
                if axis == "y":
                    # y轴模式：物体沿y轴移动
                    coord_obj_left = obj_pos_left[1]  # y坐标
                    coord_grasp_left = y_grasp_left   # 目标y坐标
                    coord_idx = 1  # y轴索引
                else:  # axis == "x"
                    # x轴模式：物体沿x轴移动
                    coord_obj_left = obj_pos_left[0]   # x坐标
                    coord_grasp_left = x_grasp_left    # 目标x坐标
                    coord_idx = 0  # x轴索引
                
                # 设置其他坐标（非移动方向）
                x_left = x_grasp_left if x_grasp_left is not None else obj_pos_left[0]
                z_left = z_grasp_left if z_grasp_left is not None else obj_pos_left[2]
                y_left = y_grasp_left if y_grasp_left is not None else obj_pos_left[1]
                
                # 计算到达目标位置的时间
                t_to_grasp_left = (coord_grasp_left - coord_obj_left) / v if coord_grasp_left is not None else float('inf')
            else:
                t_to_grasp_left = float('inf')

            # ------- 右手 -------
            if mode in ["right", "both"]:
                # 等待物体位姿可用，若超时则尝试返回当前可用数据
                while obj_pos_reader.get_position(obj_name_right) is None:
                    if timeout is not None and elapsed() > timeout:
                        print(f"[wait_for_grasp_pose] 超时等待物体 {obj_name_right} 的位姿，elapsed={elapsed():.2f}s")
                        obj_pos_right = None
                        break
                    print(f"[wait_for_grasp_pose] 等待物体 {obj_name_right} 的位姿数据...")
                    time.sleep(0.01)
                obj_pos_right = obj_pos_reader.get_position(obj_name_right)
                
                # 根据axis选择移动方向
                if axis == "y":
                    # y轴模式：物体沿y轴移动
                    coord_obj_right = obj_pos_right[1]  # y坐标
                    coord_grasp_right = y_grasp_right  # 目标y坐标
                    coord_idx = 1  # y轴索引
                else:  # axis == "x"
                    # x轴模式：物体沿x轴移动
                    coord_obj_right = obj_pos_right[0]  # x坐标
                    coord_grasp_right = x_grasp_right  # 目标x坐标
                    coord_idx = 0  # x轴索引
                
                # 设置其他坐标（非移动方向）
                x_right = x_grasp_right if x_grasp_right is not None else obj_pos_right[0]
                z_right = z_grasp_right if z_grasp_right is not None else obj_pos_right[2]
                y_right = y_grasp_right if y_grasp_right is not None else obj_pos_right[1]
                
                # 计算到达目标位置的时间
                t_to_grasp_right = (coord_grasp_right - coord_obj_right) / v if coord_grasp_right is not None else float('inf')
            else:
                t_to_grasp_right = float('inf')

            # 超时检查：如果超过 timeout，则返回基于当前观测的估计位姿（如果可用），否则返回 (None, None)
            if timeout is not None and elapsed() > timeout:
                print(f"[wait_for_grasp_pose] 超时退出主等待循环，elapsed={elapsed():.2f}s")
                # 尝试使用当前可用位姿计算 IK
                pos_left_est = None
                pos_right_est = None
                try:
                    if mode in ["left", "both"]:
                        if 'obj_pos_left' in locals() and obj_pos_left is not None:
                            x_left = x_grasp_left if x_grasp_left is not None else obj_pos_left[0]
                            z_left = z_grasp_left if z_grasp_left is not None else obj_pos_left[2]
                            y_left = y_grasp_left if y_grasp_left is not None else obj_pos_left[1]
                            pos_left_est = [x_left, y_left, z_left]
                    if mode in ["right", "both"]:
                        if 'obj_pos_right' in locals() and obj_pos_right is not None:
                            x_right = x_grasp_right if x_grasp_right is not None else obj_pos_right[0]
                            z_right = z_grasp_right if z_grasp_right is not None else obj_pos_right[2]
                            y_right = y_grasp_right if y_grasp_right is not None else obj_pos_right[1]
                            pos_right_est = [x_right, y_right, z_right]
                    left_q, right_q = Utils.compute_pose(
                        robot,
                        robot_state,
                        mode=mode,
                        pos_left=pos_left_est if mode in ['left', 'both'] else None,
                        quat_left=quat_left if mode in ['left', 'both'] else None,
                        pos_right=pos_right_est if mode in ['right', 'both'] else None,
                        quat_right=quat_right if mode in ['right', 'both'] else None,
                    )
                    return left_q, right_q
                except Exception as e:
                    print(f"[wait_for_grasp_pose] 超时后基于当前观测计算 IK 失败: {e}")
                    return None, None

            # 只要有一侧进了窗口就返回
            in_window = (
                (mode in ["left", "both"] and t_to_grasp_left <= move_lead_time) or
                (mode in ["right", "both"] and t_to_grasp_right <= move_lead_time)
            )
            if in_window:
                # 预测末端到达位置（做一次前馈）
                pos_left, pos_right = None, None
                if mode in ["left", "both"]:
                    if axis == "y":
                        # y轴模式：预测y坐标
                        y_pred_left = coord_obj_left + v * t_to_grasp_left
                        pos_left = [x_left, y_pred_left, z_left]
                    else:  # axis == "x"
                        # x轴模式：预测x坐标
                        x_pred_left = coord_obj_left + v * t_to_grasp_left
                        pos_left = [x_pred_left, y_left, z_left]
                        
                if mode in ["right", "both"]:
                    if axis == "y":
                        # y轴模式：预测y坐标
                        y_pred_right = coord_obj_right + v * t_to_grasp_right
                        pos_right = [x_right, y_pred_right, z_right]
                    else:  # axis == "x"
                        # x轴模式：预测x坐标
                        x_pred_right = coord_obj_right + v * t_to_grasp_right
                        pos_right = [x_pred_right, y_right, z_right]
                
                # 计算关节角
                left_q, right_q = Utils.compute_pose(
                    robot,
                    robot_state,
                    mode=mode,
                    pos_left=pos_left if mode in ['left', 'both'] else None,
                    quat_left=quat_left if mode in ['left', 'both'] else None,
                    pos_right=pos_right if mode in ['right', 'both'] else None,
                    quat_right=quat_right if mode in ['right', 'both'] else None,
                )
                return left_q, right_q
            time.sleep(check_interval)

    @staticmethod
    def is_front_facing(
        quat_xyzw,
        *,
        body_front_axis='x',          # 可为 'x'/'y'/'z' 或 '-x'/'-y'/'-z'
        front_world_dir='-x',         # 可为 'x'/'y'/'z'/'-x'/'-y'/'-z' 或 3D 向量(np.array/list/tuple)
        tol_deg=30.0
    ):
        """
        判定物体“正面法线”(由 body_front_axis 指定) 与期望世界方向 front_world_dir 的夹角是否 <= tol_deg。
        - quat_xyzw: 四元数，顺序 **(x, y, z, w)**，与 SciPy 一致
        - body_front_axis: 物体坐标系里的“正面法线”轴，可带负号
        - front_world_dir: 期望的世界方向(字符串轴或3D向量)
        - tol_deg: 角度阈值（度）
        返回:
            (is_front: bool, angle_deg: float)
        """

        def _axis_to_vec(axis_str: str) -> np.ndarray:
            axis_str = axis_str.strip().lower()
            table = {
                'x':  np.array([ 1., 0., 0.]),
                '-x': np.array([-1., 0., 0.]),
                'y':  np.array([ 0., 1., 0.]),
                '-y': np.array([ 0.,-1., 0.]),
                'z':  np.array([ 0., 0., 1.]),
                '-z': np.array([ 0., 0.,-1.]),
            }
            if axis_str not in table:
                raise ValueError("axis must be one of: x, -x, y, -y, z, -z")
            return table[axis_str]

        # 1) 解析 body_front_axis 为局部单位向量
        if isinstance(body_front_axis, str):
            n_body = _axis_to_vec(body_front_axis)
        else:
            raise ValueError("body_front_axis must be a string axis like 'x', '-y', etc.")

        # 2) 解析 front_world_dir 为世界单位向量
        if isinstance(front_world_dir, str):
            fw = _axis_to_vec(front_world_dir)
        else:
            fw = np.asarray(front_world_dir, dtype=float)
            if fw.shape != (3,):
                raise ValueError("front_world_dir vector must be shape (3,)")
            n = np.linalg.norm(fw)
            if n < 1e-12:
                raise ValueError("front_world_dir vector norm is zero")
            fw = fw / n

        # 3) 归一化四元数并构造旋转 (SciPy 使用 xyzw)
        q = np.asarray(quat_xyzw, dtype=float)
        if q.shape != (4,):
            raise ValueError("quat_xyzw must be length-4 (x, y, z, w)")
        nq = np.linalg.norm(q)
        if nq < 1e-12:
            # 退化情形：视为单位四元数
            q = np.array([0., 0., 0., 1.], dtype=float)
        else:
            q = q / nq

        Rw = R.from_quat(q)

        # 4) 将局部“正面法线”旋转到世界系
        n_world = Rw.apply(n_body)
        n = np.linalg.norm(n_world)
        if n < 1e-12:
            # 极小概率的数值异常，直接返回 False, 180°
            return False, 180.0
        n_world = n_world / n

        # 5) 计算夹角并判定
        cosang = float(np.clip(np.dot(n_world, fw), -1.0, 1.0))
        angle = math.degrees(math.acos(cosang))
        return (angle <= tol_deg), angle

    @staticmethod
    def _build_segment_rad(target_rad, start_rad=None):
        """构造仅含起点和终点的轨迹，交给 Ruckig 做时间参数化（弧度版本）"""
        if start_rad is None:
            start_rad = [0.0] * len(target_rad)
        return [list(start_rad), list(target_rad)]

    @staticmethod
    def execute_trajectory_with_ruckig(
            robot: KuavoRobot,
            robot_state: KuavoRobotState,
            traj_ctrl,
            q_target,
            q_start: Optional[List[float]] = None,
            sleep_time: float = 0.01,
            joint_v_max: Optional[List[float]] = None,
            joint_a_max: Optional[List[float]] = None,
            joint_j_max: Optional[List[float]] = None,
            speed_scale: float = 3.0,
            vel_settle_thresh: float = 0.03,
            publish_frequency: float = 100.0
        ):
        """
        使用 Ruckig 进行在线时间参数化的轨迹执行
        
        该函数使用 Ruckig 库对轨迹进行平滑的时间参数化，确保满足速度、加速度和加加速度约束。
        完全按照 task1_ruckig.py 中的 execute_with_ruckig 实现。
        
        参数:
            robot: KuavoRobot 实例
            robot_state: KuavoRobotState 实例，用于获取当前关节状态
            traj_ctrl: TrajectoryController 实例，用于发布关节角度命令
            q_target: 目标关节角度（弧度），可以是：
                    - List[float]: 单个目标点，会与 q_start 拼接成 [[start], [target]]
                    - List[List[float]]: 多个waypoints，直接使用
                        注意：多个waypoint时，第一个waypoint在执行时会被替换为当前位置
                        （与 task1_ruckig.py 的实现一致，确保从当前位置开始）
            q_start: 起点关节角度（弧度），仅在 q_target 是单个点时使用，默认None（使用当前位置）
            sleep_time: 用于计算自适应速度的基准时间间隔（秒），默认0.01
            joint_v_max: 每个关节的最大速度限制（rad/s），默认[2.5]*14
            joint_a_max: 每个关节的最大加速度限制（rad/s^2），默认[5.0]*14
            joint_j_max: 每个关节的最大加加速度限制（rad/s^3），默认[50.0]*14
            speed_scale: 速度缩放系数，用于提升规划速度，默认3.0
            vel_settle_thresh: 速度阈值（rad/s），低于此值视为静止，默认0.03
            publish_frequency: 轨迹发布频率（Hz），默认100.0
        
        返回:
            bool: 执行是否成功
        
        参数配置建议:
            - joint_v_max: 根据机器人硬件限制设置，典型值 1.0-3.0 rad/s
            - joint_a_max: 通常为 joint_v_max 的 2-3 倍，典型值 3.0-10.0 rad/s^2
            - joint_j_max: 通常为 joint_a_max 的 5-10 倍，典型值 20.0-100.0 rad/s^3
            - speed_scale: 根据实际需求调整，1.0-5.0 之间，值越大速度越快但可能不够平滑
            - vel_settle_thresh: 用于抑制启动抖动，典型值 0.01-0.05 rad/s
            - sleep_time: 用于自适应速度计算，通常与期望的轨迹执行时间相关
        
        示例:
            >>> from utils.trajectory_controller import TrajectoryController
            >>> traj_ctrl = TrajectoryController(robot)
            
            # 方式1: 单个目标点，自动拼接起点和终点
            >>> q_start = [0.0, 0.0, ...]  # 起点（弧度）
            >>> q_target = [0.1, 0.2, ...]  # 终点（弧度）
            >>> Utils.execute_trajectory_with_ruckig(
            ...     robot, robot_state, traj_ctrl, q_target, q_start=q_start,
            ...     sleep_time=0.01
            ... )
            
            # 方式2: 多个waypoints，直接传入列表
            >>> q_list = [[q1], [q2], [q3]]  # 多个waypoints（弧度）
            >>> Utils.execute_trajectory_with_ruckig(
            ...     robot, robot_state, traj_ctrl, q_list,
            ...     sleep_time=0.01
            ... )
        """
        try:
            from ruckig import Ruckig, InputParameter, OutputParameter, Result
            HAS_RUCKIG = True
        except ImportError:
            HAS_RUCKIG = False
            if not hasattr(Utils.execute_trajectory_with_ruckig, "_warned"):
                import rospy
                rospy.logwarn("Ruckig 未安装，将使用固定插值执行轨迹")
                Utils.execute_trajectory_with_ruckig._warned = True
        
        import rospy
        
        # 处理输入：判断是单个点还是多个waypoints
        if not q_target:
            rospy.logerr("无效的输入：q_target 不能为空")
            return False
        
        # 判断输入类型
        if isinstance(q_target[0], (int, float)):
            # 单个目标点：使用 _build_segment_rad 拼接起点和终点
            if q_start is None:
                # 如果没有提供起点，从当前位置获取
                curr_state = robot_state.arm_joint_state()
                q_start = list(curr_state.position)
            q_list = Utils._build_segment_rad(q_target, q_start)
        elif isinstance(q_target[0], (list, tuple)):
            # 多个waypoints：直接使用
            # 注意：第一个waypoint会被用作起点，但执行时会被替换为当前位置（与原始实现一致）
            q_list = [list(q) for q in q_target]
        else:
            rospy.logerr(f"无效的输入类型：q_target[0] 的类型是 {type(q_target[0])}")
            return False
        
        # 验证输入
        if not q_list:
            return True
        
        # 如果没有 Ruckig，回退到固定插值
        if not HAS_RUCKIG:
            traj_ctrl.execute_trajectory(q_list, sleep_time=sleep_time)
            return True
        
        # 设置默认参数
        dof = len(q_list[0])
        if joint_v_max is None:
            joint_v_max = [2.5] * dof
        if joint_a_max is None:
            joint_a_max = [5.0] * dof
        if joint_j_max is None:
            joint_j_max = [50.0] * dof
        
        # 初始化 Ruckig
        dt = 1.0 / publish_frequency
        otg = Ruckig(dof, dt)
        out = OutputParameter(dof)
        
        # 获取当前状态（从 robot_state 获取，第一个waypoint会在循环中被替换）
        curr_state = robot_state.arm_joint_state()
        curr_pos = list(curr_state.position)
        measured_vel = list(getattr(curr_state, "velocity", [0.0] * dof) or [0.0] * dof)
        vel_norm = sum(v * v for v in measured_vel) ** 0.5
        if vel_norm < vel_settle_thresh:
            measured_vel = [0.0] * dof
        curr_vel = measured_vel
        curr_acc = [0.0] * dof
        min_v = 0.01  # 避免 vmax 为 0 触发 Ruckig 错误
        
        traj_ctrl.set_target_positions(curr_pos)
        time.sleep(dt * 2.0)
        
        for idx, waypoint in enumerate(q_list):
            if idx == 0:
                waypoint = curr_pos.copy()
            
            delta_max = max(abs(waypoint[j] - curr_pos[j]) for j in range(dof))
            if delta_max < 1e-3:
                continue
            # 若 waypoint 与当前非常接近，则跳过
            if max(abs(waypoint[j] - curr_pos[j]) for j in range(dof)) < 1e-6:
                continue
            
            inp = InputParameter(dof)
            inp.current_position = curr_pos
            inp.current_velocity = curr_vel
            inp.current_acceleration = curr_acc
            inp.target_position = waypoint
            # 为了贴近原始轨迹节奏，按预期步长对 vmax 做自适应缩放
            adaptive_vmax = []
            for j in range(dof):
                raw_delta = abs(waypoint[j] - curr_pos[j])
                desired_v = raw_delta / max(sleep_time, 1e-3)
                vmax = min(joint_v_max[j] * speed_scale, desired_v * 1.2 * speed_scale)
                adaptive_vmax.append(max(min_v, vmax))
            inp.target_velocity = [0.0] * dof  # 确保每个离散点都到位，防止"绕过"waypoint
            inp.target_acceleration = [0.0] * dof
            inp.max_velocity = adaptive_vmax
            inp.max_acceleration = joint_a_max
            inp.max_jerk = joint_j_max
            
            res = Result.Working
            safety = 0
            try:
                while res == Result.Working and safety < 5000:
                    res = otg.update(inp, out)
                    traj_ctrl.set_target_positions(out.new_position)
                    time.sleep(dt)
                    # 将输出作为下一循环的 current
                    inp.current_position = out.new_position
                    inp.current_velocity = out.new_velocity
                    inp.current_acceleration = out.new_acceleration
                    safety += 1
            except Exception as e:
                rospy.logwarn(f"Ruckig 出错，退回固定插值执行此段: {e}")
                remaining = q_list[idx:]
                traj_ctrl.execute_trajectory(remaining, sleep_time=sleep_time)
                return True
            curr_pos = out.new_position
            curr_vel = out.new_velocity
            curr_acc = out.new_acceleration
        
        rospy.loginfo("Ruckig 轨迹执行完成")
        return True

    @staticmethod
    def augment_trajectory(
        q_list,
        base_sleep,
        noise_deg=3,
        via_noise_deg=3,
        stretch_range=(0.8, 1.2),
        per_step_random=False
    ):
        """
        对轨迹做轻量随机形变以提升轨迹泛化，返回新轨迹与休眠时间
        
        参数:
            q_list: 原始轨迹点列表
            base_sleep: 基准休眠时间
            noise_deg: 角度噪声（度）
            via_noise_deg: 中间点噪声（度）
            stretch_range: 时间拉伸范围 (min, max)
            per_step_random: 如果为True，每个点之间的延迟都随机；如果为False，整个轨迹使用同一个延迟
        
        返回:
            augmented: 增强后的轨迹
            sleep_time: 如果 per_step_random=False，返回单个值；如果 per_step_random=True，返回列表
        """
        if not q_list:
            if per_step_random:
                return q_list, [base_sleep]
            return q_list, base_sleep
        n = len(q_list)

        def center_taper(idx):
            if n <= 1:
                return 0.0
            # 让噪声在中段更大，两端更小，避免起终点失配
            return max(0.0, 1.0 - abs(2.0 * idx / (n - 1) - 1.0))

        augmented = []
        for i, q in enumerate(q_list):
            weight = center_taper(i)
            jitter = [
                val + math.radians(random.uniform(-noise_deg, noise_deg)) * weight
                for val in q
            ]
            augmented.append(jitter)

        # 随机插入一个经扰动的中间 via point，让轨迹形状略有改变
        if n > 6 and random.random() < 0.6:
            mid_idx = random.randint(n // 4, 3 * n // 4)
            via = [
                augmented[mid_idx][j] + math.radians(random.uniform(-via_noise_deg, via_noise_deg))
                for j in range(len(augmented[mid_idx]))
            ]
            augmented = augmented[:mid_idx] + [via] + augmented[mid_idx:]

        if per_step_random:
            # 每个点之间的延迟都随机
            sleep_times = []
            for i in range(len(augmented)):
                sleep_times.append(base_sleep * random.uniform(stretch_range[0], stretch_range[1]))
            
            if random.random() < 0.5:
                # 偶尔在中途增加一个短暂停留，模拟不同的速度曲线
                pause_idx = random.randint(len(augmented) // 3, 2 * len(augmented) // 3)
                augmented.insert(pause_idx, augmented[pause_idx])
                # 为插入的点也添加一个随机的 sleep_time
                sleep_times.insert(pause_idx, base_sleep * random.uniform(stretch_range[0], stretch_range[1]))
            
            return augmented, sleep_times
        else:
            # 整个轨迹使用同一个延迟（原始行为）
            sleep_time = base_sleep * random.uniform(stretch_range[0], stretch_range[1])
            if random.random() < 0.5:
                # 偶尔在中途增加一个短暂停留，模拟不同的速度曲线
                pause_idx = random.randint(len(augmented) // 3, 2 * len(augmented) // 3)
                augmented.insert(pause_idx, augmented[pause_idx])
                
            return augmented, sleep_time
    
    @staticmethod
    def euler_to_quat(roll, pitch, yaw):
        """ZYX 欧拉角转四元数"""
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return {'x': qx, 'y': qy, 'z': qz, 'w': qw}

    def sample_orientation_jitter(max_yaw_deg=180.0, max_pitch_deg=0, max_roll_deg=0):
        """为每个生成物体添加轻量姿态扰动"""
        yaw = math.radians(random.uniform(-max_yaw_deg, max_yaw_deg))
        pitch = math.radians(random.uniform(-max_pitch_deg, max_pitch_deg))
        roll = math.radians(random.uniform(-max_roll_deg, max_roll_deg))
        return Utils.euler_to_quat(roll, pitch, yaw)