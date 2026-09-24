import math
from typing import List, Tuple, Optional, Literal
import xml.etree.ElementTree as ET
import random, time
from kuavo_humanoid_sdk import KuavoPose, KuavoRobotState, KuavoRobot
from kuavo_humanoid_sdk.interfaces.data_types import KuavoIKParams
from scipy.spatial.transform import Rotation as R
import numpy as np
from utils.object_pos import ObjectPose

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

        if num <= 0:
            raise ValueError("num must be positive")

        q_list = []
        for i in range(1, num + 1):
            q_tmp = [
                math.radians(q_start[j] + i / float(num) * (q_target[j] - q_start[j]))
                for j in range(len(q_target))
            ]
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
        ik_base_position_world: Optional[List[float]] = None,
        ik_base_orientation_world: Optional[List[float]] = None,
        ik_params: Optional[KuavoIKParams] = None,
    ) -> Tuple[List[float], List[float]]:
        """
        将世界系 TCP 目标转换到 IK 根坐标系并求左右臂关节角。

        Scene 1 必须显式传入 waist_yaw_link 的世界位姿，因为 S400062
        上半身 IK 模型的根虽然命名为 base_link，几何上实际对应整机模型的
        waist_yaw_link。未传入时保留旧调用行为，使用 /odom 中的 base_link。
        """
        has_ik_base_position = ik_base_position_world is not None
        has_ik_base_orientation = ik_base_orientation_world is not None
        if has_ik_base_position != has_ik_base_orientation:
            raise ValueError("IK base position and orientation must be provided together")

        if has_ik_base_position:
            base_pos = np.array(ik_base_position_world)
            base_quat = np.array(ik_base_orientation_world)
        else:
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
        q_curr = list(robot_state.arm_joint_state().position)
        if len(q_curr) != 14:
            raise RuntimeError(
                "Expected 14 live arm joints for IK seed, got {}".format(
                    len(q_curr)))
        if mode == "left":
            right_pose = robot.arm_fk(q_curr)[1]
        elif mode == "right":
            left_pose = robot.arm_fk(q_curr)[0]

        # The current live arm state is the seed, so the solver starts on the
        # same IK branch as the posture visible in the simulator.  Constraint
        # policy remains explicit at the task call site.
        q_result = robot.arm_ik(
            left_pose,
            right_pose,
            arm_q0=q_curr,
            params=ik_params,
        )

        if q_result is None:
            raise RuntimeError("IK 求解失败，请检查目标位姿是否合理")

        if ik_params is not None and (ik_params.constraint_mode & 0x02):
            solved_fk = robot.arm_fk(q_result)
            if solved_fk is None or len(solved_fk) != 2:
                raise RuntimeError("FK failed while validating the IK result")
            acceptance = max(2.0 * ik_params.pos_constraint_tol, 1e-3)
            pose_checks = []
            if mode in ["left", "both"]:
                pose_checks.append(("left", solved_fk[0], left_pose))
            if mode in ["right", "both"]:
                pose_checks.append(("right", solved_fk[1], right_pose))
            for side, solved_pose, target_pose in pose_checks:
                position_error = np.linalg.norm(
                    np.array(solved_pose.position)
                    - np.array(target_pose.position))
                if position_error > acceptance:
                    raise RuntimeError(
                        "IK returned {} TCP position error {:.6f} m "
                        "(allowed {:.6f} m)".format(
                            side, position_error, acceptance))

        left_q = list(q_result[:7]) if mode in ["left", "both"] else []
        right_q = list(q_result[7:]) if mode in ["right", "both"] else []

        return left_q, right_q

    @staticmethod
    def ik_base_orientation_to_world(
            quat_ik_base: List[float],
            ik_base_orientation_world: List[float]) -> List[float]:
        """Convert an IK-root-frame orientation to world-frame XYZW."""
        rot_world_ik_base = R.from_quat(ik_base_orientation_world)
        rot_ik_base_tcp = R.from_quat(quat_ik_base)
        return (rot_world_ik_base * rot_ik_base_tcp).as_quat().tolist()

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
            y_grasp_left=None,    # 左手目标抓取 y（世界系），可选
            quat_left=None,       # 左手抓取姿态四元数，可选
            y_grasp_right=None,   # 右手目标抓取 y（世界系），可选
            quat_right=None,      # 右手抓取姿态四元数，可选
            v=0.1,                # 传送带y方向速度
            mode="left",          # "left" / "right" / "both"
            obj_name_left="box_grab",  # 左手物体名
            obj_name_right="box_grab", # 右手物体名
            x_grasp_left=None,    # 左手目标x，可选
            z_grasp_left=None,    # 左手目标z，可选
            x_grasp_right=None,   # 右手目标x，可选
            z_grasp_right=None,   # 右手目标z，可选
            time_offset = None,
            move_lead_time=0.3,   # 提前量（秒）
            check_interval=0.01   # 轮询周期
        ):

        from utils.utils import Utils

        obj_pos_reader = ObjectPose()

        while True:
            # ------- 左手 -------
            if mode in ["left", "both"]:
                while obj_pos_reader.get_position(obj_name_left) is None:
                    print(f"[wait_for_grasp_pose] 等待物体 {obj_name_left} 的位姿数据...")
                    time.sleep(0.01)
                obj_pos_left = obj_pos_reader.get_position(obj_name_left)
                y_obj_left = obj_pos_left[1]
                x_left = x_grasp_left if x_grasp_left is not None else obj_pos_left[0]
                z_left = z_grasp_left if z_grasp_left is not None else obj_pos_left[2]
                t_to_grasp_left = (y_grasp_left - y_obj_left) / v if y_grasp_left is not None else float('inf')
            else:
                t_to_grasp_left = float('inf')

            # ------- 右手 -------
            if mode in ["right", "both"]:
                while obj_pos_reader.get_position(obj_name_right) is None:
                    print(f"[wait_for_grasp_pose] 等待物体 {obj_name_right} 的位姿数据...")
                    time.sleep(0.01)
                obj_pos_right = obj_pos_reader.get_position(obj_name_right)
                y_obj_right = obj_pos_right[1]
                x_right = x_grasp_right if x_grasp_right is not None else obj_pos_right[0]
                z_right = z_grasp_right if z_grasp_right is not None else obj_pos_right[2]
                t_to_grasp_right = (y_grasp_right - y_obj_right) / v if y_grasp_right is not None else float('inf')
            else:
                t_to_grasp_right = float('inf')

            # 只要有一侧进了窗口就返回
            in_window = (
                (mode in ["left", "both"] and t_to_grasp_left <= move_lead_time) or
                (mode in ["right", "both"] and t_to_grasp_right <= move_lead_time)
            )
            if in_window:
                # 预测末端到达位置（做一次前馈）
                pos_left, pos_right = None, None
                if mode in ["left", "both"]:
                    y_pred_left = y_obj_left + v * t_to_grasp_left
                    pos_left = [x_left, y_pred_left, z_left]
                if mode in ["right", "both"]:
                    y_pred_right = y_obj_right + v * t_to_grasp_right
                    pos_right = [x_right, y_pred_right, z_right]
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
