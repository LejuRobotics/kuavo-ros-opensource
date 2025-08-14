import math
from typing import List, Tuple, Optional, Literal
import xml.etree.ElementTree as ET
import random, time
from kuavo_humanoid_sdk import KuavoPose, KuavoRobotState, KuavoRobot
from scipy.spatial.transform import Rotation as R
import numpy as np
from utils.object_pos import ObjectPose
from utils.object_randomizer import ObjectRandomizer  # 你的类
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
            right_pose = robot.arm_fk(q_curr)[1]
        elif mode == "right":
            left_pose = robot.arm_fk(q_curr)[0]

        q_result = robot.arm_ik(left_pose, right_pose)

        # if q_result is None:
        #     raise RuntimeError("IK 求解失败，请检查目标位姿是否合理")

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
        body_front_axis='x',              # 物体坐标系里“正面法线”是哪根轴：'x'/'y'/'z'
        front_world_dir=np.array([-1,0,0]),  # 期望的“正面”在世界坐标系指向哪边，默认朝世界 -X
        tol_deg=30.0                      # 允许的夹角阈值（度）
    ):
        """
        返回 (is_front, angle_deg)
        - is_front: 是否判定为“正面朝向”
        - angle_deg: 正面法线与目标方向的夹角（度）
        注意：只对“正面法线”方向做判断，绕法线的自旋不影响结果。
        """
        # 物体坐标系中的正面法线（单位向量）
        if body_front_axis == 'x':
            n_body = np.array([1,0,0], dtype=float)
        elif body_front_axis == 'y':
            n_body = np.array([0,1,0], dtype=float)
        elif body_front_axis == 'z':
            n_body = np.array([0,0,1], dtype=float)
        else:
            raise ValueError("body_front_axis must be 'x'|'y'|'z'")

        # 将物体法线旋转到世界系
        q = R.from_quat(quat_xyzw)  # 注意 SciPy 用的是 [x,y,z,w]
        n_world = q.apply(n_body)

        # 与期望方向的夹角
        fw = np.array(front_world_dir, dtype=float)
        fw /= np.linalg.norm(fw)
        n_world /= np.linalg.norm(n_world)

        cosang = float(np.clip(np.dot(n_world, fw), -1.0, 1.0))
        angle = math.degrees(math.acos(cosang))

        return (angle <= tol_deg), angle