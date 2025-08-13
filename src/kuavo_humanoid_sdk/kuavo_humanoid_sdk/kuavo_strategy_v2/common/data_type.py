import numpy as np
from typing import Tuple
from scipy.spatial.transform import Rotation as R
from enum import Enum

from kuavo_humanoid_sdk.kuavo_strategy_v2.utils.utils import quaternion_rotate, quaternion_multiply, quaternion_inverse


class Frame(str, Enum):
    BASE = "base_link"  # 基座坐标系
    CAMERA_LINK = "camera_link"  # 相机坐标系
    ODOM = "odom"  # 里程计原点
    # WORLD = "world"  # 世界坐标系
    ROBOT = "robot"  # 机器人坐标系
    TAG = "tag"  # AprilTag坐标系


class Point:
    def __init__(self, x: float, y: float, z: float, frame: Frame):
        """
        初始化三维坐标点。

        参数：
            x (float): x坐标。
            y (float): y坐标。
            z (float): z坐标。
            frame (Frame): 坐标系框架名称。
        """
        self.x = x
        self.y = y
        self.z = z
        self.frame = frame  # 坐标系框架名称

    def __repr__(self):
        """
        返回Point对象的字符串表示。
        """
        return f"Point(x={self.x}, \n y={self.y}, \n z={self.z}, \n frame={self.frame})"


class Pose:
    def __init__(self,
                 pos: Tuple[float, float, float],
                 quat: Tuple[float, float, float, float],
                 frame: str = None,
                 ):
        """
        初始化姿态对象，包含位置和四元数。

        参数：
            pos (Tuple[float, float, float]): 位置坐标。
            quat (Tuple[float, float, float, float]): 四元数 (x, y, z, w)。
            frame (str): 坐标系框架名称，可选。
        """
        self.pos = np.array(pos)
        self.quat = np.array(quat)
        self.frame = frame  # 坐标系框架名称，可选

    @classmethod
    def from_euler(cls, pos: Tuple[float, float, float], euler: Tuple[float, float, float], frame=None, degrees=True):
        """
        从欧拉角创建Pose对象。

        参数：
            pos (Tuple[float, float, float]): 位置坐标。
            euler (Tuple[float, float, float]): 欧拉角。
            frame (str): 坐标系框架名称，可选。
            degrees (bool): 欧拉角是否为度数。

        返回：
            Pose: 创建的Pose对象。
        """
        r = R.from_euler('xyz', euler, degrees=degrees)
        quat = r.as_quat()  # x, y, z, w
        # array to tuple
        return cls(pos, quat, frame)

    @classmethod
    def from_rotmat(cls, pos: Tuple[float, float, float], rotmat: Tuple[float, float, float], frame=None):
        """
        从旋转矩阵创建Pose对象。

        参数：
            pos (Tuple[float, float, float]): 位置坐标。
            rotmat (Tuple[float, float, float]): 旋转矩阵。
            frame (str): 坐标系框架名称，可选。

        返回：
            Pose: 创建的Pose对象。
        """
        r = R.from_matrix(rotmat)
        quat = r.as_quat()
        return cls(pos, quat, frame)

    def get_quat(self):
        """
        获取姿态的四元数。

        返回：
            np.ndarray: 四元数。
        """
        return self.quat

    def get_euler(self, degrees=False):
        """
        获取姿态的欧拉角。

        参数：
            degrees (bool): 是否返回度数。

        返回：
            np.ndarray: 欧拉角。
        """
        r = R.from_quat(self.quat)
        return r.as_euler('xyz', degrees=degrees)

    def get_rotmat(self):
        """
        获取姿态的旋转矩阵。

        返回：
            np.ndarray: 旋转矩阵。
        """
        r = R.from_quat(self.quat)
        return r.as_matrix()

    def position_l1_norm(self, other: "Pose"):
        """
        计算位置的L1范数（曼哈顿距离）。

        参数：
            other (Pose): 另一个姿态对象。

        返回：
            float: L1范数。
        """
        return np.sum(np.abs(self.pos - other.pos))

    def position_l2_norm(self, other: "Pose"):
        """
        计算位置的L2范数（欧氏距离）。

        参数：
            other (Pose): 另一个姿态对象。

        返回：
            float: L2范数。
        """
        return np.linalg.norm(self.pos - other.pos)

    def position_l2_norm_squared(self, other: "Pose"):
        """
        计算位置的L2范数的平方。

        参数：
            other (Pose): 另一个姿态对象。

        返回：
            float: L2范数的平方。
        """
        return np.sum((self.pos - other.pos) ** 2)

    def angle(self, other: "Pose"):
        """
        计算姿态的角度差（弧度）。

        参数：
            other (Pose): 另一个姿态对象。

        返回：
            float: 角度差。
        """
        # 确保四元数已归一化
        quat1_norm = np.linalg.norm(self.quat)
        quat2_norm = np.linalg.norm(other.quat)

        if quat1_norm == 0 or quat2_norm == 0:
            return 0.0

        # 计算归一化的四元数点积
        quat1_normalized = self.quat / quat1_norm
        quat2_normalized = other.quat / quat2_norm

        quat_dot = np.abs(np.dot(quat1_normalized, quat2_normalized))

        # 确保点积在有效范围内 [-1, 1]
        quat_dot = np.clip(quat_dot, -1.0, 1.0)

        return 2 * np.arccos(quat_dot)

    def angle_yaw(self, other: "Pose"):
        """
        计算姿态的角度差（弧度）。
        只考虑yaw的误差。用于2d情况下的计算。
        参数：
            other (Pose): 另一个姿态对象。

        返回：
            float: 角度差。
        """
        # 确保四元数已归一化
        # quat1_norm = np.linalg.norm(self.quat)
        # quat2_norm = np.linalg.norm(other.quat)
        #
        # if quat1_norm == 0 or quat2_norm == 0:
        #     return 0.0
        #
        # # 计算归一化的四元数点积
        # quat1_normalized = self.quat / quat1_norm
        # quat2_normalized = other.quat / quat2_norm

        quat1_normalized = R.from_quat(self.quat)
        quat2_normalized = R.from_quat(other.quat)

        # 相对旋转：从 self 变换到 other
        r_rel = quat2_normalized * quat1_normalized.inv()
        print(r_rel)
        # 转换为欧拉角
        yaw = r_rel.as_euler("xyz", degrees=False)[2]
        return yaw

    def __repr__(self):
        """
        返回Pose对象的字符串表示。
        """
        return f"Pose(pos={self.pos}, \n quat={self.quat}, \n euler={self.get_euler(degrees=True)}, \n frame={self.frame})"


class Tag:
    def __init__(self, id: int, pose: Pose, size: float = None):
        """
        初始化AprilTag对象。

        参数：
            id (int): AprilTag ID。
            pose (Pose): Tag的Pose对象，包含位置和方向。
            size (float): AprilTag物理尺寸，单位米。
        """
        self.id = id
        self.size = size
        self.pose = pose

    def __repr__(self):
        """
        返回Tag对象的字符串表示。
        """
        return f"Tag(id={self.id}, size={self.size}, pose={self.pose})"

class Transform3D:
    def __init__(self, trans_pose: Pose, source_frame: Frame, target_frame: Frame):
        """
        初始化3D变换对象，通过translation和rotation来定义。

        参数：
            trans_pose (Pose): 变换的Pose对象。
            source_frame (Frame): 被转换数据所在的坐标系。
            target_frame (Frame): 要转到的坐标系。
        """
        assert trans_pose.frame == target_frame, f"转换的目标坐标系{target_frame}必须与传入的Pose的frame {trans_pose.frame}一致"
        self.trans_pose = trans_pose
        self.source_frame = source_frame  # 源坐标系
        self.target_frame = target_frame  # 目标坐标系

    def apply_to_pose_inverse(self, pose: Pose):
        """
        将Pose从target_frame逆变换到source_frame。
        """

        assert pose.frame == self.target_frame, "逆变换要求Pose的frame必须是target_frame"

        # 1. 姿态逆变换
        quat_pose = np.array(pose.quat)
        quat_trans = np.array(self.trans_pose.quat)
        quat_trans_inv = quaternion_inverse(quat_trans)
        quat_new = quaternion_multiply(quat_trans_inv, quat_pose)

        # 2. 位置逆变换
        # pos_diff = pose.pos - trans.pos，然后用 trans.quat 的逆旋转回来
        pos_diff = np.array(pose.pos) - np.array(self.trans_pose.pos)
        pos_new = quaternion_rotate(quat_trans_inv, pos_diff)

        return Pose(
            pos=pos_new,
            quat=quat_new,
            frame=self.source_frame
        )

    def apply_to_pose(self, pose: Pose):
        """
        将Pose从source_frame转换到target_frame。

        参数：
            pose (Pose): 要转换的Pose对象。

        返回：
            Pose: 转换后的Pose对象。
        """
        assert pose.frame == self.source_frame, "转换的目标坐标系必须与传入的Pose的frame一致"
        # 位置转换
        rotated_pos = quaternion_rotate(
            np.array(self.trans_pose.quat),  # 确保四元数是numpy数组
            pose.pos
        )
        pos = np.asarray(self.trans_pose.pos) + rotated_pos

        # 姿态转换（显式转换为numpy数组）
        quat_1 = np.array(pose.quat)
        quat_2 = np.array(self.trans_pose.quat)
        quat = quaternion_multiply(quat_2, quat_1)

        return Pose(
            pos=pos.tolist(),
            quat=quat,
            frame=self.target_frame  # 返回转换后的坐标系
        )

    def apply_to_point(self, point: Point):
        """
        将Point从source_frame转换到target_frame。

        参数：
            point (Point): 要转换的Point对象。

        返回：
            Point: 转换后的Point对象。
        """
        assert point.frame == self.source_frame, "转换的目标坐标系必须与传入的Point的frame一致"
        rotated_pos = quaternion_rotate(
            np.array(self.trans_pose.quat),  # 确保四元数是numpy数组
            [point.x, point.y, point.z]  # 将Point转换为列表
        )
        pos = np.asarray(self.trans_pose.pos) + rotated_pos
        pos = pos.tolist()

        return Point(
            x=pos[0],
            y=pos[1],
            z=pos[2],
            frame=self.target_frame  # 返回转换后的坐标系
        )