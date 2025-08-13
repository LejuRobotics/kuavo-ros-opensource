from typing import Tuple
from dataclasses import dataclass
import numpy as np

@dataclass
class KuavoPose:
    """表示机器人姿态的数据类。"""
    position: Tuple[float, float, float] # x, y, z
    orientation: Tuple[float, float, float, float] # x, y, z, w

    def __str__(self) -> str:
        """Returns a formatted string representation of the pose.
        
        Returns:
            str: Formatted pose string with position and orientation
        """
        return (
            f"Position (x,y,z): ({self.position[0]:.3f}, {self.position[1]:.3f}, {self.position[2]:.3f})\n"
            f"Orientation (x,y,z,w): ({self.orientation[0]:.3f}, {self.orientation[1]:.3f}, "
            f"{self.orientation[2]:.3f}, {self.orientation[3]:.3f})"
        )


@dataclass
class HomogeneousMatrix:
    """用于3D变换的4x4齐次变换矩阵。
    
    表示3D空间中的旋转和平移。可用于坐标系变换和姿态合成。 \n
    
    属性:
        matrix (np.ndarray): 形状为(4, 4)的4x4 numpy数组，包含::
        
            [[R, t],
             [0, 1]] 
            
        其中R是3x3旋转矩阵，t是3x1平移向量
    """
    matrix: np.ndarray  # 形状(4,4)的齐次变换矩阵

@dataclass
class PoseQuaternion:
    """使用位置和四元数方向的3D姿态表示"""
    position: Tuple[float, float, float]
    """XYZ坐标，单位为米"""
    orientation: Tuple[float, float, float, float]
    """单位四元数，采用(x, y, z, w)格式，遵循ROS约定"""
