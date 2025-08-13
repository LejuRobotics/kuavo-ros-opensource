import time
import math
from kuavo_humanoid_sdk.kuavo_strategy.kuavo_strategy import KuavoRobotStrategyBase
from kuavo_humanoid_sdk.interfaces.data_types import KuavoPose
from kuavo_humanoid_sdk.interfaces.data_types import KuavoManipulationMpcFrame, KuavoManipulationMpcCtrlMode, KuavoManipulationMpcControlFlow
from kuavo_humanoid_sdk.interfaces.data_types import EndEffectorSide
from kuavo_humanoid_sdk.interfaces.data_types import AprilTagData, HomogeneousMatrix, PoseQuaternion
from kuavo_humanoid_sdk import KuavoRobot, KuavoRobotState, KuavoRobotTools, KuavoRobotVision
from kuavo_humanoid_sdk.common.logger import SDKLogger
from dataclasses import dataclass
from typing import Tuple
import numpy as np
from scipy.spatial.transform import Rotation as R

class KuavoGraspBoxUtils:
    @staticmethod
    def extract_yaw_from_quaternion(quaternion: Tuple[float, float, float, float])-> float:
        """从四元数中提取yaw角
        
        Args:
            quaternion: 四元数 (x, y, z, w)
            
        Returns:
            float: yaw角（弧度）
        """
        if not quaternion or len(quaternion) != 4:
            SDKLogger.error("无法获取有效的四元数")
            return 0.0
            
        # 计算yaw角 (围绕z轴的旋转)
        # 四元数到欧拉角的简化计算，仅提取yaw
        x, y, z, w = quaternion
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return yaw      

    @staticmethod
    def util_euler_to_quat(euler):
        # x, y, z, w
        """将欧拉角转换为四元数"""
        quat = R.from_euler('xyz', euler, degrees=False).as_quat()
        return [quat[0], quat[1], quat[2], quat[3]]
    
    @staticmethod
    def extract_tag_pose(tag_info)-> KuavoPose:
        if (tag_info is not None and isinstance(tag_info, dict) and 
                            'poses' in tag_info and len(tag_info['poses']) > 0):
            tag_pose = KuavoPose(
                position=(tag_info['poses'][0].position.x,
                            tag_info['poses'][0].position.y, 
                            tag_info['poses'][0].position.z),
                    orientation=(tag_info['poses'][0].orientation.x,
                            tag_info['poses'][0].orientation.y,
                            tag_info['poses'][0].orientation.z,
                            tag_info['poses'][0].orientation.w)
                )
            return tag_pose
        else:
            raise ValueError(f"未找到 AprilTag ID 为 {tag_info['id']} 的位姿信息, 无法创建 BoxInfo 实例")

    @staticmethod
    def get_box_world_pose(tag_pose, box_in_tag_xyz):
        """根据目标信息和目标距离计算目标位姿
        p_wa ： tag的世界系下位置
        R_wa ： tag的世界系下旋转矩阵

        注意： box的朝向默认和tag的朝向一致
        xyz轴上可以有平移
        注意看好实际中tag的轴朝向
        """
        pose = KuavoGraspBoxUtils.extract_tag_pose(tag_pose)
        p_wa = np.array([pose.position[0], pose.position[1], pose.position[2]])
        quat_wa = np.array([pose.orientation[0], pose.orientation[1], pose.orientation[2], pose.orientation[3]])  # x,y,z,w
        R_wa = R.from_quat(quat_wa).as_matrix()

        p_at = np.array(box_in_tag_xyz, np.float32)
        tag_z_uni = np.array([0, 0, 1], np.float32)  # tag的z轴朝向
        p_z_w = R_wa @ tag_z_uni  # tag的z轴在世界系下的朝向

        SDKLogger.debug(f'---------------- p_z_w {p_z_w}')
        yaw = math.atan2(p_z_w[1], p_z_w[0])
        yaw += math.pi
        yaw = KuavoGraspBoxUtils.util_cast_to_pi(yaw)

        SDKLogger.debug(f'---------------- yaw {yaw}')

        p_at_w = R_wa @ p_at
        p_wt = p_wa + p_at_w

        return p_wt, yaw
    
    def util_cast_to_pi(yaw):   
        while yaw > math.pi:
            yaw -= 2 * math.pi
        while yaw < -math.pi:
            yaw += 2 * math.pi
        return yaw

@dataclass
class BoxInfo:
    """箱子信息数据类
    
    描述箱子的位置、尺寸和质量信息，用于箱子抓取策略
    
    Attributes:
        pose (KuavoPose): 箱子的位姿信息
        size (Tuple[float, float, float]): 箱子的尺寸 ( 宽, 长, 高) 单位: 米
        mass (float): 箱子的质量 单位: 千克 
    """
    pose: KuavoPose
    size: Tuple[float, float, float] = (0.3, 0.4, 0.22)  # 默认箱子尺寸
    mass: float = 1.5  # 默认箱子质量(kg)

    def __init__(self, pose: KuavoPose = None, size: Tuple[float, float, float] = (0.3, 0.4, 0.22), mass: float = 2.0):
        """初始化箱子信息

        Args:
            pose (KuavoPose, optional): 箱子的位姿信息. 默认为 None
            size (Tuple[float, float, float], optional): 箱子尺寸(长,宽,高). 默认为 (0.3, 0.4, 0.22)
            mass (float, optional): 箱子质量(kg). 默认为 2.0
        """
        self.pose = pose
        self.size = size
        self.mass = mass

    @classmethod
    def from_apriltag(cls, tag_info: dict, xyz_offset: Tuple[float, float, float] = (0.0, 0.0, 0.0), size: Tuple[float, float, float] = (0.4, 0.3, 0.22), mass: float = 2.0):
        """从粘贴在箱子正面的 AprilTag 信息创建 BoxInfo 实例

        Warning:
            必须正确粘贴 AprilTag，AprilTag 朝向请参考: https://chev.me/arucogen/
            
            错误的粘贴方向会导致箱子位姿错乱。

        Args:
            tag_info (dict): 从 :meth:`KuavoRobotVision.get_data_by_id_from_odom` 获取的 AprilTag 信息
            xyz_offset (Tuple[float, float, float], optional): 相对与 AprilTag中心点的偏移量(右手坐标系) \n
                例如：\n
                1. 箱子粘贴在货架上，需要把箱子放下距离货架的高度 -0.5m 则 xyz_offset=(size[1]/2, 0.0, -0.5) \n
                2. 箱子粘贴在箱子正面，为了得到箱子中心点，因此偏移量为箱子宽度的一半 则 xyz_offset=(size[1]/2, 0.0, 0.0) \n
            size (Tuple[float, float, float], optional): 箱子尺寸(长,宽,高). 默认为 (0.4, 0.3, 0.22) \n
            mass (float, optional): 箱子质量(kg). 默认为 2.0

        Returns:
            BoxInfo: 新的 BoxInfo 实例
        """
        # tag 的右手坐标系: z 轴正方向朝向tag面对的方向，xy为平面坐标系
        box_in_tag_xyz = [-xyz_offset[1], xyz_offset[2], -xyz_offset[0]]
        pos_world, yaw_world = KuavoGraspBoxUtils.get_box_world_pose(tag_info, box_in_tag_xyz=box_in_tag_xyz)

        pose = KuavoPose(
            position=pos_world,
            orientation=KuavoGraspBoxUtils.util_euler_to_quat([0, 0, yaw_world])
        )
        return cls(pose, size, mass)
    
class KuavoGraspBox(KuavoRobotStrategyBase):
    """箱子抓取策略类，继承自基础策略类"""
    
    def __init__(self, robot:KuavoRobot, robot_state:KuavoRobotState, robot_tools:KuavoRobotTools, robot_vision:KuavoRobotVision):
        """初始化箱子抓取策略类
        
        Args:
            robot: KuavoRobot实例
            robot_state: KuavoRobotState实例
            robot_tools: KuavoRobotTools实例
            robot_vision: KuavoRobotVision实例
        """
        super().__init__(robot, robot_state, robot_tools, robot_vision)
        
        # 箱子抓取相关的配置参数
        self.search_timeout = 20.0  # 搜索超时时间(秒)
        self.approach_timeout = 30.0  # 接近超时时间(秒)
        self.grasp_height_offset = 0.15  # 抓取高度偏移量(米)
        self.grasp_horizontal_offset = -0.20  # 手指与箱子表面的偏移量，取反为远离箱子 | 取正为靠近箱子
        # 存放头部寻找AprilTag的目标，初始化为异常ID 9999
        self.head_find_target_current_info_pose = AprilTagData(
            id=[9999],  # 异常ID
            size=[0.0],  # 默认尺寸为0
            pose=[PoseQuaternion(
                position=(0.0, 0.0, 0.0),  # 默认零位置
                orientation=(0.0, 0.0, 0.0, 1.0)  # 默认朝向（无旋转）
            )]
        )
        # 新增安全参数
        self.orientation_safety_threshold = math.radians(20)  # 20度安全阈值
        # 新增位置安全参数
        self.workspace_radius = 0.92  # 工作空间半径0.92米
            
    def head_find_target(self, target_info:AprilTagData, max_search_time=None, search_pattern="rotate_head", **kwargs):
        """使用头部旋转寻找AprilTag目标
        
        Args:
            target_info: AprilTag的信息
            max_search_time: 最大搜索时间(秒)，如果为None则使用默认值
            search_pattern: 搜索模式，"rotate_head"或"rotate_body"
            
        Returns:
            bool: 是否成功找到目标
        
        logic:
            1. 判断目标位置是否在机器人FOV(70度视场角)内
            2. 如果不在FOV内且search_pattern为"rotate_body"，先旋转机器人身体朝向目标位置
            3. 无论如何都使用头部搜索模式尝试找到目标
            4. 找到apriltag_data_from_odom之后，马上开始头部追踪
        """
        # 初始目标赋值
        self.head_find_target_current_info_pose = target_info
        
        # 设置搜索超时时间
        if max_search_time is None:
            max_search_time = self.search_timeout
        
        # 获取需要追踪的目标ID
        target_id = target_info.id[0]
        
        if target_id > 9999:
            SDKLogger.error(f"target_id: {target_id} 大于 9999, 无效的AprilTag家族ID")
            return False
        
        # 判断目标位置是否在FOV内
        if len(target_info.pose) > 0:
            target_position = target_info.pose[0].position
            robot_position = self.state.robot_position()
            robot_orientation = self.state.robot_orientation()
            
            # 计算目标相对于机器人的位置向量
            dx = target_position[0] - robot_position[0]
            dy = target_position[1] - robot_position[1]
            
            # 计算目标相对于机器人的角度
            target_angle = math.atan2(dy, dx)
            
            # 获取机器人当前朝向的yaw角
            robot_yaw = KuavoGraspBoxUtils.extract_yaw_from_quaternion(robot_orientation)
            
            # 计算目标与机器人朝向的角度差
            angle_diff = target_angle - robot_yaw
            # 标准化角度到[-pi, pi]
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            
            # 检查是否在FOV内（70度 = 约1.22弧度）
            FOV_HALF_ANGLE = math.radians(35)  # 70度/2 = 35度
            is_in_fov = abs(angle_diff) <= FOV_HALF_ANGLE
            
            SDKLogger.debug(f"目标位置: ({target_position[0]:.2f}, {target_position[1]:.2f})")
            SDKLogger.debug(f"机器人位置: ({robot_position[0]:.2f}, {robot_position[1]:.2f})")
            SDKLogger.debug(f"目标角度: {math.degrees(target_angle):.2f}度")
            SDKLogger.debug(f"机器人朝向: {math.degrees(robot_yaw):.2f}度")
            SDKLogger.debug(f"角度差: {math.degrees(angle_diff):.2f}度")
            SDKLogger.debug(f"是否在FOV内: {is_in_fov}")
            
            # 如果目标不在FOV内且模式允许旋转身体，先旋转机器人身体
            if not is_in_fov and search_pattern == "rotate_body":
                SDKLogger.info("目标超出FOV，调整机器人朝向...")
                # 调整机器人朝向
                SDKLogger.info(f"开始调整 - 机器人位置: {robot_position}")
                SDKLogger.info(f"开始调整 - 目标角度: {math.degrees(target_angle):.2f}度")
                SDKLogger.info(f"开始调整 - 目标角度: {target_angle}")
                self.robot.control_command_pose_world(
                    robot_position[0], # 保持机器人当前x位置
                    robot_position[1], # 保持机器人当前y位置
                    0.0, # 保持当前z高度
                    target_angle # 朝向目标位置 转换为弧度
                )
                
                # 等待机器人旋转到位，使用闭环控制替代固定时间等待
                self._wait_for_orientation(target_angle, max_wait_time=10.0, angle_threshold=0.1)
        
        # 开始搜索计时
        start_time = time.time()
        found_target = False
        
        # 执行头部搜索模式，无论search_pattern是什么
        # 定义头部搜索参数（角度制）
        pitch_angles_deg = [25, -25]  # 两档pitch角度：抬头和低头，角度制
        yaw_angles_deg = [-30, -15, 0, 15, 30]  # 左右扫描的yaw角度，角度制
        
        # 在超时前循环搜索
        while time.time() - start_time < max_search_time and not found_target:
            # 遍历两档pitch角度
            for pitch_deg in pitch_angles_deg:
                # 遍历yaw角度进行左右扫描
                for yaw_deg in yaw_angles_deg:
                    # 将角度转换为弧度
                    yaw_rad = yaw_deg * 0.0174533  # 度转弧度，math.pi/180
                    pitch_rad = pitch_deg * 0.0174533  # 度转弧度
                    
                    # 控制头部旋转（使用弧度）
                    self.robot.control_head(yaw=yaw_rad, pitch=pitch_rad)
                    # 等待头部移动到位
                    time.sleep(0.5)
                    
                    # 检查是否找到目标
                    target_data = self.vision.get_data_by_id_from_odom(target_id)
                    SDKLogger.debug(f"target_data: {target_data}")

                    if (target_data is not None and isinstance(target_data, dict) and 
                        'poses' in target_data and len(target_data['poses']) > 0):
                        SDKLogger.info(f"Target AprilTag {target_id} found!")
                        found_target = True
                        # 开始头部追踪
                        SDKLogger.info("---- 开始头部追踪 ---- ")
                        self.robot.enable_head_tracking(target_id) # self.robot.disable_head_tracking()
                        break
                
                if found_target:
                    break
        
        return found_target

    def _is_orientation_aligned(self, orientation1, orientation2, threshold=0.3):
        """检查两个朝向是否大致一致
        
        Args:
            orientation1: 第一个朝向的四元数
            orientation2: 第二个朝向的四元数
            threshold: 判断为一致的阈值
            
        Returns:
            bool: 朝向是否一致
        """
        # 这里简化实现，实际应用需要进行四元数计算
        # 提取两个朝向的yaw角并比较差异
        yaw1 = KuavoGraspBoxUtils.extract_yaw_from_quaternion(orientation1)
        yaw2 = KuavoGraspBoxUtils.extract_yaw_from_quaternion(orientation2)
        
        # 计算角度差异
        diff = abs(yaw1 - yaw2)
        while diff > math.pi:
            diff -= 2 * math.pi
        
        return abs(diff) < threshold

    def _track_target_with_head(self, target_data):
        """使用头部追踪目标
        
        Args:
            target_data: 目标数据，包含位置信息
        """
        # 从目标数据中提取相对位置
        position = target_data["position"]
        
        # 计算目标相对于机器人的方向
        dx = position[0]
        dy = position[1]
        dz = position[2]
        
        # 计算yaw和pitch角度来指向目标
        # 简单的反正切计算（结果为弧度）
        yaw_rad = math.atan2(dy, dx)
        distance = math.sqrt(dx*dx + dy*dy)
        pitch_rad = math.atan2(dz, distance)
        
        # 限制角度范围（弧度）
        yaw_rad = min(math.radians(80), max(math.radians(-80), yaw_rad))  # 限制在±80度
        pitch_rad = min(math.radians(25), max(math.radians(-25), pitch_rad))  # 限制在±25度
        
        # 控制头部指向目标（输入为弧度）
        self.robot.control_head(yaw=yaw_rad, pitch=pitch_rad)
    
    def walk_approach_target(self, target_id:int, target_distance=0.5, approach_speed=0.15, **kwargs):
        """走路接近 ID 为 `target_id` 的 AprilTag 目标
        
        Args:
            target_id: 目标 AprilTag ID
            target_distance: 与目标的期望距离(米)
            approach_speed: 接近速度(米/秒)
            
        Returns:
            bool: 是否成功接近目标
        """
        approach_success = False
        start_time = time.time()
        target_data = self.vision.get_data_by_id_from_odom(target_id)
        if target_data is None:
            SDKLogger.error(f"未找到目标ID: {target_id} 的目标数据!")
            return False
        target_pose = target_data["poses"][0]
        SDKLogger.debug(f"target_pose in _approach_target: {target_pose}")
        while not approach_success:
            approach_success = self._approach_target(target_pose, target_distance, approach_speed, **kwargs)
            time.sleep(1)
            time_cost = time.time() - start_time
            SDKLogger.debug(f"walking approach target..., time_cost: {time_cost:.2f}秒.")
        return approach_success
    
    def _approach_target(self, target_pose, target_distance=0.5, approach_speed=0.15, **kwargs):
        """根据目标信息和目标距离计算目标位姿
        
        Args:
            target_pose: 目标位姿信息
            target_distance: 与目标的期望距离(米)
            approach_speed: 接近速度(米/秒)
        
        Returns:
            bool: 是否成功接近目标
        """
        p_wa = np.array([target_pose.position.x, target_pose.position.y, target_pose.position.z])
        quat_wa = np.array([target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w]) # x,y,z,w
        R_wa = R.from_quat(quat_wa).as_matrix()
        def get_target_pose_by_distance(p_wa, R_wa, target_distance=0.5):
            """根据目标信息和目标距离计算目标位姿"""
            p_at = np.array([0, 0, target_distance], np.float32)
            p_at_w = R_wa @ p_at
            p_wt = p_wa + p_at_w
            yaw = math.atan2(p_at_w[1], p_at_w[0])
            yaw += math.pi
            while yaw > math.pi:
                yaw -= 2 * math.pi
            while yaw < -math.pi:
                yaw += 2 * math.pi
            return p_wt, yaw
        
        p_wt, angle = get_target_pose_by_distance(p_wa, R_wa, target_distance)
        self.robot.control_command_pose_world(p_wt[0], p_wt[1], 0, angle)
        
        yaw_reached = self._yaw_check(angle)
        pos_reached = self._pos_check(p_wt)
        stance_check = (self.state == 'stance')
        SDKLogger.debug(f"yaw_reached: {yaw_reached}, pos_reached: {pos_reached}, stance_check: {stance_check}")
        return yaw_reached and pos_reached # and stance_check
    
    def _check_target_reachable(self, target_info:BoxInfo) -> bool:
        """检查目标位置是否在机器人手臂可达区域内
        
        Args:
            target_info: 目标信息，包含位置、尺寸等
            
        Returns:
            bool: 目标是否可达
            
        Note:
            此函数为预留接口，待实现以下功能：
            1. 获取机器人当前位姿
            2. 获取机器人手臂工作空间范围
            3. 检查目标位置是否在工作空间内
        """
        # TODO: 实现可达性检查逻辑
        # 1. 获取机器人当前位姿
        # robot_pose = self.state.robot_pose()
        
        # 2. 获取机器人手臂工作空间范围
        # workspace_range = self.robot.get_arm_workspace()
        
        # 3. 检查目标位置是否在工作空间内
        # target_position = target_info.pose.position
        # is_in_workspace = check_position_in_workspace(target_position, workspace_range)
        
        # 临时返回True，等待接口实现后修改
        return True

    # 添加四元数乘法函数
    @staticmethod
    def _quaternion_multiply(q1, q2):
        """
        四元数乘法，用于组合旋转
        q1, q2: 两个四元数 [x, y, z, w]
        """
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
        z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
        
        return [x, y, z, w]
    
    def _quaternion_rotate(self, q, v):
        """
        使用四元数旋转向量
        q: 四元数 [x, y, z, w]
        v: 三维向量 [x, y, z]
        """
        q = np.array(q)
        v = np.array(v)
        q_conj = np.array([-q[0], -q[1], -q[2], q[3]])
        v_quat = np.array([v[0], v[1], v[2], 0.0])
        rotated = KuavoGraspBox._quaternion_multiply(KuavoGraspBox._quaternion_multiply(q, v_quat), q_conj)
        return rotated[:3]
    
    # 坐标转换函数
    def _transform_to_odom(self, pose, transform):
        """将姿态从base_link转换到odom坐标系"""
        # 位置转换（显式转换为numpy数组）
        pos_base = np.array(pose.position)
        transform_pos = np.array(transform.position)
        
        # 使用显式类型转换确保运算正确
        rotated_pos = self._quaternion_rotate(
            np.array(transform.orientation),  # 确保四元数是numpy数组
            pos_base
        )
        pos_odom = transform_pos + rotated_pos
        
        # 姿态转换（显式转换为numpy数组）
        transform_quat = np.array(transform.orientation)
        pose_quat = np.array(pose.orientation)
        rot_odom = KuavoGraspBox._quaternion_multiply(transform_quat, pose_quat)
        
        # 转换为Python原生类型
        return KuavoPose(
            position=tuple(pos_odom.tolist()),
            orientation=rot_odom  # rot_odom 已经是列表，不需要转换
        )

    def _transform_to_base_link(self, pose):
        """将姿态从odom坐标系转换到base_link坐标系
        
        Args:
            pose: KuavoPose类型，表示odom坐标系下的位姿
            
        Returns:
            KuavoPose: base_link坐标系下的位姿
        """
        try:
            # 获取odom到base_link的变换
            odom_to_base = self.tools.get_tf_transform("base_link", "odom")
            
            # 位置转换
            pos_odom = np.array(pose.position)
            odom_pos = np.array(odom_to_base.position)
            
            # 使用四元数旋转
            rotated_pos = self._quaternion_rotate(
                np.array(odom_to_base.orientation),
                pos_odom
            )
            pos_base = rotated_pos + odom_pos
            
            # 姿态转换
            odom_quat = np.array(odom_to_base.orientation)
            pose_quat = np.array(pose.orientation)
            # 注意：这里需要先旋转odom_quat的共轭，再与pose_quat相乘
            odom_quat_conj = np.array([-odom_quat[0], -odom_quat[1], -odom_quat[2], odom_quat[3]])
            rot_base = KuavoGraspBox._quaternion_multiply(odom_quat_conj, pose_quat)
            
            # 返回转换后的位姿
            return KuavoPose(
                position=tuple(pos_base.tolist()),
                orientation=rot_base
            )
        except Exception as e:
            SDKLogger.error(f"坐标转换出错: {str(e)}")
            return None

    @staticmethod
    def _interpolate_poses(start_pose, end_pose, num_points=20):
        """
        在两个笛卡尔空间姿态之间进行三次样条插值
        
        Args:
            start_pose: 起始KuavoPose
            end_pose: 终点KuavoPose
            num_points: 插值点数量
            
        Returns:
            插值后的KuavoPose列表
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

    def _execute_trajectory(self, left_poses, right_poses, total_time=2.0):
        """
        执行左右手轨迹
        
        Args:
            grasp_strategy: 抓取策略对象
            left_poses: 左手KuavoPose列表
            right_poses: 右手KuavoPose列表
            total_time: 总执行时间(秒)
        """

        num_points = min(len(left_poses), len(right_poses))
        time_per_point = total_time / (num_points - 1) if num_points > 1 else total_time
        
        for i in range(num_points):
            self.robot.control_robot_end_effector_pose(
                left_pose=left_poses[i],
                right_pose=right_poses[i],
                frame=KuavoManipulationMpcFrame.WorldFrame,
            )
            if i < num_points - 1:  # 最后一个点不需要延时
                time.sleep(time_per_point)
    
    def _get_target_pose(self, target_info:BoxInfo, traj_type="grasp", **kwargs):
        """获取起始位置和目标位置
        
        Args:
            target_info: 目标信息，包含位置、尺寸等
            traj_type: 轨迹类型：
                - "grasp": 抓取轨迹
                - "lift": 抬起轨迹
                - "place": 放置轨迹

        Returns:
            tuple: (left_pose_init, right_pose_init, left_pose_target, right_pose_target)
        """
        # 计算抓取姿态
        box_position = list(target_info.pose.position)
        box_orientation = list(target_info.pose.orientation)
        SDKLogger.debug(f"原始世界坐标系下的位置: {box_position}")
        SDKLogger.debug(f"原始世界坐标系下的姿态: {box_orientation}")

        box_size = target_info.size    # (length, width, height)
        
        if box_position is None:
            return None, None, None, None
        else:
            # 将四元数转换为yaw角
            qx, qy, qz, qw = box_orientation
            box_yaw = np.arctan2(2*(qw*qz + qx*qy), qw**2 + qx**2 - qy**2 - qz**2)

            # 计算箱子侧面的位置（基于box_yaw旋转）
            half_width = box_size[1] / 2.0
            grasp_height = box_position[2]  # 通常在箱子高度的中间位置抓取

            right_hand_position = ( # left_hand_position
                box_position[0] + half_width * np.sin(box_yaw),
                box_position[1] - half_width * np.cos(box_yaw),
                grasp_height
            )
            left_hand_position = ( # right_hand_position
                box_position[0] - half_width * np.sin(box_yaw),
                box_position[1] + half_width * np.cos(box_yaw),
                grasp_height
            )
            # 基础抓取姿态（只考虑roll和pitch）
            base_left_orientation = [0.06163, -0.70442, -0.06163, 0.70442]  # roll 10度 Pitch -90度
            base_right_orientation = [-0.06163, -0.70442, 0.06163, 0.70442]  # roll -10度 Pitch -90度

             # 创建yaw旋转的四元数
            yaw_quat = [0, 0, np.sin(box_yaw/2), np.cos(box_yaw/2)]
            
            # 合并四元数：结合基础姿态和yaw旋转
            left_grasp_orientation = KuavoGraspBox._quaternion_multiply(yaw_quat, base_left_orientation)
            right_grasp_orientation = KuavoGraspBox._quaternion_multiply(yaw_quat, base_right_orientation)
            
            # 计算基础姿态
            # 1. 贴合箱子侧面左右手的末端位姿
            left_hand_pose = KuavoPose(
                position=left_hand_position,
                orientation=left_grasp_orientation
            )
            right_hand_pose = KuavoPose(
                position=right_hand_position,
                orientation=right_grasp_orientation
            )

            # 2. 预抓取姿态
            left_pre_grasp = KuavoPose(
                position=(
                    left_hand_pose.position[0] + self.grasp_horizontal_offset * np.sin(box_yaw),
                    left_hand_pose.position[1] - self.grasp_horizontal_offset * np.cos(box_yaw),
                    left_hand_pose.position[2]
                ),
                orientation=left_hand_pose.orientation
            )
            
            right_pre_grasp = KuavoPose(
                position=(
                    right_hand_pose.position[0] - self.grasp_horizontal_offset * np.sin(box_yaw),
                    right_hand_pose.position[1] + self.grasp_horizontal_offset * np.cos(box_yaw),
                    right_hand_pose.position[2]
                ),
                orientation=right_hand_pose.orientation
            )

            # 3. 抓取姿态（不只是贴合箱子，抓紧）
            left_grasp = KuavoPose(
                position=(
                    left_hand_pose.position[0], # + 0.05 * np.sin(box_yaw),
                    left_hand_pose.position[1], # - 0.05 * np.cos(box_yaw),
                    left_hand_pose.position[2]
                ),
                orientation=left_hand_pose.orientation
            )
            
            right_grasp = KuavoPose(
                position=(
                    right_hand_pose.position[0], # - 0.05 * np.sin(box_yaw),
                    right_hand_pose.position[1], # + 0.05 * np.cos(box_yaw),
                    right_hand_pose.position[2]
                ),
                orientation=right_hand_pose.orientation
            )

            # 4. 抬起姿态（抓取后向上）
            left_lift = KuavoPose(
                position=(
                    left_grasp.position[0],
                    left_grasp.position[1],
                    left_grasp.position[2] + self.grasp_height_offset
                ),
                orientation=left_grasp.orientation
            )
            
            right_lift = KuavoPose(
                position=(
                    right_grasp.position[0],
                    right_grasp.position[1],
                    right_grasp.position[2] + self.grasp_height_offset
                ),
                orientation=right_grasp.orientation
            )

            # 5. 收臂姿态
            # 定义base_link坐标系下的目标姿态
            # l_arm_base_target_pose = KuavoPose(
            #     position=(0.499, 0.121, 0.370),
            #     orientation=[-0.107, -0.758, 0.063, 0.641]
            # )
            # r_arm_base_target_pose = KuavoPose(
            #     position=(0.499, -0.121, 0.370),
            #     orientation=[-0.026, -0.765, 0.049, 0.642]
            # )
            l_arm_base_target_pose = KuavoPose(
                position=(0.4, half_width, 0.370),
                orientation=[-0.107, -0.758, 0.063, 0.641]
            )
            r_arm_base_target_pose = KuavoPose(
                position=(0.4, -half_width, 0.370),
                orientation=[-0.026, -0.765, 0.049, 0.642]
            )

            # 获取base_link到odom的坐标变换
            base_to_odom = self.tools.get_tf_transform("odom", "base_link")
            
            # 添加调试信息
            SDKLogger.debug(f"base_to_odom position type: {type(base_to_odom.position)}")
            SDKLogger.debug(f"base_to_odom orientation type: {type(base_to_odom.orientation)}")

            # 确保返回的是可迭代对象
            if not isinstance(base_to_odom.position, (list, tuple, np.ndarray)):
                raise ValueError("TF变换位置信息格式错误")
            if not isinstance(base_to_odom.orientation, (list, tuple, np.ndarray)):
                raise ValueError("TF变换姿态信息格式错误")
            
            # 转换目标姿态到odom坐标系
            left_pull = self._transform_to_odom(l_arm_base_target_pose, base_to_odom)
            right_pull = self._transform_to_odom(r_arm_base_target_pose, base_to_odom)

            # 6. 放置姿态（放下箱子）
            l_arm_put_away_base_pose = KuavoPose(
                position=(0.419, half_width, 0.160),
                orientation=[-0.107, -0.758, 0.063, 0.641]
            )
            r_arm_put_away_base_pose = KuavoPose(
                position=(0.419, -half_width, 0.160),
                orientation=[-0.026, -0.765, 0.049, 0.642]
            )
            base_to_odom = self.tools.get_tf_transform("odom", "base_link")
            # 添加调试信息
            SDKLogger.debug(f"base_to_odom position type: {type(base_to_odom.position)}")
            SDKLogger.debug(f"base_to_odom orientation type: {type(base_to_odom.orientation)}")
            # 转换目标姿态到odom坐标系
            left_place = self._transform_to_odom(l_arm_put_away_base_pose, base_to_odom)
            right_place = self._transform_to_odom(r_arm_put_away_base_pose, base_to_odom)

            # 7. 松开手臂
            """
                left_hand_pose.position[0] + self.grasp_horizontal_offset * np.sin(box_yaw),
                left_hand_pose.position[1] - self.grasp_horizontal_offset * np.cos(box_yaw),
                left_hand_pose.position[2]

                right_hand_pose.position[0] - self.grasp_horizontal_offset * np.sin(box_yaw),
                right_hand_pose.position[1] + self.grasp_horizontal_offset * np.cos(box_yaw),
                right_hand_pose.position[2]
            """
            left_release = KuavoPose(
                position=(
                    left_place.position[0] + self.grasp_horizontal_offset * np.sin(box_yaw),
                    left_place.position[1] - self.grasp_horizontal_offset * np.cos(box_yaw),
                    left_place.position[2]
                ),
                orientation=left_place.orientation
            )
            
            right_release = KuavoPose(
                position=(
                    right_place.position[0] - self.grasp_horizontal_offset * np.sin(box_yaw),
                    right_place.position[1] + self.grasp_horizontal_offset * np.cos(box_yaw),
                    right_place.position[2]
                ),
                orientation=right_place.orientation
            )

            # 8.获取一下当前最近更新的位置，用于后续的轨迹计算
            left_current_position, left_current_orientation = self.tools.get_link_position(link_name="zarm_l7_end_effector", reference_frame="odom")
            right_current_position, right_current_orientation = self.tools.get_link_position(link_name="zarm_r7_end_effector", reference_frame="odom")

            left_current = KuavoPose(
                position=left_current_position,
                orientation=left_current_orientation
            )
            right_current = KuavoPose(
                position=right_current_position,
                orientation=right_current_orientation
            )

            # 根据轨迹类型返回对应的姿态
            if traj_type == "grasp":
                return left_pre_grasp, right_pre_grasp, left_grasp, right_grasp
            elif traj_type == "lift":
                return left_grasp, right_grasp, left_lift, right_lift
            elif traj_type == "pull":
                return left_current, right_current, left_pull, right_pull
            elif traj_type == "place":
                return left_current, right_current, left_place, right_place
            elif traj_type == "release":
                return left_place, right_place, left_release, right_release
            else:
                SDKLogger.error(f"未知的轨迹类型: {traj_type}")
                return None, None, None, None

    def _check_orientation_safety(self, target_orientation, threshold=None):
        """检查目标朝向与机器人当前朝向的安全性"""
        if threshold is None:
            threshold = self.orientation_safety_threshold
            
        # 获取当前机器人朝向
        current_orientation = self.state.robot_orientation()
        current_yaw = KuavoGraspBoxUtils.extract_yaw_from_quaternion(current_orientation)
        
        # 提取目标朝向的yaw角
        target_yaw = KuavoGraspBoxUtils.extract_yaw_from_quaternion(target_orientation)
        
        # 计算角度差
        angle_diff = abs(target_yaw - current_yaw)
        angle_diff = min(2*math.pi - angle_diff, angle_diff)  # 取最小角度差
        
        SDKLogger.debug(f"[安全检查] 当前朝向: {math.degrees(current_yaw):.1f}°, 目标朝向: {math.degrees(target_yaw):.1f}°, 差异: {math.degrees(angle_diff):.1f}°")
        
        if angle_diff > threshold:
            SDKLogger.error(f"❌ 方向偏差超过安全阈值({math.degrees(threshold):.1f}°)，终止操作！")
            return False
        return True

    def _check_position_safety(self, target_info: BoxInfo) -> bool:
        """检查目标位置是否在工作空间内"""
        try:
            # 将目标位置转换到base_link坐标系
            target_pose_base = self._transform_to_base_link(target_info.pose)
            
            # 获取左右臂关节位置（需要解包位置和姿态）
            l_pos, _ = self.tools.get_link_position("zarm_l1_link")  # 解包位置和姿态
            r_pos, _ = self.tools.get_link_position("zarm_r1_link")  # 只取位置部分
            
            # 转换为numpy数组
            target_pos = np.array(target_pose_base.position)
            l_joint_pos = np.array(l_pos)
            r_joint_pos = np.array(r_pos)

            # 计算水平距离（只取x,y坐标）
            l_distance = np.linalg.norm(target_pos[:2] - l_joint_pos[:2])
            r_distance = np.linalg.norm(target_pos[:2] - r_joint_pos[:2])

            SDKLogger.debug(f"[位置安全检查] 左臂距离: {l_distance:.2f}m, 右臂距离: {r_distance:.2f}m, 安全阈值: {self.workspace_radius:.2f}m")
            
            # 检查是否在安全范围内
            if l_distance > self.workspace_radius or r_distance > self.workspace_radius:
                SDKLogger.error(f"❌ 目标位置超出工作空间范围({self.workspace_radius}m)")
                return False
            return True
        except Exception as e:
            SDKLogger.error(f"位置安全检查出错: {str(e)}")
            return False

    def _check_height_safety(self, target_info: BoxInfo) -> bool:
        """检查目标位置的高度是否在机器人工作范围内
        
        Args:
            target_info: 目标信息，包含位置、尺寸等
            
        Returns:
            bool: 高度是否在安全范围内
        """
        target_height = target_info.pose.position[2]
        min_height = 0.5  # 最小工作高度
        max_height = 1.8  # 最大工作高度
        
        SDKLogger.debug(f"[高度安全检查] 目标高度: {target_height:.2f}m, 工作范围: {min_height:.2f}m - {max_height:.2f}m")
        
        if target_height < min_height or target_height > max_height:
            SDKLogger.error(f"❌ 目标高度 {target_height:.2f}m 超出工作范围({min_height:.2f}m - {max_height:.2f}m)，终止操作！")
            return False
        return True

    def arm_move_to_target(self, target_info:BoxInfo, arm_mode="manipulation_mpc", **kwargs):
        """添加安全保护检查"""
        # 统一的安全检查
        if not self._check_orientation_safety(target_info.pose.orientation):
            return False
        if not self._check_position_safety(target_info):
            return False
        if not self._check_height_safety(target_info):
            return False
        
        # 原有代码保持不变
        if arm_mode == "manipulation_mpc":
            self.robot.set_manipulation_mpc_mode(KuavoManipulationMpcCtrlMode.BaseArm)
        else:
            self.robot.set_fixed_arm_mode()

        # 获取预抓取轨迹
        left_pose_init, right_pose_init, left_pose_target, right_pose_target = self._get_target_pose(target_info, traj_type="grasp")
        if left_pose_init is None:
            return False

        # 控制手臂移动到预抓取位置
        if not self.robot.control_robot_end_effector_pose(
            left_pose_init,
            right_pose_init,
            KuavoManipulationMpcFrame.WorldFrame
        ):
            return False
        
        SDKLogger.debug("执行预抓取姿态到抓取姿态的轨迹...")
        left_traj_grasp = KuavoGraspBox._interpolate_poses(left_pose_init, left_pose_target)
        right_traj_grasp = KuavoGraspBox._interpolate_poses(right_pose_init, right_pose_target)
        self._execute_trajectory(left_traj_grasp, right_traj_grasp)       
        return True

    def _check_box_lifting_status(self, target_info:BoxInfo) -> bool:
        """检查箱子是否成功抬起
        
        Args:
            target_info: 目标信息，包含位置、尺寸等
            
        Returns:
            bool: 是否成功抬起箱子
            
        Note:
            此函数为预留接口，待实现以下功能：
            1. 获取手部力反馈数据
            2. 根据箱子重量和力反馈判断是否成功抬起
            3. 检查力反馈是否稳定
        """
        # TODO: 实现力反馈检测逻辑
        # 1. 获取手部力反馈数据
        # left_force = self.state.get_end_effector_force(EndEffectorSide.Left)
        # right_force = self.state.get_end_effector_force(EndEffectorSide.Right)
        
        # 2. 根据箱子重量和力反馈判断
        # expected_force = target_info.mass * 9.8
        # actual_force = calculate_total_force(left_force, right_force)
        
        # 3. 检查力反馈稳定性
        # force_stable = check_force_stability()
        
        # 临时返回True，等待接口实现后修改
        return True

    def arm_transport_target_up(self, target_info:BoxInfo, arm_mode="manipulation_mpc", sim_mode=False):
        """添加安全检查"""
        # 统一的安全检查
        if not self._check_orientation_safety(target_info.pose.orientation):
            return False
        if not self._check_position_safety(target_info):
            return False
        if not self._check_height_safety(target_info):
            return False
        
        # 原有代码保持不变
        if arm_mode == "manipulation_mpc":
            self.robot.set_manipulation_mpc_mode(KuavoManipulationMpcCtrlMode.BaseArm)
        else:
            self.robot.set_fixed_arm_mode()

        # 智能计算夹持力参数
        g = 9.8  # 重力加速度
        force_ratio = 0.34   # 经验系数（根据1.5kg对应5N得出：5/(1.5*9.8)≈0.34）
        
        # 计算基础Z向力（考虑安全系数和经验比例）
        force_z_base = target_info.mass * g * force_ratio
        force_z = -abs(force_z_base)  # z方向负值表示向上施加力
        
        # 侧向夹持力计算（基于Z向力的比例）
        lateral_ratio = 3.0  # 侧向力与垂直力的比例（根据15N/5N=3得出）
        lateral_force = abs(force_z) * lateral_ratio
        
        # 判断是否为仿真模式
        if sim_mode:
            force_z = 0
            left_force = 0
            right_force = 0
        else:
            left_force = 15   # 左手侧向力（正值为夹紧方向）
            right_force = -15 # 右手侧向力（负值为夹紧方向）
        
        # 提起箱子调用末端力（使用计算得到的力）
        self.robot.control_hand_wrench(
            [0, left_force,  force_z, 0, 0, 0],   # 左手力
            [0, right_force, force_z, 0, 0, 0]    # 右手力
        )
        time.sleep(2)

        # 获取抬起轨迹
        left_pose_init, right_pose_init, left_pose_target, right_pose_target = self._get_target_pose(target_info, traj_type="lift")
        if left_pose_init is None:
            return False

        # 执行抬起轨迹
        SDKLogger.debug("执行抬起轨迹")
        left_traj_lift = KuavoGraspBox._interpolate_poses(left_pose_init, left_pose_target)
        right_traj_lift = KuavoGraspBox._interpolate_poses(right_pose_init, right_pose_target)
        self._execute_trajectory(left_traj_lift, right_traj_lift)
        time.sleep(2)
        
        # 暂时关闭
        self.robot.manipulation_mpc_reset()
        time.sleep(2)
        
        # 机器人往后退
        self.robot.stance()
        time.sleep(2)
        self.robot.control_command_pose(-0.5, 0, 0, 0)
        time.sleep(5) # 等待机器人执行到位 

        # 执行收臂轨迹
        SDKLogger.debug("执行收臂轨迹")
        left_pose_init, right_pose_init, left_pose_target, right_pose_target = self._get_target_pose(target_info, traj_type="pull")
        if left_pose_init is None:
            return False
        self.robot.set_manipulation_mpc_control_flow(KuavoManipulationMpcControlFlow.DirectToWbc)
        self.robot.set_manipulation_mpc_mode(KuavoManipulationMpcCtrlMode.ArmOnly)
        left_traj_pull = KuavoGraspBox._interpolate_poses(left_pose_init, left_pose_target)    # left_pose_init left_pose_target
        right_traj_pull = KuavoGraspBox._interpolate_poses(right_pose_init, right_pose_target) #  right_pose_init right_pose_target
        self._execute_trajectory(left_traj_pull, right_traj_pull)

        if not self._check_box_lifting_status(target_info):
            return False

        # 暂时关闭
        self.robot.manipulation_mpc_reset()
        time.sleep(5)

        return True

    
    def _arrive_pose(self, target_position: list, target_yaw: float, timeout: float = 20.0) -> bool:
        """控制机器人移动到指定位姿并等待到达
        
        Args:
            target_position: 目标位置 [x, y, z]
            target_yaw: 目标朝向角度（弧度）
            timeout: 等待超时时间（秒）
            
        Returns:
            bool: 是否成功到达目标位姿
        """
        # 控制机器人移动到目标位姿
        self.robot.control_command_pose_world(
            target_position[0],  # x
            target_position[1],  # y
            0,                   # z (保持当前高度)
            target_yaw          # 目标朝向
        )
        
        # 等待机器人到达目标位姿
        start_time = time.time()
        rate_hz = 10  # 检查频率
        wait_interval = 1.0 / rate_hz
        
        while time.time() - start_time < timeout:
            # 检查位置和朝向是否到位
            pos_reached = self._pos_check(target_position)
            yaw_reached = self._yaw_check(target_yaw)
            
            if pos_reached and yaw_reached:
                SDKLogger.debug("机器人已到达目标位姿!")
                return True
            
            # 短暂等待再次检查
            time.sleep(wait_interval)
        
        # 超时
        SDKLogger.warn(f"等待机器人到达目标位姿超时!")
        return False
    
    def walk_to_pose(self, target_pose:KuavoPose, target_distance=0.5, approach_speed=0.15, timeout=10.0, **kwargs):
        """让机器人走到指定的位姿
        
        Args:
            target_pose: 目标位姿
            target_distance: 与目标的期望距离(米)
            approach_speed: 接近速度(米/秒)
            timeout: 超时时间(秒)
        Returns:
            bool: 是否成功到达目标位姿
        """
        # 获取目标位置和朝向
        target_position = target_pose.position
        target_orientation = target_pose.orientation
        SDKLogger.debug(f"target_position: {target_position}, target_orientation: {target_orientation}")
        
        # 从四元数中提取yaw角
        target_yaw = KuavoGraspBoxUtils.extract_yaw_from_quaternion(target_orientation)
        SDKLogger.debug(f"target_yaw: {target_yaw}")
        
        # 计算偏移后的位置
        # 根据目标朝向计算偏移方向
        offset_x = -target_distance * math.cos(target_yaw)  # 负号是因为要远离目标
        offset_y = -target_distance * math.sin(target_yaw)
        
        # 计算新的目标位置
        new_target_position = [
            target_position[0] + offset_x,
            target_position[1] + offset_y,
            target_position[2]
        ]
        
        SDKLogger.info(f"开始移动到目标位姿:")
        SDKLogger.info(f"原始目标位置: ({target_position[0]:.2f}, {target_position[1]:.2f}, {target_position[2]:.2f})")
        SDKLogger.info(f"偏移后位置: ({new_target_position[0]:.2f}, {new_target_position[1]:.2f}, {new_target_position[2]:.2f})")
        SDKLogger.info(f"目标朝向: {math.degrees(target_yaw):.2f}度")
        
        if not self._arrive_pose(
            new_target_position,
            target_yaw,
            timeout
        ):
            return False
        
        return True

    def arm_transport_target_down(self, target_info:BoxInfo, arm_mode="manipulation_mpc"):
        """添加安全检查"""
        # 统一的安全检查
        if not self._check_orientation_safety(target_info.pose.orientation):
            return False
        if not self._check_position_safety(target_info):  # 添加位置检查
            return False
        if not self._check_height_safety(target_info):
            return False
        
        # 原有代码保持不变
        if arm_mode == "manipulation_mpc":
            self.robot.set_manipulation_mpc_mode(KuavoManipulationMpcCtrlMode.BaseArm)
        else:
            self.robot.set_fixed_arm_mode()
            
        # 获取放置轨迹
        left_pose_init, right_pose_init, left_pose_target, right_pose_target = self._get_target_pose(target_info, traj_type="place")
        if left_pose_init is None:
            return False

        # 执行放置轨迹
        left_traj_place = KuavoGraspBox._interpolate_poses(left_pose_init, left_pose_target)
        right_traj_place = KuavoGraspBox._interpolate_poses(right_pose_init, right_pose_target)
        self._execute_trajectory(left_traj_place, right_traj_place)
        
        time.sleep(2)
        
        # 箱子已经放到平面
        self.robot.control_hand_wrench([0,0,0,0,0,0],
                                       [0,0,0,0,0,0])
        time.sleep(2)

        # 放开箱子
        left_pose_init, right_pose_init, left_pose_target, right_pose_target = self._get_target_pose(target_info, traj_type="release")
        if left_pose_init is None:
            return False        

        # 执行放开轨迹
        left_traj_place = KuavoGraspBox._interpolate_poses(left_pose_init, left_pose_target)
        right_traj_place = KuavoGraspBox._interpolate_poses(right_pose_init, right_pose_target)
        self._execute_trajectory(left_traj_place, right_traj_place)
        time.sleep(2)
        
        self.robot.manipulation_mpc_reset()
        time.sleep(2)

        # 机器人往后退 0.5m
        self.robot.stance()
        time.sleep(2)
        self.robot.control_command_pose(-0.5, 0, 0, 0)
        time.sleep(5)

        # 手臂归中
        self.robot.disable_head_tracking()
        self.robot.stance()
        time.sleep(0.5)
        self.robot.arm_reset() 
        time.sleep(2)

        return True

    def _wait_for_orientation(self, target_angle, max_wait_time=10.0, angle_threshold=0.1):
        """等待机器人旋转到指定朝向
        
        Args:
            target_angle: 目标朝向角度（弧度）
            max_wait_time: 最大等待时间（秒）
            angle_threshold: 角度阈值（弧度），小于此阈值认为已到位
            
        Returns:
            bool: 是否成功到达目标朝向
        """
        start_time = time.time()
        rate_hz = 10  # 检查频率
        wait_interval = 1.0 / rate_hz
        
        SDKLogger.info(f"等待机器人旋转到位，目标角度: {math.degrees(target_angle):.2f}度")
        
        while time.time() - start_time < max_wait_time:
            if self._yaw_check(target_angle, angle_threshold):
                return True
            
            # 短暂等待再次检查
            time.sleep(wait_interval)
        
        # 超时
        SDKLogger.warn(f"等待机器人旋转到位超时! 已经等待了 {max_wait_time:.2f}秒")
        return False

    def _yaw_check(self, yaw_angle_target, angle_threshold=0.1):
        """检查机器人当前朝向与目标朝向的差异
        
        Args:
            yaw_angle_target: 目标朝向角度（弧度）
            angle_threshold: 角度阈值（弧度），小于此阈值认为已到位
            
        Returns:
            bool: 是否成功到达目标朝向
        """
        # 获取当前机器人朝向
        current_orientation = self.state.robot_orientation()
        current_yaw = KuavoGraspBoxUtils.extract_yaw_from_quaternion(current_orientation)
        
        # 计算角度差
        angle_diff = yaw_angle_target - current_yaw
        # 标准化到[-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # 输出当前状态
        SDKLogger.debug(f"当前朝向: {math.degrees(current_yaw):.2f}度, 目标朝向: {math.degrees(yaw_angle_target):.2f}度, 差值: {math.degrees(abs(angle_diff)):.2f}度")
        
        # 检查是否已到位
        if abs(angle_diff) < angle_threshold:
            SDKLogger.debug(f"机器人已旋转到位!")
            return True
        else:
            return False
        
    def _pos_check(self, pos_target, pos_threshold=0.2):
        """检查机器人当前位置(x, y)与目标位置的差异
        
        Args:
            pos_target: 目标位置
            pos_threshold: 位置阈值（米），小于此阈值认为已到位
        """
        current_pos = self.state.robot_position()
        if not current_pos or len(current_pos) < 2:
            SDKLogger.error("无法获取有效的机器人当前位置")
            return False
            
        # SDKLogger.debug(f"current_pos: {current_pos}, pos_target: {pos_target}")
        pos_diff = np.linalg.norm(np.array(pos_target[:2]) - np.array(current_pos[:2]))
        SDKLogger.debug(f"当前位置(x,y): ({current_pos[0]:.2f}, {current_pos[1]:.2f}), 目标位置(x,y): ({pos_target[0]:.2f}, {pos_target[1]:.2f}), 距离: {pos_diff:.2f}米")
        if pos_diff < pos_threshold:
            SDKLogger.debug(f"机器人已到达目标位置!")
            return True
        else:
            return False