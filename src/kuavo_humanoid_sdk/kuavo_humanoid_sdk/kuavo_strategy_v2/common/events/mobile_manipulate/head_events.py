import time
from typing import Any, Tuple, List
import numpy as np

from kuavo_humanoid_sdk.kuavo_strategy_v2.common.events.base_event import BaseEvent, EventStatus
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.data_type import Pose, Tag, Frame
from kuavo_humanoid_sdk.kuavo_strategy_v2.utils.utils import normalize_angle
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.data_type import Transform3D
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.robot_sdk import RobotSDK

class EventPercep(BaseEvent):
    def __init__(self,
                 robot_sdk: RobotSDK,
                 half_fov,
                 timeout,
                 ):
        """
        初始化感知事件。
        感知事件负责：
        1. 维护最近的tag位置。

        参数：
            timeout (float): 事件超时时间，单位秒。
        """

        super().__init__(
            event_name="EventPercep",
        )
        self.robot_sdk = robot_sdk  # 使用封装的RobotSDK类

        # members
        self.latest_tag: Tag = None  # 儲存最近的tag位置
        self.target: int = None  # 当前目标tag的ID
        self.is_new_tag = False

        # params
        self.timeout = timeout  # 事件超时时间
        self.half_fov = np.deg2rad(half_fov)  # 半视场角，单位弧度

    def reset(self):
        """
        重置事件状态。
        """
        self.latest_tag = None
        self.target = None
        self.is_new_tag = False

    def set_target(self, target, *args, **kwargs):
        """
        设置事件的目标。

        参数：
            target (Any): 目标。
            `*args`: 额外的参数。
            `**kwargs`: 额外的关键字参数。

        返回：
            bool: 如果目标设置成功返回True，否则返回False。
        """
        res = super().set_target(target, *args, **kwargs)
        if res:
            self.latest_tag = None
            self.is_new_tag = False
        return res

    def step(self, tag_up_axis: str = "y") -> EventStatus:
        """
        执行事件的每一步操作。
        """
        # 每次调用step()时，去底层拿最新的tag位置
        target_data = self.robot_sdk.vision.get_data_by_id_from_odom(self.target)
        if target_data is not None:
            tag_pose = target_data["poses"][0]  # 获取第一个tag的位姿
            # 判断是否和上一个tag相似
            pose=Pose(
                        pos=(tag_pose.position.x, tag_pose.position.y, tag_pose.position.z),
                        quat=(tag_pose.orientation.x, tag_pose.orientation.y, tag_pose.orientation.z,
                            tag_pose.orientation.w),
                        frame=Frame.ODOM  # 假设感知到的tag位姿在odom坐标系下
            )
            if self.latest_tag is not None:
                pos_diff = pose.position_l1_norm(self.latest_tag.pose)
                angle_diff = pose.angle(self.latest_tag.pose)
            else:
                pos_diff = 0.0
                angle_diff = 0.0

            if pos_diff < 0.2 and abs(angle_diff) < np.deg2rad(15):
                self.latest_tag = Tag(
                    id=self.target,
                    pose=Pose(
                        pos=(tag_pose.position.x, tag_pose.position.y, tag_pose.position.z),
                        quat=(tag_pose.orientation.x, tag_pose.orientation.y, tag_pose.orientation.z,
                            tag_pose.orientation.w),
                        frame=Frame.ODOM  # 假设感知到的tag位姿在odom坐标系下
                    )
                )
                # FIXME: 如果latest_tag不垂直地面，强行改成垂直。(只保留yaw角)
                old_tag_euler = self.latest_tag.pose.get_euler(degrees=True)
                if tag_up_axis == "y":
                    old_tag_euler[0] = 90
                    old_tag_euler[1] = 0.0
                elif tag_up_axis == "z":
                    old_tag_euler[0] = 0.0
                    old_tag_euler[1] = 0.0
                elif tag_up_axis == "x":
                    old_tag_euler[0] = 0.0
                    old_tag_euler[1] = -90.0

                self.latest_tag.pose = Pose.from_euler(
                    pos=(tag_pose.position.x, tag_pose.position.y, tag_pose.position.z),
                    euler=old_tag_euler,
                    frame=Frame.ODOM,
                    degrees=True
                )

                self.is_new_tag = True
            else:
                self.logger.error(f"Tag出现异常值，跳过此tag。pos_diff {pos_diff}, ang_diff {angle_diff}")

        return self.get_status()

    def close(self):
        """
        关闭事件并重置状态。
        """
        super().close()
        # 结束时候清空一些东西
        self.latest_tag = None
        self.target = None

        self.reset()

    def new_tag_pose_came(self):
        """
        更新最新的tag位姿。

        返回：
            bool: 如果有新的tag位姿返回True，否则返回False。
        """
        if self.is_new_tag:
            self.is_new_tag = False
            return True
        self.is_new_tag = False
        return False

    def get_tag_in_world(self) -> Tag:
        """
        获取感知到的目标位置。

        返回：
            Tag: 感知到的目标位置的Tag对象。
        """
        return self.latest_tag

    def transform_pose_from_tag_to_world(self, tag: Tag, pose: Pose) -> Pose:
        """
        将tag坐标系下的位姿转换到世界坐标系下。

        参数：
            tag (Tag): Tag对象，包含位姿信息。
            pose (Pose): 需要转换的位姿。

        返回：
            Pose: 转换后的Pose对象。
        """
        # 转换stand_pose_in_tag到世界坐标系。注意、需要搞清楚tag的坐标定义和机器人的坐标定义

        # print("🔵 ========== Pose转换调试信息 ==========")
        # print(f"🔵 输入pose (在TAG坐标系中):")
        # print(f"   - 位置: {pose.pos}")
        # print(f"   - 四元数: {pose.quat}")
        # print(f"   - 坐标系: {pose.frame}")

        # print(f"🔵 Tag位姿 (在ODOM坐标系中):")
        # print(f"   - 位置: {tag.pose.pos}")
        # print(f"   - 四元数: {tag.pose.quat}")
        # print(f"   - 坐标系: {tag.pose.frame}")

        transform_tag_to_world = Transform3D(
            trans_pose=tag.pose,
            source_frame=Frame.TAG,  # 源坐标系为Tag坐标系
            target_frame=Frame.ODOM  # 目标坐标系为里程计坐标系
        )

        # print(f"🔵 变换矩阵信息:")
        # print(f"   - 源坐标系: {Frame.TAG}")
        # print(f"   - 目标坐标系: {Frame.ODOM}")
        # print(f"   - 变换pose: {tag.pose.pos}, {tag.pose.quat}")

        stand_pose_in_world = transform_tag_to_world.apply_to_pose(
            pose  # 将站立位置转换到世界坐标系
        )

        # print(f"🔵 转换后pose (在ODOM坐标系中):")
        # print(f"   - 位置: {stand_pose_in_world.pos}")
        # print(f"   - 四元数: {stand_pose_in_world.quat}")
        # print(f"   - 坐标系: {stand_pose_in_world.frame}")

        # import numpy as np
        # from scipy.spatial.transform import Rotation as R

        # tag_quat = tag.pose.quat  # [w, x, y, z]
        # tag_rotation = R.from_quat([tag_quat[1], tag_quat[2], tag_quat[3], tag_quat[0]])  # 转换为scipy格式
        # tag_rotation_matrix = tag_rotation.as_matrix()

        # pose_quat = pose.quat  # [w, x, y, z]
        # pose_rotation = R.from_quat([pose_quat[1], pose_quat[2], pose_quat[3], pose_quat[0]])
        # pose_rotation_matrix = pose_rotation.as_matrix()

        # combined_rotation_matrix = tag_rotation_matrix @ pose_rotation_matrix

        # pose_pos_array = np.array(pose.pos)
        # tag_pos_array = np.array(tag.pose.pos)
        # transformed_pos = tag_rotation_matrix @ pose_pos_array + tag_pos_array

        # print(f"   - TAG旋转矩阵: \n{tag_rotation_matrix}")
        # print(f"   - 输入pose旋转矩阵: \n{pose_rotation_matrix}")
        # print(f"   - 组合旋转矩阵: \n{combined_rotation_matrix}")
        # print(f"   - 数学计算位置: {transformed_pos}")
        # print(f"   - 实际转换位置: {stand_pose_in_world.pos}")
        # print(f"   - 位置差异: {np.array(stand_pose_in_world.pos) - transformed_pos}")
        # print("🔵 ======================================")

        return stand_pose_in_world

    def get_tag_in_base(self) -> Tag:
        """
        获取tag在base_link坐标系下的位姿。

        返回：
            Tag: tag在base_link坐标系下的位姿。
        """
        raise NotImplementedError("get_tag_in_base方法尚未实现")

    def check_in_fov(self, tag: Tag):
        """
        检查目标位置是否在头部视场内。

        参数：
            tag (Tag): 目标的Tag对象。

        返回：
            Tuple[bool, float]: 是否在视场内以及目标方向。
        """
        robot_pose = Pose(
            pos=self.robot_sdk.state.robot_position(),
            quat=self.robot_sdk.state.robot_orientation()
        )
        robot_yaw = robot_pose.get_euler(degrees=False)[2]

        tag_pose = tag.pose

        # 计算目标相对于机器人的位置向量
        dx = tag_pose.pos[0] - robot_pose.pos[0]
        dy = tag_pose.pos[1] - robot_pose.pos[1]
        target_direction = np.arctan2(dy, dx)

        angle_diff = normalize_angle(target_direction - robot_yaw)

        is_in_fov = abs(angle_diff) <= self.half_fov

        self.logger.info(f'初始tag的猜测是否在fov里： {is_in_fov}, 角度差：{angle_diff}')

        return is_in_fov, target_direction

    def _check_target_valid(self, target: Any) -> bool:
        """
        验证目标的有效性。

        参数：
            target (Any): 目标。

        返回：
            bool: 如果目标有效返回True，否则返回False。
        """
        return True

    def _check_failed(self):
        """
        检查事件是否失败。

        返回：
            bool: 如果事件失败返回True，否则返回False。
        """
        # return self.latest_tag is None
        return False

    def _check_success(self):
        """
        检查事件是否成功。

        返回：
            bool: 如果事件成功返回True，否则返回False。
        """
        # 此事件无成功一说
        return False


class EventHeadMoveKeyPoint(BaseEvent):
    def __init__(self,
                 robot_sdk: RobotSDK,
                 timeout,
                 dt: float = 0.7  # 两个step()调用的最小间隔，单位秒
                 ):
        """
        初始化头移动关键点事件。

        参数：
            robot_sdk (RobotSDK): 机器人SDK实例。
            timeout (float): 事件超时时间。
            dt (float): 两个step()调用的最小间隔，单位秒。
        """
        super().__init__(event_name="EventHeadMoveKeyPoint")
        self.robot_sdk = robot_sdk  # 使用封装的RobotSDK类

        # members
        self.cur_head_target_index = 0  # 当前头部目标点索引
        self.target: List[Tuple[float, float]] = []  # 头部目标点列表，格式为[(yaw, pitch), ...]
        self.last_step_time = None

        # params
        self.dt = dt  # 两个step()调用的最小间隔
        self.timeout = timeout  # 事件超时时间，单位秒

    def open(self, *args, **kwargs):
        """
        开始头部移动事件。

        参数：
            `*args`: 额外的参数。
            `**kwargs`: 额外的关键字参数。
        """
        super().open()
        self.last_step_time = time.time()

    def close(self):
        """
        关闭头部移动事件。
        """
        self.cur_head_target_index = 0  # 重置当前头部目标点索引
        self.target = []
        self.last_step_time = None

    def _check_target_valid(self, target: List[Tuple[float, float]]):
        """
        验证头部移动目标的有效性。

        参数：
            target (List[Tuple[float, float]]): 目标列表。

        返回：
            bool: 如果目标有效返回True，否则返回False。
        """
        if not isinstance(target, list):
            print("头部目标点必须是一个列表")
            return False

        return True

    def step(self):
        """
        执行头部移动事件的每一步操作。
        """
        cur_head_target = self.target[self.cur_head_target_index]
        self.robot_sdk.control.control_head(yaw=cur_head_target[0], pitch=cur_head_target[1])
        self.cur_head_target_index += 1

        sleep_time = np.max([0, self.dt - (time.time() - self.last_step_time)])  # 确保每次调用间隔至少为dt
        time.sleep(sleep_time)
        self.last_step_time = time.time()  # 记录上次执行时间

        return self.get_status()

    def _check_failed(self):
        """
        检查头部移动事件是否失败。

        返回：
            bool: 如果事件失败返回True，否则返回False。
        """
        # 此任务未定义失败
        return False

    def _check_success(self):
        """
        检查头部移动事件是否成功。

        返回：
            bool: 如果事件成功返回True，否则返回False。
        """
        """
        检查是否所有的keypoint都被执行
        """
        # 因为头部运动不需要十分精确，所以此处不做闭环位姿检测。
        if self.cur_head_target_index >= len(self.target):
            return True
        return False
