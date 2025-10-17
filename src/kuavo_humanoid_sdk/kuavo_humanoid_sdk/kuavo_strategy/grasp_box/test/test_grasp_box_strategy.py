import pytest
import time
from unittest.mock import MagicMock, patch
from kuavo_humanoid_sdk.kuavo_strategy.grasp_box.grasp_box_strategy import KuavoGraspBox, BoxInfo
from kuavo_humanoid_sdk.interfaces.data_types import AprilTagData, PoseQuaternion, KuavoPose
import math

class TestKuavoGraspBox:
    
    @pytest.fixture
    def setup_grasp_box(self):
        # 创建模拟对象
        mock_robot = MagicMock()
        mock_robot_state = MagicMock()
        mock_robot_tools = MagicMock()
        mock_robot_vision = MagicMock()
        
        # 创建策略实例
        grasp_box = KuavoGraspBox(
            robot=mock_robot,
            robot_state=mock_robot_state,
            robot_tools=mock_robot_tools,
            robot_vision=mock_robot_vision
        )
        
        return grasp_box, mock_robot, mock_robot_state, mock_robot_tools, mock_robot_vision
    
    def test_walk_approach_target_success(self, setup_grasp_box):
        """测试成功接近目标"""
        grasp_box, mock_robot, mock_robot_state, mock_robot_tools, mock_robot_vision = setup_grasp_box
        
        # 创建测试用的AprilTag数据
        test_tag = AprilTagData(
            id=[123],
            size=[0.1],
            pose=[PoseQuaternion(
                position=(1.0, 0.0, 0.0),
                orientation=(0.0, 0.0, 0.0, 1.0)
            )]
        )
        
        # 模拟vision返回的数据
        mock_robot_vision.get_data_by_id_from_odom.return_value = {
            "poses": [{
                "position": (1.0, 0.0, 0.0),
                "orientation": (0.0, 0.0, 0.0, 1.0)
            }]
        }
        
        # 模拟_approach_target方法
        grasp_box._approach_target = MagicMock(return_value=True)
        
        # 调用方法
        result = grasp_box.walk_approach_target(
            target_id=test_tag.id[0],
            target_distance=0.5,
            approach_speed=0.15
        )
        
        # 验证结果
        assert result is True
        grasp_box._approach_target.assert_called_once()
    
    def test_walk_approach_target_no_data(self, setup_grasp_box):
        """测试目标数据不存在的情况"""
        grasp_box, mock_robot, mock_robot_state, mock_robot_tools, mock_robot_vision = setup_grasp_box
        
        # 创建测试用的AprilTag数据
        test_tag = AprilTagData(
            id=[456],
            size=[0.2],
            pose=[PoseQuaternion(
                position=(0.0, 1.0, 0.0),
                orientation=(0.0, 0.0, 0.0, 1.0)
            )]
        )
        
        # 模拟vision返回None
        mock_robot_vision.get_data_by_id_from_odom.return_value = None
        
        # 将 _approach_target 替换为 MagicMock 对象
        grasp_box._approach_target = MagicMock()
        
        # 调用方法
        result = grasp_box.walk_approach_target(
            target_id=test_tag.id[0],
            target_distance=0.8,
            approach_speed=0.1
        )
        
        # 验证结果
        assert result is False
        
        # 验证 _approach_target 没有被调用
        grasp_box._approach_target.assert_not_called()

    def test_arm_move_to_target_success(self, setup_grasp_box):
        """测试手臂移动到目标位置成功"""
        grasp_box, mock_robot, mock_robot_state, mock_robot_tools, mock_robot_vision = setup_grasp_box
        
        # 创建测试用的目标信息
        test_box = BoxInfo(
            pose=KuavoPose(
                position=(1.0, 0.0, 0.8),  # 调整高度到0.8m
                orientation=(0.0, 0.0, 0.0, 1.0)
            ),
            size=(0.1, 0.1, 0.1),
            mass=1.0
        )

        # 添加高度安全检查的mock
        grasp_box._check_height_safety = MagicMock(return_value=True)

        # 模拟_get_target_pose方法
        grasp_box._get_target_pose = MagicMock(return_value=(
            KuavoPose(position=(0.0, 0.0, 0.0),orientation=(0.0, 0.0, 0.0, 1.0)),
            KuavoPose(position=(0.0, 0.0, 0.0),orientation=(0.0, 0.0, 0.0, 1.0)),
            KuavoPose(position=(0.0, 0.0, 0.0),orientation=(0.0, 0.0, 0.0, 1.0)),
            KuavoPose(position=(0.0, 0.0, 0.0),orientation=(0.0, 0.0, 0.0, 1.0))
        ))

        # 模拟安全检查方法
        grasp_box._check_orientation_safety = MagicMock(return_value=True)
        grasp_box._check_position_safety = MagicMock(return_value=True)

        # 模拟robot方法
        mock_robot.control_robot_end_effector_pose = MagicMock(return_value=True)
        mock_robot.set_manipulation_mpc_mode = MagicMock()
        mock_robot.set_manipulation_mpc_control_flow = MagicMock()
        grasp_box._check_target_reachable = MagicMock(return_value=True)
        
        # 调用方法
        result = grasp_box.arm_move_to_target(
            target_info=test_box,
            arm_mode="manipulation_mpc"
        )   

        # 验证结果
        assert result is True
        assert mock_robot.control_robot_end_effector_pose.call_count == 21
        assert mock_robot.set_manipulation_mpc_mode.call_count == 2
        mock_robot.set_manipulation_mpc_control_flow.assert_called_once()
        grasp_box._check_orientation_safety.assert_called_once()
        grasp_box._check_position_safety.assert_called_once()

    def test_arm_move_to_target_failure(self, setup_grasp_box):
        """测试手臂移动到目标位置失败"""
        grasp_box, mock_robot, mock_robot_state, mock_robot_tools, mock_robot_vision = setup_grasp_box
        
        # 创建测试用的目标信息
        test_box = BoxInfo(
            pose=KuavoPose(
                position=(1.0, 0.0, 0.8),  # 调整高度到0.8m
                orientation=(0.0, 0.0, 0.0, 1.0)
            ),
            size=(0.1, 0.1, 0.1),
            mass=1.0
        )

        # 添加高度安全检查的mock
        grasp_box._check_height_safety = MagicMock(return_value=True)

        # 模拟安全检查方法返回False
        grasp_box._check_orientation_safety = MagicMock(return_value=False)
        grasp_box._check_position_safety = MagicMock(return_value=True)

        # 模拟_get_target_pose返回None
        grasp_box._get_target_pose = MagicMock(return_value=(None, None, None, None))

        # 模拟robot方法
        mock_robot.control_robot_end_effector_pose = MagicMock(return_value=False)
        mock_robot.set_manipulation_mpc_mode = MagicMock()
        mock_robot.set_manipulation_mpc_control_flow = MagicMock()

        # 调用方法
        result = grasp_box.arm_move_to_target(
            target_info=test_box,
            arm_mode="manipulation_mpc"
        )
        
        # 验证结果
        assert result is False
        mock_robot.control_robot_end_effector_pose.assert_not_called()
        mock_robot.set_manipulation_mpc_mode.assert_not_called()
        mock_robot.set_manipulation_mpc_control_flow.assert_not_called()
        grasp_box._check_orientation_safety.assert_called_once()
        grasp_box._check_position_safety.assert_not_called()

    def test_arm_transport_target_up_success(self, setup_grasp_box):
        """测试抓取成功"""
        grasp_box, mock_robot, mock_robot_state, mock_robot_tools, mock_robot_vision = setup_grasp_box
        
        # 创建测试用的目标信息
        test_box = BoxInfo(
            pose=KuavoPose(
                position=(1.0, 0.0, 0.8),  # 调整高度到0.8m
                orientation=(0.0, 0.0, 0.0, 1.0)
            ),
            size=(0.1, 0.1, 0.1),
            mass=1.0
        )

        # 添加安全检查的mock
        grasp_box._check_orientation_safety = MagicMock(return_value=True)
        grasp_box._check_position_safety = MagicMock(return_value=True)
        grasp_box._check_height_safety = MagicMock(return_value=True)

        # 模拟_get_target_pose方法
        grasp_box._get_target_pose = MagicMock(return_value=(
            KuavoPose(position=(0.0, 0.0, 0.0),orientation=(0.0, 0.0, 0.0, 1.0)),
            KuavoPose(position=(0.0, 0.0, 0.0),orientation=(0.0, 0.0, 0.0, 1.0)),
            KuavoPose(position=(0.0, 0.0, 0.0),orientation=(0.0, 0.0, 0.0, 1.0)),
            KuavoPose(position=(0.0, 0.0, 0.0),orientation=(0.0, 0.0, 0.0, 1.0))
        ))

        # 模拟control_robot_end_effector_pose方法
        mock_robot.control_robot_end_effector_pose = MagicMock(return_value=True)
        mock_robot.set_manipulation_mpc_mode = MagicMock()
        mock_robot.set_manipulation_mpc_control_flow = MagicMock()

        # 调用方法
        result = grasp_box.arm_transport_target_up(
            target_info=test_box,
            arm_mode="manipulation_mpc"
        )

        # 验证结果
        assert result is True
        assert mock_robot.control_robot_end_effector_pose.call_count == 40
        assert mock_robot.set_manipulation_mpc_mode.call_count == 2
        mock_robot.set_manipulation_mpc_control_flow.assert_called_once()
        
    def test_arm_transport_target_up_failure(self, setup_grasp_box):
        """测试抓取失败"""
        grasp_box, mock_robot, mock_robot_state, mock_robot_tools, mock_robot_vision = setup_grasp_box
        
        # 创建测试用的目标信息
        test_box = BoxInfo(
            pose=KuavoPose(
                position=(1.0, 0.0, 0.8),  # 调整高度到0.8m
                orientation=(0.0, 0.0, 0.0, 1.0)
            ),
            size=(0.1, 0.1, 0.1),
            mass=1.0
        )

        # 添加安全检查的mock
        grasp_box._check_orientation_safety = MagicMock(return_value=True)
        grasp_box._check_position_safety = MagicMock(return_value=True)
        grasp_box._check_height_safety = MagicMock(return_value=True)

        # 模拟_get_target_pose方法
        grasp_box._get_target_pose = MagicMock(return_value=(None,None,None,None))

        # 模拟control_arm_move_to_target方法
        grasp_box.control_arm_move_to_target = MagicMock(return_value=False)

        # 调用方法
        result = grasp_box.arm_transport_target_up(
            target_info=test_box,
            arm_mode="manipulation_mpc"
        )   

        # 验证结果
        assert result is False
        grasp_box.control_arm_move_to_target.assert_not_called()

    def test_walk_to_pose_success(self, setup_grasp_box):
        """测试成功接近目标"""
        grasp_box, mock_robot, mock_robot_state, mock_robot_tools, mock_robot_vision = setup_grasp_box
        
        # 创建测试用的BoxInfo数据
        test_box = BoxInfo(
            pose=KuavoPose(
                position=(1.0, 0.0, 0.0),
                orientation=(0.0, 0.0, 0.0, 1.0)
            ),
            size=(0.1, 0.1, 0.1),
            mass=1.0
        )
        
        # 模拟_arrive_pose方法
        grasp_box._arrive_pose = MagicMock(return_value=True)
        
        # 调用方法
        result = grasp_box.walk_to_pose(
            target_info=test_box,
            target_distance=0.5,
            approach_speed=0.15
        )
        
        # 验证结果
        assert result is True
        grasp_box._arrive_pose.assert_called_once()
    
    def test_walk_to_pose_failure(self, setup_grasp_box):
        """测试目标数据不存在的情况"""
        grasp_box, mock_robot, mock_robot_state, mock_robot_tools, mock_robot_vision = setup_grasp_box
        
        # 创建测试用的BoxInfo数据
        test_box = BoxInfo(
            pose=KuavoPose(
                position=(1.0, 0.0, 0.0),
                orientation=(0.0, 0.0, 0.0, 1.0)
            ),
            size=(0.1, 0.1, 0.1),
            mass=1.0
        )
        
        # 模拟_arrive_pose方法
        grasp_box._arrive_pose = MagicMock(return_value=False)
        
        # 调用方法
        result = grasp_box.walk_to_pose(
            target_info=test_box,
            target_distance=0.8,
            approach_speed=0.1
        )
        
        # 验证结果
        assert result is False
        grasp_box._arrive_pose.assert_called_once()  # 验证_arrive_pose被调用了一次

    def test_arm_transport_target_down_success(self, setup_grasp_box):
        """测试放下成功"""
        grasp_box, mock_robot, mock_robot_state, mock_robot_tools, mock_robot_vision = setup_grasp_box
        
        # 创建测试用的目标信息
        test_box = BoxInfo(
            pose=KuavoPose(
                position=(1.0, 0.0, 0.8),  # 调整高度到0.8m
                orientation=(0.0, 0.0, 0.0, 1.0)
            ),
            size=(0.1, 0.1, 0.1),
            mass=1.0
        )   

        # 添加安全检查的mock
        grasp_box._check_orientation_safety = MagicMock(return_value=True)
        grasp_box._check_position_safety = MagicMock(return_value=True)
        grasp_box._check_height_safety = MagicMock(return_value=True)

        # 模拟_get_target_pose方法
        grasp_box._get_target_pose = MagicMock(side_effect=[
            # 第一次调用返回放置轨迹的位姿
            (
                KuavoPose(position=(0.0, 0.0, 0.0),orientation=(0.0, 0.0, 0.0, 1.0)),
                KuavoPose(position=(0.0, 0.0, 0.0),orientation=(0.0, 0.0, 0.0, 1.0)),
                KuavoPose(position=(0.0, 0.0, 0.0),orientation=(0.0, 0.0, 0.0, 1.0)),
                KuavoPose(position=(0.0, 0.0, 0.0),orientation=(0.0, 0.0, 0.0, 1.0))
            ),
            # 第二次调用返回释放轨迹的位姿
            (
                KuavoPose(position=(0.0, 0.0, 0.0),orientation=(0.0, 0.0, 0.0, 1.0)),
                KuavoPose(position=(0.0, 0.0, 0.0),orientation=(0.0, 0.0, 0.0, 1.0)),
                KuavoPose(position=(0.0, 0.0, 0.0),orientation=(0.0, 0.0, 0.0, 1.0)),
                KuavoPose(position=(0.0, 0.0, 0.0),orientation=(0.0, 0.0, 0.0, 1.0))
            )
        ])

        # 模拟robot方法
        mock_robot.control_robot_end_effector_pose = MagicMock(return_value=True)
        mock_robot.set_manipulation_mpc_mode = MagicMock()
        mock_robot.set_manipulation_mpc_control_flow = MagicMock()
        mock_robot.manipulation_mpc_reset = MagicMock()
        mock_robot.arm_reset = MagicMock()

        # 调用方法
        result = grasp_box.arm_transport_target_down(
            target_info=test_box,
            arm_mode="manipulation_mpc"
        )

        # 验证结果
        assert result is True
        assert mock_robot.control_robot_end_effector_pose.call_count == 20  # 修改为20次
        assert mock_robot.set_manipulation_mpc_mode.call_count == 2
        mock_robot.set_manipulation_mpc_control_flow.assert_called_once()
        grasp_box._check_orientation_safety.assert_called_once()
        grasp_box._check_position_safety.assert_called_once()
        grasp_box._check_height_safety.assert_called_once()
        mock_robot.manipulation_mpc_reset.assert_called_once()
        mock_robot.arm_reset.assert_called_once()

    def test_arm_transport_target_down_failure(self, setup_grasp_box):
        """测试放下失败"""
        grasp_box, mock_robot, mock_robot_state, mock_robot_tools, mock_robot_vision = setup_grasp_box
        
        # 创建测试用的目标信息
        test_box = BoxInfo(
            pose=KuavoPose(
                position=(1.0, 0.0, 0.8),
                orientation=(0.0, 0.0, 0.0, 1.0)
            ),
            size=(0.1, 0.1, 0.1),
            mass=1.0
        )   

        # 修改安全检查的mock，让其中一个返回False
        grasp_box._check_orientation_safety = MagicMock(return_value=False)  # 改为False
        grasp_box._check_position_safety = MagicMock(return_value=True)
        grasp_box._check_height_safety = MagicMock(return_value=True)

        # 模拟_get_target_pose方法
        grasp_box._get_target_pose = MagicMock(return_value=(
            KuavoPose(position=(0.0, 0.0, 0.0),orientation=(0.0, 0.0, 0.0, 1.0)),
            KuavoPose(position=(0.0, 0.0, 0.0),orientation=(0.0, 0.0, 0.0, 1.0)),
            KuavoPose(position=(0.0, 0.0, 0.0),orientation=(0.0, 0.0, 0.0, 1.0)),
            KuavoPose(position=(0.0, 0.0, 0.0),orientation=(0.0, 0.0, 0.0, 1.0))
        ))

        # 模拟robot方法
        mock_robot.control_robot_end_effector_pose = MagicMock(return_value=False)
        mock_robot.set_manipulation_mpc_mode = MagicMock()
        mock_robot.set_manipulation_mpc_control_flow = MagicMock()

        # 调用方法
        result = grasp_box.arm_transport_target_down(
            target_info=test_box,
            arm_mode="manipulation_mpc"
        )

        # 验证结果
        assert result is False
        mock_robot.control_robot_end_effector_pose.assert_not_called()
        mock_robot.set_manipulation_mpc_mode.assert_not_called()
        mock_robot.set_manipulation_mpc_control_flow.assert_not_called()
        grasp_box._check_orientation_safety.assert_called_once()
        grasp_box._check_position_safety.assert_not_called()  # 由于orientation检查失败，这个应该不会被调用
        grasp_box._check_height_safety.assert_not_called()    # 由于orientation检查失败，这个应该不会被调用

    def test_head_find_target_success_rotate_head(self, setup_grasp_box):
        """测试头部旋转成功找到目标（仅头部旋转模式）"""
        grasp_box, mock_robot, mock_robot_state, _, mock_robot_vision = setup_grasp_box
        
        # 配置mock返回有效数据
        mock_robot_state.robot_orientation.return_value = (0.0, 0.0, 0.0, 1.0)  # 四元数
        mock_robot_state.robot_position.return_value = (0.0, 0.0, 0.0)  # 初始位置
        
        # 创建测试用的AprilTag数据
        test_tag = AprilTagData(
            id=[123],
            size=[0.1],
            pose=[PoseQuaternion(
                position=(1.0, 0.0, 0.0),
                orientation=(0.0, 0.0, 0.0, 1.0)
            )]
        )
        
        # 模拟视觉数据返回
        mock_robot_vision.get_data_by_id_from_odom.side_effect = [
            None,  # 第一次扫描未找到
            None,  # 第二次扫描未找到
            {"poses": [{"position": (1.0, 0.0, 0.0), "orientation": (0.0, 0.0, 0.0, 1.0)}]}  # 第三次找到
        ]
        
        # 调用方法（使用默认的头部旋转模式）
        result = grasp_box.head_find_target(test_tag, search_pattern="rotate_head")
        
        # 验证结果
        assert result is True
        assert mock_robot.control_head.call_count >= 2  # 至少调用了两次头部控制
        mock_robot.enable_head_tracking.assert_called_once_with(123)

    def test_head_find_target_success_rotate_body(self, setup_grasp_box):
        """测试需要旋转身体后成功找到目标"""
        grasp_box, mock_robot, mock_robot_state, _, mock_robot_vision = setup_grasp_box
        
        # 配置mock返回有效数据
        mock_robot_state.robot_orientation.return_value = (0.0, 0.0, 0.0, 1.0)
        mock_robot_state.robot_position.return_value = (0.0, 0.0, 0.0)
        
        # 创建测试用的AprilTag数据（位置在机器人右侧）
        test_tag = AprilTagData(
            id=[456],
            size=[0.2],
            pose=[PoseQuaternion(
                position=(0.0, 1.0, 0.0),  # 在机器人右侧
                orientation=(0.0, 0.0, 0.0, 1.0)
            )]
        )
        
        # 模拟视觉数据返回
        mock_robot_vision.get_data_by_id_from_odom.return_value = {
            "poses": [{"position": (0.0, 1.0, 0.0), "orientation": (0.0, 0.0, 0.0, 1.0)}]
        }
        
        # 调用方法（使用身体旋转模式）
        result = grasp_box.head_find_target(test_tag, search_pattern="rotate_body")
        
        # 验证结果
        assert result is True
        mock_robot.control_command_pose_world.assert_called_once_with(0.0, 0.0, 0.0, math.pi/2)
        mock_robot.enable_head_tracking.assert_called_once_with(456)

    def test_head_find_target_timeout(self, setup_grasp_box):
        """测试搜索超时未找到目标"""
        grasp_box, mock_robot, mock_robot_state, _, mock_robot_vision = setup_grasp_box
        
        # 配置mock返回有效数据
        mock_robot_state.robot_orientation.return_value = (0.0, 0.0, 0.0, 1.0)
        mock_robot_state.robot_position.return_value = (0.0, 0.0, 0.0)
        
        test_tag = AprilTagData(
            id=[200],
            size=[0.3],
            pose=[PoseQuaternion(
                position=(2.0, 0.0, 0.0),
                orientation=(0.0, 0.0, 0.0, 1.0)
            )]
        )
        
        # 模拟始终返回None
        mock_robot_vision.get_data_by_id_from_odom.return_value = None
        
        # 调用方法（设置较短超时时间）
        result = grasp_box.head_find_target(test_tag, max_search_time=2.0)
        
        # 验证结果
        assert result is False
        assert mock_robot.control_head.call_count >= 4  # 至少完成两轮扫描
        mock_robot.enable_head_tracking.assert_not_called()

    def test_head_find_target_invalid_id(self, setup_grasp_box):
        """测试目标ID不存在的情况"""
        grasp_box, mock_robot, mock_robot_state, _, mock_robot_vision = setup_grasp_box

        # 配置mock返回有效数据
        mock_robot_state.robot_orientation.return_value = (0.0, 0.0, 0.0, 1.0)
        mock_robot_state.robot_position.return_value = (0.0, 0.0, 0.0)

        # 创建测试用的AprilTag数据（使用无效ID）
        test_tag = AprilTagData(
            id=[10000],  # 初始化时的异常ID
            size=[0.0],
            pose=[PoseQuaternion(
                position=(0.0, 0.0, 0.0),
                orientation=(0.0, 0.0, 0.0, 1.0)
            )]
        )

        # 修复点1: 确保视觉模块返回None
        mock_robot_vision.get_data_by_id_from_odom.return_value = None

        # 调用方法
        result = grasp_box.head_find_target(test_tag)

        # 验证结果
        assert result is False
        mock_robot_vision.get_data_by_id_from_odom.assert_not_called()  # 应该没有被调用
        mock_robot.control_head.assert_not_called()
        mock_robot.enable_head_tracking.assert_not_called()