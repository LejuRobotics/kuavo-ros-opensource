"""
轮臂躯干运动控制案例

使用 NodeWheelMoveTimedCmd 通过 /mobile_manipulator_timed_single_cmd 服务发送躯干位姿。
使用 planner_index=2，命令格式为 [x, z, yaw, pitch]（4维）。
"""
import os
import sys
import time
from typing import List, Optional, Tuple

import rospy
import py_trees

sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))

from kuavo_msgs.srv import getLbTorsoInitialPose, getLbTorsoInitialPoseRequest
from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.nodes import NodeWheelMoveTimedCmd, NodeFuntion
from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.api import TimedCmdAPI
from kuavo_humanoid_sdk.kuavo_strategy_pytree.common.robot_sdk import RobotSDK


def get_torso_initial_pose(need_pose: bool = True) -> Tuple[bool, Optional[dict]]:
    """
    获取躯干初始位姿
    
    Args:
        need_pose: 是否需要获取位姿，如果为False则返回失败
    Returns:
        (success, pose_dict): 
            success: True/False 是否成功
            pose_dict: 字典，包含位置和欧拉角信息
                       {
                           'position': [x, y, z],
                           'euler': [yaw, pitch, roll]  # 注意：顺序为ZYX
                       }
    """
    try:
        rospy.wait_for_service('/mobile_manipulator_get_torso_initial_pose', timeout=5.0)
        client = rospy.ServiceProxy('/mobile_manipulator_get_torso_initial_pose', getLbTorsoInitialPose)
        req = getLbTorsoInitialPoseRequest()
        req.isNeed = need_pose
        
        resp = client(req)
        if resp.result:
            # 从geometry_msgs/Twist中提取数据
            # 注意：angular中的顺序是z,y,x对应yaw,pitch,roll
            pose_data = {
                'position': [resp.linear.x, resp.linear.y, resp.linear.z],
                'euler': [resp.angular.z, resp.angular.y, resp.angular.x]  # ZYX顺序
            }
            rospy.loginfo(f"✅ 获取躯干初始位姿成功")
            rospy.loginfo(f"   位置: [{pose_data['position'][0]:.3f}, {pose_data['position'][1]:.3f}, {pose_data['position'][2]:.3f}]")
            rospy.loginfo(f"   欧拉角(ZYX): [{pose_data['euler'][0]:.3f}, {pose_data['euler'][1]:.3f}, {pose_data['euler'][2]:.3f}]")
            return True, pose_data
        else:
            rospy.logwarn(f"⚠️  获取躯干初始位姿失败: {resp.message}")
            return False, None
    except rospy.ROSException as e:
        rospy.logerr(f"❌ 服务等待超时: {e}")
        return False, None
    except rospy.ServiceException as e:
        rospy.logerr(f"❌ 服务调用失败: {e}")
        return False, None
    except Exception as e:
        rospy.logerr(f"❌ 未知错误: {e}")
        return False, None


def generate_torso_keypoints(torso_poses: List[dict], initial_pose: Optional[dict] = None):
    """
    生成躯干关键点列表
    
    Args:
        torso_poses: 位姿列表，每个包含 time, pose([lx, lz, yaw, pitch]，米/弧度)
                     如果提供了initial_pose，则pose为相对增量；否则为绝对坐标
        initial_pose: 初始位姿字典，包含 'position': [x, y, z]
    Returns:
        (keypoints, times): 位姿列表、时间列表
    """
    keypoints = []
    times = []
    
    if initial_pose is not None:
        initial_pos = initial_pose['position']
        print(f"使用初始位姿: {initial_pos}")
    
    for pose in torso_poses:
        # 如果提供了初始位姿，将相对增量转换为绝对坐标
        if initial_pose is not None:
            lx, lz, az, ay = pose['pose']
            abs_x = initial_pos[0] + lx
            abs_z = initial_pos[2] + lz
            abs_pose = [abs_x, abs_z, az, ay]
            keypoints.append(abs_pose)
        else:
            keypoints.append(pose['pose'])
        times.append(pose['time'])
    return keypoints, times


def make_tree(timed_cmd_api, torso_poses: Optional[List[dict]] = None, initial_pose: Optional[dict] = None):
    """构建行为树"""
    poses = torso_poses if torso_poses is not None else TORSO_POSES
    keypoints, times = generate_torso_keypoints(poses, initial_pose)
    
    keypoints_key = "torso_keypoints"
    times_key = "torso_keypoint_times"
    
    def set_keypoints_to_blackboard():
        bb = py_trees.blackboard.Client(name="set_torso_keypoints")
        bb.register_key(keypoints_key, py_trees.common.Access.WRITE)
        bb.register_key(times_key, py_trees.common.Access.WRITE)
        setattr(bb, keypoints_key, keypoints)
        setattr(bb, times_key, times)
        print(f"设置 {len(keypoints)} 个躯干关键点")
        return True
    
    root = py_trees.composites.Sequence(name="torso_demo", memory=True)
    root.add_children([
        NodeFuntion(name="set_torso_keypoints", fn=set_keypoints_to_blackboard),
        NodeWheelMoveTimedCmd(name="move_torso", timed_cmd_api=timed_cmd_api, cmd_type='torso'),
    ])
    return root


if __name__ == '__main__':
    # 初始化ROS节点
    rospy.init_node('torso_pose_pytree', anonymous=True)
    
    # 首先获取躯干初始位姿
    print("正在获取躯干初始位姿...")
    success, initial_pose = get_torso_initial_pose(True)
    
    if not success:
        print("❌ 无法获取躯干初始位姿，退出程序")
        sys.exit(1)
    
    # 提取初始位置
    if initial_pose is not None:
        initialTorsoPose_ = initial_pose['position']
        print(f"初始躯干位置: {initialTorsoPose_}")
    else:
        print("警告: 初始位姿为None，将使用绝对坐标")
        initialTorsoPose_ = None
    
    # 躯干关键点：time, pose([lx, lz, yaw, pitch]，米/弧度)
    # 注意：lx, lz 是相对于初始位置的增量，yaw和pitch是绝对角度
    # x为前后位置，z为高度，yaw为偏航角，pitch为俯仰角
    TORSO_POSES = [
        {'time': 2.0, 'pose': [0.0, 0.3, 0.0, 0.0]},      # 初始抬高
        {'time': 2.0, 'pose': [0.2, 0.3, 0.0, 0.0]},       # 前移
        {'time': 2.0, 'pose': [0.2, 0.3, 0.52356, 0.0]},      # 偏航+30° (约180°)
        {'time': 2.0, 'pose': [0.2, 0.3, -0.52356, 0.0]}, # 偏航-30° (约-180°)
        {'time': 2.0, 'pose': [0.2, 0.3, 0.0, -0.1745]},   # 俯仰-10°
        {'time': 2.0, 'pose': [0.2, 0.3, 0.0, 0.524]},     # 俯仰+30°
        {'time': 2.0, 'pose': [0.0, 0.0, 0.0, 0.0]},       # 复位
    ]

    timed_cmd_api = TimedCmdAPI()
    root = make_tree(timed_cmd_api, initial_pose=initial_pose)
    tree = py_trees.trees.BehaviourTree(root)
    
    print("开始躯干控制...")
    while True:
        tree.tick()
        status = root.status
        print(py_trees.display.unicode_tree(root, show_status=True))
        
        if status == py_trees.common.Status.SUCCESS:
            print("躯干运动完成!")
            break
        if status == py_trees.common.Status.FAILURE:
            print("躯干运动失败.")
            break
        time.sleep(0.1)

