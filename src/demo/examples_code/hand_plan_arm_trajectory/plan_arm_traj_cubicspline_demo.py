#!/usr/bin/env python3

import rospy
import json
import math
import os
import sys
import numpy as np
import rospkg
from humanoid_plan_arm_trajectory.srv import planArmTrajectoryCubicSpline, planArmTrajectoryCubicSplineRequest
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from kuavo_msgs.srv import changeArmCtrlMode, changeArmCtrlModeRequest
from kuavo_msgs.msg import sensorsData

# 使用 rospkg 获取 kuavo_common 包路径并导入 RobotVersion
try:
    _kuavo_common_python = os.path.join(rospkg.RosPack().get_path('kuavo_common'), 'python')
    if _kuavo_common_python not in sys.path:
        sys.path.insert(0, _kuavo_common_python)
    from robot_version import RobotVersion
except (rospkg.ResourceNotFound, ImportError):
    _kuavo_common_python = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../../kuavo_common/python'))
    if _kuavo_common_python not in sys.path:
        sys.path.insert(0, _kuavo_common_python)
    from robot_version import RobotVersion

# 声明全局变量 用于sensors_data_raw中手臂角度的索引
joint_data_header = None
joint_data_footer = None

# 获取机器人版本
def get_version_parameter():
    param_name = 'robot_version'
    try:
        param_value = rospy.get_param(param_name)
        rospy.loginfo(f"参数 {param_name} 的值为: {param_value}")
        if not RobotVersion.is_valid(param_value):
            rospy.logerr(f"无效的机器人版本号: {param_value}，版本号格式不合法！程序退出。")
            rospy.signal_shutdown("参数无效")
            return param_value

        rv = RobotVersion.create(int(param_value))
        if (rv.start_with(4, 2) or rv.start_with(4, 5) or rv.start_with(4, 9)
                or rv.start_with(5, 2)):
            rospy.loginfo(f"✅ 机器人版本号有效: {param_value} ({rv.version_name()})")
        else:
            rospy.logerr(f"无效的机器人版本号: {param_value} ({rv.version_name()})，仅支持 42 45 49 52 系列！程序退出。")
            rospy.signal_shutdown("参数无效")
        return param_value
    except rospy.ROSException:
        rospy.logerr(f"参数 {param_name} 不存在！程序退出。")
        rospy.signal_shutdown("参数获取失败")
        return None

# 存储当前机器人手臂关节状态
current_arm_joint_state = []

def deg_to_rad(deg):
    """
    将角度转换为弧度
    
    参数:
        deg: 角度值
    返回:
        弧度值
    """
    return math.radians(deg)

def sensors_data_callback(msg):
    """
    传感器数据回调函数，用于获取当前手臂关节状态
    
    参数:
        msg: 传感器数据消息
    """
    global current_arm_joint_state
    # 获取索引12到26的关节位置数据（对应手臂关节）
    current_arm_joint_state = msg.joint_data.joint_q[joint_data_header:joint_data_footer]
    # 将关节位置数据四舍五入到小数点后两位
    current_arm_joint_state = [round(pos, 2) for pos in current_arm_joint_state]


# def mpc_obs_callback(msg):
#     global current_arm_joint_state
#     current_arm_joint_state = msg.state.value[24:]
#     current_arm_joint_state = [round(pos, 2) for pos in current_arm_joint_state]

# 预定义的手臂关节位置序列，每个位置包含14个关节角度（左右手臂各7个关节）
positions = [
    # 初始位置 - 所有关节角度为0
    [deg_to_rad(angle) for angle in [0,0,0,0,0,0,0,0,0,0,0,0,0,0]],
    # 左右手臂稍微抬起
    [deg_to_rad(angle) for angle in [20,0,0,-30,0,0,0,20,0,0,-30,0,0,0]],
    # 左手保持，右手弯曲
    [deg_to_rad(angle) for angle in [20,0,0,-30,0,0,0,-30,0,30,-88,8,-22,-4]],
    # 左手保持，右手调整
    [deg_to_rad(angle) for angle in [20,0,0,-30,0,0,0,-30,-25,-54,-15,-6,-22,-4]],
    # 左手弯曲，右手保持
    [deg_to_rad(angle) for angle in [10,10,-20,-70,0,0,-24,-30,-25,-54,-15,-6,-22,-4]],
    # 左手进一步调整，右手保持
    [deg_to_rad(angle) for angle in [14,20,33,-35,76,-18,3.5,-30,-25,-54,-15,-6,-22,-4]],
    # 回到中间位置
    [deg_to_rad(angle) for angle in [20,0,0,-30,0,0,0,20,0,0,-30,0,0,0]],
    # 回到初始位置
    [deg_to_rad(angle) for angle in [0,0,0,0,0,0,0,0,0,0,0,0,0,0]],
]
# 位置序列的长度
position_size = len(positions)
# 为每个位置设置时间点（从3秒开始，每个位置间隔1.5秒）
times = [3 + 1.5 * i for i in range(position_size)]
# 创建关节状态消息
joint_state = JointState()

def traj_callback(msg):
    """
    轨迹回调函数，处理接收到的关节轨迹消息
    
    参数:
        msg: 关节轨迹消息
    """
    global joint_state
    if len(msg.points) == 0:
        return
    point = msg.points[0]
    # 设置关节名称
    joint_state.name = [
        "l_arm_pitch", "l_arm_roll", "l_arm_yaw", "l_forearm_pitch", 
        "l_hand_yaw", "l_hand_pitch", "l_hand_roll",
        "r_arm_pitch", "r_arm_roll", "r_arm_yaw", "r_forearm_pitch", 
        "r_hand_yaw", "r_hand_pitch", "r_hand_roll",
    ]
    # 将位置从弧度转换为角度
    joint_state.position = [math.degrees(pos) for pos in point.positions[:14]]
    # 将速度从弧度/秒转换为角度/秒
    joint_state.velocity = [math.degrees(vel) for vel in point.velocities[:14]]
    # 设置力矩为0
    joint_state.effort = [0] * 14

def call_change_arm_ctrl_mode_service(arm_ctrl_mode):
    """
    调用服务更改手臂控制模式
    
    参数:
        arm_ctrl_mode: 手臂控制模式
    返回:
        调用服务是否成功
    """
    result = True
    service_name = "humanoid_change_arm_ctrl_mode"
    try:
        rospy.wait_for_service(service_name, timeout=0.5)
        change_arm_ctrl_mode = rospy.ServiceProxy(
            "humanoid_change_arm_ctrl_mode", changeArmCtrlMode
        )
        change_arm_ctrl_mode(control_mode=arm_ctrl_mode)
        rospy.loginfo("服务调用成功")
    except rospy.ServiceException as e:
        rospy.loginfo("服务调用失败: %s", e)
        result = False
    except rospy.ROSException:
        rospy.logerr(f"服务 {service_name} 不可用")
        result = False
    finally:
        return result

def plan_arm_traj_cubicspline_demo():
    """
    使用三次样条插值规划手臂轨迹
    
    返回:
        规划是否成功
    """
    # 等待服务可用
    rospy.wait_for_service('/cubic_spline/plan_arm_trajectory')
    # 创建服务代理
    plan_arm_traj_cubicspline = rospy.ServiceProxy('/cubic_spline/plan_arm_trajectory', planArmTrajectoryCubicSpline)
    # 创建请求
    request = planArmTrajectoryCubicSplineRequest()
    # 创建关节轨迹
    joint_trajectory = JointTrajectory()
    print(times)
    print(positions[0])
    # 添加轨迹点
    for i in range(len(times)):
        joint_trajectory.points.append(JointTrajectoryPoint())
        joint_trajectory.points[-1].positions = positions[i]
        joint_trajectory.points[-1].time_from_start = rospy.Duration(times[i])
    # 设置请求的关节轨迹
    request.joint_trajectory = joint_trajectory
    # 设置关节名称
    request.joint_trajectory.joint_names = ["l_arm_pitch", "l_arm_roll", "l_arm_yaw", "l_forearm_pitch", "l_hand_yaw", "l_hand_pitch", "l_hand_roll", "r_arm_pitch", "r_arm_roll", "r_arm_yaw", "r_forearm_pitch", "r_hand_yaw", "r_hand_pitch", "r_hand_roll"]
    # 调用服务
    response = plan_arm_traj_cubicspline(request)

    return response.success

def main():
    """
    主函数，初始化节点并运行轨迹规划演示
    """
    # 初始化ROS节点
    rospy.init_node('arm_trajectory_cubicspline_demo')
    # 获取机器人版本
    robot_version = get_version_parameter()
    # 根据机器人版本 设定sensors_data_raw中手臂角度的索引
    global joint_data_header, joint_data_footer
    rv = RobotVersion.create(int(robot_version))
    if rv.start_with(4, 2) or rv.start_with(4, 5) or rv.start_with(4, 9):
        joint_data_header = 12
        joint_data_footer = 26
    elif rv.start_with(5, 2):
        joint_data_header = 13
        joint_data_footer = 27
    # 订阅三次样条轨迹话题
    traj_sub = rospy.Subscriber('/cubic_spline/arm_traj', JointTrajectory, traj_callback, queue_size=1, tcp_nodelay=True)
    # 发布手臂轨迹话题
    kuavo_arm_traj_pub = rospy.Publisher('/kuavo_arm_traj', JointState, queue_size=1, tcp_nodelay=True)
    # 订阅传感器数据话题
    sensor_data_sub = rospy.Subscriber(
            '/sensors_data_raw', 
            sensorsData, 
            sensors_data_callback, 
            queue_size=1, 
            tcp_nodelay=True
    )
    # 设置手臂控制模式为2
    call_change_arm_ctrl_mode_service(2)
    
    # 等待获取当前手臂关节状态
    while len(current_arm_joint_state) != 0:
        break
    # 在轨迹开始前添加当前位置作为起始点
    times.insert(0, 2)
    positions.insert(0, current_arm_joint_state)
    # 规划轨迹
    success = plan_arm_traj_cubicspline_demo()
    if success:
        rospy.loginfo("手臂轨迹规划成功")
    else:
        rospy.logerr("手臂轨迹规划失败")

    while kuavo_arm_traj_pub.get_num_connections() == 0 and not rospy.is_shutdown():
        rospy.loginfo("Waiting for kuavo_arm_traj_pub subscriber...")
        rospy.sleep(0.1)

    # 设置发布频率为100Hz
    rate = 100
    # 主循环
    while not rospy.is_shutdown():
        try:
            global joint_state
            if len(joint_state.position) == 0:
                continue
            # 发布关节状态
            kuavo_arm_traj_pub.publish(joint_state)
        except Exception as e:
            rospy.logerr(f"发布手臂轨迹失败: {e}")
        except KeyboardInterrupt:
            break
        rospy.sleep(1/rate)

if __name__ == "__main__":
    main()
