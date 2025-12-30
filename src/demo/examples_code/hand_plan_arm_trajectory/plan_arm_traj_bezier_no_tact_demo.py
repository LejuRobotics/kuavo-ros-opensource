#!/usr/bin/env python3

import rospy
import json
import math
import numpy as np
from humanoid_plan_arm_trajectory.srv import planArmTrajectoryBezierCurve, planArmTrajectoryBezierCurveRequest
from humanoid_plan_arm_trajectory.msg import bezierCurveCubicPoint, jointBezierTrajectory
from kuavo_ros_interfaces.srv import stopPlanArmTrajectory
from kuavo_ros_interfaces.msg import robotHandPosition, robotHeadMotionData
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from kuavo_msgs.srv import changeArmCtrlMode, changeArmCtrlModeRequest
from kuavo_msgs.msg import sensorsData

# 机器人手臂的初始位置（以角度表示）
# 机器人手臂的初始位置,包含28个关节角度值(单位:度)
# 前14个值为左右手臂的实际关节角度,后14个值为预留的零值
INIT_ARM_POS = [20, 0, 0, -30, 0, 0, 0, 20, 0, 0, -30, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

# 轨迹的起始和结束时间(单位:秒)
START_FRAME_TIME = 0  # 轨迹开始时间
END_FRAME_TIME = 22   # 轨迹结束时间

# 时间偏移量,用于调整轨迹时间
x_shift = START_FRAME_TIME - 1

# 用于存储关节状态的消息对象
joint_state = JointState()
hand_state = robotHandPosition()
head_state = robotHeadMotionData()

# 存储当前手臂关节状态的列表
current_arm_joint_state = []


def sensors_data_callback(msg):
    """
    传感器数据回调函数
    从msg.joint_data.joint_q[12:26]获取手臂关节状态并四舍五入到小数点后两位
    然后在末尾添加14个零值作为预留位置

    Args:
        msg: sensorsData类型的消息,包含机器人的传感器数据
    """
    global current_arm_joint_state
    current_arm_joint_state = msg.joint_data.joint_q[12:26]
    current_arm_joint_state = [round(pos, 2) for pos in current_arm_joint_state]
    current_arm_joint_state.extend([0] * 14)

import numpy as np

def traj_callback(msg):
    """
    轨迹回调函数
    处理接收到的轨迹消息,更新joint_state对象中的关节状态信息
    
    Args:
        msg: JointTrajectory类型的消息,包含关节轨迹点数据
        
    主要功能:
    1. 检查轨迹点是否为空
    2. 设置关节名称列表(左右手臂各7个关节)
    3. 将关节位置从弧度转换为角度
    4. 将关节速度从弧度/秒转换为角度/秒
    5. 设置关节力矩为0
    """
    global joint_state, hand_state, head_state
    if len(msg.points) == 0:
        return
    point = msg.points[0]
    joint_state.name = [
        "l_arm_pitch",
        "l_arm_roll", 
        "l_arm_yaw",
        "l_forearm_pitch",
        "l_hand_yaw",
        "l_hand_pitch",
        "l_hand_roll",
        "r_arm_pitch",
        "r_arm_roll",
        "r_arm_yaw", 
        "r_forearm_pitch",
        "r_hand_yaw",
        "r_hand_pitch",
        "r_hand_roll",
    ]
    # 将关节位置从弧度转换为角度
    joint_state.position = [math.degrees(pos) for pos in point.positions[:14]]
    # 将关节速度从弧度/秒转换为角度/秒 
    joint_state.velocity = [math.degrees(vel) for vel in point.velocities[:14]]
    # 设置关节力矩为0
    joint_state.effort = [0] * 14

    hand_state.left_hand_position = [int(math.degrees(pos)) for pos in point.positions[14:20]]
    hand_state.right_hand_position = [int(math.degrees(pos)) for pos in point.positions[20:26]]

    head_state.joint_data = [math.degrees(pos) for pos in point.positions[26:]]


def call_change_arm_ctrl_mode_service(arm_ctrl_mode):
    """
    调用服务来改变手臂控制模式
    
    Args:
        arm_ctrl_mode: 手臂控制模式参数
        
    Returns:
        result: bool类型,表示服务调用是否成功
        
    主要功能:
    1. 等待服务可用(超时时间0.5秒)
    2. 创建服务代理并调用服务
    3. 处理可能的异常情况
    4. 返回服务调用结果
    """
    result = True
    service_name = "humanoid_change_arm_ctrl_mode"
    try:
        # 等待服务可用,超时时间0.5秒
        rospy.wait_for_service(service_name, timeout=0.5)
        # 创建服务代理
        change_arm_ctrl_mode = rospy.ServiceProxy(
            "humanoid_change_arm_ctrl_mode", changeArmCtrlMode
        )
        # 调用服务
        change_arm_ctrl_mode(control_mode=arm_ctrl_mode)
        rospy.loginfo("Service call successful")
    except rospy.ServiceException as e:
        # 服务调用失败的异常处理
        rospy.loginfo("Service call failed: %s", e)
        result = False
    except rospy.ROSException:
        # 服务不可用的异常处理
        rospy.logerr(f"Service {service_name} not available")
        result = False
    finally:
        return result
    


def add_init_frame(frames):
    """
    为轨迹添加初始帧
    Args:
        frames: 包含关节位置和属性的帧列表
    Returns:
        action_data: 包含贝塞尔曲线控制点的轨迹数据字典
        
    主要功能:
    1. 为每个关节添加初始位置帧
    2. 计算贝塞尔曲线控制点
    3. 转换角度单位(度->弧度)
    """
    action_data = {}
    for frame in frames:
        servos, keyframe, attribute = frame["servos"], frame["keyframe"], frame["attribute"]
        for index, value in enumerate(servos):
            key = index + 1
            # 为每个关节初始化轨迹数据
            if key not in action_data:
                action_data[key] = []
                # 如果不是第一帧，添加初始位置
                if keyframe != 0 and len(action_data[key]) == 0:
                    if key <= len(INIT_ARM_POS):
                        action_data[key].append([
                            [0, math.radians(INIT_ARM_POS[key-1])],
                            [0, math.radians(INIT_ARM_POS[key-1])],
                            [0, math.radians(INIT_ARM_POS[key-1])],
                ])
            if value is not None:
                CP = attribute[str(key)]["CP"]
                left_CP, right_CP = CP
                # 添加贝塞尔曲线控制点
                action_data[key].append([
                    [round(keyframe/100, 1), math.radians(value)],
                    [round((keyframe+left_CP[0])/100, 1), math.radians(value+left_CP[1])],
                    [round((keyframe+right_CP[0])/100, 1), math.radians(value+right_CP[1])],
                ])
    return action_data

def frames_to_custom_action_data(frames):
    """
    将帧数据转换为自定义动作数据格式
    
    Args:
        frames: 包含关节位置和属性的帧列表
        
    Returns:
        action_data: 包含贝塞尔曲线控制点的轨迹数据字典
        
    主要功能:
    1. 解析每一帧的关节数据
    2. 为每个关节生成贝塞尔曲线控制点
    3. 调整时间戳和角度单位
    """
    action_data = {}
    for frame in frames:
        # 解析每一帧的关节数据、关键帧时间和属性
        servos, keyframe, attribute = frame["servos"], frame["keyframe"], frame["attribute"]
        for index, value in enumerate(servos):
            key = index + 1
            # 为每个关节初始化轨迹数据
            if key not in action_data:
                action_data[key] = []
                # 如果不是第一帧，添加初始位置
                if keyframe != 0 and len(action_data[key]) == 0:
                    if key <= len(INIT_ARM_POS):
                        action_data[key].append([
                            [0, math.radians(INIT_ARM_POS[key-1])],  # 初始位置
                            [0, 0],  # 左控制点
                            [0, math.radians(INIT_ARM_POS[key-1])],  # 右控制点
                ])
            # 处理有效的关节值
            if value is not None:
                # 获取贝塞尔曲线控制点参数
                CP = attribute[str(key)]["CP"]
                left_CP, right_CP = CP
                # 添加贝塞尔曲线控制点，包括位置点和左右控制点
                action_data[key].append([
                    [round(keyframe/100, 1) - x_shift, math.radians(value)],  # 位置点
                    [round((keyframe+left_CP[0])/100, 1) - x_shift, math.radians(value+left_CP[1])],  # 左控制点
                    [round((keyframe+right_CP[0])/100, 1) - x_shift, math.radians(value+right_CP[1])],  # 右控制点
                ])
    return action_data


def filter_data(action_data):
    """
    轨迹数据过滤和平滑处理
    
    Args:
        action_data: 原始轨迹数据字典
        
    Returns:
        filtered_action_data: 经过过滤和平滑处理的轨迹数据
        
    主要功能:
    1. 在起始位置添加平滑过渡
    2. 使用贝塞尔曲线控制点实现平滑过渡
    3. 调整时间戳并过滤轨迹点
    """
    filtered_action_data = {}
    for key, frames in action_data.items():
        filtered_frames = []
        found_start = False
        skip_next = False
        for i in range(-1, len(frames)):
            frame = frames[i]
            if i == len(frames) - 1:
                next_frame = frame
            else:
                next_frame = frames[i+1]
            end_time = next_frame[0][0]
            
            # 找到起始帧并创建平滑过渡
            if not found_start and end_time >= START_FRAME_TIME:
                found_start = True
                p0 = np.array([0, current_arm_joint_state[key-1]])
                p3 = np.array([next_frame[0][0] - x_shift, next_frame[0][1]])
                
                # 计算控制点以实现平滑过渡
                curve_length = np.linalg.norm(p3 - p0)
                p1 = p0 + curve_length * 0.25 * np.array([1, 0])  # Move 1/4 curve length to the right
                p2 = p3 - curve_length * 0.25 * np.array([1, 0])  # Move 1/4 curve length to the left
                
                # 添加新的过渡帧
                filtered_frames.append([
                    p0.tolist(),
                    p0.tolist(),
                    p1.tolist()
                ])
                next_frame[1] = p2.tolist()  # 修改下一帧的左控制点
                skip_next = True
                continue
            
            # 处理后续帧
            if found_start:
                if skip_next:
                    skip_next = False
                    continue
                # 调整时间并添加到过滤后的帧列表
                end_point = [round(frame[0][0] - x_shift, 1), round(frame[0][1], 1)]
                left_control_point = [round(frame[1][0] - x_shift, 1), round(frame[1][1], 1)]
                right_control_point = [round(frame[2][0] - x_shift, 1), round(frame[2][1], 1)]
                filtered_frames.append([end_point, left_control_point, right_control_point])

        filtered_action_data[key] = filtered_frames
    return filtered_action_data

def create_bezier_request(action_data):
    """
    创建贝塞尔曲线轨迹规划请求
    
    Args:
        action_data: 过滤后的轨迹数据
        
    Returns:
        req: 包含完整轨迹信息的规划请求对象
        
    主要功能:
    1. 为每个关节创建贝塞尔曲线轨迹
    2. 设置轨迹的起始和结束时间
    3. 配置关节名称列表
    """
    req = planArmTrajectoryBezierCurveRequest()
    # 为每个关节创建贝塞尔曲线轨迹
    for key, value in action_data.items():
        msg = jointBezierTrajectory()
        for frame in value:
            point = bezierCurveCubicPoint()
            point.end_point, point.left_control_point, point.right_control_point = frame
            msg.bezier_curve_points.append(point)
        req.multi_joint_bezier_trajectory.append(msg)
    req.start_frame_time = START_FRAME_TIME
    req.end_frame_time = END_FRAME_TIME
    req.joint_names = ["l_arm_pitch", "l_arm_roll", "l_arm_yaw", "l_forearm_pitch", "l_hand_yaw", "l_hand_pitch", "l_hand_roll", "r_arm_pitch", "r_arm_roll", "r_arm_yaw", "r_forearm_pitch", "r_hand_yaw", "r_hand_pitch", "r_hand_roll"]
    return req


def plan_arm_trajectory_bezier_curve_client(req):
    """
    调用贝塞尔曲线轨迹规划服务的客户端函数
    
    Args:
        req: planArmTrajectoryBezierCurveRequest 类型的请求对象,包含轨迹规划所需的参数
        
    Returns:
        bool: 规划是否成功
        - True: 规划成功
        - False: 规划失败
        
    主要功能:
    1. 等待服务可用
    2. 创建服务代理对象
    3. 调用服务并处理响应
    4. 异常处理
    """
    # 设置服务名称
    service_name = '/bezier/plan_arm_trajectory'
    # 等待服务可用
    rospy.wait_for_service(service_name)
    try:
        # 创建服务代理对象
        plan_service = rospy.ServiceProxy(service_name, planArmTrajectoryBezierCurve)
        # 调用服务并获取响应
        res = plan_service(req)
        return res.success
    except rospy.ServiceException as e:
        # 服务调用失败时记录错误日志
        rospy.logerr(f"Service call failed: {e}")
        return False

def stop_arm_trajectory_client():
    """
    停止手臂轨迹执行的客户端函数
    
    Returns:
        bool: 停止命令是否执行成功
        - True: 停止成功
        - False: 停止失败
        
    主要功能:
    1. 等待停止服务可用
    2. 调用停止服务
    3. 异常处理
    """
    service_name = '/stop_plan_arm_trajectory'
    rospy.wait_for_service(service_name)
    try:
        stop_service = rospy.ServiceProxy(service_name, stopPlanArmTrajectory)
        res = stop_service()
        return res.result
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False

def create_action_data():
    """
    直接创建动作数据，而不是从.tact文件读取
    
    Returns:
        list: 包含关键帧数据的列表
    """
    frames = [
        {
            "servos": [20,0,0,-30,0,0,0,20,0,0,-30,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
            "keyframe": 50,
            "attribute": {str(i+1): {"CP": [[0,0],[21,0]]} for i in range(28)}
        },
        {
            "servos": [20,0,0,-30,0,0,0,-30,0,30,-88,8,-22,-4,0,0,0,0,0,0,0,0,90,90,90,90,0,0],
            "keyframe": 120,
            "attribute": {str(i+1): {"CP": [[-21,0],[22.8,0]]} for i in range(28)}
        },
        # 可以继续添加更多关键帧...
    ]
    return {"frames": frames}

def main():
    """
    主函数修改
    """
    # 初始化ROS节点
    rospy.init_node('arm_trajectory_bezier_demo')
    
    # 创建订阅者和发布者
    rospy.Subscriber('/bezier/arm_traj', JointTrajectory, traj_callback, queue_size=1, tcp_nodelay=True)
    rospy.Subscriber('/sensors_data_raw', sensorsData, sensors_data_callback, queue_size=1, tcp_nodelay=True)
    kuavo_arm_traj_pub = rospy.Publisher('/kuavo_arm_traj', JointState, queue_size=1, tcp_nodelay=True)
    control_hand_pub = rospy.Publisher('/control_robot_hand_position', robotHandPosition, queue_size=1, tcp_nodelay=True)
    control_head_pub = rospy.Publisher('/robot_head_motion_data', robotHeadMotionData, queue_size=1, tcp_nodelay=True)
    
    # 切换手臂控制模式为双臂控制
    call_change_arm_ctrl_mode_service(2)
    
    # 直接创建动作数据，替代从文件读取
    data = create_action_data()

    # 生成轨迹规划请求
    action_data = add_init_frame(data["frames"])
    req = create_bezier_request(filter_data(action_data))

    # 执行轨迹规划
    rospy.loginfo("Planning arm trajectory...")
    success = plan_arm_trajectory_bezier_curve_client(req)
    if success:
        rospy.loginfo("Arm trajectory planned successfully")
    else:
        rospy.logerr("Failed to plan arm trajectory")

    while kuavo_arm_traj_pub.get_num_connections() == 0 and not rospy.is_shutdown():
        rospy.loginfo("Waiting for kuavo_arm_traj_pub subscriber...")
        rospy.sleep(0.1)

    # 以100Hz的频率发布轨迹数据
    rate = 100
    while not rospy.is_shutdown():
        try:
            global joint_state, hand_state, head_state
            if len(joint_state.position) == 0:
                continue
            kuavo_arm_traj_pub.publish(joint_state)
            control_hand_pub.publish(hand_state)
            control_head_pub.publish(head_state)
        except Exception as e:
            rospy.logerr(f"Failed to publish arm trajectory: {e}")
        except KeyboardInterrupt:
            break
        rospy.sleep(1/rate)

if __name__ == "__main__":
    main()

