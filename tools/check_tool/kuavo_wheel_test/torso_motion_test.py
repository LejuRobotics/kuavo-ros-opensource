#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
躯干往复运动测试程序
控制躯干在可工作空间内往复运动：
- Z轴上下30CM
- X轴前后10CM  
- pitch前倾20度
"""

import rospy, rospkg
import json
import math
import numpy as np
import time
import os
from geometry_msgs.msg import Quaternion
from kuavo_msgs.srv import lbBaseLinkPoseCmdSrv, lbBaseLinkPoseCmdSrvRequest
from kuavo_msgs.srv import lbLegControlSrv, lbLegControlSrvRequest
from sensor_msgs.msg import JointState
from kuavo_msgs.msg import sensorsData
from kuavo_msgs.srv import changeArmCtrlMode, changeArmCtrlModeRequest


def euler_to_quaternion(roll, pitch, yaw):
    """
    将欧拉角转换为四元数
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return [qw, qx, qy, qz]


def load_torso_motion_config():
    """
    从kuavo.json配置文件加载躯干运动参数
    如果找不到配置文件或参数，使用默认值
    
    配置选项说明：
    - z_range: Z轴运动范围(度)，从配置文件中的z_range数组获取
    - x_range: X轴运动范围(度)，从配置文件中的x_range数组获取  
    - pitch_range: pitch角度范围(度)，从配置文件中的pitch_range数组获取
    """
    # 默认配置
    default_config = {
        "z_range": [0.74, 0.94],  # Z轴范围 [min, max]
        "x_range": [0.1, 0.2],  # X轴范围 [min, max]
        "pitch_range": [0.0, 20.0],  # pitch角度范围 [min, max]
        "base_position": [0.05, 0.0, 0.74],  # 初始base_link位置
        "base_orientation": [1.0, 0.0, 0.0, 0.0],  # 初始base_link姿态(四元数)
        "motion_duration": 5.0,  # 每个控制段停留时间(秒)
        "cycle_count": 1,  # 循环次数
    }

    robot_version = os.getenv("ROBOT_VERSION", 60)
    if robot_version is None:
        rospy.logwarn("ROBOT_VERSION 环境变量未设置，使用默认配置")
        return default_config
    else:
        rospy.loginfo(f"ROBOT_VERSION 环境变量设置为: {robot_version}")
    
    # 尝试加载kuavo.json配置文件
    # 使用rospack找到kuavoassets包路径
    try:
        rospack = rospkg.RosPack()
        kuavo_assets_path = rospack.get_path('kuavo_assets')
        kuavo_config_paths = os.path.join(kuavo_assets_path, 'config', f'kuavo_v{robot_version}', 'kuavo.json')
        
        rospy.loginfo(f"搜索配置文件路径: {kuavo_config_paths}")
    except rospkg.common.ResourceNotFound:
        rospy.logwarn("未找到kuavoassets包，使用默认配置")
        return default_config

    if os.path.exists(kuavo_config_paths):
        try:
            with open(kuavo_config_paths, 'r') as f:
                kuavo_config = json.load(f)
            
            # 查找躯干运动配置，如果存在则覆盖默认值
            if "kuavo_wheel_torso_limit" in kuavo_config:
                torso_config = kuavo_config["kuavo_wheel_torso_limit"]
                if "z_range" in torso_config:
                    default_config["z_range"] = torso_config["z_range"]
                if "x_range" in torso_config:
                    default_config["x_range"] = torso_config["x_range"]
                if "pitch_range" in torso_config:
                    default_config["pitch_range"] = torso_config["pitch_range"]
                if "base_position" in torso_config:
                    default_config["base_position"] = torso_config["base_position"]
                pass
            
            rospy.loginfo(f"成功加载配置文件: {kuavo_config_paths}")
        
        except Exception as e:
            rospy.logwarn(f"加载配置文件失败: {kuavo_config_paths}, 错误: {e}")
    else:
        rospy.loginfo("未找到kuavo.json配置文件，使用默认配置")
    
    return default_config


def torso_motion_ik_service(base_link_pose, with_chassis=True, chassis_info=[0.0, 0.0, 0.0], q_lb=[0.0, 0.0, 0.0, 0.0]):
    """
    调用腿部IK服务控制躯干运动
    
    Args:
        base_link_pose: [x, y, z, qw, qx, qy, qz] - 目标base_link位姿
        with_chassis: 是否包含底盘信息
        chassis_info: 底盘信息 [x, y, yaw]
        q_lb: 腿部关节角度 [knee, leg, waist_pitch, waist_yaw]
    
    Returns:
        bool: 是否成功执行
    """
    try:
        # 等待服务可用
        rospy.wait_for_service('/lb_leg_ik_srv', timeout=10.0)
        rospy.wait_for_service('/lb_leg_control_srv', timeout=10.0)
        
        # 创建服务代理
        ik_service = rospy.ServiceProxy('/lb_leg_ik_srv', lbBaseLinkPoseCmdSrv)
        control_service = rospy.ServiceProxy('/lb_leg_control_srv', lbLegControlSrv)
        
        # 创建IK请求
        request = lbBaseLinkPoseCmdSrvRequest()
        request.with_chassis = with_chassis
        request.chassis_info = chassis_info
        request.q_lb = q_lb
        request.base_link = base_link_pose
        request.control_type = 1
        
        rospy.loginfo(f"发送IK请求: base_link={base_link_pose}")
        
        # 调用IK服务
        ik_response = ik_service(request)
        
        if ik_response.success:
            rospy.loginfo(f"IK求解成功，耗时: {ik_response.time_cost:.2f} ms")
            rospy.loginfo(f"腿部关节角度: {ik_response.lb_leg}")
            
            # 调用控制服务执行运动
            control_request = lbLegControlSrvRequest()
            control_request.target_joints = ik_response.lb_leg
            
            control_response = control_service(control_request)
            
            if control_response.success:
                rospy.loginfo("控制服务调用成功")
                return True
            else:
                rospy.logwarn("控制服务调用失败")
                return False
            return True
        else:
            rospy.logwarn(f"IK求解失败，耗时: {ik_response.time_cost:.2f} ms")
            return False
            
    except rospy.ServiceException as e:
        rospy.logerr(f"服务调用失败: {e}")
        return False


def get_services():
    """获取并返回IK与控制服务代理（仅创建一次）"""
    rospy.wait_for_service('/lb_leg_ik_srv', timeout=10.0)
    rospy.wait_for_service('/lb_leg_control_srv', timeout=10.0)
    ik_service = rospy.ServiceProxy('/lb_leg_ik_srv', lbBaseLinkPoseCmdSrv)
    control_service = rospy.ServiceProxy('/lb_leg_control_srv', lbLegControlSrv)
    rospy.loginfo("服务已连接: /lb_leg_ik_srv, /lb_leg_control_srv")
    return ik_service, control_service


def compute_ik_only(ik_service, base_link_pose, with_chassis=True, chassis_info=[0.0, 0.0, 0.0], q_lb=[0.0, 0.0, 0.0, 0.0]):
    """仅进行IK求解，返回(success, lb_leg, time_ms)"""
    req = lbBaseLinkPoseCmdSrvRequest()
    req.with_chassis = with_chassis
    req.chassis_info = chassis_info
    req.q_lb = q_lb
    req.base_link = base_link_pose
    req.control_type = 1
    try:
        resp = ik_service(req)
        if getattr(resp, 'success', False):
            return True, list(resp.lb_leg), getattr(resp, 'time_cost', 0.0)
        else:
            return False, None, getattr(resp, 'time_cost', 0.0)
    except Exception as e:
        rospy.logwarn(f"IK调用异常: {e}")
        return False, None, 0.0


def precompute_grid(config):
    """
    预计算在多个Z高度分段、每段内X方向范围采样得到的全部IK结果。
    在每个位置测试多个pitch角度。
    返回：list(dict(pose=[...7], lb_leg=[4], z_idx=int, x_idx=int, pitch_idx=int))
    """
    base_x, base_y, base_z = config["base_position"]
    x_half = config["x_range"] / 2.0
    z_half = config["z_range"] / 2.0

    z_min = 0.7
    z_max = 0.9
    x_min = 0
    x_max = 0.1

    # 使用固定步长模式
    z_step = config.get("z_step", 0.1)
    x_step = config.get("x_step", 0.1)
    
    # 计算基于步长的分段数
    z_levels = max(1, int((z_max - z_min) / z_step) + 1)
    x_samples = max(2, int((x_max - x_min) / x_step) + 1)
    
    # 获取pitch测试配置
    pitch_range = config.get("pitch_range", 20.0)  # 默认±20度
    pitch_step = config.get("pitch_step", 5.0)     # 默认5度步长
    
    # 计算基于步长的pitch角度数
    pitch_samples = max(1, int((2 * pitch_range) / pitch_step) + 1)
    
    rospy.loginfo(f"使用固定步长模式: Z步长={z_step:.3f}m, X步长={x_step:.3f}m")
    rospy.loginfo(f"预计算参数: z_range=[{z_min:.3f}, {z_max:.3f}] levels={z_levels} (step={z_step:.3f}m); x_range=[{x_min:.3f}, {x_max:.3f}] samples={x_samples} (step={x_step:.3f}m)")
    rospy.loginfo(f"Pitch测试: 范围±{pitch_range}度, 步长{pitch_step}度, 采样数{pitch_samples}")

    ik_service, _ = get_services()

    precomputed = []
    total = z_levels * x_samples * pitch_samples
    success_cnt = 0
    fail_cnt = 0

    for zi, z in enumerate(np.linspace(z_min, z_max, z_levels)):
        level_success = 0
        level_fail = 0
        rospy.loginfo(f"开始Z层[{zi+1}/{z_levels}] z={z:.3f}m 的X扫描...")
        
        for xi, x in enumerate(np.linspace(x_min, x_max, x_samples)):
            # 在每个位置测试多个pitch角度
            for pi, pitch in enumerate(np.arange(0.0, pitch_range + pitch_step, pitch_step)):
                # 生成姿态（绕Y轴pitch角度）
                qw, qx, qy, qz = euler_to_quaternion(0.0, np.radians(pitch), 0.0)
                
                pose = [x, base_y, max(0.7, z), qw, qx, qy, qz]
                ok, lb_leg, tcost = compute_ik_only(ik_service, pose)
                
                if ok:
                    success_cnt += 1
                    level_success += 1
                    precomputed.append({
                        'pose': pose,
                        'lb_leg': lb_leg,
                        'z_idx': zi,
                        'x_idx': xi,
                        'pitch_idx': pi,
                        'pitch_deg': pitch,
                        'time_ms': tcost
                    })
                    rospy.loginfo(f"IK OK z[{zi}/{z_levels-1}] x[{xi}/{x_samples-1}] pitch[{pi}/{pitch_samples-1}]={pitch:.1f}° pose={pose} t={tcost:.1f}ms")
                else:
                    fail_cnt += 1
                    level_fail += 1
                    rospy.logwarn(f"IK FAIL z[{zi}/{z_levels-1}] x[{xi}/{x_samples-1}] pitch[{pi}/{pitch_samples-1}]={pitch:.1f}° pose={pose}")
        
        rospy.loginfo(f"完成Z层[{zi+1}/{z_levels}] z={z:.3f}m：成功{level_success} 失败{level_fail}")

    rospy.loginfo(f"预计算完成: 总数={total}, 成功={success_cnt}, 失败={fail_cnt}")
    return precomputed


def return_to_initial_position(control_service, config):
    """
    回到初始位置（base位置，pitch=0度）
    
    Args:
        control_service: 控制服务代理
        config: 配置字典
    
    Returns:
        bool: 是否成功回到初始位置
    """
    try:
        rospy.loginfo("开始回到初始位置...")
        
        # 获取初始位置信息
        base_x, base_y, base_z = config["base_position"]
        base_qw, base_qx, base_qy, base_qz = config["base_orientation"]
        
        # 创建初始位置姿态
        initial_pose = [base_x, base_y, base_z, base_qw, base_qx, base_qy, base_qz]
        rospy.loginfo(f"目标初始位置: {initial_pose}")
        
        # 调用IK服务计算初始位置的关节角度
        ik_service, control_service_ = get_services()
        ok, lb_leg, tcost = compute_ik_only(ik_service, initial_pose)
        
        if not ok:
            rospy.logwarn("计算初始位置IK失败")
            return False
        
        rospy.loginfo(f"初始位置IK计算成功，关节角度: {lb_leg}")
        
        # 执行回到初始位置
        control_req = lbLegControlSrvRequest()
        control_req.target_joints = lb_leg
        
        rospy.loginfo("执行回到初始位置...")
        resp = control_service_(control_req)
        
        if not getattr(resp, 'success', False):
            rospy.logwarn("回到初始位置控制失败")
            return False
        
        rospy.loginfo("成功回到初始位置")
        time.sleep(2)  # 等待2秒确保稳定
        return True
        
    except Exception as e:
        rospy.logerr(f"回到初始位置过程中发生异常: {e}")
        return False


def publish_arm_target_pose_if_configured(config):
    """
    使用rospy发布一次 /kuavo_arm_traj (sensor_msgs/JointState)。
    position单位使用度(与示例一致)。
    """
    try:
        target_q_deg = [float(v) for v in [0,30,0,0,0,0,0, 0,-30,0,0,0,0,0]]

        sensed = read_arm_start_from_sensors(timeout_sec=1.0)
        if sensed is None: return False
        start_q_deg = [float(v) for v in sensed]
        topic = "/kuavo_arm_traj"
        duration = 2.0
        rate_hz = 50.0
        if not target_q_deg:
            rospy.loginfo("未配置arm_target_q_deg，跳过手臂目标位姿发布")
            return True
        if len(target_q_deg) != 14 or len(start_q_deg) != 14:
            rospy.logwarn(f"arm_start/target_q_deg长度不为14: start={len(start_q_deg)} target={len(target_q_deg)}")
        pub = rospy.Publisher(topic, JointState, queue_size=10)
        # 等待连接
        timeout_time = rospy.Time.now() + rospy.Duration(2.0)
        rate = rospy.Rate(rate_hz)
        while pub.get_num_connections() == 0 and not rospy.is_shutdown():
            if rospy.Time.now() > timeout_time:
                break
            rospy.sleep(0.01)
        # 线性插值步数
        steps = max(1, int(duration * rate_hz))
        for i in range(steps + 1):
            alpha = i / float(steps)
            q_now = [(1.0 - alpha) * start_q_deg[j] + alpha * target_q_deg[j] for j in range(min(14, len(start_q_deg), len(target_q_deg)))]
            # 填充/截断到14
            if len(q_now) < 14:
                q_now = q_now + [0.0] * (14 - len(q_now))
            elif len(q_now) > 14:
                q_now = q_now[:14]
            msg = JointState()
            msg.header.stamp = rospy.Time.now()
            msg.name = [f"arm_joint_{j}" for j in range(1, 15)]
            msg.position = q_now
            pub.publish(msg)
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break
        rospy.loginfo(f"已按插值方式发布手臂目标位姿到{topic}，时长{duration}s，频率{rate_hz}Hz，步数{steps}")
        return True
    except Exception as e:
        rospy.logwarn(f"发布手臂目标位姿异常: {e}")
        return False


def read_arm_start_from_sensors(timeout_sec=1.0):
    """
    阻塞读取一次 /sensors_data_raw，返回14个手臂关节角(度)。
    假设 joint_data.joint_q 中前14项为双臂关节（单位：度）。
    读取失败则返回 None。
    """
    try:
        msg = rospy.wait_for_message('/sensors_data_raw', sensorsData, timeout=timeout_sec)
        q = list(msg.joint_data.joint_q)
        if len(q) < 14:
            rospy.logwarn(f"/sensors_data_raw joint_q 长度不足: {len(q)}")
            return None
        return [math.degrees(v) for v in q[4:18]]
    except Exception as e:
        rospy.logwarn(f"读取/sensors_data_raw失败: {e}")
        return None


def change_arm_control_mode(target_mode):
    """
    切换手臂控制模式
    
    Args:
        target_mode: 目标控制模式 (1: 正常模式, 2: 外部控制模式)
    
    Returns:
        bool: 是否成功切换
    """
    try:
        # 等待服务可用
        rospy.wait_for_service('/change_arm_ctrl_mode', timeout=10.0)
        
        # 创建服务代理
        arm_mode_service = rospy.ServiceProxy('/change_arm_ctrl_mode', changeArmCtrlMode)
        
        # 创建请求
        request = changeArmCtrlModeRequest()
        request.control_mode = target_mode
        
        rospy.loginfo(f"切换手臂控制模式到: {target_mode}")
        
        # 调用服务
        response = arm_mode_service(request)
        
        if response.result:
            rospy.loginfo(f"手臂控制模式切换成功: {response.mode}, 消息: {response.message}")
            return True
        else:
            rospy.logwarn(f"手臂控制模式切换失败: {response.message}")
            return False
            
    except rospy.ServiceException as e:
        rospy.logerr(f"手臂控制模式切换服务调用失败: {e}")
        return False
    except Exception as e:
        rospy.logerr(f"手臂控制模式切换过程中发生异常: {e}")
        return False


def compute_limit_positions(config):
    """
    计算最大限制位置，每个高度都测试X轴范围，每个位置都测试前倾
    不测试base位置，只测试位置
    返回：list(dict(pose=[...7], lb_leg=[4], position_name=str))
    """
    base_x, base_y, base_z = config["base_position"]
    z_range = config["z_range"]
    x_range = config["x_range"] 
    pitch_range = config["pitch_range"]
    
    rospy.loginfo(f"计算位置: z_range={z_range}, x_range={x_range}, pitch_range={pitch_range}")
    
    ik_service, _ = get_services()
    precomputed = []
    
    # 定义要测试的Z高度（不包含base高度）
    z_heights = [
        {
            "name": "Z轴最低高度", 
            "z": z_range[0]
        },
        {
            "name": "Z轴最高高度",
            "z": z_range[1]
        }
    ]
    
    # 定义要测试的X位置（不包含base X位置）
    x_positions = [
        {
            "name": "X轴最前",
            "x": x_range[0]
        },
        {
            "name": "X轴最后", 
            "x": x_range[1]
        }
    ]
    
    success_cnt = 0
    fail_cnt = 0
    
    # 为每个Z高度测试X轴范围
    for z_info in z_heights:
        rospy.loginfo(f"计算Z高度: {z_info['name']} (z={z_info['z']:.3f}m)")
        
        for x_info in x_positions:
            rospy.loginfo(f"  计算X位置: {x_info['name']} (x={x_info['x']:.3f}m)")
            
            # 测试pitch角度：0度、最大前倾、最大后倾
            pitch_angles = [pitch_range[1], pitch_range[0]]
            
            for pitch in pitch_angles:
                # 生成姿态（绕Y轴pitch角度）
                if pitch == 0.0:
                    pose = [x_info['x'], base_y, z_info['z'], 1.0, 0.0, 0.0, 0.0]
                    position_name = f"{z_info['name']} {x_info['name']} (pitch=0°)"
                else:
                    qw, qx, qy, qz = euler_to_quaternion(0.0, np.radians(pitch), 0.0)
                    pose = [x_info['x'], base_y, z_info['z'], qw, qx, qy, qz]
                    position_name = f"{z_info['name']} {x_info['name']} (pitch={pitch:.1f}°)"
                
                rospy.loginfo(f"    计算姿态: {position_name}, pose={pose}")
                
                ok, lb_leg, tcost = compute_ik_only(ik_service, pose)
                
                if ok:
                    success_cnt += 1
                    precomputed.append({
                        'pose': pose,
                        'lb_leg': lb_leg,
                        'position_name': position_name,
                        'time_ms': tcost
                    })
                    rospy.loginfo(f"    IK成功: {position_name}, 耗时: {tcost:.1f}ms")
                else:
                    fail_cnt += 1
                    rospy.logwarn(f"    IK失败: {position_name}")
    
    rospy.loginfo(f"位置计算完成: 成功={success_cnt}, 失败={fail_cnt}")
    return precomputed


def execute_limit_positions(precomputed, motion_duration, config):
    """按顺序执行位置，最后回到初始位置"""
    if not precomputed:
        rospy.logwarn("无位置可执行")
        return False

    _, control_service = get_services()

    rospy.loginfo(f"开始执行位置序列，共 {len(precomputed)} 个位置；每位置停留 {motion_duration}s")

    for idx, item in enumerate(precomputed):
        control_req = lbLegControlSrvRequest()
        control_req.target_joints = item['lb_leg']
        
        rospy.loginfo(f"执行第 {idx+1}/{len(precomputed)} 个位置: {item['position_name']}")
        rospy.loginfo(f"目标姿态: {item['pose']}")
        rospy.loginfo(f"关节角度: {item['lb_leg']}")
        
        try:
            resp = control_service(control_req)
            if not getattr(resp, 'success', False):
                rospy.logwarn(f"控制失败: {item['position_name']}")
                return False
        except Exception as e:
            rospy.logerr(f"控制服务异常: {e}")
            return False
        
        rospy.loginfo(f"位置 {item['position_name']} 执行完成，休息 {motion_duration}s ...")
        time.sleep(motion_duration)

    rospy.loginfo("所有位置测试完成")
    return True


def run_torso_motion_test():
    """
    运行躯干位置测试
    """
    rospy.init_node('torso_motion_test', anonymous=True)
    rospy.loginfo("=== 躯干位置测试程序启动 ===")
    
    # 加载配置
    config = load_torso_motion_config()
    rospy.loginfo(f"运动配置: {config}")
    
    # 在开始移动前，先切换手臂控制模式到模式2
    rospy.loginfo("准备切换手臂控制模式到外部控制模式...")
    if not change_arm_control_mode(2):
        rospy.logerr("手臂控制模式切换到外部控制模式失败，退出程序")
        return False
    else:
        rospy.loginfo("手臂控制模式已切换到外部控制模式，开始执行躯干测试")
    
    # 在开始移动前，先发布手臂目标位姿（如果配置了）
    if not publish_arm_target_pose_if_configured(config):
        rospy.logwarn("手臂目标位姿发布失败，继续执行躯干测试")
    
    # 先移动到base位置
    rospy.loginfo("先移动到base位置...")
    if not return_to_initial_position(None, config):
        rospy.logwarn("移动到base位置失败，继续执行测试")
    
    # 计算位置
    precomputed = compute_limit_positions(config)
    if not precomputed:
        rospy.logerr("位置计算全部失败，退出")
        # 即使失败也要尝试切换回手臂控制模式1
        rospy.loginfo("尝试切换手臂控制模式回正常模式...")
        change_arm_control_mode(1)
        return False
    rospy.loginfo("位置计算完成，开始执行...")
    
    # 执行位置序列
    ok = execute_limit_positions(precomputed, config["motion_duration"], config)
    if not ok:
        rospy.logwarn("执行阶段失败")
    
    # 最后回到base位置
    rospy.loginfo("测试完成，准备回到base位置...")
    if not return_to_initial_position(None, config):
        rospy.logwarn("回到base位置失败")
    
    # 测试完成后，切换手臂控制模式回模式1
    rospy.loginfo("准备切换手臂控制模式回正常模式...")
    if not change_arm_control_mode(1):
        rospy.logwarn("手臂控制模式切换回模式1失败")
    else:
        rospy.loginfo("手臂控制模式已切换回模式1")
    
    if ok:
        rospy.loginfo("=== 躯干位置测试完成 ===")
    else:
        rospy.logwarn("=== 躯干位置测试未能完成 ===")
    
    return ok


def main():
    """
    主函数
    """
    try:
        success = run_torso_motion_test()
        if success:
            rospy.loginfo("测试成功完成")
        else:
            rospy.logwarn("测试未能完成")
    except KeyboardInterrupt:
        rospy.loginfo("用户中断测试")
    except Exception as e:
        rospy.logerr(f"测试过程中发生错误: {e}")


if __name__ == "__main__":
    main() 