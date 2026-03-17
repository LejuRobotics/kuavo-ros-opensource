#!/usr/bin/env python3
"""
lb_ctrl_api  ——  轮臂常用服务调用封装
"""
import rospy
from kuavo_msgs.srv import changeTorsoCtrlMode, changeTorsoCtrlModeRequest
from kuavo_msgs.srv import changeLbQuickModeSrv, changeLbQuickModeSrvRequest
from kuavo_msgs.srv import getLbTorsoInitialPose, getLbTorsoInitialPoseRequest
from kuavo_msgs.srv import setRuckigPlannerParams, setRuckigPlannerParamsRequest
from kuavo_msgs.srv import changeArmCtrlMode, changeArmCtrlModeRequest
from kuavo_msgs.srv import changeLbMpcObsUpdateModeSrv, changeLbMpcObsUpdateModeSrvRequest
from kuavo_msgs.srv import lbTimedPosCmd, lbTimedPosCmdRequest
from kuavo_msgs.srv import lbMultiTimedPosCmd, lbMultiTimedPosCmdRequest
from kuavo_msgs.srv import lbMultiTimedOfflineTraj, lbMultiTimedOfflineTrajRequest
from kuavo_msgs.msg import timedPoint, lbTimedOfflineTraj
from kuavo_msgs.msg import timedSingleCmd
from std_srvs.srv import SetBool, SetBoolRequest
from std_msgs.msg import Bool

def set_control_mode(target_mode: int) -> bool:
    """
    设置移动机械臂控制模式

    :param target_mode: 0~4 对应不同模式
    :return: True 成功，False 失败
    """
    MODES = {
        0: "NoControl",
        1: "ArmOnly",
        2: "BaseOnly",
        3: "BaseArm",
        4: "ArmEeOnly",
    }
    if target_mode not in MODES:
        rospy.logerr(f"无效模式号 {target_mode}，允许值 {list(MODES.keys())}")
        return False

    try:
        rospy.wait_for_service('/mobile_manipulator_mpc_control', timeout=5.0)
        client = rospy.ServiceProxy('/mobile_manipulator_mpc_control', changeTorsoCtrlMode)
        req = changeTorsoCtrlModeRequest()
        req.control_mode = target_mode
        resp = client(req)
        if resp.result:
            rospy.loginfo(f"✅ 已切换到模式 {target_mode}: {MODES[target_mode]}")
            return True
        else:
            rospy.logerr(f"❌ 切换失败: {resp.message}")
            return False
    except rospy.ROSException as e:
        rospy.logerr(f"❌ 服务等待超时: {e}")
        return False
    except rospy.ServiceException as e:
        rospy.logerr(f"❌ 服务调用失败: {e}")
        return False
    except Exception as e:
        rospy.logerr(f"❌ 未知错误: {e}")
        return False

def set_arm_control_mode(control_mode: int) -> bool:
    """
    设置手臂控制模式

    :param control_mode: 手臂控制模式
        0: 保持当前位置控制
        1: 重置手臂到初始目标位置
        2: 使用外部控制器
    :return: True 成功，False 失败
    """
    # 模式定义字典
    MODES = {
        0: "保持当前位置控制 (Keep current control position)",
        1: "重置手臂到初始目标位置 (Reset arm to initial target)",
        2: "使用外部控制器 (Using external controller)"
    }
    
    # 验证输入模式
    if control_mode not in MODES:
        rospy.logerr(f"❌ 无效的手臂控制模式: {control_mode}，允许值: {list(MODES.keys())}")
        return False
    
    try:
        # 等待服务
        rospy.wait_for_service('/wheel_arm_change_arm_ctrl_mode', timeout=5.0)
        client = rospy.ServiceProxy('/wheel_arm_change_arm_ctrl_mode', changeArmCtrlMode)
        
        # 准备请求
        req = changeArmCtrlModeRequest()
        req.control_mode = control_mode
        
        # 调用服务
        resp = client(req)
        
        if resp.result:
            rospy.loginfo(f"✅ 手臂控制模式设置成功")
            rospy.loginfo(f"   模式: {control_mode} - {MODES[control_mode]}")
            rospy.loginfo(f"   返回消息: {resp.message}")
            rospy.loginfo(f"   当前模式值: {resp.mode}")
            return True
        else:
            rospy.logerr(f"❌ 手臂控制模式设置失败: {resp.message}")
            return False
            
    except rospy.ROSException as e:
        rospy.logerr(f"❌ 服务等待超时: {e}")
        return False
    except rospy.ServiceException as e:
        rospy.logerr(f"❌ 服务调用失败: {e}")
        return False
    except Exception as e:
        rospy.logerr(f"❌ 未知错误: {e}")
        return False

def set_arm_quick_mode(quickMode):
    """
    设置手臂快速模式
    Args:
        全身快速模式类型: 0-关闭, 1-下肢快, 2-上肢快, 3-上下肢快
    """
    print(f"call set_arm_quick_mode:{quickMode}")
    rospy.wait_for_service('/enable_lb_arm_quick_mode')
    try:
        set_arm_quick_mode_service = rospy.ServiceProxy('/enable_lb_arm_quick_mode', changeLbQuickModeSrv)
        req = changeLbQuickModeSrvRequest()
        req.quickMode = quickMode
        resp = set_arm_quick_mode_service(req)
        if resp.success:
            rospy.loginfo(f"Successfully enabled {quickMode} quick mode")
        else:
            rospy.logwarn(f"Failed to enable {quickMode} quick mode")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False

def set_mpc_obs_update_mode(obs_update_mode: int) -> bool:
    """
    设置MPC观测更新模式

    :param obs_update_mode: MPC优化采用的反馈机制
        0: 全部反馈
        1: 屏蔽下肢电机反馈  
        2: 屏蔽上肢电机反馈
        3: 同时屏蔽上下肢电机反馈
    :return: True 成功，False 失败
    """
    # 模式定义字典
    MODES = {
        0: "全部反馈",
        1: "屏蔽下肢电机反馈",
        2: "屏蔽上肢电机反馈", 
        3: "同时屏蔽上下肢电机反馈"
    }
    
    # 验证输入模式
    if obs_update_mode not in MODES:
        rospy.logerr(f"❌ 无效的MPC观测更新模式: {obs_update_mode}，允许值: {list(MODES.keys())}")
        return False
    
    try:
        # 等待服务
        rospy.wait_for_service('/change_lb_mpc_obs_update_mode', timeout=5.0)
        client = rospy.ServiceProxy('/change_lb_mpc_obs_update_mode', changeLbMpcObsUpdateModeSrv)
        
        # 准备请求
        req = changeLbMpcObsUpdateModeSrvRequest()
        req.obsUpdateMode = obs_update_mode
        
        # 调用服务
        resp = client(req)
        
        if resp.success:
            rospy.loginfo(f"✅ MPC观测更新模式设置成功")
            rospy.loginfo(f"   模式: {obs_update_mode} - {MODES[obs_update_mode]}")
            rospy.loginfo(f"   返回消息: {resp.message}")
            return True
        else:
            rospy.logerr(f"❌ MPC观测更新模式设置失败: {resp.message}")
            return False
            
    except rospy.ROSException as e:
        rospy.logerr(f"❌ 服务等待超时: {e}")
        return False
    except rospy.ServiceException as e:
        rospy.logerr(f"❌ 服务调用失败: {e}")
        return False
    except Exception as e:
        rospy.logerr(f"❌ 未知错误: {e}")
        return False

def get_torso_initial_pose(need_pose=True):
    """
    获取躯干初始位姿

    :param need_pose: 是否需要获取位姿，如果为False则返回失败
    :return: (success, pose_dict) 
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


def set_ruckig_planner_params(planner_index: int,
                              is_sync: bool,
                              velocity_max: list,
                              acceleration_max: list,
                              jerk_max: list,
                              velocity_min: list = None,
                              acceleration_min: list = None) -> bool:
    """
    设置Ruckig规划器参数

    :param planner_index: 规划器索引
    :param is_sync: 是否同步模式
    :param velocity_max: 最大速度列表
    :param acceleration_max: 最大加速度列表
    :param jerk_max: 最大急动度列表
    :param velocity_min: 最小速度列表（可选）
    :param acceleration_min: 最小加速度列表（可选）
    :return: True 成功，False 失败
    """
    try:
        rospy.wait_for_service('/mobile_manipulator_set_ruckig_planner_params', timeout=5.0)
        client = rospy.ServiceProxy('/mobile_manipulator_set_ruckig_planner_params', setRuckigPlannerParams)
        req = setRuckigPlannerParamsRequest()
        req.planner_index = planner_index
        req.is_sync = is_sync
        req.velocity_max = velocity_max
        req.acceleration_max = acceleration_max
        req.jerk_max = jerk_max
        if velocity_min is not None:
            req.velocity_min = velocity_min
        if acceleration_min is not None:
            req.acceleration_min = acceleration_min

        resp = client(req)
        if resp.result:
            rospy.loginfo(f"✅ 规划器参数设置成功: {resp.message}")
            return True
        else:
            rospy.logerr(f"❌ 规划器参数设置失败: {resp.message}")
            return False
    except rospy.ROSException as e:
        rospy.logerr(f"❌ 服务等待超时: {e}")
        return False
    except rospy.ServiceException as e:
        rospy.logerr(f"❌ 服务调用失败: {e}")
        return False
    except Exception as e:
        rospy.logerr(f"❌ 未知错误: {e}")
        return False

def send_timed_single_command(planner_index: int, 
                             desire_time: float, 
                             cmd_vec: list) -> tuple:
    """
    发送单次定时指令到移动机械臂
    
    :param planner_index: 规划器索引  
        0: 底盘世界系位置运动  
        1: 底盘局部系位置运动  
        2: 躯干笛卡尔局部系运动  
        3: 下肢关节运动  
        4: 左臂笛卡尔世界系运动
        5: 右臂笛卡尔世界系运动  
        6: 左臂笛卡尔局部系运动    
        7: 右臂笛卡尔局部系运动  
        8: 左臂上肢关节运动
        9: 右臂上肢关节运动  
    :param desire_time: 期望执行时间（秒）
    :param cmd_vec: 命令向量列表，维度需与规划器匹配
    :return: (success, actual_time, message)
             success: 是否成功执行
             actual_time: 实际执行需要的时间
             message: 详细信息
    """
    # 规划器描述字典
    # PLANNER_DESCRIPTIONS = {
    #     0: "底盘世界系位置运动",
    #     1: "底盘局部系位置运动",
    #     2: "躯干笛卡尔局部系运动",
    #     3: "下肢关节运动",
    #     4: "左臂笛卡尔世界系运动", 
    #     5: "右臂笛卡尔世界系运动",
    #     6: "左臂笛卡尔局部系运动"
    #     7: "右臂笛卡尔局部系运动"
    #     8: "左臂上肢关节运动"
    #     9: "右臂上肢关节运动"
    # }
    
    # 验证规划器索引
    # if planner_index not in PLANNER_DESCRIPTIONS:
    #     rospy.logerr(f"❌ 无效的规划器索引: {planner_index}，允许值: {list(PLANNER_DESCRIPTIONS.keys())}")
    #     return False, 0.0, f"无效的规划器索引: {planner_index}"
    
    # 验证期望时间
    if desire_time <= 0:
        rospy.logwarn(f"⚠️  期望执行时间应大于0，当前值: {desire_time:.3f}")
    
    # 验证命令向量
    if not cmd_vec:
        rospy.logerr(f"❌ 命令向量为空")
        return False, 0.0, "命令向量为空"
    
    try:
        # 等待服务
        rospy.wait_for_service('/mobile_manipulator_timed_single_cmd', timeout=5.0)
        client = rospy.ServiceProxy('/mobile_manipulator_timed_single_cmd', lbTimedPosCmd)
        
        # 准备请求
        req = lbTimedPosCmdRequest()
        req.planner_index = planner_index
        req.desireTime = desire_time
        req.cmdVec = cmd_vec
        
        # 调用服务
        resp = client(req)
        
        if resp.isSuccess:
            rospy.loginfo(f"✅ 指令执行成功")
            rospy.loginfo(f"   实际时间: {resp.actualTime}s")
            rospy.loginfo(f"   消息: {resp.message}")
            return True, resp.actualTime
        else:
            rospy.logerr(f"❌ 指令执行失败")
            rospy.logerr(f"   消息: {resp.message}")
            return False, resp.actualTime
            
    except rospy.ROSException as e:
        error_msg = f"服务等待超时: {e}"
        rospy.logerr(f"❌ {error_msg}")
        return False, 0.0, error_msg
    except rospy.ServiceException as e:
        error_msg = f"服务调用失败: {e}"
        rospy.logerr(f"❌ {error_msg}")
        return False, 0.0, error_msg
    except Exception as e:
        error_msg = f"未知错误: {e}"
        rospy.logerr(f"❌ {error_msg}")
        return False, 0.0, error_msg

def send_timed_multi_commands(timed_cmd_vec: list, is_sync: bool = False) -> tuple:
    """
    发送多条定时指令到移动机械臂
    
    :param timed_cmd_vec: 定时指令列表，每个元素为字典格式：
        {
            'planner_index': int,    # 规划器索引
            'desire_time': float,     # 期望执行时间（秒）
            'cmd_vec': list           # 命令向量列表
        }
    :param is_sync: 多个规划器是否做时间同步
                    True: 做同步, 按同步时间执行
                    False: 不做同步, 按各自预设执行
    :return: (success, actual_time, message)
             success: 是否成功执行
             actual_time: 实际执行需要的时间
             message: 详细信息
    """
    
    # 验证指令列表
    if not timed_cmd_vec:
        rospy.logerr(f"❌ 指令列表为空")
        return False, 0.0, "指令列表为空"
    
    try:
        # 等待服务
        rospy.wait_for_service('/mobile_manipulator_timed_multi_cmd', timeout=5.0)
        client = rospy.ServiceProxy('/mobile_manipulator_timed_multi_cmd', lbMultiTimedPosCmd)
        
        # 准备请求
        req = lbMultiTimedPosCmdRequest()
        req.isSync = is_sync
        
        # 构建指令列表
        for cmd in timed_cmd_vec:
            single_cmd = timedSingleCmd()
            single_cmd.planner_index = cmd['planner_index']
            single_cmd.desireTime = cmd['desire_time']
            single_cmd.cmdVec = cmd['cmd_vec']
            req.timedCmdVec.append(single_cmd)
        
        # 调用服务
        resp = client(req)
        
        if resp.isSuccess:
            rospy.loginfo(f"✅ 多指令执行成功")
            rospy.loginfo(f"   实际时间: {resp.actualTime}s")
            rospy.loginfo(f"   消息: {resp.message}")
            rospy.loginfo(f"   同步模式: {'同步' if is_sync else '异步'}")
            rospy.loginfo(f"   指令数量: {len(timed_cmd_vec)}")
            return True, resp.actualTime, resp.message
        else:
            rospy.logerr(f"❌ 多指令执行失败")
            rospy.logerr(f"   消息: {resp.message}")
            return False, resp.actualTime, resp.message
            
    except rospy.ROSException as e:
        error_msg = f"服务等待超时: {e}"
        rospy.logerr(f"❌ {error_msg}")
        return False, 0.0, error_msg
    except rospy.ServiceException as e:
        error_msg = f"服务调用失败: {e}"
        rospy.logerr(f"❌ {error_msg}")
        return False, 0.0, error_msg
    except Exception as e:
        error_msg = f"未知错误: {e}"
        rospy.logerr(f"❌ {error_msg}")
        return False, 0.0, error_msg
    
def reset_torso_to_initial():
    """
    重置躯干到初始位姿
    Returns:
        float: 估计的复位时间（秒），如果失败返回0.0
    """
    rospy.wait_for_service('/mobile_manipulator_reset_torso')
    try:
        reset_service = rospy.ServiceProxy('/mobile_manipulator_reset_torso', SetBool)
        resp = reset_service(True)
        if resp.success:
            try:
                time_estimate = float(resp.message)
                rospy.loginfo(f"Torso reset initiated. Estimated time: {time_estimate:.2f}s")
                return time_estimate
            except ValueError:
                rospy.logwarn(f"Invalid time estimate: {resp.message}")
                return 0.0
        else:
            rospy.logwarn("Torso reset request denied")
            return 0.0
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return 0.0

def set_focus_ee(focus_ee):
    """
    设置笛卡尔跟踪焦点
    Args:
        focus_ee: True跟踪末端，False跟踪躯干
    """
    pub = rospy.Publisher('/mobile_manipulator_focus_ee', Bool, queue_size=10, latch=True)
    msg = Bool(data=focus_ee)
    pub.publish(msg)
    rospy.loginfo(f"Focus set to {'EE' if focus_ee else 'Torso'}")

def set_lb_multi_timed_offline_traj(offline_trajectories: list) -> tuple:
    """
    设置多条离线定时轨迹到移动机械臂
    
    :param offline_trajectories: 离线轨迹列表，每个元素为字典格式：
        {
            'planner_index': int,       # 规划器索引 0:左臂,1:右臂,2:躯干
            'frame': int,                # 坐标系 (0:世界系, 1:局部系)
            'timed_traj': list           # 定时轨迹点列表，每个点为字典：
                {
                    'desire_time': float,   # 期望执行时间(秒), 第一帧必须为0
                    'cmd_vec': list         # 命令向量，左/右臂6维, 躯干4维
                }
        }
    :return: (success, message)
    """
    # 基本验证
    if not offline_trajectories:
        return False, "轨迹数据为空"

    try:
        # 等待服务
        rospy.wait_for_service('/mobile_manipulator_timed_offline_traj', timeout=3.0)
        client = rospy.ServiceProxy('/mobile_manipulator_timed_offline_traj', lbMultiTimedOfflineTraj)
        
        # 准备请求 - 注意服务定义是 lbTimedOfflineTraj[] offlineTraj
        req = lbMultiTimedOfflineTrajRequest()
        
        # 处理每条轨迹
        for i, traj in enumerate(offline_trajectories):
            # 验证规划器索引
            if traj['planner_index'] not in [0, 1, 2]:
                return False, f"轨迹{i}规划器索引无效: {traj['planner_index']}"
            
            # 验证轨迹点
            if not traj['timed_traj']:
                return False, f"轨迹{i}轨迹点为空"
            
            # 创建离线轨迹对象 - 直接使用消息类型
            offline_traj = lbTimedOfflineTraj()
            offline_traj.plannerIndex = traj['planner_index']
            offline_traj.frame = traj.get('frame', 0)
            
            # 处理每个轨迹点
            for j, point in enumerate(traj['timed_traj']):
                # 验证必要字段
                if 'desire_time' not in point or 'cmd_vec' not in point:
                    return False, f"轨迹{i}点{j}缺少必要字段"
                
                # 验证第一帧时间
                if j == 0 and abs(point['desire_time']) > 1e-6:
                    return False, f"轨迹{i}第一帧时间不为0"
                
                # 验证时间递增
                if j > 0 and point['desire_time'] <= traj['timed_traj'][j-1]['desire_time']:
                    return False, f"轨迹{i}时间未严格递增"
                
                # 验证命令向量维度
                expected_dim = 6 if traj['planner_index'] in [0, 1] else 4
                if len(point['cmd_vec']) != expected_dim:
                    return False, f"轨迹{i}点{j}命令向量维度错误: 期望{expected_dim}维"
                
                # 创建定时点消息
                timed_cmd = timedPoint()
                timed_cmd.desireTime = point['desire_time']
                timed_cmd.cmdVec = point['cmd_vec']
                offline_traj.timedTraj.append(timed_cmd)
            
            # 将离线轨迹添加到请求
            req.offlineTraj.append(offline_traj)
        
        # 调用服务
        resp = client(req)
        
        if resp.isSuccess:
            rospy.loginfo(f"✅ 设置{len(offline_trajectories)}条离线轨迹成功")
            return True, resp.message
        else:
            return False, f"设置失败: {resp.message}"
            
    except rospy.ServiceException as e:
        return False, f"服务调用失败: {e}"
    except Exception as e:
        return False, f"未知错误: {e}"

def set_offline_trajectory_enable(enable: bool) -> bool:
    """
    启用或禁用离线轨迹功能
    
    :param enable: True启用离线轨迹，False禁用离线轨迹
    :return: bool 是否设置成功
    """
    rospy.wait_for_service('/mobile_manipulator_timed_offline_traj_enable')
    try:
        enable_service = rospy.ServiceProxy('/mobile_manipulator_timed_offline_traj_enable', SetBool)
        resp = enable_service(enable)
        
        if resp.success:
            status = "Enabled" if enable else "Disabled"
            rospy.loginfo(f"Offline trajectory {status}. Message: {resp.message}")
            return True
        else:
            rospy.logwarn(f"Offline trajectory enable request denied: {resp.message}")
            return False
            
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")
        return False