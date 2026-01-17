import rospy
import json
from humanoid_plan_arm_trajectory.srv import ExecuteArmAction, ExecuteArmActionRequest
from kuavo_msgs.msg import footPose, footPoseTargetTrajectories
import numpy as np
from utils.sat import RotatingRectangle  # 导入用于碰撞检测的工具类

# ==================== 辅助函数 (用于移动) ====================

def euler_to_rotation_matrix(yaw, pitch, roll):
    """将欧拉角转换为旋转矩阵"""
    R_yaw = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                      [np.sin(yaw), np.cos(yaw), 0],
                      [0, 0, 1]])
    R_pitch = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                        [0, 1, 0],
                        [-np.sin(pitch), 0, np.cos(pitch)]])
    R_roll = np.array([[1, 0, 0],
                       [0, np.cos(roll), -np.sin(roll)],
                       [0, np.sin(roll), np.cos(roll)]])
    R = np.dot(R_roll, np.dot(R_pitch, R_yaw))
    return R

def get_foot_pose_traj_msg(time_traj, foot_idx_traj, foot_traj, torso_traj):
    """
    创建并返回一个 footPoseTargetTrajectories 消息对象。
    
    参数：
    - time_traj: 时间轨迹列表
    - foot_idx_traj: 脚索引轨迹列表
    - foot_traj: 脚姿态轨迹列表
    - torso_traj: 躯干姿态轨迹列表
    
    返回：
    - footPoseTargetTrajectories 消息对象
    """
    msg = footPoseTargetTrajectories() # 创建消息实例
    msg.timeTrajectory = time_traj # 设置时间轨迹
    msg.footIndexTrajectory = foot_idx_traj # 设置脚索引
    msg.footPoseTrajectory = [] # 初始化脚姿态轨迹
    for i in range(len(time_traj)): # 遍历时间轨迹
        foot_pose_msg = footPose() # 创建脚姿态消息实例
        foot_pose_msg.footPose = foot_traj[i] # 设置脚姿态
        foot_pose_msg.torsoPose = torso_traj[i] # 设置躯干姿态
        msg.footPoseTrajectory.append(foot_pose_msg) # 添加到轨迹列表
    return msg # 返回消息对象

def generate_steps(torso_pos, torso_yaw, foot_bias):
    """
    根据躯干位置和偏航角生成左右脚的位置。
    
    参数：
    - torso_pos: 躯干位置
    - torso_yaw: 躯干偏航角（弧度）
    - foot_bias: 脚偏移量
    
    返回：
    - 左脚和右脚的位置
    """
    l_foot_bias = np.array([0, foot_bias, -torso_pos[2]])
    r_foot_bias = np.array([0, -foot_bias, -torso_pos[2]])
    R_z = np.array([
        [np.cos(torso_yaw), -np.sin(torso_yaw), 0],
        [np.sin(torso_yaw), np.cos(torso_yaw), 0],
        [0, 0, 1]
    ])
    l_foot = torso_pos + R_z.dot(l_foot_bias)
    r_foot = torso_pos + R_z.dot(r_foot_bias)
    return l_foot, r_foot

def get_multiple_steps_msg(body_poses:list, dt:float, is_left_first:bool=True, collision_check:bool=True):
    """生成多步移动消息
    
    Args:
        body_poses: 身体姿态列表 [[x, y, z, yaw_degrees], ...]
        dt: 时间步长
        is_left_first: 是否左脚先迈
        collision_check: 是否进行碰撞检测
        
    Returns:
        footPoseTargetTrajectories 消息
    """

    num_steps = 2 * len(body_poses)  # 总步数（左右脚各算一步）
    time_traj = []                   # 时间轨迹
    foot_idx_traj = []               # 脚索引轨迹（0=左脚，1=右脚）
    foot_traj = []                   # 脚姿态轨迹
    torso_traj = []                  # 躯干姿态轨迹
    l_foot_rect_last = RotatingRectangle(center=(0, 0.1), width=0.24, height=0.1, angle=0)
    r_foot_rect_last = RotatingRectangle(center=(0,-0.1), width=0.24, height=0.1, angle=0)
    torso_pose_last = np.array([0, 0, 0, 0])  # 上一次的躯干姿态（初始化为原点）
    
    for i in range(num_steps):
        # 计算当前时间戳
        time_traj.append(dt * (i + 1))  # 添加时间点
        body_pose = body_poses[i // 2]  # 获取当前身体姿态
        torso_pos = np.asarray(body_pose[:3])  # 躯干位置
        torso_yaw = np.radians(body_pose[3])  # 躯干偏航角（转换为弧度）

        # 左右脚的步态数据
        l_foot, r_foot = generate_steps(torso_pos, torso_yaw, 0.1)  # 生成左右脚位置
        l_foot = [*l_foot[:3], torso_yaw]  # 左脚姿态
        r_foot = [*r_foot[:3], torso_yaw]  # 右脚姿态

        # 偶数步
        if i % 2 == 0:
            torso_pose = np.array([*body_pose[:3], torso_yaw])
            R_wl = euler_to_rotation_matrix(torso_pose_last[3], 0, 0)

            # 计算躯干姿态相当于上一次的变化量
            delta_pos = R_wl.T @ (torso_pose[:3] - torso_pose_last[:3])
            
            # 根据偏航角或横向位移决定起始脚
            if torso_yaw > 0.0 or delta_pos[1] > 0.0:
                is_left_first = True
            else:
                is_left_first = False

        if(collision_check and i%2 == 0):
            l_foot_rect_next = RotatingRectangle(center=(l_foot[0],l_foot[1]), width=0.24, height=0.1, angle=torso_yaw)
            r_foot_rect_next = RotatingRectangle(center=(r_foot[0],r_foot[1]), width=0.24, height=0.1, angle=torso_yaw)
            l_collision = l_foot_rect_next.is_collision(r_foot_rect_last)
            r_collision = r_foot_rect_next.is_collision(l_foot_rect_last)
            if l_collision and r_collision:
                rospy.loginfo("[Control] 检测到碰撞，请调整身体姿态输入!!!")
                break
            elif l_collision:
                rospy.loginfo("[Control] 左脚碰撞，切换到右脚")
                is_left_first = False
            elif r_collision:
                rospy.loginfo("[Control] 右脚碰撞，切换到左脚")
                is_left_first = True
            l_foot_rect_last = l_foot_rect_next
            r_foot_rect_last = r_foot_rect_next
        
        # 步态交替逻辑
        if i % 2 == 0:
            # 偶数步：迈左脚或右脚
            torso_traj.append((torso_pose_last + torso_pose) / 2.0)
            if is_left_first:
                foot_idx_traj.append(0) # 左脚 
                foot_traj.append(l_foot)
            else:
                foot_idx_traj.append(1) # 右脚
                foot_traj.append(r_foot)
        else:
            # 奇数步：迈另一只脚
            torso_traj.append(torso_pose)
            if is_left_first:
                foot_idx_traj.append(1)
                foot_traj.append(r_foot)
            else:
                foot_idx_traj.append(0)
                foot_traj.append(l_foot)
        
        # 更新上一次的躯干姿态
        torso_pose_last = torso_traj[-1]
        # torso_traj.append([*body_pose[:3], torso_yaw])  # 添加躯干姿态
        # print("time_traj:", time_traj)
        # print("foot_idx_traj:", foot_idx_traj)
        # print("foot_traj:", foot_traj)
        # print("torso_traj:", torso_traj)
        
    return get_foot_pose_traj_msg(time_traj, foot_idx_traj, foot_traj, torso_traj)


# ==================== 回调处理类 ====================

class MotionExecutor:
    """ROS回调处理类，负责将指令转换为ROS动作和消息"""
    
    def __init__(self):
        """初始化回调处理器"""
        self.foot_pose_pub = None
        self.execute_action_proxy = None
        
        try:
            
            self._init_publishers()
            self._init_service_proxies()
            rospy.loginfo("ROS回调处理器已初始化")

        except Exception as e:
            rospy.logerr(f"ROS回调处理器初始化失败: {e}")
    
    def _init_publishers(self):
        """初始化所有发布者"""
        try:
            foot_pose_topic = '/humanoid_mpc_foot_pose_target_trajectories'
            self.foot_pose_pub = rospy.Publisher(foot_pose_topic, footPoseTargetTrajectories, queue_size=1)
            rospy.loginfo(f"[MOTION_EXECUTOR] 初始化发布者: {foot_pose_topic}")
        except Exception as e:
            rospy.logerr(f"[MOTION_EXECUTOR] 初始化发布者失败: {e}")
    
    def _init_service_proxies(self):
        """初始化所有服务代理"""
        try:
            rospy.wait_for_service('/execute_arm_action', timeout=1.0)
            self.execute_action_proxy = rospy.ServiceProxy('/execute_arm_action', ExecuteArmAction)
            rospy.loginfo("[MOTION_EXECUTOR] 初始化动作服务代理成功")
        except (rospy.ROSException, rospy.ServiceException) as e:
            rospy.logwarn(f"[MOTION_EXECUTOR] 初始化动作服务代理失败: {e}")
            self.execute_action_proxy = None
    
    def trigger_action_callback(self, action_name):
        """动作意图回调函数，调用 /execute_arm_action 服务执行"""
        rospy.loginfo(f"[MOTION_EXECUTOR] 触发动作回调: {action_name}")
        
        try:
            if self.execute_action_proxy is None:
                rospy.logwarn("[MOTION_EXECUTOR] 动作服务代理未初始化，尝试重新初始化...")
                self._init_service_proxies()
            
            if self.execute_action_proxy is None:
                rospy.logerr("[MOTION_EXECUTOR] 动作执行失败：服务代理不可用")
                return
            
            req = ExecuteArmActionRequest()

            req.action_name = action_name
            
            # 
            rospy.loginfo(f"[MOTION_EXECUTOR] 调用动作服务: action_name={action_name}")
            response = self.execute_action_proxy(req)
            
            if response.success:
                rospy.loginfo(f"[MOTION_EXECUTOR] 动作执行成功: {action_name} - {response.message}")
            else:
                rospy.logerr(f"[MOTION_EXECUTOR] 动作执行失败: {action_name} - {response.message}")
                
        except rospy.ServiceException as e:
            rospy.logerr(f"[MOTION_EXECUTOR] 动作服务调用失败: {e}")
            self.execute_action_proxy = None # 重置代理
        except Exception as e:
            rospy.logerr(f"[MOTION_EXECUTOR] 动作回调执行错误: {e}")
    
    def trigger_move_callback(self, slot):
        """移动意图回调函数，发布足部姿态轨迹消息
        
        Args:
            slot: JSON字符串，格式为 {"direction": "前|后|左|右|左转|右转", "step": 步数}
        
        示例:
            slot = '{"direction": "前", "step": 2}'  # 向前走2步
            slot = '{"direction": "左转", "step": 1}'  # 左转
            
        移动规则：
            - 前后左右单步距离: 0.1米
            - 左右转单步角度: 15度
            - 向前: x为正
            - 向后: x为负
            - 向左: y为正
            - 向右: y为负
            - 左转: yaw为正
            - 右转: yaw为负
        """

        try:
            if self.foot_pose_pub is None:
                rospy.logerr("[MOTION_EXECUTOR] 移动失败：足部姿态发布者未初始化")
                return
                
            move_params = json.loads(slot)
            direction = move_params.get("direction")
            step = move_params.get("step", 1)

            # 对中文方向进行映射
            direction_map = {
                "前": "前", "forward": "前",
                "后": "后", "back": "后", "退": "后",
                "左": "左", "left": "左",
                "右": "右", "right": "右",
                "左转": "左转", "turn_left": "左转",
                "右转": "右转", "turn_right": "右转"
            }
            direction = direction_map.get(direction, "未知")

            rospy.loginfo(f"[MOTION_EXECUTOR] 触发移动回调 - 方向: {direction}, 步数: {step}")
            
            body_poses = []
            step_distance = 0.1  # 单步举例
            step_angle = 15      # 单步转向角度
            
            for i in range(step):
                if direction == "前":
                    # 向前：x为正
                    body_poses.append([(i + 1) * step_distance, 0, 0, 0])
                elif direction == "后":
                    # 向后：x为负
                    body_poses.append([-(i + 1) * step_distance, 0, 0, 0])
                elif direction == "左":
                    # 向左：y为正
                    body_poses.append([0, (i + 1) * step_distance, 0, 0])
                elif direction == "右":
                    # 向右：y为负
                    body_poses.append([0, -(i + 1) * step_distance, 0, 0])
                elif direction == "左转":
                    # 左转：yaw为正
                    body_poses.append([0, 0, 0, (i + 1) * step_angle])
                elif direction == "右转":
                    # 右转：yaw为负
                    body_poses.append([0, 0, 0, -(i + 1) * step_angle])
                else:
                    rospy.logerr(f"[MOTION_EXECUTOR] 未知的移动方向: {direction}")
                    return

            # 时间步长
            dt = 0.4
            # rospy.loginfo(f"[MOTION_EXECUTOR] 生成的 body_poses: {body_poses}")
            
            # 生成足部姿态轨迹消息
            msg = get_multiple_steps_msg(body_poses, dt, is_left_first=True, collision_check=True)
            
            self.foot_pose_pub.publish(msg)
            # 左转右转时，步数调整
            step = 0 if direction in ["左转", "右转"] else step
            # rospy.loginfo(f"[MOTION_EXECUTOR] 发布足部姿态轨迹消息: {msg}")
            rospy.loginfo(f"[MOTION_EXECUTOR] 移动指令已发布 - 方向: {direction}，步数： {step}")
            
        except json.JSONDecodeError as e:
            rospy.logerr(f"[MOTION_EXECUTOR] 移动参数JSON解析失败: {e}, slot: {slot}")
        except Exception as e:
            rospy.logerr(f"[MOTION_EXECUTOR] 移动回调执行错误: {e}")
