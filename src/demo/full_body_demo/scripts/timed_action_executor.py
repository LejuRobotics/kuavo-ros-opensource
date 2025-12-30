#!/usr/bin/env python3

import rospy
import yaml
import time
import subprocess
import os
import signal
import json
import subprocess
import sys
# 检查pandas版本
try:
    import pandas as pd
    current_version = pd.__version__
    required_version = '2.0.3'
    
    if current_version != required_version:
        rospy.logwarn(f"当前pandas版本 {current_version} 与要求版本 {required_version} 不符")
        rospy.loginfo(f"正在安装pandas {required_version}")
        subprocess.check_call([sys.executable, "-m", "pip", "install", f"pandas=={required_version}"])
        rospy.loginfo("pandas安装完成，请重新运行程序")
    import pandas as pd
except ImportError:
    rospy.logwarn("未检测到pandas，正在安装...")
    subprocess.check_call([sys.executable, "-m", "pip", "install", f"pandas==2.0.3"])
    rospy.loginfo("pandas安装完成，请重新运行程序")
    import pandas as pd

from sensor_msgs.msg import Joy
from std_msgs.msg import String, Bool, Float32
from geometry_msgs.msg import Twist
from kuavo_msgs.srv import SetJoyTopic, changeArmCtrlMode
from ocs2_msgs.msg import mode_schedule

# 获取脚本所在目录和配置文件
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(script_dir, "../../.."))
# 导入动作播放器
from demo.csv2body_demo.step_player_csv_ocs2 import ActionPlayer
from demo.full_body_demo.csv_trajectory_publisher import CsvTrajectoryPublisher

class TimedActionExecutor:
    def __init__(self):
        # 记录起始时间
        rospy.init_node('timed_action_executor')
        self.countdown_duration = rospy.get_param('~countdown', 5.0)  # 默认5秒倒计时
        countdown_start = time.time()
        self.start_time = countdown_start + self.countdown_duration  # 实际开始时间
        rospy.loginfo(f"\n倒计时 {self.countdown_duration} 秒开始...")
        
        
        # 获取脚本所在目录和配置文件
        # script_dir = os.path.dirname(os.path.abspath(__file__))
        self.workspace_root = os.path.abspath(os.path.join(script_dir, "../../../.."))
        
        # 加载配置文件
        config_file = os.path.join(script_dir, "action_sequence.yaml")
        with open(config_file, 'r') as f:
            self.config = yaml.safe_load(f)
        
        # 初始化按键映射
        self.joyButtonMap = {
            "BUTTON_STANCE": 0,
            "BUTTON_TROT": 1,
            "BUTTON_JUMP": 2,
            "BUTTON_WALK": 3,
            "BUTTON_LB": 4,
            "BUTTON_RB": 5,
            "BUTTON_BACK": 6,
            "BUTTON_START": 7
        }
        
        self.joyAxisMap = {
            "AXIS_LEFT_STICK_Y": 0,
            "AXIS_LEFT_STICK_X": 1,
            "AXIS_LEFT_LT": 2,
            "AXIS_RIGHT_STICK_YAW": 3,
            "AXIS_RIGHT_STICK_Z": 4,
            "AXIS_RIGHT_RT": 5,
            "AXIS_LEFT_RIGHT_TRIGGER": 6,
            "AXIS_FORWARD_BACK_TRIGGER": 7
        }
        
        # 如果存在channel_map_path参数，则从JSON文件加载按键映射
        if rospy.has_param("channel_map_path"):
            channel_map_path = rospy.get_param("channel_map_path")
            rospy.loginfo(f"从 {channel_map_path} 加载按键映射")
            self.load_joy_json_config(channel_map_path)
        else:
            rospy.logwarn("未找到channel_map_path参数，使用默认按键映射")
        
        # 初始化ROS发布者和订阅者
        self.joy_pub = rospy.Publisher(self.config['joy_remap']['remapped_topic'], Joy, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.joy_sub = rospy.Subscriber(self.config['joy_remap']['input_topic'], Joy, self.joy_callback)
        
        # 运行时变量
        self.latest_joy_msg = None
        self.remapped_joy_msg = None
        self.next_action_index = 0
        
        # 按钮触发相关变量
        self.button_press_time = None  # 按钮按下的时间
        self.button_duration = 1.1  # 按钮按下持续时间（秒）
        self.pressed_button_id = None  # 当前按下的按钮ID
        
        # 添加remap_joy状态控制
        self.current_remap_joy = False  # 默认允许remap
        
        # 将动作序列转换为有序列表，保持字符串格式的时间点
        self.actions = [(t, action) for t, action in self.config['action_sequence'].items()]
        self.actions.sort(key=lambda x: float(x[0]))  # 根据时间点的浮点数值排序

        # 设置信号处理
        signal.signal(signal.SIGINT, self.signal_handler)
        
        # 等待服务可用
        rospy.wait_for_service('/set_joy_topic')
        rospy.wait_for_service('/humanoid_change_arm_ctrl_mode')
        self.set_joy_topic = rospy.ServiceProxy('/set_joy_topic', SetJoyTopic)
        self.change_arm_mode = rospy.ServiceProxy('/humanoid_change_arm_ctrl_mode', changeArmCtrlMode)
        
        # 切换到重映射的话题
        try:
            response = self.set_joy_topic(self.config['joy_remap']['remapped_topic'])
            if response.success:
                rospy.loginfo("成功切换到重映射的joy话题")
            else:
                rospy.logwarn(f"切换joy话题失败: {response.message}")
        except rospy.ServiceException as e:
            rospy.logerr(f"调用服务失败: {e}")
            
        # 预加载动作播放器
        self.gesture_dance_player = None
        self.taichi_player = None
        self.dance_player = None
        self.preload_actions()
        
        # 添加当前动作配置存储
        self.current_action = None
        
    def preload_actions(self):
        """预加载所有动作序列"""
        rospy.loginfo("开始预加载动作序列..." )
        start_time = time.time()
        for time_str, action in self.actions:
            print("**"*20)
            if action['type'] == "function":
                if action['command'] == "play_gesture_dance":
                    # 预加载手势舞
                    csv_file = os.path.join(self.workspace_root, action['params']['csv_file'])
                    self.gesture_dance_player = CsvTrajectoryPublisher()
                    if self.gesture_dance_player.load_action_with_csv(csv_file):
                        rospy.loginfo(f"预加载手势舞成功: {csv_file}")
                    else:
                        rospy.logerr(f"预加载手势舞失败: {csv_file}")
                        
                elif action['command'] == "play_taichi":
                    # 预加载太极
                    csv_file = os.path.join(self.workspace_root, action['params']['csv_file'])
                    self.taichi_player = ActionPlayer()
                    if self.taichi_player.load_action_with_csv(csv_file):
                        rospy.loginfo(f"预加载太极成功: {csv_file}")
                    else:
                        rospy.logerr(f"预加载太极失败: {csv_file}")
                        
                elif action['command'] == "play_dance":
                    # 预加载跳舞
                    csv_file = os.path.join(self.workspace_root, action['params']['csv_file'])
                    self.dance_player = CsvTrajectoryPublisher()
                    if self.dance_player.load_action_with_csv(csv_file):
                        rospy.loginfo(f"预加载跳舞成功: {csv_file}")
                    else:
                        rospy.logerr(f"预加载跳舞失败: {csv_file}")
        print("**"*20)
        rospy.loginfo(f"动作序列预加载完成, 总耗时: {time.time() - start_time:.2f}秒， 距离开始时间: {self.start_time - time.time():.2f}秒")

    def load_joy_json_config(self, config_file):
        """从JSON文件加载按键映射配置"""
        try:
            with open(config_file, 'r') as f:
                data = json.load(f)
                
            # 更新按键映射
            for key, value in data.get("JoyButton", {}).items():
                if key in self.joyButtonMap:
                    self.joyButtonMap[key] = value
                else:
                    self.joyButtonMap[key] = value
                    
            for key, value in data.get("JoyAxis", {}).items():
                if key in self.joyAxisMap:
                    self.joyAxisMap[key] = value
                else:
                    self.joyAxisMap[key] = value
                    
            rospy.loginfo("成功加载按键映射配置")
        except Exception as e:
            rospy.logerr(f"加载按键映射配置失败: {str(e)}")

    def switch_to_walk(self):
        """切换到行走步态"""
        if self.latest_joy_msg is None:
            rospy.logwarn("未收到joy消息，无法切换步态")
            return
        self.trigger_button(self.joyButtonMap["BUTTON_WALK"])
        rospy.loginfo("切换到行走步态")

    def switch_to_stance(self):
        """切换到站立步态"""
        if self.latest_joy_msg is None:
            rospy.logwarn("未收到joy消息，无法切换步态")
            return
        self.trigger_button(self.joyButtonMap["BUTTON_STANCE"])
        rospy.loginfo("切换到站立步态")
    def reset_joy_remap(self):
        rospy.loginfo("正在恢复joy话题设置...")
        try:
            response = self.set_joy_topic('/joy')
            if response.success:
                rospy.loginfo("已恢复原始joy话题")
            else:
                rospy.logwarn(f"恢复joy话题失败: {response.message}")
        except rospy.ServiceException as e:
            rospy.logerr(f"调用服务失败: {e}")
    def signal_handler(self, signum, frame):
        """处理Ctrl+C信号"""
        self.reset_joy_remap()
        
        # 正常退出程序
        rospy.signal_shutdown("用户中断")
        exit(0)

    def joy_callback(self, msg):
        """处理接收到的joy消息"""
        current_time = time.time()
        self.latest_joy_msg = msg
        
        # 创建重映射的joy消息（默认全0）
        if self.remapped_joy_msg is None:
            self.remapped_joy_msg = Joy()
            self.remapped_joy_msg.buttons = [0] * len(msg.buttons)
            self.remapped_joy_msg.axes = [0.0] * len(msg.axes)
        
        # 如果允许remap_joy，则处理xyz和yaw的控制
        if self.current_remap_joy:
            # 复制原始消息的轴值
            self.remapped_joy_msg.axes = list(msg.axes)
        else:
            self.remapped_joy_msg.axes = [0.0] * len(msg.axes)
        
        # 始终映射BACK按钮
        back_button_index = self.joyButtonMap["BUTTON_BACK"]
        self.remapped_joy_msg.buttons[back_button_index] = msg.buttons[back_button_index]
        
        # 检查是否需要重置按钮状态
        if self.button_press_time is not None and self.pressed_button_id is not None:
            if current_time - self.button_press_time >= self.button_duration:
                # 重置按钮状态，但不重置BACK按钮
                if self.pressed_button_id != back_button_index:
                    self.remapped_joy_msg.buttons[self.pressed_button_id] = 0
                    self.button_press_time = None
                    self.pressed_button_id = None
        
        # 发布重映射的joy消息
        self.joy_pub.publish(self.remapped_joy_msg)

    def create_joy_msg(self, button_id, value):
        """创建带有指定按钮状态的Joy消息"""
        joy_msg = Joy()
        if self.latest_joy_msg:
            joy_msg = self.latest_joy_msg
            joy_msg.buttons = list(joy_msg.buttons)  # 转换为列表以便修改
        else:
            joy_msg.buttons = [0] * 8  # 创建8个按钮的列表
            joy_msg.axes = [0.0] * 8   # 创建8个轴的列表
        
        joy_msg.buttons[button_id] = value
        return joy_msg

    def send_cmd_vel(self, linear_x=0.0, linear_y=0.0, linear_z=0.0, angular_x=0.0, angular_y=0.0, angular_z=0.0):
        """发送速度命令"""
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_x
        cmd_vel.linear.y = linear_y
        cmd_vel.linear.z = linear_z
        cmd_vel.angular.x = angular_x
        cmd_vel.angular.y = angular_y
        cmd_vel.angular.z = angular_z
        self.cmd_vel_pub.publish(cmd_vel)

    def trigger_button(self, button_id):
        """触发按钮动作"""
        if self.remapped_joy_msg is None:
            rospy.logwarn("未初始化重映射joy消息，无法触发按钮")
            return
            
        # 设置按钮状态和计时器
        self.remapped_joy_msg.buttons[button_id] = 1
        self.button_press_time = time.time()
        self.pressed_button_id = button_id
        print(f"触发按钮: {button_id}")
        # 发布消息
        self.joy_pub.publish(self.remapped_joy_msg)

    def switch_arm_mode(self, mode):
        """切换手臂控制模式"""
        try:
            response = self.change_arm_mode(mode)
            if response.result:
                rospy.loginfo(f"成功切换手臂控制模式到: {mode}")
                return True
            else:
                rospy.logwarn("切换手臂控制模式失败")
                return False
        except rospy.ServiceException as e:
            rospy.logerr(f"调用手臂控制模式服务失败: {e}")
            return False

    def play_gesture_dance(self):
        """播放手势舞动作"""
        if self.gesture_dance_player:
            rospy.loginfo("开始播放手势舞")
            time_offset = self.current_action.get('params', {}).get('time_offset')
            return self.gesture_dance_player.run(time_offset=time_offset)
        else:
            rospy.logerr("手势舞播放器未初始化")
            return False

    def play_taichi(self):
        """播放太极动作"""
        if self.taichi_player:
            rospy.loginfo("开始播放太极")
            time_offset = self.current_action.get('params', {}).get('time_offset')
            return self.taichi_player.execute_action_with_csv(time_offset)
        else:
            rospy.logerr("太极播放器未初始化")
            return False

    def play_dance(self):
        """播放跳舞动作"""
        if self.dance_player:
            rospy.loginfo("开始播放跳舞")
            time_offset = self.current_action.get('params', {}).get('time_offset')
            return self.dance_player.run(time_offset=time_offset)
        else:
            rospy.logerr("跳舞播放器未初始化")
            return False

    def finish_action(self):
        """完成动作序列"""
        rospy.loginfo("动作序列执行完成")
        return True

    def execute_action(self, action):
        """执行动作"""
        # 更新remap_joy状态
        self.current_remap_joy = action.get('remap_joy', True)
        self.current_action = action  # 保存当前动作配置
        rospy.loginfo(f"更新remap_joy状态: {self.current_remap_joy}")
        
        if action['type'] == 'function':
            command = action['command']
            if command == 'stance' or command == 'stand':
                rospy.loginfo(f"执行动作: {command}")
                self.trigger_button(self.joyButtonMap["BUTTON_STANCE"])
            elif command == 'walk':
                rospy.loginfo("执行动作: walk")
                self.trigger_button(self.joyButtonMap["BUTTON_WALK"])
                self.switch_to_walk()
            elif command == 'walk_and_reset':
                rospy.loginfo("执行动作: walk_and_reset")
                self.trigger_button(self.joyButtonMap["BUTTON_WALK"])
                self.send_cmd_vel(linear_z=-0.02)
            elif command == 'stance_and_reset':
                rospy.loginfo("执行动作: stance_and_reset")
                self.trigger_button(self.joyButtonMap["BUTTON_STANCE"])
                self.send_cmd_vel(linear_z=0.0)
            elif command.startswith('change_arm_mode'):
                # 解析模式参数，格式为 "change_arm_mode:1" 或 "change_arm_mode:0"
                try:
                    mode = int(command.split(':')[1])
                    rospy.loginfo(f"切换手臂控制模式到: {mode}")
                    self.switch_arm_mode(mode)
                except (IndexError, ValueError) as e:
                    rospy.logerr(f"无效的手臂控制模式命令: {command}, 错误: {e}")
            elif command == 'play_gesture_dance':
                self.play_gesture_dance()
            elif command == 'play_taichi':
                self.play_taichi()
            elif command == 'play_dance':
                self.play_dance()
            elif command == 'finish_action':
                self.finish_action()
            else:
                rospy.logerr(f"未找到动作: {command}")
        elif action['type'] == 'shell':
            rospy.loginfo(f"执行shell命令: {action['command']}")
            try:
                subprocess.run(action['command'], shell=True, cwd=self.workspace_root)
            except Exception as e:
                rospy.logerr(f"执行命令出错: {str(e)}")

        else:
            rospy.logerr(f"未找到动作类型: {action['type']}")

    def print_time(self, current_time):
        """打印时间，支持负数时间"""
        abs_time = abs(current_time)
        sign = '-' if current_time < 0 else ' '
        minutes = int(abs_time // 60)
        seconds = int(abs_time % 60)
        milliseconds = int((abs_time % 1) * 1000)
        print(f"\r当前时间: {sign}{minutes:02d}:{seconds:02d}.{milliseconds:03d}", end='', flush=True)

    def run(self):
        """运行动作序列"""
        rate = rospy.Rate(100)  # 100Hz
        
        index = 0
        
        while not rospy.is_shutdown():
            index += 1
            current_time = time.time() - self.start_time  # 当前时间（相对于实际开始时间）
            
            # 检查是否所有动作都已执行完毕
            if self.next_action_index >= len(self.actions):
                break
            
            # 获取下一个动作的信息
            next_time_str, next_action = self.actions[self.next_action_index]
            next_time = float(next_time_str)
            
            # 每隔一定次数更新显示时间
            if index % 12 == 0:
                self.print_time(current_time)
            
            # 检查是否需要执行下一个动作
            if current_time >= next_time:
                rospy.loginfo(f"\n执行时间点 {next_time_str} 的动作")
                self.execute_action(next_action)
                self.next_action_index += 1
            
            rate.sleep()
        
        print("\n动作序列执行完成")
        self.reset_joy_remap()
        

if __name__ == '__main__':
    try:
        executor = TimedActionExecutor()
        executor.run()
    except rospy.ROSInterruptException:
        pass 
