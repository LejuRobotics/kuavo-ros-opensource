#!/usr/bin/env python3

from typing import Tuple
import rospy
import sys
import os
# 添加当前脚本所在目录到Python路径中，以便正确导入同目录下的模块
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from voice_recognition import VoiceCoordinator
from motion_executor import MotionExecutor
import traceback
import json
from queue import Empty 
from std_srvs.srv import Trigger, TriggerRequest

# ==========================================
# 1. 加载动作配置文件
# ==========================================
def load_action_config():
    """加载并返回动作配置文件内容"""
    config_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "key_words.json"
    )
    try:
        with open(config_path, 'r', encoding='utf-8') as f:
            rospy.loginfo(f"成功加载动作配置文件: {config_path}")
            return json.load(f)
    except Exception as e:
        rospy.logerr(f"加载动作配置文件 {config_path} 失败: {e}")
        return {}

ACTION_CONFIG = load_action_config()

# ==========================================
# 2. 通用动作执行器
# ==========================================
def execute_action(motion_executor, action_key):
    """
    根据 action_key 从配置文件中查找动作类型和数据，并执行。
    """
    if not ACTION_CONFIG:
        rospy.logerr("动作配置文件为空或加载失败，无法执行任何动作。")
        return

    action_details = ACTION_CONFIG.get(action_key)
    
    if not action_details:
        rospy.logwarn(f"警告: 在动作配置文件中找不到 '{action_key}' 对应的配置！")
        return

    action_type = action_details.get("type")
    action_data = action_details.get("data")

    if action_data is None:
        rospy.logwarn(f"警告: 动作 '{action_key}' 的 'data' 字段为空！")
        return

    try:
        rospy.loginfo(f">>> 执行动作: {action_key} (类型: {action_type})")
        if action_type == "SINGLE_STEP":
            # 对于单步移动，data 是一个字典，需要转换为 JSON 字符串
            slot = json.dumps(action_data, ensure_ascii=False)
            # 调用移动函数
            motion_executor.trigger_move_callback(slot)
        elif action_type == "ARM_ACTION":
            # 对于手臂动作，data 就是字符串
            success,status =get_robot_launch_status()
            if not success or not status == "launched":
                rospy.logwarn(f"警告: 机器人未启动，无法执行手臂动作:'{action_key}' ,当前status:{status}")
                return
            motion_executor.trigger_action_callback(action_data)
        else:
            rospy.logwarn(f"警告: 未知的动作类型 '{action_type}'，无法执行 '{action_key}'。")

    except Exception as e:
        rospy.logerr(f"执行动作 {action_key} 时发生错误: {traceback.format_exc()}")


def main():
    # 1. 初始化 ROS 节点
    rospy.init_node('voice_control_node', anonymous=False)
    
    # 2. 定义配置重载回调函数（用于更新 ACTION_CONFIG）
    def update_action_config_callback():
        """在 VoiceCoordinator 重载配置后调用，更新 main.py 的 ACTION_CONFIG"""
        global ACTION_CONFIG
        ACTION_CONFIG = load_action_config()
        rospy.loginfo("ACTION_CONFIG 已成功更新")
    
    # 3. 创建实例，传入回调函数
    try:
        voice_coordinator = VoiceCoordinator(subscribe_topic="/micphone_data", 
                                            on_reload_callback=update_action_config_callback)
        motion_executor = MotionExecutor()
    except Exception as e:
        rospy.logerr(f"初始化失败: {e}")
        return

    rospy.loginfo("语音控制系统启动，等待语音输入... (按 Ctrl+C 退出)")
    rospy.loginfo("使用 '/voice_control/reload_keywords' 服务可以重载关键词配置")

    # 清除初始化过程中接收到的麦克风数据
    voice_coordinator.reset_micphone_data()
    
    # 4. 使用 ROS 标准循环
    while not rospy.is_shutdown():
        try:
            # 使用带超时的阻塞式获取，以降低延迟并提高响应速度
            action_key = voice_coordinator.result_queue.get(timeout=0.1)
            if action_key:
                rospy.loginfo(f"接收到语音命令，匹配到动作: {action_key}")
                execute_action(motion_executor,action_key)
                # TODO 是否有必要动作执行后，重置麦克风数据，丢弃在ASR和动作执行期间累积的音频。
                # voice_coordinator.reset_micphone_data()

        except Empty:
            # 队列为空是正常情况，继续等待，无需休眠
            continue
        except rospy.ROSInterruptException:
            # 捕获 ROS 中断异常，如ctrl+C
            rospy.loginfo("收到ctrl+c命令，程序已正常停止")
            break
        except Exception as e:
            rospy.logerr(f"主循环发生未知错误，程序即将停止: {e}")
            break

    rospy.loginfo("程序已正常停止")
def get_robot_launch_status()->Tuple[bool, str]:
        status_client = rospy.ServiceProxy('/websocket_sdk_srv/get_robot_launch_status', Trigger)
        try:
            status_client.wait_for_service(timeout=2.0)
        except rospy.ROSException as e:
            # 等待服务超时（服务不存在）
            print(f"Service does not exist: {e}")
            return True, "unknown"  # 关键修改：服务不存在时返回(True, "unknown")
    
        try:
            req = TriggerRequest()
            status_client.wait_for_service(timeout=1.5)
            # Call the service
            response = status_client.call(req)
            if response.success:
                print(f"RealInitializeSrv service call successful")
                return True, response.message
            else:
                print(f"Failed to callRealInitializeSrv service")
                return False, "unknown"
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
            return False, f"unknown"
        
if __name__ == "__main__":
    main()