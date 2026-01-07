#!/usr/bin/env python3
"""
H12 遥控器 Trigger 发送脚本
用法: python send_h12_trigger.py <trigger_name> [current_state]
示例: python send_h12_trigger.py walk stance
     python send_h12_trigger.py initial_pre
"""

import rospy
import sys
import json
import os
import rospkg
from h12pro_controller_node.msg import h12proRemoteControllerChannel

# Channel 值定义
MIN_VALUE = 282
MID_VALUE = 1002
MAX_VALUE = 1722

# 默认通道值（摇杆在中间，其他为最小值）
DEFAULT_CHANNELS = [MID_VALUE] * 4 + [MIN_VALUE] * 8

# 状态到channel值的映射
STATE_TO_CHANNEL = {
    "E_LEFT": {"index": 4, "value": MIN_VALUE},      # Channel 5 (索引4)
    "E_MIDDLE": {"index": 4, "value": MID_VALUE},
    "E_RIGHT": {"index": 4, "value": MAX_VALUE},
    "F_LEFT": {"index": 5, "value": MIN_VALUE},      # Channel 6 (索引5)
    "F_MIDDLE": {"index": 5, "value": MID_VALUE},
    "F_RIGHT": {"index": 5, "value": MAX_VALUE},
    "A_PRESS": {"index": 6, "value": MAX_VALUE},      # Channel 7 (索引6)
    "A_RELEASE": {"index": 6, "value": MIN_VALUE},
    "A_LONG_PRESS": {"index": 6, "value": MAX_VALUE},  # 长按和短按的channel值相同
    "B_PRESS": {"index": 7, "value": MAX_VALUE},      # Channel 8 (索引7)
    "B_RELEASE": {"index": 7, "value": MIN_VALUE},
    "B_LONG_PRESS": {"index": 7, "value": MAX_VALUE},
    "C_PRESS": {"index": 8, "value": MAX_VALUE},      # Channel 9 (索引8)
    "C_RELEASE": {"index": 8, "value": MIN_VALUE},
    "C_LONG_PRESS": {"index": 8, "value": MAX_VALUE},
    "D_PRESS": {"index": 9, "value": MAX_VALUE},       # Channel 10 (索引9)
    "D_RELEASE": {"index": 9, "value": MIN_VALUE},
    "D_LONG_PRESS": {"index": 9, "value": MAX_VALUE},
}

# 长按阈值（秒）
LONG_PRESS_THRESHOLD = 1.0


def load_config():
    """加载H12遥控器配置文件"""
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('h12pro_controller_node')
    config_path = os.path.join(pkg_path, "src", "h12pro_node", "h12pro_remote_controller.json")
    
    with open(config_path, 'r') as f:
        config = json.load(f)
    
    return config


def find_trigger_config(trigger_name, current_state=None):
    """查找trigger对应的按键组合配置"""
    config = load_config()
    key_combinations = config.get("multi_robot_state_transition_keycombination", {})
    
    # 如果指定了当前状态，只在该状态下查找
    if current_state:
        if current_state in key_combinations:
            if trigger_name in key_combinations[current_state]:
                return key_combinations[current_state][trigger_name]
        return None
    
    # 否则在所有状态下查找
    for state, triggers in key_combinations.items():
        if trigger_name in triggers:
            return triggers[trigger_name]
    
    return None


def build_channel_message(key_combination):
    """根据按键组合构建channel消息
    
    Returns:
        tuple: (channels, has_long_press) - channel消息和是否包含长按
    """
    channels = DEFAULT_CHANNELS.copy()
    has_long_press = False
    
    for key_state in key_combination:
        # 检测是否有长按
        if key_state.endswith("_LONG_PRESS"):
            has_long_press = True
            rospy.loginfo(f"[H12Trigger] 检测到长按: {key_state}")
        
        if key_state in STATE_TO_CHANNEL:
            mapping = STATE_TO_CHANNEL[key_state]
            channels[mapping["index"]] = mapping["value"]
        else:
            rospy.logwarn(f"[H12Trigger] 未知的按键状态: {key_state}")
    
    return channels, has_long_press


def send_trigger(trigger_name, current_state=None):
    """发送trigger对应的H12消息"""
    # 查找配置
    key_combination = find_trigger_config(trigger_name, current_state)
    
    if not key_combination:
        rospy.logerr(f"[H12Trigger] 未找到trigger '{trigger_name}'")
        if current_state:
            rospy.logerr(f"[H12Trigger] 在状态 '{current_state}' 下未找到该trigger")
        else:
            rospy.logerr(f"[H12Trigger] 在所有状态下都未找到该trigger")
        return False
    
    rospy.loginfo(f"[H12Trigger] 找到trigger '{trigger_name}'，按键组合: {key_combination}")
    
    # 构建消息并检测是否有长按
    channels, has_long_press = build_channel_message(key_combination)
    
    # 创建并发布消息
    msg = h12proRemoteControllerChannel()
    msg.channels = channels
    msg.sbus_state = 1
    
    # 发布消息
    pub = rospy.Publisher('/h12pro_channel', h12proRemoteControllerChannel, queue_size=10)
    
    # 等待订阅者
    rospy.sleep(0.1)
    
    # 发布按键按下消息
    pub.publish(msg)
    rospy.loginfo(f"[H12Trigger] 已发送按下消息: channels={channels}")
    
    # 根据是否有长按决定保持时间
    if has_long_press:
        # 长按：保持按下状态至少 LONG_PRESS_THRESHOLD 秒
        rospy.loginfo(f"[H12Trigger] 长按模式：保持按下状态 {LONG_PRESS_THRESHOLD} 秒")
        rospy.sleep(LONG_PRESS_THRESHOLD)
        # 持续发布消息以确保长按状态（可选，如果需要更可靠可以定期发布）
        # 这里我们只等待足够的时间
    else:
        # 短按：短暂延时后释放
        rospy.sleep(0.1)
    
    # 发送释放消息（保持E和F的值）
    reset_channels = DEFAULT_CHANNELS.copy()
    reset_channels[4] = channels[4]  # 保持E开关位置
    reset_channels[5] = channels[5]  # 保持F开关位置
    msg.channels = reset_channels
    pub.publish(msg)
    rospy.loginfo(f"[H12Trigger] 已发送释放消息（保持E/F开关位置）")
    
    return True


def list_all_triggers():
    """列出所有可用的trigger"""
    config = load_config()
    key_combinations = config.get("multi_robot_state_transition_keycombination", {})
    
    print("\n=== 所有可用的 Trigger ===")
    for state, triggers in key_combinations.items():
        print(f"\n状态: {state}")
        for trigger_name, key_combo in triggers.items():
            print(f"  - {trigger_name}: {key_combo}")
    print()


def main():
    if len(sys.argv) < 2:
        print("用法: python send_h12_trigger.py <trigger_name> [current_state]")
        print("       python send_h12_trigger.py --list  # 列出所有trigger")
        print("\n示例:")
        print("  python send_h12_trigger.py walk stance")
        print("  python send_h12_trigger.py initial_pre")
        print("  python send_h12_trigger.py switch_controller stance")
        sys.exit(1)
    
    # 列出所有trigger
    if sys.argv[1] == "--list" or sys.argv[1] == "-l":
        list_all_triggers()
        sys.exit(0)
    
    trigger_name = sys.argv[1]
    current_state = sys.argv[2] if len(sys.argv) > 2 else None
    
    # 初始化ROS节点
    rospy.init_node('send_h12_trigger', anonymous=True)
    
    # 发送trigger
    success = send_trigger(trigger_name, current_state)
    
    if success:
        rospy.loginfo(f"[H12Trigger] 成功发送trigger '{trigger_name}'")
    else:
        rospy.logerr(f"[H12Trigger] 发送trigger '{trigger_name}' 失败")
        sys.exit(1)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"[H12Trigger] 错误: {e}")
        sys.exit(1)

