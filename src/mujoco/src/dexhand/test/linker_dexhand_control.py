#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
L6/O6 灵巧手控制测试脚本
L6手控制范围：0-255 (255=完全张开，0=完全握紧)
O6手控制范围：0-100 (0=完全张开，100=完全握紧)
"""
import rospy
import os
import argparse
from sensor_msgs.msg import JointState
from kuavo_msgs.msg import robotHandPosition

# ====================== 可配置默认参数 (用户可自行修改) ======================
DEFAULT_DURATION = 10          # 默认持续时间(秒)
DEFAULT_HAND = 'left'          # 默认控制的手
DEFAULT_INTERFACE = 'l6'       # 默认接口 ('l6'=L6原生接口, 'o6'=O6兼容接口)
DEFAULT_HAND_TYPE = 'l6'       # 默认手类型 ('l6'=L6手, 'o6'=O6手)
PUBLISH_RATE = 10              # 发布频率(Hz)
# 不同手类型的参数配置
HAND_CONFIG = {
    'l6': {
        'min_value': 0,
        'max_value': 255,
        'open_value': 255,    # 完全张开值
        'close_value': 0,     # 完全握紧值
        'description': 'L6手 (0-255范围, 255=张开)'
    },
    'o6': {
        'min_value': 0,
        'max_value': 100,
        'open_value': 0,      # 完全张开值
        'close_value': 100,   # 完全握紧值
        'description': 'O6手 (0-100范围, 0=张开)'
    }
}
# ============================================================================

# 手指名称到索引的映射 (L6 手)
FINGER_INDEX = {
    'thumb': 0,       # 拇指
    'thumb_aux': 1,   # 拇指辅助关节
    'index': 2,       # 食指
    'middle': 3,      # 中指
    'ring': 4,        # 无名指
    'little': 5       # 小指
}

# 手指名称中文映射
FINGER_NAME_CN = {
    'thumb': '拇指',
    'thumb_aux': '拇指辅助关节',
    'index': '食指',
    'middle': '中指',
    'ring': '无名指',
    'little': '小指'
}

# 全局变量保存当前左右手位置（用于O6接口单手控制）
current_left_pos = None
current_right_pos = None


def clear_screen():
    """清屏"""
    os.system('cls' if os.name == 'nt' else 'clear')


def show_menu(current_hand, current_interface, current_duration, current_hand_type):
    """显示主菜单"""
    clear_screen()
    print("=" * 60)
    print("        🤖 L6/O6 灵巧手控制测试工具")
    print("=" * 60)
    hand_config = HAND_CONFIG[current_hand_type]
    print(f" 当前配置: 手={current_hand} | 接口={current_interface} | 类型={hand_config['description']} | 持续时间={current_duration}s")
    print("-" * 60)
    print("  1. 🖐️  选择控制的手 (左手/右手/双手)")
    print("  2. 🔌 选择控制接口")
    print("  3. 🤲 选择手类型 (L6/O6)")
    print("  4. ⏱️  设置持续时间")
    print("-" * 60)
    print("  5. 👉 测试单个手指")
    print("  6. 👐 测试所有手指 (张开)")
    print("  7. ✊ 测试所有手指 (握紧)")
    print("-" * 60)
    print("  0. ❌ 退出程序")
    print("=" * 60)
    choice = input("请输入选项编号: ").strip()
    return choice


def select_hand():
    """选择控制的手"""
    print("\n" + "-" * 40)
    print("选择要控制的手:")
    print("  1. 左手 (left)")
    print("  2. 右手 (right)")
    print("  3. 双手 (both)")
    print("-" * 40)
    while True:
        choice = input("请输入选项: ").strip()
        if choice == '1':
            return 'left'
        elif choice == '2':
            return 'right'
        elif choice == '3':
            return 'both'
        else:
            print("❌ 无效选项，请重新输入!")


def select_interface():
    """选择控制接口"""
    print("\n" + "-" * 40)
    print("选择控制接口:")
    print("  1. L6 原生接口 (/cb_left_hand_control_cmd, /cb_right_hand_control_cmd)")
    print("  2. O6 兼容接口 (control_robot_hand_position)")
    print("-" * 40)
    while True:
        choice = input("请输入选项: ").strip()
        if choice == '1':
            return 'l6'
        elif choice == '2':
            return 'o6'
        else:
            print("❌ 无效选项，请重新输入!")

def select_hand_type():
    """选择手类型"""
    print("\n" + "-" * 40)
    print("选择手类型:")
    print("  1. L6 手 (0-255范围, 255=完全张开)")
    print("  2. O6 手 (0-100范围, 0=完全张开)")
    print("-" * 40)
    while True:
        choice = input("请输入选项: ").strip()
        if choice == '1':
            return 'l6'
        elif choice == '2':
            return 'o6'
        else:
            print("❌ 无效选项，请重新输入!")


def set_duration():
    """设置持续时间"""
    print("\n" + "-" * 40)
    while True:
        try:
            duration = float(input("请输入持续时间(秒，0表示一直运行): ").strip())
            if duration >= 0:
                return duration
            else:
                print("❌ 持续时间不能为负数!")
        except ValueError:
            print("❌ 请输入有效的数字!")


def select_finger():
    """选择要测试的手指"""
    print("\n" + "-" * 40)
    print("选择要测试的手指:")
    for i, (key, name) in enumerate(FINGER_NAME_CN.items(), 1):
        print(f"  {i}. {name} ({key})")
    print("-" * 40)
    finger_list = list(FINGER_INDEX.keys())
    while True:
        try:
            choice = int(input("请输入选项: ").strip())
            if 1 <= choice <= len(finger_list):
                return finger_list[choice - 1]
            else:
                print(f"❌ 请输入1-{len(finger_list)}之间的数字!")
        except ValueError:
            print("❌ 请输入有效的数字!")



def input_finger_value(hand_type):
    """输入手指弯曲值"""
    config = HAND_CONFIG[hand_type]
    print("\n" + "-" * 40)
    while True:
        try:
            value = int(input(f"请输入弯曲值 ({config['open_value']}=完全张开, {config['close_value']}=完全握紧): ").strip())
            if config['min_value'] <= value <= config['max_value']:
                return value
            else:
                print(f"❌ 请输入{config['min_value']}-{config['max_value']}之间的数字!")
        except ValueError:
            print("❌ 请输入有效的数字!")


def create_joint_state_msg(hand_side, finger_values, joint_names_prefix):
    """创建 sensor_msgs/JointState 消息"""
    msg = JointState()
    msg.header.stamp = rospy.Time.now()

    # 原生L6接口只需要6个数值，对应6个手指的0-255控制值
    # 关节名称可以省略或使用简化名称
    msg.name = ["大拇指弯曲", "大拇指横摆", "食指弯曲", "中指弯曲", "无名指弯曲", "小拇指弯曲"]

    # 直接设置0-255的原始值，不需要转换为弧度
    msg.position = [float(v) for v in finger_values]

    return msg


def run_control(hand, interface, hand_type, finger=None, finger_value=None, all_value=None, duration=DEFAULT_DURATION, rate=None, l_pub=None, r_pub=None, o6_pub=None):
    """运行控制逻辑"""
    config = HAND_CONFIG[hand_type]

    # 设置控制值
    if all_value is not None:
        # 控制所有手指
        target_value = max(config['min_value'], min(config['max_value'], all_value))
        target_values = [target_value] * 6
        rospy.loginfo(f"▶ 开始控制{hand}手所有手指到 {target_value}")
    else:
        # 控制单个手指
        finger_idx = FINGER_INDEX[finger]
        target_value = max(config['min_value'], min(config['max_value'], finger_value))
        # 初始化所有手指为张开状态
        target_values = [config['open_value']] * 6
        target_values[finger_idx] = target_value
        rospy.loginfo(f"▶ 开始控制{hand}手的{FINGER_NAME_CN[finger]}弯曲到 {target_value}")

    # 构造消息
    l_msg = None
    r_msg = None
    o6_msg = None

    if interface == 'o6':
        # O6兼容接口 (control_robot_hand_position)
        o6_msg = robotHandPosition()
        o6_msg.header.stamp = rospy.Time.now()
        global current_left_pos, current_right_pos
        config = HAND_CONFIG[hand_type]
        open_value = config['open_value']

        # 初始化位置为张开状态（如果是第一次使用）
        if current_left_pos is None:
            current_left_pos = [open_value] * 6
        if current_right_pos is None:
            current_right_pos = [open_value] * 6

        # 根据选择的手设置对应的位置，未选择的手保持当前状态
        if hand in ['left', 'both']:
            o6_msg.left_hand_position = target_values
            # 更新当前左手位置记录
            current_left_pos = target_values.copy()
        else:
            # 左手保持当前位置
            o6_msg.left_hand_position = current_left_pos.copy()

        if hand in ['right', 'both']:
            o6_msg.right_hand_position = target_values
            # 更新当前右手位置记录
            current_right_pos = target_values.copy()
        else:
            # 右手保持当前位置
            o6_msg.right_hand_position = current_right_pos.copy()
    else:
        # L6原生接口
        l_msg = create_joint_state_msg('left', target_values, 'l_')
        r_msg = create_joint_state_msg('right', target_values, 'r_')

    if duration > 0:
        rospy.loginfo(f"⏱ 将持续 {duration} 秒后自动伸直")
    else:
        rospy.loginfo(f"⏱ 将持续运行，按 Ctrl+C 停止")

    try:
        start_time = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            # 检查是否达到持续时间
            if duration > 0 and (rospy.Time.now().to_sec() - start_time) >= duration:
                break

            if interface == 'o6':
                # 发布O6兼容接口消息
                o6_msg.header.stamp = rospy.Time.now()
                o6_pub.publish(o6_msg)
            else:
                # 发布L6原生接口消息
                if hand in ['left', 'both']:
                    l_msg.header.stamp = rospy.Time.now()
                    l_pub.publish(l_msg)
                if hand in ['right', 'both']:
                    r_msg.header.stamp = rospy.Time.now()
                    r_pub.publish(r_msg)

            rate.sleep()

    except KeyboardInterrupt:
        rospy.loginfo("\n⏹ 用户中断，停止控制")

    finally:
        # 退出前自动伸直所有手指，安全保护
        rospy.loginfo("⏳ 正在伸直手指...")
        open_value = config['open_value']

        if interface == 'o6':
            stop_o6_msg = robotHandPosition()
            stop_o6_msg.left_hand_position = [open_value] * 6
            stop_o6_msg.right_hand_position = [open_value] * 6
            stop_pub = o6_pub
        else:
            stop_l_msg = create_joint_state_msg('left', [open_value] * 6, 'l_') # 左手完全张开
            stop_r_msg = create_joint_state_msg('right', [open_value] * 6, 'r_') # 右手完全张开

        for _ in range(5):  # 连续发布 5 次确保指令送达
            if rospy.is_shutdown():
                break

            if interface == 'o6':
                stop_o6_msg.header.stamp = rospy.Time.now()
                stop_pub.publish(stop_o6_msg)
            else:
                if hand in ['left', 'both']:
                    stop_l_msg.header.stamp = rospy.Time.now()
                    l_pub.publish(stop_l_msg)
                if hand in ['right', 'both']:
                    stop_r_msg.header.stamp = rospy.Time.now()
                    r_pub.publish(stop_r_msg)

            rate.sleep()

        rospy.loginfo("✅ 控制结束")
        input("\n按回车键返回主菜单...")


def main():
    global current_left_pos, current_right_pos
    # 初始化ROS节点
    rospy.init_node('linker_dexhand_control_test', anonymous=True)

    # 创建发布者
    l_pub = rospy.Publisher('/cb_left_hand_control_cmd', JointState, queue_size=10)
    r_pub = rospy.Publisher('/cb_right_hand_control_cmd', JointState, queue_size=10)
    o6_pub = rospy.Publisher('/control_robot_hand_position', robotHandPosition, queue_size=10)
    rate = rospy.Rate(PUBLISH_RATE)

    # 初始化当前位置为None，第一次使用时根据手类型初始化为张开值
    current_left_pos = None
    current_right_pos = None

    # 当前配置
    current_hand = DEFAULT_HAND
    current_interface = DEFAULT_INTERFACE
    current_hand_type = DEFAULT_HAND_TYPE
    current_duration = DEFAULT_DURATION

    rospy.loginfo("✅ Linker 灵巧手控制节点初始化完成")

    try:
        while not rospy.is_shutdown():
            choice = show_menu(current_hand, current_interface, current_duration, current_hand_type)

            if choice == '0':
                # 退出
                print("\n👋 感谢使用，再见!")
                break

            elif choice == '1':
                # 选择手
                current_hand = select_hand()
                print(f"✅ 已选择控制: {current_hand}")

            elif choice == '2':
                # 选择接口
                current_interface = select_interface()
                print(f"✅ 已选择接口: {current_interface}")

            elif choice == '3':
                # 选择手类型
                current_hand_type = select_hand_type()
                print(f"✅ 已选择手类型: {HAND_CONFIG[current_hand_type]['description']}")
                # 切换手类型后重置当前位置记录，使用新手类型的张开值
                open_value = HAND_CONFIG[current_hand_type]['open_value']
                current_left_pos = [open_value] * 6
                current_right_pos = [open_value] * 6

            elif choice == '4':
                # 设置持续时间
                current_duration = set_duration()
                print(f"✅ 已设置持续时间: {current_duration}s")

            elif choice == '5':
                # 测试单个手指
                finger = select_finger()
                value = input_finger_value(current_hand_type)
                clear_screen()
                run_control(
                    hand=current_hand,
                    interface=current_interface,
                    hand_type=current_hand_type,
                    finger=finger,
                    finger_value=value,
                    duration=current_duration,
                    rate=rate,
                    l_pub=l_pub,
                    r_pub=r_pub,
                    o6_pub=o6_pub
                )

            elif choice == '6':
                # 测试所有手指 (张开)
                clear_screen()
                open_value = HAND_CONFIG[current_hand_type]['open_value']
                run_control(
                    hand=current_hand,
                    interface=current_interface,
                    hand_type=current_hand_type,
                    all_value=open_value,
                    duration=current_duration,
                    rate=rate,
                    l_pub=l_pub,
                    r_pub=r_pub,
                    o6_pub=o6_pub
                )

            elif choice == '7':
                # 测试所有手指 (握紧)
                clear_screen()
                close_value = HAND_CONFIG[current_hand_type]['close_value']
                run_control(
                    hand=current_hand,
                    interface=current_interface,
                    hand_type=current_hand_type,
                    all_value=close_value,
                    duration=current_duration,
                    rate=rate,
                    l_pub=l_pub,
                    r_pub=r_pub,
                    o6_pub=o6_pub
                )

            else:
                print("\n❌ 无效选项，请重新输入!")
                input("按回车键继续...")

    except KeyboardInterrupt:
        print("\n👋 用户中断，退出程序")


if __name__ == '__main__':
    main()
