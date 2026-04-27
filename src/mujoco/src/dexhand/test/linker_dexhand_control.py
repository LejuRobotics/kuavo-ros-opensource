#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
L6/O6 灵巧手控制测试脚本
控制范围：0-255 (255=完全张开，0=完全握紧)
"""
import rospy
import os
import argparse
from sensor_msgs.msg import JointState
from kuavo_msgs.msg import dexhandCommand

# ====================== 可配置默认参数 (用户可自行修改) ======================
DEFAULT_DURATION = 10          # 默认持续时间(秒)
DEFAULT_ALL_FINGER_VALUE = 0 # 所有手指默认张开值 (范围0-255，255=完全张开)
DEFAULT_HAND = 'left'          # 默认控制的手
DEFAULT_INTERFACE = 'l6'       # 默认接口 ('l6'=L6原生接口, 'new'=新版统一接口)
PUBLISH_RATE = 10              # 发布频率(Hz)
L6_MIN_VALUE = 0
L6_MAX_VALUE = 255
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

# 控制模式常量
POSITION_CONTROL = 0
VELOCITY_CONTROL = 1


def clear_screen():
    """清屏"""
    os.system('cls' if os.name == 'nt' else 'clear')


def show_menu(current_hand, current_interface, current_duration):
    """显示主菜单"""
    clear_screen()
    print("=" * 50)
    print("        🤖 L6/O6 灵巧手控制测试工具 (0-255范围)")
    print("=" * 50)
    print(f" 当前配置: 手={current_hand} | 接口={current_interface} | 持续时间={current_duration}s")
    print("-" * 50)
    print("  1. 🖐️  选择控制的手 (左手/右手/双手)")
    print("  2. 🔌 选择控制接口")
    print("  3. ⏱️  设置持续时间")
    print("-" * 50)
    print("  4. 👉 测试单个手指")
    print(f"  5. 👐 测试所有手指 (默认值={DEFAULT_ALL_FINGER_VALUE})")
    print("-" * 50)
    print("  0. ❌ 退出程序")
    print("=" * 50)
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
    print("  1. L6 原生接口 (/cb_l_hand_control_cmd, /cb_r_hand_control_cmd)")
    print("  2. 新版统一接口 (/dexhand/command)")
    print("-" * 40)
    while True:
        choice = input("请输入选项: ").strip()
        if choice == '1':
            return 'l6'
        elif choice == '2':
            return 'new'
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


def input_finger_value():
    """输入手指弯曲值"""
    print("\n" + "-" * 40)
    while True:
        try:
            value = int(input("请输入弯曲值 (255=完全张开, 0=完全握紧): ").strip())
            if L6_MIN_VALUE <= value <= L6_MAX_VALUE:
                return value
            else:
                print(f"❌ 请输入{L6_MIN_VALUE}-{L6_MAX_VALUE}之间的数字!")
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


def run_control(hand, interface, finger=None, finger_value=None, all_value=None, duration=DEFAULT_DURATION, rate=None, l_pub=None, r_pub=None, new_pub=None):
    """运行控制逻辑"""
    # 设置控制值
    if all_value is not None:
        # 控制所有手指
        target_value = max(L6_MIN_VALUE, min(L6_MAX_VALUE, all_value))
        target_values = [target_value] * 6
        rospy.loginfo(f"▶ 开始控制{hand}手所有手指到 {target_value}")
    else:
        # 控制单个手指
        finger_idx = FINGER_INDEX[finger]
        target_value = max(L6_MIN_VALUE, min(L6_MAX_VALUE, finger_value))
        # 初始化所有手指为 255（保持张开）
        target_values = [255] * 6
        target_values[finger_idx] = target_value
        rospy.loginfo(f"▶ 开始控制{hand}手的{FINGER_NAME_CN[finger]}弯曲到 {target_value}")

    # 构造消息
    if interface == 'new':
        # 新版接口
        msg = dexhandCommand()
        msg.control_mode = POSITION_CONTROL
        msg.data = [0] * 12  # 左手 6 个 + 右手 6 个
        if hand in ['left', 'both']:
            msg.data[0:6] = target_values
        if hand in ['right', 'both']:
            msg.data[6:12] = target_values
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

            if interface == 'new':
                # 发布新版接口消息
                new_pub.publish(msg)
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
        if interface == 'new':
            stop_msg = dexhandCommand()
            stop_msg.control_mode = POSITION_CONTROL
            stop_msg.data = [255] * 12 # 所有手指完全张开
            stop_pub = new_pub
        else:
            stop_l_msg = create_joint_state_msg('left', [255] * 6, 'l_') # 左手完全张开
            stop_r_msg = create_joint_state_msg('right', [255] * 6, 'r_') # 右手完全张开

        for _ in range(5):  # 连续发布 5 次确保指令送达
            if rospy.is_shutdown():
                break

            if interface == 'new':
                stop_pub.publish(stop_msg)
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
    # 初始化ROS节点
    rospy.init_node('l6_dexhand_control_test', anonymous=True)

    # 创建发布者
    l_pub = rospy.Publisher('/cb_l_hand_control_cmd', JointState, queue_size=10)
    r_pub = rospy.Publisher('/cb_r_hand_control_cmd', JointState, queue_size=10)
    new_pub = rospy.Publisher('/dexhand/command', dexhandCommand, queue_size=10)
    rate = rospy.Rate(PUBLISH_RATE)

    # 当前配置
    current_hand = DEFAULT_HAND
    current_interface = DEFAULT_INTERFACE
    current_duration = DEFAULT_DURATION

    rospy.loginfo("✅ L6 灵巧手控制节点初始化完成")

    try:
        while not rospy.is_shutdown():
            choice = show_menu(current_hand, current_interface, current_duration)

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
                # 设置持续时间
                current_duration = set_duration()
                print(f"✅ 已设置持续时间: {current_duration}s")

            elif choice == '4':
                # 测试单个手指
                finger = select_finger()
                value = input_finger_value()
                clear_screen()
                run_control(
                    hand=current_hand,
                    interface=current_interface,
                    finger=finger,
                    finger_value=value,
                    duration=current_duration,
                    rate=rate,
                    l_pub=l_pub,
                    r_pub=r_pub,
                    new_pub=new_pub
                )

            elif choice == '5':
                # 测试所有手指
                clear_screen()
                run_control(
                    hand=current_hand,
                    interface=current_interface,
                    all_value=DEFAULT_ALL_FINGER_VALUE,
                    duration=current_duration,
                    rate=rate,
                    l_pub=l_pub,
                    r_pub=r_pub,
                    new_pub=new_pub
                )

            else:
                print("\n❌ 无效选项，请重新输入!")
                input("按回车键继续...")

    except KeyboardInterrupt:
        print("\n👋 用户中断，退出程序")


if __name__ == '__main__':
    main()
