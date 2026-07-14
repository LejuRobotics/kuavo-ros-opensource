#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import os
from kuavo_msgs.msg import robotHandPosition, dexhandCommand
from std_msgs.msg import Header

# ====================== 可配置默认参数 (用户可自行修改) ======================
DEFAULT_DURATION = 10          # 默认持续时间(秒)
DEFAULT_ALL_FINGER_VALUE = 50  # 所有手指默认值 (范围0-100，中值)
DEFAULT_HAND = 'left'          # 默认控制的手
DEFAULT_INTERFACE = 'old'      # 默认接口 ('old'或'new')
PUBLISH_RATE = 10              # 发布频率(Hz)
# ============================================================================

# 手指名称到索引的映射
FINGER_INDEX = {
    'thumb': 0,      # 拇指
    'thumb_aux': 1,  # 拇指辅助关节
    'index': 2,      # 食指
    'middle': 3,     # 中指
    'ring': 4,       # 无名指
    'little': 5      # 小指
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


def clear_screen():
    """清屏"""
    os.system('cls' if os.name == 'nt' else 'clear')


def show_menu(current_hand, current_interface, current_duration):
    """显示主菜单"""
    clear_screen()
    print("=" * 50)
    print("          🤖 灵巧手控制测试工具 (0-100%范围)")
    print("=" * 50)
    print(f" 当前配置: 手={current_hand} | 接口={current_interface} | 持续时间={current_duration}s")
    print("-" * 50)
    print("  1. 🖐️  选择控制的手 (左手/右手/双手)")
    print("  2. 🔌 选择控制接口")
    print("  3. ⏱️  设置持续时间")
    print("-" * 50)
    print("  4. 👉 测试单个手指")
    print(f"  5. 👐 测试所有手指 (默认值={DEFAULT_ALL_FINGER_VALUE}%)")
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
    print("  1. 旧版接口 (/control_robot_hand_position)")
    print("  2. 新版接口 (/dexhand/command)")
    print("-" * 40)
    while True:
        choice = input("请输入选项: ").strip()
        if choice == '1':
            return 'old'
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
            value = int(input("请输入弯曲值 (0=完全伸直, 100=完全握紧): ").strip())
            if 0 <= value <= 100:
                return value
            else:
                print("❌ 请输入0-100之间的数字!")
        except ValueError:
            print("❌ 请输入有效的数字!")


def run_control(hand, interface, finger=None, finger_value=None, all_value=None, duration=DEFAULT_DURATION, rate=None, pub_old=None, pub_new=None):
    """运行控制逻辑"""
    # 构造控制消息
    msg = robotHandPosition()
    msg.left_hand_position = [0] * 6
    msg.right_hand_position = [0] * 6

    # 设置控制值
    if all_value is not None:
        # 控制所有手指
        target_values = [max(0, min(100, all_value))] * 6
        if hand in ['left', 'both']:
            msg.left_hand_position = target_values
        if hand in ['right', 'both']:
            msg.right_hand_position = target_values
        rospy.loginfo(f"▶ 开始控制{hand}手所有手指到 {all_value}%")
    else:
        # 控制单个手指
        finger_idx = FINGER_INDEX[finger]
        target_value = max(0, min(100, finger_value))
        if hand in ['left', 'both']:
            msg.left_hand_position[finger_idx] = target_value
        if hand in ['right', 'both']:
            msg.right_hand_position[finger_idx] = target_value
        rospy.loginfo(f"▶ 开始控制{hand}手的{FINGER_NAME_CN[finger]}弯曲到 {target_value}%")

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

            if interface == 'old':
                # 仅发布旧版接口消息
                pub_old.publish(msg)
            else:
                # 仅发布新版dexhandCommand消息
                dexhand_msg = dexhandCommand()
                dexhand_msg.header = Header()
                dexhand_msg.header.stamp = rospy.Time.now()
                dexhand_msg.control_mode = dexhandCommand.POSITION_CONTROL  # 使用位置控制模式

                # 新版接口总是需要12个数据：前6个左手，后6个右手
                # 不需要控制的手保持0值（伸直）
                dexhand_msg.data = [0] * 12
                if hand in ['left', 'both']:
                    dexhand_msg.data[0:6] = msg.left_hand_position
                if hand in ['right', 'both']:
                    dexhand_msg.data[6:12] = msg.right_hand_position

                pub_new.publish(dexhand_msg)

            rate.sleep()

    except KeyboardInterrupt:
        rospy.loginfo("\n⏹ 用户中断，停止控制")

    finally:
        # 退出前自动伸直所有手指，安全保护
        rospy.loginfo("⏳ 正在伸直手指...")
        stop_msg = robotHandPosition()
        stop_msg.left_hand_position = [0] * 6
        stop_msg.right_hand_position = [0] * 6

        # 构造新版停止消息
        stop_dexhand_msg = dexhandCommand()
        stop_dexhand_msg.header = Header()
        stop_dexhand_msg.control_mode = dexhandCommand.POSITION_CONTROL
        stop_dexhand_msg.data = [0] * 12  # 12个数据全部置0，双手都伸直

        for _ in range(5):  # 连续发布5次确保指令送达
            if rospy.is_shutdown():
                break

            if interface == 'old':
                # 仅发布旧版停止消息
                pub_old.publish(stop_msg)
            else:
                # 仅发布新版停止消息
                stop_dexhand_msg.header.stamp = rospy.Time.now()
                pub_new.publish(stop_dexhand_msg)

            rate.sleep()
        rospy.loginfo("✅ 控制结束")
        input("\n按回车键返回主菜单...")


def main():
    # 初始化ROS节点
    rospy.init_node('dexhand_control_test', anonymous=True)

    # 创建发布者
    pub_old = rospy.Publisher('/control_robot_hand_position',
                         robotHandPosition, queue_size=10)
    pub_new = rospy.Publisher('/dexhand/command',
                         dexhandCommand, queue_size=10)
    rate = rospy.Rate(PUBLISH_RATE)

    # 当前配置
    current_hand = DEFAULT_HAND
    current_interface = DEFAULT_INTERFACE
    current_duration = DEFAULT_DURATION

    rospy.loginfo("✅ 灵巧手控制节点初始化完成")

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
                    pub_old=pub_old,
                    pub_new=pub_new
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
                    pub_old=pub_old,
                    pub_new=pub_new
                )

            else:
                print("\n❌ 无效选项，请重新输入!")
                input("按回车键继续...")

    except KeyboardInterrupt:
        print("\n👋 用户中断，退出程序")


if __name__ == '__main__':
    main()
