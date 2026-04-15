#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LED Strip ROS 服务测试代码

测试流程:
1. 常亮模式 3 秒的红灯
2. 呼吸模式 3 秒的绿灯
3. 闪烁模式 3 秒的蓝灯
4. 律动模式 3 秒（颜色数据无效，硬件自动生成效果）
5. 关闭所有灯
"""

import rospy
from kuavo_msgs.srv import SetLEDMode_free
from kuavo_msgs.msg import Color
from std_srvs.srv import Trigger


def create_colors(r, g, b):
    """创建 24 个灯的颜色列表"""
    colors = []
    for i in range(24):
        color_msg = Color()
        color_msg.r = r
        color_msg.g = g
        color_msg.b = b
        colors.append(color_msg)
    return colors


def test_constant_red():
    """测试 1: 常亮模式 - 红灯 (3秒)"""
    print("\n[2] 常亮模式 - 红灯 (3秒)")

    rospy.wait_for_service('/led_strip_set_mode_and_color')
    set_mode_service = rospy.ServiceProxy('/led_strip_set_mode_and_color', SetLEDMode_free)

    colors = create_colors(255, 0, 0)  # 红色

    try:
        response = set_mode_service(mode=0, colors=colors)  # 0 = CONSTANT
        print(f"  设置结果: {'成功' if response.success else '失败'}")
        return response.success
    except rospy.ServiceException as e:
        print(f"  服务调用失败: {e}")
        return False


def test_breathing_green():
    """测试 2: 呼吸模式 - 绿灯 (3秒)"""
    print("\n[3] 呼吸模式 - 绿灯 (3秒)")

    rospy.wait_for_service('/led_strip_set_mode_and_color')
    set_mode_service = rospy.ServiceProxy('/led_strip_set_mode_and_color', SetLEDMode_free)

    colors = create_colors(0, 255, 0)  # 绿色

    try:
        response = set_mode_service(mode=1, colors=colors)  # 1 = BREATHING
        print(f"  设置结果: {'成功' if response.success else '失败'}")
        return response.success
    except rospy.ServiceException as e:
        print(f"  服务调用失败: {e}")
        return False


def test_flash_blue():
    """测试 3: 闪烁模式 - 蓝灯 (3秒)"""
    print("\n[4] 闪烁模式 - 蓝灯 (3秒)")

    rospy.wait_for_service('/led_strip_set_mode_and_color')
    set_mode_service = rospy.ServiceProxy('/led_strip_set_mode_and_color', SetLEDMode_free)

    colors = create_colors(0, 0, 255)  # 蓝色

    try:
        response = set_mode_service(mode=2, colors=colors)  # 2 = FLASH
        print(f"  设置结果: {'成功' if response.success else '失败'}")
        return response.success
    except rospy.ServiceException as e:
        print(f"  服务调用失败: {e}")
        return False


def test_rhythm():
    """测试 4: 律动模式 (3秒) - 颜色数据无效，硬件自动生成效果"""
    print("\n[5] 律动模式 (3秒) - 硬件自动生成效果")

    rospy.wait_for_service('/led_strip_set_mode_and_color')
    set_mode_service = rospy.ServiceProxy('/led_strip_set_mode_and_color', SetLEDMode_free)

    # 律动模式颜色数据不起作用，传入任意值都会被忽略
    colors = create_colors(0, 0, 0)

    try:
        response = set_mode_service(mode=3, colors=colors)  # 3 = RHYTHM
        print(f"  设置结果: {'成功' if response.success else '失败'}")
        return response.success
    except rospy.ServiceException as e:
        print(f"  服务调用失败: {e}")
        return False


def test_close():
    """测试 5: 关闭所有灯"""
    print("\n[6] 关闭所有LED...")

    rospy.wait_for_service('/led_strip_close')
    close_service = rospy.ServiceProxy('/led_strip_close', Trigger)

    try:
        response = close_service()
        print(f"  关闭结果: {'成功' if response.success else '失败'}")
        return response.success
    except rospy.ServiceException as e:
        print(f"  服务调用失败: {e}")
        return False


def main():
    """主测试函数"""
    # 初始化 ROS 节点
    rospy.init_node('led_strip_service_test')

    print("\n" + "=" * 50)
    print("LED Strip 服务测试程序")
    print("=" * 50)

    print("\n[1] 等待服务可用...")
    try:
        rospy.wait_for_service('/led_strip_set_mode_and_color', timeout=5.0)
        rospy.wait_for_service('/led_strip_close', timeout=5.0)
    except rospy.ROSException as e:
        print("[错误] 服务未可用,测试终止")
        print("请确保服务已启动:")
        print("  rosrun kuavo_led_controller led_strip_service.py")
        return

    try:
        # 测试 1: 常亮模式 - 红灯 (3秒)
        success1 = test_constant_red()
        rospy.sleep(3)

        # 测试 2: 呼吸模式 - 绿灯 (3秒)
        success2 = test_breathing_green()
        rospy.sleep(3)

        # 测试 3: 闪烁模式 - 蓝灯 (3秒)
        success3 = test_flash_blue()
        rospy.sleep(3)

        # 测试 4: 律动模式 (3秒)
        success4 = test_rhythm()
        rospy.sleep(3)

        # 测试 5: 关闭所有LED
        success5 = test_close()

    except KeyboardInterrupt:
        print("\n\n[中断] 测试被用户中断")
        # 中断时也要关闭LED
        try:
            test_close()
        except:
            pass
    except Exception as e:
        print(f"\n\n[错误] 测试过程中发生错误: {e}")
        import traceback
        traceback.print_exc()
        # 发生错误时也要关闭LED
        try:
            test_close()
        except:
            pass

    # 打印测试结果
    print("\n" + "=" * 50)
    print("测试序列执行完毕!")
    print("=" * 50)

    print("\n测试结果汇总:")
    print(f"测试 1 (常亮-红灯): {'✓ 通过' if success1 else '✗ 失败'}")
    print(f"测试 2 (呼吸-绿灯): {'✓ 通过' if success2 else '✗ 失败'}")
    print(f"测试 3 (闪烁-蓝灯): {'✓ 通过' if success3 else '✗ 失败'}")
    print(f"测试 4 (律动模式): {'✓ 通过' if success4 else '✗ 失败'}")
    print(f"测试 5 (关闭LED): {'✓ 通过' if success5 else '✗ 失败'}")
    print("=" * 50)

    if all([success1, success2, success3, success4, success5]):
        print("✓ 所有测试通过！")
    else:
        print("✗ 部分测试失败，请检查服务是否正常运行")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print("\n测试被中断")
