#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LED 灯带测试程序

测试流程:
1. 常亮模式 3 秒的红灯
2. 呼吸模式 3 秒的绿灯
3. 闪烁模式 3 秒的蓝灯
4. 律动模式 3 秒（颜色数据无效，硬件自动生成效果）
5. 关闭所有灯
"""

import sys
import os
import time

# 添加父目录到路径,以便导入控制模块
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from led.led_strip import LEDStrip, LEDMode


def test_led_sequence():
    """执行 LED 测试序列"""

    print("=" * 50)
    print("LED 灯带测试程序")
    print("=" * 50)

    # 初始化 LED 灯带 (内部会自动创建串口)
    print("\n[1] 初始化 LED 灯带...")
    led_strip = LEDStrip(port='/dev/ttyLED0', baudrate=115200)

    if not led_strip._serial_port.is_open():
        print("[错误] 串口未成功打开,测试终止")
        print("请检查:")
        print("  - 串口设备是否存在 (/dev/ttyLED0)")
        print("  - 权限是否正确")
        print("  - 是否被其他程序占用")
        return

    try:
        # 测试 1: 常亮模式 - 红灯 (3秒)
        print("\n[2] 常亮模式 - 红灯 (3秒)")
        red_colors = [(255, 0, 0)] * LEDStrip.LED_COUNT
        success = led_strip.set_mode_and_color(LEDMode.CONSTANT, red_colors)
        print(f"  设置结果: {'成功' if success else '失败'}")
        time.sleep(3)

        # 测试 2: 呼吸模式 - 绿灯 (3秒)
        print("\n[3] 呼吸模式 - 绿灯 (3秒)")
        green_colors = [(0, 255, 0)] * LEDStrip.LED_COUNT
        success = led_strip.set_mode_and_color(LEDMode.BREATHING, green_colors)
        print(f"  设置结果: {'成功' if success else '失败'}")
        time.sleep(3)

        # 测试 3: 闪烁模式 - 蓝灯 (3秒)
        print("\n[4] 闪烁模式 - 蓝灯 (3秒)")
        blue_colors = [(0, 0, 255)] * LEDStrip.LED_COUNT
        success = led_strip.set_mode_and_color(LEDMode.FLASH, blue_colors)
        print(f"  设置结果: {'成功' if success else '失败'}")
        time.sleep(3)

        # 测试 4: 律动模式 (3秒) - 颜色数据不起作用
        print("\n[5] 律动模式 (3秒) - 硬件自动生成效果")
        # 律动模式颜色数据不起作用，传入任意值都会被忽略
        dummy_colors = [(0, 0, 0)] * LEDStrip.LED_COUNT
        success = led_strip.set_mode_and_color(LEDMode.RHYTHM, dummy_colors)
        print(f"  设置结果: {'成功' if success else '失败'}")
        time.sleep(3)

        # 测试结束: 关闭所有LED
        print("\n[6] 关闭所有LED...")
        success = led_strip.close()
        print(f"  关闭结果: {'成功' if success else '失败'}")

    except KeyboardInterrupt:
        print("\n\n[中断] 测试被用户中断")
        # 中断时也要关闭LED
        print("正在关闭所有LED...")
        led_strip.close()
    except Exception as e:
        print(f"\n\n[错误] 测试过程中发生错误: {e}")
        import traceback
        traceback.print_exc()
        # 发生错误时也要关闭LED
        print("正在关闭所有LED...")
        led_strip.close()

    print("\n" + "=" * 50)
    print("测试序列执行完毕!")
    print("=" * 50)


if __name__ == '__main__':
    test_led_sequence()
