#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import sys
from led_controller import LEDController

def main():
    # 初始化LED控制器，默认使用/dev/ttyLED0端口
    try:
        led_controller = LEDController("/dev/ttyLED0")
        
        # 测试不同的LED模式
        print("测试常亮模式 (红色)")
        led_controller.set_led_mode(0, [(255, 0, 0)] * 10)  # 所有LED设置为红色常亮
        time.sleep(3)
        
        print("测试呼吸模式 (绿色)")
        led_controller.set_led_mode(1, [(0, 255, 0)] * 10)  # 所有LED设置为绿色呼吸
        time.sleep(5)
        
        print("测试快闪模式 (蓝色)")
        led_controller.set_led_mode(2, [(0, 0, 255)] * 10)  # 所有LED设置为蓝色快闪
        time.sleep(5)
        
        print("测试律动模式 (彩色)")
        # 创建彩虹颜色序列
        rainbow = [
            (255, 0, 0),    # 红
            (255, 127, 0),  # 橙
            (255, 255, 0),  # 黄
            (0, 255, 0),    # 绿
            (0, 0, 255),    # 蓝
            (75, 0, 130),   # 靛
            (148, 0, 211),  # 紫
            (255, 0, 127),  # 粉
            (255, 255, 255),# 白
            (0, 255, 255)   # 青
        ]
        led_controller.set_led_mode(3, rainbow)  # 设置为彩色律动
        time.sleep(5)
        
        # 关闭所有LED
        print("关闭所有LED")
        led_controller.deinit()
        
    except KeyboardInterrupt:
        print("程序被用户中断")
        if 'led_controller' in locals():
            led_controller.deinit()
    except Exception as e:
        print(f"发生错误: {e}")
        if 'led_controller' in locals():
            led_controller.deinit()

if __name__ == "__main__":
    main()
