#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import sys
import os

# 添加common目录到路径
current_dir = os.path.dirname(os.path.abspath(__file__))
common_dir = os.path.join(current_dir, 'common')
sys.path.append(common_dir)

from common_utils import SlaveTestLogger

def demo_slave_test():
    """演示slave测试的表格效果"""
    logger = SlaveTestLogger()
    
    # 初始化显示
    logger.initialize_display()
    
    # 模拟测试过程
    time.sleep(2)
    
    # 1. 上位机连接测试
    logger.start_test("1. 上位机连接测试")
    time.sleep(1)
    logger.print_connection_status("192.168.26.1", 22, "success")
    time.sleep(1)
    logger.update_test_status("✅", "连接成功")
    
    time.sleep(1)
    
    # 2. 音响服务测试
    logger.start_test("2. 音响服务测试")
    time.sleep(1)
    logger.print_service_test_result("play_music", "node_check", "success", "节点运行中")
    time.sleep(1)
    logger.update_test_status("✅", "音频播放正常")
    
    time.sleep(1)
    
    # 3. 麦克风服务测试
    logger.start_test("3. 麦克风服务测试")
    time.sleep(1)
    logger.print_service_test_result("record_music", "node_check", "success", "节点运行中")
    time.sleep(1)
    logger.update_test_status("✅", "音频录制正常")
    
    time.sleep(1)
    
    # 4. 相机数据测试
    logger.start_test("4. 相机数据测试")
    time.sleep(1)
    logger.print_sensor_data_status("相机", "RGB图像", "success")
    time.sleep(1)
    logger.print_sensor_data_status("相机", "深度图像", "success")
    time.sleep(1)
    logger.update_test_status("✅", "图像数据正常")
    
    time.sleep(1)
    
    # 5. 雷达数据测试
    logger.start_test("5. 雷达数据测试")
    time.sleep(1)
    logger.print_sensor_data_status("雷达", "点云数据", "success")
    time.sleep(1)
    logger.update_test_status("✅", "点云数据正常")
    
    print("\n演示完成！")

if __name__ == "__main__":
    demo_slave_test() 