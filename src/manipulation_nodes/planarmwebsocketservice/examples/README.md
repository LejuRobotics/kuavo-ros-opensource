#!/usr/bin/env python2
# -*- coding: utf-8 -*-

"""
WebSocket API 测试脚本使用说明
=================================

本目录包含了针对8888端口WebSocket服务器的所有API接口测试脚本。

文件列表:
---------
1. get_robot_info.py - 获取机器人信息和状态
2. node_operations.py - 节点操作（启动和停止脚本）
3. preview_action.py - 预览动作
4. stop_preview_action.py - 停止预览动作
5. music_operations.py - 音乐相关操作（获取列表和检查路径）
6. update_h12_config.py - 更新H12遥控器配置
7. update_data_pilot.py - 更新数据采集程序
8. adjust_zero_point.py - 调整零点
9. set_zero_point.py - 设置零点
10. execute_python_script.py - 执行Python脚本
11. execute_demo.py - 执行演示程序
12. stop_execute_demo.py - 停止演示程序
13. map_operations.py - 地图相关操作（获取地图、加载地图、获取位置）
14. init_localization_by_pose.py - 通过位姿初始化定位
15. get_apis.py - 测试所有GET类型API接口

使用方法:
--------

1. 确保WebSocket服务器正在运行在8888端口
2. 安装依赖: pip install websocket-client
3. 运行单个测试 (默认连接本地127.0.0.1):
   python2 get_apis.py

4. 指定服务器IP地址:
   python2 get_apis.py --ip 192.168.1.100

5. 运行所有GET类型测试:
   python2 get_apis.py

注意事项:
--------
1. 部分接口需要根据实际情况修改参数，如:
   - preview_action.py 中的 action_filename 和 action_file_MD5
   - adjust_zero_point.py 中的 motor_index 和 adjust_pos
   - set_zero_point.py 中的 zero_pos 数组
   - map_operations.py 中的 map_name
   - init_localization_by_pose.py 中的位姿参数

2. 需要Python2环境和websocket-client库

3. 确保网络连接正常，能够访问目标服务器IP的8888端口

4. 某些接口可能需要特定的ROS环境或硬件支持

5. 所有脚本都支持 --ip 参数指定WebSocket服务器IP地址，默认为127.0.0.1

6. 合并的接口说明:
   - get_robot_info.py: 包含get_robot_info和get_robot_status
   - node_operations.py: 包含run_node和stop_run_node（间隔10秒）
   - music_operations.py: 包含get_music_list和check_music_path
   - map_operations.py: 包含get_all_maps、load_map和get_robot_position

接口说明:
--------
所有接口都使用JSON格式进行通信，基本结构为:

请求格式:
{
    "cmd": "command_name",
    "data": {
        // 参数根据具体接口而定
    }
}

响应格式:
{
    "cmd": "command_name",
    "data": {
        "code": 0,  // 0表示成功，其他值表示失败
        // 其他响应数据
    }
}
"""