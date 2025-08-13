#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import websocket
import json
import time
import sys
import argparse

def test_get_apis(ip="127.0.0.1"):
    """
    测试所有GET类型的WebSocket API接口
    """
    print("=== 开始测试所有GET类型WebSocket API接口 ===")
    
    # 创建WebSocket连接
    ws = websocket.create_connection("ws://{}:8888".format(ip))
    
    # 测试接口列表 - 只包含获取数据的接口
    test_cases = [
        {
            "name": "获取机器人信息",
            "request": {
                "cmd": "get_robot_info"
            }
        },
        {
            "name": "获取机器人状态",
            "request": {
                "cmd": "get_robot_status"
            }
        },
        {
            "name": "获取零点",
            "request": {
                "cmd": "get_zero_point"
            }
        },
        {
            "name": "获取音乐列表",
            "request": {
                "cmd": "get_music_list",
                "data": {}
            }
        },
        {
            "name": "获取所有地图",
            "request": {
                "cmd": "get_all_maps"
            }
        },
        {
            "name": "获取机器人位置",
            "request": {
                "cmd": "get_robot_position"
            }
        }
    ]
    
    # 执行测试
    for i, test_case in enumerate(test_cases, 1):
        print("\n--- 测试 {}: {} ---".format(i, test_case["name"]))
        
        try:
            print("发送请求:", json.dumps(test_case["request"], ensure_ascii=False))
            ws.send(json.dumps(test_case["request"]))
            
            # 接收响应
            response = ws.recv()
            print("收到响应:", response)
            
        except Exception as e:
            print("测试失败:", str(e))
    
    ws.close()
    print("\n=== 所有GET接口测试完成 ===")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="测试所有GET类型WebSocket API接口")
    parser.add_argument("--ip", default="127.0.0.1", help="WebSocket服务器IP地址 (默认: 127.0.0.1)")
    args = parser.parse_args()
    
    test_get_apis(args.ip)