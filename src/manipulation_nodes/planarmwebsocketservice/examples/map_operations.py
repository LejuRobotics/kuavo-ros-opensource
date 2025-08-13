#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import websocket
import json
import time
import sys
import argparse

def test_map_operations(ip="127.0.0.1"):
    """
    测试地图相关操作接口
    """
    print("=== 测试地图相关操作 ===")
    
    # 创建WebSocket连接
    ws = websocket.create_connection("ws://{}:8888".format(ip))
    
    # 测试获取所有地图
    print("\n--- 测试获取所有地图 ---")
    request1 = {
        "cmd": "get_all_maps"
    }
    
    print("发送请求:", json.dumps(request1, ensure_ascii=False))
    ws.send(json.dumps(request1))
    
    # 接收响应
    response1 = ws.recv()
    print("收到响应:", response1)
    
    # 测试加载地图
    print("\n--- 测试加载地图 ---")
    request2 = {
        "cmd": "load_map",
        "data": {
            "map_name": "your_map_name"
        }
    }
    
    print("发送请求:", json.dumps(request2, ensure_ascii=False))
    ws.send(json.dumps(request2))
    
    # 接收响应
    response2 = ws.recv()
    print("收到响应:", response2)
    
    # 测试获取机器人位置
    print("\n--- 测试获取机器人位置 ---")
    request3 = {
        "cmd": "get_robot_position"
    }
    
    print("发送请求:", json.dumps(request3, ensure_ascii=False))
    ws.send(json.dumps(request3))
    
    # 接收响应
    response3 = ws.recv()
    print("收到响应:", response3)
    
    ws.close()
    print("=== 测试完成 ===\n")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="测试地图相关操作接口")
    parser.add_argument("--ip", default="127.0.0.1", help="WebSocket服务器IP地址 (默认: 127.0.0.1)")
    args = parser.parse_args()
    
    test_map_operations(args.ip)