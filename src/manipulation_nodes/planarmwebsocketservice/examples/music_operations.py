#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import websocket
import json
import time
import sys
import argparse

def test_music_operations(ip="127.0.0.1"):
    """
    测试音乐相关操作接口
    """
    print("=== 测试音乐相关操作 ===")
    
    # 创建WebSocket连接
    ws = websocket.create_connection("ws://{}:8888".format(ip))
    
    # 测试获取音乐列表
    print("\n--- 测试获取音乐列表 ---")
    request1 = {
        "cmd": "get_music_list",
        "data": {}
    }
    
    print("发送请求:", json.dumps(request1, ensure_ascii=False))
    ws.send(json.dumps(request1))
    
    # 接收响应
    response1 = ws.recv()
    print("收到响应:", response1)
    
    # 测试检查音乐路径
    print("\n--- 测试检查音乐路径 ---")
    request2 = {
        "cmd": "check_music_path",
        "data": {
            "is_reset_cmd": False,
            "music_filename": "music.mp3"
        }
    }
    
    print("发送请求:", json.dumps(request2, ensure_ascii=False))
    ws.send(json.dumps(request2))
    
    # 接收响应
    response2 = ws.recv()
    print("收到响应:", response2)
    
    ws.close()
    print("=== 测试完成 ===\n")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="测试音乐相关操作接口")
    parser.add_argument("--ip", default="127.0.0.1", help="WebSocket服务器IP地址 (默认: 127.0.0.1)")
    args = parser.parse_args()
    
    test_music_operations(args.ip)