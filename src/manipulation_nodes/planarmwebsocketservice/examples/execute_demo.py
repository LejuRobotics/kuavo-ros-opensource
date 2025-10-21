#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import websocket
import json
import argparse

def test_execute_trace_path_demo(ip="127.0.0.1"):
    """
    测试执行S型曲线行走演示程序
    """
    print("=== 测试执行S型曲线行走演示程序 ===")
    
    # 创建WebSocket连接
    ws = websocket.create_connection("ws://{}:8888".format(ip))
    
    # 构造请求消息
    request = {
        "cmd": "execute_demo",
        "data": {
            "demo_name": "trace_path",
            "parameters": {
                "--length": 4.0,
                "--amplitude": 2.0,
                "--half_scurve": False
            }
        }
    }
    
    print("发送请求:", json.dumps(request, ensure_ascii=False))
    ws.send(json.dumps(request))
    
    # 接收响应
    response = ws.recv()
    print("收到响应:", response)
    
    ws.close()
    print("=== 测试完成 ===\n")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="测试执行S型曲线行走演示程序")
    parser.add_argument("--ip", default="127.0.0.1", help="WebSocket服务器IP地址 (默认: 127.0.0.1)")
    args = parser.parse_args()
    
    test_execute_trace_path_demo(args.ip)