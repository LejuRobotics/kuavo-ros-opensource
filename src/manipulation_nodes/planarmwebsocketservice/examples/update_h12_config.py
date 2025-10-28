#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import websocket
import json
import time
import sys
import argparse

def test_update_h12_config(ip="127.0.0.1"):
    """
    测试更新h12遥控器配置文件接口
    """
    print("=== 测试更新H12配置 ===")
    
    # 创建WebSocket连接
    ws = websocket.create_connection("ws://{}:8888".format(ip))
    
    # 构造请求消息
    request = {
        "cmd": "update_h12_config"
    }
    
    print("发送请求:", json.dumps(request, ensure_ascii=False))
    ws.send(json.dumps(request))
    
    # 接收响应
    response = ws.recv()
    print("收到响应:", response)
    
    ws.close()
    print("=== 测试完成 ===\n")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="测试更新H12配置接口")
    parser.add_argument("--ip", default="127.0.0.1", help="WebSocket服务器IP地址 (默认: 127.0.0.1)")
    args = parser.parse_args()
    
    test_update_h12_config(args.ip)