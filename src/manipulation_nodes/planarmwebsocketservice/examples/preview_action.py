#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import websocket
import json
import time
import sys
import argparse

def test_preview_action(ip="127.0.0.1"):
    """
    测试预览动作接口
    """
    print("=== 测试预览动作 ===")
    
    # 创建WebSocket连接
    ws = websocket.create_connection("ws://{}:8888".format(ip))
    
    # 构造请求消息
    # 注意：action_filename 和 action_file_MD5 需要根据实际情况填写
    request = {
        "cmd": "preview_action",
        "data": {
            "action_filename": "welcome.tact",
            "action_file_MD5": "18a7ff928bf4965940678235010dacdf"
        }
    }
    
    print("发送请求:", json.dumps(request, ensure_ascii=False))
    ws.send(json.dumps(request))
    
    # 接收响应（可能收到多个进度更新）
    try:
        while True:
            response = ws.recv()
            print("收到响应:", response)
            
            # 解析响应检查是否完成
            data = json.loads(response)
            if data.get("cmd") == "preview_action":
                status = data.get("data", {}).get("status", 1)
                if status == 0:  # 完成
                    break
    except websocket.WebSocketConnectionClosedException:
        print("连接已关闭")
    
    ws.close()
    print("=== 测试完成 ===\n")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="测试预览动作接口")
    parser.add_argument("--ip", default="127.0.0.1", help="WebSocket服务器IP地址 (默认: 127.0.0.1)")
    args = parser.parse_args()
    
    test_preview_action(args.ip)