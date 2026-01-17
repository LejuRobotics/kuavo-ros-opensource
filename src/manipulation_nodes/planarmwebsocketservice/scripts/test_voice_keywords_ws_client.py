#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
通过WebSocket测试handler.py中新增的两个语音关键词识别接口:
1. update_voice_keywords - 更新语音关键词配置
2. get_voice_keywords - 获取语音关键词配置
"""

import asyncio
import websockets
import json
import argparse

class WebSocketVoiceKeywordsTestClient:
    def __init__(self, uri):
        self.uri = uri
        self.websocket = None

    async def connect(self):
        """连接到WebSocket服务器"""
        while True:
            try:
                self.websocket = await websockets.connect(self.uri)
                print("Connected to WebSocket server")
                return
            except Exception as e:
                print(f"Connection failed: {e}. Retrying in 5 seconds...")
                await asyncio.sleep(5)

    async def send_message(self, message):
        """发送消息到WebSocket服务器"""
        if not self.websocket:
            await self.connect()
        try:
            await self.websocket.send(json.dumps(message, ensure_ascii=False))
            print(f"Sent message: {message}")
        except websockets.exceptions.ConnectionClosed:
            print("Connection closed. Reconnecting...")
            await self.connect()
            await self.send_message(message)

    async def receive_messages(self, timeout=10):
        """接收服务器响应消息"""
        if not self.websocket:
            await self.connect()
        try:
            # 等待响应消息，设置超时时间
            response = await asyncio.wait_for(self.websocket.recv(), timeout=timeout)
            print(f"Received response: {json.dumps(response, ensure_ascii=False, indent=2)}")
            return json.loads(response)
        except asyncio.TimeoutError:
            print("Timeout waiting for response")
            return None
        except websockets.exceptions.ConnectionClosed:
            print("Connection closed. Reconnecting...")
            await self.connect()
            return None

    async def close(self):
        """关闭WebSocket连接"""
        if self.websocket:
            await self.websocket.close()
            self.websocket = None

    async def test_update_keywords(self, keywords_json):
        """测试更新语音关键词接口"""
        payload = {}
        for key, value in keywords_json.items():
            if value.get("type") == "ARM_ACTION":
                payload[key] = value["keywords"]
        # 模拟覆盖更新打招呼动作
        payload["右手打招呼"] = ["打招呼", "挥挥手", "打个招呼", "招呼一下"]
        payload["右手握手"] = ["握手", "和我握个手", "来握手", "握个手"]
        message = {
            "cmd": "update_voice_keywords",
            "data": payload
            # "data": json.dumps(keywords_json)
        }
        await self.send_message(message)
        return await self.receive_messages()

    async def test_get_keywords(self):
        """测试读取语音关键词接口"""
        message = {
            "cmd": "get_voice_keywords",
            "data": {}
        }
        await self.send_message(message)
        return await self.receive_messages()

async def run_setting_voice_keywords_tests(uri):
    """运行语音关键词识别接口测试"""
    client = WebSocketVoiceKeywordsTestClient(uri)

    try:
        await client.connect()
        
        print("=" * 60)
        print("开始测试配置语音关键词接口")
        print("=" * 60)

        response = await client.test_get_keywords()
        read_json = response["data"]["result"]
        print("=" * 60)
        await client.test_update_keywords(read_json)
        print("=" * 60)
        await client.test_get_keywords()

    except asyncio.CancelledError:
        print("Client was cancelled, closing connection.")
    except Exception as e:
        print(f"测试过程中发生错误: {e}")
    finally:
        await client.close()

def main():
    parser = argparse.ArgumentParser(description='WebSocket client for testing setting voice control keywords config interfaces')
    parser.add_argument('--uri', type=str, default="ws://0.0.0.0:8888",
                        help='WebSocket server URI (default: ws://0.0.0.0:8888)')
    
    args = parser.parse_args()
    
    try:
        asyncio.run(run_setting_voice_keywords_tests(args.uri))
    except KeyboardInterrupt:
        print("Program interrupted by user, shutting down.")
    except Exception as e:
        print(f"程序执行出错: {e}")

if __name__ == "__main__":
    main()