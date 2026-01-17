
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import asyncio
import websockets
import json
import os

# ... existing code ...
class WebSocketClient:
    def __init__(self, uri, timeout=10):
        self.uri = uri
        self.websocket = None
        self.timeout = timeout  # 超时时间

    async def connect(self):
        while True:
            try:
                self.websocket = await websockets.connect(self.uri)
                print("Connected to WebSocket server")
                return
            except Exception as e:
                print(f"Connection failed: {e}. Retrying in 5 seconds...")
                await asyncio.sleep(5)

    async def send_message(self, message):
        if not self.websocket:
            await self.connect()
        try:
            # 添加超时控制
            await asyncio.wait_for(self.websocket.send(json.dumps(message)), timeout=self.timeout)
        except (websockets.exceptions.ConnectionClosed, asyncio.TimeoutError) as e:
            print(f"Send failed: {e}. Reconnecting...")
            await self.reconnect()
            await self.send_message(message)

    async def receive_messages(self):
        if not self.websocket:
            await self.connect()
        try:
            while True:
                # 添加超时控制
                response = await asyncio.wait_for(self.websocket.recv(), timeout=self.timeout)
                print(f"Received response: {response}")
        except (websockets.exceptions.ConnectionClosed, asyncio.TimeoutError) as e:
            print(f"Receive failed: {e}. Reconnecting...")
            await self.reconnect()
            await self.receive_messages()

    async def reconnect(self):
        self.websocket = None
        await self.connect()

    async def close(self):
        if self.websocket:
            await self.websocket.close()
            self.websocket = None


def show_menu():
    """显示选项菜单"""
    print("\n请选择要发送的消息:")
    print("1. zero_point_debug - start")
    print("2. zero_point_debug - exit")
    print("3. robot_switch_pose - stand")
    print("4. robot_switch_pose - ready")
    print("5. robot_switch_pose - cali_zero")
    print("0. 退出")


def get_message_by_option(option):
    """根据选项返回对应的消息"""
    messages = {
        "1": {
            "cmd": "zero_point_debug",
            "data": {"zero_point_debug_status": "start"}
        },
        "2": {
            "cmd": "zero_point_debug",
            "data": {"zero_point_debug_status": "exit"}
        },
        "3": {
            "cmd": "robot_switch_pose",
            "data": {"robot_pose": "stand"}
        },
        "4": {
            "cmd": "robot_switch_pose",
            "data": {"robot_pose": "ready"}
        },
        "5": {
            "cmd": "robot_switch_pose",
            "data": {"robot_pose": "cali_zero"}
        }
    }
    
    return messages.get(option, None)


async def send_test_message():
    uri = "ws://10.10.20.141:8888"  # Adjust the URI as needed

    client = WebSocketClient(uri)

    # 启动一个后台任务持续接收消息
    receive_task = None
    
    try:
        await client.connect()
        
        # 创建一个任务来持续接收消息
        receive_task = asyncio.create_task(client.receive_messages())

        while True:
            show_menu()
            choice = input("请输入选项编号: ").strip()
            
            if choice == "0":
                print("退出程序")
                break
                
            message = get_message_by_option(choice)
            if message:
                await client.send_message(message)
                print(f"已发送消息: {message}")
            else:
                print("无效选项，请重新选择")

    except KeyboardInterrupt:
        print("\n程序被用户中断")
    except Exception as e:
        print(f"发生错误: {e}")
    finally:
        # 取消接收任务
        if receive_task and not receive_task.done():
            receive_task.cancel()
        await client.close()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description='WebSocket client for sending action files')
    args = parser.parse_args()
    try:
        asyncio.run(send_test_message())
    except KeyboardInterrupt:
        print("Program interrupted by user, shutting down.")

