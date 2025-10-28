#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
通过WebSocket测试handler.py中新增的四个机器人控制接口:
1. start_robot - 启动机器人
2. stop_robot - 停止机器人
3. stand_robot - 站立机器人
4. get_robot_launch_status - 获取机器人启动状态

# 测试启动机器人
python3 test_robot_websocket_client.py --command start_robot

# 测试停止机器人
python3 test_robot_websocket_client.py --command stop_robot

# 测试站立机器人
python3 test_robot_websocket_client.py --command stand_robot

# 测试获取机器人状态
python3 test_robot_websocket_client.py --command get_robot_launch_status

"""

import asyncio
import websockets
import json
import argparse

class WebSocketRobotTestClient:
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
            await self.websocket.send(json.dumps(message))
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
            print(f"Received response: {response}")
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

    async def test_start_robot(self):
        """测试启动机器人接口"""
        message = {
            "cmd": "start_robot",
            "data": {}
        }
        await self.send_message(message)
        return await self.receive_messages()

    async def test_stop_robot(self):
        """测试停止机器人接口"""
        message = {
            "cmd": "stop_robot",
            "data": {}
        }
        await self.send_message(message)
        return await self.receive_messages()

    async def test_stand_robot(self):
        """测试站立机器人接口"""
        message = {
            "cmd": "stand_robot",
            "data": {}
        }
        await self.send_message(message)
        return await self.receive_messages()

    async def test_get_robot_launch_status(self):
        """测试获取机器人启动状态接口"""
        message = {
            "cmd": "get_robot_launch_status",
            "data": {}
        }
        await self.send_message(message)
        return await self.receive_messages()

async def run_robot_control_tests(uri):
    """运行机器人控制接口测试"""
    client = WebSocketRobotTestClient(uri)

    try:
        await client.connect()
        
        print("=" * 60)
        print("开始测试机器人控制接口")
        print("=" * 60)

        # 1. 测试获取机器人启动状态
        print("\n1. 测试获取机器人启动状态...")
        response = await client.test_get_robot_launch_status()
        if response:
            print(f"   当前机器人状态: {response.get('data', {}).get('message', 'Unknown')}")
            print(f"   服务调用结果: {'成功' if response.get('data', {}).get('code', 1) == 0 else '失败'}")

        # 2. 测试启动机器人
        print("\n2. 测试启动机器人...")
        response = await client.test_start_robot()
        if response:
            print(f"   启动结果: {response.get('data', {}).get('message', 'Unknown')}")
            print(f"   服务调用结果: {'成功' if response.get('data', {}).get('code', 1) == 0 else '失败'}")

        # 等待一段时间让机器人启动
        print("   等待机器人启动完成...")
        await asyncio.sleep(5)

        # 再次检查状态
        print("   检查启动后机器人状态...")
        response = await client.test_get_robot_launch_status()
        if response:
            print(f"   启动后机器人状态: {response.get('data', {}).get('message', 'Unknown')}")

        # 3. 测试站立机器人
        print("\n3. 测试站立机器人...")
        response = await client.test_stand_robot()
        if response:
            print(f"   站立结果: {response.get('data', {}).get('message', 'Unknown')}")
            print(f"   服务调用结果: {'成功' if response.get('data', {}).get('code', 1) == 0 else '失败'}")

        # 等待一段时间让机器人站立
        print("   等待机器人站立完成...")
        await asyncio.sleep(5)

        # 再次检查状态
        print("   检查站立后机器人状态...")
        response = await client.test_get_robot_launch_status()
        if response:
            print(f"   站立后机器人状态: {response.get('data', {}).get('message', 'Unknown')}")

        # 4. 测试停止机器人
        print("\n4. 测试停止机器人...")
        response = await client.test_stop_robot()
        if response:
            print(f"   停止结果: {response.get('data', {}).get('message', 'Unknown')}")
            print(f"   服务调用结果: {'成功' if response.get('data', {}).get('code', 1) == 0 else '失败'}")

        # 等待一段时间让机器人停止
        print("   等待机器人停止完成...")
        await asyncio.sleep(3)

        # 最后检查状态
        print("   检查停止后机器人状态...")
        response = await client.test_get_robot_launch_status()
        if response:
            print(f"   停止后机器人状态: {response.get('data', {}).get('message', 'Unknown')}")

        print("\n" + "=" * 60)
        print("机器人控制接口测试完成")
        print("=" * 60)

    except asyncio.CancelledError:
        print("Client was cancelled, closing connection.")
    except Exception as e:
        print(f"测试过程中发生错误: {e}")
    finally:
        await client.close()

async def send_single_command(uri, command):
    """发送单个命令进行测试"""
    client = WebSocketRobotTestClient(uri)
    
    try:
        await client.connect()
        
        print(f"发送命令: {command}")
        
        if command == "start_robot":
            response = await client.test_start_robot()
        elif command == "stop_robot":
            response = await client.test_stop_robot()
        elif command == "stand_robot":
            response = await client.test_stand_robot()
        elif command == "get_robot_launch_status":
            response = await client.test_get_robot_launch_status()
        else:
            print(f"未知命令: {command}")
            return
            
        if response:
            print(f"响应: {response}")
            
    except Exception as e:
        print(f"发送命令时发生错误: {e}")
    finally:
        await client.close()

def main():
    parser = argparse.ArgumentParser(description='WebSocket client for testing robot control interfaces')
    parser.add_argument('--uri', type=str, default="ws://0.0.0.0:8888",
                        help='WebSocket server URI (default: ws://0.0.0.0:8888)')
    parser.add_argument('--command', type=str, choices=['start_robot', 'stop_robot', 'stand_robot', 'get_robot_launch_status', 'all'],
                        help='Specific command to test (default: all)')
    
    args = parser.parse_args()
    
    try:
        if args.command and args.command != 'all':
            asyncio.run(send_single_command(args.uri, args.command))
        else:
            asyncio.run(run_robot_control_tests(args.uri))
    except KeyboardInterrupt:
        print("Program interrupted by user, shutting down.")
    except Exception as e:
        print(f"程序执行出错: {e}")

if __name__ == "__main__":
    main()