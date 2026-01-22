#!/usr/bin/env python3
"""
WebSocket客户端测试脚本
用于测试实时对话功能接口

使用方法:
    python3 test_real_time_chat_websocket_client.py --host localhost --port 8888
"""

import asyncio
import websockets
import json
import argparse
import sys
from typing import Dict, Any
import traceback


class RealTimeChatWebSocketClient:
    """实时对话功能WebSocket客户端"""

    def __init__(self, host: str = "localhost", port: int = 8888):
        self.host = host
        self.port = port
        self.uri = f"ws://{host}:{port}"
        self.websocket = None

    async def connect(self):
        """连接到WebSocket服务器"""
        try:
            print(f"正在连接到 {self.uri}...")
            self.websocket = await websockets.connect(self.uri)
            print(f"✓ 成功连接到 {self.uri}")
            return True
        except Exception as e:
            print(f"✗ 连接失败: {e}")
            return False

    async def disconnect(self):
        """断开WebSocket连接"""
        if self.websocket:
            await self.websocket.close()
            print("✓ 已断开连接")

    async def send_message(self, cmd: str, data: Dict[str, Any] = None):
        """发送消息到服务器"""
        if not self.websocket:
            print("✗ 未连接到服务器")
            return None

        message = {"cmd": cmd, "data": data or {}}

        try:
            print(f"\n>>> 发送消息:")
            print(f"    命令: {cmd}")
            if data:
                print(f"    数据: {json.dumps(data, ensure_ascii=False, indent=4)}")

            await self.websocket.send(json.dumps(message, ensure_ascii=False))

            # 接收响应
            response = await self.websocket.recv()
            print(response)
            response_data = json.loads(response)

            print(f"\n<<< 收到响应:")
            print(json.dumps(response_data, ensure_ascii=False, indent=4))

            return response_data
        except Exception as e:
            print(f"✗ 发送/接收消息失败: {e}")
            print(traceback.format_exc())
            return None

    async def test_start_real_time_chat(self):
        """测试开启实时对话接口"""
        print("\n" + "=" * 60)
        print("测试 start_real_time_chat 接口")
        print("=" * 60)

        response = await self.send_message("start_real_time_chat", {})

        if response:
            if response.get("data", {}).get("code") == 0:
                print(f"\n✓ 实时对话开启成功: {response.get('data', {}).get('message')}")
                return True
            else:
                print(f"\n✗ 实时对话开启失败: {response.get('data', {}).get('message')}")
                return False
        return False

    async def test_stop_real_time_chat(self):
        """测试停止实时对话接口"""
        print("\n" + "=" * 60)
        print("测试 stop_real_time_chat 接口")
        print("=" * 60)

        response = await self.send_message("stop_real_time_chat", {})

        if response:
            if response.get("data", {}).get("code") == 0:
                print(f"\n✓ 实时对话停止成功: {response.get('data', {}).get('message')}")
                return True
            else:
                print(f"\n✗ 实时对话停止失败: {response.get('data', {}).get('message')}")
                return False
        return False

    async def test_get_real_time_chat_status(self):
        """测试获取实时对话状态接口"""
        print("\n" + "=" * 60)
        print("测试 get_real_time_chat_status 接口")
        print("=" * 60)

        response = await self.send_message("get_real_time_chat_status", {})

        if response:
            data = response.get("data", {})
            if data.get("code") == 0:
                print(f"\n✓ 实时对话状态获取成功: {data.get('message')}")
                print(f"    状态: {'运行中' if data.get('status') else '未运行'}")
                return data.get("status")
            else:
                print(f"\n✗ 实时对话状态获取失败: {data.get('message')}")
                return False
        return False

    async def run_real_time_chat_test(self):
        """运行实时对话功能测试"""
        print("\n" + "=" * 60)
        print("开始实时对话功能全面测试")
        print("=" * 60)

        if not await self.connect():
            return False

        try:
            # 1. 测试开启实时对话
            print("\n[步骤 1] 测试开启实时对话")
            start_success = await self.test_start_real_time_chat()

            if not start_success:
                print("\n✗ 开启实时对话测试失败，终止测试")
                return False

            # 2. 等待120秒，并在30、60、90秒时打印状态
            print("\n[步骤 2] 等待120秒，并在30、60、90秒时打印状态")
            total_wait_time = 120
            check_times = [30, 60, 90]
            start_time = asyncio.get_event_loop().time()
            
            while True:
                elapsed_time = asyncio.get_event_loop().time() - start_time
                remaining_time = total_wait_time - elapsed_time
                
                if remaining_time <= 0:
                    break
                
                # 检查是否需要打印状态
                current_seconds = int(elapsed_time)
                if current_seconds in check_times:
                    print(f"\n[{current_seconds}秒] 检查实时对话状态:")
                    status = await self.test_get_real_time_chat_status()
                    print(f"    当前状态: {'运行中' if status else '未运行'}")
                
                # 等待1秒
                await asyncio.sleep(1)

            print("\n[120秒] 等待时间结束")

            # 3. 测试关闭实时对话
            print("\n[步骤 3] 测试关闭实时对话")
            stop_success = await self.test_stop_real_time_chat()

            if not stop_success:
                print("\n✗ 关闭实时对话测试失败")
                return False

            # 4. 发送关闭chat后每隔1s查询实时对话状态，并进行状态打印
            print("\n[步骤 4] 发送关闭chat后每隔1s查询实时对话状态")
            max_checks = 10
            check_count = 0
            
            while check_count < max_checks:
                print(f"\n[第{check_count + 1}次检查]")
                status = await self.test_get_real_time_chat_status()
                
                if not status:
                    print("✓ 实时对话已成功停止")
                    break
                
                print("    实时对话仍在运行，继续等待...")
                await asyncio.sleep(1)
                check_count += 1
            
            if check_count == max_checks:
                print("\n✗ 实时对话未能在规定时间内停止")
                return False

            print("\n" + "=" * 60)
            print("✓ 所有测试通过！")
            print("=" * 60)
            return True

        except Exception as e:
            print(f"\n✗ 测试过程中发生错误: {e}")
            import traceback

            traceback.print_exc()
            return False
        finally:
            await self.disconnect()


async def main():
    """主函数"""
    parser = argparse.ArgumentParser(
        description="WebSocket客户端 - 测试实时对话功能接口"
    )
    parser.add_argument(
        "--host",
        type=str,
        default="localhost",
        help="WebSocket服务器地址 (默认: localhost)",
    )
    parser.add_argument(
        "--port", type=int, default=8888, help="WebSocket服务器端口 (默认: 8888)"
    )

    args = parser.parse_args()

    # 创建客户端
    client = RealTimeChatWebSocketClient(host=args.host, port=args.port)

    try:
        success = await client.run_real_time_chat_test()
        sys.exit(0 if success else 1)

    except KeyboardInterrupt:
        print("\n\n✗ 用户中断")
        sys.exit(1)
    except Exception as e:
        print(f"\n✗ 发生错误: {e}")
        import traceback

        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    asyncio.run(main())
