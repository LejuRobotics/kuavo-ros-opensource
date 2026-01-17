#!/usr/bin/env python3
"""
WebSocket客户端测试脚本
用于测试 store_api_keys 和 get_api_keys 接口

使用方法:
    python3 test_api_keys_websocket_client.py --host localhost --port 8888
"""

import asyncio
import websockets
import json
import argparse
import sys
from typing import Dict, Any
import traceback


class APIKeysWebSocketClient:
    """API密钥管理WebSocket客户端"""

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

    async def test_set_api_key(self, api_keys: Dict[str, str]):
        """测试设置API密钥接口"""
        print("\n" + "=" * 60)
        print("测试 set_api_key 接口")
        print("=" * 60)

        response = await self.send_message("set_api_key", api_keys)

        if response:
            if response.get("data", {}).get("code") == 0:
                print(f"\n✓ API密钥设置成功: {response.get('data', {}).get('msg')}")
                return True
            else:
                print(f"\n✗ API密钥设置失败: {response.get('data', {}).get('msg')}")
                return False
        return False

    async def test_get_api_key(self):
        """测试获取API密钥接口"""
        print("\n" + "=" * 60)
        print("测试 get_api_key 接口")
        print("=" * 60)

        response = await self.send_message("get_api_key", {})

        if response:
            data = response.get("data", {})
            if data.get("code") == 0:
                print(f"\n✓ API密钥获取成功: {data.get('msg')}")
                print("\n获取到的API密钥:")
                # 过滤掉code和msg字段，只显示API密钥
                api_keys = {k: v for k, v in data.items() if k not in ["code", "msg"]}
                if api_keys:
                    for key, value in api_keys.items():
                        # 隐藏部分密钥内容以保护安全
                        masked_value = (
                            value[:8] + "..." + value[-4:] if len(value) > 12 else value
                        )
                        print(f"    {key}: {masked_value}")
                else:
                    print("    (无API密钥)")
                return True
            else:
                print(f"\n✗ API密钥获取失败: {data.get('msg')}")
                return False
        return False

    async def test_invalid_inputs(self):
        """测试无效输入"""
        print("\n" + "=" * 60)
        print("测试无效输入场景")
        print("=" * 60)

        # 测试1: 设置包含特殊字符的API密钥
        print("\n[测试1] 测试包含特殊字符的API密钥")
        special_keys = {
            "ark_X-Api-App-ID": "abc:123",  # 包含冒号
            "ark_X-Api-Access-Key": "sk-123:456:789",  # 包含多个冒号
            "test_key": "value with spaces",  # 包含空格
        }
        set_response = await self.send_message("set_api_key", special_keys)
        if set_response and set_response.get("data", {}).get("code") == 0:
            print("✓ 成功处理包含特殊字符的API密钥")
        else:
            print("✗ 处理包含特殊字符的API密钥失败")

        # 测试2: 测试获取不存在的API密钥
        print("\n[测试2] 测试获取不存在的API密钥")
        test_key = "non_existent_key"
        await self.send_message("set_api_key", {test_key: "test_value"})
        get_response = await self.send_message("get_api_key", {})
        if get_response and get_response.get("data", {}).get(test_key) == "test_value":
            print(f"✓ 成功获取新增的API密钥: {test_key}")
        else:
            print(f"✗ 无法获取新增的API密钥: {test_key}")

        # 测试3: 测试删除API密钥（设置为None）
        print("\n[测试3] 测试删除API密钥（设置为None）")
        await self.send_message("set_api_key", {test_key: None})
        get_response2 = await self.send_message("get_api_key", {})
        if get_response2 and get_response2.get("data", {}).get(test_key) == "":
            print(f"✓ 成功删除API密钥: {test_key}")
        else:
            print(f"✗ 无法删除API密钥: {test_key}")

    async def test_empty_values(self):
        """测试空值场景"""
        print("\n" + "=" * 60)
        print("测试空值场景")
        print("=" * 60)

        # 测试1: 设置空字符串
        print("\n[测试1] 测试设置空字符串")
        test_key = "empty_value_key"
        await self.send_message("set_api_key", {test_key: ""})
        get_response = await self.send_message("get_api_key", {})
        if get_response and get_response.get("data", {}).get(test_key) == "":
            print(f"✓ 成功处理空字符串API密钥: {test_key}")
        else:
            print(f"✗ 无法处理空字符串API密钥: {test_key}")

        # 测试2: 发送空的data对象
        print("\n[测试2] 测试发送空的data对象")
        response = await self.send_message("set_api_key", {})
        if response and response.get("data", {}).get("code") == 1:
            print("✓ 成功拒绝空的API密钥设置")
        else:
            print("✗ 没有正确处理空的API密钥设置")

    async def test_get_api_key_status(self, api_type: str):
        """测试获取API密钥状态接口"""
        print("\n" + "=" * 60)
        print(f"测试 get_api_key_status 接口 (type: {api_type})")
        print("=" * 60)

        response = await self.send_message("get_api_key_status", {"type": api_type})

        if response:
            data = response.get("data", {})
            if data.get("code") == 0:
                print(f"\n✓ API密钥状态获取成功: {data.get('msg')}")
                print(f"    类型: {data.get('type')}")
                print(f"    未设置的密钥: {data.get('is_empty', [])}")
                return True
            else:
                print(f"\n✗ API密钥状态获取失败: {data.get('msg')}")
                print(f"    类型: {data.get('type')}")
                print(f"    未设置的密钥: {data.get('is_empty', [])}")
                return False
        return False

    async def test_get_api_key_status_valid_types(self):
        """测试有效类型的API密钥状态检查"""
        print("\n" + "=" * 60)
        print("测试有效类型的API密钥状态检查")
        print("=" * 60)

        # 测试实时模型
        print("\n[测试1] 测试实时模型API密钥状态")
        realtime_success = await self.test_get_api_key_status("realtime")

        # 测试非实时模型
        print("\n[测试2] 测试非实时模型API密钥状态")
        non_realtime_success = await self.test_get_api_key_status("non-realtime")

        return realtime_success and non_realtime_success

    async def test_get_api_key_status_invalid_types(self):
        """测试无效类型的API密钥状态检查"""
        print("\n" + "=" * 60)
        print("测试无效类型的API密钥状态检查")
        print("=" * 60)

        invalid_types = ["invalid", "test", "123", "", None]
        all_failed = True

        for api_type in invalid_types:
            print(f"\n[测试] 测试无效类型: {api_type}")
            response = await self.send_message("get_api_key_status", {"type": api_type})

            if response:
                data = response.get("data", {})
                if data.get("code") == 1:
                    print(f"✓ 成功拒绝无效类型: {api_type}")
                else:
                    print(f"✗ 没有正确拒绝无效类型: {api_type}")
                    print(f"    响应: {json.dumps(response, ensure_ascii=False, indent=4)}")
                    all_failed = False

        return all_failed

    async def test_edge_cases(self):
        """测试边界情况"""
        print("\n" + "=" * 60)
        print("测试边界情况")
        print("=" * 60)

        # 测试1: 非常长的API密钥
        print("\n[测试1] 测试非常长的API密钥")
        long_key = "a" * 1000
        long_value = "b" * 1000
        await self.send_message("set_api_key", {long_key: long_value})
        get_response = await self.send_message("get_api_key", {})
        if get_response and get_response.get("data", {}).get(long_key) == long_value:
            print("✓ 成功处理非常长的API密钥")
        else:
            print("✗ 无法处理非常长的API密钥")

        # 测试2: 特殊字符的键名
        print("\n[测试2] 测试特殊字符的键名")
        special_key = "key with spaces"
        special_value = "value"
        await self.send_message("set_api_key", {special_key: special_value})
        get_response = await self.send_message("get_api_key", {})
        if (
            get_response
            and get_response.get("data", {}).get(special_key) == special_value
        ):
            print(f"✓ 成功处理包含特殊字符的键名: {special_key}")
        else:
            print(f"✗ 无法处理包含特殊字符的键名: {special_key}")

    async def run_comprehensive_test(self):
        """运行全面测试"""
        print("\n" + "=" * 60)
        print("开始API密钥接口全面测试")
        print("=" * 60)

        if not await self.connect():
            return False

        try:
            # 测试数据 - 与接口文档中定义的API密钥类型匹配
            test_api_keys = {
                "ark_X-Api-App-ID": "123abc",
                "ark_X-Api-Access-Key": "sk-1234567890abcdef1234567890abcdef",
                "xfyun_APPID": "xfyun123456",
                "xfyun_APISecret": "sk-xfyun-abcdef1234567890",
                "xfyun_APIKey": "api-key-xfyun-123456",
                "ark_analysis_key": "sk-analysis-9876543210",
            }

            # 1. 测试设置API密钥
            print("\n[步骤 1] 测试设置API密钥")
            set_success = await self.test_set_api_key(test_api_keys)

            if not set_success:
                print("\n✗ 设置测试失败，终止测试")
                return False

            # 等待一下确保写入完成
            await asyncio.sleep(0.5)

            # 2. 测试获取API密钥
            print("\n[步骤 2] 测试获取API密钥")
            get_success = await self.test_get_api_key()

            if not get_success:
                print("\n✗ 获取测试失败")
                return False

            # 3. 测试更新API密钥（部分更新）
            print("\n[步骤 3] 测试更新API密钥（部分更新）")
            update_keys = {
                "ark_X-Api-App-ID": "updated-456def",
                "xfyun_APPID": "",  # 测试设置为空值
            }
            update_success = await self.test_set_api_key(update_keys)

            if not update_success:
                print("\n✗ 更新测试失败")
                return False

            # 等待一下确保写入完成
            await asyncio.sleep(0.5)

            # 4. 再次获取验证更新
            print("\n[步骤 4] 验证更新后的API密钥")
            verify_success = await self.test_get_api_key()

            if not verify_success:
                print("\n✗ 验证测试失败")
                return False

            # 运行无效输入测试
            await self.test_invalid_inputs()

            # 运行空值测试
            await self.test_empty_values()

            # 运行边界情况测试
            await self.test_edge_cases()

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

    async def run_full_test(self):
        """运行完整的测试流程"""
        print("\n" + "=" * 60)
        print("开始API密钥接口完整测试")
        print("=" * 60)

        if not await self.connect():
            return False

        try:
            # 测试数据 - 与接口文档中定义的API密钥类型匹配
            test_api_keys = {
                "ark_X-Api-App-ID": "123abc",
                "ark_X-Api-Access-Key": "sk-1234567890abcdef1234567890abcdef",
                "xfyun_APPID": "xfyun123456",
                "xfyun_APISecret": "sk-xfyun-abcdef1234567890",
                "xfyun_APIKey": "api-key-xfyun-123456",
                "ark_analysis_key": "sk-analysis-9876543210",
            }

            # 1. 测试设置API密钥
            print("\n[步骤 1] 测试设置API密钥")
            set_success = await self.test_set_api_key(test_api_keys)

            if not set_success:
                print("\n✗ 设置测试失败，终止测试")
                return False

            # 等待一下确保写入完成
            await asyncio.sleep(0.5)

            # 2. 测试获取API密钥
            print("\n[步骤 2] 测试获取API密钥")
            get_success = await self.test_get_api_key()

            if not get_success:
                print("\n✗ 获取测试失败")
                return False

            # 3. 测试更新API密钥（部分更新）
            print("\n[步骤 3] 测试更新API密钥（部分更新）")
            update_keys = {
                "ark_X-Api-App-ID": "updated-456def",
                "xfyun_APPID": "",  # 测试设置为空值
            }
            update_success = await self.test_set_api_key(update_keys)

            if not update_success:
                print("\n✗ 更新测试失败")
                return False

            # 等待一下确保写入完成
            await asyncio.sleep(0.5)

            # 4. 再次获取验证更新
            print("\n[步骤 4] 验证更新后的API密钥")
            verify_success = await self.test_get_api_key()

            if not verify_success:
                print("\n✗ 验证测试失败")
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
        description="WebSocket客户端 - 测试API密钥存储和获取接口"
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
    parser.add_argument(
        "--test",
        type=str,
        choices=["set", "get", "full", "comprehensive", "status"],
        default="comprehensive",
        help="测试类型: set(仅设置), get(仅获取), full(完整测试), comprehensive(全面测试), status(仅状态检查) (默认: comprehensive)",
    )
    parser.add_argument(
        "--api-key",
        type=str,
        action="append",
        help="API密钥，格式: key_name=key_value (可多次使用)",
    )

    args = parser.parse_args()

    # 创建客户端
    client = APIKeysWebSocketClient(host=args.host, port=args.port)

    try:
        if args.test == "comprehensive":
            success = await client.run_comprehensive_test()
            sys.exit(0 if success else 1)
        elif args.test == "full":
            success = await client.run_full_test()
            sys.exit(0 if success else 1)
        elif args.test == "status":
            if not await client.connect():
                sys.exit(1)

            try:
                # 测试所有类型的API密钥状态
                success = await client.test_get_api_key_status_valid_types()
                # 测试无效类型的API密钥状态检查
                invalid_success = await client.test_get_api_key_status_invalid_types()
                sys.exit(0 if success and invalid_success else 1)
            finally:
                await client.disconnect()
        else:
            if not await client.connect():
                sys.exit(1)

            try:
                if args.test == "set":
                    if not args.api_key:
                        print("✗ 错误: 使用 --api-key 参数指定要设置的API密钥")
                        print(
                            "   示例: --api-key ark_X-Api-App-ID=123abc --api-key ark_X-Api-Access-Key=sk-xxx"
                        )
                        sys.exit(1)

                    api_keys = {}
                    for key_pair in args.api_key:
                        if "=" not in key_pair:
                            print(f"✗ 错误: API密钥格式不正确: {key_pair}")
                            print("   正确格式: key_name=key_value")
                            sys.exit(1)
                        key_name, key_value = key_pair.split("=", 1)
                        api_keys[key_name] = key_value

                    success = await client.test_set_api_key(api_keys)
                    sys.exit(0 if success else 1)

                elif args.test == "get":
                    success = await client.test_get_api_key()
                    sys.exit(0 if success else 1)

            finally:
                await client.disconnect()

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
