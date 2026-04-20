#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
机器人状态测试客户端
用于测试robot_state_server的功能

客户端注册消息格式 (JSON):
{
    "action": "Hello",
    "client_id": "unique_client_identifier"
}

服务端响应消息格式 (JSON):
{
    "action": "OK",
    "client_id": "unique_client_identifier"
}
"""

import socket
import time
import sys
import os
import json
import uuid
import copy

# 添加protobuf路径
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../')))
import protos.robot_state_pb2 as robot_state_pb2

# 标准请求模板
HELLO_REQUEST_TEMPLATE = {
    'action': 'Hello',
    'client_id': ''
}

class RobotStateTestClient:
    """机器人状态测试客户端"""

    def __init__(self, server_host='localhost', server_port=15170, client_id=None):
        """
        初始化测试客户端

        Args:
            server_host (str): 服务器地址
            server_port (int): 服务器端口
            client_id (str): 客户端ID，如果不指定则自动生成
        """
        self.server_host = server_host
        self.server_port = server_port
        self.client_id = client_id or f"client_{uuid.uuid4().hex[:8]}"
        self.socket = None
        self.connected = False
        self.message_count = 0

    def connect(self):
        """连接到服务器"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.settimeout(5.0)  # 5秒超时

            # 发送JSON格式的HELLO消息注册为客户端
            hello_msg = copy.deepcopy(HELLO_REQUEST_TEMPLATE)
            hello_msg['client_id'] = self.client_id
            msg_bytes = json.dumps(hello_msg).encode('utf-8')
            self.socket.sendto(msg_bytes, (self.server_host, self.server_port))
            print(f"Sent HELLO message for client '{self.client_id}' to {self.server_host}:{self.server_port}")

            # 等待服务器响应
            data, _ = self.socket.recvfrom(1024)
            response = json.loads(data.decode('utf-8').strip())

            if response.get('action') == 'OK' and response.get('client_id') == self.client_id:
                self.connected = True
                print(f"Successfully connected to server! Client '{self.client_id}' confirmed")
                return True
            else:
                print(f"Unexpected response: {response}")
                return False

        except Exception as e:
            print(f"Failed to connect: {e}")
            return False

    def disconnect(self):
        """断开连接"""
        if self.socket:
            self.socket.close()
        self.connected = False
        print("Disconnected from server")

    
    def receive_messages(self, duration=10):
        """
        接收消息

        Args:
            duration (int): 接收时长（秒），0表示永久运行
        """
        if not self.connected:
            print("Not connected to server")
            return

        if duration == 0:
            print("Receiving messages permanently...")
        else:
            print(f"Receiving messages for {duration} seconds...")

        start_time = time.time()

        try:
            while True:
                # 检查是否超时（如果不是永久运行）
                if duration > 0 and time.time() - start_time >= duration:
                    break

                try:
                    # 设置较短的超时以便定期检查
                    self.socket.settimeout(1.0)
                    data, addr = self.socket.recvfrom(4096)

                    self.message_count += 1
                    print(f"\nMessage #{self.message_count} from {addr}:")
                    print(f"Data size: {len(data)} bytes")

                    # 尝试解析protobuf
                    self._parse_protobuf_data(data)

                except socket.timeout:
                    continue  # 超时继续等待
                except Exception as e:
                    print(f"Error receiving message: {e}")

        except KeyboardInterrupt:
            print("\nInterrupted by user")

        print(f"\nReceived {self.message_count} messages in total")

    def _parse_protobuf_data(self, data):
        """
        解析protobuf数据

        Args:
            data (bytes): 接收到的数据
        """
        try:
            # 尝试解析为机器人状态消息
            robot_state = robot_state_pb2.RobotState()
            robot_state.ParseFromString(data)

            # 解析消息头
            header = robot_state.header
            print(f"  Header:")
            print(f"    ID: {header.id}")
            print(f"    Timestamp: {header.timestamp}")

            # 解析关节状态
            joint_state = robot_state.joint_state
            print(f"  Joint State:")
            print(f"    Position count: {len(joint_state.position)}")
            if joint_state.position:
                print(f"    Positions: {joint_state.position}")
            print(f"    Velocity count: {len(joint_state.velocity)}")
            if joint_state.velocity:
                print(f"    Velocities: {joint_state.velocity}")
            print(f"    Torque count: {len(joint_state.torque)}")
            if joint_state.torque:
                print(f"    Torques: {joint_state.torque}")

            # 解析末端执行器状态
            ee_state = robot_state.ee_state
            print(f"  End Effector State:")
            print(f"    Position count: {len(ee_state.position)}")
            if ee_state.position:
                print(f"    Positions: {ee_state.position}")
            print(f"    Velocity count: {len(ee_state.velocity)}")
            if ee_state.velocity:
                print(f"    Velocities: {ee_state.velocity}")
            print(f"    Effort count: {len(ee_state.effort)}")
            if ee_state.effort:
                print(f"    Efforts: {ee_state.effort}")

            # 解析手臂力反馈数据（新增）
            if robot_state.HasField('arm_force_feedback'):
                arm_force = robot_state.arm_force_feedback
                print(f"  ╔═══════════════════════════════════════════════════════╗")
                print(f"  ║          VR 端力反馈模拟 (Virtual Force Feedback)        ║")
                print(f"  ╚═══════════════════════════════════════════════════════╝")

                # 左臂 VR 模拟
                left_vibration_intensity = int(arm_force.max_force[0])
                left_weight_kg = (arm_force.max_force[0] / 255.0) * 2.0
                print(f"  ║ 左臂 (Left Arm):")
                print(f"  ║   振动强度: {left_vibration_intensity}/255")
                print(f"  ║   估算重量: {left_weight_kg:.2f} kg (基于xyz中最大力)")
                print(f"  ║   振动条形图: {'█' * (left_vibration_intensity // 10)}{'░' * (25 - left_vibration_intensity // 10)}")

                # 右臂 VR 模拟
                right_vibration_intensity = int(arm_force.max_force[1])
                right_weight_kg = (arm_force.max_force[1] / 255.0) * 2.0
                print(f"  ║")
                print(f"  ║ 右臂 (Right Arm):")
                print(f"  ║   振动强度: {right_vibration_intensity}/255")
                print(f"  ║   估算重量: {right_weight_kg:.2f} kg (基于xyz中最大力)")
                print(f"  ║   振动条形图: {'█' * (right_vibration_intensity // 10)}{'░' * (25 - right_vibration_intensity // 10)}")

                # VR 触发建议
                max_vibration = max(left_vibration_intensity, right_vibration_intensity)
                if max_vibration > 200:
                    print(f"  ║")
                    print(f"  ║ 🎮 VR 动作建议: 强烈振动！抓取重物 (>1.5kg)")
                elif max_vibration > 100:
                    print(f"  ║")
                    print(f"  ║ 🎮 VR 动作建议: 中等振动，抓取中物 (0.8-1.5kg)")
                elif max_vibration > 20:
                    print(f"  ║")
                    print(f"  ║ 🎮 VR 动作建议: 轻微振动，抓取轻物 (0.2-0.8kg)")
                else:
                    print(f"  ║")
                    print(f"  ║ 🎮 VR 动作建议: 无振动，空手或极轻物体")

                print(f"  ╚═══════════════════════════════════════════════════════╝")
            else:
                print(f"  ║ Arm Force Feedback: 无数据")
                print(f"  ╚═══════════════════════════════════════════════════════╝")

        except Exception as e:
            # 如果不是protobuf格式，可能是其他消息
            try:
                text = data.decode('utf-8')
                print(f"  Text message: {text}")
            except UnicodeDecodeError:
                print(f"  Binary data (cannot decode as text): {data[:50]}...")


def main():
    """主函数"""
    # 创建客户端
    client = RobotStateTestClient('localhost', 15170, 'test_client')

    try:
        # 连接服务器
        if not client.connect():
            print("Failed to connect to server")
            return 1

        # 接收消息
        print("\nStarting to receive messages permanently (Ctrl+C to stop)...")
        client.receive_messages(duration=0)  # 0表示永久运行

    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    finally:
        client.disconnect()

    return 0


if __name__ == "__main__":
    sys.exit(main())