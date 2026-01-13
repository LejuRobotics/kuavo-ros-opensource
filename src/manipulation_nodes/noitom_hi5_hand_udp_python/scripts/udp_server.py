#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
UDP服务器类
用于向客户端发送机器人状态数据

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
import time
import copy
import socket
import json
import threading
import subprocess
import signal
import os
from typing import Tuple
from dataclasses import dataclass


# 标准请求模板
HELLO_REQUEST_TEMPLATE = {
    'action': 'Hello',
    'client_id': ''
}

# 标准响应模板
OK_RESPONSE_TEMPLATE = {
    'action': 'OK',
    'client_id': ''
}

class UDPServer:
    """UDP服务器类"""
    @dataclass
    class ClientInfo:
        """客户端信息"""
        addr: Tuple[str, int]  # (IP地址, 端口)
        last_success: float  # 最后成功发送时间戳

    def __init__(self, host='0.0.0.0', port=15170, client_timeout=60, force_port=True):
        """
        初始化UDP服务器

        Args:
            host (str): 绑定主机地址，默认为'0.0.0.0'表示所有接口
            port (int): 监听端口，默认为15170
            client_timeout (int): 客户端发送失败超时时间（秒），默认60秒
            force_port (bool): 是否强制使用端口，如果端口被占用则终止占用进程，默认为True
        """
        self.host = host
        self.port = port
        self.client_timeout = client_timeout
        self.force_port = force_port
        self.socket = None
        self.running = False
        self.clients = {}  # 存储所有注册的客户端 {client_id: UDPServer.ClientInfo}
        self.lock = threading.Lock()

    def _check_port_occupied(self):
        """
        检查端口是否被占用

        Returns:
            list: 占用该端口的进程PID列表，如果未被占用则返回空列表
        """
        try:
            # 使用 lsof 命令检查端口占用情况
            result = subprocess.run(['lsof', '-t', f'-i:{self.port}'],
                                  capture_output=True, text=True, check=False)

            if result.returncode == 0 and result.stdout.strip():
                # 解析PID列表
                pids = [pid.strip() for pid in result.stdout.strip().split('\n') if pid.strip()]
                return pids
            else:
                return []

        except Exception as e:
            print(f"UDPServer: 检查端口占用时出错: {e}")
            return []

    def _kill_processes_on_port(self, pids):
        """
        强制终止占用端口的进程

        Args:
            pids (list): 要终止的进程PID列表
        """
        for pid in pids:
            try:
                # 首先尝试优雅地终止进程 (SIGTERM)
                os.kill(int(pid), signal.SIGTERM)
                print(f"UDPServer: 已向进程 {pid} 发送 SIGTERM 信号")

                # 等待一小段时间让进程优雅退出
                time.sleep(0.5)

                # 检查进程是否还在运行
                try:
                    os.kill(int(pid), 0)  # 发送信号0检查进程是否存在
                    # 如果进程还在运行，强制终止 (SIGKILL)
                    os.kill(int(pid), signal.SIGKILL)
                    print(f"UDPServer: 已强制终止进程 {pid}")
                except OSError:
                    # 进程已经退出
                    print(f"UDPServer: 进程 {pid} 已退出")

            except (ValueError, OSError, ProcessLookupError) as e:
                print(f"UDPServer: 终止进程 {pid} 时出错: {e}")
                continue

    def start(self):
        """启动UDP服务器

        Returns:
            bool: 启动是否成功
        """
        max_attempts = 3  # 最大尝试次数

        for attempt in range(max_attempts):
            try:
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.socket.bind((self.host, self.port))
                self.running = True

                # 启动接收线程处理客户端连接
                receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
                receive_thread.start()

                print(f"\033[92mUDPServer: 成功绑定到端口 {self.port}\033[0m")
                return True

            except OSError as e:
                if e.errno == 98:  # Address already in use
                    print(f"\033[91mUDPServer: 端口 {self.port} 已被占用 (尝试 {attempt + 1}/{max_attempts})\033[0m")

                    # 如果启用强制端口模式且不是最后一次尝试
                    if self.force_port and attempt < max_attempts - 1:
                        # 检查占用端口的进程
                        pids = self._check_port_occupied()
                        if pids:
                            print(f"\033[93m发现占用端口 {self.port} 的进程: {', '.join(pids)}\033[0m")
                            print(f"\033[93m正在终止占用进程...\033[0m")

                            # 强制终止占用进程
                            self._kill_processes_on_port(pids)

                            # 等待一段时间让进程完全退出
                            time.sleep(1.0)

                            print(f"\033[93m等待进程退出后重试...\033[0m")
                            continue
                        else:
                            print(f"\033[93m未检测到占用端口 {self.port} 的进程，可能是端口释放延迟\033[0m")
                            time.sleep(1.0)
                            continue
                    else:
                        # 不强制终止或已达到最大尝试次数
                        print(f"\033[91mUDPServer: 端口 {self.port} 已被占用且无法强制终止\033[0m")

                        # 显示占用进程的详细信息
                        try:
                            result = subprocess.run(['lsof', '-i', f':{self.port}'], capture_output=True, text=True)
                            if result.stdout:
                                print(f"\033[93m占用端口{self.port}的进程信息:\033[0m")
                                print(f"\033[93m{result.stdout}\033[0m")
                            else:
                                print(f"\033[93m未找到占用端口{self.port}的进程\033[0m")
                        except Exception:
                            pass
                        return False
                else:
                    print(f"\033[91mUDPServer: 操作系统错误 - {str(e)}\033[0m")
                    return False

            except Exception as e:
                print(f"\033[91mUDPServer: 异常 - {str(e)}\033[0m")
                return False

        # 所有尝试都失败
        print(f"\033[91mUDPServer: 经过 {max_attempts} 次尝试后仍无法绑定到端口 {self.port}\033[0m")
        return False

    def stop(self):
        """停止UDP服务器"""
        self.running = False
        if self.socket:
            print(f"\033[91mUDPServer: 正在停止UDP服务器\033[0m")
            self.socket.close()

    def _receive_loop(self):
        """接收循环，处理客户端消息"""
        while self.running:
            try:
                self.socket.settimeout(1.0)  # 设置超时以便定期检查running状态
                data, addr = self.socket.recvfrom(1024)

                # 处理客户端消息
                try:
                    message = json.loads(data.decode('utf-8').strip())

                    # 处理HELLO注册消息
                    if message.get('action') == 'Hello':
                        client_id = message.get('client_id')
                        if not client_id:
                            print("UDPServer: HELLO消息中缺少client_id")
                            continue

                        with self.lock:
                            current_time = time.time()

                            if client_id not in self.clients:
                                # 新客户端，添加到字典
                                self.clients[client_id] = self.ClientInfo(addr=addr, last_success=current_time)
                                print(f"\033[92mUDPServer: 客户端 '{client_id}' 从 {addr} 连接\033[0m")
                                print(f"\033[96mUDPServer: 当前客户端 ({len(self.clients)}): {list(self.clients.keys())}\033[0m")
                            else:
                                # 已存在的客户端，更新地址和成功时间
                                old_addr = self.clients[client_id].addr
                                if old_addr != addr:
                                    self.clients[client_id].addr = addr
                                    print(f"\033[93mUDPServer: 客户端 '{client_id}' 地址从 {old_addr} 更新为 {addr}\033[0m")

                                # 更新最后成功时间
                                self.clients[client_id].last_success = current_time

                            # 发送OK响应
                            response = copy.deepcopy(OK_RESPONSE_TEMPLATE)
                            response['client_id'] = client_id
                            response_bytes = json.dumps(response).encode('utf-8')
                            self.socket.sendto(response_bytes, addr)
                        continue

                except (json.JSONDecodeError, UnicodeDecodeError):
                    # 忽略非JSON消息
                    pass

            except socket.timeout:
                continue  # 超时继续循环
            except Exception as e:
                if self.running:  # 只在服务器运行时记录错误
                    print(f"UDPServer: 接收循环中发生错误: {e}")

    
    def broadcast(self, data):
        """
        向所有注册的客户端发送数据

        Args:
            data: 要发送的数据，可以是字符串、字典或protobuf消息
        """
        if not self.clients:
            return

        # 准备发送数据
        if isinstance(data, dict):
            # 字典转为JSON字符串
            send_data = json.dumps(data, ensure_ascii=False).encode('utf-8')
        elif isinstance(data, str):
            send_data = data.encode('utf-8')
        elif hasattr(data, 'SerializeToString'):
            # protobuf消息
            send_data = data.SerializeToString()
        else:
            # 其他类型转为字符串
            send_data = str(data).encode('utf-8')

        # 向所有客户端发送数据
        current_time = time.time()
        failed_clients = []

        with self.lock:
            for client_id, client_info in list(self.clients.items()):
                addr = client_info.addr
                last_success = client_info.last_success

                try:
                    ret = self.socket.sendto(send_data, addr)
                    # 发送成功，更新最后成功时间
                    self.clients[client_id].last_success = current_time
                except Exception as e:
                    # 检查是否超过超时阈值
                    if current_time - last_success >= self.client_timeout:
                        failed_clients.append(client_id)

        # 移除超时的客户端
        for client_id in failed_clients:
            with self.lock:
                if client_id in self.clients:
                    addr = self.clients[client_id].addr
                    last_success = self.clients[client_id].last_success
                    timeout_duration = current_time - last_success
                    del self.clients[client_id]
                    print(f"\033[93mUDPServer: 客户端 '{client_id}' 已断开连接 (发送失败超过{timeout_duration:.1f}秒)\033[0m")

    def get_client_count(self):
        with self.lock:
            return len(self.clients)

    def get_clients(self):
        with self.lock:
            return list(self.clients.keys())

    def is_connected(self):
        with self.lock:
            return len(self.clients) > 0

    def is_running(self):
        return self.running