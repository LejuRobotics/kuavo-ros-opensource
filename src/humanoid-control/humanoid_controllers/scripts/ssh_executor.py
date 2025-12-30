#!/usr/bin/env python3
"""
SSH执行器模块
用于在上位机执行命令和脚本
"""

import json
import paramiko
import time
import os
import logging
from typing import List, Dict, Optional, Tuple, Callable
from pathlib import Path
import threading

class SSHExecutor:
    """SSH执行器类"""
    
    def __init__(self, config_file: str = "remote-config.json"):
        """
        初始化SSH执行器
        
        Args:
            config_file: 配置文件路径
        """
        self.config_file = config_file
        self.config = self._load_config()
        self.client = None
        self.connected = False
        # 设置日志
        # logging.basicConfig(level=logging.INFO)
        # self.logger = logging.getLogger(__name__)
        
        self.async_processes = {}
        
    def _load_config(self) -> Dict:
        """加载配置文件"""
        try:
            with open(self.config_file, 'r', encoding='utf-8') as f:
                return json.load(f)
        except FileNotFoundError:
            print(f"配置文件 {self.config_file} 不存在")
            raise
        except json.JSONDecodeError as e:
            print(f"配置文件格式错误: {e}")
            raise
            
    def connect(self) -> bool:
        """
        连接到上位机
        
        Returns:
            bool: 连接是否成功
        """
        try:
            self.client = paramiko.SSHClient()
            self.client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            
            ssh_config = self.config['ssh_config']
            self.client.connect(
                hostname=ssh_config['hostname'],
                port=ssh_config['port'],
                username=ssh_config['username'],
                password=ssh_config['password'],
                timeout=ssh_config['timeout']
            )
            
            self.connected = True
            print(f"\033[32m成功连接到 {ssh_config['hostname']}\033[0m")
            return True
            
        except Exception as e:
            print(f"连接失败: {e}")
            self.connected = False
            return False
            
    def disconnect(self):
        """断开连接"""
        if self.client:
            self.client.close()
            self.connected = False
            print("已断开连接")
            
    def execute_command(self, command: str, retry: bool = False, log: bool = True) -> Tuple[int, str, str]:
        """
        执行单个命令
        
        Args:
            command: 要执行的命令
            retry: 是否启用重试机制
            
        Returns:
            Tuple[int, str, str]: (返回码, 标准输出, 错误输出)
        """
        if not self.connected:
            if not self.connect():
                return -1, "", "连接失败"
                
        ssh_config = self.config['ssh_config']
        retry_count = ssh_config['retry_count'] if retry else 1
        retry_delay = ssh_config['retry_delay']
        
        for attempt in range(retry_count):
            try:
                if log:
                    print(f"执行命令: {command}")
                stdin, stdout, stderr = self.client.exec_command(command)
                
                # 获取输出
                exit_code = stdout.channel.recv_exit_status()
                output = stdout.read().decode('utf-8').strip()
                error = stderr.read().decode('utf-8').strip()
                
                if exit_code == 0:
                    # if log:
                    #     self.logger.info(f"命令执行成功")
                    return exit_code, output, error
                else:
                    if log:
                        print(f"指令返回码非0: {exit_code}")
                    if attempt < retry_count - 1:
                        if log:
                            print(f"等待 {retry_delay} 秒后重试...")
                        time.sleep(retry_delay)
                    else:
                        return exit_code, output, error
                        
            except Exception as e:
                print(f"执行命令时出错: {e}")
                if attempt < retry_count - 1:
                    print(f"等待 {retry_delay} 秒后重试...")
                    time.sleep(retry_delay)
                else:
                    return -1, "", str(e)
                    
        return -1, "", "重试次数已用完"
        
    def execute_commands(self, commands: List[str], retry: bool = True) -> List[Tuple[int, str, str]]:
        """
        执行命令序列
        
        Args:
            commands: 命令列表
            retry: 是否启用重试机制
            
        Returns:
            List[Tuple[int, str, str]]: 每个命令的执行结果
        """
        results = []
        for i, command in enumerate(commands):
            print(f"执行命令 {i+1}/{len(commands)}")
            result = self.execute_command(command, retry)
            results.append(result)
            
            # 如果命令失败且不是最后一个命令，可以选择是否继续
            if result[0] != 0 and i < len(commands) - 1:
                print(f"\033[31m指令 {i+1} 失败，继续执行下一个命令\033[0m")
                
        return results
        
    def _check_container_status(self, container_name: str) -> Tuple[bool, bool]:
        """
        检查容器状态
        
        Args:
            container_name: 容器名称
            
        Returns:
            Tuple[bool, bool]: (容器是否存在, 容器是否正在运行)
        """
        # 检查容器是否存在
        exit_code, output, error = self.execute_command(f"docker ps -a --format '{{{{.Names}}}}' | grep -w {container_name}", log=False)
        container_exists = exit_code == 0 and container_name in output
        
        if not container_exists:
            return False, False
            
        # 检查容器是否正在运行
        exit_code, output, error = self.execute_command(f"docker ps --format '{{{{.Names}}}}' | grep -w {container_name}", log=False)
        container_running = exit_code == 0 and container_name in output
        # print(f"container_running: {container_running}")
        # print(f"container_exists: {container_exists}")
        return True, container_running
        
    def _start_container(self, container_name: str) -> bool:
        """
        启动容器
        
        Args:
            container_name: 容器名称
            
        Returns:
            bool: 启动是否成功
        """
        print(f"尝试启动容器: {container_name}")
        exit_code, output, error = self.execute_command(f"docker start {container_name}")
        
        if exit_code == 0:
            print(f"容器 {container_name} 启动成功")
            return True
        else:
            print(f"容器 {container_name} 启动失败: {error}")
            return False
            
    def execute_container_command(self, command: str, container_name: str = None) -> Tuple[int, str, str]:
        """
        在容器内执行命令
        
        Args:
            command: 要执行的命令
            container_name: 容器名称，如果为None则使用配置文件中的默认容器
            
        Returns:
            Tuple[int, str, str]: 执行结果
        """
        if container_name is None:
            container_name = "vision_container"
            
        # 检查容器状态
        container_exists, container_running = self._check_container_status(container_name)
        
        if not container_exists:
            print(f"容器 {container_name} 不存在")
            return -1, "", f"容器 {container_name} 不存在"
            
        if not container_running:
            print(f"容器 {container_name} 存在但未运行，尝试启动...")
            if not self._start_container(container_name):
                return -1, "", f"容器 {container_name} 启动失败"
            
            # 等待容器完全启动
            time.sleep(2)
            
        # 执行容器命令
        docker_command = f"docker exec {container_name} {command}"
        return self.execute_command(docker_command)
        
    def execute_remote_script(self, script_path: str, args: str = "") -> Tuple[int, str, str]:
        """
        执行远程脚本
        
        Args:
            script_path: 脚本路径
            args: 脚本参数
            
        Returns:
            Tuple[int, str, str]: 执行结果
        """
        command = f"bash {script_path} {args}".strip()
        return self.execute_command(command)
        
    def execute_local_script(self, local_script_path: str, args: str = "", remote_path: str = None, cleanup: bool = True) -> Tuple[int, str, str]:
        """
        在远程执行本地脚本
        
        Args:
            local_script_path: 本地脚本路径
            args: 脚本参数
            remote_path: 远程脚本路径，如果为None则使用临时路径
            cleanup: 是否在执行后清理远程脚本文件
            
        Returns:
            Tuple[int, str, str]: 执行结果
        """
        if not os.path.exists(local_script_path):
            print(f"本地脚本文件不存在: {local_script_path}")
            return -1, "", f"本地脚本文件不存在: {local_script_path}"
            
        try:
            # 确定远程脚本路径
            if remote_path is None:
                script_name = os.path.basename(local_script_path)
                remote_path = f"/tmp/{script_name}_{int(time.time())}"
                
            print(f"上传本地脚本到远程: {local_script_path} -> {remote_path}")
            
            # 上传脚本文件
            if not self.upload_file(local_script_path, remote_path):
                return -1, "", "脚本文件上传失败"
                
            # 设置执行权限
            exit_code, output, error = self.execute_command(f"chmod +x {remote_path}")
            if exit_code != 0:
                print(f"设置脚本执行权限失败: {error}")
                
            # 执行脚本
            print(f"在远程执行脚本: {remote_path} {args}")
            result = self.execute_remote_script(remote_path, args)
            
            # 清理远程脚本文件
            if cleanup:
                try:
                    self.execute_command(f"rm -f {remote_path}")
                    print(f"已清理远程脚本文件: {remote_path}")
                except Exception as e:
                    print(f"清理远程脚本文件失败: {e}")
                    
            return result
            
        except Exception as e:
            print(f"执行本地脚本时出错: {e}")
            return -1, "", str(e)
            
    def execute_local_script_in_container(self, local_script_path: str, args: str = "", container_name: str = None, remote_path: str = None, cleanup: bool = True) -> Tuple[int, str, str]:
        """
        在远程容器中执行本地脚本
        
        Args:
            local_script_path: 本地脚本路径
            args: 脚本参数
            container_name: 容器名称，如果为None则使用配置文件中的默认容器
            remote_path: 远程脚本路径，如果为None则使用临时路径
            cleanup: 是否在执行后清理远程脚本文件
            
        Returns:
            Tuple[int, str, str]: 执行结果
        """
        if container_name is None:
            container_name = "vision_container"
            
        if not os.path.exists(local_script_path):
            print(f"本地脚本文件不存在: {local_script_path}")
            return -1, "", f"本地脚本文件不存在: {local_script_path}"
            
        try:
            # 确定远程脚本路径
            if remote_path is None:
                script_name = os.path.basename(local_script_path)
                remote_path = f"/tmp/{script_name}_{int(time.time())}"
                
            print(f"上传本地脚本到远程容器: {local_script_path} -> {remote_path}")
            
            # 上传脚本文件到容器
            if not self.upload_file(local_script_path, remote_path):
                return -1, "", "脚本文件上传失败"
                
            # 在容器内设置执行权限
            exit_code, output, error = self.execute_container_command(f"chmod +x {remote_path}", container_name)
            if exit_code != 0:
                print(f"在容器内设置脚本执行权限失败: {error}")
                
            # 在容器内执行脚本
            print(f"在容器内执行脚本: {remote_path} {args}")
            docker_command = f"bash {remote_path} {args}".strip()
            result = self.execute_container_command(docker_command, container_name)
            
            # 清理远程脚本文件
            if cleanup:
                try:
                    self.execute_command(f"rm -f {remote_path}")
                    print(f"已清理远程脚本文件: {remote_path}")
                except Exception as e:
                    print(f"清理远程脚本文件失败: {e}")
                    
            return result
            
        except Exception as e:
            print(f"在容器内执行本地脚本时出错: {e}")
            return -1, "", str(e)
        
    def upload_file(self, local_path: str, remote_path: str = None) -> bool:
        """
        上传文件到上位机
        
        Args:
            local_path: 本地文件路径
            remote_path: 远程文件路径，如果为None则使用配置文件中的默认路径
            
        Returns:
            bool: 上传是否成功
        """
        if not self.connected:
            if not self.connect():
                return False
                
        try:
            if remote_path is None:
                upload_path = self.config['file_transfer']['upload_path']
                filename = os.path.basename(local_path)
                remote_path = f"{upload_path}/{filename}"
                
            # 确保远程目录存在
            remote_dir = os.path.dirname(remote_path)
            self.execute_command(f"mkdir -p {remote_dir}")
            
            # 使用SFTP上传文件
            sftp = self.client.open_sftp()
            sftp.put(local_path, remote_path)
            sftp.close()
            
            print(f"文件上传成功: {local_path} -> {remote_path}")
            return True
            
        except Exception as e:
            print(f"文件上传失败: {e}")
            return False
            
    def download_file(self, remote_path: str, local_path: str = None) -> bool:
        """
        从上位机下载文件
        
        Args:
            remote_path: 远程文件路径
            local_path: 本地文件路径，如果为None则使用配置文件中的默认路径
            
        Returns:
            bool: 下载是否成功
        """
        if not self.connected:
            if not self.connect():
                return False
                
        try:
            if local_path is None:
                download_path = self.config['file_transfer']['download_path']
                filename = os.path.basename(remote_path)
                local_path = f"{download_path}/{filename}"
                
            # 确保本地目录存在
            os.makedirs(os.path.dirname(local_path), exist_ok=True)
            
            # 使用SFTP下载文件
            sftp = self.client.open_sftp()
            sftp.get(remote_path, local_path)
            sftp.close()
            
            print(f"文件下载成功: {remote_path} -> {local_path}")
            return True
            
        except Exception as e:
            print(f"文件下载失败: {e}")
            return False
            
    def execute_predefined_command(self, command_name: str) -> List[Tuple[int, str, str]]:
        """
        执行预定义的命令组
        
        Args:
            command_name: 命令组名称
            
        Returns:
            List[Tuple[int, str, str]]: 执行结果
        """
        if command_name not in self.config['commands']:
            print(f"未找到预定义命令: {command_name}")
            return []
        print(f"执行预定义命令: {command_name}")
        commands = self.config['commands'][command_name]
        if isinstance(commands, list):
            return self.execute_commands(commands)
        else:
            return [self.execute_command(commands)]
            
    def execute_predefined_container_command(self, command_name: str, container_name: str = None) -> List[Tuple[int, str, str]]:
        """
        在容器中执行预定义的命令组
        
        Args:
            command_name: 容器命令组名称
            container_name: 容器名称，如果为None则使用配置文件中的默认容器
            
        Returns:
            List[Tuple[int, str, str]]: 执行结果
        """
        if 'container-commands' not in self.config:
            print("配置文件中未找到 container-commands 配置")
            return []
            
        if command_name not in self.config['container-commands']:
            print(f"未找到预定义容器命令: {command_name}")
            return []
            
        print(f"\033[32m预定义容器命令:\033[0m {command_name}")
        commands = self.config['container-commands'][command_name]
        
        # 使用配置文件中的默认容器名称
        if container_name is None:
            container_name = self.config.get('default_container', 'vision_container')
            
        results = []
        if isinstance(commands, list):
            for i, command in enumerate(commands):
                print(f"\033[32m{command_name}: \033[0m{i+1}/{len(commands)}: {command}")
                result = self.execute_container_command(command, container_name)
                results.append(result)
                
                # 如果命令失败且不是最后一个命令，可以选择是否继续
                if result[0] != 0 and i < len(commands) - 1:
                    print(f"\033[31m容器命令 {i+1} 失败，继续执行下一个命令\033[0m")
        else:
            result = self.execute_container_command(commands, container_name)
            results.append(result)
            
        return results
            
    def get_system_info(self) -> Dict[str, str]:
        """
        获取系统信息
        
        Returns:
            Dict[str, str]: 系统信息
        """
        info = {}
        
        # 获取系统版本
        exit_code, output, error = self.execute_command("cat /etc/os-release | grep PRETTY_NAME")
        if exit_code == 0:
            info['os_version'] = output.split('=')[1].strip('"')
            
        # 获取内存信息
        exit_code, output, error = self.execute_command("free -h | grep Mem")
        if exit_code == 0:
            info['memory'] = output.split()[1]
            
        # 获取磁盘信息
        exit_code, output, error = self.execute_command("df -h / | tail -1")
        if exit_code == 0:
            info['disk_usage'] = output.split()[4]
            
        # 获取CPU信息
        exit_code, output, error = self.execute_command("nproc")
        if exit_code == 0:
            info['cpu_cores'] = output.strip()
            
        return info
 
    def __enter__(self):
        """上下文管理器入口"""
        self.connect()
        return self
        
    def __exit__(self, exc_type, exc_val, exc_tb):
        """上下文管理器出口"""
        self.disconnect()
        
    def execute_predefined_command_async(self, command_name: str, process_id: str = None, 
                                       callback: Callable = None, background: bool = True) -> str:
        """
        异步执行预定义的命令组
        
        Args:
            command_name: 命令组名称
            process_id: 进程ID，如果为None则自动生成
            callback: 完成时的回调函数
            background: 是否在后台运行
            
        Returns:
            str: 任务ID
        """
        if command_name not in self.config['commands']:
            print(f"未找到预定义命令: {command_name}")
            return ""
            
        if process_id is None:
            process_id = f"predefined_cmd_{int(time.time())}_{command_name}"
            
        print(f"异步执行预定义命令: {command_name} (ID: {process_id})")
        
        def async_executor():
            try:
                commands = self.config['commands'][command_name]
                results = []
                
                if isinstance(commands, list):
                    for i, command in enumerate(commands):
                        print(f"执行命令 {i+1}/{len(commands)}: {command}")
                        result = self.execute_command(command, retry=True, log=False)
                        results.append(result)
                        
                        # 如果命令失败且不是最后一个命令，可以选择是否继续
                        if result[0] != 0 and i < len(commands) - 1:
                            print(f"\033[31m命令 {i+1} 失败，继续执行下一个命令\033[0m")
                else:
                    result = self.execute_command(commands, retry=True, log=False)
                    results.append(result)
                    
                # 更新任务状态
                if process_id in self.async_processes:
                    self.async_processes[process_id]['status'] = 'completed'
                    self.async_processes[process_id]['results'] = results
                    self.async_processes[process_id]['end_time'] = time.time()
                    
                # 执行回调
                if callback:
                    try:
                        callback(process_id, results)
                    except Exception as e:
                        print(f"回调函数执行失败: {e}")
                        
            except Exception as e:
                print(f"异步执行预定义命令失败: {e}")
                if process_id in self.async_processes:
                    self.async_processes[process_id]['status'] = 'failed'
                    self.async_processes[process_id]['error'] = str(e)
                    self.async_processes[process_id]['end_time'] = time.time()
                    
        # 创建任务记录
        self.async_processes[process_id] = {
            'type': 'predefined_command',
            'command_name': command_name,
            'status': 'running',
            'start_time': time.time(),
            'thread': None,
            'results': None,
            'error': None,
            'end_time': None
        }
        
        # 启动线程
        thread = threading.Thread(target=async_executor, daemon=True)
        thread.start()
        self.async_processes[process_id]['thread'] = thread
        
        if not background:
            thread.join()
            
        return process_id
        
    def execute_predefined_container_command_async(self, command_name: str, container_name: str = None,
                                                 process_id: str = None, callback: Callable = None, 
                                                 background: bool = True) -> str:
        """
        异步执行预定义的容器命令组
        
        Args:
            command_name: 容器命令组名称
            container_name: 容器名称，如果为None则使用配置文件中的默认容器
            process_id: 进程ID，如果为None则自动生成
            callback: 完成时的回调函数
            background: 是否在后台运行
            
        Returns:
            str: 任务ID
        """
        if 'container-commands' not in self.config:
            print("配置文件中未找到 container-commands 配置")
            return ""
            
        if command_name not in self.config['container-commands']:
            print(f"未找到预定义容器命令: {command_name}")
            return ""
            
        # 使用配置文件中的默认容器名称
        if container_name is None:
            container_name = self.config.get('default_container', 'vision_container')
            
        if process_id is None:
            process_id = f"container_cmd_{int(time.time())}_{command_name}"
            
        print(f"\033[32m预定义容器命令(异步):\033[0m {command_name} (ID: {process_id})")
        
        def async_container_executor():
            try:
                commands = self.config['container-commands'][command_name]
                results = []
                
                if isinstance(commands, list):
                    for i, command in enumerate(commands):
                        print(f"\033[32m{command_name}: \033[0m{i+1}/{len(commands)}: {command}")
                        result = self.execute_container_command(command, container_name)
                        results.append(result)
                        
                        # 如果命令失败且不是最后一个命令，可以选择是否继续
                        if result[0] != 0 and i < len(commands) - 1:
                            print(f"\033[31m容器命令 {i+1} 失败，继续执行下一个命令\033[0m")
                else:
                    result = self.execute_container_command(commands, container_name)
                    results.append(result)
                    
                # 更新任务状态
                if process_id in self.async_processes:
                    self.async_processes[process_id]['status'] = 'completed'
                    self.async_processes[process_id]['results'] = results
                    self.async_processes[process_id]['end_time'] = time.time()
                    
                # 执行回调
                if callback:
                    try:
                        callback(process_id, results)
                    except Exception as e:
                        print(f"回调函数执行失败: {e}")
                        
            except Exception as e:
                print(f"异步执行预定义容器命令失败: {e}")
                if process_id in self.async_processes:
                    self.async_processes[process_id]['status'] = 'failed'
                    self.async_processes[process_id]['error'] = str(e)
                    self.async_processes[process_id]['end_time'] = time.time()
                    
        # 创建任务记录
        self.async_processes[process_id] = {
            'type': 'predefined_container_command',
            'command_name': command_name,
            'container_name': container_name,
            'status': 'running',
            'start_time': time.time(),
            'thread': None,
            'results': None,
            'error': None,
            'end_time': None
        }
        
        # 启动线程
        thread = threading.Thread(target=async_container_executor, daemon=True)
        thread.start()
        self.async_processes[process_id]['thread'] = thread
        
        if not background:
            thread.join()
            
        return process_id
        
    def stop_async_task(self, task_id: str, force: bool = False) -> bool:
        """
        停止特定的异步任务
        
        Args:
            task_id: 任务ID
            force: 是否强制停止
            
        Returns:
            bool: 停止是否成功
        """
        if task_id not in self.async_processes:
            print(f"任务 {task_id} 不存在")
            return False
            
        task = self.async_processes[task_id]
        
        if task['status'] in ['completed', 'failed', 'stopped']:
            print(f"任务 {task_id} 已经结束，状态: {task['status']}")
            return True
            
        try:
            # 标记任务为停止状态
            task['status'] = 'stopped'
            task['end_time'] = time.time()
            
            # 如果是容器命令，可以尝试停止相关的容器进程
            if task['type'] == 'predefined_container_command':
                container_name = task.get('container_name', 'vision_container')
                command_name = task.get('command_name', '')
                
                # 尝试停止容器内的相关进程
                if 'staircase' in command_name.lower():
                    self.execute_container_command("pkill -f stair_detection", container_name)
                    self.execute_container_command("pkill -f python3", container_name)
                    
            print(f"任务 {task_id} 已停止")
            return True
            
        except Exception as e:
            print(f"停止任务 {task_id} 失败: {e}")
            return False
            
    def stop_all_async_tasks(self, force: bool = False) -> bool:
        """
        停止所有异步任务
        
        Args:
            force: 是否强制停止
            
        Returns:
            bool: 停止是否成功
        """
        if not self.async_processes:
            print("没有正在运行的异步任务")
            return True
            
        success = True
        task_ids = list(self.async_processes.keys())
        
        for task_id in task_ids:
            if not self.stop_async_task(task_id, force):
                success = False
                
        return success
        
    def get_async_task_status(self, task_id: str) -> Dict:
        """
        获取异步任务状态
        
        Args:
            task_id: 任务ID
            
        Returns:
            Dict: 任务状态信息
        """
        if task_id not in self.async_processes:
            return {
                'status': 'not_found',
                'error': f'任务 {task_id} 不存在'
            }
            
        task = self.async_processes[task_id]
        status_info = {
            'task_id': task_id,
            'type': task['type'],
            'status': task['status'],
            'start_time': task['start_time'],
            'end_time': task.get('end_time'),
            'duration': task.get('end_time', time.time()) - task['start_time'] if task.get('end_time') else time.time() - task['start_time']
        }
        
        if task['type'] == 'predefined_command':
            status_info['command_name'] = task['command_name']
        elif task['type'] == 'predefined_container_command':
            status_info['command_name'] = task['command_name']
            status_info['container_name'] = task['container_name']
            
        if task['status'] == 'failed':
            status_info['error'] = task.get('error')
        elif task['status'] == 'completed':
            status_info['results'] = task.get('results')
            
        return status_info
        
    def list_async_tasks(self) -> List[Dict]:
        """
        列出所有异步任务
        
        Returns:
            List[Dict]: 任务列表
        """
        tasks = []
        for task_id in self.async_processes:
            tasks.append(self.get_async_task_status(task_id))
        return tasks
        
    def cleanup_async_tasks(self):
        """清理所有异步任务"""
        self.stop_all_async_tasks()
        self.async_processes.clear()
        print("已清理所有异步任务")
        
    def start_stair_detection(self, camera_type=None):
        """
        启动远程楼梯识别（与 stairClimbPlanner-vision.py 中一致）
        """
        self.execute_predefined_command("xhost_on")
        print("camera_type:",camera_type)
        if camera_type == "orbbec":
            self.execute_predefined_command_async("launch_orbbec_camera")
            print("启动上位机 orbbec 相机")
        self.execute_predefined_container_command_async("run_staircase_detection")
        if camera_type == "orbbec":
            self.execute_predefined_container_command_async("start_staircase_detection_orbbec")
        else:
            self.execute_predefined_container_command_async("start_staircase_detection")
        print("已远程启动楼梯识别脚本")

    def stop_stair_detection(self):
        """
        关闭远程楼梯识别
        """
        self.execute_predefined_container_command("kill_staircase_detection")
        print("已远程关闭楼梯识别脚本")
 