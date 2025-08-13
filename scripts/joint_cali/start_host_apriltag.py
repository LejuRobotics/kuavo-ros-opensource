#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import paramiko
import threading
import time
import sys
import os
import subprocess

class HostAprilTagLauncher:
    def __init__(self):
        result = subprocess.run(['systemctl', 'is-active', 'isc-dhcp-server'])
        if result.returncode == 0:
            remote_host = "192.168.26.12"
        else:
            remote_host = "192.168.26.1"

        robot_version = os.environ.get('ROBOT_VERSION')
        # 49 为 orbbec 相机
        if robot_version == "49":
            launch_name = "orbbec_sensor_robot_enable.launch"
            launch_file = "src/dynamic_biped/launch/" + launch_name
            launch_cmd = 'nohup bash -ic "cd ~/kuavo_ros_application && source devel/setup.bash && roslaunch dynamic_biped ' + launch_name + '" > apriltag_launch.log 2>&1 &'
        else:
            launch_name = "sensor_apriltag_only_enable.launch"  
            launch_file = "src/dynamic_biped/launch/" + launch_name
            launch_cmd = 'nohup bash -ic "cd ~/kuavo_ros_application && source devel/setup.bash && roslaunch dynamic_biped ' + launch_name + '" > apriltag_launch.log 2>&1 &'
        
        
        self.config = {
            "name": "apriltag_detection",
            "host": remote_host,
            "port": 22,
            "username1": "kuavo",
            "username2": "leju_kuavo",
            "password": "leju_kuavo",
            "remote_path": "~/kuavo_ros_application",
            "launch_name": launch_name,
            "launch_file": launch_file,
            "launch_cmd": launch_cmd,
            "nodes_to_check": [
                "/apriltag_ros_continuous_node",
                "/ar_control_node", 
                "/camera_to_real_frame"
            ]
        }
        self.ssh = None

    def print_output(self, stream, prefix=""):
        """实时输出函数"""
        for line in iter(stream.readline, ""):
            print(f"[{prefix}] {line.strip()}")

    def connect_ssh(self):
        """建立SSH连接，尝试两个用户名"""
        user_attempts = [self.config["username1"], self.config["username2"]] # Assuming your config might have these keys

        for username in user_attempts:
            if not username: # Skip if the username is None or empty
                continue

            try:
                print(f"=== 尝试使用用户名 '{username}' 连接上位机 {self.config['host']} ===")
                self.ssh = paramiko.SSHClient()
                self.ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

                self.ssh.connect(
                    hostname=self.config["host"],
                    port=self.config["port"],
                    username=username, # Use the current username in the loop
                    password=self.config["password"],
                    timeout=10
                )
                print(f"✓ SSH连接成功，使用用户名: {username}")
                return True
            except Exception as e:
                print(f"✗ SSH连接失败 (用户名: {username}): {e}")
                # Continue to the next username if this one fails

        print("✗ 所有SSH用户名尝试均失败。")
        return False

    def check_remote_path(self):
        """检查远程路径是否存在"""
        try:
            print("步骤1: 检查工作目录...")
            remote_path = self.config["remote_path"]
            command = f"test -d {remote_path} && echo 'Exists' || echo 'Not exists'"
            stdin, stdout, stderr = self.ssh.exec_command(command)
            result = stdout.read().decode().strip()
            
            if result != 'Exists':
                print(f"✗ 错误：工作目录不存在: {remote_path}")
                return False
            
            print(f"✓ 工作目录存在: {remote_path}")
            return True
        except Exception as e:
            print(f"✗ 检查工作目录失败: {e}")
            return False

    def check_launch_file(self):
        """检查launch文件是否存在"""
        try:
            print("步骤2: 检查launch文件...")
            launch_file_path = f"{self.config['remote_path']}/{self.config['launch_file']}"
            command = f"test -f {launch_file_path} && echo 'Exists' || echo 'Not exists'"
            stdin, stdout, stderr = self.ssh.exec_command(command)
            result = stdout.read().decode().strip()
            
            if result != 'Exists':
                print(f"✗ 错误：launch文件不存在: {launch_file_path}")
                # 尝试查找相关文件
                print("查找相关launch文件:")
                find_cmd = f"find {self.config['remote_path']} -name '*apriltag*' -type f 2>/dev/null | head -5"
                stdin, stdout, stderr = self.ssh.exec_command(find_cmd)
                found_files = stdout.read().decode().strip()
                if found_files:
                    print(f"找到相关文件:\n{found_files}")
                else:
                    print("未找到相关AprilTag文件")
                return False
            
            print(f"✓ launch文件存在: {self.config['launch_file']}")
            return True
        except Exception as e:
            print(f"✗ 检查launch文件失败: {e}")
            return False

    def kill_old_processes(self):
        """清理旧进程"""
        try:
            print("步骤3: 清理旧进程...")
            
            # 查找相关进程
            check_cmd = "pgrep -f '" + self.config["launch_name"] + "' || echo 'no_process'"
            stdin, stdout, stderr = self.ssh.exec_command(check_cmd)
            result = stdout.read().decode().strip()
            
            if result == 'no_process':
                print("✓ 未发现旧进程")
                return True
            
            print(f"发现运行中的进程: {result}")
            
            # 使用root权限优雅停止进程
            print("使用root权限停止进程...")
            kill_cmd = "echo 'leju_kuavo' | sudo -S pkill -15 -f '" + self.config["launch_name"] + "'"
            stdin, stdout, stderr = self.ssh.exec_command(kill_cmd)
            time.sleep(3)
            
            # 使用root权限强制停止残留进程
            print("强制停止残留进程...")
            force_kill_cmd = "echo 'leju_kuavo' | sudo -S pkill -9 -f '" + self.config["launch_name"] + "'"
            stdin, stdout, stderr = self.ssh.exec_command(force_kill_cmd)
            time.sleep(2)
            
            # 额外清理：停止所有ROS相关进程（使用root权限）
            print("清理其他可能的ROS进程...")
            cleanup_cmd = "echo 'leju_kuavo' | sudo -S pkill -f 'ros'"
            stdin, stdout, stderr = self.ssh.exec_command(cleanup_cmd)
            time.sleep(1)
            
            print("✓ 进程清理完成")
            return True
        except Exception as e:
            print(f"✗ 清理进程失败: {e}")
            return False

    def start_apriltag_service(self):
        """启动AprilTag识别服务"""
        try:
            print("步骤4: 启动AprilTag识别系统...")
            
            # 修改启动命令，确保权限正确
            launch_cmd = self.config["launch_cmd"]
            print(f"执行命令: {launch_cmd}")
            
            stdin, stdout, stderr = self.ssh.exec_command(launch_cmd)
            
            # 启动输出监控线程
            stdout_thread = threading.Thread(
                target=self.print_output,
                args=(stdout, "APRILTAG_OUT")
            )
            stderr_thread = threading.Thread(
                target=self.print_output, 
                args=(stderr, "APRILTAG_ERR")
            )
            stdout_thread.daemon = True
            stderr_thread.daemon = True
            stdout_thread.start()
            stderr_thread.start()
            
            print("✓ 启动命令已执行")
            return True
        except Exception as e:
            print(f"✗ 启动AprilTag服务失败: {e}")
            return False

    def check_nodes(self):
        """检查节点是否正常启动"""
        try:
            print("步骤5: 检查节点启动状态...")
            time.sleep(8)  # 等待启动
            
            source_command = 'bash -ic "source ~/kuavo_ros_application/devel/setup.bash && rosnode list"'
            stdin, stdout, stderr = self.ssh.exec_command(source_command)
            output = stdout.read().decode()
            
            if not output.strip():
                print("⚠ 警告：无法获取节点列表，可能ROS环境未正确设置")
                return False
            
            running_nodes = output.strip().split('\n')
            print(f"当前运行的节点: {len(running_nodes)} 个")
            
            missing_nodes = []
            for node in self.config["nodes_to_check"]:
                if node not in running_nodes:
                    missing_nodes.append(node)
            
            if missing_nodes:
                print(f"⚠ 以下关键节点未启动: {', '.join(missing_nodes)}")
                print("运行中的节点:")
                for node in running_nodes:
                    if node.strip():
                        print(f"  - {node}")
                return False
            
            print("✓ 所有关键节点启动成功")
            return True
        except Exception as e:
            print(f"✗ 检查节点失败: {e}")
            return False

    def verify_processes(self):
        """验证进程是否在运行"""
        try:
            print("步骤6: 验证进程运行状态...")
            check_cmd = "pgrep -f '" + self.config["launch_name"] + "'"
            stdin, stdout, stderr = self.ssh.exec_command(check_cmd)
            result = stdout.read().decode().strip()
            
            if result:
                print(f"✓ AprilTag进程正在运行 (PID: {result})")
                return True
            else:
                print("✗ AprilTag进程未运行")
                # 尝试查看日志
                log_cmd = "tail -10 ~/kuavo_ros_application/apriltag_launch.log 2>/dev/null || echo 'No log file'"
                stdin, stdout, stderr = self.ssh.exec_command(log_cmd)
                log_output = stdout.read().decode().strip()
                print(f"日志内容:\n{log_output}")
                return False
        except Exception as e:
            print(f"✗ 验证进程失败: {e}")
            return False

    def run(self):
        """主运行函数"""
        print("=========================================================")
        print("           启动上位机AprilTag识别系统")
        print("=========================================================")
        
        try:
            # 1. 建立SSH连接
            if not self.connect_ssh():
                return False
            
            # 2. 检查环境
            if not self.check_remote_path():
                return False
            
            if not self.check_launch_file():
                return False
            
            # 3. 清理旧进程
            if not self.kill_old_processes():
                return False
            
            # 4. 启动服务
            if not self.start_apriltag_service():
                return False
            
            # 5. 验证启动
            process_ok = self.verify_processes()
            nodes_ok = self.check_nodes()
            
            if process_ok and nodes_ok:
                print("=========================================================")
                print("✓ 上位机AprilTag识别系统启动成功！")
                print("=========================================================")
                return True
            elif process_ok:
                print("=========================================================")
                print("⚠ 上位机AprilTag进程启动，但部分节点未正常运行")
                print("建议检查ROS环境和网络连接")
                print("=========================================================")
                return True
            else:
                print("=========================================================")
                print("✗ 上位机AprilTag识别系统启动失败")
                print("=========================================================")
                return False
                
        except Exception as e:
            print(f"执行过程中发生错误: {e}")
            return False
        finally:
            if self.ssh:
                self.ssh.close()
                print("SSH连接已关闭")

def main():
    launcher = HostAprilTagLauncher()
    success = launcher.run()
    sys.exit(0 if success else 1)

if __name__ == "__main__":
    main() 