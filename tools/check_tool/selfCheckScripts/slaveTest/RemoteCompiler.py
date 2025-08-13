# import paramiko
# import time

# # 配置信息
# remote_host = '192.168.26.1'
# remote_port = 22
# remote_username = 'kuavo'
# remote_password = 'leju_kuavo'
# workspace_path = '/home/kuavo/kuavo_ros_navigation'
# ros_distro = 'noetic'  # 确保此处与远程实际版本一致

# # 创建SSH客户端
# ssh = paramiko.SSHClient()
# ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

# try:
#     ssh.connect(remote_host, remote_port, remote_username, remote_password)
#     print("成功连接到远程主机。")

#     # 关键修改：完整的环境初始化流程并添加退出命令
#     command = f"""
#     unset ROS_DISTRO
#     source /etc/profile
#     source ~/.bashrc
#     source /opt/ros/{ros_distro}/setup.bash
#     export PATH=$PATH:/opt/ros/{ros_distro}/bin
#     cd {workspace_path}
#     rm -rf build devel
#     catkin build livox_ros_driver2
#     echo "编译完成，Ctrl+C退出远程Terminal..."
#     """

#     # 使用登录Shell并分配伪终端
#     channel = ssh.invoke_shell()
#     # 将多行命令一次性发送（注意处理换行符）
#     channel.send(f'/bin/bash -l -c "{command}"\n')

#     # 动态等待命令执行完成
#     output = ''
#     compile_success = False
#     while True:
#         if channel.recv_ready():
#             data = channel.recv(65535).decode('utf-8', errors='ignore')
#             output += data
#             print(data, end='')
            
#             # 检测编译成功的关键字
#             if "Failed:    None." in data:
#                 compile_success = True
#                 print("\n检测到编译成功！")
#                 # 主动发送exit命令
#                 channel.send("exit\n")
                
#         # 检测通道关闭状态
#         if channel.exit_status_ready():
#             exit_status = channel.recv_exit_status()
#             print(f"\n远程Shell已退出，状态码：{exit_status}")
#             break
#         time.sleep(0.1)

#     # 验证编译结果
#     if compile_success and exit_status == 0:
#         print("\n✅ 编译成功！")
#         ssh.close()
#     else:
#         print(f"\n❌ 编译失败！退出状态码：{exit_status}")
#         ssh.close()

# except Exception as e:
#     print(f"连接或执行命令时出现错误：{e}")
# finally:
#     ssh.close()


# import paramiko
# import time

# class RemoteCompiler:
#     """远程编译ROS包的工具类"""
    
#     def __init__(self, host, port=22, username=None, password=None):
#         """初始化远程编译器"""
#         self.host = host
#         self.port = port
#         self.username = username
#         self.password = password
        
#     def compile_package(self, workspace_path, package_name, ros_distro='noetic'):
#         """编译指定工作空间中的ROS包"""
#         # 创建SSH客户端
#         ssh = paramiko.SSHClient()
#         ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        
#         try:
#             # 连接到远程主机
#             ssh.connect(self.host, self.port, self.username, self.password)
#             print(f"成功连接到远程主机: {self.host}")
            
#             # 构建编译命令
#             command = f"""
#             unset ROS_DISTRO
#             source /etc/profile
#             source ~/.bashrc
#             source /opt/ros/{ros_distro}/setup.bash
#             export PATH=$PATH:/opt/ros/{ros_distro}/bin
#             cd {workspace_path}
#             rm -rf build devel
#             catkin build {package_name}
#             echo "编译完成，退出远程Terminal..."
#             """
            
#             # 使用登录Shell并分配伪终端
#             channel = ssh.invoke_shell()
#             # 将多行命令一次性发送
#             channel.send(f'/bin/bash -l -c "{command}"\n')
            
#             # 动态等待命令执行完成
#             output = ''
#             compile_success = False
            
#             while True:
#                 if channel.recv_ready():
#                     data = channel.recv(65535).decode('utf-8', errors='ignore')
#                     output += data
#                     print(data, end='')
                    
#                     # 检测编译成功的关键字
#                     if "Failed:    None." in data:
#                         compile_success = True
#                         print("\n检测到编译成功！")
#                         # 主动发送exit命令
#                         channel.send("exit\n")
                
#                 # 检测通道关闭状态
#                 if channel.exit_status_ready():
#                     exit_status = channel.recv_exit_status()
#                     print(f"\n远程Shell已退出，状态码：{exit_status}")
#                     break
#                 time.sleep(0.1)
            
#             # 验证编译结果
#             if compile_success and exit_status == 0:
#                 print("\n✅ 编译成功！")
#                 return True
#             else:
#                 print(f"\n❌ 编译失败！退出状态码：{exit_status}")
#                 return False
                
#         except Exception as e:
#             print(f"连接或执行命令时出现错误：{e}")
#             return False
#         finally:
#             ssh.close()

# # 示例使用（当作为脚本直接运行时）
# if __name__ == "__main__":
#     # 创建编译器实例
#     compiler = RemoteCompiler(
#         host='192.168.26.1',
#         username='kuavo',
#         password='leju_kuavo'
#     )
    
#     # 编译指定包
#     success = compiler.compile_package(
#         workspace_path='/home/kuavo/kuavo_ros_navigation',
#         package_name='livox_ros_driver2'
#     )
    
#     # 输出结果
#     print(f"编译结果: {'成功' if success else '失败'}")


import paramiko
import time
import sys

class ROSRemoteCompiler:
    """远程编译ROS包的工具类"""
    
    def __init__(self, host, port=22, username=None, password=None):
        """初始化远程编译器"""
        self.host = host
        self.port = port
        self.username = username
        self.password = password
        
    def compile_packages(self, workspace_path, package_list, ros_distro='noetic'):
        """编译多个ROS包（按顺序编译，支持依赖关系）"""
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        
        try:
            ssh.connect(self.host, self.port, self.username, self.password)
            print(f"成功连接到远程主机: {self.host}")
            
            # 构建按顺序编译的命令（使用 && 连接）
            compile_commands = []
            for pkg in package_list:
                compile_commands.append(f"catkin build {pkg}")
            
            command = f"""
            unset ROS_DISTRO
            source /etc/profile
            source ~/.bashrc
            source /opt/ros/{ros_distro}/setup.bash
            export PATH=$PATH:/opt/ros/{ros_distro}/bin
            cd {workspace_path}
            rm -rf build devel
            {' && '.join(compile_commands)}  # 使用 && 连接所有编译命令
            echo "===ALL_PACKAGES_BUILT_SUCCESSFULLY==="  # 添加标志性输出，用于检测整体成功
            """
            
            channel = ssh.invoke_shell()
            channel.send(f'/bin/bash -l -c "{command}"\n')
            
            output = ''
            all_packages_success = False
            exit_command_sent = False
            
            while True:
                # 检查通道是否已关闭
                if channel.exit_status_ready():
                    exit_status = channel.recv_exit_status()
                    print(f"\n远程Shell已退出，状态码：{exit_status}")
                    break
                
                # 读取数据
                if channel.recv_ready():
                    try:
                        data = channel.recv(65535).decode('utf-8', errors='ignore')
                        output += data
                        # print(data, end='')
                        
                        # 检测整体编译成功的标志性输出
                        if "===ALL_PACKAGES_BUILT_SUCCESSFULLY===" in data and not exit_command_sent:
                            all_packages_success = True
                            # print("检测到编译成功，准备退出...")
                            # 等待一小段时间确保所有输出都被接收
                            time.sleep(1)
                            # 发送exit命令
                            channel.send("exit\n")
                            exit_command_sent = True
                    except Exception as e:
                        print(f"读取输出时出错: {e}")
                        break
                
                time.sleep(0.1)
            
            # 验证编译结果（需同时满足标志性输出和状态码0）
            if all_packages_success and exit_status == 0:
                print("\n✅ 所有包编译成功！")
                return True
            else:
                print(f"\n❌ 编译失败！状态码：{exit_status}")
                return False
                
        except Exception as e:
            print(f"编译过程中出现错误：{e}")
            return False
        finally:
            # 确保SSH连接被关闭
            try:
                if ssh and ssh.get_transport() and ssh.get_transport().is_active():
                    ssh.close()
            except:
                pass

if __name__ == "__main__":
    # 测试代码（可选）
    import argparse
    
    parser = argparse.ArgumentParser(description='远程编译ROS包')
    parser.add_argument('--host', required=True, help='远程主机IP')
    parser.add_argument('--username', required=True, help='用户名')
    parser.add_argument('--password', required=True, help='密码')
    parser.add_argument('--workspace', required=True, help='工作空间路径')
    parser.add_argument('--packages', required=True, help='要编译的包列表，用逗号分隔')
    parser.add_argument('--ros-distro', default='noetic', help='ROS版本')
    
    args = parser.parse_args()
    package_list = args.packages.split(',')
    
    compiler = ROSRemoteCompiler(
        host=args.host,
        username=args.username,
        password=args.password
    )
    
    success = compiler.compile_packages(
        workspace_path=args.workspace,
        package_list=package_list,
        ros_distro=args.ros_distro
    )
    
    sys.exit(0 if success else 1)