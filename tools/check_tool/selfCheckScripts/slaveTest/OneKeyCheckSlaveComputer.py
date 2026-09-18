import subprocess
import paramiko
import os
from common.common_utils import print_colored_text
from sshConnAndExec import start_all_services, close_all_connections

# 远程主机信息
host = '192.168.26.1'
port = 22
username = 'kuavo'
password = 'leju_kuavo'

active_ssh_connections = []

# 创建 SSH 对象
ssh = paramiko.SSHClient()
# 允许连接不在 know_hosts 文件中的主机
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

# 获取当前脚本所在的目录
current_dir = os.path.dirname(os.path.abspath(__file__))
# 构建 CameraCheck.py 的绝对路径
image_detection_script_path = os.path.join(current_dir, 'CameraCheck.py')
# 构建 LidarCheck.py 的绝对路径
LidarCheck_path = os.path.join(current_dir, 'LidarCheck.py')

def check_roslaunch_success(node_check):
    global ssh
    try:
        ssh.connect(hostname=host, port=port, username=username, password=password)
        # 先打印当前路径，再获取 rosnode list
        source_command = 'bash -ic "rosnode list"'
        stdin, stdout, stderr = ssh.exec_command(source_command)
        output = stdout.read().decode()
        error = stderr.read().decode()
        # if error:
        #     print(f"检查 ROS 节点时出错: {error}")
        #     return False
        # 输出中第一行是 pwd 的结果，后续行是 rosnode list 的结果
        running_nodes = output.strip().split('\n')
        if node_check not in running_nodes:
            print_colored_text(f"以下节点未成功启动: {node_check}", color="yellow", bold=True)
            return False
        else:
            print_colored_text(node_check + "已成功启动。", color="green", bold=True)
            return True
    except Exception as e:
        print_colored_text(f"检查 ROS 节点时出错: {e}", color="red", bold=True)
        return False

def ssh_send_command(host, port, username, password, command):
    global ssh

    try:
        # 连接服务器
        ssh.connect(hostname=host, port=port, username=username, password=password)
        # 执行命令
        stdin, stdout, stderr = ssh.exec_command(command)
        # 打印输出信息
        output = stdout.read().decode()
        error = stderr.read().decode()
        # if output:
        #     print("命令输出:", output)
        # if error:
        #     print("命令错误:", error)
        #     print("命令输出:", output)
        # 关闭连接
        ssh.close()
    except Exception as e:
        print(f"执行 SSH 命令时出错: {e}")


def main():
    global active_ssh_connections
    # 测试与上位机的连接
    print("1. 检测与上位机连接：")
    try:
        ssh.connect(hostname=host, port=port, username=username, password=password)
        print_colored_text("成功连接从(上位)机", color="green", bold=True)
        ssh.close()
    except paramiko.AuthenticationException:
        print_colored_text("认证失败，请检查用户名和密码", color="yellow", bold=True)
        return
    except paramiko.SSHException as e:
        print(f"SSH 连接错误: {str(e)}")
        return
    except Exception as e:
        print(f"发生其他错误: {str(e)}")
        return

    active_ssh_connections = start_all_services()

    # 测试音箱
    print("2. 检测触发音响服务：")
    play_music_node_check = "/play_music_node"
    if check_roslaunch_success(play_music_node_check):
        command = f'bash -ic "cd ~/kuavo_ros_application/ && rosservice call /play_music \'{{music_number: \'say_hello_sir.mp3\', volume: 80}}\'"'
        ssh_send_command(host, port, username, password, command)

        command = f'bash -ic "cd ~/kuavo_ros_application/ && rosservice call /play_music \'{{music_number: \'say_hello_sir\', volume: 80}}\'"'
        ssh_send_command(host, port, username, password, command)


    # 测试麦克风
    print("3. 检测触发麦克风服务：")
    record_music_node_check = "/record_music_node"
    if check_roslaunch_success(record_music_node_check):
        music_number = "999"
        time_out = 2
        # 构建服务调用命令
        command = f'bash -ic "cd ~/kuavo_ros_application/ && rosservice call /record_music \'{{music_number: \\\"{music_number}\\\", time_out: {time_out}}}\'"'
        ssh_send_command(host, port, username, password, command)


    # 测试相机
    print("4. 检测获取相机RBG和Depth图片信息：")
    try:
        # 调用本地图片检测脚本
        subprocess.run(['python3', image_detection_script_path])
    except Exception as e:
        print_colored_text(f"运行图片检测脚本时出错: {e}", color="yellow", bold=True)

    # 测试雷达数据
    print("5. 检测获取雷达点云信息：")
    try:
        # 调用本地雷达检测脚本
        subprocess.run(['python3', LidarCheck_path])
    except Exception as e:
        print_colored_text(f"运行雷达检测脚本时出错: {e}", color="yellow", bold=True)

    close_all_connections(active_ssh_connections)

if __name__ == "__main__":
    main()
    