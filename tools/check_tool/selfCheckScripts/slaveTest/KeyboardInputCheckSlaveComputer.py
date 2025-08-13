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

app_nodes_to_check = [
        "/apriltag_ros_continuous_node",
        "/ar_control_node",
        "/camera/realsense2_camera",
        "/camera/realsense2_camera_manager",
        "/camera_to_real_frame",
        # "/h12pro_channel_publisher",
        # "/humanoid_plan_arm_trajectory_node",
        "/joint_state_publisher",
        # "/joy_node",
        "/play_music_node",
        "/point_cloud_mask_node",
        "/realsense_yolo_segment_node",
        "/realsense_yolo_transform_torso_node",
        "/record_music_node",
        "/robot_state_publisher",
        "/rosout",
        "/rviz"
    ]

lidar_node_to_check = [
            "/apriltag_ros_continuous_node",
            "/camera/realsense2_camera",
            "/camera/realsense2_camera_manager",
            "/camera_to_real_frame",
            "/livox_lidar_publisher2",
            "/robot_state_publisher",
            "/rosout",
            "/rviz"
    ]

# 创建 SSH 对象
ssh = paramiko.SSHClient()
# 允许连接不在 know_hosts 文件中的主机
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

# 获取当前脚本所在的目录
current_dir = os.path.dirname(os.path.abspath(__file__))
# 构建 sshConnAndExec.py 的绝对路径
existing_script_path = os.path.join(current_dir, 'sshConnAndExec.py')
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
        missing_nodes = []
        for node in node_check:
            if node not in running_nodes:
                missing_nodes.append(node)
        if missing_nodes:
            print_colored_text(f"以下节点未成功启动: {', '.join(missing_nodes)}", color="yellow", bold=True)
            return False
        else:
            print_colored_text("kuavo_ros_application所有指定的 ROS 节点已成功启动。", color="green", bold=True)
            return True
    except Exception as e:
        print_colored_text(f"检查 ROS 节点时出错: {e}", color="red", bold=True)
        return False

def run_ros_launch_script():
    try:
        # 运行现有脚本
        result = subprocess.run(['python3', existing_script_path], capture_output=True, text=True)
        # 打印标准输出
        # if result.stdout:
        #     print("现有脚本标准输出:")
        #     print(result.stdout)
        # 打印标准错误输出
        if result.stderr:
            print("现有脚本标准错误输出:")
            print(result.stderr)
        if result.returncode == 0:
            print_colored_text("roslaunch 成功启动指定节点。", color="green", bold=True)
            return True
        else:
            print_colored_text("roslaunch 启动节点失败...", color="red", bold=True)
            return False
    except Exception as e:
        print(f"运行现有脚本时出错: {e}")
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
        if error:
            print("命令错误:", error)
            print("命令输出:", output)
        # 关闭连接
        ssh.close()
    except Exception as e:
        print(f"执行 SSH 命令时出错: {e}")

def main():
    #判断上位机程序启动了没有
    global active_ssh_connections

    # 运行现有脚本
    run_flag = True
    while run_flag:
        print("\n请选择要执行的操作:")
        print("1. 测试与上位机的连接")
        print("2. 测试音箱")
        print("3. 测试麦克风")
        print("4. 测试相机")
        print("5. 测试雷达数据")
        print("6. 查看kuavo_ros_application启动状态")
        print("7. 查看kuavo_ros_navigation启动状态")
        print("8. 启动kuavo_ros_application & kuavo_ros_navigation")
        print("9. 关闭kuavo_ros_application & kuavo_ros_navigation")

        print("0. 退出")
        choice = input("输入选项: ")
        if choice == '1':
            try:
                ssh.connect(hostname=host, port=port, username=username, password=password)
                print_colored_text("成功连接到服务器", color="green", bold=True)
                ssh.close()
            except paramiko.AuthenticationException:
                print_colored_text("认证失败，请检查用户名和密码", color="yellow", bold=True)
            except paramiko.SSHException as e:
                print(f"SSH 连接错误: {str(e)}")
            except Exception as e:
                print(f"发生其他错误: {str(e)}")
        elif choice == '2':
            command = f'bash -ic "source /opt/ros/humble/setup.bash && cd ~/kuavo_ros_application/ && rosservice call /play_music \'{{music_number: \'say_hello_sir.mp3\', volume: 100}}\'"'
            ssh_send_command(host, port, username, password, command)
        elif choice == '3':
            music_number = "999"
            time_out = 2
            # 构建服务调用命令
            command = f'bash -ic "cd ~/kuavo_ros_application/ && rosservice call /record_music \'{{music_number: \\\"{music_number}\\\", time_out: {time_out}}}\'"'
            ssh_send_command(host, port, username, password, command)
        elif choice == '4':
            try:
                # 调用本地图片检测脚本
                subprocess.run(['python3', image_detection_script_path])
            except Exception as e:
                print(f"运行图片检测脚本时出错: {e}")
        elif choice == '5':
            try:
                # 调用本地雷达检测脚本
                subprocess.run(['python3', LidarCheck_path])
            except Exception as e:
                print(f"运行图片检测脚本时出错: {e}")
        elif choice == '6':
            check_roslaunch_success(app_nodes_to_check)
        elif choice == '7':
            check_roslaunch_success(lidar_node_to_check)
        elif choice == '8':
            # run_ros_launch_script()
            # 启动所有服务
            active_ssh_connections = start_all_services()
        elif choice == '9':
            close_all_connections(active_ssh_connections)
        elif choice == '0':
            ssh.close()
            print("退出程序。")
            break
        else:
            print("无效的选项，请重新输入。")

if __name__ == "__main__":
    main()