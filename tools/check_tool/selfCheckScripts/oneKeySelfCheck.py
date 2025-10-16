import subprocess
import sys
import threading
import signal
import os
import rospy
import select
from common.common_utils import print_colored_text

roslaunch_running = False

h12_msg_valid = False
h12_msg = None
current_process = None


# 获取当前脚本所在的目录
current_dir = os.path.dirname(os.path.abspath(__file__))
# 构建 LidarCheck.py 的绝对路径
OneKeyCheckSalveComputer_path = os.path.join(current_dir, 'slaveTest', 'OneKeyCheckSlaveComputer.py')

# 反推kuavo-ros-control的路径
kuavo_ros_control_path = os.path.abspath(os.path.join(current_dir, "../../.."))
# 打印kuavo-ros-control的路径
print(f"kuavo-ros-control的路径为: {kuavo_ros_control_path}")

# 获取当前Python脚本所在目录
shell_script_path = os.path.join(current_dir, "compiling_dependent_lib.sh")

def h12_topic_callback(msg):
    global h12_msg
    global h12_msg_valid
    if h12_msg:
        h12_msg = msg
        channels = msg.channels
        has_non_zero = any(channel != 0 for channel in channels)
        if has_non_zero:
            h12_msg_valid = True
        # else:
            # print(f"收到 /h12pro_channel 话题消息，但消息中的数据全为 0: {msg}")

def h12_remote_control_detection():
    global h12_msg
    global h12_msg_valid

    workspace_path  = "../../../devel/lib/python3/dist-packages"
    if workspace_path not in sys.path:
        sys.path.append(workspace_path)

    # 检查并赋予执行权限（关键修复）
    if not os.access(shell_script_path, os.X_OK):
        os.chmod(shell_script_path, 0o755)
        # print(f"已为 {shell_script_path} 添加执行权限")

    try:
        result = os.system(shell_script_path)
        if result != 0:
            print(f"Error occurred while running the shell script. Return code: {result}")
        from h12pro_controller_node.msg import h12proRemoteControllerChannel
        print("成功导入消息模块")
    except ImportError as e:
        print(f"导入模块时出错: {e}")
        return

    rospy.Subscriber('/h12pro_channel', h12proRemoteControllerChannel, h12_topic_callback)
    start_time = rospy.Time.now()
    h12_msg = True
    while not rospy.is_shutdown() and (rospy.Time.now() - start_time).to_sec() < 0.5:
        rospy.rostime.wallsleep(0.1)
    h12_msg = False
    if h12_msg_valid:
        print_colored_text(f"收到 /h12pro_channel 话题消息成功", color="green", bold=True)
    else:
        print_colored_text(f"收到 /h12pro_channel 话题消息异常", color="red", bold=True)

def signal_handler(sig, frame):
    """处理Ctrl+C信号"""
    global current_process, roslaunch_running
    print("\n收到Ctrl+C, 正在关闭程序...")
    if current_process and current_process.poll() is None:
        # 发送退出命令
        if current_process.stdin:
            current_process.stdin.write('x\n')  # 发送退出命令
            current_process.stdin.flush()
        current_process.terminate()  # 发送SIGTERM
        try:
            current_process.wait(timeout=3)  # 等待3秒
        except subprocess.TimeoutExpired:
            current_process.kill()  # 强制终止
    roslaunch_running = False
    rospy.signal_shutdown("Received Ctrl+C")
    sys.exit(0)

def handle_hardware_self_check():
    """处理硬件自检选择"""
    try:
        from hardwareTest.hardwareSelfCheck import run_hardware_self_check
        run_hardware_self_check()
    except ImportError as e:
        print_colored_text(f"导入硬件自检模块失败: {e}", color="red", bold=True)
        print_colored_text("请确保 hardwareTest/hardwareSelfCheck.py 文件存在", color="yellow")
    except Exception as e:
        print_colored_text(f"硬件自检执行失败: {e}", color="red", bold=True)

def handle_slave_computer_check():
    """处理上位机检测"""
    subprocess.run(['python3', OneKeyCheckSalveComputer_path])

def handle_h12_detection():
    """处理H12遥控器信号检测"""
    h12_remote_control_detection()

def run_motor_follow_test_with_region(test_region="full"):
    """运行电机跟随性测试的通用函数"""
    try:
        # 动态添加motorTest目录到Python路径
        motor_test_dir = os.path.join(current_dir, 'motorTest')
        if motor_test_dir not in sys.path:
            sys.path.insert(0, motor_test_dir)
        
        from motorFollowTest import MotorFollowTest
        tester = MotorFollowTest(test_region=test_region)
        tester.run_motor_follow_test()
    except ImportError as e:
        print_colored_text(f"导入电机跟随性测试模块失败: {e}", color="red", bold=True)
        print_colored_text("请确保 motorTest/motorFollowTest.py 文件存在", color="yellow")
    except Exception as e:
        print_colored_text(f"电机跟随性测试执行失败: {e}", color="red", bold=True)

def handle_motor_follow_test():
    """处理电机跟随性测试"""
    # 测试区域配置
    regions = {
        '1': ("upper", "上半身"),
        '2': ("lower", "下半身"), 
        '3': ("full", "全身")
    }
    
    print("\n请选择电机跟随性测试区域:")
    print("1. 上半身测试 (7对电机: 手臂)")
    print("2. 下半身测试 (6对电机: 腿部)")
    print("3. 全身测试 (13对电机: 除头部以外全身)")
    
    try:
        choice = input("请选择测试区域 (1/2/3): ").strip()
        test_region, region_name = regions.get(choice, ("full", "全身"))
        
        if choice not in regions:
            print_colored_text("无效选择，使用默认全身测试", color="yellow")
        
        print_colored_text(f"开始{region_name}电机跟随性测试...", color="green", bold=True)
        run_motor_follow_test_with_region(test_region)
        
    except KeyboardInterrupt:
        print_colored_text("\n用户取消选择", color="yellow")
    except Exception as e:
        print_colored_text(f"选择过程出错: {e}", color="red")
        run_motor_follow_test_with_region("full")

def handle_readme_display():
    """处理README显示"""
    readme_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "README.md")
    if os.path.exists(readme_path):
        with open(readme_path, 'r', encoding='utf-8') as f:
            print("\n===== 自检脚本使用说明 (README) =====\n")
            print(f.read())
            print("\n===== 按回车键返回主菜单 =====\n")
            input()
    else:
        print("未找到README.md文件！")

def display_main_menu():
    """显示主菜单"""
    print("\n请选择要执行的操作:")
    print("1. 下位机检测")
    print("2. 上位机检测")
    print("3. h12遥控器信号检测")
    print("4. 电机跟随性测试")
    print("5. 查看自检说明(README)")
    print("0. 退出")

def handle_user_choice(choice):
    """处理用户选择"""
    if choice == '1':
        handle_hardware_self_check()
    elif choice == '2':
        handle_slave_computer_check()
    elif choice == '3':
        handle_h12_detection()
    elif choice == '4':
        handle_motor_follow_test()
    elif choice == '5':
        handle_readme_display()
    elif choice == '0':
        print("退出程序。")
        return False
    else:
        print_colored_text(f"无效的选项，请重新输入...", color="yellow", bold=True)
    return True

if __name__ == "__main__":
    run_flag = True
    rospy.init_node('main_node', anonymous=True)
    while run_flag:
        if not roslaunch_running:
            display_main_menu()
            try:
                choice = input("输入选项: ")
                run_flag = handle_user_choice(choice)
            except KeyboardInterrupt:
                print("\n收到Ctrl+C，退出程序。")
                run_flag = False
            except Exception as e:
                print_colored_text(f"发生错误: {e}", color="red", bold=True)
                print("返回主菜单...")
