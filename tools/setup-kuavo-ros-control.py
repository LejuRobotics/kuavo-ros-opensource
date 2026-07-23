import subprocess
import requests
import os
import openai
from datetime import datetime, timedelta
import shutil
import re
import serial
import serial.tools.list_ports
import autogen
import sys
from colorama import Fore, Style, init
from setup_kuavo_ros_control_remote import (
    FACTORY_URL,
    GITCODE_URL,
    GITEE_URL,
    KUAVO_ROS_ALLOWED_URLS,
    SOURCE_MODE_AUTO,
    SOURCE_MODE_FACTORY,
    SOURCE_MODE_GITCODE,
    SOURCE_MODE_GITEE,
    VERSION_REGISTRY,
    branch_exists,
    clone_sources,
    commit_exists_in_origin_factory,
    expected_urls_text,
    get_valid_display_versions,
    get_version_internal,
    is_allowed_remote,
    is_valid_version,
)

init(autoreset=True)

robot_version, repo_commit, weight, board_type, effector_choice, need_setup = None, None, None, None, None, None
source_mode = SOURCE_MODE_AUTO


def print_info(text):
  print(Fore.BLUE + Style.BRIGHT + "[INFO] " + text + Style.RESET_ALL)

def print_error(text):
  print(Fore.RED + Style.BRIGHT + "[ERROR] " + text + Style.RESET_ALL)

def print_success(text):
  print(Fore.GREEN + Style.BRIGHT + "[SUCCESS] " + text + Style.RESET_ALL)

def prompt_code_source_mode():
    global source_mode
    while True:
        print_info("请选择代码源:")
        print("1) 自动选择（推荐，优先 GitCode → 工厂镜像 → Gitee）")
        print("2) 仅工厂镜像")
        print("3) 仅 GitCode")
        print("4) 仅 Gitee")
        choice = input().strip()
        if choice in ("", "1"):
            source_mode = SOURCE_MODE_AUTO
            mode_text = "自动选择（优先 GitCode → 工厂镜像 → Gitee）"
            break
        if choice == "2":
            source_mode = SOURCE_MODE_FACTORY
            mode_text = "仅工厂镜像"
            break
        if choice == "3":
            source_mode = SOURCE_MODE_GITCODE
            mode_text = "仅 GitCode"
            break
        if choice == "4":
            source_mode = SOURCE_MODE_GITEE
            mode_text = "仅 Gitee"
            break
        print_error("无效的代码源选项，请输入 1、2、3 或 4")
    print_info(f"当前代码源策略: {mode_text}")


def ask_user_for_kinds_of_settings():
    global robot_version, repo_commit, weight, board_type, effector_choice, need_setup, branch

    valid_versions = get_valid_display_versions()
    while True:
        robot_version = input(f"请输入机器人版本 ({'/'.join(valid_versions)}): ")
        if is_valid_version(robot_version):
            robot_version = str(get_version_internal(robot_version))
            break
        else:
            print_error("无效的机器人版本，请重新输入。")

    while True:
        try:
            weight = float(input("请输入机器人重量: "))
            break
        except ValueError:
            print_error("无效的机器人重量，请输入一个数字。")

    while True:
        branch = input("请输入代码仓分支（回车默认master）: ")
        if branch:
            break
        else:
            branch = "master"
            break

    while True:
        repo_commit = input("请输入代码仓commit(回车默认当前分支最新commit): ")
        if repo_commit:
            break
        else:
            repo_commit = "latest"
            break

    prompt_code_source_mode()

    while True:
        board_type = input("请输入驱动板类型 (elmo/youda): ").lower()
        if board_type in ["elmo", "youda"]:
            break
        else:
            print_error("无效的驱动板类型，请重新输入。")

    while True:
        effector_choice = input("请输入末端执行器类型 (1: 灵巧手, 2: 二指夹爪): ")
        if effector_choice in ["1", "2"]:
            effector_choice = int(effector_choice)
            break
        else:
            print_error("无效的末端执行器类型，请重新输入。")

    need_setup = input("请输入是否需要配置H12PRO遥控器(y/N): ").lower() == "y"

ask_user_for_kinds_of_settings()

KUAVO_ROS_OPENSOURCE_REPO_URL = GITCODE_URL
KUAVO_OPENSOURCE_REPO_URL = "https://gitee.com/leju-robot/kuavo_opensource.git"

system_message = """
你是一个机器人程序出厂的助手。

我会提供以下功能函数，请你按照我的指示完成任务。

1. setup_pip(): 用于设置 PIP 镜像源。

2. clone_repos(): 用于克隆/更新 kuavo-ros-control 代码仓。

3. setup_robot_version(): 用于设置机器人版本。

4. setup_robot_weight(): 用于设置机器人重量。

5. setup_drive_board(): 用于设置驱动板类型。

6. setup_arm_motor(): 用于配置手臂电机配置文件。

7. setup_end_effector(): 用于配置末端执行器。

8. install_vr_deps(): 用于安装VR相关依赖。

9. build_project(): 用于编译 KUAVO-ROS-CONTROL 项目。

10. setup_h12pro(): 用于配置H12PRO遥控器。

11. cleanup_code(): 用于清理闭源代码。

所有函数返回值均为布尔值，如果函数调用失败，会自动抛出异常
如果在函数调用的时候有任何报错，请按照以下格式输出：

```
发生错误的环节：<发生错误的环节>
错误信息：<错误信息>
错误日志：<错误日志>(需要你提取出重要关键的错误日志)
修复建议：<修复建议>
```
并且停止后续任务, 输出"TERMINATE"，直接结束对话。

如果任务完成，请输出：

KUAVO-ROS-CONTROL 出厂任务完成 TERMINATE 结束对话。

"""

llm_config = {
    "config_list": [
        {
            "model": "gpt-4o",  # modify to your model
            "api_key": os.environ["OPENAI_API_KEY"],  # modify to your openai api key
            "base_url": os.environ["OPENAI_BASE_URL"],  # modify to your openai base url
        }
    ],
    "timeout": 120,
}

chatbot = autogen.AssistantAgent(
    name="chatbot",
    system_message=system_message,
    llm_config=llm_config,
)

user_proxy = autogen.UserProxyAgent(
    name="user_proxy",
    human_input_mode="NEVER",
    is_termination_msg=lambda x: x.get("content", "")
    and x.get("content", "").rstrip().endswith("TERMINATE"),
    max_consecutive_auto_reply=10,
)


def run_command(command, check=True, shell=False):
    """Runs a command and returns the output.  Raises an exception if it fails."""
    try:
        if shell:
            process = subprocess.Popen(
                command,
                shell=True,
                executable="/bin/bash",
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,  # 合并标准错误到标准输出
                bufsize=1,
                universal_newlines=True
            )
        else:
            process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,  # 合并标准错误到标准输出
                bufsize=1,
                universal_newlines=True
            )

        stdout_lines = []
        # 实时输出日志
        for line in iter(process.stdout.readline, ''):
            sys.stdout.write(Fore.CYAN + line)  # 用青色显示实时日志
            sys.stdout.flush()
            stdout_lines.append(line)

        process.stdout.close()
        return_code = process.wait()

        stdout = ''.join(stdout_lines).strip()
        stderr = ''  # 已合并到stdout

        if check and return_code != 0:
            print_error(f"Command failed: {' '.join(command)}")
            print_error(f"Exit code: {return_code}")
            print_error(f"Output: {stdout}")
            raise subprocess.CalledProcessError(return_code, command, output=stdout, stderr=stdout)

        return stdout, stderr, return_code

    except FileNotFoundError as e:
        print_error(f"Command not found: {e.filename}")
        raise
    except Exception as e:
        print_error(f"An unexpected error occurred: {e}")
        raise

@user_proxy.register_for_execution()
@chatbot.register_for_llm(
    description="配置PIP镜像源",
)
def setup_pip() -> bool:
    print_info("配置PIP镜像源...")
    commands = [
        ["pip", "config", "set", "global.index-url", "https://pypi.tuna.tsinghua.edu.cn/simple"],
        ["pip", "config", "set", "global.trusted-host", "pypi.tuna.tsinghua.edu.cn"],
        ["pip", "config", "set", "install.trusted-host", "pypi.tuna.tsinghua.edu.cn"],
    ]
    for command in commands:
        run_command(command)
    print_success("PIP配置完成。")
    return True

@user_proxy.register_for_execution()
@chatbot.register_for_llm(
    description="克隆/更新KUAVO-ROS-OPENSOURCE代码仓库",
)
def clone_repos() -> bool:
    global repo_commit, branch
    print_info("克隆/更新KUAVO-ROS-OPENSOURCE代码仓库...")
    home_dir = os.path.expanduser("~")
    os.chdir(home_dir)
    need_clone = True

    def validate_branch(branch_name):
        last_error = None
        for repo_url in clone_sources(source_mode):
            try:
                branches = subprocess.check_output(
                    ["git", "ls-remote", "--heads", repo_url],
                    stderr=subprocess.STDOUT,
                    text=True,
                )
            except subprocess.CalledProcessError as e:
                last_error = e
                print_info(f"无法从 {repo_url} 验证分支，继续尝试其他远端")
                continue
            if branch_exists(branches, branch_name):
                return True

        if last_error is not None:
            output = (last_error.output or "").strip()
            raise Exception(f"无法验证分支: {output}")
        return False

    # 验证kuavo-ros-opensource分支
    if not validate_branch(branch):
        print_error(f"分支 {branch} 在所有代码源中都不存在")
        raise ValueError("无效的分支名称")

    # kuavo-ros-opensource仓库操作
    if os.path.isdir("kuavo-ros-opensource") and os.path.isdir("kuavo-ros-opensource/.git"):
        os.chdir("kuavo-ros-opensource")
        remote_url, _, _ = run_command(["git", "remote", "get-url", "origin"])
        if is_allowed_remote(remote_url, KUAVO_ROS_ALLOWED_URLS):
            print_info("代码仓已存在，更新代码仓...")
            run_command(["git", "reset", "--hard", "HEAD"])
            run_command(["git", "clean", "-fd"])
            need_clone = False
        else:
            print("")
            print("==========================================")
            print("  WARNING: kuavo-ros-opensource 远程 URL 不匹配")
            print("==========================================")
            print(f"  合法 URL: {expected_urls_text(KUAVO_ROS_ALLOWED_URLS)}")
            print(f"  实际 URL: {remote_url}")
            print("")
            print("  选择 yes 将删除当前目录并优先从工厂镜像重新克隆。")
            print("  选择 no 将中止部署。")
            print("")
            user_choice = input("是否继续？(yes/no): ").strip().lower()
            if user_choice == "yes":
                print_info("用户确认，删除并重新克隆...")
                os.chdir(home_dir)
                shutil.rmtree("kuavo-ros-opensource")
                need_clone = True
            else:
                print_error("用户取消，中止部署。")
                sys.exit(1)

    if need_clone:
        clone_error = None
        for source_url in clone_sources(source_mode):
            try:
                print_info(f"尝试从 {source_url} 克隆 kuavo-ros-opensource...")
                run_command(["git", "clone", source_url, "--branch", branch])
                clone_error = None
                break
            except subprocess.CalledProcessError as e:
                clone_error = e
                print_info(f"从 {source_url} 克隆失败，尝试下一个远端")
        if clone_error is not None:
            raise clone_error
        os.chdir("kuavo-ros-opensource")

    track_origin_factory = source_mode != SOURCE_MODE_GITEE
    if track_origin_factory:
        origin_factory_url, _, origin_factory_code = run_command(
            ["git", "remote", "get-url", "origin_factory"],
            check=False,
        )
        if origin_factory_code != 0:
            run_command(["git", "remote", "add", "origin_factory", FACTORY_URL])
        fetch_output, _, fetch_code = run_command(["git", "fetch", "origin_factory"], check=False)
        if fetch_code != 0:
            print_info(f"origin_factory fetch 失败，使用已有数据继续: {fetch_output}")

    run_command(["git", "checkout", branch])
    if repo_commit != "latest":
        run_command(["git", "checkout", repo_commit])
        if track_origin_factory:
            contains_output, _, _ = run_command(
                ["git", "branch", "-r", "--contains", repo_commit],
                check=False,
            )
            if not commit_exists_in_origin_factory(contains_output):
                print("")
                print("==========================================")
                print(f"  WARNING: commit {repo_commit} 在 origin_factory 远端不存在")
                print("==========================================")
                print("  请联系 IT 将最新代码同步到工厂服务器 (10.11.99.175)")
                print("")
    else:
        run_command(["git", "pull"])

    # kuavo_opensource
    os.chdir(home_dir)
    if os.path.isdir("kuavo_opensource") and os.path.isdir("kuavo_opensource/.git"):
        os.chdir("kuavo_opensource")
        remote_url = run_command(["git", "remote", "get-url", "origin"])
        if KUAVO_OPENSOURCE_REPO_URL in remote_url:
            print_info("kuavo_opensource目录存在且远程仓库匹配，清理工作区...")
            run_command(["git", "reset", "--hard", "HEAD"])
            run_command(["git", "clean", "-fd"])
            run_command(["git", "pull", "origin", "master"])
        else:
            print("")
            print("==========================================")
            print("  WARNING: kuavo_opensource 远程 URL 不匹配")
            print("==========================================")
            print(f"  期望 URL: {KUAVO_OPENSOURCE_REPO_URL}")
            print(f"  实际 URL: {remote_url}")
            print("")
            print("  选择 yes 将删除当前目录并从期望 URL 重新克隆。")
            print("  选择 no 将中止部署。")
            print("")
            user_choice = input("是否继续？(yes/no): ").strip().lower()
            if user_choice == "yes":
                print_info("用户确认，删除并重新克隆...")
                os.chdir(home_dir)
                shutil.rmtree("kuavo_opensource")
                run_command(["git", "clone", KUAVO_OPENSOURCE_REPO_URL, "--branch", "master", "--depth", "1"])
            else:
                print_error("用户取消，中止部署。")
                sys.exit(1)
    else:
        run_command(["git", "clone", KUAVO_OPENSOURCE_REPO_URL, "--branch", "master", "--depth", "1"])

    print_success("代码仓克隆/更新成功。")
    return True

@user_proxy.register_for_execution()
@chatbot.register_for_llm(
    description="设置机器人版本",
)
def setup_robot_version() -> bool:
    """设置机器人版本，并更新用户和 root 的 .bashrc 文件。"""
    global robot_version
    print_info("设置机器人版本...")

    export_line = f"export ROBOT_VERSION={robot_version}"

    # 更新用户的 .bashrc
    bashrc_path = os.path.expanduser("~/.bashrc")
    try:
        with open(bashrc_path, "r") as f:
            content = f.read()

        # 使用正则表达式全局替换
        new_content = re.sub(r"^export ROBOT_VERSION=.*$", 
                            export_line, 
                            content, 
                            flags=re.MULTILINE)

        # 如果没有匹配则追加
        if new_content == content:
            new_content += f"\n{export_line}\n"

        with open(bashrc_path, "w") as f:
            f.write(new_content)

        print_success(f"机器人版本已设置为 {robot_version} (用户)")

    except IOError as e:
        raise Exception(f"无法更新用户的 .bashrc 文件: {e}")

    # 更新 root 的 .bashrc
    root_bashrc_path = "/root/.bashrc"
    try:
        # 使用sed命令全局替换
        cmd = f"sudo sed -i 's/^export ROBOT_VERSION=.*/{export_line}/g' {root_bashrc_path}"
        subprocess.run(cmd, shell=True, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        
        # 检查是否已存在，不存在则追加
        check_cmd = f"sudo grep -q '^{export_line}$' {root_bashrc_path} || sudo sh -c 'echo \"{export_line}\" >> {root_bashrc_path}'"
        subprocess.run(check_cmd, shell=True, check=True)

        print_success(f"机器人版本已设置为 {robot_version} (root)")

    except subprocess.CalledProcessError as e:
        raise Exception(f"更新 root 的 .bashrc 文件时发生错误: {e.stderr.decode()}")
    
    return True

@user_proxy.register_for_execution()
@chatbot.register_for_llm(
    description="设置机器人重量",
)
def setup_robot_weight() -> bool:
    """设置机器人重量，并写入到配置文件。"""
    global robot_version  # 声明使用全局变量 robot_version
    global weight

    print_info("设置机器人重量...")

    config_dir = os.path.expanduser("~/.config/lejuconfig")
    config_file = os.path.join(config_dir, f"TotalMassV{robot_version}")  # 使用全局变量 robot_version

    try:
        # 确保目录存在
        os.makedirs(config_dir, exist_ok=True)

        # 删除旧文件（如果存在）
        if os.path.exists(config_file):
            os.remove(config_file)

        # 写入新的重量
        with open(config_file, "w") as f:
            f.write(str(weight))  # 使用全局变量 weight

        print_success(f"机器人重量设置为 {weight}kg")  # 使用全局变量 weight

    except OSError as e:
        raise Exception(f"设置机器人重量时发生错误: {e}")
    
    return True


@user_proxy.register_for_execution()
@chatbot.register_for_llm(
    description="设置驱动板类型, 只支持 elmo 或 youda",
)
def setup_drive_board() -> bool:
    global board_type
    print_info("设置驱动板类型...")

    if board_type not in ["elmo", "youda"]:
        raise Exception("驱动板类型只支持 elmo 或 youda")

    home_dir = os.path.expanduser("~")
    lejuconfig_dir = os.path.join(home_dir, ".config", "lejuconfig")
    ecmaster_file = os.path.join(lejuconfig_dir, "EcMasterType.ini")

    os.makedirs(lejuconfig_dir, exist_ok=True)
    try:
        with open(ecmaster_file, "w") as f:
            f.write(board_type + "\n")
    except IOError as e:
        raise Exception(f"Error writing to {ecmaster_file}: {e}")

    print_success(f"驱动板类型设置为 {board_type}")
    return True

@user_proxy.register_for_execution()
@chatbot.register_for_llm(
    description="配置手臂电机配置文件",
)
def setup_arm_motor() -> bool:
    print_info("配置手臂电机配置文件")

    # Find config.yaml in ruiwo_controller directory
    home_dir = os.path.expanduser("~")
    kuavo_ros_opensource_dir = os.path.join(home_dir, "kuavo-ros-opensource")
    try:
        stdout, stderr, returncode = run_command(["find", kuavo_ros_opensource_dir, "-type", "f", "-path", "*/ruiwo_controller/config.yaml"], check=False)
        src_config = stdout.splitlines()[0] if stdout else None

        if not src_config:
            print_error("未找到 ruiwo_controller/config.yaml 文件")
            return

        print_info(f"找到配置文件: {src_config}")

        # Destination config directory and file
        dest_dir = os.path.join(home_dir, ".config", "lejuconfig")
        dest_config = os.path.join(dest_dir, "config.yaml")

        # Create destination directory if it doesn't exist
        os.makedirs(dest_dir, exist_ok=True)

        # Copy the config file
        run_command(["cp", src_config, dest_config])

        print_success("手臂电机配置文件配置完成")
    except subprocess.CalledProcessError as e:
        print_error(f"An error occurred during arm motor setup: {e}")

    return True

@user_proxy.register_for_execution()
@chatbot.register_for_llm(
    description="""配置末端执行器
    """
)
def setup_end_effector() -> bool:
    print_info("配置末端执行器...")
    home_dir = os.path.expanduser("~")
    global effector_choice
    if effector_choice == 1:
        print_info("配置灵巧手USB设备...")
        device_list = []
        folder_path = os.path.join(home_dir, "kuavo_opensource", "tools", "check_tool")
        # 获取所有连接的串口设备
        ports = list(serial.tools.list_ports.comports())
        for port in ports:
            if(port.description == "LJ485A - LJ485A"):
                hwid_string = port.hwid
                # 编写正则表达式
                pattern = re.compile(r"SER=([0-9A-Z]+)")
                # 使用findall方法查找所有匹配项，这将返回一个包含所有匹配结果的列表
                matches = pattern.findall(hwid_string)
                # 输出SER值
                for match in matches:
                    print("串口：", port.device,"SER", match)  # 输出: DB81ASSB

                    device_list.append(port.device)

            if(port.description == "LJ485B - LJ485B"):
                hwid_string = port.hwid
                # 编写正则表达式
                pattern = re.compile(r"SER=([0-9A-Z]+)")
                # 使用findall方法查找所有匹配项，这将返回一个包含所有匹配结果的列表
                matches = pattern.findall(hwid_string)
                # 输出SER值
                for match in matches:
                    print("串口：", port.device,"SER", match)  # 输出: DB81ASSB

                    device_list.append(port.device)

        swap_str = input("是否交换左右手(no/yes)：")
        swap_flag = 0
        if(swap_str[0].lower() == 'y'):
            swap_flag = 1
        if len(device_list) == 2:
            # 定义脚本路径和参数
            if(swap_flag):
                arg1 = device_list[0]
            else:
                arg1 = device_list[1]
            arg2 = "stark_serial_R"
            # 定义要运行的命令
            command = "sudo bash "+ folder_path +"/generate_serial.sh "  +  arg1  + " " + arg2
            run_command(command, shell=True)
            
            if(swap_flag):
                arg1 = device_list[1]
            else:
                arg1 = device_list[0]
            arg2 = "stark_serial_L"
            # 定义要运行的命令
            command = "sudo bash "+ folder_path +"/generate_serial.sh "  +  arg1  + " " + arg2
            run_command(command, shell=True)
        else:
            raise Exception("失败，kuavo电源板485设备连接异常")

        print_success("灵巧手USB设备配置完成")

    elif effector_choice == 2:
        robot_version = os.environ.get("ROBOT_VERSION")
        if not robot_version:
            print_error("ROBOT_VERSION environment variable not set.")
            return

        kuavo_json_path = os.path.join(home_dir, "kuavo-ros-opensource", "src", "kuavo_assets", "config", f"kuavo_v{robot_version}", "kuavo.json")
        try:
            run_command(["sed", "-i", r's/"EndEffectorType": \[.*\]/"EndEffectorType": ["lejuclaw", "lejuclaw"]/', kuavo_json_path])
            print_success("二指夹爪配置完成")
        except subprocess.CalledProcessError as e:
            print_error(f"Error modifying kuavo.json: {e}")
    return True

@user_proxy.register_for_execution()
@chatbot.register_for_llm(
    description="用于安装VR相关依赖",
)
def install_vr_deps() -> bool:
    print_info("安装VR相关依赖...")
    home_dir = os.path.expanduser("~")
    os.chdir(os.path.join(home_dir, "kuavo-ros-opensource"))
    run_command(["pip3", "install", "-r", "src/manipulation_nodes/noitom_hi5_hand_udp_python/requirements.txt"])
    run_command(["sudo", "pip3", "install", "-r", "src/manipulation_nodes/noitom_hi5_hand_udp_python/requirements.txt"])
    print_success("VR依赖安装完成")
    return True

@user_proxy.register_for_execution()
@chatbot.register_for_llm(
    description="用于编译 KUAVO 项目",
)
def build_project() -> bool:
    print_info("编译 KUAVO 项目...")
    home_dir = os.path.expanduser("~")
    os.chdir(os.path.join(home_dir, "kuavo-ros-opensource"))

    robot_version = os.environ.get("ROBOT_VERSION")
    if not robot_version:
        print_error("ROBOT_VERSION environment variable not set.")
        return

    os.environ["ROBOT_VERSION"] = robot_version

    # 先清理
    run_command(["sudo", "-E", "su", "-c", "catkin clean -y || true"])

    # 配置并编译
    run_command(["sudo", "-E", "su", "-c", "catkin config -DCMAKE_ASM_COMPILER=/usr/bin/as -DCMAKE_BUILD_TYPE=Release"])
    run_command(["sudo", "-E", "su", "-c", "source installed/setup.bash && catkin build humanoid_controllers"])

    print_success("项目编译完成")
    return True

@user_proxy.register_for_execution()
@chatbot.register_for_llm(
    description="用于配置H12PRO遥控器, 需要输入y或N",
)
def setup_h12pro() -> bool:
    global need_setup
    if not need_setup:
        print_info("不配置H12PRO遥控器")
        return True

    print_info("配置H12PRO遥控器...")
    home_dir = os.path.expanduser("~")
    os.chdir(os.path.join(home_dir, "kuavo-ros-opensource", "src", "humanoid-control", "h12pro_controller_node", "scripts"))
    run_command(["sudo", "-E", "su", "-c", "./deploy_autostart.sh"])
    print_success("H12PRO遥控器配置完成")
    return True


@user_proxy.register_for_execution()
@chatbot.register_for_llm(
    description="用于清理闭源代码",
)
def cleanup_code() -> bool:
    print_info("清理闭源代码...")
    home_dir = os.path.expanduser("~")

    try:
        stdout, stderr, returncode = run_command(["find", home_dir, "-type", "d", "-name", "*kuavo*"], check=False)
        for dir in stdout.splitlines():
            if os.path.isdir(os.path.join(dir, ".git")):
                try:
                    remote_url, _, _ = run_command(["git", "-C", dir, "remote", "get-url", "origin"], check=False)

                    if (remote_url == "ssh://git@www.lejuhub.com:10026/highlydynamic/kuavo.git" or
                        remote_url == "https://www.lejuhub.com/highlydynamic/kuavo.git" or
                        remote_url == "ssh://git@www.lejuhub.com:10026/highlydynamic/kuavo-ros-control.git" or
                        remote_url == "https://www.lejuhub.com/highlydynamic/kuavo-ros-control.git"):
                        print_info(f"Deleting: {dir}")
                        run_command(["sudo", "rm", "-rf", dir])

                except subprocess.CalledProcessError as e:
                    # Ignore errors getting remote URL, directory might not be a git repo
                    pass
    except subprocess.CalledProcessError as e:
        print_error(f"Error finding directories: {e}")

    print_success("闭源代码清理完成")
    return True


res = user_proxy.initiate_chat(
    chatbot,
    message="""你是一个机器人出厂的助手, 请按照以上步骤顺序完成任务。

    1. 配置PIP镜像源。
    2. 克隆/更新KUAVO-ROS-CONTROL代码仓库。
    3. 配置机器人版本。
    4. 配置机器人重量。
    5. 配置驱动板类型。
    6. 配置手臂电机配置文件。
    7. 配置末端执行器。
    8. 安装VR相关依赖。
    9. 编译KUAVO-ROS-CONTROL项目。
    10. 配置H12PRO遥控器。
    11. 清理闭源代码。
    
    请按照以上步骤顺序完成任务, 每个任务对应一个函数调用，如果任务失败了，后续任务不能执行，并且请输出"TERMINATE"，直接结束对话。
    """,
    summary_method="reflection_with_llm",
)
