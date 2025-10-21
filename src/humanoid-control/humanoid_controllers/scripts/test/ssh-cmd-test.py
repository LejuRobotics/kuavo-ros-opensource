#!/usr/bin/env python3
import sys
sys.path.append("../")
from ssh_executor import SSHExecutor
import time

# 创建SSH执行器
executor = SSHExecutor("../remote-config.json")

# 连接到远程主机
if not executor.connect():
    print("❌ 连接失败")
    exit(1)
container_name = "kuavo_gendexgrasp_staircase_estimation_dev"

# executor.execute_predefined_command("xhost_on")
executor.execute_predefined_command("launch_orbbec_camera")


# executor.execute_predefined_container_command_async("run_staircase_detection", container_name)
# executor.execute_predefined_container_command_async("start_staircase_detection", container_name)

print("异步指令开始执行")
time.sleep(50)
executor.execute_predefined_container_command("kill_staircase_detection", container_name)
executor.disconnect()
