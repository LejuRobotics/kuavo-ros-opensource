#!/usr/bin/env python3
import sys
sys.path.append("./")
from ssh_executor import SSHExecutor
import time

# 创建SSH执行器
executor = SSHExecutor("./remote-config.json")

# 连接到远程主机
if not executor.connect():
    print("❌ 连接失败")
    exit(1)

# executor.execute_predefined_command("xhost_on")
executor.execute_predefined_command("launch_waist_orbbec_camera_stereo_30hz")
