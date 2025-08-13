# logger_setup.py

import sys
import os
from datetime import datetime

class Logger:
    def __init__(self, filename):
        self.terminal = sys.stdout
        self.log = open(filename, "w", encoding='utf-8')

    def write(self, message):
        self.terminal.write(message)
        self.log.write(message)

    def flush(self):
        self.terminal.flush()
        self.log.flush()

def init_logging(log_dir="logs", filename_prefix="log", enable=True):
    """
    初始化日志系统，将 stdout 和 stderr 同时写入文件和终端。

    参数:
        log_dir: 日志目录，相对于当前工作目录
        filename_prefix: 日志文件名前缀
        enable: 是否启用日志重定向
    返回:
        日志文件的完整路径
    """
    if not enable:
        return None

    os.makedirs(log_dir, exist_ok=True)
    now = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    log_path = os.path.join(log_dir, f"{filename_prefix}_{now}.log")
    sys.stdout = Logger(log_path)
    sys.stderr = sys.stdout
    print(f"[日志初始化] 所有输出将记录到: {log_path}")
    return log_path
