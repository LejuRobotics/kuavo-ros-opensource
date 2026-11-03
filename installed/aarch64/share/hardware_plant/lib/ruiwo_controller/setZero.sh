#!/bin/bash
# 获取真实脚本路径
SCRIPT_PATH=$(readlink -f "$0")
SCRIPT_DIR=$(dirname "$SCRIPT_PATH")
cd "$SCRIPT_DIR"

export LD_LIBRARY_PATH=../python-can-3.3.4/can/interfaces/bmcan:$LD_LIBRARY_PATH
export PYTHONPATH=.:../python-can-3.3.4:$PYTHONPATH
python3 setZero.py
exit_code=$?
# 打印退出状态
echo "Exit code: $exit_code"
# 返回退出状态，可以在调用该脚本的地方捕获这个返回值
exit $exit_code
