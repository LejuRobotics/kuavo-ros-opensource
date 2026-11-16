#!/bin/bash
cd "$(dirname "$0")"

python3 hip_imu_test.py
exit_code=$?
# 打印退出状态
echo "Exit code: $exit_code"
# 返回退出状态，可以在调用该脚本的地方捕获这个返回值
exit $exit_code