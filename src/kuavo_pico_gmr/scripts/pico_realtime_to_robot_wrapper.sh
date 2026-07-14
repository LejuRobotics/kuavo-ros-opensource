#!/bin/bash
# Wrapper script to run pico_realtime_to_robot.py in gmr_venv

# 获取脚本所在目录的绝对路径
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(dirname "$SCRIPT_DIR")"

echo "SCRIPT_DIR: $SCRIPT_DIR"
echo "PACKAGE_DIR: $PACKAGE_DIR"
echo "传入参数: $@"


# 激活虚拟环境
source "$PACKAGE_DIR/gmr_venv/bin/activate"

# 运行 Python 脚本，传递所有参数
exec python3 "$PACKAGE_DIR/scripts/pico_realtime_to_robot.py" "$@"
