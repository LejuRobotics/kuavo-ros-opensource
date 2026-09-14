#!/bin/bash
# 仅当压力传感器设备存在时才启动 modbus_io_bridge_node。
# 设备不存在（非气泵机型）时直接退出，避免在整机运行中，反复告警无法打开端口。
#
# 用法（由 roslaunch 调用）：
#   start_modbus_io_bridge.sh [设备路径] [__name:=xxx] [__log:=xxx] ...
#   默认设备路径为 /dev/kuavo_pressure。

if [ -n "$1" ] && [ "${1#__}" = "$1" ]; then
    PRESSURE_PORT="$1"
    shift
else
    PRESSURE_PORT="/dev/kuavo_pressure"
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NODE_SCRIPT="$SCRIPT_DIR/modbus_io_bridge_node.py"

# 检测设备是否存在（含符号链接目标）
if [ -e "$PRESSURE_PORT" ] || [ -L "$PRESSURE_PORT" ]; then
    exec "$NODE_SCRIPT" "$@"
fi

# 设备不存在：打印提示后跳过，不启动节点
echo "[modbus_io_bridge] 未检测到压力传感器设备 $PRESSURE_PORT，跳过启动 modbus_io_bridge_node（非气泵机型）"
exit 0
