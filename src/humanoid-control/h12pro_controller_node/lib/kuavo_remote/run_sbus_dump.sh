#!/usr/bin/env bash
# ============================================================
# 一键编译并运行 SBUS 底层数据打印工具（不依赖 ROS/msg）
#
# 用法:
#   ./run_sbus_dump.sh              # 编译 + sudo 运行(自动输入密码? 否,需手动)
#   ./run_sbus_dump.sh --build-only # 只编译不运行
#   ./run_sbus_dump.sh --print-all  # 打印全部通道(默认只打印变化的)
#
# 依赖: gcc, 遥控器接收器 USB 已插入并生成 /dev/usb_remote
#       (usb_remote.rules 已装时插入自动建链; 否则先 sudo cp usb_remote.rules
#        /etc/udev/rules.d/ && sudo udevadm control --reload)
# ============================================================
set -u
cd "$(dirname "${BASH_SOURCE[0]}")"

BUILD_ONLY=0
PRINT_ALL=0
for a in "$@"; do
    case "$a" in
        --build-only) BUILD_ONLY=1 ;;
        --print-all)  PRINT_ALL=1 ;;
        -h|--help) grep '^#' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    esac
done

echo "=== 编译 sbus_dump ==="
# --print-all: 用 -D 覆盖源文件内的默认值(源文件已用 #ifndef 兜底, -D 能生效)。
PRINT_ALL_FLAG=()
if [ "$PRINT_ALL" -eq 1 ]; then
    PRINT_ALL_FLAG=(-DPRINT_ALL_ALWAYS=1)
fi
gcc -O2 -Wall -o sbus_dump sbus_dump.c src/drivers_sbus.c -I src "${PRINT_ALL_FLAG[@]}" \
    || { echo "[ERROR] 编译失败"; exit 1; }
echo "[OK] 编译产物: ./sbus_dump (PRINT_ALL_ALWAYS=$PRINT_ALL)"

# 检查接收器设备
if [ ! -e /dev/usb_remote ]; then
    echo "[WARN] 未找到 /dev/usb_remote (接收器未插入? udev 规则未生效?)"
    ls /dev/ttyUSB* 2>/dev/null | head
    echo "       若存在 ttyUSBx, 可: sudo ln -sf /dev/ttyUSB0 /dev/usb_remote"
    if [ "$BUILD_ONLY" -eq 1 ]; then exit 0; fi
    read -r -p "按回车继续尝试运行, Ctrl+C 退出" _ || exit 0
fi

if [ "$BUILD_ONLY" -eq 1 ]; then
    echo "[OK] 仅编译完成"
    exit 0
fi

echo ""
echo "=== 运行(需要 root 访问串口) ==="
sudo ./sbus_dump
