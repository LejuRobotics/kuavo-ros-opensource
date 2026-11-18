#!/bin/bash

# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 定义规则文件名和目标路径
RULE_FILE="H12_log_serial.rules"
RULE_FILE_PATH="$SCRIPT_DIR/udev/$RULE_FILE"
TARGET_DIR="/etc/udev/rules.d"
TARGET_FILE="$TARGET_DIR/99-$RULE_FILE"
EXPECTED_H12_MODEL="CP2102_USB_to_UART_Bridge_Controller"
# 测试可用 H12_TTY_GLOB 指向伪造设备；生产环境始终使用默认值。
TTY_DEVICE_GLOB="${H12_TTY_GLOB:-/dev/ttyUSB*}"

# 检查规则文件是否存在
if [ ! -f "$RULE_FILE_PATH" ]; then
    echo "错误：规则文件 '$RULE_FILE_PATH' 不存在！"
    exit 1
fi

# 安装前检测独立 H12 日志串口。CP2102N 是当前 HIPNUC IMU，必须排除。
echo "安装前检测独立的 H12 日志串口..."
candidate_devices=()
for dev in $TTY_DEVICE_GLOB; do
    [ -e "$dev" ] || continue

    props="$(udevadm info --query=property --name="$dev" 2>/dev/null)" || continue
    vendor="$(printf '%s\n' "$props" | sed -n 's/^ID_VENDOR_ID=//p')"
    model="$(printf '%s\n' "$props" | sed -n 's/^ID_MODEL=//p')"

    if [[ "$vendor" == "10c4" && "$model" == "$EXPECTED_H12_MODEL" ]]; then
        candidate_devices+=("$dev")
    fi
done

if [ "${#candidate_devices[@]}" -eq 0 ]; then
    echo "未检测到独立的 H12 CP2102 日志串口，拒绝安装规则。"
    echo "HIPNUC CP2102N 不会被当作 H12。"
    exit 1
fi

if [ "${#candidate_devices[@]}" -ne 1 ]; then
    echo "检测到多个普通 CP2102 候选设备，无法唯一确定 H12："
    printf '  %s\n' "${candidate_devices[@]}"
    echo "为避免误配置，未安装规则。"
    exit 1
fi

H12_DEV="${candidate_devices[0]}"
HIPNUC_TARGET="$(readlink -f /dev/HIPNUC_IMU 2>/dev/null || true)"
if [[ -n "$HIPNUC_TARGET" && "$HIPNUC_TARGET" == "$H12_DEV" ]]; then
    echo "候选设备 $H12_DEV 已被 /dev/HIPNUC_IMU 使用，拒绝安装。"
    exit 1
fi

H12_PROPS="$(udevadm info --query=property --name="$H12_DEV" 2>/dev/null)"
H12_SERIAL="$(printf '%s\n' "$H12_PROPS" | sed -n 's/^ID_SERIAL_SHORT=//p')"
echo "检测到 H12 日志串口：$H12_DEV"
echo "设备序列号：${H12_SERIAL:-未提供}"

# 只读检测模式，供部署前检查和自动化测试使用，不复制规则、不执行 sudo。
if [[ "${1:-}" == "--detect-only" ]]; then
    echo "DETECT_OK $H12_DEV"
    exit 0
fi

# 使用sudo将规则文件拷贝到指定目录
echo "正在将规则文件 '$RULE_FILE_PATH' 拷贝到 '$TARGET_DIR'..."
sudo cp "$RULE_FILE_PATH" "$TARGET_FILE"

if [ $? -ne 0 ]; then
    echo "错误：文件拷贝失败，请检查权限。"
    exit 1
fi
echo "拷贝成功！"

# 只重新加载规则，不做全局 trigger，避免重放 IMU 等其他设备事件。
echo "正在重新加载 udev 规则..."
sudo udevadm control --reload-rules
echo "udev 规则已更新。"

# 查找所有非系统用户，并将其加入 dialout 组
echo "正在将所有用户加入 'dialout' 用户组..."
for user in $(grep -E ':[0-9]{4}:' /etc/passwd | cut -d: -f1); do
    if ! id -nG "$user" | grep -qw "dialout"; then
        sudo usermod -a -G dialout "$user"
        echo "用户 '$user' 已加入 'dialout' 组。"
    else
        echo "用户 '$user' 已经在 'dialout' 组中。"
    fi
done

echo "脚本执行完毕。请拔下并重新插入 H12 日志串口，或在维护窗口重启，使规则生效。"
exit 0
