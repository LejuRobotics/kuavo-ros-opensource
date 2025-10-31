#!/bin/bash

# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 定义规则文件名和目标路径
RULE_FILE="H12_log_serial.rules"
RULE_FILE_PATH="$SCRIPT_DIR/udev/$RULE_FILE"
TARGET_DIR="/etc/udev/rules.d"
TARGET_FILE="$TARGET_DIR/99-$RULE_FILE"

# 检查规则文件是否存在
if [ ! -f "$RULE_FILE_PATH" ]; then
    echo "错误：规则文件 '$RULE_FILE_PATH' 不存在！"
    exit 1
fi

# 使用sudo将规则文件拷贝到指定目录
echo "正在将规则文件 '$RULE_FILE_PATH' 拷贝到 '$TARGET_DIR'..."
sudo cp "$RULE_FILE_PATH" "$TARGET_FILE"

# 检查拷贝是否成功
if [ $? -ne 0 ]; then
    echo "错误：文件拷贝失败，请检查权限。"
    exit 1
fi
echo "拷贝成功！"

# 重新加载udev规则并触发设备事件
echo "正在重新加载 udev 规则并触发..."
sudo udevadm control --reload-rules
sudo udevadm trigger
echo "udev 规则已更新。"

# 查找所有非系统用户，并将其加入 dialout 组
echo "正在将所有用户加入 'dialout' 用户组..."
# 通过 /etc/passwd 文件查找用户，UID >= 1000 通常为普通用户
for user in $(grep -E ':[0-9]{4}:' /etc/passwd | cut -d: -f1); do
    if ! id -nG "$user" | grep -qw "dialout"; then
        sudo usermod -a -G dialout "$user"
        echo "用户 '$user' 已加入 'dialout' 组。"
    else
        echo "用户 '$user' 已经在 'dialout' 组中。"
    fi
done

echo "脚本执行完毕。请拔下并重新插入设备以使更改完全生效。"

exit 0