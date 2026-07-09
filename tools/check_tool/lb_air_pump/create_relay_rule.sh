#!/bin/bash
# 1、获取设备物理端口位置：脚本使用 udevadm info 获取设备的物理 USB 端口路径（如 3-8.1:1.0）
# 2、创建 udev 规则：生成规则文件 /etc/udev/rules.d/99-kuavo-relay.rules
#    内容类似：KERNEL=="ttyUSB*", KERNELS=="3-8.1:1.0", MODE:="0666", SYMLINK+="kuavo_relay"
# 3、重载规则：执行 udevadm control --reload-rules 和 udevadm trigger 使规则生效
#
# 使用方法:
#   默认（自动配置）:
#     sudo bash create_relay_rule.sh
#          /dev/ttyUSB1 → kuavo_pressure (压力表)
#          /dev/ttyUSB0 → kuavo_relay    (继电器)
#
#   交换映射:
#     sudo bash create_relay_rule.sh --change
#          /dev/ttyUSB0 → kuavo_pressure (压力表)
#          /dev/ttyUSB1 → kuavo_relay    (继电器)
#
#   手动指定单个设备:
#     sudo bash create_relay_rule.sh /dev/ttyUSB2 kuavo_pressure_right

set -e

RULE_FILE="/etc/udev/rules.d/99-kuavo-relay.rules"

check_root() {
    if [ "$EUID" -ne 0 ]; then
        echo "请使用 sudo 运行此脚本"
        echo "示例: sudo bash create_relay_rule.sh"
        exit 1
    fi
}

get_kernels() {
    local port="$1"
    local k
    k=$(udevadm info --attribute-walk --name="$port" 2>/dev/null | grep -m 1 "KERNELS==\"[0-9]*-[0-9]*\.[0-9]*\.[0-9]*\"" | cut -d'"' -f2)
    if [ -z "$k" ]; then
        k=$(udevadm info --attribute-walk --name="$port" 2>/dev/null | grep -m 1 "KERNELS==\"[0-9]*-[0-9]*\.[0-9]*\"" | cut -d'"' -f2)
    fi
    if [ -z "$k" ]; then
        k=$(udevadm info --attribute-walk --name="$port" 2>/dev/null | grep -m 1 "KERNELS==\"[0-9]*-[0-9]*\"" | cut -d'"' -f2)
    fi
    echo "$k"
}

add_rule() {
    local port="$1"
    local symlink="$2"

    if [ ! -e "$port" ]; then
        echo "错误: $port 不存在，跳过"
        return 1
    fi

    local kernels=$(get_kernels "$port")

    if [ -z "$kernels" ]; then
        echo "警告: 无法获取 $port 的物理端口，尝试使用 ID_SERIAL_SHORT"
        local serial_short=$(udevadm info --query=property --name="$port" 2>/dev/null | grep "ID_SERIAL_SHORT=" | cut -d= -f2)
        if [ -n "$serial_short" ] && [[ "$serial_short" != *":"* ]]; then
            echo "  使用序列号: $serial_short"
            RULE='KERNEL=="ttyUSB*", ENV{ID_SERIAL_SHORT}=="'"$serial_short"'", MODE:="0666", SYMLINK+="'"$symlink"'"'
        else
            echo "错误: 无法获取 $port 的有效标识"
            return 1
        fi
    else
        echo "  $port 锁定物理端口: $kernels"
        RULE='KERNEL=="ttyUSB*", KERNELS=="'"$kernels"'", MODE:="0666", SYMLINK+="'"$symlink"'"'
    fi

    # 去重：删除已有同名 symlink 的规则
    if [ -f "$RULE_FILE" ]; then
        sed -i '/SYMLINK+="'"$symlink"'"/d' "$RULE_FILE"
    fi

    echo "$RULE" >> "$RULE_FILE"
    echo "  规则: $RULE"
}

apply_rules() {
    echo ""
    echo "规则文件 $RULE_FILE 内容:"
    echo "----------------------------------------"
    cat "$RULE_FILE" 2>/dev/null || echo "(空)"
    echo "----------------------------------------"

    echo "正在重载 udev 规则..."
    udevadm control --reload-rules
    udevadm trigger
    sleep 1

    echo ""
    for name in kuavo_relay kuavo_pressure; do
        if [ -L "/dev/$name" ]; then
            echo "  /dev/$name -> $(readlink -f /dev/$name)"
        else
            echo "  /dev/$name 尚未生成 (请重新插拔设备)"
        fi
    done
    echo ""
    echo "配置完成。"
}

# ========== 主逻辑 ==========

check_root

case "${1:-}" in
    --change|-c)
        echo "交换模式: ttyUSB0→压力表  ttyUSB1→继电器"
        > "$RULE_FILE"
        add_rule "/dev/ttyUSB0" "kuavo_pressure"
        add_rule "/dev/ttyUSB1" "kuavo_relay"
        apply_rules
        ;;
    "")
        echo "默认模式: ttyUSB1→压力表  ttyUSB0→继电器"
        > "$RULE_FILE"
        add_rule "/dev/ttyUSB1" "kuavo_pressure"
        add_rule "/dev/ttyUSB0" "kuavo_relay"
        apply_rules
        ;;
    --help|-h)
        echo "用法:"
        echo "  sudo bash create_relay_rule.sh              # 默认: ttyUSB1→压力表, ttyUSB0→继电器"
        echo "  sudo bash create_relay_rule.sh --change     # 交换: ttyUSB0→压力表, ttyUSB1→继电器"
        echo "  sudo bash create_relay_rule.sh <端口> <名称> # 手动指定单个设备"
        ;;
    *)
        TARGET_PORT="$1"
        SYMLINK_NAME="${2:-kuavo_relay}"
        if [ ! -e "$TARGET_PORT" ]; then
            echo "错误: $TARGET_PORT 不存在"
            exit 1
        fi
        echo "手动模式: $TARGET_PORT → $SYMLINK_NAME"
        add_rule "$TARGET_PORT" "$SYMLINK_NAME"
        apply_rules
        ;;
esac
