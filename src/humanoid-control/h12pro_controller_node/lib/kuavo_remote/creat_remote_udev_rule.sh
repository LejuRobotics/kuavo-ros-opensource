#!/bin/bash

# Function to create udev rule
create_udev_rule() {
    local rule_name=$1
    local product=$2
    echo 'KERNEL=="ttyUSB*", ACTION=="add", ATTRS{product}=="'$product'", MODE:="0777", ATTR{latency_timer}="1", SYMLINK+="'$rule_name'"' > /etc/udev/rules.d/$rule_name.rules
    echo "生成成功! 请重启计算机或者插拔设备以使规则生效。"
    # Reload udev rules
    udevadm control --reload-rules
    udevadm trigger
}

# Get all ttyUSB devices
usb_devices=$(ls /dev/ttyUSB* 2>/dev/null)

if [ -z "$usb_devices" ]; then
    echo '未找到任何ttyUSB设备，请检查设备连接。'
    exit 1
fi

found_device=false

for dev in $usb_devices; do
    echo "正在检查设备: $dev"
    # 获取设备属性信息
    udevadm_info=$(udevadm info --attribute-walk --name=$dev)
    echo "udevadm info 输出: $udevadm_info"
    
    # 检查 LJREMOTE 设备
    ljremote_product=$(echo "$udevadm_info" | grep 'ATTRS{product}=="LJREMOTE"' | awk -F'=="' '{print $2}' | sed 's/"//g')
    echo "ljremote_product: $ljremote_product"  # 打印 ljremote_product 的值

    if [ -n "$ljremote_product" ]; then
        echo '找到LJREMOTE设备，正在应用udev规则...'
        create_udev_rule "usb_remote" "LJREMOTE"
        found_device=true
        break
    fi

    # 检查 USB Serial 设备
    usb_serial_product=$(echo "$udevadm_info" | grep 'ATTRS{product}=="USB Serial"' | awk -F'=="' '{print $2}' | sed 's/"//g')
    echo "usb_serial_product: $usb_serial_product"  # 打印 usb_serial_product 的值

    if [ -n "$usb_serial_product" ]; then
        echo '找到USB Serial设备，正在应用udev规则...'
        create_udev_rule "usb_remote" "USB Serial"
        found_device=true
        break
    fi
done

if [ "$found_device" = false ]; then
    echo '未找到LJREMOTE或USB Serial设备，请检查电源板遥控器线束是否连接,或遥控器外接模块是否存在。'
fi
