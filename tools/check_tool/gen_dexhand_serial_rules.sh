#!/bin/bash

echo -e "\033[32m--------------------------------------------------\033[0m"
echo -e "\033[32m欢迎使用灵巧手串口设备绑定工具\033[0m"
echo -e "\033[32m--------------------------------------------------\033[0m"

# 选择手的类型
# 非触觉手: 串口设备名称为: stark_serial_L, stark_serial_R
# 触觉手: 串口设备名称为: stark_serial_touch_L, stark_serial_touch_R
while true; do
    echo -e "请选择手的类型:"
    echo "0 : 非触觉手"
    echo "1 : 触觉手"
    read -p "请输入对应的数字(0/1): " hand_type
    case $hand_type in
        0|1)
            break
            ;;
        *)
            echo -e "\033[31m无效的输入，请输入 0 或 1.\033[0m"
            ;;
    esac
done

if [ "$hand_type" -eq 0 ]; then
    echo -e "\033[32m--------------------------------------------------\033[0m"    
    echo -e "\033[32m已选择: 非触觉手\033[0m"
    echo -e "\033[32m--------------------------------------------------\033[0m"
    right_hand_rule_name="stark_serial_R"
    left_hand_rule_name="stark_serial_L"
else
    echo -e "\033[32m--------------------------------------------------\033[0m"
    echo -e "\033[32m已选择: 触觉手\033[0m"
    echo -e "\033[32m--------------------------------------------------\033[0m"
    right_hand_rule_name="stark_serial_touch_R"
    left_hand_rule_name="stark_serial_touch_L"
fi

# list ttyUSB* devices
echo -e "\033[32m\n正在列出所有 'ttyUSB*' 设备:\033[0m"
# Check if any ttyUSB devices exist
if ! ls /dev/ttyUSB* >/dev/null 2>&1; then
    echo -e "\033[31m\nError: 没有找到任何 ttyUSB 设备.\033[0m"
    exit 1
fi
ls -lh /dev/ttyUSB*
echo -e "--------------------------------------------------"

read -p "请输入想要绑定的串口号(数字, 例如: 0, 1, 2, 3, ...):" dev
echo "你选择的串口为: /dev/ttyUSB$dev"

if [ ! -e "/dev/ttyUSB$dev" ]; then
    echo -e "\033[31m\nError: /dev/ttyUSB$dev 不存在.\033[0m"
    exit 1
fi

# 询问绑定到左手或右手
while true; do
    read -p "你想绑定 /dev/ttyUSB$dev 到左手还是右手? (l/r): " hand
    case $hand in
        l|L)
            rule_name=$left_hand_rule_name
            break
            ;;
        r|R)
            rule_name=$right_hand_rule_name
            break
            ;;
        *)
            echo "无效的输入，请输入 l 或 r."
            ;;
    esac
done

echo -e "\033[33m/dev/ttyUSB$dev 将被自定义为: $rule_name\033[0m"

# 确认是否继续
while true; do
    read -p "确认要为此设备创建规则吗? (y/n): " confirm
    case $confirm in
        y|Y)
            break
            ;;
        n|N)
            echo "用户取消操作, 退出程序"
            exit 0
            ;;
        *)
            echo "无效的输入，请输入 y 或 n"
            ;;
    esac
done


serial=$(udevadm info --attribute-walk --name=/dev/ttyUSB$dev|grep ATTRS{serial} | cut -d= -f3 | sed 's/"//g'|head -n 1)
# # 如果属性不为空，创建一个udev规则文件
if [ -n "$serial" ]; then
echo '正在为序列号为'$serial'的设备生成udev规则...'
echo 'KERNEL=="ttyUSB*", ATTRS{serial}=="'$serial'", MODE:="0777", SYMLINK+="'$rule_name'"' > /etc/udev/rules.d/$rule_name.rules
echo '生成成功! 请重启计算机或者插拔设备以使规则生效。'
else 
echo '未找到序列号，请检查设备是否已连接。'
fi

# 重新加载udev规则
udevadm control --reload-rules
udevadm trigger