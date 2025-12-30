#!/bin/bash

# 获取环境变量 ROBOT_VERSION
ROBOT_VERSION=${ROBOT_VERSION}

# 定义文件路径
FILE_PATH="$HOME/.config/lejuconfig/EcMasterType.ini"

# 检查环境变量是否为 42版本才有ecmaster驱动器区别
if [ "$ROBOT_VERSION" == "42" ] || [ "$ROBOT_VERSION" == "32" ]; then
    if [ ! -f "$FILE_PATH" ]; then
        echo -e "\033[33m\nWarning: 未指定硬件EcMaster类型(只运行仿真可以忽略), 实物机器将默认使用\`Elmo\`类型驱动器\033[0m" >&2
        echo "elmo" > $FILE_PATH
        echo -e "\033[33m通过\`echo youda > $FILE_PATH\` 命令可以指定EcMaster类型为youda\033[0m\n" >&2
    else
        echo -e "\033[33m\n由 $FILE_PATH 文件指定的EcMaster类型为:\`$(cat $FILE_PATH)\`\033[0m\n" >&2
    fi
    chmod 0777 $FILE_PATH
fi

IMU_TYPE_FILE="$HOME/.config/lejuconfig/ImuType.ini"
if [ ! -f "$IMU_TYPE_FILE" ]; then
    echo -e "\033[33m\nWarning: 未指定IMU类型(只运行仿真可以忽略), 实物机器将默认使用\`xsens\`类型IMU\033[0m" >&2
    echo "xsens" > $IMU_TYPE_FILE
    echo -e "\033[33m通过\`echo hipnuc > $IMU_TYPE_FILE\` 命令可以指定IMU类型为hipnuc\033[0m\n" >&2
else
    echo -e "\033[33m\n由 $IMU_TYPE_FILE 文件指定的IMU类型为:\`$(cat $IMU_TYPE_FILE)\`\033[0m\n" >&2
fi
chmod 0777 $IMU_TYPE_FILE

