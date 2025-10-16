#!/bin/bash
###
 # @Author: dongdongmingming
 # @Date: 2024-05-17 09:56:55
 # @LastEditors: Please set LastEditors
 # @LastEditTime: 2024-05-17 10:00:24
 # @FilePath: /kuavo/tools/check_tool/ruiwo_negtive_set.sh
 # @Description: 瑞沃电机设置方向
### 

# 检查是否传递了参数
if [ $# -eq 0 ]; then
    echo "错误：请传递机器人类型参数"
    echo "用法: $0 <robot_type>"
    echo "robot_type: 4pro 或 roban2"
    exit 1
fi

ROBOT_TYPE=$1

# 获取当前脚本所在文件夹的绝对路径
current_script_dir=$(dirname "$(realpath "$0")")

# 使用字符串操作来截取前部分路径
prefix="${current_script_dir%/tools/check_tool}"

kuavo_ros_folder_path="$prefix/src/kuavo-ros-control-lejulib/hardware_plant/lib/ruiwo_controller"
kuavo_open_folder_path="$prefix/installed/share/hardware_plant/lib/ruiwo_controller"

# 根据机器人类型选择脚本
if [ "$ROBOT_TYPE" = "4pro" ]; then
    kuavo_ros_file_path="$kuavo_ros_folder_path/Negtive_4Pro.sh"
    kuavo_open_file_path="$kuavo_open_folder_path/Negtive_4Pro.sh"
    echo "选择4Pro型机器人脚本"
elif [ "$ROBOT_TYPE" = "roban2" ]; then
    kuavo_ros_file_path="$kuavo_ros_folder_path/Negtive_Roban2.sh"
    kuavo_open_file_path="$kuavo_open_folder_path/Negtive_Roban2.sh"
    echo "选择Roban2型机器人脚本"
else
    echo "错误：无效的机器人类型 '$ROBOT_TYPE'"
    echo "支持的机器人类型: 4pro, roban2"
    exit 1
fi

# 检查文件是否存在
if [ -f "$kuavo_ros_file_path" ]; then
    setZeroPath=$kuavo_ros_folder_path
    if [ "$ROBOT_TYPE" = "4pro" ]; then 
        script_name="Negtive_4Pro.sh"
    else 
        script_name="Negtive_Roban2.sh"
    fi
elif [ -f "$kuavo_open_file_path" ]; then
    setZeroPath=$kuavo_open_folder_path
    if [ "$ROBOT_TYPE" = "4pro" ]; then 
        script_name="Negtive_4Pro.sh"
    else 
        script_name="Negtive_Roban2.sh"
    fi
else
    echo "错误：未找到机器人类型 '$ROBOT_TYPE' 的脚本文件"
    echo "kuavo_ros路径: $kuavo_ros_file_path"
    echo "kuavo_open路径: $kuavo_open_file_path"
    exit 1
fi

echo "执行脚本路径: $setZeroPath"
echo "执行脚本: $script_name"

cd $setZeroPath

./$script_name
