#!/bin/bash
###
 # @Author: Name
 # @Date: 2025-06-30 10:33:00
 # @LastEditors: Please set LastEditors
 # @LastEditTime: 2025-06-30 10:33:00
 # @FilePath: /kuavo/tools/check_tool/hip_imu_test.sh
 # @Description: 国产IMU测试
### 

# 获取当前脚本所在文件夹的绝对路径
current_script_dir=$(dirname "$(realpath "$0")")


# 使用字符串操作来截取前部分路径
prefix="${current_script_dir%/tools/check_tool}"

kuavo_ros_folder_path="$prefix/src/kuavo-ros-control-lejulib/hardware_plant/lib/hipnuc_imu/scripts"
kuavo_ros_file_path="$kuavo_ros_folder_path/hip_imu_test.sh"

kuavo_open_folder_path="$prefix/installed/share/hardware_plant/lib/hipnuc_imu/scripts"
kuavo_open_file_path="$kuavo_open_folder_path/hip_imu_test.sh"


# 检查文件是否存在
if [ -f "$kuavo_ros_file_path" ]; then
    hipimutestPath=$kuavo_ros_folder_path
elif [ -f "$kuavo_open_file_path" ]; then
    hipimutestPath=$kuavo_open_folder_path
else
    echo "The file does not exist."
    exit 1
fi

cd $hipimutestPath
./hip_imu_test.sh
