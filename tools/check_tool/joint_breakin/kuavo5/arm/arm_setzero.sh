#!/bin/bash
###
 # @Author: Name
 # @Date: 2025-04-01 16:56:55
 # @LastEditors: Please set LastEditors
 # @LastEditTime: 2025-04-01 16:56:55
 # @FilePath: /kuavo/tools/check_tool/arm_setzero.sh
 # @Description: 手臂零点校准
### 

# 获取当前脚本所在文件夹的绝对路径
current_script_dir=$(dirname "$(realpath "$0")")

# 尝试多种方式找到项目根目录
# 方法1: 如果脚本在 tools/check_tool 目录下
if [[ "$current_script_dir" == *"/tools/check_tool"* ]]; then
    prefix="${current_script_dir%/tools/check_tool*}"
# 方法2: 如果脚本在 joint_breakin/kuavo5/arm 目录下，向上查找
elif [[ "$current_script_dir" == *"/joint_breakin/"* ]]; then
    # 向上查找，直到找到包含 tools/check_tool 的路径
    temp_dir="$current_script_dir"
    while [ "$temp_dir" != "/" ]; do
        if [ -d "$temp_dir/tools/check_tool" ] || [ -d "$temp_dir/../tools/check_tool" ]; then
            # 找到包含 tools/check_tool 的目录，提取项目根目录
            if [ -d "$temp_dir/tools/check_tool" ]; then
                prefix="$temp_dir"
            else
                prefix=$(dirname "$temp_dir")
            fi
            break
        fi
        temp_dir=$(dirname "$temp_dir")
    done
    # 如果还是没找到，尝试从当前路径推断
    if [ -z "$prefix" ] || [ "$prefix" == "/" ]; then
        # 从 joint_breakin 向上查找项目根目录
        prefix=$(echo "$current_script_dir" | sed 's|/joint_breakin/.*||')
    fi
else
    # 默认方法：尝试从当前目录向上查找
    temp_dir="$current_script_dir"
    while [ "$temp_dir" != "/" ]; do
        if [ -d "$temp_dir/src/kuavo-ros-control-lejulib" ] || [ -d "$temp_dir/installed/share/hardware_plant" ]; then
            prefix="$temp_dir"
            break
        fi
        temp_dir=$(dirname "$temp_dir")
    done
fi

# 如果还是没找到，使用默认路径（假设在 /home/lab/kuavo_opensource 或类似位置）
if [ -z "$prefix" ] || [ "$prefix" == "/" ]; then
    if [ -d "/home/lab/kuavo_opensource" ]; then
        prefix="/home/lab/kuavo_opensource"
    elif [ -d "$HOME/kuavo_opensource" ]; then
        prefix="$HOME/kuavo_opensource"
    else
        echo "Error: Cannot find project root directory."
        exit 1
    fi
fi

kuavo_ros_folder_path="$prefix/src/kuavo-ros-control-lejulib/hardware_plant/lib/ruiwo_controller"
kuavo_ros_file_path="$kuavo_ros_folder_path/arm_setzero.sh"

kuavo_open_folder_path="$prefix/installed/share/hardware_plant/lib/ruiwo_controller"
kuavo_open_file_path="$kuavo_open_folder_path/arm_setzero.sh"

# 检查文件是否存在
if [ -f "$kuavo_ros_file_path" ]; then
    armsetzeroPath=$kuavo_ros_folder_path
elif [ -f "$kuavo_open_file_path" ]; then
    armsetzeroPath=$kuavo_open_folder_path
else
    echo "The file does not exist."
    echo "Searched paths:"
    echo "  1. $kuavo_ros_file_path"
    echo "  2. $kuavo_open_file_path"
    exit 1
fi

cd "$armsetzeroPath" || exit 1
./arm_setzero.sh
