#!/bin/bash
###
 # @Author: Name
 # @Date: 2025-04-01 16:56:55
 # @LastEditors: Please set LastEditors
 # @LastEditTime: 2025-04-01 16:56:55
 # @FilePath: /kuavo/tools/check_tool/arm_breakin.sh
 # @Description: 手臂磨线脚本
### 

# 获取当前脚本所在文件夹的绝对路径
current_script_dir=$(dirname "$(realpath "$0")")

# 尝试多种方式找到项目根目录
# 方法1: 如果脚本在 tools/check_tool/joint_breakin/kuavo5/arm 目录下
if [[ "$current_script_dir" == *"/joint_breakin/kuavo5/arm"* ]]; then
    # 向上查找项目根目录
    temp_dir="$current_script_dir"
    while [ "$temp_dir" != "/" ]; do
        if [ -d "$temp_dir/tools/check_tool" ] || [ -d "$temp_dir/../tools/check_tool" ]; then
            if [ -d "$temp_dir/tools/check_tool" ]; then
                prefix="$temp_dir"
            else
                prefix=$(dirname "$temp_dir")
            fi
            break
        fi
        temp_dir=$(dirname "$temp_dir")
    done
# 方法2: 如果脚本在 tools/check_tool 目录下
elif [[ "$current_script_dir" == *"/tools/check_tool"* ]]; then
    prefix="${current_script_dir%/tools/check_tool*}"
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

# 如果还是没找到，使用默认路径
if [ -z "$prefix" ] || [ "$prefix" == "/" ]; then
    if [ -d "/home/lab/kuavo_opensource" ]; then
        prefix="/home/lab/kuavo_opensource"
    elif [ -d "$HOME/kuavo_opensource" ]; then
        prefix="$HOME/kuavo_opensource"
    else
        echo "Error: Cannot find project root directory."
        echo "Searched paths:"
        echo "  1. $current_script_dir/../.. (for tools/check_tool)"
        echo "  2. /home/lab/kuavo_opensource"
        echo "  3. $HOME/kuavo_opensource"
        exit 1
    fi
fi

kuavo_ros_folder_path="$prefix/src/kuavo-ros-control-lejulib/hardware_plant/lib/ruiwo_controller"
kuavo_ros_file_path="$kuavo_ros_folder_path/arm_breakin.sh"

kuavo_open_folder_path="$prefix/installed/share/hardware_plant/lib/ruiwo_controller"
kuavo_open_file_path="$kuavo_open_folder_path/arm_breakin.sh"

# 尝试查找 arm_breakin.py（在项目根目录下）
arm_breakin_py_path="$prefix/arm_breakin.py"

# 检查文件是否存在
if [ -f "$kuavo_ros_file_path" ]; then
    armbreakinPath=$kuavo_ros_folder_path
    cd "$armbreakinPath" || exit 1
    ./arm_breakin.sh "$@"
elif [ -f "$kuavo_open_file_path" ]; then
    armbreakinPath=$kuavo_open_folder_path
    cd "$armbreakinPath" || exit 1
    ./arm_breakin.sh "$@"
elif [ -f "$arm_breakin_py_path" ]; then
    # 直接调用 arm_breakin.py
    cd "$prefix" || exit 1
    python3 "$arm_breakin_py_path" "$@"
else
    echo "Error: Cannot find arm_breakin script."
    echo "Searched paths:"
    echo "  1. $kuavo_ros_file_path"
    echo "  2. $kuavo_open_file_path"
    echo "  3. $arm_breakin_py_path"
    exit 1
fi
