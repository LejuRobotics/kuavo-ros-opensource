#!/bin/bash

# 获取当前脚本所在文件夹的绝对路径
current_script_dir=$(dirname "$(realpath "$0")")
echo "current_script_dir: $current_script_dir"

PROJECT_DIR=$(realpath "$current_script_dir/../../..")

EXECUTABLE_PATHS=(
    "$PROJECT_DIR/devel/lib/hardware_node/lejuclaw_can_test_single_position"
    "$PROJECT_DIR/devel/lib/lejuclaw_can_test_single_position"
    "$PROJECT_DIR/installed/bin/lejuclaw_can_test_single_position"
    "$PROJECT_DIR/installed/lib/lejuclaw_can_test_single_position"
)

exec_path=""
for path in "${EXECUTABLE_PATHS[@]}"; do
    if [ -f "$path" ] && [ -x "$path" ]; then
        exec_path="$path"
        break
    fi
done

if [ -z "$exec_path" ]; then
    echo "错误: 未找到 lejuclaw_can_test_single_position 可执行文件"
    echo "尝试了以下路径:"
    for path in "${EXECUTABLE_PATHS[@]}"; do
        echo "  - $path"
    done
    exit 1
fi

export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${PROJECT_DIR}/devel/lib
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${PROJECT_DIR}/installed/lib

echo "LD_LIBRARY_PATH after update:"
echo "  $LD_LIBRARY_PATH"
echo "执行: $exec_path"

"$exec_path"