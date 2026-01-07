#!/bin/bash

# 获取当前脚本所在文件夹的绝对路径
current_script_dir=$(dirname "$(realpath "$0")")
echo "current_script_dir: $current_script_dir"

cd $current_script_dir

# 计算项目根目录
project_root=$(cd "$current_script_dir/../../.." && pwd)
echo "project_root: $project_root"

export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${project_root}/installed/lib:${project_root}/devel/lib

./lejuclaw_can_test