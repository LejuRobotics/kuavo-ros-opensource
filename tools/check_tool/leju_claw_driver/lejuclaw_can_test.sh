#!/bin/bash

# 获取当前脚本所在文件夹的绝对路径
current_script_dir=$(dirname "$(realpath "$0")")
echo "current_script_dir: $current_script_dir"

cd $current_script_dir

export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${current_script_dir}/3rd_party/bin/unix64/release

./lejuclaw_can_test