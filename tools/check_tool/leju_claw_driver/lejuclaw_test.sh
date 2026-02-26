#!/bin/bash

# 获取当前脚本所在文件夹的绝对路径
current_script_dir=$(dirname "$(realpath "$0")")
echo "current_script_dir: $current_script_dir"

cd $current_script_dir

export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${current_script_dir}/3rd_party/bin/unix64/release

# 检查参数并执行相应操作
if [ "$1" == "--continuous" ]; then
    # 持续发布0、100位置开合测试
    ./lejuclaw_test --continuous
elif [ "$1" == "--single" ] && [ -n "$2" ]; then
    # 发布单次目标位置控制夹爪运动
    ./lejuclaw_test --single $2
elif [ "$1" == "--start-server" ]; then
    # 启动夹爪服务器
    if [ -f "./lejuclaw_server" ]; then
        sudo env HARDWARE_TOOL_STARTED=1 LD_LIBRARY_PATH="$LD_LIBRARY_PATH" ./lejuclaw_server
    else
        echo "错误: 未找到 lejuclaw_server 可执行文件"
        exit 1
    fi
elif [ "$1" == "--send-position" ] && [ -n "$2" ] && [ -n "$3" ]; then
    # 发布目标位置控制夹爪运动
    # $2 是左夹爪位置，$3 是右夹爪位置
    if [ -f "./lejuclaw_controller.py" ]; then
        python3 ./lejuclaw_controller.py $2 $3
    else
        echo "错误: 未找到 lejuclaw_controller.py 脚本"
        exit 1
    fi
else
    # 默认模式
    ./lejuclaw_test "$@"
fi