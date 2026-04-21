#!/bin/bash

export ROS_WS_PATH=/opt/ros/noetic
source $ROS_WS_PATH/setup.bash

# 检查必要的环境变量
if [ -z "$NODE_SCRIPT" ]; then
    echo "Error: NODE_SCRIPT environment variable is not set"
    exit 1
fi

if [ ! -f "$NODE_SCRIPT" ]; then
    echo "Error: Node script not found at $NODE_SCRIPT"
    exit 1
fi

NODE_PID=""

start_node() {
    source $HOME/.bashrc
    $NODE_SCRIPT &
    NODE_PID=$!
    echo "Started h12pro node with PID: $NODE_PID"
}

stop_node() {
    if [ ! -z "$NODE_PID" ]; then
        kill -SIGINT $NODE_PID
        wait $NODE_PID 2>/dev/null || true
        NODE_PID=""
        echo "Stopped h12pro node"
    fi
}

# 添加信号处理
cleanup() {
    echo "Cleaning up..."
    stop_node
    exit 0
}

# 注册信号处理器
trap cleanup SIGTERM SIGINT SIGHUP

while true; do
    START_WAY=$(rosparam get /start_way 2>/dev/null)
    JOY_NODE_RUNNING=$(rosnode list 2>/dev/null | grep -q "/joy_node" && echo "true" || echo "false")
    
    echo "JOY_NODE_RUNNING: $JOY_NODE_RUNNING"
    echo "START_WAY: $START_WAY"
    
    if [ "$START_WAY" = "manual" ] && [ "$JOY_NODE_RUNNING" = "true" ]; then
        if [ ! -z "$NODE_PID" ]; then
            stop_node
        fi
    else
        if [ "$JOY_NODE_RUNNING" = "false" ] && [ -z "$NODE_PID" ]; then
            start_node
        fi
    fi
    
    # 检查进程是否还在运行
    if [ ! -z "$NODE_PID" ]; then
        if ! kill -0 $NODE_PID 2>/dev/null; then
            echo "Node process died unexpectedly"
            NODE_PID=""
        fi
    fi
    
    sleep 5
done