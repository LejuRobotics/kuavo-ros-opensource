#!/bin/bash

# 移动操作器回放模式自动化脚本
# 用法: ./playback_automation.sh <bag_file_path> [start_time]

set -e  # 遇到错误时退出

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 打印带颜色的消息
print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 检查参数
if [ $# -lt 1 ]; then
    print_error "用法: $0 <bag_file_path> [start_time]"
    print_error "示例: $0 bt_2025-07-18-13-53-32_0.bag 12"
    exit 1
fi

BAG_FILE="$1"
START_TIME="${2:-0}"  # 默认从0秒开始

# 检查bag文件是否存在
if [ ! -f "$BAG_FILE" ]; then
    print_error "Bag文件不存在: $BAG_FILE"
    exit 1
fi

print_info "开始回放模式自动化..."
print_info "Bag文件: $BAG_FILE"
print_info "开始时间: ${START_TIME}秒"

# 检查ROS环境
if [ -z "$ROS_DISTRO" ]; then
    print_error "ROS环境未设置，请先source setup.bash"
    exit 1
fi

# 检查roscore是否运行
if ! rostopic list > /dev/null 2>&1; then
    print_warn "roscore未运行，正在启动..."
    roscore &
    ROSCORE_PID=$!
    sleep 3
fi

# 定义话题列表
TOPICS=(
    "/humanoid_mpc_observation"
    "/humanoid_wbc_observation"
    "/humanoid_mpc_policy"
    "/sensors_data_raw"
    "/mm/control_type"
    "/mm/end_effector_trajectory"
)

# 构建话题参数
TOPICS_PARAM=""
for topic in "${TOPICS[@]}"; do
    TOPICS_PARAM="$TOPICS_PARAM $topic"
done

# 播放bag文件
print_info "开始播放bag文件..."
if [ "$START_TIME" -gt 0 ]; then
    rosbag play "$BAG_FILE" --topics $TOPICS_PARAM -s "$START_TIME"
else
    rosbag play "$BAG_FILE" --topics $TOPICS_PARAM
fi

print_info "Bag文件播放完成"

# 清理
print_info "清理进程..."
if [ ! -z "$CONTROLLER_PID" ]; then
    kill $CONTROLLER_PID 2>/dev/null || true
fi

if [ ! -z "$ROSCORE_PID" ]; then
    kill $ROSCORE_PID 2>/dev/null || true
fi

print_info "回放模式自动化完成" 