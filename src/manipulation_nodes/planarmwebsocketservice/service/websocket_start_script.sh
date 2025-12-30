#!/bin/bash

# 要检测的 launch 文件名
LAUNCH1="humanoid_plan_arm_trajectory.launch"
LAUNCH2="plan_arm_action_websocket_server.launch"

# 检查并杀掉已运行的 launch 进程
for LAUNCH in "$LAUNCH1" "$LAUNCH2"
do
    # 查找包含该 launch 文件名的进程
    PIDS=$(ps aux | grep "[r]oslaunch" | grep "$LAUNCH" | awk '{print $2}')
    if [[ -n "$PIDS" ]]; then
        echo "检测到 $LAUNCH 正在运行，正在终止..."
        for PID in $PIDS
        do
            kill $PID
            echo "已终止进程 $PID"
        done
        # 可选：等待进程完全退出
        sleep 2
    fi
done

# 获取脚本自身所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "SCRIPT_DIR: $SCRIPT_DIR"
cd "$SCRIPT_DIR"

# 使用 git 命令获取仓库根目录
REPO_ROOT=$(git rev-parse --show-toplevel)
if [[ -z "$REPO_ROOT" ]]; then
    echo "错误：无法找到 git 仓库根目录"
    exit 1
fi
echo "仓库根目录: $REPO_ROOT"
cd "$REPO_ROOT"

source /opt/ros/noetic/setup.bash --extend
source devel/setup.bash --extend

# 检查是否有 wifi 连接
echo "正在检测 WiFi 连接..."


HOTSPOT_IP="192.168.12.1"
while true; do
    ssid=$(nmcli -t -f active,ssid dev wifi | grep '^yes' | cut -d: -f2 || echo "")
    hotspot_name=$(grep "ROBOT_SERIAL_NUMBER" /etc/environment.d/RRNIS.env | cut -d'=' -f2 || echo "")
    ap_interface=$(ifconfig | grep -o 'ap[0-9]*' || echo "")
    ap_ip=$(ifconfig $ap_interface 2>/dev/null | grep 'inet ' | awk '{print $2}' || echo "")

    if [ -n "$ssid" ]; then
        echo "$(date '+%Y-%m-%d %H:%M:%S') ✅ 检测到机器人已连接的 WiFi：$ssid"
    fi

    if [ -n "$hotspot_name" ] && [ -n "$ap_interface" ] && [ "$ap_ip" == "$HOTSPOT_IP" ]; then
        echo "$(date '+%Y-%m-%d %H:%M:%S') ✅ 检测到机器人发射的热点：${hotspot_name}的热点"
    fi

    if [ -n "$ssid" ] || ([ -n "$hotspot_name" ] && [ -n "$ap_interface" ] && [ "$ap_ip" == "$HOTSPOT_IP" ]); then
        break
    else
        echo "$(date '+%Y-%m-%d %H:%M:%S') ❌ 机器人尚未连接 WiFi 或 发射热点，继续检测中..."
    fi

    sleep 2
done

# 启动 websocket 服务节点
roslaunch planarmwebsocketservice plan_arm_action_websocket_server.launch robot_type:=ocs2

# 定义退出时的清理操作
cleanup() {
    if [[ -n "$PLAN_PID" ]] && kill -0 "$PLAN_PID" 2>/dev/null; then
        kill "$PLAN_PID"
        echo "已杀掉进程 $PLAN_PID"
    fi
}
trap cleanup EXIT
