#!/bin/bash

# Docker 环境检测函数
is_running_in_docker() {
    # 方法1: 检查 /.dockerenv 文件
    if [[ -f /.dockerenv ]]; then
        return 0
    fi
    
    # 方法2: 检查 /proc/1/cgroup 中是否包含 docker
    if [[ -f /proc/1/cgroup ]] && grep -q docker /proc/1/cgroup; then
        return 0
    fi
    
    # 方法3: 检查容器运行时环境变量
    if [[ -n "$DOCKER_CONTAINER" ]] || [[ -n "$container" ]]; then
        return 0
    fi
    
    return 1
}

# 要检测的 launch 文件名
LAUNCH1="humanoid_plan_arm_trajectory.launch"
LAUNCH2="plan_arm_action_websocket_server.launch"

# 检测 launch 进程是否已运行（避免重复启动）
LAUNCH1_RUNNING=false
LAUNCH2_RUNNING=false

for LAUNCH in "$LAUNCH1" "$LAUNCH2"
do
    PIDS=$(ps aux | grep "[r]oslaunch" | grep "$LAUNCH" | awk '{print $2}')
    if [[ -n "$PIDS" ]]; then
        echo "检测到 $LAUNCH 正在运行"
        if [[ "$LAUNCH" == "$LAUNCH1" ]]; then
            LAUNCH1_RUNNING=true
        else
            LAUNCH2_RUNNING=true
        fi
    fi
done

# 获取脚本自身所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "SCRIPT_DIR: $SCRIPT_DIR"

# 从脚本目录向上查找包含 devel/setup.bash 的工作空间根目录
current_path="$SCRIPT_DIR"
REPO_ROOT=""
while [[ "$current_path" != "/" ]]; do
    if [[ -f "$current_path/devel/setup.bash" ]] || [[ -f "$current_path/install/setup.bash" ]]; then
        REPO_ROOT="$current_path"
        echo "找到工作空间根目录: $REPO_ROOT"
        cd "$REPO_ROOT"
        break
    fi
    current_path=$(dirname "$current_path")
done

# 检查是否找到工作空间根目录
if [[ -z "$REPO_ROOT" ]]; then
    echo "错误: 未找到包含 devel/setup.bash 或 install/setup.bash 的工作空间目录"
    exit 1
fi

source /opt/ros/noetic/setup.bash --extend
source $REPO_ROOT/devel/setup.bash

# 根据 LAUNCH1_RUNNING 和 LAUNCH2_RUNNING 分别决定启动哪些节点

# 启动 humanoid_plan_arm_trajectory（仅当 LAUNCH1 未运行时）
if [ "$LAUNCH1_RUNNING" = "false" ]; then
    echo "启动 humanoid_plan_arm_trajectory 节点"
    roslaunch humanoid_plan_arm_trajectory humanoid_plan_arm_trajectory.launch &
    PLAN_PID=$!

    # 检测动作执行节点启动
    echo "正在启动动作执行节点..."
    while ! rosnode list | grep -q "autostart_arm_trajectory_bezier_demo"; do
        sleep 1
    done
    echo "动作执行节点已启动。"
else
    echo "humanoid_plan_arm_trajectory 已在运行，跳过启动"
    PLAN_PID=""
fi

# 启动 h12pro_controller_node（仅当 LAUNCH2 未运行时）
if [ "$LAUNCH2_RUNNING" = "false" ]; then
    echo "启动 h12pro_controller_node 节点"
    roslaunch h12pro_controller_node kuavo_humanoid_sdk_ws_srv.launch &
    CONTROLLER_PID=$!

    # 检测 h12pro_controller_node 启动
    echo "正在启动 h12pro_controller_node..."
    sleep 3
    echo "h12pro_controller_node 已启动。"
else
    echo "h12pro_controller_node 相关服务已在运行，跳过启动"
    CONTROLLER_PID=""
fi

# 检测运行环境
if is_running_in_docker; then
    echo "🐳 检测到正在 Docker 容器中运行，跳过 WiFi 连接检测"
else
    echo "🖥️  检测到在宿主机环境中运行，开始 WiFi 连接检测"
    
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
fi

echo "正在启动太极触发节点..."
roslaunch taiji_trigger_node taiji_trigger.launch &

# 启动 websocket 服务节点
if [ "$LAUNCH2_RUNNING" = "false" ]; then
    echo "正在启动 websocket 服务节点..."
    roslaunch planarmwebsocketservice plan_arm_action_websocket_server.launch robot_type:=ocs2 camera_type:=$CAMERA_TYPE
else
    echo "websocket 服务节点已在运行，跳过启动"
fi

# 定义退出时的清理操作
cleanup() {
    if [[ -n "$CONTROLLER_PID" ]] && kill -0 "$CONTROLLER_PID" 2>/dev/null; then
        kill "$CONTROLLER_PID"
        echo "已杀掉 h12pro_controller_node 进程 $CONTROLLER_PID"
    fi
    if [[ -n "$PLAN_PID" ]] && kill -0 "$PLAN_PID" 2>/dev/null; then
        kill "$PLAN_PID"
        echo "已杀掉进程 $PLAN_PID"
    fi
}

trap cleanup EXIT
