#!/bin/bash

# Kuavo Wheel Bridge Startup Script

set -e

if [ "${ROBOT_VERSION}" != "60" ]; then
    echo "ROBOT_VERSION 不是 60, 当前不是 5 代轮臂, 不启动转发节点"
    exit 1
fi

# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# 设置ROS环境
source /opt/ros/noetic/setup.bash

# 设置Python路径
export PYTHONPATH=$PROJECT_ROOT/src:$PYTHONPATH

# 切换到项目目录
cd $PROJECT_ROOT

# 从环境变量获取kuavo配置
if [[ -z "$KUAVO_MASTER_URI" ]]; then
    KUAVO_MASTER="http://169.254.128.130:11311"
    echo "KUAVO_MASTER_URI not set, using default: $KUAVO_MASTER"
else
    KUAVO_MASTER="$KUAVO_MASTER_URI"
    echo "Using KUAVO_MASTER_URI: $KUAVO_MASTER"
fi

KUAVO_SLAVE_IP="169.254.128.130"


# 默认wheel配置
WHEEL_MASTER="http://169.254.128.2:11311"

# 自动检测wheel IP和匹配网段
detect_wheel_network() {
    echo "Detecting wheel network configuration..."
    
    # 获取所有网络接口的IP地址
    local interfaces=$(ip addr show | grep -E "^[0-9]+:" | awk '{print $2}' | sed 's/://')
    
    for interface in $interfaces; do
        # 跳过lo接口
        if [[ "$interface" == "lo" ]]; then
            continue
        fi
        
        # 获取接口的IP地址
        local ip=$(ip addr show $interface | grep -E "inet " | awk '{print $2}' | cut -d'/' -f1)
        
        if [[ -n "$ip" ]]; then
            echo "Found interface $interface with IP: $ip"
            
            # 检查是否是169.254.x.x网段（wheel网络）
            if [[ "$ip" =~ ^169\.254\. ]]; then
                echo "Detected wheel network interface: $interface"
                echo "Wheel IP: $ip"
                
                # 设置wheel slave IP为当前机器在wheel网段的IP
                WHEEL_SLAVE_IP="$ip"
                echo "Configured wheel slave IP: $WHEEL_SLAVE_IP"
                return 0
            fi
        fi
    done
    
    echo "Warning: No wheel network (169.254.x.x) detected, using default configuration"
    WHEEL_SLAVE_IP="169.254.128.130"
    return 0
}

# 检测wheel网络配置
detect_wheel_network

# 启动车辆桥接器
echo "Starting Kuavo Wheel Bridge..."
echo "Project root: $PROJECT_ROOT"
echo "Script path: $SCRIPT_DIR/wheel_bridge.py"
echo "Configuration:"
echo "  Kuavo Master: $KUAVO_MASTER"
echo "  Wheel Master: $WHEEL_MASTER"
echo "  Kuavo Slave IP: $KUAVO_SLAVE_IP"
echo "  Wheel Slave IP: $WHEEL_SLAVE_IP"

if rostopic list >/dev/null 2>&1; then
    echo "roscore 已启动"
else
    echo "roscore 未启动"
    roscore &
fi


# 使用检测到的参数启动桥接器
python3 $SCRIPT_DIR/wheel_bridge.py \
    --kuavo-master "$KUAVO_MASTER" \
    --wheel-master "$WHEEL_MASTER" \
    --kuavo-slave-ip "$KUAVO_SLAVE_IP" \
    --wheel-slave-ip "$WHEEL_SLAVE_IP" 