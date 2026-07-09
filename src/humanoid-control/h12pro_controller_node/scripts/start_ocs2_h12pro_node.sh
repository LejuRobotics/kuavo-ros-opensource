#!/bin/bash
export ROS_WS_PATH=/opt/ros/noetic

source $ROS_WS_PATH/setup.bash
source $KUAVO_ROS_CONTROL_WS_PATH/devel/setup.bash
H12PRO_CONTROLLER_NODE_DIR=$(rospack find h12pro_controller_node)
export PYTHONPATH=$PYTHONPATH:$H12PRO_CONTROLLER_NODE_DIR
cd $H12PRO_CONTROLLER_NODE_DIR

echo "current robot version: $ROBOT_VERSION"
echo "ROS_MASTER_URI: $ROS_MASTER_URI"
echo "ROS_IP: $ROS_IP"

# Check if safe to start launch.
# Condition: rosmaster not running OR all h12 nodes are absent.
# owner 模型：若 /start_way=manual 且任一 h12 节点仍在线，说明终端 launch 仍持有 H12 owner，
# 服务不应抢占，也不应把 start_way 改回 auto；只有 manual 但所有 h12 节点都不在时，
# 才认为是终端退出后的残留参数，修正回 auto 让服务恢复常驻节点树。
can_start=true

ros_node_exists() {
    rosnode list 2>/dev/null | grep -qx "$1"
}

h12_node_online() {
    local node
    for node in /h12pro_channel_publisher /joy_node /websocket_sdk_start_node; do
        if ros_node_exists "$node"; then
            return 0
        fi
    done
    return 1
}

if pgrep rosmaster > /dev/null; then
    current_start_way=$(rosparam get /start_way 2>/dev/null)

    if h12_node_online; then
        can_start=false
        if [ "$current_start_way" = "manual" ]; then
            echo "[H12] manual owner is active; service will not reclaim or change start_way"
        else
            echo "[H12] h12 node is already running; service will not start another tree"
        fi
    elif [ "$current_start_way" = "manual" ]; then
        # 终端 roslaunch 退出后可能残留 start_way=manual；确认没有 h12 owner 后，
        # 才允许服务改回 auto 并恢复常驻节点树。
        echo "[H12] correcting residual start_way: manual -> auto"
        rosparam set /start_way auto 2>/dev/null
    fi
fi

if $can_start; then
    roslaunch h12pro_controller_node h12pro_autostart.launch
else
    echo "H12 node is already running or manual owner is active"
    echo "Please check running nodes with 'rosnode list'"
fi