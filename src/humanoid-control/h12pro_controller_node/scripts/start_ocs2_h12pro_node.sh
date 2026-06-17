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

# Check if safe to start launch
# Condition: rosmaster not running OR both key nodes are absent
can_start=true

if pgrep rosmaster > /dev/null; then
    # rosmaster exists, check if either key node is already running
    if rosnode list 2>/dev/null | grep -q "/h12pro_channel_publisher" || \
       rosnode list 2>/dev/null | grep -q "/joy_node"; then
        can_start=false
    fi

    # 修正残留的 start_way 参数
    # 场景：终端 roslaunch 启动机器人后 Ctrl+C 退出，参数服务器上残留 start_way=manual，
    # 导致 monitor 读到 manual 后执行 yielding，杀掉刚启动的 h12pro 节点树
    current_start_way=$(rosparam get /start_way 2>/dev/null)
    if [ "$current_start_way" = "manual" ]; then
        echo "[Fix] Correcting residual start_way: manual -> auto"
        rosparam set /start_way auto 2>/dev/null
    fi
fi

if $can_start; then
    roslaunch h12pro_controller_node h12pro_autostart.launch
else
    echo "Node /h12pro_channel_publisher or /joy_node is already running"
    echo "Please check running nodes with 'rosnode list'"
fi