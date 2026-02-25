# #!/bin/bash
# export ROS_WS_PATH=/opt/ros/noetic

# source $ROS_WS_PATH/setup.bash
# source $KUAVO_ROS_CONTROL_WS_PATH/devel/setup.bash
# H12PRO_CONTROLLER_NODE_DIR=$(rospack find h12pro_controller_node)
# export PYTHONPATH=$PYTHONPATH:$H12PRO_CONTROLLER_NODE_DIR
# cd $H12PRO_CONTROLLER_NODE_DIR

# echo "current robot version: $ROBOT_VERSION"
# echo "ROS_MASTER_URI: $ROS_MASTER_URI"
# echo "ROS_IP: $ROS_IP"
# roslaunch h12pro_controller_node ocs2_robot_control_server.launch 
# roslaunch rosbridge_server rosbridge_websocket.launch port:=8080 
# # Check if both nodes are not running
# if ! rosnode list | grep -q "/h12pro_channel_publisher" && ! rosnode list | grep -q "/joy_node"; then
#     # roslaunch h12pro_controller_node h12pro_autostart.launch
#     # roslaunch h12pro_controller_node ocs2_robot_control_server.launch 
# else
#     echo "Node /h12pro_channel_publisher or /joy_node is already running"
#     echo "Please check running nodes with 'rosnode list'"
# fi





#!/bin/bash
# ============================================
# ROS Launch 自动启动脚本优化
# ============================================

# ROS 工作空间路径
export ROS_WS_PATH=/opt/ros/noetic
source $ROS_WS_PATH/setup.bash
source $KUAVO_ROS_CONTROL_WS_PATH/devel/setup.bash

# H12Pro 控制节点路径
H12PRO_CONTROLLER_NODE_DIR=$(rospack find h12pro_controller_node)
export PYTHONPATH=$PYTHONPATH:$H12PRO_CONTROLLER_NODE_DIR
cd $H12PRO_CONTROLLER_NODE_DIR

echo "Current robot version: $ROBOT_VERSION"
echo "ROS_MASTER_URI: $ROS_MASTER_URI"
echo "ROS_IP: $ROS_IP"

# =========================
# Helper 函数：检查节点是否存在
# =========================
function is_node_running() {
    local node_name="$1"
    rosnode list | grep -q "$node_name"
    return $?
}

# =========================
# 启动 ocs2_robot_control_server.launch
# =========================
if ! is_node_running "/h12pro_controller_node"; then
    echo "Starting ocs2_robot_control_server.launch..."
    roslaunch h12pro_controller_node ocs2_robot_control_server.launch &
    sleep 5  # 给节点启动时间
else
    echo "Node /h12pro_controller_node is already running"
fi

# =========================
# 启动 rosbridge_websocket.launch
# =========================
if ! is_node_running "/rosbridge_websocket"; then
    echo "Starting rosbridge_websocket.launch on port 9090..."
    roslaunch rosbridge_server rosbridge_websocket.launch port:=9090 &
    sleep 3  # 给节点启动时间
else
    echo "Node /rosbridge_websocket is already running"
fi

# =========================
# 检查关键节点
# =========================
if ! is_node_running "/h12pro_channel_publisher" && ! is_node_running "/joy_node"; then
    echo "Key nodes are not running. Consider starting autostart launch files if needed."
else
    echo "Key nodes already running. Use 'rosnode list' to check."
fi

echo "All launch files executed. Use 'rosnode list' to verify running nodes."