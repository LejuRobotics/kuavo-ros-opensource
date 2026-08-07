#! /bin/bash
set -e

if systemctl is-active --quiet roban_joy_monitor.service; then
    echo "服务 roban_joy_monitor.service 已开启，正在停止..."
    sudo systemctl stop roban_joy_monitor.service
    sudo systemctl disable roban_joy_monitor.service
    echo "服务 roban_joy_monitor.service 已停止。"
else
    echo "服务 roban_joy_monitor.service 未开启。"
fi

if systemctl is-active --quiet h12pro_monitor.service; then
    echo "服务 h12pro_monitor.service 已开启，正在停止..."
    sudo systemctl stop h12pro_monitor.service
    sudo systemctl disable h12pro_monitor.service
    echo "服务 h12pro_monitor.service 已停止。"
else
    echo "服务 h12pro_monitor.service 未开启。"
fi

if systemctl is-active --quiet lejulab_joy_monitor.service; then
    echo "服务 lejulab_joy_monitor.service 已开启，正在停止..."
    sudo systemctl stop lejulab_joy_monitor.service
    sudo systemctl disable lejulab_joy_monitor.service
    echo "服务 lejulab_joy_monitor.service 已停止。"
else
    echo "服务 lejulab_joy_monitor.service 未开启。"
fi

if systemctl is-active --quiet ocs2_h12pro_monitor.service; then
    echo "服务 ocs2_h12pro_monitor.service 已开启，正在停止..."
    sudo systemctl stop ocs2_h12pro_monitor.service
    sudo systemctl disable ocs2_h12pro_monitor.service
    echo "服务 ocs2_h12pro_monitor.service 已停止。"
else
    echo "服务 ocs2_h12pro_monitor.service 未开启。"
fi

while true; do
    echo "请选择控制方案 (1: ocs2, 2: rl, 3: multi)。若为 rl，请先修改 ROBOT_VERSION=46，并将正确的仓库路径修改在脚本中，再运行该脚本:"
    read -r user_input
    if [ "$user_input" = "1" ]; then
        KUAVO_CONTROL_SCHEME=ocs2
        echo "已选择: ocs2"
        break
    elif [ "$user_input" = "2" ]; then
        KUAVO_CONTROL_SCHEME=rl
        echo "已选择: rl"
        break
    elif [ "$user_input" = "3" ]; then
        KUAVO_CONTROL_SCHEME=multi
        echo "已选择: multi"
        break
    else
        echo "输入无效，请输入1、2或3。"
    fi
done

KUAVO_RL_WS_PATH="/home/lab/kuavo-RL/kuavo-robot-deploy" # 在没合并到 kuavo-ros-control 的之前，先固定路径或手动修改
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
H12PRO_CONTROLLER_NODE_DIR=$(dirname $SCRIPT_DIR)
SERVICE_DIR=$(dirname $SCRIPT_DIR)/services
OCS2_H12PRO_MONITOR_SERVICE=$SERVICE_DIR/ocs2_h12pro_monitor.service
START_OCS2_H12PRO_NODE=$SCRIPT_DIR/start_ocs2_h12pro_node.sh
MONITOR_OCS2_H12PRO=$SCRIPT_DIR/monitor_ocs2_h12pro.sh
KUAVO_ROS_CONTROL_WS_PATH=$(dirname $(dirname $(dirname $(dirname $SCRIPT_DIR))))
NOITOM_HI5_HAND_UDP_PYTHON=$KUAVO_ROS_CONTROL_WS_PATH/src/manipulation_nodes/noitom_hi5_hand_udp_python
KUAVO_REMOTE_PATH=$(dirname $SCRIPT_DIR)/lib/kuavo_remote
ROBOT_VERSION=$ROBOT_VERSION
INSTALLED_DIR=$KUAVO_ROS_CONTROL_WS_PATH/installed
RL_INSTALLED_DIR=$KUAVO_RL_WS_PATH/installed
# 架构：仅用于 arm64 额外编译 hardware_node
ARCH=$(uname -m)
IS_ARM64=0
if [ "$ARCH" = "aarch64" ] || [ "$ARCH" = "arm64" ]; then
    IS_ARM64=1
fi

# 系统版本：Ubuntu >= 24.04 使用 requirements.noble.txt + PEP 668 兼容安装
UBUNTU_VERSION_ID=""
if [ -r /etc/os-release ]; then
    # shellcheck disable=SC1091
    . /etc/os-release
    UBUNTU_VERSION_ID="${VERSION_ID:-}"
fi
IS_UBUNTU_24_04_OR_NEWER=0
if [ -n "$UBUNTU_VERSION_ID" ]; then
    if dpkg --compare-versions "$UBUNTU_VERSION_ID" ge "24.04"; then
        IS_UBUNTU_24_04_OR_NEWER=1
    fi
fi
echo "Current architecture: $ARCH (IS_ARM64=$IS_ARM64)"
echo "Ubuntu VERSION_ID: ${UBUNTU_VERSION_ID:-unknown} (IS_UBUNTU_24_04_OR_NEWER=$IS_UBUNTU_24_04_OR_NEWER)"

# 解析要安装的 requirements 文件：
# - Ubuntu 24.04+：优先同目录 requirements.noble.txt
# - 其他系统（含传统 x86/20.04）：使用传入的原 requirements.txt
resolve_requirements_file() {
    local req_file="$1"
    local req_dir
    local noble_file

    if [ "$IS_UBUNTU_24_04_OR_NEWER" -eq 1 ]; then
        req_dir="$(dirname "$req_file")"
        if [ "$req_dir" = "." ]; then
            noble_file="requirements.noble.txt"
        else
            noble_file="${req_dir}/requirements.noble.txt"
        fi
        if [ -f "$noble_file" ]; then
            echo "$noble_file"
            return 0
        fi
        echo "warning: Ubuntu ${UBUNTU_VERSION_ID} 未找到 $noble_file，回退到 $req_file" >&2
    fi
    echo "$req_file"
}

pip_install_requirements() {
    local req_file="$1"
    local resolved_req

    resolved_req="$(resolve_requirements_file "$req_file")"
    if [ ! -f "$resolved_req" ]; then
        echo "requirements 文件不存在，跳过: $resolved_req"
        return 0
    fi

    echo "Installing Python deps from: $resolved_req"
    if [ "$IS_UBUNTU_24_04_OR_NEWER" -eq 1 ]; then
        # PEP 668: 系统 Python 需显式允许（部署脚本场景）
        python3 -m pip install -r "$resolved_req" --break-system-packages
    else
        # 传统环境（如 x86 + Ubuntu 20.04）保持原行为
        pip3 install -r "$resolved_req"
    fi
}

cd $H12PRO_CONTROLLER_NODE_DIR
pip_install_requirements "requirements.txt"
pip_install_requirements "$NOITOM_HI5_HAND_UDP_PYTHON/requirements.txt"

echo "KUAVO_ROS_CONTROL_WS_PATH: $KUAVO_ROS_CONTROL_WS_PATH"
echo "SERVICE_DIR: $SERVICE_DIR"
echo "MONITOR_OCS2_H12PRO: $MONITOR_OCS2_H12PRO"
echo "KUAVO_REMOTE_PATH: $KUAVO_REMOTE_PATH"

if [ "$KUAVO_CONTROL_SCHEME" = "rl" ]; then
    cd $KUAVO_RL_WS_PATH
    catkin config -DCMAKE_ASM_COMPILER=/usr/bin/as -DCMAKE_BUILD_TYPE=Release
    if [ -d "$RL_INSTALLED_DIR" ] && [ -f "$RL_INSTALLED_DIR/setup.bash" ]; then
        echo "Sourcing existing installation..."
        source $RL_INSTALLED_DIR/setup.bash
    fi
    catkin build humanoid_controllers
    if [ "$IS_ARM64" -eq 1 ]; then
        echo "arm64: building hardware_node after humanoid_controllers (rl)..."
        catkin build hardware_node
    fi
fi

cd $KUAVO_ROS_CONTROL_WS_PATH
catkin config -DCMAKE_ASM_COMPILER=/usr/bin/as -DCMAKE_BUILD_TYPE=Release
if [ -d "$INSTALLED_DIR" ] && [ -f "$INSTALLED_DIR/setup.bash" ]; then
    echo "Sourcing existing installation..."
    source $INSTALLED_DIR/setup.bash
fi
catkin build humanoid_controllers
if [ "$IS_ARM64" -eq 1 ]; then
    echo "arm64: building hardware_node after humanoid_controllers..."
    catkin build hardware_node
fi
catkin build h12pro_controller_node
catkin build humanoid_plan_arm_trajectory
catkin build kuavo_ros_interfaces

if ls /dev | grep usb_remote; then
  echo "Device file exists."
else
  echo "Device file does not exist."
  cd $KUAVO_REMOTE_PATH
  sudo chmod +x creat_remote_udev_rule.sh
  sudo ./creat_remote_udev_rule.sh
fi

while true; do
    echo "是否需要加载遥控器查看 log 使用串口的 udev 规则？确认有接线才可以使用。(y/n): "
    read -r load_h12_log_channel
    if [[ "$load_h12_log_channel" == "y" || "$load_h12_log_channel" == "Y" ]]; then
        if ls /dev | grep H12_log_channel; then
            echo "遥控器设备文件已存在，无需加载规则。"
        else
            echo "正在加载遥控器 udev 规则..."
            cd $SCRIPT_DIR
            sudo chmod +x load_h12_log_serial_rule.sh
            sudo ./load_h12_log_serial_rule.sh                        
        fi
        break
    elif [[ "$load_h12_log_channel" == "n" || "$load_h12_log_channel" == "N" ]]; then
        echo "跳过加载遥控器 udev 规则。"
        break
    else
        echo "输入无效，请输入 y 或 n。"
    fi
done

echo "Current robot version: $ROBOT_VERSION"

if [ -z "$ROS_MASTER_URI" ]; then
    ROS_MASTER_URI="http://localhost:11311"
    echo "ROS_MASTER_URI is empty, using default: $ROS_MASTER_URI"
fi

if [ -z "$ROS_IP" ]; then
    ROS_IP="127.0.0.1"
    echo "ROS_IP is empty, using default: $ROS_IP"
fi

if [ -z "$ROS_HOSTNAME" ]; then
    if [ "$ROS_MASTER_URI" == "http://kuavo_master:11311" ]; then
        ROS_HOSTNAME=kuavo_master  
        echo "ROS_MASTER_URI is http://kuavo_master:11311, using ROS_HOSTNAME: $ROS_HOSTNAME"
    fi
fi

echo "Current ROS_MASTER_URI: $ROS_MASTER_URI"
echo "Current ROS_IP: $ROS_IP"
echo "Current ROS_HOSTNAME:$ROS_HOSTNAME"

# 询问用户选择楼梯建图相机类型
echo "请问机器人楼梯建图的相机类型为："
echo "1. 奥比中光"
echo "2. D435"
echo -n "请选择 (默认为奥比中光，直接回车选择默认): "
read -r camera_choice

case $camera_choice in
    2)
        STAIR_DETECTION_CAMERA="d435"
        echo "已选择: D435"
        ;;
    1|""|*)
        STAIR_DETECTION_CAMERA="orbbec"
        echo "已选择: 奥比中光"
        ;;
esac

echo "楼梯建图相机类型: $STAIR_DETECTION_CAMERA"

# 询问用户选择 VR 拉起方式
echo "请选择 VR 拉起方式："
echo "1. 仅 VR 控制"
echo "2. VR 控制 + Orbbec 视频回传"
echo -n "请选择 (默认为仅 VR 控制，直接回车选择默认): "
read -r vr_launch_choice

case $vr_launch_choice in
    2)
        LAUNCH_VR_REMOTE_CONTROL_CMD="roslaunch noitom_hi5_hand_udp_python launch_quest3_ik_videostream_orbbec.launch"
        echo -n "请输入 Quest3 IP (可选，直接回车不指定): "
        read -r quest3_ip_address
        if [ -n "$quest3_ip_address" ]; then
            # 写入用户配置 yaml，运行时统一读取该文件，避免硬编码进 systemd 环境变量
            USER_VR_CONFIG="$HOME/.config/lejuconfig/h12_vr_launch.yaml"
            if [ -f "$USER_VR_CONFIG" ]; then
                if grep -q '^ip_address:' "$USER_VR_CONFIG"; then
                    sed -i "s|^ip_address:.*|ip_address: \"$quest3_ip_address\"|" "$USER_VR_CONFIG"
                else
                    echo "ip_address: \"$quest3_ip_address\"" >> "$USER_VR_CONFIG"
                fi
            else
                mkdir -p "$HOME/.config/lejuconfig"
                echo "ip_address: \"$quest3_ip_address\"" > "$USER_VR_CONFIG"
            fi
            echo "已写入 $USER_VR_CONFIG"
        fi
        echo "已选择: VR 控制 + Orbbec 视频回传"
        ;;
    1|""|*)
        LAUNCH_VR_REMOTE_CONTROL_CMD="roslaunch noitom_hi5_hand_udp_python launch_quest3_ik.launch"
        echo "已选择: 仅 VR 控制"
        ;;
esac

echo "VR 拉起命令: $LAUNCH_VR_REMOTE_CONTROL_CMD"

sed -i "s|^Environment=ROS_MASTER_URI=.*|Environment=ROS_MASTER_URI=$ROS_MASTER_URI|" $OCS2_H12PRO_MONITOR_SERVICE
sed -i "s|^Environment=ROS_IP=.*|Environment=ROS_IP=$ROS_IP|" $OCS2_H12PRO_MONITOR_SERVICE
sed -i "s|^Environment=KUAVO_CONTROL_SCHEME=.*|Environment=KUAVO_CONTROL_SCHEME=$KUAVO_CONTROL_SCHEME|" $OCS2_H12PRO_MONITOR_SERVICE
sed -i "s|^Environment=ROS_HOSTNAME=.*|Environment=ROS_HOSTNAME=$ROS_HOSTNAME|" $OCS2_H12PRO_MONITOR_SERVICE
sed -i "s|^Environment=KUAVO_ROS_CONTROL_WS_PATH=.*|Environment=KUAVO_ROS_CONTROL_WS_PATH=$KUAVO_ROS_CONTROL_WS_PATH|" $OCS2_H12PRO_MONITOR_SERVICE
sed -i "s|^Environment=KUAVO_RL_WS_PATH=.*|Environment=KUAVO_RL_WS_PATH=$KUAVO_RL_WS_PATH|" $OCS2_H12PRO_MONITOR_SERVICE
sed -i "s|^Environment=ROBOT_VERSION=.*|Environment=ROBOT_VERSION=$ROBOT_VERSION|" $OCS2_H12PRO_MONITOR_SERVICE
sed -i "s|^Environment=NODE_SCRIPT=.*|Environment=NODE_SCRIPT=$START_OCS2_H12PRO_NODE|" $OCS2_H12PRO_MONITOR_SERVICE
sed -i "s|^Environment=STAIR_DETECTION_CAMERA=.*|Environment=STAIR_DETECTION_CAMERA=$STAIR_DETECTION_CAMERA|" $OCS2_H12PRO_MONITOR_SERVICE
if grep -q '^Environment="*LAUNCH_VR_REMOTE_CONTROL_CMD=' $OCS2_H12PRO_MONITOR_SERVICE; then
    sed -i "s|^Environment=\"*LAUNCH_VR_REMOTE_CONTROL_CMD=.*|Environment=\"LAUNCH_VR_REMOTE_CONTROL_CMD=$LAUNCH_VR_REMOTE_CONTROL_CMD\"|" $OCS2_H12PRO_MONITOR_SERVICE
else
    sed -i "/^Environment=STAIR_DETECTION_CAMERA=.*/a Environment=\"LAUNCH_VR_REMOTE_CONTROL_CMD=$LAUNCH_VR_REMOTE_CONTROL_CMD\"" $OCS2_H12PRO_MONITOR_SERVICE
fi
sed -i "s|^ExecStart=.*|ExecStart=$MONITOR_OCS2_H12PRO|" $OCS2_H12PRO_MONITOR_SERVICE

sudo cp $OCS2_H12PRO_MONITOR_SERVICE /etc/systemd/system/
sudo systemctl daemon-reload

# 同步写入 bashrc，确保终端 roslaunch 也能读取 KUAVO_CONTROL_SCHEME
if grep -q "^export KUAVO_CONTROL_SCHEME=" ~/.bashrc; then
    sed -i "s|^export KUAVO_CONTROL_SCHEME=.*|export KUAVO_CONTROL_SCHEME=$KUAVO_CONTROL_SCHEME|" ~/.bashrc
else
    echo "export KUAVO_CONTROL_SCHEME=$KUAVO_CONTROL_SCHEME" >> ~/.bashrc
fi
echo "已将 KUAVO_CONTROL_SCHEME=$KUAVO_CONTROL_SCHEME 写入 ~/.bashrc"

sudo apt-get install tmux

if [ ! -f ~/.tmux.conf ]; then
    touch ~/.tmux.conf
fi

if ! grep -q "set-option -g default-shell /bin/bash" ~/.tmux.conf; then
    echo "set-option -g default-shell /bin/bash" >> ~/.tmux.conf
    echo "set-option -g mouse on" >> ~/.tmux.conf
fi


echo "stop all ros node"
sudo pkill ros -f || true
sudo pkill -f h12pro_channel_publisher || true
echo "stop all ros node successfully"

sudo systemctl start ocs2_h12pro_monitor.service
sudo systemctl enable ocs2_h12pro_monitor.service

echo "h12pro monitor service deploy successfully"
