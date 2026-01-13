#!/bin/bash

# Kuavo ROS Control 环境一键安装脚本
# 基于 Dockerfile 配置生成
# 适用于 Ubuntu 20.04 系统

set -e  # 遇到错误立即退出

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 日志函数
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 检查是否为root用户
check_root() {
    if [[ $EUID -eq 0 ]]; then
        log_warning "检测到以root用户运行，建议使用普通用户并配置sudo权限"
    fi
}

# 检查系统版本
check_system() {
    if [[ ! -f /etc/os-release ]]; then
        log_error "无法检测系统版本"
        exit 1
    fi
    
    source /etc/os-release
    if [[ "$ID" != "ubuntu" ]] || [[ "$VERSION_ID" != "20.04" ]]; then
        log_warning "此脚本专为Ubuntu 20.04设计，当前系统: $ID $VERSION_ID"
        read -p "是否继续安装? (y/N): " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 1
        fi
    fi
}

# 更新系统包管理器
update_system() {
    log_info "更新系统包管理器..."
    sudo apt-get update -y
    sudo apt-get install ca-certificates -y
    log_success "系统包管理器更新完成"
}

# 配置软件源
configure_sources() {
    log_info "配置软件源..."
    
    # 备份原始sources.list
    sudo cp /etc/apt/sources.list /etc/apt/sources.list.backup
    
    # 配置USTC镜像源
    sudo sh -c 'echo "deb https://mirrors.ustc.edu.cn/ubuntu/ focal main restricted universe multiverse
deb https://mirrors.ustc.edu.cn/ubuntu/ focal-updates main restricted universe multiverse
deb https://mirrors.ustc.edu.cn/ubuntu/ focal-backports main restricted universe multiverse
deb https://mirrors.ustc.edu.cn/ubuntu/ focal-security main restricted universe multiverse" > /etc/apt/sources.list'
    
    # 配置时区
    sudo ln -snf /usr/share/zoneinfo/Asia/Shanghai /etc/localtime
    echo 'Asia/Shanghai' | sudo tee /etc/timezone
    
    sudo DEBIAN_FRONTEND=noninteractive apt-get install -y tzdata
    sudo apt-get install -y dirmngr
    
    # 配置ROS源
    sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'

    curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -

    sudo apt-get update -y
    log_success "软件源配置完成"
}

# 安装基础工具
install_basic_tools() {
    log_info "安装基础开发工具..."
    sudo apt-get update -y
    sudo apt-get install -y git wget curl vim nano htop build-essential cmake pkg-config
    log_success "基础工具安装完成"
}

# 安装zsh和oh-my-zsh
install_zsh() {
    log_info "安装zsh和oh-my-zsh..."
    
    # 安装zsh
    sudo apt-get install -y zsh
    
    # 安装oh-my-zsh
    if [[ ! -d "$HOME/.oh-my-zsh" ]]; then
        git clone https://github.com/ohmyzsh/ohmyzsh.git ~/.oh-my-zsh
    fi
    
    # 安装zsh插件
    ZSH_CUSTOM="$HOME/.oh-my-zsh/custom"
    mkdir -p "$ZSH_CUSTOM/plugins"
    
    if [[ ! -d "$ZSH_CUSTOM/plugins/zsh-autosuggestions" ]]; then
        git clone https://github.com/zsh-users/zsh-autosuggestions.git "$ZSH_CUSTOM/plugins/zsh-autosuggestions"
    fi
    
    if [[ ! -d "$ZSH_CUSTOM/plugins/zsh-syntax-highlighting" ]]; then
        git clone https://github.com/zsh-users/zsh-syntax-highlighting.git "$ZSH_CUSTOM/plugins/zsh-syntax-highlighting"
    fi
    
    # 配置zsh
    sh -c "$(wget -O- https://github.com/deluan/zsh-in-docker/releases/download/v1.1.5/zsh-in-docker.sh)" -- \
        -t ys \
        -p git \
        -p zsh-syntax-highlighting \
        -p zsh-autosuggestions
    
    log_success "zsh和oh-my-zsh安装完成"
}

# 安装Gnome终端支持
install_gnome_terminal() {
    log_info "安装Gnome终端支持..."
    sudo apt-get update -y
    sudo apt-get install -y gnome-terminal dbus-x11 libcanberra-gtk-module libcanberra-gtk3-module
    log_success "Gnome终端支持安装完成"
}

# 安装ROS Noetic
install_ros() {
    log_info "安装ROS Noetic..."
    sudo apt-get update -y
    sudo apt-get install -y wget python3 python3-yaml python3-distro
    sudo apt-get install -y ros-noetic-ros-base python3-rosdep python3-rosinstall \
        python3-rosinstall-generator python3-wstool python3-catkin-tools build-essential
    sudo apt-get install -y ros-noetic-desktop-full ros-noetic-behaviortree-cpp-v3
    sudo apt-get install -y ros-noetic-plotjuggler*
    sudo apt-get install -y ros-noetic-interactive-markers ros-noetic-tf ros-noetic-tf2-ros ros-noetic-urdf \
        ros-noetic-kdl-parser ros-noetic-robot-state-publisher ros-noetic-rqt-graph
    sudo apt-get install -y ros-noetic-realtime-tools ros-noetic-ros-control ros-noetic-xacro
    sudo apt-get install -y ros-noetic-apriltag*
    sudo apt-get install -y ros-noetic-joy
    sudo apt-get install libgeographic-dev ros-noetic-geographic* -y

    # 配置ROS环境
    echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc
    echo 'source /opt/ros/noetic/setup.zsh' >> ~/.zshrc
    
    # 初始化rosdep
    sudo rosdep init || true
    rosdep update || true
    
    log_success "ROS Noetic安装完成"
}


# 安装Kuavo相关库
install_kuavo_libs() {
    log_info "安装Kuavo相关库..."
    sudo apt-get update -y
    sudo apt-get install -y libeigen3-dev libboost-all-dev
    sudo apt-get install -y libyaml-cpp-dev libtinyxml2-dev
    sudo apt-get install -y libncurses5-dev libncursesw5-dev
    sudo apt-get install -y libmodbus-dev
    sudo apt-get install -y libprotobuf-c-dev
    sudo apt-get install -y liblcm-dev libgflags-dev libgoogle-glog-dev liblmdb-dev
    log_success "Kuavo相关库安装完成"
}

# 安装LCM库
install_lcm() {
    log_info "安装LCM库..."
    sudo apt-get update -y
    sudo apt-get install -y build-essential libglib2.0-dev git cmake
    
    if [[ ! -d "lcm" ]]; then
        git clone https://github.com/lcm-proj/lcm.git
    fi
    
    cd lcm
    sed -i '/find_package(GLib2 REQUIRED)/a find_package(Python3 3.8 REQUIRED COMPONENTS Interpreter Development)' CMakeLists.txt
    mkdir -p build && cd build
    cmake ..
    make -j$(nproc)
    sudo make install
    cd ../..
    
    sudo ln -sf /usr/lib/x86_64-linux-gnu/liblcm.so /usr/local/lib/liblcm.so
    log_success "LCM库安装完成"
}

# 安装Drake
install_drake() {
    log_info "安装Drake机器人学库..."
    sudo apt-get update -y
    sudo apt-get install --no-install-recommends ca-certificates gnupg lsb-release wget -y
    
    wget -qO- https://drake-apt.csail.mit.edu/drake.asc | gpg --dearmor - | sudo tee /etc/apt/trusted.gpg.d/drake.gpg >/dev/null
    echo "deb [arch=amd64] https://drake-apt.csail.mit.edu/$(lsb_release -cs) $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/drake.list >/dev/null
    
    sudo apt-get update -y
    sudo apt-get install --no-install-recommends drake-dev=1.19.0-1 -y
    
    # 配置Drake环境
    echo 'export PATH="/opt/drake/bin${PATH:+:${PATH}}"' >> ~/.zshrc
    echo "export PYTHONPATH=\"/opt/drake/lib/python\$(python3 -c 'import sys; print(\"{0}.{1}\".format(*sys.version_info))')/site-packages\${PYTHONPATH:+:\${PYTHONPATH}}\"" >> ~/.zshrc
    echo 'export PATH="/opt/drake/bin${PATH:+:${PATH}}"' >> ~/.bashrc
    echo "export PYTHONPATH=\"/opt/drake/lib/python\$(python3 -c 'import sys; print(\"{0}.{1}\".format(*sys.version_info))')/site-packages\${PYTHONPATH:+:\${PYTHONPATH}}\"" >> ~/.bashrc
    echo 'export LD_LIBRARY_PATH=/opt/drake/lib/:$LD_LIBRARY_PATH' >> ~/.bashrc
    echo 'export LD_LIBRARY_PATH=/opt/drake/lib/:$LD_LIBRARY_PATH' >> ~/.zshrc
    
    log_success "Drake安装完成"
}

# 安装CasADi
install_casadi() {
    log_info "安装CasADi优化库..."
    sudo apt-get update -y
    sudo apt-get install -y gcc g++ gfortran git cmake liblapack-dev pkg-config --install-recommends
    
    if [[ ! -d "casadi" ]]; then
        git clone https://github.com/casadi/casadi.git
    fi
    
    cd casadi
    git checkout 3.5.5
    mkdir -p build && cd build
    cmake .. -DWITH_EXAMPLES=OFF
    make -j$(nproc)
    sudo make install
    cd ../..
    
    log_success "CasADi安装完成"
}

# 安装OCS2相关依赖
install_ocs2_deps() {
    log_info "安装OCS2相关依赖..."
    sudo apt-get update -y
    sudo apt-get install -y git
    sudo apt-get install -y liburdfdom-dev liboctomap-dev libassimp-dev ros-noetic-rqt-multiplot ros-noetic-grid-map-rviz-plugin
    
    # 添加ROS快照仓库
    sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key 4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA
    sudo sh -c 'echo "deb http://snapshots.ros.org/noetic/2024-10-31/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-snapshots.list'
    sudo apt-get update -y
    
    # 安装特定版本的pinocchio
    sudo apt install ros-noetic-pinocchio=2.6.21-1focal.20240830.092123 --reinstall -y
    sudo apt install ros-noetic-hpp-fcl-dbgsym ros-noetic-hpp-fcl -y
    
    log_success "OCS2相关依赖安装完成"
}

# 安装MuJoCo
install_mujoco() {
    log_info "安装MuJoCo物理仿真..."
    sudo apt-get update -q
    sudo DEBIAN_FRONTEND=noninteractive apt-get install -y \
        curl git libgl1-mesa-dev libgl1-mesa-glx libglew-dev libosmesa6-dev \
        software-properties-common net-tools vim virtualenv wget xpra \
        xserver-xorg-dev libglfw3-dev
    
    # 下载并安装MuJoCo
    mkdir -p ~/.mujoco
    if [[ ! -d "~/.mujoco/mujoco210" ]]; then
        wget https://github.com/google-deepmind/mujoco/releases/download/2.1.0/mujoco210-linux-x86_64.tar.gz -O mujoco.tar.gz
        tar -xf mujoco.tar.gz -C ~/.mujoco
        rm mujoco.tar.gz
    fi
    
    # 配置MuJoCo环境
    echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/root/.mujoco/mujoco210/bin' >> ~/.bashrc
    echo 'export PATH=$LD_LIBRARY_PATH:$PATH' >> ~/.bashrc
    echo 'export LD_PRELOAD=$LD_PRELOAD:/usr/lib/x86_64-linux-gnu/libGLEW.so' >> ~/.bashrc
    echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/root/.mujoco/mujoco210/bin' >> ~/.zshrc
    echo 'export PATH=$LD_LIBRARY_PATH:$PATH' >> ~/.zshrc
    echo 'export LD_PRELOAD=$LD_PRELOAD:/usr/lib/x86_64-linux-gnu/libGLEW.so' >> ~/.zshrc
    
    log_success "MuJoCo安装完成"
}


# 安装其他工具
install_other_tools() {
    log_info "安装其他工具..."
    sudo apt update -y
    sudo apt install -y python3-tk glpk-utils libglpk-dev
    
    # 安装VR运动捕捉相关Python包
    pip3 install scipy==1.10.1 numpy-quaternion==2023.0.4 protobuf==5.27.2
    
    log_success "其他工具安装完成"
}

configure_docker() {
    log_info "配置Docker..."
    sudo apt-get update
    # Add Docker's official GPG key:
    sudo apt-get install ca-certificates curl gnome-terminal -y
    sudo install -m 0755 -d /etc/apt/keyrings
    sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
    sudo chmod a+r /etc/apt/keyrings/docker.asc

    # Add the repository to Apt sources:
    echo \
    "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
    $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
    sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
    sudo apt-get update
    sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin -y
    sudo groupadd docker
    sudo usermod -aG docker ${USER}
    sudo service docker restart
    newgrp docker

    log_success "Docker配置完成"
}

# 配置环境变量
configure_environment() {
    log_info "配置环境变量..."
    
    sudo mkdir -p /var/ocs2/
    sudo chmod 777 /var/ocs2/

    # 设置语言环境
    export LANG=C.UTF-8
    export LC_ALL=C.UTF-8
    echo 'export LANG=C.UTF-8' >> ~/.bashrc
    echo 'export LC_ALL=C.UTF-8' >> ~/.bashrc
    echo 'export LANG=C.UTF-8' >> ~/.zshrc
    echo 'export LC_ALL=C.UTF-8' >> ~/.zshrc
    
    # 设置机器人版本
    echo 'export ROBOT_VERSION=45' >> ~/.bashrc
    echo 'export ROBOT_VERSION=45' >> ~/.zshrc
    
    ./install_openvino.sh
    ./config_ccache.sh
    configure_docker
    log_success "环境变量配置完成"
}

# 清理缓存
cleanup() {
    log_info "清理安装缓存..."
    sudo apt clean && sudo apt autoclean
    sudo rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*
    log_success "清理完成"
}

# 主函数
main() {
    log_info "开始安装Kuavo ROS Control环境..."
    log_info "此过程可能需要较长时间，请耐心等待..."
    
    check_root
    check_system
    
    update_system
    configure_sources
    install_basic_tools
    install_zsh
    install_gnome_terminal
    install_ros
    install_kuavo_libs
    install_lcm
    install_drake
    install_casadi
    install_ocs2_deps
    install_mujoco
    install_other_tools
    configure_environment
    cleanup
    
    log_success "Kuavo ROS Control环境安装完成！"
    log_info "请重新登录或执行 'source ~/.bashrc' 来加载环境变量"
    log_info "建议重启终端以确保所有配置生效"
}

# 错误处理
trap 'log_error "安装过程中发生错误，请检查上述输出信息"; exit 1' ERR

# 执行主函数
main "$@"

