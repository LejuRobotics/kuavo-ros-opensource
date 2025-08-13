#!/bin/bash

set -e

# Function to print colored output
print_info() {
    echo -e "\033[1;34m[INFO] $1\033[0m"
}

print_error() {
    echo -e "\033[1;31m[ERROR] $1\033[0m"
}

print_success() {
    echo -e "\033[1;32m[SUCCESS] $1\033[0m"
}

# Configure PIP
setup_pip() {
    print_info "配置 PIP 镜像源..."
    pip config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple
    pip config set global.trusted-host pypi.tuna.tsinghua.edu.cn
    pip config set install.trusted-host pypi.tuna.tsinghua.edu.cn
    print_success "PIP 配置完成"
}

# Clone repositories
clone_repos() {
    print_info "克隆代码仓库..."
    # 检查是否为root用户
    if [ "$(id -u)" -eq 0 ]; then
        print_error "请不要以root用户运行此脚本！"
        print_error "请使用普通用户权限运行，需要时脚本会通过sudo请求权限。"
        exit 1
    fi
    cd ~
    print_info "请输入分支名称（直接回车则使用默认 master 分支)"
    read -r branch
    # 设置默认分支
    [ -z "$branch" ] && branch="master"

    print_info "请输入仓库commit(直接回车则使用最新 commit)"
    read -r commit
    # 清理 /root 下的 kuavo-ros-opensource 文件夹
    if [ -d "/root/kuavo-ros-opensource" ]; then
        print_info "清理 /root 下的 kuavo-ros-opensource 文件夹..."
        sudo rm -rf /root/kuavo-ros-opensource
        print_success "/root/kuavo-ros-opensource 清理完成"
    else
        print_info "/root 下不存在 kuavo-ros-opensource 文件夹，无需清理"
    fi

    # 检查 kuavo-ros-opensource 目录和远程仓库
    if [ -d "kuavo-ros-opensource" ] && [ -d "kuavo-ros-opensource/.git" ]; then
        cd kuavo-ros-opensource
        REMOTE_URL=$(git remote get-url origin 2>/dev/null)
        if [ "$REMOTE_URL" = "https://gitee.com/leju-robot/kuavo-ros-opensource.git" ]; then
            print_info "目录已存在且远程仓库正确，清理工作区..."
            git reset --hard HEAD
            git clean -fd
        else
            print_info "目录存在但远程仓库不匹配，重新克隆..."
            cd ~
            rm -rf kuavo-ros-opensource
            need_clone=true
        fi
    else
        need_clone=true
    fi

    if [ "$need_clone" = true ]; then
        git clone https://gitee.com/leju-robot/kuavo-ros-opensource.git --branch "$branch"
        cd kuavo-ros-opensource
    fi

    git checkout "$branch"
    # 只有当commit非空时才checkout
    if [ -n "$commit" ]; then
        git checkout "$commit"
    else
        git pull
    fi
 
    print_success "代码仓库克隆/更新完成"
}

check_samplerate(){
    print_info "正在安装 samplerate..."
    sudo bash ~/kuavo-ros-opensource/tools/user_groups_tools/samplerate_install.sh
    if [ $? -eq 0 ]; then
        print_success "samplerate 安装完成"
    else
        print_error "samplerate 安装失败，请检查日志"
        exit 1
    fi
}

# Configure robot version
setup_robot_version() {
    print_info "设置机器人版本..."
    print_info "请输入机器人版本 (42代表短臂, 45代表长臂):"
    read -r version
    
    # 准备要添加的环境变量行
    export_line="export ROBOT_VERSION=$version"
    
    # 检查用户的 .bashrc
    if grep -q "^export ROBOT_VERSION=" ~/.bashrc; then
        # 使用全局替换确保处理所有重复行
        sed -i "s/^export ROBOT_VERSION=.*/$export_line/g" ~/.bashrc
    else
        # 如果不存在，则添加
        echo "$export_line" >> ~/.bashrc
    fi
    
    # 检查 root 的 .bashrc
    if sudo grep -q "^export ROBOT_VERSION=" /root/.bashrc; then
        # 使用全局替换确保处理所有重复行
        sudo sed -i "s/^export ROBOT_VERSION=.*/$export_line/g" /root/.bashrc
    else
        # 如果不存在，则添加
        sudo su -c "echo '$export_line' >> ~/.bashrc"
    fi
    
    print_success "机器人版本设置为 $version"
}

# Configure robot weight
setup_robot_weight() {
    print_info "设置机器人重量..."
    
    export ROBOT_VERSION=$version
    # 检查已有配置
    if [ -f ~/.config/lejuconfig/TotalMassV${ROBOT_VERSION} ]; then
        current_mass=$(cat ~/.config/lejuconfig/TotalMassV${ROBOT_VERSION})
        print_info "当前记录重量为 ${current_mass}kg"
    fi

    read -p "请输入机器人重量(kg): " robot_mass
    rm -rf ~/.config/lejuconfig/TotalMassV${ROBOT_VERSION}
    mkdir -p ~/.config/lejuconfig
    echo "$robot_mass" > ~/.config/lejuconfig/TotalMassV${ROBOT_VERSION}
    print_success "机器人重量设置为 ${robot_mass}kg"
}

# Configure drive board type
setup_drive_board() {
    print_info "设置驱动板类型..."
    print_info "请输入驱动板类型 (elmo/youda):"
    read -r board_type
    
    mkdir -p ~/.config/lejuconfig
    echo "$board_type" > ~/.config/lejuconfig/EcMasterType.ini
    
    print_success "驱动板类型设置为 $board_type"
}

# 检查音频设备并修改配置文件中的音频格式
check_and_update_audio_config() {
    print_info "检查音频设备..."
    # 检查是否存在音频设备（如card0）
    if sudo aplay -l | grep -i Audio ; then
        print_success "检测到音频设备，正在修改音频配置文件中的 .mp3 为 .wav ..."
        config_file=~/kuavo-ros-opensource/src/humanoid-control/h12pro_controller_node/config/customize_config.json
        if [ -f "$config_file" ]; then
            sed -i 's/\.mp3/\.wav/g' "$config_file"
            print_success "已将 $config_file 中的 .mp3 替换为 .wav"
        else
            print_error "未找到配置文件 $config_file，无法修改音频格式"
        fi
    else
        print_info "未检测到音频设备，无需修改音频配置文件"
    fi
}

# 在合适位置调用该函数

# Configure arm motor config
setup_arm_motor() {
    print_info "配置手臂电机配置文件"

    # Ask if it's a long arm
    read -p "是长手臂机器人吗？ (y/n): " is_long_arm

    # Determine config file name
    if [[ "$is_long_arm" == "y" || "$is_long_arm" == "Y" ]]; then
        config_file="long_arm_config.yaml"


        cd ~/kuavo-ros-opensource/src/demo/examples_code/hand_plan_arm_trajectory/action_files

        # 检查是否有*_45.tact文件
        tact_files_count=$(find . -name "*_45.tact" | wc -l)
        if [ $tact_files_count -eq 0 ]; then
            print_error "未找到长手臂动作 tact 动作文件"
            print_info "将继续执行，但可能缺少长手臂动作文件"
        else
            print_info "找到 $tact_files_count 个长手臂动作 tact 动作文件"
            mkdir -p ~/.config/lejuconfig/action_files/
            cp -r ./*_45.tact ~/.config/lejuconfig/action_files/
        fi
    else
        config_file="config.yaml"
    fi

    # Find config.yaml or long_arm_config.yaml in ruiwo_controller directory
    src_config=$(find ~/kuavo-ros-opensource -type f -path "*/ruiwo_controller/long_arm_config.yaml" 2>/dev/null)

    if [ -z "$src_config" ]; then
        print_error "未找到 ruiwo_controller/${config_file} 文件"
        if [[ "$config_file" == "long_arm_config.yaml" ]]; then
            print_error "未找到长手臂配置文件。请确认你安装了长手臂版本的软件。"
        fi
        return 1
    fi
    
    print_info "找到配置文件: $src_config"
    
    # Destination config directory and file
    dest_dir=~/.config/lejuconfig
    dest_config=${dest_dir}/config.yaml
    
    # Create destination directory if it doesn't exist
    mkdir -p "$dest_dir"
    
    # Copy the config file
    cp "$src_config" "$dest_config"
    
    print_success "手臂电机配置文件配置完成"
}

setup_hand_usb() {
  local device_list=()
  local swap_flag

  # 使用awk处理多行输出
  while IFS=$'\t' read -r device desc hwid; do
    if [[ "$desc" == *"LJ485A"* || "$desc" == *"LJ485B"* ]]; then
      # 提取SER值
      ser=$(echo "$hwid" | grep -o "SER=[0-9A-Z]*" | cut -d= -f2)
      echo "串口：$device SER: $ser"
      device_list+=("$device")
    fi
  done < <(python3 -m serial.tools.list_ports -v | awk '
    /^\/dev\// {
        device=$1
        getline
        desc=substr($0, index($0,": ")+2)
        getline
        hwid=substr($0, index($0,": ")+2)
        print device "\t" desc "\t" hwid
    }')

  read -p "是否交换左右手(no/yes)：" swap_str
  swap_str=$(echo "$swap_str" | tr '[:upper:]' '[:lower:]')
  if [[ "${swap_str:0:1}" == "y" ]]; then
    swap_flag=1
  else
    swap_flag=0
  fi

  if [[ ${#device_list[@]} -eq 2 ]]; then
    folder_path="$HOME/kuavo-ros-opensource/tools/check_tool"
    
    # 检查脚本是否存在
    if [ ! -f "$folder_path/generate_serial.sh" ]; then
      print_error "未找到 generate_serial.sh 文件"
      return 1
    fi

    # 处理第一个设备
    if [[ "$swap_flag" -eq 1 ]]; then
      arg1="${device_list[0]}"
    else
      arg1="${device_list[1]}"
    fi
    echo "Running: sudo bash $folder_path/generate_serial.sh $arg1 stark_serial_R"
    sudo bash "$folder_path/generate_serial.sh" "$arg1" "stark_serial_R"

    # 处理第二个设备
    if [[ "$swap_flag" -eq 1 ]]; then
      arg1="${device_list[1]}"
    else
      arg1="${device_list[0]}"
    fi
    echo "Running: sudo bash $folder_path/generate_serial.sh $arg1 stark_serial_L"
    sudo bash "$folder_path/generate_serial.sh" "$arg1" "stark_serial_L"

  else
    print_error "失败，找到 ${#device_list[@]} 个485设备，需要2个"
  fi
}

# Configure end effector
setup_end_effector() {
    print_info "配置末端执行器..."
    print_info "请选择末端执行器类型:"
    echo "1) 灵巧手"
    echo "2) 二指夹爪"
    read -r effector_choice
    
    if [ "$effector_choice" = "1" ]; then
        # Configure smart hand USB devices
        print_info "配置灵巧手USB设备..."
        setup_hand_usb
        print_success "灵巧手配置完成"
        
    elif [ "$effector_choice" = "2" ]; then
        sed -i 's/"EndEffectorType": \[.*\]/"EndEffectorType": ["lejuclaw", "lejuclaw"]/' ~/kuavo-ros-opensource/src/kuavo_assets/config/kuavo_v${ROBOT_VERSION}/kuavo.json
        print_success "二指夹爪配置完成"
    fi
}

# Install VR dependencies
install_vr_deps() {
    print_info "安装VR相关依赖..."
    cd ~/kuavo-ros-opensource
    pip3 install -r src/manipulation_nodes/noitom_hi5_hand_udp_python/requirements.txt
    sudo pip3 install -r src/manipulation_nodes/noitom_hi5_hand_udp_python/requirements.txt
    print_success "VR依赖安装完成"
}

# Build the project
build_project() {
    print_info "编译项目..."
    cd $HOME/kuavo-ros-opensource
    
    export ROBOT_VERSION=$version

    # 先清理
    sudo -E su -c "catkin clean -y || true"
    
    # 配置并编译
    sudo -E su -c "catkin config -DCMAKE_ASM_COMPILER=/usr/bin/as -DCMAKE_BUILD_TYPE=Release"
    sudo -E su -c "source installed/setup.bash && catkin build humanoid_controllers"
    
    print_success "项目编译完成"
}

#Check hosts ip
check_ip(){
    print_info "检查下位机连接ip..."
    cd ~/kuavo-ros-opensource/docs/others/CHANGE_ROS_MASTER_URI/
    output=$(./get_ip.sh)
    if [[ "$output" == "192.168.26.12" || "$output" == "192.168.26.1" ]]; then
        echo "IP 匹配成功: $output"
    else
        echo "IP 匹配失败！请联系技术支持检查上位机 DHCP 配置！"
        return 1
    fi
}
#Change hosts mapping
modify_hosts_mapping(){
    print_info "修改hosts映射关系..."
    cd ~/kuavo-ros-opensource/docs/others/CHANGE_ROS_MASTER_URI/
    sudo -E su -c "./add_ros_master_hosts.sh"
}

#Change ROS_MASTER_URI
modiyf_ros_master_uri(){
    print_info "修改 ROS_MASTER_URI 和 ROS_HOSTNAME 配置..."
    cd ~/kuavo-ros-opensource/docs/others/CHANGE_ROS_MASTER_URI/
    source ./add_ros_master_uri.sh body
}


# Setup H12PRO controller
setup_h12pro() {
    print_info "是否配置H12PRO遥控器? (y/N)"
    read -r setup_controller
    
    if [[ $setup_controller =~ ^[Yy]$ ]]; then
        print_info "配置H12PRO遥控器..."
        cd $HOME/kuavo-ros-opensource/src/humanoid-control/h12pro_controller_node/scripts
        export ROBOT_VERSION=$version
        sudo -E su -c "./deploy_autostart.sh"
        print_success "H12PRO遥控器配置完成"
    fi
}

# Clean up closed source code
cleanup_code() {
    print_info "清理闭源代码..."
    find ~ -type d -name "*kuavo*" | while read -r dir; do 
        if [ -d "$dir/.git" ]; then 
            REMOTE_URL=$(git -C "$dir" remote get-url origin 2>/dev/null)
            if [[ "$REMOTE_URL" == "ssh://git@www.lejuhub.com:10026/highlydynamic/kuavo.git" || \
                  "$REMOTE_URL" == "https://www.lejuhub.com/highlydynamic/kuavo.git" || \
                  "$REMOTE_URL" == "ssh://git@www.lejuhub.com:10026/highlydynamic/kuavo-ros-control.git" || \
                  "$REMOTE_URL" == "https://www.lejuhub.com/highlydynamic/kuavo-ros-control.git" ]]; then
                print_info "Deleting: $dir"
                sudo rm -rf "$dir"
            fi
        fi
    done
    print_success "闭源代码清理完成"
}


# enable change wifi at vnc desktop
enable_vnc_network_config() {

    folder_path_="$HOME/kuavo-ros-opensource/tools"
    # 检查脚本是否存在
    if [ ! -f "$folder_path_/enable_vnc_network_config.sh" ]; then
      print_error "未找到 enable_vnc_network_config.sh 文件"
      return 1
    fi
    sudo -E su -c "$folder_path_/enable_vnc_network_config.sh"
}

check_system_time(){
    print_info "检查系统时间..."
    print_info "当前系统时间: $(date)"
    
    read -p "当前时间是否正确? (y/n): " time_correct
    
    if [[ $time_correct =~ ^[Yy]$ ]]; then
        print_success "系统时间正确，无需修改"
        return 0
    else
        sudo apt update
        sudo apt install ntpdate -y
        sudo timedatectl set-ntp false
        print_info "正在修改系统时间..."
        sudo ntpdate -u pool.ntp.org
        sudo timedatectl set-ntp true
        sudo systemctl restart systemd-timesyncd
        print_success "系统时间已更新"
        
        # 再次检查时间
        print_info "修改后的系统时间: $(date)"
        
        read -p "修改后的时间是否正确? (y/n): " time_correct_after
        
        if [[ ! $time_correct_after =~ ^[Yy]$ ]]; then
            print_error "系统时间设置失败，请联系技术支持"
            exit 1
        fi
        
        print_success "系统时间已正确设置"
    fi
}
# Main execution
main() {
    print_info "开始KUAVO-ROS-CONTROL安装配置脚本..."
    
    check_system_time
    setup_pip
    clone_repos
    check_samplerate
    check_and_update_audio_config
    setup_robot_version
    setup_robot_weight
    setup_drive_board
    setup_arm_motor
    setup_end_effector
    install_vr_deps
    build_project
    check_ip
    modify_hosts_mapping
    modiyf_ros_master_uri
    setup_h12pro
    enable_vnc_network_config
    cleanup_code
    
    print_success "KUAVO-ROS-CONTROL安装配置成功完成!"
}

# Run main function
main