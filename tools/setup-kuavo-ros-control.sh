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
    # 版本选择和确认循环
    while true; do
        print_info "请输入机器人版本:"
        print_info "42 (短臂版本)"
        print_info "45 (长臂版本)"
        print_info "49 (pro max版本)"
        print_info "45.1 (假手版)"
        print_info "49.1 (展厅版)"
        print_info "52 (普通kuavo5)"
        print_info "53 (kuavo5，手臂pitch电机改ruiwo)"
        print_info "60 (悟时底盘轮臂)"
        print_info "61 (玖物底盘轮臂)"
        print_info "13 (roban2.0版本)"
        print_info "14 (roban2.1版本)"
        print_info "15 (roban2.2版本)"
        read -r version

        # 验证输入的版本是否有效
        if [[ "$version" != "42" && "$version" != "45" && "$version" != "49" &&
              "$version" != "45.1" && "$version" != "49.1" &&
              "$version" != "52" && "$version" != "53" &&
              "$version" != "60" && "$version" != "61" &&
              "$version" != "13" && "$version" != "14" && "$version" != "15" ]]; then
            print_error "无效的版本号: $version"
            print_info "请选择上述列出的有效版本号"
            continue  # 重新开始循环
        fi

        # 处理特殊版本号转换
        if [[ "$version" == "45.1" || "$version" == "49.1" ]]; then
            if [[ "$version" == "45.1" ]]; then
                version=100045
            else
                version=100049
            fi
        elif [[ "$version" == "13" ]]; then
            version=13
        elif [[ "$version" == "14" ]]; then
            version=14
        elif [[ "$version" == "15" ]]; then
            version=15
        fi

        # 显示选择的版本并要求确认
        print_info "您选择的机器人版本是: $version"
        print_info "确定使用这个版本吗？(y/n)"
        read -r confirm

        if [[ "$confirm" == "y" || "$confirm" == "Y" ]]; then
            break  # 确认成功，退出循环
        else
            print_info "请重新选择版本..."
        fi
    done

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

    # 根据ROBOT_VERSION设置ROBOT_NAME
    if [[ "$version" == 1* ]]; then
        robot_name="ROBAN"
    else
        robot_name="KUAVO"
    fi

    # 准备ROBOT_NAME环境变量行
    robot_name_line="export ROBOT_NAME=$robot_name"

    # 检查并设置用户的 .bashrc中的ROBOT_NAME
    if grep -q "^export ROBOT_NAME=" ~/.bashrc; then
        sed -i "s/^export ROBOT_NAME=.*/$robot_name_line/g" ~/.bashrc
    else
        echo "$robot_name_line" >> ~/.bashrc
    fi

    # 检查并设置root的 .bashrc中的ROBOT_NAME
    if sudo grep -q "^export ROBOT_NAME=" /root/.bashrc; then
        sudo sed -i "s/^export ROBOT_NAME=.*/$robot_name_line/g" /root/.bashrc
    else
        sudo su -c "echo '$robot_name_line' >> /root/.bashrc"
    fi

    # 设置当前会话的环境变量
    export ROBOT_NAME="$robot_name"

    print_success "机器人版本设置为 $version"
    print_success "机器人名称设置为 $robot_name"
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
    print_info "请输入驱动板类型 (elmo/youda/youda3/leju):"
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

# Configure hand real
setup_hand_real() {
    print_info "配置手部真实设备"

    # Ask if it's a long arm
    read -p "是假手机器人吗？ (y/n): " is_fake_hand

    if [[ "$is_fake_hand" == "y" || "$is_fake_hand" == "Y" ]]; then
        print_info "配置假手机器人手部设备"
        sed -i 's/Left_joint_arm_5: 0x[^ ]*/Left_joint_arm_5: 0x00/' ~/.config/lejuconfig/config.yaml
        sed -i 's/Left_joint_arm_6: 0x[^ ]*/Left_joint_arm_6: 0x00/' ~/.config/lejuconfig/config.yaml
        sed -i 's/Right_joint_arm_5: 0x[^ ]*/Right_joint_arm_5: 0x00/' ~/.config/lejuconfig/config.yaml
        sed -i 's/Right_joint_arm_6: 0x[^ ]*/Right_joint_arm_6: 0x00/' ~/.config/lejuconfig/config.yaml
    else
        print_info "配置展厅版手部设备"
    fi
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
    echo "3) 触觉灵巧手"
    echo "4) 没有灵巧手"
    read -r effector_choice
    
    if [ "$effector_choice" = "1" ]; then
        # Configure smart hand USB devices
        print_info "配置灵巧手USB设备..."
        setup_hand_usb
        print_success "灵巧手配置完成"
        
    elif [ "$effector_choice" = "2" ]; then
        sed -i 's/"EndEffectorType": \[.*\]/"EndEffectorType": ["lejuclaw", "lejuclaw"]/' ~/kuavo-ros-opensource/src/kuavo_assets/config/kuavo_v${ROBOT_VERSION}/kuavo.json
        sed -i 's/<arg name="ee_type" default="qiangnao"\/>/<arg name="ee_type" default="lejuclaw"\/>/' ~/kuavo-ros-opensource/src/humanoid-control/humanoid_controllers/launch/load_kuavo_real_with_vr.launch
        sed -i 's/<arg name="ee_type" default="qiangnao"\/>/<arg name="ee_type" default="lejuclaw"\/>/' ~/kuavo-ros-opensource/src/manipulation_nodes/noitom_hi5_hand_udp_python/launch/launch_quest3_ik.launch
        print_success "二指夹爪配置完成"
    elif [ "$effector_choice" = "3" ]; then
        sed -i 's/"EndEffectorType": \[.*\]/"EndEffectorType": ["qiangnao_touch", "qiangnao_touch"]/' ~/kuavo-ros-opensource/src/kuavo_assets/config/kuavo_v${ROBOT_VERSION}/kuavo.json
         sed -i 's/<arg name="ee_type" default="qiangnao"\/>/<arg name="ee_type" default="qiangnao_touch"\/>/' ~/kuavo-ros-opensource/src/humanoid-control/humanoid_controllers/launch/load_kuavo_real_with_vr.launch
        sed -i 's/<arg name="ee_type" default="qiangnao"\/>/<arg name="ee_type" default="qiangnao_touch"\/>/' ~/kuavo-ros-opensource/src/manipulation_nodes/noitom_hi5_hand_udp_python/launch/launch_quest3_ik.launch
        print_success "触觉灵巧手配置完成"
    elif [ "$effector_choice" = "4" ]; then
        sed -i 's/"EndEffectorType": \[.*\]/"EndEffectorType": ["none", "none"]/' ~/kuavo-ros-opensource/src/kuavo_assets/config/kuavo_v${ROBOT_VERSION}/kuavo.json
        sed -i 's/<arg name="ee_type" default="qiangnao"\/>/<arg name="ee_type" default="none"\/>/' ~/kuavo-ros-opensource/src/humanoid-control/humanoid_controllers/launch/load_kuavo_real_with_vr.launch
        sed -i 's/<arg name="ee_type" default="qiangnao"\/>/<arg name="ee_type" default="none"\/>/' ~/kuavo-ros-opensource/src/manipulation_nodes/noitom_hi5_hand_udp_python/launch/launch_quest3_ik.launch
        print_success "没有灵巧手配置完成"
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


# Setup controller
setup_controller() {
    print_info "是否配置遥控器? (y/N)"
    read -r setup_controller

    if [[ $setup_controller =~ ^[Yy]$ ]]; then
        while true; do
            print_info "请选择遥控器类型:"
            print_info "1) H12PRO遥控器"
            print_info "2) 北通遥控器"
            read -r controller_choice

            if [[ "$controller_choice" == "1" ]]; then
                print_info "配置H12PRO遥控器..."
                cd $HOME/kuavo-ros-opensource/src/humanoid-control/h12pro_controller_node/scripts
                export ROBOT_VERSION=$version
                sudo -E su -c "./deploy_autostart.sh"
                print_success "H12PRO遥控器配置完成"
                break
            elif [[ "$controller_choice" == "2" ]]; then
                print_info "配置北通遥控器..."
                cd $HOME/kuavo-ros-opensource/src/humanoid-control/joystick_drivers/joy/services
                export ROBOT_VERSION=$version
                sudo -E su -c "./deploy_autostart.sh"
                print_success "北通遥控器配置完成"
                break
            else
                print_error "无效的选择: $controller_choice"
                print_info "请选择 1 或 2"
            fi
        done
    fi
}

# Setup WebSocket service
setup_websocket() {
    print_info "是否配置WebSocket服务? (y/N)"
    read -r setup_websocket

    if [[ $setup_websocket =~ ^[Yy]$ ]]; then
        print_info "配置WebSocket服务..."
        cd $HOME/kuavo-ros-opensource/src/manipulation_nodes/planarmwebsocketservice/service
        export ROBOT_VERSION=$version
        sudo -E su -c "./websocket_deploy_script.sh"
        print_success "WebSocket服务配置完成"
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
        # 临时禁用 set-e，避免 apt update 因重复源警告导致脚本退出
        set +e
        sudo apt update
        sudo apt install ntpdate -y
        set -e  # 重新启用 set-e

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

# Copy preset actions and music files
setup_preset_files() {
    print_info "设置预置动作和音乐文件..."

    # Use the absolute path for kuavo-ros-opensource
    KUAVO_ROS_CONTROL_DIR="/home/lab/kuavo-ros-opensource"

    # For kuavo5 versions (52, 53), use resources/kuavo5 directory
    if [[ "$version" == "52" || "$version" == "53" ]]; then
        RESOURCES_DIR="$KUAVO_ROS_CONTROL_DIR/resources/kuavo5"
        print_info "使用 kuavo5 资源目录: $RESOURCES_DIR"
    else
        RESOURCES_DIR="$KUAVO_ROS_CONTROL_DIR/resources"
    fi

    # Check if resources directory exists
    if [ ! -d "$RESOURCES_DIR" ]; then
        print_error "未找到资源目录: $RESOURCES_DIR"
        return 1
    fi

    # Target directory for actions and music
    TARGET_DIR="/home/lab/.config/lejuconfig"

    # Create target directory if it doesn't exist
    sudo mkdir -p "$TARGET_DIR"

    # Copy action files and music files from resources to target directory
    if [ -n "$(ls -A "$RESOURCES_DIR" 2>/dev/null)" ]; then
        print_info "复制动作文件和音乐文件从 $RESOURCES_DIR 到 $TARGET_DIR"
        print_info "覆盖已存在的文件..."

        # 复制 action_files 目录
        if [ -d "$RESOURCES_DIR/action_files" ]; then
            sudo cp -rf "$RESOURCES_DIR/action_files" "$TARGET_DIR/"
            print_info "action_files 目录复制完成"
        fi

        # 复制 music 目录
        if [ -d "$RESOURCES_DIR/music" ]; then
            sudo cp -rf "$RESOURCES_DIR/music" "$TARGET_DIR/"
            print_info "music 目录复制完成"
        fi

        # Set proper ownership
        sudo chown -R lab:lab "$TARGET_DIR"

        print_success "预置动作和音乐文件复制完成"
        print_info "文件已覆盖到目标目录"
    else
        print_info "资源目录为空，无需复制"
    fi

    # Copy customize_config.json to joystick config directory
    SOURCE_CONFIG="$RESOURCES_DIR/customize_config.json"
    TARGET_CONFIG="$KUAVO_ROS_CONTROL_DIR/src/humanoid-control/joystick_drivers/joy/config/customize_config.json"

    if [ -f "$SOURCE_CONFIG" ]; then
        print_info "复制 customize_config.json 从 $SOURCE_CONFIG 到 $TARGET_CONFIG"

        # Create target directory if it doesn't exist
        mkdir -p "$(dirname "$TARGET_CONFIG")"

        # Copy the config file with force overwrite
        cp -f "$SOURCE_CONFIG" "$TARGET_CONFIG"

        print_success "customize_config.json 复制完成"
    else
        print_info "未找到 customize_config.json 文件，跳过复制"
    fi
}
# Replace config file with resources version
setup_config_file() {
    # Only run for roban versions (version starts with '1': 13, 14, 15)
    if [[ "$ROBOT_VERSION" != 1* ]]; then
        print_info "非roban版本机器，跳过配置文件替换"
        return 0
    fi

    print_info "替换配置文件（roban版本）..."
    source_config_file="/home/lab/kuavo-ros-opensource/resources/config.yaml"
    dest_config_file="$HOME/.config/lejuconfig/config.yaml"

    # Check if source config file exists
    if [ ! -f "$source_config_file" ]; then
        print_error "未找到配置文件: $source_config_file"
        return 1
    fi

    # Backup existing config file if it exists
    if [ -f "$dest_config_file" ]; then
        print_info "备份现有配置文件到 ${dest_config_file}.backup"
        cp "$dest_config_file" "${dest_config_file}.backup"
    fi

    # Copy the new config file
    cp "$source_config_file" "$dest_config_file"

    print_success "配置文件替换完成"
}

# Setup audio device configuration
setup_audio_config() {
    print_info "配置音频设备..."
    
    # 脚本路径
    audio_script_path="$HOME/kuavo-ros-opensource/tools/udev_rules/setup_audio.sh"
    
    # 检查脚本是否存在
    if [ ! -f "$audio_script_path" ]; then
        print_error "未找到音频配置脚本: $audio_script_path"
        return 1
    fi
    
    # 检查脚本是否可执行
    if [ ! -x "$audio_script_path" ]; then
        print_info "设置脚本执行权限..."
        chmod +x "$audio_script_path"
    fi
    
    # 调用脚本（脚本内部会检查 root 权限，所以使用 sudo）
    print_info "执行音频配置脚本..."
    if sudo bash "$audio_script_path"; then
        print_success "音频设备配置完成"
    else
        print_error "音频设备配置失败，请检查日志"
        return 1
    fi
}

setup_canbus_config() {
    print_info "配置CANBUS配置..."

    canbus_config_script_path="$HOME/kuavo-ros-opensource/tools/check_tool/canbus_config.sh"

    if [ ! -f "$canbus_config_script_path" ]; then
        print_error "未找到CANBUS配置脚本: $canbus_config_script_path"
        return 1
    fi

    if [ ! -x "$canbus_config_script_path" ]; then
        print_info "设置脚本执行权限..."
        chmod +x "$canbus_config_script_path"
    fi

    print_info "执行CANBUS配置脚本..."
    if sudo bash "$canbus_config_script_path"; then
        print_success "CANBUS配置完成"
    else
        print_error "CANBUS配置失败，请检查日志"
        return 1
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
    setup_hand_real
    setup_end_effector
    install_vr_deps
    setup_preset_files
    setup_config_file
    setup_audio_config
    setup_canbus_config
    build_project
    check_ip
    modify_hosts_mapping
    modiyf_ros_master_uri
    setup_websocket
    setup_controller
    enable_vnc_network_config
    cleanup_code

    print_success "KUAVO-ROS-CONTROL安装配置成功完成!"
}

# Run main function
main