#!/bin/bash

# 基础配置
set +u
set -eo pipefail

# 全局参数（双用户配置，新增豆包语音相关参数）
SSH_USERS=("leju_kuavo" "kuavo")  # 优先尝试leju_kuavo，失败后尝试kuavo
SSH_PASS="leju_kuavo"              # 双用户共用密码（与你的环境匹配）
UPPER_IP="192.168.26.12"           # 固定上位机IP
# 豆包语音专用参数（简化版）
UPPER_WS_DOUBAO="/home/leju_kuavo/kuavo_ros_application"  # 上位机豆包工作空间
MIC_LAUNCH_CMD="roslaunch kuavo_audio_receiver receive_voice.launch"  # 麦克风启动命令
MIC_CLEAN_KEYWORD="receive_voice.launch|kuavo_audio_receiver"  # 麦克风进程清理关键词
# 二维码抓取原有参数
UPPER_WS_QR="/home/leju_kuavo/kuavo_ros_application"  # 二维码工作空间（与豆包一致）
LAUNCH_CMD_QR="roslaunch dynamic_biped load_robot_head.launch use_orbbec:=true"  # 二维码启动命令
LAUNCH_KEYWORD_QR="load_robot_head.launch"  # 二维码进程清理关键词
LOWER_WS="/home/lab/kuavo-ros-opensource/"  # 固定下位机工作空间
CURRENT_SSH_USER=""  # 存储成功连接的用户名

# 日志函数
info() { echo -e "\033[34m[INFO] $1\033[0m"; }
success() { echo -e "\033[32m[SUCCESS] $1\033[0m"; }
error() { echo -e "\033[31m[ERROR] $1\033[0m"; exit 1; }
warn() { echo -e "\033[33m[WARN] $1\033[0m"; }

# 1. 依赖检查
check_deps() {
    if ! command -v sshpass &> /dev/null; then
        info "安装sshpass..."
        sudo apt update >/dev/null 2>&1
        sudo apt install -y sshpass || error "sshpass安装失败，手动执行：sudo apt install sshpass"
    fi
    if ! command -v rosnode &> /dev/null; then
        error "未检测到ROS环境！先执行：source $LOWER_WS/devel/setup.bash"
    fi
}

# 2. 主菜单（新增豆包语音选项1）
show_menu() {
    clear
    echo "======================================"
    echo "         机器人案例选择菜单          "
    echo "======================================"
    echo "1. 豆包语音"
    echo "2. 二维码抓取水瓶"
    echo "3. 清除豆包 app_id; access_key"
    echo "4. 退出"
    echo "======================================"
    read -p "请输入选项 (1/2/3/4): " choice
    if ! [[ "$choice" =~ ^[1234]$ ]]; then
        error "无效选项！输入1/2/3/4"
    fi
}

# 3. 双用户自动SSH连接（通用函数，豆包和二维码共用）
auto_ssh_connect() {
    info "===== 阶段1/3：自动建立上位机SSH连接 ====="
    info "将依次尝试用户：${SSH_USERS[*]}@$UPPER_IP（密码：$SSH_PASS）"
    
    # 循环尝试用户列表
    for user in "${SSH_USERS[@]}"; do
        info "正在尝试连接 $user@$UPPER_IP..."
        # 测试SSH连接（自动传递密码）
        if sshpass -p "$SSH_PASS" ssh -o ConnectTimeout=10 -o StrictHostKeyChecking=no \
            "$user@$UPPER_IP" "echo 'SSH连接测试成功'" 2>&1; then
            CURRENT_SSH_USER="$user"  # 保存成功的用户名
            success "SSH自动连接成功：$CURRENT_SSH_USER@$UPPER_IP"
            return  # 连接成功，退出函数
        else
            warn "连接 $user@$UPPER_IP 失败，尝试下一个用户..."
        fi
    done
    
    # 所有用户尝试失败
    error "所有用户（${SSH_USERS[*]}）均连接失败！检查：1.IP=$UPPER_IP 2.密码=$SSH_PASS 3.上位机是否在线"
}

case_doubao_simple() {
    echo -e "\n======================================"
    info "完整流程：清理旧进程 → 装依赖 → 下位机后台（日志分离） → 麦克风后台 → 主程序当前终端启动"
    info "下位机/麦克风逻辑不变，仅分离下位机输出解决主程序卡住问题"
    echo "=======================================\n"

    # 临时关闭错误中断
    set +e
    original_set=$(set -o | grep errexit | awk '{print $2}')
    MIC_PID=""         # 上位机麦克风进程ID
    LOWER_PID=""       # 本地下位机进程ID
    MAX_CHECK=15       # 节点检测重试次数
    CHECK_COUNT=0      # 当前检测次数
    MIC_LOG="/tmp/upper_mic.log"  # 麦克风日志
    MAIN_LOG="/tmp/upper_main.log" # 主程序日志
    LOWER_LOG="/tmp/lower_audio.log" # 下位机日志（新增，分离输出）

    # -------------------------- 步骤1：清理旧进程（不变） --------------------------
    echo -e "\033[33m【步骤1/5】清理所有旧ROS进程\033[0m"
    # 本地清理
    pkill -f "play_music.launch|play_music_node|audio_stream_player_node" >/dev/null 2>&1
    rm -f $LOWER_LOG  # 删除旧下位机日志
    sleep 2
    success "本地旧进程清理完成"

    # 上位机清理
    sshpass -p "$SSH_PASS" ssh -o ConnectTimeout=10 \
        "leju_kuavo@$UPPER_IP" "
        pkill -f 'receive_voice.launch|micphone_receiver_node|python3 src/kuavo_doubao_model/start_communication.py' >/dev/null 2>&1
        rm -f $MIC_LOG $MAIN_LOG
        sleep 2
        echo '[上位机] 旧进程和日志清理完成'
    "
    success "上位机旧进程清理完成\n"

    # -------------------------- 步骤2：安装依赖（不变） --------------------------
    echo -e "\033[33m【步骤2/5】上位机安装依赖\033[0m"
    sshpass -p "$SSH_PASS" ssh -tt -o ConnectTimeout=30 \
        "leju_kuavo@$UPPER_IP" "
        TSINGHUA_PIP='https://pypi.tuna.tsinghua.edu.cn/simple'
        python3 -m pip install -i \$TSINGHUA_PIP --user librosa==0.11.0 samplerate==0.2.1 torch==2.4.1 --upgrade && echo '依赖安装完成' || echo '依赖已存在'
        exit 0
    "
    [ $? -eq 0 ] && success "上位机依赖安装完成\n" || warn "依赖安装有警告，继续执行\n"

    # -------------------------- 步骤3：下位机启动（核心优化：分离输出） --------------------------
    echo -e "\033[33m【步骤3/5】下位机后台运行（输出写入日志，不占用终端）\033[0m"
    if [ ! -d "/home/lab/kuavo-ros-opensource/" ]; then
        error "本地工作空间不存在：/home/lab/kuavo-ros-opensource/"
    fi

    # 关键优化：下位机输出全部写入日志，不占用当前终端IO（解决主程序卡住）
    bash -c "
        cd /home/lab/kuavo-ros-opensource/
        source devel/setup.bash
        export ROS_MASTER_URI=http://kuavo_master:11311
        roslaunch kuavo_audio_player play_music.launch > $LOWER_LOG 2>&1
    " &
    LOWER_PID=$!
    info "下位机启动，PID：$LOWER_PID，日志：$LOWER_LOG，等待节点初始化..."

    # 节点检测逻辑不变
    while [ $CHECK_COUNT -lt $MAX_CHECK ]; do
        CHECK_COUNT=$((CHECK_COUNT + 1))
        NODE_EXISTS=$(bash -c "
            source /home/lab/kuavo-ros-opensource/devel/setup.bash
            export ROS_MASTER_URI=http://kuavo_master:11311
            rosnode list 2>/dev/null | grep -E 'audio_stream_player_node|play_music_node' | wc -l
        ")
        if [ "$NODE_EXISTS" -eq 2 ]; then
            success "下位机启动成功！\n"
            break
        else
            info "第 $CHECK_COUNT/$MAX_CHECK 次检测：当前存在 $NODE_EXISTS/2 个节点，2秒后重试..."
            sleep 2
        fi
    done

    if [ "$NODE_EXISTS" -ne 2 ]; then
        kill -9 "$LOWER_PID" >/dev/null 2>&1
        error "下位机启动失败！查看日志：cat $LOWER_LOG"
        return 1
    fi

    # -------------------------- 步骤4：麦克风启动（完全不变） --------------------------
    echo -e "\033[33m【步骤4/5】上位机后台启动麦克风节点\033[0m"
    MIC_PID=$(sshpass -p "$SSH_PASS" ssh -o ConnectTimeout=20 \
        "leju_kuavo@$UPPER_IP" "
        cd /home/leju_kuavo/kuavo_ros_application;
        source /opt/ros/noetic/setup.bash;
        export ROS_MASTER_URI=http://kuavo_master:11311;
        export ROS_IP=192.168.26.12;
        source devel/setup.bash;
        nohup roslaunch kuavo_audio_receiver receive_voice.launch > $MIC_LOG 2>&1 &
        echo \$!;
    ")
    info "麦克风节点启动，PID：$MIC_PID，等待5秒验证..."
    sleep 5

    VERIFY_RESULT=$(sshpass -p "$SSH_PASS" ssh -o ConnectTimeout=20 \
        "leju_kuavo@$UPPER_IP" "
        source /opt/ros/noetic/setup.bash
        export ROS_MASTER_URI=http://kuavo_master:11311
        export ROS_IP=192.168.26.12
        NODE_EXISTS=\$(rosnode list 2>/dev/null | grep -w 'micphone_receiver_node' | wc -l)
        TOPIC_EXISTS=\$(rostopic list 2>/dev/null | grep -w '/micphone_data' | wc -l)
        [ \$NODE_EXISTS -eq 1 ] && [ \$TOPIC_EXISTS -eq 1 ] && echo 1 || echo 0
    ")

    if [ "$VERIFY_RESULT" -eq 1 ]; then
        success "麦克风节点验证正常！\n"
    else
        warn "麦克风节点启动失败！查看日志：ssh leju_kuavo@$UPPER_IP 'cat $MIC_LOG'"
        sshpass -p "$SSH_PASS" ssh -o ConnectTimeout=10 \
            "leju_kuavo@$UPPER_IP" "kill -9 $MIC_PID 2>/dev/null"
        error "麦克风节点异常，无法启动主程序"
    fi

    # -------------------------- 步骤5：主程序当前终端启动（解决卡住） --------------------------
    echo -e "\033[33m【步骤5/5】当前终端启动主程序（无IO冲突）\033[0m"
    info "启动主程序（输出直接显示，按Ctrl+C终止）..."
    
    # 关键：由于下位机输出已写入日志，当前终端无IO冲突，主程序不会卡住
    sshpass -p "$SSH_PASS" ssh -tt -o ConnectTimeout=30 \
        "leju_kuavo@$UPPER_IP" "
        cd /home/leju_kuavo/kuavo_ros_application;
        source /opt/ros/noetic/setup.bash;
        export ROS_MASTER_URI=http://kuavo_master:11311;
        export ROS_IP=192.168.26.12
        source devel/setup.bash;
        # 主程序输出同时写入日志和当前终端
        python3 src/kuavo_doubao_model/start_communication.py 
    "

    # 主程序结束判断
    if [ $? -eq 0 ]; then
        success "主程序执行完成！\n"
    else
        warn "主程序执行异常！查看日志：ssh leju_kuavo@$UPPER_IP 'cat $MAIN_LOG'"
    fi

    # -------------------------- 清理流程（不变） --------------------------
    echo -e "\n======================================"
    info "开始清理所有进程..."

    # 清理麦克风
    [ -n "$MIC_PID" ] && sshpass -p "$SSH_PASS" ssh -o ConnectTimeout=10 \
        "leju_kuavo@$UPPER_IP" "kill -9 $MIC_PID 2>/dev/null && echo '[上位机] 麦克风已终止'"

    # 清理下位机
    [ -n "$LOWER_PID" ] && kill -9 "$LOWER_PID" 2>/dev/null && echo "[本地] 下位机已终止"

    # 恢复原始配置
    [ "$original_set" = "on" ] && set -e

    success "所有进程清理完成！"
    info "下位机日志：cat $LOWER_LOG | 主程序日志：ssh leju_kuavo@$UPPER_IP 'cat $MAIN_LOG'"
}

# 4. 启动上位机摄像头（使用成功连接的用户名）
start_upper_camera() {
    # 确保已成功获取用户名
    if [ -z "$CURRENT_SSH_USER" ]; then
        error "未获取到有效SSH用户名，无法启动摄像头"
    fi

    info "===== 阶段2/3：启动摄像头系统 ====="
    info "使用用户 $CURRENT_SSH_USER@$UPPER_IP 执行启动命令..."
    
    if ! sshpass -p "$SSH_PASS" ssh -tt "$CURRENT_SSH_USER@$UPPER_IP" << EOF
        echo "[上位机] 开始自动启动流程（用户：$CURRENT_SSH_USER）"
        
        # 1. 加载系统ROS环境
        echo "[上位机] 加载系统ROS：source /opt/ros/noetic/setup.bash"
        source /opt/ros/noetic/setup.bash || {
            echo "[上位机 ERROR] 系统ROS加载失败！请检查ROS安装"
            exit 1
        }
        
        # 2. 加载工作空间环境（适配双用户，确保路径一致）
        echo "[上位机] 加载工作空间：source $UPPER_WS_QR/kuavo_ros_application/devel/setup.bash"
        source $UPPER_WS_QR/kuavo_ros_application/devel/setup.bash || {
            echo "[上位机 ERROR] 工作空间加载失败！路径：$UPPER_WS_QR/kuavo_ros_application"
            exit 1
        }
        
        # 3. 清理旧进程
        echo "[上位机] 清理旧进程（关键词：$LAUNCH_KEYWORD_QR）..."
        pkill -f '$LAUNCH_KEYWORD_QR' >/dev/null 2>&1
        sleep 3
        
        # 4. 启动摄像头（后台运行）
        echo "[上位机] 执行启动命令：$LAUNCH_CMD_QR"
        export ROS_MASTER_URI=http://kuavo_master:11311
        export ROS_IP=192.168.26.12
        $LAUNCH_CMD_QR &
        sleep 20  # 等待节点完全启动
        
        # 5. 验证apriltag节点
        echo "[上位机] 验证apriltag节点..."
        if rosnode list 2>/dev/null | grep -q "/apriltag_ros_continuous_node"; then
            echo "[上位机 SUCCESS] apriltag节点已启动！"
            exit 0
        else
            echo "[上位机 ERROR] apriltag节点未启动！"
            echo "[上位机 ERROR] 当前运行的ROS节点："
            rosnode list 2>/dev/null
            exit 1
        fi
EOF
    then
        error "上位机摄像头启动失败（以上为详细错误日志）"
    fi
    success "上位机摄像头系统启动成功！apriltag节点已确认"
}

# 6. 二维码抓取执行函数（保持不变）
run_grab_program_qr() {
    local effector_choice="$1"
    local FIRST_RUN="$2"

    info "===== 阶段3/3：执行抓取程序 ====="
    cd "$LOWER_WS" || error "下位机工作空间不存在：$LOWER_WS"
    
    if [ "$FIRST_RUN" -eq 1 ]; then
        info "首次执行，编译kuavo_sdk和kuavo_msgs..."
        catkin build kuavo_sdk kuavo_msgs || error "编译失败"
        success "编译完成"
    else
        info "重复执行，跳过编译（使用已有结果）"
    fi
    
    source devel/setup.bash || error "ROS环境加载失败"
    
    local grab_cmd=""
    if [ "$effector_choice" -eq 1 ]; then
        grab_cmd="python3 src/demo/arm_capture_apriltag/arm_capture_apriltag.py --offset_start True --cost_weight 0.1"
    else
        grab_cmd="python3 src/demo/arm_capture_apriltag/claw_capture_apriltag.py --offset_start True --cost_weight 0.1"
    fi
    info "启动抓取程序：$grab_cmd"
    $grab_cmd || error "抓取程序执行失败"
    success "抓取程序执行完成！"
}

# 7. 二维码抓取案例入口（保持不变）
case_qrcode() {
    local effector_choice=""
    local effector_name=""
    local FIRST_RUN=1

    clear
    info "===== 启动二维码抓取水瓶案例 ====="
    read -p "请确保机器人已站立，按回车继续..."
    success "机器人站立确认完成"

    auto_ssh_connect
    start_upper_camera

    while true; do
        echo -e "\n======================================"
        echo "     选择末端执行器类型      "
        echo "======================================"
        echo "1. 灵巧手"
        echo "2. 夹爪"
        echo "======================================"
        read -p "输入选项 (1/2): " effector_choice
        if [[ "$effector_choice" =~ ^[12]$ ]]; then
            effector_name=$([ "$effector_choice" -eq 1 ] && echo "灵巧手" || echo "夹爪")
            break
        else
            warn "无效选项！输入1或2"
        fi
    done

    local repeat=1
    while [ $repeat -eq 1 ]; do
        run_grab_program_qr "$effector_choice" "$FIRST_RUN"
        FIRST_RUN=0

        info "===== 执行完成 ====="
        local repeat_choice=""
        while true; do
            echo -e "\n======================================"
            echo "       程序执行完成                "
            echo "======================================"
            echo "1. 重复执行当前抓取程序（$effector_name）"
            echo "2. 返回主菜单（自动关闭摄像头）"
            echo "======================================"
            read -p "输入选项 (1/2): " repeat_choice
            if [[ "$repeat_choice" =~ ^[12]$ ]]; then
                break
            else
                warn "无效选项！输入1或2"
            fi
        done

        if [ "$repeat_choice" -ne 1 ]; then
            repeat=0
            info "自动关闭上位机摄像头进程（用户：$CURRENT_SSH_USER）..."
            sshpass -p "$SSH_PASS" ssh -tt "$CURRENT_SSH_USER@$UPPER_IP" "pkill -f '$LAUNCH_KEYWORD_QR'" >/dev/null 2>&1
            success "已关闭摄像头，返回主菜单"
        fi
    done
}


case_restore() {
    # 上位机清理
    sshpass -p "$SSH_PASS" ssh -o ConnectTimeout=10 \
        "leju_kuavo@$UPPER_IP" "
        cd /home/leju_kuavo/kuavo_ros_application
        git restore src/kuavo_doubao_model/start_communication.py
        sleep 2
        echo '[上位机] 豆包 app_id; access_key清理完成'
    "
    success "豆包 app_id; access_key清理完成\n"
}

# 主入口（新增豆包语音选项分支）
check_deps
info "脚本启动成功（支持：1.豆包语音简化版 2.二维码抓取 3.清除豆包 app_id; access_key）"
while true; do
    show_menu
    case $choice in
        1) 
            case_doubao_simple  # 执行简化版豆包语音
            ;;
        2) 
            case_qrcode  # 执行二维码抓取
            ;;
        3) 
            case_restore  # 清除豆包 app_id; access_key
            success "豆包 app_id; access_key清理完成"
            ;;
        4) 
            success "程序退出"
            exit 0 
            ;;
    esac
done