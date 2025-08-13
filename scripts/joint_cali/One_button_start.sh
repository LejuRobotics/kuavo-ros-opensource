#!/bin/bash

# 一键启动脚本 - 机器人关节标定
# 功能：自动启动下位机、上位机，为头部和手臂标定做准备

echo "==========================================================="
echo "           机器人关节标定一键启动脚本"
echo "==========================================================="

# 1. 安装环境（已注释，按需启动）
sudo bash scripts/joint_cali/create_venv.sh

# 获取当前仓库根目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# 2. 启动下位机launch（实物机器人）
echo "步骤2: 准备启动下位机launch文件..."
cd $REPO_ROOT
catkin clean -y
if [[ "$REPO_ROOT" == *"kuavo-ros-opensource"* ]]; then
    source installed/setup.bash
fi
catkin config -DCMAKE_ASM_COMPILER=/usr/bin/as -DCMAKE_BUILD_TYPE=Release
catkin build humanoid_controllers motion_capture_ik mobile_manipulator_controllers
source devel/setup.bash

# 检查是否已有roscore在运行
# 先关闭roscore
sudo killall -9 ros roslaunch roscore rosmaster



echo ""
echo "==========================================================="
echo "步骤2.5: 启动机器人控制系统..."
echo "==========================================================="

# 检查是否需要启动机器人控制系统
read -p "是否启动机器人控制系统？(y/N): " start_robot
if [[ ! "$start_robot" =~ ^[Yy]$ ]]; then
    echo "跳过机器人控制系统启动"
else
    echo "准备启动机器人控制系统..."
    
    # 检查screen是否已安装
    if ! command -v screen &> /dev/null; then
        echo "安装screen工具..."
        sudo apt update
        sudo apt install -y screen
    fi
    
    # 检查launch文件是否存在
    ROBOT_LAUNCH="$REPO_ROOT/scripts/joint_cali/launch/cali_real.launch"
    if [ ! -f "$ROBOT_LAUNCH" ]; then
        echo "✗ 错误：launch文件不存在: $ROBOT_LAUNCH"
        exit 1
    fi
    
    # 使用screen在后台启动roslaunch
    echo "在后台启动机器人控制系统..."
    SCREEN_NAME="robot_control"
    
    # 杀掉可能存在的同名screen会话
    screen -S $SCREEN_NAME -X quit 2>/dev/null || true
    sleep 10
    
    
    # 启动新的screen会话并运行roslaunch
    screen -dmS $SCREEN_NAME bash -c "
        cd $REPO_ROOT
        source devel/setup.bash
        echo 'Starting robot control system...'
        roslaunch $ROBOT_LAUNCH
    "
    
    echo "✓ 机器人控制系统已在后台启动 (screen会话: $SCREEN_NAME)"
    echo "提示：可使用 'screen -r $SCREEN_NAME' 查看输出"
    
    # 等待系统启动
    echo "等待系统初始化..."
    sleep 5
    
    # 用户交互判断机器人状态
    echo ""
    echo "==========================================================="
    echo "机器人状态确认"
    echo "==========================================================="
    echo "请观察机器人状态："
    echo "- 机器人应该会有缩腿动作"
    echo "- 等待机器人完成缩腿动作后，需要发送站立命令"
    echo ""
    
    while true; do
        read -p "机器人是否已完成缩腿动作？(按下y机器人会站立)(y/N): " robot_ready
        if [[ "$robot_ready" =~ ^[Yy]$ ]]; then
            echo "机器人已准备好，请注意！！！！！！  将在10秒后发送站立命令,..."
            
            # 倒计时
            for i in {10..1}; do
                echo "倒计时: $i 秒"
                sleep 1
            done
            
            echo "发送站立命令 'o' 到机器人控制系统..."
            
            # 检查是否使用screen模式
            if [ "${USE_SCREEN:-true}" = "true" ] && screen -list | grep -q "$SCREEN_NAME"; then
                # 向screen会话发送'o'和回车
                screen -S $SCREEN_NAME -p 0 -X stuff "o$(printf \\r)"
                echo "✓ 站立命令已通过screen发送"
            else
                # 备用方案：通过ROS话题发送命令或其他方式
                echo "⚠ screen会话不可用，使用备用方案..."
                echo "请手动在机器人控制终端输入 'o' 并回车"
                echo "或者使用以下命令连接到控制台："
                if [ -n "$MANUAL_PID" ]; then
                    echo "  tail -f /tmp/robot_control.log  # 查看日志"
                    echo "  kill $MANUAL_PID  # 停止进程"
                fi
            fi
            
            echo "等待机器人站立..."
            sleep 8
            
            echo "✓ 机器人控制系统启动完成"
            break
        else
            echo "等待机器人完成缩腿动作..."
            sleep 2
        fi
    done
fi

# 3. 启动上位机launch（标定所需的相机和Tag识别）
echo "==========================================================="
echo "步骤3: 启动上位机AprilTag识别系统..."

# 检查Python环境和依赖
echo "检查Python环境..."
if ! python3 -c "import paramiko" 2>/dev/null; then
    echo "安装paramiko库..."
    pip3 install paramiko
fi

# 使用Python脚本启动上位机AprilTag系统
echo "使用Python脚本启动上位机..."
python3 ./scripts/joint_cali/start_host_apriltag.py

if [ $? -eq 0 ]; then
    echo "✓ 上位机AprilTag识别系统启动完成"
else
    echo "✗ 上位机启动失败，请检查网络连接和配置"
    exit 1
fi

echo "==========================================================="
echo "系统启动完成！"
echo "下一步："
echo "1. 可以开始头部标定：python3 ./scripts/joint_cali/head_cali.py --use_cali_tool"
echo "2. 头部标定完成后进行手臂标定：python3 ./scripts/joint_cali/arm_cail_noui.py --real"
echo "==========================================================="

# 4. 启动头部标定py脚本
echo ""
echo "==========================================================="
echo "步骤4: 启动头部标定..."
echo "==========================================================="

# 检查虚拟环境是否存在
VENV_PATH="/home/lab/kuavo_venv/joint_cali"
if [ ! -d "$VENV_PATH" ]; then
    echo "✗ 错误：虚拟环境不存在: $VENV_PATH"
    echo "请先运行: sudo bash scripts/joint_cali/create_venv.sh"
    exit 1
fi
echo "✓ 虚拟环境存在"

# 检查标定配置文件
CONFIG_FILE="scripts/joint_cali/config/head_cali_config.yaml"
if [ ! -f "$CONFIG_FILE" ]; then
    echo "✗ 错误：头部标定配置文件不存在: $CONFIG_FILE"
    exit 1
fi
echo "✓ 头部标定配置文件存在"

# 显示配置信息
echo "当前头部标定配置："
cat $CONFIG_FILE

echo ""
echo "请确认："
echo "1. 标定工具已安装在机器人躯干上"
echo "2. AprilTag已正确贴在标定工具上"
echo "3. 上位机相机正常工作"
read -p "是否继续头部标定？(y/N): " confirm_head_cali

if [[ ! "$confirm_head_cali" =~ ^[Yy]$ ]]; then
    echo "跳过头部标定"
else
    echo "开始头部标定..."
    
    # 激活虚拟环境并设置环境变量
    echo "激活虚拟环境和设置环境..."
    
    # 创建临时脚本来执行头部标定
    HEAD_CALI_SCRIPT="/tmp/run_head_cali.sh"
    cat > $HEAD_CALI_SCRIPT << EOF
#!/bin/bash

# 激活虚拟环境
source $VENV_PATH/bin/activate

# 设置ROS环境
source "$REPO_ROOT/devel/setup.bash"

# 设置LD_LIBRARY_PATH
unset LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$VENV_PATH/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/opt/ros/noetic/lib:$LD_LIBRARY_PATH

# 进入工作目录
cd "$REPO_ROOT"

echo "环境设置完成，开始头部标定..."
echo "执行命令: python3 ./scripts/joint_cali/head_cali.py --use_cali_tool"
echo ""

# 运行头部标定
python3 ./scripts/joint_cali/head_cali.py --use_cali_tool
EOF

    # 给脚本执行权限
    chmod +x $HEAD_CALI_SCRIPT
    
    # 运行头部标定脚本
    echo "正在运行头部标定..."
    if $HEAD_CALI_SCRIPT; then
        echo ""
        echo "✓ 头部标定脚本执行完成"
        
        # 检查是否生成了备份文件（说明标定已执行）
        BACKUP_FILE="/home/lab/.config/lejuconfig/arms_zero.yaml.head_cali.bak"
        if [ -f "$BACKUP_FILE" ]; then
            echo "✓ 发现标定备份文件，头部标定已完成"
            echo "备份文件: $BACKUP_FILE"
        else
            echo "⚠ 未发现标定备份文件，可能标定未成功完成"
        fi
    else
        echo "✗ 头部标定执行失败"
        rm -f $HEAD_CALI_SCRIPT
        exit 1
    fi
    
    # 清理临时脚本
    rm -f $HEAD_CALI_SCRIPT
    
    echo ""
    echo "==========================================================="
    echo "✓ 头部标定完成！"
    echo "==========================================================="
fi

# 5. 启动arm_cail_noui.py（待实现）
# echo "步骤5: 启动手臂标定..."
source $VENV_PATH/bin/activate
source devel/setup.bash
unset LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$VENV_PATH/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/opt/ros/noetic/lib:$LD_LIBRARY_PATH
python3 ./scripts/joint_cali/arm_cail_noui.py --real

# 6. 执行完成推出脚本（待实现）
echo "手臂标定完成"
echo "                                   
 ▄▄▄▄                   ██              
 ▀▀██                   ▀▀              
   ██       ▄████▄    ████     ██    ██ 
   ██      ██▄▄▄▄██     ██     ██    ██ 
   ██      ██▀▀▀▀▀▀     ██     ██    ██ 
   ██▄▄▄   ▀██▄▄▄▄█     ██     ██▄▄▄███ 
    ▀▀▀▀     ▀▀▀▀▀      ██      ▀▀▀▀ ▀▀ 
                     ████▀              
                                   
"