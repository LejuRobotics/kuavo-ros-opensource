#!/bin/bash

# 安全启动机器人控制系统脚本
# 解决 nodelet_manager 进程死亡问题

set -e  # 遇到错误立即退出

echo "=========================================="
echo "         机器人安全启动脚本"
echo "=========================================="

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 检查函数
check_system_resources() {
    echo -e "${YELLOW}检查系统资源...${NC}"
    
    # 检查内存
    FREE_MEM=$(free -m | awk 'NR==2{printf "%.1f", $7/1024}')
    if (( $(echo "$FREE_MEM < 8.0" | bc -l) )); then
        echo -e "${RED}错误: 可用内存不足 (${FREE_MEM}GB < 8GB)${NC}"
        echo "请释放内存后重试"
        exit 1
    fi
    echo -e "${GREEN}✓ 内存检查通过 (${FREE_MEM}GB 可用)${NC}"
    
    # 检查磁盘空间
    DISK_USAGE=$(df -h /root/.ros 2>/dev/null | awk 'NR==2 {print $5}' | sed 's/%//' || echo "0")
    if [ "$DISK_USAGE" -gt 80 ]; then
        echo -e "${YELLOW}警告: 磁盘使用率较高 (${DISK_USAGE}%)${NC}"
        echo "正在清理 ROS 日志..."
        rosclean purge -y
    fi
    echo -e "${GREEN}✓ 磁盘空间检查通过${NC}"
}

# 检查 ROS 环境
check_ros_environment() {
    echo -e "${YELLOW}检查 ROS 环境...${NC}"
    
    if [ -z "$ROS_PACKAGE_PATH" ]; then
        echo -e "${RED}错误: ROS 环境未设置${NC}"
        echo "请运行: source devel/setup.bash"
        exit 1
    fi
    
    # 检查 roscore
    if ! pgrep -f roscore > /dev/null; then
        echo -e "${YELLOW}启动 roscore...${NC}"
        roscore &
        sleep 3
    fi
    echo -e "${GREEN}✓ ROS 环境检查通过${NC}"
}

# 清理之前的进程
cleanup_processes() {
    echo -e "${YELLOW}清理之前的进程...${NC}"
    
    # 杀死可能存在的节点
    rosnode kill /nodelet_manager 2>/dev/null || true
    
    # 等待进程完全退出
    sleep 2
    
    # 清理共享内存
    sudo ipcs -m | awk '$6 == 0 {print $2}' | xargs -r sudo ipcrm -m
    
    echo -e "${GREEN}✓ 进程清理完成${NC}"
}

# 设置 CPU 调度策略
setup_cpu_scheduling() {
    echo -e "${YELLOW}设置 CPU 调度策略...${NC}"
    
    # 设置实时调度策略（需要 root 权限）
    if [ "$EUID" -eq 0 ]; then
        echo 'performance' | tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor > /dev/null
        echo -e "${GREEN}✓ CPU 性能模式已设置${NC}"
    else
        echo -e "${YELLOW}警告: 需要 root 权限设置 CPU 性能模式${NC}"
    fi
}

# 监控启动过程
monitor_startup() {
    echo -e "${YELLOW}监控启动过程...${NC}"
    
    local timeout=60
    local count=0
    
    while [ $count -lt $timeout ]; do
        if rosnode list | grep -q "/nodelet_manager"; then
            echo -e "${GREEN}✓ nodelet_manager 启动成功${NC}"
            return 0
        fi
        sleep 1
        ((count++))
        if [ $((count % 10)) -eq 0 ]; then
            echo "等待 nodelet_manager 启动... (${count}/${timeout})"
        fi
    done
    
    echo -e "${RED}错误: nodelet_manager 启动超时${NC}"
    return 1
}

# 主函数
main() {
    echo "开始安全启动流程..."
    
    # 1. 系统检查
    check_system_resources
    check_ros_environment
    
    # 2. 清理环境
    cleanup_processes
    
    # 3. 性能优化
    setup_cpu_scheduling
    
    # 4. 启动系统
    echo -e "${YELLOW}启动机器人控制系统...${NC}"
    
    # 设置环境变量
    export ROSCONSOLE_CONFIG_FILE="$(rospack find humanoid_controllers)/config/rosconsole.conf"
    export ROS_PYTHON_LOG_CONFIG_FILE="$(rospack find humanoid_controllers)/config/rosconsole.conf"
    
    # 启动改进的 launch 文件
    roslaunch humanoid_controllers load_kuavo_mujoco_sim_fixed.launch joystick_type:=bt2 &
    LAUNCH_PID=$!
    
    # 5. 监控启动
    if monitor_startup; then
        echo -e "${GREEN}=========================================="
        echo -e "         启动成功！"
        echo -e "==========================================${NC}"
        
        # 等待用户中断
        trap "echo '正在安全关闭...'; kill $LAUNCH_PID; wait $LAUNCH_PID; cleanup_processes; exit 0" INT TERM
        wait $LAUNCH_PID
    else
        echo -e "${RED}启动失败，正在清理...${NC}"
        kill $LAUNCH_PID 2>/dev/null || true
        cleanup_processes
        exit 1
    fi
}

# 检查依赖
if ! command -v bc &> /dev/null; then
    echo -e "${RED}错误: 缺少 bc 命令，请安装: sudo apt install bc${NC}"
    exit 1
fi

# 运行主函数
main "$@" 