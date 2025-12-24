#!/bin/bash

# IMU测试脚本启动器
# 提供简单的界面来运行IMU测试，包含Yaw角偏移测试(±5°)和频率测试

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 查找可执行文件，优先使用devel目录中的版本
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/../../../" && pwd)"
EXECUTABLE_DEVEL="$WORKSPACE_ROOT/devel/lib/hardware_node/kuavo_imu_test"

# 选择可用的可执行文件
if [ -f "$EXECUTABLE_DEVEL" ]; then
    EXECUTABLE="$EXECUTABLE_DEVEL"
else
    EXECUTABLE="$EXECUTABLE_DEVEL"  # 默认使用devel路径用于错误提示
fi

# 检查可执行文件是否存在
check_executable() {
    if [ ! -f "$EXECUTABLE" ]; then
        echo "错误: 找不到可执行文件"
        echo "检查的路径:"
        echo "  devel: $EXECUTABLE_DEVEL"
        echo
        echo "请先编译程序: cd $WORKSPACE_ROOT && catkin build hardware_node"
        return 1
    fi
    echo "使用可执行文件: $EXECUTABLE"
    return 0
}

# 运行IMU测试
run_imu_test() {
    local imu_type=$1
    
    if ! check_executable; then
        return 1
    fi
    
    echo "启动 $imu_type IMU 测试..."
    echo "测试顺序："
    echo "  1. 基本数据读取测试"
    echo "  2. 频率测试 (使用rostopic hz)"
    echo "  3. Yaw角偏移测试 (±5°范围)"
    echo "  4. 连续数据发布直到用户终止"
    echo "注意: 程序将持续运行，按 Ctrl+C 停止"
    echo
    
    # 运行测试程序
    "$EXECUTABLE" "$imu_type"
    
    local exit_code=$?
    echo
    if [ $exit_code -eq 0 ]; then
        echo "测试正常结束"
    else
        echo "测试异常退出 (退出码: $exit_code)"
        echo "请检查IMU是否连接正常"
        
    fi
}


# 主循环
main() {
    
    while true; do
        echo
        echo "==========================================="
        echo "         KUAVO IMU 测试工具"
        echo "==========================================="
        echo
        echo "请选择要执行的操作:"
        echo
        echo "  1) 测试 Xsens IMU"
        echo "  2) 测试 HIPNUC IMU"
        echo "  q) 退出"
        echo
        echo -n "请输入选择: "
        
        read choice
        echo
        
        case $choice in
            1)
                run_imu_test "xsens"
                echo
                echo "按回车键继续..."
                read
                ;;
            2)
                run_imu_test "hipnuc"
                echo
                echo "按回车键继续..."
                read
                ;;
            q|Q)
                echo "再见！"
                exit 0
                ;;
            *)
                echo "无效选择，请重试"
                sleep 1
                ;;
        esac
    done
}

# 脚本入口点
if [ "${BASH_SOURCE[0]}" == "${0}" ]; then
    main "$@"
fi 