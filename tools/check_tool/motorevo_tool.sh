#!/bin/bash
SCRIPT_DIR=$(dirname "$(realpath "$0")")
PROJECT_DIR=$(realpath "$SCRIPT_DIR/../../") # project: kuavo-ros-control/kuavo-ros-opensource

# 配置文件路径
CONFIG_DIR="$HOME/.config/lejuconfig"
CANBUS_CONFIG_FILE="$CONFIG_DIR/canbus_device_cofig.yaml"

# 可执行文件路径列表（按优先级排序）
EXECUTABLE_PATHS=(
    "$PROJECT_DIR/installed/bin/motorevo_tool"
    "$PROJECT_DIR/devel/lib/hardware_node/motorevo_tool"
)

# 打印带颜色的信息
echo_success() {
    echo -e "\033[0;32m$1\033[0m"
}

echo_warning() {
    echo -e "\033[1;33m$1\033[0m"
}

echo_info() {
    echo -e "\033[0;36m$1\033[0m"
}

# 打印帮助信息
show_help() {
    local exec_path=""

    # 查找到的可执行文件路径
    for path in "${EXECUTABLE_PATHS[@]}"; do
        if [ -f "$path" ] && [ -x "$path" ]; then
            exec_path="$path"
            break
        fi
    done

    echo_success "使用方法: $exec_path [选项]"
    echo ""
    echo "选项:"
    echo "  --negative  电机方向辨识"
    echo "  --cali      电机校准"
    echo "  --help     显示此帮助信息"
    echo ""
    echo "示例:"
    echo "  $exec_path --negative  # 设置电机方向"
    echo "  $exec_path --cali      # 电机校准"
}

# 查找并执行 motorevo_tool
execute_motorevo_tool() {
    local extra_args="$1"
    local check_config_change="$2"  # 是否检查电机方向变化
    local negtive_before=""
    # 如果需要检查电机方向变化，先保存所有 negtive 字段的值
    if [ "$check_config_change" = "true" ] && [ -f "$CANBUS_CONFIG_FILE" ]; then
        negtive_before=$(grep "negtive:" "$CANBUS_CONFIG_FILE" 2>/dev/null)
    fi

    for exec_path in "${EXECUTABLE_PATHS[@]}"; do
        if [ -f "$exec_path" ] && [ -x "$exec_path" ]; then
            echo "执行: $exec_path $extra_args"

            # 根据可执行文件位置选择对应的 setup.bash
            if [[ "$exec_path" == *"/devel/"* ]]; then
                # devel 版本
                if [ -f "$PROJECT_DIR/devel/setup.bash" ]; then
                    source "$PROJECT_DIR/devel/setup.bash"
                fi
            else
                # installed 版本
                if [ -f "$PROJECT_DIR/installed/setup.bash" ]; then
                    source "$PROJECT_DIR/installed/setup.bash"
                fi
            fi

            "$exec_path" $extra_args
            local exec_result=$?

            # 检查电机方向是否有变化
            if [ "$check_config_change" = "true" ] && [ -f "$CANBUS_CONFIG_FILE" ]; then
                local negtive_after=$(grep "negtive:" "$CANBUS_CONFIG_FILE" 2>/dev/null)
                if [ "$negtive_before" != "$negtive_after" ]; then
                    echo ""
                    echo_warning "╔══════════════════════════════════════════════════╗"
                    echo_warning "║  ⚠️  检测到电机方向已更新，请重新标定电机零点  ║"
                    echo_warning "╚══════════════════════════════════════════════════╝"
                fi
            fi

            return $exec_result
        fi
    done

    echo "错误: 未找到 motorevo_tool 可执行文件"
    echo "尝试了以下路径:"
    for path in "${EXECUTABLE_PATHS[@]}"; do
        echo "  - $path"
    done
    return 1
}

# 主函数
main() {
    local extra_args=""
    local check_config_change="false"

    # 如果没有参数，显示帮助信息
    if [[ $# -eq 0 ]]; then
        show_help
        exit 0
    fi

    # 解析命令行参数
    while [[ $# -gt 0 ]]; do
        case $1 in
            --negative)
                extra_args="--negative"
                check_config_change="true"
                shift
                ;;
            --cali)
                extra_args="--cali"
                shift
                ;;
            --help)
                show_help
                exit 0
                ;;
            *)
                echo "错误: 未知参数 '$1'"
                show_help
                exit 1
                ;;
        esac
    done

    # 执行 motorevo_tool
    execute_motorevo_tool "$extra_args" "$check_config_change"
}

# 运行主函数
main "$@"

