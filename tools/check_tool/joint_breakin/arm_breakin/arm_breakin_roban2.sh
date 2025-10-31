#!/bin/bash
SCRIPT_DIR=$(dirname "$(realpath "$0")")
PROJECT_DIR=$(realpath "$SCRIPT_DIR/../../../../") # project: kuavo-ros-control (闭源)

# 尝试查找开源仓库路径
OPENSOURCE_DIR=""
# 首先尝试从环境变量
if [ -n "$KUAVO_ROS_OPENSOURCE_DIR" ]; then
    OPENSOURCE_DIR="$KUAVO_ROS_OPENSOURCE_DIR"
elif [ -d "$HOME/kuavo-ros-opensource" ]; then
    OPENSOURCE_DIR="$HOME/kuavo-ros-opensource"
elif [ -d "$HOME/wang/kuavo-ros-opensource" ]; then
    OPENSOURCE_DIR="$HOME/wang/kuavo-ros-opensource"
fi

# 可执行文件路径列表（按优先级排序）
EXECUTABLE_PATHS=()

# 开源仓库路径（如果找到）
if [ -n "$OPENSOURCE_DIR" ]; then
    OPENSOURCE_DIR=$(realpath "$OPENSOURCE_DIR")
    EXECUTABLE_PATHS+=("$OPENSOURCE_DIR/installed/bin/arm_breakin_roban2")
fi

# 开源仓库
EXECUTABLE_PATHS+=("$PROJECT_DIR/installed/bin/arm_breakin_roban2")

# 闭源仓库路径
EXECUTABLE_PATHS+=("$PROJECT_DIR/devel/lib/hardware_node/arm_breakin_roban2")
EXECUTABLE_PATHS+=("$PROJECT_DIR/devel/lib/motorevo_controller/arm_breakin/arm_breakin_roban2")

# 打印带颜色的信息
echo_success() {
    echo -e "\033[0;32m$1\033[0m"
}

echo_info() {
    echo -e "\033[0;34m$1\033[0m"
}

echo_warning() {
    echo -e "\033[0;33m$1\033[0m"
}

echo_error() {
    echo -e "\033[0;31m$1\033[0m"
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

    echo_success "使用方法: $exec_path [配置文件路径] [磨线时长(秒)]"
    echo ""
    echo "参数:"
    echo "  配置文件路径  (可选) 指定配置文件路径，如果不提供则使用默认配置文件"
    echo "  磨线时长      (可选) 指定磨线时长（秒），建议至少 14 秒（完成一个动作周期）"
    echo ""
    echo "注意:"
    echo "  - 如果不提供磨线时长参数，程序会在初始化完成后提示您输入"
    echo "  - 程序会自动查找电机并初始化，请确保配置正确"
    echo ""
    echo "示例:"
    echo "  $exec_path                           # 使用默认配置，初始化后提示输入磨线时长"
    echo "  $exec_path /path/to/config.yaml      # 使用指定配置文件，初始化后提示输入磨线时长"
    echo "  $exec_path /path/to/config.yaml 60   # 使用指定配置文件，直接磨线 60 秒（无需输入）"
    echo "  $exec_path '' 60                     # 使用默认配置文件，直接磨线 60 秒（无需输入）"
    echo ""
    echo_info "注意: 脚本会自动查找以下路径的可执行文件（按优先级）:"
    for path in "${EXECUTABLE_PATHS[@]}"; do
        echo "  - $path"
    done
}

# 查找并执行 arm_breakin_roban2
execute_arm_breakin_roban2() {
    # 使用数组接收参数，确保正确处理选项参数
    local extra_args=("$@")

    for exec_path in "${EXECUTABLE_PATHS[@]}"; do
        if [ -f "$exec_path" ] && [ -x "$exec_path" ]; then
            echo_info "找到可执行文件: $exec_path"
            
            # 根据可执行文件位置选择对应的 setup.bash
            if [[ "$exec_path" == *"/devel/"* ]]; then
                # devel 版本（闭源）
                if [ -f "$PROJECT_DIR/devel/setup.bash" ]; then
                    echo_info "加载 devel 环境变量..."
                    source "$PROJECT_DIR/devel/setup.bash"
                fi
            else
                # installed 版本（开源）
                if [ -n "$OPENSOURCE_DIR" ] && [ -f "$OPENSOURCE_DIR/installed/setup.bash" ]; then
                    echo_info "加载 opensource installed 环境变量..."
                    source "$OPENSOURCE_DIR/installed/setup.bash"
                elif [ -f "$PROJECT_DIR/installed/setup.bash" ]; then
                    echo_info "加载 installed 环境变量..."
                    source "$PROJECT_DIR/installed/setup.bash"
                fi
            fi

            # 显示将要执行的命令
            if [ ${#extra_args[@]} -gt 0 ]; then
                echo_success "执行: $exec_path ${extra_args[*]}"
            else
                echo_success "执行: $exec_path"
            fi
            echo ""
            # 使用数组展开确保参数正确传递
            "$exec_path" "${extra_args[@]}"
            return $?
        fi
    done

    echo_error "错误: 未找到 arm_breakin_roban2 可执行文件"
    echo_warning "尝试了以下路径:"
    for path in "${EXECUTABLE_PATHS[@]}"; do
        if [ -f "$path" ]; then
            echo_warning "  - $path (存在但不可执行)"
        else
            echo_error "  - $path (不存在)"
        fi
    done
    return 1
}

# 主函数
main() {
    local extra_args=()
    local wait_for_leg=false

    # 解析命令行参数
    while [[ $# -gt 0 ]]; do
        case $1 in
            --help|-h)
                show_help
                exit 0
                ;;
            --wait-for-leg|-w)
                # 保留 --wait-for-leg 参数，稍后传递
                wait_for_leg=true
                extra_args+=("$1")
                shift
                ;;
            *)
                # 将所有其他参数作为额外参数传递给可执行文件
                extra_args+=("$1")
                shift
                ;;
        esac
    done

    # 执行 arm_breakin_roban2，使用数组确保参数正确传递
    execute_arm_breakin_roban2 "${extra_args[@]}"
}

# 运行主函数
main "$@"

