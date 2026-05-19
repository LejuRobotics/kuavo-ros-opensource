#!/bin/bash
SCRIPT_DIR=$(dirname "$(realpath "$0")")
PROJECT_DIR=$(realpath "$SCRIPT_DIR/../../") # project: kuavo-ros-control/kuavo-ros-opensource

# 可执行文件路径列表（按优先级排序）
EXECUTABLE_PATHS=(
    "$PROJECT_DIR/devel/lib/hardware_node/dexhand_test"
    "$PROJECT_DIR/installed/bin/dexhand_test"
)

# 打印带颜色的信息
echo_success() {
    echo -e "\033[0;32m$1\033[0m"
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
    echo "  --touch      Kuavo Revo1 触觉手测试模式"
    echo "  --normal     Kuavo Revo1 普通手测试模式"
    echo "  --revo1can   Kuavo Revo1 Can协议灵巧手测试模式"
    echo "  --revo2      Roban2 Revo2 普通灵巧手测试模式"
    echo "  --revo2can   Roban2 Revo2 Can协议灵巧手测试模式"
    echo "  --scan       扫描设备(revo2can不支持), 识别 ttyUSB 设备"
    echo "  --test [round] 测试灵巧手运动, round 为测试次数，默认为 5 次"
    echo "  --help       显示此帮助信息"
    echo ""
    echo "示例:"
    echo "  $exec_path --touch --scan      # Kuavo Revo1 触觉手扫描"
    echo "  $exec_path --touch --test      # Kuavo Revo1 触觉手测试"
    echo "  $exec_path --normal --test     # Kuavo Revo1 普通手测试"
    echo "  $exec_path --revo1can --test   # Kuavo Revo1 Can协议灵巧手测试"
    echo "  $exec_path --revo2 --test      # Roban2 Revo2 普通手测试"
    echo "  $exec_path --revo2can --test   # Roban2 Revo2 Can协议灵巧手测试"
}

# 查找并执行 dexhand_test
execute_dexhand_test() {
    local extra_args="$@"

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
            return 0
        fi
    done

    echo "错误: 未找到 dexhand_test 可执行文件"
    echo "尝试了以下路径:"
    for path in "${EXECUTABLE_PATHS[@]}"; do
        echo "  - $path"
    done
    return 1
}

# 主函数
main() {
    local extra_args=""

    # 如果没有参数，显示帮助信息
    if [[ $# -eq 0 ]]; then
        show_help
        exit 0
    fi

    # 解析命令行参数
    while [[ $# -gt 0 ]]; do
        case $1 in
            --touch|--normal|--revo1can|--revo2|--revo2can)
                extra_args="$extra_args $1"
                shift
                ;;
            --scan)
                extra_args="$extra_args $1"
                shift
                ;;
            --test)
                extra_args="$extra_args $1"
                if [[ $# -gt 1 && "$2" =~ ^[0-9]+$ ]]; then
                    extra_args="$extra_args $2"
                    shift
                fi
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

    # 执行 dexhand_test
    execute_dexhand_test $extra_args
}

# 运行主函数
main "$@"