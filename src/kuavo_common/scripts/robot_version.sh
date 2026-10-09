#!/bin/bash
# ============================================================================
# 版本注册表 —— 全项目 Shell 端唯一源头
# 新增版本只需在此处加一行，其余逻辑自动生效。
#
# 用法:
#   source "/path/to/src/kuavo_common/scripts/robot_version.sh"
#   或通过项目根目录:
#   source "$PROJECT_ROOT/src/kuavo_common/scripts/robot_version.sh"
#
# 格式: "显示版本|内部版本|资源系列|描述"
#   - 显示版本: 用户输入/展示用的版本号
#   - 内部版本: 写入 ROBOT_VERSION 的数值
#   - 资源系列: 用于选择资源目录和机型判断 (kuavo4/kuavo5/kuavo5w/roban)
#   - 描述: 版本说明
#
# Python 端对应注册表: src/kuavo_common/python/robot_version.py (保持同步)
# ============================================================================

_VERSION_REGISTRY=(
    "42|42|kuavo4|短臂版本"
    "45|45|kuavo4|长臂版本"
    "49|49|kuavo4|pro max版本"
    "45.1|100045|kuavo4|假手版"
    "49.1|100049|kuavo4|展厅版"
    "52|52|kuavo5|普通kuavo5"
    "53|53|kuavo5|手臂pitch电机改ruiwo"
    "55|55|kuavo5|手臂部分电机改ruiwoPA4310"
    "60|60|kuavo5w|悟时底盘轮臂"
    "61|61|kuavo5w|玖物底盘轮臂"
    "13|13|roban|roban2.0版本"
    "14|14|roban|roban2.1版本"
    "15|15|roban|roban2.2版本"
)

# 获取所有合法的显示版本号（空格分隔）
get_valid_display_versions() {
    local versions=()
    for entry in "${_VERSION_REGISTRY[@]}"; do
        IFS='|' read -r disp _ _ _ <<< "$entry"
        versions+=("$disp")
    done
    echo "${versions[@]}"
}

# 校验版本号是否合法
# 用法: is_valid_version "55" && echo "valid" || echo "invalid"
is_valid_version() {
    local target="$1"
    for entry in "${_VERSION_REGISTRY[@]}"; do
        IFS='|' read -r disp _ _ _ <<< "$entry"
        if [[ "$target" == "$disp" ]]; then
            return 0
        fi
    done
    return 1
}

# 获取版本对应的内部版本号 (处理 45.1 -> 100045 等转换)
# 用法: ver=$(get_version_internal "45.1")  # ver=100045
get_version_internal() {
    local target="$1"
    for entry in "${_VERSION_REGISTRY[@]}"; do
        IFS='|' read -r disp ver _ _ <<< "$entry"
        if [[ "$target" == "$disp" ]]; then
            echo "$ver"
            return 0
        fi
    done
    echo "$target"
}

# 获取版本所属的资源系列 (kuavo4/kuavo5/kuavo5w/roban)
# 用法: series=$(get_version_series "55")  # series=kuavo5
get_version_series() {
    local target="$1"
    for entry in "${_VERSION_REGISTRY[@]}"; do
        IFS='|' read -r disp _ series _ <<< "$entry"
        if [[ "$target" == "$disp" ]]; then
            echo "$series"
            return 0
        fi
    done
    echo "kuavo4"
}

# 获取版本描述
# 用法: desc=$(get_version_desc "55")
get_version_desc() {
    local target="$1"
    for entry in "${_VERSION_REGISTRY[@]}"; do
        IFS='|' read -r disp _ _ desc <<< "$entry"
        if [[ "$target" == "$disp" ]]; then
            echo "$desc"
            return 0
        fi
    done
    echo ""
}

# 打印所有可用版本（带描述）
print_available_versions() {
    for entry in "${_VERSION_REGISTRY[@]}"; do
        IFS='|' read -r disp _ _ desc <<< "$entry"
        echo "  $disp ($desc)"
    done
}
