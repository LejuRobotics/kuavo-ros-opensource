#!/bin/bash
SCRIPT_DIR=$(dirname "$(realpath "$0")")
PROJECT_DIR=$(realpath "$SCRIPT_DIR/../../") # project: kuavo-ros-control/kuavo-ros-opensource

# 配置文件路径定义
CONFIG_DIR="$HOME/.config/lejuconfig"
CANBUS_WIRING_TYPE_FILE="$CONFIG_DIR/CanbusWiringType.ini"
HAND_PROTOCOL_TYPE_FILE="$CONFIG_DIR/HandProtocolType.ini"
CANBUS_CONFIG_FILE="$CONFIG_DIR/canbus_device_cofig.yaml"

# Roban2-0
ROBAN2_0_DUAL_SOURCE_CONFIG_FILE="$PROJECT_DIR/src/kuavo_assets/config/roban2-0_dual_canbus_cofig.yaml"
ROBAN2_0_SINGLE_SOURCE_CONFIG_FILE="$PROJECT_DIR/src/kuavo_assets/config/roban2-0_single_canbus_cofig.yaml"

# Roban2-1
ROBAN2_1_DUAL_SOURCE_CONFIG_FILE="$PROJECT_DIR/src/kuavo_assets/config/roban2-1_dual_canbus_cofig.yaml"
ROBAN2_1_SINGLE_SOURCE_CONFIG_FILE="$PROJECT_DIR/src/kuavo_assets/config/roban2-1_single_canbus_cofig.yaml"

# Kuavo5
KUAVO5_DUAL_SOURCE_CONFIG_FILE="$PROJECT_DIR/src/kuavo_assets/config/kuavo5_dual_canbus_cofig.yaml"
KUAVO5_SINGLE_SOURCE_CONFIG_FILE="$PROJECT_DIR/src/kuavo_assets/config/kuavo5_single_canbus_cofig.yaml"

# Kuavo5 v53
KUAVO5_V53_DUAL_SOURCE_CONFIG_FILE="$PROJECT_DIR/src/kuavo_assets/config/kuavo5_v53_dual_canbus_cofig.yaml"

# Kuavo4pro
KUAVO4PRO_SINGLE_SOURCE_CONFIG_FILE="$PROJECT_DIR/src/kuavo_assets/config/kuavo4pro_single_canbus_cofig.yaml"
KUAVO4PRO_DUAL_SOURCE_CONFIG_FILE="$PROJECT_DIR/src/kuavo_assets/config/kuavo4pro_dual_canbus_cofig.yaml"

# 打印带颜色的标题
echo_title() {
    echo -e "\033[1m\033[0;34m=== $1 ===\033[0m"
}

echo_info() {
    echo -e "\033[0;36m$1\033[0m"
}

echo_success() {
    echo -e "\033[0;32m$1\033[0m"
}

echo_warning() {
    echo -e "\033[1;33m$1\033[0m"
}

echo_error() {
    echo -e "\033[0;31m$1\033[0m"
}

echo_error_stderr() {
    echo -e "\033[0;31m$1\033[0m"
}


# 显示菜单选项函数
show_menu() {
    local title="$1"
    shift
    local options=("$@")

    echo_title "$title"
    for i in "${!options[@]}"; do
        if [ $((i+1)) -le 9 ]; then
            echo -e "\033[0;32m$((i+1)).\033[0m \033[1;37m${options[$i]}\033[0m"
        else
            echo -e "\033[0;32m$((i+1)).\033[0m \033[1;37m${options[$i]}\033[0m"
        fi
    done
}

# 获取用户选择函数
get_user_selection() {
    local -n result_ref=$2
    local max=$1
    local selection

    while true; do
        read -p "请选择(1-$max): " selection
        if [[ "$selection" =~ ^[0-9]+$ ]] && [ "$selection" -ge 1 ] && [ "$selection" -le "$max" ]; then
            result_ref="$selection"
            return
        else
            echo_error_stderr "无效选择，请输入 1-$max"
        fi
    done
}

# 选择CAN总线接线类型函数
select_wiring_type() {
    local -n result_ref=$1
    local wiring_options=("dual_bus -- 双总线, 左右手臂各接一个 CAN 模块" "single_bus -- 单总线, 左右手臂共用一个 CAN 模块")
    show_menu "选择 CAN 总线接线类型" "${wiring_options[@]}"
    get_user_selection 2 wiring_selection

    local wiring_types=("dual_bus" "single_bus")
    local selected_type="${wiring_types[$((wiring_selection-1))]}"
    local selected_description="${wiring_options[$((wiring_selection-1))]}"

    echo_success "✓ 选择: $selected_description"

    result_ref="$selected_type"
}

select_hand_protocol_type() {
    local -n result_ref=$1
    local hand_protocol_options=("proto_buf -- 485协议" "proto_can -- CAN协议")
    show_menu "选择手部协议类型" "${hand_protocol_options[@]}"
    get_user_selection 2 hand_protocol_selection

    local hand_protocol_types=("proto_buf" "proto_can")
    local selected_type="${hand_protocol_types[$((hand_protocol_selection-1))]}"
    local selected_description="${hand_protocol_options[$((hand_protocol_selection-1))]}"

    echo_success "✓ 选择: $selected_description"

    result_ref="$selected_type"
}

# 选择末端执行器类型函数
select_end_effector_type() {
    local -n result_ref=$2
    local side="$1"  # 左、右或具体描述
    local end_effector_options=("revo1_hand -- 灵巧手" "revo2_hand -- 灵巧手" "lejuclaw -- 自研夹爪" "none -- 没有末端")
    show_menu "选择${side}末端执行器类型" "${end_effector_options[@]}"
    local eef_selection
    get_user_selection 4 eef_selection

    local end_effector_types=("revo1_hand" "revo2_hand" "lejuclaw" "none")
    local selected_type="${end_effector_types[$((eef_selection-1))]}"
    local selected_description="${end_effector_options[$((eef_selection-1))]}"

    echo_success "✓ ${side}末端执行器: $selected_type"

    result_ref="$selected_type"
}


# 根据类型获取末端执行器名称
get_end_effector_name() {
    local side="$1"  # L 或 R
    local type="$2"  # revo2_hand, lejuclaw, none

    case "$type" in
        "revo1_hand")
            echo "${side}hand_revo1_hand"
            ;;
        "revo2_hand")
            echo "${side}hand_revo2_hand"
            ;;
        "lejuclaw")
            echo "${side}lejuclaw"
            ;;
        "none")
            echo "${side}hand_none"
            ;;
    esac
}

# 根据类型获取末端执行器类名
get_end_effector_class() {
    local type="$2"  # revo2_hand, lejuclaw, none

    case "$type" in
        "revo1_hand"|"revo2_hand"|"lejuclaw")
            echo "$type"
            ;;
        "none")
            echo "none"
            ;;
    esac
}

# 根据类型和侧获取设备ID
get_end_effector_device_id() {
    local side="$1"  # L 或 R
    local type="$2"  # revo2_hand, lejuclaw, none
    local robot_type="$3"  # 机器人型号，用于区分不同机型的ID规则

    case "$type" in
        "revo1_hand")
            if [ "$side" = "L" ]; then
                echo "0x01"
            else
                echo "0x02"
            fi
            ;;
        "revo2_hand")
            if [ "$side" = "L" ]; then
                echo "0x01"
            else
                echo "0x02"
            fi
            ;;
        "lejuclaw")
            # kuavo5_v53 使用 0x11/0x12，其他机型使用 0x0F/0x10
            if [ "$robot_type" = "kuavo5_v53" ]; then
                if [ "$side" = "L" ]; then
                    echo "0x11"
                else
                    echo "0x12"
                fi
            else
                if [ "$side" = "L" ]; then
                    echo "0x0F"
                else
                    echo "0x10"
                fi
            fi
            ;;
        "none")
            echo "0x00"
            ;;
    esac
}

# 选择CANBUS类型函数
select_canbus_type() {
    local -n result_ref=$2
    local side="$1"  # L 或 R
    local canbus_options=("BUSMUST_A -- BUSMUST A 款" "BUSMUST_B -- BUSMUST B 款" 
                          "LEJU_CAN_A -- 乐聚 CAN_A 模块" "LEJU_CAN_B -- 乐聚 CAN_B 模块")
    show_menu "选择${side}侧CANBUS类型" "${canbus_options[@]}"
    get_user_selection 4 canbus_selection
    local canbus_types=("BUSMUST_A" "BUSMUST_B" "LEJU_CAN_A" "LEJU_CAN_B")
    local canbus_type="${canbus_types[$((canbus_selection-1))]}"
    echo_success "✓ 选择${side}侧CANBUS类型: $canbus_type"

    result_ref="$canbus_type"
}

# 写入配置文件函数
write_config_files() {
    local wiring_type="$1"
    local config_file="$2"
    local hand_protocol_type="$3"
    
    # 写入CAN总线接线类型
    mkdir -p "$CONFIG_DIR"
    echo "$wiring_type" > "$CANBUS_WIRING_TYPE_FILE"
    echo_success "✓ CAN总线接线类型已保存到: $CANBUS_WIRING_TYPE_FILE"

    # 写入手部协议类型
    if [ -n "$hand_protocol_type" ]; then
        mkdir -p "$CONFIG_DIR"
        echo "$hand_protocol_type" > "$HAND_PROTOCOL_TYPE_FILE"
        echo_success "✓ 手部协议类型已保存到: $HAND_PROTOCOL_TYPE_FILE"
    fi

    # 拷贝最终CANBUS配置文件
    if [ -n "$config_file" ] && [ -f "$config_file" ]; then
        cp "$config_file" "$CANBUS_CONFIG_FILE"
        echo_success "✓ CANBUS配置文件已保存到: $CANBUS_CONFIG_FILE"
        cat "$CANBUS_CONFIG_FILE"
    fi
}

# 更新单个CANBUS类型配置
update_canbus_type() {
    local config_file="$1"
    local canbus_id="$2"  # CANBUS0 或 CANBUS1
    local canbus_type="$3"

    case "$canbus_id" in
        "CANBUS0")
            sed -i "/# ANCHOR_CANBUS0_TYPE/c\    type: $canbus_type" "$config_file"
            echo_success "✓ 左CANBUS类型已更新为: $canbus_type"
            ;;
        "CANBUS1")
            sed -i "/# ANCHOR_CANBUS1_TYPE/c\    type: $canbus_type" "$config_file"
            echo_success "✓ 右CANBUS类型已更新为: $canbus_type"
            ;;
        *)
            echo_error "✗ 错误: 无效的CANBUS ID: $canbus_id"
            return 1
            ;;
    esac
}

# 更新CANBUS类型配置
update_canbus_type_config() {
    local config_file="$1"
    local left_type="$2"
    local right_type="$3"

    echo "开始更新CANBUS类型配置..."

    # 更新左CANBUS类型
    update_canbus_type "$config_file" "CANBUS0" "$left_type"

    # 更新右CANBUS类型
    update_canbus_type "$config_file" "CANBUS1" "$right_type"

    echo "CANBUS类型配置更新完成"
}

# 替换单个末端执行器配置
replace_single_end_effector() {
    local config_file="$1"
    local side="$2"  # L 或 R
    local type="$3"
    local robot_type="$4"  # 机器人型号

    if [ "$type" != "none" ]; then
        local name=$(get_end_effector_name "$side" "$type")
        local class=$(get_end_effector_class "$side" "$type")
        local device_id=$(get_end_effector_device_id "$side" "$type" "$robot_type")

        if [ "$side" = "L" ]; then
            sed -i "/# ANCHOR_L_NAME/c\  - name: $name        # ANCHOR_L_NAME" "$config_file"
            sed -i "/# ANCHOR_L_CLASS/c\    class: $class             # ANCHOR_L_CLASS" "$config_file"
            sed -i "/# ANCHOR_L_DEVICE_ID/c\    device_id: $device_id               # ANCHOR_L_DEVICE_ID" "$config_file"
            sed -i "/# ANCHOR_L_IGNORE/c\    ignore: false                 # ANCHOR_L_IGNORE" "$config_file"
            # 若为 lejuclaw，补充必要的参数
            if [ "$type" = "lejuclaw" ]; then
                # 追加顺序：params -> ratio -> negtive（使用倒序插入以保证最终顺序）
                sed -i "/# ANCHOR_L_DEVICE_ID/a\    negtive: false" "$config_file"
                sed -i "/# ANCHOR_L_DEVICE_ID/a\    ratio: 25" "$config_file"
                sed -i "/# ANCHOR_L_DEVICE_ID/a\    params: [0, 0, 0, 0, 0, 0, 0]" "$config_file"
            fi
        else
            sed -i "/# ANCHOR_R_NAME/c\  - name: $name        # ANCHOR_R_NAME" "$config_file"
            sed -i "/# ANCHOR_R_CLASS/c\    class: $class             # ANCHOR_R_CLASS" "$config_file"
            sed -i "/# ANCHOR_R_DEVICE_ID/c\    device_id: $device_id               # ANCHOR_R_DEVICE_ID" "$config_file"
            sed -i "/# ANCHOR_R_IGNORE/c\    ignore: false                 # ANCHOR_R_IGNORE" "$config_file"
            # 若为 lejuclaw，补充必要的参数
            if [ "$type" = "lejuclaw" ]; then
                # 追加顺序：params -> ratio -> negtive（使用倒序插入以保证最终顺序）
                sed -i "/# ANCHOR_R_DEVICE_ID/a\    negtive: false" "$config_file"
                sed -i "/# ANCHOR_R_DEVICE_ID/a\    ratio: 25" "$config_file"
                sed -i "/# ANCHOR_R_DEVICE_ID/a\    params: [0, 0, 0, 0, 0, 0, 0]" "$config_file"
            fi
        fi

        # TODO: 也许需要对夹爪增加额外的配置，比如电机的默认参数等...

        echo "${side}末端执行器已更新为: $type ($name)"
    else
        # 删除整个末端执行器配置块
        if [ "$side" = "L" ]; then
            sed -i "/# ANCHOR_L_NAME/,/# ANCHOR_L_IGNORE.*$/d" "$config_file"
        else
            sed -i "/# ANCHOR_R_NAME/,/# ANCHOR_R_IGNORE.*$/d" "$config_file"
        fi
        echo "${side}末端执行器已删除"
    fi
}

# 替换末端执行器配置函数
replace_end_effector_config() {
    local config_file="$1"
    local left_type="$2"
    local right_type="$3"
    local robot_type="$4"  # 机器人型号

    echo "开始更新末端执行器配置..."

    replace_single_end_effector "$config_file" "L" "$left_type" "$robot_type"
    replace_single_end_effector "$config_file" "R" "$right_type" "$robot_type"

    echo "末端执行器配置更新完成"
}

# 配置roban2机器人函数
configure_roban2() {
    local robot_type="$1"
    echo_success "🤖 配置 $robot_type 机器人"

    # 选择CAN总线接线类型
    local wiring_type
    select_wiring_type wiring_type

    # roban 配置单总线直接打印成功并退出脚本
    if [ "$wiring_type" = "single_bus" ]; then
        echo_success "✓ 配置完成: $robot_type 单总线模式"
        exit 0
    fi

    # 选择手部协议类型
    local hand_protocol_type
    select_hand_protocol_type hand_protocol_type

    # 根据robot_type选择配置文件路径
    local dual_config_file=""
    local single_config_file=""
    case "$robot_type" in
        "roban2.0")
            dual_config_file="$ROBAN2_0_DUAL_SOURCE_CONFIG_FILE"
            single_config_file="$ROBAN2_0_SINGLE_SOURCE_CONFIG_FILE"
            ;;
        "roban2.1")
            dual_config_file="$ROBAN2_1_DUAL_SOURCE_CONFIG_FILE"
            single_config_file="$ROBAN2_1_SINGLE_SOURCE_CONFIG_FILE"
            ;;
    esac

    # 初始化配置文件变量
    local config_file=""

    # 如果是双总线，配置CANBUS类型和末端执行器
    if [ "$wiring_type" = "dual_bus" ]; then
        # 选择左CANBUS类型
        local left_canbus_type
        select_canbus_type "左" left_canbus_type

        # 选择右CANBUS类型
        local right_canbus_type
        select_canbus_type "右" right_canbus_type

        echo ""
        echo_title "配置末端执行器类型"

        # 选择左末端执行器
        local left_type
        select_end_effector_type "左" left_type

        # 选择右末端执行器
        local right_type
        select_end_effector_type "右" right_type

        # 拷贝并修改配置文件
        local temp_file="/tmp/roban2_canbus_device_cofig.yaml"

        if [ -f "$dual_config_file" ]; then
            cp "$dual_config_file" "$temp_file"
            echo_success "✓ 配置文件已拷贝到: $temp_file"

            # 更新CANBUS类型配置
            update_canbus_type_config "$temp_file" "$left_canbus_type" "$right_canbus_type"

            # 替换末端执行器配置
            replace_end_effector_config "$temp_file" "$left_type" "$right_type" "$robot_type"
            echo_success "✓ 配置文件已更新: $temp_file"
            config_file="$temp_file"
        else
            echo_error "✗ 错误: 源配置文件不存在: $dual_config_file"
            config_file=""
        fi
    # else

    #     echo ""
    #     echo_title "配置单总线CANBUS类型"
    #     local single_bus_canbus_type
    #     select_canbus_type "单总线" single_bus_canbus_type
    #     echo_success "✓ 选择单总线CANBUS类型: $single_bus_canbus_type"

    #     # 拷贝并修改配置文件
    #     local temp_file="/tmp/roban2_canbus_device_cofig.yaml"

    #     if [ -f "$single_config_file" ]; then
    #         cp "$single_config_file" "$temp_file"
    #         echo_success "✓ 配置文件已拷贝到: $temp_file"
    #         # 更新CANBUS0类型为单总线类型
    #         update_canbus_type "$temp_file" "CANBUS0" "$single_bus_canbus_type"
    #         config_file="$temp_file"
    #     else
    #         echo_error "✗ 错误: 源配置文件不存在: $single_config_file"
    #         config_file=""
    #     fi    
    fi

    # 统一写入所有配置文件
    write_config_files "$wiring_type" "$config_file" "$hand_protocol_type"
}

# 配置kuavo机器人函数
configure_kuavo() {
    local robot_type="$1"
    echo_success "🤖 配置 $robot_type 机器人"

    # 选择CAN总线接线类型
    local wiring_type
    # kuavo5_v53版本只支持双总线
    if [ "$robot_type" = "kuavo5_v53" ]; then
        wiring_type="dual_bus"
        echo_success "✓ kuavo5_v53版本仅支持双总线模式"
    else
        select_wiring_type wiring_type
    fi

    # 选择手部协议类型
    local hand_protocol_type
    select_hand_protocol_type hand_protocol_type

    # 根据robot_type选择配置文件路径
    local dual_config_file=""
    case "$robot_type" in
        "kuavo5")
            dual_config_file="$KUAVO5_DUAL_SOURCE_CONFIG_FILE"
            ;;
        "kuavo5_v53")
            dual_config_file="$KUAVO5_V53_DUAL_SOURCE_CONFIG_FILE"
            # kuavo5_v53版本不支持单总线，不需要single_config_file
            single_config_file=""
            ;;
    esac

    # 初始化配置文件变量
    local config_file=""

    # 配置CANBUS类型和末端执行器
    # 选择左CANBUS类型
    local left_canbus_type
    select_canbus_type "左" left_canbus_type

    # 选择右CANBUS类型
    local right_canbus_type
    select_canbus_type "右" right_canbus_type

    echo ""
    echo_title "配置末端执行器类型"

    # 选择左末端执行器
    local left_type
    select_end_effector_type "左" left_type

    # 选择右末端执行器
    local right_type
    select_end_effector_type "右" right_type

    # 拷贝并修改配置文件
    local temp_file="/tmp/kuavo_canbus_device_cofig.yaml"

    if [ -f "$dual_config_file" ]; then
        cp "$dual_config_file" "$temp_file"
        echo_success "✓ 配置文件已拷贝到: $temp_file"

        # 更新CANBUS类型配置
        update_canbus_type_config "$temp_file" "$left_canbus_type" "$right_canbus_type"

        # 替换末端执行器配置
        replace_end_effector_config "$temp_file" "$left_type" "$right_type"
        echo_success "✓ 配置文件已更新: $temp_file"
        config_file="$temp_file"
    else
        # kuavo5_v53版本不支持单总线
        if [ "$robot_type" = "kuavo5_v53" ]; then
            echo_error "✗ 错误: kuavo5_v53版本不支持单总线模式"
            return 1
        fi

        echo ""
        echo_title "配置单总线CANBUS类型"
        local single_bus_canbus_type
        select_canbus_type "单总线" single_bus_canbus_type
        echo_success "✓ 选择单总线CANBUS类型: $single_bus_canbus_type"

        # 拷贝并修改配置文件
        local temp_file="/tmp/kuavo_canbus_device_cofig.yaml"

        if [ -f "$single_config_file" ]; then
            cp "$single_config_file" "$temp_file"
            echo_success "✓ 配置文件已拷贝到: $temp_file"
            # 更新CANBUS0类型为单总线类型
            update_canbus_type "$temp_file" "CANBUS0" "$single_bus_canbus_type"
            config_file="$temp_file"
        else
            echo_error "✗ 错误: 源配置文件不存在: $single_config_file"
            config_file=""
        fi
    fi

    # 统一写入所有配置文件
    write_config_files "$wiring_type" "$config_file" "$hand_protocol_type"
}


# 主函数
main() {
    echo_title "🔧 CANBUS 配置脚本"
    echo ""

    # 检查接线和canbus配置文件是否存在，如果存在则提示用户是否覆盖
    if [ -f "$CANBUS_WIRING_TYPE_FILE" ] || [ -f "$CANBUS_CONFIG_FILE" ] || [ -f "$HAND_PROTOCOL_TYPE_FILE" ]; then
        echo_warning "⚠️  检测到已存在的配置文件:"
        [ -f "$CANBUS_WIRING_TYPE_FILE" ] && echo_info "  - $CANBUS_WIRING_TYPE_FILE"
        [ -f "$CANBUS_CONFIG_FILE" ] && echo_info "  - $CANBUS_CONFIG_FILE"
        [ -f "$HAND_PROTOCOL_TYPE_FILE" ] && echo_info "  - $HAND_PROTOCOL_TYPE_FILE"
        echo ""

        local overwrite_options=("是 -- 覆盖现有配置" "否 -- 保留现有配置")
        show_menu "是否覆盖现有配置" "${overwrite_options[@]}"
        get_user_selection 2 overwrite_option

        if [[ "$overwrite_option" == "2" ]]; then
            echo_success "✓ 保留现有配置，退出脚本"
            exit 0
        fi
        echo_success "✓ 将覆盖现有配置"
    fi

    # 选择机器人类型
    local robot_options=("roban2.0" "roban2.1" "kuavo4pro" "kuavo5" "kuavo5_v53")
    show_menu "选择机器人类型" "${robot_options[@]}"
    get_user_selection 5 robot_selection

    local robot_type="${robot_options[$((robot_selection-1))]}"
    echo_success "选择机器人类型: $robot_type"
    echo ""

    # kuavo4pro 暂不需要该配置，直接跳过并退出
    if [ "$robot_type" = "kuavo4pro" ]; then
        echo_warning "kuavo4pro 暂不需要该配置，已跳过。"
        exit 0
    fi

    # 根据机器人类型进行配置
    case "$robot_type" in
        "roban2.0"|"roban2.1")
            configure_roban2 "$robot_type"
            ;;
        "kuavo4pro"|"kuavo5"|"kuavo5_v53")
            configure_kuavo "$robot_type"
            ;;
    esac

    echo_success "\n🎉 配置完成!"
}

# 运行主函数
main "$@"
