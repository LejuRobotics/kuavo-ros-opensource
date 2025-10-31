#!/bin/bash

# KUAVO机器人启动脚本
# 提供交互式菜单选择启动仿真或实物机器人
# 工作空间: /opt/ros/leju
# 注意: 此脚本必须使用 root 权限运行

set -e  # 遇到错误立即退出

# 设置工作空间路径
WORKSPACE_DIR="/opt/ros/leju"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 打印带颜色的消息
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 检查环境
check_environment() {
    print_info "检查运行环境..."
    
    # 检查工作空间目录
    if [ ! -d "$WORKSPACE_DIR" ]; then
        print_error "未找到工作空间目录: $WORKSPACE_DIR"
        exit 1
    fi
    
    # 检查是否在正确的目录
    if [ ! -f "$WORKSPACE_DIR/src/humanoid-control/humanoid_controllers/package.xml" ]; then
        print_error "未找到humanoid_controllers包，请确保在$WORKSPACE_DIR目录中有正确的项目文件"
        exit 1
    fi
    
    # 检查ROBOT_VERSION环境变量
    if [ -z "$ROBOT_VERSION" ]; then
        print_warning "未设置ROBOT_VERSION环境变量"
        echo "请输入机器人版本号 (40/41/42/43/44/45): "
        read -r version
        case $version in
            40|41|42|43|44|45)
                export ROBOT_VERSION=$version
                print_success "已设置ROBOT_VERSION=$version"
                ;;
            *)
                print_error "无效的机器人版本号: $version"
                exit 1
                ;;
        esac
    else
        print_success "ROBOT_VERSION已设置为: $ROBOT_VERSION"
    fi
    
    # 检查是否已编译
    if [ ! -f "$WORKSPACE_DIR/devel/setup.bash" ]; then
        print_warning "未找到编译后的环境，请先在 $WORKSPACE_DIR 目录运行 catkin build humanoid_controllers"
        exit 1
    fi
    
    # 检查机器人质量配置文件
    mass_file="$HOME/.config/lejuconfig/TotalMassV$ROBOT_VERSION"
    if [ ! -f "$mass_file" ]; then
        print_warning "未找到机器人质量配置文件: $mass_file"
        print_warning "请先配置机器人质量，否则可能影响控制精度"
        echo "是否继续？(y/N): "
        read -r continue_anyway
        if [[ ! "$continue_anyway" =~ ^[Yy]$ ]]; then
            exit 1
        fi
    fi
}

# 显示主菜单
show_main_menu() {
    echo ""
    echo "=========================================="
    echo "      KUAVO机器人启动系统"
    echo "=========================================="
    echo "当前机器人版本: ${ROBOT_VERSION:-未设置}"
    echo ""
    echo "请选择启动方式:"
    echo "1) 启动仿真机器人"
    echo "2) 启动实物机器人"
    echo "3) 一键部署工具"
    echo "4) 硬件检测工具"
    echo "5) 查看环境信息"
    echo "6) 退出"
    echo ""
    echo -n "请输入选项 (1-6): "
}

# 显示仿真菜单
show_simulation_menu() {
    echo ""
    echo "=========================================="
    echo "      仿真机器人选择"
    echo "=========================================="
    echo "请选择仿真环境:"
    echo "1) Mujoco仿真 (推荐)"
    echo "2) Gazebo仿真"
    echo "3) 返回主菜单"
    echo ""
    echo -n "请输入选项 (1-4): "
}

# 显示实物机器人菜单
show_real_robot_menu() {
    echo ""
    echo "=========================================="
    echo "      实物机器人启动"
    echo "=========================================="
    echo "请选择启动模式:"
    echo "1) 正常模式"
    echo "2) 校准模式 (cali:=true)"
    echo "3) 半身模式 (仅上半身)"
    echo "4) 返回主菜单"
    echo ""
    echo -n "请输入选项 (1-4): "
}

# 显示操控器选择菜单
show_joystick_menu() {
    echo ""
    echo "=========================================="
    echo "      操控器选择"
    echo "=========================================="
    echo "请选择操控器类型:"
    echo "1) 北通阿修罗2 (bt2)"
    echo "2) H12pro 遥控器 (h12)"
    echo "3) 无操控器 (none)"
    echo "4) 返回上级菜单"
    echo ""
    echo -n "请输入选项 (1-4): "
}

# 显示系统配置菜单
show_system_config_menu() {
    echo ""
    echo "=========================================="
    echo "      一键部署工具"
    echo "=========================================="
    echo "请选择配置工具:"
    echo "1) KUAVO系统一键部署脚本"
    echo "2) 返回主菜单"
    echo ""
    echo -n "请输入选项 (1-2): "
}

# 启动一键部署工具
launch_system_config() {
    local config_choice=$1
    
    case $config_choice in
        1) # KUAVO系统自动配置
            print_info "启动KUAVO系统自动配置脚本..."
            if [ -f "$WORKSPACE_DIR/tools/setup-kuavo-ros-control.sh" ]; then
                print_success "准备执行: $WORKSPACE_DIR/tools/setup-kuavo-ros-control.sh"
                print_info "此脚本将自动配置KUAVO机器人系统环境"
                print_warning "注意：此脚本会修改系统配置，请谨慎操作"
                print_warning "注意：此脚本需要以普通用户身份运行"
                echo ""
                
                # 检查当前用户
                current_user=$(whoami)
                if [ "$current_user" = "root" ]; then
                    print_warning "检测到当前为root用户，setup脚本需要以普通用户运行"
                    print_info "请输入要切换到的普通用户名（默认为lab）:"
                    read -r target_user
                    target_user=${target_user:-lab}
                    
                    # 检查用户是否存在
                    if id "$target_user" &>/dev/null; then
                        print_info "将切换到用户 $target_user 运行配置脚本"
                        print_warning "5秒后启动配置脚本..."
                        sleep 5
                        
                        # 获取目标用户的家目录
                        target_home=$(getent passwd "$target_user" | cut -d: -f6)
                        
                        # 切换到脚本目录并以目标用户身份执行，同时设置正确的HOME环境变量
                        cd "$WORKSPACE_DIR/tools"
                        sudo -u "$target_user" HOME="$target_home" bash setup-kuavo-ros-control.sh
                        cd "$WORKSPACE_DIR"
                    else
                        print_error "用户 $target_user 不存在"
                        print_info "可用的用户："
                        cut -d: -f1,3 /etc/passwd | grep -E '^[^:]+:[0-9]{4}$' | awk -F: '{print $1}'
                        return
                    fi
                else
                    print_info "当前用户为 $current_user，可以直接运行配置脚本"
                    print_warning "5秒后启动配置脚本..."
                    sleep 5
                    
                    # 切换到脚本目录并执行
                    cd "$WORKSPACE_DIR/tools"
                    bash setup-kuavo-ros-control.sh
                    cd "$WORKSPACE_DIR"
                fi
            else
                print_error "未找到配置脚本: $WORKSPACE_DIR/tools/setup-kuavo-ros-control.sh"
                print_info "请确认脚本路径是否正确"
            fi
            ;;
        2) # 返回主菜单
            return
            ;;
        *)
            print_error "无效的配置选项: $config_choice"
            ;;
    esac
}

# 显示硬件检测菜单
show_hardware_check_menu() {
    echo ""
    echo "=========================================="
    echo "      硬件检测工具"
    echo "=========================================="
    echo "请选择检测工具:"
    echo "1) 硬件检测和配置工具"
    echo "2) 返回主菜单"
    echo ""
    echo -n "请输入选项 (1-2): "
}

# 启动硬件检测工具
launch_hardware_check() {
    local hardware_choice=$1
    
    case $hardware_choice in
        1) # 硬件检测工具
            print_info "启动硬件检测和配置工具..."
            if [ -f "$WORKSPACE_DIR/tools/check_tool/Hardware_tool.py" ]; then
                print_success "准备执行: python3 $WORKSPACE_DIR/tools/check_tool/Hardware_tool.py"
                print_info "此工具提供全面的硬件检测和配置功能"
                print_warning "注意：需要Python3环境和相关依赖"
                echo ""
                print_warning "5秒后启动硬件检测工具..."
                sleep 5
                
                # 检查Python3环境
                if command -v python3 &> /dev/null; then
                    # 切换到脚本目录并执行
                    cd "$WORKSPACE_DIR/tools/check_tool"
                    python3 Hardware_tool.py
                    cd "$WORKSPACE_DIR"
                else
                    print_error "未找到Python3环境"
                    print_info "请先安装Python3: sudo apt install python3"
                fi
            else
                print_error "未找到硬件检测脚本: $WORKSPACE_DIR/tools/check_tool/Hardware_tool.py"
                print_info "请确认脚本路径是否正确"
            fi
            ;;
        2) # 返回主菜单
            return
            ;;
        *)
            print_error "无效的硬件检测选项: $hardware_choice"
            ;;
    esac
}

# 启动仿真机器人
launch_simulation() {
    local sim_type=$1
    local launch_cmd=""
    
    print_info "准备启动仿真机器人..."
    
    # 确保环境已source
    source "$WORKSPACE_DIR/devel/setup.bash"
    
    case $sim_type in
        1) # Mujoco
            print_info "启动Mujoco仿真..."
            launch_cmd="roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch"
            ;;
        2) # Gazebo
            print_info "启动Gazebo仿真..."
            launch_cmd="roslaunch humanoid_controllers load_kuavo_gazebo_sim.launch"
            ;;
        *)
            print_error "无效的仿真类型: $sim_type"
            return 1
            ;;
    esac
    
    print_success "准备执行: $launch_cmd"
    print_info "按Ctrl+C可以停止仿真"
    echo ""
    print_warning "5秒后启动仿真..."
    sleep 5
    
    # 启动仿真
    eval "$launch_cmd"
}

# 启动实物机器人
launch_real_robot() {
    local mode=$1
    local launch_cmd=""
    local extra_params=""
    local joystick_type=""
    
    print_info "准备启动实物机器人..."
    
    # 让用户选择操控器类型
    while true; do
        show_joystick_menu
        read -r joystick_choice
        
        case $joystick_choice in
            1)
                joystick_type="bt2"
                print_success "已选择北通阿修罗2操控器"
                break
                ;;
            2)
                joystick_type="h12"
                print_success "已选择H12pro遥控器"
                break
                ;;
            3)
                joystick_type="none"
                print_success "已选择无操控器"
                break
                ;;
            4)
                print_info "返回上级菜单"
                return 1
                ;;
            *)
                print_error "无效选项，请重新选择"
                ;;
        esac
    done
    
    # 由于脚本已经检查过root权限，这里不需要再次检查
    
    # 确保环境已source
    source "$WORKSPACE_DIR/devel/setup.bash"
    
    case $mode in
        1) # 正常模式
            print_info "启动实物机器人 (正常模式)..."
            if [ "$joystick_type" = "none" ]; then
                launch_cmd="roslaunch humanoid_controllers load_kuavo_real.launch"
            else
                launch_cmd="roslaunch humanoid_controllers load_kuavo_real.launch joystick_type:=$joystick_type"
            fi
            ;;
        2) # 校准模式
            print_info "启动实物机器人 (校准模式)..."
            if [ "$joystick_type" = "none" ]; then
                launch_cmd="roslaunch humanoid_controllers load_kuavo_real.launch cali:=true"
            else
                launch_cmd="roslaunch humanoid_controllers load_kuavo_real.launch cali:=true joystick_type:=$joystick_type"
            fi
            ;;
        3) # 半身模式
            print_info "启动实物机器人 (半身模式)..."
            if [ "$joystick_type" = "none" ]; then
                launch_cmd="roslaunch humanoid_controllers load_kuavo_real_half_up_body.launch"
            else
                launch_cmd="roslaunch humanoid_controllers load_kuavo_real_half_up_body.launch joystick_type:=$joystick_type"
            fi
            ;;
        *)
            print_error "无效的启动模式: $mode"
            return 1
            ;;
    esac
    
    print_success "准备执行: $launch_cmd"
    print_info "按Ctrl+C可以停止机器人"
    echo ""
    
    # 显示操控器信息
    if [ "$joystick_type" != "none" ]; then
        print_info "操控器信息:"
        case $joystick_type in
            "bt2")
                echo "  - 操控器: 北通阿修罗2无线版"
                echo "  - 字母键切换步态: A(STANCE), B(TROT), X(JUMP), Y(WALK)"
                echo "  - 左摇杆: 前后左右控制"
                echo "  - 右摇杆: 转向和上下蹲控制"
                echo "  - Start键: 从悬挂准备阶段切换到站立"
                ;;
            "h12")
                echo "  - 操控器: H12pro遥控器"
                echo "  - 左摇杆: 前后左右控制"
                echo "  - 右摇杆: 转向和上下蹲控制"
                echo "  - Start开关: 切换到站立状态"
                echo "  - 左侧开关: 终止程序"
                ;;
        esac
        echo ""
    fi
    
    # 校准模式安全提醒
    if [ "$mode" -eq 2 ]; then
        print_warning "校准模式提醒:"
        print_warning "1. 确保机器人处于安全位置，周围无障碍物"
        print_warning "2. 准备扶住机器人，防止跌倒"
        print_warning "3. 电机可能会移动，注意安全"
        print_warning "4. 校准完成后按'o'键启动机器人"
        echo ""
        print_warning "10秒后启动校准模式，请做好准备..."
        sleep 10
    else
        print_warning "5秒后启动实物机器人..."
        sleep 5
    fi
    
    # 启动实物机器人
    eval "$launch_cmd"
}

# 显示环境信息
show_environment_info() {
    echo ""
    echo "=========================================="
    echo "      环境信息"
    echo "=========================================="
    echo "工作空间目录: $WORKSPACE_DIR"
    echo "当前目录: $(pwd)"
    echo "ROBOT_VERSION: ${ROBOT_VERSION:-未设置}"
    echo "用户权限: $(whoami)"
    echo "编译状态: $([ -f "$WORKSPACE_DIR/devel/setup.bash" ] && echo "已编译" || echo "未编译")"
    
    # 检查机器人质量配置
    mass_file="$HOME/.config/lejuconfig/TotalMassV$ROBOT_VERSION"
    if [ -f "$mass_file" ]; then
        mass=$(cat "$mass_file")
        echo "机器人质量: ${mass} kg"
    else
        echo "机器人质量: 未配置"
    fi
    
    # 检查校准文件
    offset_file="$HOME/.config/lejuconfig/offset.csv"
    if [ -f "$offset_file" ]; then
        echo "腿部零位校准: 已配置"
    else
        echo "腿部零位校准: 未配置"
    fi
    
    arm_zero_file="$HOME/.config/lejuconfig/arms_zero.yaml"
    if [ -f "$arm_zero_file" ]; then
        echo "手臂零位校准: 已配置"
    else
        echo "手臂零位校准: 未配置"
    fi
    
    echo ""
    echo "按回车键返回主菜单..."
    read -r
}

# 检查root权限
check_root_permission() {
    if [ "$EUID" -ne 0 ]; then
        print_error "此脚本必须使用 root 权限运行"
        print_info "请使用以下命令重新运行:"
        echo "  sudo $0"
        echo "  或"
        echo "  sudo ./scripts/startup.sh"
        exit 1
    fi
    
    print_success "Root 权限检查通过"
}

# 主程序
main() {
    # 检查root权限
    check_root_permission
    
    # 检查环境
    check_environment
    
    while true; do
        show_main_menu
        read -r main_choice
        
        case $main_choice in
            1) # 仿真机器人
                while true; do
                    show_simulation_menu
                    read -r sim_choice
                    
                    case $sim_choice in
                        1|2|3)
                            launch_simulation "$sim_choice"
                            break
                            ;;
                        4) # 返回主菜单
                            break
                            ;;
                        *)
                            print_error "无效选项，请重新选择"
                            ;;
                    esac
                done
                ;;
            2) # 实物机器人
                while true; do
                    show_real_robot_menu
                    read -r real_choice
                    
                    case $real_choice in
                        1|2|3)
                            launch_real_robot "$real_choice"
                            break
                            ;;
                        4) # 返回主菜单
                            break
                            ;;
                        *)
                            print_error "无效选项，请重新选择"
                            ;;
                    esac
                done
                ;;
            3) # 一键部署工具
                while true; do
                    show_system_config_menu
                    read -r config_choice
                    
                    case $config_choice in
                        1|2)
                            launch_system_config "$config_choice"
                            break
                            ;;
                        *)
                            print_error "无效选项，请重新选择"
                            ;;
                    esac
                done
                ;;
            4) # 硬件检测工具
                while true; do
                    show_hardware_check_menu
                    read -r hardware_choice
                    
                    case $hardware_choice in
                        1|2)
                            launch_hardware_check "$hardware_choice"
                            break
                            ;;
                        *)
                            print_error "无效选项，请重新选择"
                            ;;
                    esac
                done
                ;;
            5) # 环境信息
                show_environment_info
                ;;
            6) # 退出
                print_info "感谢使用KUAVO机器人启动系统"
                exit 0
                ;;
            *)
                print_error "无效选项，请重新选择"
                ;;
        esac
    done
}

# 脚本入口
if [ "${BASH_SOURCE[0]}" = "${0}" ]; then
    main "$@"
fi