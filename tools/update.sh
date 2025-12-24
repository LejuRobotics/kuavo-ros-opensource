#!/bin/bash

set -e

# Function to print colored output
print_info() {
    echo -e "\033[1;34m[INFO] $1\033[0m"
}

print_error() {
    echo -e "\033[1;31m[ERROR] $1\033[0m"
}

# 检测系统架构 
detect_architecture() {
    local arch
    arch=$(uname -m)
    case $arch in
        x86_64)
            # 进一步检测是i7还是i9
            if lscpu | grep -qi "i9"; then
                echo "i9"
            elif lscpu | grep -qi "i7"; then
                echo "i7"
            else
                echo "x86_64"
            fi
            ;;
        aarch64)
            # 检测是NX还是AGX
            if cat /proc/device-tree/model | grep -qi "jetson-xavier-nx"; then
                echo "NX"
            elif cat /proc/device-tree/model | grep -qi "jetson-agx"; then
                echo "AGX"
            else
                echo "aarch64"
            fi
            ;;
        *)
            echo "unknown"
            ;;
    esac
}

update_i9(){
    print_info "update_i9 start ..."
    # 1. 更新开源EC_Master_Tools工具，删除原有的临时ec_master_tool
    # 1.1 检查EC_Master_Tools的远程仓库是否匹配
    if [ ! -d "$HOME/EC_Master_Tools" ]; then
        print_info "$HOME/EC_Master_Tools 不存在，需要克隆..."
        need_clone_EC=true
    else
        print_info "$HOME/EC_Master_Tools 存在，检查远程仓库..."
        cd "$HOME/EC_Master_Tools"
        REMOTE_URL=$(git remote get-url origin 2>/dev/null)
        if [ "${REMOTE_URL}" = "https://gitee.com/leju-robot/EC_Master_Tools.git" ]; then
            print_info "目录已存在且远程仓库正确，清理工作区..."
            need_clone_EC=false
            # 重置已经跟踪的文件到最近一次提交的状态
            git reset --hard HEAD
            # f: 强制。d: 删除所有未跟踪的文件和目录
            git clean -fd
        else
            print_info "目录存在但远程仓库不匹配。REMOTE_URL: ${REMOTE_URL}，需要克隆..."
            rm -rf "$HOME/EC_Master_Tools"
            need_clone_EC=true
        fi
    fi

    # 1.2 克隆EC_Master_Tools
    if [ "${need_clone_EC}" = true ]; then
        cd "$HOME"
        git clone https://gitee.com/leju-robot/EC_Master_Tools.git --branch master
        cd "$HOME"
    fi
    
    # 2. roban2.1声卡udev规则写入
    print_info "roban2.1声卡udev规则写入..."
    sudo bash -c 'echo -n "ACTION==\"add\", SUBSYSTEM==\"usb\", ATTR{idVendor}==\"0d8c\", ATTR{idProduct}==\"013c\", RUN+=\"/sbin/modprobe snd-usb-audio\"" > /etc/udev/rules.d/90-roban-soundcard.rules'
    
    # 3. 关闭todeskd服务
    print_info "关闭todeskd服务..."
    sudo systemctl stop todeskd.service
    sudo systemctl disable todeskd.service

    print_info "update_i9 finish..."
}

main() {
    # 检查运行的用户
    if [ "$(id -u)" -eq 0 ]; then
        print_error "请不要以root用户运行此脚本！"
        print_error "请使用普通用户权限运行，需要时脚本会通过sudo请求权限。"
        exit 1
    fi

    if [ $# -ne 1 ]; then
        print_error "参数错误: 需要指定一个架构参数"
        print_info "示例: ./update.sh i7"
        print_info "示例: ./update.sh nx"
        print_info "示例: ./update.sh agx"
        exit 1
    fi

    # 检查 update.sh 和 remote_update.sh 是否存在
    current_dir=$(pwd)
    if [ -f "update.sh" ] && [ -f "remote_update.sh" ]; then
        print_info "update.sh remote_update.sh 在当前目录"
    else
        print_error "有文件缺失："
        [ -f "update.sh" ] || print_error "  - update.sh 不在当前目录"
        [ -f "remote_update.sh" ] || print_error "  - remote_update.sh 不在当前目录"
        exit 1
    fi

    # 检测系统架构
    arch=$(detect_architecture)
    if [ "$arch" != "i9" ]; then
        print_info "当前架构 $arch 不是期望架构，不进行更新"
    else
        # 执行 i9 镜像更新
        print_info "执行 i9 镜像更新..."
        update_i9

        if ! command -v sshpass &> /dev/null; then
            echo "安装 sshpass..."
            sudo apt update && sudo apt install -y sshpass
        fi

        # 尝试自动设置 target_ip
        current_ip=$(hostname -I | grep -oE '192\.168\.26\.[0-9]{1,3}' | head -1)
	    print_info "当前IP地址 ${current_ip}"
        if [ "$current_ip" = "192.168.26.1" ]; then
            target_ip="192.168.26.12"
            print_info "自动设置 target_ip: ${target_ip}"
        elif [ "$current_ip" = "192.168.26.12" ]; then
            target_ip="192.168.26.1"
            print_info "自动设置 target_ip: ${target_ip}"
        else
            read -p "target_ip 无法自动设置，请输入: " target_ip
        fi

        # 转换为大写，便于比较
        remote_arch_upper=$(echo "$1" | tr '[:lower:]' '[:upper:]')
        # 根据目标设备架构进行更新
        print_info "上传 remote_update.sh 到目标设备..."
        cd "$current_dir"
        ssh_command="bash ~/remote_update.sh '$1'"
        case "$remote_arch_upper" in
            "I7")
                # 上传 remote_update.sh 到目标设备   
                sshpass -p 'leju_kuavo' scp ./remote_update.sh kuavo@"$target_ip":~/remote_update.sh
                print_info "上传完成"
		        # 目标设备执行更新脚本
                sshpass -p 'leju_kuavo' ssh kuavo@"$target_ip" "$ssh_command"
                ;;
            "NX")
                # 上传 remote_update.sh 到目标设备
                sshpass -p 'leju_kuavo' scp ./remote_update.sh kuavo_leju@"$target_ip":~/remote_update.sh
                print_info "上传完成"

                if [ -f "vnc_setup.sh" ]; then
                    print_info "vnc_setup.sh 在当前目录"
                else
                    print_error "vnc_setup.sh 不在当前目录"
                    exit 1
                fi
                # 上传 remote_update.sh 到目标设备       
                print_info "上传 vnc_setup.sh 到目标设备..."
                sshpass -p 'leju_kuavo' scp ./vnc_setup.sh kuavo_leju@"$target_ip":~/vnc_setup.sh
                print_info "上传完成"

                # 目标设备执行更新脚本
                sshpass -p 'leju_kuavo' ssh kuavo_leju@"$target_ip" "$ssh_command"
                ;;
            "AGX")
                # 上传 remote_update.sh 到目标设备        
                sshpass -p 'leju_kuavo' scp ./remote_update.sh kuavo_leju@"$target_ip":~/remote_update.sh
                print_info "上传完成"
                
                # 目标设备执行更新脚本
                sshpass -p 'leju_kuavo' ssh kuavo_leju@"$target_ip" "$ssh_command"
                ;;
            *)
                print_error " $remote_arch 不是支持的架构"
                exit 1
                ;;
        esac
    fi
    
}

# 执行主函数
main "$@"
