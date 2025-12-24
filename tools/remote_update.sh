#!/bin/bash

set -e

# Function to print colored output
print_info() {
    echo -e "\033[1;34m[INFO] $1\033[0m"
}

print_error() {
    echo -e "\033[1;31m[ERROR] $1\033[0m"
}

update_i7(){

    print_info "update_i7 start ..."

    # 1. LED测试脚本所需环境依赖
    print_info "安装LED测试脚本所需环境依赖..."
    python3 -m pip install -i https://pypi.tuna.tsinghua.edu.cn/simple pyserial==3.5

    # 2. 豆包实时语音大模型案例依赖
    print_info "安装豆包实时语音大模型案例依赖..."
    sudo apt install -y portaudio19-dev python3-pyaudio 
    pip install samplerate==0.1.0 -i https://pypi.tuna.tsinghua.edu.cn/simple
    pip install librosa==0.11.0 -i https://pypi.tuna.tsinghua.edu.cn/simple
    pip uninstall -y torch torchvision
    pip install torchvision -i https://pypi.tuna.tsinghua.edu.cn/simple
    pip install torch>=2.0.0 -i https://pypi.tuna.tsinghua.edu.cn/simple

    print_info "update_i7 finish..."
}

update_nx(){
    print_info "update_nx start ..."

    # 1. LED测试脚本环境依赖
    print_info "安装LED测试脚本所需环境依赖..."
    python3 -m pip install -i https://pypi.tuna.tsinghua.edu.cn/simple pyserial==3.5

    # 2. 豆包实时语音大模型案例依赖
    print_info "安装豆包实时语音大模型案例依赖..."
    sudo apt install -y portaudio19-dev python3-pyaudio 
    pip install samplerate==0.2.1 -i https://pypi.tuna.tsinghua.edu.cn/simple
    pip install librosa==0.11.0 -i https://pypi.tuna.tsinghua.edu.cn/simple
    pip uninstall -y torch torchvision
    pip install torchvision -i https://pypi.tuna.tsinghua.edu.cn/simple
    pip install torch>=2.0.0 -i https://pypi.tuna.tsinghua.edu.cn/simple

    # 3. 上下键补全 
    print_info "安装上下键补全..."
    # -e 开启转义字符解释 
    grep -q 'bind.*"\\e\[A": history-search-backward' ~/.bashrc || echo 'bind '"'"'"\e[A": history-search-backward'"'" >> ~/.bashrc
    grep -q 'bind.*"\\e\[B": history-search-forward' ~/.bashrc || echo 'bind '"'"'"\e[B": history-search-forward'"'" >> ~/.bashrc
    source ~/.bashrc

    # 新增VNC
    print_info "安装VNC服务..."
    if [ ! -f "$HOME/vnc_setup.sh" ]; then
        print_error "$HOME/vnc_setup.sh 不存在，VNC安装失败..."
    else
        print_info "执行 $HOME/vnc_setup.sh 安装VNC服务..."
        sudo bash "$HOME/vnc_setup.sh"
    fi

    # 新增中文输入法 
    print_info "新增中文输入法..."
    # sudo apt install -y fcitx fcitx-googlepinyin fcitx-config-gtk
    sudo apt install -y ibus ibus-libpinyin && im-config -n ibus && ibus-daemon --daemonize --xim

    print_info "update_nx finish..."
    print_info "重启即可使用中文输入法..."
}

update_agx(){
    print_error "update_agx 暂未实现..."
}

main() {
    # 检查运行的用户
    if [ "$(id -u)" -eq 0 ]; then
        print_error "请不要以root用户运行此脚本！"
        print_error "请使用普通用户权限运行，需要时脚本会通过sudo请求权限。"
        exit 1
    fi

    # 检查参数
    if [ $# -ne 1 ]; then
        print_error "参数错误: 需要指定一个架构参数"
        print_info "示例: ./remote_update.sh i7"
        print_info "示例: ./remote_update.sh nx"
        print_info "示例: ./remote_update.sh agx"
        exit 1
    fi

    # 默认全部免密
    SUDO_PASSWORD="leju_kuavo"
    echo "$SUDO_PASSWORD" | sudo -S sh -c "echo 'kuavo ALL=(ALL) NOPASSWD:ALL' | tee -a /etc/sudoers"

    # 换镜像源
    sudo cp /etc/apt/sources.list /etc/apt/sources.list.bak 
    sudo sed -i 's|http://.*ubuntu.com|https://mirrors.tuna.tsinghua.edu.cn|g' /etc/apt/sources.list 
    sudo sed -i 's|https://.*ubuntu.com|https://mirrors.tuna.tsinghua.edu.cn|g' /etc/apt/sources.list 
    sudo apt update

    # 转换为大写
    arch_upper=$(echo "$1" | tr '[:lower:]' '[:upper:]')
    # 根据入参选择更新
    case "$arch_upper" in
        "I7")
            print_info "执行 i7 镜像更新..."
            update_i7
            ;;
        "NX")
            print_info "执行 NX 镜像更新..."
            update_nx
            ;;
        "AGX")
            print_info "执行 AGX 镜像更新..."
            update_agx
            ;;
        *)
            print_error "不支持的架构: $arch"
            print_info "支持的架构: i7, NX, AGX"
            exit 1
            ;;
    esac
}

# 执行主函数
main "$@"
rm -f "$0"
