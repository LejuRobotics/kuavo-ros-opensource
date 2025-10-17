#!/bin/bash
if [ "$(id -u)" -ne 0 ]; then
    echo "请使用root用户执行此脚本！"
    exit 1
fi

echo "检查是否已安装 samplerate..."
pip3 show samplerate &> /dev/null
if [ $? -eq 0 ]; then
    echo "检测到已安装 samplerate，正在卸载..."
    pip3 uninstall -y samplerate
    if [ $? -eq 0 ]; then
        echo "samplerate 卸载成功。"
    else
        echo "samplerate 卸载失败！"
        exit 1
    fi
else
    echo "未检测到已安装的 samplerate。"
fi

echo "开始下载whl文件..."
wget https://kuavo.lejurobot.com/statics/samplerate-0.2.1-cp38-cp38-linux_x86_64.whl
if [ $? -eq 0 ]; then
    echo "whl文件下载成功。"
else
    echo "whl文件下载失败！"
    exit 1
fi

echo "开始安装whl包..."
pip3 install samplerate-0.2.1-cp38-cp38-linux_x86_64.whl
if [ $? -eq 0 ]; then
    echo "whl包安装成功。"
else
    echo "whl包安装失败！"
    exit 1
fi

echo "删除whl文件..."
rm -f samplerate-0.2.1-cp38-cp38-linux_x86_64.whl
echo "whl文件已删除。"
