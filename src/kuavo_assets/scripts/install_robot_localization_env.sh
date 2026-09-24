#!/bin/sh
# 检查 libgeographic-dev 是否已安装
if ! dpkg -l | grep -q "libgeographic-dev"; then
    echo "\033[33m正在安装 libgeographic-dev...\033[0m"
    sudo apt-get install libgeographic-dev -y
# else
#     echo "\033[33mlibgeographic-dev 已安装.\033[0m"
fi

# Jammy 无 ros-noetic-geographic deb；源码 ROS 只需 libgeographic-dev
if grep -q '22.04' /etc/os-release 2>/dev/null; then
    : # libgeographic-dev handled above
elif ! dpkg -l | grep -q "ros-noetic-geographic"; then
    echo "\033[33m正在安装 ros-noetic-geographic 相关包...\033[0m"
    sudo apt-get install ros-noetic-geographic* -y
fi
