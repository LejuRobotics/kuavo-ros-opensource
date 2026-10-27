#!/bin/bash
# H12PRO joy_node 监控入口（常驻 Python 版）
#
# 职责:
#   1. 关键节点异常退出/被踢/hang 时，重启整棵 launch 树
#   2. 检测到外部手动路径（如 bt2 launch，start_way=manual）后主动让位
#
# 本脚本只做 ROS 环境准备与参数校验，监控主循环在 monitor_ocs2_h12pro.py 常驻进程中执行。
# 旧 bash 版每秒 fork 4 个 Python CLI 子进程（rosparam/rosnode×2/pgrep），实测 ~0.45s CPU/s
# （RK3588 上 ~1.5 核）；常驻版热路径全部进程内完成（rosgraph.getParam / XMLRPC getPid /
# /proc 扫描），开销 ~0.01s CPU/s，且 hang 检测保持 1s 级。
#
# 前置要求: h12pro_autostart.launch 中 joy_node 必须关闭 respawn，
#          让本脚本作为唯一的重启来源，避免同名竞争。

export ROS_WS_PATH=/opt/ros/noetic
source $ROS_WS_PATH/setup.bash

[ -z "$NODE_SCRIPT" ] && { echo "Error: NODE_SCRIPT not set"; exit 1; }
[ ! -f "$NODE_SCRIPT" ] && { echo "Error: $NODE_SCRIPT not found"; exit 1; }

SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
exec python3 "$SCRIPT_DIR/monitor_ocs2_h12pro.py"
