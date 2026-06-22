#!/bin/bash
# h12 节点门卫（用作手动 launch 路径下、与服务侧同名的 h12 节点的 launch-prefix）。
#
# 背景：服务 ocs2_h12pro_monitor 已常驻运行一套 h12 控制节点。命令行手动
# `roslaunch load_kuavo_real.launch joystick_type:=h12` 会再起一套同名节点
# （joy_node / h12pro_channel_publisher / websocket_sdk_start_node），与服务那套
# 争抢同名节点：joy_node 反复被顶替且无 respawn 不自愈 -> 手柄失灵；
# websocket 远程控制错乱等。
#
# 本门卫在【手动那套】各 h12 节点启动的瞬间做一次自检：
#   - 服务正常（active 且 /joy_node 已在线，代表服务那套整体已就绪）
#       => 直接退出，不启动本节点，复用服务那套；
#   - 服务未启动/未就绪
#       => exec 真正的节点命令，按原流程自起。
# 判据统一用 /joy_node 是否在线作为“服务那套是否就绪”的代表信号。
#
# 仅用于手动路径；服务自己起节点时 guard_prefix 为空、不经过本门卫，故对服务侧零影响。
#
# 用法：launch-prefix="<本脚本绝对路径>"
#       roslaunch 会把真正的节点命令作为 "$@" 传入。

if systemctl is-active --quiet ocs2_h12pro_monitor.service 2>/dev/null \
   && rosnode list 2>/dev/null | grep -qx "/joy_node"; then
    echo "[h12_node_guard] ocs2_h12pro_monitor service active and /joy_node online; " \
         "skip starting manual node, reuse service-side node: $*"
    exit 0
fi

echo "[h12_node_guard] service not ready; starting manual node: $*"
exec "$@"
