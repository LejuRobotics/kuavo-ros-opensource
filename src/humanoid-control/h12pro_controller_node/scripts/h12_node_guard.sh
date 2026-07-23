#!/bin/bash
# h12 节点门卫（用作手动 launch 路径下、与服务侧同名的 h12 节点的 launch-prefix）。
#
# 本门卫在【手动那套】各 h12 节点启动前做 owner 切换：
#   1. 将 /start_way 置为 manual，通知 ocs2_h12pro_monitor 服务侧让位；
#   2. 等待 monitor 让位完成（/h12_yield_done=1）且服务侧同名 h12 节点释放；
#   3. 释放后 exec 真正的节点命令，让终端拥有自己的 /joy_node 与 FSM 生命周期。
#
# 不再复用服务侧 FSM：复用会导致控制栈由终端启动、按键由服务侧 FSM 解释，
# 服务侧 initial FSM 下按 C 会重复拉起控制栈，顶掉当前控制栈。
#
# 与旧逻辑（服务 active + /joy_node 在线就 exit 0 复用服务节点）相比，本门卫
# 从“复用门卫”改为“owner 切换门卫”。
#
# 仅用于手动路径；服务自己起节点时 guard_prefix 为空、不经过本门卫，对服务侧零影响。
#
# 用法：launch-prefix="<本脚本绝对路径>"
#       roslaunch 会把真正的节点命令作为 "$@" 传入。

WAIT_TIMEOUT_SEC=${H12_NODE_GUARD_TIMEOUT_SEC:-20}
CHECK_INTERVAL_SEC=${H12_NODE_GUARD_INTERVAL_SEC:-0.5}
REQUIRED_ABSENT_TICKS=${H12_NODE_GUARD_ABSENT_TICKS:-2}
H12_NODES="/h12pro_channel_publisher /joy_node /websocket_sdk_start_node"

# 从 roslaunch 透传的参数里解析 __name:=xxx，得到本次要启动的 ROS 节点名。
target_node=""
for arg in "$@"; do
    case "$arg" in
        __name:=*)
            target_node="/${arg#__name:=}"
            ;;
    esac
done

ros_node_exists() {
    rosnode list 2>/dev/null | grep -qx "$1"
}

any_h12_node_exists() {
    local node
    for node in $H12_NODES; do
        if ros_node_exists "$node"; then
            return 0
        fi
    done
    return 1
}

# 本次要启动的同名节点是否已释放（解析不到 __name 时退化为任一 h12 节点都不在）。
node_released() {
    if [ -n "$target_node" ]; then
        ! ros_node_exists "$target_node"
    else
        ! any_h12_node_exists
    fi
}

# 服务让位完成标志，由 monitor stop_tree() 末尾置 1。
service_yielded() {
    [ "$(rosparam get /h12_yield_done 2>/dev/null)" = "1" ]
}

rosparam set /start_way manual 2>/dev/null || true

echo "[h12_node_guard] manual owner requested for ${target_node:-unknown node}: $*"

# 冷启动：服务那套整体不在线，无需等待让位，直接放行。
if ! any_h12_node_exists; then
    echo "[h12_node_guard] no service-side h12 nodes online; starting manual node directly: $*"
    exec "$@"
fi

# 服务那套在线：等 monitor 明确让位（/h12_yield_done=1）且本次同名节点已释放再 exec。
# 用 param 握手替代 sleep 猜清理窗口，消除 stop_tree 末尾 rosnode kill/cleanup
# 误伤终端新节点的残留竞态。三个 guard 并行，各自只等自己的同名节点释放。
echo "[h12_node_guard] service-side h12 nodes online; wait for monitor yield (/h12_yield_done=1) and ${target_node:-h12 nodes} release"
absent_ticks=0
start_time=$(date +%s)
while [ "$absent_ticks" -lt "$REQUIRED_ABSENT_TICKS" ]; do
    now=$(date +%s)
    if [ $((now - start_time)) -ge "$WAIT_TIMEOUT_SEC" ]; then
        echo "[h12_node_guard] timeout waiting for monitor yield / ${target_node:-h12 nodes} release; abort: $*" >&2
        exit 1
    fi

    if service_yielded && node_released; then
        absent_ticks=$((absent_ticks + 1))
    else
        echo "[h12_node_guard] waiting for monitor yield (/h12_yield_done=1) and ${target_node:-h12 nodes} release"
        absent_ticks=0
    fi
    sleep "$CHECK_INTERVAL_SEC"
done

echo "[h12_node_guard] monitor yielded and same-name node released; starting manual node: $*"
exec "$@"
