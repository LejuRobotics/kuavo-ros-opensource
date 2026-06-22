#!/bin/bash
# H12PRO joy_node 监控脚本
#
# 职责:
#   1. 关键节点异常退出/被踢/hang 时，重启整棵 launch 树
#   2. 检测到外部手动路径（如 bt2 launch，start_way=manual）后主动让位
#
# 探活策略（由便宜到贵，按需触发）:
#   - kill -0 NODE_PID:     μs 级，判断 launch 树是否还活
#   - pgrep -f ...py:       ms 级，判断 joy_node Python 进程是否还活（覆盖 SIGSEGV zombie）
#   - rosnode ping:         1s 级周期触发，兜底 hang/deadlock/stale registration
#
# 前置要求: h12pro_autostart.launch 中 joy_node 必须关闭 respawn，
#          让本脚本作为唯一的重启来源，避免同名竞争。

export ROS_WS_PATH=/opt/ros/noetic
source $ROS_WS_PATH/setup.bash

[ -z "$NODE_SCRIPT" ] && { echo "Error: NODE_SCRIPT not set"; exit 1; }
[ ! -f "$NODE_SCRIPT" ] && { echo "Error: $NODE_SCRIPT not found"; exit 1; }

POLL_INTERVAL=1         # 主循环间隔
HANG_CHECK_EVERY=1      # 每 N 个 tick 做一次 rosnode ping 兜底（约 1s 一次）
ROS_PING_TIMEOUT=1      # rosnode ping 超时时间
MANUAL_IDLE_THRESHOLD=15 # start_way=manual 且无任何 /joy_node 持续 N tick 后接管
                        # 容忍 bt2 启动过程中"param 已设但 joy_node 未注册"的窗口
JOY_PROC_PATTERN="ocs2_h12pro_node.py"

NODE_PID=""
TICK=0
MANUAL_IDLE_TICKS=0
JOY_PROC_SEEN=false  # 标记 joy_node 进程是否曾出现过，用于区分启动延迟和运行中崩溃

log() { echo "[$(date +%F\ %T)] $*"; }

start_tree() {
    source "$HOME/.bashrc"
    # 不用 setsid：roslaunch 隐式拉起的 rosmaster 因此不在本脚本的进程组里，
    # stop 阶段才不会连带把 master 一起杀掉（外部 joy_node 如 bt2 会被殃及）。
    "$NODE_SCRIPT" &
    NODE_PID=$!
    JOY_PROC_SEEN=false
    log "started pid=$NODE_PID"
}

stop_tree() {
    [ -z "$NODE_PID" ] && return
    # 只对 roslaunch 本 pid 发信号；它的子进程（含隐式 rosmaster）会被 init 接管继续存活。
    # 业务节点用 rosnode kill 点名处理，避免 pgid kill 的副作用。
    kill -9 "$NODE_PID" 2>/dev/null || true
    sleep 1
    rosnode kill /h12pro_channel_publisher 2>/dev/null || true
    rosnode kill /joy_node 2>/dev/null || true
    rosnode kill /websocket_sdk_start_node 2>/dev/null || true
    yes | timeout 3 rosnode cleanup >/dev/null 2>&1 || true
    NODE_PID=""
    log "stopped"
}

trap 'stop_tree; exit 0' SIGTERM SIGINT SIGHUP

while true; do
    TICK=$((TICK + 1))

    # launch 树还在吗
    if [ -n "$NODE_PID" ] && ! kill -0 "$NODE_PID" 2>/dev/null; then
        NODE_PID=""
    fi

    START_WAY=$(rosparam get /start_way 2>/dev/null)

    if [ -z "$NODE_PID" ]; then
        if [ "$START_WAY" != "manual" ]; then
            # 用户未显式要 manual，直接起
            MANUAL_IDLE_TICKS=0
            start_tree
        elif rosnode list 2>/dev/null | grep -qx "/joy_node"; then
            # start_way=manual 且外部 /joy_node 在工作（如 bt2），继续让位
            MANUAL_IDLE_TICKS=0
        else
            # start_way=manual 但没有任何 /joy_node 在工作 —— 可能 bt2 已退出
            # 但没清理 rosparam。去抖确认不是 bt2 的启动空窗后，自行接管
            # （start_tree 会通过 launch param 把 start_way 重置回 auto）
            MANUAL_IDLE_TICKS=$((MANUAL_IDLE_TICKS + 1))
            if [ "$MANUAL_IDLE_TICKS" -ge "$MANUAL_IDLE_THRESHOLD" ]; then
                log "manual joy_node absent for $((MANUAL_IDLE_THRESHOLD * POLL_INTERVAL))s, reclaiming"
                MANUAL_IDLE_TICKS=0
                start_tree
            fi
        fi
    elif [ "$START_WAY" = "manual" ]; then
        # 手动路径（如 bt2）接管，让位
        MANUAL_IDLE_TICKS=0
        log "start_way=manual, yielding"
        stop_tree
    elif ! pgrep -f "$JOY_PROC_PATTERN" >/dev/null 2>&1; then
        # joy_node 进程不存在
        if $JOY_PROC_SEEN; then
            # 运行中 joy_node 进程消失（SIGSEGV 等），需要重启
            MANUAL_IDLE_TICKS=0
            log "joy_node process gone, restarting"
            stop_tree
        fi
        # 否则：启动期间 joy_node 进程还没创建，属于正常现象，不做处理
    elif [ $((TICK % HANG_CHECK_EVERY)) -eq 0 ]; then
        # 周期性兜底：进程在但关键节点不响应 XMLRPC（hang/stale）
        # 同时标记 joy_node 进程已出现（pgrep 通过），后续消失才算异常
        pgrep -f "$JOY_PROC_PATTERN" >/dev/null 2>&1 && JOY_PROC_SEEN=true
        for ROS_NODE in /joy_node /h12pro_channel_publisher; do
            if ! timeout "$ROS_PING_TIMEOUT" rosnode ping -c 1 "$ROS_NODE" >/dev/null 2>&1; then
                log "$ROS_NODE unresponsive (possible hang/stale), restarting"
                stop_tree
                break
            fi
        done
    fi

    sleep "$POLL_INTERVAL"
done
