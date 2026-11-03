#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""H12PRO joy_node 常驻监控主循环（由 monitor_ocs2_h12pro.sh 薄壳入口启动）。

职责:
  1. 关键节点异常退出/被踢/hang 时，重启整棵 launch 树
  2. 检测到外部手动路径（如 bt2 launch，start_way=manual）后主动让位

热路径全部在本进程内完成，不 fork 任何子进程（旧 bash 版每秒 fork 4 个
Python CLI 子进程，实测 ~0.45s CPU/s，RK3588 上 ~1.5 核）:
  - rosparam get /start_way      -> rosgraph.Master.getParam()      (~ms)
  - rosnode ping                 -> lookupNode + XMLRPC getPid      (~ms)
  - pgrep -f ocs2_h12pro_node.py -> 扫描 /proc/*/cmdline            (~ms)
冷路径（start_tree/stop_tree）保持 shell 调用（roslaunch / rosnode kill /
cleanup），只在崩溃/让位时发生，与 bash 版行为一致。

逻辑分支与 bash 版 1:1 对应（kill -0 / start_way 让位 / 崩溃重启 / hang 兜底），
修正（bash 版同源的竞态，常驻版 1s tick 使其必然复现）:
  - hang 兜底要求"曾 ping 通过 + 连续 N 次失败"才判 hang（PING_FAIL_RESTART_THRESHOLD），
    未过启动宽限期（STARTUP_PING_GRACE_TICKS）的 ping 失败视为启动延迟，
    避免 python 节点导入期或高负载单次 XMLRPC 超时被误杀重启
  - "进程消失"重启要求进程连续存活过 N tick（JOY_PROC_GONE_ARM_TICKS），
    上一棵树 stop 残留进程退出不误杀新树
  - start_tree 立即把 /start_way 置回 auto，避免回收路径下下一 tick 读到残留 manual
    把刚拉起的树当场让位杀掉（15s reclaim -> 1s yield 死循环）

前置要求: h12pro_autostart.launch 中 joy_node 必须关闭 respawn，
          让本脚本作为唯一的重启来源，避免同名竞争。
"""
import os
import signal
import socket
import subprocess
import sys
import time
import xmlrpc.client

import rosgraph

POLL_INTERVAL = 1          # 主循环间隔
HANG_CHECK_EVERY = 1       # 每 N 个 tick 做一次 XMLRPC ping 兜底（进程内调用 ~ms 级，无需降频）
PING_TIMEOUT = 1.0         # 单次 ping 的 socket 超时，等价 bash 版 timeout 1 rosnode ping -c 1
MANUAL_IDLE_THRESHOLD = 15 # start_way=manual 且无任何 /joy_node 持续 N tick 后接管
                           # 容忍 bt2 启动过程中"param 已设但 joy_node 未注册"的窗口
STARTUP_PING_GRACE_TICKS = 30  # 树启动后，节点从未 ping 通过超过 N tick 才按启动失败重启
                               # joy_node python 导入+注册需数秒，且启动脚本本身要跑数秒，
                               # 未过宽限期的 ping 失败一律视为启动延迟，不再误判 hang
PING_FAIL_RESTART_THRESHOLD = 3  # 曾 ping 通过的节点连续失败 N tick 才判 hang；
                                 # 高负载（cartographer 等）下单次 1s XMLRPC 超时很常见，需去抖
JOY_PROC_GONE_ARM_TICKS = 3  # joy_node 进程连续存活 N tick 后，"进程消失"才按崩溃重启；
                             # 存活不足 N tick 就消失的，多半是上一棵树 stop 残留进程退出，
                             # 用它误杀刚拉起的树会产生新的残留，形成连锁
JOY_PROC_PATTERN = 'ocs2_h12pro_node.py'
PING_NODES = ('/joy_node', '/h12pro_channel_publisher')

NODE_SCRIPT = os.environ['NODE_SCRIPT']

master = rosgraph.Master('/h12pro_monitor')

node_pid = None           # launch 树进程 pid（等价 bash 的 NODE_PID）
tick = 0
manual_idle_ticks = 0
joy_proc_seen = False     # 标记 joy_node 进程是否曾出现过，用于区分启动延迟和运行中崩溃
joy_proc_alive_ticks = 0  # joy_node 进程连续存活 tick 数，达到 JOY_PROC_GONE_ARM_TICKS 才启用"进程消失"判定
ping_ok_seen = {n: False for n in PING_NODES}      # 各节点是否曾 ping 通过，用于区分启动延迟和运行中 hang
ping_fail_streak = {n: 0 for n in PING_NODES}      # 各节点连续 ping 失败 tick 数，达到阈值才判 hang
tree_start_tick = 0       # 最近一次 start_tree 时的 tick，用于启动宽限计时


def log(msg):
    print('[%s] %s' % (time.strftime('%F %T'), msg), flush=True)


def start_tree():
    """等价 bash 版：source ~/.bashrc; "$NODE_SCRIPT" &"""
    global node_pid, joy_proc_seen, joy_proc_alive_ticks, ping_ok_seen, ping_fail_streak, tree_start_tick
    # 不用 setsid：roslaunch 隐式拉起的 rosmaster 因此不在本脚本的进程组里，
    # stop 阶段才不会连带把 master 一起杀掉（外部 joy_node 如 bt2 会被殃及）。
    proc = subprocess.Popen(
        ['bash', '-c', 'source "$HOME/.bashrc"; exec "$1"', 'bash', NODE_SCRIPT])
    node_pid = proc.pid
    joy_proc_seen = False
    joy_proc_alive_ticks = 0
    ping_ok_seen = dict.fromkeys(PING_NODES, False)
    ping_fail_streak = dict.fromkeys(PING_NODES, 0)
    tree_start_tick = tick
    # 服务重新持有 owner，尚未让位：清掉让位完成标志，下一次终端接管需等一次新的让位。
    # 同时立即把 /start_way 置回 auto：manual->auto 的修正原本由 start 脚本/launch 完成，
    # 但耗时数秒；不在这里置回，下一个 tick 仍读到 manual，会把刚拉起的树当场让位杀掉
    # （回收路径的 15s reclaim -> 1s yield 死循环）。
    try:
        master.setParam('/h12_yield_done', 0)
        master.setParam('/start_way', 'auto')
    except Exception:
        pass
    log('started pid=%d' % node_pid)


def stop_tree():
    """等价 bash 版：kill -9 launch 树 -> rosnode kill 业务节点 -> cleanup -> 置让位标志"""
    global node_pid
    if node_pid is None:
        return
    # 只对 roslaunch 本 pid 发信号；它的子进程（含隐式 rosmaster）会被 init 接管继续存活。
    # 业务节点用 rosnode kill 点名处理，避免 pgid kill 的副作用。
    try:
        os.kill(node_pid, signal.SIGKILL)
    except OSError:
        pass
    node_pid = None
    time.sleep(1)
    for ros_node in ('/h12pro_channel_publisher', '/joy_node', '/websocket_sdk_start_node'):
        subprocess.run(['rosnode', 'kill', ros_node],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(['timeout', '3', 'rosnode', 'cleanup'],
                   input=b'y\n' * 500, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    # 让位整体完成（同名节点已 kill+cleanup）：置位让位完成标志，
    # h12_node_guard.sh 等到这个明确信号才 exec 终端节点，避免末尾 kill/cleanup 误伤终端新节点。
    try:
        master.setParam('/h12_yield_done', 1)
    except Exception:
        pass
    log('stopped')


def joy_proc_alive():
    """等价 pgrep -f ocs2_h12pro_node.py：扫描 /proc/*/cmdline 是否包含模式串"""
    try:
        pids = os.listdir('/proc')
    except OSError:
        return False
    for pid in pids:
        if not pid.isdigit():
            continue
        try:
            with open('/proc/%s/cmdline' % pid, 'rb') as f:
                cmdline = f.read().replace(b'\0', b' ').decode('utf-8', 'ignore')
        except (OSError, IOError):
            continue
        if JOY_PROC_PATTERN in cmdline:
            return True
    return False


def node_pingable(node_name):
    """等价 timeout 1 rosnode ping -c 1 <node>：查 master 拿节点 XMLRPC URI，再调 getPid 探活"""
    try:
        uri = master.lookupNode(node_name)
    except Exception:
        return False
    if not uri:
        return False
    old_timeout = socket.getdefaulttimeout()
    try:
        socket.setdefaulttimeout(PING_TIMEOUT)
        xmlrpc.client.ServerProxy(uri).getPid('/h12pro_monitor')
        return True
    except Exception:
        return False
    finally:
        socket.setdefaulttimeout(old_timeout)


def on_term(signum, frame):
    """等价 bash 的 trap 'stop_tree; exit 0' SIGTERM SIGINT SIGHUP"""
    stop_tree()
    sys.exit(0)


signal.signal(signal.SIGTERM, on_term)
signal.signal(signal.SIGINT, on_term)
signal.signal(signal.SIGHUP, on_term)

while True:
    tick += 1

    # launch 树还在吗
    if node_pid is not None:
        try:
            os.kill(node_pid, 0)
        except ProcessLookupError:
            node_pid = None
        except OSError:
            pass

    try:
        start_way = master.getParam('/start_way')
    except Exception:
        start_way = None

    if node_pid is None:
        if start_way != 'manual':
            # 用户未显式要 manual，直接起
            manual_idle_ticks = 0
            start_tree()
        elif node_pingable('/joy_node'):
            # start_way=manual 且外部 /joy_node 在工作（如 bt2），继续让位
            # （bash 版用 rosnode list | grep，这里等价于 lookupNode 能查到）
            manual_idle_ticks = 0
        else:
            # start_way=manual 但没有任何 /joy_node 在工作 —— 可能 bt2 已退出
            # 但没清理 rosparam。去抖确认不是 bt2 的启动空窗后，自行接管
            # （start_tree 会通过 launch param 把 start_way 重置回 auto）
            manual_idle_ticks += 1
            if manual_idle_ticks >= MANUAL_IDLE_THRESHOLD:
                log('manual joy_node absent for %ds, reclaiming'
                    % (MANUAL_IDLE_THRESHOLD * POLL_INTERVAL))
                manual_idle_ticks = 0
                start_tree()
    elif start_way == 'manual':
        # 手动路径（如 bt2）接管，让位
        manual_idle_ticks = 0
        log('start_way=manual, yielding')
        stop_tree()
    elif not joy_proc_alive():
        # joy_node 进程不存在
        if joy_proc_seen and joy_proc_alive_ticks >= JOY_PROC_GONE_ARM_TICKS:
            # 进程连续存活过 N tick 后消失（SIGSEGV 等），需要重启
            manual_idle_ticks = 0
            log('joy_node process gone, restarting')
            stop_tree()
        elif tick - tree_start_tick >= STARTUP_PING_GRACE_TICKS:
            # 树拉起后 joy_node 进程始终没出现（启动脚本卡死/被拒等），按启动失败重启
            log('joy_node process never appeared within %ds, restarting'
                % (STARTUP_PING_GRACE_TICKS * POLL_INTERVAL))
            stop_tree()
        # 否则：启动期间 joy_node 进程还没创建，或只是上一棵树 stop 残留进程退出，不做处理
        joy_proc_alive_ticks = 0
    else:
        # 进程在：每 tick 标记"已出现"，保持 1s 级崩溃检测精度
        joy_proc_seen = True
        joy_proc_alive_ticks += 1
        if tick % HANG_CHECK_EVERY == 0:
            # 周期性兜底：进程在但关键节点不响应 XMLRPC（hang/stale）。
            # 节点启动（python 导入+注册）需要数秒，"从未 ping 通过"不算 hang：
            # 只有此前 ping 通过、且连续 PING_FAIL_RESTART_THRESHOLD 次失联，才按 hang 重启
            # （高负载下单次 1s XMLRPC 超时很常见，需去抖）；
            # 超过启动宽限时间仍从未 ping 通过，才按启动失败重启。
            for ros_node in PING_NODES:
                if node_pingable(ros_node):
                    ping_ok_seen[ros_node] = True
                    ping_fail_streak[ros_node] = 0
                    continue
                ping_fail_streak[ros_node] += 1
                if ping_ok_seen[ros_node] and ping_fail_streak[ros_node] >= PING_FAIL_RESTART_THRESHOLD:
                    log('%s unresponsive (possible hang/stale), restarting' % ros_node)
                    stop_tree()
                    break
                if tick - tree_start_tick >= STARTUP_PING_GRACE_TICKS:
                    log('%s never became pingable within %ds, restarting'
                        % (ros_node, STARTUP_PING_GRACE_TICKS * POLL_INTERVAL))
                    stop_tree()
                    break

    time.sleep(POLL_INTERVAL)
