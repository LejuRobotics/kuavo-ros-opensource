#!/bin/bash
# @stdoutlog
# FIX: 避免跨秒生成多个日志目录
# 原理：使用 PPID 进程（roslaunch）的启动时间作为日志目录名。
#       同一次 roslaunch 启动的所有节点共享同一个 PPID，其启动时间是固定不变的，
#       因此每个节点独立计算得到的目录名一致，配合 mkdir -p 天然复用同一目录，
#       不依赖锁文件，避免锁文件残留、跨秒竞态等隐患。
# 兼容原有时间戳目录格式，不影响 CrashReport.sh 等现有工具。
CORE_DUMP_DIR="$HOME/.ros/coredumps/$PPID"
START_ARG_IDX=1

if [ "$1" = "--with-coredump" ]; then
    START_ARG_IDX=2
    NODE_NAME=$(basename "$2")
    # 检查是否有sudo权限, 有则创建coredump目录
    if [ $(id -u) -eq 0 ]; then
        # 设置core dump相关参数
        mkdir -p "${CORE_DUMP_DIR}"
        if [ -d "${CORE_DUMP_DIR}" ]; then
            echo "coredump for kuavo:${NODE_NAME}" >> "${CORE_DUMP_DIR}/README.txt"
        fi
        sudo sysctl -w kernel.core_pattern="${CORE_DUMP_DIR}/core.%e.%p.%t"
        ulimit -c unlimited
    fi
else
    START_ARG_IDX=1
    NODE_NAME=$(basename "$1")
fi

STDROOT_DIR="$HOME/.ros/stdout"
mkdir -p "${STDROOT_DIR}"

# 使用 PPID 进程的启动时间作为日志目录名，保证同一次启动的所有节点目录一致
# ps -o lstart= 输出形如 "Mon Jun 15 17:07:34 2026"，date -d 解析后格式化为标准时间戳
PPID_LSTART=$(ps -o lstart= -p "${PPID}" 2>/dev/null)
if [ -n "${PPID_LSTART}" ]; then
    TIMESTAMP=$(date -d "${PPID_LSTART}" +%Y-%m-%d_%H-%M-%S 2>/dev/null)
fi
# 兜底：若获取 PPID 启动时间失败，则退化为当前时间（理论上不会触发）
# 打印告警信息便于事后排查（输出到 stderr，避免污染节点 stdout）
if [ -z "${TIMESTAMP}" ]; then
    echo "[start_node.sh][WARN] 获取 PPID(${PPID}) 启动时间失败，PPID_LSTART='${PPID_LSTART}'，回退使用当前时间作为日志目录名。NODE_NAME=${NODE_NAME}" >&2
    TIMESTAMP=$(date +%Y-%m-%d_%H-%M-%S)
fi

LOG_DIR="${STDROOT_DIR}/${TIMESTAMP}"
mkdir -p "${LOG_DIR}"

# 所有节点的日志都输出到同一个文件
LOGFILE="${LOG_DIR}/stdout.log"

# 获取当前脚本的父进程ID和名称，输出到stdout.log的开头
echo "PPID: $PPID, NODE_NAME: ${NODE_NAME}" >> "${LOG_DIR}/stdout.log"

# Ignore INT signal and pass it to child processes
trap '' INT
trap '' TERM
trap '' HUP
trap 'echo "Script interrupted at $(date)" >> ${LOGFILE}' EXIT
exec stdbuf -o0 -e0 "${@:${START_ARG_IDX}}" 2>&1 | stdbuf -i0 -o0 -e0 tee -a ${LOGFILE}