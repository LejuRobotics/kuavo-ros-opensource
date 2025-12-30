#!/bin/bash
TIMESTAMP=$(date +%Y-%m-%d_%H-%M-%S)
LOG_DIR="$HOME/.ros/stdout/${TIMESTAMP}"
CORE_DUMP_DIR="$HOME/.ros/coredumps/$PPID"
START_ARG_IDX=1

if [ "$1" = "--with-coredump" ]; then
    START_ARG_IDX=2
    NODE_NAME=$(basename "$2")
    # 检查是否有sudo权限, 有则创建coredump目录
    if [ $(id -u) -eq 0 ]; then
        # 设置core dump相关参数
        mkdir -p "${CORE_DUMP_DIR}"
        sudo sysctl -w kernel.core_pattern="${CORE_DUMP_DIR}/core.%e.%p.%t"
        ulimit -c unlimited
    fi
else
    START_ARG_IDX=1
    NODE_NAME=$(basename "$1")
fi

# @stdoutlog
mkdir -p "${LOG_DIR}"
# 所有节点的日志都输出到同一个文件
LOGFILE="${LOG_DIR}/stdout.log" 
# # 每个节点的日志单独输出到文件
# LOGFILE="${LOG_DIR}/${NODE_NAME}.log" 

# 获取当前脚本的父进程ID和名称，输出到stdout.log的开头
echo "PPID: $PPID, NODE_NAME: ${NODE_NAME}" >> "${LOG_DIR}/stdout.log"

# Ignore INT signal and pass it to child processes
trap '' INT
trap '' TERM
trap '' HUP
trap 'echo "Script interrupted at $(date)" >> ${LOGFILE}' EXIT
exec stdbuf -o0 -e0 "${@:${START_ARG_IDX}}" 2>&1 | stdbuf -i0 -o0 -e0 tee -a ${LOGFILE}