#!/bin/bash
# @stdoutlog
# FIX: 避免跨秒生成多个日志目录
# 同一次 roslaunch 的节点共享 PPID。目录名由第一个节点写入，其余节点读取同一份。
# 键使用 /proc/<ppid>/stat 的 starttime（开机后的时钟滴答），不解析 ps -o lstart 文本，
# 避免 date -d 失败后各自退回墙钟、启动跨秒时拆成两个目录。
# 时间戳目录格式保持 YYYY-MM-DD_HH-MM-SS，不影响 CrashReport.sh。
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

# /proc/pid/stat 在最后一个 ')' 之后，第 20 个字段是 starttime（时钟滴答）。
# comm 可能含空格和括号，所以先丢掉 "pid (comm)"。
ppid_start_ticks() {
    local stat_file="$1"
    [ -r "${stat_file}" ] || return 1
    sed 's/.*) //' "${stat_file}" 2>/dev/null | awk '{print $20}'
}

START_TICKS=$(ppid_start_ticks "/proc/${PPID}/stat")
case "${START_TICKS}" in
    ''|*[!0-9]*) START_TICKS="" ;;
esac

# 戳放在 stdout 外面，避免被当成一次运行的日志目录。
STAMP_ROOT="${HOME}/.ros/stdout_launch_stamp"
TIMESTAMP=""
if [ -n "${START_TICKS}" ]; then
    STAMP_DIR="${STAMP_ROOT}/${PPID}_${START_TICKS}"
    mkdir -p "${STAMP_ROOT}"
    if mkdir "${STAMP_DIR}" 2>/dev/null; then
        date +%Y-%m-%d_%H-%M-%S > "${STAMP_DIR}/stamp"
    else
        # 同一次启动里别的节点已经抢到目录，等它把时间戳写完。
        for _ in 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39 40; do
            [ -s "${STAMP_DIR}/stamp" ] && break
            sleep 0.05
        done
    fi
    if [ -s "${STAMP_DIR}/stamp" ]; then
        TIMESTAMP=$(cat "${STAMP_DIR}/stamp")
    fi

    # 清掉已经退出的父进程留下的戳，避免 PID 复用后读到旧时间。
    for old_dir in "${STAMP_ROOT}"/*; do
        [ -d "${old_dir}" ] || continue
        old_name=$(basename "${old_dir}")
        [ "${old_name}" = "${PPID}_${START_TICKS}" ] && continue
        old_pid=${old_name%%_*}
        old_ticks=${old_name#*_}
        if [ ! -d "/proc/${old_pid}" ]; then
            rm -rf "${old_dir}"
            continue
        fi
        live_ticks=$(ppid_start_ticks "/proc/${old_pid}/stat")
        if [ "${live_ticks}" != "${old_ticks}" ]; then
            rm -rf "${old_dir}"
        fi
    done
fi

# 读不到父进程启动滴答，或共享戳没写出来时，才退回当前时间。
if [ -z "${TIMESTAMP}" ]; then
    echo "[start_node.sh][WARN] 共享日志目录时间戳失败，PPID=${PPID} START_TICKS='${START_TICKS}'，回退使用当前时间。NODE_NAME=${NODE_NAME}" >&2
    TIMESTAMP=$(date +%Y-%m-%d_%H-%M-%S)
fi

LOG_DIR="${STDROOT_DIR}/${TIMESTAMP}"
mkdir -p "${LOG_DIR}"

# 所有节点的日志都输出到同一个文件
LOGFILE="${LOG_DIR}/stdout.log"

# 下面把节点输出接进管道。管道不是终端，Python 会把 stdout 攒成块再写，
# rospy 的 INFO 走 stdout，所以运行中终端看不到，进程退出才一次性刷出。
# stdbuf 只作用于 C 的 stdio，盖不住这层缓冲。
export PYTHONUNBUFFERED=1

# 获取当前脚本的父进程ID和名称，输出到stdout.log的开头
echo "PPID: $PPID, NODE_NAME: ${NODE_NAME}" >> "${LOG_DIR}/stdout.log"

# Ignore INT signal and pass it to child processes
trap '' INT
trap '' TERM
trap '' HUP
trap 'echo "Script interrupted at $(date)" >> ${LOGFILE}' EXIT
exec stdbuf -o0 -e0 "${@:${START_ARG_IDX}}" 2>&1 | stdbuf -i0 -o0 -e0 tee -a ${LOGFILE}