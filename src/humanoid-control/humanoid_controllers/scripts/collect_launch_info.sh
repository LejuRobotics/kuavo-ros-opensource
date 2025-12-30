#!/bin/bash
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
PROJECT_DIR=$(realpath "$SCRIPT_DIR/../../../../") # project: kuavo-ros-control
LAUNCH_INDO_DIR="$HOME/.ros/kuavo_launch/$PPID"
TIMESTAMP=$(date +%Y-%m-%d_%H-%M-%S)

# 检查git命令是否存在
if command -v git >/dev/null 2>&1; then
    git config --global --add safe.directory "$PROJECT_DIR"
    cd "$SCRIPT_DIR"
    GIT_COMMIT=$(git rev-parse HEAD 2>/dev/null || echo "unknown")
    GIT_BRANCH=$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo "unknown")
    GIT_REMOTE=$(git config --get remote.origin.url 2>/dev/null || echo "unknown")
    # 检查GIT_REMOTE是否包含kuavo-ros-control
    if [[ "$GIT_REMOTE" == *"kuavo-ros-control"* ]]; then
        SYNC_COMMIT="$GIT_COMMIT"
    else
        # 查找最近的kuavo_CI同步提交
        git_log=$(git log -n 1 --pretty=format:"%s")
        # 从git_log中提取kuavo_CI同步提交信息
        if [[ "$git_log" =~ sync\ https://www\.lejuhub\.com/highlydynamic/kuavo-ros-control/commit/([a-f0-9]+) ]]; then
            SYNC_COMMIT=$(git rev-parse "${BASH_REMATCH[1]}")
        else
            SYNC_COMMIT="unknown"
        fi
    fi
    cd -
else
    SYNC_COMMIT="unknown"
    GIT_COMMIT="unknown"
    GIT_BRANCH="unknown"
    GIT_REMOTE="unknown"
    echo "Warning: git command not found, using default values for branch and commit"
fi
ROBOT_SERIAL_NUMBER=""
if [ -f "/etc/environment.d/RRNIS.env" ]; then
    ROBOT_SERIAL_NUMBER=$(awk -F= '$1 == "ROBOT_SERIAL_NUMBER" {print $2}' /etc/environment.d/RRNIS.env)
else
    echo "Warning: Robot serial number file not found at /etc/environment.d/RRNIS.env"
fi

mkdir -p "${LAUNCH_INDO_DIR}"
echo "launch_id: $PPID" > "${LAUNCH_INDO_DIR}/info.txt"
echo "date: $TIMESTAMP" >> "${LAUNCH_INDO_DIR}/info.txt"
echo "remote: $GIT_REMOTE" >> "${LAUNCH_INDO_DIR}/info.txt"
echo "branch: $GIT_BRANCH" >> "${LAUNCH_INDO_DIR}/info.txt"
echo "sync_commit: $SYNC_COMMIT" >> "${LAUNCH_INDO_DIR}/info.txt"
echo "crash_commit: $GIT_COMMIT" >> "${LAUNCH_INDO_DIR}/info.txt"
if [ -n "$ROBOT_SERIAL_NUMBER" ]; then
    echo "ROBOT_NAME: $ROBOT_SERIAL_NUMBER" >> "${LAUNCH_INDO_DIR}/info.txt"
fi
echo "coredump_dir: $HOME/.ros/coredumps/$PPID" >> "${LAUNCH_INDO_DIR}/info.txt"

# 检查并清理过期的launch信息目录
LAUNCH_DIR="$HOME/.ros/kuavo_launch"
COREDUMP_DIR="$HOME/.ros/coredumps/"
MAX_DIRS=50

# 清理过期的launch信息目录
# 清理过期的launch信息目录
DIR_COUNT=$(find "$LAUNCH_DIR" -maxdepth 1 -type d -not -path "$LAUNCH_DIR" | wc -l)
if [ "$DIR_COUNT" -gt "$MAX_DIRS" ]; then
    # 计算需要删除的目录数量
    DELETE_COUNT=$((DIR_COUNT - MAX_DIRS))

    # 按修改时间排序（升序），删除最早的目录
    find "$LAUNCH_DIR" -maxdepth 1 -type d -not -path "$LAUNCH_DIR" -exec ls -dtr {} + | head -n "$DELETE_COUNT" | xargs rm -rf
fi

# 清理coredump
CD_DIR_COUNT=$(find "$COREDUMP_DIR" -maxdepth 1 -type d -not -path "$COREDUMP_DIR" | wc -l)
if [ "$CD_DIR_COUNT" -gt "$MAX_DIRS" ]; then
    # 计算需要删除的目录数量
    DELETE_COUNT=$((CD_DIR_COUNT - MAX_DIRS))

    # 按修改时间排序（升序），删除最早的目录
    find "$COREDUMP_DIR" -maxdepth 1 -type d -not -path "$COREDUMP_DIR" -exec ls -dtr {} + | head -n "$DELETE_COUNT" | xargs rm -rf
fi