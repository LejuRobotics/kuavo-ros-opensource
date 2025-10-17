#!/bin/bash
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
PROJECT_DIR=$(realpath "$SCRIPT_DIR/../../../../") # project: kuavo-ros-control
LAUNCH_INDO_DIR="$HOME/.ros/kuavo_launch/$PPID"
TIMESTAMP=$(date +%Y-%m-%d_%H-%M-%S)

# 版本信息文件目录（打包时可生成）
VERSION_DIR="$PROJECT_DIR/.version"

# 默认值
GIT_COMMIT="unknown"
GIT_BRANCH="unknown"
GIT_REMOTE="unknown"
SYNC_COMMIT="unknown"

# 如果有 git 并且有 .git 目录，就用 git
if command -v git >/dev/null 2>&1 && [ -d "$PROJECT_DIR/.git" ]; then
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
        if [[ "$git_log" =~ sync\ https://www\.lejuhub\.com/highlydynamic/kuavo-ros-control/commit/([a-f0-9]+) ]]; then
            SYNC_COMMIT=$(git rev-parse "${BASH_REMATCH[1]}")
        else
            SYNC_COMMIT="unknown"
        fi
    fi
    cd - >/dev/null
# 否则，尝试从 .version 文件夹读取
elif [ -d "$VERSION_DIR" ]; then
    GIT_COMMIT=$(cat "$VERSION_DIR/GIT_COMMIT" 2>/dev/null || echo "unknown")
    GIT_BRANCH=$(cat "$VERSION_DIR/GIT_BRANCH" 2>/dev/null || echo "unknown")
    GIT_REMOTE=$(cat "$VERSION_DIR/GIT_REMOTE" 2>/dev/null || echo "unknown")
    SYNC_COMMIT="$GIT_COMMIT"
    echo "⚠️ Using pre-recorded version info from $VERSION_DIR"
else
    echo "Warning: git not available and no version info found, using default values"
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

# 保持原来的清理逻辑
LAUNCH_DIR="$HOME/.ros/kuavo_launch"
COREDUMP_DIR="$HOME/.ros/coredumps/"
MAX_DIRS=50

DIR_COUNT=$(find "$LAUNCH_DIR" -maxdepth 1 -type d -not -path "$LAUNCH_DIR" | wc -l)
if [ "$DIR_COUNT" -gt "$MAX_DIRS" ]; then
    DELETE_COUNT=$((DIR_COUNT - MAX_DIRS))
    find "$LAUNCH_DIR" -maxdepth 1 -type d -not -path "$LAUNCH_DIR" -exec ls -dtr {} + | head -n "$DELETE_COUNT" | xargs rm -rf
fi

CD_DIR_COUNT=$(find "$COREDUMP_DIR" -maxdepth 1 -type d -not -path "$COREDUMP_DIR" | wc -l)
if [ "$CD_DIR_COUNT" -gt "$MAX_DIRS" ]; then
    DELETE_COUNT=$((CD_DIR_COUNT - MAX_DIRS))
    find "$COREDUMP_DIR" -maxdepth 1 -type d -not -path "$COREDUMP_DIR" -exec ls -dtr {} + | head -n "$DELETE_COUNT" | xargs rm -rf
fi
