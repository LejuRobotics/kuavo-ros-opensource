#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_WS_PATH="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
ROS_DEVEL_PATH="$ROS_WS_PATH/devel"
ROS_SETUP="$ROS_DEVEL_PATH/setup.bash"
SESSION="depth_locomotion_controller"
MODE="full"
ATTACH=1
MINIMAL_POST=false
USE_GAUSSIAN="${USE_GAUSSIAN:-false}"

usage() {
    echo "用法: $0 [--camera-only] [--minimal-post] [--gaussian|--no-gaussian] [--no-attach]"
    echo "  默认启动增强相机、时间采样后处理和机器人控制器。"
    echo "  --camera-only 只启动相机链路，安全测试时使用。"
    echo "  --minimal-post 跳过 Raw 专用白点/膨胀处理，仅在无效像素过多时修补。"
    echo "  --gaussian 开启高斯模糊。"
    echo "  --no-gaussian 关闭高斯模糊（SDK 默认）。"
}

while (($#)); do
    case "$1" in
        --camera-only) MODE="camera-only" ;;
        --minimal-post) MINIMAL_POST=true ;;
        --gaussian) USE_GAUSSIAN=true ;;
        --no-gaussian) USE_GAUSSIAN=false ;;
        --no-attach) ATTACH=0 ;;
        -h|--help) usage; exit 0 ;;
        *) echo "未知参数: $1" >&2; usage >&2; exit 2 ;;
    esac
    shift
done

command -v tmux >/dev/null || { echo "缺少 tmux" >&2; exit 3; }
[[ -f "$ROS_SETUP" ]] || {
    echo "最小工作区尚未编译：$ROS_SETUP 不存在" >&2
    exit 4
}
[[ -f "$SCRIPT_DIR/ssh-lingbot-depth-camera.py" ]] || {
    echo "缺少 $SCRIPT_DIR/ssh-lingbot-depth-camera.py" >&2
    exit 5
}
[[ -f "$SCRIPT_DIR/../../../image_processing/scripts/depth_history_buffer.py" ]] || {
    echo "缺少 compact 深度历史节点" >&2
    exit 5
}

# This robot is kuavo_v55. Refuse a full hardware launch if the shell selects
# another robot configuration; camera-only mode remains available for testing.
if [[ "$MODE" == "full" && "${ROBOT_VERSION:-}" != "55" ]]; then
    echo "拒绝启动控制器：本机要求 ROBOT_VERSION=55，当前为 ${ROBOT_VERSION:-未设置}。" >&2
    echo "请设置正确的机器人版本；可用 --camera-only 验证相机链路。" >&2
    exit 6
fi
if [[ "$MODE" == "full" && ! -e "$ROS_DEVEL_PATH/lib/libhumanoid_controllers.so" ]]; then
    echo "拒绝启动控制器：lza 分支的 libhumanoid_controllers.so 尚未构建成功。" >&2
    echo "请先完成 minimal profile 的 humanoid_controllers 构建。" >&2
    exit 7
fi

tmux kill-session -t "$SESSION" 2>/dev/null || true
tmux new-session -d -s "$SESSION" -n main
tmux set-option -t "$SESSION" remain-on-exit on
tmux split-window -h -t "$SESSION:0"
tmux split-window -h -t "$SESSION:0"
tmux select-pane -t "$SESSION:0.0"
tmux split-window -v -f -t "$SESSION:0.0"

LOWER_ROS_ENV="export ROS_MASTER_URI=http://192.168.26.1:11311; export ROS_IP=192.168.26.1"
CORE_CMD="$LOWER_ROS_ENV; source '$ROS_SETUP'; if rosnode list >/dev/null 2>&1; then echo '复用已有 ROS master'; exec bash; else exec roscore; fi"
POST_CMD="$LOWER_ROS_ENV; source '$ROS_SETUP'; until rostopic list >/dev/null 2>&1; do sleep 0.2; done; exec rosrun image_processing depth_history_buffer.py _input_topic:=/waist_camera/depth/image_enhanced_compact _output_width:=64 _output_height:=36 _history_length:=8 _history_interval_sec:=0.1"
CAMERA_ARGS=""
if [[ "$MINIMAL_POST" == true ]]; then
    CAMERA_ARGS="--minimal-post"
fi
if [[ "$USE_GAUSSIAN" == true ]]; then
    CAMERA_ARGS="$CAMERA_ARGS --gaussian"
else
    CAMERA_ARGS="$CAMERA_ARGS --no-gaussian"
fi
CAMERA_CMD="cd '$SCRIPT_DIR'; source /opt/ros/noetic/setup.bash; exec python3 '$SCRIPT_DIR/ssh-lingbot-depth-camera.py' $CAMERA_ARGS"

tmux send-keys -t "$SESSION:0.0" "$CORE_CMD" C-m
tmux send-keys -t "$SESSION:0.1" "$POST_CMD" C-m
tmux send-keys -t "$SESSION:0.2" "$CAMERA_CMD" C-m

if [[ "$MODE" == "full" ]]; then
    CONTROLLER_CMD="$LOWER_ROS_ENV; source '$ROS_SETUP'; echo '等待 0.7 秒有效深度历史...'; until rostopic echo -n 1 /camera/depth/history_ready 2>/dev/null | grep -q 'data: True'; do sleep 0.2; done; echo '深度历史就绪，启动控制器'; exec sudo -E bash -lc 'export ROBOT_VERSION=55; export ROS_MASTER_URI=http://192.168.26.1:11311; export ROS_IP=192.168.26.1; source \"$ROS_SETUP\"; exec roslaunch humanoid_controllers load_kuavo_real.launch joystick_type:=bt2'"
else
    CONTROLLER_CMD="$LOWER_ROS_ENV; source '$ROS_SETUP'; echo 'camera-only：不会启动机器人控制器'; exec rostopic echo /camera/depth/postprocess_diagnostics"
fi
tmux send-keys -t "$SESSION:0.3" "$CONTROLLER_CMD" C-m

tmux select-layout -t "$SESSION:0" tiled
echo "会话 $SESSION 已启动（模式：$MODE）"
echo "上位机增强输入：/waist_camera/depth/image_enhanced_raw"
echo "跨机 compact：/waist_camera/depth/image_enhanced_compact（36x64，32FC1）"
echo "上位机 SDK 后处理：$([[ $MINIMAL_POST == true ]] && echo 'minimal（条件式 inpaint）' || echo 'legacy')"
echo "高斯模糊：$([[ $USE_GAUSSIAN == true ]] && echo '开启' || echo '关闭')"
echo "模型历史：/camera/depth/depth_history_array（8x36x64，按 0.1 秒时间戳采样，总跨度 0.7 秒）"

if ((ATTACH)); then
    exec tmux attach -t "$SESSION"
fi
