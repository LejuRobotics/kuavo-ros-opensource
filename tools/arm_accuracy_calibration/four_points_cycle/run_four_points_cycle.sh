#!/bin/bash
# 四点循环运动 + 动捕记录 一键启动脚本（本脚本与 four_points_cycle.py 均在本目录，输出默认在本目录 output/）
# 用法: ./run_four_points_cycle.sh [--build] [--mocap-sdk=optitrack|nokov] [-- 四点脚本参数...]
# 示例: ./run_four_points_cycle.sh --build -- --cycles 2 --record_hz 100

# ========== 在这里配置 ==========
MOCAP_SERVER_IP="10.10.30.229"
MOCAP_SDK="optitrack"
# =============================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 本目录在 arm_accuracy_calibration/four_points_cycle/，工作空间根目录再上两级
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
BUILD_FIRST=0
FOUR_POINTS_ARGS=()

# 解析参数：-- 后的参数传给 four_points_cycle.py
while [ $# -gt 0 ]; do
    if [ "$1" = "--" ]; then
        shift
        FOUR_POINTS_ARGS=("$@")
        break
    elif [ "$1" = "--build" ]; then
        BUILD_FIRST=1
    elif [[ "$1" == --mocap-sdk=* ]]; then
        MOCAP_SDK="${1#*=}"
    fi
    shift
done

if [ "$MOCAP_SDK" != "optitrack" ] && [ "$MOCAP_SDK" != "nokov" ]; then
    echo "错误: MOCAP_SDK 只能是 optitrack 或 nokov，当前为: $MOCAP_SDK"
    exit 1
fi

cleanup() {
    echo ""
    echo "正在退出，清理动捕进程..."
    [ -n "$MOCAP_PID" ] && kill $MOCAP_PID 2>/dev/null || true
    exit 0
}
trap cleanup INT TERM

echo "=========================================="
echo "  四点循环 + 动捕记录 - 一键启动"
echo "=========================================="
echo "  动捕 SDK: $MOCAP_SDK  服务器 IP: $MOCAP_SERVER_IP"
echo "  脚本与输出目录: $SCRIPT_DIR"
echo "  四点脚本参数: ${FOUR_POINTS_ARGS[*]}"
echo "------------------------------------------"

# 1. 可选编译
if [ $BUILD_FIRST -eq 1 ]; then
    echo "[1/4] 编译动捕包 ($MOCAP_SDK)..."
    cd "$WORKSPACE_ROOT"
    source /opt/ros/noetic/setup.bash 2>/dev/null || source /opt/ros/melodic/setup.bash 2>/dev/null || true
    if [ "$MOCAP_SDK" = "optitrack" ]; then
        catkin build optitrack_data_receive -DCMAKE_BUILD_TYPE=Release
    else
        catkin build kuavo_msgs -DCMAKE_BUILD_TYPE=Release || true
        catkin build motioncapture --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tee /tmp/motioncapture_build.log || true
        if [ -f "build/motioncapture/Makefile" ]; then
            cd build/motioncapture
            catkin build --get-env motioncapture | catkin env -si /usr/bin/make motioncapture_generate_messages_cpp || true
            cd "$WORKSPACE_ROOT"
        fi
        catkin build motioncapture kuavo_msgs -DCMAKE_BUILD_TYPE=Release
    fi
    echo "编译完成"
else
    echo "[1/4] 跳过编译（使用 --build 可强制编译）"
fi

# 2. Source 工作空间
echo "[2/4] 加载 ROS 环境..."
cd "$WORKSPACE_ROOT"
if [ -f "devel/setup.bash" ]; then
    source devel/setup.bash
elif [ -f "install/setup.bash" ]; then
    source install/setup.bash
else
    echo "错误: 未找到 devel/setup.bash 或 install/setup.bash，请先编译"
    exit 1
fi

# 3. 启动动捕
echo "[3/4] 启动动捕 ($MOCAP_SDK)，连接 $MOCAP_SERVER_IP ..."
if [ "$MOCAP_SDK" = "optitrack" ]; then
    export MOCAP_SERVER_IP="$MOCAP_SERVER_IP"
    rosrun optitrack_data_receive OptiTrack_Data_Receive.py "$MOCAP_SERVER_IP" &
else
    rosrun motioncapture SampleClient "$MOCAP_SERVER_IP" &
fi
MOCAP_PID=$!
sleep 3
if ! kill -0 $MOCAP_PID 2>/dev/null; then
    echo "错误: 动捕 ($MOCAP_SDK) 启动失败"
    exit 1
fi

# 4. 运行四点循环脚本（输出默认在 SCRIPT_DIR/output/）
echo "[4/4] 启动四点循环 (four_points_cycle.py)..."
echo "按 Ctrl+C 退出"
echo "------------------------------------------"
cd "$SCRIPT_DIR"
python3 four_points_cycle.py "${FOUR_POINTS_ARGS[@]}"
cleanup
