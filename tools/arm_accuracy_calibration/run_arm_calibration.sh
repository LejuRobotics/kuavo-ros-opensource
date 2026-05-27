#!/bin/bash
# 手臂精度标定一键启动脚本
# 自动完成：编译、source、启动动捕 SDK、运行模块一 run_data_collection.py
# 用法: ./run_arm_calibration.sh [--build] [--mocap-sdk=optitrack|nokov] [-- <run_data_collection.py 参数...>]

# ========== 在这里配置 ==========
# 动捕服务器 IP（两个 SDK 均使用此 IP，无需在程序内再输入）
MOCAP_SERVER_IP="10.10.31.4"
# 动捕 SDK 选择：optitrack（OptiTrack/Motive） 或 nokov（Nokov 星云/SeekerSDK）
MOCAP_SDK="optitrack"
# =============================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
BUILD_FIRST=0
PY_ARGS=()

# 解析参数
while [[ $# -gt 0 ]]; do
    case "$1" in
        --build)
            BUILD_FIRST=1
            shift
            ;;
        --mocap-sdk=*)
            MOCAP_SDK="${1#*=}"
            shift
            ;;
        --)
            shift
            PY_ARGS+=("$@")
            break
            ;;
        *)
            PY_ARGS+=("$1")
            shift
            ;;
    esac
done

# 校验 SDK 选择
if [ "$MOCAP_SDK" != "optitrack" ] && [ "$MOCAP_SDK" != "nokov" ]; then
    echo "错误: MOCAP_SDK 只能是 optitrack 或 nokov，当前为: $MOCAP_SDK"
    exit 1
fi

# 清理函数：Ctrl+C 时杀掉动捕进程
cleanup() {
    echo ""
    echo "正在退出，清理动捕进程..."
    [ -n "$MOCAP_PID" ] && kill $MOCAP_PID 2>/dev/null || true
    exit 0
}
trap cleanup INT TERM

echo "=========================================="
echo "  手臂精度标定 - 一键启动"
echo "=========================================="
echo "  动捕 SDK: $MOCAP_SDK  服务器 IP: $MOCAP_SERVER_IP"
echo "------------------------------------------"

# 1. 可选：按选择的 SDK 编译
if [ $BUILD_FIRST -eq 1 ]; then
    echo "[1/4] 编译动捕包 ($MOCAP_SDK)..."
    cd "$WORKSPACE_ROOT"
    source /opt/ros/noetic/setup.bash 2>/dev/null || source /opt/ros/melodic/setup.bash 2>/dev/null || true

    if [ "$MOCAP_SDK" = "optitrack" ]; then
        echo "  编译 optitrack_data_receive..."
        catkin build optitrack_data_receive -DCMAKE_BUILD_TYPE=Release
    else
        # nokov: motioncapture（依赖 kuavo_msgs 与消息生成）
        echo "  步骤 1/3: 编译依赖包 kuavo_msgs..."
        catkin build kuavo_msgs -DCMAKE_BUILD_TYPE=Release || true
        echo "  步骤 2/3: 配置 motioncapture 并生成消息头文件..."
        catkin build motioncapture --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tee /tmp/motioncapture_build.log || true
        if [ -f "build/motioncapture/Makefile" ]; then
            echo "  正在生成消息头文件..."
            cd build/motioncapture
            catkin build --get-env motioncapture | catkin env -si /usr/bin/make motioncapture_generate_messages_cpp || true
            cd "$WORKSPACE_ROOT"
        fi
        echo "  步骤 3/3: 完整编译 motioncapture..."
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

# 3. 启动动捕 SDK（IP 由脚本指定，无需手动输入）
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

# 4. 运行模块一数据采集
echo "[4/4] 启动模块一数据采集 (run_data_collection.py)..."
echo "按 Ctrl+C 退出"
echo "------------------------------------------"
cd "$SCRIPT_DIR"
python3 run_data_collection.py "${PY_ARGS[@]}"
cleanup
