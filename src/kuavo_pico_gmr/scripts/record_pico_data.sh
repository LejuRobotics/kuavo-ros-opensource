#!/bin/bash
# 录制 PICO 重定向姿态数据
# 输出: bag 文件 + npz 文件（含世界坐标 root 位姿，可直接用于 MuJoCo 可视化）

# 默认参数
TOPICS="/pico/retargeted_pose /pico/world_bone_poses /pico/raw_bone_poses"
DURATION=""  # 不限时长，手动 Ctrl+C 停止
OUTPUT_DIR="${HOME}/pico_recordings"
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
BAG_NAME="pico_data_${TIMESTAMP}.bag"
NPZ_NAME="pico_data_${TIMESTAMP}_retarget.npz"

# 创建输出目录
mkdir -p "${OUTPUT_DIR}"

OUTPUT_PATH="${OUTPUT_DIR}/${BAG_NAME}"
NPZ_PATH="${OUTPUT_DIR}/${NPZ_NAME}"

# 获取转换脚本路径
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONVERT_SCRIPT="${SCRIPT_DIR}/convert_pico_bag_to_bin.py"

echo "=========================================="
echo "  PICO 数据录制工具"
echo "=========================================="
echo "录制话题:"
echo "  - /pico/retargeted_pose  (重定向姿态)"
echo "  - /pico/world_bone_poses (世界坐标骨骼)"
echo "  - /pico/raw_bone_poses   (原始骨骼数据)"
echo ""
echo "输出:"
echo "  bag: ${OUTPUT_PATH}"
echo "  npz: ${NPZ_PATH}"
echo ""
echo "按 Ctrl+C 停止录制..."
echo "=========================================="
echo ""

# 启动录制
if [ -z "$DURATION" ]; then
    rosbag record -O "${OUTPUT_PATH}" ${TOPICS} &
else
    rosbag record -O "${OUTPUT_PATH}" --duration="${DURATION}" ${TOPICS} &
fi

ROSBAG_PID=$!
START_TIME=$(date +%s)
echo "录制中..."

# ---- 公共函数 ----

print_duration() {
    local END_TIME=$(date +%s)
    local TOTAL=$((END_TIME - START_TIME))
    local H=$((TOTAL / 3600))
    local M=$(((TOTAL % 3600) / 60))
    local S=$((TOTAL % 60))
    echo ""
    echo "=========================================="
    echo "录制完成！"
    echo "总时长: $(printf '%02d:%02d:%02d' $H $M $S)"
    echo "Bag 文件: ${OUTPUT_PATH}"
    echo "=========================================="
}

do_convert() {
    echo ""
    echo "正在转换为 npz 格式..."

    # convert_pico_bag_to_bin.py 会同时生成 .bin 和 .npz，转换完删除 .bin
    local TMP_BIN="${OUTPUT_DIR}/pico_data_${TIMESTAMP}_retarget.bin"

    if [ ! -f "${CONVERT_SCRIPT}" ]; then
        echo "⚠ 找不到转换脚本: ${CONVERT_SCRIPT}"
        return 1
    fi

    python3 "${CONVERT_SCRIPT}" "${OUTPUT_PATH}" "${TMP_BIN}" --topic /pico/retargeted_pose
    local RESULT=$?

    # 删除不需要的 bin 文件，只保留 npz
    rm -f "${TMP_BIN}"

    return $RESULT
}

print_result() {
    local CONVERT_RESULT=$1
    echo ""
    echo "=========================================="
    if [ $CONVERT_RESULT -eq 0 ] && [ -f "${NPZ_PATH}" ]; then
        echo "✓ 转换成功！"
        echo "  npz: ${NPZ_PATH}"
        echo ""
        echo "npz 内容:"
        echo "  vmp_data: [N, 77]  VMP 77维数据"
        echo "  root:     [N, 7]   世界坐标位姿 (x,y,z,qx,qy,qz,qw)"
        echo ""
        echo "MuJoCo 可视化:"
        echo "  python ${SCRIPT_DIR}/visualize_retarget_bin.py --npz ${NPZ_PATH}"
    else
        echo "⚠ npz 转换失败（bag 文件已保存，可手动转换）"
        echo "  手动转换: python3 ${CONVERT_SCRIPT} ${OUTPUT_PATH} /tmp/tmp.bin"
    fi
    echo ""
    echo "查看 bag 信息: rosbag info ${OUTPUT_PATH}"
    echo "=========================================="
}

# ---- Ctrl+C 处理 ----

cleanup_and_convert() {
    echo ''
    echo '录制被中断，正在停止...'
    kill ${ROSBAG_PID} 2>/dev/null
    wait ${ROSBAG_PID} 2>/dev/null || true
    sleep 2

    print_duration
    do_convert
    print_result $?
    exit 0
}

trap cleanup_and_convert INT

# ---- 实时计时 ----

while kill -0 ${ROSBAG_PID} 2>/dev/null; do
    CURRENT_TIME=$(date +%s)
    ELAPSED=$((CURRENT_TIME - START_TIME))
    H=$((ELAPSED / 3600))
    M=$(((ELAPSED % 3600) / 60))
    S=$((ELAPSED % 60))
    printf "\r已录制: %02d:%02d:%02d" $H $M $S
    sleep 1
done

wait ${ROSBAG_PID} 2>/dev/null || true

print_duration
do_convert
print_result $?
