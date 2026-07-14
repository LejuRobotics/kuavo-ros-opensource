#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ── Step 1: 更新自定义 URDF ──────────────────────────────────────────────────
echo "[1/3] 更新自定义 URDF: $SCRIPT_DIR/config/biped_v3_arm_custom.urdf"
python3 "$SCRIPT_DIR/update_urdf.py" \
  --update-urdf "$SCRIPT_DIR/config/biped_v3_arm_custom.urdf"
echo "[1/3] URDF 更新完成"

# ── Step 2: 后台启动主节点（提供 FK 服务）────────────────────────────────────
echo "[2/3] 启动 humanoid_controllers 主节点..."
roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch \
  > /tmp/humanoid_launch.log 2>&1 &
LAUNCH_PID=$!

# 脚本退出时自动清理后台节点
cleanup() {
  if kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "[cleanup] 终止后台节点 (pid=$LAUNCH_PID)"
    kill "$LAUNCH_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT

# ── Step 3: 等待 FK 服务就绪（最多 10 秒）────────────────────────────────────
echo "[3/3] 等待 /ik/fk_srv_with_refer_frame 服务（超时 10s）..."
TIMEOUT=10
ELAPSED=0
SERVICE_FOUND=0

while [ "$ELAPSED" -lt "$TIMEOUT" ]; do
  if rosservice list 2>/dev/null | grep -q "/ik/fk_srv_with_refer_frame"; then
    SERVICE_FOUND=1
    break
  fi
  sleep 1
  ELAPSED=$((ELAPSED + 1))
  echo "  等待中... ${ELAPSED}s / ${TIMEOUT}s"
done

if [ "$SERVICE_FOUND" -eq 0 ]; then
  echo "[ERROR] 超过 ${TIMEOUT}s 未检测到 /ik/fk_srv_with_refer_frame，退出。"
  exit 1
fi

echo "[3/3] 服务就绪，运行 FK 对比脚本..."
python3 "$SCRIPT_DIR/plt_fk_optimized_result.py"
