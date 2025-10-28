#!/usr/bin/env bash
set -euo pipefail

# === 配置项 ===
APP_NAME="kuavo_joint_cali_standalone"
MAIN_FILE="joint_cali_standalone.py"

# 所有相关文件都在同一目录，head_cali_config.yaml 放在 ./config/ 下
ASSETS=(
  "start_host_apriltag.py:."
  "head_cali.py:."
  "arm_cail_noui.py:."
  "arm_kinematics.py:."
  "apriltag_cube.py:."
  "ik_cmd.py:."
  "identifiability_analyzer.py:."
  "target_tracker.py:."
  "create_venv.sh:."
  "requirements.txt:."
  "bags/hand_move_demo_left.bag:bags"
  "bags/hand_move_demo_right.bag:bags"
  "config/cube_config.yaml:config"
  "config/head_cali_config.yaml:config"   # 注意：目标是目录，不是同名文件
)

# === 进入脚本所在目录 ===
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "==> 打包目录: $SCRIPT_DIR"
echo "==> 主程序:   $MAIN_FILE"
echo "==> 输出名:   $APP_NAME"

# === 基本检查 ===
[[ -f "$MAIN_FILE" ]] || { echo "ERROR: 找不到 $MAIN_FILE"; exit 1; }

MISSING=0
for item in "${ASSETS[@]}"; do
  SRC="${item%%:*}"
  if [[ ! -f "$SRC" ]]; then
    echo "ERROR: 找不到资源文件: $SRC"
    MISSING=1
  fi
done
(( MISSING == 0 )) || exit 1

# === 确保 ~/.local/bin 在 PATH（pip --user 默认装这里）===
export PATH="$HOME/.local/bin:$PATH"

# === 安装 / 检测 PyInstaller ===
if ! command -v pyinstaller >/dev/null 2>&1; then
  echo "==> 未检测到 pyinstaller，正在安装（--user）..."
  python3 -m pip install --user -U pyinstaller
fi

# 选择调用方式：优先可执行，其次模块调用
if command -v pyinstaller >/dev/null 2>&1; then
  PYI_CMD="pyinstaller"
else
  PYI_CMD="python3 -m PyInstaller"
fi
echo "==> 使用 PyInstaller 命令: $PYI_CMD"

# === 清理旧产物 ===
echo "==> 清理旧产物..."
rm -rf build dist "${APP_NAME}.spec"

# === 生成构建信息 ===
BUILD_META="build_info.json"
echo "==> 生成构建信息: $BUILD_META"
cat > "$BUILD_META" <<'EOF'
{
  "placeholder": true
}
EOF
# 用实际信息覆盖（保持 JSON 合法）
jq -n \
  --arg app_name "$APP_NAME" \
  --arg built_at_local "$(date +%Y-%m-%dT%H:%M:%S%z)" \
  > "$BUILD_META" 2>/dev/null || {
  # 没有 jq 也能用
  cat > "$BUILD_META" <<EOF
{
  "app_name": "$APP_NAME",
  "built_at_local": "$(date +%Y-%m-%dT%H:%M:%S%z)"
}
EOF
}

# === 将构建信息加入打包资源 ===
ASSETS+=( "$BUILD_META:." )

# === 组装 --add-data 参数 ===
ADD_DATA_ARGS=()
for item in "${ASSETS[@]}"; do
  ADD_DATA_ARGS+=( --add-data "$item" )
done

# === 开始打包 ===
echo "==> 开始打包..."
# shellcheck disable=SC2068
$PYI_CMD \
  --onefile \
  --name "$APP_NAME" \
  ${ADD_DATA_ARGS[@]} \
  "$MAIN_FILE"

# === 打包完成 ===
OUT="dist/${APP_NAME}"
if [[ -f "$OUT" ]]; then
  chmod +x "$OUT" || true
  echo "==> 打包完成: $OUT"
  echo
  echo "用法示例："
  echo "  ${OUT} --yes"
  echo "  ${OUT} --ros-setup '' --ws-setup '' --yes"
  echo "  ${OUT} --skip-host --skip-arm --yes"
else
  echo "ERROR: 未找到输出文件 ${OUT}"
  exit 1
fi
