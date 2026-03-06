#!/bin/bash
set -e

# ===== 项目根目录（执行脚本的位置）=====
ROOT_DIR=$(cd "$(dirname "$0")" && pwd)

PROTO_DIR="${ROOT_DIR}/protos"
PY_OUT="${ROOT_DIR}/protos"
CS_OUT="${ROOT_DIR}/csharp"

echo "Proto dir : ${PROTO_DIR}"
echo "Python out: ${PY_OUT}"
echo "CSharp out: ${CS_OUT}"

# ===== 准备目录 =====
mkdir -p "${PY_OUT}"
mkdir -p "${CS_OUT}"

# ===== 清理旧的 pb2（非常重要）=====
rm -f "${PY_OUT}"/*_pb2.py
rm -rf "${PY_OUT}/protos"

# ===== 一次性生成所有 proto =====
cd "${ROOT_DIR}"
protoc \
  -I="." \
  --python_out="${PY_OUT}" \
  --csharp_out="${CS_OUT}" \
  protos/hand_pose.proto \
  protos/robot_info.proto \
  protos/robot_state.proto \
  protos/hand_wrench_srv.proto \
  protos/kuavo_vr_events.proto

# 移动生成的文件到正确位置
if [ -d "${PY_OUT}/protos" ]; then
  mv "${PY_OUT}/protos"/*_pb2.py "${PY_OUT}/" 2>/dev/null || true
  rmdir "${PY_OUT}/protos" 2>/dev/null || true
fi

echo "✅ Protobuf generation finished successfully."
