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

# ===== 一次性生成所有 proto =====
protoc \
  -I="${ROOT_DIR}" \
  --python_out="${PY_OUT}" \
  --csharp_out="${CS_OUT}" \
  "${PROTO_DIR}"/*.proto

echo "✅ Protobuf generation finished successfully."
