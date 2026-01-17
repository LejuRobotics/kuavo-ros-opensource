#!/bin/bash
# 生成 Protobuf C++ 文件的脚本
# 使用方法: ./generate.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "$SCRIPT_DIR")"
PROTOS_DIR="$PKG_DIR/protos"

echo "=== Protobuf C++ Generator ==="
echo "Protoc version: $(protoc --version)"
echo "Proto files dir: $PROTOS_DIR"
echo "Output dir: $SCRIPT_DIR"
echo ""

# 检查 proto 文件是否存在
PROTO_FILES=(
    "$PROTOS_DIR/hand_pose.proto"
    "$PROTOS_DIR/robot_info.proto"
    "$PROTOS_DIR/robot_state.proto"
    "$PROTOS_DIR/hand_wrench_srv.proto"
)

for f in "${PROTO_FILES[@]}"; do
    if [ ! -f "$f" ]; then
        echo "Error: Proto file not found: $f"
        exit 1
    fi
done

# 生成 C++ 文件
echo "Generating C++ files..."
protoc --proto_path="$PROTOS_DIR" --cpp_out="$SCRIPT_DIR" \
    hand_pose.proto \
    robot_info.proto \
    robot_state.proto \
    hand_wrench_srv.proto

# 修复 include 路径
echo "Fixing include paths..."
cd "$SCRIPT_DIR"
for file in *.pb.cc *.pb.h; do
    if [ -f "$file" ]; then
        sed -i 's|#include "protos/\([^"]*\)"|#include "\1"|g' "$file"
    fi
done

echo ""
echo "=== Generated files ==="
ls -la *.pb.cc *.pb.h 2>/dev/null || echo "No files generated"
echo ""
echo "Done!"

