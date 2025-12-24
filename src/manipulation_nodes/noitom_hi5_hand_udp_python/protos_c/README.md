# Pre-generated Protobuf C++ Files

本目录包含预生成的 Protobuf C++ 文件。

## 如何重新生成

当 `.proto` 文件修改后，需要重新生成这些文件：

```bash
# 进入包目录
cd src/manipulation_nodes/noitom_hi5_hand_udp_python

# 确保使用正确版本的 protoc（需要与目标系统的 libprotobuf 版本匹配）
protoc --version  # 应该显示 libprotoc 3.6.1

# 生成 C++ 文件（使用 --proto_path 指定搜索路径）
protoc --proto_path=protos --cpp_out=protos_c \
    hand_pose.proto \
    robot_info.proto \
    robot_state.proto \
    hand_wrench_srv.proto
```

## 一键生成脚本

也可以使用以下脚本：

```bash
#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "$SCRIPT_DIR")"

echo "Protoc version: $(protoc --version)"

# 生成 C++ 文件
protoc --cpp_out="$SCRIPT_DIR" \
    "$PKG_DIR/protos/hand_pose.proto" \
    "$PKG_DIR/protos/robot_info.proto" \
    "$PKG_DIR/protos/robot_state.proto"

# 修复 include 路径
cd "$SCRIPT_DIR"
for file in *.pb.cc *.pb.h; do
    sed -i 's|#include "protos/\([^"]*\)"|#include "\1"|g' "$file"
done

echo "Generated files:"
ls -la *.pb.cc *.pb.h
```

## 文件列表

| 文件 | 源 proto 文件 |
|------|--------------|
| hand_pose.pb.cc/.h | protos/hand_pose.proto |
| robot_info.pb.cc/.h | protos/robot_info.proto |
| robot_state.pb.cc/.h | protos/robot_state.proto |

## 注意事项

1. **版本匹配**：生成时使用的 protoc 版本必须与目标系统的 libprotobuf 版本兼容
2. **修改后重新生成**：修改 .proto 文件后，务必重新生成并提交 .pb.cc/.pb.h 文件
3. **不要手动编辑**：这些文件是自动生成的，不要手动修改

