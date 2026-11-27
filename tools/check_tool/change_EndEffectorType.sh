#!/bin/bash

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 末端执行器配置更改脚本
# 检查当前配置并允许用户选择要更换的末端执行器类型

echo -e "${BLUE}=== 末端执行器配置更改脚本 ===${NC}"
echo ""

# 检查当前目录向上3层是否为kuavo-ros-opensource或kuavo-ros-control
project_root=""
current_path="$PWD"

# 检查当前目录及向上3层目录，找到包含 src/ 子目录的项目根目录
for i in {0..3}; do
    check_path="$current_path"
    for ((j=0; j<i; j++)); do
        check_path=$(dirname "$check_path")
    done
    
    if [ -d "$check_path/src" ]; then
        project_root="$check_path"
        break
    fi
done

if [ -z "$project_root" ]; then
    echo -e "${RED}错误: 在当前目录向上3层内未找到包含 src/ 子目录的项目根目录${NC}"
    echo "当前目录: $current_path"
    echo "请确保在正确的项目目录下运行此脚本"
    exit 1
fi

# 切换到项目根目录
cd "$project_root" || exit 1
echo -e "${GREEN}✓ 找到项目根目录: $project_root${NC}"
echo ""

# 检查ROBOT_VERSION环境变量
if [ -z "$ROBOT_VERSION" ]; then
    echo -e "${RED}错误: 未设置ROBOT_VERSION环境变量${NC}"
    echo "请先设置ROBOT_VERSION环境变量，例如: export ROBOT_VERSION=45"
    exit 1
fi

echo -e "${BLUE}检测到ROBOT_VERSION: $ROBOT_VERSION${NC}"
echo ""

# 根据ROBOT_VERSION动态确定配置文件路径
config_file="src/kuavo_assets/config/kuavo_v${ROBOT_VERSION}/kuavo.json"

# 检查文件是否存在
files_to_check=(
    "$config_file"
    "src/manipulation_nodes/noitom_hi5_hand_udp_python/launch/launch_quest3_ik.launch"
    "src/humanoid-control/humanoid_controllers/launch/load_kuavo_real_with_vr.launch"
)

for file in "${files_to_check[@]}"; do
    if [ -f "$file" ]; then
        echo -e "${GREEN}✓${NC} $file"
    else
        echo -e "${RED}✗${NC} $file"
        echo -e "${RED}错误: 无法找到文件 $file${NC}"
        exit 1
    fi
done

echo -e "${GREEN}✓ 所有文件存在${NC}"
echo ""

echo -e "${BLUE}=== 检查当前配置 ===${NC}"
current_config=$(python3 - "${files_to_check[@]}" <<'PY'
import json
import sys
import xml.etree.ElementTree as ET
from collections import Counter
from pathlib import Path

try:
    config_path, *launch_paths = sys.argv[1:]
    config = json.loads(Path(config_path).read_bytes())
    hands = config.get("EndEffectorType") if isinstance(config, dict) else None
    if not isinstance(hands, list) or len(hands) != 2 or not all(
        isinstance(value, str) and value for value in hands
    ):
        raise ValueError(f"{config_path} 的 EndEffectorType 必须包含左右手两个类型")

    counts = Counter(value.lower() for value in hands)
    for launch_path in launch_paths:
        args = ET.parse(launch_path).findall("./arg[@name='ee_type']")
        if len(args) != 1 or args[0].get("default") is None:
            raise ValueError(f"{launch_path} 的顶层 ee_type 缺失、重复或没有默认值")
        default = args[0].get("default")
        if default:
            counts[default.lower()] += 1

except (ValueError, OSError, ET.ParseError) as error:
    print(f"读取当前末端配置失败：{error}", file=sys.stderr)
    sys.exit(1)

if len(counts) == 1:
    current_type, count = next(iter(counts.items()))
    print(current_type)
    print(count)
else:
    print("mixed")
    print(", ".join(f"{name}: {count}" for name, count in sorted(counts.items())))
PY
) || exit 1
current_type=${current_config%%$'\n'*}
type_counts=${current_config#*$'\n'}

echo -e "${YELLOW}当前配置的末端执行器类型:${NC}"
if [ "$current_type" = "mixed" ]; then
    printf '  %b混合配置（%s）%b\n' "$YELLOW" "$type_counts" "$NC"
else
    printf '  %b%s%b\n' "$GREEN" "$current_type" "$NC"
    printf '  (出现次数: %s)\n' "$type_counts"
fi
echo ""

# 显示菜单让用户选择
echo -e "${BLUE}=== 选择要更换的末端执行器类型 ===${NC}"
echo -e "${GREEN}1.${NC} qiangnao"
echo -e "${GREEN}2.${NC} lejuclaw"
echo -e "${GREEN}3.${NC} none"
echo -e "${GREEN}4.${NC} qiangnao_touch"
echo -e "${GREEN}5.${NC} qibeng"
echo -e "${GREEN}6.${NC} linker_hand"
echo -e "${GREEN}7.${NC} heiman"
echo -e "${GREEN}q.${NC} 退出"
echo ""

while true; do
    read -p "请选择 (1/2/3/4/5/6/7/q): " choice
    case "$choice" in
        1)
            target_type="qiangnao"
            break
            ;;
        2)
            target_type="lejuclaw"
            break
            ;;
        3)
            target_type="none"
            break
            ;;
        4)
            target_type="qiangnao_touch"
            break
            ;;
        5)
            target_type="qibeng"
            break
            ;;
        6)
            target_type="linker_hand"
            break
            ;;
        7)
            target_type="heiman"
            break
            ;;
        q|Q)
            echo -e "${YELLOW}操作已取消${NC}"
            exit 0
            ;;
        *)
            echo -e "${RED}无效选择，请输入 1、2、3、4、5、6、7 或 q${NC}"
            ;;
    esac
done

# 如果选择的是当前类型，询问是否继续
if [ "$target_type" = "$current_type" ] && [ "$current_type" != "mixed" ]; then
    echo -e "${YELLOW}当前配置已经是 $target_type，是否继续？(y/n)${NC}"
    read -r confirm
    if [[ ! "$confirm" =~ ^[Yy]$ ]]; then
        echo -e "${YELLOW}操作已取消${NC}"
        exit 0
    fi
fi

echo ""
echo -e "${BLUE}开始执行替换操作...${NC}"
echo -e "${YELLOW}目标类型: $target_type${NC}"
echo ""

# 同步更新当前机型的 JSON 和两份 Launch。
if ! python3 - "$target_type" "${files_to_check[@]}" <<'PY'
import json
import os
import re
import stat
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

target = sys.argv[1]
updates = []
temporary_files = []
written = []

try:
    # 所有文件通过字段和格式检查后，再开始写入。
    for filename in sys.argv[2:]:
        path = Path(filename)
        original = path.read_bytes()
        text = original.decode("utf-8")

        if path.suffix == ".json":
            data = json.loads(text)
            values = data.get("EndEffectorType")
            if not isinstance(values, list) or len(values) != 2:
                raise ValueError(f"{path} 的 EndEffectorType 必须包含左右手两个值")

            value = "None" if target == "none" else target
            expected = [value, value]
            pattern = r'(?m)^([ \t]*"EndEffectorType"\s*:\s*)\[[^\]]*\]'
            updated, count = re.subn(
                pattern,
                lambda match: match[1] + json.dumps(expected),
                text,
            )
            data["EndEffectorType"] = expected
            if count != 1 or json.loads(updated) != data:
                raise ValueError(f"{path} 的 EndEffectorType 替换校验失败")

        elif path.suffix == ".launch":
            # 只匹配固定默认值，包括空值，保留 $(arg ee_type) 转发。
            pattern = (
                r'(?m)^([ \t]*<arg\b(?=[^>]*\bname="ee_type")'
                r'[^>]*\bdefault=")[^"$]*(")'
            )
            updated, count = re.subn(
                pattern,
                lambda match: match[1] + target + match[2],
                text,
            )
            args = ET.fromstring(updated).findall("./arg[@name='ee_type']")
            if count != 1 or len(args) != 1 or args[0].get("default") != target:
                raise ValueError(f"{path} 的顶层 ee_type 缺失、重复或替换校验失败")

        else:
            raise ValueError(f"不支持的文件类型 {path}")

        updates.append((path, original, updated.encode("utf-8")))

    # 在各文件所在目录准备新内容和回滚内容，避免写入中途截断。
    for path, original, updated in updates:
        file_stat = path.stat()
        staged = []
        for content in (updated, original):
            with tempfile.NamedTemporaryFile(dir=path.parent, delete=False) as stream:
                temporary = Path(stream.name)
                temporary_files.append(temporary)
                stream.write(content)
                stream.flush()
                os.fsync(stream.fileno())
                os.fchown(stream.fileno(), file_stat.st_uid, file_stat.st_gid)
                os.fchmod(stream.fileno(), stat.S_IMODE(file_stat.st_mode))
            staged.append(temporary)
        written.append((path, staged[0], staged[1]))

    for path, staged, backup in written:
        os.replace(staged, path)

    # 写入后逐个读回核对，全部成功后输出成功提示。
    for path, original, updated in updates:
        if path.read_bytes() != updated:
            raise OSError(f"{path} 写入后内容校验失败")

except (Exception, KeyboardInterrupt) as error:
    print(f"配置修改失败 {error}", file=sys.stderr)
    for path, staged, backup in reversed(written):
        if not staged.exists():
            try:
                os.replace(backup, path)
            except OSError as rollback_error:
                # 保留原内容，便于恢复。
                temporary_files.remove(backup)
                print(f"回滚失败 {path}，原内容保留在 {backup}，{rollback_error}", file=sys.stderr)
    sys.exit(1)
finally:
    for temporary in temporary_files:
        temporary.unlink(missing_ok=True)

for path, original, updated in updates:
    print(f"✓ 已更新并校验 {path}")
PY
then
    echo -e "${RED}末端执行器配置更新失败${NC}" >&2
    exit 1
fi

echo ""
echo -e "${GREEN}=== 替换操作完成 ===${NC}"
