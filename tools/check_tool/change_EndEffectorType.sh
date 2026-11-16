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

# 检查当前配置的末端执行器类型
echo -e "${BLUE}=== 检查当前配置 ===${NC}"
current_type=""
qiangnao_total=0
qiangnao_touch_total=0
lejuclaw_total=0
none_total=0
qibeng_total=0
linker_hand_total=0

for file in "${files_to_check[@]}"; do
    # 检查各种格式
    qiangnao_count=$(grep -oE '(qiangnao|"qiangnao"|default="qiangnao")' "$file" 2>/dev/null | wc -l)
    qiangnao_touch_count=$(grep -oE '(qiangnao_touch|"qiangnao_touch"|default="qiangnao_touch")' "$file" 2>/dev/null | wc -l)
    lejuclaw_count=$(grep -oE '(lejuclaw|"lejuclaw"|default="lejuclaw")' "$file" 2>/dev/null | wc -l)
    none_count=$(grep -oE '("None"|"none"|default="none")' "$file" 2>/dev/null | wc -l)
    qibeng_count=$(grep -oE '("Qibeng"|"qibeng"|default="qibeng")' "$file" 2>/dev/null | wc -l)
    linker_hand_count=$(grep -oE '("linker_hand"|"Linker_hand"|default="linker_hand")' "$file" 2>/dev/null | wc -l)
    
    qiangnao_total=$((qiangnao_total + qiangnao_count))
    qiangnao_touch_total=$((qiangnao_touch_total + qiangnao_touch_count))
    lejuclaw_total=$((lejuclaw_total + lejuclaw_count))
    none_total=$((none_total + none_count))
    qibeng_total=$((qibeng_total + qibeng_count))
    linker_hand_total=$((linker_hand_total + linker_hand_count))
done

# 判断当前配置的末端执行器类型
if [ "$qiangnao_total" -gt 0 ] && [ "$qiangnao_touch_total" -eq 0 ] && [ "$lejuclaw_total" -eq 0 ] && [ "$none_total" -eq 0 ] && [ "$qibeng_total" -eq 0 ] && [ "$linker_hand_total" -eq 0 ]; then
    current_type="qiangnao"
elif [ "$qiangnao_touch_total" -gt 0 ] && [ "$qiangnao_total" -eq 0 ] && [ "$lejuclaw_total" -eq 0 ] && [ "$none_total" -eq 0 ] && [ "$qibeng_total" -eq 0 ] && [ "$linker_hand_total" -eq 0 ]; then
    current_type="qiangnao_touch"
elif [ "$lejuclaw_total" -gt 0 ] && [ "$qiangnao_total" -eq 0 ] && [ "$qiangnao_touch_total" -eq 0 ] && [ "$none_total" -eq 0 ] && [ "$qibeng_total" -eq 0 ] && [ "$linker_hand_total" -eq 0 ]; then
    current_type="lejuclaw"
elif [ "$none_total" -gt 0 ] && [ "$qiangnao_total" -eq 0 ] && [ "$qiangnao_touch_total" -eq 0 ] && [ "$lejuclaw_total" -eq 0 ] && [ "$qibeng_total" -eq 0 ] && [ "$linker_hand_total" -eq 0 ]; then
    current_type="none"
elif [ "$qibeng_total" -gt 0 ] && [ "$qiangnao_total" -eq 0 ] && [ "$qiangnao_touch_total" -eq 0 ] && [ "$lejuclaw_total" -eq 0 ] && [ "$none_total" -eq 0 ] && [ "$linker_hand_total" -eq 0 ]; then
    current_type="qibeng"
elif [ "$linker_hand_total" -gt 0 ] && [ "$qiangnao_total" -eq 0 ] && [ "$qiangnao_touch_total" -eq 0 ] && [ "$lejuclaw_total" -eq 0 ] && [ "$none_total" -eq 0 ] && [ "$qibeng_total" -eq 0 ]; then
    current_type="linker_hand"
elif [ "$qiangnao_total" -eq 0 ] && [ "$qiangnao_touch_total" -eq 0 ] && [ "$lejuclaw_total" -eq 0 ] && [ "$none_total" -eq 0 ] && [ "$qibeng_total" -eq 0 ] && [ "$linker_hand_total" -eq 0 ]; then
    current_type="none"
else
    current_type="mixed"
fi

# 显示当前配置
echo -e "${YELLOW}当前配置的末端执行器类型:${NC}"
if [ "$current_type" = "mixed" ]; then
    echo -e "  ${YELLOW}混合配置 (qiangnao: $qiangnao_total, qiangnao_touch: $qiangnao_touch_total, lejuclaw: $lejuclaw_total, none: $none_total, qibeng: $qibeng_total, linker_hand: $linker_hand_total)${NC}"
else
    echo -e "  ${GREEN}$current_type${NC}"
    if [ "$current_type" = "qiangnao" ]; then
        echo -e "  (出现次数: $qiangnao_total)"
    elif [ "$current_type" = "qiangnao_touch" ]; then
        echo -e "  (出现次数: $qiangnao_touch_total)"
    elif [ "$current_type" = "lejuclaw" ]; then
        echo -e "  (出现次数: $lejuclaw_total)"
    elif [ "$current_type" = "none" ]; then
        echo -e "  (出现次数: $none_total)"
    elif [ "$current_type" = "qibeng" ]; then
        echo -e "  (出现次数: $qibeng_total)"
    elif [ "$current_type" = "linker_hand" ]; then
        echo -e "  (出现次数: $linker_hand_total)"
    fi
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
echo -e "${GREEN}q.${NC} 退出"
echo ""

while true; do
    read -p "请选择 (1/2/3/4/5/6/q): " choice
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
        q|Q)
            echo -e "${YELLOW}操作已取消${NC}"
            exit 0
            ;;
        *)
            echo -e "${RED}无效选择，请输入 1、2、3、4、5、6 或 q${NC}"
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

# 执行替换操作
# 如果是 qibeng 或 linker_hand，只处理 config_file
if [ "$target_type" = "qibeng" ] || [ "$target_type" = "linker_hand" ]; then
    files_to_process=("$config_file")
else
    files_to_process=("${files_to_check[@]}")
fi

for file in "${files_to_process[@]}"; do
    echo "处理: $file"
    
    # 根据文件类型进行不同的替换
    if [[ "$file" == *.json ]]; then
        # JSON 文件：替换 "EndEffectorType" 数组中的值
        if [ "$target_type" = "none" ]; then
            # 替换为 "None"
            sed -i 's/"qiangnao"/"None"/g' "$file"
            sed -i 's/"qiangnao_touch"/"None"/g' "$file"
            sed -i 's/"lejuclaw"/"None"/g' "$file"
            sed -i 's/"qibeng"/"None"/g' "$file"
            sed -i 's/"linker_hand"/"None"/g' "$file"
            sed -i 's/"None"/"None"/g' "$file"  # 确保已经是 None
            echo -e "${GREEN}✓ JSON 文件已更新为 None${NC}"
        else
            # 替换为目标类型（带引号）
            sed -i "s/\"qiangnao\"/\"$target_type\"/g" "$file"
            sed -i "s/\"qiangnao_touch\"/\"$target_type\"/g" "$file"
            sed -i "s/\"lejuclaw\"/\"$target_type\"/g" "$file"
            sed -i "s/\"qibeng\"/\"$target_type\"/g" "$file"
            sed -i "s/\"linker_hand\"/\"$target_type\"/g" "$file"
            sed -i "s/\"None\"/\"$target_type\"/g" "$file"
            echo -e "${GREEN}✓ JSON 文件已更新为 $target_type${NC}"
        fi
    elif [[ "$file" == *.launch ]]; then
        # Launch 文件：替换 default="..." 格式
        if [ "$target_type" = "none" ]; then
            sed -i 's/default="qiangnao"/default="none"/g' "$file"
            sed -i 's/default="qiangnao_touch"/default="none"/g' "$file"
            sed -i 's/default="lejuclaw"/default="none"/g' "$file"
            sed -i 's/default="qibeng"/default="none"/g' "$file"
            sed -i 's/default="linker_hand"/default="none"/g' "$file"
            sed -i 's/default="none"/default="none"/g' "$file"  # 确保已经是 none
            echo -e "${GREEN}✓ Launch 文件已更新为 none${NC}"
        else
            sed -i "s/default=\"qiangnao\"/default=\"$target_type\"/g" "$file"
            sed -i "s/default=\"qiangnao_touch\"/default=\"$target_type\"/g" "$file"
            sed -i "s/default=\"lejuclaw\"/default=\"$target_type\"/g" "$file"
            sed -i "s/default=\"qibeng\"/default=\"$target_type\"/g" "$file"
            sed -i "s/default=\"linker_hand\"/default=\"$target_type\"/g" "$file"
            sed -i "s/default=\"none\"/default=\"$target_type\"/g" "$file"
            echo -e "${GREEN}✓ Launch 文件已更新为 $target_type${NC}"
        fi
    else
        # 其他文件：通用替换
        if [ "$target_type" = "none" ]; then
            sed -i 's/qiangnao/none/g' "$file"
            sed -i 's/qiangnao_touch/none/g' "$file"
            sed -i 's/lejuclaw/none/g' "$file"
            sed -i 's/qibeng/none/g' "$file"
            sed -i 's/linker_hand/none/g' "$file"
            echo -e "${GREEN}✓ 文件已更新为 none${NC}"
        else
            sed -i "s/qiangnao/$target_type/g" "$file"
            sed -i "s/qiangnao_touch/$target_type/g" "$file"
            sed -i "s/lejuclaw/$target_type/g" "$file"
            sed -i "s/qibeng/$target_type/g" "$file"
            sed -i "s/linker_hand/$target_type/g" "$file"
            echo -e "${GREEN}✓ 文件已更新为 $target_type${NC}"
        fi
    fi
done

echo ""
echo -e "${GREEN}=== 替换操作完成 ===${NC}"
