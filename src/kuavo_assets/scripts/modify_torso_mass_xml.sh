#!/bin/bash

# 函数：获取 URDF 文件的总质量

function get_xml_total_mass() {
    urdf_file=$1
    grep -Pzo '(?s)mass\s*[^>]*?="\K[0-9.]+' "$urdf_file" | tr '\0' '\n' | awk '{sum += $1} END {printf "%.4f\n", sum}'
}

function get_link_mass() {
    urdf_file=$1
    link_name=$2
    grep -Pzo '(?s)<body[[:space:]]*name="'"$link_name"'".*?mass\s*[^>]*?="\K[0-9.]+' "$urdf_file" | tr '\0' '\n'
}

# 函数：修改特定 link 的质量值
function modify_link_mass() {
    urdf_file=$1
    link_name=$2
    new_mass_value=$3

    original_owner=$(stat -c '%u:%g' "$urdf_file")
    original_perms=$(stat -c '%a' "$urdf_file")

    # 创建临时文件
    temp_file=$(mktemp)
    
    # 使用sed进行跨行匹配和替换
    sed -E '
    # 读取整个文件到模式空间
    :a
    N
    $!ba
    # 替换指定link中的mass值
    s/(<body[[:space:]]*name="'"$link_name"'"[[:space:]]*.*?mass[[:space:]]*=")[0-9.]+(".*?mesh="(base_link|torso)".*?)/\1'"$new_mass_value"'\2/
    ' "$urdf_file" > "$temp_file"
    # s/(<link[[:space:]]*name="base_link"[[:space:]]*>.*?<mass[[:space:]]*value=")[0-9.]+(".*?<\/link>)/\1'"$new_mass_value"'\2/
    # 检查替换是否成功
    if diff "$urdf_file" "$temp_file" >/dev/null; then
        echo "未修改:$urdf_file"
        rm "$temp_file"
        return 1
    else
        # 将修改后的内容移回原文件
        chown $original_owner "$temp_file"
        chmod $original_perms "$temp_file"
        
        mv "$temp_file" "$urdf_file"
        echo "已修改 $link_name 的质量值"
        return 0
    fi
}
# 主程序部分
urdf_file="$1"        # 第一个参数为 URDF 文件路径
link_name="${3:-base_link}"         #  link 名称
new_mass_value="$2" # 

# 原始质量
original_total_mass=$(get_xml_total_mass "$urdf_file")
base_link_mass=$(get_link_mass "$urdf_file" "$link_name")
echo "原始总质量: $original_total_mass kg"
echo "原始 $link_name 的质量: $base_link_mass kg"

mass_diff=$(echo "$new_mass_value - $original_total_mass"| bc)
echo "质量差值: $mass_diff kg"
abs_mass_diff=$(echo "if ($mass_diff < 0) -1 * $mass_diff else $mass_diff" | bc)
if (( $(echo "$abs_mass_diff > 0.00001" | bc -l) )); then

    new_base_link_mass=$(echo "$base_link_mass + $mass_diff"| bc)

    modify_link_mass "$urdf_file" "$link_name" "$new_base_link_mass"

    echo "修改后的 $link_name 的质量值: $(get_link_mass "$urdf_file" "$link_name")"

    # 输出替换后的总质量
    echo "替换后的总质量: $(get_xml_total_mass "$urdf_file")"
else
    echo "$urdf_file 质量${original_total_mass}kg,未变化"
fi
