#!/bin/bash

# 接收源码目录作为参数
SRC_DIR=$1
# SRC_DIR="/home/fandes/workspace/kuavo-ros-control/src/kuavo_assets"
# 获取机器人版本
if [ -z "${ROBOT_VERSION}" ]; then
    echo -e "\033[31m\nError: ROBOT_VERSION 机器人版本号环境变量未设置！\033[0m" >&2
    echo -e "\033[31m请参考README.md文档, 确认你的机器人版本，通过以下方式设置机器人版本号:\033[0m" >&2
    echo -e "\033[32m1. 在当前终端执行(临时设置): \n
        export ROBOT_VERSION=40\033[0m" >&2 
    echo -e "\033[32m\n2. 将其添加到你的 ~/.bashrc 或者 ~/.zshrc 终端配置文件中:
    如执行: \n
        echo 'export ROBOT_VERSION=40' >> ~/.bashrc \n
    添加到 ~/.bashrc 文件(bash终端)末尾，重启终端后生效\033[0m\n" >&2
    exit 1 
fi

# 允许的机器人版本号列表
allowed_versions=("11" "13" "30" "31" "32" "40" "41" "42" "43" "45" "46" "47" "48" "49" "50" "100045" "100049")
if ! [[ " ${allowed_versions[@]} " =~ " ${ROBOT_VERSION} " ]]; then
    echo -e "\033[31m\nError: 机器人版本号(环境变量中 ROBOT_VERSION 的值) = '${ROBOT_VERSION}' 无效 \033[0m" >&2
    echo -e "\033[31m请参考readme.md文档，确认你的机器人版本号\n目前可用的版本号有: \n[${allowed_versions[*]}] \033[0m\n" >&2
    exit 1
fi

ROBOT_VERSION=${ROBOT_VERSION}  # 如果环境变量未设置，默认为1

echo $ROBOT_VERSION
# 进入脚本所在目录
cd "$(dirname "$0")"

# 定义URDF文件列表
URDF_FILES=(
    "biped_s${ROBOT_VERSION}.urdf"
    "biped_s${ROBOT_VERSION}_gazebo.urdf"
    "drake/biped_v3_all_joint.urdf"
    "drake/biped_v3.urdf"
    "drake/biped_v3_full.urdf"
    # 可以继续添加其他URDF文件
)

# 需要修改的link名称
LINK_NAMES=(
    "base_link"
    "base_link"
    "torso"
    "torso"
    "torso"
)


# 构建基础URDF路径
BASE_URDF_PATH="${SRC_DIR}/models/biped_s${ROBOT_VERSION}/urdf"

CONFIG_DIR=~/.config/lejuconfig
MASS_FILE=${CONFIG_DIR}/TotalMassV${ROBOT_VERSION}
MASS_BAK=${CONFIG_DIR}/.TotalMassV${ROBOT_VERSION}.bak

# 修改OCS2_DIR路径
OCS2_DIR=/var/ocs2/kuavo_v${ROBOT_VERSION}

# 清理旧的cppad缓存文件夹函数
cleanup_old_cppad_cache() {
    local cache_dir=$1
    local keep_num=50

    if [ ! -d "$cache_dir" ]; then
        return
    fi

    # 获取文件夹列表，按照最后修改时间排序（包含子文件夹的修改时间）
    # -t 按时间排序，-r 反向排序（最新的在前），-d 只列出目录
    folders=$(find "$cache_dir" -mindepth 1 -maxdepth 1 -type d -printf '%T@ %p\n' | sort -nr | cut -d' ' -f2-)
    
    # 计算文件夹总数
    total_folders=$(echo "$folders" | wc -l)
    
    if [ "$total_folders" -gt "$keep_num" ]; then
        # 删除旧的文件夹，保留最新的50个
        echo -e "\033[33m\nNote: Cleaning up old cppad cache directories in ${cache_dir}, keeping newest ${keep_num} directories\033[0m" >&2
        echo "$folders" | tail -n +$((keep_num + 1)) | while read folder; do
            echo "Removing old cache: $folder"
            rm -rf "$folder"
        done
    fi
}

# 确保配置目录存在
mkdir -p ${CONFIG_DIR}

# 定义MD5配置文件路径
MD5_FILE=${CONFIG_DIR}/.urdf_md5_v${ROBOT_VERSION}

# 如果totalMass不存在，创建文件
if [ ! -f ${MASS_FILE} ]; then
    # 对每个URDF文件执行get_urdf_mass.sh
    for URDF_FILE in "${URDF_FILES[@]}"; do
        FULL_PATH="${BASE_URDF_PATH}/${URDF_FILE}"
        echo $FULL_PATH
        if [ -f "$FULL_PATH" ]; then
            origin_mass=$(./get_urdf_mass.sh "$FULL_PATH")
            origin_md5=$(md5sum "$FULL_PATH" | awk '{ print $1 }')
            echo "get total mass :$origin_mass"
            echo $origin_mass > ${MASS_FILE}
            echo $origin_mass > ${MASS_BAK}
            echo $origin_md5 > ${MD5_FILE}
            break  # 只需要从第一个存在的URDF文件获取质量即可
        fi
    done
    echo "Created new TotalMass file"
fi

# 检查备份文件
if [ ! -f ${MASS_BAK} ]; then
    # 没有备份文件，创建备份
    cp ${MASS_FILE} ${MASS_BAK}
    echo "Created backup mass file"
else
    # 比较当前文件和备份文件
    CURRENT_MASS=$(cat ${MASS_FILE})
    BACKUP_MASS=$(cat ${MASS_BAK})
    
    if [ "$CURRENT_MASS" != "$BACKUP_MASS" ]; then
        cp ${MASS_FILE} ${MASS_BAK}
        echo -e "\033[33m\nNote: Total Mass changed\033[0m" >&2
    fi
fi

# 获取总质量并修改所有URDF
TOTAL_MASS=$(cat ${MASS_FILE})
echo "由配置文件 ${MASS_FILE} 指定总质量为: $TOTAL_MASS kg" >&2
# 遍历URDF_FILES，并同步取出链接名称
for index in "${!URDF_FILES[@]}"; do
    URDF_FILE="${URDF_FILES[$index]}"
    link_name="${LINK_NAMES[$index]}"  # 获取对应的链接名称
    FULL_PATH="${BASE_URDF_PATH}/${URDF_FILE}"

    if [ -f "$FULL_PATH" ]; then
        # 调用修改质量的脚本，同时传入链接名称
        ./modify_torso_mass.sh "$FULL_PATH" "${TOTAL_MASS}" "$link_name"  # 传入link_name作为参数
        echo " Updated mass of ${FULL_PATH} to ${TOTAL_MASS}" >&2
    else
        echo "Warning: ${URDF_FILE} not found" >&2
    fi
done


# 修改mujoco xml文件
BASE_XML_PATH="${SRC_DIR}/models/biped_s${ROBOT_VERSION}/xml"
XML_FILE_PATH="${BASE_XML_PATH}/biped_s${ROBOT_VERSION}.xml"
if [ -f "$XML_FILE_PATH" ]; then
    # 调用修改质量的脚本，同时传入链接名称
    ./modify_torso_mass_xml.sh "$XML_FILE_PATH" "${TOTAL_MASS}"  # 传入link_name作为参数
    echo "Updated xml mass for ${XML_FILE_PATH}"
else
    echo "Warning: ${XML_FILE_PATH} not found" >&2
fi

# 修改完成后检查MD5
current_md5=""
for URDF_FILE in "${URDF_FILES[@]}"; do
    FULL_PATH="${BASE_URDF_PATH}/${URDF_FILE}"
    if [ -f "$FULL_PATH" ]; then
        current_md5=$(md5sum "$FULL_PATH" | awk '{ print $1 }')
        echo "URDF md5: $current_md5" >&2
        break
    fi
done

if [ -f ${MD5_FILE} ]; then
    saved_md5=$(cat ${MD5_FILE})
    if [ "$current_md5" != "$saved_md5" ]; then
        echo $current_md5 > ${MD5_FILE}
        echo -e "\033[33m\nNote: URDF file changed\033[0m" >&2
    fi
else
    echo $current_md5 > ${MD5_FILE}
    echo -e "\033[33m\nNote: MD5 file not found, created new one\033[0m" >&2
fi

# 在脚本结束前清理旧的缓存文件夹
cleanup_old_cppad_cache "${OCS2_DIR}"

chmod 0666 ${MASS_FILE} ${MASS_BAK} ${MD5_FILE}
