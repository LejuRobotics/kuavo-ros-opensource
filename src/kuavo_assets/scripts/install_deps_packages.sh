#!/bin/bash

SCRIPT_DIR=$(dirname $(readlink -f "$0"))
PACKAGE_FILE=$SCRIPT_DIR/packages_with_versions.txt
WS_PATH="$(realpath "${SCRIPT_DIR}/../../../")"

ECHO_WARN() {
    local msg="$1"
    echo -e "\033[33m[Depend] ${msg}\033[0m" >&2
}

ECHO_ERR() {
    local msg="$1"
    echo -e "\033[31m[Depend] ${msg}\033[0m" >&2
}

# Check Depend packages.
# return 0 - All Packages are installed.
# return 1 - Not all packages are installed.
function check_packages_installed() {
     while IFS= read -r pkg || [[ -n "$pkg" ]]; do
        pkg_name=$(echo "$pkg" | cut -d'=' -f1 | cut -d':' -f1)
        pkg_version=$(echo "$pkg" | cut -d'=' -f2)
        installed_version=$(dpkg-query -W -f='${Version}\n' "$pkg_name" 2>/dev/null)
        if [[ "$installed_version" != "$pkg_version" ]]; then
            return 1
        fi
    done < <(cat "$PACKAGE_FILE")

    return 0
}

function install_packages() {
    sudo apt-get update || echo -e "\033[31mapt-get update failed, continuing...\033[0m" >&2
    while IFS= read -r pkg || [[ -n "$pkg" ]]; do
        pkg_name=$(echo "$pkg" | cut -d'=' -f1 | cut -d':' -f1)
        pkg_version=$(echo "$pkg" | cut -d'=' -f2)
        installed_version=$(dpkg-query -W -f='${Version}\n' "$pkg_name" 2>/dev/null)
        if [[ "$installed_version" != "$pkg_version" ]]; then
            ECHO_WARN "Installing depend package: [$pkg]"
            sudo apt-get install -y "$pkg"
            if [ $? -ne 0 ]; then
                ECHO_ERR "Install depend package [$pkg] failed."
                exit 1
            fi
        fi
    done < <(cat $PACKAGE_FILE; echo)
}

function install_dexhand_deps() {
    local expected_md5s=(
        "e20954ddf56d1d4b28068cb15b042d5a"
        "8908f9c67c8eb4366bb382149da5e587"
    )
    local source_libs=(
        "$WS_PATH/installed/lib/libstark.so"
        "$WS_PATH/src/kuavo-ros-control-lejulib/hardware_plant/lib/dexhand_sdk/protobuf_sdk/dist/linux/shared/libstark.so"
    )

    # 检查 MD5 是否在允许的列表中
    function check_md5_in_list() {
        local md5_to_check="$1"
        local md5_list=("${expected_md5s[@]}")
        for valid_md5 in "${md5_list[@]}"; do
            if [[ "$md5_to_check" == "$valid_md5" ]]; then
                return 0
            fi
        done
        return 1
    }

    ECHO_WARN "Checking libstark.so installation..."

    # 检查系统目录和用户目录是否已有正确版本的文件
    local system_lib="/usr/local/lib/libstark.so"
    local user_lib="$HOME/.local/lib/libstark.so"
    local found_correct_lib=false

    # 先检查系统目录
    if [[ -f "$system_lib" ]]; then
        local current_md5=$(md5sum "$system_lib" | cut -d' ' -f1)
        if check_md5_in_list "$current_md5"; then
            ECHO_WARN "libstark.so is already installed with correct MD5 at $system_lib"
            found_correct_lib=true
        fi
    fi

    # 如果系统目录没有正确版本，检查用户目录
    if [ "$found_correct_lib" = false ] && [[ -f "$user_lib" ]]; then
        local current_md5=$(md5sum "$user_lib" | cut -d' ' -f1)
        if check_md5_in_list "$current_md5"; then
            ECHO_WARN "libstark.so is already installed with correct MD5 at $user_lib"
            found_correct_lib=true
        fi
    fi

    # 如果已找到正确版本，直接返回
    if [ "$found_correct_lib" = true ]; then
        return 0
    fi

    # 检查sudo权限，决定拷贝目标
    local has_sudo=false
    local target_lib
    if sudo -n true 2>/dev/null; then
        has_sudo=true
        target_lib="$system_lib"
        ECHO_WARN "检测到sudo权限，将安装到系统目录: $target_lib"
    else
        target_lib="$user_lib"
        # 创建用户本地库目录
        mkdir -p "$HOME/.local/lib"
        ECHO_WARN "未检测到sudo权限，将安装到用户目录: $target_lib"
    fi

    ECHO_WARN "libstark.so not found or MD5 mismatch, installing..."

    # 循环检查源文件是否存在并拷贝
    local found_source=""
    for source_lib in "${source_libs[@]}"; do
        if [[ -f "$source_lib" ]]; then
            ECHO_WARN "Found libstark.so at $source_lib, copying to $target_lib"
            if [ "$has_sudo" = true ]; then
                sudo cp "$source_lib" "$target_lib"
                sudo chmod 644 "$target_lib"
            else
                cp "$source_lib" "$target_lib"
                chmod 644 "$target_lib"
            fi
            found_source="$source_lib"
            ECHO_WARN "libstark.so installed successfully."
            break
        fi
    done

    # 如果没有找到任何源文件
    if [[ -z "$found_source" ]]; then
        ECHO_ERR "libstark.so not found in any of the following locations:"
        for source_lib in "${source_libs[@]}"; do
            ECHO_ERR "  $source_lib"
        done
        return 1
    fi

    # 验证安装后的MD5
    if [[ -f "$target_lib" ]]; then
        local final_md5=$(md5sum "$target_lib" | cut -d' ' -f1)
        if check_md5_in_list "$final_md5"; then
            ECHO_WARN "libstark.so installation verified with correct MD5."
            ECHO_WARN "已安装到: $target_lib"
            return 0
        else
            ECHO_ERR "libstark.so installed but MD5 verification failed."
            ECHO_ERR "Expected one of: ${expected_md5s[*]}"
            ECHO_ERR "Actual: $final_md5"
            return 1
        fi
    else
        ECHO_ERR "libstark.so installation failed - file not found after copy."
        return 1
    fi
}


function install_openvino() {
    local install_script_path="$WS_PATH/scripts/install_openvino.sh"
    local packages=(
        "openvino"
        "gnupg"
    )

    # Check if packages are already installed
    local all_installed=true
    for pkg in "${packages[@]}"; do
        if ! dpkg-query -W -f='${Status}' "$pkg" 2>/dev/null | grep -q "install ok installed"; then
            all_installed=false
            break
        fi
    done

    # If all packages are installed, return early
    if [ "$all_installed" = true ]; then
        ECHO_WARN "OpenVINO and GnuPG are already installed."
        return 0
    fi

    # Check for sudo permissions
    if sudo -n true 2>/dev/null; then
        ECHO_WARN "Detected sudo permissions, executing OpenVINO installation script..."
        bash "$install_script_path"
        if [ $? -eq 0 ]; then
            ECHO_WARN "OpenVINO installation completed successfully."
        else
            ECHO_ERR "OpenVINO installation failed."
            return 1
        fi
    else
        echo -e "\033[1;31m======================================= 警告 =======================================" >&2
        echo -e "\033[1;31m    检测到没有 sudo 权限！" >&2
        echo -e "\033[1;31m    OpenVINO 和 GnuPG 安装需要 sudo 权限。" >&2
        echo -e "\033[1;31m    请使用 sudo 权限运行此脚本或手动安装 OpenVINO。" >&2
        echo -e "\033[1;31m    可以手动执行: bash $install_script_path" >&2
        echo -e "\033[1;31m====================================================================================" >&2
        echo -e "\033[0m" >&2
        return 1
    fi
}

### Start
ECHO_WARN "Install dexhand_sdk ..."
install_dexhand_deps
if [ $? -eq 0 ]; then
    ECHO_WARN "DexHand Protobuf SDK 安装成功"
else
    ECHO_WARN "DexHand Protobuf SDK 安装失败"
fi

ECHO_WARN "Install openvino ..."
install_openvino

ECHO_WARN "Check and Install depend packages ..."

bash $SCRIPT_DIR/install_robot_localization_env.sh >&2


ECHO_WARN "Check and Install libudev-dev ..."
if ! dpkg -l | grep -q "libudev-dev"; then
    ECHO_WARN "Installing libudev-dev ..."
    sudo apt-get install libudev-dev -y
else
    ECHO_WARN "libudev-dev is already installed."
fi


check_packages_installed "bash"
result=$?

if [ $result -eq 0 ]; then
    ECHO_WARN "All Packages are installed."
    exit 0
else
    install_packages
fi
exit 0
