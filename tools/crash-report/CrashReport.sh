#!/bin/bash
# Lejurobot is licensed under the BSD License. 
# This means that you can use, modify, and distribute the software freely, 
# as long as you include the original copyright notice and license in any 
# copies or substantial portions of the software. 
# The software is provided "as is", without warranty of any kind.
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
PROJECT_DIR="$(readlink -f "$SCRIPT_DIR/../..")"
ARICHE_DIR="/tmp/kuavo-crash"
TIMESTAMP=""
LAUNCH_ID=""

install_deps_packages() {
    # Check and install ROS debug packages
    packages=(
        "tree"
        "pv"
        "pigz"
        "zstd"
        "curl"
        "jq"
    )

    # Array to store packages that need to be installed
    packages_to_install=()

    for pkg in "${packages[@]}"; do
        if ! dpkg -l "$pkg" 2>/dev/null | grep -q "^ii"; then
            packages_to_install+=("$pkg")
        fi
    done

    if [ ${#packages_to_install[@]} -gt 0 ]; then
        echo -e "\033[32m提示:本工具需要安装依赖，请允许我们为您安装以下依赖包:\033[0m"
        for pkg in "${packages_to_install[@]}"; do
            echo -e "\033[32m  - $pkg\033[0m"
        done
        read -p "是否继续安装? [y/N] " response
        if [[ $response =~ ^[Yy]$ ]]; then
            sudo apt-get install -y "${packages_to_install[@]}"
        else
            echo "Error: 用户取消了安装依赖, 无法继续..." >&2
            exit 1
        fi
    fi
}

choose_compressor() {
    case "${KUAVO_CRASH_COMPRESSOR:-auto}" in
        zstd)
            command -v zstd >/dev/null || exit_with_fail "KUAVO_CRASH_COMPRESSOR=zstd but zstd is not installed"
            echo "zstd"
            ;;
        gzip)
            echo "gzip"
            ;;
        auto|*)
            if command -v zstd >/dev/null; then
                echo "zstd"
            else
                echo "gzip"
            fi
            ;;
    esac
}

extract_launch_info() {
    local info_file="$1"
    # Check if the log file exists
    if [ ! -f "$log_file" ]; then
        echo "Error: Log都得  file does not exist: $log_file" >&2
        return 1
    fi
    
    # Extract launch ID from the log file
    LAUNCH_ID=$(grep -o "PPID:[[:space:]]*[[:alnum:]_-]\+" "$log_file" | head -1 | awk '{print $2}')
    
    if [ -z "$LAUNCH_ID" ]; then
        echo "Warning: Could not extract launch ID from log file" >&2
        return 1
    fi
    
    return 0
}

archive_files() {
    local launch_id="$1"
    local timestamp="$2"
    local archive_dir="$3"
    local zip_file="$4"

    # Display a message indicating the archiving process is starting
    echo -e "\033[32m📦 开始归档文件...\033[0m"
    
    # Check if the zip file already exists and is valid
    if [ -f "$zip_file" ]; then
        echo -e "\033[32m📦 检测到已存在的压缩包: $zip_file\033[0m"
        # Verify the archive integrity
        if $VERIFY_CMD "$zip_file" 2>/dev/null; then
            echo -e "\033[32m✅ 压缩包完好，跳过归档步骤\033[0m"
            return 0
        else
            echo -e "\033[33m 现有压缩包损坏，将重新创建\033[0m"
        fi
    fi
    
    # create archive directory
    rm -rf "$archive_dir"
    mkdir -p "$archive_dir"
    mkdir -p "$archive_dir/stdout"
    mkdir -p "$archive_dir/coredumps"

    # copy info.txt 
    info_file="$HOME/.ros/kuavo_launch/${launch_id}/info.txt"
    if [ -f "$info_file" ]; then
        cp "$info_file" "$archive_dir"
        if [ -d "$PROJECT_DIR/.git" ]; then
            current_commit=$(git rev-parse HEAD)
            echo "current_commit: $current_commit" >> "$archive_dir/info.txt"
        else
            echo "current_commit: not found" >> "$archive_dir/info.txt"
        fi
    else
        exit_with_fail "没有找到info.txt文件: $info_file"
    fi

    # copy stdout.log
    src_dir="$HOME/.ros/stdout/${timestamp}"
    if [ -d "$src_dir" ] && [ "$(ls -A "$src_dir" 2>/dev/null)" ]; then
        cp -r "$src_dir"/* "$archive_dir/stdout"
    else
        exit_with_fail "没有找到日志文件夹: $src_dir"
    fi

    # copy coredumps
    src_dir="$HOME/.ros/coredumps/${launch_id}"
    if [ -d "$src_dir" ] && [ "$(ls -A "$src_dir" 2>/dev/null)" ]; then
        cp -r "$src_dir"/* "$archive_dir/coredumps"
    else
        exit_with_fail "coredump 目录下没有文件: $src_dir"
        exit 1
    fi

    # Copy installed and devel directories to the archive directory
    installed_dir="$PROJECT_DIR/installed"
    devel_dir="$PROJECT_DIR/devel"
    if [ -d "$installed_dir" ]; then
        cp -r "$installed_dir" "$archive_dir"
    else
        echo "没有找到已安装的目录: $installed_dir"
    fi
    if [ -d "$devel_dir" ]; then
        cp -rL "$devel_dir" "$archive_dir"
        private_dir="$archive_dir/devel/.private"
        if [ -d "$private_dir" ]; then
            rm -rf "$private_dir"
        fi
    else
        echo "没有找到开发目录: $devel_dir"
    fi

    # remove unused files
    # Remove coredump files that don't start with core.humanoid or core.nodelet
    # Define whitelist for coredump files
    whitelist_patterns=("core.humanoid*" "core.nodelet*" "core.mobile_manipula*")
    
    # Build find command with whitelist patterns
    find_cmd="find \"$archive_dir/coredumps\" -type f"
    for pattern in "${whitelist_patterns[@]}"; do
        find_cmd="$find_cmd -not -name \"$pattern\""
    done
    find_cmd="$find_cmd -exec rm -f {} \\;"
    
    # Execute the find command
    eval "$find_cmd"

    # tar archive
    rm -rf "$zip_file"
    echo "coredumps:"
    tree --noreport "$archive_dir/coredumps"
    echo -e "\033[32m📦 正在压缩文件: $zip_file\033[0m"
    if tar -c -C "$(dirname "$archive_dir")" "$(basename "$archive_dir")" | pv | $COMPRESS_CMD > "$zip_file"; then
        echo "Successfully created archive: $zip_file ($(du -h "$zip_file" | cut -f1))"
        echo -e "\033[32m📦 压缩成功，压缩文件存放路径: $zip_file\033[0m" 
        echo -e "\033[32mTips: 如果文件上传失败，您可以考虑手动拷贝该文件给乐聚技术支持人员，谢谢!\033[0m"
    else
        echo "Failed to create archive" >&2
        exit_with_fail "压缩归档文件失败"
    fi

}

upload_file() {
    local zip_file="$1"

    # Display upload message with emoji
    echo -e "\033[32m📤 正在上传文件: $zip_file\033[0m"

    # Check if the file exists
    if [ ! -f "$zip_file" ]; then
        exit_with_fail "归档文件不存在: $zip_file"
    fi
    
    # Check if the file is readable
    if [ ! -r "$zip_file" ]; then
        exit_with_fail "无法读取归档文件: $zip_file"
    fi
    
    # Check if the file is not empty
    if [ ! -s "$zip_file" ]; then
        exit_with_fail "归档文件为空: $zip_file"
    fi
    
    # 使用 curl 的 -# 参数显示进度条
    local response=$(curl -# -X POST https://crash.lejurobot.com/files/upload/error \
    -H "Content-Type: multipart/form-data" \
    -F "file=@$zip_file")

    # Parse response status
    local status=$(echo "$response" | jq -r '.status')
    if [ "$status" != "success" ]; then
        exit_with_fail "日志文件上传失败: $response"
    fi
}

exit_with_fail() {
    if [ -n "$1" ]; then
        echo -e "\033[31mError:$1\033[0m"
    fi
    exit 1
}

exit_with_success() {
    local zip_file="$1"
    local zip_basename=$(basename "$zip_file")
    echo ""
    echo -e "🙏 感谢您的配合，📦 日志文件已上传成功，请将如下信息复制给乐聚技术支持人员:"
    echo ""
    echo -e "\033[32m用户日志已上传, 详情请查看: $zip_basename\033[0m"
    echo ""
    exit 0
}

#@@@ MAIN @@@
if [ -z "$1" ]; then
  echo "Usage: $0 <stdout_log_dir>"
  echo "Example: $0 ~/.ros/stdout/2025-04-10_14-30-45/"
  exit 1
fi

# 检查stdout.log文件是否存在
log_file="$1/stdout.log"
if [ ! -f "$log_file" ]; then
    echo -e "\033[31mError: 日志文件不存在: $log_file\033[0m"
    exit 1
fi

# 提取时间戳
if [[ "$1" =~ .*/([0-9]{4}-[0-9]{2}-[0-9]{2}_[0-9]{2}-[0-9]{2}-[0-9]{2})/?$ ]]; then
    TIMESTAMP="${BASH_REMATCH[1]}"
else
    echo "Error: Could not extract timestamp from input path: $1"
    exit 1
fi

# 提取launch ID
LAUNCH_ID=$(head -3 "${log_file}" | awk -F'[^0-9]+' '/PPID:/{print $2; exit}')
if [ -z "$LAUNCH_ID" ]; then
    echo "Error: Could not extract launch ID from log file"
    exit 1
fi

info_file="$HOME/.ros/kuavo_launch/${LAUNCH_ID}/info.txt"
GIT_BRANCH=$(awk -F': +' '$1 == "branch" {print $2}' "$info_file")
GIT_SYNC_COMMIT=$(awk -F': +' '$1 == "sync_commit" {print $2}' "$info_file")
GIT_COMMIT=$(awk -F': +' '$1 == "commit" {print $2}' "$info_file")
if [ -z "$GIT_BRANCH" ] || [ -z "$GIT_SYNC_COMMIT" ]; then
    echo "Error: 未找到相关的的分支和commit信息, 无法创建归档文件"
    exit 1
fi

# 将 GIT_BRANCH 中的 '/' 替换为 '@' 以便在文件路径中使用
GIT_ORI_BRANCH="$GIT_BRANCH"
GIT_BRANCH="${GIT_ORI_BRANCH//\//@}"

echo -e "\033[1;36m"
echo -e "********************************************************************************"
echo -e "*                                                                              *"
echo -e "*                        Kuavo Crash Report Tool                               *"
echo -e "*                                                                              *"
echo -e "********************************************************************************"
echo -e "\033[0m"
echo -e "\033[32mLAUNCH_ID: $LAUNCH_ID\033[0m"
echo -e "\033[32mTIMESTAMP: $TIMESTAMP\033[0m"
echo -e "\033[32mGIT_BRANCH: $GIT_ORI_BRANCH\033[0m"
echo -e "\033[32mGIT_COMMIT: $GIT_COMMIT\033[0m"
echo -e "\033[32mGIT_SYNC_COMMIT: $GIT_SYNC_COMMIT\033[0m"

echo "-------------------------"
mkdir -p "$ARICHE_DIR/${TIMESTAMP}"
install_deps_packages

# Pick compressor: prefer zstd; honor KUAVO_CRASH_COMPRESSOR=gzip/zstd; fall back to gzip when zstd is unavailable.
COMPRESSOR=$(choose_compressor)
case "$COMPRESSOR" in
    zstd) TAR_EXT="tar.zst"; COMPRESS_CMD="zstd -19 -T0 -q"; VERIFY_CMD="zstd -t" ;;
    gzip) TAR_EXT="tar.gz";  COMPRESS_CMD="pigz -9";        VERIFY_CMD="pigz -t" ;;
esac

TAR_GZ_BASENAME="kuavo-crash_${TIMESTAMP}_${GIT_SYNC_COMMIT}"
TAR_FILE="/tmp/kuavo-crash/${TAR_GZ_BASENAME}.${TAR_EXT}"
archive_files "${LAUNCH_ID}" "${TIMESTAMP}" "$ARICHE_DIR/${TAR_GZ_BASENAME}" "$TAR_FILE"
upload_file "$TAR_FILE"
exit_with_success "$TAR_FILE"