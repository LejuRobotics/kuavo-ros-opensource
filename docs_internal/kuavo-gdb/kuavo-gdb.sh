#!/bin/bash
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
PROJECT_DIR="$(readlink -f "$SCRIPT_DIR/../..")"
DEBUG_DIR="$(pwd)/kuavo-gdb-debug"
KUAVO_SYMBOL_DIR="$DEBUG_DIR/symbols"
CORE_FILE=""
EXE_FILE=""
DOWNLOAD_SYMBOL_URL=""
DOWNLOAD_CRASH_URL=""

usage() {
    echo "Usage: $0 [options]"
    echo "Options:"
    echo "  -c, --crash_zip <kuavo-crash filename>  Specify the crash zip file to analyze"
    echo "  -d, --download-debug-symbol <kuavo-dbgsym filename> Download debug symbols"
    echo "  -h, --help                             Display this help message"
    echo ""
    echo "Example:"
    echo "  $0 -c kuavo-crash_2025-04-09-15-48_5f7c74016.tar.gz"
    echo "  $0 -c kuavo-crash_2025-04-09-15-48_5f7c74016.tar.zst"
    echo "  $0 -d kuavo-dbgsym_b315e2605.tar.gz"
}

install_deps_packages() {
    # Check and install ROS debug packages
    packages=(
        "ros-noetic-roscpp-dbgsym"
        "ros-noetic-bondcpp-dbgsym"
        "ros-noetic-cpp-common-dbgsym" 
        "ros-noetic-nodelet-dbgsym"
        "pv"
        "pigz"
        "zstd"
        "curl"
        "gdb"
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
        echo -e "\033[32m提示: GDB 调试需要 ROS 相关的符号信息或依赖包，请允许我们为您安装以下依赖包:\033[0m"
        for pkg in "${packages_to_install[@]}"; do
            echo -e "\033[32m  - $pkg\033[0m"
        done
        read -p "是否继续安装? [y/N] " response
        if [[ $response =~ ^[Yy]$ ]]; then
            sudo apt-get install -y "${packages_to_install[@]}"
        fi
    fi
}

check_branch_commit_match() {
    local repo="$1"
    local branch="$2"
    local crash_commit="$3"      # kuavo-crash commit
    local sync_commit="$4" # sync kuavo-ros-control commit 

    local local_repo=$(git config --get remote.origin.url 2>/dev/null)
    local local_commit=$(git rev-parse HEAD 2>/dev/null)
    local local_branch=$(git rev-parse --abbrev-ref HEAD 2>/dev/null)

    echo -e "-----------------------------------------------"
    echo -e "\033[33mCrash: \033[0m"
    echo -e "\033[33m -branch: $branch\033[0m"
    echo -e "\033[33m -crash commit: $crash_commit\033[0m"
    echo -e "\033[33m -sync commit: $sync_commit\033[0m"
    echo -e "\033[33m -repo: $repo\033[0m"
    echo -e "\033[34mLocal:\033[0m"
    echo -e "\033[34m -branch: $local_branch\033[0m"
    echo -e "\033[34m -commit: $local_commit\033[0m"
    echo -e "\033[34m -repo: $local_repo\033[0m"
    echo -e "-----------------------------------------------"

    git_match=0
    # Check if commits match
    if [[ "$local_commit" == "$sync_commit" ]]; then
        echo -e "\033[32mCommit match: Yes\033[0m"
    else
        git_match=1
        echo -e "\033[33mCommit match: No\033[0m"
    fi

    if [[ $git_match -eq 1 ]]; then
        local co_branch="$branch"
        if [[ "$co_branch" == "HEAD" || "$co_branch" == "unknown" ]]; then
            co_branch="dev"
        fi
        echo -e "\n-----------------------------------------------"
        echo -e "\033[31m检测到当前commit 与 kuavo-crash 中的信息不一致\033[0m"
        echo -e "\033[31m请切换到对应的分支和commit后重新执行:\033[0m"
        echo -e "\033[31mgit fetch\033[0m"
        echo -e "\033[31mgit checkout $branch\033[0m"
        echo -e "\033[31mgit pull\033[0m"
        echo -e "\033[31mgit checkout $sync_commit\033[0m"
        exit_with_fail
    fi
}

get_kuavo_crash_info() {
    local info_txt="$1"
    local -n __remote_ref="$2"  # 使用 nameref 传递变量引用
    local -n __branch_ref="$3"  # 使用 nameref 传递变量引用
    local -n __crash_commit_ref="$4"
    local -n __sync_commit_ref="$5"

    
    # 检查文件存在性
    if [ ! -f "$info_txt" ]; then
        exit_with_fail "Info file not found: $info_txt"
    fi
    
    printf "\033[32mReading crash information from %s\033[0m\n" "$info_txt"
    
    # 提取信息并检查有效性
    __remote_ref=$(awk '/remote:/ {print $2; exit}' "$info_txt")
    __branch_ref=$(awk '/branch:/ {print $2; exit}' "$info_txt")
    __crash_commit_ref=$(awk '/crash_commit:/ {print $2; exit}' "$info_txt")
    __sync_commit_ref=$(awk '/sync_commit:/ {print $2; exit}' "$info_txt")
    
    if [ -z "$__remote_ref" ] || [ -z "$__branch_ref" ] || [ -z "$__crash_commit_ref" ] || [ -z "$__sync_commit_ref" ]; then
        exit_with_fail "Invalid info.txt format: missing 'remote:' missing 'branch:' or 'commit:' or 'sync_commit:' in $info_txt"
    fi
}

get_download_url() {
    symbol_zip="$1" # "kuavo-dbgsym_${git_commit}.tar.gz"
    crash_zip="$2" # kuavo-crash_2025-04-11_15-03-00_f1e881a74.tar.gz

    # Check if VALID_CODE is already set
    echo -e "\033[32m请访问企业通知群, 获取下载验证码\033[0m"
    local code=""
    while [ -z "$code" ] || ! [[ "$code" =~ ^[a-z0-9]{10}$ ]]; do
        read -p "请输入验证码: " code
        if [ -z "$code" ]; then
            echo -e "\033[31m错误: 验证码不能为空\033[0m"
        elif ! [[ "$code" =~ ^[a-z0-9]{10}$ ]]; then
            echo -e "\033[31m错误: 验证码格式不正确，应为10位字母数字组合\033[0m"
            code=""
        fi
    done
    VALID_CODE="${code}"

    local response=$(curl -sS -X GET "https://crash.lejurobot.com/files/download/compiled/$symbol_zip?captcha=${VALID_CODE}")
    local download_url=$(echo "$response" | jq -r '.url')
    local statusCode=$(echo "$response" | jq -r '.statusCode')
    local message=$(echo "$response" | jq -r '.message')
    echo "response:$response"
    if [ -z "$download_url" ] || [ "$download_url" = "null" ]; then
        echo -e "\033[31mServer message: $message\033[0m"
        exit_with_fail "Failed to get  symbol download URL from server"
    fi
    DOWNLOAD_SYMBOL_URL="$download_url"

    if [ -n "$crash_zip" ]; then
        local response=$(curl -sS -X GET "https://crash.lejurobot.com/files/download/error/$crash_zip?captcha=${VALID_CODE}")
        local download_url=$(echo "$response" | jq -r '.url')
        local statusCode=$(echo "$response" | jq -r '.statusCode')
        local message=$(echo "$response" | jq -r '.message')
        echo "response:$response"
        if [ -z "$download_url" ] || [ "$download_url" = "null" ]; then
            echo -e "\033[31mServer message: $message\033[0m"
            exit_with_fail "Failed to get kuavo-crash download URL from server"
        fi
        DOWNLOAD_CRASH_URL="$download_url"
    fi

    return 1
}

check_exist_kuavo_crash() {
    local coredump_zip="$1"
    local output_dir="$2"

    # Strip .tar.gz / .tar.zst suffix to get archive basename
    local zip_basename=$(strip_archive_suffix "$coredump_zip")
    local extracted_dir="$output_dir/${zip_basename}"
    
    if [ -d "$extracted_dir" ]; then
        if [ -d "$extracted_dir/coredumps" ] && [ "$(ls -A "$extracted_dir/coredumps")" ]; then
            echo -e "\033[32mCore dump files already exist in $extracted_dir/coredumps\033[0m"
            return 0
        fi
    fi

    return 1
}

download_kuavo_crash() {
    # kuavo-crash_2025-04-09-15-48_5f7c74016.tar.gz
    local crash_file_name="$1"
    local output_dir="$2"
    
    if check_exist_kuavo_crash "$crash_file_name" "$output_dir"; then
        echo -e "\033[32mUsing existing crash files\033[0m"
        return 0
    fi

    echo "Downloading crash file: $crash_file_name"
    local tmp_file="/tmp/$crash_file_name"
    rm -rf "$tmp_file"
    curl -X GET "${DOWNLOAD_CRASH_URL}" --output "${tmp_file}"
    if [ $? -ne 0 ]; then
        exit_with_fail "Failed to download kuavo-crash file"
    fi
    echo "Successfully downloaded crash file to $tmp_file"

    # Check if directory already exists
    local zip_basename=$(strip_archive_suffix "$crash_file_name")
    local extracted_dir="$output_dir/${zip_basename}"
    echo "Extracting archive to ${extracted_dir}..."
    extract_archive "$tmp_file" "$output_dir"
    if [ $? -ne 0 ]; then
        exit_with_fail "Failed to extract crash archive"
    fi
}

check_exist_debug_symbols() {
    local git_commit="$1"
    local symbol_dir="$2"

    commit_symbol_dir="$symbol_dir/$git_commit"
    if [ -d "$commit_symbol_dir" ]; then
        # Check if .build-id directory and info.txt exist
        if [ -d "$commit_symbol_dir/debug/.build-id" ] && [ -f "$commit_symbol_dir/info.txt" ]; then
            echo -e "\033[32mDebug symbols already exist for commit $git_commit\033[0m"
            return 0
        fi
    fi

    return 1
}

download_debug_symbols() {
    local git_commit="$1"
    local symbol_dir="$2"

    # check exist 
    if check_exist_debug_symbols "$git_commit" "$symbol_dir"; then
        return 0
    fi

    # download debug symbols from server
    local tmp_file="/tmp/kuavo-dbgsym_${GIT_COMMIT}.tar.gz"
    rm -rf "$tmp_file"
    echo "Downloading debug symbols..."
    curl -X GET "${DOWNLOAD_SYMBOL_URL}" --output "${tmp_file}"
    if [ $? -ne 0 ]; then
        exit_with_fail "Failed to download debug symbols"
    fi

    echo "Extracting debug symbols..."
    extract_archive "$tmp_file" "$symbol_dir"
    if [ $? -ne 0 ]; then
        exit_with_fail "Failed to extract debug symbols"
    fi
}

select_coredump() {
    local coredump_dir="$1"
    local selected_coredump=""

    # Check if coredump directory exists
    if [ ! -d "$coredump_dir" ]; then
        exit_with_fail "Core dump directory does not exist: $coredump_dir"
    fi

    # Find all coredump files
    local coredump_files=()
    while IFS= read -r -d $'\0' file; do
        coredump_files+=("$file")
    done < <(find "$coredump_dir" -name "core.*" -type f -print0)

    # Check if any coredump files were found
    if [ ${#coredump_files[@]} -eq 0 ]; then
        exit_with_fail "No coredump files found in $coredump_dir"
    fi

    # If only one coredump file exists, select it automatically
    if [ ${#coredump_files[@]} -eq 1 ]; then
        echo -e "\033[32mFound single coredump file: ${coredump_files[0]}\033[0m"
        selected_coredump="${coredump_files[0]}"
    else
        # Display menu for selection
        echo -e "\033[32mMultiple coredump files found. Please select one:\033[0m"
        for i in "${!coredump_files[@]}"; do
            echo "[$((i+1))] $(basename "${coredump_files[$i]}")"
        done

        # Get user selection
        local selection
        while true; do
            read -p "Enter selection number [1-${#coredump_files[@]}]: " selection
            if [[ "$selection" =~ ^[0-9]+$ ]] && [ "$selection" -ge 1 ] && [ "$selection" -le "${#coredump_files[@]}" ]; then
                selected_coredump="${coredump_files[$((selection-1))]}"
                break
            else
                echo -e "\033[31mInvalid selection. Please try again.\033[0m"
            fi
        done
    fi

    echo -e "\033[32mSelected coredump file: $(basename "$selected_coredump")\033[0m"
    CORE_FILE="$selected_coredump"
}

search_executable() {
    local search_dir="$1"
    local exe_basename="$2"
    local -n __exec_file_ref="$3"

    # Find all executables matching basename
    matching_files=()
    while IFS= read -r -d $'\0' file; do
        if [[ -x "$file" && -f "$file" ]]; then
            matching_files+=("$file")
        fi
    done < <(find "$search_dir" -type f -name "*$exe_basename*" -print0)

    # Find longest matching executable name
    longest_match=""
    max_length=0
    for file in "${matching_files[@]}"; do
        basename=$(basename "$file")
        if [[ ${#basename} -gt $max_length ]]; then
            max_length=${#basename}
            longest_match="$file"
        fi
    done

    if [ -n "$longest_match" ]; then
        __exec_file_ref="$longest_match"
    fi
}

gdb_debug() {
    local kuavo_crash_dir="$1"
    local coredump="$2"
    local symbols_dir="$3"

    # Extract executable name from core dump filename (format: core.%e.%p.%t)
    local exe_basename=$(echo "$(basename "$coredump")" | cut -d'.' -f2)
    declare exe_file=""

    # 查找执行程序路径
    if [[ "$exe_basename" == *"nodelet"* ]]; then
        exe_file="/opt/ros/noetic/lib/nodelet/nodelet"
    else
        # Find executable in installed directory if it exists
        local installed_dir="$(readlink -f "$kuavo_crash_dir/installed")"
        local devel_dir="$(readlink -f "$kuavo_crash_dir/devel")"
        # 优先从 devel_dir 目录中查找
        if [ -d "$devel_dir" ]; then
            search_executable "$devel_dir" "$exe_basename" exe_file
        fi
        # 优先从 installed_dir 目录中查找
        if [ -z "$exe_file" ]; then
            if [ -d "$installed_dir" ]; then
                search_executable "$installed_dir" "$exe_basename" exe_file
            fi
        fi
    fi

    if [ ! -f "$exe_file" ]; then
        exit_with_fail "Executable file not found: $exe_file"
    fi

    local coredump_dir=$(dirname "$CORE_FILE")
    local source_repo_path="" # kuavo-ros-control
    # Extract source repository path from the core dump file
    source_repo_path=$(strings $CORE_FILE | grep "kuavo-ros-control/devel/lib" | head -1)
    if [ -z "$source_repo_path" ]; then
        exit_with_fail "Failed to extract source repository path from the core dump file"
    else
        # Remove the trailing "devel/lib/" from source_repo_path
        source_repo_path=$(echo "$source_repo_path" | sed -E 's|/(devel/lib/?.*$)||')
    fi
    
    local opensource_source_repo_path="" # kuavo-ros-opensource or craic_code_repo
    opensource_source_repo_path=$(strings $CORE_FILE | grep "kuavo-ros-opensource/devel/lib\|craic_code_repo/devel/lib" | head -1)
    if [ -z "$source_repo_path" ]; then
        exit_with_fail "Failed to extract source repository path from the core dump file"
    else
        # Remove the trailing "devel/lib/" from opensource_source_repo_path
        opensource_source_repo_path=$(echo "$opensource_source_repo_path" | sed -E 's|/(devel/lib/?.*$)||')
    fi

    # 计算相对路径
    local exe_rel_path=$(realpath --relative-to="$PROJECT_DIR" "$exe_file" 2>/dev/null || echo "$exe_file")
    local symbols_rel_path=$(realpath --relative-to="$PROJECT_DIR" "$symbols_dir" 2>/dev/null || echo "$symbols_dir")
    local coredump_rel_path=$(realpath --relative-to="$PROJECT_DIR" "$coredump_dir" 2>/dev/null || echo "$coredump_dir")
    echo -e "\033[32m- Executable file: $exe_rel_path\033[0m"
    echo -e "\033[32m- Debug symbols directory: $symbols_rel_path\033[0m"
    echo -e "\033[32m- Core file directory: $coredump_rel_path\033[0m"
    echo -e "\033[32m- Source repository path: $source_repo_path\033[0m"
    echo -e "\033[32m- Opensource Source repository path: $opensource_source_repo_path\033[0m"
    echo -e "\033[32m- Source project path: $PROJECT_DIR\033[0m"

    # Define GDB commands
    GDB_COMMANDS=(
        "set debug-file-directory ${symbols_dir}:/usr/lib/debug"
        "set solib-search-path ${coredump_dir}/../installed/lib:${coredump_dir}/../devel/lib"
        "set substitute-path ${source_repo_path} ${PROJECT_DIR}"
        "set substitute-path ${opensource_source_repo_path} ${PROJECT_DIR}"
        "file ${exe_file}"
        "core-file ${CORE_FILE}"
    )
    
    # Display GDB command
    echo -e "\033[33m--------------------------------------------------------------------------------\033[0m"
    echo -e "\033[33mgdb\033[0m"
    for cmd in "${GDB_COMMANDS[@]}"; do
        echo -e "\033[33m-ex \"$cmd\"${cmd//?/ }\033[0m"
    done
    echo -e "\033[33m--------------------------------------------------------------------------------\033[0m"
    # Execute GDB with commands
    gdb_cmd="gdb"
    for cmd in "${GDB_COMMANDS[@]}"; do
        gdb_cmd+=" -ex \"$cmd\""
    done
    eval $gdb_cmd
}

detect_compress_format() {
    local magic=$(od -An -tx1 -N4 "$1" 2>/dev/null | tr -d ' \n')
    case "$magic" in
        1f8b*)    echo "gzip"   ;;
        28b52ffd) echo "zstd"   ;;
        *)        echo "unknown";;
    esac
}

extract_archive() {
    local archive="$1"
    local outdir="$2"
    case "$(detect_compress_format "$archive")" in
        gzip) pv "$archive" | pigz -dc | tar xf - -C "$outdir" ;;
        zstd) pv "$archive" | zstd -dc | tar xf - -C "$outdir" ;;
        *)    exit_with_fail "Unknown archive format: $archive (magic byte not gzip/zstd)" ;;
    esac
}

strip_archive_suffix() {
    basename "$1" | sed -E 's/\.tar\.(gz|zst)$//'
}

exit_with_fail() {
    if [ -n "$1" ]; then
        echo -e "\033[31mError: $1\033[0m"
    fi
    exit 1
}

#@@@ MAIN @@@
analyze_kuavo_crash() {
    local CRASH_ZIP="$1"
    # Check required arguments
    if [ -z "$CRASH_ZIP" ]; then
        echo -e "\033[31mError: --crash_zip argument is required\033[0m"
        usage
        exit 1
    fi

    # Validate crash zip filename format (accept .tar.gz or .tar.zst)
    if ! [[ "$CRASH_ZIP" =~ kuavo-crash_[0-9]{4}-[0-9]{2}-[0-9]{2}_[0-9]{2}-[0-9]{2}-[0-9]{2}_.*\.tar\.(gz|zst)$ ]]; then
        echo -e "\033[31mError: Invalid crash zip filename format\033[0m"
        echo -e "\033[31mExample: kuavo-crash_2025-04-11_15-03-00_f1e881a74.tar.gz (or .tar.zst)\033[0m"
        exit 1
    fi

    # Extract git info from $CRASH_ZIP: kuavo-crash_<TS>_<COMMIT>.tar.{gz,zst}
    CRASH_ZIP_NAME=$(strip_archive_suffix "$CRASH_ZIP")
    GIT_COMMIT=$(echo "$CRASH_ZIP_NAME" | awk -F'_' '{print $4}')
    if [ -z "$GIT_COMMIT" ]; then
        exit_with_fail "Invalid crash zip filename format, e.g. kuavo-crash_2025-04-09-15-48_5f7c74016.tar.gz"
    fi
    echo -e "\033[32mGIT_COMMIT: $GIT_COMMIT\033[0m"
    mkdir -p "$DEBUG_DIR"
    mkdir -p "$KUAVO_SYMBOL_DIR"
    echo "*" > "$DEBUG_DIR/.gitignore"

    install_deps_packages                           # install deps
    # Check if we need to download files
    if ! check_exist_kuavo_crash "$CRASH_ZIP" "$DEBUG_DIR" || 
       ! check_exist_debug_symbols "$GIT_COMMIT" "$KUAVO_SYMBOL_DIR"; then
        get_download_url "kuavo-dbgsym_${GIT_COMMIT}.tar.gz" "$CRASH_ZIP" 
    fi
    download_kuavo_crash  "$CRASH_ZIP" "$DEBUG_DIR"   # download kuavo crash
    download_debug_symbols "$GIT_COMMIT" "$KUAVO_SYMBOL_DIR" # download debug symbols
    declare crash_remote crash_branch crash_commit sync_commit
    get_kuavo_crash_info "$DEBUG_DIR/$CRASH_ZIP_NAME/info.txt" crash_remote crash_branch crash_commit sync_commit
    check_branch_commit_match "$crash_remote" "$crash_branch" "$crash_commit" "$sync_commit"
    select_coredump  "$DEBUG_DIR/$CRASH_ZIP_NAME/coredumps/" # select coredump file
    gdb_debug "$DEBUG_DIR/$CRASH_ZIP_NAME" "$CORE_FILE" "$KUAVO_SYMBOL_DIR/${GIT_COMMIT}/debug" # gdb debug
}

download_kuavo_symbols() {
    local KUAVO_SYMBOL_ZIP="$1"

    # Check required arguments
    if [ -z "$KUAVO_SYMBOL_ZIP" ]; then
        echo -e "\033[31mError: --download-debug-symbol argument is required\033[0m"
        usage
        exit 1
    fi

    # Validate kuavo-dbgsym zip filename format (accept .tar.gz or .tar.zst)
    # kuavo-dbgsym_b315e2605.tar.gz
    if ! [[ "$KUAVO_SYMBOL_ZIP" =~ kuavo-dbgsym_.*\.tar\.(gz|zst)$ ]]; then
        echo -e "\033[31mError: Invalid symbol zip filename format\033[0m"
        echo -e "\033[31mExample: kuavo-dbgsym_b315e2605.tar.gz\033[0m"
        exit 1
    fi

    ZIP_NAME=$(strip_archive_suffix "$KUAVO_SYMBOL_ZIP")
    GIT_COMMIT=$(echo "$ZIP_NAME" | awk -F'_' '{print $2}')
    if [ -z "$GIT_COMMIT" ]; then
        exit_with_fail "Invalid symbol zip format, e.g. kuavo-dbgsym_b315e2605.tar.gz"
    fi
    echo -e "\033[32mGIT_COMMIT: $GIT_COMMIT\033[0m"
    mkdir -p "$KUAVO_SYMBOL_DIR"

    if ! check_exist_debug_symbols "$GIT_COMMIT" "$KUAVO_SYMBOL_DIR"; then
        get_download_url "$KUAVO_SYMBOL_ZIP"    
    fi
    download_debug_symbols "$GIT_COMMIT" "$KUAVO_SYMBOL_DIR" # download debug symbols
}

# Parse command line arguments
if [[ $# -eq 0 ]]; then
    usage
    exit 0
fi

while [[ $# -gt 0 ]]; do
    case "$1" in
        -c|--crash_zip)
            if [ -n "$2" ]; then
                CRASH_ZIP="$2"
                shift 2
            else
                echo -e "\033[31mError: Argument for $1 is missing\033[0m"
                exit 1
            fi
            analyze_kuavo_crash "$CRASH_ZIP"
            ;;
        -d|--download-debug-symbol)
            if [ -n "$2" ]; then
                KUAVO_SYMBOL_ZIP="$2"
                shift 2
            else
                echo -e "\033[31mError: Argument for $1 is missing\033[0m"
                exit 1
            fi
            download_kuavo_symbols "$KUAVO_SYMBOL_ZIP"
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo -e "\033[31mError: Unknown option $1\033[0m"
            usage
            exit 1
            ;;
    esac
done
