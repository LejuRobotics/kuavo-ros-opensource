#!/bin/bash
SCRIPT_DIR=$(dirname "$(realpath "$0")")
PROJECT_DIR=$(realpath "$SCRIPT_DIR/../..")
SDK_PROJECT_DIR="$PROJECT_DIR/src/kuavo_humanoid_sdk"  # project: kuavo_humanoid_sdk
DEVEL_DIR="$PROJECT_DIR/devel/"
INSTALLED_DIR="$PROJECT_DIR/installed/lib/python3/dist-packages"
BRANCH=$(git rev-parse --abbrev-ref HEAD)
VERSION=$(git -C "$PROJECT_DIR" describe --tags --always 2>/dev/null)

# Backup current pip source and switch to faster source
echo "🔄 Switching to faster pip source..."
ORIGINAL_PIP_SOURCE=$(pip config get global.index-url 2>/dev/null || echo "")
echo "Original pip source: ${ORIGINAL_PIP_SOURCE:-'default'}"

# Define multiple pip mirrors (in order of preference)
PIP_MIRRORS=(
    "https://pypi.tuna.tsinghua.edu.cn/simple/"  # 清华大学镜像
    "https://mirrors.aliyun.com/pypi/simple/"     # 阿里云镜像
    "https://pypi.mirrors.ustc.edu.cn/simple/"   # 中科大镜像
    "https://pypi.org/simple/"                    # 官方 PyPI (fallback)
)

# Switch to first mirror (Tsinghua University)
CURRENT_MIRROR_INDEX=0
pip config set global.index-url "${PIP_MIRRORS[$CURRENT_MIRROR_INDEX]}"
echo "✅ Switched to Tsinghua University mirror"

# Function to restore original pip source
restore_pip_source() {
    echo "🔄 Restoring original pip source..."
    if [ -n "$ORIGINAL_PIP_SOURCE" ]; then
        pip config set global.index-url "$ORIGINAL_PIP_SOURCE"
        echo "✅ Restored to original source: $ORIGINAL_PIP_SOURCE"
    else
        pip config unset global.index-url 2>/dev/null || true
        echo "✅ Restored to default source"
    fi
}

# Set trap to restore source on script exit
trap restore_pip_source EXIT

# Function to try installing with different mirrors
install_with_fallback() {
    local install_cmd="$1"
    local max_retries=2
    local retry_count=0
    local temp_log=$(mktemp)
    
    # Clean up temp file on exit
    trap "rm -f $temp_log" RETURN
    
    while [ $retry_count -lt $max_retries ]; do
        # Try current mirror
        echo -e "\033[33m尝试使用镜像源: ${PIP_MIRRORS[$CURRENT_MIRROR_INDEX]}\033[0m"
        
        # Execute command and capture output
        if eval "$install_cmd" > "$temp_log" 2>&1; then
            cat "$temp_log"
            echo -e "\033[32m✅ 安装成功！\033[0m"
            return 0
        fi
        
        # Show error output
        cat "$temp_log" >&2
        
        # Check if error is 403 Forbidden or connection issue
        if grep -qiE "403|Forbidden|Connection.*refused|timeout|SSL.*error" "$temp_log"; then
            echo -e "\033[33m⚠️  当前镜像源出现问题，尝试切换到下一个镜像源...\033[0m"
            
            # Try next mirror
            CURRENT_MIRROR_INDEX=$((CURRENT_MIRROR_INDEX + 1))
            
            if [ $CURRENT_MIRROR_INDEX -ge ${#PIP_MIRRORS[@]} ]; then
                echo -e "\033[31m❌ 所有镜像源都尝试失败，尝试使用官方 PyPI\033[0m"
                # Last resort: try official PyPI
                pip config set global.index-url "https://pypi.org/simple/"
                pip config unset global.trusted-host 2>/dev/null || true
                
                if eval "$install_cmd"; then
                    echo -e "\033[32m✅ 使用官方 PyPI 安装成功！\033[0m"
                    return 0
                else
                    echo -e "\033[31m❌ 所有安装方式都失败了\033[0m"
                    return 1
                fi
            fi
            
            # Switch to next mirror
            pip config set global.index-url "${PIP_MIRRORS[$CURRENT_MIRROR_INDEX]}"
            echo -e "\033[32m已切换到: ${PIP_MIRRORS[$CURRENT_MIRROR_INDEX]}\033[0m"
            
            # Reset retry count when switching mirror
            retry_count=0
        else
            # Other errors, just retry
            retry_count=$((retry_count + 1))
            if [ $retry_count -lt $max_retries ]; then
                echo -e "\033[33m⚠️  安装失败，${retry_count}/${max_retries} 次重试...\033[0m"
                sleep 2
            fi
        fi
    done
    
    echo -e "\033[31m❌ 安装失败，已尝试所有镜像源和重试次数\033[0m"
    return 1
}

# echo "SCRIPT_DIR: $SCRIPT_DIR"
# echo "PROJECT_DIR: $PROJECT_DIR"
# echo "DEVEL_DIR: $DEVEL_DIR"
# echo "INSTALLED_DIR: $INSTALLED_DIR"

# Define the ROS message packages to be copied
# These packages contain message definitions needed by the SDK
MSG_PACKAGES="kuavo_msgs ocs2_msgs motion_capture_ik"

copy_ros_msg() {
    local src_dir=$1
    local dest_dir=$2
    local msg_pkg=$3

    # echo "$src_dir/$msg_pkg"
    if [ -d "$src_dir/$msg_pkg" ]; then
        echo "src: $src_dir"
        echo -e "\033[32mCopying $msg_pkg ...\033[0m"
        if [ -d "$dest_dir/$msg_pkg" ]; then
            rm -rf "$dest_dir/$msg_pkg"
        fi
        mkdir "$dest_dir/$msg_pkg"
        cp -r "$src_dir/$msg_pkg" "$dest_dir" && chmod -R a+w "$dest_dir/$msg_pkg"

        # Create __init__.py file with import statements
        echo "import os
import sys

# Add package path to sys.path if not already there
package_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if package_path not in sys.path:
    sys.path.append(package_path)" > "$dest_dir/$msg_pkg/__init__.py"
        
    else 
        echo -e "\033[31mError: 未找到对应的消息包，请先执行 catkin build $msg_pkg 构建\033[0m"
        exit 1
    fi
}

check_and_format_version() {
    local branch="$1"
    local -n __version_ref="$2"
    
    if [ $? -ne 0 ] || [ -z "$__version_ref" ]; then
        exit_with_failure "Failed to get version from git describe"
    fi

    # 通过git获取(e.g.): 1.1.0-324-g792046c35, 1.2.0 ...
    # Remove the hash part (g followed by alphanumeric characters) from the version
    local version1=$(echo "$__version_ref" | sed 's/-g[0-9a-f]\+//') # 删除 hash后缀
    if [ "$branch" == "beta" ]; then
        # Replace hyphens with 'b' in the version string
        version1=$(echo "$version1" | sed 's/-/b/g')      # beta 版本: 1.1.0-324  ---> 1.1.0b324
        if [[ ! "$version1" == *"b"* ]]; then
            # 避免在beta分支上发布 1.1.0 的情况(1.1.0这样的版本号是给正式版使用的) --> 1.1.0b0
            version1="${version1}b0"  # Append 'b0' if version does not contain 'b'
        fi
    elif [ "$branch" == "master" ]; then
        version1=$(echo "$version1" | sed 's/-/.post/g')  # master 正式版: 1.1.0-324  ---> 1.1.0.post324
    # if U want to publish to dev branch, you can add it here.
    else 
        # Replace hyphens with 'a' in the version string
        version1=$(echo "$version1" | sed 's/-/a/g')      # 其他 版本: 1.1.0-324  ---> 1.1.0a324
        if [[ ! "$version1" == *"a"* ]]; then
            # 避免在beta分支上发布 1.1.0 的情况(1.1.0这样的版本号是给正式版使用的) --> 1.1.0a0
            version1="${version1}a0"  # Append 'a0' if version does not contain 'a'
        fi
    fi
    
    __version_ref="$version1"
}

get_version_from_git() {
    local -n __version_ref="$1"
    # Check if git is available
    if ! command -v git &> /dev/null; then
        echo -e "\033[31mError: git is not installed or not in PATH\033[0m"
        exit 1
    fi
    
    # Check if we're in a git repository
    if ! git rev-parse --git-dir &> /dev/null; then
        echo -e "\033[31mError: Not in a git repository\033[0m"
        exit 1
    fi

    tag=$(git describe --tags --match="*" --abbrev=0 --candidates=1 master)
    tag_commit=$(git rev-list -n 1 $tag)
    number=$(git rev-list --count $tag_commit)
    commit_hash=$(git rev-parse --short HEAD)
    __version_ref="${tag}-${number}-g${commit_hash}"
}

clean_cache() {
    # Clean up message directories
    echo "Cleaning message directories..."
    if [ -d "$SDK_PROJECT_DIR/kuavo_humanoid_sdk/msg" ]; then
        # Find and remove all directories under msg/ (but keep the msg directory itself)
        find "$SDK_PROJECT_DIR/kuavo_humanoid_sdk/msg" -mindepth 1 -type d -exec rm -rf {} \; 2>/dev/null || true
        echo -e "\033[32mMessage directories cleaned successfully\033[0m"
    else
        echo -e "\033[33mWarning: Message directory does not exist: $SDK_PROJECT_DIR/kuavo_humanoid_sdk/msg\033[0m"
    fi

    # Clean up build and dist directories
    for dir in "build" "dist"; do
        if [ -d "$SDK_PROJECT_DIR/$dir" ]; then
            rm -rf "$SDK_PROJECT_DIR/$dir"
            echo -e "\033[32m${dir^} directory cleaned successfully\033[0m"
        fi
    done
}

exit_with_failure() {
    # Check if we're in a pushed directory and pop if needed
    if [ -n "$OLDPWD" ]; then
        popd 2>/dev/null || true
    fi
    clean_cache
    exit 1
}

# SCRIPT BEGIN
# Check if kuavo-humanoid-sdk-ws is installed
check_conflicting_package() {
    if pip show kuavo-humanoid-sdk-ws >/dev/null 2>&1; then
        echo -e "\033[33m⚠️  检测到已安装 kuavo-humanoid-sdk-ws，与 kuavo-humanoid-sdk 可能存在冲突\033[0m"
        pip show kuavo-humanoid-sdk-ws | grep -E "Name:|Version:" || true
        echo ""
        echo -e "\033[33m是否要卸载 kuavo-humanoid-sdk-ws 并继续安装 kuavo-humanoid-sdk？\033[0m"
        read -p "请输入 [y/Y] 继续卸载并安装，或 [n/N] 取消安装: " -n 1 -r
        echo ""
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            echo -e "\033[32m正在卸载 kuavo-humanoid-sdk-ws...\033[0m"
            pip uninstall kuavo-humanoid-sdk-ws -y
            if [ $? -eq 0 ]; then
                echo -e "\033[32m✅ kuavo-humanoid-sdk-ws 已成功卸载\033[0m"
                
                # Clean up easy-install.pth file if it contains references to kuavo-humanoid-sdk-ws
                cleanup_easy_install_pth() {
                    local python_version=$(python3 --version 2>/dev/null | cut -d ' ' -f 2 | cut -d '.' -f 1-2)
                    # Check multiple possible locations: current user and system-wide
                    local easy_install_paths=(
                        "$HOME/.local/lib/python${python_version}/dist-packages/easy-install.pth"
                        "/usr/local/lib/python${python_version}/dist-packages/easy-install.pth"
                    )
                    
                    local cleaned_count=0
                    for easy_install_path in "${easy_install_paths[@]}"; do
                        if [ -f "$easy_install_path" ]; then
                            echo -e "\033[33m🔧 检查并清理: $easy_install_path\033[0m"
                            # Create backup
                            if [ -w "$easy_install_path" ]; then
                                cp "$easy_install_path" "${easy_install_path}.bak" 2>/dev/null || true
                            else
                                sudo cp "$easy_install_path" "${easy_install_path}.bak" 2>/dev/null || true
                            fi
                            
                            # Remove lines containing kuavo-humanoid-sdk-ws or kuavo_humanoid_sdk_ws
                            if grep -q "kuavo.*humanoid.*sdk.*ws" "$easy_install_path" 2>/dev/null; then
                                # Use sed to remove lines containing the conflicting package
                                # Try with sudo if file is not writable
                                if [ -w "$easy_install_path" ]; then
                                    sed -i '/kuavo.*humanoid.*sdk.*ws/d' "$easy_install_path" 2>/dev/null || \
                                    sed -i.bak '/kuavo.*humanoid.*sdk.*ws/d' "$easy_install_path" 2>/dev/null || true
                                else
                                    sudo sed -i '/kuavo.*humanoid.*sdk.*ws/d' "$easy_install_path" 2>/dev/null || \
                                    sudo sed -i.bak '/kuavo.*humanoid.*sdk.*ws/d' "$easy_install_path" 2>/dev/null || true
                                fi
                                echo -e "\033[32m✅ 已清理: $easy_install_path\033[0m"
                                cleaned_count=$((cleaned_count + 1))
                            else
                                echo -e "\033[32m✅ 未发现残留条目: $easy_install_path\033[0m"
                            fi
                        fi
                    done
                    
                    if [ $cleaned_count -eq 0 ]; then
                        echo -e "\033[32m✅ 所有 easy-install.pth 文件中均未发现残留条目\033[0m"
                    fi
                }
                
                cleanup_easy_install_pth
            else
                echo -e "\033[31m❌ 卸载 kuavo-humanoid-sdk-ws 失败，安装已取消\033[0m"
                exit 1
            fi
        else
            echo -e "\033[33m安装已取消\033[0m"
            exit 0
        fi
    fi
}

# Check for conflicting package before installation
check_conflicting_package

# Check if VERSION follows the expected format (e.g., 0.0.1)
if [[ ! "$VERSION" =~ ^[0-9]+\.[0-9]+\.[0-9]+ ]]; then
    echo -e "\033[33mWarning: VERSION format is invalid, attempting to get version from git...\033[0m"
    get_version_from_git VERSION
fi

check_and_format_version "$BRANCH" VERSION
echo -e "\033[32mVersion: $VERSION\033[0m"
echo -e "\033[32mBranch: $BRANCH\033[0m"
clean_cache

#copy kuavo message packages
dest_dir="$SCRIPT_DIR/kuavo_humanoid_sdk/msg"
IFS=' ' read -r -a MSG_ARRAY <<< "$MSG_PACKAGES"
for msg_pkg in "${MSG_ARRAY[@]}"; do
    if [ -d "$DEVEL_DIR/.private/$msg_pkg/lib/python3/dist-packages" ]; then
        devel_pkg_dir="$DEVEL_DIR/.private/$msg_pkg/lib/python3/dist-packages"
    else
        devel_pkg_dir="$DEVEL_DIR/lib/python3/dist-packages/"
    fi
    if [ -d "$devel_pkg_dir" ]; then
        # Copy the ROS message packages from the installed directory to the destination directory
        copy_ros_msg "$devel_pkg_dir" "$dest_dir" "$msg_pkg" 
    else
        echo -e "\033[31mError: Neither the installed nor the devel directory exists. Path: $devel_pkg_dir\033[0m"
        exit 1
    fi
done

# 检查是否已安装 python3-pyaudio
if ! dpkg -s python3-pyaudio >/dev/null 2>&1; then
  echo "🔧 python3-pyaudio 未安装，正在安装..."
  sudo apt update
  sudo apt install -y python3-pyaudio
else
  echo "✅ python3-pyaudio 已安装，跳过安装"
fi
# pip install
pushd $SCRIPT_DIR

# Upgrade conflicting dependencies first
echo "🔧 Upgrading conflicting dependencies..."
# Only upgrade requests, skip scikit-learn due to Python 3.8 compatibility
install_with_fallback 'pip install --upgrade "requests>=2.25.0"' || echo "⚠️  Warning: requests could not be upgraded, continuing with installation..."
# Note: scikit-learn 1.6+ requires Python 3.9+, keeping 1.3.2 for Python 3.8 compatibility

# Install the package editably with fallback mechanism
echo "🔧 Installing kuavo_humanoid_sdk..."
if install_with_fallback "KUAVO_HUMANOID_SDK_VERSION=\"$VERSION\" pip install -e ./"; then
    echo -e "\033[32m\n🎉🎉🎉 Installation successful! \033[0m"
    echo -e "\033[32m-------------------------------------------\033[0m"
    pip show kuavo_humanoid_sdk
    echo -e "\033[32m-------------------------------------------\033[0m"
    
    # Check for remaining conflicts
    echo -e "\033[33m🔍 Checking for remaining version conflicts...\033[0m"
    pip check || echo -e "\033[33m⚠️  Some version conflicts remain. The SDK should still work, but some features may be limited.\033[0m"
    
    # Ensure all files are accessible by all users
    echo -e "\033[33m🔧 Setting file permissions for all users...\033[0m"
    sudo chmod -R a+rwx "$SCRIPT_DIR"
    echo -e "\033[32m✅ File permissions set for all users\033[0m"
else
    echo -e "\033[31m❌ Installation failed after trying all available mirrors\033[0m"
    echo -e "\033[33m💡 建议检查网络连接 \033[0m"
    exit_with_failure
fi
popd
