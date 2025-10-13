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

# Switch to Tsinghua University mirror (faster for Chinese users)
pip config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple/
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
pip install --upgrade "requests>=2.25.0" || echo "⚠️  Warning: requests could not be upgraded, continuing with installation..."
# Note: scikit-learn 1.6+ requires Python 3.9+, keeping 1.3.2 for Python 3.8 compatibility

# Install the package editably
if KUAVO_HUMANOID_SDK_VERSION="$VERSION" pip install -e ./; then
    echo -e "\033[32m\n🎉🎉🎉 Installation successful! \033[0m"
    echo -e "\033[32m-------------------------------------------\033[0m"
    pip show kuavo_humanoid_sdk
    echo -e "\033[32m-------------------------------------------\033[0m"
    
    # Check for remaining conflicts
    echo -e "\033[33m🔍 Checking for remaining version conflicts...\033[0m"
    pip check || echo -e "\033[33m⚠️  Some version conflicts remain. The SDK should still work, but some features may be limited.\033[0m"
fi
popd
