#!/bin/bash
# Script to generate documentation for Kuavo Humanoid SDK
SCRIPT_DIR=$(dirname "$(realpath "$0")")
PROJECT_DIR=$(realpath "$SCRIPT_DIR/../..")
SDK_PROJECT_DIR="$PROJECT_DIR/src/kuavo_humanoid_sdk"  # project: kuavo_humanoid_sdk
DEVEL_DIR="$PROJECT_DIR/devel/.private"
BRANCH=$(git rev-parse --abbrev-ref HEAD)
VERSION=$(git -C "$PROJECT_DIR" describe --tags --always 2>/dev/null)

# Exit on error
set -e

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
        exit_with_failure
    fi
}

install_dependencies() {
    echo "Installing dependencies..."
    for pkg in sphinx sphinx-rtd-theme sphinx-markdown-builder; do
        pip list | grep "$pkg" > /dev/null || (echo "Installing $pkg..." && pip install "$pkg")
    done
}

check_ros_env() {
    # Check if ROS environment is set up
    if [ -z "$ROS_DISTRO" ]; then
        echo "Error: ROS environment not found. Please source your ROS setup file first."
        echo "Example: source /opt/ros/<distro>/setup.bash"
        exit_with_failure
    fi

    # Check if humanoid_controllers is running
    # if ! pgrep -f "roslaunch humanoid_controllers" > /dev/null; then
    #     echo -e "\033[31mError: humanoid_controllers not running. Please start it first.\033[0m"
    #     echo -e "\033[32mExample(mujoco sim): roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch\033[0m"
    #     echo -e "\033[32mExample(real): roslaunch humanoid_controllers load_kuavo_real.launch\033[0m"
    #     exit_with_failure
    # fi
}

copy_ros_msgs() {
    MSG_PACKAGES="kuavo_msgs ocs2_msgs motion_capture_ik"
    IFS=' ' read -r -a MSG_ARRAY <<< "$MSG_PACKAGES"
    dest_dir="$SCRIPT_DIR/kuavo_humanoid_sdk/msg"
    for msg_pkg in "${MSG_ARRAY[@]}"; do
        devel_pkg_dir="$DEVEL_DIR/$msg_pkg/lib/python3/dist-packages"
        if [ -d "$devel_pkg_dir" ]; then
            # Copy the ROS message packages from the installed directory to the destination directory
            copy_ros_msg "$devel_pkg_dir" "$dest_dir" "$msg_pkg" 
        else
            echo -e "\033[31mError: Neither the installed nor the devel directory exists. Path: $devel_pkg_dir\033[0m"
            exit_with_failure
        fi
    done
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

pre_process() {
    check_and_format_version "$BRANCH" VERSION
    export GEN_KUAVO_HUMANOID_SDK_DOCS="true"
    export KUAVO_HUMANOID_SDK_VERSION="$VERSION"
}

post_process() {
    unset GEN_KUAVO_HUMANOID_SDK_DOCS
    unset KUAVO_HUMANOID_SDK_VERSION
}

exit_with_failure() {
    if [ "$1" ]; then
        echo -e "\033[31mError: $1\033[0m"
    fi
    post_process
    exit 1
}

#@@@ Main @@@@@
pre_process
install_dependencies
check_ros_env
copy_ros_msgs

echo "Generating HTML documentation..."
sphinx-build -b html -D language=zh_CN docs/ "docs/html"
echo "HTML documentation generated successfully in docs/html"

echo -e "\nGenerating Markdown documentation..."
sphinx-build -b markdown ./docs/ ./docs/markdown/
echo "Markdown documentation generated successfully in docs/markdown/"

post_process