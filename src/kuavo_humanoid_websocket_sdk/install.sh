#!/bin/bash
SCRIPT_DIR=$(dirname "$(realpath "$0")")
PROJECT_DIR=$(realpath "$SCRIPT_DIR/../..")
SDK_PROJECT_DIR="$PROJECT_DIR/src/kuavo_humanoid_sdk"  # project: kuavo_humanoid_sdk
DEVEL_DIR="$PROJECT_DIR/devel/"
INSTALLED_DIR="$PROJECT_DIR/installed/lib/python3/dist-packages"
BRANCH=$(git rev-parse --abbrev-ref HEAD)
VERSION=$(git -C "$PROJECT_DIR" describe --tags --always 2>/dev/null)

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

# Function to clean easy-install.pth file
clean_easy_install_pth() {
    local package_name=$1
    echo -e "\033[33m🔍 检查并清理 easy-install.pth 文件...\033[0m"
    
    # Map package names to directory path patterns
    # Convert package name to directory name pattern (e.g., kuavo-humanoid-sdk-ws -> kuavo_humanoid_websocket_sdk)
    local path_patterns=()
    case "$package_name" in
        "kuavo-humanoid-sdk-ws")
            path_patterns=("kuavo_humanoid_websocket_sdk")
            ;;
        "kuavo-humanoid-sdk")
            path_patterns=("kuavo_humanoid_sdk")
            ;;
        *)
            # If no mapping, try converting hyphens to underscores
            path_patterns=("$(echo "$package_name" | sed 's/-/_/g')")
            ;;
    esac
    
    # Find all easy-install.pth files in Python site-packages directories
    local python_version=$(python3 -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')" 2>/dev/null || echo "")
    
    # Common locations for easy-install.pth
    local pth_files=()
    
    # Get site-packages directories
    local site_packages=$(python3 -c "import site; print('\n'.join(site.getsitepackages()))" 2>/dev/null || echo "")
    
    # Also check common system locations
    if [ -n "$python_version" ]; then
        # root用户使用系统路径，普通用户使用用户路径
        if [ "$EUID" -eq 0 ]; then
            pth_files+=("/usr/local/lib/python${python_version}/dist-packages/easy-install.pth")
        else
            pth_files+=("$HOME/.local/lib/python${python_version}/site-packages/easy-install.pth")
        fi
    fi
    
    # Add site-packages directories
    # root用户添加所有路径，普通用户只添加用户路径
    while IFS= read -r dir; do
        if [ -n "$dir" ] && [ -d "$dir" ]; then
            if [ "$EUID" -eq 0 ]; then
                # root用户：添加所有路径
                pth_files+=("$dir/easy-install.pth")
            else
                # 普通用户：只添加用户路径（$HOME/.local 开头的路径）
                if [[ "$dir" == "$HOME/.local"* ]]; then
                    pth_files+=("$dir/easy-install.pth")
                fi
            fi
        fi
    done <<< "$site_packages"
    
    local cleaned=false
    for pth_file in "${pth_files[@]}"; do
        if [ -f "$pth_file" ]; then
            echo "检查文件: $pth_file"
            
            # Check if file contains any of the patterns
            local has_pattern=false
            for pattern in "${path_patterns[@]}"; do
                if grep -qi "$pattern" "$pth_file" 2>/dev/null; then
                    has_pattern=true
                    break
                fi
            done
            
            if [ "$has_pattern" = true ]; then
                # Create backup (use sudo if needed)
                if [ ! -w "$pth_file" ]; then
                    sudo cp "$pth_file" "${pth_file}.bak" 2>/dev/null || true
                else
                    cp "$pth_file" "${pth_file}.bak" 2>/dev/null || true
                fi
                
                # Use sed to remove lines containing any of the patterns
                # Create temp file in /tmp to avoid permission issues
                local temp_file="/tmp/easy-install-$$.tmp"
                local temp_file2="/tmp/easy-install-$$-2.tmp"
                
                # Read original file and filter out matching lines
                > "$temp_file"  # Clear temp file
                while IFS= read -r line || [ -n "$line" ]; do
                    local should_remove=false
                    for pattern in "${path_patterns[@]}"; do
                        if echo "$line" | grep -qi "$pattern" 2>/dev/null; then
                            should_remove=true
                            echo -e "\033[33m  发现并删除路径: $line\033[0m"
                            break
                        fi
                    done
                    if [ "$should_remove" != true ]; then
                        echo "$line" >> "$temp_file"
                    fi
                done < "$pth_file"
                
                # Replace original file (use sudo if needed)
                if [ ! -w "$pth_file" ]; then
                    if sudo cp "$temp_file" "$pth_file" 2>/dev/null; then
                        cleaned=true
                        echo -e "\033[32m  ✅ 已清理 $pth_file\033[0m"
                    else
                        echo -e "\033[33m  ⚠️  清理 $pth_file 时出现问题（需要 sudo 权限）\033[0m"
                    fi
                else
                    if cp "$temp_file" "$pth_file" 2>/dev/null; then
                        cleaned=true
                        echo -e "\033[32m  ✅ 已清理 $pth_file\033[0m"
                    else
                        echo -e "\033[33m  ⚠️  清理 $pth_file 时出现问题\033[0m"
                    fi
                fi
                
                # Clean up temp files
                rm -f "$temp_file" "$temp_file2" 2>/dev/null || true
            else
                echo -e "\033[32m  ✅ 未发现需要清理的路径: $pth_file\033[0m"
            fi
        fi
    done
    
    if [ "$cleaned" = true ]; then
        echo -e "\033[32m✅ easy-install.pth 文件清理完成\033[0m"
    else
        echo -e "\033[33mℹ️  未找到需要清理的 easy-install.pth 文件\033[0m"
    fi
}


# SCRIPT BEGIN
# Check if kuavo-humanoid-sdk is installed
check_conflicting_package() {
    if pip show kuavo-humanoid-sdk >/dev/null 2>&1; then
        echo -e "\033[33m⚠️  检测到已安装 kuavo-humanoid-sdk，与 kuavo-humanoid-sdk-ws 可能存在冲突\033[0m"
        pip show kuavo-humanoid-sdk | grep -E "Name:|Version:" || true
        echo ""
        echo -e "\033[33m是否要卸载 kuavo-humanoid-sdk 并继续安装 kuavo-humanoid-sdk-ws？\033[0m"
        read -p "请输入 [y/Y] 继续卸载并安装，或 [n/N] 取消安装: " -n 1 -r
        echo ""
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            echo -e "\033[32m正在卸载 kuavo-humanoid-sdk...\033[0m"
            pip uninstall kuavo-humanoid-sdk -y
            if [ $? -eq 0 ]; then
                echo -e "\033[32m✅ kuavo-humanoid-sdk 已成功卸载\033[0m"
                # Clean easy-install.pth file after uninstall
                clean_easy_install_pth "kuavo-humanoid-sdk"
            else
                echo -e "\033[31m❌ 卸载 kuavo-humanoid-sdk 失败，安装已取消\033[0m"
                exit 1
            fi
        else
            echo -e "\033[33m安装已取消\033[0m"
            exit 0
        fi
    fi
}

# Check if kuavo-humanoid-sdk-ws is already installed and update path if needed
check_and_update_existing_sdk() {
    if pip show kuavo-humanoid-sdk-ws >/dev/null 2>&1; then
        local installed_version=$(pip show kuavo-humanoid-sdk-ws | grep "^Version:" | awk '{print $2}')
        echo -e "\033[33m⚠️  检测到已安装 kuavo-humanoid-sdk-ws (版本: $installed_version)\033[0m"
        echo -e "\033[33m正在卸载旧版本以更新SDK路径...\033[0m"
        
        pip uninstall kuavo-humanoid-sdk-ws -y
        if [ $? -eq 0 ]; then
            echo -e "\033[32m✅ kuavo-humanoid-sdk-ws 旧版本已成功卸载\033[0m"
            # Clean easy-install.pth file after uninstall to remove old paths
            clean_easy_install_pth "kuavo-humanoid-sdk-ws"
        else
            echo -e "\033[31m❌ 卸载 kuavo-humanoid-sdk-ws 失败，安装已取消\033[0m"
            exit 1
        fi
    fi
}

# Check for conflicting package before installation
check_conflicting_package

# Check and update existing SDK installation
check_and_update_existing_sdk

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

# pip install
pushd $SCRIPT_DIR
# Install the package editably
if KUAVO_HUMANOID_SDK_VERSION="$VERSION" pip install -e ./; then
    echo -e "\033[32m\n🎉🎉🎉 Installation successful! \033[0m"
    echo -e "\033[32m-------------------------------------------\033[0m"
    pip show kuavo_humanoid_sdk_ws
    echo -e "\033[32m-------------------------------------------\033[0m"
fi
popd