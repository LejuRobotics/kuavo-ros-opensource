#!/bin/bash

# JavaScript/npm 版本的构建和发布脚本
SCRIPT_DIR=$(dirname "$(realpath "$0")")
PROJECT_DIR=$(realpath "$SCRIPT_DIR/../..")
SDK_PROJECT_DIR="$PROJECT_DIR/src/kuavo-humanoid-sdk"  # project: kuavo-humanoid-sdk
DIST_DIR="$SDK_PROJECT_DIR/dist"
BRANCH=$(git rev-parse --abbrev-ref HEAD)
VERSION=$(git -C "$PROJECT_DIR" describe --tags --always 2>/dev/null)

echo "SCRIPT_DIR: $SCRIPT_DIR"
echo "PROJECT_DIR: $PROJECT_DIR"
echo "SDK_PROJECT_DIR: $SDK_PROJECT_DIR"
echo "DIST_DIR: $DIST_DIR"

# 定义需要处理的消息包
MSG_PACKAGES="kuavo_msgs ocs2_msgs motion_capture_ik"

# 复制和转换消息定义文件
copy_and_convert_msgs() {
    local src_dir=$1
    local dest_dir=$2
    local msg_pkg=$3

    if [ -d "$src_dir/$msg_pkg" ]; then
        echo "src: $src_dir"
        echo -e "\033[32mCopying and converting $msg_pkg ...\033[0m"
        
        # 清理旧的目录
        if [ -d "$dest_dir/$msg_pkg" ]; then
            rm -rf "$dest_dir/$msg_pkg"
        fi
        mkdir -p "$dest_dir/$msg_pkg"

        # 复制消息文件（这里需要根据实际情况调整）
        cp -r "$src_dir/$msg_pkg" "$dest_dir" && chmod -R a+w "$dest_dir/$msg_pkg"

        # 创建 index.js 文件（相当于 __init__.py）
        cat > "$dest_dir/$msg_pkg/index.js" << 'EOF'
// Message package exports
// Auto-generated during build process

// Export all message types
export * from './messages';
export * from './services'; 
export * from './actions';

// Package metadata
export const PACKAGE_NAME = process.env.npm_package_name || 'kuavo-humanoid-sdk';
export const PACKAGE_VERSION = process.env.npm_package_version || '1.0.0';
EOF
        
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

    # 通过git获取版本号 (e.g.): 1.1.0-324-g792046c35, 1.2.0 ...
    # 删除hash部分
    local version1=$(echo "$__version_ref" | sed 's/-g[0-9a-f]\+//')
    
    if [ "$branch" == "beta" ]; then
        # beta版本: 1.1.0-324 ---> 1.1.0-beta.324
        version1=$(echo "$version1" | sed 's/-/-beta./')
        if [[ ! "$version1" == *"beta"* ]]; then
            version1="${version1}-beta.0"
        fi
    elif [ "$branch" == "master" ] || [ "$branch" == "main" ]; then
        # master正式版: 1.1.0-324 ---> 1.1.0 (移除后续版本号)
        version1=$(echo "$version1" | cut -d'-' -f1)
    else 
        # 其他分支: 1.1.0-324 ---> 1.1.0-alpha.324
        version1=$(echo "$version1" | sed 's/-/-alpha./')
        if [[ ! "$version1" == *"alpha"* ]]; then
            version1="${version1}-alpha.0"
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

    tag=$(git describe --tags --match="*" --abbrev=0 --candidates=1 main 2>/dev/null || git describe --tags --match="*" --abbrev=0 --candidates=1 master 2>/dev/null)
    tag_commit=$(git rev-list -n 1 $tag)
    number=$(git rev-list --count $tag_commit)
    commit_hash=$(git rev-parse --short HEAD)
    __version_ref="${tag}-${number}-g${commit_hash}"
}

clean_cache() {
    echo "Cleaning build cache..."
    if [ -d "$SDK_PROJECT_DIR/src/msg" ]; then
        find "$SDK_PROJECT_DIR/src/msg" -mindepth 1 -type d -exec rm -rf {} \; 2>/dev/null || true
        echo -e "\033[32mMessage directories cleaned successfully\033[0m"
    fi

    # 清理构建目录
    for dir in "dist" "node_modules/.cache" ".rollup.cache"; do
        if [ -d "$SDK_PROJECT_DIR/$dir" ]; then
            rm -rf "$SDK_PROJECT_DIR/$dir"
            echo -e "\033[32m${dir} directory cleaned successfully\033[0m"
        fi
    done
}

exit_with_failure() {
    if [ -n "$OLDPWD" ]; then
        popd 2>/dev/null || true
    fi
    clean_cache
    exit 1
}

# 检查 Node.js 和 npm
check_node_environment() {
    if ! command -v node &> /dev/null; then
        echo -e "\033[31mError: Node.js is not installed\033[0m"
        exit 1
    fi
    
    if ! command -v npm &> /dev/null; then
        echo -e "\033[31mError: npm is not installed\033[0m"
        exit 1
    fi
    
    echo -e "\033[32mNode.js version: $(node --version)\033[0m"
    echo -e "\033[32mnpm version: $(npm --version)\033[0m"
}

# 更新 package.json 中的版本号
update_package_version() {
    local version=$1
    pushd "$SDK_PROJECT_DIR"
    
    # 使用 npm version 命令更新版本（不创建git标签）
    npm version "$version" --no-git-tag-version
    
    popd
}

# SCRIPT BEGIN
echo -e "\033[34m=== Kuavo Humanoid SDK JavaScript Build Script ===\033[0m"

# 检查环境
check_node_environment

# 检查版本格式
if [[ ! "$VERSION" =~ ^[0-9]+\.[0-9]+\.[0-9]+ ]]; then
    echo -e "\033[33mWarning: VERSION format is invalid, attempting to get version from git...\033[0m"
    get_version_from_git VERSION
fi

check_and_format_version "$BRANCH" VERSION
echo -e "\033[32mVersion: $VERSION\033[0m"
echo -e "\033[32mBranch: $BRANCH\033[0m"

# 清理缓存
clean_cache

# 复制消息包
dest_dir="$SDK_PROJECT_DIR/src/msg"
mkdir -p "$dest_dir"

IFS=' ' read -r -a MSG_ARRAY <<< "$MSG_PACKAGES"
for msg_pkg in "${MSG_ARRAY[@]}"; do
    devel_pkg_dir="$PROJECT_DIR/devel/lib/python3/dist-packages/"
    if [ -d "$devel_pkg_dir" ]; then
        copy_and_convert_msgs "$devel_pkg_dir" "$dest_dir" "$msg_pkg" 
    else
        echo -e "\033[31mError: Devel directory does not exist. Path: $devel_pkg_dir\033[0m"
        exit 1
    fi
done

# 进入项目目录进行构建
pushd "$SDK_PROJECT_DIR"

# 检查是否已登录 npm
if ! npm whoami &> /dev/null; then
    echo -e "\033[33mWarning: Not logged in to npm. Please run 'npm login' first.\033[0m"
    read -p "Do you want to login now? (y/n): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        npm login
    else
        echo -e "\033[31mAborted: npm login required for publishing\033[0m"
        exit 1
    fi
fi

# 更新版本号
update_package_version "$VERSION"

# 安装依赖
echo -e "\033[32mInstalling dependencies...\033[0m"
if ! npm install; then
    echo -e "\033[31mError: Failed to install dependencies\033[0m"
    exit_with_failure
fi

# 运行构建
echo -e "\033[32mBuilding package...\033[0m"
if ! npm run build; then
    echo -e "\033[31mError: Build failed\033[0m"
    exit_with_failure
fi

# 运行测试（如果存在）
if [ -f "package.json" ] && grep -q '"test"' package.json; then
    echo -e "\033[32mRunning tests...\033[0m"
    npm test
fi

# 检查包内容
echo -e "\033[32mChecking package contents...\033[0m"
npm pack --dry-run

# 发布询问
echo -e "\033[33m=== Ready to publish ===\033[0m"
echo -e "\033[33mPackage: kuavo-humanoid-sdk@$VERSION\033[0m"
echo -e "\033[33mBranch: $BRANCH\033[0m"

read -p "Do you want to publish this package to npm? (y/n): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    # 发布包
    if [ "$BRANCH" == "beta" ]; then
        # Beta 版本发布到 beta tag
        echo -e "\033[32mPublishing beta version...\033[0m"
        npm publish --tag beta
    elif [[ "$VERSION" == *"alpha"* ]]; then
        # Alpha 版本发布到 alpha tag
        echo -e "\033[32mPublishing alpha version...\033[0m"
        npm publish --tag alpha
    else
        # 正式版本发布到 latest tag
        echo -e "\033[32mPublishing stable version...\033[0m"
        npm publish
    fi
    
    if [ $? -eq 0 ]; then
        echo -e "\033[32m\n🎉🎉🎉 Publication successful! \033[0m"
        echo -e "\033[32m-------------------------------------------\033[0m"
        echo -e "\033[32mPackage: kuavo-humanoid-sdk@$VERSION\033[0m"
        echo -e "\033[32mInstall with: npm install kuavo-humanoid-sdk\033[0m"
        if [ "$BRANCH" == "beta" ] || [[ "$VERSION" == *"alpha"* ]]; then
            echo -e "\033[33mNote: This is a pre-release version\033[0m"
            echo -e "\033[33mInstall with: npm install kuavo-humanoid-sdk@$BRANCH\033[0m"
        fi
        echo -e "\033[32m-------------------------------------------\033[0m"
    else
        echo -e "\033[31mError: Publication failed\033[0m"
        exit_with_failure
    fi
else
    echo -e "\033[33mPublication cancelled by user\033[0m"
fi

popd

echo -e "\033[32mScript completed successfully!\033[0m"