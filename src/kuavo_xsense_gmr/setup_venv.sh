#!/bin/bash
# 自动创建虚拟环境脚本
# 根据 setup.py 自动创建和配置 gmr_venv 虚拟环境

set -e  # 遇到错误立即退出

# 获取脚本所在目录的绝对路径
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="$SCRIPT_DIR/gmr_venv"

echo "=========================================="
echo "  GMR 虚拟环境自动配置脚本"
echo "=========================================="
echo ""

echo "🔍 检查可用 Python 版本..."

# 搜索 python3.x 可执行文件，例如 python3.8 python3.10 python3.12
AVAILABLE_PYTHONS=( $(compgen -c | grep -E "^python3\.[0-9]+$" | sort -V) )

if [ ${#AVAILABLE_PYTHONS[@]} -eq 0 ]; then
    echo "❌ 未找到任何 python3.x 版本，请安装 Python>=3.10"
    exit 1
fi

echo "📌 检测到这些 Python 版本："
for p in "${AVAILABLE_PYTHONS[@]}"; do
    echo "   - $p"
done

# 自动选择最高版本（sort -V 已经排序）
PYTHON_CMD="${AVAILABLE_PYTHONS[-1]}"

echo "👉 使用 Python: $PYTHON_CMD"

PYTHON_VERSION=$($PYTHON_CMD --version 2>&1 | awk '{print $2}')
PYTHON_MAJOR=$(echo $PYTHON_VERSION | cut -d. -f1)
PYTHON_MINOR=$(echo $PYTHON_VERSION | cut -d. -f2)

if [ "$PYTHON_MAJOR" -lt 3 ] || ([ "$PYTHON_MAJOR" -eq 3 ] && [ "$PYTHON_MINOR" -lt 10 ]); then
    echo "❌ 错误: Python 版本 $PYTHON_VERSION 不符合要求 (需要 >= 3.10)"
    exit 1
fi

echo "✓ Python 版本: $PYTHON_VERSION (符合要求)"
echo ""

# 检查是否已存在虚拟环境
if [ -d "$VENV_DIR" ]; then
    echo "⚠️  虚拟环境目录已存在: $VENV_DIR"
    read -p "是否要删除并重新创建? (y/N): " -n 1 -r
    echo ""
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "🗑️  删除现有虚拟环境..."
        rm -rf "$VENV_DIR"
        echo "✓ 已删除"
    else
        echo "📦 使用现有虚拟环境，仅更新依赖..."
        USE_EXISTING=true
    fi
    echo ""
fi

# 创建虚拟环境（如果不存在）
if [ ! "$USE_EXISTING" = true ]; then
    echo "📦 创建虚拟环境: $VENV_DIR"
    $PYTHON_CMD -m venv "$VENV_DIR"
    echo "✓ 虚拟环境创建成功"
    echo ""
fi

# 激活虚拟环境
echo "🔌 激活虚拟环境..."
source "$VENV_DIR/bin/activate"

# 升级 pip, setuptools, wheel
echo "⬆️  升级 pip, setuptools, wheel..."
PIP_MIRROR="-i https://pypi.tuna.tsinghua.edu.cn/simple --trusted-host pypi.tuna.tsinghua.edu.cn"
# 设置全局 pip 镜像，加速所有 pip install（包括 -e 安装时的依赖解析）
export PIP_INDEX_URL="https://pypi.tuna.tsinghua.edu.cn/simple"
export PIP_TRUSTED_HOST="pypi.tuna.tsinghua.edu.cn"
pip install --upgrade pip setuptools wheel --quiet ${PIP_MIRROR}
echo "✓ 升级完成"
echo ""

# 检查是否有 requirements.txt（如果有，优先使用）
if [ -f "$SCRIPT_DIR/requirements.txt" ]; then
    echo "📋 发现 requirements.txt，安装依赖..."
    pip install -r "$SCRIPT_DIR/requirements.txt" --quiet ${PIP_MIRROR}
    echo "✓ requirements.txt 依赖安装完成"
    echo ""
fi

# 以开发模式安装当前包（这会自动安装 setup.py 中的依赖）
echo "📦 安装 kuavo_gmr 包及其依赖..."
echo "   注意: rosbag 和 bagpy 可能需要在 ROS 环境中安装"
echo ""

set +e  # 暂时允许错误，以便捕获安装问题
pip install -e "$SCRIPT_DIR" 2>&1 | tee /tmp/pip_install.log
INSTALL_STATUS=$?
set -e  # 恢复错误退出模式

# 检查是否是因为 rosbag 或 bagpy 导致的问题
if [ $INSTALL_STATUS -ne 0 ]; then
    if grep -qiE "(rosbag|bagpy|找不到|No module named)" /tmp/pip_install.log; then
        echo ""
        echo "⚠️  警告: ROS 相关依赖安装失败"
        echo ""
        echo "   解决方案："
        echo "   1. 确保已设置 ROS 环境变量"
        echo "      source /opt/ros/<distro>/setup.bash"
        echo ""
        echo "   2. 在 ROS 工作空间中运行此脚本"
        echo "      cd ~/catkin_ws && source devel/setup.bash"
        echo ""
        echo "   3. 如果不需要 rosbag/bagpy 功能，可以手动安装其他依赖"
        echo ""
        read -p "是否尝试仅安装非 ROS 依赖? (Y/n): " -n 1 -r
        echo ""
        if [[ ! $REPLY =~ ^[Nn]$ ]]; then
            echo "🔄 尝试仅安装非 ROS 依赖..."
            # 安装包本身（不安装依赖）
            pip install -e "$SCRIPT_DIR" --no-deps --quiet
            # 手动安装非 ROS 依赖
            pip install loop_rate_limiters mink mujoco numpy scipy "qpsolvers[proxqp]" rich tqdm opencv-python natsort psutil protobuf "redis[hiredis]" "imageio[ffmpeg]" pandas --quiet ${PIP_MIRROR}
            echo "⚠️  已跳过 rosbag/bagpy，其他依赖已安装"
            echo "   （如果需要 rosbag 功能，请在 ROS 环境中手动安装）"
        else
            echo "❌ 安装失败，请检查错误信息或设置 ROS 环境后重试"
            rm -f /tmp/pip_install.log
            exit 1
        fi
    else
        echo ""
        echo "❌ 安装失败，错误信息:"
        tail -20 /tmp/pip_install.log
        rm -f /tmp/pip_install.log
        exit 1
    fi
else
    echo "✓ 包安装完成"
fi
pip install torch torchvision torchaudio -i https://pypi.tuna.tsinghua.edu.cn/simple
rm -f /tmp/pip_install.log
echo ""

# 验证安装
echo "✅ 验证安装..."
python -c "import kuavo_gmr; print(f'✓ kuavo_gmr 版本: {kuavo_gmr.__version__ if hasattr(kuavo_gmr, \"__version__\") else \"未知\"}')" 2>/dev/null || echo "⚠️  无法验证包版本"

# 显示安装的包列表
echo ""
echo "📋 已安装的主要包:"
pip list | grep -E "(kuavo-gmr|mujoco|numpy|scipy|rich|protobuf)" || echo "（无法列出包列表）"

echo ""
echo "=========================================="
echo "✅ 虚拟环境配置完成！"
echo "=========================================="
echo ""
echo "📍 虚拟环境路径: $VENV_DIR"
echo ""
echo "💡 使用方法:"
echo "   source $VENV_DIR/bin/activate"
echo ""
echo "💡 退出虚拟环境:"
echo "   deactivate"
echo ""
