#!/bin/bash
# Setup virtual environment for kuavo_mocap_gmr package
# Requires Python >= 3.10

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
VENV_DIR="$PKG_DIR/gmr_venv"

echo "=========================================="
echo "  kuavo_mocap_gmr 虚拟环境配置"
echo "=========================================="
echo ""

# Find Python 3.10+
echo "🔍 检查 Python 版本..."

PYTHON_CMD=""
for py in python3.12 python3.11 python3.10; do
    if command -v $py &> /dev/null; then
        PYTHON_CMD=$py
        break
    fi
done

if [ -z "$PYTHON_CMD" ]; then
    echo "❌ 未找到 Python 3.10+，请安装 Python >= 3.10"
    exit 1
fi

PYTHON_VERSION=$($PYTHON_CMD --version 2>&1 | awk '{print $2}')
echo "✓ 使用 Python: $PYTHON_CMD ($PYTHON_VERSION)"
echo ""

# Create virtual environment
if [ -d "$VENV_DIR" ]; then
    echo "⚠️  虚拟环境已存在: $VENV_DIR"
    read -p "是否删除并重新创建? (y/N): " -n 1 -r
    echo ""
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        rm -rf "$VENV_DIR"
    else
        echo "使用现有虚拟环境"
    fi
fi

if [ ! -d "$VENV_DIR" ]; then
    echo "📦 创建虚拟环境..."
    $PYTHON_CMD -m venv "$VENV_DIR"
fi

# Activate
source "$VENV_DIR/bin/activate"

# Upgrade pip
echo "⬆️  升级 pip..."
pip install --upgrade pip setuptools wheel --quiet

# Install dependencies
echo "📦 安装依赖..."
pip install \
    numpy \
    scipy \
    mujoco \
    mink \
    loop_rate_limiters \
    "qpsolvers[proxqp]" \
    rich \
    tqdm \
    opencv-python \
    pyyaml \
    rospkg \
    catkin_pkg \
    netifaces \
    defusedxml \
    "imageio[ffmpeg]" \
    smplx \
    torch \
    torchvision \
    -i https://pypi.tuna.tsinghua.edu.cn/simple

echo ""
echo "=========================================="
echo "✅ 虚拟环境配置完成！"
echo "=========================================="
echo ""
echo "📍 路径: $VENV_DIR"
echo ""
echo "💡 使用方法:"
echo "   roslaunch kuavo_mocap_gmr gmr_sim.launch"
echo ""
