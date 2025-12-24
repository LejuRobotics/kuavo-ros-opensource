#!/bin/bash

# 语音控制节点安装脚本
# 用于安装Python依赖和预下载模型

set -e  # 遇到错误时停止执行

echo "开始安装voice_control_node依赖..."

# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# 切换到项目目录
cd "$PROJECT_DIR"

# 检查requirements.txt是否存在
if [ ! -f "requirements.txt" ]; then
    echo "错误: requirements.txt 文件不存在"
    exit 1
fi

# 检查当前环境中是否已安装依赖
echo "检查当前环境中已安装的依赖..."

# 检查modelscope版本
if python3 -c "import modelscope; print(modelscope.__version__)" &> /dev/null; then
    INSTALLED_MODELSCOPE=$(python3 -c "import modelscope; print(modelscope.__version__)")
    REQUIRED_MODELSCOPE="1.10.0"
    
    echo "检测到已安装的modelscope版本: $INSTALLED_MODELSCOPE"
    
    if [ "$INSTALLED_MODELSCOPE" != "$REQUIRED_MODELSCOPE" ]; then
        echo "警告: 当前modelscope版本($INSTALLED_MODELSCOPE)与所需版本($REQUIRED_MODELSCOPE)不匹配"
        echo "卸载当前版本并安装所需版本..."
        pip3 uninstall -y modelscope
        pip3 install modelscope==$REQUIRED_MODELSCOPE
        echo "modelscope $REQUIRED_MODELSCOPE 安装完成"
    else
        echo "modelscope版本匹配，无需重新安装"
    fi
else
    echo "未检测到modelscope，将安装指定版本..."
    pip3 install modelscope==1.10.0
fi

# 安装modelscope的额外依赖（解决uvicorn缺失问题）
echo "安装modelscope所需的额外依赖..."
pip3 install uvicorn fastapi

# 安装其他依赖（跳过已安装的）
echo "正在安装其他Python依赖..."
pip3 install -r requirements.txt

echo "Python依赖安装完成"

# 检查funasr是否安装成功
if ! python3 -c "import funasr" &> /dev/null; then
    echo "警告: funasr未正确安装，尝试单独安装..."
    pip3 install funasr
fi

# 检查umap-learn版本
if python3 -c "import umap; print(umap.__version__)" &> /dev/null; then
    INSTALLED_UMAP=$(python3 -c "import umap; print(umap.__version__)")
    # 使用Python检查版本是否符合要求
    if python3 -c "import sys; from packaging import version; sys.exit(0 if version.parse('$INSTALLED_UMAP') <= version.parse('0.5.5') else 1)" 2>/dev/null; then
        echo "umap-learn版本兼容，无需重新安装"
    else
        echo "警告: umap-learn版本过高($INSTALLED_UMAP)，可能会引起兼容性问题"
        echo "卸载当前版本并安装兼容版本..."
        pip3 uninstall -y umap-learn
        pip3 install "umap-learn<=0.5.5"
        echo "umap-learn兼容版本安装完成"
    fi
else
    echo "安装umap-learn..."
    pip3 install "umap-learn<=0.5.5"
fi

# 提前安装本地模型
# modelscope 1.10.0 默认安装到~/.cache/modelscope/hub/iic
# 其他版本可能安装到~/.cache/modelscope/hub/models/iic

echo "开始下载语音识别模型..."

echo "正在下载ASR模型: speech_paraformer-large_asr_nat-zh-cn-16k-common-vocab8404-pytorch"
modelscope download --revision v2.0.4 iic/speech_paraformer-large_asr_nat-zh-cn-16k-common-vocab8404-pytorch

echo "正在下载VAD模型: speech_fsmn_vad_zh-cn-16k-common-pytorch"
modelscope download --revision v2.0.4 iic/speech_fsmn_vad_zh-cn-16k-common-pytorch

echo "模型下载完成"

# TODO模型下载完成后移除额外安装的uvicorn fastapi

echo "所有依赖和模型安装完成!"