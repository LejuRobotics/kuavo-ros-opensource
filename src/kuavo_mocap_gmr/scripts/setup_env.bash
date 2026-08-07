#!/bin/bash
# Source this file to setup GMR environment
# Usage: source $(rospack find kuavo_mocap_gmr)/scripts/setup_env.bash

# Get package directory
if [ -n "$BASH_SOURCE" ]; then
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
else
    SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
fi
PKG_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# Find virtual environment
if [ -n "$GMR_VENV_PATH" ] && [ -d "$GMR_VENV_PATH" ]; then
    VENV_PATH="$GMR_VENV_PATH"
elif [ -d "/home/zhongxu/work/gmr/venv" ]; then
    VENV_PATH="/home/zhongxu/work/gmr/venv"
elif [ -d "$PKG_DIR/venv" ]; then
    VENV_PATH="$PKG_DIR/venv"
else
    echo "[GMR] Warning: Virtual environment not found"
    echo "[GMR] Please set GMR_VENV_PATH or create venv"
    return 1
fi

# Activate virtual environment
echo "[GMR] Activating virtual environment: $VENV_PATH"
source "$VENV_PATH/bin/activate"

# Add gmr_core to PYTHONPATH
GMR_CORE_PATH="$PKG_DIR/src/kuavo_mocap_gmr/gmr_core"
export PYTHONPATH="$GMR_CORE_PATH:$PYTHONPATH"
echo "[GMR] Added to PYTHONPATH: $GMR_CORE_PATH"

echo "[GMR] Environment setup complete"
echo "[GMR] Python: $(which python3)"
