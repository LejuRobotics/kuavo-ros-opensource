#!/bin/bash
# Wrapper script to run GMR node with Python 3.10 virtual environment
# This is necessary because GMR's .so files are compiled for Python 3.10

set -e

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# Path to the Python 3.10 virtual environment
# Priority: 1. GMR_VENV_PATH env var  2. venv in package  3. gmr_venv in package
if [ -n "$GMR_VENV_PATH" ] && [ -d "$GMR_VENV_PATH" ]; then
    VENV_PATH="$GMR_VENV_PATH"
elif [ -d "$PKG_DIR/venv" ]; then
    VENV_PATH="$PKG_DIR/venv"
elif [ -d "$PKG_DIR/gmr_venv" ]; then
    VENV_PATH="$PKG_DIR/gmr_venv"
else
    echo "[GMR] Error: Python 3.10 virtual environment not found!"
    echo "[GMR] Please create it with one of the following commands:"
    echo ""
    echo "  # Option 1: Create venv in package directory"
    echo "  cd $PKG_DIR && bash scripts/setup_venv.sh"
    echo ""
    echo "  # Option 2: Set custom path via environment variable"
    echo "  export GMR_VENV_PATH=/path/to/your/python310/venv"
    echo ""
    exit 1
fi

echo "[GMR] Using virtual environment: $VENV_PATH"

# Activate virtual environment
source "$VENV_PATH/bin/activate"

# Add paths to PYTHONPATH for module imports
# - gmr_core: for direct imports
# - src/kuavo_mocap_gmr: for general_motion_retargeting and kuavo_optitrack_runtime symlinks
GMR_CORE_PATH="$PKG_DIR/src/kuavo_mocap_gmr/gmr_core"
GMR_SRC_PATH="$PKG_DIR/src/kuavo_mocap_gmr"
export PYTHONPATH="$GMR_CORE_PATH:$GMR_SRC_PATH:$PYTHONPATH"

# Get the Python script to run (first argument)
PYTHON_SCRIPT="$1"
shift

# If catkin wrapper is passed, extract the actual script path and run directly
if [ -f "$PYTHON_SCRIPT" ] && grep -q "python_script = " "$PYTHON_SCRIPT" 2>/dev/null; then
    # Extract the actual script path from catkin wrapper
    ACTUAL_SCRIPT=$(grep "python_script = " "$PYTHON_SCRIPT" | head -1 | sed "s/python_script = '//" | sed "s/'//")
    echo "[GMR] Detected catkin wrapper, using actual script: $ACTUAL_SCRIPT"
    PYTHON_SCRIPT="$ACTUAL_SCRIPT"
fi

echo "[GMR] Running: python3 $PYTHON_SCRIPT $@"

# Run with the virtual environment's Python
exec python3 "$PYTHON_SCRIPT" "$@"
