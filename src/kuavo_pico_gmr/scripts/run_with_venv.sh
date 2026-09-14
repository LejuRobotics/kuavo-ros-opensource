#!/bin/bash
# Wrapper script to run PICO GMR nodes with Python 3.10 virtual environment
# This is necessary because GMR's .so files are compiled for Python 3.10
# Analogous to kuavo_mocap_gmr/scripts/run_with_venv.sh

set -e

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# Path to the Python 3.10 virtual environment
# Priority: 1. PICO_GMR_VENV_PATH env var  2. gmr_venv in package  3. venv in package
if [ -n "$PICO_GMR_VENV_PATH" ] && [ -d "$PICO_GMR_VENV_PATH" ]; then
    VENV_PATH="$PICO_GMR_VENV_PATH"
elif [ -d "$PKG_DIR/gmr_venv" ]; then
    VENV_PATH="$PKG_DIR/gmr_venv"
elif [ -d "$PKG_DIR/venv" ]; then
    VENV_PATH="$PKG_DIR/venv"
else
    echo "[PICO GMR] Error: Python 3.10 virtual environment not found!"
    echo "[PICO GMR] Please create it with one of the following commands:"
    echo ""
    echo "  # Option 1: Create venv in package directory"
    echo "  cd $PKG_DIR && bash setup_venv.sh"
    echo ""
    echo "  # Option 2: Set custom path via environment variable"
    echo "  export PICO_GMR_VENV_PATH=/path/to/your/python310/venv"
    echo ""
    exit 1
fi

echo "[PICO GMR] Using virtual environment: $VENV_PATH"

# Activate virtual environment
source "$VENV_PATH/bin/activate"

# Add paths to PYTHONPATH for module imports
# - kuavo_gmr: GMR core module (in package root)
# - scripts: for gmr_ros_node_dev.py and other script-level modules
export PYTHONPATH="$PKG_DIR:$PKG_DIR/kuavo_gmr:$PKG_DIR/scripts:$PYTHONPATH"

# Get the Python script to run (first argument)
PYTHON_SCRIPT="$1"
shift

# If catkin wrapper is passed, extract the actual script path and run directly
if [ -f "$PYTHON_SCRIPT" ] && grep -q "python_script = " "$PYTHON_SCRIPT" 2>/dev/null; then
    # Extract the actual script path from catkin wrapper
    ACTUAL_SCRIPT=$(grep "python_script = " "$PYTHON_SCRIPT" | head -1 | sed "s/.*python_script = '\(.*\)'/\1/")
    if [ -n "$ACTUAL_SCRIPT" ] && [ -f "$ACTUAL_SCRIPT" ]; then
        echo "[PICO GMR] Running actual script: $ACTUAL_SCRIPT"
        exec python3 "$ACTUAL_SCRIPT" "$@"
    fi
fi

echo "[PICO GMR] Running: $PYTHON_SCRIPT $@"
exec python3 "$PYTHON_SCRIPT" "$@"
