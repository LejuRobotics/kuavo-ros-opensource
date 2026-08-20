#!/bin/bash
# Wrapper script to run pico_comm_minimal.py
# This script is a simple wrapper that doesn't need venv since pico_comm_minimal
# only uses standard ROS packages (rospy, tf, protobuf)
# It can run in the system Python environment.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(dirname "$SCRIPT_DIR")"

# Add kuavo_gmr to PYTHONPATH for protobuf imports
export PYTHONPATH="$PACKAGE_DIR/kuavo_gmr:$PACKAGE_DIR:$PYTHONPATH"

exec python3 "$PACKAGE_DIR/kuavo_gmr/pico_comm_minimal.py" "$@"
