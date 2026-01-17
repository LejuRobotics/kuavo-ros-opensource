#!/bin/bash

# Kuavo Wheel Joystick Control Tool

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Source ROS environment
if [ -f "/opt/ros/noetic/setup.bash" ]; then
    source /opt/ros/noetic/setup.bash
fi
# Source workspace environment if available
if [ -f "${SCRIPT_DIR}/../../../devel/setup.bash" ]; then
    source "${SCRIPT_DIR}/../../../devel/setup.bash"
fi

# Parameters (can be overridden via env vars)
JOY_LAUNCH_PACKAGE=${JOY_LAUNCH_PACKAGE:-"joy"}
JOY_LAUNCH_FILE=${JOY_LAUNCH_FILE:-"joy_node.launch"}
JOY_DEVICE=${JOY_DEVICE:-"/dev/input/js0"}

# Topics and scales
CMD_VEL_TOPIC=${CMD_VEL_TOPIC:-"/cmd_vel"}
JOY_TOPIC=${JOY_TOPIC:-"/joy"}
LINEAR_AXIS_INDEX=${LINEAR_AXIS_INDEX:-1}
ANGULAR_AXIS_INDEX=${ANGULAR_AXIS_INDEX:-0}
LINEAR_SCALE=${LINEAR_SCALE:-0.8}
ANGULAR_SCALE=${ANGULAR_SCALE:-1.2}
DEADZONE=${DEADZONE:-0.05}

# Start processes and keep PIDs for cleanup
PIDS=()

cleanup() {
    echo "\n[joy_wheel_control_tool] Stopping..."
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" >/dev/null 2>&1; then
            kill "$pid" || true
        fi
    done
    wait || true
    echo "[joy_wheel_control_tool] Stopped."
}

trap cleanup INT TERM EXIT

# Launch joystick driver
echo "[joy_wheel_control_tool] Launching joystick driver: $JOY_LAUNCH_PACKAGE $JOY_LAUNCH_FILE"
roslaunch "$JOY_LAUNCH_PACKAGE" "$JOY_LAUNCH_FILE" &
PIDS+=($!)

sleep 1

# Start joy -> cmd_vel converter
echo "[joy_wheel_control_tool] Starting joy_to_cmd_vel.py joy=$JOY_TOPIC -> cmd_vel=$CMD_VEL_TOPIC"
python3 $SCRIPT_DIR/joy_to_cmd_vel.py \
  _joy_topic:=$JOY_TOPIC \
  _cmd_vel_topic:=$CMD_VEL_TOPIC \
  _linear_axis_index:=$LINEAR_AXIS_INDEX \
  _angular_axis_index:=$ANGULAR_AXIS_INDEX \
  _linear_scale:="$LINEAR_SCALE" \
  _angular_scale:="$ANGULAR_SCALE" \
  _deadzone:=$DEADZONE &
PIDS+=($!)

sleep 1

if rosnode list 2>/dev/null | grep -qE '^/kuavo_bridge_node'; then
    echo "kuavo_bridge_node 正在跑"
else
    # Start wheel bridge script (handles cmd_vel forwarding between masters)
    echo "[joy_wheel_control_tool] Starting wheel bridge via start_wheel_bridge.sh"
    "$SCRIPT_DIR/start_wheel_bridge.sh" &
    PIDS+=($!)
fi


echo "[joy_wheel_control_tool] All processes started. Press Ctrl+C to stop."

# Wait for background processes
wait
