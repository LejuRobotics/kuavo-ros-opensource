#!/bin/bash

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "Starting ROS Noetic container with workspace mounted..."
echo "Workspace: $SCRIPT_DIR"
echo ""

xhost +local:docker

docker run -it --rm --privileged \
  --network host \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /dev:/dev \
  -v /etc/udev:/etc/udev:ro \
  -v "$SCRIPT_DIR":/catkin_ws \
  -w /catkin_ws \
  kuavo_mpc_wbc_img:0.5.4
