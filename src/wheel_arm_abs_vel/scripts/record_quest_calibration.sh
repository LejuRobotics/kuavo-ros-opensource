#!/usr/bin/env bash
# No controller, mapper, glove driver or hardware node is launched.
set -euo pipefail
if [[ $# -lt 1 ]]; then
  echo "Usage: $0 QUEST_IP [roslaunch arguments...]" >&2
  exit 2
fi
quest_ip=$1
shift
# A Quest application streams poses to one receiver. Two receivers can keep
# answering handshakes while starving each other of the actual pose stream.
if pgrep -f '(^|/)(monitor_quest3_velocity[.]py|monitor_quest3[.]py|monitor_quest3_cpp)( |$)' >/dev/null; then
  echo "A Quest receiver is already running. Record its pose topic with rosbag instead of starting a second receiver." >&2
  exit 2
fi
unset ROS_HOSTNAME
export ROS_IP=127.0.0.1
export ROS_MASTER_URI=http://127.0.0.1:11329
mkdir -p "$HOME/.ros"
exec roslaunch --port 11329 wheel_arm_abs_vel record_quest_calibration.launch "ip_address:=$quest_ip" "$@"
