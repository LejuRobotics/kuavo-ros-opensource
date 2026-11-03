#!/usr/bin/env bash
set -euo pipefail
exec rosrun wheel_arm_abs_vel direct_tcp_quest_node "$@"
