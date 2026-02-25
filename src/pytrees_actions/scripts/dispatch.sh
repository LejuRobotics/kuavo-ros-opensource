#!/bin/bash
source /home/lab/embodied/devel/setup.bash

# Start the first Python script and keep it running
python3 /home/lab/embodied/src/pytrees_actions/scripts/dispatch_robot_info_server.py &
PID1=$!

# Start the second Python script and keep it running  
python3 /home/lab/embodied/src/pytrees_actions/scripts/pub_robot_odom.py &
PID2=$!

# Wait for both processes (they should run indefinitely)
wait $PID1
wait $PID2
