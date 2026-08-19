# 机器人版本
export ROBOT_VERSION=53
source devel/setup.bash

# 离线
# roslaunch humanoid_controllers load_kuavo_real.launch joystick_type:=bt2

# MoCap
# roslaunch humanoid_controllers load_kuavo_real.launch with_mocap_gmr:=true gmr_use_sim:=false gmr_server_ip:=10.42.0.205 gmr_client_ip:=10.42.0.1 joystick_type:=bt2

# PICO
roslaunch humanoid_controllers load_kuavo_real.launch with_pico_gmr:=true pico_enable_viewer:=false joystick_type:=bt2

# Xsens
# roslaunch humanoid_controllers load_kuavo_real_with_xsense_gmr.launch with_xsense_gmr:=true xsense_enable_viewer:=false joystick_type:=bt2 xsense_pose_queue_stats_hz:=10. #cali:=true cali_arm:=true


