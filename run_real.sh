# export ROBOT_VERSION=54
# source devel/setup.bash
# roslaunch humanoid_controllers load_kuavo_real.launch with_mocap_gmr:=true gmr_use_sim:=false gmr_server_ip:=10.42.0.205 gmr_client_ip:=10.42.0.1 joystick_type:=bt2

export ROBOT_VERSION=53
source devel/setup.bash
roslaunch humanoid_controllers load_kuavo_real.launch with_pico_gmr:=true pico_enable_viewer:=false joystick_type:=bt2