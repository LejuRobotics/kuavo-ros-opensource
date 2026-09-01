export ROBOT_VERSION=53
source devel/setup.zsh
roslaunch humanoid_controllers load_kuavo_real.launch with_pico_gmr:=true pico_enable_viewer:=false joystick_type:=bt2
# roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch with_pico_gmr:=true pico_enable_viewer:=true joystick_type:=bt2