#pragma once

#include <ros/ros.h>
#include <string>

int simulate_loop(ros::NodeHandle &nh, bool spin_thread = false);

void request_simulation_stop();
