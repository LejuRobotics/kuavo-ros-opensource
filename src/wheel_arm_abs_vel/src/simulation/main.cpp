#include <ros/ros.h>
#include <iostream>
int simulate_loop(ros::NodeHandle&, bool);
namespace { ros::AsyncSpinner* active_spinner = nullptr; }
void absolute_stop_callbacks() {
  if (active_spinner) active_spinner->stop();
}
int main(int argc, char** argv) {
  ros::init(argc, argv, "absolute_mujoco", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;
  bool real = true;
  nh.getParam("/real", real);
  if (real) {
    ROS_FATAL("Absolute MuJoCo requires /real=false");
    return 2;
  }
  ros::AsyncSpinner spinner(1);
  active_spinner = &spinner;
  spinner.start();
  const int result = simulate_loop(nh, false);
  spinner.stop();
  active_spinner = nullptr;
  std::cout << "[AbsoluteSimulation] clean exit" << std::endl;
  return result;
}
