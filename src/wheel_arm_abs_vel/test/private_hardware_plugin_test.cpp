// Load and construct the plugin, without calling Nodelet::init/onInit. No bus I/O.
#include <pluginlib/class_loader.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <iostream>
int main(int argc,char** argv) {
  ros::init(argc,argv,"private_hardware_plugin_test",ros::init_options::AnonymousName);
  try {
    pluginlib::ClassLoader<nodelet::Nodelet> loader("nodelet","nodelet::Nodelet");
    auto plugin=loader.createInstance("wheel_arm_abs_vel/AbsoluteHardwareNodelet");
    plugin.reset();
    std::cout<<"PASS private plugin loads and destroys without initialization\n";
    return 0;
  } catch(const std::exception& e) { std::cerr<<e.what()<<'\n';return 1; }
}
