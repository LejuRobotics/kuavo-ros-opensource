#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <thread>
#include <iostream>
#include "mujoco_node.h"
class MujocoNodelet : public nodelet::Nodelet
{
public:
  virtual void onInit()
  {
    NODELET_INFO("Initializing HumanoidControllerNodelet nodelet...");
    control_thread = std::thread(&MujocoNodelet::simloop, this);
    if (!control_thread.joinable())
    {
      std::cerr << "Failed to start control thread." << std::endl;
      exit(1);
    }
    NODELET_INFO("HumanoidControllerNodelet nodelet initialized.");
  }

private:
  ros::NodeHandle nh;
  std::thread control_thread;
  void simloop()
  {
    nh = getNodeHandle();
    simulate_loop(nh, false);
  }
};

PLUGINLIB_EXPORT_CLASS(MujocoNodelet, nodelet::Nodelet)
