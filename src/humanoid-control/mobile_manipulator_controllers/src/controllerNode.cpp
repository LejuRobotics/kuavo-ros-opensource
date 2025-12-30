#include "mobile_manipulator_controllers/mobileManipulatorController.h"

using namespace mobile_manipulator_controller;

int main(int argc, char **argv)
{
  const std::string robotName = "mobile_manipulator";

  // Initialize ros node
  ros::init(argc, argv, robotName + "_mpc");
  ros::NodeHandle nodeHandle;

  int frequency = 100;
  // Initialize controller
  MobileManipulatorController controller;
  controller.init(nodeHandle, frequency);
  // controller.starting();

  ros::Rate loopRate(frequency);
  while(ros::ok())
  {
    controller.update();
    ros::spinOnce();
    loopRate.sleep();
  }
  return 0;
}