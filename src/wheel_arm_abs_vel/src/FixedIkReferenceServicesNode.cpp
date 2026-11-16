// Retain the existing fixed-IK control services without an MPC optimizer.
#include <humanoid_wheel_interface/HumanoidWheelInterface.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ros/ros.h>

#include <exception>
#include <string>

int main(int argc, char** argv) {
  ros::init(argc, argv, "fixed_ik_reference_services");
  ros::NodeHandle node;
  std::string task_file, library_folder, urdf_file;
  if (!node.getParam("/taskFile", task_file) ||
      !node.getParam("/libFolder", library_folder) ||
      !node.getParam("/urdfFile", urdf_file)) {
    ROS_FATAL("Fixed IK reference services require taskFile, libFolder and urdfFile");
    return 1;
  }
  try {
    // Use the original ReferenceManager, including its enable-control gates,
    // initial pose and arm-mode services. No SqpMpc/DDP/MPC_ROS_Interface exists
    // here, so this process cannot start solver workers or publish MPC policies.
    ocs2::mobile_manipulator::HumanoidWheelInterface interface(
        task_file, library_folder, urdf_file, true);
    ocs2::RosReferenceManager reference_manager(
        "mobile_manipulator", interface.getReferenceManagerPtr());
    reference_manager.subscribe(node);
    ROS_INFO("[FixedVelocityIK] Reference services ready; MPC optimizer absent");
    // setupSubscriptions already owns the ROS AsyncSpinner.
    ros::waitForShutdown();
  } catch (const std::exception& error) {
    ROS_FATAL("Fixed IK reference services failed: %s", error.what());
    return 1;
  }
  return 0;
}
