// Minimal biped controller stubs when Drake is unavailable (e.g. AGX Orin aarch64 wheel-arm builds).
#include "humanoid_controllers/humanoidController.h"
#include <ros/ros.h>

namespace humanoid_controller
{

humanoidController::~humanoidController() = default;

bool humanoidController::init(HybridJointInterface* /*robot_hw*/, ros::NodeHandle& /*controller_nh*/,
                              bool /*is_nodelet_node*/)
{
  ROS_ERROR("[humanoidController] Biped MPC controller is not available without Drake on this platform.");
  return false;
}

bool humanoidController::preUpdate(const ros::Time& /*time*/) { return true; }

void humanoidController::update(const ros::Time& /*time*/, const ros::Duration& /*period*/) {}

void humanoidController::starting(const ros::Time& /*time*/) {}

void humanoidController::waitForNextCycle() {}

double humanoidController::getControlFrequency() const { return 500.0; }

void humanoidController::setupStateEstimate(const std::string& /*taskFile*/, bool /*verbose*/,
                                            const std::string& /*referenceFile*/)
{
}

void humanoidController::updateStateEstimation(const ros::Time& /*time*/, bool /*is_init*/) {}

void humanoidController::setupHumanoidInterface(const std::string& /*taskFile*/, const std::string& /*urdfFile*/,
                                                const std::string& /*referenceFile*/, const std::string& /*gaitFile*/,
                                                bool /*verbose*/, RobotVersion /*rb_version*/)
{
}

void humanoidController::setupMpc() {}

void humanoidController::setupMrt() {}

void humanoidCheaterController::setupStateEstimate(const std::string& /*taskFile*/, bool /*verbose*/,
                                                   const std::string& /*referenceFile*/)
{
}

void humanoidKuavoController::setupStateEstimate(const std::string& /*taskFile*/, bool /*verbose*/,
                                                 const std::string& /*referenceFile*/)
{
}

}  // namespace humanoid_controller
