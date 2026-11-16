#include "wheel_arm_abs_vel/hardware/end_effector_settings.h"
#include <cassert>
#include <vector>

std::string ros::param::contract;

struct TestSettings {
  struct { std::vector<EndEffectorType> end_effector_type; } hardware_settings;
};

int main() {
  TestSettings settings;
  ros::param::contract = R"({"robot_version":63,"types":["qibeng","qibeng"],"real":true,"source":"explicit ee_type"})";
  wheel_arm_abs_vel::applyEndEffectorContract(settings, 63);
  assert(settings.hardware_settings.end_effector_type.size() == 2);
  assert(settings.hardware_settings.end_effector_type[0] == EndEffectorType::qibeng);
  assert(settings.hardware_settings.end_effector_type[1] == EndEffectorType::qibeng);
}
