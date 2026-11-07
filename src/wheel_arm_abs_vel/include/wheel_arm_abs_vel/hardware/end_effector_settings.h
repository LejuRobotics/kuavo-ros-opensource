#pragma once
#include <ros/ros.h>
#include "wheel_arm_abs_vel/end_effector_contract.h"
#include "kuavo_common/common/kuavo_settings.h"
#include <map>
#include <type_traits>
namespace wheel_arm_abs_vel {
template<class Settings>
inline void applyEndEffectorContract(Settings& settings, int version) {
  std::string text;
  if (!ros::param::get("/absolute_velocity_ik/end_effector_contract", text)) return;
  const auto types = kuavo_common::endEffectorContract(text.c_str(), version);
  using E = typename std::decay_t<decltype(settings.hardware_settings.end_effector_type)>::value_type;
  const std::map<std::string,E> values = {{"none",E::none},{"jodell",E::jodell},
    {"qiangnao",E::qiangnao},{"qiangnao_touch",E::qiangnao_touch},
    {"lejuclaw",E::lejuclaw},{"revo2",E::revo2},{"heiman",E::heiman},{"linker_hand",E::linker_hand},{"qibeng",E::qibeng}};
  if(types.size()!=2) throw std::runtime_error("Missing absolute end-effector contract");
  settings.hardware_settings.end_effector_type = {values.at(types[0]),values.at(types[1])};
}
}
