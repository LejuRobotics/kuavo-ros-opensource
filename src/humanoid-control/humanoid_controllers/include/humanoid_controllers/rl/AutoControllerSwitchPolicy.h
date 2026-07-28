#pragma once

#include <string>

namespace humanoid_controller
{
  inline bool isAutoWalkingSwitchSourceAllowed(const std::string& current_controller,
                                               const std::string& manipulation_controller,
                                               const std::string& walking_controller)
  {
    if (walking_controller.empty())
    {
      return false;
    }
    return !manipulation_controller.empty() && current_controller == manipulation_controller;
  }

  inline bool isWalkingCommandExecutionAllowedByAutoSwitch(const std::string& current_controller,
                                                           const std::string& manipulation_controller,
                                                           const std::string& walking_controller)
  {
    if (manipulation_controller.empty() || walking_controller.empty())
    {
      return true;
    }
    if (manipulation_controller == walking_controller)
    {
      return true;
    }
    return current_controller != manipulation_controller;
  }
}  // namespace humanoid_controller
