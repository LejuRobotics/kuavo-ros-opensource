#pragma once

#include <cstddef>
#include <kuavo_msgs/jointCmd.h>
#include "humanoid_controllers/rl/rl_switch_config.h"

namespace ocs2
{
namespace humanoid
{
class TopicLogger;
}
}

namespace humanoid_controller
{

/** RL→MPC：仅 quintic 混合 q/v/tau；kp/kd 与 control_mode 同源绑定，不插值。 */
class ControllerCmdBlend
{
public:
  void setDuration(double seconds) { duration_ = seconds; }

  void start(double time, const kuavo_msgs::jointCmd& source, size_t body_joint_count, const char* tag);
  void apply(double time, kuavo_msgs::jointCmd& joint_cmd, size_t body_joint_count,
             ocs2::humanoid::TopicLogger* logger);
  void stop() { active_ = false; }
  bool active() const { return active_; }

private:
  bool active_{false};
  double start_time_{0.0};
  double duration_{CONTROLLER_CMD_BLEND_DURATION_DEFAULT};
  const char* tag_{""};
  kuavo_msgs::jointCmd source_;
};

}  // namespace humanoid_controller
