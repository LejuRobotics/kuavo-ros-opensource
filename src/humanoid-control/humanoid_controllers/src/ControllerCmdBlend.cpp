#include "humanoid_controllers/ControllerCmdBlend.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <ros/ros.h>
#include "humanoid_interface/common/TopicLogger.h"

namespace humanoid_controller
{
namespace
{
double quinticBlend(double alpha)
{
  alpha = std::clamp(alpha, 0.0, 1.0);
  return alpha * alpha * alpha * (10.0 + alpha * (-15.0 + 6.0 * alpha));
}

double blendScalar(double start, double target, double alpha)
{
  return start + (target - start) * alpha;
}

void selfCheckOnce()
{
  static bool done = false;
  if (done)
    return;
  // ponytail: ceiling — fixed-duration source-mode hold; ankle still flips → defer torque mode
  auto check = [](bool ok, const char* msg) {
    if (!ok)
    {
      ROS_FATAL("[Switch/CmdBlend] self-check failed: %s", msg);
      throw std::runtime_error(msg);
    }
  };
  check(std::abs(quinticBlend(0.0)) < 1e-12, "quintic(0)");
  check(std::abs(quinticBlend(1.0) - 1.0) < 1e-12, "quintic(1)");
  check(std::abs(blendScalar(0.0, 10.0, quinticBlend(0.25)) - 2.5) > 0.5, "quintic!=linear");
  done = true;
}

bool cmdSizeOk(const kuavo_msgs::jointCmd& cmd, size_t n)
{
  return cmd.joint_q.size() >= n && cmd.joint_v.size() >= n && cmd.tau.size() >= n &&
         cmd.joint_kp.size() >= n && cmd.joint_kd.size() >= n && cmd.control_modes.size() >= n;
}
}  // namespace

void ControllerCmdBlend::start(double time, const kuavo_msgs::jointCmd& source, size_t body_joint_count,
                               const char* tag)
{
  selfCheckOnce();
  if (!cmdSizeOk(source, body_joint_count))
  {
    ROS_WARN("[Switch/%s] skip: incomplete source size", tag);
    stop();
    return;
  }
  source_ = source;
  start_time_ = time;
  tag_ = tag;
  active_ = true;
  ROS_INFO("[Switch/%s] blend %.2fs", tag, std::max(duration_, 1e-3));
}

void ControllerCmdBlend::apply(double time, kuavo_msgs::jointCmd& joint_cmd, size_t body_joint_count,
                               ocs2::humanoid::TopicLogger* logger)
{
  if (!active_)
    return;

  const size_t n = std::min({body_joint_count, joint_cmd.joint_q.size(), joint_cmd.joint_v.size(),
                             joint_cmd.tau.size(), joint_cmd.joint_kp.size(), joint_cmd.joint_kd.size(),
                             joint_cmd.control_modes.size()});
  if (n == 0)
    return;

  const double duration = std::max(duration_, 1e-3);
  const double alpha = std::clamp((time - start_time_) / duration, 0.0, 1.0);
  const double blend = quinticBlend(alpha);
  double max_d_raw = 0.0, max_d_out = 0.0;

  for (size_t i = 0; i < n; ++i)
  {
    max_d_raw = std::max(max_d_raw, std::abs(joint_cmd.tau[i] - source_.tau[i]));
    const double out_tau = blendScalar(source_.tau[i], joint_cmd.tau[i], blend);
    max_d_out = std::max(max_d_out, std::abs(out_tau - source_.tau[i]));
    joint_cmd.joint_q[i] = blendScalar(source_.joint_q[i], joint_cmd.joint_q[i], blend);
    joint_cmd.joint_v[i] = blendScalar(source_.joint_v[i], joint_cmd.joint_v[i], blend);
    joint_cmd.tau[i] = out_tau;
    joint_cmd.joint_kp[i] = blendScalar(source_.joint_kp[i], joint_cmd.joint_kp[i], blend);
    joint_cmd.joint_kd[i] = blendScalar(source_.joint_kd[i], joint_cmd.joint_kd[i], blend);
    joint_cmd.control_modes[i] = (alpha < 1.0) ? source_.control_modes[i] : joint_cmd.control_modes[i];
  }

  if (logger)
  {
    logger->publishValue("/humanoid_controller/cmd_blend/alpha", alpha);
    logger->publishValue("/humanoid_controller/cmd_blend/max_tau_delta_raw", max_d_raw);
    logger->publishValue("/humanoid_controller/cmd_blend/max_tau_delta_blended", max_d_out);
  }

  if (alpha >= 1.0)
    active_ = false;
}

}  // namespace humanoid_controller
