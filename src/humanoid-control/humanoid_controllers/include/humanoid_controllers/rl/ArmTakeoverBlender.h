#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>

namespace humanoid_controller
{

class ArmTakeoverBlender
{
public:
  void configure(bool enabled, double blend_duration)
  {
    enabled_ = enabled;
    blend_duration_ = std::max(0.0, blend_duration);
  }

  void reset()
  {
    active_ = false;
    start_time_sec_ = 0.0;
    start_arm_action_.resize(0);
  }

  void start(double start_time_sec, int arm_action_size)
  {
    if (!enabled_ || blend_duration_ <= 0.0 || arm_action_size == 0)
    {
      reset();
      return;
    }

    start_time_sec_ = start_time_sec;
    start_arm_action_ = Eigen::VectorXd::Zero(arm_action_size);
    active_ = true;
  }

  bool isActive() const { return active_; }

  Eigen::VectorXd blendArmAction(double time_sec,
                                 const Eigen::VectorXd& rl_arm_action)
  {
    if (!active_ || !enabled_)
    {
      return rl_arm_action;
    }

    if (rl_arm_action.size() == 0 ||
        rl_arm_action.size() != start_arm_action_.size())
    {
      active_ = false;
      return rl_arm_action;
    }

    const double alpha = computeBlendFactor(time_sec);
    return (1.0 - alpha) * start_arm_action_ + alpha * rl_arm_action;
  }

private:
  double computeBlendFactor(double time_sec)
  {
    if (blend_duration_ <= 0.0)
    {
      active_ = false;
      return 1.0;
    }

    const double raw_alpha = (time_sec - start_time_sec_) / blend_duration_;
    const double alpha = std::clamp(raw_alpha, 0.0, 1.0);
    if (alpha >= 1.0)
    {
      active_ = false;
      return 1.0;
    }

    // Smoothstep: zero velocity at both endpoints.
    return alpha * alpha * (3.0 - 2.0 * alpha);
  }

  bool enabled_{false};
  bool active_{false};
  double blend_duration_{0.0};
  double start_time_sec_{0.0};
  Eigen::VectorXd start_arm_action_;
};

}  // namespace humanoid_controller
