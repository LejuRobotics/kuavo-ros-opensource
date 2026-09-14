#pragma once

#include <algorithm>

namespace humanoid_controller
{

class StanceDwellGate
{
public:
  bool update(bool low_speed, double time_sec, double dwell_duration_sec)
  {
    if (!low_speed)
    {
      reset();
      return false;
    }

    if (!active_)
    {
      low_speed_start_time_sec_ = time_sec;
      active_ = true;
      return dwell_duration_sec <= 0.0;
    }

    return time_sec - low_speed_start_time_sec_ >=
           std::max(0.0, dwell_duration_sec);
  }

  void reset()
  {
    active_ = false;
    low_speed_start_time_sec_ = 0.0;
  }

private:
  bool active_{false};
  double low_speed_start_time_sec_{0.0};
};

}  // namespace humanoid_controller
