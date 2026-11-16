#include <fstream>
#include <iomanip>
#include <iostream>
#include "humanoid_controllers/ArmTrajectoryInterpolator.h"

// Offline command replay: timestamped q/dq inputs, never publishes ROS commands.
int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "Usage: arm_command_bag_replay input.txt output.txt\n";
    return 1;
  }
  ros::Time::init();
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Error);
  std::ifstream input(argv[1]);
  std::ofstream output(argv[2]);
  if (!input || !output) return 1;
  using humanoidController_wheel_wbc::ArmTrajectoryInterpolator;
  ArmTrajectoryInterpolator before, after;
  ArmTrajectoryInterpolator::Config config;
  config.controlCycleSec = 0.001;
  config.kalmanVLimit = 30.0;
  config.kalmanProcessAccCap = 400.0;
  config.kalmanProcessQCap = 16000.0;
  before.configure(config);
  config.boundedCommandTracking = true;
  after.configure(config);
  ArmTrajectoryInterpolator::ModeFlags mode;
  mode.useArmTrajectoryControl = true;
  Eigen::VectorXd initial(14), q(14), v(14);
  for (int i = 0; i < 14; ++i) if (!(input >> initial[i])) return 1;
  double t, received;
  bool first = true;
  output << std::setprecision(17);
  while (input >> t >> received) {
    for (int i = 0; i < 14; ++i) if (!(input >> q[i])) return 1;
    for (int i = 0; i < 14; ++i) if (!(input >> v[i])) return 1;
    ros::Time now, stamp;
    now.fromSec(t);
    stamp.fromSec(received);
    if (first) {
      before.compute(now, mode, initial);
      after.compute(now, mode, initial);
      first = false;
    }
    before.ingestRawTarget(stamp, q, v);
    after.ingestRawTarget(stamp, q, v);
    const auto old = before.compute(now, mode, initial);
    const auto fixed = after.compute(now, mode, initial);
    output << t;
    for (const auto* vec : {&old.smoothQ, &old.smoothV, &fixed.smoothQ, &fixed.smoothV}) {
      if (vec->size() != 14 || !vec->allFinite()) return 1;
      for (int i = 0; i < 14; ++i) output << ' ' << (*vec)[i];
    }
    output << '\n';
  }
  return input.eof() ? 0 : 1;
}
