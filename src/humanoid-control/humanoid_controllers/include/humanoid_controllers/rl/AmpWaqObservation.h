#ifndef HUMANOID_AMP_WAQ_OBSERVATION_H
#define HUMANOID_AMP_WAQ_OBSERVATION_H
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <stdexcept>

namespace humanoid_controller {
// Export contract: nine PREVIOUS frames followed by current, oldest first.
// The ONNX already contains the observation normalizer and WAQ encoder.
class AmpWaqObservation {
 public:
  static constexpr int kJoints = 27;
  static constexpr int kCurrent = 91;
  static constexpr int kHistory = 9;
  static constexpr int kInput = kCurrent * (kHistory + 1);
  AmpWaqObservation() : history_(Eigen::VectorXd::Zero(kHistory * kCurrent)) {}
  void reset() { history_.setZero(); }
  Eigen::VectorXd push(const Eigen::VectorXd& current) {
    if (current.size() != kCurrent || !current.allFinite())
      throw std::invalid_argument("AMP WAQ requires 91 finite observation values");
    Eigen::VectorXd input(kInput);
    input << history_, current;
    history_.head((kHistory - 1) * kCurrent) = history_.tail((kHistory - 1) * kCurrent).eval();
    history_.tail(kCurrent) = current;
    return input;
  }
  static Eigen::VectorXd current(const Eigen::Vector3d& angular_velocity,
      const Eigen::Quaterniond& orientation, const Eigen::Vector3d& velocity_command,
      double activity, const Eigen::VectorXd& position, const Eigen::VectorXd& default_position,
      const Eigen::VectorXd& velocity, const Eigen::VectorXd& previous_action) {
    if (position.size() != kJoints || default_position.size() != kJoints ||
        velocity.size() != kJoints || previous_action.size() != kJoints ||
        !orientation.coeffs().allFinite() || orientation.norm() < 1e-6 ||
        !std::isfinite(activity) || activity < 0 || activity > 1)
      throw std::invalid_argument("Invalid AMP WAQ joint, quaternion or activity input");
    Eigen::VectorXd result(kCurrent);
    result << angular_velocity,
        orientation.normalized().conjugate() * Eigen::Vector3d(0, 0, -1),
        velocity_command, activity, position - default_position, velocity, previous_action;
    if (!result.allFinite()) throw std::invalid_argument("Nonfinite AMP WAQ observation");
    return result;
  }
 private:
  Eigen::VectorXd history_;
};
}  // namespace humanoid_controller
#endif
