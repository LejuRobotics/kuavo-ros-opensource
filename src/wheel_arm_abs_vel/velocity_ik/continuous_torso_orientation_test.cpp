#include "continuous_torso_orientation.h"
#include <Eigen/Geometry>
#include <cassert>
#include <iostream>
#include <limits>

namespace {
constexpr double pi = 3.14159265358979323846;
constexpr double deg = pi / 180.0;
Eigen::Matrix3d rotation(double roll, double pitch, double yaw) {
  return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
}
void close(double actual, double expected, double tolerance = 1e-9) {
  if (std::abs(actual - expected) > tolerance) {
    std::cerr << "actual=" << actual / deg << " expected=" << expected / deg
              << " tolerance=" << tolerance / deg << " deg\n";
    assert(false);
  }
}
}

int main() {
  using velocity_ik::ContinuousTorsoOrientation;
  double old_yaw_jump = 0.0;
  for (double roll : {0.0, 15.0 * deg, -20.0 * deg}) {
    for (double sign : {-1.0, 1.0}) {
      ContinuousTorsoOrientation observation;
      double expected_yaw = .01 * deg;
      for (int i = 0; i <= 2400; ++i) {
        const double pitch = sign * (i <= 1200 ? i : 2400 - i) * .1 * deg;
        const auto r = rotation(roll, pitch, .01 * deg);
        observation.update(r, 10.0 + i * .01);
        expected_yaw = ContinuousTorsoOrientation::nearest(
            velocity_ik::torsoHeading(r).yaw, i == 0 ? 0.0 : expected_yaw);
        close(observation.rpy.y(), pitch);
        close(observation.rpy.z(), expected_yaw);
        // A pure forward bow must not copy the singular roll split into yaw.
        assert(std::abs(observation.rpy.z() - .01 * deg) < 25.0 * deg);
        assert(observation.pitch_yaw_rate.allFinite());
        if (roll == 0.0) assert(std::abs(observation.pitch_yaw_rate.y()) < 1e-8);
        const double old_yaw = std::atan2(r(1, 0), r(0, 0));
        old_yaw_jump = std::max(old_yaw_jump, std::abs(old_yaw - .01 * deg));
      }
    }
  }
  assert(old_yaw_jump > 179.0 * deg);

  // Turning must remain observable even at vertical pitch. Seed roll before
  // entering the singularity; exact-vertical yaw and roll cannot be separated.
  for (double pitch : {60.0 * deg, 90.0 * deg, -90.0 * deg, 110.0 * deg}) {
    ContinuousTorsoOrientation observation;
    observation.update(rotation(0, 0, 0), 1.0);
    for (int i = 1; i <= 1100; ++i)
      observation.update(rotation(0, pitch * i / 1100., 0), 1. + i * .01);
    for (int i = 1; i <= 600; ++i) {
      observation.update(rotation(0, pitch, i * .1 * deg), 12. + i * .01);
      close(observation.rpy.z(), i * .1 * deg);
      close(observation.pitch_yaw_rate.y(), 10.0 * deg, 1e-8);
    }
  }

  // Side lean uses the forward/left heading, unwrapped across +/-180 degrees.
  ContinuousTorsoOrientation combined;
  double expected_combined_yaw = 0.0;
  for (int i = 0; i < 2000; ++i) {
    const double roll = 20 * deg * std::sin(i * .009);
    const double pitch = 70 * deg * std::sin(i * .005);
    const double yaw = i * .2 * deg;
    const auto r = rotation(roll, pitch, yaw);
    combined.update(r, i * .01);
    expected_combined_yaw = ContinuousTorsoOrientation::nearest(
        velocity_ik::torsoHeading(r).yaw, i == 0 ? 0.0 : expected_combined_yaw);
    close(combined.rpy.x(), roll); close(combined.rpy.y(), pitch);
    close(combined.rpy.z(), expected_combined_yaw);
    if (std::abs(pitch) <= 60.0 * deg) close(combined.rpy.z(), yaw);
  }

  // Simultaneous roll/yaw at vertical cannot be uniquely decomposed. Check
  // smooth continuation and recovery, without claiming zero error there.
  ContinuousTorsoOrientation crossing;
  double max_combined_error = 0.0, max_combined_step = 0.0;
  double expected_crossing_yaw = 0.0;
  for (int i = 0; i <= 2400; ++i) {
    const double pitch = (i <= 1200 ? i : 2400 - i) * .1 * deg;
    const double roll = 15 * deg * std::sin(i * .003);
    const double yaw = 30 * deg * std::sin(i * .002);
    const auto r = rotation(roll, pitch, yaw);
    const double old = crossing.rpy.z();
    crossing.update(r, i * .01);
    expected_crossing_yaw = ContinuousTorsoOrientation::nearest(
        velocity_ik::torsoHeading(r).yaw, i == 0 ? 0.0 : expected_crossing_yaw);
    close(crossing.rpy.z(), expected_crossing_yaw);
    max_combined_error = std::max(max_combined_error, std::abs(crossing.rpy.z() - yaw));
    if (i) max_combined_step = std::max(max_combined_step, std::abs(crossing.rpy.z() - old));
    if (std::abs(pitch) <= 60.0 * deg) close(crossing.rpy.z(), yaw);
  }
  assert(max_combined_error < 20.0 * deg);
  assert(max_combined_step < 1.0 * deg);

  // Tiny physical orientation noise near vertical used to become a pi heading
  // switch. Check both sides of 90 degrees and bound the resulting yaw/rate.
  double max_noise_yaw = 0.0, max_noise_rate = 0.0;
  for (double frequency : {72.0, 90.0, 120.0}) {
   for (double sign : {-1.0, 1.0}) {
    ContinuousTorsoOrientation noisy;
    for (int i = 0; i < 2401; ++i) {
      const double pitch = sign * (i <= 1200 ? i : 2400 - i) * .1 * deg;
      const Eigen::Matrix3d noise = rotation(.02 * deg * std::sin(i * 1.7),
          .02 * deg * std::cos(i * 1.1), .02 * deg * std::sin(i * .7));
      noisy.update(noise * rotation(0, pitch, 0), 1.0 + i / frequency);
      max_noise_yaw = std::max(max_noise_yaw, std::abs(noisy.rpy.z()));
      max_noise_rate = std::max(max_noise_rate, std::abs(noisy.pitch_yaw_rate.y()));
      assert(noisy.rpy.allFinite() && noisy.pitch_yaw_rate.allFinite());
    }
   }
  }
  std::cout << "noise max yaw=" << max_noise_yaw / deg << " deg; rate="
            << max_noise_rate / deg << " deg/s" << std::endl;
  assert(max_noise_yaw < 1.0 * deg);
  // At the regular-region boundary heading conditioning is 1/0.10. A
  // 0.02-deg oscillation sampled at 72-120 Hz can therefore exceed 25 deg/s;
  // this bound checks that it remains finite without hiding a branch flip.
  assert(max_noise_rate < 50.0 * deg);

  // Quaternion sign, duplicate/reordered stamps, initialization already bent,
  // and a fresh coordinate epoch must not introduce representation side effects.
  for (double pitch : {120 * deg, -120 * deg, 90 * deg, -90 * deg}) {
    ContinuousTorsoOrientation qsign;
    Eigen::Quaterniond q(rotation(0, pitch, 25 * deg));
    qsign.update(q.toRotationMatrix(), 1.0);
    close(qsign.rpy.y(), pitch); close(qsign.rpy.z(), 25 * deg);
    q.coeffs() *= -1;
    qsign.update(q.toRotationMatrix(), 1.01);
    close(qsign.pitch_yaw_rate.norm(), 0.0);
    const auto saved = qsign.rpy;
    qsign.update(rotation(0, 0, 0), 1.01);
    qsign.update(rotation(0, 0, 0), .9);
    close((qsign.rpy - saved).norm(), 0.0);
    auto invalid = rotation(0, 0, 0);
    invalid(0, 0) = std::numeric_limits<double>::quiet_NaN();
    qsign.update(invalid, 2.0);
    close((qsign.rpy - saved).norm(), 0.0);
    qsign = ContinuousTorsoOrientation{};
    qsign.update(rotation(0, 0, -30 * deg), .1);
    close(qsign.rpy.z(), -30 * deg); close(qsign.pitch_yaw_rate.norm(), 0.0);
  }
  std::cout << "PASS continuous torso orientation: signed 120-deg bows, turning, "
            << "side lean, yaw wrap, quaternion sign, lifecycle stamps; noisy max yaw="
            << max_noise_yaw / deg << " deg, max yaw rate=" << max_noise_rate / deg
            << " deg/s; combined max error=" << max_combined_error / deg
            << " deg, max step=" << max_combined_step / deg
            << " deg; old heading error=" << old_yaw_jump / deg << " deg\n";
}
