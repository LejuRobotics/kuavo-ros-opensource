#pragma once

#include <Eigen/Geometry>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>

namespace wheel_arm_abs_vel::direct_tcp {
using Vec = Eigen::Vector3d;
using Rot = Eigen::Matrix3d;
using Pose = Eigen::Isometry3d;
using Json = nlohmann::json;
enum class Control : std::uint8_t { Inactive=0, Armed=1, Active=2 };
Control controlFromWire(std::uint8_t value);

// Validation occurs once at each external input boundary. Mathematical routines
// consume these values; they do not repair, normalize or recheck the packet.
struct Sample { Pose pose; Vec linear, angular; };
Sample sample(const std::array<double, 7>& xyzw, const Vec& linear, const Vec& angular);
Pose transform(const Json& matrix);
struct ScalarProjection { double value, derivative; };
ScalarProjection softProject(double x, double lower, double upper, double width);
Rot rotation(int axis, double angle);

struct WristGeometry {
  Rot shoulder_parent = Rot::Identity();
  std::array<double,4> upstream_lower, upstream_upper;
  std::array<double,4> upstream_stopping_linear, upstream_stopping_quadratic, upstream_vmax;
  Pose j7_to_tcp;
  Vec j6_to_j7, shoulder, upper_vector, forearm_vector;
  double upper_length, forearm_length;
  double elbow_lower, elbow_upper;
  std::array<double,3> lower, upper, stopping_linear, stopping_quadratic;
};
struct ArmConfig {
  Pose controller_to_virtual;
  Rot alignment;
  Vec shoulder_rotation_weights;
  double reach_margin, reach_width, wrist_width, reference_cutoff, wrist_reference_cutoff;
  double reference_fast_cutoff, reference_fast_start, reference_fast_full;
  double feasible_reference_fast_cutoff=0.;
};
struct Config {
  std::string urdf, model_sha256, calibration, input_provenance, budget_status;
  std::string input_frame, output_frame;
  bool simulation_only;
  std::array<std::string,2> controlled_frames;
  double position_budget, orientation_budget, engage_duration, source_max_age, source_gap_timeout;
  std::array<ArmConfig,2> arms;
  Json document;
  explicit Config(const std::string& path);
};

struct RobotArm { Sample tcp; Rot forearm; Vec forearm_angular; Sample elbow, shoulder; Rot shoulder_axes; Rot shoulder_parent=Rot::Identity(); };
class RobotModel {
 public:
  explicit RobotModel(const Config& config);
  ~RobotModel();
  RobotModel(const RobotModel&) = delete;
  RobotModel& operator=(const RobotModel&) = delete;
  std::array<RobotArm,2> evaluate(const std::vector<std::string>& names,
      const std::vector<double>& q, const std::vector<double>& dq);
  std::array<Sample,2> joint7() const;
  Pose torso() const;
  Sample torsoSample() const;
  const std::array<WristGeometry,2>& geometry() const;
  const Json& audit() const;
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

struct Output {
  double position_cutoff_hz{};
  std::array<double,4> upstream, upstream_rate;
  std::array<double,4> projected_upstream;
  Sample nominal_reference; // Diagnostic only: before proximal rate constraints.
  Sample virtual_tcp, raw, projected, command, elbow;
  Vec position_projection, orientation_projection, position_filter, orientation_filter, requested_wrist, applied_wrist, reference_wrist, dynamic_lower, dynamic_upper;
  Vec requested_p6, requested_v6, reference_wrist_rate, final_domain_lower, final_domain_upper;
};

// An EngagedMapper cannot exist without a controller and a robot anchor. No
// uninitialized/default-pose path exists. Body skeletons are absent by design.
class EngagedMapper {
 public:
  EngagedMapper(const ArmConfig& config, const WristGeometry& geometry,
      const Sample& controller_anchor, const RobotArm& robot_anchor);
  // Static diagnostics only; runtime always uses advance().
  Output map(const Sample& controller, const RobotArm& robot, double elapsed);
  Output advance(const Sample& controller, const RobotArm& robot, double elapsed);
  Sample virtualPoint(const Sample& controller) const;
 private:
  ArmConfig config_;
  WristGeometry geometry_;
  Sample virtual_anchor_, robot_anchor_;
  Rot shoulder_anchor_,shoulder_metric_;
  Vec reference_p6_,reference_v6_,reference_a6_,input_p6_,reference_q_,reference_dq_,input_q_,wrist_stage1_,wrist_stage2_;
  Eigen::Vector4d reference_up_,upstream_dq_,upstream_engage_correction_;
  double reference_time_=0.;
  double input_feasibility_=0.;
  double input_translation_confidence_=1.;
  Rot input_raw_rotation_=Rot::Identity();
  Vec previous_wrist_ = Vec::Zero();
};
}  // namespace wheel_arm_abs_vel::direct_tcp
