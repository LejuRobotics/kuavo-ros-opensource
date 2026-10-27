#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <string>
#include <vector>
namespace velocity_ik {
struct ModelContext {
  struct Impl;
  std::unique_ptr<Impl> impl;
  ModelContext();
  ~ModelContext();
};
struct Rotation {
  Eigen::Matrix3d value;
  const Eigen::Matrix3d& matrix() const { return value; }
};
struct Pose {
  Eigen::Vector3d p;
  Rotation R;
  const Eigen::Vector3d& translation() const { return p; }
  const Rotation& rotation() const { return R; }
};
class ModelPinocchio;
struct Frame {
  const ModelPinocchio* model;
  unsigned long index;
  Pose CalcPose(const ModelContext&, const Frame&) const;
  Pose CalcPoseInWorld(const ModelContext&) const;
};
struct Joint {
  int q_index, v_index, nq, nv;
  Eigen::VectorXd lower, upper;
  int num_positions() const { return nq; }
  int num_velocities() const { return nv; }
  int position_start() const { return q_index; }
  int velocity_start() const { return v_index; }
  const Eigen::VectorXd& position_lower_limits() const { return lower; }
  const Eigen::VectorXd& position_upper_limits() const { return upper; }
};
// Pinocchio-only model. Each context owns its own Data and complete q; the
// controller's accepted reference is the sole evolving command trajectory.
class ModelPinocchio {
 public:
  ModelPinocchio(const std::string& urdf, const std::string& root,
                 const std::vector<std::string>& fixed_joints = {});
  ~ModelPinocchio();
  void addTcp(const std::string& name, const std::string& parent,
              const Eigen::Vector3d& translation, const Eigen::Matrix3d& rotation);
  void Finalize();
  int num_positions() const;
  int num_velocities() const;
  std::unique_ptr<ModelContext> CreateDefaultContext() const;
  Eigen::VectorXd GetPositions(const ModelContext&) const;
  void SetPositions(ModelContext*, const Eigen::VectorXd&) const;
  Joint GetJointByName(const std::string&) const;
  const Frame& GetFrameByName(const std::string&) const;
  const Frame& world_frame() const;
  Pose pose(const ModelContext&, const Frame&, const Frame&) const;
  // Spatial rows are explicitly [angular; linear], expressed in expressed.
  // Relative point velocity includes the moving-reference rotation term.
  void spatialJacobian(const ModelContext&, const Frame&, const Eigen::Vector3d&,
                       const Frame& relative, const Frame& expressed, Eigen::MatrixXd*) const;
  void translationJacobian(const ModelContext&, const Frame&, const Eigen::Vector3d&,
                           const Frame&, const Frame&, Eigen::MatrixXd*) const;
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
