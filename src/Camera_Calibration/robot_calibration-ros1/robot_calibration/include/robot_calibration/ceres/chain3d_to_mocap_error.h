/*
 * Copyright (C) 2026 Leju Robotics
 *
 * 6DOF residual between a kinematic chain FK and a mocap-measured pose.
 *
 * This error block is the core of the mocap joint-zero calibration (scheme 2a):
 * for each sample, it projects the chain FK (waist_yaw_link -> hand link,
 * with joint bias applied) and compares it against the mocap-measured relative
 * pose (hand_in_torso). Residual = 3 translation + 3 rotation (axis-magnitude).
 *
 * Unlike Chain3dToChain3d (checkerboard), this uses a SINGLE chain against an
 * external measured pose, so joint bias never cancels out.
 */

#ifndef ROBOT_CALIBRATION_CERES_CHAIN3D_TO_MOCAP_ERROR_H
#define ROBOT_CALIBRATION_CERES_CHAIN3D_TO_MOCAP_ERROR_H

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <ceres/ceres.h>
#include <ros/console.h>

#include <robot_calibration/calibration/offset_parser.h>
#include <robot_calibration/ceres/calibration_data_helpers.h>
#include <robot_calibration/models/chain.h>
#include <robot_calibration_msgs/CalibrationData.h>
#include <kdl/frames.hpp>

namespace robot_calibration
{

/**
 * \brief Residual between a single kinematic chain FK and a mocap-measured
 *        6DOF pose. Calibrates joint zero-offsets (bias).
 *
 * Observations come from Observation.poses (geometry_msgs/PoseStamped), where
 * position is in meters and orientation is a quaternion. Only the first pose
 * of the observation is used (one rigid body per sensor).
 */
struct Chain3dToMocap
{
  struct Config
  {
    double position_weight{1.0};
    double rotation_weight{1.0};
  };

  Chain3dToMocap(ChainModel* chain_model,
                 CalibrationOffsetParser* offsets,
                 robot_calibration_msgs::CalibrationData& data,
                 const Config& config)
    : chain_model_(chain_model)
    , offsets_(offsets)
    , data_(data)
    , config_(config)
  {
  }

  virtual ~Chain3dToMocap() {}

  /**
   * \brief Operator called by CERES optimizer.
   * \param free_params The offsets (bias) to be applied to joints.
   * \param residuals The residuals computed.
   */
  bool operator()(double const * const * free_params, double* residuals) const
  {
    // Apply current bias to the offset parser
    offsets_->update(free_params[0]);

    // FK from root (waist_yaw_link) to hand tip, including bias
    KDL::Frame fk = chain_model_->getChainFK(*offsets_, data_.joint_states);

    // Find the observation for this sensor
    int sensor_idx = getSensorIndex(data_, chain_model_->getName());
    if (sensor_idx < 0 || data_.observations[sensor_idx].poses.empty())
    {
      // No valid mocap observation; zero residuals so this sample is neutral
      for (int i = 0; i < 6; ++i)
        residuals[i] = 0.0;
      return true;
    }

    const geometry_msgs::PoseStamped& obs =
        data_.observations[sensor_idx].poses[0];

    // --- Position residual (meters) ---
    const double ox = obs.pose.position.x;
    const double oy = obs.pose.position.y;
    const double oz = obs.pose.position.z;
    residuals[0] = config_.position_weight * (fk.p.x() - ox);
    residuals[1] = config_.position_weight * (fk.p.y() - oy);
    residuals[2] = config_.position_weight * (fk.p.z() - oz);

    // --- Rotation residual (axis-magnitude, radians) ---
    // R_err = R_obs^-1 * R_fk
    KDL::Rotation R_obs = KDL::Rotation::Quaternion(
        obs.pose.orientation.x,
        obs.pose.orientation.y,
        obs.pose.orientation.z,
        obs.pose.orientation.w);
    KDL::Rotation R_err = R_obs.Inverse() * fk.M;
    // Convert to rotation vector (axis*angle). Use a numerically stable
    // formula (from log(R)) instead of axis_magnitude_from_rotation, which
    // divides by sqrt(1-qw^2) and produces NaN when R_err is near identity.
    {
      const double r00 = R_err.data[0], r01 = R_err.data[1], r02 = R_err.data[2];
      const double r10 = R_err.data[3], r11 = R_err.data[4], r12 = R_err.data[5];
      const double r20 = R_err.data[6], r21 = R_err.data[7], r22 = R_err.data[8];

      // cos(theta) from trace
      double cos_theta = 0.5 * (r00 + r11 + r22 - 1.0);
      if (cos_theta > 1.0) cos_theta = 1.0;
      if (cos_theta < -1.0) cos_theta = -1.0;
      const double theta = std::acos(cos_theta);

      double ax, ay, az;
      if (theta < 1e-8)
      {
        // R_err ~ I: use the anti-symmetric part (2*sin(theta) ~ 2*theta)
        ax = 0.5 * (r21 - r12);
        ay = 0.5 * (r02 - r20);
        az = 0.5 * (r10 - r01);
      }
      else
      {
        const double s = 0.5 * theta / std::sin(theta);
        ax = s * (r21 - r12);
        ay = s * (r02 - r20);
        az = s * (r10 - r01);
      }
      residuals[3] = config_.rotation_weight * ax;
      residuals[4] = config_.rotation_weight * ay;
      residuals[5] = config_.rotation_weight * az;
    }

    return true;  // always return true
  }

  /**
   * \brief Factory function to create the cost function.
   */
  static ceres::CostFunction* Create(ChainModel* chain_model,
                                     CalibrationOffsetParser* offsets,
                                     robot_calibration_msgs::CalibrationData& data,
                                     const Config& config)
  {
    ceres::DynamicNumericDiffCostFunction<Chain3dToMocap>* func =
        new ceres::DynamicNumericDiffCostFunction<Chain3dToMocap>(
            new Chain3dToMocap(chain_model, offsets, data, config));
    func->AddParameterBlock(offsets->size());
    func->SetNumResiduals(6);
    return static_cast<ceres::CostFunction*>(func);
  }

  ChainModel* chain_model_;
  CalibrationOffsetParser* offsets_;
  robot_calibration_msgs::CalibrationData data_;
  Config config_;
};

}  // namespace robot_calibration

#endif  // ROBOT_CALIBRATION_CERES_CHAIN3D_TO_MOCAP_ERROR_H
