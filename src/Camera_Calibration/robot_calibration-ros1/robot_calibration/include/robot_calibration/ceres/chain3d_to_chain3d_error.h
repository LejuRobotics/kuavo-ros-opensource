/*
 * Copyright (C) 2018 Michael Ferguson
 * Copyright (C) 2015 Fetch Robotics Inc.
 * Copyright (C) 2013-2014 Unbounded Robotics Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// Author: Michael Ferguson

#ifndef ROBOT_CALIBRATION_CERES_CHAIN3D_TO_CHAIN3D_ERROR_H
#define ROBOT_CALIBRATION_CERES_CHAIN3D_TO_CHAIN3D_ERROR_H

#include <string>
#include <cmath>
#include <vector>
#include <limits>
#include <ceres/ceres.h>
#include <ros/console.h>
#include <robot_calibration/calibration/offset_parser.h>
#include <robot_calibration/ceres/calibration_data_helpers.h>
#include <robot_calibration/models/camera3d.h>
#include <robot_calibration/models/chain.h>
#include <robot_calibration_msgs/CalibrationData.h>
#include <kdl/frames.hpp>

namespace robot_calibration
{

/**
 *  \brief Error block for computing the residual error between two
 *         3d data sources. This can be used to calibrate 3d cameras
 *         to arms, or 3d cameras to other 3d cameras.
 */
struct Chain3dToChain3d
{
  struct Config
  {
    double center_weight{1.0};
    double corner_pose_weight{0.35};
    double grid_edge_length_m{0.03};
    double min_edge_ratio{0.6};
    double max_edge_ratio{1.4};
    bool enable_corner_filter{true};
  };

  /**
   *  \brief This function is not used direcly, instead use the Create() function.
   *  \param a_model The model for the first chain, used for reprojection.
   *  \param b_model The model for the second chain, used for reprojection.
   *  \param offsets Easy access to the free parameters.
   *  \param data The calibration data collected.
   */
  Chain3dToChain3d(ChainModel* a_model,
                   ChainModel* b_model,
                   CalibrationOffsetParser* offsets,
                   robot_calibration_msgs::CalibrationData& data,
                   const Config& config)
  {
    a_model_ = a_model;
    b_model_ = b_model;
    offsets_ = offsets;
    data_ = data;
    config_ = config;
  }

  virtual ~Chain3dToChain3d() {}

  /**
   *  \brief Operator called by CERES optimizer.
   *  \param free_params The offsets to be applied to joints/transforms.
   *  \param residuals The residuals computed, to be returned to the optimizer.
   */
  bool operator()(double const * const * free_params,
                  double* residuals) const
  {
    static int debug_calls = 0;

    // Update calibration offsets based on free params
    offsets_->update(free_params[0]);

    // Project the observations into common base frame
    std::vector<geometry_msgs::PointStamped> a_pts =
        a_model_->project(data_, *offsets_);
    std::vector<geometry_msgs::PointStamped> b_pts =
        b_model_->project(data_, *offsets_);

    if (a_pts.size() != b_pts.size())
    {
      std::cerr << "Observations do not match in size." << std::endl;
      return false;
    }
    if (a_pts.empty())
    {
      std::cerr << "Observations are empty." << std::endl;
      return false;
    }

    if (a_pts[0].header.frame_id != b_pts[0].header.frame_id)
      std::cerr << "Projected observation frame_ids do not match." << std::endl;

    double ax = 0.0, ay = 0.0, az = 0.0;
    double bx = 0.0, by = 0.0, bz = 0.0;
    for (size_t i = 0; i < a_pts.size(); ++i)
    {
      ax += a_pts[i].point.x;
      ay += a_pts[i].point.y;
      az += a_pts[i].point.z;
      bx += b_pts[i].point.x;
      by += b_pts[i].point.y;
      bz += b_pts[i].point.z;
    }
    const double inv_n = 1.0 / static_cast<double>(a_pts.size());
    ax *= inv_n; ay *= inv_n; az *= inv_n;
    bx *= inv_n; by *= inv_n; bz *= inv_n;

    const double rx_center = ax - bx;
    const double ry_center = ay - by;
    const double rz_center = az - bz;

    if (!std::isfinite(rx_center) || !std::isfinite(ry_center) || !std::isfinite(rz_center))
    {
      ROS_WARN_STREAM("Non-finite center residual in Chain3dToChain3d: "
                      << "a_center=[" << ax << ", " << ay << ", " << az << "]"
                      << " b_center=[" << bx << ", " << by << ", " << bz << "]");
      return false;
    }

    // Center residual controls position.
    residuals[0] = config_.center_weight * rx_center;
    residuals[1] = config_.center_weight * ry_center;
    residuals[2] = config_.center_weight * rz_center;

    // Corner quality gate using known 30 mm checkerboard spacing.
    // For each corner, estimate nearest-neighbor spacing in both sets.
    // If spacing is implausible, treat that corner as outlier and zero its residual.
    std::vector<char> valid(a_pts.size(), 1);
    if (config_.enable_corner_filter)
    {
      const double min_edge = config_.grid_edge_length_m * config_.min_edge_ratio;
      const double max_edge = config_.grid_edge_length_m * config_.max_edge_ratio;
      for (size_t i = 0; i < a_pts.size(); ++i)
      {
        double dmin_a = std::numeric_limits<double>::infinity();
        double dmin_b = std::numeric_limits<double>::infinity();
        for (size_t j = 0; j < a_pts.size(); ++j)
        {
          if (i == j) continue;
          const double dax = a_pts[i].point.x - a_pts[j].point.x;
          const double day = a_pts[i].point.y - a_pts[j].point.y;
          const double daz = a_pts[i].point.z - a_pts[j].point.z;
          const double dbx = b_pts[i].point.x - b_pts[j].point.x;
          const double dby = b_pts[i].point.y - b_pts[j].point.y;
          const double dbz = b_pts[i].point.z - b_pts[j].point.z;
          dmin_a = std::min(dmin_a, std::sqrt(dax*dax + day*day + daz*daz));
          dmin_b = std::min(dmin_b, std::sqrt(dbx*dbx + dby*dby + dbz*dbz));
        }

        const bool a_ok = std::isfinite(dmin_a) && dmin_a >= min_edge && dmin_a <= max_edge;
        const bool b_ok = std::isfinite(dmin_b) && dmin_b >= min_edge && dmin_b <= max_edge;
        if (!(a_ok && b_ok))
          valid[i] = 0;
      }
    }

    // De-meaned corner residual controls orientation/shape without affecting translation.
    for (size_t i = 0; i < a_pts.size(); ++i)
    {
      const double adx = a_pts[i].point.x - ax;
      const double ady = a_pts[i].point.y - ay;
      const double adz = a_pts[i].point.z - az;
      const double bdx = b_pts[i].point.x - bx;
      const double bdy = b_pts[i].point.y - by;
      const double bdz = b_pts[i].point.z - bz;

      const double rx_pose = adx - bdx;
      const double ry_pose = ady - bdy;
      const double rz_pose = adz - bdz;
      if (!std::isfinite(rx_pose) || !std::isfinite(ry_pose) || !std::isfinite(rz_pose))
      {
        ROS_WARN_STREAM("Non-finite de-meaned corner residual in Chain3dToChain3d: "
                        << "i=" << i
                        << " a_demean=[" << adx << ", " << ady << ", " << adz << "]"
                        << " b_demean=[" << bdx << ", " << bdy << ", " << bdz << "]");
        return false;
      }

      if (valid[i])
      {
        residuals[3 + (3*i) + 0] = config_.corner_pose_weight * rx_pose;
        residuals[3 + (3*i) + 1] = config_.corner_pose_weight * ry_pose;
        residuals[3 + (3*i) + 2] = config_.corner_pose_weight * rz_pose;
      }
      else
      {
        residuals[3 + (3*i) + 0] = 0.0;
        residuals[3 + (3*i) + 1] = 0.0;
        residuals[3 + (3*i) + 2] = 0.0;
      }
    }

    // 调试打印已关闭：需要时建议看 capture 阶段的 base<-tag(obs/urdf/delta) 与 post-solve 统计。

    return true;  // always return true
  }

  /**
   *  \brief Helper factory function to create a new error block. Parameters
   *         are described in the class constructor, which this function calls.
   */
  static ceres::CostFunction* Create(ChainModel* a_model,
                                     ChainModel* b_model,
                                     CalibrationOffsetParser* offsets,
                                     robot_calibration_msgs::CalibrationData& data,
                                     const Config& config)
  {
    int index = getSensorIndex(data, a_model->getName());
    if (index == -1)
    {
      // In theory, we should never get here, because the optimizer does a check
      std::cerr << "Sensor name doesn't match any of the existing finders" << std::endl;
      return 0;
    }

    ceres::DynamicNumericDiffCostFunction<Chain3dToChain3d> * func;
    func = new ceres::DynamicNumericDiffCostFunction<Chain3dToChain3d>(
                    new Chain3dToChain3d(a_model, b_model, offsets, data, config));
    func->AddParameterBlock(offsets->size());
    func->SetNumResiduals(3 + data.observations[index].features.size() * 3);

    return static_cast<ceres::CostFunction*>(func);
  }

  ChainModel * a_model_;
  ChainModel * b_model_;
  CalibrationOffsetParser * offsets_;
  robot_calibration_msgs::CalibrationData data_;
  Config config_;
};

}  // namespace robot_calibration

#endif  // ROBOT_CALIBRATION_CERES_CHAIN3D_TO_CHAIN3D_ERROR_H
