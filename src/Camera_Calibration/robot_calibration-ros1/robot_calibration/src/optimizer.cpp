/*
 * Copyright (C) 2018-2022 Michael Ferguson
 * Copyright (C) 2014-2015 Fetch Robotics Inc.
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

#include <robot_calibration/ceres/optimizer.h>

#include <ceres/ceres.h>

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <robot_calibration_msgs/CalibrationData.h>

#include <robot_calibration/calibration/offset_parser.h>
#include <robot_calibration/ceres/calibration_data_helpers.h>
#include <robot_calibration/ceres/chain3d_to_chain3d_error.h>
#include <robot_calibration/ceres/chain3d_to_mesh_error.h>
#include <robot_calibration/ceres/chain3d_to_plane_error.h>
#include <robot_calibration/ceres/plane_to_plane_error.h>
#include <robot_calibration/ceres/outrageous_error.h>
#include <robot_calibration/models/camera3d.h>
#include <robot_calibration/models/chain.h>
#include <boost/shared_ptr.hpp>
#include <string>
#include <map>
#include <algorithm>
#include <limits>
#include <cmath>

namespace robot_calibration
{

namespace
{

class IterationDebugCallback : public ceres::IterationCallback
{
public:
  IterationDebugCallback() : has_prev_cost_(false), prev_cost_(0.0)
  {
  }

  virtual ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary)
  {
    std::ostringstream oss;
    oss.setf(std::ios::fixed);
    oss.precision(6);
    oss << "Optimizer iteration " << summary.iteration
        << ": cost=" << summary.cost
        << ", cost_change=" << summary.cost_change
        << ", gradient_max_norm=" << summary.gradient_max_norm
        << ", step_norm=" << summary.step_norm
        << ", step_is_valid=" << (summary.step_is_valid ? "true" : "false")
        << ", step_is_successful=" << (summary.step_is_successful ? "true" : "false")
        << ", trust_region_radius=" << summary.trust_region_radius;

    if (has_prev_cost_)
    {
      const double delta = summary.cost - prev_cost_;
      if (delta > std::max(1e-9, std::fabs(prev_cost_) * 0.05))
      {
        oss << " [WARN cost increased sharply: +" << delta << "]";
      }
    }

    ROS_INFO_STREAM(oss.str());
    has_prev_cost_ = true;
    prev_cost_ = summary.cost;
    return ceres::SOLVER_CONTINUE;
  }

private:
  bool has_prev_cost_;
  double prev_cost_;
};

}  // namespace

Optimizer::Optimizer(const std::string& robot_description) :
  num_params_(0),
  num_residuals_(0)
{
  if (!model_.initString(robot_description))
    std::cerr << "Failed to parse URDF." << std::endl;

  // Maintain consistent offset parser so we hold onto offsets
  offsets_.reset(new CalibrationOffsetParser());

  // Create a mesh loader
  mesh_loader_.reset(new MeshLoader(model_));
}

Optimizer::~Optimizer()
{
}

int Optimizer::optimize(OptimizationParams& params,
                        std::vector<robot_calibration_msgs::CalibrationData> data,
                        bool progress_to_stdout)
{
  // Load KDL from URDF
  if (!kdl_parser::treeFromUrdfModel(model_, tree_))
  {
    std::cerr << "Failed to construct KDL tree" << std::endl;
    return -1;
  }

  // Create models
  for (size_t i = 0; i < params.models.size(); ++i)
  {
    if (params.models[i].type == "chain")
    {
      if (progress_to_stdout)
        ROS_INFO_STREAM("Creating chain '" << params.models[i].name << "' from " <<
                                              params.base_link << " to " <<
                                              params.models[i].params["frame"]);
      ChainModel* model = new ChainModel(params.models[i].name, tree_, params.base_link, params.models[i].params["frame"]);
      models_[params.models[i].name] = model;
    }
    else if (params.models[i].type == "camera3d")
    {
      if (progress_to_stdout)
        ROS_INFO_STREAM("Creating camera3d '" << params.models[i].name << "' in frame " <<
                                                 params.models[i].params["frame"]);
      std::string param_name = params.models[i].params["param_name"];
      if (param_name == "")
      {
        // Default to same name as sensor
        param_name = params.models[i].name;
      }
      Camera3dModel* model = new Camera3dModel(params.models[i].name, param_name, tree_, params.base_link, params.models[i].params["frame"]);
      models_[params.models[i].name] = model;
    }
    else
    {
      // ERROR unknown
    }
  }

  // Reset which parameters are free (offset values are retained)
  offsets_->reset();

  // Setup  parameters to calibrate
  for (size_t i = 0; i < params.free_params.size(); ++i)
  {
    offsets_->add(params.free_params[i]);
  }
  for (size_t i = 0; i < params.free_frames.size(); ++i)
  {
    offsets_->addFrame(params.free_frames[i].name,
                       params.free_frames[i].x,
                       params.free_frames[i].y,
                       params.free_frames[i].z,
                       params.free_frames[i].roll,
                       params.free_frames[i].pitch,
                       params.free_frames[i].yaw);
  }
  for (size_t i = 0; i < params.free_frames_initial_values.size(); ++i)
  {
    if (!offsets_->setFrame(params.free_frames_initial_values[i].name,
                            params.free_frames_initial_values[i].x,
                            params.free_frames_initial_values[i].y,
                            params.free_frames_initial_values[i].z,
                            params.free_frames_initial_values[i].roll,
                            params.free_frames_initial_values[i].pitch,
                            params.free_frames_initial_values[i].yaw))
    {
      ROS_ERROR_STREAM("Error setting initial value for " <<
                       params.free_frames_initial_values[i].name);
    }
  }

  // Allocate space
  double* free_params = new double[offsets_->size()];
  offsets_->initialize(free_params);

  // Houston, we have a problem...
  ceres::Problem* problem = new ceres::Problem();

  // 调试用统计信息：chain3d_to_chain3d 误差块的样本和点数量
  int debug_chain3d_samples_used = 0;
  int debug_chain3d_residual_blocks = 0;
  size_t debug_chain3d_total_points = 0;

  // For each sample of data:
  for (size_t i = 0; i < data.size(); ++i)
  {
    for (size_t j = 0; j < params.error_blocks.size(); ++j)
    {
      if (params.error_blocks[j].type == "chain3d_to_chain3d")
      {
        // This error block can process data generated by the LedFinder,
        // CheckboardFinder, or any other finder that can sample the pose
        // of one or more data points that are connected at a constant offset
        // from a link a kinematic chain (the "arm").
        std::string a_name = static_cast<std::string>(params.error_blocks[j].params["model_a"]);
        std::string b_name = static_cast<std::string>(params.error_blocks[j].params["model_b"]);

        // Do some basic error checking for bad params
        if (a_name == "" || b_name == "" || a_name == b_name)
        {
          ROS_ERROR("chain3d_to_chain3d improperly configured: model_a and model_b params must be set!");
          return 0;
        }

        // Check that this sample has the required features/observations
        if (!hasSensor(data[i], a_name) || !hasSensor(data[i], b_name))
          continue;

        // 调试：统计当前 sample 中该误差块的点数
        int a_index = getSensorIndex(data[i], a_name);
        int b_index = getSensorIndex(data[i], b_name);
        if (a_index >= 0 && b_index >= 0)
        {
          size_t n_pts_a = data[i].observations[a_index].features.size();
          size_t n_pts_b = data[i].observations[b_index].features.size();
          size_t n_pts   = std::min(n_pts_a, n_pts_b);
          debug_chain3d_total_points += n_pts;
          debug_chain3d_residual_blocks++;
          debug_chain3d_samples_used++;
          if (progress_to_stdout)
            ROS_INFO_STREAM("Optimizer(chain3d_to_chain3d): using sample " << i
                            << " for block '" << params.error_blocks[j].name << "'"
                            << " with " << n_pts << " points"
                            << " (a=" << a_name << " has " << n_pts_a
                            << ", b=" << b_name << " has " << n_pts_b << ")");
        }

        // Create the block
        Chain3dToChain3d::Config chain3d_cfg;
        chain3d_cfg.center_weight =
          params.getParam(params.error_blocks[j], "center_weight", chain3d_cfg.center_weight);
        chain3d_cfg.corner_pose_weight =
          params.getParam(params.error_blocks[j], "corner_pose_weight", chain3d_cfg.corner_pose_weight);
        chain3d_cfg.grid_edge_length_m =
          params.getParam(params.error_blocks[j], "grid_edge_length_m", chain3d_cfg.grid_edge_length_m);
        chain3d_cfg.min_edge_ratio =
          params.getParam(params.error_blocks[j], "min_edge_ratio", chain3d_cfg.min_edge_ratio);
        chain3d_cfg.max_edge_ratio =
          params.getParam(params.error_blocks[j], "max_edge_ratio", chain3d_cfg.max_edge_ratio);
        chain3d_cfg.enable_corner_filter =
          params.getParam(params.error_blocks[j], "enable_corner_filter", chain3d_cfg.enable_corner_filter);

        if (progress_to_stdout)
        {
          ROS_INFO_STREAM("chain3d_to_chain3d config for block '" << params.error_blocks[j].name << "': "
                          << "center_weight=" << chain3d_cfg.center_weight
                          << ", corner_pose_weight=" << chain3d_cfg.corner_pose_weight
                          << ", enable_corner_filter=" << (chain3d_cfg.enable_corner_filter ? "true" : "false")
                          << ", grid_edge_length_m=" << chain3d_cfg.grid_edge_length_m
                          << ", min_edge_ratio=" << chain3d_cfg.min_edge_ratio
                          << ", max_edge_ratio=" << chain3d_cfg.max_edge_ratio);
        }

        ceres::CostFunction * cost = Chain3dToChain3d::Create(models_[a_name],
                                                              models_[b_name],
                                                              offsets_.get(),
                                                              data[i],
                                                              chain3d_cfg);

        // Output initial error
        if (progress_to_stdout)
        {
          double ** params = new double*[1];
          params[0] = free_params;
          double * residuals = new double[cost->num_residuals()];

          cost->Evaluate(params, residuals, NULL);

          std::cout << "INITIAL COST (" << i << ")" << std::endl << "  x: ";
          for (size_t k = 0; k < static_cast<size_t>(cost->num_residuals() / 3); ++k)
            std::cout << "  " << std::setw(10) << std::fixed << residuals[(3*k + 0)];
          std::cout << std::endl << "  y: ";
          for (size_t k = 0; k < static_cast<size_t>(cost->num_residuals() / 3); ++k)
            std::cout << "  " << std::setw(10) << std::fixed << residuals[(3*k + 1)];
          std::cout << std::endl << "  z: ";
          for (size_t k = 0; k < static_cast<size_t>(cost->num_residuals() / 3); ++k)
            std::cout << "  " << std::setw(10) << std::fixed << residuals[(3*k + 2)];
          std::cout << std::endl << std::endl;
        }

        problem->AddResidualBlock(cost,
                                  NULL,  // squared loss
                                  free_params);
      }
      else if (params.error_blocks[j].type == "chain3d_to_plane")
      {
        // This error block can process data generated by the PlaneFinder
        std::string chain_name = static_cast<std::string>(params.error_blocks[j].params["model_a"]);

        // Do some basic error checking for bad params
        if (chain_name == "")
        {
          ROS_ERROR("chain3d_to_plane improperly configured: model_a param must be set!");
          return 0;
        }

        // Check that this sample has the required features/observations
        if (!hasSensor(data[i], chain_name))
          continue;

        // Create the block
        ceres::CostFunction * cost =
          Chain3dToPlane::Create(models_[chain_name],
                                 offsets_.get(),
                                 data[i],
                                 params.getParam(params.error_blocks[j], "a", 0.0),
                                 params.getParam(params.error_blocks[j], "b", 0.0),
                                 params.getParam(params.error_blocks[j], "c", 1.0),
                                 params.getParam(params.error_blocks[j], "d", 0.0),
                                 params.getParam(params.error_blocks[j], "scale", 1.0));

        // Output initial error
        if (progress_to_stdout)
        {
          double ** params = new double*[1];
          params[0] = free_params;
          double * residuals = new double[cost->num_residuals()];

          cost->Evaluate(params, residuals, NULL);

          std::cout << "INITIAL COST (" << i << ")" << std::endl << "  d: ";
          for (size_t k = 0; k < static_cast<size_t>(cost->num_residuals()); ++k)
            std::cout << "  " << std::setw(10) << std::fixed << residuals[(k)];
          std::cout << std::endl << std::endl;
        }

        problem->AddResidualBlock(cost,
                                  NULL /* squared loss */,
                                  free_params);
      }
      else if (params.error_blocks[j].type == "chain3d_to_mesh")
      {
        // This error block can process data generated by the RobotFinder
        std::string chain_name = static_cast<std::string>(params.error_blocks[j].params["model_a"]);

        // Do some basic error checking for bad params
        if (chain_name == "")
        {
          ROS_ERROR("chain3d_to_mesh improperly configured: model_a param must be set!");
          return 0;
        }

        // Check that this sample has the required features/observations
        if (!hasSensor(data[i], chain_name))
          continue;

        // Get the mesh
        std::string link_name = params.getParam(params.error_blocks[j], "link_name", std::string(""));
        MeshPtr mesh = mesh_loader_->getCollisionMesh(link_name);
        if (!mesh)
        {
          ROS_ERROR("chain3d_to_mesh improperly configured: cannot load mesh for %s", link_name.c_str());
          return 0;
        }

        // Create the block
        ceres::CostFunction * cost =
          Chain3dToMesh::Create(models_[chain_name],
                                offsets_.get(),
                                data[i],
                                mesh);

        // Output initial error
        if (progress_to_stdout)
        {
          double ** params = new double*[1];
          params[0] = free_params;
          double * residuals = new double[cost->num_residuals()];

          cost->Evaluate(params, residuals, NULL);

          std::cout << "INITIAL COST (" << i << ")" << std::endl << "  d: ";
          for (size_t k = 0; k < static_cast<size_t>(cost->num_residuals()); ++k)
            std::cout << "  " << std::setw(10) << std::fixed << residuals[(k)];
          std::cout << std::endl << std::endl;
        }

        problem->AddResidualBlock(cost,
                                  NULL /* squared loss */,
                                  free_params);


      }
      else if (params.error_blocks[j].type == "plane_to_plane")
      {
        // This error block can process data generated by the PlaneFinder,
        // CheckerboardFinder, or any other finder that returns a series of
        // planar points.
        std::string a_name = static_cast<std::string>(params.error_blocks[j].params["model_a"]);
        std::string b_name = static_cast<std::string>(params.error_blocks[j].params["model_b"]);

        // Do some basic error checking for bad params
        if (a_name == "" || b_name == "" || a_name == b_name)
        {
          ROS_ERROR("plane_to_plane improperly configured: model_a and model_a params must be set!");
          return 0;
        }

        // Check that this sample has the required features/observations
        if (!hasSensor(data[i], a_name) || !hasSensor(data[i], b_name))
          continue;

        // Create the block
        ceres::CostFunction * cost =
          PlaneToPlaneError::Create(models_[a_name],
                                    models_[b_name],
                                    offsets_.get(),
                                    data[i],
                                    params.getParam(params.error_blocks[j], "scale_normal", 1.0),
                                    params.getParam(params.error_blocks[j], "scale_offset", 1.0));

        // Output initial error
        if (progress_to_stdout)
        {
          double ** params = new double*[1];
          params[0] = free_params;
          double * residuals = new double[cost->num_residuals()];

          cost->Evaluate(params, residuals, NULL);
          std::cout << "INITIAL COST (" << i << ")" << std::endl << "  a: ";
          std::cout << "  " << std::setw(10) << std::fixed << residuals[0];
          std::cout << std::endl << "  b: ";
          std::cout << "  " << std::setw(10) << std::fixed << residuals[1];
          std::cout << std::endl << "  c: ";
          std::cout << "  " << std::setw(10) << std::fixed << residuals[2];
          std::cout << std::endl << "  d: ";
          std::cout << "  " << std::setw(10) << std::fixed << residuals[3];
          std::cout << std::endl << std::endl;
        }

        problem->AddResidualBlock(cost,
                                  NULL,  // squared loss
                                  free_params);
      }
      else if (params.error_blocks[j].type == "outrageous")
      {
        // Outrageous error block requires no particular sensors, add to every sample
        problem->AddResidualBlock(
          OutrageousError::Create(offsets_.get(),
                                  static_cast<std::string>(params.error_blocks[j].params["param"]),
                                  params.getParam(params.error_blocks[j], "joint_scale", 1.0),
                                  params.getParam(params.error_blocks[j], "position_scale", 1.0),
                                  params.getParam(params.error_blocks[j], "rotation_scale", 1.0)),
          NULL, // squared loss
          free_params);
      }
      else
      {
        ROS_ERROR_STREAM("Unknown error block: " << params.error_blocks[j].type);
        return 0;
      }
    }
  }

  // 优化前总览：帮助判断“大残差来自初值”还是“在迭代中出现”
  {
    std::vector<double> residuals;
    ceres::Problem::EvaluateOptions eval_options;
    const bool ok = problem->Evaluate(eval_options, NULL, &residuals, NULL, NULL);
    if (ok && !residuals.empty())
    {
      double sum_sq = 0.0;
      double max_abs = 0.0;
      for (size_t i = 0; i < residuals.size(); ++i)
      {
        const double r = residuals[i];
        sum_sq += r * r;
        max_abs = std::max(max_abs, std::fabs(r));
      }
      const double rmse = std::sqrt(sum_sq / static_cast<double>(residuals.size()));
      ROS_INFO_STREAM("Pre-solve residual overview: count=" << residuals.size()
                      << ", rmse=" << rmse
                      << ", max_abs=" << max_abs);
    }
    else
    {
      ROS_WARN_STREAM("Pre-solve residual overview unavailable: problem->Evaluate failed or empty residuals.");
    }
  }

  // Setup the actual optimization
  ceres::Solver::Options options;
  options.use_nonmonotonic_steps = true;
  options.function_tolerance = 1e-10;
  options.linear_solver_type = ceres::DENSE_QR;
  options.max_num_iterations = params.max_num_iterations;
  // Only print cost/summary based on our ROS_INFO blocks below.
  // Avoid per-iteration spam from Ceres itself.
  options.minimizer_progress_to_stdout = false;
  // IterationCallback 仅用于逐次迭代调试输出；静默模式下不必每步更新状态
  options.update_state_every_iteration = progress_to_stdout;
  IterationDebugCallback iter_debug_callback;
  // Avoid per-iteration "Optimizer iteration ..." spam.
  // If you need iteration-level debugging later, we can re-introduce a dedicated flag.

  if (progress_to_stdout)
    std::cout << "\nSolver output:" << std::endl;
  summary_.reset(new ceres::Solver::Summary());
  ceres::Solve(options, problem, summary_.get());

  // Ceres 解写入 offset 解析器（与是否打印日志无关）
  offsets_->update(free_params);

  // 打印 ceres 总结和误差信息（只打印一次，不逐迭代刷屏）
  // 即便 progress_to_stdout=false（即 verbose=false），也希望你能看到 cost。
  ROS_INFO_STREAM("Optimizer(chain3d_to_chain3d): total samples used = "
                  << debug_chain3d_samples_used
                  << ", residual blocks = " << debug_chain3d_residual_blocks
                  << ", total point triplets = " << debug_chain3d_total_points);
  ROS_INFO_STREAM("Ceres brief report: " << summary_->BriefReport());
  ROS_INFO_STREAM("Ceres iterations: " << summary_->iterations.size()
                  << ", initial cost: " << summary_->initial_cost
                  << ", final cost: " << summary_->final_cost);
  if (!summary_->IsSolutionUsable())
  {
    ROS_WARN_STREAM("Ceres reports solution not usable, termination: "
                    << summary_->termination_type
                    << ", message: " << summary_->message);
  }

  // 优化后总览：与优化前同口径对比
  {
    std::vector<double> residuals;
    ceres::Problem::EvaluateOptions eval_options;
    const bool ok = problem->Evaluate(eval_options, NULL, &residuals, NULL, NULL);
    if (ok && !residuals.empty())
    {
      double sum_sq = 0.0;
      double max_abs = 0.0;
      for (size_t i = 0; i < residuals.size(); ++i)
      {
        const double r = residuals[i];
        sum_sq += r * r;
        max_abs = std::max(max_abs, std::fabs(r));
      }
      const double rmse = std::sqrt(sum_sq / static_cast<double>(residuals.size()));
      ROS_INFO_STREAM("Post-solve residual overview: count=" << residuals.size()
                      << ", rmse=" << rmse
                      << ", max_abs=" << max_abs);
    }
    else
    {
      ROS_WARN_STREAM("Post-solve residual overview unavailable: problem->Evaluate failed or empty residuals.");
    }
  }

  // 诊断输出：对每个 sample 重新评估残差（单位：米），便于定位“哪一帧/哪一组角点”拖后腿。
  // 注意：chain3d_to_chain3d residual 的每个分量 rx/ry/rz 均为米，点到点误差的欧式距离同样是米。
  if (progress_to_stdout)
  {
    for (size_t i = 0; i < data.size(); ++i)
    {
      for (size_t j = 0; j < params.error_blocks.size(); ++j)
      {
        if (params.error_blocks[j].type != "chain3d_to_chain3d")
          continue;

        const std::string a_name = static_cast<std::string>(params.error_blocks[j].params["model_a"]);
        const std::string b_name = static_cast<std::string>(params.error_blocks[j].params["model_b"]);
        if (a_name == "" || b_name == "" || a_name == b_name)
          continue;
        if (!hasSensor(data[i], a_name) || !hasSensor(data[i], b_name))
          continue;

        // 直接用模型投影拿到 base_link 下的 3D 点（已经包含 offsets/fk）
        const std::vector<geometry_msgs::PointStamped> a_pts = models_[a_name]->project(data[i], *offsets_);
        const std::vector<geometry_msgs::PointStamped> b_pts = models_[b_name]->project(data[i], *offsets_);
        const size_t n = std::min(a_pts.size(), b_pts.size());
        if (n == 0)
          continue;

        double sum_sq = 0.0;
        double max_dist = 0.0;
        size_t max_i = 0;

        bool frame_mismatch = false;
        std::string frame_id = a_pts[0].header.frame_id;
        for (size_t k = 0; k < n; ++k)
        {
          if (a_pts[k].header.frame_id != b_pts[k].header.frame_id)
            frame_mismatch = true;
          const double rx = a_pts[k].point.x - b_pts[k].point.x;
          const double ry = a_pts[k].point.y - b_pts[k].point.y;
          const double rz = a_pts[k].point.z - b_pts[k].point.z;
          const double dist = std::sqrt(rx * rx + ry * ry + rz * rz);
          sum_sq += dist * dist;
          if (dist > max_dist)
          {
            max_dist = dist;
            max_i = k;
          }
        }

        const double rmse = std::sqrt(sum_sq / std::max<size_t>(1, n));

        std::ostringstream oss;
        oss.setf(std::ios::fixed);
        oss.precision(3);
        oss << "Post-solve chain3d_to_chain3d sample=" << i
            << " block='" << params.error_blocks[j].name << "'"
            << " models: a=" << a_name << ", b=" << b_name
            << " frame=" << frame_id
            << " n_pts=" << n
            << " rmse_mm=" << (rmse * 1000.0)
            << " max_mm=" << (max_dist * 1000.0) << " (idx=" << max_i << ")";
        if (frame_mismatch)
          oss << " [WARN frame_id mismatch]";

        ROS_INFO_STREAM(oss.str());
      }
    }
  }

  // 特别打印 camera_joint 的当前 offset（若存在），便于观察是否出现异常大值
  if (progress_to_stdout)
  {
    KDL::Frame cam_offset;
    if (offsets_->getFrame("camera_joint", cam_offset))
    {
      double r, p, y;
      cam_offset.M.GetRPY(r, p, y);
      ROS_INFO_STREAM("camera_joint offset after solve: "
                      << "p=[" << cam_offset.p.x() << ", "
                      << cam_offset.p.y() << ", "
                      << cam_offset.p.z() << "], "
                      << "rpy=[" << r << ", " << p << ", " << y << "]");
    }
  }

  // Save some status
  num_params_ = problem->NumParameters();
  num_residuals_ = problem->NumResiduals();

  // Note: the error blocks will be managed by scoped_ptr in cost functor
  //       which takes ownership, and so we do not need to delete them here

  // Done with our free params
  delete[] free_params;
  delete problem;

  return 0;
}

std::vector<std::string> Optimizer::getCameraNames()
{
  std::vector<std::string> camera_names;
  for (auto it = models_.begin(); it != models_.end(); ++it)
  {
    if (it->second->getType() == "Camera3dModel")
    {
       camera_names.push_back(it->first);
    }
  }
  return camera_names;
}

}  // namespace robot_calibration
