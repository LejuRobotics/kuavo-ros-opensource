#include <robot_calibration/calibration_csv_io.h>

#include <ros/ros.h>

#include <std_msgs/String.h>
#include <XmlRpcValue.h>

#include <robot_calibration/calibration/offset_parser.h>
#include <robot_calibration/fk_debug_utils.h>
#include <robot_calibration/models/chain.h>
#include <kdl/frames.hpp>

#include "robot_calibration/ceres/optimizer.h"
#include "robot_calibration/calibration/export.h"

#include <Eigen/Dense>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <numeric>

namespace
{
static inline int getSensorIndexByName(const robot_calibration_msgs::CalibrationData& msg,
                                      const std::string& sensor_name)
{
  for (size_t i = 0; i < msg.observations.size(); ++i)
  {
    if (msg.observations[i].sensor_name == sensor_name)
      return static_cast<int>(i);
  }
  return -1;
}

static inline double rotationAngleDeg(const Eigen::Matrix3d& R_est,
                                      const Eigen::Matrix3d& R_ref)
{
  const Eigen::Matrix3d R_err = R_ref.transpose() * R_est;
  double c = (R_err.trace() - 1.0) * 0.5;
  if (c > 1.0) c = 1.0;
  if (c < -1.0) c = -1.0;
  return std::acos(c) * 180.0 / M_PI;
}

static inline bool estimateRigidTransformSvd(const std::vector<geometry_msgs::PointStamped>& src_board,
                                             const std::vector<geometry_msgs::PointStamped>& dst_cam,
                                             Eigen::Matrix3d* R_cam_from_board,
                                             Eigen::Vector3d* t_cam_from_board)
{
  if (!R_cam_from_board || !t_cam_from_board)
    return false;
  const size_t n = std::min(src_board.size(), dst_cam.size());
  if (n < 4)
    return false;

  Eigen::MatrixXd X(3, n);
  Eigen::MatrixXd Y(3, n);
  for (size_t i = 0; i < n; ++i)
  {
    X(0, i) = src_board[i].point.x;
    X(1, i) = src_board[i].point.y;
    X(2, i) = src_board[i].point.z;
    Y(0, i) = dst_cam[i].point.x;
    Y(1, i) = dst_cam[i].point.y;
    Y(2, i) = dst_cam[i].point.z;
  }
  const Eigen::Vector3d mx = X.rowwise().mean();
  const Eigen::Vector3d my = Y.rowwise().mean();
  X.colwise() -= mx;
  Y.colwise() -= my;

  const Eigen::Matrix3d H = X * Y.transpose();
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d V = svd.matrixV();
  Eigen::Matrix3d R = V * U.transpose();
  if (R.determinant() < 0.0)
  {
    V.col(2) *= -1.0;
    R = V * U.transpose();
  }
  *R_cam_from_board = R;
  *t_cam_from_board = my - R * mx;
  return true;
}

void remapBoardObservationOrigin(std::vector<robot_calibration_msgs::CalibrationData>& data,
                                 const std::string& board_sensor_name,
                                 const std::string& board_frame_id,
                                 double shift_x,
                                 double shift_y,
                                 double shift_z)
{
  std::size_t sample_count = 0;
  std::size_t point_count = 0;
  for (auto& sample : data)
  {
    bool touched_this_sample = false;
    for (auto& obs : sample.observations)
    {
      if (obs.sensor_name != board_sensor_name)
        continue;
      for (auto& feat : obs.features)
      {
        if (feat.header.frame_id != board_frame_id)
          continue;
        feat.point.x -= shift_x;
        feat.point.y -= shift_y;
        feat.point.z -= shift_z;
        touched_this_sample = true;
        ++point_count;
      }
    }
    if (touched_this_sample)
      ++sample_count;
  }

  ROS_INFO_STREAM("Applied board observation origin remap: sensor='" << board_sensor_name
                  << "', frame_id='" << board_frame_id
                  << "', shift_xyz(m)=[" << shift_x << ", " << shift_y << ", " << shift_z << "]"
                  << ", affected_samples=" << sample_count
                  << ", affected_points=" << point_count);
}

static inline void rejectOutliersByDelta(const std::string& urdf_xml,
                                        const std::string& fk_base,
                                        const std::string& fk_tip_camera,
                                        const std::string& fk_tip_board,
                                        const std::string& obs_camera_sensor,
                                        const std::string& board_obs_sensor,
                                        const robot_calibration::CalibrationOffsetParser& offsets,
                                        double pos_thr_m,
                                        double rot_thr_deg,
                                        std::vector<robot_calibration_msgs::CalibrationData>& data,
                                        std::vector<int>& sample_ids)
{
  if (pos_thr_m <= 0.0 && rot_thr_deg <= 0.0)
    return;

  urdf::Model model;
  if (!model.initString(urdf_xml))
  {
    ROS_WARN("Outlier rejection: failed to parse URDF, skipping.");
    return;
  }
  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model, tree))
  {
    ROS_WARN("Outlier rejection: failed to build KDL tree, skipping.");
    return;
  }

  robot_calibration::ChainModel chain_base_to_cam("_rej_fk_cam_", tree, fk_base, fk_tip_camera);
  robot_calibration::ChainModel chain_base_to_board("_rej_fk_board_", tree, fk_base, fk_tip_board);

  std::vector<size_t> keep;
  keep.reserve(data.size());

  std::ostringstream dropped;
  size_t drop_n = 0;

  for (size_t i = 0; i < data.size(); ++i)
  {
    const auto& sample = data[i];

    const int icam = getSensorIndexByName(sample, obs_camera_sensor);
    const int iboard = getSensorIndexByName(sample, board_obs_sensor);
    if (icam < 0 || iboard < 0)
    {
      keep.push_back(i);
      continue;
    }
    const auto& feats_cam = sample.observations[static_cast<size_t>(icam)].features;
    const auto& feats_board = sample.observations[static_cast<size_t>(iboard)].features;
    if (feats_cam.size() < 4 || feats_board.size() < 4)
    {
      keep.push_back(i);
      continue;
    }

    // FK: base<-cam, base<-board(urdf)
    const KDL::Frame T_base_cam = chain_base_to_cam.getChainFK(offsets, sample.joint_states);
    const KDL::Frame T_base_board_urdf = chain_base_to_board.getChainFK(offsets, sample.joint_states);

    Eigen::Matrix3d R_cb;  // cam<-board
    Eigen::Vector3d t_cb;
    if (!estimateRigidTransformSvd(feats_board, feats_cam, &R_cb, &t_cb))
    {
      keep.push_back(i);
      continue;
    }

    Eigen::Matrix3d R_base_cam;
    Eigen::Vector3d t_base_cam;
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c)
        R_base_cam(r, c) = T_base_cam.M(r, c);
    t_base_cam << T_base_cam.p.x(), T_base_cam.p.y(), T_base_cam.p.z();

    // base<-board(obs) = base<-cam * cam<-board
    const Eigen::Matrix3d R_base_board_obs = R_base_cam * R_cb;
    const Eigen::Vector3d t_base_board_obs = R_base_cam * t_cb + t_base_cam;

    Eigen::Matrix3d R_base_board_urdf;
    Eigen::Vector3d t_base_board_urdf;
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c)
        R_base_board_urdf(r, c) = T_base_board_urdf.M(r, c);
    t_base_board_urdf << T_base_board_urdf.p.x(), T_base_board_urdf.p.y(), T_base_board_urdf.p.z();

    // delta = urdf^-1 * obs
    const Eigen::Matrix3d R_delta = R_base_board_urdf.transpose() * R_base_board_obs;
    const Eigen::Vector3d t_delta = R_base_board_urdf.transpose() * (t_base_board_obs - t_base_board_urdf);

    const double pos_err = t_delta.norm();
    const double rot_err = rotationAngleDeg(R_base_board_obs, R_base_board_urdf);

    const bool drop_pos = (pos_thr_m > 0.0) && (pos_err > pos_thr_m);
    const bool drop_rot = (rot_thr_deg > 0.0) && (rot_err > rot_thr_deg);
    if (drop_pos || drop_rot)
    {
      ++drop_n;
      dropped << " s" << i << "(pos=" << pos_err << "m, rot=" << rot_err << "deg)";
      continue;
    }
    keep.push_back(i);
  }

  if (drop_n > 0)
  {
    std::vector<robot_calibration_msgs::CalibrationData> filtered;
    std::vector<int> filtered_ids;
    filtered.reserve(keep.size());
    filtered_ids.reserve(keep.size());
    for (const auto idx : keep)
    {
      filtered.push_back(data[idx]);
      filtered_ids.push_back(sample_ids[idx]);
    }
    data.swap(filtered);
    sample_ids.swap(filtered_ids);

    ROS_WARN_STREAM("Outlier rejection enabled: pos_thr_m=" << pos_thr_m
                    << ", rot_thr_deg=" << rot_thr_deg
                    << ", dropped=" << drop_n
                    << ", kept=" << data.size()
                    << "; dropped samples:" << dropped.str());
  }
  else
  {
    ROS_INFO_STREAM("Outlier rejection enabled: pos_thr_m=" << pos_thr_m
                    << ", rot_thr_deg=" << rot_thr_deg
                    << ", dropped=0, kept=" << data.size());
  }
}

void logFkAllSamples(const std::string& urdf,
                     const std::vector<robot_calibration_msgs::CalibrationData>& data,
                     const std::string& fk_base,
                     const std::string& fk_tip_camera,
                     const std::string& fk_tip_board,
                     const std::string& phase_tag,
                     const robot_calibration::CalibrationOffsetParser& offsets,
                     const std::string& obs_camera_sensor,
                     bool print_board_dual_chain)
{
  for (size_t i = 0; i < data.size(); ++i)
  {
    const std::string sid = std::to_string(i);
    robot_calibration::logChainFkFromUrdf(urdf, data[i].joint_states, fk_base, fk_tip_camera,
                                          phase_tag + "_cam_s" + sid, offsets);
    if (print_board_dual_chain)
    {
      robot_calibration::logBoardInBaseUrdfVsCameraCsv(urdf, data[i].joint_states, fk_base, fk_tip_camera,
                                                       fk_tip_board, data[i], obs_camera_sensor, offsets,
                                                       phase_tag + "_s" + sid);
    }
    else
    {
      robot_calibration::logChainFkFromUrdf(urdf, data[i].joint_states, fk_base, fk_tip_board,
                                              phase_tag + "_board_s" + sid, offsets);
    }
  }
}
static void writeOptimizationUsedSampleIds(const std::string& csv_dir, const std::vector<int>& sample_ids)
{
  const std::string path = csv_dir + "/optimization_used_sample_ids.txt";
  std::ofstream f(path.c_str());
  if (!f)
  {
    ROS_WARN_STREAM("Failed to write: " << path);
    return;
  }
  f << "# CSV sample_id values actually used by optimize_from_csv (order matches internal batch after outlier "
       "rejection)\n";
  for (int sid : sample_ids)
    f << sid << "\n";
  ROS_INFO_STREAM("Wrote " << path << " (" << sample_ids.size() << " samples) for plot_board_error_from_csv");
}

}  // namespace

void logFreeParamsOffsets(const robot_calibration::OptimizationParams& params,
                           robot_calibration::Optimizer& opt,
                           const std::string& tag)
{
  if (params.free_params.empty())
    return;

  auto offsets = opt.getOffsets();
  if (!offsets)
    return;

  ROS_INFO_STREAM("Solved free_params (" << tag << "):");
  for (const auto& name : params.free_params)
  {
    const double v_rad = offsets->get(name);
    const double v_deg = v_rad * 180.0 / M_PI;
    ROS_INFO_STREAM("  " << name << ": " << v_rad << " rad (" << v_deg << " deg)");
  }
}

void logFreeFramesOffsets(const robot_calibration::OptimizationParams& params,
                           robot_calibration::Optimizer& opt,
                           const std::string& tag)
{
  if (params.free_frames.empty())
    return;

  auto offsets = opt.getOffsets();
  if (!offsets)
    return;

  ROS_INFO_STREAM("Solved free_frames (" << tag << "):");
  for (const auto& ff : params.free_frames)
  {
    const std::string& name = ff.name;
    const double x = offsets->get(name + "_x");
    const double y = offsets->get(name + "_y");
    const double z = offsets->get(name + "_z");

    // Rotation offsets are stored in axis-magnitude form (a,b,c) even if roll/pitch/yaw are disabled.
    const double a = offsets->get(name + "_a");
    const double b = offsets->get(name + "_b");
    const double c = offsets->get(name + "_c");

    const double angle = std::sqrt(a * a + b * b + c * c);

    // Convert axis-magnitude (axis * angle) -> quaternion wxyz for printing only.
    // axis = v/|v|, angle = |v| (KDL uses the same convention in rotation_from_axis_magnitude()).
    double qw = 1.0, qx = 0.0, qy = 0.0, qz = 0.0;
    if (angle > 1e-12)
    {
      const double half = angle * 0.5;
      const double s = std::sin(half);
      qw = std::cos(half);
      qx = (a / angle) * s;
      qy = (b / angle) * s;
      qz = (c / angle) * s;
    }

    ROS_INFO_STREAM("  " << name
                      << ": xyz(m)=[" << x << ", " << y << ", " << z << "]"
                      << ", axis-magnitude(rad)=[" << a << ", " << b << ", " << c << "]"
                      << " (|v|=" << angle << ")"
                      << ", quat_wxyz=[" << qw << ", " << qx << ", " << qy << ", " << qz << "]");

    {
      const KDL::Rotation R_off = robot_calibration::rotation_from_axis_magnitude(a, b, c);
      double rr, pp, yy;
      R_off.GetRPY(rr, pp, yy);
      const double kRad2Deg = 180.0 / M_PI;
      ROS_INFO_STREAM("  " << name
                        << " (extrinsic): xyz_mm=[" << (x * 1000.0) << ", " << (y * 1000.0) << ", " << (z * 1000.0) << "]"
                        << ", rpy_deg=[roll=" << (rr * kRad2Deg) << ", pitch=" << (pp * kRad2Deg) << ", yaw=" << (yy * kRad2Deg) << "]"
                        << " (axis-magnitude -> KDL::GetRPY, same as URDF rpy convention)");
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_calibration");
  ros::NodeHandle nh("~");
  ros::NodeHandle nh_global;

  bool verbose = false;
  nh.param<bool>("verbose", verbose, false);

  std::string csv_dir = "/tmp/calibration_csv";
  nh.param<std::string>("csv_dir", csv_dir, csv_dir);

  bool print_fk = true;
  nh.param<bool>("print_fk", print_fk, true);
  std::string fk_base = "base_link";
  nh.param<std::string>("fk_base", fk_base, fk_base);
  std::string fk_tip_camera = "head_camera_color_optical_frame";
  nh.param<std::string>("fk_tip_camera", fk_tip_camera, fk_tip_camera);
  std::string fk_tip_board = "checkerboard_link";
  nh.param<std::string>("fk_tip_board", fk_tip_board, fk_tip_board);

  // Must match kuavo_head_capture.yaml camera_sensor_name (CSV features for camera_to_base)
  std::string obs_camera_sensor = "camera_to_base";
  nh.param<std::string>("obs_camera_sensor", obs_camera_sensor, obs_camera_sensor);
  bool print_obs_cam_board = true;
  nh.param<bool>("print_obs_cam_board", print_obs_cam_board, true);
  // BOARD_IN_BASE[camera_chain+CSV] repeats same CSV feature0; default true to print post_opt too.
  bool print_obs_cam_board_post = true;
  nh.param<bool>("print_obs_cam_board_post", print_obs_cam_board_post, print_obs_cam_board_post);

  std::vector<robot_calibration_msgs::CalibrationData> data;
  std::vector<int> csv_sample_ids;
  if (!robot_calibration::readCalibrationFromCsv(csv_dir, data, &csv_sample_ids))
  {
    ROS_FATAL_STREAM("Failed to load calibration from csv dir: " << csv_dir);
    return -1;
  }
  if (csv_sample_ids.size() != data.size())
  {
    ROS_FATAL_STREAM("Internal error: sample_ids size mismatch vs data (" << csv_sample_ids.size()
                                                                         << " vs " << data.size() << ")");
    return -1;
  }

  // Optional: remap board-model points from corner-origin to center-origin in-memory,
  // so existing CSV can be reused without re-capturing.
  bool remap_board_obs_origin = false;
  nh.param<bool>("remap_board_obs_origin", remap_board_obs_origin, remap_board_obs_origin);
  bool remap_board_obs_auto_from_grid = true;
  nh.param<bool>("remap_board_obs_auto_from_grid", remap_board_obs_auto_from_grid, remap_board_obs_auto_from_grid);
  std::string board_obs_sensor = "tag_to_base";
  nh.param<std::string>("board_obs_sensor", board_obs_sensor, board_obs_sensor);
  std::string board_obs_frame = "checkerboard_link";
  nh.param<std::string>("board_obs_frame", board_obs_frame, board_obs_frame);
  int board_points_x = 11;
  int board_points_y = 8;
  double board_square_size = 0.03;
  nh.param<int>("board_points_x", board_points_x, board_points_x);
  nh.param<int>("board_points_y", board_points_y, board_points_y);
  nh.param<double>("board_square_size", board_square_size, board_square_size);
  double board_shift_x = (std::max(1, board_points_x) - 1) * board_square_size * 0.5;
  double board_shift_y = (std::max(1, board_points_y) - 1) * board_square_size * 0.5;
  double board_shift_z = 0.0;
  nh.param<double>("board_obs_shift_z", board_shift_z, board_shift_z);
  if (!remap_board_obs_auto_from_grid)
  {
    // When disabled, board_obs_shift_* are fully manual.
    nh.param<double>("board_obs_shift_x", board_shift_x, board_shift_x);
    nh.param<double>("board_obs_shift_y", board_shift_y, board_shift_y);
  }
  if (remap_board_obs_origin)
  {
    ROS_INFO_STREAM("Board remap config: auto_from_grid="
                    << (remap_board_obs_auto_from_grid ? "true" : "false")
                    << ", points_x=" << board_points_x
                    << ", points_y=" << board_points_y
                    << ", square_size=" << board_square_size);
    remapBoardObservationOrigin(data, board_obs_sensor, board_obs_frame,
                                board_shift_x, board_shift_y, board_shift_z);
  }

  // Load URDF from parameter.
  std_msgs::String description_msg;
  if (!nh_global.getParam("/robot_description", description_msg.data))
  {
    ROS_FATAL("Missing global parameter: /robot_description");
    return -1;
  }

  robot_calibration::CalibrationOffsetParser nominal_offsets;

  // Outlier rejection (simple thresholds).
  bool outlier_reject_enable = false;
  nh.param<bool>("outlier_reject_enable", outlier_reject_enable, outlier_reject_enable);
  double outlier_reject_pos_m = 0.1;
  nh.param<double>("outlier_reject_pos_m", outlier_reject_pos_m, outlier_reject_pos_m);
  double outlier_reject_rot_deg = 10.0;
  nh.param<double>("outlier_reject_rot_deg", outlier_reject_rot_deg, outlier_reject_rot_deg);
  if (outlier_reject_enable)
  {
    rejectOutliersByDelta(description_msg.data,
                          fk_base, fk_tip_camera, fk_tip_board,
                          obs_camera_sensor, board_obs_sensor,
                          nominal_offsets,
                          outlier_reject_pos_m, outlier_reject_rot_deg,
                          data,
                          csv_sample_ids);
  }

  writeOptimizationUsedSampleIds(csv_dir, csv_sample_ids);

  if (print_fk)
  {
    ROS_INFO("FK (pre-optimize, nominal URDF / zero calibration offsets):");
    if (print_obs_cam_board)
    {
      ROS_INFO("OPT_INPUT logs enabled: print raw optimization inputs only "
               "(FK base->camera/base->board and CSV feature points, no extra reconstruction). "
               "Set ~print_obs_cam_board:=false to use plain FK[..._board] only.");
    }
    logFkAllSamples(description_msg.data, data, fk_base, fk_tip_camera, fk_tip_board, "pre_opt", nominal_offsets,
                    obs_camera_sensor, print_obs_cam_board);
  }

  // Create optimizer.
  robot_calibration::Optimizer opt(description_msg.data);

  // Load optimization steps (same mechanism as calibrate.cpp).
  robot_calibration::OptimizationParams params;

  XmlRpc::XmlRpcValue cal_steps;
  if (nh.getParam("cal_steps", cal_steps))
  {
    if (cal_steps.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_FATAL("Parameter 'cal_steps' should be a struct.");
      return -1;
    }

    XmlRpc::XmlRpcValue::iterator it;
    size_t step = 0;
    const size_t max_step = (cal_steps.size() > 0) ? cal_steps.size() : 1;

    for (step = 0, it = cal_steps.begin(); step < max_step; step++, it++)
    {
      std::string name = static_cast<std::string>(it->first);
      ros::NodeHandle cal_steps_handle(nh, "cal_steps/" + name);
      params.LoadFromROS(cal_steps_handle);
      opt.optimize(params, data, verbose);
      logFreeParamsOffsets(params, opt, "step_" + name);
      logFreeFramesOffsets(params, opt, "step_" + name);
    }
  }
  else
  {
    // Single step calibration
    params.LoadFromROS(nh);
    opt.optimize(params, data, verbose);
    logFreeParamsOffsets(params, opt, "single_step");
    logFreeFramesOffsets(params, opt, "single_step");
  }

  if (print_fk && opt.getOffsets())
  {
    ROS_INFO("FK (post-optimize, with solved calibration offsets applied in ChainModel):");
    if (print_obs_cam_board && print_obs_cam_board_post)
    {
      ROS_INFO("OPT_INPUT post_opt: same CSV as pre_opt; ~print_obs_cam_board_post:=false to hide.");
    }
    logFkAllSamples(description_msg.data, data, fk_base, fk_tip_camera, fk_tip_board, "post_opt", *opt.getOffsets(),
                    obs_camera_sensor, print_obs_cam_board && print_obs_cam_board_post);
  }

  robot_calibration::exportResults(opt, description_msg.data, data);
  ROS_INFO("Done optimizing from csv.");
  return 0;
}

