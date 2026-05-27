#include <robot_calibration/fk_debug_utils.h>

#include <cmath>
#include <iomanip>
#include <sstream>

#include <ros/ros.h>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <Eigen/Dense>

#include <robot_calibration/calibration/offset_parser.h>
#include <robot_calibration/models/chain.h>

#include <functional>
#include <memory>

namespace robot_calibration
{
namespace
{
struct FkDebugCache
{
  bool valid{false};
  std::size_t urdf_hash{0};
  std::unique_ptr<KDL::Tree> tree;
};

FkDebugCache& getCache()
{
  static FkDebugCache cache;
  return cache;
}

int getSensorIndexLocal(const robot_calibration_msgs::CalibrationData& msg,
                        const std::string& sensor)
{
  for (size_t i = 0; i < msg.observations.size(); ++i)
  {
    if (msg.observations[i].sensor_name == sensor)
      return static_cast<int>(i);
  }
  return -1;
}

void appendFramePose(std::ostringstream& oss, const KDL::Frame& T)
{
  oss.setf(std::ios::fixed);
  oss.precision(4);
  double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
  T.M.GetQuaternion(qx, qy, qz, qw);
  // Make quaternion sign consistent so that angle is always in [0, 180] deg.
  if (qw < 0.0)
  {
    qw = -qw;
    qx = -qx;
    qy = -qy;
    qz = -qz;
  }

  oss << "p_m=[" << T.p.x() << ", " << T.p.y() << ", " << T.p.z() << "] "
      << "p_mm=[" << (T.p.x() * 1000.0) << ", " << (T.p.y() * 1000.0) << ", " << (T.p.z() * 1000.0) << "] "
      << "quat_wxyz=[" << qw << ", " << qx << ", " << qy << ", " << qz << "]";
}

bool estimateRigidTransformSvd(const std::vector<geometry_msgs::PointStamped>& src,
                              const std::vector<geometry_msgs::PointStamped>& dst,
                              Eigen::Matrix3d* R,
                              Eigen::Vector3d* t)
{
  if (!R || !t)
    return false;
  const size_t n = std::min(src.size(), dst.size());
  if (n < 4)
    return false;

  Eigen::MatrixXd X(3, n);
  Eigen::MatrixXd Y(3, n);
  for (size_t i = 0; i < n; ++i)
  {
    X(0, i) = src[i].point.x;
    X(1, i) = src[i].point.y;
    X(2, i) = src[i].point.z;
    Y(0, i) = dst[i].point.x;
    Y(1, i) = dst[i].point.y;
    Y(2, i) = dst[i].point.z;
  }

  const Eigen::Vector3d mx = X.rowwise().mean();
  const Eigen::Vector3d my = Y.rowwise().mean();
  X.colwise() -= mx;
  Y.colwise() -= my;

  const Eigen::Matrix3d H = X * Y.transpose();
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d V = svd.matrixV();
  Eigen::Matrix3d Rxy = V * U.transpose();
  if (Rxy.determinant() < 0.0)
  {
    V.col(2) *= -1.0;
    Rxy = V * U.transpose();
  }

  *R = Rxy;
  *t = my - (*R) * mx;
  return true;
}

bool ensureTreeCache(const std::string& urdf_xml, const std::string& tag)
{
  if (urdf_xml.empty())
  {
    ROS_WARN_STREAM("FK debug (" << tag << "): empty URDF string");
    return false;
  }
  const std::size_t h = std::hash<std::string>{}(urdf_xml);
  auto& cache = getCache();
  if (!cache.valid || cache.urdf_hash != h || !cache.tree)
  {
    urdf::Model model;
    if (!model.initString(urdf_xml))
    {
      ROS_WARN_STREAM("FK debug (" << tag << "): failed to parse URDF");
      return false;
    }
    KDL::Tree new_tree;
    if (!kdl_parser::treeFromUrdfModel(model, new_tree))
    {
      ROS_WARN_STREAM("FK debug (" << tag << "): failed to build KDL tree");
      return false;
    }
    cache.tree = std::make_unique<KDL::Tree>(new_tree);
    cache.urdf_hash = h;
    cache.valid = true;
  }
  return true;
}

bool getChainFkFrame(const std::string& urdf_xml,
                     const sensor_msgs::JointState& js,
                     const std::string& root,
                     const std::string& tip,
                     const CalibrationOffsetParser& offsets,
                     KDL::Frame* out,
                     const std::string& tag)
{
  if (!out || !ensureTreeCache(urdf_xml, tag))
    return false;
  try
  {
    auto& cache = getCache();
    ChainModel chain("_fk_debug_", *cache.tree, root, tip);
    *out = chain.getChainFK(offsets, js);
    return true;
  }
  catch (const std::exception& e)
  {
    ROS_WARN_STREAM("FK debug (" << tag << "): chain " << root << " -> " << tip << " failed: " << e.what());
    return false;
  }
}
}  // namespace

void logChainFkFromUrdf(const std::string& urdf_xml,
                        const sensor_msgs::JointState& js,
                        const std::string& root,
                        const std::string& tip,
                        const std::string& tag,
                        const CalibrationOffsetParser& offsets)
{
  KDL::Frame fk;
  if (!getChainFkFrame(urdf_xml, js, root, tip, offsets, &fk, tag))
    return;
  std::ostringstream oss;
  appendFramePose(oss, fk);
  ROS_INFO_STREAM("FK[" << tag << "] " << root << " -> " << tip << " : " << oss.str());
}

void logBoardInBaseUrdfVsCameraCsv(const std::string& urdf_xml,
                                   const sensor_msgs::JointState& js,
                                   const std::string& fk_base,
                                   const std::string& fk_tip_camera,
                                   const std::string& fk_tip_board,
                                   const robot_calibration_msgs::CalibrationData& data,
                                   const std::string& camera_sensor_name,
                                   const CalibrationOffsetParser& offsets,
                                   const std::string& tag)
{
  KDL::Frame T_board_to_base_urdf;
  if (!getChainFkFrame(urdf_xml, js, fk_base, fk_tip_board, offsets, &T_board_to_base_urdf, tag))
    return;

  KDL::Frame T_cam_to_base;
  if (!getChainFkFrame(urdf_xml, js, fk_base, fk_tip_camera, offsets, &T_cam_to_base, tag))
    return;

  const int ia = getSensorIndexLocal(data, camera_sensor_name);
  if (ia < 0)
  {
    ROS_WARN_STREAM("BOARD_IN_BASE[camera_chain+CSV " << tag << "]: no observation '" << camera_sensor_name << "'.");
    return;
  }
  const auto& feats_cam = data.observations[static_cast<size_t>(ia)].features;
  if (feats_cam.empty())
  {
    ROS_WARN_STREAM("BOARD_IN_BASE[camera_chain+CSV " << tag << "]: empty features for '" << camera_sensor_name << "'.");
    return;
  }

  // Find board-frame points as provided in CalibrationData (no extra solving/reconstruction).
  int ib = -1;
  for (size_t o = 0; o < data.observations.size(); ++o)
  {
    if (static_cast<int>(o) == ia || data.observations[o].features.empty())
      continue;
    if (data.observations[o].features[0].header.frame_id == fk_tip_board)
    {
      ib = static_cast<int>(o);
      break;
    }
  }

  std::ostringstream oss_base_cam;
  appendFramePose(oss_base_cam, T_cam_to_base);
  ROS_INFO_STREAM("OPT_INPUT[base_to_camera_fk " << tag << "] "
                  << fk_base << " -> " << fk_tip_camera << " : " << oss_base_cam.str());

  std::ostringstream oss_base_board;
  appendFramePose(oss_base_board, T_board_to_base_urdf);
  ROS_INFO_STREAM("OPT_INPUT[base_to_board_fk " << tag << "] "
                  << fk_base << " -> " << fk_tip_board << " : " << oss_base_board.str());

  const auto& f0_cam = feats_cam[0];
  ROS_INFO_STREAM("OPT_INPUT[camera_obs_feature0 " << tag << "] "
                  << "sensor=" << camera_sensor_name
                  << " frame_id=" << f0_cam.header.frame_id
                  << " p_m=[" << f0_cam.point.x << ", " << f0_cam.point.y << ", " << f0_cam.point.z << "]"
                  << " p_mm=[" << (f0_cam.point.x * 1000.0) << ", " << (f0_cam.point.y * 1000.0) << ", "
                  << (f0_cam.point.z * 1000.0) << "]"
                  << " features_n=" << feats_cam.size());

  if (ib >= 0)
  {
    const auto& feats_board = data.observations[static_cast<size_t>(ib)].features;
    const auto& f0_board = feats_board[0];
    ROS_INFO_STREAM("OPT_INPUT[board_obs_feature0 " << tag << "] "
                    << "sensor=" << data.observations[static_cast<size_t>(ib)].sensor_name
                    << " frame_id=" << f0_board.header.frame_id
                    << " p_m=[" << f0_board.point.x << ", " << f0_board.point.y << ", " << f0_board.point.z << "]"
                    << " p_mm=[" << (f0_board.point.x * 1000.0) << ", " << (f0_board.point.y * 1000.0) << ", "
                    << (f0_board.point.z * 1000.0) << "]"
                    << " features_n=" << feats_board.size());

    // 额外打印：利用 (board_frame points) 与 (camera_frame points) 估计 base<-board(obs)
    // 只用于调试，与优化无关。
    ros::NodeHandle pnh("~");
    bool print_board_obs_pose = false;
    pnh.param<bool>("print_board_obs_pose", print_board_obs_pose, false);
    if (print_board_obs_pose)
    {
      Eigen::Matrix3d R_cb;  // board -> cam
      Eigen::Vector3d t_cb;
      if (estimateRigidTransformSvd(feats_board, feats_cam, &R_cb, &t_cb))
      {
        Eigen::Matrix3d R_base_cam;
        Eigen::Vector3d t_base_cam;
        for (int r = 0; r < 3; ++r)
          for (int c = 0; c < 3; ++c)
            R_base_cam(r, c) = T_cam_to_base.M(r, c);
        t_base_cam << T_cam_to_base.p.x(), T_cam_to_base.p.y(), T_cam_to_base.p.z();

        // 约定：
        // - estimateRigidTransformSvd(feats_board -> feats_cam) 得到的是 cam<-board，即：
        //     p_cam = R_cb * p_board + t_cb
        // - T_cam_to_base 是 base<-cam，即：
        //     p_base = R_base_cam * p_cam + t_base_cam
        // 所以 base<-board(obs) 为：
        //     p_base = R_base_cam * (R_cb * p_board + t_cb) + t_base_cam
        const Eigen::Matrix3d R_base_board_obs = R_base_cam * R_cb;
        const Eigen::Vector3d t_base_board_obs = R_base_cam * t_cb + t_base_cam;

        KDL::Rotation Rk(
            R_base_board_obs(0, 0), R_base_board_obs(0, 1), R_base_board_obs(0, 2),
            R_base_board_obs(1, 0), R_base_board_obs(1, 1), R_base_board_obs(1, 2),
            R_base_board_obs(2, 0), R_base_board_obs(2, 1), R_base_board_obs(2, 2));
        KDL::Vector pk(t_base_board_obs(0), t_base_board_obs(1), t_base_board_obs(2));
        KDL::Frame T_board_obs(Rk, pk);

        std::ostringstream oss_obs;
        appendFramePose(oss_obs, T_board_obs);
        ROS_INFO_STREAM("BOARD_IN_BASE[cam_chain+csv_fit " << tag << "] "
                        << fk_base << " -> " << fk_tip_board << " : " << oss_obs.str());

        // delta(urdf^-1 * obs)
        const KDL::Frame T_delta = T_board_to_base_urdf.Inverse() * T_board_obs;
        double rr = 0.0, pp = 0.0, yy = 0.0;
        T_delta.M.GetRPY(rr, pp, yy);
        ROS_INFO_STREAM("DELTA[urdf^-1*obs " << tag << "] "
                        << "t_mm=[" << (T_delta.p.x() * 1000.0) << ", " << (T_delta.p.y() * 1000.0) << ", "
                        << (T_delta.p.z() * 1000.0) << "] "
                        << "rpy_deg=["
                        << (rr * 180.0 / M_PI) << ", " << (pp * 180.0 / M_PI) << ", " << (yy * 180.0 / M_PI) << "]");
      }
      else
      {
        ROS_WARN_STREAM("BOARD_IN_BASE[cam_chain+csv_fit " << tag << "]: not enough points for SVD fit.");
      }
    }
  }
  else
  {
    ROS_WARN_STREAM("OPT_INPUT[board_obs_feature0 " << tag << "]: no board-frame observation found (frame_id="
                    << fk_tip_board << ").");
  }
}

}  // namespace robot_calibration
