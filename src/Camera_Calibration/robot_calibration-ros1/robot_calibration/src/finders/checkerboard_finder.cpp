/*
 * Copyright (C) 2023 Michael Ferguson
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

#include <pluginlib/class_list_macros.hpp>
#include <robot_calibration/capture/checkerboard_finder.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <sstream>

PLUGINLIB_EXPORT_CLASS(robot_calibration::CheckerboardFinder<sensor_msgs::PointCloud2>, robot_calibration::FeatureFinder)
PLUGINLIB_EXPORT_CLASS(robot_calibration::CheckerboardFinder<sensor_msgs::ImagePtr>, robot_calibration::FeatureFinder)

namespace robot_calibration
{

// We use a number of PC2 iterators, define the indexes here
const unsigned X = 0;
const unsigned Y = 1;
const unsigned Z = 2;

static inline void frameToXYZRPYmmDeg(const KDL::Frame& T,
                                      double& x_mm, double& y_mm, double& z_mm,
                                      double& roll_deg, double& pitch_deg, double& yaw_deg)
{
  x_mm = T.p.x() * 1000.0;
  y_mm = T.p.y() * 1000.0;
  z_mm = T.p.z() * 1000.0;
  double r = 0.0, p = 0.0, y = 0.0;
  T.M.GetRPY(r, p, y);
  roll_deg = r * 180.0 / M_PI;
  pitch_deg = p * 180.0 / M_PI;
  yaw_deg = y * 180.0 / M_PI;
}

static inline std::vector<std::string> chainJointNames(const KDL::Chain& chain)
{
  std::vector<std::string> names;
  names.reserve(chain.getNrOfJoints());
  for (unsigned i = 0; i < chain.getNrOfSegments(); ++i)
  {
    const auto& j = chain.getSegment(i).getJoint();
    if (j.getType() != KDL::Joint::None)
      names.push_back(j.getName());
  }
  return names;
}

static inline bool positionFromJointState(const sensor_msgs::JointState& js,
                                          const std::string& name,
                                          double& out)
{
  for (size_t i = 0; i < js.name.size() && i < js.position.size(); ++i)
  {
    if (js.name[i] == name)
    {
      out = js.position[i];
      return true;
    }
  }
  return false;
}

static inline bool computeFK(const KDL::Chain& chain,
                             const std::vector<std::string>& joint_names,
                             const sensor_msgs::JointState& js,
                             KDL::ChainFkSolverPos_recursive& solver,
                             KDL::Frame& out)
{
  if (joint_names.size() != chain.getNrOfJoints())
    return false;
  KDL::JntArray q(chain.getNrOfJoints());
  for (size_t i = 0; i < joint_names.size(); ++i)
  {
    double v = 0.0;
    if (!positionFromJointState(js, joint_names[i], v))
      return false;
    q(static_cast<unsigned>(i)) = v;
  }
  return solver.JntToCart(q, out) >= 0;
}

template <typename T>
CheckerboardFinder<T>::CheckerboardFinder() :
  waiting_(false)
{
}

template <typename T>
bool CheckerboardFinder<T>::init(const std::string& name,
                                 ros::NodeHandle & nh)
{
  if (!FeatureFinder::init(name, nh))
    return false;

  // Setup Scriber
  std::string topic_name;
  nh.param<std::string>("topic", topic_name, "/points");
  subscriber_ = nh.subscribe(topic_name,
                             1,
                             &CheckerboardFinder::cameraCallback,
                             this);

  // Size of checkerboard
  nh.param<int>("points_x", points_x_, 5);
  nh.param<int>("points_y", points_y_, 4);
  nh.param<double>("size", square_size_, 0.0245);
  if (points_x_ % 2 == 1 && points_y_ % 2 == 1)
  {
    ROS_ERROR("Checkerboard is symmetric - orientation estimate can be wrong");
  }

  // Should we include debug image/cloud in observations
  nh.param<bool>("debug", output_debug_, false);

  // Name of checkerboard frame that will be used during optimization
  nh.param<std::string>("frame_id", frame_id_, "checkerboard");
  nh.param<bool>("origin_at_center", origin_at_center_, false);
  nh.param<double>("board_offset_x", board_offset_x_, 0.0);
  nh.param<double>("board_offset_y", board_offset_y_, 0.0);
  nh.param<double>("board_offset_z", board_offset_z_, 0.0);
  if (origin_at_center_)
  {
    ROS_INFO_STREAM("CheckerboardFinder board origin mode: center"
                    << ", board_offset_xyz(m)=[" << board_offset_x_ << ", "
                    << board_offset_y_ << ", " << board_offset_z_ << "]");
  }
  else
  {
    ROS_INFO_STREAM("CheckerboardFinder board origin mode: corner"
                    << ", board_offset_xyz(m)=[" << board_offset_x_ << ", "
                    << board_offset_y_ << ", " << board_offset_z_ << "]");
  }

  // Root frame for debug pose printouts (base_link by default)
  nh.param<std::string>("base_frame", base_frame_, std::string("base_link"));
  // Camera frame used for base<-camera lookup (default to /points optical frame)
  nh.param<std::string>("camera_frame", camera_frame_, std::string("head_camera_color_optical_frame"));

  // Name of the sensor model that will be used during optimization
  nh.param<std::string>("camera_sensor_name", camera_sensor_name_, "camera");
  nh.param<std::string>("chain_sensor_name", chain_sensor_name_, "arm");

  // Publish where checkerboard points were seen
  publisher_ = nh.advertise<sensor_msgs::PointCloud2>(getName() + "_points", 10);

  // Setup to get camera depth info
  if (!depth_camera_manager_.init(nh))
  {
    // Error will have been printed by manager
    return false;
  }

  // Build KDL chains for FK debug printing (no TF usage).
  // This is only used for debug printouts of base<-tag(obs/urdf/delta).
  fk_ready_ = false;
  cam_joint_names_.clear();
  tag_joint_names_.clear();
  fk_cam_solver_.reset();
  fk_tag_solver_.reset();
  {
    std::string urdf_xml;
    ros::NodeHandle nh_root;
    if (!nh_root.getParam("/robot_description", urdf_xml))
    {
      ROS_WARN("CheckerboardFinder: cannot load /robot_description, FK debug disabled.");
    }
    else
    {
      urdf::Model model;
      if (!model.initString(urdf_xml))
      {
        ROS_WARN("CheckerboardFinder: failed to parse URDF from /robot_description, FK debug disabled.");
      }
      else if (!kdl_parser::treeFromUrdfModel(model, kdl_tree_))
      {
        ROS_WARN("CheckerboardFinder: failed to build KDL tree from URDF, FK debug disabled.");
      }
      else
      {
        const bool ok_cam = kdl_tree_.getChain(base_frame_, camera_frame_, chain_base_to_cam_);
        const bool ok_tag = kdl_tree_.getChain(base_frame_, frame_id_, chain_base_to_tag_);
        if (!ok_cam)
          ROS_WARN_STREAM("CheckerboardFinder: failed to build KDL chain " << base_frame_ << " -> " << camera_frame_);
        if (!ok_tag)
          ROS_WARN_STREAM("CheckerboardFinder: failed to build KDL chain " << base_frame_ << " -> " << frame_id_);
        if (ok_cam && ok_tag)
        {
          cam_joint_names_ = chainJointNames(chain_base_to_cam_);
          tag_joint_names_ = chainJointNames(chain_base_to_tag_);
          fk_cam_solver_.reset(new KDL::ChainFkSolverPos_recursive(chain_base_to_cam_));
          fk_tag_solver_.reset(new KDL::ChainFkSolverPos_recursive(chain_base_to_tag_));
          fk_ready_ = true;
        }
      }
    }
  }

  return true;
}

template <typename T>
void CheckerboardFinder<T>::cameraCallback(const T& msg)
{
  if (waiting_)
  {
    msg_ = msg;
    waiting_ = false;

    static int cb_debug = 0;
    if (cb_debug < 10)
    {
      ROS_INFO("cameraCallback(): received camera message.");
      cb_debug++;
    }
  }
}

// Returns true if we got a message, false if we timeout
template <typename T>
bool CheckerboardFinder<T>::waitForMsg()
{
  // Initial wait cycle so that camera is definitely up to date.
  ros::Duration(1/10.0).sleep();

  waiting_ = true;
  int count = 250;
  while (--count)
  {
    if (!waiting_)
    {
      // success
      ROS_INFO_THROTTLE(1.0, "waitForMsg(): got camera message.");
      return true;
    }
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }
  ROS_ERROR("Failed to get message");
  return !waiting_;
}

template <typename T>
bool CheckerboardFinder<T>::find(robot_calibration_msgs::CalibrationData * msg)
{
  // Try up to 50 frames
  for (int i = 0; i < 50; ++i)
  {
    // temporary copy of msg, so we throw away all changes if findInternal() returns false
    robot_calibration_msgs::CalibrationData tmp_msg(*msg);
    if (findInternal(&tmp_msg))
    {
      *msg = tmp_msg;
      return true;
    }
  }
  return false;
}

template <>
bool CheckerboardFinder<sensor_msgs::PointCloud2>::findInternal(robot_calibration_msgs::CalibrationData * msg)
{
  ROS_INFO_THROTTLE(1.0, "CheckerboardFinder<PointCloud2>::findInternal(): waiting for point cloud message...");
  // Get cloud
  if (!waitForMsg())
  {
    ROS_ERROR("No point cloud data");
    return false;
  }

  if (msg_.height == 1)
  {
    ROS_ERROR("OpenCV does not support unorganized cloud/image.");
    return false;
  }

  // Get an image message from point cloud (用于检测 2D 角点)
  sensor_msgs::ImagePtr image_msg(new sensor_msgs::Image);
  sensor_msgs::PointCloud2ConstIterator<uint8_t> rgb(msg_, "rgb");
  image_msg->encoding = "bgr8";
  image_msg->height = msg_.height;
  image_msg->width = msg_.width;
  image_msg->step = image_msg->width * sizeof (uint8_t) * 3;
  image_msg->data.resize(image_msg->step * image_msg->height);
  for (size_t y = 0; y < msg_.height; y++)
  {
    for (size_t x = 0; x < msg_.width; x++)
    {
      uint8_t* pixel = &(image_msg->data[y * image_msg->step + x * 3]);
      pixel[0] = rgb[0];
      pixel[1] = rgb[1];
      pixel[2] = rgb[2];
      ++rgb;
    }
  }

  std::vector<cv::Point2f> points;
  if (!findCheckerboardPoints(image_msg, points))
  {
    ROS_WARN("CheckerboardFinder<PointCloud2>::findInternal(): checkerboard NOT found in this frame.");
    return false;
  }

  ROS_INFO("CheckerboardFinder<PointCloud2>::findInternal(): Found the checkboard, running solvePnP...");

  // 方案 A：用 solvePnP 得到棋盘在相机坐标系下的位姿，再生成相机侧 3D 角点
  cv::Mat rvec, tvec;
  try
  {
    // 轻量级调试：直接用点云估计棋盘格实际物理尺寸（不修改任何标定流程）
    // 按行/列对邻接角点，从 msg_ 中取 3D 点，计算欧氏距离并取平均。
    // 用 Welford 在线算法统计均值/方差，避免存全量样本
    double mean_h = 0.0, m2_h = 0.0;
    double mean_v = 0.0, m2_v = 0.0;
    double max_abs_err_h = 0.0;
    double max_abs_err_v = 0.0;
    int    cnt_h = 0;
    int    cnt_v = 0;
    {
      // 为了边界检查方便，提前缓存宽高
      const int img_w = static_cast<int>(msg_.width);
      const int img_h = static_cast<int>(msg_.height);

      for (int j = 0; j < points_y_; ++j)
      {
        for (int i = 0; i < points_x_; ++i)
        {
          int idx = j * points_x_ + i;

          // 横向相邻 (i, j) -> (i+1, j)
          if (i + 1 < points_x_)
          {
            int idx2 = idx + 1;
            int u0 = static_cast<int>(points[idx].x + 0.5f);
            int v0 = static_cast<int>(points[idx].y + 0.5f);
            int u1 = static_cast<int>(points[idx2].x + 0.5f);
            int v1 = static_cast<int>(points[idx2].y + 0.5f);

            if (u0 >= 0 && v0 >= 0 && u1 >= 0 && v1 >= 0 &&
                u0 < img_w && u1 < img_w && v0 < img_h && v1 < img_h)
            {
              sensor_msgs::PointCloud2ConstIterator<float> it0(msg_, "x");
              sensor_msgs::PointCloud2ConstIterator<float> it1(msg_, "x");
              it0 += v0 * img_w + u0;
              it1 += v1 * img_w + u1;

              double x0 = it0[0], y0 = it0[1], z0 = it0[2];
              double x1 = it1[0], y1 = it1[1], z1 = it1[2];
              if (std::isfinite(x0) && std::isfinite(y0) && std::isfinite(z0) &&
                  std::isfinite(x1) && std::isfinite(y1) && std::isfinite(z1))
              {
                double dx = x1 - x0;
                double dy = y1 - y0;
                double dz = z1 - z0;
                double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
                if (std::isfinite(dist) && dist > 0.0)
                {
                  cnt_h++;
                  // 更新均值/方差
                  double delta = dist - mean_h;
                  mean_h += delta / static_cast<double>(cnt_h);
                  double delta2 = dist - mean_h;
                  m2_h += delta * delta2;
                  // 与配置 size 的最大绝对偏差
                  double abs_err = std::fabs(dist - square_size_);
                  if (abs_err > max_abs_err_h) max_abs_err_h = abs_err;
                }
              }
            }
          }

          // 纵向相邻 (i, j) -> (i, j+1)
          if (j + 1 < points_y_)
          {
            int idx2 = idx + points_x_;
            int u0 = static_cast<int>(points[idx].x + 0.5f);
            int v0 = static_cast<int>(points[idx].y + 0.5f);
            int u1 = static_cast<int>(points[idx2].x + 0.5f);
            int v1 = static_cast<int>(points[idx2].y + 0.5f);

            if (u0 >= 0 && v0 >= 0 && u1 >= 0 && v1 >= 0 &&
                u0 < img_w && u1 < img_w && v0 < img_h && v1 < img_h)
            {
              sensor_msgs::PointCloud2ConstIterator<float> it0(msg_, "x");
              sensor_msgs::PointCloud2ConstIterator<float> it1(msg_, "x");
              it0 += v0 * img_w + u0;
              it1 += v1 * img_w + u1;

              double x0 = it0[0], y0 = it0[1], z0 = it0[2];
              double x1 = it1[0], y1 = it1[1], z1 = it1[2];
              if (std::isfinite(x0) && std::isfinite(y0) && std::isfinite(z0) &&
                  std::isfinite(x1) && std::isfinite(y1) && std::isfinite(z1))
              {
                double dx = x1 - x0;
                double dy = y1 - y0;
                double dz = z1 - z0;
                double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
                if (std::isfinite(dist) && dist > 0.0)
                {
                  cnt_v++;
                  double delta = dist - mean_v;
                  mean_v += delta / static_cast<double>(cnt_v);
                  double delta2 = dist - mean_v;
                  m2_v += delta * delta2;
                  double abs_err = std::fabs(dist - square_size_);
                  if (abs_err > max_abs_err_v) max_abs_err_v = abs_err;
                }
              }
            }
          }
        }
      }

      if (cnt_h > 0 || cnt_v > 0)
      {
        double std_h = (cnt_h > 1) ? std::sqrt(m2_h / static_cast<double>(cnt_h)) : 0.0;
        double std_v = (cnt_v > 1) ? std::sqrt(m2_v / static_cast<double>(cnt_v)) : 0.0;
        ROS_INFO_STREAM("Checkerboard square from depth (avg): "
                        << "horizontal=" << mean_h << " m over " << cnt_h
                        << ", vertical=" << mean_v << " m over " << cnt_v
                        << ", config size=" << square_size_ << " m");
        ROS_INFO_STREAM("Checkerboard square from depth (stats): "
                        << "h_std=" << std_h << " m"
                        << ", v_std=" << std_v << " m"
                        << ", h_max_abs_err=" << max_abs_err_h << " m"
                        << ", v_max_abs_err=" << max_abs_err_v << " m");
      }
    }

    // 构造棋盘在自身坐标系下的 3D 点 (Z=0)
    std::vector<cv::Point3f> obj_points;
    obj_points.reserve(points_x_ * points_y_);
    for (int j = 0; j < points_y_; ++j)
    {
      for (int i = 0; i < points_x_; ++i)
      {
        obj_points.emplace_back(static_cast<float>(i * square_size_),
                                static_cast<float>(j * square_size_),
                                0.0f);
      }
    }

    // 2D 像素点
    std::vector<cv::Point2f> img_points = points;

    // 相机内参与畸变系数
    robot_calibration_msgs::ExtendedCameraInfo ext_info = depth_camera_manager_.getDepthCameraInfo();
    const sensor_msgs::CameraInfo& cam_info = ext_info.camera_info;

    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
      cam_info.K[0], 0.0,            cam_info.K[2],
      0.0,            cam_info.K[4], cam_info.K[5],
      0.0,            0.0,            1.0);

    cv::Mat dist_coeffs;
    if (!cam_info.D.empty())
    {
      dist_coeffs = cv::Mat(cam_info.D).clone();
    }
    else
    {
      dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);
    }

    bool pnp_ok = cv::solvePnP(obj_points, img_points,
                               camera_matrix, dist_coeffs,
                               rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
    if (!pnp_ok)
    {
      ROS_WARN("solvePnP failed to compute checkerboard pose (PointCloud2).");
      return false;
    }

    // PnP 重投影误差（像素 RMSE）：反映角点检测 + PnP 拟合一致性
    {
      std::vector<cv::Point2f> reprojected;
      cv::projectPoints(obj_points, rvec, tvec, camera_matrix, dist_coeffs, reprojected);
      if (reprojected.size() == img_points.size() && !reprojected.empty())
      {
        double sum_sq = 0.0;
        double max_err = 0.0;
        for (size_t i = 0; i < reprojected.size(); ++i)
        {
          double dx = static_cast<double>(reprojected[i].x - img_points[i].x);
          double dy = static_cast<double>(reprojected[i].y - img_points[i].y);
          double e = std::sqrt(dx*dx + dy*dy);
          sum_sq += (dx*dx + dy*dy);
          if (e > max_err) max_err = e;
        }
        double rmse_px = std::sqrt(sum_sq / static_cast<double>(reprojected.size()));
        ROS_INFO_STREAM("Checkerboard PnP reprojection error: rmse=" << rmse_px
                        << " px, max=" << max_err << " px, n=" << reprojected.size());
      }
    }

    // 打印棋盘在相机坐标系下的位姿
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    double sy = std::sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) +
                          R.at<double>(1, 0) * R.at<double>(1, 0));
    bool singular = sy < 1e-6;
    double yaw, pitch, roll;
    if (!singular)
    {
      yaw   = std::atan2(R.at<double>(1, 0), R.at<double>(0, 0));
      pitch = std::atan2(-R.at<double>(2, 0), sy);
      roll  = std::atan2(R.at<double>(2, 1), R.at<double>(2, 2));
    }
    else
    {
      yaw   = std::atan2(-R.at<double>(0, 1), R.at<double>(1, 1));
      pitch = std::atan2(-R.at<double>(2, 0), sy);
      roll  = 0.0;
    }
    const double tx_mm = tvec.at<double>(0) * 1000.0;
    const double ty_mm = tvec.at<double>(1) * 1000.0;
    const double tz_mm = tvec.at<double>(2) * 1000.0;
    const double roll_deg = roll * 180.0 / M_PI;
    const double pitch_deg = pitch * 180.0 / M_PI;
    const double yaw_deg = yaw * 180.0 / M_PI;
    ROS_INFO_STREAM("Checkerboard pose in camera frame (from PointCloud2): "
                    << "t_mm=[" << tx_mm << ", " << ty_mm << ", " << tz_mm << "], "
                    << "rpy_deg=[roll=" << roll_deg
                    << ", pitch=" << pitch_deg
                    << ", yaw=" << yaw_deg << "]");

    // base<-tag(obs/urdf/delta) 的调试日志已迁移到 Image 分支（B 方案）输出

    // Create PointCloud2 to publish（可视化仍使用 /points 中的 xyz）
    sensor_msgs::PointCloud2 cloud;
    cloud.width = 0;
    cloud.height = 0;
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = msg_.header.frame_id;
    sensor_msgs::PointCloud2Modifier cloud_mod(cloud);
    cloud_mod.setPointCloud2FieldsByString(1, "xyz");
    cloud_mod.resize(points_x_ * points_y_);
    sensor_msgs::PointCloud2Iterator<float> iter_cloud(cloud, "x");

    // Set msg size
    int idx_cam = msg->observations.size();
    int idx_chain = idx_cam + 1;
    msg->observations.resize(msg->observations.size() + 2);
    msg->observations[idx_cam].sensor_name = camera_sensor_name_;
    msg->observations[idx_chain].sensor_name = chain_sensor_name_;

    msg->observations[idx_cam].features.resize(points_x_ * points_y_);
    msg->observations[idx_chain].features.resize(points_x_ * points_y_);

    // Setup observed points
    geometry_msgs::PointStamped rgbd;
    rgbd.header = msg_.header;
    // 重要：不要强改 frame_id。
    // solvePnP 的外参是基于当前点云消息的相机坐标系（msg_.header.frame_id）求出来的，
    // 若仅修改 frame_id 而不做坐标变换，会导致后续 base<-camera * camera<-tag 的链路不自洽。
    geometry_msgs::PointStamped world;
    world.header.frame_id = frame_id_;

    sensor_msgs::PointCloud2ConstIterator<float> xyz(msg_, "x");

    for (size_t i = 0; i < points.size(); ++i)
    {
      // 棋盘坐标系下的角点
      const double raw_x = (i % points_x_) * square_size_;
      const double raw_y = (i / points_x_) * square_size_;
      const double center_shift_x = origin_at_center_ ? ((points_x_ - 1) * square_size_ * 0.5) : 0.0;
      const double center_shift_y = origin_at_center_ ? ((points_y_ - 1) * square_size_ * 0.5) : 0.0;
      world.point.x = raw_x - center_shift_x + board_offset_x_;
      world.point.y = raw_y - center_shift_y + board_offset_y_;
      world.point.z = board_offset_z_;

      // p_cam = R * p_board + tvec
      cv::Mat p_cb  = (cv::Mat_<double>(3,1) << world.point.x, world.point.y, 0.0);
      cv::Mat p_cam = R * p_cb + tvec;

      rgbd.point.x = p_cam.at<double>(0);
      rgbd.point.y = p_cam.at<double>(1);
      rgbd.point.z = p_cam.at<double>(2);

      // 调试：只在前几次调用时打印第 0 个角点的 PnP 3D 坐标
      static int depth_debug_calls = 0;
      if (i == 0 && depth_debug_calls < 5)
      {
        ROS_INFO_STREAM("CheckerboardFinder PnP-based feature[0] in frame '"
                        << rgbd.header.frame_id << "': "
                        << "p=[" << rgbd.point.x << ", "
                        << rgbd.point.y << ", "
                        << rgbd.point.z << "]");
        depth_debug_calls++;
      }

      msg->observations[idx_cam].features[i] = rgbd;
      msg->observations[idx_cam].ext_camera_info = ext_info;
      msg->observations[idx_chain].features[i] = world;

      // 可视化仍然沿用 /points 中的 xyz
      int index = static_cast<int>(points[i].y) * msg_.width + static_cast<int>(points[i].x);
      iter_cloud[0] = (xyz + index)[X];
      iter_cloud[1] = (xyz + index)[Y];
      iter_cloud[2] = (xyz + index)[Z];
      ++iter_cloud;
    }

    // Add debug cloud to message
    if (output_debug_)
    {
      msg->observations[idx_cam].cloud = msg_;
    }

    // Publish results
    publisher_.publish(cloud);

    // Found all points
    return true;
  }
  catch (const std::exception& e)
  {
    ROS_WARN_STREAM("Exception while computing checkerboard pose (PointCloud2): " << e.what());
    return false;
  }
}

template <>
bool CheckerboardFinder<sensor_msgs::ImagePtr>::findInternal(robot_calibration_msgs::CalibrationData * msg)
{
  ROS_INFO_THROTTLE(1.0, "CheckerboardFinder<ImagePtr>::findInternal(): waiting for image message...");
  // Get image
  if (!waitForMsg())
  {
    ROS_ERROR("No image data");
    return false;
  }

  std::vector<cv::Point2f> points;
  if (findCheckerboardPoints(msg_, points))
  {
    ROS_INFO("CheckerboardFinder<ImagePtr>::findInternal(): Found the checkboard, running solvePnP...");

    // 计算并打印棋盘相对于相机坐标系的位姿（使用 solvePnP）
    cv::Mat R_cam_tag;
    cv::Mat tvec_cam_tag;
    bool pnp_ok = false;
    robot_calibration_msgs::ExtendedCameraInfo ext_info = depth_camera_manager_.getDepthCameraInfo();
    try
    {
      // 构造棋盘在自身坐标系下的 3D 点 (Z=0)
      std::vector<cv::Point3f> obj_points;
      obj_points.reserve(points_x_ * points_y_);
      for (int j = 0; j < points_y_; ++j)
      {
        for (int i = 0; i < points_x_; ++i)
        {
          obj_points.emplace_back(static_cast<float>(i * square_size_),
                                  static_cast<float>(j * square_size_),
                                  0.0f);
        }
      }

      // 2D 像素点
      std::vector<cv::Point2f> img_points = points;

      // 相机内参与畸变系数
      const sensor_msgs::CameraInfo& cam_info = ext_info.camera_info;

      cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
        cam_info.K[0], 0.0,            cam_info.K[2],
        0.0,            cam_info.K[4], cam_info.K[5],
        0.0,            0.0,            1.0);

      cv::Mat dist_coeffs;
      if (!cam_info.D.empty())
      {
        dist_coeffs = cv::Mat(cam_info.D).clone();
      }
      else
      {
        dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);
      }

      cv::Mat rvec;
      pnp_ok = cv::solvePnP(obj_points, img_points,
                            camera_matrix, dist_coeffs,
                            rvec, tvec_cam_tag, false, cv::SOLVEPNP_ITERATIVE);

      if (pnp_ok)
      {
        cv::Rodrigues(rvec, R_cam_tag);

        double sy = std::sqrt(R_cam_tag.at<double>(0, 0) * R_cam_tag.at<double>(0, 0) +
                              R_cam_tag.at<double>(1, 0) * R_cam_tag.at<double>(1, 0));
        bool singular = sy < 1e-6;

        double yaw, pitch, roll;
        if (!singular)
        {
          yaw   = std::atan2(R_cam_tag.at<double>(1, 0), R_cam_tag.at<double>(0, 0));
          pitch = std::atan2(-R_cam_tag.at<double>(2, 0), sy);
          roll  = std::atan2(R_cam_tag.at<double>(2, 1), R_cam_tag.at<double>(2, 2));
        }
        else
        {
          yaw   = std::atan2(-R_cam_tag.at<double>(0, 1), R_cam_tag.at<double>(1, 1));
          pitch = std::atan2(-R_cam_tag.at<double>(2, 0), sy);
          roll  = 0.0;
        }

        const double tx_mm = tvec_cam_tag.at<double>(0) * 1000.0;
        const double ty_mm = tvec_cam_tag.at<double>(1) * 1000.0;
        const double tz_mm = tvec_cam_tag.at<double>(2) * 1000.0;
        const double roll_deg = roll * 180.0 / M_PI;
        const double pitch_deg = pitch * 180.0 / M_PI;
        const double yaw_deg = yaw * 180.0 / M_PI;
        ROS_INFO_STREAM("Checkerboard pose in camera frame (from Image): "
                        << "t_mm=[" << tx_mm << ", " << ty_mm << ", " << tz_mm << "], "
                        << "rpy_deg=[roll=" << roll_deg
                        << ", pitch=" << pitch_deg
                        << ", yaw=" << yaw_deg << "]");

        // B 方案：在 Image 分支打印两套 base<-tag 及 delta
        if (fk_ready_ && fk_cam_solver_ && fk_tag_solver_)
        {
          KDL::Frame T_base_cam;
          KDL::Frame T_base_tag_urdf;
          if (computeFK(chain_base_to_cam_, cam_joint_names_, msg->joint_states, *fk_cam_solver_, T_base_cam) &&
              computeFK(chain_base_to_tag_, tag_joint_names_, msg->joint_states, *fk_tag_solver_, T_base_tag_urdf))
          {
            const KDL::Rotation R_cam_tag_kdl(
                R_cam_tag.at<double>(0, 0), R_cam_tag.at<double>(0, 1), R_cam_tag.at<double>(0, 2),
                R_cam_tag.at<double>(1, 0), R_cam_tag.at<double>(1, 1), R_cam_tag.at<double>(1, 2),
                R_cam_tag.at<double>(2, 0), R_cam_tag.at<double>(2, 1), R_cam_tag.at<double>(2, 2));
            const KDL::Vector t_cam_tag(tvec_cam_tag.at<double>(0), tvec_cam_tag.at<double>(1), tvec_cam_tag.at<double>(2));
            const KDL::Frame T_cam_tag(R_cam_tag_kdl, t_cam_tag);

            const KDL::Frame T_base_tag_obs = T_base_cam * T_cam_tag;
            const KDL::Frame T_delta = T_base_tag_urdf.Inverse() * T_base_tag_obs;

            double x_obs, y_obs, z_obs, r_obs, p_obs, yw_obs;
            double x_urdf, y_urdf, z_urdf, r_urdf, p_urdf, yw_urdf;
            double x_d, y_d, z_d, r_d, p_d, yw_d;
            frameToXYZRPYmmDeg(T_base_tag_obs, x_obs, y_obs, z_obs, r_obs, p_obs, yw_obs);
            frameToXYZRPYmmDeg(T_base_tag_urdf, x_urdf, y_urdf, z_urdf, r_urdf, p_urdf, yw_urdf);
            frameToXYZRPYmmDeg(T_delta, x_d, y_d, z_d, r_d, p_d, yw_d);

            ROS_INFO_STREAM("base<-tag(obs):  t_mm=[" << x_obs << "," << y_obs << "," << z_obs
                            << "], rpy_deg=[" << r_obs << "," << p_obs << "," << yw_obs << "]");
            ROS_INFO_STREAM("base<-tag(urdf): t_mm=[" << x_urdf << "," << y_urdf << "," << z_urdf
                            << "], rpy_deg=[" << r_urdf << "," << p_urdf << "," << yw_urdf
                            << "] (tag_frame='" << frame_id_ << "')");
            ROS_INFO_STREAM("delta(urdf^-1*obs): t_mm=[" << x_d << "," << y_d << "," << z_d
                            << "], rpy_deg=[" << r_d << "," << p_d << "," << yw_d << "]");
          }
          else
          {
            ROS_WARN_STREAM("FK debug failed (missing joint states or chain mismatch). base_frame='" << base_frame_
                            << "', cam_frame='" << camera_frame_ << "', tag_frame='" << frame_id_ << "'");
          }
        }
      }
      else
      {
        ROS_WARN("solvePnP failed to compute checkerboard pose (Image).");
      }
    }
    catch (const std::exception& e)
    {
      ROS_WARN_STREAM("Exception while computing checkerboard pose (Image): " << e.what());
    }

    if (!pnp_ok)
      return false;

    // Set msg size
    int idx_cam = msg->observations.size() + 0;
    int idx_chain = msg->observations.size() + 1;
    msg->observations.resize(msg->observations.size() + 2);
    msg->observations[idx_cam].sensor_name = camera_sensor_name_;
    msg->observations[idx_chain].sensor_name = chain_sensor_name_;

    msg->observations[idx_cam].features.resize(points_x_ * points_y_);
    msg->observations[idx_chain].features.resize(points_x_ * points_y_);

    // Setup observed points
    geometry_msgs::PointStamped rgbd;
    rgbd.header = msg_->header;
    geometry_msgs::PointStamped world;
    world.header.frame_id = frame_id_;

    // Fill in message
    for (size_t i = 0; i < points.size(); ++i)
    {
      // Create 3d position of corner (in checkerboard_frame)
      const double raw_x = (i % points_x_) * square_size_;
      const double raw_y = (i / points_x_) * square_size_;
      const double center_shift_x = origin_at_center_ ? ((points_x_ - 1) * square_size_ * 0.5) : 0.0;
      const double center_shift_y = origin_at_center_ ? ((points_y_ - 1) * square_size_ * 0.5) : 0.0;
      world.point.x = raw_x - center_shift_x + board_offset_x_;
      world.point.y = raw_y - center_shift_y + board_offset_y_;
      world.point.z = board_offset_z_;

      // 用 PnP 位姿将棋盘角点从 board 坐标系变换到相机坐标系（单位：m）
      cv::Mat p_cb  = (cv::Mat_<double>(3,1) << world.point.x, world.point.y, 0.0);
      cv::Mat p_cam = R_cam_tag * p_cb + tvec_cam_tag;
      rgbd.point.x = p_cam.at<double>(0);
      rgbd.point.y = p_cam.at<double>(1);
      rgbd.point.z = p_cam.at<double>(2);

      msg->observations[idx_cam].features[i] = rgbd;
      msg->observations[idx_cam].ext_camera_info = ext_info;
      msg->observations[idx_chain].features[i] = world;
    }

    // Add debug cloud to message
    if (output_debug_)
    {
      msg->observations[idx_cam].image = *msg_;
    }

    {
      std::stringstream ss;
      ss << "CheckerboardFinder2d output: "
         << "cam_sensor='" << msg->observations[idx_cam].sensor_name
         << "' cam_features=" << msg->observations[idx_cam].features.size()
         << " chain_sensor='" << msg->observations[idx_chain].sensor_name
         << "' chain_features=" << msg->observations[idx_chain].features.size();
      if (msg->observations[idx_cam].ext_camera_info.camera_info.P.size() >= 12)
      {
        const auto& P = msg->observations[idx_cam].ext_camera_info.camera_info.P;
        ss << " intrinsics(fx,fy,cx,cy)=(" << P[0] << "," << P[5] << "," << P[2] << "," << P[6] << ")";
      }
      if (!msg->observations[idx_cam].ext_camera_info.parameters.empty())
      {
        ss << " ext_params={";
        for (size_t i = 0; i < msg->observations[idx_cam].ext_camera_info.parameters.size(); ++i)
        {
          const auto& p = msg->observations[idx_cam].ext_camera_info.parameters[i];
          if (i > 0)
            ss << ",";
          ss << p.name << ":" << p.value;
        }
        ss << "}";
      }
      ROS_INFO_STREAM(ss.str());
    }

    // Found all points
    return true;
  }

  return false;
}

template <typename T>
bool CheckerboardFinder<T>::findCheckerboardPoints(const sensor_msgs::ImagePtr& image,
                                                   std::vector<cv::Point2f>& points)
{
  // Get an OpenCV image from the cloud
  cv_bridge::CvImagePtr bridge;
  try
  {
    bridge = cv_bridge::toCvCopy(image, "mono8");  // TODO: was rgb8? does this work?
  }
  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("Conversion failed");
    return false;
  }

  // Find checkerboard
  points.resize(points_x_ * points_y_);
  cv::Size checkerboard_size(points_x_, points_y_);
  return cv::findChessboardCorners(bridge->image, checkerboard_size,
                                   points, cv::CALIB_CB_ADAPTIVE_THRESH);
}

}  // namespace robot_calibration
