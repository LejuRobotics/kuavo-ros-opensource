/*
 * Copyright (C) 2018-2023 Michael Ferguson
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

#ifndef ROBOT_CALIBRATION_CAPTURE_CHECKERBOARD_FINDER_H
#define ROBOT_CALIBRATION_CAPTURE_CHECKERBOARD_FINDER_H

#include <ros/ros.h>
#include <robot_calibration/capture/depth_camera.h>
#include <robot_calibration/plugins/feature_finder.h>
#include <robot_calibration_msgs/CalibrationData.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

namespace robot_calibration
{

/**
 *  \brief Finds checkerboards in images or point clouds
 */
template <typename T>
class CheckerboardFinder : public FeatureFinder
{
public:
  CheckerboardFinder();
  bool init(const std::string& name, ros::NodeHandle & n);
  bool find(robot_calibration_msgs::CalibrationData * msg);

private:
  bool findInternal(robot_calibration_msgs::CalibrationData * msg);
  bool findCheckerboardPoints(const sensor_msgs::ImagePtr& msg,
                              std::vector<cv::Point2f>& points);

  void cameraCallback(const T& msg);
  bool waitForMsg();

  ros::Subscriber subscriber_;  /// Incoming message
  ros::Publisher publisher_;   /// Outgoing sensor_msgs::PointCloud2

  bool waiting_;
  T msg_;
  DepthCameraInfoManager depth_camera_manager_;

  /*
   * ROS Parameters
   */
  int points_x_;        /// Size of checkerboard
  int points_y_;        /// Size of checkerboard

  double square_size_;     /// Size of a square on checkboard (in meters)

  // Board coordinate origin configuration for generated checkerboard model points.
  // If true, generated board points are shifted so (0,0,0) is at board center.
  bool origin_at_center_{false};
  // Additional user-defined offsets (meters), applied after origin mode.
  double board_offset_x_{0.0};
  double board_offset_y_{0.0};
  double board_offset_z_{0.0};

  bool output_debug_;   /// Should we output debug image/cloud?

  std::string frame_id_;   /// Name of checkerboard frame

  std::string camera_sensor_name_;
  std::string chain_sensor_name_;

  // FK for printing base<-camera and base<-board (URDF) at capture time.
  std::string base_frame_;
  std::string camera_frame_;

  bool fk_ready_{false};
  KDL::Tree kdl_tree_;
  KDL::Chain chain_base_to_cam_;
  KDL::Chain chain_base_to_tag_;
  std::vector<std::string> cam_joint_names_;
  std::vector<std::string> tag_joint_names_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_cam_solver_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_tag_solver_;
};

}  // namespace robot_calibration

#endif  // ROBOT_CALIBRATION_CAPTURE_CHECKERBOARD_FINDER_H
