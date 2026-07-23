/*
 * Copyright (C) 2018-2022 Michael Ferguson
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

#include <ctime>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <robot_calibration_msgs/CalibrationData.h>
#include <robot_calibration_msgs/CaptureConfig.h>
#include <cmath>
#include <sstream>

#include "robot_calibration/ceres/optimizer.h"
#include "robot_calibration/capture/capture_manager.h"
#include "robot_calibration/capture/poses.h"
#include "robot_calibration/calibration/export.h"
#include "robot_calibration/load_bag.h"

// 关键帧触发标志/完成标志，由外部话题置位
static bool g_capture_flag = false;
static bool g_done_flag = false;

static void keyframeCallback(const std_msgs::Float64::ConstPtr& msg)
{
  // 收到非零数据时认为是“到达关键帧”，触发一次采样
  if (msg && std::fabs(msg->data) > 1e-9)
  {
    g_capture_flag = true;
  }
}

static void doneCallback(const std_msgs::Bool::ConstPtr& msg)
{
  if (msg && msg->data)
  {
    g_done_flag = true;
  }
}

/** \mainpage
 * \section parameters Parameters of the Optimization:
 *   - joint angle offsets
 *   - frame 6DOF corrections (currently head pan frame, and camera frame)
 *   - camera intrinsics (2d & 3d)
 *
 * \section residuals Residual Blocks:
 *   - difference of reprojection through the arm and camera
 *   - residual blocks that limit offsets from growing outrageously large
 *
 * \section modules Modules:
 *   - Capture:
 *     - move joints to a particular place
 *     - wait to settle
 *     - find target (led or checkerboard)
 *     - write sample to bag file: joint angles, position of targets in camera.
 *   - Calibrate:
 *     - load urdf, samples from bag file.
 *     - create arm and camera reprojection chains.
 *     - create residual blocks.
 *     - run calibration.
 *     - write results to URDF.
 */

/*
 * usage:
 *  calibrate --manual
 *  calibrate calibration_poses.bag
 *  calibrate --from-bag calibration_data.bag
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv,"robot_calibration");
  ros::NodeHandle nh("~");
  ros::NodeHandle nh_global;

  // Should we be stupidly verbose?
  bool verbose;
  nh.param<bool>("verbose", verbose, false);

  // The calibration data
  std_msgs::String description_msg;
  std::vector<robot_calibration_msgs::CalibrationData> data;

  // What bag to use to load calibration poses out of (for capture)
  std::string pose_bag_name("calibration_poses.bag");
  if (argc > 1)
    pose_bag_name = argv[1];

  if (pose_bag_name.compare("--from-bag") != 0)
  {
    // No name provided for a calibration bag file, must do capture
    robot_calibration::CaptureManager capture_manager;
    capture_manager.init(nh);

    // Save URDF for calibration/export step
    description_msg.data = capture_manager.getUrdf();

    // Load a set of calibration poses
    std::vector<robot_calibration_msgs::CaptureConfig> poses;
    bool manual_mode = (pose_bag_name.compare("--manual") == 0);

    // 在手动模式下，通过关键帧触发话题来替代控制台按 Enter
    ros::Subscriber keyframe_sub;
    ros::Subscriber done_sub;
    std::string keyframe_topic;
    std::string done_topic;
    nh.param<std::string>("keyframe_flag_topic", keyframe_topic, std::string("/head_keyframe_flag"));
    nh.param<std::string>("keyframe_done_topic", done_topic, std::string("/head_keyframe_done"));
    if (manual_mode)
    {
      ROS_INFO_STREAM("Manual calibration mode: waiting keyframe trigger on topic " << keyframe_topic);
      g_capture_flag = false;
      g_done_flag = false;
      keyframe_sub = nh_global.subscribe(keyframe_topic, 10, keyframeCallback);
      done_sub = nh_global.subscribe(done_topic, 10, doneCallback);
    }

    if (!manual_mode)
    {
      if (!robot_calibration::getPosesFromBag(pose_bag_name, poses))
      {
        // Error will be printed in function
        return -1;
      }
    }
    else
    {
      ROS_INFO("Using manual calibration mode (triggered by keyframe topic)...");
    }

    // For each pose in the capture sequence.
    for (unsigned pose_idx = 0;
         (pose_idx < poses.size() || poses.empty()) && ros::ok();
         ++pose_idx)
    {
      robot_calibration_msgs::CalibrationData msg;
      if (poses.empty())
      {
        // Manual calibration：等待关键帧触发话题，而不是等待控制台按键
        if (manual_mode)
        {
          // 等待一次触发（例如 head_keyframe_publisher 在 capture=true 的关键帧发送 Float64(time)）
          while (ros::ok() && !g_capture_flag && !g_done_flag)
          {
            ros::spinOnce();
            ros::Duration(0.01).sleep();
          }
          if (g_done_flag)
          {
            ROS_INFO("Received done flag, finishing manual capture.");
            break;
          }
          if (!ros::ok())
            break;
          // 清除标志，准备下一次
          g_capture_flag = false;
        }

        // Empty vector causes us to capture all features
        std::vector<std::string> features;
        if (!capture_manager.captureFeatures(features, msg))
        {
          ROS_WARN("Failed to capture sample %u.", pose_idx);
          continue;
        }
      }
      else
      {
        // Move head/arm to pose
        if (!capture_manager.moveToState(poses[pose_idx].joint_states))
        {
          ROS_WARN("Unable to move to desired state for sample %u.", pose_idx);
          continue;
        }

        // Make sure sensor data is up to date after settling
        ros::Duration(0.1).sleep();

        // Get pose of the features
        if (!capture_manager.captureFeatures(poses[pose_idx].features, msg))
        {
          ROS_WARN("Failed to capture sample %u.", pose_idx);
          continue;
        }
      }

      ROS_INFO("Captured pose %u", pose_idx);

      // 采样诊断打印：关节角 + 各 observation 概览（便于复盘数据流）
      {
        const auto& js = msg.joint_states;
        auto get_pos = [&](const std::string& name, double& out) -> bool
        {
          for (size_t ii = 0; ii < js.name.size() && ii < js.position.size(); ++ii)
          {
            if (js.name[ii] == name)
            {
              out = js.position[ii];
              return true;
            }
          }
          return false;
        };

        double yaw = 0.0, pitch = 0.0;
        const bool has_yaw = get_pos("zhead_1_joint", yaw);
        const bool has_pitch = get_pos("zhead_2_joint", pitch);
        if (has_yaw || has_pitch)
        {
          std::ostringstream oss;
          oss.setf(std::ios::fixed);
          oss.precision(6);
          oss << "Sample " << pose_idx << " joint_states: ";
          if (has_yaw)
            oss << "zhead_1_joint=" << yaw << " rad (" << (yaw * 180.0 / M_PI) << " deg) ";
          if (has_pitch)
            oss << "zhead_2_joint=" << pitch << " rad (" << (pitch * 180.0 / M_PI) << " deg)";
          ROS_INFO_STREAM(oss.str());
        }
        else
        {
          ROS_INFO_STREAM("Sample " << pose_idx << " joint_states: (no zhead_1_joint/zhead_2_joint found; names=" << js.name.size()
                                    << " positions=" << js.position.size() << ")");
        }
      }

      // Add to samples
      data.push_back(msg);
    }

    ROS_INFO("Done capturing samples");
  }
  else
  {
    // Load calibration data from bagfile
    std::string data_bag_name("/tmp/calibration_data.bag");
    if (argc > 2)
      data_bag_name = argv[2];
    ROS_INFO_STREAM("Loading calibration data from " << data_bag_name);

    if (!robot_calibration::load_bag(data_bag_name, description_msg, data))
    {
      // Error will have been printed in function
      return -1;
    }
  }

  // Create instance of optimizer
  robot_calibration::OptimizationParams params;
  robot_calibration::Optimizer opt(description_msg.data);

  // Load calibration steps (if any)
  XmlRpc::XmlRpcValue cal_steps;
  if (nh.getParam("cal_steps", cal_steps))
  {
    // Should be a struct (mapping name -> config)
    if (cal_steps.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_FATAL("Parameter 'cal_steps' should be a struct.");
      return false;
    }

    XmlRpc::XmlRpcValue::iterator it;
    size_t step;
    size_t max_step = (cal_steps.size()>0)?cal_steps.size():1;
    std::vector<std::string> prev_frame_names;
    std::string prev_params_yaml;
    for (step = 0, it = cal_steps.begin(); step < max_step; step++, it++)
    {
      std::string name = static_cast<std::string>(it->first);
      ros::NodeHandle cal_steps_handle(nh, "cal_steps/"+name);
      params.LoadFromROS(cal_steps_handle);
      opt.optimize(params, data, verbose);
      if (verbose)
      {
        std::cout << "Parameter Offsets:" << std::endl;
        std::cout << opt.getOffsets()->getOffsetYAML() << std::endl;
      }
    }
  }
  else
  {
    // Single step calibration
    params.LoadFromROS(nh);
    opt.optimize(params, data, verbose);
    if (verbose)
    {
      std::cout << "Parameter Offsets:" << std::endl;
      std::cout << opt.getOffsets()->getOffsetYAML() << std::endl;
    }
  }

  // Write outputs
  robot_calibration::exportResults(opt, description_msg.data, data);

  ROS_INFO("Done calibrating");

  return 0;
}
