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

#include <fstream>
#include <sstream>
#include <map>
#include <string>
#include <cmath>
#include <boost/filesystem.hpp>
#include <ros/ros.h>
#include <ros/console.h>
#include <camera_calibration_parsers/parse.h>
#include <ros/package.h>
#include "robot_calibration/calibration/export.h"
#include <robot_calibration/models/chain.h>
#include <kdl/frames.hpp>

namespace robot_calibration
{
bool exportResults(Optimizer& optimizer, const std::string& initial_urdf,
                   const std::vector<robot_calibration_msgs::CalibrationData>& data)
{
  // Generate datecode
  char datecode[80];
  {
    std::time_t t = std::time(NULL);
    std::strftime(datecode, 80, "%Y_%m_%d_%H_%M_%S", std::localtime(&t));
  }

  // 输出目录：
  // - 可通过该节点私参 ~output_dir 覆盖（推荐：demo 各自传入 src/Camera_Calibration/output/<demo>/）
  // - 不设置时保持历史默认（避免老脚本失效）
  ros::NodeHandle nh("~");
  std::string base_output_dir;
  nh.param<std::string>(
      "output_dir",
      base_output_dir,
      ros::package::getPath("robot_calibration") + "/../../kuavo_head_demo/output/");
  if (!base_output_dir.empty() && base_output_dir.back() != '/')
    base_output_dir.push_back('/');

  // 输出文件名策略：
  // - fixed_output_names=true: 覆盖写固定文件名（不再按时间戳生成）
  // - fixed_output_names=false: 保持历史行为（带 datecode）
  bool fixed_output_names = true;
  nh.param<bool>("fixed_output_names", fixed_output_names, true);

  // 可选：将 calibrated URDF 输出到指定绝对路径（用于跨 demo 共享同一份 calibrated urdf）
  std::string urdf_output_path;
  nh.param<std::string>("urdf_output_path", urdf_output_path, std::string(""));

  // 可选：将 offsets YAML 输出到指定绝对路径
  std::string calibration_yaml_path;
  nh.param<std::string>("calibration_yaml_path", calibration_yaml_path, std::string(""));

  try
  {
    boost::filesystem::create_directories(base_output_dir);
  }
  catch (...)
  {
    ROS_WARN_STREAM("Failed to create output_dir: " << base_output_dir);
  }

  // 将优化得到的 offsets 写回 URDF（与 YAML 一并输出到 kuavo_head_demo/output/）
  {
    std::string s = optimizer.getOffsets()->updateURDF(initial_urdf);
    std::string urdf_name;
    if (!urdf_output_path.empty())
    {
      urdf_name = urdf_output_path;
      try
      {
        boost::filesystem::create_directories(boost::filesystem::path(urdf_name).parent_path());
      }
      catch (...)
      {
        ROS_WARN_STREAM("Failed to create urdf_output_path parent dir: " << urdf_name);
      }
    }
    else
    {
      std::stringstream ss;
      ss << base_output_dir << "calibrated";
      if (!fixed_output_names)
        ss << "_" << datecode;
      ss << ".urdf";
      urdf_name = ss.str();
    }
    std::ofstream file;
    file.open(urdf_name.c_str());
    file << s;
    file.close();
    ROS_INFO_STREAM("Wrote calibrated URDF: " << urdf_name);
  }

  // depth/rgb 相机内参 YAML：仅在存在 Camera3d 模型且需要导出内参时启用（头部链式标定可保持关闭）
#if 0
  // Output camera calibration(s)
  std::vector<std::string> camera_names = optimizer.getCameraNames();
  for (auto it = camera_names.begin(); it != camera_names.end(); ++it)
  {
    // Find original CameraInfo
    sensor_msgs::CameraInfo camera_info;
    bool found_camera_info = false;
    for (auto obs = data.front().observations.begin();
         obs != data.front().observations.end(); ++obs)
    {
      if (obs->sensor_name == *it)
      {
        camera_info = obs->ext_camera_info.camera_info;
        found_camera_info = true;
        break;
      }
    }

    if (!found_camera_info)
    {
      ROS_WARN("Unable to export camera_info for %s", it->c_str());
      continue;
    }

    std::stringstream depth_name;
    depth_name << base_output_dir << "depth_";
    if (*it != "camera")
    {
      // We include the name if the name is not "camera" for backwards compatability
      depth_name << *it << "_";
    }
    depth_name << datecode << ".yaml";
    camera_calibration_parsers::writeCalibration(depth_name.str(), "",
        robot_calibration::updateCameraInfo(
                         optimizer.getOffsets()->get(*it + "_fx"),
                         optimizer.getOffsets()->get(*it + "_fy"),
                         optimizer.getOffsets()->get(*it + "_cx"),
                         optimizer.getOffsets()->get(*it + "_cy"),
                         camera_info));

    std::stringstream rgb_name;
    rgb_name << base_output_dir << "rgb_";
    if (*it != "camera")
    {
      // We include the name if the name is not "camera" for backwards compatability
      rgb_name << *it << "_";
    }
    rgb_name << datecode << ".yaml";
    camera_calibration_parsers::writeCalibration(rgb_name.str(), "",
        robot_calibration::updateCameraInfo(
                         optimizer.getOffsets()->get(*it + "_fx"),
                         optimizer.getOffsets()->get(*it + "_fy"),
                         optimizer.getOffsets()->get(*it + "_cx"),
                         optimizer.getOffsets()->get(*it + "_cy"),
                         camera_info));
  }
#endif

  // 输出标定结果 YAML（offset 列表）；URDF 已在上方写出；depth/rgb 内参 YAML 见上方 #if 0
  {
    std::string yaml_name;
    if (!calibration_yaml_path.empty())
    {
      yaml_name = calibration_yaml_path;
      try
      {
        boost::filesystem::create_directories(boost::filesystem::path(yaml_name).parent_path());
      }
      catch (...)
      {
        ROS_WARN_STREAM("Failed to create calibration_yaml_path parent dir: " << yaml_name);
      }
    }
    else
    {
      std::stringstream ss;
      ss << base_output_dir << "calibration";
      if (!fixed_output_names)
        ss << "_" << datecode;
      ss << ".yaml";
      yaml_name = ss.str();
    }
    std::ofstream file;
    file.open(yaml_name.c_str());
    file << optimizer.getOffsets()->getOffsetYAML();
    file.close();

    ROS_INFO_STREAM("Wrote calibration offsets YAML: " << yaml_name);

    // 额外诊断：将 axis-magnitude (a,b,c) 转成 rpy，并打印出关键 frame 的 offset。
    // 注意：offsets YAML 中 *_a/_b/_c 是 axis-magnitude（旋转向量），不是 rpy。
    const std::string yaml_text = optimizer.getOffsets()->getOffsetYAML();
    std::map<std::string, std::map<std::string, double>> grouped;
    std::istringstream iss(yaml_text);
    std::string line;
    while (std::getline(iss, line))
    {
      std::istringstream ls(line);
      std::string key_with_colon;
      double value;
      if (!(ls >> key_with_colon >> value))
        continue;
      if (key_with_colon.size() < 3 || key_with_colon.back() != ':')
        continue;
      const std::string key = key_with_colon.substr(0, key_with_colon.size() - 1);
      const size_t pos = key.rfind('_');
      if (pos == std::string::npos)
        continue;
      const std::string base = key.substr(0, pos);
      const std::string suf = key.substr(pos + 1);
      if (suf != "x" && suf != "y" && suf != "z" && suf != "a" && suf != "b" && suf != "c")
        continue;
      grouped[base][suf] = value;
    }

    for (const auto& it : grouped)
    {
      const std::string& name = it.first;
      const auto& m = it.second;
      if (!(m.count("x") && m.count("y") && m.count("z") && m.count("a") && m.count("b") && m.count("c")))
        continue;

      const double ax = m.at("a");
      const double ay = m.at("b");
      const double az = m.at("c");
      const double angle = std::sqrt(ax * ax + ay * ay + az * az);
      // Convert axis-magnitude (axis*angle) -> quaternion wxyz for printing only.
      double qw = 1.0, qx = 0.0, qy = 0.0, qz = 0.0;
      if (angle > 1e-12)
      {
        const double half = angle * 0.5;
        const double s = std::sin(half);
        qw = std::cos(half);
        qx = (ax / angle) * s;
        qy = (ay / angle) * s;
        qz = (az / angle) * s;
      }

      ROS_INFO_STREAM("offset[" << name << "]: "
                      << "xyz(m)=[" << m.at("x") << ", " << m.at("y") << ", " << m.at("z") << "], "
                      << "axis-magnitude(rad)=[" << ax << ", " << ay << ", " << az << "] "
                      << "(|v|=" << angle << "), "
                      << "quat_wxyz=[" << qw << ", " << qx << ", " << qy << ", " << qz << "]");

      {
        const KDL::Rotation R_off = rotation_from_axis_magnitude(ax, ay, az);
        double rr, pp, yy;
        R_off.GetRPY(rr, pp, yy);
        const double kRad2Deg = 180.0 / M_PI;
        ROS_INFO_STREAM("offset[" << name << "] (extrinsic): xyz_mm=[" << (m.at("x") * 1000.0) << ", " << (m.at("y") * 1000.0) << ", "
                        << (m.at("z") * 1000.0) << "], rpy_deg=[roll=" << (rr * kRad2Deg) << ", pitch=" << (pp * kRad2Deg) << ", yaw=" << (yy * kRad2Deg) << "] "
                        << "(axis-magnitude -> KDL::GetRPY, same as URDF rpy convention)");
      }
    }

    // 额外诊断：把 offsets 应用到 URDF 后，输出更新后的 joint origin（便于直接对照 TF 语义）。
    // 这里只打印几个常用的 frame/joint（例如 checkerboard_joint），避免刷屏。
    try
    {
      const std::string updated_urdf = optimizer.getOffsets()->updateURDF(initial_urdf);
      if (grouped.count("checkerboard_joint"))
      {
        // 用非常轻量的字符串查找打印 checkerboard_joint 的 origin（避免再次引入 XML 解析依赖）
        const std::string needle = "joint name=\"checkerboard_joint\"";
        const size_t p = updated_urdf.find(needle);
        if (p != std::string::npos)
        {
          const size_t origin_p = updated_urdf.find("<origin", p);
          const size_t origin_end = updated_urdf.find("/>", origin_p);
          if (origin_p != std::string::npos && origin_end != std::string::npos)
          {
            ROS_INFO_STREAM("updated URDF checkerboard_joint origin: "
                            << updated_urdf.substr(origin_p, origin_end - origin_p + 2));
          }
        }
      }
    }
    catch (...)
    {
      ROS_WARN_STREAM("Failed to compute updated URDF for debug printing.");
    }
  }

  return true;
}

}  // namespace robot_calibration
