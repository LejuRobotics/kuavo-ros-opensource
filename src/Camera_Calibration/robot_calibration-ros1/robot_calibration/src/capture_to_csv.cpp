#include <robot_calibration/calibration_csv_io.h>

#include <ros/ros.h>

#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

#include "robot_calibration/capture/capture_manager.h"
#include "robot_calibration/capture/poses.h"
#include <robot_calibration/calibration/offset_parser.h>
#include <robot_calibration/fk_debug_utils.h>

#include <cmath>
#include <sstream>

static bool g_capture_flag = false;
static bool g_done_flag = false;

static void keyframeCallback(const std_msgs::Float64::ConstPtr& msg)
{
  // Receive non-zero payload => trigger one sample capture.
  if (msg && std::fabs(msg->data) > 1e-9)
  {
    g_capture_flag = true;
  }
}

static void doneCallback(const std_msgs::Bool::ConstPtr& msg)
{
  if (msg && msg->data)
    g_done_flag = true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_calibration");
  ros::NodeHandle nh("~");
  ros::NodeHandle nh_global;

  bool verbose = false;
  nh.param<bool>("verbose", verbose, false);

  std::string out_dir = "/tmp/calibration_csv";
  nh.param<std::string>("out_dir", out_dir, out_dir);

  bool print_fk = true;
  nh.param<bool>("print_fk", print_fk, true);
  std::string fk_base = "base_link";
  nh.param<std::string>("fk_base", fk_base, fk_base);
  std::string fk_tip_camera = "head_camera_color_optical_frame";
  nh.param<std::string>("fk_tip_camera", fk_tip_camera, fk_tip_camera);
  std::string fk_tip_board = "checkerboard_link";
  nh.param<std::string>("fk_tip_board", fk_tip_board, fk_tip_board);

  // Topic names for manual trigger.
  std::string keyframe_topic;
  std::string done_topic;
  nh.param<std::string>("keyframe_flag_topic", keyframe_topic, std::string("/head_keyframe_flag"));
  nh.param<std::string>("keyframe_done_topic", done_topic, std::string("/head_keyframe_done"));

  // Determine capture mode.
  bool manual_mode = false;
  std::string pose_bag_name;

  if (argc > 1)
  {
    const std::string arg1 = argv[1];
    if (arg1 == "--manual")
    {
      manual_mode = true;
    }
    else if (arg1 == "--from-bag")
    {
      manual_mode = false;
      if (argc > 2)
        pose_bag_name = argv[2];
      else
        pose_bag_name = "calibration_poses.bag";
    }
    else
    {
      // Treat argv[1] as bag name.
      manual_mode = false;
      pose_bag_name = argv[1];
    }
  }
  else
  {
    // Default: manual mode (works with kuavo_head_demo workflow).
    manual_mode = true;
  }

  // Capture manager (loads finders + chains).
  robot_calibration::CaptureManager capture_manager;
  if (!capture_manager.init(nh))
  {
    ROS_FATAL("Failed to init capture manager.");
    return -1;
  }

  std::string urdf_xml;
  if (print_fk && !nh_global.getParam("/robot_description", urdf_xml))
  {
    ROS_WARN("print_fk=true but /robot_description missing; FK logs disabled.");
    print_fk = false;
  }

  robot_calibration::CalibrationOffsetParser nominal_offsets;

  std::vector<robot_calibration_msgs::CalibrationData> data;

  if (manual_mode)
  {
    ROS_INFO_STREAM("Capture node running in manual mode, waiting keyframe trigger: "
                    << keyframe_topic << ", done: " << done_topic);

    g_capture_flag = false;
    g_done_flag = false;

    ros::Subscriber keyframe_sub = nh_global.subscribe(keyframe_topic, 10, keyframeCallback);
    ros::Subscriber done_sub = nh_global.subscribe(done_topic, 10, doneCallback);

    while (ros::ok() && !g_done_flag)
    {
      // Wait for one capture trigger.
      while (ros::ok() && !g_capture_flag && !g_done_flag)
      {
        ros::spinOnce();
        ros::Duration(0.01).sleep();
      }
      if (!ros::ok() || g_done_flag)
        break;

      g_capture_flag = false;

      // Empty feature list => capture all features from all finders.
      std::vector<std::string> features;
      robot_calibration_msgs::CalibrationData msg;
      if (!capture_manager.captureFeatures(features, msg))
      {
        ROS_WARN("Failed to capture manual sample.");
        continue;
      }

      if (msg.observations.empty())
      {
        ROS_WARN("Manual sample has 0 observations, skip this sample (likely feature finder not loaded or no valid camera feature).");
        continue;
      }

      {
        std::stringstream ss;
        ss << "manual sample detail: observations=" << msg.observations.size();
        for (size_t oi = 0; oi < msg.observations.size(); ++oi)
        {
          const auto& obs = msg.observations[oi];
          ss << " | [" << oi << "] sensor='" << obs.sensor_name
             << "' features=" << obs.features.size()
             << " frame='" << (obs.features.empty() ? std::string("") : obs.features[0].header.frame_id) << "'";
          if (obs.ext_camera_info.camera_info.P.size() >= 12)
          {
            ss << " fx=" << obs.ext_camera_info.camera_info.P[0]
               << " fy=" << obs.ext_camera_info.camera_info.P[5]
               << " cx=" << obs.ext_camera_info.camera_info.P[2]
               << " cy=" << obs.ext_camera_info.camera_info.P[6];
          }
          if (!obs.ext_camera_info.parameters.empty())
          {
            ss << " params={";
            for (size_t pi = 0; pi < obs.ext_camera_info.parameters.size(); ++pi)
            {
              const auto& p = obs.ext_camera_info.parameters[pi];
              if (pi > 0)
                ss << ",";
              ss << p.name << ":" << p.value;
            }
            ss << "}";
          }
        }
        ROS_INFO_STREAM(ss.str());
      }

      data.push_back(msg);
      const size_t sid = data.size() - 1;
      ROS_INFO_STREAM("Captured manual sample #" << sid);
      if (print_fk)
      {
        robot_calibration::logChainFkFromUrdf(urdf_xml, msg.joint_states, fk_base, fk_tip_camera,
                           "capture_cam_s" + std::to_string(sid), nominal_offsets);
        robot_calibration::logChainFkFromUrdf(urdf_xml, msg.joint_states, fk_base, fk_tip_board,
                           "capture_board_s" + std::to_string(sid), nominal_offsets);
      }
    }
  }
  else
  {
    if (pose_bag_name.empty())
      pose_bag_name = "calibration_poses.bag";

    ROS_INFO_STREAM("Capture node running from bag: " << pose_bag_name);

    std::vector<robot_calibration_msgs::CaptureConfig> poses;
    if (!robot_calibration::getPosesFromBag(pose_bag_name, poses))
    {
      ROS_FATAL("Failed to load poses from bag.");
      return -1;
    }

    for (unsigned pose_idx = 0; pose_idx < poses.size() && ros::ok(); ++pose_idx)
    {
      const auto& pose = poses[pose_idx];
      if (!capture_manager.moveToState(pose.joint_states))
      {
        ROS_WARN("Unable to move to desired state for sample %u.", pose_idx);
        continue;
      }

      ros::Duration(0.1).sleep();

      robot_calibration_msgs::CalibrationData msg;
      std::vector<std::string> features = pose.features;
      if (!capture_manager.captureFeatures(features, msg))
      {
        ROS_WARN("Failed to capture sample %u.", pose_idx);
        continue;
      }

      if (msg.observations.empty())
      {
        ROS_WARN("Sample %u has 0 observations, skip.", pose_idx);
        continue;
      }

      data.push_back(msg);
      ROS_INFO("Captured pose %u", pose_idx);
      if (print_fk)
      {
        robot_calibration::logChainFkFromUrdf(urdf_xml, msg.joint_states, fk_base, fk_tip_camera,
                           "capture_cam_s" + std::to_string(pose_idx), nominal_offsets);
        robot_calibration::logChainFkFromUrdf(urdf_xml, msg.joint_states, fk_base, fk_tip_board,
                           "capture_board_s" + std::to_string(pose_idx), nominal_offsets);
      }
    }
  }

  if (!robot_calibration::writeCalibrationToCsv(out_dir, data))
  {
    ROS_FATAL("Failed to write calibration csv.");
    return -1;
  }

  ROS_INFO("Done capturing and writing calibration csv.");
  return 0;
}

