// C++ implementation of Quest3BoneFramePublisher (monitor_quest3.py)
// Responsible for:
//  - UDP handshake & receiving LejuHandPoseEvent protobuf from Quest3
//  - Publishing PoseInfoList and JoySticks ROS messages
//  - Publishing TF frames for Quest3 bones and finger tips
//  - Computing and publishing robotHeadMotionData from Chest->Head TF
//  - Discovering Quest3 via UDP broadcasts and broadcasting RobotDescription

#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <tf2_msgs/TFMessage.h>

#include <kuavo_ros_interfaces/robotHeadMotionData.h>
#include <kuavo_msgs/JoySticks.h>

#include <noitom_hi5_hand_udp_python/PoseInfo.h>
#include <noitom_hi5_hand_udp_python/PoseInfoList.h>

#include <hand_pose.pb.h>
#include <robot_info.pb.h>

#include <ros/package.h>

#include <fstream>
#include <sstream>

#include "json.hpp"
using json = nlohmann::json;

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#include <ifaddrs.h>

#include <cerrno>
#include <cmath>
#include <cstring>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <thread>
#include <vector>
#include <atomic>

namespace
{
constexpr int QUEST3_DEFAULT_DATA_PORT = 10019;
constexpr int QUEST3_DISCOVERY_START_PORT = 11000;
constexpr int QUEST3_DISCOVERY_END_PORT   = 11010;
constexpr int ROBOT_INFO_START_PORT       = 11050;
constexpr int ROBOT_INFO_END_PORT         = 11060;

double deg(double rad) { return rad * 180.0 / M_PI; }

// Normalize angle to behave similar to Python normalize_degree_in_180
double normalize_degree_in_180(double degree)
{
  if (degree > 180.0)
  {
    degree -= 180.0;
  }
  else if (degree < -180.0)
  {
    degree += 180.0;
  }
  return degree;
}

struct HeadMotionRange
{
  // Use a wide default range; Python reads from config.json but here we keep it simple.
  double pitch_min{-30.0};
  double pitch_max{30.0};
  double yaw_min{-85.0};
  double yaw_max{85.0};
};

std::vector<std::string> getLocalBroadcastIps()
{
  std::vector<std::string> result;
  std::set<std::string> unique;

  struct ifaddrs* ifaddr = nullptr;
  if (getifaddrs(&ifaddr) == -1)
  {
    ROS_ERROR("getifaddrs failed: %s", strerror(errno));
    return result;
  }

  for (struct ifaddrs* ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next)
  {
    if (!ifa->ifa_addr || ifa->ifa_addr->sa_family != AF_INET)
      continue;

    std::string name(ifa->ifa_name ? ifa->ifa_name : "");
    if (name.rfind("docker", 0) == 0 || name.rfind("br-", 0) == 0 || name.rfind("veth", 0) == 0)
      continue;

    struct sockaddr_in* addr = reinterpret_cast<struct sockaddr_in*>(ifa->ifa_broadaddr);
    if (!addr)
      continue;

    char buf[INET_ADDRSTRLEN] = {0};
    if (!inet_ntop(AF_INET, &addr->sin_addr, buf, sizeof(buf)))
      continue;

    std::string ip(buf);
    if (!ip.empty() && unique.insert(ip).second)
      result.push_back(ip);
  }

  freeifaddrs(ifaddr);

  std::sort(result.begin(), result.end());
  return result;
}

}  // namespace

class Quest3BoneFramePublisherCpp
{
public:
  Quest3BoneFramePublisherCpp(ros::NodeHandle& nh)
    : nh_(nh)
    , enable_head_control_(true)
    , data_socket_(-1)
    , listening_udp_ports_cnt_(0)
    , exit_listen_thread_for_quest3_broadcast_(false)
  {
    init();
  }

  ~Quest3BoneFramePublisherCpp()
  {
    shutdown();
  }

  void shutdown()
  {
    exit_listen_thread_for_quest3_broadcast_ = true;

    if (data_socket_ >= 0)
    {
      close(data_socket_);
      data_socket_ = -1;
    }
  }

  void broadcastRobotInfoAndWaitForQuest3()
  {
    // Start periodic broadcaster thread
    std::thread periodic_broadcaster(&Quest3BoneFramePublisherCpp::periodicRobotInfoBroadcaster, this);

    // Start discovery listeners
    std::vector<std::thread> threads;
    for (int port = QUEST3_DISCOVERY_START_PORT; port <= QUEST3_DISCOVERY_END_PORT; ++port)
    {
      threads.emplace_back(&Quest3BoneFramePublisherCpp::listenForQuest3Broadcasts, this, port);
    }

    if (listening_udp_ports_cnt_ == 0)
    {
      std::cerr << "\033[91m"
                << "All UDP broadcast ports are occupied. Please check with 'lsof -i :11000-11010'."
                << "\033[0m" << std::endl;
      ros::shutdown();
    }

    for (auto& t : threads)
    {
      if (t.joinable())
        t.join();
    }

    if (periodic_broadcaster.joinable())
      periodic_broadcaster.join();

    if (!server_ip_.empty())
    {
      std::cout << "\033[92mQuest3 device found at IP: " << server_ip_ << "\033[0m" << std::endl;
      std::cout << "\033[92mReceived Quest3 Broadcast, starting to connect.\033[0m" << std::endl;
    }
  }

  bool setupSocket(const std::string& server_address, int port)
  {
    if (data_socket_ >= 0)
    {
      ROS_INFO("Socket already established, skipping creation.");
      return true;
    }

    data_socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (data_socket_ < 0)
    {
      ROS_ERROR("Failed to create UDP socket: %s", strerror(errno));
      return false;
    }

    memset(&server_addr_, 0, sizeof(server_addr_));
    server_addr_.sin_family = AF_INET;
    server_addr_.sin_port   = htons(port);

    if (inet_pton(AF_INET, server_address.c_str(), &server_addr_.sin_addr) <= 0)
    {
      ROS_ERROR("Invalid server address: %s", server_address.c_str());
      ::close(data_socket_);
      data_socket_ = -1;
      return false;
    }

    struct timeval tv;
    tv.tv_sec  = 1;
    tv.tv_usec = 0;
    if (setsockopt(data_socket_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0)
    {
      ROS_WARN("Failed to set socket timeout: %s", strerror(errno));
    }

    server_ip_   = server_address;
    server_port_ = port;
    return true;
  }

  bool sendInitialMessage()
  {
    if (data_socket_ < 0)
    {
      ROS_ERROR("Data socket is not initialized.");
      return false;
    }

    const char* msg = "hi";
    const int   max_retries = 200;
    char buffer[1024];

    for (int attempt = 0; attempt < max_retries && ros::ok(); ++attempt)
    {
      ssize_t sent = sendto(data_socket_, msg, strlen(msg), 0,
                            reinterpret_cast<struct sockaddr*>(&server_addr_),
                            sizeof(server_addr_));
      if (sent < 0)
      {
        ROS_WARN("Failed to send 'hi' to Quest3: %s", strerror(errno));
      }

      struct sockaddr_in from_addr;
      socklen_t          from_len = sizeof(from_addr);
      ssize_t            recvd    = recvfrom(data_socket_, buffer, sizeof(buffer), 0,
                                      reinterpret_cast<struct sockaddr*>(&from_addr),
                                      &from_len);

      if (recvd > 0)
      {
        std::cout << "\033[92mAcknowledgment From Quest3 received on attempt "
                  << (attempt + 1) << ", start to receiving data...\033[0m" << std::endl;
        return true;
      }
      else
      {
        std::cout << "\033[91mQuest3_timeout: Attempt " << (attempt + 1)
                  << " timed out. Retrying...\033[0m" << std::endl;
      }
    }

    std::cout << "Failed to send message after 200 attempts." << std::endl;
    return false;
  }

  bool restartSocket()
  {
    std::cout << "Restarting socket connection..." << std::endl;
    if (data_socket_ >= 0)
    {
      ::close(data_socket_);
      data_socket_ = -1;
    }

    if (!setupSocket(server_ip_, server_port_))
    {
      std::cout << "Failed to recreate socket." << std::endl;
      return false;
    }

    if (!sendInitialMessage())
    {
      std::cout << "Failed to restart socket connection." << std::endl;
      return false;
    }

    std::cout << "Socket connection restarted successfully." << std::endl;
    return true;
  }

  void run()
  {
    int loop_count = 0;
    while (ros::ok())
    {
      ++loop_count;
      uint8_t buf[4096];
      struct sockaddr_in from_addr;
      socklen_t          from_len = sizeof(from_addr);

      ssize_t recvd = recvfrom(data_socket_, buf, sizeof(buf), 0,
                               reinterpret_cast<struct sockaddr*>(&from_addr),
                               &from_len);
      if (recvd < 0)
      {
        if (errno == EAGAIN || errno == EWOULDBLOCK)
        {
          std::cout << "Timeout occurred, no data received. Restarting socket..." << std::endl;
          if (!restartSocket())
          {
            break;
          }
          continue;
        }
        else
        {
          std::cerr << "An error occurred in recvfrom: " << strerror(errno) << std::endl;
          if (!restartSocket())
          {
            break;
          }
          continue;
        }
      }

      // Parse protobuf
      LejuHandPoseEvent event;
      if (!event.ParseFromArray(buf, static_cast<int>(recvd)))
      {
        ROS_WARN("Failed to parse LejuHandPoseEvent protobuf.");
        continue;
      }

      ros::Time now = ros::Time::now();

      noitom_hi5_hand_udp_python::PoseInfoList pose_info_list_msg;
      kuavo_msgs::JoySticks                     joysticks_msg;

      // Process joystick data
      processJoystickData(event, joysticks_msg);

      // Process pose data
      processPoseData(event, pose_info_list_msg, now);

      // Fill meta fields and publish
      pose_info_list_msg.timestamp_ms      = event.timestamp();
      pose_info_list_msg.is_high_confidence = event.isdatahighconfidence();
      pose_info_list_msg.is_hand_tracking   = event.ishandtracking();

      joysticks_pub_.publish(joysticks_msg);

      if (pose_info_list_msg.is_high_confidence)
      {
        pose_pub_.publish(pose_info_list_msg);
      }
      else
      {
        ROS_WARN("Low confidence pose data, not publishing.");
      }

      ros::spinOnce();
      rate_.sleep();
    }
  }

  void updateBroadcastIps(const std::vector<std::string>& ips)
  {
    std::set<std::string> unique(ips.begin(), ips.end());
    broadcast_ips_.assign(unique.begin(), unique.end());
    std::sort(broadcast_ips_.begin(), broadcast_ips_.end());
    ROS_INFO_STREAM("Updated broadcast IPs to: "
                    << (broadcast_ips_.empty() ? std::string("[]") : ""));
  }

private:
  void init()
  {
    bone_names_ = {
      "LeftArmUpper", "LeftArmLower", "RightArmUpper", "RightArmLower",
      "LeftHandPalm", "RightHandPalm", "LeftHandThumbMetacarpal",
      "LeftHandThumbProximal", "LeftHandThumbDistal", "LeftHandThumbTip",
      "LeftHandIndexTip", "LeftHandMiddleTip", "LeftHandRingTip",
      "LeftHandLittleTip", "RightHandThumbMetacarpal", "RightHandThumbProximal",
      "RightHandThumbDistal", "RightHandThumbTip", "RightHandIndexTip",
      "RightHandMiddleTip", "RightHandRingTip", "RightHandLittleTip",
      "Root", "Chest", "Neck", "Head"
    };

    for (std::size_t i = 0; i < bone_names_.size(); ++i)
    {
      bone_name_to_index_[bone_names_[i]] = static_cast<int>(i);
      index_to_bone_name_[static_cast<int>(i)] = bone_names_[i];
    }

    rate_ = ros::Rate(100.0);

    loadHeadMotionRangeFromConfig();

    pose_pub_      = nh_.advertise<noitom_hi5_hand_udp_python::PoseInfoList>("/leju_quest_bone_poses", 2);
    head_data_pub_ = nh_.advertise<kuavo_ros_interfaces::robotHeadMotionData>("/robot_head_motion_data", 10);
    joysticks_pub_ = nh_.advertise<kuavo_msgs::JoySticks>("/quest_joystick_data", 2);
    hand_finger_tf_pub_ = nh_.advertise<tf2_msgs::TFMessage>("/quest_hand_finger_tf", 10);

    nh_.param("enable_head_control", enable_head_control_, true);
    ROS_INFO_STREAM("enable_head_control: " << (enable_head_control_ ? "true" : "false"));
  }

  void loadHeadMotionRangeFromConfig()
  {
    // 默认范围为 [-180, 180]，与结构体默认值一致
    std::string pkg_path = ros::package::getPath("noitom_hi5_hand_udp_python");
    if (pkg_path.empty())
    {
      ROS_WARN("Failed to get package path for noitom_hi5_hand_udp_python. Using default head motion range.");
      return;
    }

    std::string config_path = pkg_path + "/scripts/config.json";
    std::ifstream in(config_path.c_str());
    if (!in.is_open())
    {
      ROS_WARN("Failed to open config.json at '%s'. Using default head motion range.", config_path.c_str());
      return;
    }

    try
    {
      json j = json::parse(in);

      if (j.contains("head_motion_range"))
      {
        auto& range = j["head_motion_range"];

        if (range.contains("yaw") && range["yaw"].is_array() && range["yaw"].size() >= 2)
        {
          head_motion_range_.yaw_min = range["yaw"][0].get<double>();
          head_motion_range_.yaw_max = range["yaw"][1].get<double>();
        }

        if (range.contains("pitch") && range["pitch"].is_array() && range["pitch"].size() >= 2)
        {
          head_motion_range_.pitch_min = range["pitch"][0].get<double>();
          head_motion_range_.pitch_max = range["pitch"][1].get<double>();
        }
      }

      ROS_INFO("Head motion range loaded from config.json: "
               "yaw [%.1f, %.1f], pitch [%.1f, %.1f]",
               head_motion_range_.yaw_min, head_motion_range_.yaw_max,
               head_motion_range_.pitch_min, head_motion_range_.pitch_max);
    }
    catch (const json::exception& e)
    {
      ROS_WARN("Failed to parse config.json: %s. Using default head motion range.", e.what());
    }
  }

  void processJoystickData(const LejuHandPoseEvent& event, kuavo_msgs::JoySticks& msg)
  {
    const auto& left  = event.left_joystick();
    const auto& right = event.right_joystick();

    msg.left_x  = left.x();
    msg.left_y  = left.y();
    msg.left_trigger = left.trigger();
    msg.left_grip    = left.grip();
    msg.left_first_button_pressed  = left.firstbuttonpressed();
    msg.left_second_button_pressed = left.secondbuttonpressed();
    msg.left_first_button_touched  = left.firstbuttontouched();
    msg.left_second_button_touched = left.secondbuttontouched();

    msg.right_x  = right.x();
    msg.right_y  = right.y();
    msg.right_trigger = right.trigger();
    msg.right_grip    = right.grip();
    msg.right_first_button_pressed  = right.firstbuttonpressed();
    msg.right_second_button_pressed = right.secondbuttonpressed();
    msg.right_first_button_touched  = right.firstbuttontouched();
    msg.right_second_button_touched = right.secondbuttontouched();
  }

  void processPoseData(const LejuHandPoseEvent& event,
                       noitom_hi5_hand_udp_python::PoseInfoList& pose_list,
                       const ros::Time& now)
  {
    std::map<std::string, double> scale_factor = {{"x", 3.0}, {"y", 3.0}, {"z", 3.0}};

    const int n = std::min(static_cast<int>(event.poses_size()),
                           static_cast<int>(bone_names_.size()));
    for (int i = 0; i < n; ++i)
    {
      const auto& pose = event.poses(i);
      std::string bone_name = index_to_bone_name_[i];

      double lx = pose.position().x();
      double ly = pose.position().y();
      double lz = pose.position().z();

      // Convert from Quest3 left-handed to our right-handed frame (same as Python)
      double rx = -lz;
      double ry = -lx;
      double rz =  ly;

      double qx = pose.quaternion().x();
      double qy = pose.quaternion().y();
      double qz = pose.quaternion().z();
      double qw = pose.quaternion().w();

      double rqx = -qz;
      double rqy = -qx;
      double rqz =  qy;
      double rqw =  qw;

      noitom_hi5_hand_udp_python::PoseInfo info;
      info.position.x    = rx;
      info.position.y    = ry;
      info.position.z    = rz;
      info.orientation.x = rqx;
      info.orientation.y = rqy;
      info.orientation.z = rqz;
      info.orientation.w = rqw;
      pose_list.poses.push_back(info);

      // Apply scale for TF frame
      rx *= scale_factor["x"];
      ry *= scale_factor["y"];
      rz *= scale_factor["z"];

      updateFrame(bone_name, rx, ry, rz, rqx, rqy, rqz, rqw, now);

      if (bone_name == "Head" && enable_head_control_)
      {
        publishHeadMotionData();
      }
    }

    // Also update the finger relative TFs similar to Python node
    updateQuestHandFingerTf();
  }

  void updateFrame(const std::string& frame_name,
                   double x, double y, double z,
                   double qx, double qy, double qz, double qw,
                   const ros::Time& t)
  {
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = t;
    tf_msg.header.frame_id = "torso";
    tf_msg.child_frame_id  = frame_name;
    tf_msg.transform.translation.x = x;
    tf_msg.transform.translation.y = y;
    tf_msg.transform.translation.z = z;
    tf_msg.transform.rotation.x = qx;
    tf_msg.transform.rotation.y = qy;
    tf_msg.transform.rotation.z = qz;
    tf_msg.transform.rotation.w = qw;

    tf_broadcaster_.sendTransform(tf_msg);
  }

  void updateQuestHandFingerTf()
  {
    static const std::vector<std::string> frames = {
      "LeftHandPalm", "RightHandPalm",
      "LeftHandThumbMetacarpal", "LeftHandThumbProximal", "LeftHandThumbDistal", "LeftHandThumbTip",
      "LeftHandIndexTip", "LeftHandMiddleTip", "LeftHandRingTip", "LeftHandLittleTip",
      "RightHandThumbMetacarpal", "RightHandThumbProximal", "RightHandThumbDistal", "RightHandThumbTip",
      "RightHandIndexTip", "RightHandMiddleTip", "RightHandRingTip", "RightHandLittleTip"
    };

    tf2_msgs::TFMessage msg;

    for (const auto& frame_name : frames)
    {
      try
      {
        std::string parent = (frame_name.find("Left") != std::string::npos) ?
                             "LeftHandPalm" : "RightHandPalm";

        tf::StampedTransform tf_trans;
        tf_listener_.lookupTransform(parent, frame_name, ros::Time(0), tf_trans);

        geometry_msgs::TransformStamped ts;
        ts.header.stamp = ros::Time::now();
        ts.header.frame_id = parent;
        ts.child_frame_id  = frame_name;
        ts.transform.translation.x = tf_trans.getOrigin().x();
        ts.transform.translation.y = tf_trans.getOrigin().y();
        ts.transform.translation.z = tf_trans.getOrigin().z();

        tf::Quaternion q = tf_trans.getRotation();
        ts.transform.rotation.x = q.x();
        ts.transform.rotation.y = q.y();
        ts.transform.rotation.z = q.z();
        ts.transform.rotation.w = q.w();

        msg.transforms.push_back(ts);
      }
      catch (const tf::TransformException& ex)
      {
        ROS_ERROR_STREAM_THROTTLE(1.0, "TF lookup failed in updateQuestHandFingerTf: " << ex.what());
        return;
      }
    }

    hand_finger_tf_pub_.publish(msg);
  }

  void publishHeadMotionData()
  {
    try
    {
      tf::StampedTransform tf_trans;
      tf_listener_.lookupTransform("Chest", "Head", ros::Time(0), tf_trans);

      tf::Quaternion q = tf_trans.getRotation();
      double roll, pitch, yaw;
      tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

      double pitch_deg = normalize_degree_in_180(std::round(deg(roll) * 100.0) / 100.0);
      double yaw_deg   = normalize_degree_in_180(std::round(deg(pitch) * 100.0) / 100.0);

      // Clamp to configured range
      pitch_deg = std::max(head_motion_range_.pitch_min,
                           std::min(head_motion_range_.pitch_max, pitch_deg));
      yaw_deg   = std::max(head_motion_range_.yaw_min,
                           std::min(head_motion_range_.yaw_max, yaw_deg));

      kuavo_ros_interfaces::robotHeadMotionData msg;
      msg.joint_data.resize(2);
      msg.joint_data[0] = yaw_deg;
      msg.joint_data[1] = pitch_deg;
      head_data_pub_.publish(msg);
    }
    catch (const tf::TransformException& ex)
    {
      ROS_ERROR_STREAM_THROTTLE(1.0, "TF lookup failed for Chest->Head transform: " << ex.what());
    }
  }

  void sendRobotInfoOnBroadcastIps(const std::string& robot_name,
                                   int robot_version,
                                   int start_port,
                                   int end_port)
  {
    if (broadcast_ips_.empty())
    {
      ROS_WARN_THROTTLE(10.0, "No broadcast IPs configured. Cannot send robot info.");
      return;
    }

    RobotDescription desc;
    desc.set_robot_name(robot_name);
    desc.set_robot_version(robot_version);

    std::string serialized = desc.SerializeAsString();

    int sock = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0)
    {
      ROS_ERROR("Failed to create broadcast socket: %s", strerror(errno));
      return;
    }

    int yes = 1;
    if (setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(yes)) < 0)
    {
      ROS_ERROR("Failed to set SO_BROADCAST: %s", strerror(errno));
      ::close(sock);
      return;
    }

    for (const auto& ip : broadcast_ips_)
    {
      for (int p = start_port; p <= end_port; ++p)
      {
        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_port   = htons(p);

        if (inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) <= 0)
        {
          ROS_ERROR("Invalid broadcast IP: %s", ip.c_str());
          continue;
        }

        ssize_t sent = sendto(sock, serialized.data(), serialized.size(), 0,
                              reinterpret_cast<struct sockaddr*>(&addr),
                              sizeof(addr));
        if (sent < 0)
        {
          ROS_ERROR("Error sending robot info to %s:%d : %s",
                    ip.c_str(), p, strerror(errno));
        }
      }
    }

    ::close(sock);
  }

  void periodicRobotInfoBroadcaster()
  {
    ROS_INFO("Starting periodic robot info broadcaster thread (kuavo, ports 11050-11060).");
    int robot_version = 45;
    nh_.param("/robot_version", robot_version, 45);

    while (!exit_listen_thread_for_quest3_broadcast_ && ros::ok())
    {
      sendRobotInfoOnBroadcastIps("kuavo", robot_version,
                                  ROBOT_INFO_START_PORT, ROBOT_INFO_END_PORT);

      // Sleep 1 second in 0.1 steps for more responsive exit
      for (int i = 0; i < 10; ++i)
      {
        if (exit_listen_thread_for_quest3_broadcast_ || !ros::ok())
          break;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }

    ROS_INFO("Stopping periodic robot info broadcaster thread.");
  }

  void listenForQuest3Broadcasts(int port)
  {
    int sock = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0)
    {
      return;
    }

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port        = htons(port);

    if (bind(sock, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0)
    {
      ::close(sock);
      return;
    }

    struct timeval tv;
    tv.tv_sec  = 1;
    tv.tv_usec = 0;
    if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0)
    {
      // ignore
    }

    ++listening_udp_ports_cnt_;

    char buf[1024];
    while (!exit_listen_thread_for_quest3_broadcast_ && ros::ok())
    {
      struct sockaddr_in from_addr;
      socklen_t          from_len = sizeof(from_addr);
      ssize_t            recvd    = recvfrom(sock, buf, sizeof(buf) - 1, 0,
                                      reinterpret_cast<struct sockaddr*>(&from_addr),
                                      &from_len);
      if (recvd < 0)
      {
        if (errno == EAGAIN || errno == EWOULDBLOCK)
          continue;
        else
          continue;
      }

      buf[recvd] = '\0';
      char ip_str[INET_ADDRSTRLEN] = {0};
      inet_ntop(AF_INET, &from_addr.sin_addr, ip_str, sizeof(ip_str));

      std::cout << "Received message from Quest3: " << buf
                << " from " << ip_str << " on port " << port
                << " - Setting up socket connection" << std::endl;

      exit_listen_thread_for_quest3_broadcast_ = true;
      server_ip_ = ip_str;
      setupSocket(server_ip_, QUEST3_DEFAULT_DATA_PORT);
      break;
    }

    ::close(sock);
  }

private:
  ros::NodeHandle nh_;

  std::vector<std::string> bone_names_;
  std::map<std::string, int> bone_name_to_index_;
  std::map<int, std::string> index_to_bone_name_;

  bool enable_head_control_;
  HeadMotionRange head_motion_range_;

  ros::Publisher pose_pub_;
  ros::Publisher head_data_pub_;
  ros::Publisher joysticks_pub_;
  ros::Publisher hand_finger_tf_pub_;

  tf::TransformBroadcaster tf_broadcaster_;
  tf::TransformListener    tf_listener_;

  int data_socket_;
  struct sockaddr_in server_addr_;
  std::string        server_ip_;
  int                server_port_{QUEST3_DEFAULT_DATA_PORT};

  std::vector<std::string> broadcast_ips_;
  std::atomic<int>  listening_udp_ports_cnt_;
  std::atomic<bool> exit_listen_thread_for_quest3_broadcast_;

  ros::Rate rate_{100.0};
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Quest3_bone_frame_publisher_cpp");
  ros::NodeHandle nh("~");

  Quest3BoneFramePublisherCpp publisher(nh);

  // Get local broadcast IPs and update publisher
  std::vector<std::string> broadcast_ips = getLocalBroadcastIps();
  std::cout << "Local broadcast IPs: ";
  for (const auto& ip : broadcast_ips)
  {
    std::cout << ip << " ";
  }
  std::cout << std::endl;

  publisher.updateBroadcastIps(broadcast_ips);

  std::string ip_arg;
  if (argc < 2 || std::string(argv[1]).find('.') == std::string::npos)
  {
    std::cout << "IP not specified. Waiting for Quest3 to connect. Please ensure Quest3 "
                 "and the robot are on the same LAN and the router has broadcast mode enabled.\n"
              << "未指定IP。正在等待 Quest3 主动连接。请确保 Quest3 和机器人在同一个局域网下，并且路由器已开启广播模式。"
              << std::endl;

    publisher.broadcastRobotInfoAndWaitForQuest3();
  }
  else
  {
    std::string arg = argv[1];
    std::string server_address;
    int         port = QUEST3_DEFAULT_DATA_PORT;

    auto pos = arg.find(':');
    if (pos != std::string::npos)
    {
      server_address = arg.substr(0, pos);
      port           = std::stoi(arg.substr(pos + 1));
    }
    else
    {
      server_address = arg;
    }

    if (!publisher.setupSocket(server_address, port))
    {
      ROS_ERROR("Failed to setup socket for Quest3.");
      return 1;
    }
  }

  if (publisher.sendInitialMessage())
  {
    publisher.run();
  }
  else
  {
    std::cout << "Failed to establish initial connection." << std::endl;
  }

  ros::shutdown();
  return 0;
}


