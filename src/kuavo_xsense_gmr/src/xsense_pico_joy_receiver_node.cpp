#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <atomic>
#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstring>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <memory>
#include <utility>
#include <vector>

#include <ros/ros.h>
#include <kuavo_msgs/JoySticks.h>
#include <kuavo_msgs/robotBodyMatrices.h>
#include <kuavo_msgs/switchToNextController.h>
#include <std_srvs/SetBool.h>

namespace
{

struct ByteView
{
  const uint8_t* data = nullptr;
  std::size_t size = 0;
};

struct ProtoField
{
  uint32_t number = 0;
  uint32_t wire_type = 0;
  uint64_t varint_value = 0;
  ByteView bytes;
};

struct ControllerState
{
  double thumb_x = 0.0;
  double thumb_y = 0.0;
  double grip = 0.0;
  double trigger = 0.0;
  bool primary = false;
  bool secondary = false;
};

using Matrix4 = std::array<double, 16>;

struct BroadcastTarget
{
  std::string local_ip;
  std::string broadcast_ip;
};

const std::vector<std::string>& bodyTrackerRoles()
{
  static const std::vector<std::string> roles = {
      "Pelvis", "LEFT_HIP", "RIGHT_HIP", "SPINE1", "LEFT_KNEE", "RIGHT_KNEE",
      "SPINE2", "LEFT_ANKLE", "RIGHT_ANKLE", "SPINE3", "LEFT_FOOT", "RIGHT_FOOT",
      "NECK", "LEFT_COLLAR", "RIGHT_COLLAR", "HEAD", "LEFT_SHOULDER", "RIGHT_SHOULDER",
      "LEFT_ELBOW", "RIGHT_ELBOW", "LEFT_WRIST", "RIGHT_WRIST", "LEFT_HAND", "RIGHT_HAND"};
  return roles;
}

Matrix4 identityMatrix()
{
  return Matrix4{1.0, 0.0, 0.0, 0.0,
                 0.0, 1.0, 0.0, 0.0,
                 0.0, 0.0, 1.0, 0.0,
                 0.0, 0.0, 0.0, 1.0};
}

Matrix4 multiplyMatrix(const Matrix4& lhs, const Matrix4& rhs)
{
  Matrix4 out{};
  for (int row = 0; row < 4; ++row)
  {
    for (int col = 0; col < 4; ++col)
    {
      double value = 0.0;
      for (int k = 0; k < 4; ++k)
        value += lhs[row * 4 + k] * rhs[k * 4 + col];
      out[row * 4 + col] = value;
    }
  }
  return out;
}

Matrix4 rollMatrix(const double degrees)
{
  const double radians = degrees * M_PI / 180.0;
  const double c = std::cos(radians);
  const double s = std::sin(radians);
  return Matrix4{1.0, 0.0, 0.0, 0.0,
                 0.0, c, -s, 0.0,
                 0.0, s, c, 0.0,
                 0.0, 0.0, 0.0, 1.0};
}

Matrix4 quaternionToMatrix(double qx, double qy, double qz, double qw)
{
  const double norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
  if (norm > 1e-12)
  {
    qx /= norm;
    qy /= norm;
    qz /= norm;
    qw /= norm;
  }
  else
  {
    qx = qy = qz = 0.0;
    qw = 1.0;
  }

  const double xx = qx * qx;
  const double yy = qy * qy;
  const double zz = qz * qz;
  const double xy = qx * qy;
  const double xz = qx * qz;
  const double yz = qy * qz;
  const double wx = qw * qx;
  const double wy = qw * qy;
  const double wz = qw * qz;

  Matrix4 matrix = identityMatrix();
  matrix[0] = 1.0 - 2.0 * (yy + zz);
  matrix[1] = 2.0 * (xy - wz);
  matrix[2] = 2.0 * (xz + wy);
  matrix[4] = 2.0 * (xy + wz);
  matrix[5] = 1.0 - 2.0 * (xx + zz);
  matrix[6] = 2.0 * (yz - wx);
  matrix[8] = 2.0 * (xz - wy);
  matrix[9] = 2.0 * (yz + wx);
  matrix[10] = 1.0 - 2.0 * (xx + yy);
  return matrix;
}

Matrix4 transformPicoMatrixToRobot(Matrix4 matrix, const std::size_t role_index)
{
  // 与旧 Pico 链路保持一致：Pico/Unity 坐标系 -> ROS/机器人 URDF 坐标系。
  static const Matrix4 unity_to_ros{0.0, 0.0, -1.0, 0.0,
                                    -1.0, 0.0, 0.0, 0.0,
                                    0.0, 1.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 1.0};
  static const Matrix4 ros_to_robot_urdf{0.0, -1.0, 0.0, 0.0,
                                         0.0, 0.0, 1.0, 0.0,
                                         -1.0, 0.0, 0.0, 0.0,
                                         0.0, 0.0, 0.0, 1.0};
  static const Matrix4 left_arm_correction = rollMatrix(90.0);
  static const Matrix4 right_arm_correction = rollMatrix(-90.0);

  Matrix4 out = multiplyMatrix(unity_to_ros, matrix);
  out = multiplyMatrix(out, ros_to_robot_urdf);
  out[3] = -out[3];

  static const std::array<double, 3> reflection{1.0, -1.0, -1.0};
  for (int row = 0; row < 3; ++row)
  {
    for (int col = 0; col < 3; ++col)
      out[row * 4 + col] = reflection[row] * out[row * 4 + col] * reflection[col];
  }

  if (role_index >= 16 && role_index <= 23)
  {
    const bool left_arm = (role_index == 16 || role_index == 18 || role_index == 20 || role_index == 22);
    out = multiplyMatrix(out, left_arm ? left_arm_correction : right_arm_correction);
  }

  return out;
}

// 这里只实现 Pico 手柄协议需要的最小 protobuf wire 解析，不作为通用 protobuf 解析器使用。
bool readVarint(const ByteView& view, std::size_t& offset, uint64_t& value)
{
  value = 0;
  int shift = 0;
  while (offset < view.size && shift < 64)
  {
    const uint8_t byte = view.data[offset++];
    value |= static_cast<uint64_t>(byte & 0x7F) << shift;
    if ((byte & 0x80) == 0)
      return true;
    shift += 7;
  }
  return false;
}

bool nextField(const ByteView& view, std::size_t& offset, ProtoField& field)
{
  if (offset >= view.size)
    return false;

  uint64_t key = 0;
  if (!readVarint(view, offset, key))
    throw std::runtime_error("invalid protobuf varint key");

  field = ProtoField{};
  field.number = static_cast<uint32_t>(key >> 3);
  field.wire_type = static_cast<uint32_t>(key & 0x07);
  if (field.number == 0)
    throw std::runtime_error("invalid protobuf field number");

  switch (field.wire_type)
  {
    case 0:
      if (!readVarint(view, offset, field.varint_value))
        throw std::runtime_error("invalid protobuf varint value");
      return true;

    case 1:
      if (offset + 8 > view.size)
        throw std::runtime_error("truncated protobuf fixed64");
      field.bytes = ByteView{view.data + offset, 8};
      offset += 8;
      return true;

    case 2:
    {
      uint64_t length = 0;
      if (!readVarint(view, offset, length))
        throw std::runtime_error("invalid protobuf length");
      if (length > view.size - offset)
        throw std::runtime_error("truncated protobuf length-delimited field");
      field.bytes = ByteView{view.data + offset, static_cast<std::size_t>(length)};
      offset += static_cast<std::size_t>(length);
      return true;
    }

    case 5:
      if (offset + 4 > view.size)
        throw std::runtime_error("truncated protobuf fixed32");
      field.bytes = ByteView{view.data + offset, 4};
      offset += 4;
      return true;

    default:
      throw std::runtime_error("unsupported protobuf wire type");
  }
}

bool firstMessageField(const ByteView& view, const uint32_t field_number, ByteView& value)
{
  std::size_t offset = 0;
  ProtoField field;
  while (nextField(view, offset, field))
  {
    if (field.number == field_number && field.wire_type == 2)
    {
      value = field.bytes;
      return true;
    }
  }
  return false;
}

double decodeFixed64Double(const ByteView& value)
{
  if (value.size != 8)
    throw std::runtime_error("invalid protobuf double length");
  double out = 0.0;
  std::memcpy(&out, value.data, sizeof(out));
  return out;
}

void appendPackedDoubles(const ByteView& value, std::vector<double>& out)
{
  if (value.size % 8 != 0)
    throw std::runtime_error("invalid packed double length");

  for (std::size_t offset = 0; offset < value.size; offset += 8)
  {
    double item = 0.0;
    std::memcpy(&item, value.data + offset, sizeof(item));
    out.push_back(item);
  }
}

ControllerState parseControllerState(const ByteView& view)
{
  ControllerState state;
  std::vector<double> thumbstick;

  std::size_t offset = 0;
  ProtoField field;
  while (nextField(view, offset, field))
  {
    if (field.number == 1 && field.wire_type == 0)
      state.primary = (field.varint_value != 0);
    else if (field.number == 2 && field.wire_type == 0)
      state.secondary = (field.varint_value != 0);
    else if (field.number == 5 && field.wire_type == 1)
      state.grip = decodeFixed64Double(field.bytes);
    else if (field.number == 6 && field.wire_type == 1)
      state.trigger = decodeFixed64Double(field.bytes);
    else if (field.number == 7 && field.wire_type == 1)
      thumbstick.push_back(decodeFixed64Double(field.bytes));
    else if (field.number == 7 && field.wire_type == 2)
      appendPackedDoubles(field.bytes, thumbstick);
  }

  if (!thumbstick.empty())
    state.thumb_x = thumbstick[0];
  if (thumbstick.size() > 1)
    state.thumb_y = thumbstick[1];

  return state;
}

void parsePoseToMatrix(const ByteView& view, Matrix4& matrix)
{
  double pos_x = 0.0;
  double pos_y = 0.0;
  double pos_z = 0.0;
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  double qw = 1.0;

  std::size_t offset = 0;
  ProtoField field;
  while (nextField(view, offset, field))
  {
    if (field.wire_type != 1)
      continue;

    if (field.number == 1)
      pos_x = decodeFixed64Double(field.bytes);
    else if (field.number == 2)
      pos_y = decodeFixed64Double(field.bytes);
    else if (field.number == 3)
      pos_z = decodeFixed64Double(field.bytes);
    else if (field.number == 4)
      qx = decodeFixed64Double(field.bytes);
    else if (field.number == 5)
      qy = decodeFixed64Double(field.bytes);
    else if (field.number == 6)
      qz = decodeFixed64Double(field.bytes);
    else if (field.number == 7)
      qw = decodeFixed64Double(field.bytes);
  }

  matrix = quaternionToMatrix(qx, qy, qz, qw);
  matrix[3] = pos_x;
  matrix[7] = pos_y;
  matrix[11] = pos_z;
}

bool parsePicoJoyPacket(const uint8_t* data, const std::size_t size, kuavo_msgs::JoySticks& joy)
{
  const ByteView packet{data, size};

  // VRData.controller -> ControllerData.controllers -> left/right ControllerState。
  ByteView controller_data;
  if (!firstMessageField(packet, 4, controller_data))
    return false;

  ByteView controllers;
  if (!firstMessageField(controller_data, 1, controllers))
    return false;

  ControllerState left;
  ControllerState right;
  ByteView left_data;
  ByteView right_data;
  if (firstMessageField(controllers, 1, left_data))
    left = parseControllerState(left_data);
  if (firstMessageField(controllers, 2, right_data))
    right = parseControllerState(right_data);

  joy = kuavo_msgs::JoySticks{};
  joy.left_x = static_cast<float>(left.thumb_x);
  joy.left_y = static_cast<float>(left.thumb_y);
  joy.left_grip = static_cast<float>(left.grip);
  joy.left_trigger = static_cast<float>(left.trigger);
  joy.left_first_button_pressed = left.primary;
  joy.left_second_button_pressed = left.secondary;
  joy.left_first_button_touched = false;
  joy.left_second_button_touched = false;

  joy.right_x = static_cast<float>(right.thumb_x);
  joy.right_y = static_cast<float>(right.thumb_y);
  joy.right_grip = static_cast<float>(right.grip);
  joy.right_trigger = static_cast<float>(right.trigger);
  joy.right_first_button_pressed = right.primary;
  joy.right_second_button_pressed = right.secondary;
  joy.right_first_button_touched = false;
  joy.right_second_button_touched = false;
  return true;
}

bool parsePicoBodyMatricesPacket(const uint8_t* data,
                                 const std::size_t size,
                                 const std::string& frame_id,
                                 const std::string& data_source,
                                 kuavo_msgs::robotBodyMatrices& message)
{
  const ByteView packet{data, size};

  ByteView full_body_data;
  if (!firstMessageField(packet, 2, full_body_data))
    return false;

  const auto& roles = bodyTrackerRoles();
  std::vector<Matrix4> matrices;
  matrices.reserve(roles.size());

  std::size_t offset = 0;
  ProtoField field;
  while (nextField(full_body_data, offset, field))
  {
    if (field.number != 1 || field.wire_type != 2)
      continue;

    if (matrices.size() >= roles.size())
      break;

    Matrix4 raw_matrix = identityMatrix();
    parsePoseToMatrix(field.bytes, raw_matrix);
    matrices.push_back(transformPicoMatrixToRobot(raw_matrix, matrices.size()));
  }

  if (matrices.empty())
    return false;

  message = kuavo_msgs::robotBodyMatrices{};
  const ros::Time now = ros::Time::now();
  message.header.stamp = now;
  message.header.frame_id = frame_id;
  message.timestamp = now;
  message.num_matrices = static_cast<uint32_t>(matrices.size());
  message.body_parts.assign(roles.begin(), roles.begin() + static_cast<std::ptrdiff_t>(matrices.size()));
  message.data_source = data_source;
  message.matrices_data.reserve(matrices.size() * 16);
  for (const Matrix4& matrix : matrices)
    message.matrices_data.insert(message.matrices_data.end(), matrix.begin(), matrix.end());

  return true;
}

std::vector<BroadcastTarget> getBroadcastTargets()
{
  std::vector<BroadcastTarget> targets;
  ifaddrs* interfaces = nullptr;
  if (getifaddrs(&interfaces) != 0)
  {
    ROS_WARN_STREAM("Failed to get IPv4 interfaces for Pico broadcast: " << std::strerror(errno));
    return targets;
  }

  for (ifaddrs* item = interfaces; item != nullptr; item = item->ifa_next)
  {
    // 与原 Python 节点保持一致：广播所有非回环 IPv4，让 Pico 端能自动发现机器人。
    if (item->ifa_addr == nullptr || item->ifa_addr->sa_family != AF_INET)
      continue;
    if ((item->ifa_flags & IFF_UP) == 0 || (item->ifa_flags & IFF_LOOPBACK) != 0)
      continue;

    char local_ip[INET_ADDRSTRLEN] = {0};
    const auto* local_addr = reinterpret_cast<sockaddr_in*>(item->ifa_addr);
    if (inet_ntop(AF_INET, &local_addr->sin_addr, local_ip, sizeof(local_ip)) == nullptr)
      continue;

    std::string broadcast_ip = "255.255.255.255";
    if (item->ifa_broadaddr != nullptr && (item->ifa_flags & IFF_BROADCAST) != 0)
    {
      char broadcast[INET_ADDRSTRLEN] = {0};
      const auto* broadcast_addr = reinterpret_cast<sockaddr_in*>(item->ifa_broadaddr);
      if (inet_ntop(AF_INET, &broadcast_addr->sin_addr, broadcast, sizeof(broadcast)) != nullptr)
        broadcast_ip = broadcast;
    }

    targets.push_back(BroadcastTarget{local_ip, broadcast_ip});
  }

  freeifaddrs(interfaces);
  return targets;
}

std::string jsonEscape(const std::string& input)
{
  std::ostringstream out;
  for (const char ch : input)
  {
    if (ch == '\\' || ch == '"')
      out << '\\' << ch;
    else
      out << ch;
  }
  return out.str();
}

std::string buildRobotInfoPayload(const std::string& robot_name, const std::string& robot_ip)
{
  std::ostringstream payload;
  payload << "{\"data\":{\"robot_name\":\"" << jsonEscape(robot_name)
          << "\",\"robot_ip\":\"" << jsonEscape(robot_ip)
          << "\",\"eef_type\":null}}";
  return payload.str();
}

class RobotInfoBroadcaster
{
public:
  RobotInfoBroadcaster(std::string robot_name, const int broadcast_port)
    : robot_name_(std::move(robot_name)),
      broadcast_port_(broadcast_port),
      targets_(getBroadcastTargets())
  {
  }

  void start()
  {
    if (targets_.empty())
    {
      ROS_WARN("Pico robot info broadcast has no IPv4 broadcast targets");
      return;
    }

    running_.store(true);
    thread_ = std::thread(&RobotInfoBroadcaster::run, this);

    std::ostringstream desc;
    for (std::size_t i = 0; i < targets_.size(); ++i)
    {
      if (i > 0)
        desc << ", ";
      desc << targets_[i].local_ip << "->" << targets_[i].broadcast_ip << ":" << broadcast_port_;
    }
    ROS_INFO_STREAM("Pico robot info broadcast enabled: " << desc.str());
  }

  void stop()
  {
    running_.store(false);
    if (thread_.joinable())
      thread_.join();
  }

private:
  void run()
  {
    const int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0)
    {
      ROS_WARN_STREAM("Failed to create Pico broadcast socket: " << std::strerror(errno));
      return;
    }

    const int yes = 1;
    setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(yes));
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    while (running_.load() && ros::ok())
    {
      for (const BroadcastTarget& target : targets_)
      {
        const std::string payload = buildRobotInfoPayload(robot_name_, target.local_ip);

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(static_cast<uint16_t>(broadcast_port_));
        if (inet_pton(AF_INET, target.broadcast_ip.c_str(), &addr.sin_addr) != 1)
          continue;

        const ssize_t sent = sendto(fd,
                                    payload.data(),
                                    payload.size(),
                                    0,
                                    reinterpret_cast<sockaddr*>(&addr),
                                    sizeof(addr));
        if (sent < 0)
        {
          ROS_WARN_STREAM_THROTTLE(5.0, "Failed to broadcast Pico robot info to "
                                            << target.broadcast_ip << ":" << broadcast_port_
                                            << ": " << std::strerror(errno));
        }
      }

      for (int i = 0; i < 10 && running_.load() && ros::ok(); ++i)
        ros::Duration(0.1).sleep();
    }

    close(fd);
  }

  std::string robot_name_;
  int broadcast_port_ = 8443;
  std::vector<BroadcastTarget> targets_;
  std::atomic_bool running_{false};
  std::thread thread_;
};

class PicoJoyReceiverNode
{
public:
  PicoJoyReceiverNode()
    : private_nh_("~")
  {
    private_nh_.param<std::string>("host", host_, "0.0.0.0");
    private_nh_.param<int>("port", port_, 12345);
    private_nh_.param<std::string>("joy_topic", joy_topic_, "/pico/joy");
    private_nh_.param<std::string>("body_matrices_topic", body_matrices_topic_, "/robot_body_matrices");
    private_nh_.param<std::string>("body_matrices_frame_id", body_matrices_frame_id_, "world");
    private_nh_.param<std::string>("body_matrices_data_source", body_matrices_data_source_, "pico_xsense_gmr");
    private_nh_.param<bool>("enable_body_matrices", enable_body_matrices_, true);
    private_nh_.param<bool>("enable_ip_broadcast", enable_ip_broadcast_, true);
    private_nh_.param<std::string>("robot_name", robot_name_, "KUAVO");
    private_nh_.param<int>("broadcast_port", broadcast_port_, 8443);
    private_nh_.param<double>("receive_timeout_s", receive_timeout_s_, 1.0);
    private_nh_.param<bool>("enable_vmp_stream_control", enable_vmp_stream_control_, true);
    private_nh_.param<std::string>("vmp_stream_control_service",
                                   vmp_stream_control_service_,
                                   "/vmp/pico_stream_control");
    private_nh_.param<double>("vmp_stream_trigger_threshold",
                              vmp_stream_trigger_threshold_,
                              0.5);
    // Keep the standalone receiver backward-compatible.  The launch file
    // explicitly enables this feature together with Pico AMP walking.
    private_nh_.param<bool>("enable_controller_switch", enable_controller_switch_, false);
    private_nh_.param<double>("controller_switch_grip_threshold",
                              controller_switch_grip_threshold_,
                              0.5);
    private_nh_.param<double>("controller_switch_trigger_threshold",
                              controller_switch_trigger_threshold_,
                              0.5);

    if (!std::isfinite(controller_switch_grip_threshold_) ||
        !std::isfinite(controller_switch_trigger_threshold_))
    {
      throw std::invalid_argument("controller switch thresholds must be finite");
    }
    controller_switch_grip_threshold_ =
        std::max(0.0, std::min(1.0, controller_switch_grip_threshold_));
    controller_switch_trigger_threshold_ =
        std::max(0.0, std::min(1.0, controller_switch_trigger_threshold_));

    joy_pub_ = nh_.advertise<kuavo_msgs::JoySticks>(joy_topic_, 10);
    if (enable_body_matrices_)
      body_matrices_pub_ = nh_.advertise<kuavo_msgs::robotBodyMatrices>(body_matrices_topic_, 10);
    if (enable_vmp_stream_control_)
    {
      vmp_stream_control_client_ =
          nh_.serviceClient<std_srvs::SetBool>(vmp_stream_control_service_);
      ROS_INFO_STREAM("VMP stream control enabled: RT+Y=pause, RT+X=resume, service="
                      << vmp_stream_control_service_);
    }
    if (enable_controller_switch_)
    {
      switch_next_client_ = nh_.serviceClient<kuavo_msgs::switchToNextController>(
          "/humanoid_controller/switch_to_next_controller");
      switch_previous_client_ = nh_.serviceClient<kuavo_msgs::switchToNextController>(
          "/humanoid_controller/switch_to_previous_controller");
      ROS_INFO_STREAM("Pico controller switch enabled: RG+A=next, RG+B=previous, grip_threshold="
                      << controller_switch_grip_threshold_ << ", trigger_exclusion_threshold="
                      << controller_switch_trigger_threshold_);
    }

    if (enable_ip_broadcast_)
    {
      broadcaster_.reset(new RobotInfoBroadcaster(robot_name_, broadcast_port_));
      broadcaster_->start();
    }
  }

  ~PicoJoyReceiverNode()
  {
    stopControllerSwitchWorkers();
    if (broadcaster_)
      broadcaster_->stop();
    if (socket_fd_ >= 0)
      close(socket_fd_);
  }

  bool openSocket()
  {
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0)
    {
      ROS_ERROR_STREAM("Failed to create Pico joy UDP socket: " << std::strerror(errno));
      return false;
    }

    const int yes = 1;
    setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    timeval timeout{};
    timeout.tv_sec = static_cast<int>(receive_timeout_s_);
    timeout.tv_usec = static_cast<int>((receive_timeout_s_ - timeout.tv_sec) * 1000000.0);
    setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port_));
    if (host_.empty() || host_ == "0.0.0.0")
    {
      addr.sin_addr.s_addr = htonl(INADDR_ANY);
    }
    else if (inet_pton(AF_INET, host_.c_str(), &addr.sin_addr) != 1)
    {
      ROS_ERROR_STREAM("Invalid Pico joy listen host: " << host_);
      return false;
    }

    if (bind(socket_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0)
    {
      ROS_ERROR_STREAM("Failed to bind Pico joy UDP socket on " << host_ << ":" << port_
                                                               << ": " << std::strerror(errno));
      return false;
    }

    ROS_INFO_STREAM("Pico joy receiver listening on " << host_ << ":" << port_);
    ROS_INFO_STREAM("Publishing Pico controller data to " << joy_topic_);
    if (enable_body_matrices_)
      ROS_INFO_STREAM("Publishing Pico full_body matrices to " << body_matrices_topic_);
    else
      ROS_INFO("Pico full_body matrices publishing disabled");
    return true;
  }

  void run()
  {
    if (!openSocket())
      return;

    std::vector<uint8_t> buffer(65535);
    while (ros::ok())
    {
      sockaddr_in peer{};
      socklen_t peer_len = sizeof(peer);
      const ssize_t received = recvfrom(socket_fd_,
                                        buffer.data(),
                                        buffer.size(),
                                        0,
                                        reinterpret_cast<sockaddr*>(&peer),
                                        &peer_len);

      if (received < 0)
      {
        if (errno == EAGAIN || errno == EWOULDBLOCK)
        {
          ros::spinOnce();
          continue;
        }
        ROS_ERROR_STREAM("Pico joy UDP receive error: " << std::strerror(errno));
        break;
      }

      try
      {
        bool parsed_any = false;

        kuavo_msgs::JoySticks joy;
        if (parsePicoJoyPacket(buffer.data(), static_cast<std::size_t>(received), joy))
        {
          joy_pub_.publish(joy);
          handleVmpStreamButtons(joy);
          handleControllerSwitchButtons(joy);
          parsed_any = true;
          ROS_INFO_THROTTLE(5.0, "Receiving Pico controller data and publishing %s", joy_topic_.c_str());
        }

        if (enable_body_matrices_)
        {
          kuavo_msgs::robotBodyMatrices body_matrices;
          if (parsePicoBodyMatricesPacket(buffer.data(),
                                          static_cast<std::size_t>(received),
                                          body_matrices_frame_id_,
                                          body_matrices_data_source_,
                                          body_matrices))
          {
            body_matrices_pub_.publish(body_matrices);
            parsed_any = true;
            ROS_INFO_THROTTLE(5.0, "Receiving Pico full_body data and publishing %s",
                              body_matrices_topic_.c_str());
          }
        }

        if (!parsed_any)
          ROS_DEBUG_THROTTLE(2.0, "Pico UDP packet has no controller/full_body data");
      }
      catch (const std::exception& exc)
      {
        ROS_WARN_STREAM_THROTTLE(2.0, "Failed to parse Pico protobuf UDP packet: " << exc.what());
      }

      ros::spinOnce();
    }
  }

private:
  void handleControllerSwitchButtons(const kuavo_msgs::JoySticks& joy)
  {
    if (!enable_controller_switch_)
      return;

    const bool right_grip_pressed =
        joy.right_grip >= controller_switch_grip_threshold_;
    const bool a_pressed = joy.right_first_button_pressed;
    const bool b_pressed = joy.right_second_button_pressed;
    const bool a_rising = a_pressed && !previous_a_pressed_;
    const bool b_rising = b_pressed && !previous_b_pressed_;

    previous_a_pressed_ = a_pressed;
    previous_b_pressed_ = b_pressed;

    if (!right_grip_pressed)
      return;

    if (a_rising)
    {
      requestControllerSwitch(true);
    }
    else if (b_rising &&
             joy.left_trigger < controller_switch_trigger_threshold_ &&
             joy.right_trigger < controller_switch_trigger_threshold_)
    {
      // LT+B/RT+B may be used by calibration flows; do not also switch controllers.
      requestControllerSwitch(false);
    }
  }

  void requestControllerSwitch(const bool next)
  {
    if (switch_worker_state_->in_flight.exchange(true))
    {
      ROS_WARN_STREAM_THROTTLE(2.0,
                               "Ignore Pico controller switch: another request is still in flight");
      return;
    }

    // ServiceClient::call has no application-level timeout.  Run at most one
    // call outside the UDP receive loop so controller-manager latency cannot
    // stop Joy/body-matrix reception.  The worker captures no node object;
    // shared state and the copied client remain valid if shutdown races it.
    ros::ServiceClient client = next ? switch_next_client_ : switch_previous_client_;
    const std::string service_name = next
                                         ? "/humanoid_controller/switch_to_next_controller"
                                         : "/humanoid_controller/switch_to_previous_controller";
    const std::string combo = next ? "RG+A" : "RG+B";
    const std::shared_ptr<ControllerSwitchWorkerState> state = switch_worker_state_;
    try
    {
      std::thread([client, service_name, combo, state]() mutable {
        const auto finish = [&state]() {
          state->in_flight.store(false);
          state->finished_cv.notify_all();
        };
        if (!state->owner_alive.load() || !ros::ok())
        {
          finish();
          return;
        }

        bool call_ok = false;
        kuavo_msgs::switchToNextController request;
        std::string error_message;
        bool service_available = false;
        try
        {
          service_available = client.waitForExistence(ros::Duration(0.2));
          if (service_available && state->owner_alive.load() && ros::ok())
            call_ok = client.call(request);
        }
        catch (const std::exception& exception)
        {
          error_message = exception.what();
        }
        catch (...)
        {
          error_message = "unknown exception";
        }

        const bool may_log = state->owner_alive.load() && ros::ok();
        if (may_log && !error_message.empty())
        {
          ROS_ERROR_STREAM("Controller switch worker failed: " << error_message);
        }
        else if (may_log && !service_available)
        {
          ROS_WARN_STREAM("Controller switch service unavailable: " << service_name);
        }
        else if (may_log && !call_ok)
        {
          ROS_WARN_STREAM("Failed to call controller switch service: " << service_name);
        }
        else if (may_log && request.response.success)
        {
          ROS_INFO_STREAM("Controller switch (" << combo << "): "
                                                 << request.response.current_controller << " -> "
                                                 << request.response.next_controller);
        }
        else if (may_log)
        {
          ROS_WARN_STREAM("Controller switch (" << combo << ") rejected: "
                                                 << request.response.message);
        }
        finish();
      }).detach();
    }
    catch (const std::exception& exception)
    {
      state->in_flight.store(false);
      state->finished_cv.notify_all();
      ROS_ERROR_STREAM("Failed to start controller switch worker: " << exception.what());
    }
    catch (...)
    {
      state->in_flight.store(false);
      state->finished_cv.notify_all();
      ROS_ERROR("Failed to start controller switch worker: unknown exception");
    }
  }

  void stopControllerSwitchWorkers()
  {
    const std::shared_ptr<ControllerSwitchWorkerState> state = switch_worker_state_;
    state->owner_alive.store(false);
    if (!state->in_flight.load())
      return;

    // Normal local calls complete quickly.  Give them a bounded grace period,
    // but never make receiver shutdown depend indefinitely on a remote service.
    std::unique_lock<std::mutex> lock(state->finished_mutex);
    state->finished_cv.wait_for(lock, std::chrono::milliseconds(200), [state]() {
      return !state->in_flight.load();
    });
  }

  struct ControllerSwitchWorkerState
  {
    std::atomic_bool in_flight{false};
    std::atomic_bool owner_alive{true};
    std::mutex finished_mutex;
    std::condition_variable finished_cv;
  };

  void handleVmpStreamButtons(const kuavo_msgs::JoySticks& joy)
  {
    if (!enable_vmp_stream_control_)
      return;

    const bool x_pressed = joy.left_first_button_pressed;
    const bool y_pressed = joy.left_second_button_pressed;
    const bool rt_pressed = joy.right_trigger >= vmp_stream_trigger_threshold_;
    const bool pause_requested = rt_pressed && y_pressed && !previous_y_pressed_;
    const bool resume_requested = rt_pressed && x_pressed && !previous_x_pressed_;

    previous_x_pressed_ = x_pressed;
    previous_y_pressed_ = y_pressed;

    // Match pico_comm_minimal.py: button rising edges while RT is held.
    if (pause_requested)
      callVmpStreamControl(true, "RT+Y");
    if (resume_requested)
      callVmpStreamControl(false, "RT+X");
  }

  void callVmpStreamControl(const bool pause, const char* combo)
  {
    if (!vmp_stream_control_client_.exists())
    {
      ROS_WARN_STREAM_THROTTLE(2.0, "VMP stream control service is unavailable: "
                                        << vmp_stream_control_service_);
      return;
    }

    std_srvs::SetBool request;
    request.request.data = pause;
    if (!vmp_stream_control_client_.call(request))
    {
      ROS_WARN_STREAM("Failed to call VMP stream control service "
                      << vmp_stream_control_service_ << " (" << combo << ")");
      return;
    }

    if (request.response.success)
    {
      ROS_INFO_STREAM("VMP stream " << (pause ? "paused" : "resumed")
                                    << " by Pico combo " << combo);
    }
    else
    {
      ROS_WARN_STREAM("VMP stream control rejected " << combo << ": "
                                                     << request.response.message);
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher joy_pub_;
  ros::Publisher body_matrices_pub_;
  ros::ServiceClient vmp_stream_control_client_;
  ros::ServiceClient switch_next_client_;
  ros::ServiceClient switch_previous_client_;
  std::unique_ptr<RobotInfoBroadcaster> broadcaster_;
  int socket_fd_ = -1;

  std::string host_;
  int port_ = 12345;
  std::string joy_topic_;
  std::string body_matrices_topic_;
  std::string body_matrices_frame_id_;
  std::string body_matrices_data_source_;
  bool enable_body_matrices_ = true;
  bool enable_ip_broadcast_ = true;
  std::string robot_name_;
  int broadcast_port_ = 8443;
  double receive_timeout_s_ = 1.0;
  bool enable_vmp_stream_control_ = true;
  std::string vmp_stream_control_service_;
  double vmp_stream_trigger_threshold_ = 0.5;
  bool previous_x_pressed_ = false;
  bool previous_y_pressed_ = false;
  bool enable_controller_switch_ = false;
  double controller_switch_grip_threshold_ = 0.5;
  double controller_switch_trigger_threshold_ = 0.5;
  bool previous_a_pressed_ = false;
  bool previous_b_pressed_ = false;
  std::shared_ptr<ControllerSwitchWorkerState> switch_worker_state_ =
      std::make_shared<ControllerSwitchWorkerState>();
};

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "xsense_pico_joy_receiver");
  PicoJoyReceiverNode node;
  node.run();
  return 0;
}
