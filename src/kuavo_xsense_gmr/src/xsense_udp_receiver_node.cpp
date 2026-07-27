#include <algorithm>
#include <cctype>
#include <deque>
#include <iomanip>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <kuavo_msgs/JoySticks.h>
#include <kuavo_msgs/xsensePoseInfo.h>
#include <kuavo_msgs/xsensePoseInfoList.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include "kuavo_xsense_gmr/xsens_udp/fragment_assembler.h"
#include "kuavo_xsense_gmr/xsens_udp/parser.h"
#include "kuavo_xsense_gmr/xsens_udp/udp_receiver.h"

namespace
{

struct Publishers
{
  ros::Publisher summary;
  ros::Publisher pose_info;
  ros::Publisher pose_queue_stats;
};

struct BodyMapping
{
  const char* output_name;
  const char* xsens_segment_name;
};

const std::vector<BodyMapping>& bodyMappings()
{
  static const std::vector<BodyMapping> mappings = {
      {"Hips", "Pelvis"},
      {"Chest", "L5"},
      {"Neck", "Neck"},
      {"Head", "Head"},
      {"LeftCollar", "Left Shoulder"},
      {"LeftShoulder", "Left Upper Arm"},
      {"LeftElbow", "Left Forearm"},
      {"LeftWrist", "Left Hand"},
      {"RightCollar", "Right Shoulder"},
      {"RightShoulder", "Right Upper Arm"},
      {"RightElbow", "Right Forearm"},
      {"RightWrist", "Right Hand"},
      {"LeftHip", "Left Upper Leg"},
      {"LeftKnee", "Left Lower Leg"},
      {"LeftAnkle", "Left Foot"},
      {"LeftToe", "Left Toe"},
      {"RightHip", "Right Upper Leg"},
      {"RightKnee", "Right Lower Leg"},
      {"RightAnkle", "Right Foot"},
      {"RightToe", "Right Toe"},
  };
  return mappings;
}

std::string jsonEscape(const std::string& value)
{
  std::ostringstream out;
  for (const char c : value)
  {
    switch (c)
    {
      case '\\':
        out << "\\\\";
        break;
      case '"':
        out << "\\\"";
        break;
      case '\n':
        out << "\\n";
        break;
      case '\r':
        out << "\\r";
        break;
      case '\t':
        out << "\\t";
        break;
      default:
        out << c;
        break;
    }
  }
  return out.str();
}

std::size_t parsedItemCount(const kuavo_xsense_gmr::ParsedPacket& packet)
{
  const std::string& type = packet.header.message_type;
  if (type == "01")
    return packet.pose_euler.size();
  if (type == "02")
    return packet.pose_quaternion.size();
  if (type == "03")
    return packet.point_positions.size();
  if (type == "05")
    return packet.unity_pose.size();
  if (type == "12")
    return packet.metadata_tags.size();
  if (type == "13")
    return packet.scale_segments.size() + packet.scale_points.size();
  if (type == "20")
    return packet.joint_angles.size();
  if (type == "21")
    return packet.linear_kinematics.size();
  if (type == "22")
    return packet.angular_kinematics.size();
  if (type == "23")
    return packet.tracker_kinematics.size();
  if (type == "24")
    return packet.valid ? 1U : 0U;
  if (type == "25")
    return packet.time_code.empty() ? 0U : 1U;
  return packet.raw_payload_only ? packet.payload.size() : 0U;
}

std::string packetSummaryJson(const kuavo_xsense_gmr::ParsedPacket& packet,
                              const int64_t sample_delta)
{
  const kuavo_xsense_gmr::XsensPacketHeader& header = packet.header;
  std::ostringstream json;
  json << "{";
  json << "\"valid\":" << (packet.valid ? "true" : "false");
  json << ",\"id_string\":\"" << jsonEscape(header.id_string) << "\"";
  json << ",\"type\":\"" << jsonEscape(header.message_type) << "\"";
  json << ",\"sample_counter\":" << header.sample_counter;
  json << ",\"sample_delta\":" << sample_delta;
  json << ",\"datagram_counter\":" << static_cast<unsigned int>(header.datagram_counter);
  json << ",\"fragment_index\":" << static_cast<unsigned int>(header.fragment_index());
  json << ",\"is_last_fragment\":" << (header.is_last_fragment() ? "true" : "false");
  json << ",\"reconstructed\":" << (packet.reconstructed ? "true" : "false");
  json << ",\"items\":" << static_cast<unsigned int>(header.number_of_items);
  json << ",\"parsed_items\":" << parsedItemCount(packet);
  json << ",\"time_code\":" << header.time_code;
  json << ",\"character_id\":" << static_cast<unsigned int>(header.character_id);
  json << ",\"body_segments\":" << static_cast<unsigned int>(header.body_segments);
  json << ",\"props\":" << static_cast<unsigned int>(header.props);
  json << ",\"finger_tracking_segments\":"
       << static_cast<unsigned int>(header.finger_tracking_segments);
  json << ",\"reserved\":" << header.reserved;
  json << ",\"payload_size\":" << header.payload_size;
  json << ",\"datagram_size\":" << header.datagram_size;
  json << ",\"raw_payload_only\":" << (packet.raw_payload_only ? "true" : "false");
  json << ",\"coordinate_system\":\"" << jsonEscape(packet.coordinate_system) << "\"";
  json << ",\"invalid_reason\":\"" << jsonEscape(packet.invalid_reason) << "\"";
  json << "}";
  return json.str();
}

geometry_msgs::Point toPoint(const kuavo_xsense_gmr::Vector3f& value)
{
  geometry_msgs::Point point;
  point.x = value.x;
  point.y = value.y;
  point.z = value.z;
  return point;
}

geometry_msgs::Quaternion toQuaternion(const kuavo_xsense_gmr::Quaternionf& value)
{
  geometry_msgs::Quaternion quaternion;
  quaternion.w = value.w;
  quaternion.x = value.x;
  quaternion.y = value.y;
  quaternion.z = value.z;
  return quaternion;
}

std::string segmentNameFromId(const uint32_t segment_id)
{
  static const char* kStandardNames[] = {
      "Pelvis",          "L5",            "L3",           "T12",
      "T8",              "Neck",          "Head",         "Right Shoulder",
      "Right Upper Arm", "Right Forearm", "Right Hand",   "Left Shoulder",
      "Left Upper Arm",  "Left Forearm",  "Left Hand",    "Right Upper Leg",
      "Right Lower Leg", "Right Foot",    "Right Toe",    "Left Upper Leg",
      "Left Lower Leg",  "Left Foot",     "Left Toe"};
  constexpr std::size_t kNameCount = 23;

  if (segment_id >= 1 && segment_id <= kNameCount)
    return kStandardNames[segment_id - 1];
  if (segment_id == 0)
    return kStandardNames[0];
  if (segment_id == 24)
    return "Prop1";
  if (segment_id >= 25 && segment_id <= 28)
    return "Prop" + std::to_string(segment_id - 24);
  return "";
}

std::string normalizeComboToken(const std::string& raw_token)
{
  std::size_t first = 0;
  while (first < raw_token.size() &&
         std::isspace(static_cast<unsigned char>(raw_token[first])))
  {
    ++first;
  }

  std::size_t last = raw_token.size();
  while (last > first && std::isspace(static_cast<unsigned char>(raw_token[last - 1])))
  {
    --last;
  }

  std::string token = raw_token.substr(first, last - first);
  for (char& c : token)
    c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
  return token;
}

std::vector<std::string> parseComboTokens(const std::string& combo)
{
  std::vector<std::string> tokens;
  std::string current;

  const auto flush_current = [&tokens, &current]() {
    const std::string token = normalizeComboToken(current);
    if (!token.empty())
      tokens.push_back(token);
    current.clear();
  };

  for (const char c : combo)
  {
    if (c == '+' || c == ',' || std::isspace(static_cast<unsigned char>(c)))
    {
      flush_current();
      continue;
    }
    current.push_back(c);
  }
  flush_current();

  return tokens;
}

std::string comboTokensToString(const std::vector<std::string>& tokens)
{
  if (tokens.empty())
    return "<empty>";

  std::ostringstream out;
  for (std::size_t i = 0; i < tokens.size(); ++i)
  {
    if (i > 0)
      out << "+";
    out << tokens[i];
  }
  return out.str();
}

bool comboTokenActive(const kuavo_msgs::JoySticks& joy,
                      const std::string& token,
                      const double trigger_threshold,
                      const double grip_threshold)
{
  if (token == "X")
    return joy.left_first_button_pressed;
  if (token == "Y")
    return joy.left_second_button_pressed;
  if (token == "A")
    return joy.right_first_button_pressed;
  if (token == "B")
    return joy.right_second_button_pressed;
  if (token == "LT" || token == "LEFT_TRIGGER" || token == "L_TRIGGER")
    return joy.left_trigger >= trigger_threshold;
  if (token == "RT" || token == "RIGHT_TRIGGER" || token == "R_TRIGGER")
    return joy.right_trigger >= trigger_threshold;
  if (token == "LG" || token == "LEFT_GRIP" || token == "L_GRIP")
    return joy.left_grip >= grip_threshold;
  if (token == "RG" || token == "RIGHT_GRIP" || token == "R_GRIP")
    return joy.right_grip >= grip_threshold;

  ROS_WARN_STREAM_THROTTLE(2.0, "Unknown Pico joy combo token '" << token
                                                                 << "'. Supported tokens: X,Y,A,B,LT,RT,LG,RG");
  return false;
}

bool comboActive(const kuavo_msgs::JoySticks& joy,
                 const std::vector<std::string>& tokens,
                 const double trigger_threshold,
                 const double grip_threshold)
{
  if (tokens.empty())
    return false;

  // 组合键需全部按下（AND）
  return std::all_of(tokens.begin(), tokens.end(),
                     [&](const std::string& token) {
                       return comboTokenActive(joy, token, trigger_threshold, grip_threshold);
                     });
}

struct PicoPauseControlState
{
  bool enabled = false;
  bool paused = false;
  bool last_pause_combo_active = false;
  bool last_resume_combo_active = false;
  std::mutex mutex;
};

struct PosePublishQueue
{
  // 该队列用于吸收 Xsens UDP burst 到达，把 ROS 姿态话题发布节奏稳定到 pose_publish_hz。
  std::deque<kuavo_msgs::xsensePoseInfoList> queue;
  kuavo_msgs::xsensePoseInfoList last_published_msg;
  bool have_last_published = false;
  bool prefill_ready = false;
  bool have_input = false;
  ros::WallTime last_input_time;
  std::size_t underflow_count = 0;
  std::size_t overflow_drop_count = 0;
  std::size_t enqueued_count = 0;
  std::size_t published_count = 0;
  std::size_t hold_publish_count = 0;
  std::size_t paused_hold_publish_count = 0;

  bool stats_initialized = false;
  ros::WallTime stats_last_time;
  std::size_t stats_last_depth = 0;
  double stats_window_depth_time_sum = 0.0;
  double stats_total_depth_time_sum = 0.0;
  double stats_window_elapsed_s = 0.0;
  double stats_total_elapsed_s = 0.0;
  std::size_t stats_window_min_depth = 0;
  std::size_t stats_window_max_depth = 0;
  std::size_t stats_total_min_depth = 0;
  std::size_t stats_total_max_depth = 0;
  std::size_t stats_window_enqueued_count = 0;
  std::size_t stats_window_published_count = 0;
  std::size_t stats_window_underflow_count = 0;
  std::size_t stats_window_overflow_drop_count = 0;
  std::size_t stats_window_hold_publish_count = 0;
  std::size_t stats_window_paused_hold_publish_count = 0;
  std::mutex mutex;
};

void observePoseQueueDepthLocked(PosePublishQueue& pose_queue)
{
  const std::size_t depth = pose_queue.queue.size();
  pose_queue.stats_last_depth = depth;

  if (depth < pose_queue.stats_window_min_depth)
    pose_queue.stats_window_min_depth = depth;
  if (depth > pose_queue.stats_window_max_depth)
    pose_queue.stats_window_max_depth = depth;
  if (depth < pose_queue.stats_total_min_depth)
    pose_queue.stats_total_min_depth = depth;
  if (depth > pose_queue.stats_total_max_depth)
    pose_queue.stats_total_max_depth = depth;
}

void accountPoseQueueDepthLocked(PosePublishQueue& pose_queue, const ros::WallTime& now)
{
  if (!pose_queue.stats_initialized)
  {
    const std::size_t depth = pose_queue.queue.size();
    pose_queue.stats_initialized = true;
    pose_queue.stats_last_time = now;
    pose_queue.stats_last_depth = depth;
    pose_queue.stats_window_min_depth = depth;
    pose_queue.stats_window_max_depth = depth;
    pose_queue.stats_total_min_depth = depth;
    pose_queue.stats_total_max_depth = depth;
    return;
  }

  const double dt_s = (now - pose_queue.stats_last_time).toSec();
  if (dt_s > 0.0)
  {
    pose_queue.stats_window_depth_time_sum +=
        static_cast<double>(pose_queue.stats_last_depth) * dt_s;
    pose_queue.stats_total_depth_time_sum +=
        static_cast<double>(pose_queue.stats_last_depth) * dt_s;
    pose_queue.stats_window_elapsed_s += dt_s;
    pose_queue.stats_total_elapsed_s += dt_s;
  }
  pose_queue.stats_last_time = now;
}

void resetPoseQueueStatsWindowLocked(PosePublishQueue& pose_queue)
{
  const std::size_t current_depth = pose_queue.queue.size();
  pose_queue.stats_window_depth_time_sum = 0.0;
  pose_queue.stats_window_elapsed_s = 0.0;
  pose_queue.stats_window_min_depth = current_depth;
  pose_queue.stats_window_max_depth = current_depth;
  pose_queue.stats_window_enqueued_count = 0;
  pose_queue.stats_window_published_count = 0;
  pose_queue.stats_window_underflow_count = 0;
  pose_queue.stats_window_overflow_drop_count = 0;
  pose_queue.stats_window_hold_publish_count = 0;
  pose_queue.stats_window_paused_hold_publish_count = 0;
}

std::string boolJson(const bool value)
{
  return value ? "true" : "false";
}

std::string poseQueueStatsJsonLocked(PosePublishQueue& pose_queue,
                                     const ros::WallTime& now,
                                     const double pose_publish_hz,
                                     const std::size_t queue_target_depth,
                                     const std::size_t queue_max_size,
                                     const bool pose_queue_prefill,
                                     const bool pose_hold_on_empty,
                                     const bool paused)
{
  accountPoseQueueDepthLocked(pose_queue, now);
  observePoseQueueDepthLocked(pose_queue);

  const std::size_t current_depth = pose_queue.queue.size();
  const double window_s = pose_queue.stats_window_elapsed_s;
  const double total_s = pose_queue.stats_total_elapsed_s;
  const double avg_depth_window =
      window_s > 0.0 ? pose_queue.stats_window_depth_time_sum / window_s
                     : static_cast<double>(current_depth);
  const double avg_depth_total =
      total_s > 0.0 ? pose_queue.stats_total_depth_time_sum / total_s
                    : static_cast<double>(current_depth);
  const double enqueued_rate_hz =
      window_s > 0.0 ? static_cast<double>(pose_queue.stats_window_enqueued_count) / window_s : 0.0;
  const double published_rate_hz =
      window_s > 0.0 ? static_cast<double>(pose_queue.stats_window_published_count) / window_s : 0.0;
  const double last_input_age_ms =
      pose_queue.have_input ? (now - pose_queue.last_input_time).toSec() * 1000.0 : -1.0;

  std::ostringstream out;
  out << std::fixed << std::setprecision(3);
  out << "{";
  out << "\"window_s\":" << window_s;
  out << ",\"pose_publish_hz\":" << pose_publish_hz;
  out << ",\"queue_target_depth\":" << queue_target_depth;
  out << ",\"queue_max_size\":" << queue_max_size;
  out << ",\"current_depth\":" << current_depth;
  out << ",\"avg_depth_window\":" << avg_depth_window;
  out << ",\"min_depth_window\":" << pose_queue.stats_window_min_depth;
  out << ",\"max_depth_window\":" << pose_queue.stats_window_max_depth;
  out << ",\"avg_depth_total\":" << avg_depth_total;
  out << ",\"min_depth_total\":" << pose_queue.stats_total_min_depth;
  out << ",\"max_depth_total\":" << pose_queue.stats_total_max_depth;
  out << ",\"hit_max_window\":" << boolJson(pose_queue.stats_window_max_depth >= queue_max_size);
  out << ",\"is_full_now\":" << boolJson(current_depth >= queue_max_size);
  out << ",\"enqueued_window\":" << pose_queue.stats_window_enqueued_count;
  out << ",\"published_window\":" << pose_queue.stats_window_published_count;
  out << ",\"input_rate_hz_window\":" << enqueued_rate_hz;
  out << ",\"output_rate_hz_window\":" << published_rate_hz;
  out << ",\"overflow_drop_window\":" << pose_queue.stats_window_overflow_drop_count;
  out << ",\"underflow_window\":" << pose_queue.stats_window_underflow_count;
  out << ",\"hold_publish_window\":" << pose_queue.stats_window_hold_publish_count;
  out << ",\"paused_hold_publish_window\":" << pose_queue.stats_window_paused_hold_publish_count;
  out << ",\"enqueued_total\":" << pose_queue.enqueued_count;
  out << ",\"published_total\":" << pose_queue.published_count;
  out << ",\"overflow_drop_total\":" << pose_queue.overflow_drop_count;
  out << ",\"underflow_total\":" << pose_queue.underflow_count;
  out << ",\"hold_publish_total\":" << pose_queue.hold_publish_count;
  out << ",\"paused_hold_publish_total\":" << pose_queue.paused_hold_publish_count;
  out << ",\"last_input_age_ms\":" << last_input_age_ms;
  out << ",\"prefill_ready\":" << boolJson(pose_queue.prefill_ready);
  out << ",\"pose_queue_prefill\":" << boolJson(pose_queue_prefill);
  out << ",\"pose_hold_on_empty\":" << boolJson(pose_hold_on_empty);
  out << ",\"paused\":" << boolJson(paused);
  out << "}";
  return out.str();
}

bool buildPoseInfoList(const kuavo_xsense_gmr::ParsedPacket& packet,
                       const std::string& frame_id,
                       kuavo_msgs::xsensePoseInfoList* msg)
{
  if (msg == nullptr)
    return false;
  if (!packet.valid || packet.header.message_type != "02")
    return false;

  std::map<std::string, const kuavo_xsense_gmr::XsensSegmentPoseQuaternion*> segment_by_name;
  for (const auto& segment : packet.pose_quaternion)
  {
    const std::string name = segmentNameFromId(segment.segment_id);
    if (!name.empty())
      segment_by_name[name] = &segment;
  }

  msg->poses.clear();
  msg->header.stamp = ros::Time::now();
  msg->header.frame_id = frame_id;

  for (const BodyMapping& mapping : bodyMappings())
  {
    const auto it = segment_by_name.find(mapping.xsens_segment_name);
    if (it == segment_by_name.end())
    {
      ROS_WARN_STREAM_THROTTLE(2.0, "Skipping Xsens pose frame: missing segment "
                                        << mapping.xsens_segment_name);
      return false;
    }

    const kuavo_xsense_gmr::XsensSegmentPoseQuaternion& source = *it->second;
    kuavo_msgs::xsensePoseInfo pose;
    pose.name = mapping.output_name;
    pose.segment_id = source.segment_id;
    pose.position = toPoint(source.position_m);
    pose.orientation = toQuaternion(source.orientation);
    msg->poses.push_back(pose);
  }

  return true;
}

bool isReconstructedPacket(const std::vector<uint8_t>& original_datagram,
                           const kuavo_xsense_gmr::XsensPacketParser& parser)
{
  const kuavo_xsense_gmr::XsensPacketHeader header = parser.parseHeader(original_datagram);
  if (!header.valid)
    return false;
  return !(header.fragment_index() == 0 && header.is_last_fragment());
}

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "xsense_udp_receiver_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string listen_ip;
  int listen_port = 9763;
  int expected_character_id = -1;
  double summary_rate_limit_hz = 20.0;
  int receive_timeout_ms = 100;
  double fragment_timeout_s = 1.0;
  std::string pose_topic;
  std::string summary_topic;
  std::string frame_id;
  bool enable_pico_pause_control = true;
  std::string pico_joy_topic;
  std::string pico_pause_combo;
  std::string pico_resume_combo;
  double pico_trigger_threshold = 0.5;
  double pico_grip_threshold = 0.5;
  double pose_publish_hz = 120.0;
  int pose_queue_target_depth = 5;
  int pose_queue_max_size = 10;
  bool pose_queue_prefill = true;
  bool pose_hold_on_empty = true;
  double pose_stale_timeout_s = 0.2;
  bool update_output_header_stamp = true;
  std::string pose_queue_stats_topic;
  double pose_queue_stats_hz = 0.0;

  pnh.param<std::string>("listen_ip", listen_ip, "0.0.0.0");
  pnh.param("listen_port", listen_port, 9763);
  pnh.param("expected_character_id", expected_character_id, -1);
  pnh.param("summary_rate_limit_hz", summary_rate_limit_hz, 20.0);
  pnh.param("receive_timeout_ms", receive_timeout_ms, 100);
  pnh.param("fragment_timeout_s", fragment_timeout_s, 1.0);
  pnh.param<std::string>("pose_topic", pose_topic, "/xsense/world_bone_poses");
  pnh.param<std::string>("summary_topic", summary_topic, "/xsense/packet_summary");
  pnh.param<std::string>("frame_id", frame_id, "xsense");
  pnh.param("enable_pico_pause_control", enable_pico_pause_control, true);
  pnh.param<std::string>("pico_joy_topic", pico_joy_topic, "/pico/joy");
  pnh.param<std::string>("pico_pause_combo", pico_pause_combo, "RT+Y");
  pnh.param<std::string>("pico_resume_combo", pico_resume_combo, "RT+X");
  pnh.param("pico_trigger_threshold", pico_trigger_threshold, 0.5);
  pnh.param("pico_grip_threshold", pico_grip_threshold, 0.5);
  pnh.param("pose_publish_hz", pose_publish_hz, 120.0);
  pnh.param("pose_queue_target_depth", pose_queue_target_depth, 10);
  pnh.param("pose_queue_max_size", pose_queue_max_size, 15);
  pnh.param("pose_queue_prefill", pose_queue_prefill, true);
  pnh.param("pose_hold_on_empty", pose_hold_on_empty, true);
  pnh.param("pose_stale_timeout_s", pose_stale_timeout_s, 0.2);
  pnh.param("update_output_header_stamp", update_output_header_stamp, true);
  pnh.param<std::string>("pose_queue_stats_topic", pose_queue_stats_topic, "/xsense/pose_queue_stats");
  pnh.param("pose_queue_stats_hz", pose_queue_stats_hz, 0.0);

  if (listen_port <= 0 || listen_port > 65535)
  {
    ROS_FATAL_STREAM("Invalid listen_port: " << listen_port);
    return 1;
  }
  if (summary_rate_limit_hz <= 0.0)
    summary_rate_limit_hz = 20.0;
  if (fragment_timeout_s <= 0.0)
    fragment_timeout_s = 1.0;
  if (pico_trigger_threshold < 0.0)
    pico_trigger_threshold = 0.0;
  if (pico_grip_threshold < 0.0)
    pico_grip_threshold = 0.0;
  if (pose_publish_hz <= 0.0)
    pose_publish_hz = 120.0;
  if (pose_queue_max_size <= 0)
    pose_queue_max_size = 1;
  if (pose_queue_target_depth <= 0)
    pose_queue_target_depth = 1;
  if (pose_queue_target_depth > pose_queue_max_size)
    pose_queue_target_depth = pose_queue_max_size;
  if (pose_stale_timeout_s <= 0.0)
    pose_stale_timeout_s = 0.2;
  if (pose_queue_stats_hz < 0.0)
    pose_queue_stats_hz = 0.0;

  Publishers publishers;
  publishers.summary = nh.advertise<std_msgs::String>(summary_topic, 20);
  publishers.pose_info = nh.advertise<kuavo_msgs::xsensePoseInfoList>(pose_topic, 20);
  if (pose_queue_stats_hz > 0.0 && !pose_queue_stats_topic.empty())
    publishers.pose_queue_stats = nh.advertise<std_msgs::String>(pose_queue_stats_topic, 5);

  PicoPauseControlState pause_state;
  pause_state.enabled = enable_pico_pause_control;
  PosePublishQueue pose_queue;
  pose_queue.prefill_ready = !pose_queue_prefill;
  const std::size_t queue_target_depth = static_cast<std::size_t>(pose_queue_target_depth);
  const std::size_t queue_max_size = static_cast<std::size_t>(pose_queue_max_size);
  const ros::WallDuration pose_stale_timeout(pose_stale_timeout_s);
  const std::vector<std::string> pause_combo_tokens = parseComboTokens(pico_pause_combo);
  const std::vector<std::string> resume_combo_tokens = parseComboTokens(pico_resume_combo);
  const std::string pause_combo_desc = comboTokensToString(pause_combo_tokens);
  const std::string resume_combo_desc = comboTokensToString(resume_combo_tokens);

  if (pause_state.enabled && (pause_combo_tokens.empty() || resume_combo_tokens.empty()))
  {
    ROS_WARN_STREAM("Pico pause control is disabled because pause/resume combo is empty. pause='"
                    << pico_pause_combo << "', resume='" << pico_resume_combo << "'");
    pause_state.enabled = false;
  }

  ros::Subscriber pico_joy_subscriber;
  if (pause_state.enabled)
  {
    pico_joy_subscriber = nh.subscribe<kuavo_msgs::JoySticks>(
        pico_joy_topic, 20,
        [&](const kuavo_msgs::JoySticks::ConstPtr& joy_msg) {
          if (!joy_msg)
            return;

          const bool pause_combo_active = comboActive(*joy_msg,
                                                      pause_combo_tokens,
                                                      pico_trigger_threshold,
                                                      pico_grip_threshold);
          const bool resume_combo_active = comboActive(*joy_msg,
                                                       resume_combo_tokens,
                                                       pico_trigger_threshold,
                                                       pico_grip_threshold);

          bool paused_now = false;
          bool resumed_now = false;
          bool ignored_pause_without_pose = false;
          bool have_pose_for_pause = false;

          {
            std::lock_guard<std::mutex> lock(pose_queue.mutex);
            have_pose_for_pause = pose_queue.have_last_published;
          }

          {
            std::lock_guard<std::mutex> lock(pause_state.mutex);
            const bool pause_edge = pause_combo_active && !pause_state.last_pause_combo_active;
            const bool resume_edge = resume_combo_active && !pause_state.last_resume_combo_active;

            pause_state.last_pause_combo_active = pause_combo_active;
            pause_state.last_resume_combo_active = resume_combo_active;

            if (resume_edge)
            {
              if (pause_state.paused)
              {
                pause_state.paused = false;
                resumed_now = true;
              }
            }
            else if (pause_edge)
            {
              if (have_pose_for_pause)
              {
                if (!pause_state.paused)
                {
                  pause_state.paused = true;
                  paused_now = true;
                }
              }
              else
              {
                ignored_pause_without_pose = true;
              }
            }
          }

          if (paused_now)
            ROS_INFO_STREAM("Paused Xsens UDP pose forwarding by Pico combo " << pause_combo_desc);
          if (resumed_now)
          {
            {
              std::lock_guard<std::mutex> lock(pose_queue.mutex);
              // 恢复时丢弃暂停期间可能积压的旧动作，重新等待恢复后的新 UDP 姿态填充队列。
              const ros::WallTime now = ros::WallTime::now();
              accountPoseQueueDepthLocked(pose_queue, now);
              pose_queue.queue.clear();
              observePoseQueueDepthLocked(pose_queue);
              pose_queue.prefill_ready = !pose_queue_prefill;
              pose_queue.have_input = true;
              pose_queue.last_input_time = now;
            }
            ROS_INFO_STREAM("Resumed Xsens UDP pose forwarding by Pico combo " << resume_combo_desc);
          }
          if (ignored_pause_without_pose)
            ROS_WARN_STREAM("Ignoring Pico pause combo " << pause_combo_desc
                                                         << ": no valid Xsens pose frame has been received yet");
        });
  }

  ros::Timer pose_publish_timer = nh.createTimer(
      ros::Duration(1.0 / pose_publish_hz),
      [&](const ros::TimerEvent&) {
        kuavo_msgs::xsensePoseInfoList publish_msg;
        bool should_publish = false;
        bool warn_stale = false;
        const ros::WallTime now = ros::WallTime::now();

        bool paused = false;
        {
          std::lock_guard<std::mutex> lock(pause_state.mutex);
          paused = pause_state.enabled && pause_state.paused;
        }

        {
          std::lock_guard<std::mutex> lock(pose_queue.mutex);
          // 入队和出队不是固定频率，队列平均深度按真实时间加权统计。
          accountPoseQueueDepthLocked(pose_queue, now);

          if (paused)
          {
            if (pose_queue.have_last_published)
            {
              publish_msg = pose_queue.last_published_msg;
              should_publish = true;
              ++pose_queue.paused_hold_publish_count;
              ++pose_queue.stats_window_paused_hold_publish_count;
            }
          }
          else
          {
            if (pose_queue_prefill && !pose_queue.prefill_ready)
            {
              if (pose_queue.queue.size() >= queue_target_depth)
              {
                pose_queue.prefill_ready = true;
              }
              else if (pose_hold_on_empty && pose_queue.have_last_published &&
                       pose_queue.have_input &&
                       (now - pose_queue.last_input_time) <= pose_stale_timeout)
              {
                // 队列重新预填充期间，短时间复用上一帧，相当于人体姿态短暂冻结。
                publish_msg = pose_queue.last_published_msg;
                should_publish = true;
                ++pose_queue.hold_publish_count;
                ++pose_queue.stats_window_hold_publish_count;
              }
            }

            if (pose_queue.prefill_ready)
            {
              if (!pose_queue.queue.empty())
              {
                publish_msg = pose_queue.queue.front();
                pose_queue.queue.pop_front();
                observePoseQueueDepthLocked(pose_queue);
                should_publish = true;
              }
              else
              {
                ++pose_queue.underflow_count;
                ++pose_queue.stats_window_underflow_count;
                pose_queue.prefill_ready = !pose_queue_prefill;
                if (pose_hold_on_empty && pose_queue.have_last_published &&
                    pose_queue.have_input &&
                    (now - pose_queue.last_input_time) <= pose_stale_timeout)
                {
                  // FIFO 被短暂耗空时复用上一帧，避免 ROS 话题频率瞬间掉到 0。
                  publish_msg = pose_queue.last_published_msg;
                  should_publish = true;
                  ++pose_queue.hold_publish_count;
                  ++pose_queue.stats_window_hold_publish_count;
                }
                else
                {
                  warn_stale = true;
                }
              }
            }
          }
          observePoseQueueDepthLocked(pose_queue);
        }

        if (!should_publish)
        {
          if (warn_stale)
          {
            ROS_WARN_THROTTLE(1.0,
                              "Xsens pose FIFO is empty and no fresh UDP pose frame is available");
          }
          return;
        }

        if (update_output_header_stamp)
          publish_msg.header.stamp = ros::Time::now();
        {
          std::lock_guard<std::mutex> lock(pose_queue.mutex);
          pose_queue.last_published_msg = publish_msg;
          pose_queue.have_last_published = true;
          ++pose_queue.published_count;
          ++pose_queue.stats_window_published_count;
        }
        publishers.pose_info.publish(publish_msg);
      });

  ros::Timer pose_queue_stats_timer;
  if (pose_queue_stats_hz > 0.0 && !pose_queue_stats_topic.empty())
  {
    pose_queue_stats_timer = nh.createTimer(
        ros::Duration(1.0 / pose_queue_stats_hz),
        [&](const ros::TimerEvent&) {
          const ros::WallTime now = ros::WallTime::now();
          bool paused = false;
          {
            std::lock_guard<std::mutex> lock(pause_state.mutex);
            paused = pause_state.enabled && pause_state.paused;
          }

          std_msgs::String msg;
          {
            std::lock_guard<std::mutex> lock(pose_queue.mutex);
            msg.data = poseQueueStatsJsonLocked(pose_queue,
                                                now,
                                                pose_publish_hz,
                                                queue_target_depth,
                                                queue_max_size,
                                                pose_queue_prefill,
                                                pose_hold_on_empty,
                                                paused);
            resetPoseQueueStatsWindowLocked(pose_queue);
          }

          if (publishers.pose_queue_stats)
            publishers.pose_queue_stats.publish(msg);
        });
  }

  kuavo_xsense_gmr::UdpReceiver receiver;
  std::string error;
  if (!receiver.open(listen_ip, static_cast<uint16_t>(listen_port), &error))
  {
    ROS_FATAL_STREAM("Failed to open UDP receiver: " << error);
    return 1;
  }

  ROS_INFO_STREAM("Listening for Xsens UDP stream on " << listen_ip << ":" << listen_port);
  ROS_INFO_STREAM("Publishing Xsens poses to " << pose_topic);
  ROS_INFO_STREAM("Xsens pose FIFO publishing enabled: pose_publish_hz=" << pose_publish_hz
                                                                         << ", target_depth="
                                                                         << queue_target_depth
                                                                         << ", max_size="
                                                                         << queue_max_size
                                                                         << ", prefill="
                                                                         << (pose_queue_prefill ? "true" : "false")
                                                                         << ", hold_on_empty="
                                                                         << (pose_hold_on_empty ? "true" : "false"));
  if (pose_queue_stats_hz > 0.0 && !pose_queue_stats_topic.empty())
  {
    ROS_INFO_STREAM("Xsens pose FIFO stats enabled: topic="
                    << pose_queue_stats_topic
                    << ", hz=" << pose_queue_stats_hz);
  }
  ROS_INFO_STREAM("Publishing Xsens packet summary to " << summary_topic);
  if (pause_state.enabled)
  {
    ROS_INFO_STREAM("Pico pause control enabled: topic=" << pico_joy_topic
                                                         << ", pause="
                                                         << comboTokensToString(pause_combo_tokens)
                                                         << ", resume="
                                                         << comboTokensToString(resume_combo_tokens)
                                                         << ", pose_publish_hz=" << pose_publish_hz);
  }
  else
  {
    ROS_INFO_STREAM("Pico pause control disabled");
  }

  kuavo_xsense_gmr::XsensPacketParser parser;
  kuavo_xsense_gmr::FragmentAssembler assembler(fragment_timeout_s);
  std::map<std::string, uint32_t> last_sample_by_type;
  ros::WallTime last_summary_publish = ros::WallTime(0);
  const ros::WallDuration min_summary_period(1.0 / summary_rate_limit_hz);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  while (ros::ok())
  {
    std::vector<uint8_t> datagram;
    error.clear();
    if (!receiver.receive(&datagram, receive_timeout_ms, &error))
    {
      if (!error.empty())
        ROS_WARN_STREAM_THROTTLE(2.0, error);
      continue;
    }

    std::vector<uint8_t> complete_datagram;
    if (!assembler.addDatagram(datagram, &complete_datagram, &error))
    {
      if (!error.empty())
        ROS_WARN_STREAM_THROTTLE(2.0, error);
      continue;
    }

    kuavo_xsense_gmr::ParsedPacket packet = parser.parsePacket(complete_datagram);
    packet.reconstructed = isReconstructedPacket(datagram, parser);

    if (packet.valid && expected_character_id >= 0 &&
        static_cast<int>(packet.header.character_id) != expected_character_id)
    {
      packet.valid = false;
      packet.header.valid = false;
      packet.invalid_reason = "character_id does not match expected_character_id";
      packet.header.invalid_reason = packet.invalid_reason;
    }

    int64_t sample_delta = -1;
    if (!packet.header.message_type.empty())
    {
      const auto it = last_sample_by_type.find(packet.header.message_type);
      if (it != last_sample_by_type.end())
      {
        sample_delta = static_cast<int64_t>(packet.header.sample_counter) -
                       static_cast<int64_t>(it->second);
      }
      last_sample_by_type[packet.header.message_type] = packet.header.sample_counter;
    }

    kuavo_msgs::xsensePoseInfoList pose_msg;
    if (buildPoseInfoList(packet, frame_id, &pose_msg))
    {
      bool paused = false;
      {
        std::lock_guard<std::mutex> lock(pause_state.mutex);
        paused = pause_state.enabled && pause_state.paused;
      }

      // 暂停期间继续收 UDP，但不把暂停期间的新动作放入对外发布队列。
      if (!paused)
      {
        std::lock_guard<std::mutex> lock(pose_queue.mutex);
        const ros::WallTime now = ros::WallTime::now();
        accountPoseQueueDepthLocked(pose_queue, now);
        // UDP 到达可能是 burst，这里只负责解析和入队，不直接发布 ROS 话题。
        while (pose_queue.queue.size() >= queue_max_size)
        {
          pose_queue.queue.pop_front();
          ++pose_queue.overflow_drop_count;
          ++pose_queue.stats_window_overflow_drop_count;
          ROS_WARN_STREAM_THROTTLE(1.0, "Xsens pose FIFO overflow, dropping oldest frame. dropped="
                                            << pose_queue.overflow_drop_count);
        }
        pose_queue.queue.push_back(pose_msg);
        observePoseQueueDepthLocked(pose_queue);
        pose_queue.have_input = true;
        pose_queue.last_input_time = now;
        ++pose_queue.enqueued_count;
        ++pose_queue.stats_window_enqueued_count;
      }
    }

    const ros::WallTime now = ros::WallTime::now();
    if ((now - last_summary_publish) >= min_summary_period || !packet.valid)
    {
      std_msgs::String msg;
      msg.data = packetSummaryJson(packet, sample_delta);
      publishers.summary.publish(msg);
      last_summary_publish = now;
    }
  }

  spinner.stop();
  receiver.close();
  return 0;
}
