#include "kuavo_xsense_gmr/xsens_udp/xsens_pose_builder.h"

#include <cmath>
#include <map>
#include <sstream>
#include <vector>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <kuavo_msgs/xsensePoseInfo.h>
#include <ros/time.h>
#include <tf/transform_datatypes.h>

namespace kuavo_xsense_gmr
{
namespace
{

enum class PositionSource
{
  kSegmentOrigin,
  kLeftLateralEpicondyle,
  kRightLateralEpicondyle
};

struct BodyMapping
{
  const char* output_name;
  const char* xsens_segment_name;
  PositionSource position_source;
  const char* position_segment_name;
};

const std::vector<BodyMapping>& bodyMappings()
{
  static const std::vector<BodyMapping> mappings = {
      {"Hips", "Pelvis", PositionSource::kSegmentOrigin, nullptr},
      {"Chest", "L5", PositionSource::kSegmentOrigin, nullptr},
      {"Neck", "Neck", PositionSource::kSegmentOrigin, nullptr},
      {"Head", "Head", PositionSource::kSegmentOrigin, nullptr},
      {"LeftCollar", "Left Shoulder", PositionSource::kSegmentOrigin, nullptr},
      {"LeftShoulder", "Left Upper Arm", PositionSource::kSegmentOrigin, nullptr},
      {"LeftElbow", "Left Forearm", PositionSource::kLeftLateralEpicondyle, "Left Upper Arm"},
      {"LeftWrist", "Left Hand", PositionSource::kSegmentOrigin, nullptr},
      {"RightCollar", "Right Shoulder", PositionSource::kSegmentOrigin, nullptr},
      {"RightShoulder", "Right Upper Arm", PositionSource::kSegmentOrigin, nullptr},
      {"RightElbow", "Right Forearm", PositionSource::kRightLateralEpicondyle, "Right Upper Arm"},
      {"RightWrist", "Right Hand", PositionSource::kSegmentOrigin, nullptr},
      {"LeftHip", "Left Upper Leg", PositionSource::kSegmentOrigin, nullptr},
      {"LeftKnee", "Left Lower Leg", PositionSource::kSegmentOrigin, nullptr},
      {"LeftAnkle", "Left Foot", PositionSource::kSegmentOrigin, nullptr},
      {"LeftToe", "Left Toe", PositionSource::kSegmentOrigin, nullptr},
      {"RightHip", "Right Upper Leg", PositionSource::kSegmentOrigin, nullptr},
      {"RightKnee", "Right Lower Leg", PositionSource::kSegmentOrigin, nullptr},
      {"RightAnkle", "Right Foot", PositionSource::kSegmentOrigin, nullptr},
      {"RightToe", "Right Toe", PositionSource::kSegmentOrigin, nullptr},
  };
  return mappings;
}

void setError(std::string* error, const std::string& value)
{
  if (error != nullptr)
    *error = value;
}

geometry_msgs::Point toPoint(const Vector3f& value)
{
  geometry_msgs::Point point;
  point.x = value.x;
  point.y = value.y;
  point.z = value.z;
  return point;
}

geometry_msgs::Quaternion toQuaternion(const Quaternionf& value)
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
      "Right Lower Leg", "Right Foot",     "Right Toe",    "Left Upper Leg",
      "Left Lower Leg",  "Left Foot",      "Left Toe"};
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

bool isFinite(const Vector3f& value)
{
  return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

bool isFinite(const Quaternionf& value)
{
  return std::isfinite(value.w) && std::isfinite(value.x) &&
         std::isfinite(value.y) && std::isfinite(value.z);
}

bool calculateLandmarkWorldPosition(const XsensSegmentPoseQuaternion& segment,
                                    const Vector3f& local_offset,
                                    Vector3f* world_position,
                                    std::string* error)
{
  if (world_position == nullptr)
  {
    setError(error, "landmark world position output is null");
    return false;
  }
  if (!isFinite(segment.position_m) || !isFinite(segment.orientation) ||
      !isFinite(local_offset))
  {
    setError(error, "landmark input contains NaN or Inf");
    return false;
  }

  tf::Quaternion rotation(segment.orientation.x,
                          segment.orientation.y,
                          segment.orientation.z,
                          segment.orientation.w);
  if (rotation.length2() <= 1.0e-12)
  {
    setError(error, "landmark segment quaternion has zero length");
    return false;
  }
  rotation.normalize();

  const tf::Vector3 rotated =
      tf::quatRotate(rotation, tf::Vector3(local_offset.x, local_offset.y, local_offset.z));
  world_position->x = segment.position_m.x + static_cast<float>(rotated.x());
  world_position->y = segment.position_m.y + static_cast<float>(rotated.y());
  world_position->z = segment.position_m.z + static_cast<float>(rotated.z());

  if (!isFinite(*world_position))
  {
    setError(error, "calculated landmark position contains NaN or Inf");
    return false;
  }
  return true;
}

}  // namespace

bool buildXsensPoseInfoList(const ParsedPacket& packet,
                            const ElbowLandmarkOffsets& elbow_offsets,
                            const std::string& frame_id,
                            kuavo_msgs::xsensePoseInfoList* msg,
                            std::string* error)
{
  if (error != nullptr)
    error->clear();
  if (msg == nullptr)
  {
    setError(error, "pose message output is null");
    return false;
  }
  if (!packet.valid || packet.header.message_type != "02")
  {
    setError(error, "packet is not a valid Xsens quaternion pose");
    return false;
  }

  std::map<std::string, const XsensSegmentPoseQuaternion*> segment_by_name;
  for (const XsensSegmentPoseQuaternion& segment : packet.pose_quaternion)
  {
    const std::string name = segmentNameFromId(segment.segment_id);
    if (!name.empty())
      segment_by_name[name] = &segment;
  }

  kuavo_msgs::xsensePoseInfoList built;
  built.header.stamp = ros::Time::now();
  built.header.frame_id = frame_id;
  built.poses.reserve(bodyMappings().size());

  for (const BodyMapping& mapping : bodyMappings())
  {
    const auto source_it = segment_by_name.find(mapping.xsens_segment_name);
    if (source_it == segment_by_name.end())
    {
      setError(error, std::string("missing Xsens segment ") + mapping.xsens_segment_name);
      return false;
    }

    const XsensSegmentPoseQuaternion& source = *source_it->second;
    if (!isFinite(source.position_m) || !isFinite(source.orientation))
    {
      setError(error, std::string("Xsens segment contains NaN or Inf: ") +
                          mapping.xsens_segment_name);
      return false;
    }
    Vector3f output_position = source.position_m;

    if (mapping.position_source != PositionSource::kSegmentOrigin)
    {
      const auto position_segment_it = segment_by_name.find(mapping.position_segment_name);
      if (position_segment_it == segment_by_name.end())
      {
        setError(error, std::string("missing Xsens position segment ") +
                            mapping.position_segment_name);
        return false;
      }

      const Vector3f& offset =
          mapping.position_source == PositionSource::kLeftLateralEpicondyle
              ? elbow_offsets.left_lateral_epicondyle_m
              : elbow_offsets.right_lateral_epicondyle_m;
      if (!calculateLandmarkWorldPosition(*position_segment_it->second,
                                          offset,
                                          &output_position,
                                          error))
      {
        return false;
      }
    }

    kuavo_msgs::xsensePoseInfo pose;
    pose.name = mapping.output_name;
    pose.segment_id = source.segment_id;
    pose.position = toPoint(output_position);
    pose.orientation = toQuaternion(source.orientation);
    built.poses.push_back(pose);
  }

  *msg = built;
  return true;
}

}  // namespace kuavo_xsense_gmr
