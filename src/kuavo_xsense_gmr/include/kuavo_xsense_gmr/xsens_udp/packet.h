#ifndef KUAVO_XSENSE_GMR_XSENS_UDP_PACKET_H
#define KUAVO_XSENSE_GMR_XSENS_UDP_PACKET_H

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace kuavo_xsense_gmr
{

struct XsensPacketHeader
{
  static constexpr std::size_t kHeaderSize = 24;

  std::string id_string;
  std::string message_type;
  uint32_t sample_counter = 0;
  uint8_t datagram_counter = 0;
  uint8_t number_of_items = 0;
  uint32_t time_code = 0;
  uint8_t character_id = 0;
  uint8_t body_segments = 0;
  uint8_t props = 0;
  uint8_t finger_tracking_segments = 0;
  uint16_t reserved = 0;
  uint16_t payload_size = 0;
  std::size_t datagram_size = 0;

  bool valid = false;
  std::string invalid_reason;

  uint8_t fragment_index() const
  {
    return static_cast<uint8_t>(datagram_counter & 0x7F);
  }

  bool is_last_fragment() const
  {
    return (datagram_counter & 0x80) != 0;
  }
};

struct Vector3f
{
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
};

struct Quaternionf
{
  float w = 1.0F;
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
};

struct XsensSegmentPoseEuler
{
  uint32_t segment_id = 0;
  Vector3f position_m;
  Vector3f euler_deg;
};

struct XsensSegmentPoseQuaternion
{
  uint32_t segment_id = 0;
  Vector3f position_m;
  Quaternionf orientation;
};

struct XsensPointPosition
{
  uint32_t point_id = 0;
  Vector3f position_m;
};

struct XsensMetadataTag
{
  std::string key;
  std::string value;
};

struct XsensScaleSegment
{
  std::string name;
  Vector3f origin_m;
};

struct XsensScalePoint
{
  uint16_t segment_id = 0;
  uint16_t point_id = 0;
  std::string name;
  uint32_t flags = 0;
  Vector3f position_m;
};

struct XsensJointAngle
{
  uint32_t parent_point_id = 0;
  uint32_t child_point_id = 0;
  Vector3f rotation;
};

struct XsensLinearSegmentKinematics
{
  uint32_t segment_id = 0;
  Vector3f position_m;
  Vector3f velocity;
  Vector3f acceleration;
};

struct XsensAngularSegmentKinematics
{
  uint32_t segment_id = 0;
  Quaternionf orientation;
  Vector3f angular_velocity;
  Vector3f angular_acceleration;
};

struct XsensMotionTrackerKinematics
{
  uint32_t segment_id = 0;
  Quaternionf orientation;
  Vector3f free_acceleration;
  Vector3f magnetic_field;
};

struct ParsedPacket
{
  XsensPacketHeader header;
  bool valid = false;
  bool reconstructed = false;
  std::string invalid_reason;
  std::string coordinate_system;

  std::vector<uint8_t> payload;
  std::vector<XsensSegmentPoseEuler> pose_euler;
  std::vector<XsensSegmentPoseQuaternion> pose_quaternion;
  std::vector<XsensPointPosition> point_positions;
  std::vector<XsensSegmentPoseQuaternion> unity_pose;
  std::vector<XsensMetadataTag> metadata_tags;
  std::vector<XsensScaleSegment> scale_segments;
  std::vector<XsensScalePoint> scale_points;
  std::vector<XsensJointAngle> joint_angles;
  std::vector<XsensLinearSegmentKinematics> linear_kinematics;
  std::vector<XsensAngularSegmentKinematics> angular_kinematics;
  std::vector<XsensMotionTrackerKinematics> tracker_kinematics;
  Vector3f center_of_mass_m;
  std::string time_code;
  bool raw_payload_only = false;
};

}  // namespace kuavo_xsense_gmr

#endif  // KUAVO_XSENSE_GMR_XSENS_UDP_PACKET_H
