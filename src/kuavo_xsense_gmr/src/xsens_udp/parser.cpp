#include "kuavo_xsense_gmr/xsens_udp/parser.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <sstream>

namespace kuavo_xsense_gmr
{
namespace
{

class ByteReader
{
public:
  ByteReader(const std::vector<uint8_t>& data, std::size_t offset)
    : data_(data), offset_(offset)
  {
  }

  std::size_t remaining() const
  {
    return data_.size() - offset_;
  }

  bool readU16(uint16_t* value)
  {
    if (remaining() < 2)
      return false;
    *value = XsensPacketParser::readU16BE(&data_[offset_]);
    offset_ += 2;
    return true;
  }

  bool readI32(int32_t* value)
  {
    if (remaining() < 4)
      return false;
    *value = XsensPacketParser::readI32BE(&data_[offset_]);
    offset_ += 4;
    return true;
  }

  bool readU32(uint32_t* value)
  {
    if (remaining() < 4)
      return false;
    *value = XsensPacketParser::readU32BE(&data_[offset_]);
    offset_ += 4;
    return true;
  }

  bool readF32(float* value)
  {
    if (remaining() < 4)
      return false;
    *value = XsensPacketParser::readF32BE(&data_[offset_]);
    offset_ += 4;
    return true;
  }

  bool readString(std::string* value, std::string* error)
  {
    int32_t length = 0;
    if (!readI32(&length))
    {
      *error = "string length exceeds payload";
      return false;
    }
    if (length < 0)
    {
      *error = "string length is negative";
      return false;
    }
    if (remaining() < static_cast<std::size_t>(length))
    {
      *error = "string bytes exceed payload";
      return false;
    }
    value->assign(reinterpret_cast<const char*>(&data_[offset_]),
                  static_cast<std::size_t>(length));
    offset_ += static_cast<std::size_t>(length);
    return true;
  }

private:
  const std::vector<uint8_t>& data_;
  std::size_t offset_;
};

void setInvalid(ParsedPacket* packet, const std::string& reason)
{
  packet->valid = false;
  packet->invalid_reason = reason;
  packet->header.valid = false;
  packet->header.invalid_reason = reason;
}

std::string coordinateSystemFor(const std::string& type)
{
  if (type == "01")
    return "Y-Up right-handed, positions in m, Euler rotations in degrees";
  if (type == "02")
    return "BVH-aligned frame: X-left, Y-up, Z-forward, converted from Xsens type02 raw Z-up, positions in m";
  if (type == "03")
    return "Y-Up right-handed, positions in m";
  if (type == "05")
    return "Y-Up left-handed Unity3D protocol, positions in m";
  if (type == "13")
    return "Y-Up right-handed, converted from Xsens Z-Up, positions in m";
  if (type == "20" || type == "21" || type == "22" || type == "23" || type == "24")
    return "Y-Up right-handed, converted from Xsens Z-Up, protocol units";
  return "";
}

Vector3f zUpToYUp(const Vector3f& value)
{
  Vector3f converted;
  converted.x = value.x;
  converted.y = value.z;
  converted.z = -value.y;
  return converted;
}

Quaternionf multiply(const Quaternionf& lhs, const Quaternionf& rhs)
{
  Quaternionf result;
  result.w = lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z;
  result.x = lhs.w * rhs.x + lhs.x * rhs.w + lhs.y * rhs.z - lhs.z * rhs.y;
  result.y = lhs.w * rhs.y - lhs.x * rhs.z + lhs.y * rhs.w + lhs.z * rhs.x;
  result.z = lhs.w * rhs.z + lhs.x * rhs.y - lhs.y * rhs.x + lhs.z * rhs.w;
  return result;
}

Quaternionf conjugate(const Quaternionf& value)
{
  Quaternionf result;
  result.w = value.w;
  result.x = -value.x;
  result.y = -value.y;
  result.z = -value.z;
  return result;
}

Quaternionf normalize(const Quaternionf& value)
{
  const float norm =
      std::sqrt(value.w * value.w + value.x * value.x + value.y * value.y + value.z * value.z);
  if (norm <= 0.0F)
    return value;

  Quaternionf result;
  result.w = value.w / norm;
  result.x = value.x / norm;
  result.y = value.y / norm;
  result.z = value.z / norm;
  return result;
}

Quaternionf zUpToYUp(const Quaternionf& value)
{
  constexpr float kHalfSqrt2 = 0.7071067811865476F;
  const Quaternionf basis_rotation{kHalfSqrt2, -kHalfSqrt2, 0.0F, 0.0F};
  return normalize(multiply(multiply(basis_rotation, value), conjugate(basis_rotation)));
}

struct Matrix3f
{
  float m[3][3] = {{0.0F, 0.0F, 0.0F},
                   {0.0F, 0.0F, 0.0F},
                   {0.0F, 0.0F, 0.0F}};
};

Matrix3f quaternionToMatrix(const Quaternionf& value)
{
  const Quaternionf q = normalize(value);
  const float ww = q.w * q.w;
  const float xx = q.x * q.x;
  const float yy = q.y * q.y;
  const float zz = q.z * q.z;
  const float wx = q.w * q.x;
  const float wy = q.w * q.y;
  const float wz = q.w * q.z;
  const float xy = q.x * q.y;
  const float xz = q.x * q.z;
  const float yz = q.y * q.z;

  Matrix3f result;
  result.m[0][0] = ww + xx - yy - zz;
  result.m[0][1] = 2.0F * (xy - wz);
  result.m[0][2] = 2.0F * (xz + wy);
  result.m[1][0] = 2.0F * (xy + wz);
  result.m[1][1] = ww - xx + yy - zz;
  result.m[1][2] = 2.0F * (yz - wx);
  result.m[2][0] = 2.0F * (xz - wy);
  result.m[2][1] = 2.0F * (yz + wx);
  result.m[2][2] = ww - xx - yy + zz;
  return result;
}

Matrix3f multiplyMatrix(const Matrix3f& lhs, const Matrix3f& rhs)
{
  Matrix3f result;
  for (int row = 0; row < 3; ++row)
  {
    for (int col = 0; col < 3; ++col)
    {
      for (int k = 0; k < 3; ++k)
        result.m[row][col] += lhs.m[row][k] * rhs.m[k][col];
    }
  }
  return result;
}

Matrix3f transposeMatrix(const Matrix3f& value)
{
  Matrix3f result;
  for (int row = 0; row < 3; ++row)
  {
    for (int col = 0; col < 3; ++col)
      result.m[row][col] = value.m[col][row];
  }
  return result;
}

Quaternionf matrixToQuaternion(const Matrix3f& value)
{
  Quaternionf result;
  const float trace = value.m[0][0] + value.m[1][1] + value.m[2][2];

  if (trace > 0.0F)
  {
    const float s = 2.0F * std::sqrt(trace + 1.0F);
    result.w = 0.25F * s;
    result.x = (value.m[2][1] - value.m[1][2]) / s;
    result.y = (value.m[0][2] - value.m[2][0]) / s;
    result.z = (value.m[1][0] - value.m[0][1]) / s;
  }
  else if (value.m[0][0] > value.m[1][1] && value.m[0][0] > value.m[2][2])
  {
    const float s = 2.0F * std::sqrt(1.0F + value.m[0][0] - value.m[1][1] - value.m[2][2]);
    result.w = (value.m[2][1] - value.m[1][2]) / s;
    result.x = 0.25F * s;
    result.y = (value.m[0][1] + value.m[1][0]) / s;
    result.z = (value.m[0][2] + value.m[2][0]) / s;
  }
  else if (value.m[1][1] > value.m[2][2])
  {
    const float s = 2.0F * std::sqrt(1.0F + value.m[1][1] - value.m[0][0] - value.m[2][2]);
    result.w = (value.m[0][2] - value.m[2][0]) / s;
    result.x = (value.m[0][1] + value.m[1][0]) / s;
    result.y = 0.25F * s;
    result.z = (value.m[1][2] + value.m[2][1]) / s;
  }
  else
  {
    const float s = 2.0F * std::sqrt(1.0F + value.m[2][2] - value.m[0][0] - value.m[1][1]);
    result.w = (value.m[1][0] - value.m[0][1]) / s;
    result.x = (value.m[0][2] + value.m[2][0]) / s;
    result.y = (value.m[1][2] + value.m[2][1]) / s;
    result.z = 0.25F * s;
  }

  return normalize(result);
}

const Matrix3f& xsensType02RawToBvhFrameMatrix()
{
  static const Matrix3f basis{{{0.0F, 1.0F, 0.0F},
                               {0.0F, 0.0F, 1.0F},
                               {1.0F, 0.0F, 0.0F}}};
  return basis;
}

Vector3f xsensType02RawToBvhFrame(const Vector3f& value)
{
  Vector3f converted;
  converted.x = value.y;
  converted.y = value.z;
  converted.z = value.x;
  return converted;
}

Quaternionf xsensType02RawToBvhFrame(const Quaternionf& value)
{
  const Matrix3f& basis = xsensType02RawToBvhFrameMatrix();
  const Matrix3f raw_rotation = quaternionToMatrix(value);
  const Matrix3f converted_rotation =
      multiplyMatrix(multiplyMatrix(basis, raw_rotation), transposeMatrix(basis));
  return matrixToQuaternion(converted_rotation);
}

void convertCoordinateFrames(ParsedPacket* packet)
{
  const std::string& type = packet->header.message_type;

  if (type == "02")
  {
    for (auto& segment : packet->pose_quaternion)
    {
      segment.position_m = xsensType02RawToBvhFrame(segment.position_m);
      segment.orientation = xsensType02RawToBvhFrame(segment.orientation);
    }
  }
  else if (type == "13")
  {
    for (auto& segment : packet->scale_segments)
      segment.origin_m = zUpToYUp(segment.origin_m);
    for (auto& point : packet->scale_points)
      point.position_m = zUpToYUp(point.position_m);
  }
  else if (type == "20")
  {
    for (auto& joint : packet->joint_angles)
      joint.rotation = zUpToYUp(joint.rotation);
  }
  else if (type == "21")
  {
    for (auto& segment : packet->linear_kinematics)
    {
      segment.position_m = zUpToYUp(segment.position_m);
      segment.velocity = zUpToYUp(segment.velocity);
      segment.acceleration = zUpToYUp(segment.acceleration);
    }
  }
  else if (type == "22")
  {
    for (auto& segment : packet->angular_kinematics)
    {
      segment.orientation = zUpToYUp(segment.orientation);
      segment.angular_velocity = zUpToYUp(segment.angular_velocity);
      segment.angular_acceleration = zUpToYUp(segment.angular_acceleration);
    }
  }
  else if (type == "23")
  {
    for (auto& segment : packet->tracker_kinematics)
    {
      segment.orientation = zUpToYUp(segment.orientation);
      segment.free_acceleration = zUpToYUp(segment.free_acceleration);
      segment.magnetic_field = zUpToYUp(segment.magnetic_field);
    }
  }
  else if (type == "24")
  {
    packet->center_of_mass_m = zUpToYUp(packet->center_of_mass_m);
  }
}

bool validateFixedPayload(const XsensPacketHeader& header,
                          std::size_t record_size,
                          const std::string& label,
                          std::size_t* count,
                          std::string* error)
{
  if (record_size == 0 || (header.payload_size % record_size) != 0)
  {
    std::ostringstream out;
    out << label << " payload size is not a multiple of " << record_size;
    *error = out.str();
    return false;
  }

  *count = header.payload_size / record_size;
  if (header.number_of_items != *count)
  {
    std::ostringstream out;
    out << label << " item count mismatch: header="
        << static_cast<unsigned int>(header.number_of_items)
        << " parsed=" << *count;
    *error = out.str();
    return false;
  }
  return true;
}

bool readVector3(ByteReader* reader, Vector3f* value)
{
  return reader->readF32(&value->x) && reader->readF32(&value->y) &&
         reader->readF32(&value->z);
}

bool readQuaternion(ByteReader* reader, Quaternionf* value)
{
  return reader->readF32(&value->w) && reader->readF32(&value->x) &&
         reader->readF32(&value->y) && reader->readF32(&value->z);
}

bool parsePoseEuler(const XsensPacketHeader& header,
                    ByteReader* reader,
                    ParsedPacket* packet,
                    std::string* error)
{
  std::size_t count = 0;
  if (!validateFixedPayload(header, 28, "pose Euler", &count, error))
    return false;

  packet->pose_euler.reserve(count);
  for (std::size_t i = 0; i < count; ++i)
  {
    XsensSegmentPoseEuler item;
    if (!reader->readU32(&item.segment_id) || !readVector3(reader, &item.position_m) ||
        !readVector3(reader, &item.euler_deg))
    {
      *error = "pose Euler record exceeds payload";
      return false;
    }
    packet->pose_euler.push_back(item);
  }
  return true;
}

bool parsePoseQuaternion(const XsensPacketHeader& header,
                         ByteReader* reader,
                         std::vector<XsensSegmentPoseQuaternion>* out,
                         const std::string& label,
                         std::string* error)
{
  std::size_t count = 0;
  if (!validateFixedPayload(header, 32, label, &count, error))
    return false;

  out->reserve(count);
  for (std::size_t i = 0; i < count; ++i)
  {
    XsensSegmentPoseQuaternion item;
    if (!reader->readU32(&item.segment_id) || !readVector3(reader, &item.position_m) ||
        !readQuaternion(reader, &item.orientation))
    {
      *error = label + " record exceeds payload";
      return false;
    }
    out->push_back(item);
  }
  return true;
}

bool parsePointPositions(const XsensPacketHeader& header,
                         ByteReader* reader,
                         ParsedPacket* packet,
                         std::string* error)
{
  std::size_t count = 0;
  if (!validateFixedPayload(header, 16, "point positions", &count, error))
    return false;

  packet->point_positions.reserve(count);
  for (std::size_t i = 0; i < count; ++i)
  {
    XsensPointPosition item;
    if (!reader->readU32(&item.point_id) || !readVector3(reader, &item.position_m))
    {
      *error = "point position record exceeds payload";
      return false;
    }
    packet->point_positions.push_back(item);
  }
  return true;
}

bool parseMetadata(const std::vector<uint8_t>& payload, ParsedPacket* packet)
{
  const std::string text(payload.begin(), payload.end());
  std::size_t start = 0;
  while (start <= text.size())
  {
    const std::size_t end = text.find('\n', start);
    std::string line = text.substr(start, end == std::string::npos ? std::string::npos :
                                                                    end - start);
    if (!line.empty() && line.back() == '\r')
      line.pop_back();

    const std::size_t colon = line.find(':');
    if (colon != std::string::npos)
    {
      XsensMetadataTag tag;
      tag.key = line.substr(0, colon);
      tag.value = line.substr(colon + 1);
      if (!tag.value.empty() && tag.value.front() == ' ')
        tag.value.erase(tag.value.begin());
      packet->metadata_tags.push_back(tag);
    }

    if (end == std::string::npos)
      break;
    start = end + 1;
  }
  return true;
}

bool parseScaleInfo(ByteReader* reader, ParsedPacket* packet, std::string* error)
{
  uint32_t segment_count = 0;
  if (!reader->readU32(&segment_count))
  {
    *error = "scale segment count exceeds payload";
    return false;
  }

  packet->scale_segments.reserve(segment_count);
  for (uint32_t i = 0; i < segment_count; ++i)
  {
    XsensScaleSegment segment;
    if (!reader->readString(&segment.name, error) ||
        !readVector3(reader, &segment.origin_m))
    {
      if (error->empty())
        *error = "scale segment exceeds payload";
      return false;
    }
    packet->scale_segments.push_back(segment);
  }

  uint32_t point_count = 0;
  if (!reader->readU32(&point_count))
  {
    *error = "scale point count exceeds payload";
    return false;
  }

  packet->scale_points.reserve(point_count);
  for (uint32_t i = 0; i < point_count; ++i)
  {
    XsensScalePoint point;
    if (!reader->readU16(&point.segment_id) || !reader->readU16(&point.point_id) ||
        !reader->readString(&point.name, error) || !reader->readU32(&point.flags) ||
        !readVector3(reader, &point.position_m))
    {
      if (error->empty())
        *error = "scale point exceeds payload";
      return false;
    }
    packet->scale_points.push_back(point);
  }

  if (reader->remaining() != 0)
  {
    *error = "scale info payload has trailing bytes";
    return false;
  }
  return true;
}

bool parseJointAngles(const XsensPacketHeader& header,
                      ByteReader* reader,
                      ParsedPacket* packet,
                      std::string* error)
{
  std::size_t count = 0;
  if (!validateFixedPayload(header, 20, "joint angles", &count, error))
    return false;

  packet->joint_angles.reserve(count);
  for (std::size_t i = 0; i < count; ++i)
  {
    XsensJointAngle item;
    if (!reader->readU32(&item.parent_point_id) ||
        !reader->readU32(&item.child_point_id) ||
        !readVector3(reader, &item.rotation))
    {
      *error = "joint angle record exceeds payload";
      return false;
    }
    packet->joint_angles.push_back(item);
  }
  return true;
}

bool parseLinearKinematics(const XsensPacketHeader& header,
                           ByteReader* reader,
                           ParsedPacket* packet,
                           std::string* error)
{
  std::size_t count = 0;
  if (!validateFixedPayload(header, 40, "linear kinematics", &count, error))
    return false;

  packet->linear_kinematics.reserve(count);
  for (std::size_t i = 0; i < count; ++i)
  {
    XsensLinearSegmentKinematics item;
    if (!reader->readU32(&item.segment_id) || !readVector3(reader, &item.position_m) ||
        !readVector3(reader, &item.velocity) || !readVector3(reader, &item.acceleration))
    {
      *error = "linear kinematics record exceeds payload";
      return false;
    }
    packet->linear_kinematics.push_back(item);
  }
  return true;
}

bool parseAngularKinematics(const XsensPacketHeader& header,
                            ByteReader* reader,
                            ParsedPacket* packet,
                            std::string* error)
{
  std::size_t count = 0;
  if (!validateFixedPayload(header, 44, "angular kinematics", &count, error))
    return false;

  packet->angular_kinematics.reserve(count);
  for (std::size_t i = 0; i < count; ++i)
  {
    XsensAngularSegmentKinematics item;
    if (!reader->readU32(&item.segment_id) ||
        !readQuaternion(reader, &item.orientation) ||
        !readVector3(reader, &item.angular_velocity) ||
        !readVector3(reader, &item.angular_acceleration))
    {
      *error = "angular kinematics record exceeds payload";
      return false;
    }
    packet->angular_kinematics.push_back(item);
  }
  return true;
}

bool parseTrackerKinematics(const XsensPacketHeader& header,
                            ByteReader* reader,
                            ParsedPacket* packet,
                            std::string* error)
{
  std::size_t count = 0;
  if (!validateFixedPayload(header, 44, "motion tracker kinematics", &count, error))
    return false;

  packet->tracker_kinematics.reserve(count);
  for (std::size_t i = 0; i < count; ++i)
  {
    XsensMotionTrackerKinematics item;
    if (!reader->readU32(&item.segment_id) ||
        !readQuaternion(reader, &item.orientation) ||
        !readVector3(reader, &item.free_acceleration) ||
        !readVector3(reader, &item.magnetic_field))
    {
      *error = "motion tracker kinematics record exceeds payload";
      return false;
    }
    packet->tracker_kinematics.push_back(item);
  }
  return true;
}

bool parseCenterOfMass(const XsensPacketHeader& header,
                       ByteReader* reader,
                       ParsedPacket* packet,
                       std::string* error)
{
  if (header.payload_size != 12)
  {
    *error = "center of mass payload size is not 12";
    return false;
  }
  if (!readVector3(reader, &packet->center_of_mass_m))
  {
    *error = "center of mass payload exceeds payload";
    return false;
  }
  return true;
}

bool parseTimeCode(const std::vector<uint8_t>& payload, ParsedPacket* packet, std::string* error)
{
  if (payload.size() != 12)
  {
    *error = "time code payload size is not 12";
    return false;
  }
  packet->time_code.assign(payload.begin(), payload.end());
  return true;
}

}  // namespace

XsensPacketHeader XsensPacketParser::parseHeader(const std::vector<uint8_t>& datagram) const
{
  XsensPacketHeader header;
  header.datagram_size = datagram.size();

  if (datagram.size() < XsensPacketHeader::kHeaderSize)
  {
    header.invalid_reason = "datagram shorter than 24-byte Xsens header";
    return header;
  }

  header.id_string.assign(reinterpret_cast<const char*>(&datagram[0]), 6);

  if (header.id_string.size() != 6 || header.id_string.compare(0, 4, "MXTP") != 0)
  {
    header.invalid_reason = "ID string does not start with MXTP";
    return header;
  }

  header.message_type = header.id_string.substr(4, 2);
  if (!std::all_of(header.message_type.begin(), header.message_type.end(),
                   [](const char c) { return std::isdigit(static_cast<unsigned char>(c)) != 0; }))
  {
    header.invalid_reason = "message type is not ASCII digits";
    return header;
  }

  header.sample_counter = readU32BE(&datagram[6]);
  header.datagram_counter = datagram[10];
  header.number_of_items = datagram[11];
  header.time_code = readU32BE(&datagram[12]);
  header.character_id = datagram[16];
  header.body_segments = datagram[17];
  header.props = datagram[18];
  header.finger_tracking_segments = datagram[19];
  header.reserved = readU16BE(&datagram[20]);
  header.payload_size = readU16BE(&datagram[22]);

  const std::size_t expected_size = XsensPacketHeader::kHeaderSize + header.payload_size;
  if (datagram.size() != expected_size)
  {
    header.invalid_reason = "payload size does not match datagram size";
    return header;
  }

  header.valid = true;
  return header;
}

ParsedPacket XsensPacketParser::parsePacket(const std::vector<uint8_t>& datagram) const
{
  ParsedPacket packet;
  packet.header = parseHeader(datagram);
  packet.valid = packet.header.valid;
  packet.invalid_reason = packet.header.invalid_reason;

  if (!packet.header.valid)
    return packet;

  packet.payload.assign(datagram.begin() + XsensPacketHeader::kHeaderSize, datagram.end());
  packet.coordinate_system = coordinateSystemFor(packet.header.message_type);

  ByteReader reader(datagram, XsensPacketHeader::kHeaderSize);
  std::string error;
  const std::string& type = packet.header.message_type;
  bool ok = true;

  if (type == "01")
    ok = parsePoseEuler(packet.header, &reader, &packet, &error);
  else if (type == "02")
    ok = parsePoseQuaternion(packet.header, &reader, &packet.pose_quaternion,
                             "pose quaternion", &error);
  else if (type == "03")
    ok = parsePointPositions(packet.header, &reader, &packet, &error);
  else if (type == "04" || type == "10" || type == "11")
    packet.raw_payload_only = true;
  else if (type == "05")
    ok = parsePoseQuaternion(packet.header, &reader, &packet.unity_pose,
                             "Unity3D pose", &error);
  else if (type == "12")
    ok = parseMetadata(packet.payload, &packet);
  else if (type == "13")
    ok = parseScaleInfo(&reader, &packet, &error);
  else if (type == "20")
    ok = parseJointAngles(packet.header, &reader, &packet, &error);
  else if (type == "21")
    ok = parseLinearKinematics(packet.header, &reader, &packet, &error);
  else if (type == "22")
    ok = parseAngularKinematics(packet.header, &reader, &packet, &error);
  else if (type == "23")
    ok = parseTrackerKinematics(packet.header, &reader, &packet, &error);
  else if (type == "24")
    ok = parseCenterOfMass(packet.header, &reader, &packet, &error);
  else if (type == "25")
    ok = parseTimeCode(packet.payload, &packet, &error);
  else
  {
    ok = false;
    error = "unsupported Xsens message type";
  }

  if (!ok)
    setInvalid(&packet, error);
  else
    convertCoordinateFrames(&packet);

  return packet;
}

uint16_t XsensPacketParser::readU16BE(const uint8_t* data)
{
  return static_cast<uint16_t>((static_cast<uint16_t>(data[0]) << 8) |
                               static_cast<uint16_t>(data[1]));
}

int32_t XsensPacketParser::readI32BE(const uint8_t* data)
{
  return static_cast<int32_t>(readU32BE(data));
}

uint32_t XsensPacketParser::readU32BE(const uint8_t* data)
{
  return (static_cast<uint32_t>(data[0]) << 24) |
         (static_cast<uint32_t>(data[1]) << 16) |
         (static_cast<uint32_t>(data[2]) << 8) |
         static_cast<uint32_t>(data[3]);
}

float XsensPacketParser::readF32BE(const uint8_t* data)
{
  const uint32_t bits = readU32BE(data);
  float value = 0.0F;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

}  // namespace kuavo_xsense_gmr
