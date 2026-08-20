#include "kuavo_xsense_gmr/xsens_udp/elbow_landmark_cache.h"

#include <cmath>
#include <sstream>

namespace kuavo_xsense_gmr
{
namespace
{

constexpr uint16_t kRightUpperArmSegmentId = 9;
constexpr uint16_t kLeftUpperArmSegmentId = 13;
constexpr uint16_t kLateralEpicondylePointId = 3;
constexpr float kOffsetEqualityTolerance = 1.0e-6F;

const char* kRightLateralEpicondyleName = "pRightArmLatEpicondyle";
const char* kLeftLateralEpicondyleName = "pLeftArmLatEpicondyle";

bool isFinite(const Vector3f& value)
{
  return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

bool nearlyEqual(const float lhs, const float rhs)
{
  return std::fabs(lhs - rhs) <= kOffsetEqualityTolerance;
}

bool nearlyEqual(const Vector3f& lhs, const Vector3f& rhs)
{
  return nearlyEqual(lhs.x, rhs.x) && nearlyEqual(lhs.y, rhs.y) &&
         nearlyEqual(lhs.z, rhs.z);
}

bool offsetsEqual(const ElbowLandmarkOffsets& lhs, const ElbowLandmarkOffsets& rhs)
{
  return nearlyEqual(lhs.left_lateral_epicondyle_m, rhs.left_lateral_epicondyle_m) &&
         nearlyEqual(lhs.right_lateral_epicondyle_m, rhs.right_lateral_epicondyle_m);
}

std::string tupleDescription(const XsensScalePoint& point)
{
  std::ostringstream out;
  out << "(" << point.segment_id << "," << point.point_id << "," << point.name << ")";
  return out.str();
}

}  // namespace

ScaleCacheResult ElbowLandmarkCache::consume(const ParsedPacket& packet)
{
  ScaleCacheResult result;
  result.character_id = packet.header.character_id;

  if (!packet.valid || packet.header.message_type != "13")
    return result;

  Candidate& candidate = candidates_[packet.header.character_id];

  // 带段定义的包是一批新 Scale 的起点。在新批次完成前，committed_ 仍保留旧值。
  if (!packet.scale_segments.empty())
  {
    candidate = Candidate();
    if (packet.header.datagram_counter != 0x00)
    {
      result.event = ScaleCacheEvent::kRejected;
      result.reason = "Scale segment batch does not start with datagram counter 0x00";
      return result;
    }
    candidate.active = true;
  }

  if (!candidate.active)
    return result;

  for (const XsensScalePoint& point : packet.scale_points)
    updateTargetPoint(point, &candidate);

  if (!packet.header.is_last_fragment())
  {
    result.event = ScaleCacheEvent::kCollecting;
    return result;
  }

  return finishCandidate(packet.header.character_id);
}

bool ElbowLandmarkCache::getOffsets(const uint8_t character_id,
                                    ElbowLandmarkOffsets* offsets) const
{
  if (offsets == nullptr)
    return false;

  const auto it = committed_.find(character_id);
  if (it == committed_.end())
    return false;

  *offsets = it->second;
  return true;
}

bool ElbowLandmarkCache::updateTargetPoint(const XsensScalePoint& point,
                                           Candidate* candidate)
{
  if (candidate == nullptr)
    return false;

  const bool right_id =
      point.segment_id == kRightUpperArmSegmentId &&
      point.point_id == kLateralEpicondylePointId;
  const bool left_id =
      point.segment_id == kLeftUpperArmSegmentId &&
      point.point_id == kLateralEpicondylePointId;
  const bool right_name = point.name == kRightLateralEpicondyleName;
  const bool left_name = point.name == kLeftLateralEpicondyleName;

  if (!right_id && !left_id && !right_name && !left_name)
    return false;

  if (right_id != right_name || left_id != left_name)
  {
    candidate->valid = false;
    candidate->invalid_reason =
        "lateral epicondyle ID/name mismatch: " + tupleDescription(point);
    return false;
  }

  if (!isFinite(point.position_m))
  {
    candidate->valid = false;
    candidate->invalid_reason =
        "lateral epicondyle offset is not finite: " + tupleDescription(point);
    return false;
  }

  if (right_id)
  {
    if (candidate->have_right && !nearlyEqual(candidate->right_m, point.position_m))
    {
      candidate->valid = false;
      candidate->invalid_reason = "right lateral epicondyle is duplicated with different offsets";
      return false;
    }
    candidate->right_m = point.position_m;
    candidate->have_right = true;
    return true;
  }

  if (candidate->have_left && !nearlyEqual(candidate->left_m, point.position_m))
  {
    candidate->valid = false;
    candidate->invalid_reason = "left lateral epicondyle is duplicated with different offsets";
    return false;
  }
  candidate->left_m = point.position_m;
  candidate->have_left = true;
  return true;
}

ScaleCacheResult ElbowLandmarkCache::finishCandidate(const uint8_t character_id)
{
  ScaleCacheResult result;
  result.character_id = character_id;

  const auto candidate_it = candidates_.find(character_id);
  if (candidate_it == candidates_.end() || !candidate_it->second.active)
    return result;

  const Candidate candidate = candidate_it->second;
  candidates_.erase(candidate_it);

  if (!candidate.valid)
  {
    result.event = ScaleCacheEvent::kRejected;
    result.reason = candidate.invalid_reason;
    return result;
  }

  if (!candidate.have_left || !candidate.have_right)
  {
    result.event = ScaleCacheEvent::kRejected;
    result.reason = "Scale batch does not contain both lateral epicondyles";
    return result;
  }

  ElbowLandmarkOffsets next;
  next.left_lateral_epicondyle_m = candidate.left_m;
  next.right_lateral_epicondyle_m = candidate.right_m;

  const auto committed_it = committed_.find(character_id);
  if (committed_it == committed_.end())
  {
    committed_[character_id] = next;
    result.event = ScaleCacheEvent::kReady;
    return result;
  }

  if (offsetsEqual(committed_it->second, next))
  {
    result.event = ScaleCacheEvent::kUnchanged;
    return result;
  }

  // 左右点在候选批次完整校验后一次替换，避免单侧新旧标定混用。
  committed_it->second = next;
  result.event = ScaleCacheEvent::kUpdated;
  return result;
}

}  // namespace kuavo_xsense_gmr
