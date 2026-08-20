#ifndef KUAVO_XSENSE_GMR_XSENS_UDP_ELBOW_LANDMARK_CACHE_H
#define KUAVO_XSENSE_GMR_XSENS_UDP_ELBOW_LANDMARK_CACHE_H

#include <cstdint>
#include <map>
#include <string>

#include "kuavo_xsense_gmr/xsens_udp/packet.h"

namespace kuavo_xsense_gmr
{

struct ElbowLandmarkOffsets
{
  Vector3f left_lateral_epicondyle_m;
  Vector3f right_lateral_epicondyle_m;
};

enum class ScaleCacheEvent
{
  kIgnored,
  kCollecting,
  kReady,
  kUpdated,
  kUnchanged,
  kRejected
};

struct ScaleCacheResult
{
  ScaleCacheEvent event = ScaleCacheEvent::kIgnored;
  uint8_t character_id = 0;
  std::string reason;
};

class ElbowLandmarkCache
{
public:
  ScaleCacheResult consume(const ParsedPacket& packet);

  bool getOffsets(uint8_t character_id, ElbowLandmarkOffsets* offsets) const;

private:
  struct Candidate
  {
    bool active = false;
    bool valid = true;
    bool have_left = false;
    bool have_right = false;
    Vector3f left_m;
    Vector3f right_m;
    std::string invalid_reason;
  };

  static bool updateTargetPoint(const XsensScalePoint& point, Candidate* candidate);
  ScaleCacheResult finishCandidate(uint8_t character_id);

  std::map<uint8_t, Candidate> candidates_;
  std::map<uint8_t, ElbowLandmarkOffsets> committed_;
};

}  // namespace kuavo_xsense_gmr

#endif  // KUAVO_XSENSE_GMR_XSENS_UDP_ELBOW_LANDMARK_CACHE_H
