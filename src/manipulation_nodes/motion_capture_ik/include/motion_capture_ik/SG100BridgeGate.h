#pragma once
namespace HighlyDynamic {
// Caller serializes access. User intent and input freshness are independent:
// fresh input starts/resumes VR by default; an explicit disable stays disabled.
class SG100BridgeGate {
 public:
  bool enable(bool requested, bool fresh) {
    if (requested && !fresh) return false;
    enabled_ = requested;
    return true;
  }
  bool allow(bool fresh) const {
    return enabled_ && fresh;
  }
 private:
  bool enabled_ = true;
};
}
