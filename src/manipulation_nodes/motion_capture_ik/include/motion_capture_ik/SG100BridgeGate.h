#pragma once
namespace HighlyDynamic {
// Caller serializes access. User intent and input freshness are independent:
// fresh input starts/resumes VR by default; an explicit disable stays disabled.
class SG100BridgeGate {
 public:
  bool enable(bool requested, bool fresh) {
    if (requested && !service_control_ && !fresh) return false;
    enabled_ = requested;
    return true;
  }
  bool allow(bool fresh) const {
    return enabled_ && (service_control_ || fresh);
  }
  // Explicit service authority does not depend on a VR heartbeat.
  void selectService() { service_control_ = true; }
  void selectVr() { service_control_ = false; }
  bool serviceControl() const { return service_control_; }
 private:
  bool service_control_ = false;
  bool enabled_ = true;
};
}
