#pragma once
#include <cstdint>
#include <chrono>
#include <fstream>
#include <stdexcept>
#include <string>
namespace velocity_ik_execution {
inline std::string readExecutionIdentity(const char* path) {
  std::ifstream stream(path); std::string value; std::getline(stream,value);
  if(value.size()!=36 || value[8]!='-' || value[13]!='-' || value[18]!='-' || value[23]!='-')
    throw std::runtime_error(std::string("Invalid execution clock/process identity: ")+path);
  return value;
}
// Linux processes on this host/container share CLOCK_MONOTONIC. Cross-boot or
// remote-host references are rejected; ROS timestamps are never steady epochs.
inline std::string executionClockDomain() { return readExecutionIdentity("/proc/sys/kernel/random/boot_id"); }
inline std::string executionInstance() { return readExecutionIdentity("/proc/sys/kernel/random/uuid"); }
inline int64_t executionSteadyNs() {
 return std::chrono::duration_cast<std::chrono::nanoseconds>(
   std::chrono::steady_clock::now().time_since_epoch()).count();
}
}
