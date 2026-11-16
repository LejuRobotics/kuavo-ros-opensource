#pragma once
#include <string>

namespace ros { namespace param {
extern std::string contract;
inline bool get(const std::string&, std::string& value) {
  value = contract;
  return true;
}
} }
