#include "wheel_arm_abs_vel/model/FullModelContract.h"
#include <iostream>
#include <stdexcept>

int main(int argc, char** argv) {
  try {
    if (argc != 2) throw std::runtime_error("assets directory required");
    const std::string assets = argv[1];
    using wheel_arm_abs_vel::fullModelRelativePath;
    using wheel_arm_abs_vel::requireFullModel;
    for (int version : {62, 63, 400062}) {
      const std::string full = assets + "/models/" + fullModelRelativePath(version);
      requireFullModel(version, assets, full);
      for (int other : {62, 63, 400062}) {
        if (version == other) continue;
        bool rejected = false;
        try { requireFullModel(version, assets, assets + "/models/" + fullModelRelativePath(other)); }
        catch (const std::exception&) { rejected = true; }
        if (!rejected) throw std::runtime_error("cross-version full URDF accepted");
      }
      for (const auto& invalid : {std::string(), assets + "/models/missing.urdf",
              assets + "/models/biped_s" + std::to_string(version) + "/urdf/drake/biped_v3_arm.urdf"}) {
        bool rejected = false;
        try { requireFullModel(version, assets, invalid); }
        catch (const std::exception&) { rejected = true; }
        if (!rejected) throw std::runtime_error("missing or legacy arm URDF accepted");
      }
      bool rejected = false;
      try { requireFullModel(version, "", full); }
      catch (const std::exception&) { rejected = true; }
      if (!rejected) throw std::runtime_error("empty asset root accepted");
    }
    for (int version : {0, 15, 60, 64}) {
      bool rejected = false;
      try { fullModelRelativePath(version); }
      catch (const std::exception&) { rejected = true; }
      if (!rejected) throw std::runtime_error("unsupported version accepted");
    }
    std::cout << "PASS: 62/63/400062 full models; cross-version, legacy, missing and unsupported models rejected\n";
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
