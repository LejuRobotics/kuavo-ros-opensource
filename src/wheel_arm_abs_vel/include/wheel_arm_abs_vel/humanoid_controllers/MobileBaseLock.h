#pragma once

namespace humanoidController_wheel_wbc {
// Final controller output policy; no extra publisher, brake action, or runtime
// toggle. Default is fail-closed. Applies equally to WBC and RNEA backends.
template<class Twist>
Twist applyMobileBaseLock(const Twist& command, bool locked = true) {
  return locked ? Twist{} : command;
}
}
