#include "wheel_arm_abs_vel/humanoid_controllers/MobileBaseLock.h"
#include <cassert>
#include <initializer_list>
#include <limits>

struct Vector { double x=0, y=0, z=0; };
struct Twist { Vector linear, angular; };
int main() {
  using humanoidController_wheel_wbc::applyMobileBaseLock;
  Twist cmd;cmd.linear={.4,-.2,.1};cmd.angular={.3,-.4,.5};
  for (bool explicit_lock : {false,true}) {
    auto out=explicit_lock?applyMobileBaseLock(cmd,true):applyMobileBaseLock(cmd);
    assert(out.linear.x==0 && out.linear.y==0 && out.linear.z==0);
    assert(out.angular.x==0 && out.angular.y==0 && out.angular.z==0);
  }
  const auto out=applyMobileBaseLock(cmd,false);
  assert(out.linear.x==.4 && out.linear.y==-.2 && out.linear.z==.1);
  assert(out.angular.x==.3 && out.angular.y==-.4 && out.angular.z==.5);
  cmd.linear.x=std::numeric_limits<double>::quiet_NaN();
  assert(applyMobileBaseLock(cmd).linear.x==0);
  // Never changes the upstream command or adds a second output integrator.
  assert(cmd.angular.z==.5);
}
