#include "wheel_arm_abs_vel/direct_tcp_startup.h"
#include <cassert>
using S=wheel_arm_abs_vel::DirectTcpStartup;
int main(){S s;
 assert(s.update(false,true,false,true,false,true)==S::None);
 assert(s.update(true,false,false,true,false,true)==S::Start);
 assert(!s.completed);
 assert(s.update(true,false,false,true,false,true)==S::None);
 s.update(false,false,false,true,false,true);
 assert(s.update(true,false,false,true,false,true)==S::Start); // retry failed first start
 s.completed=true;
 s.update(false,false,false,true,false,true);
 assert(s.update(true,false,false,true,false,true)==S::None);
 assert(s.update(false,true,false,true,false,true)==S::Start);
 assert(s.update(false,false,true,false,true,false)==S::None); // unhealthy owner
 s.update(false,false,false,false,true,true);
 assert(s.update(false,false,true,false,true,true)==S::ToggleArmHold);
 assert(s.update(false,false,true,false,true,true)==S::None); // held button
 s.update(false,false,false,false,true,true);
 assert(s.update(false,false,true,false,true,true)==S::ToggleArmHold);
 s.update(false,false,false,true,false,true);
 assert(s.update(false,false,true,true,false,true)==S::None); // cannot start via X+B
}
