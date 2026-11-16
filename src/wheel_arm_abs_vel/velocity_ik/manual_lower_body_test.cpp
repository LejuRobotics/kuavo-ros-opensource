#include "manual_lower_body.h"
#include <cassert>
#include <iostream>
#include <limits>
using M=velocity_ik::ManualLowerBody;
int main() {
  M m; std::string why; M::Joints target{{.1,-.1,0.,0.}};
  assert(!m.request(target,1.,why));
  M::Snapshot s; s.now=1.; s.allowed=s.stationary=true;
  s.lower.fill(-1.); s.upper.fill(1.);
  m.update(s,.002);
  assert(!m.request(target,1.11,why));
  auto bad=target; bad[2]=std::numeric_limits<double>::quiet_NaN();
  assert(!m.request(bad,1.,why)); bad[2]=2.; assert(!m.request(bad,1.,why));
  assert(m.request(target,1.,why)); assert(!m.request(target,1.,why));
  assert(!m.cancel(m.id+1,why));
  s.stationary=false; m.update(s,.002); assert(m.state==M::Moving);
  assert(m.velocity()[0]>0. && m.velocity()[1]<0.);
  ++s.key[0]; m.update(s,.002); assert(m.state==M::Braking);
  assert(m.velocity()==M::Joints{}); s.stationary=true;m.update(s,.002);
  assert(m.state==M::Aborted); m.update(s,.002);assert(m.state==M::Aborted);
  assert(m.request(target,1.,why)); s.allowed=false;m.update(s,.002); assert(m.state==M::Aborted);
  s.allowed=true;m.update(s,.002);assert(m.request(target,1.,why));
  s.q=target;for(int k=0;k<100;++k)m.update(s,.002);
  assert(m.state==M::Moving); // reference alone cannot prove arrival
  s.measured=target;for(int k=0;k<60;++k)m.update(s,.002);assert(m.state==M::Settled);
  assert(m.request(target,1.,why));assert(m.cancel(m.id,why));m.update(s,.002);assert(m.state==M::Aborted);
  assert(m.request(target,1.,why));s.now=100.;m.update(s,.002);assert(m.state==M::Aborted);
  s.now=101.;m.update(s,.002);assert(m.request(target,101.,why));m.update(s,.1);assert(m.state==M::Aborted);
  std::cout<<"PASS admission, stale status, finite bounds, ownership, cancellation, timeout, measured settling\n";
}
