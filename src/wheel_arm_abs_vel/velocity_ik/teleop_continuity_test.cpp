#include "teleop_continuity.h"
#include "operating_envelope.h"
#include <cassert>
#include <iostream>
using namespace velocity_ik;
int main() {
  InputContinuity c;
  assert(c.config.valid());
  double t=1.; uint64_t seq=1;
  for(int i=0;i<20;++i) { c.update(t,.01,seq++,true,false,true); t+=.01; }
  assert(c.canTrack());
  for(double age : {.027,.044,.059}) { c.update(t,age,seq,true,false,false); t+=.01; assert(c.canTrack()); }
  c.update(t,.100,seq,true,false,false); assert(c.phase==InputPhase::Degraded && c.budget>0 && c.budget<1);
  c.update(t,.151,seq,true,false,false); assert(!c.canTrack() && !c.confirmation_required);
  for(int i=0;i<25;++i) {t+=.01;c.update(t,.01,seq,true,false,true);}
  assert(!c.canTrack()); // Repeated packet cannot satisfy stable recovery.
  for(int i=0;i<20;++i) {t+=.01;c.update(t,.01,seq++,true,false,true);}
  assert(c.canTrack()); // Identical pose with advancing source frames is healthy.
  c.update(t,.501,seq,true,false,true); assert(c.confirmation_required);
  for(int i=0;i<20;++i) {t+=.01;c.update(t,.01,seq++,true,false,true);}
  assert(!c.canTrack() && c.stable);
  c.update(t,.01,seq++,true,true,false); assert(!c.canTrack());
  c.update(t,.01,seq++,true,true,true); assert(c.canTrack());
  c.update(t,.01,seq++,false,false,true); assert(!c.canTrack()); // Explicit stop bypasses grace.
  c.suspend(); assert(!c.confirmation_required);
  c.update(t,1.,seq++,true,false,true); assert(!c.confirmation_required);
  c.update(t,1.,seq++,true,false,true); assert(c.confirmation_required);
  c.suspend(); assert(c.confirmation_required); // Pause cannot erase existing loss.
  Acquisition a;
  for(int i=0;i<2000;++i) a.update(.002,false,true);
  assert(a.progress==1 && a.catchup); // Far target only caps correction, not ongoing VR motion.
  for(int i=0;i<700;++i) a.update(.002,true,true);
  assert(a.progress==1 && !a.catchup);
  a.update(.002,false,true); assert(a.progress==1);
  a.reset(); assert(a.blend()==0 && a.catchup);
  a.update(.1,false,true); const double before=a.progress;
  a.update(.1,false,false); assert(a.progress==before); // Short degradation never rewinds startup.
  for(int i=0;i<500;++i) a.update(.002,false,true);
  assert(a.blend()==1 && a.catchup);
  // New lower budget must preserve the currently feasible braking endpoint.
  auto interval=feasibleOperatingInterval(.8,1.2,-.3,.3);
  assert(interval.valid && interval.lower==.8 && interval.upper==.8);
  std::cout << "PASS continuity, loss/recovery, explicit stop, acquisition and feasible cap\n";
}
