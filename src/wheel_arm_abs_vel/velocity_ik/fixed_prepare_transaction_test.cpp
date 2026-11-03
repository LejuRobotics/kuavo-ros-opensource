#include "fixed_prepare_transaction.h"
#include <cassert>
#include <iostream>
using velocity_ik::FixedPrepareTransaction;
int main() {
  FixedPrepareTransaction t;
  double host=1000000000., source=1000.;
  uint32_t seq=10;
  auto send=[&](bool prepare,bool control,bool pose,uint64_t session=7,uint64_t generation=2,uint64_t epoch=3) {
    host+=8000000.;source+=.008;++seq;
    return t.observe(session,generation,epoch,seq,source,host,0,.15,prepare,control,pose);
  };
  assert(!send(true,true,false)); // Low-confidence first takeover cannot admit.
  assert(send(true,true,true));
  assert(send(true,true,false)); // Accepted fixed transaction survives pose loss.
  assert(t.fresh(7,2,3,host+140000000.));
  assert(!t.fresh(7,2,3,host+151000000.)); // Communication still required.
  assert(!t.observe(7,2,3,seq,source,host+160000000.,0,.15,true,true,false));
  assert(!t.fresh(7,2,3,host+160000000.)); // Old resend cannot renew.
  host+=160000000.; // A truly new command can reconnect through existing stop-first.
  assert(send(true,true,false));
  assert(!t.fresh(7,2,4,host)); // Coordinate epoch does not carry authority.
  assert(!t.fresh(7,3,3,host));
  assert(!t.observe(7,2,3,seq+1,source-.001,host+1,0,.15,true,true,true));
  assert(!send(false,true,true)); // Explicit normal exit cancels.
  assert(!send(true,true,false)); // Pose-free packet cannot resurrect cancellation.
  assert(send(true,true,true));
  assert(!send(true,false,false)); // Control authorization lost.
  assert(!send(true,true,false));
  assert(send(true,true,true));
  assert(!send(true,true,false,7,3)); // New generation needs valid admission.
  assert(send(true,true,true,7,3));
  assert(!send(true,true,false,7,3,4)); // New body epoch also needs admission.
  assert(!t.observe(7,3,4,seq+1,source+.008,host+8000000.,.151,.15,true,true,true));
  FixedPrepareTransaction restarted;
  assert(!restarted.observe(7,3,4,200,2000,host+8000000.,0,.15,true,true,false));
  // 115 Hz / 72 Hz timing is source-driven, independent of nominal packet rate.
  for (double dt:{.008,.009,.014,.017}) {
    FixedPrepareTransaction f;
    assert(f.observe(1,1,1,1,100,1e9,0,.15,true,true,true));
    for(uint32_t i=2;i<100;++i)
      assert(f.observe(1,1,1,i,100+dt*i,1e9+dt*i*1e9,0,.15,true,true,false));
  }
  std::cout<<"PASS fixed PREPARE admission, pose loss, source/sequence freshness, expiry, recovery, cancel, identity, restart, variable dt\n";
}
