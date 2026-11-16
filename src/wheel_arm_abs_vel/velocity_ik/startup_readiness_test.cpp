#include "startup_readiness.h"
#include <cassert>
#include <iostream>
int main(int argc,char** argv) {
  using G=velocity_ik::StartupReadiness;
  G g;g.settings.validate();G::Values q{},v{};
  if(argc>1) { // exact sensor samples, ROS-free bag audit
    double t;int fresh;
    while(std::cin>>t>>fresh){for(auto& x:q)std::cin>>x;for(auto& x:v)std::cin>>x;
      std::cout<<g.update(t,q,v,fresh)<<' '<<g.reason<<' '<<g.joint<<' '<<g.position_range<<' '<<g.speed_rms<<'\n';}
    return 0;
  }
  for(int i=0;i<100;++i){q[11]=(i%2)*.0004;v[11]=i%3==0?.09:-.015;g.update(1+i*.001,q,v,true);}
  assert(g.ready); // encoder-scale displacement and isolated velocity spikes
  assert(!g.update(1.1,q,v,false));
  g.reset();q.fill(0);v.fill(0);
  for(int i=0;i<100;++i){q[11]=i*.0001;v[11]=.1;g.update(2+i*.001,q,v,true);}
  assert(!g.ready); // genuine slow motion cannot qualify
  g.reset();q.fill(0);v.fill(0);
  for(int i=0;i<100;++i){v[11]=.16;g.update(3+i*.001,q,v,true);}
  assert(!g.ready && g.reason==4); // sustained raw speed with a stuck q
  g.reset();v.fill(0);
  for(int i=0;i<100;++i)g.update(4+i*.001,q,v,true);
  assert(g.ready);v[11]=.31;assert(!g.update(4.1,q,v,true));
  v.fill(0);assert(!g.update(4.101,q,v,true));
  g.reset();for(int i=0;i<100;++i)g.update(5.,q,v,true);
  assert(!g.ready); // repeated sensor frame never fills the window
  g.reset();for(int i=0;i<100;++i){v[5]=.04;g.update(6+i*.001,q,v,true);}
  assert(!g.ready); // stricter lower-body speed
  assert(!g.update(5,q,v,true)); // backwards clock clears history
  std::cout<<"startup readiness tests passed\n";
}
