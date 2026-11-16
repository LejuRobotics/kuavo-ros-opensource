#include "wheel_arm_abs_vel/humanoid_controllers/ClockedReferenceFreshness.h"
#include <cassert>
#include <iostream>
using G=velocity_ik_execution::ClockedReferenceFreshness;
int main(){
 const int64_t L=50000000,S=100000000000,R=1700000000000000000;
 G g("boot");auto accept=[&](uint64_t seq,int64_t ps,int64_t rs,int64_t pr,int64_t rr){
   return g.accept("boot","producer",1,1,seq,ps,rs,pr,rr,L);};
 assert(accept(1,S,S,R,R)==G::Result::NewSample);
 assert(accept(1,S,S+1000000,R,R+1000000)==G::Result::DuplicateOrReordered);
 assert(accept(2,S+2000000,S+2000000,R,R+2000000)==G::Result::OldStamp);
 assert(accept(2,S,S+3000000,R+3000000,R+3000000)==G::Result::OldStamp);
 assert(accept(2,S+4000000,S+4000000,R+4000000,R+4000000)==G::Result::NewSample);
 assert(accept(3,S+6000000,S+6000000,R-60000000000,R-60000000000)==G::Result::NewSample);
 // Replayed prior-clock packet cannot become young by receipt or a new ROS stamp.
 assert(accept(2,S+4000000,S+8000000,R-60000000000+2000000,R-60000000000+2000000)
        ==G::Result::DuplicateOrReordered);
 assert(accept(4,S+10000000,S+10000000,R+60000000000,R+60000000000)==G::Result::NewSample);
 assert(accept(5,S+12000000,S+12000000,R+12000000,R+12000000)==G::Result::NewSample);
 assert(g.accept("other-boot","producer",1,1,6,S+14000000,S+14000000,R+14000000,R+14000000,L)==G::Result::WrongClockDomain);
 assert(g.accept("boot","",1,1,6,S+14000000,S+14000000,R+14000000,R+14000000,L)==G::Result::MissingIdentity);
 assert(accept(6,S+14000000,S+65000000,R+14000000,R+65000000)==G::Result::ExpiredOrFuture);
 assert(accept(6,S+68000000,S+67000000,R+67000000,R+67000000)==G::Result::ExpiredOrFuture);
 // Genuine process restart needs a new instance and fresh generation time.
 assert(g.accept("boot","producer-restarted",1,1,1,S+70000000,S+70000000,R+70000000,R+70000000,L)==G::Result::NewSample);
 assert(g.accept("boot","producer",1,1,99,S+12000000,S+72000000,R+72000000,R+72000000,L)!=G::Result::NewSample);
 std::cout<<"PASS source identity/domain, duplicate/old stamp, both ROS jumps, stale/future, process restart ordering\n";
}
