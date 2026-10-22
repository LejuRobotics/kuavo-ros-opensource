// Instrument the ACTUAL formal publication function, not a mock encoder.
#include <cstdlib>
#include <new>
static thread_local bool count_allocations=false;
static thread_local size_t allocations=0;
void* operator new(std::size_t n) { if(count_allocations)++allocations; if(auto p=std::malloc(n?n:1))return p;throw std::bad_alloc(); }
void* operator new[](std::size_t n) { return ::operator new(n); }
void operator delete(void* p) noexcept {std::free(p);}
void operator delete[](void* p) noexcept {std::free(p);}
void operator delete(void* p,std::size_t) noexcept {std::free(p);}
void operator delete[](void* p,std::size_t) noexcept {std::free(p);}
#define VELOCITY_IK_OFFLINE_TEST
#include "../src/pinocchio_velocity_ik_node.cpp"
#include <sys/shm.h>
#include <cassert>
#include <unistd.h>
struct VelocityIkOfflineTest {
 static void run(PinocchioVelocityIk& n,int key) {
  n.nh_.setParam("/hotpath/reference/transport","shm");
  n.nh_.setParam("/hotpath/reference/shm_reference_key",key);
  n.nh_.setParam("/hotpath/reference/shm_owner_key",key+1);
  n.reference_transport_.configure(n.nh_,"/hotpath/reference",true);
  n.formal_reference_pub_=n.nh_.advertise<wheel_arm_abs_vel::WholeBodyIkReference>("/hotpath/mirror",2);
  n.enable_takeover_=n.reference_initialized_=n.takeover_started_=n.session_anchor_.valid=true;
  n.reference_q_.setConstant(.1);n.reference_dq_.setConstant(.2);
  const auto ddq=DecisionVector::Constant(21,.3).eval();
  PinocchioVelocityIk::TargetSnapshot t;t.session_id=41;t.generation=17;t.control_state=wheel_arm_abs_vel::TeleopTcpTarget::CONTROL_ACTIVE;
  std::atomic<int> mirrors{0};
  const auto pub=n.formal_reference_pub_;
  n.formal_mirror_.start([&mirrors,pub](const kuavo_reference_shm::Payload& p){
    wheel_arm_abs_vel::WholeBodyIkReference m;assert(kuavo_reference_shm::decode(p,m));
    assert(m.session_id==41 && m.generation==17 && m.q[0]==.1 && m.dq[0]==.2 && m.ddq[0]==.3);
    pub.publish(m);++mirrors;
  });
  ros::AsyncSpinner spinner(1);spinner.start();
  const boost::function<void(const wheel_arm_abs_vel::WholeBodyIkReference::ConstPtr&)> callback=
      [](const wheel_arm_abs_vel::WholeBodyIkReference::ConstPtr&){};
  auto sub=n.nh_.subscribe<wheel_arm_abs_vel::WholeBodyIkReference>("/hotpath/mirror",2,callback);
  for(int i=0;i<200 && pub.getNumSubscribers()==0;++i)ros::WallDuration(.01).sleep();
  assert(pub.getNumSubscribers()>0);
  const auto initial_writes=n.reference_transport_.referenceWritten();
  size_t phase_allocations[5]{};double worst[5]{};
  for(int tick=0;tick<20000;++tick) {
    n.diagnostic_tick_=tick;n.reference_sequence_=tick;
    allocations=0;count_allocations=true;
    const auto start=std::chrono::steady_clock::now();
    n.publishFormalReference(t,0.,ShadowState::kTracking,true,false,1.,ddq);
    const auto end=std::chrono::steady_clock::now();
    count_allocations=false;
    phase_allocations[tick%5]+=allocations;
    worst[tick%5]=std::max(worst[tick%5],std::chrono::duration<double,std::micro>(end-start).count());
  }
  n.formal_mirror_.stop();
  assert(mirrors>0 && n.reference_transport_.referenceWritten()==initial_writes+40000); // sequence lock advances twice/write
  for(int p=0;p<5;++p){assert(phase_allocations[p]==0);std::cout<<"phase="<<p<<" allocations="<<phase_allocations[p]<<" max_us="<<worst[p]<<'\n';}
  const auto committed=n.reference_transport_.referenceWritten();
  n.terminal_stop_requested_=true;n.publishFormalReference(t,0.,ShadowState::kTracking,true,false,1.,ddq);
  assert(n.reference_transport_.referenceWritten()==committed);
  for(uint64_t tick=0;tick<100;++tick) {
    unsigned due=0;for(unsigned phase=0;phase<5;++phase)due+=velocity_ik::diagnosticPhaseDue(tick,5,phase);
    assert(due==1);
  }
  std::cout<<"PASS actual formal SHM: all five phases allocation-free; mirrors="<<mirrors
           <<" dropped="<<n.formal_mirror_.dropped()<<"; stop rejects further commits\n";
 }
};
int main(int argc,char**argv) {
 ros::init(argc,argv,"formal_reference_hotpath_test",ros::init_options::NoSigintHandler);
 const int key=0x580000+(getpid()%100000)*2;
 int code=0;
 try {PinocchioVelocityIk n;VelocityIkOfflineTest::run(n,key);}
 catch(const std::exception& e){std::cerr<<e.what()<<'\n';code=1;}
 for(int k:{key,key+1}) {const int id=shmget(k,1,0600);if(id>=0)shmctl(id,IPC_RMID,nullptr);}
 return code;
}
