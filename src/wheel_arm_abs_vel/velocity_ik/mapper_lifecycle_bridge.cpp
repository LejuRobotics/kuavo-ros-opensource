// Isolated IPC bridge: production message admission, control tick and status.
#define VELOCITY_IK_OFFLINE_TEST
#include "../src/pinocchio_velocity_ik_node.cpp"
#include <ros/serialization.h>
using Json=nlohmann::json;
struct VelocityIkOfflineTest {
 template<class T>static Json wire(const T& m){std::vector<uint8_t>b(ros::serialization::serializationLength(m));ros::serialization::OStream os(b.data(),b.size());ros::serialization::serialize(os,m);return b;}
 static void run(PinocchioVelocityIk& n,const Json& seed) {
  n.offline_capture_messages_=true;n.diagnostics_async_=false;n.publish_input_debug_=false;
  n.robot_state_valid_=n.arm_mode_known_=true;n.current_arm_mode_=n.target_arm_mode_=2;n.arm_mode_transitioning_=false;
  n.measured_q_.resize(n.decisionDofCount());for(int i=0;i<n.decisionDofCount();++i)n.measured_q_[i]=seed[i][0];n.measured_dq_=DecisionVector::Zero(n.decisionDofCount());
  uint64_t feedback_seq=0;std::string line;
  while(std::getline(std::cin,line)) {
   auto j=Json::parse(line);auto bytes=j["target"].get<std::vector<uint8_t>>();
   auto m=boost::make_shared<wheel_arm_abs_vel::TeleopTcpTarget>();ros::serialization::IStream is(bytes.data(),bytes.size());ros::serialization::deserialize(is,*m);
   n.targetCallback(m);
   // Ideal measured plant and execution consumer are the only fixtures.
   // The first owner grant echoes the production stopped seed; later grants
   // renew the same owner. No Mapper ACTIVE/reference-init state is seeded.
   if(n.offline_formal_reference_.valid){
    auto f=boost::make_shared<wheel_arm_abs_vel::WholeBodyIkReference>(n.offline_formal_reference_);
    f->execution_owner_instance="offline-consumer";f->owner_feedback_seq=++feedback_seq;
    f->header.stamp=ros::Time::now();f->generated_monotonic_ns=velocity_ik_execution::executionSteadyNs();f->producer_lease_deadline_ns=f->generated_monotonic_ns+50000000;
    f->authority_state=f->AUTHORITY_ACTIVE;f->safety_braking=false;
    n.acceptExecutionFeedback(f);
    n.measured_q_=Eigen::Map<DecisionVector>(f->q.data(),f->q.size());n.measured_dq_=Eigen::Map<DecisionVector>(f->dq.data(),f->dq.size());
   }
   auto now=ros::Time::now();n.sensor_source_stamp_=now;
   n.robot_measurement_freshness_.observe(now.toNSec(),now.toNSec(),n.monotonicNs(),n.robot_state_timeout_);
   const auto before=n.reference_q_;const auto v=n.reference_dq_;
   const bool initialized=n.reference_initialized_;const auto generation=n.session_anchor_.generation;
   n.publishOnce();
   const double initial_seed_error=!initialized && n.reference_initialized_ ? (n.reference_q_-n.measured_q_.cwiseMax(n.position_lower_).cwiseMin(n.position_upper_)).cwiseAbs().maxCoeff():0.;
   double jump=0.;if(initialized && generation!=n.session_anchor_.generation)jump=(n.reference_q_-before-.001*(v+n.reference_dq_)).cwiseAbs().maxCoeff();
   std::cout<<"BRIDGE "<<Json({{"status",wire(n.offline_mapper_status_)},{"owner",wire(n.offline_formal_reference_)},{"initial_seed_error",initial_seed_error},{"speed",maxAbs(n.reference_dq_)},{"initialized",n.reference_initialized_},{"started",n.takeover_started_},{"bound",n.session_target_alignment_.valid},{"fault",n.dynamic_fault_latched_},{"jump",jump},{"ready",n.startup_readiness_.ready},{"target_valid",n.target_.valid}}).dump()<<std::endl;
  }
 }
};
int main(int argc,char**argv){ros::init(argc,argv,"operating_replay",ros::init_options::NoSigintHandler);try{Json j;std::ifstream(argv[1])>>j;PinocchioVelocityIk n;VelocityIkOfflineTest::run(n,j["segments"][0]["seed"]);}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 2;}}
